#!/usr/bin/env python3

import inspect
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


JETCOBOT_UTILS_SRC = '/home/jetson/jetcobot_ws/src/jetcobot_utils/src'


class GraspHardwareBridge(Node):
    def __init__(self) -> None:
        super().__init__('grasp_hw_bridge')

        self.declare_parameter('enable_real_grasp', True)
        self.declare_parameter('fallback_drop_pose_label', 'Old_school_bag')
        self.declare_parameter('watch_joint1', -90)
        self.declare_parameter('watch_joint2', 0)
        self.declare_parameter('watch_joint3', 0)
        self.declare_parameter('watch_joint4', -83)
        self.declare_parameter('watch_joint5', -6)
        self.declare_parameter('watch_joint6', -1)
        self.declare_parameter('grasp_target_swap_xy', False)
        self.declare_parameter('grasp_target_x_sign', 1.0)
        self.declare_parameter('grasp_target_y_sign', 1.0)
        self.declare_parameter('grasp_target_x_offset', 0.0)
        self.declare_parameter('grasp_target_y_offset', 0.0)
        self.declare_parameter('use_bridge_grasp_pose', True)
        self.declare_parameter('grasp_rx', -175.0)
        self.declare_parameter('grasp_ry', 0.0)
        self.declare_parameter('grasp_rz', -45.0)
        self.declare_parameter('use_custom_drop_joints', False)
        self.declare_parameter('drop_joint1', -90)
        self.declare_parameter('drop_joint2', 0)
        self.declare_parameter('drop_joint3', 0)
        self.declare_parameter('drop_joint4', -60)
        self.declare_parameter('drop_joint5', 0)
        self.declare_parameter('drop_joint6', -45)
        self.declare_parameter('drop_settle_sec', 1.0)
        self.declare_parameter('return_after_drop', True)

        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.watch_pose_ready_pub = self.create_publisher(Bool, '/arm/watch_pose_ready', ready_qos)
        self.result_pub = self.create_publisher(String, '/arm/grasp_hw_result', 10)
        self.request_sub = self.create_subscription(
            String,
            '/arm/grasp_hw_request',
            self._on_request,
            10,
        )

        self.grasp_controller = None
        self.supports_height_profile = False
        self.busy = False
        self.fallback_drop_pose_label = str(self.get_parameter('fallback_drop_pose_label').value)
        self.known_drop_pose_labels = {
            'Zip_top_can',
            'Newspaper',
            'Old_school_bag',
            'Book',
            'Syringe',
            'Used_batteries',
            'Expired_cosmetics',
            'Expired_tablets',
            'Fish_bone',
            'Watermelon_rind',
            'Apple_core',
            'Egg_shell',
            'Cigarette_butts',
            'Toilet_paper',
            'Peach_pit',
            'Disposable_chopsticks',
        }

        self._publish_watch_pose_ready(False)
        self._try_init_real_controller()
        self.get_logger().info('grasp_hw_bridge started.')

    def _publish_watch_pose_ready(self, ready: bool) -> None:
        msg = Bool()
        msg.data = bool(ready)
        self.watch_pose_ready_pub.publish(msg)

    def _param_bool(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_int(self, name: str) -> int:
        return int(float(self.get_parameter(name).value))

    def _custom_drop_joints(self) -> list:
        return [
            self._param_int('drop_joint1'),
            self._param_int('drop_joint2'),
            self._param_int('drop_joint3'),
            self._param_int('drop_joint4'),
            self._param_int('drop_joint5'),
            self._param_int('drop_joint6'),
        ]

    def _try_init_real_controller(self) -> None:
        if not bool(self.get_parameter('enable_real_grasp').value):
            self.get_logger().info('Real grasp disabled. mock mode is active.')
            self._publish_watch_pose_ready(True)
            return

        if JETCOBOT_UTILS_SRC in sys.path:
            sys.path.remove(JETCOBOT_UTILS_SRC)
        sys.path.insert(0, JETCOBOT_UTILS_SRC)

        try:
            from jetcobot_utils.grasp_controller import GraspController

            self.grasp_controller = GraspController()
            self.supports_height_profile = 'height_profile' in inspect.signature(
                self.grasp_controller.grasp_run
            ).parameters
            self.grasp_controller.init_watch_pose()
            watch_pose_applied = self._apply_configured_watch_pose()
            self._publish_watch_pose_ready(watch_pose_applied)
            self.get_logger().info(
                f'Real GraspController loaded. height_profile support={self.supports_height_profile}.'
            )
        except Exception as exc:
            self.grasp_controller = None
            self.supports_height_profile = False
            self._publish_watch_pose_ready(False)
            self.get_logger().warning(
                f'Failed to load real GraspController. switch to mock mode: {exc}'
            )

    def _apply_configured_watch_pose(self) -> bool:
        if self.grasp_controller is None or not hasattr(self.grasp_controller, 'go_angles'):
            return False

        joints = [
            int(self.get_parameter('watch_joint1').value),
            int(self.get_parameter('watch_joint2').value),
            int(self.get_parameter('watch_joint3').value),
            int(self.get_parameter('watch_joint4').value),
            int(self.get_parameter('watch_joint5').value),
            int(self.get_parameter('watch_joint6').value),
        ]
        self.grasp_controller.go_angles(joints, 2)
        self.get_logger().info(f'Configured watch pose applied: joints={joints}')
        return True

    def _on_request(self, msg: String) -> None:
        if self.busy:
            self.get_logger().warning('Hardware bridge is busy. ignore new request.')
            return

        try:
            request = self._parse_request(msg.data)
        except Exception as exc:
            self._publish_result(f'grasp_failed:{exc}')
            return

        self.busy = True
        threading.Thread(target=self._execute_request, args=(request,), daemon=True).start()

    def _parse_request(self, payload: str) -> dict:
        values = {}
        for item in str(payload).split(';'):
            if '=' not in item:
                continue
            key, value = item.split('=', 1)
            values[key.strip()] = value.strip()

        required = (
            'x',
            'y',
            'source_label',
            'execute_label',
            'approach_z',
            'grasp_z',
            'joint1',
            'joint4',
            'joint5',
            'joint6',
        )
        missing = [key for key in required if key not in values]
        if missing:
            raise ValueError(f'missing fields: {",".join(missing)}')

        return {
            'x': float(values['x']),
            'y': float(values['y']),
            'source_label': str(values['source_label']),
            'execute_label': self._normalize_execute_label(str(values['execute_label'])),
            'approach_z': int(float(values['approach_z'])),
            'grasp_z': int(float(values['grasp_z'])),
            'joints': [
                int(float(values['joint1'])),
                int(float(values.get('joint2', 0))),
                int(float(values.get('joint3', 0))),
                int(float(values['joint4'])),
                int(float(values['joint5'])),
                int(float(values['joint6'])),
            ],
            'joint1456': [
                int(float(values['joint1'])),
                int(float(values['joint4'])),
                int(float(values['joint5'])),
                int(float(values['joint6'])),
            ],
        }

    def _normalize_execute_label(self, execute_label: str) -> str:
        label = str(execute_label).strip()
        if label in self.known_drop_pose_labels:
            return label

        self.get_logger().warning(
            f'Unknown drop pose label "{label}", fallback to {self.fallback_drop_pose_label}.'
        )
        return self.fallback_drop_pose_label

    def _execute_request(self, request: dict) -> None:
        success = False
        error_message = ''

        try:
            if self.grasp_controller is not None:
                success = self._run_real_grasp(request)
            else:
                success = self._run_mock_grasp(request)
        except Exception as exc:
            error_message = str(exc)

        if success:
            self._publish_result('grasp_finished')
            self.get_logger().info('Hardware grasp finished.')
        else:
            result = f'grasp_failed:{error_message}' if error_message else 'grasp_failed'
            self._publish_result(result)
            self.get_logger().warning(f'Hardware grasp failed: {error_message}')

        self.busy = False

    def _run_real_grasp(self, request: dict) -> bool:
        source_label = request['source_label']
        execute_label = request['execute_label']
        target_x, target_y = self._map_grasp_target(float(request['x']), float(request['y']))
        detect_msg = {
            execute_label: (-target_y, target_x)
        }
        height_profile = {
            'approach_z': int(request['approach_z']),
            'grasp_z': int(request['grasp_z']),
        }
        joints = request['joints']
        joint1456 = request['joint1456']

        self.get_logger().info(
            'Run real grasp: '
            f'source_label={source_label}, '
            f'execute_label={execute_label}, '
            f'raw_xy=({request["x"]:.3f},{request["y"]:.3f}), '
            f'mapped_xy=({target_x:.3f},{target_y:.3f}), '
            f'detect_pos=({detect_msg[execute_label][0]:.3f},{detect_msg[execute_label][1]:.3f}), '
            f'approach_z={height_profile["approach_z"]}, '
            f'grasp_z={height_profile["grasp_z"]}, '
            f'joints={joints}, '
            f'joint1456={joint1456}'
        )

        if (
            self.supports_height_profile
            and not self._param_bool('use_bridge_grasp_pose')
            and not self._param_bool('use_custom_drop_joints')
        ):
            self.grasp_controller.grasp_run(
                'sorting',
                'garbage',
                detect_msg,
                joint1456,
                height_profile=height_profile,
            )
        else:
            if not self.supports_height_profile:
                self.get_logger().warning(
                    'Legacy GraspController detected. bridge-side profile grasp will be used.'
                )
            else:
                self.get_logger().info('Bridge-side grasp pose is enabled.')
            self._run_legacy_profile_grasp(
                detect_pos=detect_msg[execute_label],
                execute_label=execute_label,
                joint1456=joint1456,
                joints=joints,
                height_profile=height_profile,
            )
        return True

    def _map_grasp_target(self, raw_x: float, raw_y: float) -> tuple:
        if self._param_bool('grasp_target_swap_xy'):
            mapped_x = raw_y
            mapped_y = raw_x
        else:
            mapped_x = raw_x
            mapped_y = raw_y

        mapped_x = mapped_x * self._param_float('grasp_target_x_sign')
        mapped_y = mapped_y * self._param_float('grasp_target_y_sign')
        mapped_x += self._param_float('grasp_target_x_offset')
        mapped_y += self._param_float('grasp_target_y_offset')
        return mapped_x, mapped_y

    def _run_legacy_profile_grasp(
        self,
        detect_pos,
        execute_label: str,
        joint1456: list,
        joints: list,
        height_profile: dict,
    ) -> None:
        if hasattr(self.grasp_controller, 'func_start'):
            self.grasp_controller.func_start = True

        offset_x, offset_y = self.grasp_controller.grasp_get_offset_xy(
            'sorting',
            'garbage',
            detect_pos[1],
            -detect_pos[0],
        )
        x = (detect_pos[1] + offset_x) * 1000.0
        y = (-detect_pos[0] + offset_y) * 1000.0
        approach_z = int(height_profile['approach_z'])
        grasp_z = int(height_profile['grasp_z'])
        grasp_rx = self._param_float('grasp_rx')
        grasp_ry = self._param_float('grasp_ry')
        grasp_rz = self._param_float('grasp_rz')
        coords_init = [x, y, approach_z, grasp_rx, grasp_ry, grasp_rz]

        self.get_logger().info(
            f'Legacy profile grasp: target={execute_label}, x={x:.1f}mm, y={y:.1f}mm, '
            f'approach_z={approach_z}, grasp_z={grasp_z}, '
            f'grasp_rpy=[{grasp_rx:.1f},{grasp_ry:.1f},{grasp_rz:.1f}]'
        )

        self.grasp_controller.go_coords(coords_init, 3)
        self.grasp_controller.ctrl_gripper_height(grasp_z, 1.0)
        self.grasp_controller.close_gripper(1.5)
        self.grasp_controller.ctrl_gripper_height(approach_z, 2.0)

        if self._param_bool('use_custom_drop_joints'):
            drop_joints = self._custom_drop_joints()
            drop_settle_sec = max(0.0, self._param_float('drop_settle_sec'))
            self.get_logger().info(
                f'Custom drop joints enabled: joints={drop_joints}, '
                f'open gripper after {drop_settle_sec:.1f}s settle.'
            )
            self.grasp_controller.go_angles(drop_joints, 2)
            if drop_settle_sec > 0.0:
                time.sleep(drop_settle_sec)
            self.grasp_controller.open_gripper(1.0)
        else:
            self.grasp_controller.goOverPose('sorting', 'garbage')
            self.grasp_controller.goTargetPose('sorting', 'garbage', '1', execute_label)
            self.grasp_controller.open_gripper(1.0)
            self.grasp_controller.lift_gripper('sorting', 'garbage', '1', execute_label)
            self.grasp_controller.goOverPose('sorting', 'garbage')

        if self._param_bool('return_after_drop'):
            self.grasp_controller.go_angles(joints, 2)
        else:
            self.get_logger().info('return_after_drop is false. Keep arm at drop pose for tuning.')

        if hasattr(self.grasp_controller, 'func_start'):
            self.grasp_controller.func_start = False

    def _run_mock_grasp(self, request: dict) -> bool:
        self.get_logger().info(
            f'Mock grasp: x={request["x"]:.3f}, y={request["y"]:.3f}, '
            f'label={request["source_label"]}, approach_z={request["approach_z"]}, '
            f'grasp_z={request["grasp_z"]}'
        )
        time.sleep(2.0)
        return True

    def _publish_result(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.result_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GraspHardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
