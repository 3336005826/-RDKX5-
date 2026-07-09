#!/usr/bin/env python3

import inspect
import os
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


JETCOBOT_UTILS_SRC = '/home/jetson/jetcobot_ws/src/jetcobot_utils/src'


class GraspExecutor(Node):
    def __init__(self) -> None:
        super().__init__('grasp_executor')

        self.declare_parameter('enable_real_grasp', True)
        self.declare_parameter('joint1', -90)
        self.declare_parameter('joint2', 0)
        self.declare_parameter('joint3', 0)
        self.declare_parameter('joint4', -83)
        self.declare_parameter('joint5', -6)
        self.declare_parameter('joint6', -1)
        self.declare_parameter('default_label', 'plastic')
        self.declare_parameter('default_approach_z', 170)
        self.declare_parameter('default_grasp_z', 115)
        self.declare_parameter('drop_mode', 'single_bag')
        self.declare_parameter('drop_label', 'plastic')
        self.declare_parameter('grasp_profile_config', '')
        self.declare_parameter('auto_pick_from_detector', True)
        self.declare_parameter('min_pick_interval_sec', 8.0)
        self.declare_parameter('min_target_shift_m', 0.03)

        self.arm_busy_pub = self.create_publisher(Bool, '/mission/arm_busy', 10)
        self.result_pub = self.create_publisher(String, '/arm/grasp_result', 10)
        self.label_sub = self.create_subscription(String, '/mission/trash_label', self._on_label, 10)
        self.target_sub = self.create_subscription(PoseStamped, '/mission/arm_pick_target', self._on_pick_target, 10)
        self.detector_target_sub = self.create_subscription(
            PoseStamped,
            '/mission/trash_pose',
            self._on_detector_target,
            10,
        )

        self.current_label = str(self.get_parameter('default_label').value)
        self.enable_real_grasp = bool(self.get_parameter('enable_real_grasp').value)
        self.grasp_controller = None
        self.supports_height_profile = False

        self.label_aliases = {}
        self.grasp_profiles = {}
        self.default_profile = {
            'approach_z': int(self.get_parameter('default_approach_z').value),
            'grasp_z': int(self.get_parameter('default_grasp_z').value),
        }
        self.drop_mode = str(self.get_parameter('drop_mode').value).strip().lower()
        self.drop_label = str(self.get_parameter('drop_label').value)

        self.auto_pick_from_detector = bool(self.get_parameter('auto_pick_from_detector').value)
        self.min_pick_interval_sec = float(self.get_parameter('min_pick_interval_sec').value)
        self.min_target_shift_m = float(self.get_parameter('min_target_shift_m').value)
        self.pick_in_progress = False
        self.last_pick_time = 0.0
        self.last_pick_target = None

        self._load_grasp_profile_config()
        self._try_init_real_controller()

        self.get_logger().info(
            '机械臂抓取执行节点已启动。'
            f'drop_mode={self.drop_mode}, '
            f'drop_label={self.drop_label}, '
            f'auto_pick_from_detector={self.auto_pick_from_detector}'
        )

    def _default_config_path(self) -> str:
        package_share = get_package_share_directory('arm_mission_system')
        return os.path.join(package_share, 'config', 'grasp_profiles.yaml')

    def _load_grasp_profile_config(self) -> None:
        config_path = str(self.get_parameter('grasp_profile_config').value).strip()
        if not config_path:
            config_path = self._default_config_path()

        try:
            with open(config_path, 'r', encoding='utf-8') as config_file:
                config_data = yaml.safe_load(config_file) or {}
        except Exception as exc:
            self.get_logger().warning(f'抓取配置文件加载失败，使用默认参数: {exc}')
            return

        param_root = {}
        for root_key in ('grasp_executor', 'arm_mission_system', '/**'):
            root_value = config_data.get(root_key, {})
            if isinstance(root_value, dict) and isinstance(root_value.get('ros__parameters'), dict):
                param_root = root_value.get('ros__parameters', {})
                break
        if not isinstance(param_root, dict):
            self.get_logger().warning('抓取配置文件格式不正确，使用默认参数。')
            return

        default_profile = param_root.get('default_profile', {})
        if isinstance(default_profile, dict):
            self.default_profile = {
                'approach_z': int(default_profile.get('approach_z', self.default_profile['approach_z'])),
                'grasp_z': int(default_profile.get('grasp_z', self.default_profile['grasp_z'])),
            }

        aliases = param_root.get('label_aliases', {})
        if isinstance(aliases, dict):
            self.label_aliases = {
                str(key).strip().lower(): str(value).strip()
                for key, value in aliases.items()
            }

        profiles = param_root.get('grasp_profiles', {})
        if isinstance(profiles, dict):
            loaded_profiles = {}
            for label, profile in profiles.items():
                if not isinstance(profile, dict):
                    continue
                loaded_profiles[str(label).strip()] = {
                    'approach_z': int(profile.get('approach_z', self.default_profile['approach_z'])),
                    'grasp_z': int(profile.get('grasp_z', self.default_profile['grasp_z'])),
                }
            self.grasp_profiles = loaded_profiles

        drop_mode = param_root.get('drop_mode')
        if drop_mode is not None:
            self.drop_mode = str(drop_mode).strip().lower()

        drop_label = param_root.get('drop_label')
        if drop_label is not None:
            self.drop_label = str(drop_label).strip()

        self.get_logger().info(
            f'已加载抓取配置: {config_path}, profiles={len(self.grasp_profiles)}, aliases={len(self.label_aliases)}'
        )

    def _try_init_real_controller(self) -> None:
        if not self.enable_real_grasp:
            self.get_logger().info('当前关闭真实抓取模式，使用模拟抓取。')
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
            self.get_logger().info(
                '已成功加载真实机械臂抓取控制器，'
                f'height_profile支持={self.supports_height_profile}。'
            )
        except Exception as exc:
            self.grasp_controller = None
            self.supports_height_profile = False
            self.get_logger().warning(f'真实抓取控制器加载失败，回退到模拟模式: {exc}')

    def _on_label(self, msg: String) -> None:
        self.current_label = msg.data

    def _on_pick_target(self, msg: PoseStamped) -> None:
        self.get_logger().info(
            f'收到抓取目标: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, label={self.current_label}'
        )
        self._start_pick(msg)

    def _on_detector_target(self, msg: PoseStamped) -> None:
        if not self.auto_pick_from_detector:
            return
        if self.pick_in_progress:
            return

        now = time.time()
        if now - self.last_pick_time < self.min_pick_interval_sec:
            return

        current_target = (float(msg.pose.position.x), float(msg.pose.position.y))
        if self.last_pick_target is not None:
            dx = current_target[0] - self.last_pick_target[0]
            dy = current_target[1] - self.last_pick_target[1]
            if (dx * dx + dy * dy) ** 0.5 < self.min_target_shift_m:
                return

        self.get_logger().info(
            f'检测到垃圾后自动触发抓取: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, label={self.current_label}'
        )
        self._start_pick(msg)

    def _start_pick(self, msg: PoseStamped) -> None:
        if self.pick_in_progress:
            return
        self.pick_in_progress = True
        self.last_pick_time = time.time()
        self.last_pick_target = (float(msg.pose.position.x), float(msg.pose.position.y))
        threading.Thread(target=self._execute_pick, args=(msg,), daemon=True).start()

    def _execute_pick(self, msg: PoseStamped) -> None:
        self._publish_busy(True)

        success = False
        error_message = ''
        try:
            if self.grasp_controller is not None:
                success = self._run_real_grasp(msg)
            else:
                success = self._run_mock_grasp(msg)
        except Exception as exc:
            error_message = str(exc)

        result = String()
        if success:
            result.data = 'grasp_finished'
            self.get_logger().info('抓取流程已完成，机械臂回到等待状态。')
        else:
            result.data = f'grasp_failed:{error_message}' if error_message else 'grasp_failed'
            self.get_logger().warning(f'抓取流程失败: {error_message}')

        self.result_pub.publish(result)
        self._publish_busy(False)
        self.pick_in_progress = False

    def _run_real_grasp(self, msg: PoseStamped) -> bool:
        source_label = self._normalize_label(self.current_label or 'plastic')
        execute_label = self._resolve_drop_label(source_label)
        detect_msg = {
            execute_label: (-float(msg.pose.position.y), float(msg.pose.position.x))
        }
        joints = [
            int(self.get_parameter('joint1').value),
            int(self.get_parameter('joint2').value),
            int(self.get_parameter('joint3').value),
            int(self.get_parameter('joint4').value),
            int(self.get_parameter('joint5').value),
            int(self.get_parameter('joint6').value),
        ]
        joint1456 = [
            joints[0],
            joints[3],
            joints[4],
            joints[5],
        ]
        height_profile = self._get_height_profile(source_label)

        self.get_logger().info(
            f'开始真实抓取: source_label={source_label}, execute_label={execute_label}, '
            f'approach_z={height_profile["approach_z"]}, grasp_z={height_profile["grasp_z"]}'
        )

        if self.supports_height_profile:
            self.grasp_controller.grasp_run(
                'sorting',
                'garbage',
                detect_msg,
                joint1456,
                height_profile=height_profile,
            )
        else:
            self.get_logger().warning(
                '当前GraspController是旧接口，改用grasp_executor内置流程执行分类高度抓取。'
            )
            self._run_legacy_profile_grasp(
                detect_pos=detect_msg[execute_label],
                execute_label=execute_label,
                joint1456=joint1456,
                joints=joints,
                height_profile=height_profile,
            )
        return True

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
        coords_init = [x, y, approach_z, -175, 0, -45]

        self.get_logger().info(
            f'旧接口兼容抓取: target={execute_label}, '
            f'x={x:.1f}mm, y={y:.1f}mm, '
            f'approach_z={approach_z}, grasp_z={grasp_z}'
        )

        self.grasp_controller.go_coords(coords_init, 3)
        self.grasp_controller.ctrl_gripper_height(grasp_z, 1.0)
        self.grasp_controller.close_gripper(1.5)
        self.grasp_controller.ctrl_gripper_height(approach_z, 2.0)
        self.grasp_controller.goOverPose('sorting', 'garbage')
        self.grasp_controller.goTargetPose('sorting', 'garbage', '1', execute_label)
        self.grasp_controller.open_gripper(1.0)
        self.grasp_controller.lift_gripper('sorting', 'garbage', '1', execute_label)
        self.grasp_controller.goOverPose('sorting', 'garbage')

        self.grasp_controller.go_angles(joints, 2)

        if hasattr(self.grasp_controller, 'func_start'):
            self.grasp_controller.func_start = False

    def _run_mock_grasp(self, msg: PoseStamped) -> bool:
        source_label = self._normalize_label(self.current_label)
        height_profile = self._get_height_profile(source_label)
        self.get_logger().info(
            f'当前使用模拟抓取: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, label={source_label}, '
            f'approach_z={height_profile["approach_z"]}, '
            f'grasp_z={height_profile["grasp_z"]}'
        )
        time.sleep(2.0)
        return True

    def _normalize_label(self, label: str) -> str:
        label_text = str(label or '').strip()
        alias_key = label_text.lower()
        return self.label_aliases.get(alias_key, label_text)

    def _resolve_drop_label(self, source_label: str) -> str:
        if self.drop_mode == 'single_bag':
            return self.drop_label
        return source_label

    def _get_height_profile(self, label: str) -> dict:
        normalized_label = self._normalize_label(label)
        profile = self.grasp_profiles.get(normalized_label)
        if profile is not None:
            return profile
        return dict(self.default_profile)

    def _publish_busy(self, busy: bool) -> None:
        msg = Bool()
        msg.data = busy
        self.arm_busy_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GraspExecutor()
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
