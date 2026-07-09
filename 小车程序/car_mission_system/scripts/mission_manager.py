#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Twist
from nav2_msgs.action import BackUp, FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class MissionState(Enum):
    IDLE = 'idle'
    NAVIGATING_TO_SITE = 'navigating_to_site'
    SHUTTLE_TO_GOAL = 'shuttle_to_goal'
    SHUTTLE_RETURNING = 'shuttle_returning'
    WAITING_REGION_SELECTION = 'waiting_region_selection'
    COVERAGE_RUNNING = 'coverage_running'
    PAUSING_FOR_PICK = 'pausing_for_pick'
    ARM_PICKING = 'arm_picking'
    RETURNING_HOME = 'returning_home'


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager')

        self.declare_parameter('auto_return_home_after_coverage', True)
        self.declare_parameter('stop_cmd_repeat', 5)
        self.declare_parameter('picked_target_merge_distance_m', 0.18)
        self.declare_parameter('max_picked_targets', 200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_warn_after_sec', 2.0)
        self.declare_parameter('enable_backup_recovery', False)
        self.declare_parameter('backup_distance_m', 0.25)
        self.declare_parameter('backup_speed_mps', 0.08)
        self.declare_parameter('backup_time_allowance_sec', 6.0)
        self.declare_parameter('max_backup_retries', 1)
        self.declare_parameter('shuttle_record_current_pose', True)
        self.declare_parameter('shuttle_map_frame', 'map')
        self.declare_parameter('shuttle_base_frame', 'base_footprint')
        self.declare_parameter('shuttle_base_frame_fallback', 'base_link')

        self.state = MissionState.IDLE
        self.home_pose: Optional[PoseStamped] = None
        self.site_goal_pose: Optional[PoseStamped] = None
        self.shuttle_goal_pose: Optional[PoseStamped] = None
        self.shuttle_resume_pose: Optional[PoseStamped] = None
        self.shuttle_resume_state: Optional[MissionState] = None
        self.pick_resume_kind = ''
        self.coverage_waypoints: List[PoseStamped] = []
        self.current_waypoint_index = 0
        self.coverage_resume_index = 0
        self.latest_trash_pose: Optional[PoseStamped] = None
        self.latest_trash_label = 'unknown'
        self.pending_pick_target: Optional[PoseStamped] = None
        self.arm_busy = False
        self.last_grasp_result = ''
        self.pending_return_home = False
        self.click_mode = 'nav'

        self.nav_goal_handle = None
        self.nav_goal_serial = 0
        self.coverage_goal_handle = None
        self.coverage_goal_serial = 0
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value or '/cmd_vel')
        self.cmd_vel_msg_count = 0
        self.cmd_vel_count_at_motion_start = 0
        self.last_cmd_vel_time = None
        self.last_nonzero_cmd_vel_time = None
        self.last_motion_diag_time = None
        self.last_nav_feedback_report_time = None
        self.last_distance_remaining = None
        self.spin_warning_start_time = None
        self.last_cmd_vel_linear = 0.0
        self.last_cmd_vel_angular = 0.0
        self.motion_start_time = None
        self.motion_start_reason = ''
        self.active_nav_pose: Optional[PoseStamped] = None
        self.active_nav_target_state: Optional[MissionState] = None
        self.nav_recovery_attempts = 0
        self.coverage_recovery_attempts = 0
        self.pending_backup_recovery_kind = ''
        self.pending_backup_coverage_start = 0

        self.picked_targets: List[Tuple[float, float, str]] = []
        self.active_pick_signature: Optional[Tuple[float, float, str]] = None

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.arm_pick_pub = self.create_publisher(PoseStamped, '/mission/arm_pick_target', 10)
        self.stop_cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(PoseStamped, '/mission/home_pose', self._on_home_pose, 10)
        self.create_subscription(PointStamped, '/mission/home_point', self._on_home_point, 10)
        self.create_subscription(PoseStamped, '/mission/nav_goal_pose', self._on_nav_goal_pose, 10)
        self.create_subscription(PoseStamped, '/mission/shuttle_goal_pose', self._on_shuttle_goal_pose, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose_alias, 10)
        self.create_subscription(String, '/station/click_mode', self._on_click_mode, 10)
        self.create_subscription(PoseStamped, '/mission/region_pose', self._on_legacy_region_pose, 10)
        self.create_subscription(PoseArray, '/mission/coverage_waypoints', self._on_coverage_waypoints, 10)
        self.create_subscription(PoseStamped, '/mission/trash_pose', self._on_trash_pose, 10)
        self.create_subscription(String, '/mission/trash_label', self._on_trash_label, 10)
        self.create_subscription(String, '/arm/grasp_result', self._on_grasp_result, 10)
        self.create_subscription(Bool, '/mission/arm_busy', self._on_arm_busy, 10)
        self.create_subscription(Bool, '/mission/return_home', self._on_return_home, 10)
        self.create_subscription(Bool, '/mission/clear_region', self._on_clear_region, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)

        self.create_timer(0.5, self._publish_state)
        self.create_timer(1.0, self._publish_motion_diagnostics)
        self.get_logger().info(f'[MOTION] watching velocity topic: {self.cmd_vel_topic}')
        self.get_logger().info('小车任务状态机已启动。')

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel_msg_count += 1
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_cmd_vel_linear = float(msg.linear.x)
        self.last_cmd_vel_angular = float(msg.angular.z)
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_nonzero_cmd_vel_time = self.last_cmd_vel_time

    def _start_motion_watch(self, reason: str) -> None:
        self.motion_start_time = self.get_clock().now()
        self.motion_start_reason = reason
        self.cmd_vel_count_at_motion_start = self.cmd_vel_msg_count
        self.last_motion_diag_time = None
        self.last_nav_feedback_report_time = None
        self.spin_warning_start_time = None

    def _seconds_since(self, stamp) -> Optional[float]:
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def _publish_motion_diagnostics(self) -> None:
        if self.state not in (
            MissionState.NAVIGATING_TO_SITE,
            MissionState.SHUTTLE_TO_GOAL,
            MissionState.SHUTTLE_RETURNING,
            MissionState.COVERAGE_RUNNING,
            MissionState.RETURNING_HOME,
        ):
            return

        now = self.get_clock().now()
        if (
            self.last_motion_diag_time is not None
            and (now - self.last_motion_diag_time).nanoseconds < 2_000_000_000
        ):
            return
        self.last_motion_diag_time = now

        warn_after = float(self.get_parameter('cmd_vel_warn_after_sec').value)
        elapsed = self._seconds_since(self.motion_start_time) or 0.0
        new_cmd_count = self.cmd_vel_msg_count - self.cmd_vel_count_at_motion_start
        if new_cmd_count <= 0:
            if elapsed >= warn_after:
                self.get_logger().warning(
                    f'[MOTION] state={self.state.value} reason={self.motion_start_reason}: '
                    f'no new Twist on {self.cmd_vel_topic} after {elapsed:.1f}s. '
                    'Planner may have a path, but controller/base is not outputting velocity.'
                )
            return

        age = self._seconds_since(self.last_cmd_vel_time)
        nonzero_age = self._seconds_since(self.last_nonzero_cmd_vel_time)
        if age is not None and age > warn_after:
            self.get_logger().warning(
                f'[MOTION] state={self.state.value}: last Twist on {self.cmd_vel_topic} '
                f'is stale ({age:.1f}s old).'
            )
            return

        if (
            abs(self.last_cmd_vel_linear) <= 1e-3
            and abs(self.last_cmd_vel_angular) <= 1e-3
            and (nonzero_age is None or nonzero_age > warn_after)
        ):
            self.get_logger().warning(
                f'[MOTION] state={self.state.value}: Twist is zero on {self.cmd_vel_topic}. '
                'Check local costmap obstacles, TF, controller_server, and goal tolerance.'
            )
            return

        distance = self.last_distance_remaining
        if (
            distance is not None
            and distance > 0.5
            and abs(self.last_cmd_vel_linear) < 0.04
            and abs(self.last_cmd_vel_angular) > 0.20
        ):
            if self.spin_warning_start_time is None:
                self.spin_warning_start_time = now
            spin_seconds = (now - self.spin_warning_start_time).nanoseconds / 1e9
            if spin_seconds >= 3.0:
                self.get_logger().warning(
                    f'[MOTION] spinning while far from goal: distance={distance:.2f}m, '
                    f'cmd_vel v={self.last_cmd_vel_linear:.3f}, '
                    f'w={self.last_cmd_vel_angular:.3f}. '
                    'Check rotation shim threshold, local costmap obstacles, and odom/angular sign.'
                )
                return
        else:
            self.spin_warning_start_time = None

        self.get_logger().info(
            f'[MOTION] state={self.state.value}: cmd_vel v={self.last_cmd_vel_linear:.3f}, '
            f'w={self.last_cmd_vel_angular:.3f}'
        )

    def _goal_status_label(self, status: int) -> str:
        labels = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return labels.get(status, f'UNKNOWN_STATUS_{status}')

    def _pose_yaw(self, pose: PoseStamped) -> float:
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_home_pose(self, msg: PoseStamped) -> None:
        self.home_pose = msg
        self.get_logger().info(
            f'已更新回家点: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )

    def _on_home_point(self, msg: PointStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = msg.point.z
        pose.pose.orientation.w = 1.0
        self._on_home_pose(pose)

    def _lookup_current_robot_pose(self) -> Optional[PoseStamped]:
        target_frame = str(self.get_parameter('shuttle_map_frame').value or 'map')
        base_frame = str(self.get_parameter('shuttle_base_frame').value or 'base_footprint')
        fallback_frame = str(self.get_parameter('shuttle_base_frame_fallback').value or '').strip()
        base_frames = [base_frame]
        if fallback_frame and fallback_frame not in base_frames:
            base_frames.append(fallback_frame)

        last_error = None
        for source_frame in base_frames:
            try:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
            except TransformException as exc:
                last_error = exc
                continue

            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.get_logger().info(
                f'[SHUTTLE] recorded current pose as return origin from TF '
                f'{target_frame}->{source_frame}: '
                f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
            )
            return pose

        self.get_logger().warning(
            f'[SHUTTLE] cannot record current pose from TF. '
            f'target_frame={target_frame}, base_frames={base_frames}, error={last_error}'
        )
        return None

    def _on_nav_goal_pose(self, msg: PoseStamped) -> None:
        self.site_goal_pose = msg
        self.get_logger().info(
            f'收到作业现场导航点: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )
        self._navigate_to(msg, MissionState.NAVIGATING_TO_SITE)

    def _on_shuttle_goal_pose(self, msg: PoseStamped) -> None:
        if bool(self.get_parameter('shuttle_record_current_pose').value):
            current_pose = self._lookup_current_robot_pose()
            if current_pose is not None:
                self.home_pose = current_pose

        if self.home_pose is None:
            self.get_logger().warning(
                '[SHUTTLE] return origin is not available, cannot start shuttle navigation. '
                'Set /mission/home_pose or check TF map->base_footprint.'
            )
            return

        self.shuttle_goal_pose = msg
        self.shuttle_resume_pose = None
        self.shuttle_resume_state = None
        self.pick_resume_kind = ''
        self.pending_pick_target = None
        self.active_pick_signature = None
        self.picked_targets.clear()
        self.get_logger().info(
            f'[SHUTTLE] received target: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )

        if self.coverage_goal_handle is not None:
            self.get_logger().info('[SHUTTLE] cancel current coverage before shuttle navigation.')
            cancel_future = self.coverage_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _future: self._start_shuttle_to_goal())
            self._publish_stop_cmd()
            return

        self._start_shuttle_to_goal()

    def _start_shuttle_to_goal(self) -> None:
        if self.shuttle_goal_pose is None:
            return
        self.coverage_goal_handle = None
        self.get_logger().info('[SHUTTLE] navigating to target point.')
        self._navigate_to(self.shuttle_goal_pose, MissionState.SHUTTLE_TO_GOAL)

    def _on_goal_pose_alias(self, msg: PoseStamped) -> None:
        if self.click_mode in ('shuttle', 'round_trip'):
            self._on_shuttle_goal_pose(msg)
            return
        if self.click_mode not in ('nav', 'navigation', 'goal'):
            self.get_logger().info(f'Ignored /goal_pose while station mode is {self.click_mode}.')
            return
        self._on_nav_goal_pose(msg)

    def _on_click_mode(self, msg: String) -> None:
        value = msg.data.strip()
        if value:
            self.click_mode = value

    def _on_legacy_region_pose(self, msg: PoseStamped) -> None:
        self.site_goal_pose = msg
        self.get_logger().info(
            f'收到旧版区域入口点: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )
        self._navigate_to(msg, MissionState.NAVIGATING_TO_SITE)

    def _on_coverage_waypoints(self, msg: PoseArray) -> None:
        self.coverage_waypoints = []
        for pose in msg.poses:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = pose
            self.coverage_waypoints.append(pose_stamped)

        self.current_waypoint_index = 0
        self.coverage_resume_index = 0
        self.picked_targets.clear()
        self.active_pick_signature = None

        if not self.coverage_waypoints:
            self.get_logger().warning('收到的扫荡路径为空。')
            return

        self.get_logger().info(f'已生成扫荡路径，共 {len(self.coverage_waypoints)} 个路径点。')

        if self.state in (MissionState.WAITING_REGION_SELECTION, MissionState.IDLE):
            self._follow_coverage_path(start_index=0)

    def _on_trash_label(self, msg: String) -> None:
        self.latest_trash_label = msg.data.strip() or 'unknown'

    def _on_grasp_result(self, msg: String) -> None:
        self.last_grasp_result = msg.data.strip()

    def _on_trash_pose(self, msg: PoseStamped) -> None:
        self.latest_trash_pose = msg

        if self.arm_busy:
            return
        if self.state not in (
            MissionState.COVERAGE_RUNNING,
            MissionState.SHUTTLE_TO_GOAL,
            MissionState.SHUTTLE_RETURNING,
        ):
            return
        if self.pending_pick_target is not None:
            return

        signature = self._make_target_signature(msg, self.latest_trash_label)
        if self._is_target_already_picked(signature):
            return

        self.pending_pick_target = msg
        self.active_pick_signature = signature
        if self.state == MissionState.COVERAGE_RUNNING:
            self.pick_resume_kind = 'coverage'
            self.coverage_resume_index = min(self.current_waypoint_index, max(len(self.coverage_waypoints) - 1, 0))
        else:
            self.pick_resume_kind = 'shuttle'
            self.shuttle_resume_pose = self.active_nav_pose
            self.shuttle_resume_state = self.active_nav_target_state or self.state
        self.state = MissionState.PAUSING_FOR_PICK
        self._publish_stop_cmd()

        self.get_logger().info(
            f'任务中发现垃圾，准备暂停并抓取: label={self.latest_trash_label}, '
            f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
        )

        if self.pick_resume_kind == 'coverage' and self.coverage_goal_handle is not None:
            cancel_future = self.coverage_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_coverage_cancel_done)
        elif self.pick_resume_kind == 'shuttle' and self.nav_goal_handle is not None:
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_nav_cancel_for_pick_done)
        else:
            self._dispatch_pick_target()

    def _on_coverage_cancel_done(self, future) -> None:
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warning(f'暂停扫荡失败，将直接尝试抓取: {exc}')
        self._dispatch_pick_target()

    def _on_nav_cancel_for_pick_done(self, future) -> None:
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warning(f'[SHUTTLE] cancel navigation before pick failed, try pick directly: {exc}')
        self._dispatch_pick_target()

    def _dispatch_pick_target(self) -> None:
        if self.pending_pick_target is None:
            return

        self._publish_stop_cmd()
        self.last_grasp_result = ''
        self.arm_pick_pub.publish(self.pending_pick_target)
        self.get_logger().info('已将抓取目标发送给机械臂。')

    def _on_arm_busy(self, msg: Bool) -> None:
        previous_busy = self.arm_busy
        self.arm_busy = msg.data

        if self.arm_busy:
            self.state = MissionState.ARM_PICKING
            self.get_logger().info('机械臂抓取中。')
            return

        if previous_busy and not self.arm_busy and self.state in (
            MissionState.ARM_PICKING,
            MissionState.PAUSING_FOR_PICK,
        ):
            if self.last_grasp_result == 'grasp_finished':
                self._remember_active_pick_target()
            else:
                self.get_logger().warning(
                    f'本次抓取未成功，不做去重记录。result={self.last_grasp_result or "unknown"}'
                )

            resume_kind = self.pick_resume_kind
            resume_pose = self.shuttle_resume_pose
            resume_state = self.shuttle_resume_state
            self.pending_pick_target = None
            self.active_pick_signature = None
            self.pick_resume_kind = ''
            self.shuttle_resume_pose = None
            self.shuttle_resume_state = None

            if resume_kind == 'coverage':
                self.get_logger().info('机械臂抓取结束，继续扫荡。')
                self._follow_coverage_path(start_index=self.coverage_resume_index)
                return

            if resume_kind == 'shuttle' and resume_pose is not None and resume_state is not None:
                self.get_logger().info('[SHUTTLE] arm pick finished, resume shuttle navigation.')
                self._navigate_to(resume_pose, resume_state, reset_recovery=False)
                return

            self.get_logger().info('机械臂抓取结束，没有需要恢复的任务。')
            self.state = MissionState.IDLE

    def _on_return_home(self, msg: Bool) -> None:
        if msg.data:
            if self.coverage_goal_handle is not None:
                self.pending_return_home = True
                self.get_logger().info('收到回家指令，先取消当前扫荡任务。')
                cancel_future = self.coverage_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_return_home_cancel_done)
                self._publish_stop_cmd()
            else:
                self._start_return_home()

    def _on_clear_region(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.coverage_goal_handle is not None:
            self.get_logger().info('收到清除区域指令，正在取消当前扫荡任务。')
            cancel_future = self.coverage_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_clear_region_cancel_done)
            self._publish_stop_cmd()
        self.coverage_waypoints = []
        self.current_waypoint_index = 0
        self.coverage_resume_index = 0
        self.pending_pick_target = None
        self.active_pick_signature = None
        self.pick_resume_kind = ''
        self.shuttle_resume_pose = None
        self.shuttle_resume_state = None
        self.picked_targets.clear()
        self.coverage_goal_serial += 1
        self.get_logger().info('已清除当前区域和已抓取目标缓存。')
        if self.state != MissionState.RETURNING_HOME:
            self.state = MissionState.WAITING_REGION_SELECTION

    def _on_return_home_cancel_done(self, future) -> None:
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warning(f'取消扫荡后回家时出现异常: {exc}')
        self.coverage_goal_handle = None
        self.pending_return_home = False
        self._start_return_home()

    def _on_clear_region_cancel_done(self, future) -> None:
        try:
            _ = future.result()
        except Exception as exc:
            self.get_logger().warning(f'取消扫荡任务时出现异常: {exc}')
        self.coverage_goal_handle = None

    def _start_return_home(self) -> None:
        if self.home_pose is None:
            self.get_logger().warning('未设置回家点，无法自动回家。')
            return
        self.get_logger().info('开始返回初始位置。')
        self._navigate_to(self.home_pose, MissionState.RETURNING_HOME)

    def _navigate_to(
        self,
        pose: PoseStamped,
        target_state: MissionState,
        reset_recovery: bool = True,
    ) -> None:
        if reset_recovery:
            self.active_nav_pose = pose
            self.active_nav_target_state = target_state
            self.nav_recovery_attempts = 0

        frame_id = pose.header.frame_id or 'map'
        yaw = self._pose_yaw(pose)
        self.get_logger().info(
            f'[NAV] send NavigateToPose state={target_state.value} frame={frame_id} '
            f'x={pose.pose.position.x:.3f} y={pose.pose.position.y:.3f} yaw={yaw:.3f}'
        )
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning('NavigateToPose 动作服务暂时不可用。')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.state = target_state
        self._start_motion_watch(f'navigate:{target_state.value}')
        self.nav_goal_serial += 1
        goal_serial = self.nav_goal_serial
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(
            lambda result_future, serial=goal_serial: self._handle_nav_goal_response(
                result_future,
                serial,
            )
        )

    def _handle_nav_goal_response(self, future, goal_serial: int) -> None:
        if goal_serial != self.nav_goal_serial:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'[NAV] goal request failed: {exc}')
            self.state = MissionState.IDLE
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('导航目标被拒绝。')
            self.state = MissionState.IDLE
            return

        self.get_logger().info('[NAV] goal accepted by NavigateToPose server.')
        self.nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, serial=goal_serial: self._handle_nav_result(result, serial)
        )

    def _handle_nav_result(self, future, goal_serial: int) -> None:
        if goal_serial != self.nav_goal_serial:
            return
        try:
            result_response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'导航结果异常: {exc}')
            self.state = MissionState.IDLE
            return

        status = int(getattr(result_response, 'status', GoalStatus.STATUS_UNKNOWN))
        status_label = self._goal_status_label(status)
        self.nav_goal_handle = None
        self.get_logger().info(f'[NAV] result status={status_label} state={self.state.value}')
        if self.state in (MissionState.PAUSING_FOR_PICK, MissionState.ARM_PICKING):
            self.get_logger().info('[NAV] result ignored while arm pick is in progress.')
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f'[NAV] NavigateToPose did not succeed: status={status_label}, '
                f'result={getattr(result_response, "result", None)}'
            )
            if status == GoalStatus.STATUS_ABORTED and self._try_backup_recovery('nav', status_label):
                return
            self.state = MissionState.IDLE
            return

        if self.state == MissionState.NAVIGATING_TO_SITE:
            if self.coverage_waypoints:
                self.get_logger().info('已到达现场，开始扫荡区域。')
                self._follow_coverage_path(start_index=0)
            else:
                self.state = MissionState.WAITING_REGION_SELECTION
                self.get_logger().info('已到达现场，等待框选垃圾区域。')
            return

        if self.state == MissionState.SHUTTLE_TO_GOAL:
            if self.home_pose is None:
                self.get_logger().warning('[SHUTTLE] reached target, but home pose is missing.')
                self.state = MissionState.IDLE
                return
            self.get_logger().info('[SHUTTLE] reached target point, returning home.')
            self._navigate_to(self.home_pose, MissionState.SHUTTLE_RETURNING)
            return

        if self.state == MissionState.SHUTTLE_RETURNING:
            self.state = MissionState.IDLE
            self.get_logger().info('[SHUTTLE] returned home, shuttle mission finished.')
            return

        if self.state == MissionState.RETURNING_HOME:
            self.state = MissionState.IDLE
            self.get_logger().info('已返回初始位置，任务结束。')

    def _on_nav_feedback(self, feedback_msg) -> None:
        now = self.get_clock().now()
        if (
            self.last_nav_feedback_report_time is not None
            and (now - self.last_nav_feedback_report_time).nanoseconds < 2_000_000_000
        ):
            return
        self.last_nav_feedback_report_time = now

        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        recoveries = getattr(feedback, 'number_of_recoveries', None)
        if distance is None:
            self.get_logger().info('[NAV] feedback received.')
            return
        self.last_distance_remaining = float(distance)
        self.get_logger().info(
            f'[NAV] feedback distance_remaining={float(distance):.3f} '
            f'recoveries={recoveries if recoveries is not None else "-"}'
        )

    def _follow_coverage_path(self, start_index: int, reset_recovery: bool = True) -> None:
        if not self.coverage_waypoints:
            self.get_logger().warning('当前没有可执行的扫荡路径。')
            self.state = MissionState.WAITING_REGION_SELECTION
            return
        if reset_recovery:
            self.coverage_recovery_attempts = 0
        if start_index >= len(self.coverage_waypoints):
            self._on_coverage_complete()
            return
        if not self.follow_waypoints_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning('FollowWaypoints 动作服务暂时不可用。')
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.coverage_waypoints[start_index:]
        self.current_waypoint_index = start_index
        self.coverage_resume_index = start_index
        self.state = MissionState.COVERAGE_RUNNING
        self._start_motion_watch(f'coverage:{start_index + 1}/{len(self.coverage_waypoints)}')
        self.pending_pick_target = None

        self.coverage_goal_serial += 1
        goal_serial = self.coverage_goal_serial

        future = self.follow_waypoints_client.send_goal_async(
            goal,
            feedback_callback=self._on_coverage_feedback,
        )
        future.add_done_callback(
            lambda result_future, serial=goal_serial: self._handle_waypoint_goal_response(
                result_future,
                serial,
            )
        )
        self.get_logger().info(
            f'开始执行扫荡路径，从第 {start_index + 1}/{len(self.coverage_waypoints)} 个路径点继续。'
        )

    def _handle_waypoint_goal_response(self, future, goal_serial: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'[WAYPOINT] goal request failed: {exc}')
            self.state = MissionState.IDLE
            return
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('扫荡目标被拒绝。')
            self.state = MissionState.IDLE
            return

        self.get_logger().info('[WAYPOINT] goal accepted by FollowWaypoints server.')
        self.coverage_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, serial=goal_serial: self._handle_waypoint_result(result, serial)
        )

    def _on_coverage_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'current_waypoint'):
            self.current_waypoint_index = self.coverage_resume_index + int(feedback.current_waypoint)

    def _handle_waypoint_result(self, future, goal_serial: int) -> None:
        if goal_serial != self.coverage_goal_serial:
            return

        try:
            result_response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'扫荡执行异常: {exc}')
            self.state = MissionState.IDLE
            return

        self.coverage_goal_handle = None

        if self.state in (MissionState.PAUSING_FOR_PICK, MissionState.ARM_PICKING):
            return

        status = int(getattr(result_response, 'status', GoalStatus.STATUS_UNKNOWN))
        status_label = self._goal_status_label(status)
        self.get_logger().info(f'[WAYPOINT] result status={status_label} state={self.state.value}')

        if self.state != MissionState.COVERAGE_RUNNING:
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f'[WAYPOINT] FollowWaypoints did not succeed: status={status_label}, '
                f'result={getattr(result_response, "result", None)}'
            )
            if status == GoalStatus.STATUS_ABORTED and self._try_backup_recovery('coverage', status_label):
                return
            self.state = MissionState.WAITING_REGION_SELECTION
            return

        if self.state == MissionState.COVERAGE_RUNNING:
            self._on_coverage_complete()

    def _try_backup_recovery(self, kind: str, status_label: str) -> bool:
        if not bool(self.get_parameter('enable_backup_recovery').value):
            return False

        max_retries = int(self.get_parameter('max_backup_retries').value)
        if kind == 'nav':
            if self.active_nav_pose is None or self.active_nav_target_state is None:
                return False
            if self.nav_recovery_attempts >= max_retries:
                self.get_logger().warning('[BACKUP] nav recovery retry limit reached.')
                return False
            self.nav_recovery_attempts += 1
        elif kind == 'coverage':
            if not self.coverage_waypoints:
                return False
            if self.coverage_recovery_attempts >= max_retries:
                self.get_logger().warning('[BACKUP] coverage recovery retry limit reached.')
                return False
            self.coverage_recovery_attempts += 1
            self.pending_backup_coverage_start = min(
                max(self.current_waypoint_index, 0),
                max(len(self.coverage_waypoints) - 1, 0),
            )
        else:
            return False

        if not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning(
                '[BACKUP] /backup action server is not available. '
                'Check Nav2 behavior_server and backup behavior plugin.'
            )
            return False

        distance = abs(float(self.get_parameter('backup_distance_m').value))
        speed = abs(float(self.get_parameter('backup_speed_mps').value))
        allowance = max(1.0, float(self.get_parameter('backup_time_allowance_sec').value))

        goal = BackUp.Goal()
        goal.target.x = -distance
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = speed
        goal.time_allowance = Duration(
            sec=int(allowance),
            nanosec=int((allowance - int(allowance)) * 1e9),
        )

        self.pending_backup_recovery_kind = kind
        self._publish_stop_cmd()
        self.get_logger().warning(
            f'[BACKUP] {kind} got {status_label}; backing up {distance:.2f}m '
            f'at {speed:.2f}m/s, then replanning.'
        )
        future = self.backup_client.send_goal_async(goal)
        future.add_done_callback(self._handle_backup_goal_response)
        return True

    def _handle_backup_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'[BACKUP] goal request failed: {exc}')
            self._finish_failed_backup()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning('[BACKUP] goal rejected.')
            self._finish_failed_backup()
            return

        self.get_logger().info('[BACKUP] goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_backup_result)

    def _handle_backup_result(self, future) -> None:
        try:
            result_response = future.result()
        except Exception as exc:
            self.get_logger().warning(f'[BACKUP] result failed: {exc}')
            self._finish_failed_backup()
            return

        status = int(getattr(result_response, 'status', GoalStatus.STATUS_UNKNOWN))
        status_label = self._goal_status_label(status)
        self.get_logger().info(f'[BACKUP] result status={status_label}')
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._finish_failed_backup()
            return

        kind = self.pending_backup_recovery_kind
        self.pending_backup_recovery_kind = ''

        if kind == 'nav' and self.active_nav_pose is not None and self.active_nav_target_state is not None:
            self.get_logger().info('[BACKUP] retrying NavigateToPose after backup.')
            self._navigate_to(self.active_nav_pose, self.active_nav_target_state, reset_recovery=False)
            return

        if kind == 'coverage':
            self.get_logger().info('[BACKUP] retrying FollowWaypoints after backup.')
            self._follow_coverage_path(self.pending_backup_coverage_start, reset_recovery=False)
            return

        self._finish_failed_backup()

    def _finish_failed_backup(self) -> None:
        kind = self.pending_backup_recovery_kind
        self.pending_backup_recovery_kind = ''
        if kind == 'coverage':
            self.state = MissionState.WAITING_REGION_SELECTION
        else:
            self.state = MissionState.IDLE

    def _on_coverage_complete(self) -> None:
        self.get_logger().info('当前区域扫荡完成。')
        if bool(self.get_parameter('auto_return_home_after_coverage').value):
            self._start_return_home()
        else:
            self.state = MissionState.WAITING_REGION_SELECTION
            self.get_logger().info('等待下一次区域任务。')

    def _publish_stop_cmd(self) -> None:
        repeat = int(self.get_parameter('stop_cmd_repeat').value)
        msg = Twist()
        for _ in range(max(1, repeat)):
            self.stop_cmd_pub.publish(msg)

    def _make_target_signature(self, target: PoseStamped, label: str) -> Tuple[float, float, str]:
        return (
            float(target.pose.position.x),
            float(target.pose.position.y),
            label.strip().lower() or 'unknown',
        )

    def _is_target_already_picked(self, signature: Tuple[float, float, str]) -> bool:
        merge_distance = float(self.get_parameter('picked_target_merge_distance_m').value)
        x, y, label = signature
        for picked_x, picked_y, picked_label in self.picked_targets:
            if picked_label != label:
                continue
            if math.hypot(x - picked_x, y - picked_y) <= merge_distance:
                return True
        return False

    def _remember_active_pick_target(self) -> None:
        if self.active_pick_signature is None:
            return

        self.picked_targets.append(self.active_pick_signature)
        max_targets = int(self.get_parameter('max_picked_targets').value)
        if len(self.picked_targets) > max_targets:
            self.picked_targets = self.picked_targets[-max_targets:]


def main() -> None:
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
