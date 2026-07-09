#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class BasePickAssistant(Node):
    def __init__(self) -> None:
        super().__init__('base_pick_assistant')

        self.declare_parameter('enable_assist', True)
        self.declare_parameter('target_x_m', 0.18)
        self.declare_parameter('x_deadband_m', 0.015)
        self.declare_parameter('y_deadband_m', 0.015)
        self.declare_parameter('max_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 0.20)
        self.declare_parameter('forward_gain', 0.8)
        self.declare_parameter('turn_gain', 1.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.arm_busy = False
        self.latest_pose = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Bool, '/mission/arm_busy', self._on_arm_busy, 10)
        self.create_subscription(PoseStamped, '/mission/trash_pose', self._on_trash_pose, 10)

        timer_period = 1.0 / max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info('Base pick assistant started.')

    def _on_arm_busy(self, msg: Bool) -> None:
        self.arm_busy = msg.data
        if not self.arm_busy:
            self._publish_stop()

    def _on_trash_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def _on_timer(self) -> None:
        if not bool(self.get_parameter('enable_assist').value):
            return
        if not self.arm_busy:
            return
        if self.latest_pose is None:
            self._publish_stop()
            return

        target_x = float(self.get_parameter('target_x_m').value)
        x_deadband = float(self.get_parameter('x_deadband_m').value)
        y_deadband = float(self.get_parameter('y_deadband_m').value)
        max_linear = float(self.get_parameter('max_linear_speed').value)
        max_angular = float(self.get_parameter('max_angular_speed').value)
        forward_gain = float(self.get_parameter('forward_gain').value)
        turn_gain = float(self.get_parameter('turn_gain').value)

        error_x = self.latest_pose.pose.position.x - target_x
        error_y = self.latest_pose.pose.position.y

        cmd = Twist()

        if abs(error_x) > x_deadband:
            cmd.linear.x = self._clamp(error_x * forward_gain, -max_linear, max_linear)

        if abs(error_y) > y_deadband:
            cmd.angular.z = self._clamp(-error_y * turn_gain, -max_angular, max_angular)

        if abs(cmd.linear.x) < 1e-4 and abs(cmd.angular.z) < 1e-4:
            self._publish_stop()
            return

        self.cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min_value, min(max_value, value))


def main() -> None:
    rclpy.init()
    node = BasePickAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
