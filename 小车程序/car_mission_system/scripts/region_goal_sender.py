#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node


class RegionGoalSender(Node):
    def __init__(self) -> None:
        super().__init__('region_goal_sender')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('region_x', 1.0)
        self.declare_parameter('region_y', 0.0)
        self.declare_parameter('region_yaw', 0.0)

        self.publisher = self.create_publisher(PoseStamped, '/mission/region_pose', 10)
        self.create_timer(1.0, self._publish_once)
        self._sent = False

    def _publish_once(self) -> None:
        if self._sent:
            return

        msg = PoseStamped()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self.get_parameter('region_x').value)
        msg.pose.position.y = float(self.get_parameter('region_y').value)
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._yaw_to_quaternion(float(self.get_parameter('region_yaw').value))

        self.publisher.publish(msg)
        self._sent = True
        self.get_logger().info('已发布示例区域入口点。')

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main() -> None:
    rclpy.init()
    node = RegionGoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
