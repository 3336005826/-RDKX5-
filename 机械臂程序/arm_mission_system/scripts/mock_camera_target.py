#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node


class MockCameraTarget(Node):
    def __init__(self) -> None:
        super().__init__('mock_camera_target')

        self.publisher = self.create_publisher(PoseStamped, '/mission/trash_pose', 10)
        self.create_timer(1.0, self._publish_target)

    def _publish_target(self) -> None:
        msg = PoseStamped()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.55
        msg.pose.position.y = 0.05
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._yaw_to_quaternion(0.0)
        self.publisher.publish(msg)

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main() -> None:
    rclpy.init()
    node = MockCameraTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
