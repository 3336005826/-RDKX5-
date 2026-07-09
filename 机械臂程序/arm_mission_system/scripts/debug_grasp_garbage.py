#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class DebugGraspGarbage(Node):
    def __init__(self) -> None:
        super().__init__('debug_grasp_garbage')

        self.declare_parameter('x', 0.55)
        self.declare_parameter('y', 0.00)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('label', 'plastic')
        self.declare_parameter('frame_id', 'base_footprint')

        self.target_publisher = self.create_publisher(PoseStamped, '/mission/arm_pick_target', 10)
        self.label_publisher = self.create_publisher(String, '/mission/trash_label', 10)
        self.create_timer(1.0, self._publish_once)
        self.sent = False

    def _publish_once(self) -> None:
        if self.sent:
            return

        label_msg = String()
        label_msg.data = str(self.get_parameter('label').value)

        target_msg = PoseStamped()
        target_msg.header.frame_id = str(self.get_parameter('frame_id').value)
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.pose.position.x = float(self.get_parameter('x').value)
        target_msg.pose.position.y = float(self.get_parameter('y').value)
        target_msg.pose.position.z = float(self.get_parameter('z').value)
        target_msg.pose.orientation.w = 1.0

        self.label_publisher.publish(label_msg)
        self.target_publisher.publish(target_msg)
        self.sent = True
        self.get_logger().info(
            f'已发布调试抓取目标: label={label_msg.data}, '
            f'x={target_msg.pose.position.x:.3f}, '
            f'y={target_msg.pose.position.y:.3f}, '
            f'z={target_msg.pose.position.z:.3f}'
        )


def main() -> None:
    rclpy.init()
    node = DebugGraspGarbage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
