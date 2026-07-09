#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from nav_msgs.msg import OccupancyGrid


class RtabmapMapBridge(Node):
    def __init__(self) -> None:
        super().__init__('rtabmap_map_bridge')

        self.declare_parameter('input_topic', '/rtabmap/grid_map')
        self.declare_parameter('input_topics', ['/grid_map', '/rtabmap/grid_map'])
        self.declare_parameter('output_topic', '/map')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        input_topics = [
            topic for topic in self.get_parameter('input_topics').get_parameter_value().string_array_value
            if topic
        ]
        if input_topic and input_topic not in input_topics:
            input_topics.append(input_topic)
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sub_qos.durability = DurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._publisher = self.create_publisher(OccupancyGrid, output_topic, pub_qos)
        self._subscriptions = []
        for topic in input_topics:
            self._subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, source_topic=topic: self._handle_map(msg, source_topic),
                    sub_qos,
                )
            )
        self._published_once = False

        self.get_logger().info(
            f'Bridging occupancy grid from {input_topics} to {output_topic} '
            'with transient-local QoS for Nav2.'
        )

    def _handle_map(self, msg: OccupancyGrid, source_topic: str) -> None:
        self._publisher.publish(msg)
        if not self._published_once:
            self._published_once = True
            self.get_logger().info(f'First RTAB-Map occupancy grid forwarded to /map from {source_topic}.')


def main() -> None:
    rclpy.init()
    node = RtabmapMapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
