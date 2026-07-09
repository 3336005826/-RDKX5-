#!/usr/bin/env python3

from typing import List, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class StationMapRelay(Node):
    def __init__(self) -> None:
        super().__init__('station_map_relay')

        self.declare_parameter('source_topic', '/map')
        self.declare_parameter('input_topics', ['/map', '/grid_map', '/rtabmap/grid_map'])
        self.declare_parameter('relay_topic', '/station/map')
        self.declare_parameter('republish_period', 1.0)

        input_topics = self._input_topics()
        relay_topic = str(self.get_parameter('relay_topic').value)
        period = max(0.2, float(self.get_parameter('republish_period').value))

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        volatile_qos = QoSProfile(depth=1)
        volatile_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        volatile_qos.durability = DurabilityPolicy.VOLATILE

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.VOLATILE

        self.latest_map: Optional[OccupancyGrid] = None
        self.map_subscriptions = []
        self.map_pub = self.create_publisher(OccupancyGrid, relay_topic, pub_qos)
        for topic in input_topics:
            self.map_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, source_topic=topic: self._on_map(msg, source_topic),
                    transient_qos,
                )
            )
            self.map_subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    lambda msg, source_topic=topic: self._on_map(msg, source_topic),
                    volatile_qos,
                )
            )
        self.republish_timer = self.create_timer(period, self._republish)

        self.get_logger().info(
            f'Station map relay started: {input_topics} -> {relay_topic}, period={period:.1f}s'
        )

    def _input_topics(self) -> List[str]:
        value = self.get_parameter('input_topics').value
        topics = [str(topic).strip() for topic in value if str(topic).strip()]
        source_topic = str(self.get_parameter('source_topic').value).strip()
        if source_topic:
            topics.append(source_topic)
        unique_topics = []
        for topic in topics:
            if topic not in unique_topics:
                unique_topics.append(topic)
        return unique_topics or ['/map']

    def _on_map(self, msg: OccupancyGrid, source_topic: str) -> None:
        first_map = self.latest_map is None
        self.latest_map = msg
        self.map_pub.publish(msg)
        if first_map:
            self.get_logger().info(
                f'Relaying map for station UI from {source_topic}: '
                f'{msg.info.width} x {msg.info.height}'
            )

    def _republish(self) -> None:
        if self.latest_map is not None:
            self.map_pub.publish(self.latest_map)


def main() -> None:
    rclpy.init()
    node = StationMapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
