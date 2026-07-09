#!/usr/bin/env python3

import copy
import math
from typing import List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class DirectionalScanFilter(Node):
    """Filter near-body LaserScan returns only in configured angular sectors."""

    def __init__(self) -> None:
        super().__init__('directional_scan_filter')

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sector_min_deg', [30.0, 120.0, 210.0, 300.0])
        self.declare_parameter('sector_max_deg', [60.0, 150.0, 240.0, 330.0])
        self.declare_parameter('front_m', 0.278)
        self.declare_parameter('back_m', 0.278)
        self.declare_parameter('left_m', 0.25)
        self.declare_parameter('right_m', 0.25)
        self.declare_parameter('padding_m', 0.12)
        self.declare_parameter('transform_timeout_sec', 0.03)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.sectors = self._load_sectors()
        self.front_m = float(self.get_parameter('front_m').value)
        self.back_m = float(self.get_parameter('back_m').value)
        self.left_m = float(self.get_parameter('left_m').value)
        self.right_m = float(self.get_parameter('right_m').value)
        self.padding_m = float(self.get_parameter('padding_m').value)
        self.transform_timeout = float(self.get_parameter('transform_timeout_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_transform = None
        self.last_warn_time = 0.0
        self.filtered_total = 0

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(LaserScan, input_topic, self._handle_scan, 10)

        self.get_logger().info(
            f'Filtering {input_topic} -> {output_topic}, sectors={self.sectors}, '
            f'near-body padding={self.padding_m:.3f}m.'
        )

    def _load_sectors(self) -> List[Tuple[float, float]]:
        mins = [float(v) for v in self.get_parameter('sector_min_deg').value]
        maxs = [float(v) for v in self.get_parameter('sector_max_deg').value]
        if len(mins) != len(maxs):
            raise ValueError('sector_min_deg and sector_max_deg must have the same length')
        sectors = []
        for start, end in zip(mins, maxs):
            # Treat a 360-degree span as a full-circle sector. Without this,
            # 360 normalizes to 0 and [0, 360] would only match exactly 0 deg.
            if abs(end - start) >= 360.0:
                sectors.append((0.0, 360.0))
            else:
                sectors.append((self._norm_deg(start), self._norm_deg(end)))
        return sectors

    @staticmethod
    def _norm_deg(value: float) -> float:
        return value % 360.0

    @staticmethod
    def _stamp_to_time(stamp: TimeMsg) -> Time:
        if stamp.sec == 0 and stamp.nanosec == 0:
            return Time()
        return Time.from_msg(stamp)

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_transform(self, scan: LaserScan):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan.header.frame_id,
                self._stamp_to_time(scan.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            self.last_transform = transform
            return transform
        except TransformException as exc:
            if self.last_transform is not None:
                return self.last_transform
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_warn_time > 2.0:
                self.last_warn_time = now
                self.get_logger().warning(f'No transform for scan filtering yet: {exc}')
            return None

    def _angle_in_sector(self, angle_deg: float) -> bool:
        angle = self._norm_deg(angle_deg)
        for start, end in self.sectors:
            if start <= end:
                if start <= angle <= end:
                    return True
            elif angle >= start or angle <= end:
                return True
        return False

    def _near_body(self, x: float, y: float) -> bool:
        return (
            -self.back_m - self.padding_m <= x <= self.front_m + self.padding_m
            and -self.right_m - self.padding_m <= y <= self.left_m + self.padding_m
        )

    def _handle_scan(self, msg: LaserScan) -> None:
        transform = self._lookup_transform(msg)
        if transform is None:
            self.publisher.publish(msg)
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        filtered = 0
        output = copy.deepcopy(msg)
        for index, range_m in enumerate(msg.ranges):
            if not math.isfinite(range_m):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            x_scan = range_m * math.cos(angle)
            y_scan = range_m * math.sin(angle)

            x_base = translation.x + cos_yaw * x_scan - sin_yaw * y_scan
            y_base = translation.y + sin_yaw * x_scan + cos_yaw * y_scan
            angle_base_deg = math.degrees(math.atan2(y_base, x_base))

            if self._angle_in_sector(angle_base_deg) and self._near_body(x_base, y_base):
                output.ranges[index] = float('inf')
                if index < len(output.intensities):
                    output.intensities[index] = 0.0
                filtered += 1

        self.filtered_total += filtered
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = DirectionalScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
