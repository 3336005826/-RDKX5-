#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, Pose, PoseArray, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class CoveragePathPlanner(Node):
    def __init__(self) -> None:
        super().__init__('coverage_path_planner')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('lane_spacing', 0.5)
        self.declare_parameter('margin', 0.1)
        self.declare_parameter('min_region_size', 0.5)

        self.clicked_points: List[Tuple[float, float]] = []

        self.publisher = self.create_publisher(PoseArray, '/mission/coverage_waypoints', 10)
        self.region_polygon_pub = self.create_publisher(PolygonStamped, '/mission/selected_region', 10)
        self.region_marker_pub = self.create_publisher(Marker, '/mission/selected_region_marker', 10)

        self.create_subscription(PointStamped, '/mission/region_point', self._on_region_point, 10)

        self.get_logger().info('Coverage path planner started. Use two points to define a region.')

    def _on_region_point(self, msg: PointStamped) -> None:
        self._store_region_point(msg)

    def _store_region_point(self, msg: PointStamped) -> None:
        self.clicked_points.append((msg.point.x, msg.point.y))
        self.get_logger().info(
            f'Region point received: x={msg.point.x:.3f}, y={msg.point.y:.3f}, '
            f'count={len(self.clicked_points)}'
        )

        if len(self.clicked_points) < 2:
            return

        first = self.clicked_points[-2]
        second = self.clicked_points[-1]
        self.clicked_points = []
        self._publish_region(first, second)

    def _publish_region(self, first: Tuple[float, float], second: Tuple[float, float]) -> None:
        min_x = min(first[0], second[0])
        max_x = max(first[0], second[0])
        min_y = min(first[1], second[1])
        max_y = max(first[1], second[1])

        min_size = float(self.get_parameter('min_region_size').value)
        if (max_x - min_x) < min_size or (max_y - min_y) < min_size:
            self.get_logger().warning('Selected region is too small.')
            return

        self._publish_region_polygon(min_x, max_x, min_y, max_y)
        pose_array = self._build_coverage_path(min_x, max_x, min_y, max_y)
        self.publisher.publish(pose_array)
        self.get_logger().info(f'Published {len(pose_array.poses)} coverage waypoints.')

    def _publish_region_polygon(self, min_x: float, max_x: float, min_y: float, max_y: float) -> None:
        frame_id = str(self.get_parameter('frame_id').value)
        stamp = self.get_clock().now().to_msg()

        polygon = PolygonStamped()
        polygon.header.frame_id = frame_id
        polygon.header.stamp = stamp

        corners = [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ]

        for x, y in corners:
            point = Point32()
            point.x = x
            point.y = y
            point.z = 0.0
            polygon.polygon.points.append(point)

        self.region_polygon_pub.publish(polygon)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'selected_region'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1

        for x, y in corners + [corners[0]]:
            marker_point = PointStamped().point
            marker_point.x = x
            marker_point.y = y
            marker_point.z = 0.05
            marker.points.append(marker_point)

        self.region_marker_pub.publish(marker)

    def _build_coverage_path(self, min_x: float, max_x: float, min_y: float, max_y: float) -> PoseArray:
        lane_spacing = float(self.get_parameter('lane_spacing').value)
        margin = float(self.get_parameter('margin').value)
        frame_id = str(self.get_parameter('frame_id').value)

        inner_min_x = min_x + margin
        inner_max_x = max_x - margin
        inner_min_y = min_y + margin
        inner_max_y = max_y - margin

        if inner_min_x >= inner_max_x or inner_min_y >= inner_max_y:
            inner_min_x = min_x
            inner_max_x = max_x
            inner_min_y = min_y
            inner_max_y = max_y

        width = inner_max_x - inner_min_x
        lanes = max(2, int(width / lane_spacing) + 1)
        actual_spacing = width / max(1, lanes - 1)

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = frame_id

        for index in range(lanes):
            x = inner_min_x + index * actual_spacing
            if index % 2 == 0:
                points = [
                    (x, inner_min_y, math.pi / 2.0),
                    (x, inner_max_y, math.pi / 2.0),
                ]
            else:
                points = [
                    (x, inner_max_y, -math.pi / 2.0),
                    (x, inner_min_y, -math.pi / 2.0),
                ]

            for point_x, point_y, yaw in points:
                pose = Pose()
                pose.position.x = point_x
                pose.position.y = point_y
                pose.position.z = 0.0
                pose.orientation = self._yaw_to_quaternion(yaw)
                pose_array.poses.append(pose)

        return pose_array

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat


def main() -> None:
    rclpy.init()
    node = CoveragePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
