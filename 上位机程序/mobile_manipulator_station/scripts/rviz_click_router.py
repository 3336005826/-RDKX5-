#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String


class RvizClickRouter(Node):
    def __init__(self) -> None:
        super().__init__('rviz_click_router')

        self.mode: str = 'nav'
        self.nav_anchor: Optional[Tuple[float, float]] = None

        self.nav_goal_pub = self.create_publisher(PoseStamped, '/mission/nav_goal_pose', 10)
        self.shuttle_goal_pub = self.create_publisher(PoseStamped, '/mission/shuttle_goal_pose', 10)
        self.home_pose_pub = self.create_publisher(PoseStamped, '/mission/home_pose', 10)
        self.home_point_pub = self.create_publisher(PointStamped, '/mission/home_point', 10)
        self.region_point_pub = self.create_publisher(PointStamped, '/mission/region_point', 10)

        self.create_subscription(String, '/station/click_mode', self._on_mode, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_clicked_point, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._on_goal_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._on_initial_pose, 10)

        self.get_logger().info('RViz click router started.')

    def _on_mode(self, msg: String) -> None:
        value = (msg.data or '').strip()
        if value:
            self.mode = value
            self.nav_anchor = None
            self.get_logger().info(f'RViz click mode -> {self.mode}')

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if self.mode in ('nav', 'shuttle'):
            self._handle_nav_point(msg)
            return

        if self.mode == 'home':
            home_pose = self._point_to_pose(msg, yaw=0.0)
            self.home_pose_pub.publish(home_pose)
            self.home_point_pub.publish(msg)
            self.get_logger().info(
                f'RViz point routed to home pose: x={msg.point.x:.3f}, y={msg.point.y:.3f}'
            )
            return

        if self.mode == 'region':
            self.region_point_pub.publish(msg)
            self.get_logger().info(
                f'RViz point routed to /mission/region_point: x={msg.point.x:.3f}, y={msg.point.y:.3f}'
            )
            return

        self.get_logger().info(
            f'Ignored /clicked_point in mode={self.mode}. Use RViz 2D Goal for navigation.'
        )

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if self.mode == 'home':
            self.home_pose_pub.publish(msg)
            self.get_logger().info(
                f'RViz goal routed to /mission/home_pose: '
                f'x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}'
            )
            return

        if self.mode == 'region':
            point = self._pose_to_point(msg)
            self.region_point_pub.publish(point)
            self.get_logger().info(
                f'RViz goal routed to /mission/region_point: '
                f'x={point.point.x:.3f}, y={point.point.y:.3f}'
            )
            return

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.mode == 'initial_pose':
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        if self.mode == 'home':
            self.home_pose_pub.publish(pose)
            self.get_logger().info(
                f'RViz initial pose routed to /mission/home_pose: '
                f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
            )
            return

        if self.mode == 'region':
            point = self._pose_to_point(pose)
            self.region_point_pub.publish(point)
            self.get_logger().info(
                f'RViz initial pose routed to /mission/region_point: '
                f'x={point.point.x:.3f}, y={point.point.y:.3f}'
            )
            return

    def _handle_nav_point(self, msg: PointStamped) -> None:
        point = (msg.point.x, msg.point.y)
        if self.nav_anchor is None:
            self.nav_anchor = point
            label = 'shuttle' if self.mode == 'shuttle' else 'nav'
            self.get_logger().info(
                f'RViz {label} anchor set: x={point[0]:.3f}, y={point[1]:.3f}. '
                'Publish one more point to set heading.'
            )
            return

        anchor_x, anchor_y = self.nav_anchor
        self.nav_anchor = None
        yaw = math.atan2(point[1] - anchor_y, point[0] - anchor_x)
        if abs(point[0] - anchor_x) < 1e-6 and abs(point[1] - anchor_y) < 1e-6:
            yaw = 0.0
        pose = self._point_to_pose(msg, anchor_x, anchor_y, yaw)
        if self.mode == 'shuttle':
            topic = '/mission/shuttle_goal_pose'
            self.shuttle_goal_pub.publish(pose)
        else:
            topic = '/mission/nav_goal_pose'
            self.nav_goal_pub.publish(pose)
        self.get_logger().info(
            f'RViz point routed to {topic}: '
            f'x={anchor_x:.3f}, y={anchor_y:.3f}, yaw={math.degrees(yaw):.1f} deg'
        )

    def _point_to_pose(
        self,
        msg: PointStamped,
        x: Optional[float] = None,
        y: Optional[float] = None,
        yaw: float = 0.0,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = 'map'
        pose.pose.position.x = msg.point.x if x is None else x
        pose.pose.position.y = msg.point.y if y is None else y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _pose_to_point(self, msg: PoseStamped) -> PointStamped:
        point = PointStamped()
        point.header = msg.header
        if not point.header.frame_id:
            point.header.frame_id = 'map'
        point.point.x = msg.pose.position.x
        point.point.y = msg.pose.position.y
        point.point.z = msg.pose.position.z
        return point

def main() -> None:
    rclpy.init()
    node = RvizClickRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
