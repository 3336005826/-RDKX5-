#!/usr/bin/env python3

import base64
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Dict, List, Optional, Tuple
import shutil

try:
    import yaml
except ImportError:
    yaml = None

try:
    import websocket
except ImportError:
    websocket = None

try:
    import rclpy
    from geometry_msgs.msg import PointStamped as RosPointStamped
    from geometry_msgs.msg import PoseStamped as RosPoseStamped
    from geometry_msgs.msg import PoseWithCovarianceStamped as RosPoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
    from rclpy.node import Node as RosNode
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool as RosBool
    from std_msgs.msg import String as RosString
except ImportError:
    rclpy = None
    RosBool = None
    RosNode = None
    RosString = None
    RosPointStamped = None
    RosPoseStamped = None
    RosPoseWithCovarianceStamped = None
    RosOccupancyGrid = None
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None

from PyQt5 import QtCore, QtGui, QtWidgets
from viewer.robot_3d_view import Robot3DWidget


@dataclass
class StationConfig:
    rosbridge_url: str = "ws://192.168.1.142:9090"
    rdk_host: str = "sunrise@192.168.1.142"
    rdk_workspace: str = "/home/sunrise/test_ws"
    jetson_host: str = "jetson@192.168.1.205"
    jetson_workspace: str = "/home/jetson/jetcobot_ws"
    auto_start_arm_remote: bool = False
    map_yaml: str = "/home/sunrise/test_ws/saved_maps/map.yaml"
    map_save_stem: str = "/home/sunrise/test_ws/saved_maps/map"
    robot_urdf_path: str = ""
    mapping_command: str = (
        "source /opt/ros/humble/setup.bash && "
        "source /home/sunrise/test_ws/install/setup.bash && "
        "ros2 launch robot_rtab wheeltec_nav2_rtab.launch.py localization:=false"
    )
    nav_command: str = (
        "source /opt/ros/humble/setup.bash && "
        "source /home/sunrise/test_ws/install/setup.bash && "
        "ros2 launch car_mission_system car_bringup.launch.py localization:=true "
        "map:=/home/sunrise/test_ws/saved_maps/map.yaml"
    )
    shuttle_command: str = (
        "source /opt/ros/humble/setup.bash && "
        "source /home/sunrise/test_ws/install/setup.bash && "
        "ros2 launch car_mission_system car_bringup.launch.py localization:=true "
        "map:=/home/sunrise/test_ws/saved_maps/map.yaml"
    )
    arm_command: str = (
        "export ROS_DOMAIN_ID=30 && export ROS_LOCALHOST_ONLY=0 && "
        "source /opt/ros/humble/setup.bash && "
        "source /home/jetson/jetcobot_ws/install/setup.bash && "
        "ros2 launch arm_mission_system arm_bringup.launch.py "
        "model_path:=/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.engine "
        "camera_device:=/dev/video1 camera_reopen_sec:=2.0 "
        "publish_debug_view:=true publish_debug_topic:=true confidence_threshold:=0.8 "
        "auto_pick_from_detector:=true min_pick_interval_sec:=60.0 "
        "use_cpp_executor:=true enable_base_assist:=true "
        "wait_for_watch_pose_ready:=true "
        "joint1:=-90 joint2:=20 joint3:=-20 joint4:=-60 joint5:=0 joint6:=-45 "
        "watch_joint1:=-90 watch_joint2:=-45 watch_joint3:=10 watch_joint4:=-45 watch_joint5:=0 watch_joint6:=-45 "
        "grasp_target_swap_xy:=true grasp_target_x_sign:=1.0 grasp_target_y_sign:=1.0 "
        "grasp_target_x_offset:=0.05 grasp_target_y_offset:=0.0 "
        "use_bridge_grasp_pose:=true grasp_rx:=-150.0 grasp_ry:=10.0 grasp_rz:=-45.0 "
        "plastic_approach_z:=175 plastic_grasp_z:=140 "
        "use_custom_drop_joints:=true return_after_drop:=false "
        "drop_joint1:=110 drop_joint2:=-20 drop_joint3:=10 drop_joint4:=-20 drop_joint5:=0 drop_joint6:=-45"
    )


@dataclass
class MapMeta:
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    width: int = 0
    height: int = 0


def resource_base_dir() -> Path:
    if getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"):
        return Path(sys._MEIPASS)

    script_dir = Path(__file__).resolve().parent
    candidates = [
        script_dir.parent,
        Path.cwd() / "src" / "mobile_manipulator_station",
        Path.cwd() / "mobile_manipulator_station",
    ]
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share" / "mobile_manipulator_station")

    for candidate in candidates:
        if (candidate / "config").exists():
            return candidate
    return script_dir.parent


def runtime_base_dir() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys.executable).resolve().parent
    return resource_base_dir()


def default_map_save_stem(map_yaml: str) -> str:
    path = map_yaml.strip()
    if path.endswith(".yaml"):
        return path[:-5]
    return path


def default_robot_urdf_path() -> str:
    linux_candidate = Path(
        "/home/yang/test_ws/src/wheeltec_robot_urdf/"
        "wheeltec_robot_urdf/urdf/R550A_PLUS_4wd_arm_robot.urdf"
    )
    if linux_candidate.exists():
        return str(linux_candidate)

    workspace_root = Path(__file__).resolve().parents[4]
    candidate = (
        workspace_root
        / "src"
        / "wheeltec_robot_urdf"
        / "wheeltec_robot_urdf"
        / "urdf"
        / "R550A_PLUS_4wd_arm_robot.urdf"
    )
    return str(candidate) if candidate.exists() else ""


def is_windows() -> bool:
    return os.name == "nt"


def is_linux() -> bool:
    return sys.platform.startswith("linux")


def build_ssh_args(host: str, command: str, batch_mode: bool = False, tty: bool = False) -> List[str]:
    remote_command = command.replace("'", "'\"'\"'")
    args = [
        "ssh",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-o",
        "ConnectTimeout=8",
    ]
    if batch_mode:
        args.extend(["-o", "BatchMode=yes"])
    if tty:
        args.append("-tt")
    args.append(host)
    args.append(f"bash -lc '{remote_command}'")
    return args


def ssh_shell_command(host: str, command: str, batch_mode: bool = False, tty: bool = False) -> str:
    return " ".join(shlex.quote(arg) for arg in build_ssh_args(host, command, batch_mode=batch_mode, tty=tty))


class RosBridgeClient(QtCore.QObject):
    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal(str)
    state_received = QtCore.pyqtSignal(str)
    map_received = QtCore.pyqtSignal(object)
    odom_received = QtCore.pyqtSignal(float, float, float)
    home_pose_received = QtCore.pyqtSignal(float, float)
    nav_goal_received = QtCore.pyqtSignal(float, float, float)
    rviz_goal_received = QtCore.pyqtSignal(float, float, float)
    rviz_clicked_point_received = QtCore.pyqtSignal(float, float)
    rviz_initial_pose_received = QtCore.pyqtSignal(float, float, float)
    trash_pose_received = QtCore.pyqtSignal(float, float, str)
    trash_label_received = QtCore.pyqtSignal(str)
    region_received = QtCore.pyqtSignal(list)
    coverage_received = QtCore.pyqtSignal(list)
    arm_busy_received = QtCore.pyqtSignal(bool)
    grasp_result_received = QtCore.pyqtSignal(str)
    debug_image_received = QtCore.pyqtSignal(object)
    log_received = QtCore.pyqtSignal(str)

    def __init__(self, url: str, parent=None) -> None:
        super().__init__(parent)
        self.url = url
        self.ws = None
        self.thread = None
        self.should_run = False
        self.connected_once = False
        self.connect_timeout_sec = 5.0
        self.recv_poll_timeout_sec = 0.5
        self.last_map_received_at = 0.0
        self.last_map_request_at = 0.0
        self.map_request_interval_sec = 3.0
        self.fragments: Dict[str, Dict] = {}
        self.map_topics = (
            "/station/map",
            "/map",
            "/grid_map",
            "/rtabmap/grid_map",
        )
        self.topic_types = {
            "/mission/nav_goal_pose": "geometry_msgs/PoseStamped",
            "/mission/shuttle_goal_pose": "geometry_msgs/PoseStamped",
            "/mission/home_pose": "geometry_msgs/PoseStamped",
            "/mission/region_point": "geometry_msgs/PointStamped",
            "/mission/return_home": "std_msgs/Bool",
            "/mission/clear_region": "std_msgs/Bool",
            "/station/click_mode": "std_msgs/String",
            "/initialpose": "geometry_msgs/PoseWithCovarianceStamped",
        }

    def start(self) -> None:
        if websocket is None:
            message = "缺少 websocket-client，无法连接 rosbridge。"
            self.log_received.emit(message)
            self.disconnected.emit(message)
            return
        if self.thread and self.thread.is_alive():
            return
        self.should_run = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.should_run = False
        try:
            if self.ws is not None:
                self.ws.close()
        except Exception:
            pass

    def publish_pose(self, topic: str, x: float, y: float, yaw: float = 0.0) -> None:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        self._send_json(
            {
                "op": "publish",
                "topic": topic,
                "msg": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "position": {"x": x, "y": y, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                    },
                },
            }
        )

    def publish_region_point(self, x: float, y: float) -> None:
        self._send_json(
            {
                "op": "publish",
                "topic": "/mission/region_point",
                "msg": {
                    "header": {"frame_id": "map"},
                    "point": {"x": x, "y": y, "z": 0.0},
                },
            }
        )

    def publish_bool(self, topic: str, value: bool) -> None:
        self._send_json({"op": "publish", "topic": topic, "msg": {"data": value}})

    def publish_string(self, topic: str, value: str) -> None:
        self._send_json({"op": "publish", "topic": topic, "msg": {"data": value}})

    def request_map_once(self) -> None:
        self.last_map_request_at = time.time()
        for service in ("/map_server/map", "/static_map"):
            self._send_json(
                {
                    "op": "call_service",
                    "service": service,
                    "type": "nav_msgs/srv/GetMap",
                    "args": {},
                    "id": f"request_map:{service}",
                }
            )

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.06853891945200942
        self._send_json(
            {
                "op": "publish",
                "topic": "/initialpose",
                "msg": {
                    "header": {"frame_id": "map"},
                    "pose": {
                        "pose": {
                            "position": {"x": x, "y": y, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                        },
                        "covariance": covariance,
                    },
                },
            }
        )

    def _run(self) -> None:
        while self.should_run:
            try:
                self.ws = websocket.create_connection(
                    self.url,
                    timeout=self.connect_timeout_sec,
                    enable_multithread=True,
                )
                self.ws.settimeout(self.recv_poll_timeout_sec)
                self._advertise_publishers()
                self._subscribe_topics()
                self.request_map_once()
                self.connected.emit()
                self.connected_once = True
                self.log_received.emit(f"已连接 rosbridge: {self.url}")

                while self.should_run:
                    try:
                        raw = self.ws.recv()
                    except websocket.WebSocketTimeoutException:
                        self._request_map_if_needed()
                        continue
                    if not raw:
                        break
                    try:
                        self._handle_message(raw)
                    except Exception as exc:
                        self.log_received.emit(f"rosbridge message ignored: {exc}")
            except Exception as exc:
                if self.should_run:
                    reason = str(exc)
                    self.disconnected.emit(reason)
                    self.log_received.emit(f"rosbridge 连接中断: {reason}")
                    time.sleep(2.0)
            finally:
                try:
                    if self.ws is not None:
                        self.ws.close()
                except Exception:
                    pass
                self.ws = None

    def _advertise_publishers(self) -> None:
        for topic, topic_type in self.topic_types.items():
            self._send_json({"op": "advertise", "topic": topic, "type": topic_type})

    def _subscribe_topics(self) -> None:
        topics = [
            ("/mission/state", "std_msgs/String"),
            ("/mission/home_pose", "geometry_msgs/PoseStamped"),
            ("/mission/nav_goal_pose", "geometry_msgs/PoseStamped"),
            ("/mission/shuttle_goal_pose", "geometry_msgs/PoseStamped"),
            ("/mission/trash_pose", "geometry_msgs/PoseStamped"),
            ("/mission/trash_label", "std_msgs/String"),
            ("/mission/selected_region", "geometry_msgs/PolygonStamped"),
            ("/mission/coverage_waypoints", "geometry_msgs/PoseArray"),
            ("/mission/arm_busy", "std_msgs/Bool"),
            ("/arm/grasp_result", "std_msgs/String"),
            *[(topic, "nav_msgs/OccupancyGrid") for topic in self.map_topics],
            ("/odom", "nav_msgs/Odometry"),
            ("/mission/debug_image/compressed", "sensor_msgs/CompressedImage"),
            ("/goal_pose", "geometry_msgs/PoseStamped"),
            ("/clicked_point", "geometry_msgs/PointStamped"),
            ("/initialpose", "geometry_msgs/PoseWithCovarianceStamped"),
        ]
        for topic, topic_type in topics:
            self._send_json(
                {
                    "op": "subscribe",
                    "topic": topic,
                    "type": topic_type,
                    "throttle_rate": 200,
                }
            )

    def _send_json(self, payload: Dict) -> None:
        if self.ws is None:
            return
        self.ws.send(json.dumps(payload))

    def _handle_message(self, raw: str) -> None:
        try:
            message = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.log_received.emit(f"rosbridge message parse failed: {exc}")
            return

        if not isinstance(message, dict):
            return

        if message.get("op") == "fragment":
            self._handle_fragment(message)
            return

        if message.get("op") == "service_response":
            values = message.get("values", {}) or {}
            if not isinstance(values, dict):
                return
            map_msg = values.get("map")
            if isinstance(map_msg, dict):
                self.last_map_received_at = time.time()
                self.map_received.emit(map_msg)
            return

        if message.get("op") != "publish":
            return

        topic = message.get("topic", "")
        msg = message.get("msg", {})
        if not isinstance(msg, dict):
            return

        if topic == "/mission/state":
            self.state_received.emit(msg.get("data", "idle"))
            return

        if topic == "/mission/home_pose":
            pos = msg.get("pose", {}).get("position", {})
            self.home_pose_received.emit(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)))
            return

        if topic in ("/mission/nav_goal_pose", "/mission/shuttle_goal_pose"):
            pos = msg.get("pose", {}).get("position", {})
            ori = msg.get("pose", {}).get("orientation", {})
            yaw = self._quaternion_to_yaw(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
            self.nav_goal_received.emit(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
            return

        if topic == "/goal_pose":
            pos = msg.get("pose", {}).get("position", {})
            ori = msg.get("pose", {}).get("orientation", {})
            yaw = self._quaternion_to_yaw(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
            self.rviz_goal_received.emit(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
            return

        if topic == "/clicked_point":
            point = msg.get("point", {})
            self.rviz_clicked_point_received.emit(float(point.get("x", 0.0)), float(point.get("y", 0.0)))
            return

        if topic == "/initialpose":
            pose = msg.get("pose", {}).get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            yaw = self._quaternion_to_yaw(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
            self.rviz_initial_pose_received.emit(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
            return

        if topic == "/mission/trash_pose":
            pos = msg.get("pose", {}).get("position", {})
            self.trash_pose_received.emit(
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                "",
            )
            return

        if topic == "/mission/trash_label":
            self.trash_label_received.emit(msg.get("data", "unknown"))
            return

        if topic == "/mission/selected_region":
            points = []
            for point in msg.get("polygon", {}).get("points", []):
                points.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))
            self.region_received.emit(points)
            return

        if topic == "/mission/coverage_waypoints":
            points = []
            for pose in msg.get("poses", []):
                pos = pose.get("position", {})
                points.append((float(pos.get("x", 0.0)), float(pos.get("y", 0.0))))
            self.coverage_received.emit(points)
            return

        if topic == "/mission/arm_busy":
            self.arm_busy_received.emit(bool(msg.get("data", False)))
            return

        if topic == "/arm/grasp_result":
            self.grasp_result_received.emit(str(msg.get("data", "")).strip())
            return

        if topic in self.map_topics:
            self.last_map_received_at = time.time()
            self.map_received.emit(msg)
            return

        if topic == "/odom":
            pose = msg.get("pose", {}).get("pose", {})
            pos = pose.get("position", {})
            ori = pose.get("orientation", {})
            yaw = self._quaternion_to_yaw(
                float(ori.get("x", 0.0)),
                float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)),
                float(ori.get("w", 1.0)),
            )
            self.odom_received.emit(float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw)
            return

        if topic == "/mission/debug_image/compressed":
            data = msg.get("data", "")
            if not data:
                return
            try:
                image_bytes = base64.b64decode(data)
                image = QtGui.QImage.fromData(image_bytes)
                self.debug_image_received.emit(image)
            except Exception as exc:
                self.log_received.emit(f"调试图像解码失败: {exc}")

    @staticmethod
    def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _request_map_if_needed(self) -> None:
        now = time.time()
        if self.last_map_received_at and now - self.last_map_received_at < self.map_request_interval_sec:
            return
        if now - self.last_map_request_at < self.map_request_interval_sec:
            return
        self.request_map_once()

    def _handle_fragment(self, message: Dict) -> None:
        fragment_id = str(message.get("id", ""))
        data = message.get("data", "")
        if not fragment_id or not isinstance(data, str):
            return
        try:
            num = int(message.get("num", 0))
            total = int(message.get("total", 0))
        except (TypeError, ValueError):
            return
        if total <= 0:
            return

        fragment = self.fragments.setdefault(fragment_id, {"total": total, "parts": {}})
        fragment["total"] = total
        fragment["parts"][num] = data
        parts = fragment["parts"]
        if len(parts) < total:
            return

        if all(index in parts for index in range(total)):
            ordered = [parts[index] for index in range(total)]
        elif all(index in parts for index in range(1, total + 1)):
            ordered = [parts[index] for index in range(1, total + 1)]
        else:
            ordered = [parts[index] for index in sorted(parts)]

        self.fragments.pop(fragment_id, None)
        self._handle_message("".join(ordered))


class LocalRosClient(QtCore.QObject):
    map_received = QtCore.pyqtSignal(object)
    log_received = QtCore.pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.available = rclpy is not None
        self.node = None
        self.thread = None
        self.should_run = False
        self.subscriptions = []
        self.map_topics = ("/station/map", "/map", "/grid_map", "/rtabmap/grid_map")
        self.publishers: Dict[str, object] = {}
        self.logged_map_topics = set()

    def start(self) -> None:
        if not self.available or self.thread and self.thread.is_alive():
            return
        self.should_run = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.should_run = False
        if self.available and rclpy is not None:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            except Exception:
                pass

    def publish_pose(self, topic: str, x: float, y: float, yaw: float = 0.0) -> None:
        if self.node is None or RosPoseStamped is None:
            return
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.create_publisher(RosPoseStamped, topic, 10)
            self.publishers[topic] = publisher
        msg = RosPoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        publisher.publish(msg)

    def publish_region_point(self, x: float, y: float) -> None:
        if self.node is None or RosPointStamped is None:
            return
        topic = "/mission/region_point"
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.create_publisher(RosPointStamped, topic, 10)
            self.publishers[topic] = publisher
        msg = RosPointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        publisher.publish(msg)

    def publish_bool(self, topic: str, value: bool) -> None:
        if self.node is None or RosBool is None:
            return
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.create_publisher(RosBool, topic, 10)
            self.publishers[topic] = publisher
        msg = RosBool()
        msg.data = bool(value)
        publisher.publish(msg)

    def publish_string(self, topic: str, value: str) -> None:
        if self.node is None or RosString is None:
            return
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.create_publisher(RosString, topic, 10)
            self.publishers[topic] = publisher
        msg = RosString()
        msg.data = str(value)
        publisher.publish(msg)

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        if self.node is None or RosPoseWithCovarianceStamped is None:
            return
        topic = "/initialpose"
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.create_publisher(RosPoseWithCovarianceStamped, topic, 10)
            self.publishers[topic] = publisher
        msg = RosPoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942
        publisher.publish(msg)

    def _run(self) -> None:
        if rclpy is None or RosNode is None or RosOccupancyGrid is None:
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = RosNode("station_gui_local_bridge")
            self._create_map_subscriptions()
            self.log_received.emit("本地 ROS2 地图桥已启动。")
            while self.should_run and rclpy.ok() and self.node is not None:
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as exc:
            self.log_received.emit(f"本地 ROS2 地图桥启动失败: {exc}")
        finally:
            try:
                if self.node is not None:
                    self.node.destroy_node()
            except Exception:
                pass
            self.node = None

    def _create_map_subscriptions(self) -> None:
        if QoSProfile is None:
            return
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        volatile_qos = QoSProfile(depth=1)
        volatile_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        volatile_qos.durability = DurabilityPolicy.VOLATILE

        for topic in self.map_topics:
            self.subscriptions.append(
                self.node.create_subscription(
                    RosOccupancyGrid,
                    topic,
                    lambda msg, source_topic=topic: self._on_map(msg, source_topic),
                    transient_qos,
                )
            )
            self.subscriptions.append(
                self.node.create_subscription(
                    RosOccupancyGrid,
                    topic,
                    lambda msg, source_topic=topic: self._on_map(msg, source_topic),
                    volatile_qos,
                )
            )

    def _on_map(self, msg, source_topic: str) -> None:
        self.map_received.emit(self._map_to_dict(msg))
        if source_topic not in self.logged_map_topics:
            self.logged_map_topics.add(source_topic)
            self.log_received.emit(
                f"本地 ROS2 收到地图 {source_topic}: {msg.info.width} x {msg.info.height}"
            )

    def _map_to_dict(self, msg) -> Dict:
        return {
            "header": {
                "frame_id": msg.header.frame_id,
            },
            "info": {
                "map_load_time": {
                    "sec": msg.info.map_load_time.sec,
                    "nanosec": msg.info.map_load_time.nanosec,
                },
                "resolution": msg.info.resolution,
                "width": msg.info.width,
                "height": msg.info.height,
                "origin": {
                    "position": {
                        "x": msg.info.origin.position.x,
                        "y": msg.info.origin.position.y,
                        "z": msg.info.origin.position.z,
                    },
                    "orientation": {
                        "x": msg.info.origin.orientation.x,
                        "y": msg.info.origin.orientation.y,
                        "z": msg.info.origin.orientation.z,
                        "w": msg.info.origin.orientation.w,
                    },
                },
            },
            "data": list(msg.data),
        }


class MapCanvas(QtWidgets.QWidget):
    mapClicked = QtCore.pyqtSignal(float, float)
    mapHovered = QtCore.pyqtSignal(float, float)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(920, 700)
        self.setMouseTracking(True)
        self.mode = "nav"
        self.map_meta = MapMeta()
        self.map_image = QtGui.QImage()
        self.show_grid = True
        self.region_polygon: List[Tuple[float, float]] = []
        self.coverage_path: List[Tuple[float, float]] = []
        self.trash_points: List[Tuple[float, float, str]] = []
        self.home_point: Optional[Tuple[float, float]] = None
        self.nav_goal_point: Optional[Tuple[float, float]] = None
        self.nav_goal_yaw: float = 0.0
        self.nav_goal_anchor: Optional[Tuple[float, float]] = None
        self.nav_goal_preview: Optional[Tuple[float, float]] = None
        self.robot_point: Optional[Tuple[float, float]] = None
        self.robot_yaw: float = 0.0
        self.initial_pose_anchor: Optional[Tuple[float, float]] = None
        self.initial_pose_preview: Optional[Tuple[float, float]] = None
        self.last_hover_point: Optional[Tuple[float, float]] = None
        self.zoom_factor = 1.0
        self.min_zoom = 0.4
        self.max_zoom = 8.0
        self.pan_offset = QtCore.QPointF(0.0, 0.0)
        self.view_rotation_deg = 0.0
        self.drag_mode: Optional[str] = None
        self.drag_last_pos = QtCore.QPoint()

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        self.update()

    def update_map(self, image: QtGui.QImage, meta: MapMeta) -> None:
        first_map = self.map_image.isNull()
        self.map_image = image
        self.map_meta = meta
        if first_map:
            self.reset_view()
        self.update()

    def update_region(self, polygon: List[Tuple[float, float]]) -> None:
        self.region_polygon = polygon
        self.update()

    def update_coverage(self, path: List[Tuple[float, float]]) -> None:
        self.coverage_path = path
        self.update()

    def add_trash_point(self, x: float, y: float, label: str) -> None:
        self.trash_points.append((x, y, label))
        if len(self.trash_points) > 120:
            self.trash_points = self.trash_points[-120:]
        self.update()

    def clear_trash(self) -> None:
        self.trash_points = []
        self.update()

    def set_home(self, x: float, y: float) -> None:
        self.home_point = (x, y)
        self.update()

    def set_goal(self, x: float, y: float, yaw: Optional[float] = None) -> None:
        self.nav_goal_point = (x, y)
        if yaw is not None:
            self.nav_goal_yaw = yaw
        self.nav_goal_anchor = None
        self.nav_goal_preview = None
        self.update()

    def set_nav_goal_anchor(self, point: Optional[Tuple[float, float]]) -> None:
        self.nav_goal_anchor = point
        if point is None:
            self.nav_goal_preview = None
        self.update()

    def set_nav_goal_preview(self, point: Optional[Tuple[float, float]]) -> None:
        self.nav_goal_preview = point
        self.update()

    def set_robot(self, x: float, y: float, yaw: float) -> None:
        self.robot_point = (x, y)
        self.robot_yaw = yaw
        self.update()

    def clear_region(self) -> None:
        self.region_polygon = []
        self.coverage_path = []
        self.trash_points = []
        self.update()

    def set_initial_pose_anchor(self, point: Optional[Tuple[float, float]]) -> None:
        self.initial_pose_anchor = point
        if point is None:
            self.initial_pose_preview = None
        self.update()

    def set_initial_pose_preview(self, point: Optional[Tuple[float, float]]) -> None:
        self.initial_pose_preview = point
        self.update()

    def zoom_in(self) -> None:
        self._apply_zoom(1.18, QtCore.QPointF(self.rect().center()))

    def zoom_out(self) -> None:
        self._apply_zoom(1.0 / 1.18, QtCore.QPointF(self.rect().center()))

    def rotate_view(self, delta_deg: float) -> None:
        self.view_rotation_deg = (self.view_rotation_deg + delta_deg) % 360.0
        self.update()

    def flip_view(self) -> None:
        self.rotate_view(180.0)

    def reset_view(self) -> None:
        self.zoom_factor = 1.0
        self.pan_offset = QtCore.QPointF(0.0, 0.0)
        self.view_rotation_deg = 0.0
        self.update()

    def toggle_grid(self) -> None:
        self.show_grid = not self.show_grid
        self.update()

    def mousePressEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.RightButton:
            self.drag_mode = "rotate" if event.modifiers() & QtCore.Qt.ControlModifier else "pan"
            self.drag_last_pos = event.pos()
            event.accept()
            return

        if event.button() == QtCore.Qt.LeftButton:
            point = self._screen_to_map(event.pos())
            if point is None:
                return
            self.mapClicked.emit(point[0], point[1])

    def mouseMoveEvent(self, event: QtGui.QMouseEvent) -> None:
        point = self._screen_to_map(event.pos())
        self.last_hover_point = point
        if point is not None:
            self.mapHovered.emit(point[0], point[1])

        if self.drag_mode == "pan":
            delta = event.pos() - self.drag_last_pos
            if delta.manhattanLength() > 0:
                self.pan_offset += QtCore.QPointF(float(delta.x()), float(delta.y()))
                self.drag_last_pos = event.pos()
                self.update()
            return

        if self.drag_mode == "rotate":
            delta = event.pos() - self.drag_last_pos
            if delta.manhattanLength() > 0:
                self.view_rotation_deg = (self.view_rotation_deg + delta.x() * 0.35) % 360.0
                self.drag_last_pos = event.pos()
                self.update()
            return

        if self.mode == "initial_pose" and self.initial_pose_anchor is not None:
            self.initial_pose_preview = point
        elif self.mode in ("nav", "shuttle") and self.nav_goal_anchor is not None:
            self.nav_goal_preview = point
        self.update()

    def mouseReleaseEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.RightButton:
            self.drag_mode = None
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: QtGui.QMouseEvent) -> None:
        if event.button() == QtCore.Qt.RightButton:
            self.flip_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def wheelEvent(self, event: QtGui.QWheelEvent) -> None:
        angle = event.angleDelta().y()
        if angle == 0:
            event.ignore()
            return
        factor = 1.14 if angle > 0 else (1.0 / 1.14)
        self._apply_zoom(factor, QtCore.QPointF(event.pos()))
        event.accept()

    def leaveEvent(self, event: QtCore.QEvent) -> None:
        self.last_hover_point = None
        self.update()
        super().leaveEvent(event)

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)
        painter.fillRect(self.rect(), QtGui.QColor("#07111d"))

        viewport = self._viewport_rect()
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 23, 42, 210))
        painter.drawRoundedRect(viewport.adjusted(0, 0, 0, 0), 10, 10)

        if self.map_image.isNull() or self.map_meta.width <= 0 or self.map_meta.height <= 0:
            painter.setPen(QtGui.QColor("#94a3b8"))
            painter.setFont(QtGui.QFont("Microsoft YaHei", 20, QtGui.QFont.Bold))
            painter.drawText(viewport, QtCore.Qt.AlignCenter, "等待 /map 地图数据")
            self._draw_view_overlay(painter, viewport)
            return

        self._draw_map_shadow(painter)

        painter.save()
        painter.setClipRect(viewport)
        painter.setWorldTransform(self._view_transform())
        painter.drawImage(
            QtCore.QRectF(0.0, 0.0, float(self.map_meta.width), float(self.map_meta.height)),
            self.map_image,
        )
        if self.show_grid:
            self._draw_grid(painter)
        painter.restore()

        self._draw_region(painter)
        self._draw_coverage(painter)
        self._draw_markers(painter)
        self._draw_nav_goal_preview(painter)
        self._draw_initial_pose_preview(painter)
        self._draw_status_overlay(painter, viewport)
        self._draw_view_overlay(painter, viewport)

    def _draw_map_shadow(self, painter: QtGui.QPainter) -> None:
        polygon = self._map_screen_polygon()
        if polygon.isEmpty():
            return
        shadow = QtGui.QPolygonF(
            [
                QtCore.QPointF(point.x() + 8.0, point.y() + 10.0)
                for point in polygon
            ]
        )
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(2, 6, 23, 90))
        painter.drawPolygon(shadow)

    def _draw_grid(self, painter: QtGui.QPainter) -> None:
        pixels_per_meter = (self._base_scale() * self.zoom_factor) / max(self.map_meta.resolution, 1e-6)
        step_m = 0.25
        for candidate in [0.25, 0.5, 1.0, 2.0, 5.0, 10.0]:
            step_m = candidate
            if candidate * pixels_per_meter >= 58.0:
                break

        step_px = step_m / max(self.map_meta.resolution, 1e-6)
        x_min = self.map_meta.origin_x
        x_max = self.map_meta.origin_x + self.map_meta.width * self.map_meta.resolution
        y_min = self.map_meta.origin_y
        y_max = self.map_meta.origin_y + self.map_meta.height * self.map_meta.resolution

        grid_pen = QtGui.QPen(QtGui.QColor(113, 128, 150, 62), 0)
        painter.setPen(grid_pen)

        x_value = math.floor(x_min / step_m) * step_m
        while x_value <= x_max:
            px = (x_value - self.map_meta.origin_x) / self.map_meta.resolution
            painter.drawLine(
                QtCore.QPointF(px, 0.0),
                QtCore.QPointF(px, float(self.map_meta.height)),
            )
            x_value += step_m

        y_value = math.floor(y_min / step_m) * step_m
        while y_value <= y_max:
            py = self.map_meta.height - ((y_value - self.map_meta.origin_y) / self.map_meta.resolution)
            painter.drawLine(
                QtCore.QPointF(0.0, py),
                QtCore.QPointF(float(self.map_meta.width), py),
            )
            y_value += step_m

    def _draw_region(self, painter: QtGui.QPainter) -> None:
        if len(self.region_polygon) < 2:
            return
        polygon = QtGui.QPolygonF()
        for x, y in self.region_polygon:
            canvas = self._map_to_screen(x, y)
            if canvas is not None:
                polygon.append(QtCore.QPointF(canvas[0], canvas[1]))
        if polygon.isEmpty():
            return
        painter.setBrush(QtGui.QColor(37, 99, 235, 42))
        painter.setPen(QtGui.QPen(QtGui.QColor("#38bdf8"), 3))
        painter.drawPolygon(polygon)
        painter.setBrush(QtGui.QColor("#38bdf8"))
        for idx in range(polygon.count()):
            painter.drawEllipse(polygon.at(idx), 4, 4)

    def _draw_coverage(self, painter: QtGui.QPainter) -> None:
        if len(self.coverage_path) < 2:
            return
        glow_pen = QtGui.QPen(QtGui.QColor(14, 165, 233, 70), 8)
        glow_pen.setCapStyle(QtCore.Qt.RoundCap)
        path_pen = QtGui.QPen(QtGui.QColor("#22d3ee"), 2.5)
        path_pen.setCapStyle(QtCore.Qt.RoundCap)
        path_pen.setJoinStyle(QtCore.Qt.RoundJoin)
        segments: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
        last = None
        for x, y in self.coverage_path:
            point = self._map_to_screen(x, y)
            if point is None:
                continue
            if last is not None:
                segments.append((last, point))
            last = point
        painter.setPen(glow_pen)
        for start, end in segments:
            painter.drawLine(QtCore.QPointF(*start), QtCore.QPointF(*end))
        painter.setPen(path_pen)
        for start, end in segments:
            painter.drawLine(QtCore.QPointF(*start), QtCore.QPointF(*end))

    def _draw_markers(self, painter: QtGui.QPainter) -> None:
        if self.home_point is not None:
            self._draw_marker(painter, self.home_point, "#f59e0b", "HOME")
        if self.nav_goal_point is not None:
            self._draw_marker(painter, self.nav_goal_point, "#8b5cf6", "GOAL")
        if self.robot_point is not None:
            self._draw_robot(painter, self.robot_point, self.robot_yaw)
        for x, y, label in self.trash_points:
            self._draw_trash(painter, x, y, label)

    def _draw_direction_preview(
        self,
        painter: QtGui.QPainter,
        anchor_point: Optional[Tuple[float, float]],
        preview_point: Optional[Tuple[float, float]],
        color: str,
    ) -> None:
        if anchor_point is None:
            return
        anchor = self._map_to_screen(*anchor_point)
        if anchor is None:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(QtCore.QPointF(anchor[0], anchor[1]), 9, 9)

        if preview_point is None:
            return
        preview = self._map_to_screen(*preview_point)
        if preview is None:
            return

        painter.setPen(QtGui.QPen(QtGui.QColor(color), 3))
        painter.drawLine(int(anchor[0]), int(anchor[1]), int(preview[0]), int(preview[1]))
        angle = math.atan2(preview[1] - anchor[1], preview[0] - anchor[0])
        arrow_len = 14
        left = (
            preview[0] - arrow_len * math.cos(angle - math.pi / 6),
            preview[1] - arrow_len * math.sin(angle - math.pi / 6),
        )
        right = (
            preview[0] - arrow_len * math.cos(angle + math.pi / 6),
            preview[1] - arrow_len * math.sin(angle + math.pi / 6),
        )
        painter.drawLine(int(preview[0]), int(preview[1]), int(left[0]), int(left[1]))
        painter.drawLine(int(preview[0]), int(preview[1]), int(right[0]), int(right[1]))

    def _draw_marker(
        self,
        painter: QtGui.QPainter,
        point: Tuple[float, float],
        color: str,
        text: str,
    ) -> None:
        canvas = self._map_to_screen(point[0], point[1])
        if canvas is None:
            return
        marker_rect = QtCore.QRectF(canvas[0] - 10, canvas[1] - 10, 20, 20)
        painter.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
        painter.setBrush(QtGui.QColor(color))
        painter.drawEllipse(marker_rect)

        label_rect = QtCore.QRectF(canvas[0] + 14, canvas[1] - 16, 58, 24)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 23, 42, 215))
        painter.drawRoundedRect(label_rect, 6, 6)
        painter.setPen(QtGui.QColor("#f8fafc"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8, QtGui.QFont.Bold))
        painter.drawText(label_rect, QtCore.Qt.AlignCenter, text)

    def _draw_robot(self, painter: QtGui.QPainter, point: Tuple[float, float], yaw: float) -> None:
        canvas = self._map_to_screen(point[0], point[1])
        if canvas is None:
            return
        painter.save()
        painter.translate(canvas[0], canvas[1])
        painter.rotate((-math.degrees(yaw) + self.view_rotation_deg) % 360.0)

        shadow_color = QtGui.QColor(2, 6, 23, 110)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(shadow_color)
        painter.drawRoundedRect(QtCore.QRectF(-34, -24, 68, 48), 12, 12)

        wheel_brush = QtGui.QColor("#111827")
        wheel_highlight = QtGui.QColor("#374151")
        for wheel_rect in [
            QtCore.QRectF(-35, -28, 12, 18),
            QtCore.QRectF(23, -28, 12, 18),
            QtCore.QRectF(-35, 10, 12, 18),
            QtCore.QRectF(23, 10, 12, 18),
        ]:
            painter.setBrush(wheel_brush)
            painter.drawRoundedRect(wheel_rect, 4, 4)
            painter.setBrush(wheel_highlight)
            painter.drawRoundedRect(wheel_rect.adjusted(2, 3, -2, -3), 3, 3)

        body_rect = QtCore.QRectF(-26, -18, 52, 36)
        body_gradient = QtGui.QLinearGradient(body_rect.topLeft(), body_rect.bottomRight())
        body_gradient.setColorAt(0.0, QtGui.QColor("#f8fafc"))
        body_gradient.setColorAt(0.55, QtGui.QColor("#cbd5e1"))
        body_gradient.setColorAt(1.0, QtGui.QColor("#94a3b8"))
        painter.setBrush(QtGui.QBrush(body_gradient))
        painter.setPen(QtGui.QPen(QtGui.QColor("#e2e8f0"), 1.8))
        painter.drawRoundedRect(body_rect, 9, 9)

        painter.setPen(QtGui.QPen(QtGui.QColor("#2563eb"), 2))
        painter.drawLine(QtCore.QPointF(-12, 0), QtCore.QPointF(14, 0))
        painter.drawLine(QtCore.QPointF(14, 0), QtCore.QPointF(20, -8))

        arm_pen = QtGui.QPen(QtGui.QColor("#f59e0b"), 4, cap=QtCore.Qt.RoundCap, join=QtCore.Qt.RoundJoin)
        painter.setPen(arm_pen)
        painter.drawLine(QtCore.QPointF(-2, 0), QtCore.QPointF(8, -10))
        painter.drawLine(QtCore.QPointF(8, -10), QtCore.QPointF(18, -16))
        painter.drawLine(QtCore.QPointF(18, -16), QtCore.QPointF(28, -16))

        claw_pen = QtGui.QPen(QtGui.QColor("#fb7185"), 3, cap=QtCore.Qt.RoundCap)
        painter.setPen(claw_pen)
        painter.drawLine(QtCore.QPointF(28, -16), QtCore.QPointF(33, -20))
        painter.drawLine(QtCore.QPointF(28, -16), QtCore.QPointF(33, -12))

        painter.setPen(QtGui.QPen(QtGui.QColor("#0f172a"), 2.4))
        painter.drawLine(QtCore.QPointF(0, 0), QtCore.QPointF(30, 0))
        painter.drawLine(QtCore.QPointF(30, 0), QtCore.QPointF(22, -5))
        painter.drawLine(QtCore.QPointF(30, 0), QtCore.QPointF(22, 5))
        painter.restore()

        label_rect = QtCore.QRectF(canvas[0] + 18, canvas[1] - 26, 118, 22)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 23, 42, 215))
        painter.drawRoundedRect(label_rect, 6, 6)
        painter.setPen(QtGui.QColor("#f8fafc"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8, QtGui.QFont.Bold))
        painter.drawText(label_rect, QtCore.Qt.AlignCenter, "R550A+ 4WD ARM")

    def _draw_trash(self, painter: QtGui.QPainter, x: float, y: float, label: str) -> None:
        canvas = self._map_to_screen(x, y)
        if canvas is None:
            return
        painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1.2))
        painter.setBrush(QtGui.QColor("#ef4444"))
        painter.drawEllipse(QtCore.QPointF(canvas[0], canvas[1]), 5.5, 5.5)
        painter.setPen(QtGui.QColor("#7f1d1d"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 9))
        text_rect = QtCore.QRectF(canvas[0] + 10, canvas[1] - 22, 114, 20)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(255, 255, 255, 224))
        painter.drawRoundedRect(text_rect, 5, 5)
        painter.setPen(QtGui.QColor("#7f1d1d"))
        painter.drawText(text_rect.adjusted(6, 0, -6, 0), QtCore.Qt.AlignVCenter, label)

    def _draw_initial_pose_preview(self, painter: QtGui.QPainter) -> None:
        self._draw_direction_preview(
            painter,
            self.initial_pose_anchor,
            self.initial_pose_preview,
            "#f97316",
        )

    def _draw_nav_goal_preview(self, painter: QtGui.QPainter) -> None:
        self._draw_direction_preview(
            painter,
            self.nav_goal_anchor,
            self.nav_goal_preview,
            "#8b5cf6",
        )

    def _draw_status_overlay(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
        info_text = {
            "nav": "模式: 导航目标点",
            "home": "模式: 回家点",
            "region": "模式: 区域框选",
            "initial_pose": "模式: 2D 初始位姿",
        }.get(self.mode, f"模式: {self.mode}")

        if self.mode == "initial_pose" and self.initial_pose_anchor is not None:
            info_text += " | 第二次点击确定朝向"

        if self.last_hover_point is not None:
            info_text += f" | x={self.last_hover_point[0]:.2f}, y={self.last_hover_point[1]:.2f}"

        overlay_rect = QtCore.QRectF(rect.left() + 14, rect.bottom() - 50, 470, 34)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 23, 42, 200))
        painter.drawRoundedRect(overlay_rect, 8, 8)
        painter.setPen(QtGui.QColor("#f8fafc"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 10))
        painter.drawText(overlay_rect.adjusted(12, 0, -12, 0), QtCore.Qt.AlignVCenter, info_text)

    def _draw_view_overlay(self, painter: QtGui.QPainter, rect: QtCore.QRectF) -> None:
        zoom_text = int(self.zoom_factor * 100)
        rotate_text = int(self.view_rotation_deg) % 360
        overlay_rect = QtCore.QRectF(rect.right() - 356, rect.top() + 14, 342, 66)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(15, 23, 42, 210))
        painter.drawRoundedRect(overlay_rect, 8, 8)
        painter.setPen(QtGui.QColor("#f8fafc"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 10, QtGui.QFont.Bold))
        painter.drawText(
            overlay_rect.adjusted(12, 8, -12, -30),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            f"缩放 {zoom_text}%   视角 {rotate_text}°   网格 {'开' if self.show_grid else '关'}",
        )
        painter.setPen(QtGui.QColor("#cbd5e1"))
        painter.setFont(QtGui.QFont("Microsoft YaHei", 8))
        painter.drawText(
            overlay_rect.adjusted(12, 32, -12, -8),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            "滚轮缩放 | 右键拖动平移 | Ctrl+右键拖动旋转 | 右键双击反转 180°",
        )

    def _map_to_screen(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        if self.map_meta.width <= 0 or self.map_meta.height <= 0 or self.map_image.isNull():
            return None
        px = (x - self.map_meta.origin_x) / self.map_meta.resolution
        py = self.map_meta.height - ((y - self.map_meta.origin_y) / self.map_meta.resolution)
        point = self._view_transform().map(QtCore.QPointF(px, py))
        return (point.x(), point.y())

    def _screen_to_map(self, point: QtCore.QPoint) -> Optional[Tuple[float, float]]:
        if self.map_meta.width <= 0 or self.map_meta.height <= 0 or self.map_image.isNull():
            return None
        transform, invertible = self._view_transform().inverted()
        if not invertible:
            return None
        mapped = transform.map(QtCore.QPointF(point))
        px = mapped.x()
        py = mapped.y()
        if px < 0.0 or px > self.map_meta.width:
            return None
        if py < 0.0 or py > self.map_meta.height:
            return None
        map_x = self.map_meta.origin_x + px * self.map_meta.resolution
        map_y = self.map_meta.origin_y + (self.map_meta.height - py) * self.map_meta.resolution
        return map_x, map_y

    def _apply_zoom(self, factor: float, anchor: QtCore.QPointF) -> None:
        before = self._screen_to_map(anchor.toPoint())
        old_zoom = self.zoom_factor
        self.zoom_factor = max(self.min_zoom, min(self.max_zoom, self.zoom_factor * factor))
        if abs(self.zoom_factor - old_zoom) < 1e-6:
            return
        if before is not None:
            after = self._map_to_screen(*before)
            if after is not None:
                self.pan_offset += QtCore.QPointF(anchor.x() - after[0], anchor.y() - after[1])
        self.update()

    def _viewport_rect(self) -> QtCore.QRectF:
        return self.rect().adjusted(14, 14, -14, -14)

    def _base_scale(self) -> float:
        viewport = self._viewport_rect()
        if self.map_meta.width <= 0 or self.map_meta.height <= 0:
            return 1.0
        return min(
            viewport.width() / float(self.map_meta.width),
            viewport.height() / float(self.map_meta.height),
        )

    def _view_transform(self) -> QtGui.QTransform:
        viewport = self._viewport_rect()
        scale = self._base_scale() * self.zoom_factor
        center = viewport.center() + self.pan_offset
        transform = QtGui.QTransform()
        transform.translate(center.x(), center.y())
        transform.rotate(self.view_rotation_deg)
        transform.scale(scale, scale)
        transform.translate(-self.map_meta.width / 2.0, -self.map_meta.height / 2.0)
        return transform

    def _map_screen_polygon(self) -> QtGui.QPolygonF:
        if self.map_meta.width <= 0 or self.map_meta.height <= 0 or self.map_image.isNull():
            return QtGui.QPolygonF()
        transform = self._view_transform()
        corners = [
            QtCore.QPointF(0.0, 0.0),
            QtCore.QPointF(float(self.map_meta.width), 0.0),
            QtCore.QPointF(float(self.map_meta.width), float(self.map_meta.height)),
            QtCore.QPointF(0.0, float(self.map_meta.height)),
        ]
        return QtGui.QPolygonF([transform.map(point) for point in corners])


class MainWindow(QtWidgets.QMainWindow):
    ssh_result_ready = QtCore.pyqtSignal(str, bool, str)
    task_exit_ready = QtCore.pyqtSignal(str, int)
    rviz_log_ready = QtCore.pyqtSignal(str)

    def __init__(self, config: StationConfig) -> None:
        super().__init__()
        self.config = config
        self.resource_dir = resource_base_dir()
        self.runtime_dir = runtime_base_dir()
        self.config_path = self.runtime_dir / "config" / "station_client.yaml"

        self.mode = "nav"
        self.region_points: List[Tuple[float, float]] = []
        self.nav_goal_anchor: Optional[Tuple[float, float]] = None
        self.initial_pose_anchor: Optional[Tuple[float, float]] = None
        self.last_pick_label = "unknown"
        self.last_grasp_result = ""
        self.detect_count = 0
        self.success_pick_count = 0
        self.failed_pick_count = 0
        self.coverage_count = 0
        self.region_count = 0
        self.arm_busy = False
        self.label_counter: Counter = Counter()
        self.log_lines: Deque[str] = deque(maxlen=300)
        self.workflow_labels: List[QtWidgets.QLabel] = []
        self.connection_alert_visible = False
        self.task_started_at: Optional[float] = None
        self.last_state_value = "idle"
        self.last_nav_goal_sent_at: Optional[float] = None
        self.task_processes: Dict[str, subprocess.Popen] = {}
        self.pending_task_starts: Dict[str, QtCore.QTimer] = {}
        self.rviz_process: Optional[QtCore.QProcess] = None
        self.rviz_container: Optional[QtWidgets.QWidget] = None
        self.rviz_window: Optional[QtGui.QWindow] = None
        self.rviz_window_id: Optional[int] = None
        self.rviz_mode = "map"
        self.task_display_names = {
            "mapping": "建图模式",
            "nav": "导航模式",
            "shuttle": "往返跑模式",
            "arm": "机械臂任务",
        }

        self.setWindowTitle("机械臂小车上位机")
        self.resize(1800, 1040)
        self.setMinimumSize(1560, 920)

        self._create_config_widgets()

        self.ros = RosBridgeClient(config.rosbridge_url)
        self.local_ros = LocalRosClient()
        self.ssh_result_ready.connect(self.on_ssh_result_ready)
        self.task_exit_ready.connect(self.on_task_exit_ready)
        self.rviz_log_ready.connect(self.append_log)

        self._build_ui()
        self._apply_styles()
        self._bind_ros()
        self._load_initial_values()
        self._build_config_dialog()
        self.initialize_center_workspace()
        self.ros.start()
        self.local_ros.start()

        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self.refresh_runtime_labels)
        self.stats_timer.start(1000)

        self.nav_state_watchdog = QtCore.QTimer(self)
        self.nav_state_watchdog.timeout.connect(self.check_navigation_feedback)
        self.nav_state_watchdog.start(1000)

    def _create_config_widgets(self) -> None:
        self.rosbridge_edit = QtWidgets.QLineEdit()
        self.rdk_host_edit = QtWidgets.QLineEdit()
        self.rdk_workspace_edit = QtWidgets.QLineEdit()
        self.jetson_host_edit = QtWidgets.QLineEdit()
        self.jetson_workspace_edit = QtWidgets.QLineEdit()
        self.rosbridge_edit.setPlaceholderText("ws://192.168.1.142:9090")
        self.rdk_host_edit.setPlaceholderText("sunrise@192.168.1.142")
        self.jetson_host_edit.setPlaceholderText("jetson@机械臂IP，例如 jetson@192.168.1.205")
        self.map_edit = QtWidgets.QLineEdit()
        self.map_save_stem_edit = QtWidgets.QLineEdit()
        self.robot_urdf_edit = QtWidgets.QLineEdit()
        self.auto_start_arm_checkbox = QtWidgets.QCheckBox("任务模式自动启动 Jetson 机械臂")

        self.mapping_cmd_edit = QtWidgets.QPlainTextEdit()
        self.nav_cmd_edit = QtWidgets.QPlainTextEdit()
        self.shuttle_cmd_edit = QtWidgets.QPlainTextEdit()
        self.arm_cmd_edit = QtWidgets.QPlainTextEdit()
        self.mapping_cmd_edit.setMinimumHeight(120)
        self.nav_cmd_edit.setMinimumHeight(140)
        self.shuttle_cmd_edit.setMinimumHeight(140)
        self.arm_cmd_edit.setMinimumHeight(180)

        self.config_summary_label = QtWidgets.QLabel()
        self.config_summary_label.setObjectName("infoLabel")
        self.config_summary_label.setWordWrap(True)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        root.addWidget(self._build_header())
        root.addWidget(self._build_toolbar())

        body = QtWidgets.QHBoxLayout()
        body.setSpacing(14)
        body.addWidget(self._build_left_panel(), 24)
        body.addWidget(self._build_center_panel(), 54)
        body.addWidget(self._build_right_panel(), 22)
        root.addLayout(body, 1)

    def _build_header(self) -> QtWidgets.QWidget:
        frame = QtWidgets.QFrame()
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(24, 16, 24, 16)
        layout.setSpacing(12)

        title_box = QtWidgets.QVBoxLayout()
        title_box.setSpacing(4)
        title = QtWidgets.QLabel("机械臂小车作业站")
        title.setObjectName("titleLabel")
        subtitle = QtWidgets.QLabel("Windows 11 上位机 | RDKX5 小车端 | Jetson 机械臂端")
        subtitle.setObjectName("subtitleLabel")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)

        layout.addLayout(title_box)
        layout.addStretch(1)

        self.mode_chip = self._make_chip("地图模式: 导航点", "#0f766e")
        self.state_chip = self._make_chip("状态: DISCONNECTED", "#1d4ed8")
        layout.addWidget(self.mode_chip)
        layout.addWidget(self.state_chip)
        return frame

    def _build_toolbar(self) -> QtWidgets.QWidget:
        frame = QtWidgets.QFrame()
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(10)

        layout.addWidget(self._make_button("建图模式", self.start_mapping_mode, primary=True))
        layout.addWidget(self._make_button("导航模式", self.start_nav_mode, primary=True))
        layout.addWidget(self._make_button("往返跑模式", self.start_shuttle_mode, primary=True))
        layout.addWidget(self._make_button("任务模式", self.start_task_mode, primary=True))
        layout.addWidget(self._make_button("停止建图", self.stop_mapping_mode, danger=True))
        layout.addWidget(self._make_button("停止导航", self.stop_nav_mode, danger=True))
        layout.addWidget(self._make_button("停止往返跑", self.stop_shuttle_mode, danger=True))
        layout.addWidget(self._make_button("停止机械臂", self.stop_arm_mode, danger=True))
        layout.addWidget(self._make_button("全部停止", self.stop_all_modes, danger=True))
        layout.addWidget(self._make_button("保存地图", self.save_map_remote))
        layout.addWidget(self._make_button("配置", self.open_config_dialog))
        layout.addSpacing(12)
        self.nav_mode_button = self._make_button("导航点", lambda: self.set_mode("nav"), checkable=True)
        self.shuttle_mode_button = self._make_button("往返跑", lambda: self.set_mode("shuttle"), checkable=True)
        self.home_mode_button = self._make_button("回家点", lambda: self.set_mode("home"), checkable=True)
        self.region_mode_button = self._make_button("框选区域", lambda: self.set_mode("region"), checkable=True)
        self.initial_pose_mode_button = self._make_button(
            "2D 初始位姿",
            lambda: self.set_mode("initial_pose"),
            checkable=True,
        )
        layout.addWidget(self.nav_mode_button)
        layout.addWidget(self.shuttle_mode_button)
        layout.addWidget(self.home_mode_button)
        layout.addWidget(self.region_mode_button)
        layout.addWidget(self.initial_pose_mode_button)
        layout.addStretch(1)
        layout.addWidget(self._make_button("清空区域", self.clear_region))
        layout.addWidget(self._make_button("立即回家", self.return_home))
        return frame

    def _build_left_panel(self) -> QtWidgets.QWidget:
        frame = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setSpacing(14)

        status_panel = self._make_panel("系统状态")
        status_layout = status_panel.layout()

        metrics = QtWidgets.QGridLayout()
        metrics.setHorizontalSpacing(12)
        metrics.setVerticalSpacing(10)
        self.map_size_metric = self._make_metric("地图尺寸", "未加载")
        self.robot_metric = self._make_metric("底盘定位", "未定位")
        self.detect_metric = self._make_metric("识别总数", "0")
        self.coverage_metric = self._make_metric("扫荡点数", "0")
        metrics.addWidget(self.map_size_metric, 0, 0)
        metrics.addWidget(self.robot_metric, 0, 1)
        metrics.addWidget(self.detect_metric, 1, 0)
        metrics.addWidget(self.coverage_metric, 1, 1)
        status_layout.addLayout(metrics)

        self.status_text = QtWidgets.QLabel("等待 rosbridge 与地图数据。")
        self.status_text.setObjectName("infoLabel")
        self.status_text.setWordWrap(True)
        self.vision_text = QtWidgets.QLabel("视觉: 未接收调试画面")
        self.vision_text.setObjectName("infoLabel")
        self.last_target_text = QtWidgets.QLabel("最近目标: 无")
        self.last_target_text.setObjectName("infoLabel")
        self.arm_state_text = QtWidgets.QLabel("机械臂状态: 空闲")
        self.arm_state_text.setObjectName("infoLabel")
        self.cursor_text = QtWidgets.QLabel("地图坐标: -")
        self.cursor_text.setObjectName("infoLabel")

        status_layout.addWidget(self.status_text)
        status_layout.addWidget(self.vision_text)
        status_layout.addWidget(self.last_target_text)
        status_layout.addWidget(self.arm_state_text)
        status_layout.addWidget(self.cursor_text)

        stats_panel = self._make_panel("任务统计面板")
        stats_layout = stats_panel.layout()

        stats_grid = QtWidgets.QGridLayout()
        stats_grid.setHorizontalSpacing(12)
        stats_grid.setVerticalSpacing(10)
        self.success_metric = self._make_metric("抓取成功", "0")
        self.failed_metric = self._make_metric("抓取失败", "0")
        self.region_metric = self._make_metric("区域顶点", "0")
        self.runtime_metric = self._make_metric("任务时长", "00:00:00")
        stats_grid.addWidget(self.success_metric, 0, 0)
        stats_grid.addWidget(self.failed_metric, 0, 1)
        stats_grid.addWidget(self.region_metric, 1, 0)
        stats_grid.addWidget(self.runtime_metric, 1, 1)
        stats_layout.addLayout(stats_grid)

        stats_layout.addWidget(self._make_section_label("分类统计"))
        self.label_stats_list = QtWidgets.QListWidget()
        self.label_stats_list.setObjectName("statsList")
        self.label_stats_list.setMinimumHeight(160)
        stats_layout.addWidget(self.label_stats_list)

        workflow_panel = self._make_panel("任务流程")
        workflow_layout = workflow_panel.layout()
        for idx, text in enumerate(
            [
                "获取地图或加载已有地图",
                "设置初始位姿",
                "设置回家点",
                "设置作业导航点",
                "框选垃圾作业区域",
                "扫荡识别并暂停抓取",
                "抓取完成继续扫荡",
                "扫荡结束自动回家",
            ],
            start=1,
        ):
            row = QtWidgets.QHBoxLayout()
            badge = QtWidgets.QLabel(str(idx))
            badge.setObjectName("badgeLabel")
            label = QtWidgets.QLabel(text)
            label.setObjectName("workflowLabel")
            row.addWidget(badge)
            row.addWidget(label, 1)
            workflow_layout.addLayout(row)
            self.workflow_labels.append(label)

        layout.addWidget(status_panel)
        layout.addWidget(stats_panel)
        layout.addWidget(workflow_panel)
        layout.addStretch(1)
        return frame

    def _build_center_panel(self) -> QtWidgets.QWidget:
        frame = self._make_panel("地图作业视图", with_layout=False)
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        view_bar = QtWidgets.QHBoxLayout()
        view_bar.setSpacing(8)
        view_bar.addWidget(self._make_button("放大", lambda: self.map_canvas.zoom_in()))
        view_bar.addWidget(self._make_button("缩小", lambda: self.map_canvas.zoom_out()))
        view_bar.addWidget(self._make_button("左转 15°", lambda: self.map_canvas.rotate_view(-15.0)))
        view_bar.addWidget(self._make_button("右转 15°", lambda: self.map_canvas.rotate_view(15.0)))
        view_bar.addWidget(self._make_button("180°", lambda: self.map_canvas.flip_view()))
        view_bar.addWidget(self._make_button("复位", lambda: self.map_canvas.reset_view()))
        view_bar.addWidget(self._make_button("网格", lambda: self.map_canvas.toggle_grid()))
        view_bar.addStretch(1)

        self.map_canvas = MapCanvas()
        self.map_canvas.mapClicked.connect(self.on_map_clicked)
        self.map_canvas.mapHovered.connect(self.on_map_hovered)

        hint = QtWidgets.QLabel(
            "地图交互说明: 导航点和回家点为单击设置，区域框选连续点击两次对角点，"
            "2D 初始位姿模式下第一次点击位置、第二次点击确定朝向。"
            "滚轮缩放，右键拖动平移，Ctrl+右键拖动旋转，右键双击反转 180°。"
        )
        hint.setObjectName("hintLabel")
        hint.setWordWrap(True)

        layout.addLayout(view_bar)
        layout.addWidget(self.map_canvas, 1)
        layout.addWidget(hint)
        return frame

    def _build_right_panel(self) -> QtWidgets.QWidget:
        frame = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setSpacing(14)

        robot_panel = self._make_panel("3D 机器人视窗")
        robot_layout = robot_panel.layout()
        default_urdf = self.config.robot_urdf_path or default_robot_urdf_path()
        self.robot_3d_view = Robot3DWidget(default_urdf)
        robot_tools = QtWidgets.QHBoxLayout()
        robot_tools.setSpacing(8)
        robot_tools.addWidget(self._make_button("3D复位", self.robot_3d_view.reset_view))
        robot_tools.addStretch(1)
        robot_layout.addWidget(self.robot_3d_view)
        robot_layout.addLayout(robot_tools)

        vision_panel = self._make_panel("机械臂实时视觉")
        vision_layout = vision_panel.layout()
        self.vision_image = QtWidgets.QLabel("等待 /mission/debug_image/compressed")
        self.vision_image.setObjectName("visionImage")
        self.vision_image.setAlignment(QtCore.Qt.AlignCenter)
        vision_layout.addWidget(self.vision_image)

        logs_panel = self._make_panel("运行日志")
        logs_layout = logs_panel.layout()
        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setObjectName("logText")
        logs_layout.addWidget(self.log_text)

        summary_panel = self._make_panel("当前配置摘要")
        summary_layout = summary_panel.layout()
        summary_layout.addWidget(self.config_summary_label)
        summary_layout.addWidget(self._make_button("打开配置", self.open_config_dialog))

        layout.addWidget(robot_panel, 34)
        layout.addWidget(vision_panel, 28)
        layout.addWidget(logs_panel, 23)
        layout.addWidget(summary_panel, 15)
        return frame

    def _build_config_dialog(self) -> None:
        self.config_dialog = QtWidgets.QDialog(self)
        self.config_dialog.setWindowTitle("系统配置")
        self.config_dialog.resize(980, 760)

        root = QtWidgets.QVBoxLayout(self.config_dialog)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        tabs = QtWidgets.QTabWidget()

        base_tab = QtWidgets.QWidget()
        base_layout = QtWidgets.QVBoxLayout(base_tab)
        base_layout.setSpacing(12)
        form = QtWidgets.QFormLayout()
        form.setLabelAlignment(QtCore.Qt.AlignLeft)
        form.setFormAlignment(QtCore.Qt.AlignTop)
        form.setHorizontalSpacing(18)
        form.setVerticalSpacing(12)
        form.addRow("rosbridge 地址", self.rosbridge_edit)
        form.addRow("RDKX5 SSH（用户名@IP）", self.rdk_host_edit)
        form.addRow("RDKX5 工作空间", self.rdk_workspace_edit)
        form.addRow("机械臂/Jetson SSH（用户名@IP）", self.jetson_host_edit)
        form.addRow("机械臂/Jetson 工作空间", self.jetson_workspace_edit)
        form.addRow("导航地图 YAML", self.map_edit)
        form.addRow("保存地图目标", self.map_save_stem_edit)
        form.addRow("3D 机器人 URDF", self.robot_urdf_edit)
        base_layout.addLayout(form)
        base_layout.addWidget(self.auto_start_arm_checkbox)

        helper_row = QtWidgets.QHBoxLayout()
        helper_row.addWidget(self._make_button("设置远程地图路径", self.choose_remote_map_yaml))
        helper_row.addWidget(self._make_button("重连 rosbridge", self.restart_connection))
        helper_row.addStretch(1)
        base_layout.addLayout(helper_row)
        base_layout.addStretch(1)

        command_tab = QtWidgets.QWidget()
        command_layout = QtWidgets.QVBoxLayout(command_tab)
        command_layout.setSpacing(10)
        for title, editor in [
            ("建图命令", self.mapping_cmd_edit),
            ("导航命令", self.nav_cmd_edit),
            ("往返跑命令", self.shuttle_cmd_edit),
            ("机械臂命令", self.arm_cmd_edit),
        ]:
            command_layout.addWidget(self._make_section_label(title))
            command_layout.addWidget(editor)

        tabs.addTab(base_tab, "连接参数")
        tabs.addTab(command_tab, "远程命令")
        root.addWidget(tabs, 1)

        buttons = QtWidgets.QHBoxLayout()
        buttons.addStretch(1)
        buttons.addWidget(self._make_button("保存配置", self.save_config, primary=True))
        close_button = QtWidgets.QPushButton("关闭")
        close_button.clicked.connect(self.config_dialog.close)
        buttons.addWidget(close_button)
        root.addLayout(buttons)

    def _apply_styles(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #09111d;
                color: #e5e7eb;
                font-family: Microsoft YaHei, Segoe UI, Arial;
            }
            QFrame {
                background: #101826;
                border-radius: 8px;
            }
            QDialog {
                background: #09111d;
                color: #e5e7eb;
            }
            QStackedWidget, QWidget#mapCanvas, QFrame#rvizSurface {
                background: #0b1322;
                border-radius: 10px;
            }
            QTabWidget::pane {
                border: 1px solid #243041;
                background: #0f1826;
                border-radius: 6px;
            }
            QTabBar::tab {
                background: #172131;
                color: #dbe4f0;
                padding: 10px 16px;
                margin-right: 4px;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                font-size: 12px;
            }
            QTabBar::tab:selected {
                background: #2563eb;
                color: white;
            }
            QLabel#titleLabel {
                font-size: 34px;
                font-weight: 700;
                color: #f8fafc;
            }
            QLabel#subtitleLabel {
                font-size: 14px;
                color: #94a3b8;
            }
            QLabel#chipLabel {
                color: white;
                font-size: 12px;
                font-weight: 700;
            }
            QLabel#panelTitle {
                font-size: 18px;
                font-weight: 700;
                color: #f8fafc;
            }
            QLabel#fieldTitle {
                color: #94a3b8;
                font-size: 12px;
                margin-top: 2px;
            }
            QLabel#metricTitle {
                color: #94a3b8;
                font-size: 12px;
            }
            QLabel#metricValue {
                color: #f8fafc;
                font-size: 22px;
                font-weight: 700;
            }
            QLabel#infoLabel, QLabel#workflowLabel, QLabel#hintLabel {
                color: #d5deea;
                font-size: 12px;
            }
            QLabel#rvizPlaceholder {
                color: #cbd5e1;
                font-size: 15px;
                background: #0b1322;
                border: 1px solid #22314a;
                border-radius: 10px;
                padding: 24px;
            }
            QLabel#badgeLabel {
                background: #1f2937;
                color: #e5e7eb;
                min-width: 28px;
                max-width: 28px;
                min-height: 28px;
                max-height: 28px;
                border-radius: 5px;
                font-size: 12px;
                font-weight: 700;
                qproperty-alignment: AlignCenter;
            }
            QPushButton {
                background: #1f2937;
                color: #f8fafc;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 10px 16px;
                font-size: 13px;
            }
            QPushButton:hover {
                background: #273449;
            }
            QPushButton:checked {
                background: #0f766e;
                border: 1px solid #14b8a6;
                color: white;
            }
            QPushButton[primary="true"] {
                background: #2563eb;
                border: none;
            }
            QPushButton[primary="true"]:hover {
                background: #1d4ed8;
            }
            QPushButton[danger="true"] {
                background: #7f1d1d;
                border: none;
            }
            QPushButton[danger="true"]:hover {
                background: #991b1b;
            }
            QLineEdit, QPlainTextEdit, QListWidget {
                background: #0c1522;
                border: 1px solid #334155;
                border-radius: 6px;
                color: #e5e7eb;
                padding: 8px;
                font-size: 12px;
            }
            QLabel#visionImage {
                background: #020617;
                border: 1px solid #1e293b;
                border-radius: 6px;
                min-height: 320px;
                font-size: 16px;
            }
            QPlainTextEdit#logText, QListWidget#statsList {
                background: #020617;
            }
            QListWidget::item {
                padding: 5px 2px;
            }
            QCheckBox {
                spacing: 8px;
                color: #e5e7eb;
                font-size: 12px;
            }
            """
        )

    def _bind_ros(self) -> None:
        self.ros.connected.connect(self.on_ros_connected)
        self.ros.disconnected.connect(self.on_ros_disconnected)
        self.ros.state_received.connect(self.on_state_received)
        self.ros.map_received.connect(self.on_map_received)
        self.ros.odom_received.connect(self.on_odom_received)
        self.ros.home_pose_received.connect(self.on_home_pose_received)
        self.ros.nav_goal_received.connect(self.on_nav_goal_received)
        self.ros.rviz_goal_received.connect(self.on_rviz_goal_received)
        self.ros.rviz_clicked_point_received.connect(self.on_rviz_clicked_point_received)
        self.ros.rviz_initial_pose_received.connect(self.on_rviz_initial_pose_received)
        self.ros.trash_pose_received.connect(self.on_trash_pose_received)
        self.ros.trash_label_received.connect(self.on_trash_label_received)
        self.ros.region_received.connect(self.on_region_received)
        self.ros.coverage_received.connect(self.on_coverage_received)
        self.ros.arm_busy_received.connect(self.on_arm_busy_received)
        self.ros.grasp_result_received.connect(self.on_grasp_result_received)
        self.ros.debug_image_received.connect(self.on_debug_image_received)
        self.local_ros.map_received.connect(self.on_map_received)
        self.local_ros.log_received.connect(self.append_log)
        self.ros.log_received.connect(self.append_log)

    def _load_initial_values(self) -> None:
        self.rosbridge_edit.setText(self.config.rosbridge_url)
        self.rdk_host_edit.setText(self.config.rdk_host)
        self.rdk_workspace_edit.setText(self.config.rdk_workspace)
        self.jetson_host_edit.setText(self.config.jetson_host)
        self.jetson_workspace_edit.setText(self.config.jetson_workspace)
        self.map_edit.setText(self.config.map_yaml)
        self.map_save_stem_edit.setText(self.config.map_save_stem)
        self.robot_urdf_edit.setText(self.config.robot_urdf_path)
        self.auto_start_arm_checkbox.setChecked(self.config.auto_start_arm_remote)
        self.mapping_cmd_edit.setPlainText(self.config.mapping_command)
        self.nav_cmd_edit.setPlainText(self.config.nav_command)
        self.shuttle_cmd_edit.setPlainText(self.config.shuttle_command)
        self.arm_cmd_edit.setPlainText(self.config.arm_command)
        self.refresh_config_summary()
        self.refresh_label_stats()

    def _make_chip(self, text: str, color: str) -> QtWidgets.QWidget:
        frame = QtWidgets.QFrame()
        frame.setStyleSheet(f"background: {color}; border-radius: 8px;")
        layout = QtWidgets.QHBoxLayout(frame)
        layout.setContentsMargins(14, 8, 14, 8)
        label = QtWidgets.QLabel(text)
        label.setObjectName("chipLabel")
        layout.addWidget(label)
        frame.label = label
        return frame

    def _make_button(
        self,
        text: str,
        handler,
        primary: bool = False,
        checkable: bool = False,
        danger: bool = False,
    ) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton(text)
        button.clicked.connect(handler)
        button.setCheckable(checkable)
        if primary:
            button.setProperty("primary", True)
            button.style().unpolish(button)
            button.style().polish(button)
        if danger:
            button.setProperty("danger", True)
            button.style().unpolish(button)
            button.style().polish(button)
        return button

    def _make_panel(self, title: str, with_layout: bool = True) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame()
        if with_layout:
            layout = QtWidgets.QVBoxLayout(frame)
            layout.setContentsMargins(14, 14, 14, 14)
            layout.setSpacing(10)
            label = QtWidgets.QLabel(title)
            label.setObjectName("panelTitle")
            layout.addWidget(label)
        return frame

    def _make_metric(self, title: str, value: str) -> QtWidgets.QFrame:
        frame = QtWidgets.QFrame()
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(4)
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName("metricTitle")
        value_label = QtWidgets.QLabel(value)
        value_label.setObjectName("metricValue")
        layout.addWidget(title_label)
        layout.addWidget(value_label)
        frame.value_label = value_label
        return frame

    def _make_section_label(self, text: str) -> QtWidgets.QLabel:
        label = QtWidgets.QLabel(text)
        label.setObjectName("fieldTitle")
        return label

    def refresh_config_summary(self) -> None:
        summary = [
            f"rosbridge: {self.rosbridge_edit.text().strip() or '-'}",
            f"RDKX5: {self.rdk_host_edit.text().strip() or '-'}",
            f"机械臂/Jetson: {self.jetson_host_edit.text().strip() or '-'}",
            f"导航地图: {self.map_edit.text().strip() or '-'}",
            f"保存地图: {self.map_save_stem_edit.text().strip() or '-'}",
            f"3D模型: {Path(self.robot_urdf_edit.text().strip()).name if self.robot_urdf_edit.text().strip() else '-'}",
            "机械臂自动启动: " + ("开启" if self.auto_start_arm_checkbox.isChecked() else "关闭"),
        ]
        self.config_summary_label.setText("\n".join(summary))

    def open_config_dialog(self) -> None:
        self.config_dialog.show()
        self.config_dialog.raise_()
        self.config_dialog.activateWindow()

    def update_state_chip(self, text: str, color: str) -> None:
        self.state_chip.label.setText(text)
        self.state_chip.setStyleSheet(f"background: {color}; border-radius: 8px;")

    def set_mode(self, mode: str) -> None:
        self.mode = mode
        display = {
            "nav": "导航点",
            "shuttle": "往返跑",
            "home": "回家点",
            "region": "区域框选",
            "initial_pose": "2D 初始位姿",
        }.get(mode, mode)
        self.mode_chip.label.setText(f"地图模式: {display}")
        self.map_canvas.set_mode(mode)
        self.ros.publish_string("/station/click_mode", mode)
        self.local_ros.publish_string("/station/click_mode", mode)
        self.nav_goal_anchor = None
        self.map_canvas.set_nav_goal_anchor(None)
        self.initial_pose_anchor = None
        self.map_canvas.set_initial_pose_anchor(None)
        self._sync_mode_buttons(mode)
        self.append_log(f"切换地图操作模式: {display}")

    def _sync_mode_buttons(self, active_mode: str) -> None:
        mapping = {
            "nav": self.nav_mode_button,
            "shuttle": self.shuttle_mode_button,
            "home": self.home_mode_button,
            "region": self.region_mode_button,
            "initial_pose": self.initial_pose_mode_button,
        }
        for mode, button in mapping.items():
            button.blockSignals(True)
            button.setChecked(mode == active_mode)
            button.blockSignals(False)

    def on_map_clicked(self, x: float, y: float) -> None:
        if self.mode in ("nav", "shuttle"):
            goal_topic = "/mission/shuttle_goal_pose" if self.mode == "shuttle" else "/mission/nav_goal_pose"
            goal_label = "往返跑目标" if self.mode == "shuttle" else "导航目标"
            if self.nav_goal_anchor is None:
                self.nav_goal_anchor = (x, y)
                self.map_canvas.set_nav_goal_anchor(self.nav_goal_anchor)
                self.map_canvas.set_goal(x, y)
                self.append_log(f"设置{goal_label}位置点: x={x:.2f}, y={y:.2f}，请再点击一次确定朝向。")
                return

            anchor_x, anchor_y = self.nav_goal_anchor
            nav_yaw = math.atan2(y - anchor_y, x - anchor_x)
            if abs(x - anchor_x) < 1e-6 and abs(y - anchor_y) < 1e-6:
                nav_yaw = self.map_canvas.robot_yaw if self.map_canvas.robot_point is not None else 0.0
            self.map_canvas.set_goal(anchor_x, anchor_y, nav_yaw)
            self.map_canvas.set_nav_goal_anchor(None)
            self.nav_goal_anchor = None
            self.ros.publish_pose(goal_topic, anchor_x, anchor_y, nav_yaw)
            self.local_ros.publish_pose(goal_topic, anchor_x, anchor_y, nav_yaw)
            self.last_nav_goal_sent_at = time.time()
            self.append_log(
                f"发布{goal_label}点: x={anchor_x:.2f}, y={anchor_y:.2f}, yaw={math.degrees(nav_yaw):.1f} deg"
            )
            return

        if self.mode == "home":
            self.map_canvas.set_home(x, y)
            home_yaw = self.map_canvas.robot_yaw if self.map_canvas.robot_point is not None else 0.0
            self.ros.publish_pose("/mission/home_pose", x, y, home_yaw)
            self.local_ros.publish_pose("/mission/home_pose", x, y, home_yaw)
            self.append_log(f"设置回家点: x={x:.2f}, y={y:.2f}")
            return

        if self.mode == "region":
            self.region_points.append((x, y))
            self.region_metric.value_label.setText(str(len(self.region_points)))
            self.ros.publish_region_point(x, y)
            self.local_ros.publish_region_point(x, y)
            self.append_log(f"区域角点: x={x:.2f}, y={y:.2f}")
            if len(self.region_points) >= 2:
                self.region_points = []
                self.region_metric.value_label.setText("0")
                self.append_log("已完成区域框选，等待小车侧生成扫荡路径。")
            return

        if self.mode == "initial_pose":
            if self.initial_pose_anchor is None:
                self.initial_pose_anchor = (x, y)
                self.map_canvas.set_initial_pose_anchor(self.initial_pose_anchor)
                self.append_log(
                    f"已设置初始位姿位置点: x={x:.2f}, y={y:.2f}，请再点击一次确定朝向。"
                )
                return

            anchor_x, anchor_y = self.initial_pose_anchor
            yaw = math.atan2(y - anchor_y, x - anchor_x)
            if abs(x - anchor_x) < 1e-6 and abs(y - anchor_y) < 1e-6:
                yaw = 0.0
            self.ros.publish_initial_pose(anchor_x, anchor_y, yaw)
            self.local_ros.publish_initial_pose(anchor_x, anchor_y, yaw)
            self.map_canvas.set_robot(anchor_x, anchor_y, yaw)
            self.map_canvas.set_initial_pose_anchor(None)
            self.initial_pose_anchor = None
            self.append_log(
                f"发布 2D 初始位姿: x={anchor_x:.2f}, y={anchor_y:.2f}, yaw={math.degrees(yaw):.1f} deg"
            )

    def on_map_hovered(self, x: float, y: float) -> None:
        self.cursor_text.setText(f"地图坐标: x={x:.2f}, y={y:.2f}")

    def on_rviz_goal_received(self, x: float, y: float, yaw: float) -> None:
        self.map_canvas.set_goal(x, y, yaw)
        if self.mode in ("nav", "shuttle"):
            self.last_nav_goal_sent_at = time.time()
            goal_label = "往返跑目标" if self.mode == "shuttle" else "导航目标"
            self.append_log(
                f"RViz {goal_label}: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
            )
            return

        self._dispatch_rviz_pose_by_mode(x, y, yaw, "RViz 2D Goal")

    def on_rviz_clicked_point_received(self, x: float, y: float) -> None:
        self.cursor_text.setText(f"RViz clicked point: x={x:.2f}, y={y:.2f}")
        self.append_log(f"RViz 点击点: x={x:.2f}, y={y:.2f}")
        self.on_map_clicked(x, y)

    def on_rviz_initial_pose_received(self, x: float, y: float, yaw: float) -> None:
        if self.mode == "initial_pose":
            self.map_canvas.set_robot(x, y, yaw)
            self.append_log(
                f"RViz 初始位姿: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
            )
            return

        self._dispatch_rviz_pose_by_mode(x, y, yaw, "RViz 2D Pose")

    def _dispatch_rviz_pose_by_mode(self, x: float, y: float, yaw: float, source: str) -> None:
        if self.mode == "home":
            self.map_canvas.set_home(x, y)
            self.ros.publish_pose("/mission/home_pose", x, y, yaw)
            self.local_ros.publish_pose("/mission/home_pose", x, y, yaw)
            self.append_log(f"{source} 设置回家点: x={x:.2f}, y={y:.2f}")
            return

        if self.mode == "region":
            self.region_points.append((x, y))
            self.region_metric.value_label.setText(str(len(self.region_points)))
            self.ros.publish_region_point(x, y)
            self.local_ros.publish_region_point(x, y)
            self.append_log(f"{source} 区域角点: x={x:.2f}, y={y:.2f}")
            if len(self.region_points) >= 2:
                self.region_points = []
                self.region_metric.value_label.setText("0")
                self.append_log("已完成区域框选，等待小车侧生成清洁路径。")
            return

        if self.mode in ("nav", "shuttle"):
            goal_topic = "/mission/shuttle_goal_pose" if self.mode == "shuttle" else "/mission/nav_goal_pose"
            goal_label = "往返跑目标" if self.mode == "shuttle" else "导航点"
            self.map_canvas.set_goal(x, y, yaw)
            self.ros.publish_pose(goal_topic, x, y, yaw)
            self.local_ros.publish_pose(goal_topic, x, y, yaw)
            self.last_nav_goal_sent_at = time.time()
            self.append_log(
                f"{source} 发布{goal_label}: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f} deg"
            )

    def choose_remote_map_yaml(self) -> None:
        text, ok = QtWidgets.QInputDialog.getText(
            self,
            "导航地图 YAML",
            "请输入 RDKX5 上的地图 YAML 路径:",
            text=self.map_edit.text().strip(),
        )
        if ok and text.strip():
            self.map_edit.setText(text.strip())
            self.refresh_config_summary()

    def save_config(self) -> None:
        if yaml is None:
            self.show_warning("保存配置失败", "缺少 PyYAML，无法保存上位机配置。")
            return

        rosbridge_url = self.normalize_rosbridge_url(self.rosbridge_edit.text().strip())
        rdk_host = self.normalize_rdk_host(self.rdk_host_edit.text().strip())
        rdk_workspace = self.rdk_workspace_edit.text().strip() or self.config.rdk_workspace
        jetson_host = self.normalize_jetson_host(self.jetson_host_edit.text().strip())
        jetson_workspace = self.jetson_workspace_edit.text().strip() or self.config.jetson_workspace
        map_yaml = self.map_edit.text().strip()
        map_save_stem = self.map_save_stem_edit.text().strip() or self.map_stem_from_yaml(map_yaml)
        robot_urdf_path = self.robot_urdf_edit.text().strip()

        self.rosbridge_edit.setText(rosbridge_url)
        self.rdk_host_edit.setText(rdk_host)
        self.rdk_workspace_edit.setText(rdk_workspace)
        self.jetson_host_edit.setText(jetson_host)
        self.jetson_workspace_edit.setText(jetson_workspace)
        self.map_save_stem_edit.setText(map_save_stem)

        config_data = {
            "station_client": {
                "rosbridge_url": rosbridge_url,
                "rdk_host": rdk_host,
                "rdk_workspace": rdk_workspace,
                "jetson_host": jetson_host,
                "jetson_workspace": jetson_workspace,
                "auto_start_arm_remote": self.auto_start_arm_checkbox.isChecked(),
                "map_yaml": map_yaml,
                "map_save_stem": map_save_stem,
                "robot_urdf_path": robot_urdf_path,
                "mapping_command": self.mapping_cmd_edit.toPlainText().strip(),
                "nav_command": self.nav_cmd_edit.toPlainText().strip(),
                "shuttle_command": self.shuttle_cmd_edit.toPlainText().strip(),
                "arm_command": self.arm_cmd_edit.toPlainText().strip(),
            }
        }

        self.config_path.parent.mkdir(parents=True, exist_ok=True)
        self.config_path.write_text(
            yaml.safe_dump(config_data, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )

        self.config = StationConfig(
            rosbridge_url=rosbridge_url,
            rdk_host=rdk_host,
            rdk_workspace=rdk_workspace,
            jetson_host=jetson_host,
            jetson_workspace=jetson_workspace,
            auto_start_arm_remote=self.auto_start_arm_checkbox.isChecked(),
            map_yaml=map_yaml,
            map_save_stem=map_save_stem,
            robot_urdf_path=robot_urdf_path,
            mapping_command=self.mapping_cmd_edit.toPlainText().strip(),
            nav_command=self.nav_cmd_edit.toPlainText().strip(),
            shuttle_command=self.shuttle_cmd_edit.toPlainText().strip(),
            arm_command=self.arm_cmd_edit.toPlainText().strip(),
        )
        if hasattr(self, "robot_3d_view"):
            self.robot_3d_view.set_urdf_path(robot_urdf_path)
        self.refresh_config_summary()
        self.append_log(f"配置已保存: {self.config_path}")
        self.show_info("保存成功", "上位机配置已保存。")

    def restart_connection(self) -> None:
        rosbridge_url = self.normalize_rosbridge_url(self.rosbridge_edit.text().strip())
        self.rosbridge_edit.setText(rosbridge_url)
        self.ros.stop()
        self.ros = RosBridgeClient(rosbridge_url)
        self._bind_ros()
        self.ros.start()
        self.refresh_config_summary()
        self.append_log(f"正在重连 rosbridge: {rosbridge_url}")

    def start_mapping_mode(self) -> None:
        self.launch_managed_task(
            task_key="mapping",
            title="建图模式",
            host=self.normalize_rdk_host(self.rdk_host_edit.text().strip()),
            command=self.ensure_rdk_command_environment(self.mapping_cmd_edit.toPlainText().strip()),
            conflicts=["nav", "shuttle"],
        )

    def start_nav_mode(self) -> None:
        self.sync_nav_command_map_path()
        self.launch_managed_task(
            task_key="nav",
            title="导航模式",
            host=self.normalize_rdk_host(self.rdk_host_edit.text().strip()),
            command=self.ensure_rdk_command_environment(self.nav_cmd_edit.toPlainText().strip()),
            conflicts=["mapping", "shuttle"],
        )

    def start_shuttle_mode(self) -> None:
        self.sync_shuttle_command_map_path()
        self.launch_managed_task(
            task_key="shuttle",
            title="往返跑模式",
            host=self.normalize_rdk_host(self.rdk_host_edit.text().strip()),
            command=self.ensure_rdk_command_environment(self.shuttle_cmd_edit.toPlainText().strip()),
            conflicts=["mapping", "nav"],
        )
        self.set_mode("shuttle")
        if self.auto_start_arm_checkbox.isChecked():
            self.launch_managed_task(
                task_key="arm",
                title="往返跑模式-机械臂",
                host=self.jetson_host_edit.text().strip(),
                command=self.ensure_arm_command_environment(self.arm_cmd_edit.toPlainText().strip()),
            )
        else:
            self.append_log("未勾选自动启动机械臂，请在 Jetson 上手动启动机械臂节点。")

    def start_task_mode(self) -> None:
        self.sync_nav_command_map_path()
        self.launch_managed_task(
            task_key="nav",
            title="任务模式-小车",
            host=self.normalize_rdk_host(self.rdk_host_edit.text().strip()),
            command=self.ensure_rdk_command_environment(self.nav_cmd_edit.toPlainText().strip()),
            conflicts=["mapping", "shuttle"],
            restart_if_running=False,
        )
        if self.auto_start_arm_checkbox.isChecked():
            self.launch_managed_task(
                task_key="arm",
                title="任务模式-机械臂",
                host=self.jetson_host_edit.text().strip(),
                command=self.ensure_arm_command_environment(self.arm_cmd_edit.toPlainText().strip()),
                restart_if_running=False,
            )
        else:
            self.append_log("未勾选自动启动机械臂，请在 Jetson 上手动启动机械臂节点。")

    def stop_mapping_mode(self) -> None:
        self.stop_managed_task("mapping")

    def stop_nav_mode(self) -> None:
        self.stop_managed_task("nav")

    def stop_shuttle_mode(self) -> None:
        self.stop_managed_task("shuttle")

    def stop_arm_mode(self) -> None:
        self.stop_managed_task("arm")

    def stop_all_modes(self) -> None:
        stopped = False
        for task_key in ("mapping", "nav", "shuttle", "arm"):
            stopped = self.stop_managed_task(task_key, quiet=True) or stopped
        if stopped:
            self.append_log("已发送全部停止指令。")
        else:
            self.append_log("当前没有可停止的远程任务。")

    def save_map_remote(self) -> None:
        host = self.normalize_rdk_host(self.rdk_host_edit.text().strip())
        stem = self.map_save_stem_edit.text().strip() or self.map_stem_from_yaml(self.map_edit.text().strip())
        if not stem:
            self.show_warning("保存地图失败", "请先填写远程保存地图目标路径。")
            return

        self.map_save_stem_edit.setText(stem)
        self.map_edit.setText(f"{stem}.yaml")
        self.sync_nav_command_map_path()
        self.sync_shuttle_command_map_path()
        self.refresh_config_summary()

        workspace = self.rdk_workspace_edit.text().strip() or self.config.rdk_workspace
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {workspace}/install/setup.bash && "
            f"mkdir -p \"$(dirname \\\"{stem}\\\")\" && "
            f"ros2 run nav2_map_server map_saver_cli -f \"{stem}\""
        )
        self.append_log(f"开始远程保存地图到: {stem}")
        self.run_ssh_capture("保存地图", host, command)

    def launch_ssh_terminal(self, title: str, host: str, command: str) -> None:
        if not host:
            self.show_warning(f"{title} 启动失败", "未配置 SSH 主机。")
            return
        if not command.strip():
            self.show_warning(f"{title} 启动失败", "远程命令为空。")
            return

        full_command = ssh_shell_command(host, command, tty=True)
        self.append_log(f"{title} -> {host}")
        try:
            if is_windows():
                subprocess.Popen(
                    ["powershell", "-NoExit", "-Command", full_command],
                    creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0),
                )
            else:
                terminal_command = self._linux_terminal_command(full_command, hold_open=True)
                if terminal_command is None:
                    subprocess.Popen(
                        ["bash", "-lc", f"{full_command}; exec bash"],
                        start_new_session=True,
                    )
                else:
                    subprocess.Popen(terminal_command, start_new_session=True)
        except FileNotFoundError:
            self.show_warning(
                f"{title} 启动失败",
                "未找到可用终端或 ssh，请确认系统已安装 OpenSSH 客户端。",
            )
        except Exception as exc:
            self.show_warning(f"{title} 启动失败", str(exc))

    def _linux_terminal_command(self, shell_command: str, hold_open: bool = False) -> Optional[List[str]]:
        command = shell_command if not hold_open else f"{shell_command}; exec bash"

        if shutil.which("x-terminal-emulator"):
            return ["x-terminal-emulator", "-e", "bash", "-lc", command]
        if shutil.which("gnome-terminal"):
            return ["gnome-terminal", "--", "bash", "-lc", command]
        if shutil.which("konsole"):
            return ["konsole", "-e", "bash", "-lc", command]
        if shutil.which("xfce4-terminal"):
            return ["xfce4-terminal", "--command", f"bash -lc \"{command}\""]
        if shutil.which("xterm"):
            return ["xterm", "-hold", "-e", "bash", "-lc", command]
        return None

    def launch_managed_task(
        self,
        task_key: str,
        title: str,
        host: str,
        command: str,
        conflicts: Optional[List[str]] = None,
        restart_if_running: bool = True,
    ) -> None:
        if not host:
            self.show_warning(f"{title} 启动失败", "未配置 SSH 主机。")
            return
        if not command.strip():
            self.show_warning(f"{title} 启动失败", "远程命令为空。")
            return

        delay_ms = 0
        for conflict_key in conflicts or []:
            if self.stop_managed_task(conflict_key, quiet=True):
                delay_ms = max(delay_ms, 1500)

        if self.is_task_running(task_key):
            if not restart_if_running:
                self.append_log(f"{title} 已在运行，忽略重复启动。")
                return
            self.append_log(f"{title} 已在运行，先停止旧任务再重新启动。")
            self.stop_managed_task(task_key, quiet=True)
            delay_ms = max(delay_ms, 1500)

        self.cancel_pending_task_launch(task_key)
        self.append_log(f"{title} -> {host}")

        timer = QtCore.QTimer(self)
        timer.setSingleShot(True)
        timer.timeout.connect(lambda: self._spawn_managed_task(task_key, title, host, command))
        timer.start(delay_ms)
        self.pending_task_starts[task_key] = timer

    def _spawn_managed_task(self, task_key: str, title: str, host: str, command: str) -> None:
        self.cancel_pending_task_launch(task_key)
        ssh_args = build_ssh_args(host, command, batch_mode=True, tty=True)

        try:
            if is_windows():
                creationflags = getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
                creationflags |= getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
                process = subprocess.Popen(
                    ssh_args,
                    creationflags=creationflags,
                )
            else:
                process = subprocess.Popen(
                    ssh_args,
                    start_new_session=True,
                )
        except FileNotFoundError:
            self.show_warning(
                f"{title} 启动失败",
                "未找到 bash 或 ssh，请确认系统已安装 OpenSSH 客户端。",
            )
            return
        except Exception as exc:
            self.show_warning(f"{title} 启动失败", str(exc))
            return

        self.task_processes[task_key] = process
        self.append_log(f"{title} 已启动，可用停止按钮或 Ctrl+C 式停止。")
        threading.Thread(
            target=self._wait_for_task_process,
            args=(task_key, process),
            daemon=True,
        ).start()

    def _wait_for_task_process(self, task_key: str, process: subprocess.Popen) -> None:
        return_code = process.wait()
        self.task_exit_ready.emit(task_key, return_code)

    def on_task_exit_ready(self, task_key: str, return_code: int) -> None:
        process = self.task_processes.get(task_key)
        if process is not None and process.poll() is not None:
            self.task_processes.pop(task_key, None)
        title = self.task_display_names.get(task_key, task_key)
        self.append_log(f"{title} 已退出，返回码: {return_code}")
        if return_code == 255:
            self.append_log(
                f"{title} 返回 255：SSH 连接失败或免密登录失败，请检查主机地址、网络、known_hosts 和 ssh-copy-id。"
            )

    def is_task_running(self, task_key: str) -> bool:
        process = self.task_processes.get(task_key)
        if process is None:
            return False
        if process.poll() is not None:
            self.task_processes.pop(task_key, None)
            return False
        return True

    def cancel_pending_task_launch(self, task_key: str) -> None:
        timer = self.pending_task_starts.pop(task_key, None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()

    def stop_managed_task(self, task_key: str, quiet: bool = False) -> bool:
        self.cancel_pending_task_launch(task_key)
        process = self.task_processes.get(task_key)
        title = self.task_display_names.get(task_key, task_key)
        if process is None or process.poll() is not None:
            self.task_processes.pop(task_key, None)
            if not quiet:
                self.append_log(f"{title} 当前没有在运行的任务。")
            return False

        try:
            if is_windows() and hasattr(signal, "CTRL_BREAK_EVENT"):
                process.send_signal(signal.CTRL_BREAK_EVENT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    pass

            if process.poll() is None and not is_windows():
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=4)
                except Exception:
                    pass

            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass

            if process.poll() is None and is_windows():
                subprocess.run(
                    ["taskkill", "/PID", str(process.pid), "/T", "/F"],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="ignore",
                    timeout=10,
                )
            elif process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except Exception:
                    process.kill()
        except Exception as exc:
            if not quiet:
                self.append_log(f"{title} 停止时出现异常: {exc}")
        finally:
            self.task_processes.pop(task_key, None)

        if not quiet:
            self.append_log(f"已停止 {title}。")
        return True

    def run_ssh_capture(self, title: str, host: str, command: str) -> None:
        if not host:
            self.show_warning(f"{title} 执行失败", "未配置 SSH 主机。")
            return
        if not command.strip():
            self.show_warning(f"{title} 执行失败", "远程命令为空。")
            return

        def worker() -> None:
            try:
                completed = subprocess.run(
                    build_ssh_args(host, command, batch_mode=True, tty=False),
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="ignore",
                    timeout=180,
                )
                output = "\n".join(part for part in [completed.stdout.strip(), completed.stderr.strip()] if part)
                success = completed.returncode == 0
                if not output:
                    output = "命令执行完成。"
                self.ssh_result_ready.emit(title, success, output)
            except FileNotFoundError:
                self.ssh_result_ready.emit(
                    title,
                    False,
                    "未找到 ssh 命令，请在 Windows 11 中安装并启用 OpenSSH Client。",
                )
            except subprocess.TimeoutExpired:
                self.ssh_result_ready.emit(title, False, "远程命令执行超时。")
            except Exception as exc:
                self.ssh_result_ready.emit(title, False, str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def on_ssh_result_ready(self, title: str, success: bool, output: str) -> None:
        self.append_log(f"{title} 结果: {'成功' if success else '失败'}")
        if output:
            self.append_log(output)
        if success:
            if title == "保存地图":
                self.show_info(
                    "保存地图完成",
                    f"地图已保存到:\n{self.map_save_stem_edit.text().strip()}.yaml",
                )
        else:
            self.show_warning(f"{title} 失败", output)

    def ensure_rdk_command_environment(self, command: str) -> str:
        command_text = command.strip()
        if not command_text:
            return command_text

        prefixes = []
        if "source /opt/ros/humble/setup.bash" not in command_text:
            prefixes.append("source /opt/ros/humble/setup.bash")

        workspace = self.rdk_workspace_edit.text().strip() or self.config.rdk_workspace
        workspace_source = f"source {workspace}/install/setup.bash"
        if workspace_source not in command_text:
            prefixes.append(workspace_source)

        if not prefixes:
            return command_text
        return " && ".join(prefixes + [command_text])

    def ensure_arm_command_environment(self, command: str) -> str:
        command_text = command.strip()
        if not command_text:
            return command_text

        prefixes = []
        if "source /opt/ros/humble/setup.bash" not in command_text:
            prefixes.append("source /opt/ros/humble/setup.bash")

        workspace = self.jetson_workspace_edit.text().strip() or self.config.jetson_workspace
        workspace_source = f"source {workspace}/install/setup.bash"
        if workspace_source not in command_text:
            prefixes.append(workspace_source)

        if not prefixes:
            return command_text
        return " && ".join(prefixes + [command_text])

    def normalize_rosbridge_url(self, text: str) -> str:
        value = text.strip()
        if not value:
            return self.config.rosbridge_url
        if value.startswith("ws://") or value.startswith("wss://"):
            return value
        if ":" in value and value.count(":") == 1 and value.replace(".", "").replace(":", "").isdigit():
            return f"ws://{value}"
        return f"ws://{value}:9090"

    def normalize_rdk_host(self, text: str) -> str:
        value = text.strip()
        if not value:
            return self.config.rdk_host
        if "@" in value:
            return value
        if self.is_ipv4(value):
            return f"sunrise@{value}"
        return f"{value}@192.168.1.142"

    def normalize_jetson_host(self, text: str) -> str:
        value = text.strip()
        if not value:
            return self.config.jetson_host
        if "@" in value:
            return value
        if self.is_ipv4(value):
            return f"jetson@{value}"
        return f"jetson@{value}"

    @staticmethod
    def is_ipv4(text: str) -> bool:
        parts = text.split(".")
        if len(parts) != 4:
            return False
        try:
            return all(0 <= int(part) <= 255 for part in parts)
        except ValueError:
            return False

    @staticmethod
    def map_stem_from_yaml(yaml_path: str) -> str:
        path = yaml_path.strip()
        if path.endswith(".yaml"):
            return path[:-5]
        return path

    def sync_nav_command_map_path(self) -> None:
        map_yaml = self.map_edit.text().strip()
        if not map_yaml:
            return
        nav_command = self.nav_cmd_edit.toPlainText().strip()
        if "map:=" not in nav_command:
            return
        prefix = nav_command.split("map:=", 1)[0]
        suffix = nav_command.split("map:=", 1)[1]
        if " " in suffix:
            remainder = suffix.split(" ", 1)[1]
            nav_command = f"{prefix}map:={map_yaml} {remainder}"
        else:
            nav_command = f"{prefix}map:={map_yaml}"
        self.nav_cmd_edit.setPlainText(nav_command)

    def sync_shuttle_command_map_path(self) -> None:
        map_yaml = self.map_edit.text().strip()
        if not map_yaml:
            return
        shuttle_command = self.shuttle_cmd_edit.toPlainText().strip()
        if "map:=" not in shuttle_command:
            return
        prefix = shuttle_command.split("map:=", 1)[0]
        suffix = shuttle_command.split("map:=", 1)[1]
        if " " in suffix:
            remainder = suffix.split(" ", 1)[1]
            shuttle_command = f"{prefix}map:={map_yaml} {remainder}"
        else:
            shuttle_command = f"{prefix}map:={map_yaml}"
        self.shuttle_cmd_edit.setPlainText(shuttle_command)

    def return_home(self) -> None:
        self.ros.publish_bool("/mission/return_home", True)
        self.local_ros.publish_bool("/mission/return_home", True)
        self.append_log("已发送立即回家指令。")

    def clear_region(self) -> None:
        self.region_points = []
        self.nav_goal_anchor = None
        self.map_canvas.set_nav_goal_anchor(None)
        self.coverage_count = 0
        self.coverage_metric.value_label.setText("0")
        self.region_metric.value_label.setText("0")
        self.map_canvas.clear_region()
        self.ros.publish_bool("/mission/clear_region", True)
        self.local_ros.publish_bool("/mission/clear_region", True)
        self.append_log("已发送清空区域指令。")

    def on_ros_connected(self) -> None:
        self.update_state_chip("状态: CONNECTED", "#0f766e")
        self.status_text.setText("rosbridge 已连接，等待任务状态与地图。")
        self.connection_alert_visible = False

    def on_ros_disconnected(self, reason: str) -> None:
        self.update_state_chip("状态: DISCONNECTED", "#b91c1c")
        self.status_text.setText(f"rosbridge 断开: {reason}")
        if not self.connection_alert_visible:
            self.connection_alert_visible = True
            self.show_warning(
                "rosbridge 连接断开",
                "请检查 RDKX5 上的 rosbridge_server 是否启动。"
                f"\n当前错误: {reason}",
            )

    def on_state_received(self, state: str) -> None:
        self.last_state_value = state
        display = state.replace("_", " ").upper()
        self.update_state_chip(f"状态: {display}", "#1d4ed8")
        self.status_text.setText(f"当前任务状态: {state}")
        if state not in ("idle", "") and self.task_started_at is None:
            self.task_started_at = time.time()
        if state == "idle" and not self.arm_busy:
            self.task_started_at = None
        self.refresh_runtime_labels()
        self._refresh_workflow(state)

    def on_map_received(self, msg: Dict) -> None:
        info = msg.get("info", {})
        width = int(info.get("width", 0))
        height = int(info.get("height", 0))
        if width <= 0 or height <= 0:
            return

        resolution = float(info.get("resolution", 0.05))
        origin = info.get("origin", {}).get("position", {})
        meta = MapMeta(
            resolution=resolution,
            origin_x=float(origin.get("x", 0.0)),
            origin_y=float(origin.get("y", 0.0)),
            width=width,
            height=height,
        )

        data = msg.get("data", [])
        image = QtGui.QImage(width, height, QtGui.QImage.Format_RGB32)
        for y in range(height):
            for x in range(width):
                idx = (height - 1 - y) * width + x
                value = data[idx] if idx < len(data) else -1
                if value < 0:
                    color = QtGui.QColor("#dde3ea")
                elif value > 50:
                    intensity = max(35, 90 - min(value, 100) // 2)
                    color = QtGui.QColor(intensity, intensity, intensity)
                else:
                    intensity = 250 - int(value * 0.55)
                    intensity = max(214, min(250, intensity))
                    color = QtGui.QColor(intensity, intensity, intensity)
                image.setPixelColor(x, y, color)

        self.map_canvas.update_map(image, meta)
        self.map_size_metric.value_label.setText(f"{width} x {height}")
        self.status_text.setText(f"地图已加载: {width} x {height}")

    def on_odom_received(self, x: float, y: float, yaw: float) -> None:
        self.map_canvas.set_robot(x, y, yaw)
        self.robot_metric.value_label.setText(f"x={x:.2f}, y={y:.2f}")

    def on_home_pose_received(self, x: float, y: float) -> None:
        self.map_canvas.set_home(x, y)

    def on_nav_goal_received(self, x: float, y: float, yaw: float) -> None:
        self.map_canvas.set_goal(x, y, yaw)

    def on_trash_pose_received(self, x: float, y: float, _label: str) -> None:
        self.detect_count += 1
        self.detect_metric.value_label.setText(str(self.detect_count))
        label = self.last_pick_label or "unknown"
        self.map_canvas.add_trash_point(x, y, label)
        self.append_log(f"识别目标: {label}, x={x:.3f}, y={y:.3f}")

    def on_trash_label_received(self, label: str) -> None:
        normalized = (label or "unknown").strip() or "unknown"
        self.last_pick_label = normalized
        self.label_counter[normalized] += 1
        self.last_target_text.setText(f"最近目标: {normalized}")
        self.refresh_label_stats()

    def on_region_received(self, polygon: List[Tuple[float, float]]) -> None:
        self.region_count = len(polygon)
        self.region_metric.value_label.setText(str(self.region_count))
        self.map_canvas.update_region(polygon)

    def on_coverage_received(self, path: List[Tuple[float, float]]) -> None:
        self.coverage_count = len(path)
        self.coverage_metric.value_label.setText(str(self.coverage_count))
        self.map_canvas.update_coverage(path)
        self.append_log(f"扫荡路径已更新，共 {len(path)} 个路径点。")

    def on_arm_busy_received(self, busy: bool) -> None:
        self.arm_busy = busy
        self.arm_state_text.setText(f"机械臂状态: {'抓取中' if busy else '空闲'}")

    def on_grasp_result_received(self, result: str) -> None:
        result_text = result.strip()
        if not result_text:
            return
        self.last_grasp_result = result_text
        if result_text == "grasp_finished":
            self.success_pick_count += 1
            self.success_metric.value_label.setText(str(self.success_pick_count))
            self.append_log("机械臂抓取成功。")
            return

        if result_text.startswith("grasp_failed"):
            self.failed_pick_count += 1
            self.failed_metric.value_label.setText(str(self.failed_pick_count))
            self.append_log(f"机械臂抓取失败: {result_text}")
            self.show_warning("机械臂抓取失败", result_text)

    def on_debug_image_received(self, image: QtGui.QImage) -> None:
        if image.isNull():
            return
        pixmap = QtGui.QPixmap.fromImage(image)
        pixmap = pixmap.scaled(420, 320, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.vision_image.setPixmap(pixmap)
        self.vision_text.setText("视觉: 已接收实时调试画面")

    def append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        line = f"[{timestamp}] {text}"
        self.log_lines.append(line)
        self.log_text.setPlainText("\n".join(self.log_lines))
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def refresh_label_stats(self) -> None:
        self.label_stats_list.clear()
        if not self.label_counter:
            self.label_stats_list.addItem("暂无分类统计")
            return
        for label, count in self.label_counter.most_common():
            self.label_stats_list.addItem(f"{label}: {count}")

    def refresh_runtime_labels(self) -> None:
        if self.task_started_at is None:
            self.runtime_metric.value_label.setText("00:00:00")
            return
        elapsed = max(0, int(time.time() - self.task_started_at))
        hours = elapsed // 3600
        minutes = (elapsed % 3600) // 60
        seconds = elapsed % 60
        self.runtime_metric.value_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    def check_navigation_feedback(self) -> None:
        if self.last_nav_goal_sent_at is None:
            return
        elapsed = time.time() - self.last_nav_goal_sent_at
        if elapsed < 4.0:
            return
        if self.last_state_value in {
            "navigating_to_site",
            "waiting_region_selection",
            "coverage_running",
            "pausing_for_pick",
            "arm_picking",
            "returning_home",
        }:
            self.last_nav_goal_sent_at = None
            return
        self.append_log(
            "导航点已发布，但任务状态没有进入 navigating_to_site。"
            " 请检查小车端 mission_manager 和 Nav2 是否正在运行。"
        )
        self.last_nav_goal_sent_at = None

    def _refresh_workflow(self, state: str) -> None:
        state_to_index = {
            "idle": 0,
            "navigating_to_site": 3,
            "waiting_region_selection": 4,
            "coverage_running": 5,
            "pausing_for_pick": 5,
            "arm_picking": 6,
            "returning_home": 7,
        }
        active = state_to_index.get(state, 0)
        for idx, label in enumerate(self.workflow_labels):
            if idx < active:
                label.setStyleSheet("color: #60a5fa; font-size: 12px;")
            elif idx == active:
                label.setStyleSheet("color: #f8fafc; font-size: 12px;")
            else:
                label.setStyleSheet("color: #64748b; font-size: 12px;")

    def show_warning(self, title: str, message: str) -> None:
        QtWidgets.QMessageBox.warning(self, title, message)

    def show_info(self, title: str, message: str) -> None:
        QtWidgets.QMessageBox.information(self, title, message)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        for task_key in ("mapping", "nav", "shuttle", "arm"):
            self.stop_managed_task(task_key, quiet=True)
        self.stop_embedded_rviz()
        self.ros.stop()
        self.local_ros.stop()
        super().closeEvent(event)

    def _build_center_panel(self) -> QtWidgets.QWidget:
        frame = self._make_panel("工作区视图", with_layout=False)
        layout = QtWidgets.QVBoxLayout(frame)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        self.map_canvas = MapCanvas()
        self.map_canvas.mapClicked.connect(self.on_map_clicked)
        self.map_canvas.mapHovered.connect(self.on_map_hovered)
        self.map_canvas.setObjectName("mapCanvas")

        self.rviz_view_button = self._make_button("RViz工作区", lambda: self.switch_center_view("rviz"), checkable=True)
        self.map_view_button = self._make_button("轻量地图", lambda: self.switch_center_view("map"), checkable=True)

        view_bar = QtWidgets.QHBoxLayout()
        view_bar.setSpacing(8)
        view_bar.addWidget(self.rviz_view_button)
        view_bar.addWidget(self.map_view_button)
        view_bar.addSpacing(10)
        view_bar.addWidget(self._make_button("放大", lambda: self.map_canvas.zoom_in()))
        view_bar.addWidget(self._make_button("缩小", lambda: self.map_canvas.zoom_out()))
        view_bar.addWidget(self._make_button("左转15°", lambda: self.map_canvas.rotate_view(-15.0)))
        view_bar.addWidget(self._make_button("右转15°", lambda: self.map_canvas.rotate_view(15.0)))
        view_bar.addWidget(self._make_button("180°", lambda: self.map_canvas.flip_view()))
        view_bar.addWidget(self._make_button("复位", lambda: self.map_canvas.reset_view()))
        view_bar.addWidget(self._make_button("网格", lambda: self.map_canvas.toggle_grid()))
        view_bar.addStretch(1)
        self.center_mode_label = QtWidgets.QLabel("工作区: 轻量地图")
        self.center_mode_label.setObjectName("fieldTitle")
        view_bar.addWidget(self.center_mode_label)

        self.center_stack = QtWidgets.QStackedWidget()

        self.rviz_embed_panel = QtWidgets.QFrame()
        self.rviz_embed_panel.setObjectName("rvizSurface")
        rviz_layout = QtWidgets.QVBoxLayout(self.rviz_embed_panel)
        rviz_layout.setContentsMargins(0, 0, 0, 0)
        rviz_layout.setSpacing(0)
        self.rviz_placeholder = QtWidgets.QLabel(
            "RViz 工作区未接入。\n\n"
            "Linux + ROS2 环境下将优先尝试嵌入 rviz2。\n"
            "如果嵌入失败，会自动回退到轻量地图。"
        )
        self.rviz_placeholder.setObjectName("rvizPlaceholder")
        self.rviz_placeholder.setAlignment(QtCore.Qt.AlignCenter)
        self.rviz_placeholder.setWordWrap(True)
        rviz_layout.addWidget(self.rviz_placeholder, 1)

        self.map_panel = QtWidgets.QWidget()
        map_layout = QtWidgets.QVBoxLayout(self.map_panel)
        map_layout.setContentsMargins(0, 0, 0, 0)
        map_layout.setSpacing(10)
        map_layout.addWidget(self.map_canvas, 1)

        hint = QtWidgets.QLabel(
            "RViz工作区建议用于正式导航与区域操作；轻量地图用于 RViz 未接入时的备用交互。"
        )
        hint.setObjectName("hintLabel")
        hint.setWordWrap(True)
        map_layout.addWidget(hint)

        self.center_stack.addWidget(self.rviz_embed_panel)
        self.center_stack.addWidget(self.map_panel)

        layout.addLayout(view_bar)
        layout.addWidget(self.center_stack, 1)
        return frame

    def initialize_center_workspace(self) -> None:
        if is_linux():
            self.try_start_embedded_rviz()
        self.switch_center_view("rviz" if self.rviz_container is not None else "map")

    def switch_center_view(self, mode: str) -> None:
        self.rviz_mode = mode
        if mode == "rviz" and self.rviz_container is not None:
            self.center_stack.setCurrentWidget(self.rviz_embed_panel)
            self.center_mode_label.setText("工作区: RViz")
            self.rviz_view_button.setChecked(True)
            self.map_view_button.setChecked(False)
            return

        if mode == "rviz" and self.rviz_container is None:
            self.append_log("RViz 工作区暂未接入，已切回轻量地图。")

        self.center_stack.setCurrentWidget(self.map_panel)
        self.center_mode_label.setText("工作区: 轻量地图")
        self.rviz_view_button.setChecked(False)
        self.map_view_button.setChecked(True)

    def rviz_config_path(self) -> str:
        return str(self.resource_dir / "config" / "mobile_manipulator_station.rviz")

    def try_start_embedded_rviz(self) -> None:
        if not is_linux():
            return
        if shutil.which("rviz2") is None:
            self.rviz_log_ready.emit("未找到 rviz2，工作区保留为轻量地图。")
            return

        if self.rviz_process is not None:
            return

        self.rviz_process = QtCore.QProcess(self)
        self.rviz_process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.rviz_process.readyReadStandardOutput.connect(self._on_rviz_process_output)
        self.rviz_process.finished.connect(self._on_rviz_process_finished)

        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert("QT_QPA_PLATFORM", env.value("QT_QPA_PLATFORM", "xcb"))
        self.rviz_process.setProcessEnvironment(env)
        self.rviz_process.start("rviz2", ["-d", self.rviz_config_path()])

        if not self.rviz_process.waitForStarted(3000):
            self.rviz_log_ready.emit("RViz 启动失败，已回退到轻量地图。")
            self.rviz_process.deleteLater()
            self.rviz_process = None
            return

        QtCore.QTimer.singleShot(1800, self.attach_rviz_window)

    def attach_rviz_window(self) -> None:
        if self.rviz_process is None:
            return

        pid = int(self.rviz_process.processId())
        if pid <= 0:
            return

        for _ in range(10):
            window_id = self._find_window_id_by_pid(pid)
            if window_id:
                self._embed_rviz_window(window_id)
                return
            QtCore.QThread.msleep(300)

        self.rviz_log_ready.emit("未能嵌入 RViz 窗口，继续使用轻量地图。")

    def _find_window_id_by_pid(self, pid: int) -> Optional[int]:
        if shutil.which("xdotool") is None:
            return None
        completed = subprocess.run(
            ["xdotool", "search", "--pid", str(pid)],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="ignore",
            timeout=5,
        )
        if completed.returncode != 0:
            return None
        for raw in completed.stdout.splitlines():
            raw = raw.strip()
            if raw.isdigit():
                return int(raw)
        return None

    def _embed_rviz_window(self, window_id: int) -> None:
        if self.rviz_container is not None:
            return
        self.rviz_window_id = window_id
        self.rviz_window = QtGui.QWindow.fromWinId(window_id)
        if self.rviz_window is None:
            self.rviz_log_ready.emit("RViz 窗口句柄获取失败。")
            return
        self.rviz_window.setFlags(QtCore.Qt.FramelessWindowHint)
        self.rviz_container = QtWidgets.QWidget.createWindowContainer(self.rviz_window, self.rviz_embed_panel)
        self.rviz_container.setFocusPolicy(QtCore.Qt.StrongFocus)
        layout = self.rviz_embed_panel.layout()
        layout.removeWidget(self.rviz_placeholder)
        self.rviz_placeholder.hide()
        layout.addWidget(self.rviz_container, 1)
        QtCore.QTimer.singleShot(250, self.resize_embedded_rviz)
        QtCore.QTimer.singleShot(900, self.resize_embedded_rviz)
        self.rviz_log_ready.emit("RViz 工作区已嵌入上位机中间视图。")
        if self.rviz_mode == "rviz":
            self.switch_center_view("rviz")

    def _on_rviz_process_output(self) -> None:
        if self.rviz_process is None:
            return
        data = bytes(self.rviz_process.readAllStandardOutput()).decode("utf-8", errors="ignore").strip()
        if not data:
            return
        for line in data.splitlines()[-6:]:
            self.rviz_log_ready.emit(f"[rviz2] {line}")

    def _on_rviz_process_finished(self, exit_code: int, _exit_status) -> None:
        self.rviz_log_ready.emit(f"RViz 进程已退出，返回码: {exit_code}")
        self.stop_embedded_rviz(reset_process=False)
        self.switch_center_view("map")

    def resize_embedded_rviz(self) -> None:
        if self.rviz_container is None:
            return
        size = self.rviz_container.size()
        width = max(320, int(size.width()))
        height = max(240, int(size.height()))
        if self.rviz_window is not None:
            self.rviz_window.resize(width, height)
        if self.rviz_window_id is not None and shutil.which("xdotool"):
            try:
                subprocess.run(
                    ["xdotool", "windowsize", str(self.rviz_window_id), str(width), str(height)],
                    capture_output=True,
                    timeout=2,
                )
            except Exception:
                pass

    def stop_embedded_rviz(self, reset_process: bool = True) -> None:
        if self.rviz_container is not None:
            self.rviz_container.setParent(None)
            self.rviz_container.deleteLater()
            self.rviz_container = None
        self.rviz_window = None
        self.rviz_window_id = None

        if reset_process and self.rviz_process is not None:
            self.rviz_process.blockSignals(True)
            self.rviz_process.terminate()
            if not self.rviz_process.waitForFinished(1500):
                self.rviz_process.kill()
                self.rviz_process.waitForFinished(1000)
            self.rviz_process.deleteLater()
            self.rviz_process = None


def load_config() -> StationConfig:
    resource_config = resource_base_dir() / "config" / "station_client.yaml"
    runtime_config = runtime_base_dir() / "config" / "station_client.yaml"
    config_path = runtime_config if runtime_config.exists() else resource_config

    if not config_path.exists() or yaml is None:
        defaults = StationConfig()
        defaults.robot_urdf_path = default_robot_urdf_path()
        return defaults

    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    root = data.get("station_client", {})
    defaults = StationConfig()
    defaults.robot_urdf_path = default_robot_urdf_path()
    return StationConfig(
        rosbridge_url=root.get("rosbridge_url", defaults.rosbridge_url),
        rdk_host=root.get("rdk_host", defaults.rdk_host),
        rdk_workspace=root.get("rdk_workspace", defaults.rdk_workspace),
        jetson_host=root.get("jetson_host", defaults.jetson_host),
        jetson_workspace=root.get("jetson_workspace", defaults.jetson_workspace),
        auto_start_arm_remote=bool(root.get("auto_start_arm_remote", defaults.auto_start_arm_remote)),
        map_yaml=root.get("map_yaml", defaults.map_yaml),
        map_save_stem=root.get("map_save_stem", default_map_save_stem(defaults.map_yaml)),
        robot_urdf_path=root.get("robot_urdf_path", defaults.robot_urdf_path),
        mapping_command=root.get("mapping_command", defaults.mapping_command),
        nav_command=root.get("nav_command", defaults.nav_command),
        shuttle_command=root.get("shuttle_command", defaults.shuttle_command),
        arm_command=root.get("arm_command", defaults.arm_command),
    )


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("mobile_manipulator_station")
    config = load_config()
    window = MainWindow(config)

    def request_graceful_shutdown(*_args) -> None:
        window.append_log("收到终端停止信号，正在停止远程任务并关闭上位机。")
        window.close()

    signal.signal(signal.SIGINT, request_graceful_shutdown)
    signal.signal(signal.SIGTERM, request_graceful_shutdown)
    signal_timer = QtCore.QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(200)

    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
