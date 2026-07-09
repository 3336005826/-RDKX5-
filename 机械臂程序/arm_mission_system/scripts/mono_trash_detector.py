#!/usr/bin/env python3

import math
import os
import re
import sys
import time

import cv2 as cv
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from ultralytics import YOLO


JETCOBOT_UTILS_SRC = '/home/jetson/jetcobot_ws/src/jetcobot_utils/src'
GARBAGE_YOLO_SRC = '/home/jetson/jetcobot_ws/src/jetcobot_garbage_yolov11'
JETCOBOT_ADVANCE_SCRIPTS = '/home/jetson/jetcobot_ws/src/jetcobot_advance/scripts'


class BuiltinGarbageDetector:
    def __init__(self, model_path: str, conf_threshold: float) -> None:
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.model = YOLO(model_path)
        self.names = self.model.names
        self.colors = {}

    def garbage_run(self, image):
        frame = image.copy()
        results = self.model(frame, verbose=False, conf=self.conf_threshold)
        detect_msg = {}

        if results:
            for box in results[0].boxes:
                xywh = box.xywhn.view(-1).tolist()
                conf = float(box.conf.item())
                if conf < self.conf_threshold:
                    continue
                cls = int(box.cls.item())
                name = str(self.names[cls])

                if cls not in self.colors:
                    self.colors[cls] = (
                        int((37 * (cls + 1)) % 255),
                        int((97 * (cls + 3)) % 255),
                        int((157 * (cls + 5)) % 255),
                    )

                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                point_x = int(xywh[0] * 640)
                point_y = int(xywh[1] * 480)

                cv.circle(frame, (point_x, point_y), 5, (0, 0, 255), -1)
                cv.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), self.colors[cls], 2)
                label = f'{name} {conf:.2f}'
                cv.putText(frame, label, (xyxy[0], xyxy[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, self.colors[cls], 2)

                a = round(((point_x - 320) / 4000), 5)
                b = round(((480 - point_y) / 3000) * 0.7 + 0.15, 5)
                detect_msg[name] = (a, b)

        return frame, detect_msg


class MonoTrashDetector(Node):
    def __init__(self) -> None:
        super().__init__('mono_trash_detector')

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('camera_reopen_sec', 2.0)
        self.declare_parameter('publish_debug_view', False)
        self.declare_parameter('publish_debug_topic', True)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('wait_for_watch_pose_ready', True)
        self.declare_parameter(
            'model_path',
            '/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.pt',
        )

        self.enabled = True
        self.wait_for_watch_pose_ready = bool(self.get_parameter('wait_for_watch_pose_ready').value)
        self.watch_pose_ready = not self.wait_for_watch_pose_ready
        self.detection_started = False
        self.detector = None
        self.capture = None
        self.last_camera_reopen_time = 0.0

        self.target_pub = self.create_publisher(PoseStamped, '/mission/trash_pose', 10)
        self.label_pub = self.create_publisher(String, '/mission/trash_label', 10)
        self.debug_image_pub = self.create_publisher(CompressedImage, '/mission/debug_image/compressed', 10)

        self.create_subscription(Bool, '/arm/detection_enable', self._on_detection_enable, 10)
        ready_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Bool, '/arm/watch_pose_ready', self._on_watch_pose_ready, ready_qos)

        self.label_aliases = {
            'cardboard': 'cardboard',
            'glass': 'glass',
            'metal': 'metal',
            'organic': 'organic',
            'paper': 'paper',
            'plastic': 'plastic',
            'euai': 'plastic',
            'euaia': 'plastic',
        }

        self.create_timer(0.2, self._process_frame)
        if self.watch_pose_ready:
            self._start_detection()
        else:
            self.get_logger().info('Waiting for /arm/watch_pose_ready before starting camera detection.')

        self.get_logger().info('单目垃圾识别节点已启动，当前接入真实 YOLO 检测结果。')

    def _on_watch_pose_ready(self, msg: Bool) -> None:
        self.watch_pose_ready = bool(msg.data)
        if self.watch_pose_ready:
            self._start_detection()

    def _start_detection(self) -> None:
        if self.detection_started:
            return

        self._init_detector()
        self._init_camera()
        self.detection_started = True
        self.get_logger().info('Arm watch pose is ready. Camera detection started.')

    def _get_model_path(self) -> str:
        model_path = str(self.get_parameter('model_path').value).strip()
        if not model_path:
            raise FileNotFoundError('model_path 为空，请传入有效的 best.pt 路径。')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'模型文件不存在: {model_path}')
        return model_path

    def _init_detector(self) -> None:
        model_path = self._get_model_path()
        conf_threshold = float(self.get_parameter('confidence_threshold').value)

        for path in (JETCOBOT_UTILS_SRC, GARBAGE_YOLO_SRC, JETCOBOT_ADVANCE_SCRIPTS):
            if path not in sys.path:
                sys.path.append(path)

        try:
            os.environ['JETCOBOT_GARBAGE_MODEL_PATH'] = model_path
            os.environ['JETCOBOT_GARBAGE_CONF'] = str(conf_threshold)
            from garbage_identify import garbage_identify
            self.detector = garbage_identify()
            self.get_logger().info(
                f'已成功加载 garbage_identify 与模型: {model_path}, conf={conf_threshold:.2f}'
            )
            return
        except Exception as exc:
            self.get_logger().warning(
                f'外部 garbage_identify 加载失败，切换到内置 YOLO 检测器: {exc}'
            )

        try:
            self.detector = BuiltinGarbageDetector(model_path, conf_threshold)
            self.get_logger().info(
                f'已成功加载内置 YOLO 检测器，模型: {model_path}, conf={conf_threshold:.2f}'
            )
        except Exception as exc:
            self.detector = None
            self.get_logger().error(f'加载真实 YOLO 检测器失败: {exc}')

    def _camera_source_name(self, camera_source) -> str:
        if isinstance(camera_source, int):
            return f'/dev/video{camera_source}'
        return str(camera_source)

    def _init_camera(self) -> None:
        self._release_camera()
        camera_device = str(self.get_parameter('camera_device').value).strip()
        if camera_device:
            camera_source = camera_device
        else:
            camera_source = int(self.get_parameter('camera_index').value)
        camera_name = self._camera_source_name(camera_source)
        self.capture = cv.VideoCapture(camera_source, cv.CAP_V4L2)
        self.capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.capture.isOpened():
            self.get_logger().error(f'Cannot open camera {camera_name}')
            self._open_fallback_camera()
            return
        self.get_logger().info(f'Opened camera {camera_name}')

    def _open_fallback_camera(self) -> None:
        camera_device = str(self.get_parameter('camera_device').value).strip()
        if camera_device:
            preferred = camera_device
        else:
            preferred = f"/dev/video{int(self.get_parameter('camera_index').value)}"
        preferred_real = os.path.realpath(preferred)
        self._release_camera()

        def video_sort_key(path: str) -> int:
            match = re.search(r'video(\d+)$', path)
            return int(match.group(1)) if match else 999

        video_paths = [
            f'/dev/{name}'
            for name in os.listdir('/dev')
            if re.fullmatch(r'video\d+', name)
        ]
        for path in sorted(video_paths, key=video_sort_key):
            if path == preferred or os.path.realpath(path) == preferred_real:
                continue

            capture = cv.VideoCapture(path, cv.CAP_V4L2)
            capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
            if capture.isOpened():
                self.capture = capture
                self.get_logger().warning(f'{preferred} unavailable, switched camera to {path}')
                return

            try:
                capture.release()
            except Exception:
                pass

        self.get_logger().error(f'No fallback /dev/video* camera could be opened after {preferred} failed')

    def _release_camera(self) -> None:
        if self.capture is None:
            return

        try:
            self.capture.release()
        except Exception:
            pass
        self.capture = None

    def _try_reopen_camera(self) -> None:
        now = time.monotonic()
        reopen_sec = float(self.get_parameter('camera_reopen_sec').value)
        if now - self.last_camera_reopen_time < reopen_sec:
            return

        self.last_camera_reopen_time = now
        self.get_logger().warning('摄像头不可用，尝试重新打开。')
        self._init_camera()

    def _on_detection_enable(self, msg: Bool) -> None:
        self.enabled = msg.data

    def _normalize_label(self, raw_label: str) -> str:
        label_text = str(raw_label or '').strip()
        if not label_text:
            return 'unknown'

        alias_key = label_text.lower()
        if alias_key in self.label_aliases:
            return self.label_aliases[alias_key]

        try:
            repaired = label_text.encode('latin1').decode('gbk')
            repaired_key = repaired.strip().lower()
            if repaired_key in self.label_aliases:
                return self.label_aliases[repaired_key]
            return repaired.strip().lower()
        except Exception:
            return label_text.lower()

    def _publish_debug_image(self, image) -> None:
        if not bool(self.get_parameter('publish_debug_topic').value):
            return

        ok, encoded = cv.imencode('.jpg', image)
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter('frame_id').value)
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.debug_image_pub.publish(msg)

    def _process_frame(self) -> None:
        if not self.detection_started:
            return
        if self.detector is None:
            return
        if self.capture is None or not self.capture.isOpened():
            self._try_reopen_camera()
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._release_camera()
            self._try_reopen_camera()
            self.get_logger().warning('摄像头读取失败。')
            return

        if not self.enabled:
            self._publish_debug_image(frame)
            if bool(self.get_parameter('publish_debug_view').value):
                cv.imshow('mono_trash_detector', frame)
                cv.waitKey(1)
            return

        try:
            debug_image, detect_msg = self.detector.garbage_run(frame)
        except Exception as exc:
            self.get_logger().warning(f'YOLO 检测执行失败: {exc}')
            return

        self._publish_debug_image(debug_image)

        if bool(self.get_parameter('publish_debug_view').value):
            cv.imshow('mono_trash_detector', debug_image)
            cv.waitKey(1)

        if not detect_msg:
            return

        first_name, first_pos = next(iter(detect_msg.items()))
        normalized_label = self._normalize_label(first_name)
        pose = self._convert_detect_msg_to_pose(first_pos)

        label = String()
        label.data = normalized_label

        self.label_pub.publish(label)
        self.target_pub.publish(pose)
        self.get_logger().info(
            f'发布垃圾目标: raw_label={first_name}, label={normalized_label}, '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
        )

    def _convert_detect_msg_to_pose(self, pos) -> PoseStamped:
        a = float(pos[0])
        b = float(pos[1])

        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter('frame_id').value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = b
        pose.pose.position.y = -a
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(float(self.get_parameter('target_yaw').value))
        return pose

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quat = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def destroy_node(self):
        self._release_camera()
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = MonoTrashDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
