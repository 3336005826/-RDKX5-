#!/usr/bin/env python3
# coding: utf-8

import os
import sys
import time
import logging

import cv2 as cv
import jetcobot_utils.logger_config as logger_config
import numpy as np
from fps import FPS
from numpy import random
from ultralytics import YOLO


sys.path.append("/home/jetson/jetcobot_ws/src/jetcobot_advance/scripts")

DEFAULT_MODEL_CANDIDATES = [
    '/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.pt',
    '/home/jetson/jetcobot_ws/src/jetcobot_grasp/runs/detect/train/weights/best.pt',
]


def resolve_model_path() -> str:
    env_model_path = os.environ.get('JETCOBOT_GARBAGE_MODEL_PATH', '').strip()
    if env_model_path:
        if os.path.exists(env_model_path):
            return env_model_path
        raise FileNotFoundError(f'环境变量 JETCOBOT_GARBAGE_MODEL_PATH 指定的模型不存在: {env_model_path}')

    for candidate in DEFAULT_MODEL_CANDIDATES:
        if os.path.exists(candidate):
            return candidate

    raise FileNotFoundError(
        '未找到 best.pt，请检查以下路径之一是否存在: '
        + ', '.join(DEFAULT_MODEL_CANDIDATES)
    )


def resolve_conf_threshold() -> float:
    env_conf = os.environ.get('JETCOBOT_GARBAGE_CONF', '').strip()
    if not env_conf:
        return 0.8
    try:
        return float(env_conf)
    except Exception:
        return 0.8


MODEL_PATH = resolve_model_path()
MODEL = YOLO(MODEL_PATH)
CONF_THRESHOLD = resolve_conf_threshold()
NAMES = MODEL.names
COLORS = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(NAMES))]


class garbage_identify:
    def __init__(self):
        logger_config.setup_logger()
        logging.info('--start_program----------------------')
        logging.info(f'load model path: {MODEL_PATH}')
        logging.info(f'confidence threshold: {CONF_THRESHOLD}')
        self.fps = FPS()
        self.frame = None
        self.garbage_index = 0

    def garbage_run(self, image):
        self.frame = image
        msg = self.get_pos()
        self.fps.update()
        self.fps.show_fps(self.frame)
        return self.frame, msg

    def get_pos(self):
        try:
            prev_time = time.time()
            results = MODEL(self.frame, verbose=False, conf=CONF_THRESHOLD)
            msg = {}
            if results:
                for box in results[0].boxes:
                    xywh = box.xywhn.view(-1).tolist()
                    conf = box.conf.item()
                    if conf < CONF_THRESHOLD:
                        continue
                    cls = int(box.cls.item())
                    name = str(NAMES[cls])

                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    point_x = np.int32(xywh[0] * 640)
                    point_y = np.int32(xywh[1] * 480)

                    cv.circle(self.frame, (point_x, point_y), 5, (0, 0, 255), -1)
                    cv.rectangle(self.frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), COLORS[cls], 2)
                    label = '%s %.2f' % (name, conf)
                    cv.putText(self.frame, label, (xyxy[0], xyxy[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, COLORS[cls], 2)

                    a = round(((point_x - 320) / 4000), 5)
                    b = round(((480 - point_y) / 3000) * 0.7 + 0.15, 5)
                    msg[name] = (a, b)

            _ = time.time() - prev_time
            return msg
        except Exception as exc:
            print('error = ', exc)
        return None
