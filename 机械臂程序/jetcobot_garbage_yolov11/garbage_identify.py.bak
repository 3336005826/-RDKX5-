#!/usr/bin/env python3
# coding: utf-8

import sys

sys.path.append("/home/jetson/jetcobot_ws/src/jetcobot_advance/scripts")
model_path = '/home/jetson/ultralytics/ultralytics/data/yahboom_data/best.engine' 
import logging
import jetcobot_utils.logger_config as logger_config
from fps import FPS

import time
import torch
import cv2 as cv
import numpy as np
from numpy import random
from ultralytics import YOLO
model = YOLO(model_path)

# Get names and colors
names = model.names
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


class garbage_identify:
    def __init__(self):
        # 日志
        logger_config.setup_logger()
        logging.info("--start_program----------------------")
        # 帧率统计器
        self.fps = FPS()
        self.frame = None
        self.garbage_index=0

    def garbage_run(self, image):
        '''
        执行垃圾识别函数  Execute the garbage identification function
        :param image: 原始图像     The original image
        :return: 识别后的图像,识别信息(name, pos) Recognized image, identification information (name, pos)
        '''
        # self.frame = cv.resize(image, (640, 480))
        self.frame = image
        txt0 = 'Model-Loading...'
        msg= {}
        msg = self.get_pos()
        self.fps.update()
        self.fps.show_fps(self.frame)
        return self.frame, msg

    def get_pos(self):
        '''
        获取识别信息 Obtain identifying information
        :return: 名称,位置 name, location
        '''
        try:
            prev_time = time.time()
            results = model(self.frame, verbose=False)
            msg = {}
            if results:
                for box in results[0].boxes:
                    # 从 box 中获取归一化的 xywh 信息
                    xywh = box.xywhn.view(-1).tolist()
                    conf = box.conf.item()
                    cls = int(box.cls.item())
                    prediction_status = True
                    name = names[cls]
                    name_list = ["Vegetable_leaf", "Banana_peel", "Shell", "Plastic_bottle", "Basketball", "Carton", "Bandage", "Expired_capsule_drugs"]
                    for i in name_list:
                        if name == i:
                            prediction_status = False
                    if prediction_status:
                        xyxy = box.xyxy[0].cpu().numpy().astype(int)
                        point_x = np.int32(xywh[0] * 640)
                        point_y = np.int32(xywh[1] * 480)
                        cv.circle(self.frame, (point_x, point_y), 5, (0, 0, 255), -1)
                        # 绘制边界框
                        cv.rectangle(self.frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), colors[cls], 2)
                        label = '%s %.2f' % (names[cls], conf)
                        cv.putText(self.frame, label, (xyxy[0], xyxy[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, colors[cls], 2)
                        # 计算 (a, b) 并存储到 msg 字典
                        (a, b) = (round(((point_x - 320) / 4000), 5), round(((480 - point_y) / 3000) * 0.7 + 0.15, 5))
                        msg[name] = (a, b)
            # 计算时间
            curr_time = time.time()
            exec_time = curr_time - prev_time
            info = "time: %.2f ms" % (1000 * exec_time)
            return msg
        except Exception as e:
            print("error = ", e)
        return None
        