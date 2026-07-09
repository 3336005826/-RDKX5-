#!/usr/bin/env python
# coding: utf-8

# # 垃圾分拣玩法  Garbage sorting play

# ### 导入头文件 Import head file

# In[ ]:


import torch
torch.__version__


# In[ ]:


#!/usr/bin/env python
# coding: utf-8

import cv2 as cv
import threading
import time
import ipywidgets as widgets
from IPython.display import display
from garbage_identify import garbage_identify
from jetcobot_utils.grasp_controller import GraspController
from jetcobot_utils.jetcobot_config import *


# ### 初始化机械臂位置 Initialize robot position

# In[ ]:


target = garbage_identify()
calibration = Arm_Calibration()
grasp_controller = GraspController()
grasp_controller.init_watch_pose()
joint1456 = [47, -76, -9, -1]
dp = []
msg = {}
threshold = 140
model = "General"
XYT_path="/home/jetson/jetcobot_ws/src/jetcobot_garbage_yolov11/XYT_config.txt"

try:    
    joint1456, threshold = read_XYT(XYT_path)
except Exception:
    print("Read XYT_config Error !!!")


# ### 创建控件 Creating widget

# In[ ]:


button_layout      = widgets.Layout(width='320px', height='60px', align_self='center')
output = widgets.Output()
# 调整滑杆 Adjust the slider
joint1_slider      = widgets.IntSlider(description='joint1 :'   ,    value=joint1456[0]     , min=0 , max=90, step=1, orientation='horizontal')
joint4_slider      = widgets.IntSlider(description='joint4 :'   ,    value=joint1456[1]     , min=-110, max=-50, step=1, orientation='horizontal')
joint5_slider      = widgets.IntSlider(description='joint5 :'   ,    value=joint1456[2]     , min=-30, max=30, step=1, orientation='horizontal')
joint6_slider      = widgets.IntSlider(description='joint6 :'   ,    value=joint1456[3]     , min=-30, max=60, step=1, orientation='horizontal')
threshold_slider   = widgets.IntSlider(description='threshold :',    value=threshold , min=0  , max=255, step=1, orientation='horizontal')

# 进入标定模式  Enter calibration mode
position_model     = widgets.Button(description='position_model',  button_style='primary', layout=button_layout)
calibration_model  = widgets.Button(description='calibration_model',  button_style='primary', layout=button_layout)
calibration_ok     = widgets.Button(description='calibration_ok',     button_style='success', layout=button_layout)
calibration_cancel = widgets.Button(description='calibration_cancel', button_style='danger', layout=button_layout)

# 目标检测抓取
target_detection   = widgets.Button(description='target_detection', button_style='info', layout=button_layout)
grap = widgets.Button(description='grap', button_style='success', layout=button_layout)
# 退出  exit
exit_button = widgets.Button(description='Exit', button_style='danger', layout=button_layout)
imgbox = widgets.Image(format='jpg', height=480, width=640, layout=widgets.Layout(align_self='center'))
garbage_identify = widgets.VBox(
    [joint1_slider, joint4_slider, joint5_slider, joint6_slider, threshold_slider, position_model, calibration_model, calibration_ok, calibration_cancel, target_detection, grap,exit_button],
    layout=widgets.Layout(align_self='center'));
controls_box = widgets.HBox([imgbox, garbage_identify], layout=widgets.Layout(align_self='center'))


# ### 标定回调 Calibration callback function

# In[ ]:


def position_model_Callback(value):
    global model
    model = 'Position'
    with output: print(model)
def calibration_model_Callback(value):
    global model
    model = 'Calibration'
    with output: print(model)
def calibration_OK_Callback(value):
    global model
    model = 'calibration_OK'
    with output: print(model)
def calibration_cancel_Callback(value):
    global model
    model = 'calibration_Cancel'
    with output: print(model)
position_model.on_click(position_model_Callback)
calibration_model.on_click(calibration_model_Callback)
calibration_ok.on_click(calibration_OK_Callback)
calibration_cancel.on_click(calibration_cancel_Callback)


# ### 模式切换   switching mode

# In[ ]:


def target_detection_Callback(value):
    global model
    model = 'Detection'
    with output: print(model)
def grap_Callback(value):
    global model
    model = 'Grap'
    with output: print(model)
def exit_button_Callback(value):
    global model
    model = 'Exit'
    with output: print(model)
target_detection.on_click(target_detection_Callback)
grap.on_click(grap_Callback)
exit_button.on_click(exit_button_Callback)


# ### 主程序 Main process

# 

# In[ ]:


def camera():
    global model, dp, msg
    capture = cv.VideoCapture(0)
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # Be executed in loop when the camera is opened normally
    sorting_start = False
    now = time.time()
    detect_msg = {}
    # 当摄像头正常打开的情况下循环执行
    while capture.isOpened():
        try:
            _, img = capture.read()
            # img = cv.resize(img, (640, 480))
            joint1456 = [joint1_slider.value, joint4_slider.value,
                         joint5_slider.value, joint6_slider.value]
            if model == 'Position':
                joints_angles = [joint1456[0], 0, 0,
                                 joint1456[1], joint1456[2], joint1456[3]]
                # 将机械臂移动到标定方框的状态
                grasp_controller.go_calibration_angles(joints_angles)
            if model == 'Calibration':
                joints_angles = [joint1456[0], 0, 0,
                                 joint1456[1], joint1456[2], joint1456[3]]
                # # 将机械臂移动到标定方框的状态
                grasp_controller.go_calibration_angles(joints_angles)
                _, img = calibration.calibration_map(
                    img, threshold_slider.value)
            if model == 'calibration_OK':
                try:
                    write_XYT(XYT_path, joint1456, threshold_slider.value)
                    print("File XYT_config Save OK")
                except Exception:
                    print("File XYT_config Error !!!")
                dp, img = calibration.calibration_map(
                    img, threshold_slider.value)
                model = "General"
            if len(dp) != 0:
                img = calibration.Perspective_transform(dp, img)
            if model == 'calibration_Cancel':
                dp = []
                model = "General"
            if model == 'Detection' and len(dp) != 0:
                img, msg = target.garbage_run(img)
                now = time.time()
                # print("msg:", msg)
                detect_msg = {}
            if model == 'Grap':
                img, msg = target.garbage_run(img)
                detect_msg.update(msg)
                if len(msg) > 0:
                    print("msg:", msg)
                if time.time() - now >= 2:
                    sorting_start = grasp_controller.grasp_state()
                    if len(detect_msg) != 0 and sorting_start == False:
                        print("detect count:", len(detect_msg))
                        print("detect msg:", detect_msg)
                        threading.Thread(target=grasp_controller.grasp_run,
                                     args=("sorting", "garbage", detect_msg, joint1456)).start()
                        sorting_start = True
                        model = 'Detection'

            if model == 'Exit':
                cv.destroyAllWindows()
                capture.release()
                break
            imgbox.value = cv.imencode('.jpg', img)[1].tobytes()
        except Exception as e:
            print("except program:", e)
    capture.release()
    print("stop program")


# ### 启动  Start

# In[ ]:


display(controls_box,output)
threading.Thread(target=camera, ).start()

