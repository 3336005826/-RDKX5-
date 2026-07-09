# arm_mission_system

这个包运行在机械臂主控上，负责：

- 单目 YOLO 垃圾识别
- 抓取目标接收
- 机械臂抓取执行
- 抓取高度参数管理

## 当前方案

你选的是方案 A，也就是混合架构：

- `mono_trash_detector.py` 继续保留 Python
- `grasp_executor_cpp` 改成 C++，负责 ROS 通信、自动触发、状态管理
- `grasp_hw_bridge.py` 保留 Python，专门调用现有 `GraspController` 真机抓取

这样做的好处是：

- 不会破坏你现在已经能用的底层机械臂抓取库
- 任务调度和通信切到 C++，运行更稳一些
- 后面如果你还要继续提速，可以再把底层串口控制单独改成 C++

## 当前 YOLO 类别

- `cardboard`
- `glass`
- `metal`
- `organic`
- `paper`
- `plastic`

## 抓取高度配置

抓取高度在 [grasp_profiles.yaml](/F:/机械臂小车/src_机械臂/机械臂/arm_mission_system/config/grasp_profiles.yaml)：

- `approach_z`
- `grasp_z`

现在默认：

- `drop_mode: single_bag`
- `drop_label: Old_school_bag`

这是为了兼容旧版 `GraspController` 的投放位命名。

## 一键启动命令

### 1. 和小车正式联调

```bash
ros2 launch arm_mission_system arm_bringup.launch.py model_path:=/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.pt camera_index:=0 publish_debug_view:=true confidence_threshold:=0.8 auto_pick_from_detector:=false use_cpp_executor:=true
```

这条命令会启动：

- `mono_trash_detector.py`
- `grasp_hw_bridge.py`
- `grasp_executor_cpp`

### 2. 单独调机械臂，识别到就抓

```bash
ros2 launch arm_mission_system arm_bringup.launch.py model_path:=/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.pt camera_index:=0 publish_debug_view:=true confidence_threshold:=0.8 auto_pick_from_detector:=true use_cpp_executor:=true
```

## 单独抓取调试

先启动硬件桥：

```bash
ros2 run arm_mission_system grasp_hw_bridge.py
```

再启动 C++ 抓取执行器：

```bash
ros2 run arm_mission_system grasp_executor_cpp --ros-args --params-file /home/jetson/jetcobot_ws/install/arm_mission_system/share/arm_mission_system/config/grasp_profiles.yaml -p auto_pick_from_detector:=false
```

然后发送测试抓取目标：

```bash
ros2 run arm_mission_system debug_grasp_garbage.py --ros-args -p x:=0.55 -p y:=0.00 -p label:=plastic
```

## 实时视觉调试

识别节点会发布调试图像：

```bash
/mission/debug_image/compressed
```

查看方式：

```bash
ros2 run rqt_image_view rqt_image_view
```

然后选择 `/mission/debug_image/compressed`。

## 兼容回退

如果你临时还想用原来的 Python 抓取执行器：

```bash
ros2 launch arm_mission_system arm_bringup.launch.py use_cpp_executor:=false
```
