# 基于 RDK X5 边缘计算的实时垃圾检测与自主导航回收机器人

本项目面向室内垃圾巡检与回收场景，构建了一套由 RDK X5 移动底盘、Jetson Orin Nano Super、机械臂、深度/单目相机、激光雷达和 PC 上位机组成的智能垃圾回收机器人系统。系统基于 ROS2 Humble、Nav2、RTAB-Map 和 YOLO/TensorRT，实现垃圾实时识别、自主导航、区域清洁、往返跑巡检、机械臂抓取投放和上位机远程控制。

## 项目简介

机器人通过 RDK X5 运行小车导航和任务状态机，通过 Jetson Orin Nano Super 运行垃圾检测和机械臂抓取任务。上位机负责建图、导航、任务模式、往返跑模式、RViz 工作区、轻量地图显示和远程启动管理。机器人能够在设定区域内完成自主导航，当摄像头识别到垃圾后暂停移动，由机械臂完成抓取和投放，随后继续执行导航或返回原点。

## 主要功能

- 实时垃圾检测：基于 YOLO/TensorRT 对摄像头画面中的垃圾进行识别。
- 自主导航：基于 RTAB-Map、Nav2 和激光雷达实现建图、定位与导航。
- 往返跑模式：从当前位置记录原点，导航到目标点，途中遇到垃圾暂停抓取，完成后继续导航并返回原点。
- 清洁区域规划：上位机框选清洁区域，小车端生成覆盖路径。
- 机械臂抓取投放：支持观察位、抓取位、抓取深度、投放关节位等参数配置。
- 上位机控制：支持建图模式、导航模式、任务模式、往返跑模式、RViz 嵌入、轻量地图和远程 SSH 启动。

## 硬件组成

- RDK X5：小车导航、任务调度、地图转发和 ROS2 通信。
- Jetson Orin Nano Super：垃圾识别、机械臂抓取控制。
- 移动底盘：R550A PLUS 4WD ARM 车型。
- 机械臂：用于垃圾抓取和投放。
- 激光雷达：用于 SLAM、定位和避障。
- 摄像头：用于垃圾检测和调试画面发布。
- PC 上位机：用于可视化、模式切换和远程启动。

## 软件架构

```text
PC 上位机
  mobile_manipulator_station
    - 建图/导航/任务/往返跑模式
    - RViz 工作区和轻量地图
    - 远程启动 RDK X5 与 Jetson 节点

RDK X5 小车端
  car_mission_system
    - mission_manager 任务状态机
    - coverage_path_planner 区域路径规划
    - station_map_relay 地图转发
  robot_rtab / Nav2 / RTAB-Map
    - 建图、定位、路径规划、导航行为树

Jetson 机械臂端
  arm_mission_system
    - mono_trash_detector 垃圾检测
    - grasp_executor_cpp 抓取执行
    - grasp_hw_bridge 真实机械臂控制
  jetcobot_utils / jetcobot_garbage_yolov11
    - 机械臂底层控制和垃圾识别适配
```

## 目录结构

```text
.
├── README.md
├── README_cn.md
├── LICENSE
├── docs/
│   └── images/
│       └── nodehub_cover.png
├── 机械臂程序/
│   ├── arm_mission_system/
│   ├── jetcobot_garbage_yolov11/
│   └── jetcobot_utils/
├── 小车程序/
│   ├── car_mission_system/
│   ├── robot_rtab/
│   ├── turn_on_robot/
│   ├── turn_on_lidar/
│   ├── wheeltec_nav2/
│   ├── wheeltec_rtab/
│   ├── wheeltec_rrt2/
│   └── build_all.sh
└── 上位机程序/
    └── mobile_manipulator_station/
```

## 关键程序说明

### 机械臂程序

- `arm_mission_system`：本项目机械臂 ROS2 任务包，负责垃圾检测接入、抓取执行、硬件桥接和抓取参数配置。
- `jetcobot_garbage_yolov11`：机械臂垃圾识别程序。
- `jetcobot_utils`：机械臂底层抓取控制、夹爪控制和投放动作封装。

### 小车程序

- `car_mission_system`：小车任务状态机，负责导航、回家、清洁区域、往返跑和机械臂协同。
- `robot_rtab`：RTAB-Map 建图、定位、Nav2 集成和地图桥接。
- `turn_on_robot`：机械臂工作空间中的底盘/雷达参数适配。
- `turn_on_lidar`：雷达启动、雷达方向过滤和底盘相关适配。
- `wheeltec_nav2`、`wheeltec_rtab`、`wheeltec_rrt2`：导航参数和行为树适配。

### 上位机程序

- `mobile_manipulator_station`：PC 上位机，支持建图、导航、任务、往返跑、RViz 嵌入、轻量地图和远程启动。

## 编译方法

### RDK X5 小车端

```bash
cd ~/test_ws
colcon build --packages-select car_mission_system mobile_manipulator_station
source install/setup.bash
```

### Jetson 机械臂端

```bash
cd ~/jetcobot_ws
colcon build --packages-select arm_mission_system
source install/setup.bash
```

## 常用启动命令

### 小车端任务系统

```bash
ros2 launch car_mission_system car_bringup.launch.py localization:=true map:=/home/sunrise/test_ws/saved_maps/map.yaml
```

### 机械臂端任务系统

```bash
ros2 launch arm_mission_system arm_bringup.launch.py \
  model_path:=/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.engine \
  camera_device:=/dev/video1 \
  publish_debug_view:=true \
  publish_debug_topic:=true \
  confidence_threshold:=0.8 \
  auto_pick_from_detector:=true \
  use_cpp_executor:=true \
  enable_base_assist:=true \
  wait_for_watch_pose_ready:=true
```

### 上位机

```bash
ros2 launch mobile_manipulator_station station_bringup.launch.py
```

## NodeHub 项目信息建议

- 项目名称：基于 RDK X5 边缘计算的实时垃圾检测与自主导航回收机器人
- 项目简介：基于 RDK X5、ROS2 和机械臂，实现垃圾识别、自主导航、路径规划、抓取投放与上位机远程控制。
- 运行平台：RDK X5
- 推荐标签：RDK X5、ROS2、Nav2、RTAB-Map、YOLO、机械臂、智能垃圾回收、自主导航

## 开源说明

本仓库包含本项目自主开发的 ROS2 任务包和上位机程序，也包含基于厂家/第三方 ROS2 包进行二次开发与参数适配的文件。相关第三方依赖和厂家基础包的权利归原作者所有。

## License

本项目建议使用 Apache-2.0 License。详见 `LICENSE` 文件。
