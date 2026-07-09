# Real-Time Trash Detection and Autonomous Recycling Robot Based on RDK X5 Edge Computing

This project implements an intelligent indoor trash recycling robot based on RDK X5, Jetson Orin Nano Super, ROS2 Humble, Nav2, RTAB-Map and YOLO/TensorRT. The robot integrates autonomous navigation, real-time trash detection, area coverage planning, round-trip patrol, robotic arm grasping and a PC control station.

## Overview

The mobile base runs navigation and mission management on RDK X5. The Jetson module runs trash detection and robotic arm grasping. The PC station provides mapping, navigation, task mode, shuttle mode, RViz workspace, lightweight map display and remote startup management. During navigation, the robot can pause when trash is detected, trigger the arm to pick and drop the object, then continue the mission.

## Features

- Real-time trash detection with YOLO/TensorRT.
- Autonomous mapping, localization and navigation with RTAB-Map and Nav2.
- Shuttle mode: record the current pose as the return origin, navigate to a target, pause for trash pickup on the route, and return to the origin.
- Area coverage planning from the PC station.
- Robotic arm grasping and dropping with configurable watch pose, grasp pose, grasp depth and drop joints.
- PC station with mapping mode, navigation mode, task mode, shuttle mode, embedded RViz, lightweight map and SSH remote launch.

## Hardware

- RDK X5 for mobile base navigation, mission management and ROS2 communication.
- Jetson Orin Nano Super for trash detection and robotic arm control.
- R550A PLUS 4WD ARM mobile base.
- Robotic arm for trash pickup and dropping.
- LiDAR for SLAM, localization and obstacle avoidance.
- Camera for trash detection and debug view.
- PC station for visualization and remote control.

## Software Architecture

```text
PC Station
  mobile_manipulator_station
    - mapping, navigation, task and shuttle modes
    - embedded RViz and lightweight map
    - remote launch for RDK X5 and Jetson nodes

RDK X5 Mobile Base
  car_mission_system
    - mission_manager task state machine
    - coverage_path_planner area coverage planner
    - station_map_relay map relay
  robot_rtab / Nav2 / RTAB-Map
    - mapping, localization, path planning and behavior trees

Jetson Arm Side
  arm_mission_system
    - mono_trash_detector trash detection
    - grasp_executor_cpp grasp executor
    - grasp_hw_bridge hardware control bridge
  jetcobot_utils / jetcobot_garbage_yolov11
    - arm control and trash detection adaptation
```

## Repository Structure

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

## Main Modules

### Arm Side

- `arm_mission_system`: ROS2 task package for trash detection integration, grasp execution, hardware bridge and grasp/drop configuration.
- `jetcobot_garbage_yolov11`: trash detection programs.
- `jetcobot_utils`: low-level arm movement, gripper and drop control.

### Mobile Base

- `car_mission_system`: mission state machine for navigation, home pose, coverage area, shuttle mode and arm coordination.
- `robot_rtab`: RTAB-Map mapping/localization, Nav2 integration and map relay.
- `turn_on_robot`: base and LiDAR parameter adaptation in the arm workspace.
- `turn_on_lidar`: LiDAR launch, directional scan filtering and base-related adaptation.
- `wheeltec_nav2`, `wheeltec_rtab`, `wheeltec_rrt2`: navigation parameters and behavior tree adaptation.

### PC Station

- `mobile_manipulator_station`: PC station for mapping, navigation, task mode, shuttle mode, embedded RViz, lightweight map and remote launch.

## Build

### RDK X5

```bash
cd ~/test_ws
colcon build --packages-select car_mission_system mobile_manipulator_station
source install/setup.bash
```

### Jetson

```bash
cd ~/jetcobot_ws
colcon build --packages-select arm_mission_system
source install/setup.bash
```

## Launch

### Mobile Base

```bash
ros2 launch car_mission_system car_bringup.launch.py localization:=true map:=/home/sunrise/test_ws/saved_maps/map.yaml
```

### Arm System

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

### PC Station

```bash
ros2 launch mobile_manipulator_station station_bringup.launch.py
```

## NodeHub Information

- Project name: Real-Time Trash Detection and Autonomous Recycling Robot Based on RDK X5 Edge Computing
- Short description: A ROS2 robot system based on RDK X5 and a robotic arm for trash detection, autonomous navigation, pickup and remote PC control.
- Platform: RDK X5
- Tags: RDK X5, ROS2, Nav2, RTAB-Map, YOLO, robotic arm, autonomous navigation, trash recycling

## Open Source Notice

This repository contains project-specific ROS2 task packages and the PC station software, as well as files adapted from vendor or third-party ROS2 packages. The rights of the original third-party and vendor packages belong to their respective authors.

## License

Apache-2.0 License is recommended for this project. See `LICENSE` for details.
