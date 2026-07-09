# car_mission_system

这个包运行在小车主控端，负责：

- 复用 `robot_rtab` 已验证过精度的导航链路
- 接收上位机发布的作业现场导航点
- 接收上位机框选的垃圾拾取区域
- 生成区域扫荡路径
- 执行扫荡任务
- 识别到垃圾后暂停底盘并把抓取目标转给机械臂
- 机械臂抓取完成后继续扫荡
- 支持任务完成后回到初始位置

## 当前任务流程

1. 上位机在 RViz 中点击导航目标点
2. 小车导航到作业现场
3. 上位机切换到区域框选模式，在 RViz 中点击两个对角点
4. `coverage_path_planner.py` 生成扫荡路径
5. `mission_manager.py` 驱动小车按路径扫荡
6. 识别到垃圾后，暂停当前扫荡并转发抓取目标给机械臂
7. 机械臂抓取完成后，继续从中断位置恢复扫荡
8. 任务完成后可自动或手动返回初始位置

## 主要文件

- `launch/car_nav.launch.py`
  - 只启动导航
  - 直接复用 `robot_rtab`

- `launch/car_bringup.launch.py`
  - 启动导航
  - 启动区域扫荡规划
  - 启动任务状态机

- `scripts/mission_manager.py`
  - 小车任务状态机
  - 管理导航到现场、等待区域框选、扫荡、暂停抓取、恢复扫荡、回家

- `scripts/coverage_path_planner.py`
  - 根据两个对角点自动生成割草机式扫荡路径

- `scripts/region_goal_sender.py`
  - 旧版测试用的区域点发布器

## 常用启动命令

### 只启导航

```bash
ros2 launch car_mission_system car_nav.launch.py localization:=true map:=/home/sunrise/test_ws/saved_maps/map.yaml
```

### 启动完整小车任务系统

```bash
ros2 launch car_mission_system car_bringup.launch.py localization:=true map:=/home/sunrise/test_ws/saved_maps/map.yaml
```

## 主要话题

- `/mission/nav_goal_pose`
- `/mission/home_pose`
- `/mission/region_point`
- `/mission/coverage_waypoints`
- `/mission/trash_pose`
- `/mission/arm_pick_target`
- `/mission/arm_busy`
- `/mission/state`
- `/mission/return_home`
