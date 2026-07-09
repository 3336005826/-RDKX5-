# car_mission_system

这个包运行在小车主控上，负责以下功能：

- 复用 `robot_rtab` 导航链路
- 导航到目标区域
- 生成区域巡航路径
- 巡航过程中接收垃圾目标
- 触发机械臂侧抓取流程
- 任务结束后返航

当前默认话题：

- `/mission/home_pose`
- `/mission/region_pose`
- `/mission/coverage_waypoints`
- `/mission/trash_pose`
- `/mission/arm_pick_target`
- `/mission/arm_busy`
- `/mission/return_home`
- `/mission/state`
