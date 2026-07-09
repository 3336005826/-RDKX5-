# mobile_manipulator_station

这是运行在 Windows 11 电脑上的机械臂小车上位机。

它不要求电脑本机安装 ROS2，而是通过下面两种方式控制机器人：

- `rosbridge_server`
  - 用于通过 WebSocket 连接 RDKX5 上的小车 ROS2 系统
- `SSH`
  - 用于从 Windows 一键启动 RDKX5 和 Jetson 上的命令

## 适用架构

- 上位机：Windows 11 电脑
- 小车主控：RDKX5，运行 ROS2 Humble
- 机械臂主控：Jetson，运行视觉识别和抓取程序

## 当前支持的功能

- 地图显示 `/map`
- 3D 机器人视窗（Qt/OpenGL，直接读取 URDF + STL mesh）
- 底盘实时位置显示 `/odom`
- 视觉调试画面显示 `/mission/debug_image/compressed`
- 地图单击设置导航点
- 地图单击设置回家点
- 地图双击两点框选垃圾作业区域
- 2D 初始位姿设置
- 远程保存地图
- 任务统计面板
- 抓取失败告警弹窗
- rosbridge 断连告警弹窗
- 一键启动建图模式 / 导航模式 / 任务模式
- 单独停止建图 / 导航 / 机械臂任务
- 全部停止，效果接近 Ctrl+C 结束当前由上位机拉起的远程任务

## Windows 依赖安装

先安装 Python 3.10 或 3.11，然后在 PowerShell 中执行：

```powershell
cd F:\机械臂小车\src_机械臂\上位机\mobile_manipulator_station
pip install -r windows_requirements.txt
```

如果是第一次启用 3D 机器人视窗，`windows_requirements.txt` 新增了：

- `PyOpenGL`
- `PyOpenGL-accelerate`

## 直接运行

```powershell
cd F:\机械臂小车\src_机械臂\上位机\mobile_manipulator_station\scripts
python station_gui.py
```

如果你之前已经装过一次依赖，加入 3D 视窗后请重新执行一次：

```powershell
cd F:\机械臂小车\src_机械臂\上位机\mobile_manipulator_station
pip install -r windows_requirements.txt
```

或者直接双击：

```text
scripts\start_station_windows.bat
```

## RDKX5 端准备

### 1. 启动 rosbridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. 导航模式命令

```bash
source /opt/ros/humble/setup.bash
source ~/test_ws/install/setup.bash
ros2 launch car_mission_system car_bringup.launch.py localization:=true map:=/home/sunrise/test_ws/saved_maps/map.yaml
```

### 3. 建图模式命令

```bash
source /opt/ros/humble/setup.bash
ros2 launch robot_rtab wheeltec_nav2_rtab.launch.py localization:=false
```

## Jetson 端命令

```bash
source /opt/ros/humble/setup.bash
source ~/jetcobot_ws/install/setup.bash
ros2 launch arm_mission_system arm_bringup.launch.py \
  model_path:=/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.engine \
  camera_index:=0 \
  publish_debug_view:=true \
  confidence_threshold:=0.8 \
  auto_pick_from_detector:=true \
  use_cpp_executor:=true \
  enable_base_assist:=true
```

## 配置文件

配置文件在：

```text
config/station_client.yaml
```

当前关键字段：

- `rosbridge_url`
  - 例如 `ws://192.168.1.142:9090`
- `rdk_host`
  - 例如 `sunrise@192.168.1.142`
- `jetson_host`
  - 机械臂 Jetson 的 SSH 地址，格式是 `用户名@IP或主机名`
  - 例如 `jetson@192.168.1.205` 或 `jetson@yahboom`
  - 如果只在上位机界面里填 `192.168.1.205`，保存时会自动补成 `jetson@192.168.1.205`
- `map_yaml`
  - 当前导航使用的远程地图 YAML
- `map_save_stem`
  - 保存地图时的目标路径前缀，不带 `.yaml`
- `robot_urdf_path`
  - Windows 本机用于 3D 机器人视窗显示的 URDF 路径

默认会优先尝试加载：

```text
F:\机械臂小车\src\wheeltec_robot_urdf\wheeltec_robot_urdf\urdf\R550A_PLUS_4wd_arm_robot.urdf
```

## 3D 视窗操作

- 左键拖动：旋转视角
- 右键拖动：平移视角
- 滚轮：缩放
- `3D复位`：恢复默认观察视角

例如：

```yaml
station_client:
  rosbridge_url: "ws://192.168.1.142:9090"
  rdk_host: "sunrise@192.168.1.142"
  rdk_workspace: "/home/sunrise/test_ws"
  jetson_host: "jetson@192.168.1.205"
  jetson_workspace: "/home/jetson/jetcobot_ws"
  auto_start_arm_remote: false
  map_yaml: "/home/sunrise/test_ws/saved_maps/map.yaml"
  map_save_stem: "/home/sunrise/test_ws/saved_maps/map"
```

## 地图交互说明

### 1. 2D 初始位姿

- 点击工具栏 `2D 初始位姿`
- 第一次点击：设置机器人初始位置
- 第二次点击：设置机器人朝向
- 程序会向 `/initialpose` 发布 `PoseWithCovarianceStamped`

### 2. 保存地图

- 在左侧填写 `保存地图目标`
- 点击 `保存地图`
- 上位机会通过 SSH 在 RDKX5 上执行：

```bash
ros2 run nav2_map_server map_saver_cli -f <保存路径前缀>
```

例如保存目标为：

```text
/home/sunrise/test_ws/saved_maps/map_0702
```

则会生成：

- `/home/sunrise/test_ws/saved_maps/map_0702.yaml`
- `/home/sunrise/test_ws/saved_maps/map_0702.pgm`

## Windows 打包成 exe

### 1. 安装打包工具

```powershell
pip install pyinstaller
```

### 2. 执行打包脚本

```powershell
cd F:\机械臂小车\src_机械臂\上位机\mobile_manipulator_station\scripts
.\build_station_exe.bat
```

### 3. 生成位置

生成后的 exe 默认在：

```text
scripts\dist\station_gui.exe
```

同时会把 `config\station_client.yaml` 一起复制到输出目录，方便后续直接改配置。

## 建议的完整使用顺序

1. 在 RDKX5 上启动 `rosbridge_server`
2. 打开 Windows 上位机
3. 确认 `rosbridge` 已连接
4. 点击 `建图模式` 或 `导航模式`
5. 使用 `2D 初始位姿` 设置定位初值
6. 设置回家点
7. 设置作业导航点
8. 框选垃圾作业区域
9. 点击 `任务模式`
10. 小车扫荡，机械臂识别并抓取
11. 扫荡结束后自动回家




#上位机启动命令
cd ~/test_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash ~/test_ws/src/mobile_manipulator_station/scripts/start_station_linux.sh ~/test_ws

## RDKX5 rosbridge 开机自启

在 RDKX5 上执行：

```bash
cd ~/test_ws/src/mobile_manipulator_station/scripts
chmod +x install_rdk_rosbridge_autostart.sh
./install_rdk_rosbridge_autostart.sh /home/sunrise/test_ws
```

安装后会创建：

```text
/etc/systemd/system/rosbridge-websocket.service
/usr/local/bin/start_rosbridge_websocket.sh
```

查看状态：

```bash
systemctl status rosbridge-websocket.service
```

实时看日志：

```bash
journalctl -u rosbridge-websocket.service -f
```

手动重启：

```bash
sudo systemctl restart rosbridge-websocket.service
```

停止并取消开机自启：

```bash
sudo systemctl disable --now rosbridge-websocket.service
```
