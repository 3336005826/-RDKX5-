from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 直接复用已经验证过精度的 robot_rtab 导航链路
    localization = LaunchConfiguration('localization')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')
    enable_depth_assist = LaunchConfiguration('enable_depth_assist')
    enable_depth_pointcloud = LaunchConfiguration('enable_depth_pointcloud')
    enable_lidar_high_precision = LaunchConfiguration('enable_lidar_high_precision')

    robot_rtab_share = get_package_share_directory('robot_rtab')
    wrapped_launch = os.path.join(robot_rtab_share, 'launch', 'wheeltec_nav2_rtab.launch.py')
    default_params = os.path.join(robot_rtab_share, 'params', 'rtabmap_nav_params.yaml')
    default_map = '/home/sunrise/test_ws/saved_maps/map.yaml'

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(wrapped_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'map': map_yaml,
            'params_file': params_file,
            'qos': qos,
            'database_path': database_path,
            'enable_depth_assist': enable_depth_assist,
            'enable_depth_pointcloud': enable_depth_pointcloud,
            'enable_lidar_high_precision': enable_lidar_high_precision,
        }.items(),
    )

    note = LogInfo(
        msg=(
            '[car_mission_system] 当前导航直接复用 '
            'robot_rtab/wheeltec_nav2_rtab.launch.py，'
            '因此导航精度与原验证命令保持同源。'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        DeclareLaunchArgument('database_path', default_value='/home/sunrise/.ros/rtabmap.db'),
        DeclareLaunchArgument('enable_depth_assist', default_value='false'),
        DeclareLaunchArgument('enable_depth_pointcloud', default_value='false'),
        DeclareLaunchArgument('enable_lidar_high_precision', default_value='true'),
        note,
        nav_stack,
    ])
