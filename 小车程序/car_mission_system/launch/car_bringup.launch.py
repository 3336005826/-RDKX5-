from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    localization = LaunchConfiguration('localization')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')
    enable_depth_assist = LaunchConfiguration('enable_depth_assist')
    enable_depth_pointcloud = LaunchConfiguration('enable_depth_pointcloud')
    enable_lidar_high_precision = LaunchConfiguration('enable_lidar_high_precision')

    share_dir = get_package_share_directory('car_mission_system')
    nav_launch = os.path.join(share_dir, 'launch', 'car_nav.launch.py')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch),
        launch_arguments={
            'localization': localization,
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'qos': qos,
            'database_path': database_path,
            'enable_depth_assist': enable_depth_assist,
            'enable_depth_pointcloud': enable_depth_pointcloud,
            'enable_lidar_high_precision': enable_lidar_high_precision,
        }.items(),
    )

    mission_manager = Node(
        package='car_mission_system',
        executable='mission_manager.py',
        name='mission_manager',
        output='screen',
    )

    coverage_planner = Node(
        package='car_mission_system',
        executable='coverage_path_planner.py',
        name='coverage_path_planner',
        output='screen',
    )

    station_map_relay = Node(
        package='car_mission_system',
        executable='station_map_relay.py',
        name='station_map_relay',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('map', default_value='/home/sunrise/test_ws/saved_maps/map.yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_rtab'),
            'params',
            'rtabmap_nav_params.yaml',
        )),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        DeclareLaunchArgument('database_path', default_value='/home/sunrise/.ros/rtabmap.db'),
        DeclareLaunchArgument('enable_depth_assist', default_value='false'),
        DeclareLaunchArgument('enable_depth_pointcloud', default_value='false'),
        DeclareLaunchArgument('enable_lidar_high_precision', default_value='true'),
        navigation,
        station_map_relay,
        coverage_planner,
        mission_manager,
    ])
