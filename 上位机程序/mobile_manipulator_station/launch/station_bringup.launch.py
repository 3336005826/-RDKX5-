from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    local_workspace = LaunchConfiguration('local_workspace')
    remote_arm_host = LaunchConfiguration('remote_arm_host')
    remote_arm_workspace = LaunchConfiguration('remote_arm_workspace')
    remote_arm_autostart = LaunchConfiguration('remote_arm_autostart')
    start_in_terminal = LaunchConfiguration('start_in_terminal')

    station_gui = Node(
        package='mobile_manipulator_station',
        executable='station_gui.py',
        name='station_gui',
        output='screen',
        parameters=[{
            'local_workspace': local_workspace,
            'remote_arm_host': remote_arm_host,
            'remote_arm_workspace': remote_arm_workspace,
            'remote_arm_autostart': remote_arm_autostart,
            'start_in_terminal': start_in_terminal,
        }],
    )

    station_map_relay = Node(
        package='car_mission_system',
        executable='station_map_relay.py',
        name='station_map_relay',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('local_workspace', default_value='/home/sunrise/test_ws'),
        DeclareLaunchArgument('remote_arm_host', default_value='jetson@yahboom'),
        DeclareLaunchArgument('remote_arm_workspace', default_value='/home/jetson/jetcobot_ws'),
        DeclareLaunchArgument('remote_arm_autostart', default_value='false'),
        DeclareLaunchArgument('start_in_terminal', default_value='true'),
        station_map_relay,
        station_gui,
    ])
