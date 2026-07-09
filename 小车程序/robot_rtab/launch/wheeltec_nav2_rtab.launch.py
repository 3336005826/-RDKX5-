import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)
def generate_launch_description():
    cfg_path = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config',
        'wheeltec_param.yaml',
    )
    cfg = load_yaml(cfg_path)
    print(f"car_mode: {cfg['car_mode']}")

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    localization = LaunchConfiguration('localization')
    qos = LaunchConfiguration('qos')
    database_path = LaunchConfiguration('database_path')
    enable_depth_assist = LaunchConfiguration('enable_depth_assist')
    enable_depth_pointcloud = LaunchConfiguration('enable_depth_pointcloud')
    enable_lidar_high_precision = LaunchConfiguration('enable_lidar_high_precision')
    enable_nav_in_mapping = LaunchConfiguration('enable_nav_in_mapping')

    depth_assist_with_nav = PythonExpression([
        "'",
        localization,
        "'.lower() == 'true' and '",
        enable_depth_assist,
        "'.lower() == 'true'",
    ])

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    robot_rtab_dir = get_package_share_directory('robot_rtab')
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')

    default_map_path = os.path.join(wheeltec_nav_dir, 'map', 'WHEELTEC.yaml')
    default_params_file = os.path.join(robot_rtab_dir, 'params', 'rtabmap_nav_params.yaml')
    localization_launch = os.path.join(robot_rtab_dir, 'launch', 'rtabmap_localization.launch.py')
    default_nav_to_pose_bt_xml = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees',
        'navigate_w_replanning_time.xml',
    )
    default_nav_through_poses_bt_xml = os.path.join(
        robot_rtab_dir,
        'behavior_trees',
        'navigate_through_poses_no_backup.xml',
    )

    wheeltec_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')
        ),
        launch_arguments={
            'carto_slam': 'false',
            'robot_nav': 'true',
        }.items(),
    )
    wheeltec_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')
        ),
        launch_arguments={
            'enable_high_precision': enable_lidar_high_precision,
        }.items(),
    )
    wheeltec_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'wheeltec_camera.launch.py')
        ),
        launch_arguments={
            'depth_registration': enable_depth_assist,
            'enable_point_cloud': enable_depth_pointcloud,
            'enable_colored_point_cloud': enable_depth_pointcloud,
            'enable_color': 'true',
            'enable_depth': enable_depth_assist,
            'point_cloud_qos': 'SENSOR_DATA',
            'publish_tf': 'true',
            'tf_publish_rate': '0.0',
            'enable_d2c_viewer': 'False',
            'enable_ir': 'false',
            'color_depth_synchronization': enable_depth_assist,
            'color_width': '640',
            'color_height': '480',
            'color_fps': '15',
            'depth_width': '640',
            'depth_height': '480',
            'depth_fps': '15',
            'connection_delay': '200',
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
            'color_camera_info_qos': 'SENSOR_DATA',
            'depth_camera_info_qos': 'SENSOR_DATA',
        }.items(),
    )
    imu_processor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'imu_processor.launch.py')
        ),
    )
    rtabmap_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'qos': qos,
            'localization': localization,
            'database_path': database_path,
            'enable_depth_assist': enable_depth_assist,
        }.items(),
    )
    mapping_mode_notice = LogInfo(
        condition=UnlessCondition(localization),
        msg='[robot_rtab] Mapping mode enabled: Nav2 is disabled. Use joystick or keyboard teleop for manual RTAB-Map mapping.',
    )
    navigation_mode_notice = LogInfo(
        condition=IfCondition(localization),
        msg='[robot_rtab] Navigation mode enabled: loading saved map for Nav2 navigation with RTAB-Map visual localization.',
    )
    deprecated_nav_mode_notice = LogInfo(
        condition=IfCondition(PythonExpression([
            "'",
            enable_nav_in_mapping,
            "'.lower() == 'true'",
        ])),
        msg='[robot_rtab] enable_nav_in_mapping is deprecated and ignored. Mapping mode now runs RTAB-Map only; use localization:=true for navigation.',
    )

    map_server_node = Node(
        condition=IfCondition(localization),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}],
    )
    rtabmap_map_bridge_node = Node(
        condition=UnlessCondition(localization),
        package='robot_rtab',
        executable='rtabmap_map_bridge.py',
        name='rtabmap_map_bridge',
        output='screen',
        parameters=[{
            'input_topics': ['/grid_map', '/rtabmap/grid_map'],
            'output_topic': '/map',
        }],
    )
    planner_server_node = Node(
        condition=IfCondition(localization),
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    controller_server_node = Node(
        condition=IfCondition(localization),
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    velocity_smoother_node = Node(
        condition=IfCondition(localization),
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
        ],
    )
    collision_monitor_node = Node(
        condition=IfCondition(localization),
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    behavior_server_node = Node(
        condition=IfCondition(localization),
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    bt_navigator_node = Node(
        condition=IfCondition(localization),
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
        }],
    )
    waypoint_follower_node = Node(
        condition=IfCondition(localization),
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )
    depth_scan_node = Node(
        condition=IfCondition(enable_depth_assist),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_scan_front',
        output='screen',
        parameters=[{
            'scan_time': 0.10,
            'range_min': 0.25,
            'range_max': 1.5,
            'scan_height': 10,
            'output_frame': 'camera_link',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_depth'),
        ],
    )
    depth_scan_obstacle_node = Node(
        condition=IfCondition(depth_assist_with_nav),
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_scan_obstacle',
        output='screen',
        parameters=[{
            'scan_time': 0.10,
            'range_min': 0.25,
            'range_max': 1.8,
            'scan_height': 18,
            'output_frame': 'camera_link',
        }],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan_depth_obstacle'),
        ],
    )
    lifecycle_manager_navigation_node = Node(
        condition=IfCondition(localization),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'velocity_smoother',
                'collision_monitor',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='true'))
    ld.add_action(DeclareLaunchArgument('localization', default_value='true'))
    ld.add_action(DeclareLaunchArgument('qos', default_value='2'))
    ld.add_action(DeclareLaunchArgument('map', default_value=default_map_path))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=default_params_file))
    ld.add_action(DeclareLaunchArgument('database_path', default_value='/home/sunrise/.ros/rtabmap.db'))
    ld.add_action(DeclareLaunchArgument('enable_depth_assist', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_depth_pointcloud', default_value='false'))
    ld.add_action(DeclareLaunchArgument('enable_lidar_high_precision', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'enable_nav_in_mapping',
        default_value='false',
        description='Deprecated and ignored. Mapping mode no longer launches Nav2.',
    ))

    ld.add_action(wheeltec_robot)
    ld.add_action(wheeltec_lidar)
    ld.add_action(wheeltec_camera)
    ld.add_action(imu_processor)
    ld.add_action(rtabmap_localization)
    ld.add_action(mapping_mode_notice)
    ld.add_action(navigation_mode_notice)
    ld.add_action(deprecated_nav_mode_notice)

    ld.add_action(map_server_node)
    ld.add_action(rtabmap_map_bridge_node)
    ld.add_action(planner_server_node)
    ld.add_action(controller_server_node)
    ld.add_action(velocity_smoother_node)
    ld.add_action(collision_monitor_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(waypoint_follower_node)
    ld.add_action(depth_scan_node)
    ld.add_action(depth_scan_obstacle_node)
    ld.add_action(lifecycle_manager_navigation_node)
    return ld
