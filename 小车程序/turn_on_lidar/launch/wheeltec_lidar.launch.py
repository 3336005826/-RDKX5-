import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def include_lidar_launch(context, *args, **kwargs):
    yaml_path = LaunchConfiguration('lidar_type_yaml').perform(context)
    cfg = load_yaml(Path(yaml_path))

    lidar_type = LaunchConfiguration('lidar_type').perform(context) or cfg['lidar_type']
    enable_high_precision = LaunchConfiguration('enable_high_precision').perform(context).lower() == 'true'

    print(f'lidar_type:{lidar_type}')
    actions = []

    if lidar_type.startswith('ls'):
        if lidar_type == 'lscx':
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config',
                'lslidar_cx.yaml',
            )
            cx_cfg = yaml.safe_load(template_yaml.read_text())['cx']['lslidar_driver_node']['ros__parameters']
            if cfg['lscx']['angle_disable_min'] != 0 and cfg['lscx']['angle_disable_max'] != 0:
                cx_cfg['angle_disable_min'] = cfg['lscx']['angle_disable_min']
                cx_cfg['angle_disable_max'] = cfg['lscx']['angle_disable_max']

            lidar_launch = GroupAction(
                actions=[
                    LifecycleNode(
                        package='lslidar_driver',
                        executable='lslidar_driver_node',
                        name='lslidar_driver_node',
                        namespace='cx',
                        parameters=[cx_cfg],
                        output='screen',
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('pointcloud_to_laserscan'),
                                'launch',
                                'pointcloud_to_laserscan_launch.py',
                            )
                        ),
                    ),
                ]
            )
        else:
            template_yaml = Path(
                get_package_share_directory('lslidar_driver'),
                'config',
                'lslidar_x10.yaml',
            )
            lidar_port = cfg['x10']['lidar_port']
            x10_cfg = yaml.safe_load(template_yaml.read_text())['x10']['lslidar_driver_node']['ros__parameters']

            if lidar_type.endswith('net'):
                x10_cfg['serial_port'] = ''
            elif lidar_type.endswith('uart'):
                x10_cfg['serial_port'] = lidar_port

            if lidar_type.startswith('ls_M10'):
                x10_cfg['lidar_model'] = 'M10P' if lidar_type.startswith('ls_M10P') else 'M10'
            elif lidar_type.startswith('ls_N10'):
                x10_cfg['lidar_model'] = 'N10Plus' if lidar_type.startswith('ls_N10Plus') else 'N10'

            if cfg['x10']['angle_disable_min'] != 0 and cfg['x10']['angle_disable_max'] != 0:
                x10_cfg['angle_disable_min'] = cfg['x10']['angle_disable_min']
                x10_cfg['angle_disable_max'] = cfg['x10']['angle_disable_max']

            if enable_high_precision:
                x10_cfg['use_high_precision'] = True

            directional_filter = cfg['x10'].get('directional_filter', {})
            filter_enabled = bool(directional_filter.get('enabled', False))
            raw_scan_topic = directional_filter.get('raw_topic', '/scan_raw')
            filtered_scan_topic = directional_filter.get('filtered_topic', '/scan')
            if filter_enabled:
                x10_cfg['laserscan_topic'] = raw_scan_topic

            lidar_node = LifecycleNode(
                package='lslidar_driver',
                executable='lslidar_driver_node',
                name='lslidar_driver_node',
                namespace='x10',
                parameters=[x10_cfg],
                output='screen',
            )
            if filter_enabled:
                lidar_launch = GroupAction(
                    actions=[
                        lidar_node,
                        Node(
                            package='turn_on_wheeltec_robot',
                            executable='directional_scan_filter.py',
                            name='directional_scan_filter',
                            output='screen',
                            parameters=[{
                                'input_topic': raw_scan_topic,
                                'output_topic': filtered_scan_topic,
                                'base_frame': directional_filter.get('base_frame', 'base_footprint'),
                                'sector_min_deg': directional_filter.get(
                                    'sector_min_deg',
                                    [30.0, 120.0, 210.0, 300.0],
                                ),
                                'sector_max_deg': directional_filter.get(
                                    'sector_max_deg',
                                    [60.0, 150.0, 240.0, 330.0],
                                ),
                                'front_m': directional_filter.get('front_m', 0.278),
                                'back_m': directional_filter.get('back_m', 0.278),
                                'left_m': directional_filter.get('left_m', 0.25),
                                'right_m': directional_filter.get('right_m', 0.25),
                                'padding_m': directional_filter.get('padding_m', 0.12),
                            }],
                        ),
                    ]
                )
            else:
                lidar_launch = lidar_node
    elif lidar_type == 'ldstl19p':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'), 'launch', 'stl19p.launch.py')
            ),
        )
    elif lidar_type == 'ldstl06nbj':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'), 'launch', 'stl06nbj.launch.py')
            ),
        )
    elif lidar_type == 'ldstl19n':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ldlidar'), 'launch', 'stl19n.launch.py')
            ),
        )
    elif lidar_type == 'rplidar_c1':
        lidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')
            ),
        )
    else:
        raise ValueError(f'Unsupported lidar: {lidar_type}')

    actions.append(lidar_launch)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type_yaml',
            default_value=os.path.join(
                get_package_share_directory('turn_on_wheeltec_robot'),
                'config',
                'wheeltec_param.yaml',
            ),
            description='Path to lidar_type.yaml',
        ),
        DeclareLaunchArgument(
            'lidar_type',
            default_value='',
            description='Which lidar model to launch',
        ),
        DeclareLaunchArgument(
            'enable_high_precision',
            default_value='false',
            description='Enable high precision LaserScan mode when supported by the lidar driver.',
        ),
        OpaqueFunction(function=include_lidar_launch),
    ])
