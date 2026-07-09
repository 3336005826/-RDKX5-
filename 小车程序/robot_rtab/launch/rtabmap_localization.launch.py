from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    enable_depth_assist = LaunchConfiguration('enable_depth_assist')

    depth_remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('odom', '/odom_combined'),
        ('scan', '/scan'),
        ('imu', '/imu/data_filtered'),
    ]
    scan_remappings = [
        ('odom', '/odom_combined'),
        ('scan', '/scan'),
        ('imu', '/imu/data_filtered'),
    ]
    mapping_with_depth = IfCondition(PythonExpression([
        "'",
        localization,
        "'.lower() != 'true' and '",
        enable_depth_assist,
        "'.lower() == 'true'",
    ]))
    mapping_scan_only = IfCondition(PythonExpression([
        "'",
        localization,
        "'.lower() != 'true' and '",
        enable_depth_assist,
        "'.lower() != 'true'",
    ]))
    localization_with_depth = IfCondition(PythonExpression([
        "'",
        localization,
        "'.lower() == 'true' and '",
        enable_depth_assist,
        "'.lower() == 'true'",
    ]))
    localization_scan_only = IfCondition(PythonExpression([
        "'",
        localization,
        "'.lower() == 'true' and '",
        enable_depth_assist,
        "'.lower() != 'true'",
    ]))
    rgbd_sync = Node(
        condition=IfCondition(enable_depth_assist),
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.05,
            'topic_queue_size': 25,
            'sync_queue_size': 25,
            'use_sim_time': use_sim_time,
            'qos': qos,
            'qos_camera_info': qos,
        }],
        remappings=depth_remappings[:-2],
    )

    base_params = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom_combined',
        'map_frame_id': 'map',
        'use_sim_time': use_sim_time,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan': True,
        'subscribe_imu': True,
        'use_action_for_goal': True,
        'odom_sensor_sync': False,
        'wait_for_transform': 0.35,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_camera_info': qos,
        'qos_imu': qos,
        'publish_tf': True,
        'database_path': database_path,
        'topic_queue_size': 25,
        'sync_queue_size': 25,
        'approx_sync': True,
        'wait_imu_to_init': True,
        'Rtabmap/DetectionRate': '5.0',
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'Rtabmap/TimeThr': '600',
        'Mem/ImagePreDecimation': '1',
        'Grid/DepthDecimation': '1',
        'Grid/FlatObstacleDetected': 'true',
        'Grid/MaxObstacleHeight': '1.20',
        'Grid/NoiseFilteringRadius': '0.10',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/Scan2dUnknownSpaceFilled': 'true',
        'GridGlobal/FullUpdate': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityPathFilteringRadius': '0.5',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/LinearUpdate': '0.10',
        'RGBD/AngularUpdate': '0.06',
        'RGBD/MaxOdomCacheSize': '0',
        'RGBD/LocalLoopDetectionSpace': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '2',
        'Grid/RangeMin': '0.20',
        'Grid/RangeMax': '5.0',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.025',
        'RGBD/CreateOccupancyGrid': 'true',
        'Optimizer/GravitySigma': '0.25',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/UseOdomFeatures': 'false',
        'Mem/RehearsalSimilarity': '0.45',
        'Kp/MaxDepth': '5.0',
        'Vis/MaxDepth': '5.0',
        'Icp/VoxelSize': '0.04',
        'Icp/PointToPlaneMinComplexity': '0.04',
        'Icp/CorrespondenceRatio': '0.20',
        'Icp/MaxTranslation': '0.45',
        'Icp/MaxRotation': '0.45',
    }

    depth_base_params = {
        **base_params,
        'subscribe_rgbd': True,
        'Grid/Sensor': '2',
        'Grid/FromDepth': 'true',
    }

    scan_base_params = {
        **base_params,
        'subscribe_rgbd': False,
        'Grid/Sensor': '0',
        'Grid/FromDepth': 'false',
    }

    mapping_overrides = {
        'Rtabmap/DetectionRate': '4.0',
        'Kp/MaxFeatures': '1800',
        'Vis/MinInliers': '20',
        'Vis/InlierDistance': '0.08',
        'RGBD/LinearUpdate': '0.10',
        'RGBD/AngularUpdate': '0.08',
        'RGBD/LocalImmunizationRatio': '0.70',
        'publish_map': True,
        'Mem/IncrementalMemory': 'True',
        'Mem/InitWMWithAllNodes': 'False',
    }

    localization_overrides = {
        'subscribe_imu': True,
        'Rtabmap/DetectionRate': '6.0',
        'Rtabmap/TimeThr': '450',
        'Kp/MaxFeatures': '1500',
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.08',
        'Optimizer/GravitySigma': '0.30',
        'publish_map': False,
        'Mem/IncrementalMemory': 'False',
        'Mem/InitWMWithAllNodes': 'True',
    }

    mapping_params = {
        **depth_base_params,
        **mapping_overrides,
    }

    localization_params = {
        **depth_base_params,
        **localization_overrides,
    }

    mapping_scan_params = {
        **scan_base_params,
        **mapping_overrides,
    }

    localization_scan_params = {
        **scan_base_params,
        **localization_overrides,
    }

    rtabmap_mapping_depth_node = Node(
        condition=mapping_with_depth,
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[mapping_params],
        remappings=depth_remappings,
        arguments=['-d'],
    )

    rtabmap_mapping_scan_node = Node(
        condition=mapping_scan_only,
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[mapping_scan_params],
        remappings=scan_remappings,
        arguments=['-d'],
    )

    rtabmap_localization_depth_node = Node(
        condition=localization_with_depth,
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[localization_params],
        remappings=depth_remappings,
    )

    rtabmap_localization_scan_node = Node(
        condition=localization_scan_only,
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[localization_scan_params],
        remappings=scan_remappings,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qos', default_value='2'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('enable_depth_assist', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value='/home/sunrise/.ros/rtabmap.db',
            description='RTAB-Map database path. Database is kept on start.',
        ),
        rgbd_sync,
        rtabmap_mapping_depth_node,
        rtabmap_mapping_scan_node,
        rtabmap_localization_depth_node,
        rtabmap_localization_scan_node,
    ])
