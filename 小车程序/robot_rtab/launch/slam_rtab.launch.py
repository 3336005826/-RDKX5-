# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python import get_package_share_directory

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')
#     qos = LaunchConfiguration('qos')
#     localization = LaunchConfiguration('localization')

#     bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

#     # 包含硬件驱动
#     wheeltec_robot = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')),
#     )
#     wheeltec_lidar = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
#     )
#     wheeltec_camera = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_camera.launch.py')),
#     )

#     parameters = {
#         'frame_id': 'camera_link',
#         'odom_frame_id': 'odom_combined',
#         'map_frame_id': 'map',
#         'use_sim_time': use_sim_time,
#         'subscribe_rgbd': True,
#         'subscribe_scan': True,
#         'use_action_for_goal': True,
#         'qos_scan': qos,
#         'qos_image': qos,
#         'qos_imu': qos,
#         'publish_tf': True,
#         'publish_map': True,
#         'odom_sensor_sync': False,
#         'wait_for_transform': 1.0,
#         'approx_sync': True,
#         'Reg/Strategy': '1',
#         'Reg/Force3DoF': 'true',
#         'RGBD/NeighborLinkRefining': 'True',
#         'Grid/RangeMin': '0.2',
#         'Optimizer/GravitySigma': '0',
#         'Icp/PointToPlaneMinComplexity': '0.04',
#     }

#     remappings = [
#         ('odom', '/odom_combined'),
#         ('scan', '/scan'),
#         ('rgb/image', '/camera/color/image_raw'),
#         ('rgb/camera_info', '/camera/color/camera_info'),
#         ('depth/image', '/camera/depth/image_raw'),
#     ]

#     return LaunchDescription([
#         wheeltec_robot,
#         wheeltec_lidar,
#         wheeltec_camera,

#         DeclareLaunchArgument('use_sim_time', default_value='false'),
#         DeclareLaunchArgument('qos', default_value='2'),
#         DeclareLaunchArgument('localization', default_value='false'),

#         Node(
#             package='rtabmap_sync', executable='rgbd_sync', output='screen',
#             parameters=[{
#                 'approx_sync': True,
#                 'approx_sync_max_interval': 0.05,
#                 'use_sim_time': use_sim_time,
#                 'qos': qos
#             }],
#             remappings=remappings
#         ),

#         Node(
#             condition=UnlessCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters],
#             remappings=remappings,
#             arguments=['-d']
#         ),

#         Node(
#             condition=IfCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters, {
#                 'Mem/IncrementalMemory': 'False',
#                 'Mem/InitWMWithAllNodes': 'True'
#             }],
#             remappings=remappings
#         ),
#     ])





# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch.conditions import IfCondition, UnlessCondition
# from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python import get_package_share_directory

# def generate_launch_description():

#     use_sim_time = LaunchConfiguration('use_sim_time')
#     qos = LaunchConfiguration('qos')
#     localization = LaunchConfiguration('localization')

#     bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

#     # 包含底层硬件驱动（底盘、雷达、相机已在 turn_on_wheeltec_robot 中根据参数启动）
#     wheeltec_robot = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'turn_on_wheeltec_robot.launch.py')),
#     )
#     wheeltec_lidar = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
#     )
#     # 注意：相机已由 turn_on_wheeltec_robot 启动，此处不再重复启动，避免 Resource busy
#     wheeltec_camera = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_camera.launch.py')),
#     )

#     # RTAB‑Map 参数
#     parameters = {
#         'frame_id': 'camera_link',               # 相机坐标系，需与 TF 树一致
#         'odom_frame_id': 'odom_combined',
#         'map_frame_id': 'map',
#         'use_sim_time': use_sim_time,
#         'subscribe_rgbd': True,
#         'subscribe_scan': True,
#         'use_action_for_goal': True,
#         'qos_scan': qos,
#         'qos_image': qos,
#         'qos_imu': qos,
#         'publish_tf': True,
#         'publish_map': True,
#         'odom_sensor_sync': False,               # 关闭时间戳严格同步，避免丢帧
#         'wait_for_transform': 0.2,               # TF 等待时间延长到 1 秒
#         'approx_sync': True,
#         'Reg/Strategy': '1',
#         'Reg/Force3DoF': 'true',
#         'RGBD/NeighborLinkRefining': 'True',
#         'Grid/RangeMin': '0.2',
#         'Optimizer/GravitySigma': '0',
#         'Icp/PointToPlaneMinComplexity': '0.04',
#     }

#     # 话题重映射
#     remappings = [
#         ('odom', '/odom_combined'),
#         ('scan', '/scan'),
#         ('rgb/image', '/camera/color/image_raw'),
#         ('rgb/camera_info', '/camera/color/camera_info'),
#         ('depth/image', '/camera/depth/image_raw'),
#     ]

#     return LaunchDescription([
#         # 硬件驱动
#         wheeltec_robot,
#         wheeltec_lidar,
#          wheeltec_camera,   

#         # 启动参数
#         DeclareLaunchArgument('use_sim_time', default_value='false',
#                               description='Use simulation (Gazebo) clock if true'),
#         DeclareLaunchArgument('qos', default_value='2',
#                               description='QoS for sensor topics (0=default,1=reliable,2=best effort)'),
#         DeclareLaunchArgument('localization', default_value='false',
#                               description='Run in localization mode (requires a prebuilt database)'),

#         # RGB‑D 同步节点
#         Node(
#             package='rtabmap_sync', executable='rgbd_sync', output='screen',
#             parameters=[{
#                 'approx_sync': True,
#                 'approx_sync_max_interval': 0.1,
#                 'use_sim_time': use_sim_time,
#                 'qos': qos
#             }],
#             remappings=remappings
#         ),

#         # SLAM 模式（默认）
#         Node(
#             condition=UnlessCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters],
#             remappings=remappings,
#             arguments=['-d']               # 删除旧数据库，每次全新建图
#         ),

#         # 纯定位模式（需要已有数据库）
#         Node(
#             condition=IfCondition(localization),
#             package='rtabmap_slam', executable='rtabmap', output='screen',
#             parameters=[parameters, {
#                 'Mem/IncrementalMemory': 'False',
#                 'Mem/InitWMWithAllNodes': 'True'
#             }],
#             remappings=remappings
#         ),
#     ])



import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', )
    qos = LaunchConfiguration('qos')
    Localization = LaunchConfiguration('Localization')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )
    # parameters={
    #       'frame_id':'camera_link', #wheeltec:camera_link
    #       'use_sim_time':use_sim_time,
    #       'subscribe_rgbd':True,
    #       'subscribe_scan':True,
    #       'use_action_for_goal':True,
    #       'qos_scan':qos,
    #       'qos_image':qos,
    #       'qos_imu':qos,
    #       # 高精度建图核心参数
    #       'Reg/Strategy':'1',
    #       'Reg/Force3DoF':'true',
    #       'RGBD/NeighborLinkRefining':'True',
    #       'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
    #       'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
    #       'Vis/MaxFeatures':'1500',  # 提高特征点上限
    #       'Vis/MinInliers':'25',     # 提高匹配质量阈值
    #       'Grid/CellSize':'0.025',   # 地图分辨率提升至2.5cm
    #       'Kp/LoopClosureReextractFeatures':'true',  # 回环检测时重新提取特征
    #       'Rtabmap/MaxRepublished':'1'  # 减少重新发布的节点数量
    # }

    parameters={
        'frame_id': 'camera_link',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'use_action_for_goal': True,
        'qos_scan': qos,
        'qos_image': qos,
        'qos_imu': qos,


        #RVIZ图像
        'Rtabmap/MapsUpdateRate': '0.2', 
        'Grid/MapFrameProjection': 'true',
        'Grid/FromDepth': 'true',
        
        # ==========================================
        # 1. 视觉特征提取升级 (消耗 CPU，提升特征质量)
        # ==========================================
        # 将特征提取器从默认的FAST/GFTT升级为ORB，特征更具描述性，抗旋转能力更强
        'Kp/DetectorStrategy': '10',  # 0=Feature2D(FAST), 3=GFTT, 10=ORB
        'Kp/MaxFeatures': '2000',     # 算力足，特征点上限从1500提至2000
        'ORB/ScaleFactor': '1.2',     # ORB金字塔缩放因子
        'ORB/NLevels': '8',           # ORB金字塔层数
        
        # ==========================================
        # 2. 视觉里程计与匹配优化 (提升局部精度)
        # ==========================================
        'Vis/MinInliers': '30',       # 进一步提高内点阈值，从25提至30，确保匹配极其可靠
        'Vis/InlierDistance': '0.05', # 匹配点距离阈值，单位米，0.05表示5cm，可根据实际场景微调
        'Reg/Strategy': '1',          # 使用ICP策略(激光+视觉)，这是高精度建图的核心
        'Reg/Force3DoF': 'true',      # 强制2D平面运动，防止Z轴漂移
        'Icp/MaxTranslation': '0.3',  # ICP最大平移，防止大跨度误匹配
        'Icp/VoxelSize': '0.05',      # ICP体素下采样，0.05m平衡精度与速度
        
        # ==========================================
        # 3. 全局优化与回环检测 (提升全局一致性)
        # ==========================================
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true', # 开启空间邻近检测，辅助回环
        'RGBD/OptimizeFromGraphEnd': 'false', # 保持false，避免坐标系突变
        'Mem/RehearsalSimilarity': '0.8', # 提高重听相似度阈值，减少错误回环
        'Kp/LoopClosureReextractFeatures': 'true',
        
        # ==========================================
        # 4. 地图构建参数 (提升地图细节)
        # ==========================================
        'Grid/CellSize': '0.02',      # 进一步提高栅格地图分辨率至2cm
        'Grid/RangeMin': '0.2',
        'Grid/RayTracing': 'true',    # 开启光线投射，清理动态障碍物留下的痕迹
        'Optimizer/GravitySigma': '0'
    }
    
    remappings=[
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar,wheeltec_camera,
        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('qos',default_value='2',description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('Localization', default_value='false', description='Launch in localization mode.'),        
        # Nodes to launch
        # Node(
        #     package='rtabmap_sync', executable='rgbd_sync', output='screen',
        #     parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.2, 'use_sim_time':use_sim_time, 'qos':qos}],  # 减小同步间隔
        #     remappings=remappings),
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync': True,
                'approx_sync_max_interval': 0.05,
                'use_sim_time': use_sim_time,
                'qos': qos
            }],
            remappings=remappings
        ),

            

        # Localization mode:
        Node(
            condition=IfCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),      
        # SLAM mode:
        Node(
            condition=UnlessCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
           
    ])