import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('arm_mission_system'),
        'config',
        'grasp_profiles.yaml',
    )

    model_path = LaunchConfiguration('model_path')
    camera_index = LaunchConfiguration('camera_index')
    camera_device = LaunchConfiguration('camera_device')
    camera_reopen_sec = LaunchConfiguration('camera_reopen_sec')
    publish_debug_view = LaunchConfiguration('publish_debug_view')
    publish_debug_topic = LaunchConfiguration('publish_debug_topic')
    auto_pick_from_detector = LaunchConfiguration('auto_pick_from_detector')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    use_cpp_executor = LaunchConfiguration('use_cpp_executor')
    min_pick_interval_sec = LaunchConfiguration('min_pick_interval_sec')
    min_target_shift_m = LaunchConfiguration('min_target_shift_m')
    drop_mode = LaunchConfiguration('drop_mode')
    drop_label = LaunchConfiguration('drop_label')
    enable_base_assist = LaunchConfiguration('enable_base_assist')
    wait_for_watch_pose_ready = LaunchConfiguration('wait_for_watch_pose_ready')
    joint1 = LaunchConfiguration('joint1')
    joint2 = LaunchConfiguration('joint2')
    joint3 = LaunchConfiguration('joint3')
    joint4 = LaunchConfiguration('joint4')
    joint5 = LaunchConfiguration('joint5')
    joint6 = LaunchConfiguration('joint6')
    watch_joint1 = LaunchConfiguration('watch_joint1')
    watch_joint2 = LaunchConfiguration('watch_joint2')
    watch_joint3 = LaunchConfiguration('watch_joint3')
    watch_joint4 = LaunchConfiguration('watch_joint4')
    watch_joint5 = LaunchConfiguration('watch_joint5')
    watch_joint6 = LaunchConfiguration('watch_joint6')
    grasp_target_swap_xy = LaunchConfiguration('grasp_target_swap_xy')
    grasp_target_x_sign = LaunchConfiguration('grasp_target_x_sign')
    grasp_target_y_sign = LaunchConfiguration('grasp_target_y_sign')
    grasp_target_x_offset = LaunchConfiguration('grasp_target_x_offset')
    grasp_target_y_offset = LaunchConfiguration('grasp_target_y_offset')
    use_bridge_grasp_pose = LaunchConfiguration('use_bridge_grasp_pose')
    grasp_rx = LaunchConfiguration('grasp_rx')
    grasp_ry = LaunchConfiguration('grasp_ry')
    grasp_rz = LaunchConfiguration('grasp_rz')
    use_custom_drop_joints = LaunchConfiguration('use_custom_drop_joints')
    drop_joint1 = LaunchConfiguration('drop_joint1')
    drop_joint2 = LaunchConfiguration('drop_joint2')
    drop_joint3 = LaunchConfiguration('drop_joint3')
    drop_joint4 = LaunchConfiguration('drop_joint4')
    drop_joint5 = LaunchConfiguration('drop_joint5')
    drop_joint6 = LaunchConfiguration('drop_joint6')
    drop_settle_sec = LaunchConfiguration('drop_settle_sec')
    return_after_drop = LaunchConfiguration('return_after_drop')
    default_approach_z = LaunchConfiguration('default_approach_z')
    default_grasp_z = LaunchConfiguration('default_grasp_z')
    plastic_approach_z = LaunchConfiguration('plastic_approach_z')
    plastic_grasp_z = LaunchConfiguration('plastic_grasp_z')

    detector = Node(
        package='arm_mission_system',
        executable='mono_trash_detector.py',
        name='mono_trash_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'camera_index': camera_index,
            'camera_device': camera_device,
            'camera_reopen_sec': camera_reopen_sec,
            'publish_debug_view': publish_debug_view,
            'publish_debug_topic': publish_debug_topic,
            'confidence_threshold': confidence_threshold,
            'wait_for_watch_pose_ready': wait_for_watch_pose_ready,
        }],
    )

    executor_cpp = Node(
        package='arm_mission_system',
        executable='grasp_executor_cpp',
        name='grasp_executor',
        output='screen',
        condition=IfCondition(use_cpp_executor),
        parameters=[
            config_file,
            {
                'auto_pick_from_detector': auto_pick_from_detector,
                'min_pick_interval_sec': min_pick_interval_sec,
                'min_target_shift_m': min_target_shift_m,
                'drop_mode': drop_mode,
                'drop_label': drop_label,
                'joint1': joint1,
                'joint2': joint2,
                'joint3': joint3,
                'joint4': joint4,
                'joint5': joint5,
                'joint6': joint6,
                'default_profile.approach_z': default_approach_z,
                'default_profile.grasp_z': default_grasp_z,
                'grasp_profiles.plastic.approach_z': plastic_approach_z,
                'grasp_profiles.plastic.grasp_z': plastic_grasp_z,
            },
        ],
    )

    hw_bridge = Node(
        package='arm_mission_system',
        executable='grasp_hw_bridge.py',
        name='grasp_hw_bridge',
        output='screen',
        condition=IfCondition(use_cpp_executor),
        parameters=[{
            'watch_joint1': watch_joint1,
            'watch_joint2': watch_joint2,
            'watch_joint3': watch_joint3,
            'watch_joint4': watch_joint4,
            'watch_joint5': watch_joint5,
            'watch_joint6': watch_joint6,
            'grasp_target_swap_xy': grasp_target_swap_xy,
            'grasp_target_x_sign': grasp_target_x_sign,
            'grasp_target_y_sign': grasp_target_y_sign,
            'grasp_target_x_offset': grasp_target_x_offset,
            'grasp_target_y_offset': grasp_target_y_offset,
            'use_bridge_grasp_pose': use_bridge_grasp_pose,
            'grasp_rx': grasp_rx,
            'grasp_ry': grasp_ry,
            'grasp_rz': grasp_rz,
            'use_custom_drop_joints': use_custom_drop_joints,
            'drop_joint1': drop_joint1,
            'drop_joint2': drop_joint2,
            'drop_joint3': drop_joint3,
            'drop_joint4': drop_joint4,
            'drop_joint5': drop_joint5,
            'drop_joint6': drop_joint6,
            'drop_settle_sec': drop_settle_sec,
            'return_after_drop': return_after_drop,
        }],
    )

    base_assistant = Node(
        package='arm_mission_system',
        executable='base_pick_assistant.py',
        name='base_pick_assistant',
        output='screen',
        condition=IfCondition(enable_base_assist),
    )

    executor_python = Node(
        package='arm_mission_system',
        executable='grasp_executor.py',
        name='grasp_executor',
        output='screen',
        condition=UnlessCondition(use_cpp_executor),
        parameters=[
            config_file,
            {
                'auto_pick_from_detector': auto_pick_from_detector,
                'min_pick_interval_sec': min_pick_interval_sec,
                'min_target_shift_m': min_target_shift_m,
                'drop_mode': drop_mode,
                'drop_label': drop_label,
                'joint1': joint1,
                'joint2': joint2,
                'joint3': joint3,
                'joint4': joint4,
                'joint5': joint5,
                'joint6': joint6,
                'default_approach_z': default_approach_z,
                'default_grasp_z': default_grasp_z,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/jetson/jetcobot_ws/src/jetcobot_grasp/Dataset/runs/detect/train/weights/best.pt',
        ),
        DeclareLaunchArgument('camera_index', default_value='0'),
        DeclareLaunchArgument('camera_device', default_value=''),
        DeclareLaunchArgument('camera_reopen_sec', default_value='2.0'),
        DeclareLaunchArgument('publish_debug_view', default_value='false'),
        DeclareLaunchArgument('publish_debug_topic', default_value='true'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.8'),
        DeclareLaunchArgument(
            'auto_pick_from_detector',
            default_value='false',
        ),
        DeclareLaunchArgument('min_pick_interval_sec', default_value='8.0'),
        DeclareLaunchArgument('min_target_shift_m', default_value='0.0'),
        DeclareLaunchArgument('drop_mode', default_value='single_bag'),
        DeclareLaunchArgument('drop_label', default_value='Old_school_bag'),
        DeclareLaunchArgument('enable_base_assist', default_value='true'),
        DeclareLaunchArgument('wait_for_watch_pose_ready', default_value='true'),
        DeclareLaunchArgument('joint1', default_value='-90'),
        DeclareLaunchArgument('joint2', default_value='0'),
        DeclareLaunchArgument('joint3', default_value='0'),
        DeclareLaunchArgument('joint4', default_value='-83'),
        DeclareLaunchArgument('joint5', default_value='-6'),
        DeclareLaunchArgument('joint6', default_value='-1'),
        DeclareLaunchArgument('watch_joint1', default_value='-90'),
        DeclareLaunchArgument('watch_joint2', default_value='0'),
        DeclareLaunchArgument('watch_joint3', default_value='0'),
        DeclareLaunchArgument('watch_joint4', default_value='-83'),
        DeclareLaunchArgument('watch_joint5', default_value='-6'),
        DeclareLaunchArgument('watch_joint6', default_value='-1'),
        DeclareLaunchArgument('grasp_target_swap_xy', default_value='false'),
        DeclareLaunchArgument('grasp_target_x_sign', default_value='1.0'),
        DeclareLaunchArgument('grasp_target_y_sign', default_value='1.0'),
        DeclareLaunchArgument('grasp_target_x_offset', default_value='0.0'),
        DeclareLaunchArgument('grasp_target_y_offset', default_value='0.0'),
        DeclareLaunchArgument('use_bridge_grasp_pose', default_value='true'),
        DeclareLaunchArgument('grasp_rx', default_value='-175.0'),
        DeclareLaunchArgument('grasp_ry', default_value='0.0'),
        DeclareLaunchArgument('grasp_rz', default_value='-45.0'),
        DeclareLaunchArgument('use_custom_drop_joints', default_value='false'),
        DeclareLaunchArgument('drop_joint1', default_value='-90'),
        DeclareLaunchArgument('drop_joint2', default_value='0'),
        DeclareLaunchArgument('drop_joint3', default_value='0'),
        DeclareLaunchArgument('drop_joint4', default_value='-60'),
        DeclareLaunchArgument('drop_joint5', default_value='0'),
        DeclareLaunchArgument('drop_joint6', default_value='-45'),
        DeclareLaunchArgument('drop_settle_sec', default_value='1.0'),
        DeclareLaunchArgument('return_after_drop', default_value='true'),
        DeclareLaunchArgument('default_approach_z', default_value='170'),
        DeclareLaunchArgument('default_grasp_z', default_value='115'),
        DeclareLaunchArgument('plastic_approach_z', default_value='175'),
        DeclareLaunchArgument('plastic_grasp_z', default_value='120'),
        DeclareLaunchArgument(
            'use_cpp_executor',
            default_value='true',
        ),
        detector,
        base_assistant,
        hw_bridge,
        executor_cpp,
        executor_python,
    ])
