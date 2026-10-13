"""Bring up three independently addressed Orbbec Astra Pro cameras.

Only starts camera drivers and the front depth TF alias; existing obstacle
and AI processing launch files remain independent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    astra_pro_launch = os.path.join(
        get_package_share_directory('astra_camera'), 'launch', 'astra_pro.launch.xml'
    )

    def camera_driver(name, serial_arg, enabled_arg, color, depth, publish_tf):
        return IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_pro_launch),
            launch_arguments={
                'camera_name': name,
                'serial_number': LaunchConfiguration(serial_arg),
                'device_num': LaunchConfiguration('device_num'),
                'enable_color': 'true' if color else 'false',
                'enable_depth': 'true' if depth else 'false',
                'enable_ir': 'false',
                'enable_point_cloud': 'false',
                'enable_colored_point_cloud': 'false',
                'use_uvc_camera': 'true' if color else 'false',
                'publish_tf': 'true' if publish_tf else 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration(enabled_arg)),
        )

    # The URDF already publishes camera_depth_optical_frame and the rear optical
    # frame. Keep the depth drivers from publishing competing TF authorities.
    # Map the driver's front depth frame to the existing URDF optical frame.
    front_depth_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_astra_optical_alias',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_depth_optical_frame',
            '--child-frame-id', 'front_camera_depth_optical_frame',
        ],
        condition=IfCondition(LaunchConfiguration('enable_front')),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'ai_serial', default_value='17122611013',
            description='Serial of the forward-facing RGB/AI camera.',
        ),
        DeclareLaunchArgument(
            'front_serial', default_value='17110310855',
            description='Serial of the forward down-tilted depth camera.',
        ),
        DeclareLaunchArgument(
            'rear_serial', default_value='17122514440',
            description='Serial of the rear down-tilted depth camera.',
        ),
        DeclareLaunchArgument(
            'device_num', default_value='3',
            description='Number of physically attached Astra devices.',
        ),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        front_depth_tf_alias,
        camera_driver('front_camera', 'front_serial', 'enable_front',
                      color=False, depth=True, publish_tf=False),
        TimerAction(
            period=2.0,
            actions=[camera_driver('rear_camera', 'rear_serial', 'enable_rear',
                                   color=False, depth=True, publish_tf=False)],
        ),
        TimerAction(
            period=4.0,
            actions=[camera_driver('ai_camera', 'ai_serial', 'enable_ai',
                                   color=True, depth=False, publish_tf=True)],
        ),
    ])
