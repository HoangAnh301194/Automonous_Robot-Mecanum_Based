"""Bench-only RGB/Depth person marker preview; no Nav2 goals are emitted."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    test_parent_frame_arg = DeclareLaunchArgument('test_parent_frame', default_value='base_footprint')
    return LaunchDescription([
        DeclareLaunchArgument('publish_test_tf', default_value='false'),
        test_parent_frame_arg,
        DeclareLaunchArgument('test_x', default_value='0.0'),
        DeclareLaunchArgument('test_y', default_value='0.0'),
        DeclareLaunchArgument('test_z', default_value='1.2'),
        DeclareLaunchArgument('test_roll', default_value='-1.57079632679'),
        DeclareLaunchArgument('test_pitch', default_value='0.0'),
        DeclareLaunchArgument('test_yaw', default_value='-1.57079632679'),
        DeclareLaunchArgument('depth_frame', default_value='ai_camera_depth_optical_frame'),
        DeclareLaunchArgument('base_frame', default_value=LaunchConfiguration('test_parent_frame')),
        DeclareLaunchArgument('map_frame', default_value='map'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ai_depth_test_tf',
            condition=IfCondition(LaunchConfiguration('publish_test_tf')),
            arguments=[
                '--x', LaunchConfiguration('test_x'),
                '--y', LaunchConfiguration('test_y'),
                '--z', LaunchConfiguration('test_z'),
                '--roll', LaunchConfiguration('test_roll'),
                '--pitch', LaunchConfiguration('test_pitch'),
                '--yaw', LaunchConfiguration('test_yaw'),
                '--frame-id', LaunchConfiguration('test_parent_frame'),
                '--child-frame-id', LaunchConfiguration('depth_frame'),
            ],
        ),
        Node(
            package='person_depth_localizer',
            executable='rough_localizer',
            name='rough_person_localizer',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'depth_frame_override': '',
                'rgb_width': 640,
                'rgb_height': 480,
            }],
        ),
    ])
