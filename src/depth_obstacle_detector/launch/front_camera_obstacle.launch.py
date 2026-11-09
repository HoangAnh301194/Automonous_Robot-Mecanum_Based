import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_debug = LaunchConfiguration('enable_debug')

    front_node = Node(
        package='depth_obstacle_detector',
        executable='obstacle_detector',
        name='front_depth_obstacle_detector',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'enable_debug': enable_debug,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/a/ros2_ws/my_map/cam_truoc.yaml',
            description='Path to depth obstacle detector YAML config file.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_debug', default_value='true'),
        front_node,
    ])
