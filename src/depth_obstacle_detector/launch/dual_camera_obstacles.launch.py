from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    front_config = LaunchConfiguration('front_config')
    rear_config = LaunchConfiguration('rear_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_debug = LaunchConfiguration('enable_debug')

    front_node = Node(
        package='depth_obstacle_detector',
        executable='obstacle_detector',
        name='front_depth_obstacle_detector',
        output='screen',
        parameters=[{
            'config_file': front_config,
            'enable_debug': enable_debug,
            'use_sim_time': use_sim_time,
        }],
    )

    rear_node = Node(
        package='depth_obstacle_detector',
        executable='obstacle_detector',
        name='rear_depth_obstacle_detector',
        output='screen',
        parameters=[{
            'config_file': rear_config,
            'enable_debug': enable_debug,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'front_config',
            description='Path to front depth obstacle detector YAML.',
        ),
        DeclareLaunchArgument(
            'rear_config',
            description='Path to rear depth obstacle detector YAML.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('enable_debug', default_value='true'),
        front_node,
        rear_node,
    ])
