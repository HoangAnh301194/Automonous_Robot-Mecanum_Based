from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_config_manager',
            executable='config_manager_node',
            name='config_manager_node',
            output='screen'
        ),
        Node(
            package='robot_system_monitor',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen'
        ),
        Node(
            package='robot_navigation_manager',
            executable='navigation_manager_node',
            name='navigation_manager_node',
            output='screen'
        ),
        Node(
            package='robot_mission_control',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen'
        )
    ])
