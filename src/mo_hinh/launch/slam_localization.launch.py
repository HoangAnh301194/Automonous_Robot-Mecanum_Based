from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')

    default_slam_params = os.path.join(pkg_share, 'config', 'slam_localization.yaml')
    default_map_file_name = os.path.join(pkg_share, 'maps', 'my_slam_graph')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_file_name = LaunchConfiguration('map_file_name')

    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file_name,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock.'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to slam_toolbox localization parameters.'
        ),
        DeclareLaunchArgument(
            'map_file_name',
            default_value=default_map_file_name,
            description='Base filename of serialized pose graph (without extension).'
        ),
        slam_localization_node,
    ])
