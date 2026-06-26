from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    default_slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz_slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on real robot.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz with SLAM configuration.'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Path to slam_toolbox mapping parameters.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='Path to RViz config.'
        ),
        TimerAction(period=2.0, actions=[slam_node]),
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
