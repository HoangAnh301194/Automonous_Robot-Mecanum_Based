from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    default_map_graph = os.path.join(pkg_share, 'maps', 'my_slam_graph')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_slam_loc_params = os.path.join(pkg_share, 'config', 'slam_localization.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')

    localization_mode = LaunchConfiguration('localization_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map_yaml')
    map_graph = LaunchConfiguration('map_graph')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    use_amcl = IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"]))
    use_slam_toolbox = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'slam_toolbox'"])
    )

    amcl_nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        condition=use_amcl,
        launch_arguments={
            'slam': 'False',
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
    )

    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=use_slam_toolbox,
        parameters=[
            slam_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_file_name': map_graph,
            },
        ],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        condition=use_slam_toolbox,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart,
        }.items()
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
            'localization_mode',
            default_value='amcl',
            description="Localization mode: 'amcl' or 'slam_toolbox'."
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on real robot.'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz.'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes.'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value=default_map_yaml,
            description='Path to map yaml (used in AMCL mode).'
        ),
        DeclareLaunchArgument(
            'map_graph',
            default_value=default_map_graph,
            description='Path base name of slam posegraph without extension.'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params,
            description='Path to Nav2 parameters file.'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_loc_params,
            description='Path to slam_toolbox localization params.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='Path to RViz config.'
        ),
        TimerAction(period=3.0, actions=[amcl_nav2_stack]),
        TimerAction(period=2.0, actions=[slam_localization_node]),
        TimerAction(period=5.0, actions=[nav2_navigation]),
        TimerAction(period=6.0, actions=[rviz_node]),
    ])
