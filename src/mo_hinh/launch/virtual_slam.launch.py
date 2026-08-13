from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    virtual_gazebo_launch = os.path.join(pkg_share, 'launch', 'virtual_robot_gazebo.launch.py')
    nav2_navigation_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
    default_world = os.path.join(pkg_share, 'worlds', 'virtual_lab.world')
    default_slam_params = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz_slam.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(virtual_gazebo_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'world': world,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw': yaw,
        }.items(),
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
    )

    nav2_launch = GroupAction(
        actions=[
            # Force all Nav2 nodes in this include scope to use /clock.
            SetParameter(name='use_sim_time', value=use_sim_time),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_params_file,
                    'autostart': nav2_autostart,
                    'use_composition': 'False',
                }.items(),
            ),
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    delayed_slam = TimerAction(period=4.0, actions=[slam_node])
    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[nav2_launch],
        condition=IfCondition(use_nav2),
    )
    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('nav2_autostart', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.10'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params),
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),

        gazebo_launch,
        delayed_slam,
        delayed_nav2,
        delayed_rviz,
    ])
