#!/usr/bin/env python3
"""
Launch file for multi-waypoint mission with person detection and interception.
Usage: ros2 launch nhiemvuboss multi_waypoint_mission.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('nhiemvuboss')
    default_waypoint_file = os.path.join(pkg_share, 'config', 'waypoints_example.json')

    # ==================== DECLARE ARGUMENTS ====================
    
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='fixed',
        choices=['fixed', 'interactive'],
        description='Mission mode: fixed=use waypoint list, interactive=wait for RViz goals'
    )

    declare_waypoint_file = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to JSON file with waypoints'
    )

    declare_target_distance = DeclareLaunchArgument(
        'target_distance',
        default_value='0.4',
        description='Distance to approach detected person (meters)'
    )

    declare_wait_waypoint = DeclareLaunchArgument(
        'wait_at_waypoint_seconds',
        default_value='2.0',
        description='Time to wait at each waypoint (seconds)'
    )

    declare_wait_person = DeclareLaunchArgument(
        'wait_after_person_seconds',
        default_value='10.0',
        description='Time to wait after intercepting person (seconds)'
    )

    declare_intercept = DeclareLaunchArgument(
        'intercept_enabled',
        default_value='true',
        description='Enable person interception during waypoint navigation'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='Logging level'
    )

    # ==================== NODES ====================

    multi_waypoint_node = Node(
        package='nhiemvuboss',
        executable='multi_waypoint_mission',
        name='waypoint_mission',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {
                'use_fixed_goals': ParameterValue(
                    PythonExpression(["'", LaunchConfiguration('mode'), "' == 'fixed'"]),
                    value_type=bool
                )
            },
            {'waypoint_file': LaunchConfiguration('waypoint_file')},
            {'target_distance': LaunchConfiguration('target_distance')},
            {'wait_at_waypoint_seconds': LaunchConfiguration('wait_at_waypoint_seconds')},
            {'wait_after_person_seconds': LaunchConfiguration('wait_after_person_seconds')},
            {'intercept_enabled': LaunchConfiguration('intercept_enabled')},
            
            # Detection params
            {'detections_topic': '/yolo/detections'},
            {'camera_info_topic': '/camera/color/camera_info'},
            {'aligned_depth_topic': '/camera/depth/image_raw'},
            {'class_name': 'person'},
            {'min_confidence': 0.6},
            {'depth_roi_size': 20},
            {'max_depth_m': 6.0},
            
            # Transform/Map params
            {'map_frame': 'map'},
            {'base_frame': 'base_footprint'},
            
            # Person detection ranges
            {'intercept_min_range_m': 0.3},
            {'intercept_max_range_m': 5.0},
            {'require_map_tf': True},
            
            # Nav2 action
            {'navigate_action_name': '/navigate_to_pose'},
            
            # Goal input
            {'goal_input_topic': '/goal_pose'},
            
            # Shutdown behavior
            {'cancel_on_shutdown': True},
            {'stop_cmd_vel_on_shutdown': True},
        ]
    )

    # Info message
    log_mission_mode = LogInfo(
        condition=IfCondition(["'", LaunchConfiguration('mode'), "' == 'fixed'"]),
        msg=['Starting MULTI-WAYPOINT MISSION (Fixed mode) from: ', LaunchConfiguration('waypoint_file')]
    )

    log_interactive = LogInfo(
        condition=IfCondition(["'", LaunchConfiguration('mode'), "' == 'interactive'"]),
        msg="Starting MULTI-WAYPOINT MISSION (Interactive mode) - Use 'Nav2 Goal' in RViz to add waypoints"
    )

    # ==================== LAUNCH DESC ====================

    return LaunchDescription([
        declare_mode,
        declare_waypoint_file,
        declare_target_distance,
        declare_wait_waypoint,
        declare_wait_person,
        declare_intercept,
        declare_log_level,
        log_mission_mode,
        log_interactive,
        multi_waypoint_node,
    ])
