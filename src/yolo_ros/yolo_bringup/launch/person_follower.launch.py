# Copyright (C) 2026 Gemini CLI

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    
    # Arguments
    model_arg = DeclareLaunchArgument('model', default_value='/home/orin/ros2_ws/src/Pose_detection/yolo26n-pose.engine')
    input_image_topic_arg = DeclareLaunchArgument('input_image_topic', default_value='/camera/camera/color/image_raw')
    input_depth_topic_arg = DeclareLaunchArgument('input_depth_topic', default_value='/camera/camera/depth/image_rect_raw')
    input_depth_info_topic_arg = DeclareLaunchArgument('input_depth_info_topic', default_value='/camera/camera/depth/camera_info')
    target_frame_arg = DeclareLaunchArgument('target_frame', default_value='camera_link')
    threshold_arg = DeclareLaunchArgument('threshold', default_value='0.25')
    
    # YOLO + Tracking + 3D + Debug (Visuals)
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolo_bringup_dir, 'launch', 'yolov8.launch.py')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'use_tracking': 'True',
            'use_3d': 'True',
            'use_debug': 'True', # This enables the visual bounding boxes window
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'input_depth_topic': LaunchConfiguration('input_depth_topic'),
            'input_depth_info_topic': LaunchConfiguration('input_depth_info_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'threshold': LaunchConfiguration('threshold'),
            'imgsz_height': '640',
            'imgsz_width': '640',
            'namespace': 'yolo'
        }.items()
    )

    # Person Tracker Node (The data provider for your Behavior Tree)
    tracker_node = Node(
        package='yolo_ros',
        executable='follower_node',
        name='person_tracker_node',
        remappings=[
            ('yolo/detections_3d', '/yolo/detections_3d'),
            ('person_tracking', '/person_tracking')
        ]
    )

    return LaunchDescription([
        model_arg,
        input_image_topic_arg,
        input_depth_topic_arg,
        input_depth_info_topic_arg,
        target_frame_arg,
        threshold_arg,
        yolo_launch,
        tracker_node
    ])
