# Copyright (C) 2026 Gemini CLI

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    
    # Arguments
    model_arg = DeclareLaunchArgument('model', default_value='yolo11n.pt')
    tracker_arg = DeclareLaunchArgument('tracker', default_value='bytetrack.yaml')
    device_arg = DeclareLaunchArgument('device', default_value='cuda:0')
    input_image_topic_arg = DeclareLaunchArgument('input_image_topic', default_value='/ai_camera/color/image_raw')
    input_depth_topic_arg = DeclareLaunchArgument('input_depth_topic', default_value='')
    input_depth_info_topic_arg = DeclareLaunchArgument('input_depth_info_topic', default_value='')
    target_frame_arg = DeclareLaunchArgument('target_frame', default_value='camera_link')
    threshold_arg = DeclareLaunchArgument('threshold', default_value='0.25')
    use_3d_arg = DeclareLaunchArgument(
        'use_3d', default_value='False',
        description='Opt-in only: requires calibrated, registered RGB and Depth topics.'
    )

    # Patch 1: person-following optimized defaults
    classes_arg = DeclareLaunchArgument('classes', default_value='0',
        description='Person class only for person follower')
    imgsz_height_arg = DeclareLaunchArgument('imgsz_height', default_value='384',
        description='Reduced from 480 to save 25% compute')
    imgsz_width_arg = DeclareLaunchArgument('imgsz_width', default_value='640')
    max_det_arg = DeclareLaunchArgument('max_det', default_value='10',
        description='Reduced from 300; 10 is enough for person following')
    
    def validate_3d_topics(context, *args, **kwargs):
        # Do not silently subscribe to old /camera/depth topics when 3D is enabled.
        use_3d = context.perform_substitution(LaunchConfiguration('use_3d')).lower()
        if use_3d in {'true', '1', 'yes'}:
            depth_topic = context.perform_substitution(LaunchConfiguration('input_depth_topic'))
            info_topic = context.perform_substitution(LaunchConfiguration('input_depth_info_topic'))
            if not depth_topic or not info_topic:
                raise RuntimeError(
                    '3D person following requires explicitly configured, calibrated '
                    'and registered input_depth_topic and input_depth_info_topic. '
                    'Do not pair AI RGB with Front/Rear Depth without registration.'
                )
        return []
    # Safe default: RGB detection/tracking only; no cross-camera 3D inference.
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(yolo_bringup_dir, 'launch', 'yolo.launch.py')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'tracker': LaunchConfiguration('tracker'),
            'device': LaunchConfiguration('device'),
            'use_tracking': 'True',
            'use_3d': LaunchConfiguration('use_3d'),
            'use_debug': 'False', # Disabled to save CPU and improve FPS
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'input_depth_topic': LaunchConfiguration('input_depth_topic'),
            'input_depth_info_topic': LaunchConfiguration('input_depth_info_topic'),
            'target_frame': LaunchConfiguration('target_frame'),
            'threshold': LaunchConfiguration('threshold'),
            'classes': LaunchConfiguration('classes'),
            'imgsz_height': LaunchConfiguration('imgsz_height'),
            'imgsz_width': LaunchConfiguration('imgsz_width'),
            'max_det': LaunchConfiguration('max_det'),
            'namespace': 'yolo'
        }.items()
    )

    # PersonTracking needs valid 3D detections; do not start on RGB-only mode.
    tracker_node = Node(
        package='yolo_ros',
        executable='follower_node',
        name='person_tracker_node',
        condition=IfCondition(LaunchConfiguration('use_3d')),
        remappings=[
            ('yolo/detections_3d', '/yolo/detections_3d'),
            ('person_tracking', '/person_tracking')
        ]
    )

    return LaunchDescription([
        model_arg,
        tracker_arg,
        device_arg,
        input_image_topic_arg,
        input_depth_topic_arg,
        input_depth_info_topic_arg,
        target_frame_arg,
        threshold_arg,
        use_3d_arg,
        classes_arg,
        imgsz_height_arg,
        imgsz_width_arg,
        max_det_arg,
        OpaqueFunction(function=validate_3d_topics),
        yolo_launch,
        tracker_node
    ])
