import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    mo_hinh_dir = get_package_share_directory('mo_hinh')
    depth_obstacle_dir = get_package_share_directory('depth_obstacle_detector')
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    hand_wave_dir = get_package_share_directory('hand_wave_detection')

    ws_root = Path(robot_bringup_dir).resolve().parents[3]
    default_front_cfg = str(ws_root / 'my_map' / 'cam_truoc.yaml')
    default_rear_cfg = str(ws_root / 'my_map' / 'cam_sau.yaml')

    enable_cameras = LaunchConfiguration('enable_cameras')
    enable_obstacle_detectors = LaunchConfiguration('enable_obstacle_detectors')
    enable_ai_nodes = LaunchConfiguration('enable_ai_nodes')
    enable_front = LaunchConfiguration('enable_front')
    enable_rear = LaunchConfiguration('enable_rear')
    enable_ai = LaunchConfiguration('enable_ai')
    enable_ai_depth = LaunchConfiguration('enable_ai_depth')
    device_num = LaunchConfiguration('device_num')
    ai_color_width = LaunchConfiguration('ai_color_width')
    ai_color_height = LaunchConfiguration('ai_color_height')
    ai_color_fps = LaunchConfiguration('ai_color_fps')

    front_obstacle_config = LaunchConfiguration('front_obstacle_config')
    rear_obstacle_config = LaunchConfiguration('rear_obstacle_config')
    obstacle_debug = LaunchConfiguration('obstacle_debug')

    yolo_model = LaunchConfiguration('yolo_model')
    yolo_device = LaunchConfiguration('yolo_device')
    pose_backend = LaunchConfiguration('pose_backend')
    pose_device = LaunchConfiguration('pose_device')
    pose_max_people = LaunchConfiguration('pose_max_people')
    base_frame = LaunchConfiguration('base_frame')
    map_frame = LaunchConfiguration('map_frame')

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mo_hinh_dir, 'launch', 'three_astra.launch.py')
        ),
        condition=IfCondition(enable_cameras),
        launch_arguments={
            'enable_front': enable_front,
            'enable_rear': enable_rear,
            'enable_ai': enable_ai,
            'enable_ai_depth': enable_ai_depth,
            'device_num': device_num,
            'ai_color_width': ai_color_width,
            'ai_color_height': ai_color_height,
            'ai_color_fps': ai_color_fps,
        }.items(),
    )

    obstacle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depth_obstacle_dir, 'launch', 'dual_camera_obstacles.launch.py')
        ),
        condition=IfCondition(enable_obstacle_detectors),
        launch_arguments={
            'front_config': front_obstacle_config,
            'rear_config': rear_obstacle_config,
            'use_sim_time': 'false',
            'enable_debug': obstacle_debug,
        }.items(),
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_dir, 'launch', 'yolo.launch.py')
        ),
        launch_arguments={
            'use_tracking': 'True',
            'use_3d': 'False',
            'input_image_topic': '/ai_camera/color/image_raw',
            'model': yolo_model,
            'device': yolo_device,
            'imgsz_height': '384',
            'imgsz_width': '640',
            'tracking_image_width': '640',
            'tracking_image_height': '480',
        }.items(),
    )

    hand_wave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hand_wave_dir, 'launch', 'hand_wave_detection.launch.py')
        ),
        launch_arguments={
            'image_topic': '/ai_camera/color/image_raw',
            'tracking_topic': '/yolo/tracking',
            'backend': pose_backend,
            'device': pose_device,
            'max_people': pose_max_people,
        }.items(),
    )

    rough_localizer_node = Node(
        package='person_depth_localizer',
        executable='rough_localizer',
        name='rough_person_localizer',
        output='screen',
        parameters=[{
            'base_frame': base_frame,
            'map_frame': map_frame,
            'depth_frame_override': '',
            'rgb_width': 640,
            'rgb_height': 480,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_cameras', default_value='true'),
        DeclareLaunchArgument('enable_obstacle_detectors', default_value='true'),
        DeclareLaunchArgument('enable_ai_nodes', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_ai_depth', default_value='true'),
        DeclareLaunchArgument('device_num', default_value='3'),
        DeclareLaunchArgument('ai_color_width', default_value='640'),
        DeclareLaunchArgument('ai_color_height', default_value='480'),
        DeclareLaunchArgument('ai_color_fps', default_value='30'),
        DeclareLaunchArgument('front_obstacle_config', default_value=default_front_cfg),
        DeclareLaunchArgument('rear_obstacle_config', default_value=default_rear_cfg),
        DeclareLaunchArgument('obstacle_debug', default_value='false'),
        DeclareLaunchArgument('yolo_model', default_value='yolo11n.pt'),
        DeclareLaunchArgument('yolo_device', default_value='cuda:0'),
        DeclareLaunchArgument('pose_backend', default_value='rtmpose'),
        DeclareLaunchArgument('pose_device', default_value='cpu'),
        DeclareLaunchArgument('pose_max_people', default_value='2'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('map_frame', default_value='map'),

        cameras_launch,
        # front_camera starts at 0s and rear_camera starts at 2.0s in three_astra.launch.py.
        TimerAction(period=3.5, actions=[obstacle_launch], condition=IfCondition(enable_obstacle_detectors)),
        # Wait for three_astra.launch.py (ai RGB starts at 6s) before starting AI consumers.
        TimerAction(period=8.0, actions=[yolo_launch], condition=IfCondition(enable_ai_nodes)),
        TimerAction(period=9.0, actions=[hand_wave_launch, rough_localizer_node], condition=IfCondition(enable_ai_nodes)),
    ])
