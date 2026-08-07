from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('hand_wave_detection'))
    default_config = package_share / 'config.yaml'

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value=str(default_config),
                description='Hand-wave detector YAML configuration.',
            ),
            DeclareLaunchArgument(
                'image_topic',
                default_value='/camera/color/image_raw',
                description='Camera color image topic.',
            ),
            DeclareLaunchArgument(
                'backend',
                default_value='rtmpose',
                choices=['yolo11', 'rtmpose'],
                description='Pose backend: YOLO11 pose or YOLO11 plus RTMPose.',
            ),
            DeclareLaunchArgument(
                'device',
                default_value='cuda:0',
                description='Inference device, for example cuda:0 or cpu.',
            ),
            DeclareLaunchArgument(
                'detector_model',
                default_value='yolo11n.pt',
                description='Optional YOLO11 person-detector model override.',
            ),
            DeclareLaunchArgument(
                'yolo_pose_model',
                default_value='yolo11n-pose.pt',
                description='Optional YOLO11 pose-model override.',
            ),
            DeclareLaunchArgument(
                'rtmpose_model',
                default_value=(
                    'https://download.openmmlab.com/mmpose/v1/projects/'
                    'rtmposev1/onnx_sdk/rtmpose-s_simcc-body7_pt-body7_'
                    '420e-256x192-acd4a1ef_20230504.zip'
                ),
                description='Optional local RTMPose ONNX model override.',
            ),
            Node(
                package='hand_wave_detection',
                executable='hand_wave_detector',
                name='hand_wave_detection',
                output='screen',
                parameters=[
                    {
                        'config': LaunchConfiguration('config'),
                        'image_topic': LaunchConfiguration('image_topic'),
                        'backend': LaunchConfiguration('backend'),
                        'device': LaunchConfiguration('device'),
                        'detector_model': LaunchConfiguration('detector_model'),
                        'yolo_pose_model': LaunchConfiguration('yolo_pose_model'),
                        'rtmpose_model': LaunchConfiguration('rtmpose_model'),
                    }
                ],
            ),
        ]
    )
