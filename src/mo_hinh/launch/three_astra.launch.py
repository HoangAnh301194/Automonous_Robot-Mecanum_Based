"""Launch front/rear Astra Depth and a separately selected AI UVC/RGB stream.

The two Astra Depth devices are selected using their OpenNI serial numbers.
Astra Pro RGB interfaces have no unique USB serial, so the AI stream uses
usb_cam and a fixed USB *port path* (resolved to /dev/videoN at launch time).
Never select the AI UVC interface by a non-existent OpenNI serial number.
"""
import os
import stat

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def depth_camera(name, serial_arg, enabled_arg, fps_arg):
        return Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='camera',
            namespace=name,
            output='screen',
            condition=IfCondition(LaunchConfiguration(enabled_arg)),
            parameters=[{
                'camera_name': name,
                'serial_number': ParameterValue(
                    LaunchConfiguration(serial_arg), value_type=str
                ),
                # Number of *Astra driver processes* in this launch.
                # The AI RGB interface is owned by usb_cam, not Astra.
                'device_num': ParameterValue(
                    LaunchConfiguration('device_num'), value_type=int
                ),
                'enable_color': False,
                'enable_depth': True,
                'enable_ir': False,
                'enable_point_cloud': False,
                'enable_colored_point_cloud': False,
                'use_uvc_camera': False,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': ParameterValue(
                    LaunchConfiguration(fps_arg), value_type=int
                ),
                'publish_tf': False,
                'camera_link_frame_id': name + '_link',
            }],
        )

    def ai_rgb_camera(context, *args, **kwargs):
        # usb_cam on this Jetson does not correctly accept a /dev/v4l/by-path
        # symlink directly. Resolve it to the active /dev/videoN first.
        by_path = LaunchConfiguration('ai_video_device').perform(context)
        device = os.path.realpath(by_path)
        if not os.path.exists(device) or not stat.S_ISCHR(os.stat(device).st_mode):
            raise RuntimeError(
                'AI RGB UVC device not found: {} -> {}. Check USB connection '
                'and `ls -l /dev/v4l/by-path/`. '
                'Override with ai_video_device:=<correct path>.'.format(
                    by_path, device
                )
            )

        return [
            LogInfo(msg='[three_astra] AI RGB video device: {} -> {}'.format(
                by_path, device
            )),
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam',
                namespace='ai_camera/color',
                output='screen',
                parameters=[{
                    'video_device': device,
                    'image_width': 640,
                    'image_height': 480,
                    'framerate': ParameterValue(
                        LaunchConfiguration('ai_color_fps'), value_type=float
                    ),
                    'io_method': 'mmap',
                    'pixel_format': 'mjpeg2rgb',
                    'camera_name': 'ai_camera',
                    'frame_id': 'ai_camera_color_optical_frame',
                }],
            ),
        ]

    # The front optical frame is already a child of the existing robot URDF
    # camera_depth_optical_frame. No duplicate Astra TF is published.
    front_depth_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_astra_optical_alias',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_depth_optical_frame',
            '--child-frame-id', 'front_camera_depth_optical_frame',
        ],
        condition=IfCondition(LaunchConfiguration('enable_front')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ai_video_device',
            default_value=(
                '/dev/v4l/by-path/'
                'platform-3610000.usb-usb-0:2.2.1:1.0-video-index0'
            ),
            description='Physical USB-port path of AI RGB UVC interface.',
        ),
        DeclareLaunchArgument(
            'ai_color_fps', default_value='30',
            description='AI RGB MJPEG FPS (default: 30).',
        ),
        # Compatibility with prior CLI; no AI OpenNI node is started.
        DeclareLaunchArgument(
            'ai_serial', default_value='17122611013',
            description='Legacy AI OpenNI serial (not used by usb_cam).',
        ),
        DeclareLaunchArgument(
            'front_serial', default_value='17110310855',
            description='Serial of front depth camera.',
        ),
        DeclareLaunchArgument(
            'rear_serial', default_value='17122514440',
            description='Serial of rear depth camera.',
        ),
        DeclareLaunchArgument(
            'device_num', default_value='2',
            description='Number of Astra Depth driver nodes (default: 2).',
        ),
        DeclareLaunchArgument(
            'front_depth_fps', default_value='15',
            description='Front depth FPS (default: 15).',
        ),
        DeclareLaunchArgument(
            'rear_depth_fps', default_value='15',
            description='Rear depth FPS (default: 15).',
        ),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        front_depth_tf_alias,
        depth_camera('front_camera', 'front_serial', 'enable_front',
                     'front_depth_fps'),
        TimerAction(
            period=2.0,
            actions=[
                depth_camera('rear_camera', 'rear_serial', 'enable_rear',
                             'rear_depth_fps'),
            ],
        ),
        TimerAction(
            period=4.0,
            actions=[
                OpaqueFunction(
                    function=ai_rgb_camera,
                    condition=IfCondition(LaunchConfiguration('enable_ai')),
                ),
            ],
        ),
    ])
