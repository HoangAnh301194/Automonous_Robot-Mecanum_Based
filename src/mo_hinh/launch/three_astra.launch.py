"""Launch three Astra Pro cameras bound by their OpenNI hardware serials.

Front/Rear Depth use astra_camera with serial_number. The AI RGB UVC stream
is resolved from ai_serial via the Depth USB device and its internal hub.
Ambiguous or missing physical device identity aborts startup (no guessing).
"""

import os
import stat
import sys
from pathlib import Path

# This helper is installed alongside the launch file by the mo_hinh package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _astra_serial_resolver import resolve_video_from_serial

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def depth_camera(name, serial_arg, enabled_arg, fps_arg, width_arg, height_arg):
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
                'depth_width': ParameterValue(
                    LaunchConfiguration(width_arg), value_type=int
                ),
                'depth_height': ParameterValue(
                    LaunchConfiguration(height_arg), value_type=int
                ),
                'depth_fps': ParameterValue(
                    LaunchConfiguration(fps_arg), value_type=int
                ),
                'publish_tf': False,
                'camera_link_frame_id': name + '_link',
            }],
        )

    def ai_rgb_camera(context, *args, **kwargs):
        front_serial = LaunchConfiguration('front_serial').perform(context)
        rear_serial = LaunchConfiguration('rear_serial').perform(context)
        ai_serial = LaunchConfiguration('ai_serial').perform(context)
        device, report = resolve_video_from_serial(
            ai_serial=ai_serial,
            required_serials=[front_serial, rear_serial, ai_serial],
        )
        resolved_device = os.path.realpath(device)
        if (not os.path.exists(resolved_device)
                or not stat.S_ISCHR(os.stat(resolved_device).st_mode)):
            raise RuntimeError(
                'Serial {} resolved to {}, but it is not an active '
                'character device.'.format(ai_serial, device)
            )

        return [
            *[LogInfo(msg='[three_astra] Serial-to-RGB: {}'.format(s))
              for s in report],
            LogInfo(msg=(
                '[three_astra] AI serial {} -> {}; Front={} Rear={}'.format(
                    ai_serial, resolved_device, front_serial, rear_serial
                )
            )),
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name='usb_cam',
                namespace='ai_camera/color',
                output='screen',
                parameters=[{
                    'video_device': resolved_device,
                    'image_width': ParameterValue(
                        LaunchConfiguration('ai_color_width'), value_type=int
                    ),
                    'image_height': ParameterValue(
                        LaunchConfiguration('ai_color_height'), value_type=int
                    ),
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

    def launch_verified_cameras(context, *args, **kwargs):
        # Resolve and RELEASE all three OpenNI devices before any Depth node
        # starts; concurrent OpenNI handles may race or lock the hardware.
        start_ai = (LaunchConfiguration('enable_ai').perform(context).lower()
                    in ('true', '1', 'yes'))
        ai_actions = ai_rgb_camera(context) if start_ai else []
        return [
            front_depth_tf_alias,
            depth_camera('front_camera', 'front_serial', 'enable_front',
                         'front_depth_fps', 'front_depth_width', 'front_depth_height'),
            TimerAction(
                period=2.0,
                actions=[
                    depth_camera('rear_camera', 'rear_serial', 'enable_rear',
                                 'rear_depth_fps', 'rear_depth_width', 'rear_depth_height'),
                ],
            ),
            TimerAction(period=4.0, actions=ai_actions) if start_ai
            else LogInfo(msg='[three_astra] AI RGB disabled'),
        ]

    return LaunchDescription([
        # AI RGB Camera parameters
        DeclareLaunchArgument(
            'ai_color_width', default_value='640',
            description='AI RGB width (default: 640, or 1280 for HD).',
        ),
        DeclareLaunchArgument(
            'ai_color_height', default_value='480',
            description='AI RGB height (default: 480, or 720 for HD).',
        ),
        DeclareLaunchArgument(
            'ai_color_fps', default_value='30',
            description='AI RGB MJPEG FPS (default: 30).',
        ),
        DeclareLaunchArgument(
            'ai_serial', default_value='17122611013',
            description='OpenNI serial used to resolve the physical AI RGB UVC camera.',
        ),

        # Front Depth Camera parameters (640x480 @ 15 FPS)
        DeclareLaunchArgument(
            'front_serial', default_value='17110310855',
            description='Serial of front depth camera.',
        ),
        DeclareLaunchArgument(
            'front_depth_width', default_value='320',
            description='Front depth width (default: 320).',
        ),
        DeclareLaunchArgument(
            'front_depth_height', default_value='240',
            description='Front depth height (default: ).',
        ),
        DeclareLaunchArgument(
            'front_depth_fps', default_value='15',
            description='Front depth FPS (default: 15).',
        ),

        # Rear Depth Camera parameters (320x240 @ 15 FPS - siêu nhẹ cho Jetson)
        DeclareLaunchArgument(
            'rear_serial', default_value='17122514440',
            description='Serial of rear depth camera.',
        ),
        DeclareLaunchArgument(
            'rear_depth_width', default_value='320',
            description='Rear depth width (default: 320 - optimal lightweight).',
        ),
        DeclareLaunchArgument(
            'rear_depth_height', default_value='240',
            description='Rear depth height (default: 240).',
        ),
        DeclareLaunchArgument(
            'rear_depth_fps', default_value='15',
            description='Rear depth FPS (default: 15).',
        ),

        # System arguments
        DeclareLaunchArgument(
            'device_num', default_value='2',
            description='Number of Astra Depth driver nodes (default: 2).',
        ),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        OpaqueFunction(function=launch_verified_cameras),
    ])

