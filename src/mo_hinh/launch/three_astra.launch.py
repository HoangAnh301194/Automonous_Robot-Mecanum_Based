"""Launch three Astra Pro cameras bound by their OpenNI hardware serials.

Front/Rear/AI Depth use astra_camera with serial_number. The AI RGB UVC stream
is resolved from ai_serial via the Depth USB device and its internal hub.
Ambiguous or missing physical device identity aborts startup (no guessing).
"""

import os
import stat
import sys
from pathlib import Path

# This helper is installed alongside the launch file by the mo_hinh package.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _astra_serial_resolver import resolve_available_cameras

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_true(val: str) -> bool:
    return val.strip().lower() in ('true', '1', 'yes')


def generate_launch_description():
    def depth_camera(name, serial_arg, fps_arg, width_arg, height_arg, device_num_val):
        return Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='camera',
            namespace=name,
            output='screen',
            parameters=[{
                'camera_name': name,
                'serial_number': ParameterValue(
                    LaunchConfiguration(serial_arg), value_type=str
                ),
                # Must match the actual number of Astra depth driver processes
                # launched so shared-memory/semaphore cleanup completes cleanly.
                'device_num': device_num_val,
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

    def build_ai_rgb_node(ai_video, ai_serial, allow_missing):
        resolved_device = os.path.realpath(ai_video)
        if (not os.path.exists(resolved_device)
                or not stat.S_ISCHR(os.stat(resolved_device).st_mode)):
            msg = (
                'Serial {} resolved to {}, but it is not an active '
                'character device.'.format(ai_serial, ai_video)
            )
            if allow_missing:
                return None, msg
            raise RuntimeError(msg)

        node = Node(
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
        )
        return node, None

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
    )

    def launch_verified_cameras(context, *args, **kwargs):
        # Resolve and RELEASE OpenNI devices before any Depth node starts;
        # concurrent OpenNI handles may race or lock the hardware.
        allow_missing = _is_true(
            LaunchConfiguration('allow_missing_cameras').perform(context)
        )
        enable_front = _is_true(
            LaunchConfiguration('enable_front').perform(context)
        )
        enable_rear = _is_true(
            LaunchConfiguration('enable_rear').perform(context)
        )
        enable_ai = _is_true(
            LaunchConfiguration('enable_ai').perform(context)
        )
        enable_ai_depth = _is_true(
            LaunchConfiguration('enable_ai_depth').perform(context)
        )

        front_serial = LaunchConfiguration('front_serial').perform(context).strip()
        rear_serial = LaunchConfiguration('rear_serial').perform(context).strip()
        ai_serial = LaunchConfiguration('ai_serial').perform(context).strip()

        available_serials, ai_video, report, missing_serials = resolve_available_cameras(
            ai_serial=ai_serial,
            front_serial=front_serial,
            rear_serial=rear_serial,
            enable_ai=enable_ai,
            allow_missing=allow_missing,
        )

        start_front = enable_front and (front_serial in available_serials)
        start_rear = enable_rear and (rear_serial in available_serials)
        start_ai_depth = enable_ai and enable_ai_depth and (ai_serial in available_serials)

        ai_rgb_node = None
        ai_rgb_warn = None
        if enable_ai and ai_video is not None:
            ai_rgb_node, ai_rgb_warn = build_ai_rgb_node(
                ai_video, ai_serial, allow_missing
            )

        active_depth_count = int(start_front) + int(start_rear) + int(start_ai_depth)
        device_num_raw = LaunchConfiguration('device_num').perform(context).strip().lower()
        if allow_missing or device_num_raw in ('', 'auto'):
            device_num_val = max(1, active_depth_count)
        else:
            device_num_val = int(device_num_raw)

        actions = [
            *[LogInfo(msg='[three_astra] {}'.format(s)) for s in report],
        ]
        if missing_serials:
            actions.append(LogInfo(
                msg='[three_astra] Missing/unplugged serial(s): {} (running with available cameras only)'.format(
                    ', '.join(missing_serials)
                )
            ))
        if ai_rgb_warn:
            actions.append(LogInfo(msg='[three_astra] WARNING: {}'.format(ai_rgb_warn)))

        active_names = []
        next_delay = 0.0

        def schedule_action(act):
            nonlocal next_delay
            if next_delay <= 0.0:
                actions.append(act)
            else:
                actions.append(TimerAction(period=next_delay, actions=[act]))
            next_delay += 2.0

        if start_front:
            active_names.append('front_camera')
            actions.append(front_depth_tf_alias)
            schedule_action(
                depth_camera('front_camera', 'front_serial',
                             'front_depth_fps', 'front_depth_width', 'front_depth_height',
                             device_num_val)
            )
        elif enable_front:
            actions.append(LogInfo(msg='[three_astra] Skipping front_camera (serial {} not found)'.format(front_serial)))

        if start_rear:
            active_names.append('rear_camera')
            schedule_action(
                depth_camera('rear_camera', 'rear_serial',
                             'rear_depth_fps', 'rear_depth_width', 'rear_depth_height',
                             device_num_val)
            )
        elif enable_rear:
            actions.append(LogInfo(msg='[three_astra] Skipping rear_camera (serial {} not found)'.format(rear_serial)))

        if start_ai_depth:
            active_names.append('ai_camera_depth')
            schedule_action(
                depth_camera('ai_camera', 'ai_serial',
                             'ai_depth_fps', 'ai_depth_width', 'ai_depth_height',
                             device_num_val)
            )
        elif enable_ai and enable_ai_depth:
            actions.append(LogInfo(msg='[three_astra] Skipping ai_camera depth (serial {} not found)'.format(ai_serial)))

        if ai_rgb_node is not None:
            active_names.append('ai_camera_rgb')
            schedule_action(ai_rgb_node)
        elif enable_ai:
            actions.append(LogInfo(msg='[three_astra] Skipping ai_camera RGB (serial {} unavailable)'.format(ai_serial)))

        actions.append(LogInfo(
            msg='[three_astra] Starting cameras: {} (device_num={})'.format(
                ', '.join(active_names) if active_names else 'none', device_num_val
            )
        ))
        return actions

    return LaunchDescription([
        # AI Depth is a separate OpenNI stream from the UVC RGB image.
        # Depth and RGB pixels are NOT assumed registered.
        DeclareLaunchArgument('enable_ai_depth', default_value='true'),
        DeclareLaunchArgument('ai_depth_width', default_value='160'),
        DeclareLaunchArgument('ai_depth_height', default_value='120'),
        DeclareLaunchArgument('ai_depth_fps', default_value='15'),

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

        # Front Depth Camera parameters (160x120 @ 15 FPS)
        DeclareLaunchArgument(
            'front_serial', default_value='17110310855',
            description='Serial of front depth camera.',
        ),
        DeclareLaunchArgument(
            'front_depth_width', default_value='160',
            description='Front depth width (default: 160).',
        ),
        DeclareLaunchArgument(
            'front_depth_height', default_value='120',
            description='Front depth height (default: 120).',
        ),
        DeclareLaunchArgument(
            'front_depth_fps', default_value='15',
            description='Front depth FPS (default: 15).',
        ),

        # Rear Depth Camera parameters (160x120 @ 15 FPS - siêu nhẹ cho Jetson)
        DeclareLaunchArgument(
            'rear_serial', default_value='17122514440',
            description='Serial of rear depth camera.',
        ),
        DeclareLaunchArgument(
            'rear_depth_width', default_value='160',
            description='Rear depth width (default: 160 - optimal ultra-lightweight).',
        ),
        DeclareLaunchArgument(
            'rear_depth_height', default_value='120',
            description='Rear depth height (default: 120).',
        ),
        DeclareLaunchArgument(
            'rear_depth_fps', default_value='15',
            description='Rear depth FPS (default: 15).',
        ),

        # System arguments
        DeclareLaunchArgument(
            'device_num', default_value='auto',
            description="Number of Astra Depth driver nodes ('auto' computes from connected cameras).",
        ),
        DeclareLaunchArgument(
            'allow_missing_cameras', default_value='true',
            description='Allow starting whichever cameras are connected even if some are unplugged.',
        ),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        OpaqueFunction(function=launch_verified_cameras),
    ])

