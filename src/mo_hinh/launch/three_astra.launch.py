"""Bring up three serial-pinned Astra Pro cameras.

Create the driver nodes directly, rather than forwarding numerical-looking serial
strings through Orbbec's XML launch <param value="..."> auto type inference.
The obstacle/AI processing nodes remain separate.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    def camera_driver(name, serial_arg, enabled_arg, color, depth, publish_tf):
        return Node(
            package='astra_camera',
            executable='astra_camera_node',
            name='camera',
            namespace=name,
            output='screen',
            condition=IfCondition(LaunchConfiguration(enabled_arg)),
            parameters=[{
                'camera_name': name,
                # Critical: the serial is an identifier, never an integer.
                'serial_number': ParameterValue(
                    LaunchConfiguration(serial_arg), value_type=str
                ),
                'device_num': ParameterValue(
                    LaunchConfiguration('device_num'), value_type=int
                ),
                'enable_color': color,
                'enable_depth': depth,
                'enable_ir': False,
                'enable_point_cloud': False,
                'enable_colored_point_cloud': False,
                'use_uvc_camera': color,
                'uvc_vendor_id': 0x2bc5,
                'uvc_product_id': 0x0501,
                'color_width': 640,
                'color_height': 480,
                'color_fps': 30,
                'depth_width': 640,
                'depth_height': 480,
                'depth_fps': 30,
                'publish_tf': publish_tf,
                # Avoid colliding with camera_link owned by robot_state_publisher.
                'camera_link_frame_id': name + '_link',
            }],
        )

    # Existing URDF publishes camera_depth_optical_frame and
    # rear_camera_depth_optical_frame. Both depth drivers must not publish
    # their own transforms. Bridge front driver frame to the URDF frame.
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
            'ai_serial', default_value='17122611013',
            description='Serial of the forward-facing RGB/AI camera.',
        ),
        DeclareLaunchArgument(
            'front_serial', default_value='17110310855',
            description='Serial of the forward down-tilted depth camera.',
        ),
        DeclareLaunchArgument(
            'rear_serial', default_value='17122514440',
            description='Serial of the rear down-tilted depth camera.',
        ),
        DeclareLaunchArgument(
            'device_num', default_value='3',
            description='Total number of physically attached Astra devices.',
        ),
        DeclareLaunchArgument('enable_ai', default_value='true'),
        DeclareLaunchArgument('enable_front', default_value='true'),
        DeclareLaunchArgument('enable_rear', default_value='true'),
        front_depth_tf_alias,
        camera_driver('front_camera', 'front_serial', 'enable_front',
                      color=False, depth=True, publish_tf=False),
        TimerAction(
            period=2.0,
            actions=[camera_driver('rear_camera', 'rear_serial', 'enable_rear',
                                   color=False, depth=True, publish_tf=False)],
        ),
        TimerAction(
            period=4.0,
            actions=[camera_driver('ai_camera', 'ai_serial', 'enable_ai',
                                   color=True, depth=False, publish_tf=True)],
        ),
    ])
