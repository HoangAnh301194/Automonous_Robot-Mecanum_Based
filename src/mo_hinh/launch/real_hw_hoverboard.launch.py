from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def resolve_path(sub_dir, file_name, pkg_name='mo_hinh'):
    user_src_path = os.path.expanduser(f'~/ros2_ws/src/{pkg_name}/{sub_dir}/{file_name}')
    if os.path.exists(user_src_path):
        return user_src_path
    user_ws_path = os.path.expanduser(f'~/ros2_ws/{sub_dir}/{file_name}')
    if os.path.exists(user_ws_path):
        return user_ws_path
    try:
        return os.path.join(get_package_share_directory(pkg_name), sub_dir, file_name)
    except Exception:
        return user_src_path


def launch_setup(context, *args, **kwargs):
    urdf_file = resolve_path('urdf', 'xe.urdf')
    scan_filter_config = resolve_path('config', 'scan_filter.yaml')
    controllers_file = resolve_path('config', 'hoverboard_controllers.yaml')

    hoverboard_port_val = context.perform_substitution(LaunchConfiguration('hoverboard_port'))
    lidar_port_val = context.perform_substitution(LaunchConfiguration('lidar_port'))
    lidar_baud_val = int(context.perform_substitution(LaunchConfiguration('lidar_baudrate')))
    use_sim_time_val = context.perform_substitution(LaunchConfiguration('use_sim_time')).lower() == 'true'

    with open(urdf_file, 'r', encoding='utf-8') as f:
        urdf_text = f.read()

    # Tự động thay thế cổng nếu người dùng truyền cổng khác /dev/hoverboard
    if hoverboard_port_val and hoverboard_port_val != '/dev/hoverboard':
        urdf_text = urdf_text.replace(
            '<param name="device">/dev/hoverboard</param>',
            f'<param name="device">{hoverboard_port_val}</param>'
        )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_text,
            'use_sim_time': use_sim_time_val,
        }],
    )

    # 2. LiDAR RPLidar Node
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port_val,
            'serial_baudrate': lidar_baud_val,
            'frame_id': 'laser_link',
            'angle_compensate': True,
        }],
        remappings=[('scan', 'scan_raw')],
    )

    # 3. Laser Filter
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[scan_filter_config],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan')
        ],
        output='screen'
    )

    # 4. IMU BNO055
    bno055_node = Node(
        package='bno055_imu',
        executable='bnoneko',
        name='bno055',
        output='screen',
    )

    # 5. ros2_control Controller Manager Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': urdf_text}, controllers_file],
        output='screen',
        remappings=[
            ('/hoverboard_base_controller/cmd_vel_unstamped', '/cmd_vel'),
            ('/hoverboard_base_controller/odom', '/odom'),
        ],
    )

    # 6. Spawner cho joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 7. Spawner cho hoverboard_base_controller (diff_drive_controller)
    hoverboard_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hoverboard_base_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Đợi joint_state_broadcaster khởi động xong rồi mới kích hoạt hoverboard_base_controller
    delay_hoverboard_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[hoverboard_controller_spawner],
        )
    )

    return [
        robot_state_publisher_node,
        sllidar_node,
        laser_filter_node,
        bno055_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_hoverboard_controller_spawner,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (false on real robot).'
        ),
        DeclareLaunchArgument(
            'hoverboard_port',
            default_value='/dev/hoverboard',
            description='Hoverboard serial port (USB-TTL).'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
            description='RPLidar serial port.'
        ),
        DeclareLaunchArgument(
            'lidar_baudrate',
            default_value='115200',
            description='RPLidar baudrate.'
        ),
        OpaqueFunction(function=launch_setup)
    ])
