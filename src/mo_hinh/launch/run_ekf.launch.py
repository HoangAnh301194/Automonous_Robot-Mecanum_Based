from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


def generate_launch_description():
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_file = resolve_path('urdf', 'xe.urdf')
    default_world = resolve_path('worlds', 'virtual_lab.world')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    world = LaunchConfiguration('world')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'xe_robot',
            '-string', robot_description,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_robot]
    )

    unpause_service_node = Node(
        package='mo_hinh',
        executable='dummy_unpause.py',
        output='screen'
    )

    scan_filter_config = resolve_path('config', 'scan_filter.yaml')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/scan', '/scan_raw'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world', default_value=default_world),

        robot_state_publisher_node,
        gz_sim,
        delayed_spawn,
        unpause_service_node,
        bridge,
        laser_filter_node,
    ])
