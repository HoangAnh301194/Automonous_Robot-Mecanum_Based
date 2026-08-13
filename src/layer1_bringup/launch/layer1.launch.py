import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_layer1 = get_package_share_directory('layer1_bringup')
    pkg_mo_hinh = get_package_share_directory('mo_hinh')

    urdf_file = os.path.join(pkg_mo_hinh, 'urdf', 'xe.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    ekf_config_file = os.path.join(pkg_layer1, 'config', 'ekf_3sources.yaml')
    scan_filter_config_file = os.path.join(pkg_layer1, 'config', 'scan_filter.yaml')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    esp_port       = LaunchConfiguration('esp_port')
    lidar_port     = LaunchConfiguration('lidar_port')
    esp_baudrate   = LaunchConfiguration('esp_baudrate')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rf2o_freq      = LaunchConfiguration('rf2o_freq')

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
    )

    # 2. RPLidar Driver
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': ParameterValue(lidar_baudrate, value_type=int),
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }],
        remappings=[('scan', 'scan_raw')],
    )

    # 3. Laser Scan Filter
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        parameters=[scan_filter_config_file],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan')
        ],
    )

    # 4. BNO055 IMU Driver
    bno055_imu_node = Node(
        package='bno055_imu',
        executable='bnoneko',
        name='bno055',
        output='screen',
    )

    # 5. ESP32 Serial Hardware Bridge Node
    esp_bridge_node = Node(
        package='layer1_bringup',
        executable='esp_bridge',
        name='esp_bridge',
        output='screen',
        parameters=[{
            'esp_port': esp_port,
            'esp_baudrate': ParameterValue(esp_baudrate, value_type=int),
        }],
    )

    # 6. RF2O Laser Odometry Node (publishes /odom_rf2o, publish_tf=False)
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': ParameterValue(rf2o_freq, value_type=float),
        }],
    )

    # 7. EKF Node (robot_localization: fuses /odom_encoder, /imu/data, /odom_rf2o -> /odom & TF)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.'
        ),
        DeclareLaunchArgument(
            'esp_port',
            default_value='/dev/ttyUSB0',
            description='ESP32 serial port.'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='RPLidar serial port.'
        ),
        DeclareLaunchArgument(
            'esp_baudrate',
            default_value='115200',
            description='ESP32 baudrate.'
        ),
        DeclareLaunchArgument(
            'lidar_baudrate',
            default_value='115200',
            description='RPLidar baudrate.'
        ),
        DeclareLaunchArgument(
            'rf2o_freq',
            default_value='10.0',
            description='RF2O update frequency in Hz.'
        ),

        robot_state_publisher_node,
        sllidar_node,
        laser_filter_node,
        bno055_imu_node,
        esp_bridge_node,
        rf2o_node,
        ekf_node,
    ])
