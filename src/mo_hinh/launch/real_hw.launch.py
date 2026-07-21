from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xe.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    esp_port = LaunchConfiguration('esp_port')
    lidar_port = LaunchConfiguration('lidar_port')
    esp_baudrate = LaunchConfiguration('esp_baudrate')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    esp_publish_tf = LaunchConfiguration('esp_publish_tf')
    esp_odom_topic = LaunchConfiguration('esp_odom_topic')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.'
        ),

        DeclareLaunchArgument(
            'esp_port',
            default_value='/dev/esp32',
            description='ESP32 serial port.'
        ),

        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/rplidar',
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
            'esp_publish_tf',
            default_value='true',
            description='Publish odom -> base_footprint TF.'
        ),

        DeclareLaunchArgument(
            'esp_odom_topic',
            default_value='/odom',
            description='Odometry topic.'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),

        Node(
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
        ),

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=['/home/orin/ros2_ws/config/scan_filter.yaml'],
            remappings=[
                ('scan', 'scan_raw'),
                ('scan_filtered', 'scan')
            ],
            output='screen'
        ),

        Node(
            package='bno055_imu',
            executable='bnoneko',
            name='bno055',
            output='screen',
        ),

        Node(
            package='odom_pub',
            executable='odom_pub',
            name='odom_pub',
            output='screen',
            parameters=[{
                'publish_tf': ParameterValue(esp_publish_tf, value_type=bool),
                'esp_port': esp_port,
                'esp_baudrate': ParameterValue(esp_baudrate, value_type=int),
            }],
            remappings=[
                ('/odom', esp_odom_topic),
            ],
        ),
    ])
