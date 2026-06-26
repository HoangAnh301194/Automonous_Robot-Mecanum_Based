from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import glob
import os
import time

try:
    import serial
except ImportError:
    serial = None


def _list_serial_ports():
    return sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))


def _is_esp32_port(port, baudrate=115200, timeout=1.0):
    if serial is None:
        return False
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        ser.close()
        return line.startswith('E,')
    except Exception:
        return False


def _detect_ports():
    # Prefer stable udev aliases if available.
    esp_alias = '/dev/esp32'
    lidar_alias = '/dev/lidar'
    if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
        return esp_alias, lidar_alias

    ports = _list_serial_ports()
    esp_port = None
    other_ports = []

    for port in ports:
        if _is_esp32_port(port):
            esp_port = port
        else:
            other_ports.append(port)

    lidar_port = other_ports[0] if other_ports else None

    if esp_port is None:
        for p in ports:
            if p.endswith('ttyUSB0') or p.endswith('ttyACM0'):
                esp_port = p
                break

    if lidar_port is None:
        for p in ports:
            if p != esp_port:
                lidar_port = p
                break

    if esp_port is None:
        esp_port = '/dev/ttyUSB0'
    if lidar_port is None:
        lidar_port = '/dev/ttyUSB1'

    return esp_port, lidar_port


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xe.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    detected_esp_port, detected_lidar_port = _detect_ports()

    use_sim_time = LaunchConfiguration('use_sim_time')
    esp_port = LaunchConfiguration('esp_port')
    lidar_port = LaunchConfiguration('lidar_port')
    esp_baudrate = LaunchConfiguration('esp_baudrate')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    esp_publish_tf = LaunchConfiguration('esp_publish_tf')
    esp_odom_topic = LaunchConfiguration('esp_odom_topic')
    esp_wheel_odom_mode = LaunchConfiguration('esp_wheel_odom_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on real robot.'
        ),
        DeclareLaunchArgument(
            'esp_port',
            default_value=detected_esp_port,
            description='ESP serial port.'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value=detected_lidar_port,
            description='LiDAR serial port.'
        ),
        DeclareLaunchArgument(
            'esp_baudrate',
            default_value='115200',
            description='ESP serial baudrate.'
        ),
        DeclareLaunchArgument(
            'lidar_baudrate',
            default_value='115200',
            description='LiDAR serial baudrate.'
        ),
        DeclareLaunchArgument(
            'esp_publish_tf',
            default_value='true',
            description='Whether ESP odom node publishes odom->base_footprint TF.'
        ),
        DeclareLaunchArgument(
            'esp_odom_topic',
            default_value='/odom',
            description='Output odom topic of ESP odom node.'
        ),
        DeclareLaunchArgument(
            'esp_wheel_odom_mode',
            default_value='all_4',
            description="ESP odom wheel mode: 'all_4' or 'wheels_1_4'."
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
        ),

        Node(
            package='bno055_imu',
            executable='bnoneko',
            name='bno055',
            output='screen',
        ),

        # Single serial owner for ESP: receives /cmd_vel and publishes odom/dataenc.
        Node(
            package='odom_pub',
            executable='odom_pub',
            name='odom_pub',
            output='screen',
            parameters=[{
                'publish_tf': ParameterValue(esp_publish_tf, value_type=bool),
                'esp_port': esp_port,
                'esp_baudrate': ParameterValue(esp_baudrate, value_type=int),
                'wheel_odom_mode': esp_wheel_odom_mode,
            }],
            remappings=[
                ('/odom', esp_odom_topic),
            ],
        ),
    ])
