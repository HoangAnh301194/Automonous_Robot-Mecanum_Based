from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

import os
import glob
import time

try:
    import serial
except ImportError:
    serial = None


def list_serial_ports():
    return sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))


def is_esp32_port(port, baudrate=115200, timeout=1.0):
    if serial is None:
        return False
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(1.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        ser.close()
        return line.startswith('E,')
    except Exception:
        return False


def detect_ports():
    esp_alias = '/dev/esp32'
    lidar_alias = '/dev/lidar'

    if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
        return esp_alias, lidar_alias

    ports = list_serial_ports()
    esp_port = None
    other_ports = []

    for port in ports:
        if is_esp32_port(port):
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

    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_slam.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xe.urdf')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    esp_port, lidar_port = detect_ports()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        output='screen'
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    imu_node = Node(
        package='bno055_imu',
        executable='bnoneko',
        name='bno055',
        output='screen'
    )

    odom_node = Node(
        package='odom_pub',
        executable='odom_pub',
        name='odom_pub',
        parameters=[{
            'publish_tf': True,
            'esp_port': esp_port,
            'esp_baudrate': 115200
        }],
        output='screen'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    delayed_slam = TimerAction(period=3.0, actions=[slam_node])
    delayed_rviz = TimerAction(period=5.0, actions=[rviz_node])

    return LaunchDescription([
        rsp_node,
        lidar_node,
        imu_node,
        odom_node,
        delayed_slam,
        delayed_rviz,
    ])
