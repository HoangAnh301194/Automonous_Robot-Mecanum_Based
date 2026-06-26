from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import glob
import time
import serial


def list_serial_ports():
    ports = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
    return ports


def is_esp32_port(port, baudrate=115200, timeout=1.0):
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(1.5)  # ESP có th? reset khi m? serial

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # N?u ESP t? stream d? li?u thì ch? c?n d?c
        line = ser.readline().decode(errors='ignore').strip()

        ser.close()

        print(f'[detect] {port} -> "{line}"')

        return line.startswith('E,')

    except Exception as e:
        print(f'[detect] {port} error: {e}')
        return False


def detect_ports():
    # Uu tiên alias udev n?u b?n có t?o s?n
    esp_alias = '/dev/esp32'
    lidar_alias = '/dev/lidar'

    if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
        print('[detect] Using udev aliases')
        return esp_alias, lidar_alias

    ports = list_serial_ports()
    print(f'[detect] Found serial ports: {ports}')

    esp_port = None
    other_ports = []

    for port in ports:
        if is_esp32_port(port):
            esp_port = port
        else:
            other_ports.append(port)

    lidar_port = other_ports[0] if other_ports else None

    # fallback n?u detect chua ra
    if esp_port is None:
        for p in ports:
            if 'ttyUSB0' in p or 'ttyACM0' in p:
                esp_port = p
                break

    if lidar_port is None:
        for p in ports:
            if p != esp_port:
                lidar_port = p
                break

    print(f'[detect] ESP32 port = {esp_port}')
    print(f'[detect] LiDAR port = {lidar_port}')

    return esp_port, lidar_port


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xe.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    esp_port, lidar_port = detect_ports()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }],
        output='screen'
    )

    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': lidar_port if lidar_port else '/dev/ttyUSB1',
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
            'esp_port': esp_port if esp_port else '/dev/ttyUSB0'
        }],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        lidar_node,
        imu_node,
        odom_node
    ])
