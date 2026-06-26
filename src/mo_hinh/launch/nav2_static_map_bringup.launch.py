from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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

        print(f'[detect] {port} -> "{line}"')
        return line.startswith('E,')
    except Exception as e:
        print(f'[detect] {port} error: {e}')
        return False


def detect_ports():
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

    print(f'[detect] ESP32 port = {esp_port}')
    print(f'[detect] LiDAR port = {lidar_port}')

    return esp_port, lidar_port


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map_yaml = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    default_nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config = os.path.join(pkg_share, 'config', 'rviz.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'xe.urdf')

    map_yaml_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

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

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'false',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    delayed_nav2 = TimerAction(period=5.0, actions=[nav2_stack])
    delayed_rviz = TimerAction(period=8.0, actions=[rviz_node])

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to static map yaml file.'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params,
            description='Full path to Nav2 parameter yaml.'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='Full path to RViz config file.'
        ),

        rsp_node,
        lidar_node,
        imu_node,
        odom_node,
        delayed_nav2,
        delayed_rviz,
    ])
