from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mo_hinh')
    hw_launch_file = os.path.join(pkg_share, 'launch', 'real_hw.launch.py')

    odom_source = LaunchConfiguration('odom_source')
    use_sim_time = LaunchConfiguration('use_sim_time')
    esp_port = LaunchConfiguration('esp_port')
    lidar_port = LaunchConfiguration('lidar_port')
    esp_baudrate = LaunchConfiguration('esp_baudrate')
    lidar_baudrate = LaunchConfiguration('lidar_baudrate')
    rf2o_freq = LaunchConfiguration('rf2o_freq')

    use_esp = IfCondition(PythonExpression(["'", odom_source, "' == 'esp'"]))
    use_rf2o = IfCondition(PythonExpression(["'", odom_source, "' == 'rf2o'"]))

    hw_with_esp_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'esp_port': esp_port,
            'lidar_port': lidar_port,
            'esp_baudrate': esp_baudrate,
            'lidar_baudrate': lidar_baudrate,
            'esp_publish_tf': 'true',
            'esp_odom_topic': '/odom',
        }.items(),
        condition=use_esp,
    )

    hw_with_rf2o_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'esp_port': esp_port,
            'lidar_port': lidar_port,
            'esp_baudrate': esp_baudrate,
            'lidar_baudrate': lidar_baudrate,
            'esp_publish_tf': 'false',
            'esp_odom_topic': '/odom_esp_raw',
        }.items(),
        condition=use_rf2o,
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=use_rf2o,
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': ParameterValue(rf2o_freq, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_source',
            default_value='esp',
            description="Odom source: 'esp' or 'rf2o'."
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on real robot.'
        ),
        DeclareLaunchArgument(
            'esp_port',
            default_value='/dev/ttyUSB0',
            description='ESP serial port. Override if needed.'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB1',
            description='LiDAR serial port. Override if needed.'
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
            'rf2o_freq',
            default_value='20.0',
            description='RF2O update frequency in Hz.'
        ),
        hw_with_esp_odom,
        hw_with_rf2o_odom,
        rf2o_node,
    ])

