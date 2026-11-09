import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_urdf'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file_path = os.path.join(pkg_share, 'urdf', 'robot_urdf.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
