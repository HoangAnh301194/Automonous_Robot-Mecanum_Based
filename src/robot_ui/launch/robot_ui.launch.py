from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    package_share = get_package_share_directory("robot_ui")
    default_config = os.path.join(package_share, "config", "robot_ui.yaml")

    config = LaunchConfiguration("config")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Robot UI YAML configuration file.",
            ),
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="HTTP bind address.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8000",
                description="HTTP port.",
            ),
            Node(
                package="robot_ui",
                executable="robot_ui_server",
                name="robot_ui_server",
                output="screen",
                arguments=[
                    "--config",
                    config,
                    "--host",
                    host,
                    "--port",
                    port,
                ],
            ),
        ]
    )
