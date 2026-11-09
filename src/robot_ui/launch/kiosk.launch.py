import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    windowed = LaunchConfiguration("windowed")
    hide_cursor = LaunchConfiguration("hide_cursor")
    use_sim_time = LaunchConfiguration("use_sim_time")
    llm_env_file = LaunchConfiguration("llm_env_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "windowed",
                default_value="false",
                description="Run in a resizable development window.",
            ),
            DeclareLaunchArgument(
                "hide_cursor",
                default_value="true",
                description="Hide the pointer on the touchscreen.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "llm_env_file",
                default_value=os.environ.get(
                    "KIOSK_LLM_ENV_FILE", "~/ros2_ws/config/llm.local.env"
                ),
                description="Local 9router config file loaded by the kiosk process.",
            ),
            Node(
                package="robot_ui",
                executable="robot_ui_kiosk",
                name="robot_ui_kiosk_process",
                output="screen",
                arguments=[
                    "--windowed",
                    windowed,
                    "--hide-cursor",
                    hide_cursor,
                    "--llm-env-file",
                    llm_env_file,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
