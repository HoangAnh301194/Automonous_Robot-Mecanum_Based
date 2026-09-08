from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    windowed = LaunchConfiguration("windowed")
    hide_cursor = LaunchConfiguration("hide_cursor")
    use_sim_time = LaunchConfiguration("use_sim_time")

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
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
