"""Run the standalone wake word CLI via ROS launch without creating a ROS node.

No publishers/subscribers are created; this launch only manages the process.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    executable = PathJoinSubstitution(
        [FindPackagePrefix("robot_wakeword"), "lib", "robot_wakeword", "wakeword_probe"]
    )
    config_file = PathJoinSubstitution(
        [FindPackageShare("robot_wakeword"), "config", "keywords.json"]
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=config_file,
            description="Path to the JSON keyword configuration",
        ),
        DeclareLaunchArgument(
            "models_dir", default_value=os.path.expanduser("~/vosk-models"),
            description="Location of downloaded pretrained Vosk models",
        ),
        DeclareLaunchArgument(
            "mic_source", default_value="@DEFAULT_SOURCE@",
            description="PulseAudio source, e.g. @DEFAULT_SOURCE@",
        ),
        DeclareLaunchArgument(
            "cooldown", default_value="2.0",
            description="Minimum seconds between emitted wake events",
        ),
        ExecuteProcess(
            cmd=[
                executable, "live",
                "--config", LaunchConfiguration("config"),
                "--models-dir", LaunchConfiguration("models_dir"),
                "--source", LaunchConfiguration("mic_source"),
                "--cooldown", LaunchConfiguration("cooldown"),
            ],
            output="screen",
        ),
    ])
