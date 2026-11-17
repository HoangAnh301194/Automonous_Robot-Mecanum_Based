import os
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pre_launch_cleanup() -> None:
    """Dọn sạch dứt điểm toàn bộ tiến trình ROS 2 cũ còn sót trước khi bật Kiosk."""
    print("[KIOSK LAUNCH] Đang dọn sạch toàn bộ tiến trình ROS 2 mồ côi trước khi khởi động...")
    current_pid = os.getpid()
    cleanup_cmds = [
        # Dừng mềm trước
        "pkill -SIGINT -f 'component_container' 2>/dev/null",
        "pkill -SIGINT -f 'nav2' 2>/dev/null",
        "pkill -SIGINT -f 'robot_ui_kiosk' 2>/dev/null",
        "sleep 0.5",
        # Kill dứt điểm toàn bộ tiến trình ROS 2 và hardware bridge
        "pkill -9 -f 'component_container' 2>/dev/null",
        "pkill -9 -f 'nav2_' 2>/dev/null",
        "pkill -9 -f 'amcl' 2>/dev/null",
        "pkill -9 -f 'controller_server' 2>/dev/null",
        "pkill -9 -f 'planner_server' 2>/dev/null",
        "pkill -9 -f 'bt_navigator' 2>/dev/null",
        "pkill -9 -f 'lifecycle_manager' 2>/dev/null",
        "pkill -9 -f 'costmap_2d' 2>/dev/null",
        "pkill -9 -f 'sllidar' 2>/dev/null",
        "pkill -9 -f 'scan_to_scan_filter' 2>/dev/null",
        "pkill -9 -f 'robot_state_publisher' 2>/dev/null",
        "pkill -9 -f 'ros2_control_node' 2>/dev/null",
        "pkill -9 -f 'rf2o' 2>/dev/null",
        "pkill -9 -f 'ekf_node' 2>/dev/null",
        "pkill -9 -f 'bnoneko' 2>/dev/null",
        "pkill -9 -f 'esp_bridge' 2>/dev/null",
        # Dọn sạch log tạm
        "rm -f /tmp/ros_layer_*.log /tmp/.ros_layer_lock_*",
    ]
    # Diệt kiosk cũ nhưng loại trừ PID hiện tại
    kill_old_kiosk = f"pgrep -f 'robot_ui_kiosk' | grep -v {current_pid} | xargs -r kill -9 2>/dev/null"
    cleanup_cmds.append(kill_old_kiosk)

    try:
        subprocess.run(["bash", "-c", "; ".join(cleanup_cmds)], check=False)
        time.sleep(0.5)
        print("[KIOSK LAUNCH] Đã hoàn tất dọn sạch hệ thống! Bắt đầu nạp Kiosk GUI.")
    except Exception as exc:
        print(f"[KIOSK LAUNCH] Cảnh báo dọn dẹp: {exc}")


def generate_launch_description() -> LaunchDescription:
    _pre_launch_cleanup()

    windowed = LaunchConfiguration("windowed")
    hide_cursor = LaunchConfiguration("hide_cursor")
    use_sim_time = LaunchConfiguration("use_sim_time")
    llm_env_file = LaunchConfiguration("llm_env_file")
    auto_start_9router = LaunchConfiguration("auto_start_9router")
    router_command = LaunchConfiguration("router_command")
    debug = LaunchConfiguration("debug")

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
                "debug",
                default_value="false",
                description="Enable verbose debug logging in robot_ui.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the Gazebo simulation clock.",
            ),
            DeclareLaunchArgument(
                "llm_env_file",
                default_value=os.environ.get(
                    "KIOSK_LLM_ENV_FILE",
                    "config/llm.local.env"
                    if os.path.isfile("config/llm.local.env")
                    else "~/ros2_ws/config/llm.local.env",
                ),
                description="Local 9router config file loaded by the kiosk process.",
            ),
            DeclareLaunchArgument(
                "auto_start_9router",
                default_value="true",
                description="Automatically start local 9router when the API is offline.",
            ),
            DeclareLaunchArgument(
                "router_command",
                default_value=os.environ.get("KIOSK_9ROUTER_COMMAND", ""),
                description="Optional 9router launch command; default detects local CLI/Docker.",
            ),
            Node(
                package="robot_ui",
                executable="robot_ui_kiosk",
                name="robot_ui_kiosk_process",
                output="screen",
                emulate_tty=True,
                arguments=[
                    "--windowed",
                    windowed,
                    "--hide-cursor",
                    hide_cursor,
                    "--debug",
                    debug,
                    "--llm-env-file",
                    llm_env_file,
                    "--auto-start-9router",
                    auto_start_9router,
                    "--9router-command",
                    router_command,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
