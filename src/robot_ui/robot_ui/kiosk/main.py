from __future__ import annotations

import argparse
import os
import sys
import threading

import rclpy
from PyQt5.QtCore import QLibraryInfo, Qt
from PyQt5.QtWidgets import QApplication
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import remove_ros_args

from robot_ui.kiosk.ros_node import KioskRosNode
from robot_ui.kiosk.signals import KioskSignals
from robot_ui.kiosk.llm_env import load_llm_env_file
from robot_ui.kiosk.router_process import start_local_9router, stop_local_9router


def boolean_argument(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected true/false, received: {value}")


def parse_args(arguments: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Native Robot UI touchscreen kiosk")
    parser.add_argument(
        "--windowed",
        type=boolean_argument,
        nargs="?",
        const=True,
        default=False,
        help="Open a resizable window instead of fullscreen (development only).",
    )
    parser.add_argument(
        "--hide-cursor",
        type=boolean_argument,
        nargs="?",
        const=True,
        default=False,
        help="Hide the mouse pointer for the deployed touchscreen.",
    )
    parser.add_argument(
        "--llm-env-file",
        default=os.environ.get(
            "KIOSK_LLM_ENV_FILE",
            "config/llm.local.env"
            if os.path.isfile("config/llm.local.env")
            else "~/ros2_ws/config/llm.local.env",
        ),
        help="Local 9router settings file; read before importing the chatbot.",
    )
    parser.add_argument(
        "--auto-start-9router",
        type=boolean_argument,
        default=True,
        help="Start local 9router if its API port is not listening.",
    )
    parser.add_argument(
        "--9router-command",
        dest="router_command",
        nargs="?",
        const=os.environ.get("KIOSK_9ROUTER_COMMAND", ""),
        default=os.environ.get("KIOSK_9ROUTER_COMMAND", ""),
        help="Optional local startup command (parsed as argv, never a shell).",
    )
    return parser.parse_args(arguments[1:])


def main(args: list[str] | None = None) -> int:
    raw_args = list(sys.argv if args is None else args)
    app_args = remove_ros_args(args=raw_args)
    options = parse_args(app_args)

    # Must happen before importing KioskWindow -> llm_client, whose defaults
    # are taken from the environment at module import time.
    try:
        loaded = load_llm_env_file(options.llm_env_file)
        configured = bool(os.environ.get("LLM_API_KEY"))
        print(
            "[robot_ui] 9router config loaded from "
            f"{os.path.expanduser(options.llm_env_file)} "
            f"(URL/model loaded; API key {'configured' if configured else 'missing'})",
            flush=True,
        )
        if not all(os.environ.get(key) for key in ("LLM_BASE_URL", "LLM_API_KEY", "LLM_MODEL")):
            print("[robot_ui] Warning: some LLM settings are empty.", file=sys.stderr)
    except (OSError, ValueError) as exc:
        # The navigation UI may still be useful without 9router.
        print(f"[robot_ui] Warning: 9router settings not loaded: {exc}", file=sys.stderr)

    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    qt_plugin_path = os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH", "")
    if "cv2" in qt_plugin_path:
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(
            QLibraryInfo.PluginsPath
        )
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    application = QApplication([app_args[0]])
    application.setApplicationName("Souta Robot")
    if options.hide_cursor:
        application.setOverrideCursor(Qt.BlankCursor)

    # Import after QApplication is initialized. Importing cv2 earlier can make
    # its bundled Qt plugin override the compatible system platform plugin.
    from robot_ui.kiosk.window import KioskWindow

    owned_router = None
    if all(os.environ.get(key) for key in ("LLM_BASE_URL", "LLM_API_KEY", "LLM_MODEL")):
        owned_router = start_local_9router(
            os.environ["LLM_BASE_URL"],
            enabled=options.auto_start_9router,
            command=options.router_command,
        )
    elif options.auto_start_9router:
        print("[robot_ui] 9router autostart skipped: incomplete LLM configuration.", flush=True)

    rclpy.init(args=raw_args)
    signals = KioskSignals()
    ros_node = KioskRosNode(signals)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_node)
    # Connect Qt signals before processing latched ROS topics such as /map.
    # Otherwise the first TRANSIENT_LOCAL map can arrive before the UI is ready.
    window = KioskWindow(ros_node, signals)
    if options.windowed:
        window.resize(1024, 600)
        window.show()
    else:
        window.showFullScreen()

    ros_thread = threading.Thread(target=executor.spin, name="robot-ui-ros", daemon=True)
    ros_thread.start()

    try:
        return application.exec_()
    finally:
        try:
            executor.shutdown(timeout_sec=2.0)
            ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            ros_thread.join(timeout=2.0)
        finally:
            stop_local_9router(owned_router)


if __name__ == "__main__":
    raise SystemExit(main())
