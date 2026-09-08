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
    return parser.parse_args(arguments[1:])


def main(args: list[str] | None = None) -> int:
    raw_args = list(sys.argv if args is None else args)
    app_args = remove_ros_args(args=raw_args)
    options = parse_args(app_args)

    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    qt_plugin_path = os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH", "")
    if "cv2" in qt_plugin_path:
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(
            QLibraryInfo.PluginsPath
        )
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    application = QApplication([app_args[0]])
    application.setApplicationName("Robot Assistant")
    if options.hide_cursor:
        application.setOverrideCursor(Qt.BlankCursor)

    # Import after QApplication is initialized. Importing cv2 earlier can make
    # its bundled Qt plugin override the compatible system platform plugin.
    from robot_ui.kiosk.window import KioskWindow

    rclpy.init(args=raw_args)
    signals = KioskSignals()
    ros_node = KioskRosNode(signals)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(ros_node)
    ros_thread = threading.Thread(target=executor.spin, name="robot-ui-ros", daemon=True)
    ros_thread.start()

    window = KioskWindow(ros_node, signals)
    if options.windowed:
        window.resize(1024, 600)
        window.show()
    else:
        window.showFullScreen()

    exit_code = application.exec_()
    executor.shutdown(timeout_sec=2.0)
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join(timeout=2.0)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
