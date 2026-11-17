"""Regression tests for Kiosk UI performance optimization.

Verifies:
1. Live log updating when ROS 2 System console is visible.
2. Log buffering without QTextEdit DOM churn when hidden.
3. Chronological log snapshot restoration upon reopening.
4. Navigation pose, LiDAR scan, and obstacle points caching & refresh.
5. SLAM map caching & refresh across page transitions.
6. System Config subpage caching without redundant hidden UI rendering.
7. Non-blocking stop_layer (< 50ms on calling thread, no 500ms freeze).
8. Preservation of Start/Stop, Nav2, SLAM, Patrol, and E-Stop contracts.
9. Clean teardown and close events without unhandled exceptions.
"""
from __future__ import annotations

import os
from pathlib import Path
import time
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from robot_ui.kiosk.pages.navigation_page import NavigationPage  # noqa: E402
from robot_ui.kiosk.slam_page import SlamMapPage  # noqa: E402
from robot_ui.kiosk.pages.system_config_page import (  # noqa: E402
    SystemConfigPage,
)
from robot_ui.kiosk.pages.system_status.ros2_system import (  # noqa: E402
    ROS2SystemDetailScreen,
)


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_scenario_1_live_log_when_visible(app):
    """1. Đang mở ROS 2 System: live log cập nhật đúng vào QTextEdit."""
    screen = ROS2SystemDetailScreen()
    screen.set_console_visible(True)
    screen._current_filter = "all"

    screen.append_layer_log("layer1", "Testing live log line 1")
    screen.append_layer_log("layer2", "Testing live log line 2")

    content = screen.txt_layer_log.toPlainText()
    assert "Testing live log line 1" in content
    assert "Testing live log line 2" in content
    assert "[LAYER1]" in content
    assert "[LAYER2]" in content


def test_scenario_2_log_buffering_when_hidden(app):
    """2. Console bị ẩn: log tiếp tục lưu nhưng QTextEdit không render."""
    screen = ROS2SystemDetailScreen()
    screen.set_console_visible(True)
    screen.append_layer_log("layer1", "Visible initial log")
    initial_text = screen.txt_layer_log.toPlainText()

    # Hide console (simulate navigating to Home/Chat)
    screen.set_console_visible(False)
    screen.append_layer_log("layer1", "Hidden background log 1")
    screen.append_layer_log("layer2", "Hidden background log 2")

    # QTextEdit content MUST NOT change while hidden
    assert screen.txt_layer_log.toPlainText() == initial_text

    # But deque buffers MUST retain the logs
    all_logs = list(screen._logs_by_layer["all"])
    assert any("Hidden background log 1" in line for line in all_logs)
    assert any("Hidden background log 2" in line for line in all_logs)


def test_scenario_3_log_snapshot_on_reopening(app):
    """3. Quay lại ROS 2 System: hiển thị snapshot log gần nhất đúng thứ tự."""
    screen = ROS2SystemDetailScreen()
    screen.set_console_visible(False)

    for i in range(10):
        screen.append_layer_log("layer3", f"SLAM message #{i}")

    # QTextEdit was not updated while hidden
    assert "SLAM message" not in screen.txt_layer_log.toPlainText()

    # Reopen console
    screen.set_console_visible(True)
    content = screen.txt_layer_log.toPlainText()
    assert "SLAM message #0" in content
    assert "SLAM message #9" in content

    # Check chronological ordering
    idx0 = content.find("SLAM message #0")
    idx9 = content.find("SLAM message #9")
    assert idx0 < idx9


def test_scenario_4_navigation_pose_scan_obstacle_caching(app):
    """4. Navigation: pose, LiDAR, obstacle cập nhật mới nhất."""
    icons = {"goal": Path("/nonexistent/goal.png")}
    nav = NavigationPage(icons=icons)

    # Simulate navigating away to Home (inactive)
    nav.set_active(False)
    assert not nav._is_active
    assert not nav.map_widget._is_active

    # Receive 50 Hz poses and scans while inactive
    nav.set_pose({"x": 1.25, "y": 3.45, "yaw": 0.78})
    nav.set_scan_points([(1.0, 2.0), (3.0, 4.0)])
    nav.set_obstacle_points("front", [(1.5, 2.5)])

    # Cached data is stored
    assert nav.map_widget._pose == {"x": 1.25, "y": 3.45, "yaw": 0.78}
    assert nav.map_widget._scan_points == [(1.0, 2.0), (3.0, 4.0)]
    assert nav.map_widget._front_obs_points == [(1.5, 2.5)]

    # Reactivate page upon return
    nav.set_active(True)
    assert nav._is_active
    assert nav.map_widget._is_active


def test_scenario_5_slam_map_caching_and_display(app):
    """5. SLAM -> Chat -> SLAM: map hiển thị đúng khi trở lại."""
    icons = {
        "home": Path("/nonexistent/home.png"),
        "back": Path("/nonexistent/back.png"),
        "emergency": Path("/nonexistent/emergency.png"),
        "goal": Path("/nonexistent/goal.png"),
    }
    slam = SlamMapPage(icons=icons)

    # Deactivate
    slam.set_active(False)
    assert not slam._is_active
    assert not slam.live_map_widget._is_active

    # Update map and pose while inactive
    fake_map = {
        "preview_width": 20,
        "preview_height": 20,
        "resolution": 0.05,
        "origin": (0.0, 0.0, 0.0),
        "cells": [0] * 400,
    }
    slam.set_map(fake_map)
    slam.set_pose({"x": 0.5, "y": 0.5, "yaw": 0.0})

    # Image is prebuilt in memory and ready
    assert not slam.live_map_widget._map_image.isNull()

    # Activate on return
    slam.set_active(True)
    assert slam._is_active
    assert slam.live_map_widget._is_active


def test_scenario_6_system_config_subpage_caching(app):
    """6. Chuyển qua các subpage: không cập nhật UI ẩn không cần thiết."""
    page = SystemConfigPage()
    assert page.ros2_screen.txt_layer_log.document().maximumBlockCount() == 500

    # Initially at overview (screen 0)
    page.set_active(True)
    assert not page.ros2_screen._is_console_visible

    # Switch to ROS 2 System (screen 3)
    page._navigate_to_module("ros2")
    assert page.stack.currentIndex() == 3
    assert page.ros2_screen._is_console_visible

    # Switch to Hardware (screen 2)
    page._navigate_to_module("hardware")
    assert page.stack.currentIndex() == 2
    assert not page.ros2_screen._is_console_visible

    # Deactivate page entirely
    page.set_active(False)
    assert not page.ros2_screen._is_console_visible


def test_scenario_7_stop_layer_non_blocking_performance(app):
    """7. Bấm STOP layer: không bị block bởi time.sleep(0.5) (< 50ms)."""
    from robot_ui.kiosk.ros_node import KioskRosNode

    mock_proc = MagicMock()
    mock_proc.pid = 99999
    mock_proc.poll.return_value = None  # Process running

    class DummyRosNode:
        LAYER_ALIAS_MAP = KioskRosNode.LAYER_ALIAS_MAP
        _layer_procs = {"layer1": mock_proc}
        _layer_log_buffers = {"layer1": [], "all": []}
        _console_active = False

        def get_logger(self):
            m = MagicMock()
            return m

        signals = MagicMock()
        _record_layer_log = KioskRosNode._record_layer_log
        stop_layer = KioskRosNode.stop_layer

    node = DummyRosNode()

    t0 = time.monotonic()
    node.stop_layer("layer1")
    dt = time.monotonic() - t0

    # Must return virtually instantly (< 50 ms), NOT freeze for 500 ms
    assert dt < 0.05, f"stop_layer blocked for {dt*1000:.1f}ms!"
    assert "layer1" not in node._layer_procs


def test_scenario_8_ros2_contracts_intact(app):
    """8. Không thay đổi chức năng Start/Stop, Nav2, SLAM, Patrol, E-Stop."""
    icons = {"goal": Path("/nonexistent/goal.png")}
    nav = NavigationPage(icons=icons)

    # Verify signals and critical handlers exist and are callable
    assert hasattr(nav, "start_requested")
    assert hasattr(nav, "cancel_requested")
    assert hasattr(nav, "start_patrol_requested")
    assert hasattr(nav, "pause_patrol_requested")
    assert hasattr(nav, "emergency_stop_requested")

    # Verify E-stop emits correctly
    seen_estop = []
    nav.emergency_stop_requested.connect(lambda: seen_estop.append(True))
    nav.emergency_button.click()
    assert seen_estop == [True]


def test_scenario_9_clean_teardown_no_exceptions(app):
    """9. Không xuất hiện exception khi đóng Kiosk hoặc subprocess kết thúc."""
    nav = NavigationPage(icons={"goal": Path("/nonexistent/goal.png")})
    slam = SlamMapPage(icons={"goal": Path("/nonexistent/goal.png")})
    cfg = SystemConfigPage()

    # Call teardown/close events
    nav.close()
    slam.close()
    cfg.stop()
    cfg.close()
