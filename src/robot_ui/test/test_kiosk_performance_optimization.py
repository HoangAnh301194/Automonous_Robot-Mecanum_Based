"""Regression tests for Kiosk UI performance optimization and 5 critical fixes.

Verifies:
1. Live log updating when ROS 2 System console is visible.
2. Log buffering without QTextEdit DOM churn when hidden.
3. Chronological log snapshot restoration upon reopening via backend provider (P0 fix).
4. Navigation pose, LiDAR scan, and obstacle points caching & refresh.
5. Lazy map rebuild: no image processing while inactive, rebuilds on activation (P1 fix).
6. Strictly single visible subpage telemetry rendering in System Config (P1 fix).
7. Non-blocking stop_layer with STOPPING state and race condition prevention (P1 fix).
8. Souta video background using OpenCV C++/SIMD resizing and dimension caching (P1 fix).
9. Preservation of Start/Stop, Nav2, SLAM, Patrol, and E-Stop contracts.
10. Clean teardown and close events without unhandled exceptions.
"""
from __future__ import annotations

import os
from pathlib import Path
import time
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from robot_ui.kiosk.pages.home_page import VideoBackground  # noqa: E402
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


def test_scenario_3_log_snapshot_sync_from_backend_provider(app):
    """3. P0 Fix: Đồng bộ snapshot log từ ROS 2 backend vào widget khi console mở lại."""
    screen = ROS2SystemDetailScreen()
    screen.set_console_visible(False)

    backend_logs = [
        "[10:00:01] [LAYER1] Hardware initialized",
        "[10:00:02] [LAYER2] Nav2 stack online",
        "[10:00:03] [LAYER3] SLAM graph optimized",
    ]

    # Cung cấp mock provider mô phỏng ros_node.get_layer_log_snapshot
    mock_provider = MagicMock(return_value=backend_logs)
    screen.set_log_provider(mock_provider)

    # Khi console đang ẩn, widget chưa có log mới này
    assert "Hardware initialized" not in screen.txt_layer_log.toPlainText()

    # Người dùng mở lại console
    screen.set_console_visible(True)

    # Provider phải được gọi để lấy snapshot
    mock_provider.assert_called_with("all")

    # Log mới từ backend phải hiển thị đầy đủ trên giao diện
    content = screen.txt_layer_log.toPlainText()
    assert "Hardware initialized" in content
    assert "Nav2 stack online" in content
    assert "SLAM graph optimized" in content

    # Test đổi filter layer: tự động kéo snapshot của layer đó
    mock_provider.return_value = ["[10:00:05] [LAYER2] Local costmap updated"]
    screen._set_log_filter("layer2")
    mock_provider.assert_called_with("layer2")
    assert "Local costmap updated" in screen.txt_layer_log.toPlainText()


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


def test_scenario_5_lazy_map_rebuild_on_activation(app):
    """5. P1 Fix: Không build ảnh map khi widget ẩn; chỉ dựng khi active."""
    icons = {
        "home": Path("/nonexistent/home.png"),
        "back": Path("/nonexistent/back.png"),
        "emergency": Path("/nonexistent/emergency.png"),
        "goal": Path("/nonexistent/goal.png"),
    }
    slam = SlamMapPage(icons=icons)

    # Deactivate page (simulate user being on Chat or Idle)
    slam.set_active(False)
    assert not slam._is_active
    assert not slam.live_map_widget._is_active

    fake_map = {
        "preview_width": 20,
        "preview_height": 20,
        "resolution": 0.05,
        "origin": (0.0, 0.0, 0.0),
        "cells": [0] * 400,
    }

    # Stream bản đồ thời gian thực tới SLAM page
    slam.set_map(fake_map)

    # Kiểm tra: Khi inactive, cờ dirty phải bật và hoàn toàn KHÔNG dựng ảnh (QImage null)
    assert slam.live_map_widget._map_dirty is True
    assert slam.live_map_widget._map_image.isNull()

    # Kích hoạt lại trang khi người dùng chuyển về tab SLAM
    slam.set_active(True)
    assert slam._is_active
    assert slam.live_map_widget._is_active
    # Cờ dirty đã được xử lý và ảnh đã được dựng thành công
    assert slam.live_map_widget._map_dirty is False
    assert not slam.live_map_widget._map_image.isNull()
    assert slam.live_map_widget._map_image.width() == 20
    assert slam.live_map_widget._map_image.height() == 20


def test_scenario_6_system_config_subpage_telemetry_isolation(app):
    """6. P1 Fix: Chỉ render subpage đang chọn khi telemetry cập nhật."""
    page = SystemConfigPage()
    page.set_active(True)

    overview_calls = []
    sensors_calls = []

    orig_overview = page.overview_screen.update_telemetry
    orig_sensors = page.sensors_screen.update_telemetry

    def spy_overview(*args, **kwargs):
        overview_calls.append(True)
        return orig_overview(*args, **kwargs)

    def spy_sensors(*args, **kwargs):
        sensors_calls.append(True)
        return orig_sensors(*args, **kwargs)

    page.overview_screen.update_telemetry = spy_overview
    page.sensors_screen.update_telemetry = spy_sensors

    try:
        # Khi đang ở Overview (index 0)
        assert page.stack.currentIndex() == 0
        snap = {
            "cam_front_hz": 15.0,
            "cam_rear_hz": 15.0,
            "cam_ai_hz": 10.0,
            "lidar_hz": 12.0,
            "odom_hz": 30.0,
            "tf_map_odom_ok": True,
            "tf_odom_base_ok": True,
        }
        page.update_sensor_telemetry(snap)

        # Overview phải được cập nhật, nhưng Sensors screen KHÔNG được gọi
        assert len(overview_calls) == 1
        assert len(sensors_calls) == 0

        # Chuyển sang Sensors screen (index 1)
        overview_calls.clear()
        sensors_calls.clear()
        page._navigate_to_module("sensors")
        assert page.stack.currentIndex() == 1
        # Lúc chuyển tab, _render_current_subpage sẽ nạp dữ liệu cho sensors screen
        assert len(sensors_calls) == 1

        # Khi tiếp tục nhận telemetry ở Sensors screen: Sensors được gọi, Overview KHÔNG được gọi
        overview_calls.clear()
        sensors_calls.clear()
        page.update_sensor_telemetry(snap)
        assert len(sensors_calls) == 1
        assert len(overview_calls) == 0
    finally:
        page.stop()


def test_scenario_7_stop_layer_stopping_state_and_race_prevention(app, monkeypatch):
    """7. P1 Fix: Trạng thái STOPPING và khóa chống khởi động trùng race condition."""
    import threading
    from robot_ui.kiosk.ros_node import KioskRosNode

    # Giả lập process chưa thoát ngay, cần thời gian trong background thread
    proc_stopped_event = threading.Event()

    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.side_effect = lambda: 0 if proc_stopped_event.is_set() else None

    # Tránh gọi os.getpgid và os.killpg trên pid giả lập
    monkeypatch.setattr(os, "getpgid", lambda pid: 12345)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    mock_logger = MagicMock()

    class DummyRosNode:
        CANONICAL_LAYER_MAP = KioskRosNode.CANONICAL_LAYER_MAP
        LAYER_ALIAS_MAP = KioskRosNode.LAYER_ALIAS_MAP
        _canonical_layer_key = KioskRosNode._canonical_layer_key
        _layer_procs = {"layer1": mock_proc, "layer_hardware": mock_proc}
        _stopping_layers = set()
        _layer_log_buffers = {"layer1": [], "all": []}
        _console_active = False

        def get_logger(self):
            return mock_logger

        signals = MagicMock()
        _record_layer_log = MagicMock()
        start_layer = KioskRosNode.start_layer
        stop_layer = KioskRosNode.stop_layer

    node = DummyRosNode()

    t0 = time.monotonic()
    node.stop_layer("layer1")
    dt = time.monotonic() - t0

    # 1. Trả về ngay lập tức (< 50ms), không block main thread
    assert dt < 0.05, f"stop_layer blocked for {dt*1000:.1f}ms!"

    # 2. Layer phải được ghi nhận là STOPPING ngay
    assert "layer1" in node._stopping_layers

    # 3. Signals phát trạng thái STOPPING
    node.signals.layer_status_changed.emit.assert_called()
    called_dict = node.signals.layer_status_changed.emit.call_args[0][0]
    assert called_dict.get("layer1") == "STOPPING"

    # 4. Khi đang STOPPING, lệnh start_layer phải bị từ chối và log cảnh báo
    node.start_layer("layer1")
    mock_logger.warn.assert_called()
    assert "STOPPING" in mock_logger.warn.call_args[0][0]

    # 5. Cho phép background thread hoàn tất dừng process
    proc_stopped_event.set()
    time.sleep(0.15)
    # Lúc này layer1 đã được dọn khỏi stopping_layers và layer_procs
    assert "layer1" not in node._stopping_layers
    assert "layer1" not in node._layer_procs


def test_scenario_8_souta_video_simd_resize(app):
    """8. P1 Fix: Tối ưu video Souta bằng C++/SIMD resize và cache tỷ lệ."""
    video_widget = VideoBackground()
    video_widget.resize(640, 480)
    video_widget._display_scale = 0.8
    video_widget._frame_aspect = 16.0 / 9.0
    video_widget._update_target_dims()

    # Kích thước target phải được cache chuẩn theo 16:9
    assert video_widget._cached_target_w > 0
    assert video_widget._cached_target_h > 0
    ratio = video_widget._cached_target_w / video_widget._cached_target_h
    assert abs(ratio - 16.0 / 9.0) < 0.1

    # Tạo một dummy numpy frame 1920x1080
    dummy_frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    dummy_frame[:, :] = (120, 150, 180)

    # Giả lập capture trả về frame
    mock_capture = MagicMock()
    mock_capture.read.return_value = (True, dummy_frame)
    video_widget._capture = mock_capture

    # Đọc frame: xử lý resize và chuyển đổi pixmap mượt mà
    video_widget._read_frame()
    assert not video_widget._pixmap.isNull()
    assert video_widget._pixmap.width() == video_widget._cached_target_w
    assert video_widget._pixmap.height() == video_widget._cached_target_h


def test_scenario_9_ros2_contracts_intact(app):
    """9. Không thay đổi chức năng Start/Stop, Nav2, SLAM, Patrol, E-Stop."""
    icons = {"goal": Path("/nonexistent/goal.png")}
    nav = NavigationPage(icons=icons)

    assert hasattr(nav, "start_requested")
    assert hasattr(nav, "cancel_requested")
    assert hasattr(nav, "start_patrol_requested")
    assert hasattr(nav, "pause_patrol_requested")
    assert hasattr(nav, "emergency_stop_requested")

    seen_estop = []
    nav.emergency_stop_requested.connect(lambda: seen_estop.append(True))
    nav.emergency_button.click()
    assert seen_estop == [True]


def test_scenario_10_clean_teardown_no_exceptions(app):
    """10. Không xuất hiện exception khi đóng Kiosk hoặc subprocess kết thúc."""
    nav = NavigationPage(icons={"goal": Path("/nonexistent/goal.png")})
    slam = SlamMapPage(icons={"goal": Path("/nonexistent/goal.png")})
    cfg = SystemConfigPage()

    nav.close()
    slam.close()
    cfg.stop()
