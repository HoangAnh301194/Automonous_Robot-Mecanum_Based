"""Regression tests for Kiosk UI performance optimization and comprehensive lifecycle fixes.

Verifies:
1. Live log updating when ROS 2 System console is visible.
2. Log buffering without QTextEdit DOM churn when hidden.
3. Chronological log snapshot restoration upon reopening via backend provider (P0 fix).
4. Navigation pose, LiDAR scan, and obstacle points caching & refresh.
5. Lazy map rebuild: no image processing while inactive, rebuilds on activation (P1 fix).
6. Strictly single visible subpage telemetry rendering in System Config (P1 fix).
7. Non-blocking stop_layer with STARTING/STOPPING/ERROR state lifecycle (P0/P1 fix).
8. Souta video background using OpenCV C++/SIMD resizing and dimension caching (P1 fix).
9. Preservation of Start/Stop, Nav2, SLAM, Patrol, and E-Stop contracts.
10. Clean teardown and cooperative shutdown without unhandled thread exceptions.
11. cancel_navigation() emits dict with status and state (ROS 2 contract fix).
12. SHOW_5S_INVITE popup does not auto-trigger service wait mode (Service logic fix).
13. load_map_yaml() without waypoints does not overwrite global config files (Data protection fix).
14. apply_localization_map() passes both map_yaml and map_graph for SLAM Toolbox (Nav2 fix).
15. Token-based lifecycle prevents START-STOP-START race condition (P0 concurrency fix).
16. SHOW_SERVING decouples UI selection from command emitting (P1 loop fix).
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

    mock_provider = MagicMock(return_value=backend_logs)
    screen.set_log_provider(mock_provider)

    assert "Hardware initialized" not in screen.txt_layer_log.toPlainText()

    screen.set_console_visible(True)
    mock_provider.assert_called_with("all")

    content = screen.txt_layer_log.toPlainText()
    assert "Hardware initialized" in content
    assert "Nav2 stack online" in content
    assert "SLAM graph optimized" in content

    mock_provider.return_value = ["[10:00:05] [LAYER2] Local costmap updated"]
    screen._set_log_filter("layer2")
    mock_provider.assert_called_with("layer2")
    assert "Local costmap updated" in screen.txt_layer_log.toPlainText()


def test_scenario_4_navigation_pose_scan_obstacle_caching(app):
    """4. Navigation: pose, LiDAR, obstacle cập nhật mới nhất."""
    icons = {"goal": Path("/nonexistent/goal.png")}
    nav = NavigationPage(icons=icons)

    nav.set_active(False)
    assert not nav._is_active
    assert not nav.map_widget._is_active

    nav.set_pose({"x": 1.25, "y": 3.45, "yaw": 0.78})
    nav.set_scan_points([(1.0, 2.0), (3.0, 4.0)])
    nav.set_obstacle_points("front", [(1.5, 2.5)])

    assert nav.map_widget._pose == {"x": 1.25, "y": 3.45, "yaw": 0.78}
    assert nav.map_widget._scan_points == [(1.0, 2.0), (3.0, 4.0)]
    assert nav.map_widget._front_obs_points == [(1.5, 2.5)]

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

    slam.set_map(fake_map)
    assert slam.live_map_widget._map_dirty is True
    assert slam.live_map_widget._map_image.isNull()

    slam.set_active(True)
    assert slam._is_active
    assert slam.live_map_widget._is_active
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

        assert len(overview_calls) == 1
        assert len(sensors_calls) == 0

        overview_calls.clear()
        sensors_calls.clear()
        page._navigate_to_module("sensors")
        assert page.stack.currentIndex() == 1
        assert len(sensors_calls) == 1

        overview_calls.clear()
        sensors_calls.clear()
        page.update_sensor_telemetry(snap)
        assert len(sensors_calls) == 1
        assert len(overview_calls) == 0
    finally:
        page.stop()


def test_scenario_7_lifecycle_starting_stopping_error_and_race_prevention(app, monkeypatch):
    """7. P0/P1 Fix: Vòng đời tiến trình STARTING, STOPPING, ERROR và ngăn chặn race condition."""
    import threading
    from robot_ui.kiosk.ros_node import KioskRosNode

    proc_stopped_event = threading.Event()

    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.side_effect = lambda: 0 if proc_stopped_event.is_set() else None

    monkeypatch.setattr(os, "getpgid", lambda pid: 12345)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    mock_logger = MagicMock()

    class DummyRosNode:
        CANONICAL_LAYER_MAP = KioskRosNode.CANONICAL_LAYER_MAP
        LAYER_ALIAS_MAP = KioskRosNode.LAYER_ALIAS_MAP
        _canonical_layer_key = KioskRosNode._canonical_layer_key
        def _get_ws_prefix(self):
            return ""
        _layer_procs = {"layer1": mock_proc, "layer_hardware": mock_proc}
        _starting_layers = set()
        _cancel_starting_layers = set()
        _stopping_layers = set()
        _error_layers = set()
        _active_launch_tokens = {}
        _layer_log_buffers = {"layer1": [], "all": []}
        _console_active = False

        def get_logger(self):
            return mock_logger

        signals = MagicMock()
        _record_layer_log = MagicMock()
        start_layer = KioskRosNode.start_layer
        stop_layer = KioskRosNode.stop_layer

    node = DummyRosNode()

    # A. Thử dừng khi tiến trình đang chạy
    t0 = time.monotonic()
    node.stop_layer("layer1")
    dt = time.monotonic() - t0

    assert dt < 0.05, f"stop_layer blocked for {dt*1000:.1f}ms!"
    assert "layer1" in node._stopping_layers

    # B. Signals phát trạng thái STOPPING
    node.signals.layer_status_changed.emit.assert_called()
    called_dict = node.signals.layer_status_changed.emit.call_args[0][0]
    assert called_dict.get("layer1") == "STOPPING"

    # C. Trong lúc STOPPING, lệnh start_layer phải bị từ chối
    node.start_layer("layer1")
    mock_logger.warn.assert_called()
    assert "STOPPING" in mock_logger.warn.call_args[0][0]

    # D. Cho phép background thread dừng tiến trình thành công
    proc_stopped_event.set()
    time.sleep(0.15)
    assert "layer1" not in node._stopping_layers
    assert "layer1" not in node._layer_procs

    # E. Test trường hợp tiến trình không chịu chết -> chuyển sang ERROR, giữ handle
    mock_stuck_proc = MagicMock()
    mock_stuck_proc.pid = 54321
    mock_stuck_proc.poll.return_value = None  # Không bao giờ thoát
    mock_stuck_proc.wait = MagicMock()
    node._layer_procs["layer2"] = mock_stuck_proc

    import time as real_time
    monkeypatch.setattr(time, "sleep", lambda s: None)

    node.stop_layer("layer2")
    t_end = real_time.monotonic() + 1.5
    while real_time.monotonic() < t_end:
        if "layer2" in node._error_layers:
            break
        real_time.sleep(0.02)

    # Vì tiến trình không thoát, nó KHÔNG được xóa khỏi _layer_procs, và phát trạng thái ERROR
    assert "layer2" in node._layer_procs
    assert "layer2" in node._error_layers
    last_dict = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_dict.get("layer2") == "ERROR"

    # F. Khi đang ở ERROR, start_layer phải bị từ chối tuyệt đối để chặn start trùng
    mock_logger.reset_mock()
    node.start_layer("layer2")
    mock_logger.error.assert_called()
    assert "ERROR" in mock_logger.error.call_args[0][0]


def test_scenario_8_souta_video_simd_resize(app):
    """8. P1 Fix: Tối ưu video Souta bằng C++/SIMD resize và cache tỷ lệ."""
    video_widget = VideoBackground()
    video_widget.resize(640, 480)
    video_widget._display_scale = 0.8
    video_widget._frame_aspect = 16.0 / 9.0
    video_widget._update_target_dims()

    assert video_widget._cached_target_w > 0
    assert video_widget._cached_target_h > 0
    ratio = video_widget._cached_target_w / video_widget._cached_target_h
    assert abs(ratio - 16.0 / 9.0) < 0.1

    dummy_frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
    dummy_frame[:, :] = (120, 150, 180)

    mock_capture = MagicMock()
    mock_capture.read.return_value = (True, dummy_frame)
    video_widget._capture = mock_capture

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


def test_scenario_11_cancel_navigation_emits_dict(app):
    """11. P0 Fix: cancel_navigation() phát dict đúng contract pyqtSignal(dict)."""
    from robot_ui.kiosk.ros_node import KioskRosNode

    class DummyRosNode:
        cmd_vel_pub = MagicMock()
        _goal_handle = None
        signals = MagicMock()

        def get_logger(self):
            return MagicMock()

        cancel_navigation = KioskRosNode.cancel_navigation

    node = DummyRosNode()
    node.cancel_navigation()

    # Signal navigation_status_changed phải nhận 1 tham số dạng dict
    node.signals.navigation_status_changed.emit.assert_called_once()
    payload = node.signals.navigation_status_changed.emit.call_args[0][0]
    assert isinstance(payload, dict)
    assert payload.get("status") == "CANCELLED"


def test_scenario_12_service_invite_popup_no_auto_serving(app):
    """12. P0 Fix: Lệnh SHOW_5S_INVITE không tự động kích hoạt wait serving mode."""
    from robot_ui.kiosk.window import KioskWindow

    # Mock service page và show_page
    win = MagicMock()
    win.service_page = MagicMock()
    win._active_page = "home"
    win._on_service_popup_cmd = lambda cmd: KioskWindow._on_service_popup_cmd(win, cmd)

    # 1. Nhận lệnh SHOW_5S_INVITE: Mở trang service nhưng KHÔNG gọi select_mode_programmatically
    win._on_service_popup_cmd("SHOW_5S_INVITE")
    win.show_page.assert_called_with("service")
    win.service_page.select_mode_programmatically.assert_not_called()

    # 2. Nhận lệnh SHOW_SERVING: Phải kích hoạt wait mode với emit_signal=False để tránh phản hồi vòng lặp lệnh
    win._on_service_popup_cmd("SHOW_SERVING")
    win.service_page.select_mode_programmatically.assert_called_with("wait", emit_signal=False)


def test_scenario_13_load_map_yaml_without_waypoints_protects_config(app, tmp_path):
    """13. P0 Fix: Nạp map chưa có waypoint không ghi đè rỗng locations.yaml/routes.yaml."""
    nav = NavigationPage(icons={"goal": Path("/nonexistent/goal.png")})

    # Giả lập mock _sync_active_config_files
    nav._sync_active_config_files = MagicMock()

    # Tạo một file map YAML tạm
    map_yaml = tmp_path / "new_empty_map.yaml"
    map_yaml.write_text("image: test.png\nresolution: 0.05\norigin: [0, 0, 0]\n")

    # Mock parse_nav_map_yaml để trả về dữ liệu hợp lệ
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(
            "robot_ui.kiosk.pages.navigation_page.parse_nav_map_yaml",
            lambda path: {"preview_width": 10, "preview_height": 10, "cells": [0]*100},
        )
        success = nav.load_map_yaml(map_yaml)

    assert success is True
    # Tuyệt đối KHÔNG được gọi _sync_active_config_files làm mất file route thật
    nav._sync_active_config_files.assert_not_called()


def test_scenario_14_apply_localization_map_passes_map_graph(app):
    """14. P1 Fix: apply_localization_map() truyền đầy đủ cả map_yaml và map_graph."""
    from robot_ui.kiosk.ros_node import KioskRosNode

    spawned_cmds = []

    class DummyRosNode:
        _localization_proc = None
        _selected_map_yaml = ""
        _selected_backend = ""
        stop_slam = MagicMock()
        def _get_ws_prefix(self):
            return ""

        def get_logger(self):
            return MagicMock()

        apply_localization_map = KioskRosNode.apply_localization_map

    node = DummyRosNode()

    # Mock subprocess.Popen để bắt command line
    def fake_popen(cmd_list, **kwargs):
        spawned_cmds.append(cmd_list[2])
        m = MagicMock()
        m.poll.return_value = 0
        return m

    with pytest.MonkeyPatch.context() as mp:
        mp.setattr("subprocess.Popen", fake_popen)
        node.apply_localization_map("/home/user/my_map/khoa_a1.yaml", backend="slam_toolbox")
        time.sleep(0.05)

    assert len(spawned_cmds) == 1
    cmd = spawned_cmds[0]
    assert "map_yaml:=/home/user/my_map/khoa_a1.yaml" in cmd
    assert "map_graph:=/home/user/my_map/khoa_a1" in cmd


def test_scenario_15_token_based_race_prevention_start_stop_start(app, monkeypatch):
    """15. P0 Fix: Token-based lifecycle ngăn chặn race condition START-STOP-START."""
    import signal
    import threading
    from robot_ui.kiosk.ros_node import KioskRosNode

    spawned_procs = []
    launch_gate = threading.Event()
    killed_pids = []

    def mock_popen(cmd_list, **kwargs):
        launch_gate.wait(timeout=2.0)
        p = MagicMock()
        p.pid = 9000 + len(spawned_procs)
        p.poll.return_value = None
        p.stdout = None
        spawned_procs.append(p)
        return p

    monkeypatch.setattr("subprocess.Popen", mock_popen)
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: killed_pids.append((pgid, sig)))

    mock_logger = MagicMock()

    class DummyRosNode:
        CANONICAL_LAYER_MAP = KioskRosNode.CANONICAL_LAYER_MAP
        LAYER_ALIAS_MAP = KioskRosNode.LAYER_ALIAS_MAP
        _canonical_layer_key = KioskRosNode._canonical_layer_key

        def _get_ws_prefix(self):
            return ""

        _layer_procs = {}
        _starting_layers = set()
        _cancel_starting_layers = set()
        _stopping_layers = set()
        _error_layers = set()
        _active_launch_tokens = {}
        _layer_log_buffers = {"layer1": [], "all": []}
        _console_active = False

        def get_logger(self):
            return mock_logger

        signals = MagicMock()
        _record_layer_log = MagicMock()
        start_layer = KioskRosNode.start_layer
        stop_layer = KioskRosNode.stop_layer

    node = DummyRosNode()

    # Bước 1: Gọi start_layer("layer1") - tiến trình đang chờ ở launch_gate
    node.start_layer("layer1")
    token_1 = node._active_launch_tokens.get("layer1")
    assert token_1 is not None
    assert "layer1" in node._starting_layers

    # Bước 2: Ngay lập tức gọi stop_layer("layer1") khi thread chưa spawn xong
    node.stop_layer("layer1")
    # Token 1 phải bị vô hiệu hóa ngay lập tức
    assert "layer1" not in node._active_launch_tokens
    # Trạng thái chuyển sang STOPPING chờ thread hủy an toàn
    assert "layer1" in node._stopping_layers

    # Trong lúc đang STOPPING, lệnh start lần 2 phải bị từ chối
    node.start_layer("layer1")
    assert mock_logger.warn.called
    assert "STOPPING" in mock_logger.warn.call_args[0][0]

    # Bước 3: Cho phép Popen hoàn tất; launch thread phát hiện token đã mất và tự hủy
    launch_gate.set()
    time.sleep(0.15)

    assert "layer1" not in node._starting_layers
    assert "layer1" not in node._stopping_layers
    # Không có tiến trình nào được đưa vào _layer_procs
    assert "layer1" not in node._layer_procs
    # Tiến trình được tạo ra phải bị killpg ngay sau Popen
    assert any(sig == signal.SIGKILL for _, sig in killed_pids)


def test_scenario_16_service_page_select_mode_decoupling(app):
    """16. P1 Fix: select_mode_programmatically với emit_signal=False không phát tín hiệu."""
    from robot_ui.kiosk.pages.service_page import ServicePage

    page = ServicePage()
    emitted_modes = []
    page.mode_selected.connect(lambda m: emitted_modes.append(m))

    # 1. Gọi programmatic với emit_signal=False: nút được chọn nhưng KHÔNG emit
    page.select_mode_programmatically("wait", emit_signal=False)
    assert page.get_selected_mode() == "wait"
    assert page.btn_wait.property("selected") is True
    assert page.btn_continue.property("selected") is False
    assert len(emitted_modes) == 0

    # 2. Gọi programmatic với emit_signal=True: nút được chọn và CÓ emit
    page.select_mode_programmatically("continue", emit_signal=True)
    assert page.get_selected_mode() == "continue"
    assert page.btn_continue.property("selected") is True
    assert len(emitted_modes) == 1
    assert emitted_modes[0] == "continue"
