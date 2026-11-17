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
import signal
import threading
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


class MockLifecycleRosNode:
    """Mock KioskRosNode phục vụ kiểm thử vòng đời tiến trình thread-safe."""
    from robot_ui.kiosk.ros_node import KioskRosNode, LayerState

    CANONICAL_LAYER_MAP = KioskRosNode.CANONICAL_LAYER_MAP
    LAYER_ALIAS_MAP = KioskRosNode.LAYER_ALIAS_MAP
    _canonical_layer_key = KioskRosNode._canonical_layer_key
    _set_layer_state = KioskRosNode._set_layer_state
    get_layer_state = KioskRosNode.get_layer_state
    _terminate_process_group = staticmethod(KioskRosNode._terminate_process_group)
    _launch_worker = KioskRosNode._launch_worker
    _stop_worker = KioskRosNode._stop_worker
    start_layer = KioskRosNode.start_layer
    stop_layer = KioskRosNode.stop_layer
    _build_layer_cmd = KioskRosNode._build_layer_cmd
    _publish_system_telemetry = KioskRosNode._publish_system_telemetry
    cleanup_ros_processes = KioskRosNode.cleanup_ros_processes

    def __init__(self):
        import threading
        from robot_ui.kiosk.ros_node import LayerState

        self._lifecycle_lock = threading.RLock()
        self._layer_states = {
            "layer1": LayerState.STOPPED,
            "layer2": LayerState.STOPPED,
            "layer3": LayerState.STOPPED,
            "layer4": LayerState.STOPPED,
            "layer5": LayerState.STOPPED,
        }
        self._layer_procs = {}
        self._starting_layers = set()
        self._cancel_starting_layers = set()
        self._stopping_layers = set()
        self._error_layers = set()
        self._active_launch_tokens = {}
        self._layer_log_buffers = {k: [] for k in ("layer1", "layer2", "layer3", "layer4", "layer5", "all")}
        self._console_active = False
        self._force_swap_ports = False
        self._slam_proc = None
        self._localization_proc = None
        self.signals = MagicMock()
        self._logger = MagicMock()
        self._last_ai_rx = 0.0
        self._yolo_active = False
        self._people_count = 0
        self._is_waving = False
        self._closest_person_dist = None
        self._dbg_lock = threading.Lock()
        self._topic_stats = {}
        self._tf_buffer = MagicMock()
        self._tf_buffer.can_transform.return_value = False
        self._battery_pct = 100
        self._battery_smooth_v = 24.0
        self._battery_raw_v = 24.0
        self._battery_source = "test"
        self._water_liter = 1.0
        self._kettle_present = True

    def get_logger(self):
        return self._logger

    def _get_ws_prefix(self):
        return ""

    def _record_layer_log(self, layer: str, text: str):
        pass


def test_scenario_7_lifecycle_starting_stopping_error_and_race_prevention(app, monkeypatch):
    """7. P0/P1 Fix: Vòng đời tiến trình STARTING, STOPPING, ERROR và ngăn chặn race condition."""
    import threading
    from robot_ui.kiosk.ros_node import LayerState

    proc_stopped_event = threading.Event()

    mock_proc = MagicMock()
    mock_proc.pid = 12345
    mock_proc.poll.side_effect = lambda: 0 if proc_stopped_event.is_set() else None

    monkeypatch.setattr(os, "getpgid", lambda pid: 12345)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    node = MockLifecycleRosNode()
    node._set_layer_state("layer1", LayerState.RUNNING)
    node._layer_procs["layer1"] = mock_proc
    node._layer_procs["layer_hardware"] = mock_proc

    # A. Thử dừng khi tiến trình đang chạy
    t0 = time.monotonic()
    node.stop_layer("layer1")
    dt = time.monotonic() - t0

    assert dt < 0.05, f"stop_layer blocked for {dt*1000:.1f}ms!"
    assert "layer1" in node._stopping_layers
    assert node.get_layer_state("layer1") == LayerState.STOPPING

    # B. Signals phát trạng thái STOPPING
    node.signals.layer_status_changed.emit.assert_called()
    called_dict = node.signals.layer_status_changed.emit.call_args_list[0][0][0]
    assert called_dict.get("layer1") == "STOPPING"

    # C. Trong lúc STOPPING, lệnh start_layer phải bị từ chối
    node.start_layer("layer1")
    node.get_logger().warn.assert_called()
    assert "STOPPING" in node.get_logger().warn.call_args[0][0]

    # D. Cho phép background thread dừng tiến trình thành công
    proc_stopped_event.set()
    time.sleep(0.15)
    assert "layer1" not in node._stopping_layers
    assert "layer1" not in node._layer_procs
    assert node.get_layer_state("layer1") == LayerState.STOPPED

    # E. Test trường hợp tiến trình không chịu chết -> chuyển sang ERROR, giữ handle
    mock_stuck_proc = MagicMock()
    mock_stuck_proc.pid = 54321
    mock_stuck_proc.poll.return_value = None  # Không bao giờ thoát
    mock_stuck_proc.wait = MagicMock()
    node._set_layer_state("layer2", LayerState.RUNNING)
    node._layer_procs["layer2"] = mock_stuck_proc

    import time as real_time
    monkeypatch.setattr(time, "sleep", lambda s: None)

    node.stop_layer("layer2")
    t_end = real_time.monotonic() + 3.5
    while real_time.monotonic() < t_end:
        if "layer2" in node._error_layers:
            break
        real_time.sleep(0.02)

    # Vì tiến trình không thoát, nó KHÔNG được xóa khỏi _layer_procs, và phát trạng thái ERROR
    assert "layer2" in node._layer_procs
    assert "layer2" in node._error_layers
    assert node.get_layer_state("layer2") == LayerState.ERROR
    last_dict = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_dict.get("layer2") == "ERROR"

    # F. Khi đang ở ERROR, start_layer phải bị từ chối tuyệt đối để chặn start trùng
    node.get_logger().reset_mock()
    node.start_layer("layer2")
    node.get_logger().error.assert_called()
    assert "ERROR" in node.get_logger().error.call_args[0][0]


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
    spawned_procs = []
    launch_gate = threading.Event()
    killed_pids = []

    def mock_popen(cmd_list, **kwargs):
        launch_gate.wait(timeout=2.0)
        p = MagicMock()
        p.pid = 9000 + len(spawned_procs)
        p.poll.side_effect = lambda: 0 if any(sig in (signal.SIGINT, signal.SIGKILL) for _, sig in killed_pids) else None
        p.stdout = None
        spawned_procs.append(p)
        return p

    monkeypatch.setattr("subprocess.Popen", mock_popen)
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: killed_pids.append((pgid, sig)))

    node = MockLifecycleRosNode()

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
    assert node.get_logger().warn.called
    assert "STOPPING" in node.get_logger().warn.call_args[0][0]

    # Bước 3: Cho phép Popen hoàn tất; launch thread phát hiện token đã mất và tự hủy
    launch_gate.set()
    time.sleep(0.15)

    assert "layer1" not in node._starting_layers
    assert "layer1" not in node._stopping_layers
    # Không có tiến trình nào được đưa vào _layer_procs
    assert "layer1" not in node._layer_procs
    # Tiến trình được tạo ra phải bị dừng ngay sau Popen
    assert any(sig in (signal.SIGINT, signal.SIGKILL) for _, sig in killed_pids)


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


# ==============================================================================
# BỘ REGRESSION TESTS BẮT BUỘC T01 - T14: VÒNG ĐỜI TIẾN TRÌNH & THREAD-SAFE STATE MACHINE
# ==============================================================================

def test_lifecycle_t01_start_stop_before_popen(app, monkeypatch):
    """T01: START -> STOP trước Popen(): Không tạo process ngoài ý muốn."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_called = []

    def mock_popen(*args, **kwargs):
        popen_called.append(args)
        return MagicMock()

    monkeypatch.setattr("subprocess.Popen", mock_popen)

    worker_gate = threading.Event()
    orig_worker = node._launch_worker

    def hooked_worker(*args, **kwargs):
        worker_gate.wait(timeout=2.0)
        return orig_worker(*args, **kwargs)

    node._launch_worker = hooked_worker

    # 1. Bắt đầu start_layer
    node.start_layer("layer1")
    assert node.get_layer_state("layer1") == LayerState.STARTING
    assert "layer1" in node._active_launch_tokens

    # 2. Gọi stop_layer ngay lập tức trước khi worker chạy Popen
    node.stop_layer("layer1")
    assert "layer1" not in node._active_launch_tokens
    assert node.get_layer_state("layer1") == LayerState.STOPPING

    # 3. Cho worker chạy: phát hiện token mismatch/cờ hủy, thoát ngay mà KHÔNG Popen
    worker_gate.set()
    time.sleep(0.05)

    assert len(popen_called) == 0, "Popen must NOT be called when stopped before execution!"
    assert node.get_layer_state("layer1") == LayerState.STOPPED
    assert "layer1" not in node._layer_procs


def test_lifecycle_t02_start_stop_during_popen_rejects_second_start(app, monkeypatch):
    """T02: START -> STOP trong lúc Popen() chưa trả về: Không cho START lần 2."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    in_popen = threading.Event()
    release_popen = threading.Event()
    killed_pids = []
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: killed_pids.append((pgid, sig)))

    def mock_popen(*args, **kwargs):
        in_popen.set()
        release_popen.wait(timeout=2.0)
        p = MagicMock()
        p.pid = 2002
        p.poll.side_effect = lambda: 0 if any(sig in (signal.SIGINT, signal.SIGKILL) for _, sig in killed_pids) else None
        p.stdout = None
        return p

    monkeypatch.setattr("subprocess.Popen", mock_popen)

    # 1. Bắt đầu start
    node.start_layer("layer1")
    assert in_popen.wait(timeout=2.0), "Worker did not enter Popen"

    # 2. Gọi stop khi đang trong Popen
    node.stop_layer("layer1")
    assert node.get_layer_state("layer1") == LayerState.STOPPING

    # 3. Thử gọi START lần 2 khi đang STOPPING -> phải bị từ chối tuyệt đối
    node.get_logger().reset_mock()
    node.start_layer("layer1")
    node.get_logger().warn.assert_called()
    assert "STOPPING" in node.get_logger().warn.call_args[0][0]

    # 4. Giải phóng Popen
    release_popen.set()
    time.sleep(0.1)

    # Process bị hủy an toàn, trạng thái về STOPPED
    assert node.get_layer_state("layer1") == LayerState.STOPPED
    assert "layer1" not in node._layer_procs
    assert len(killed_pids) > 0


def test_lifecycle_t03_atomic_token_validation_and_registration(app, monkeypatch):
    """T03: STOP ngay sau token validation, trước process registration: Bảo vệ tính nguyên tử (P0-1)."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_done = threading.Event()
    step3_gate = threading.Event()
    killed_pids = []
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: killed_pids.append((pgid, sig)))

    mock_proc = MagicMock()
    mock_proc.pid = 3003
    mock_proc.poll.side_effect = lambda: 0 if any(sig in (signal.SIGINT, signal.SIGKILL) for _, sig in killed_pids) else None
    mock_proc.stdout = None

    def mock_popen(*args, **kwargs):
        popen_done.set()
        step3_gate.wait(timeout=2.0)
        return mock_proc

    monkeypatch.setattr("subprocess.Popen", mock_popen)

    node.start_layer("layer1")
    assert popen_done.wait(timeout=2.0)

    # Khi Popen chuẩn bị trả về, stop_layer được gọi và thu hồi token trước Step 3
    node.stop_layer("layer1")
    assert node.get_layer_state("layer1") == LayerState.STOPPING
    assert "layer1" not in node._active_launch_tokens

    # Cho phép Popen trả về và worker tiếp cận Step 3 dưới _lifecycle_lock
    step3_gate.set()
    time.sleep(0.1)

    # Worker dưới _lifecycle_lock thấy token không còn hợp lệ, không bao giờ đăng ký vào _layer_procs
    # Chuyển sang hủy tiến trình an toàn và cập nhật STOPPED
    assert "layer1" not in node._layer_procs
    assert node.get_layer_state("layer1") == LayerState.STOPPED
    assert any(sig in (signal.SIGINT, signal.SIGKILL) for _, sig in killed_pids)


def test_lifecycle_t04_token_revoked_after_popen_confirms_dead_before_stopped(app, monkeypatch):
    """T04: Token bị thu hồi sau Popen(): Xác nhận tiến trình chết trước khi chuyển STOPPED (P0-2)."""
    node = MockLifecycleRosNode()
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    mock_proc = MagicMock()
    mock_proc.pid = 4004
    allow_exit = threading.Event()
    mock_proc.poll.side_effect = lambda: 0 if allow_exit.is_set() else None
    mock_proc.stdout = None

    # Khi allow_exit chưa set, terminate_process_group phải báo chưa chết
    is_dead = node._terminate_process_group(mock_proc, timeout=0.05)
    assert not is_dead, "Process must NOT be reported dead while poll() is None!"

    # Khi allow_exit được set (process thực sự kết thúc), mới được báo dead
    allow_exit.set()
    is_dead = node._terminate_process_group(mock_proc, timeout=0.05)
    assert is_dead, "Process must be reported dead once poll() returns exit code!"


def test_lifecycle_t05_sigkill_failure_transitions_to_error_and_blocks_start(app, monkeypatch):
    """T05: SIGKILL thất bại: Chuyển sang ERROR, giữ process handle, chặn tuyệt đối START mới."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    mock_unkillable = MagicMock()
    mock_unkillable.pid = 5005
    mock_unkillable.poll.return_value = None  # Không bao giờ chết
    mock_unkillable.wait = MagicMock()

    node._set_layer_state("layer1", LayerState.RUNNING)
    node._layer_procs["layer1"] = mock_unkillable
    node._layer_procs["layer_hardware"] = mock_unkillable

    # Mock terminate_process_group thất bại
    node._terminate_process_group = lambda proc, timeout=1.5: False

    node.stop_layer("layer1")
    time.sleep(0.05)

    # Trạng thái phải là ERROR và handle phải được giữ lại
    assert node.get_layer_state("layer1") == LayerState.ERROR
    assert "layer1" in node._layer_procs
    assert node._layer_procs["layer1"] is mock_unkillable

    # Thử START khi đang ERROR -> PHẢI BỊ TỪ CHỐI
    node.get_logger().reset_mock()
    node.start_layer("layer1")
    node.get_logger().error.assert_called()
    assert "ERROR" in node.get_logger().error.call_args[0][0]


def test_lifecycle_t06_timeout_failure_transitions_to_error_without_fake_stopped(app, monkeypatch):
    """T06: Process không thoát sau timeout: Chuyển ERROR, không bao giờ báo STOPPED giả."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    monkeypatch.setattr(os, "getpgid", lambda pid: pid)
    monkeypatch.setattr(os, "killpg", lambda pgid, sig: None)

    mock_hanging_proc = MagicMock()
    mock_hanging_proc.pid = 6006
    mock_hanging_proc.poll.return_value = None
    mock_hanging_proc.wait = MagicMock()

    node._set_layer_state("layer2", LayerState.RUNNING)
    node._layer_procs["layer2"] = mock_hanging_proc

    # Mock timeout thất bại
    node._terminate_process_group = lambda proc, timeout=1.5: False

    node.stop_layer("layer2")
    time.sleep(0.05)

    assert node.get_layer_state("layer2") == LayerState.ERROR
    emitted_payloads = [call[0][0] for call in node.signals.layer_status_changed.emit.call_args_list]
    for p in emitted_payloads:
        if "layer2" in p:
            assert p["layer2"] != LayerState.STOPPED.value, "STOPPED must never be emitted when proc is hanging!"
    assert emitted_payloads[-1].get("layer2") == LayerState.ERROR.value


def test_lifecycle_t07_start_when_running_rejects_without_duplicate_spawn(app, monkeypatch):
    """T07: START khi RUNNING: Từ chối, không spawn thêm process."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_called = []
    monkeypatch.setattr("subprocess.Popen", lambda *args, **kwargs: popen_called.append(args))

    mock_proc = MagicMock()
    mock_proc.pid = 7007
    mock_proc.poll.return_value = None
    node._set_layer_state("layer1", LayerState.RUNNING)
    node._layer_procs["layer1"] = mock_proc

    node.start_layer("layer1")
    assert len(popen_called) == 0
    node.get_logger().warn.assert_called()
    assert "RUNNING" in node.get_logger().warn.call_args[0][0]


def test_lifecycle_t08_start_when_stopping_rejects(app, monkeypatch):
    """T08: START khi STOPPING: Từ chối."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_called = []
    monkeypatch.setattr("subprocess.Popen", lambda *args, **kwargs: popen_called.append(args))

    node._set_layer_state("layer1", LayerState.STOPPING)
    node.start_layer("layer1")
    assert len(popen_called) == 0
    node.get_logger().warn.assert_called()
    assert "STOPPING" in node.get_logger().warn.call_args[0][0]


def test_lifecycle_t09_start_when_error_rejects(app, monkeypatch):
    """T09: START khi ERROR: Từ chối."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_called = []
    monkeypatch.setattr("subprocess.Popen", lambda *args, **kwargs: popen_called.append(args))

    node._set_layer_state("layer1", LayerState.ERROR)
    node.start_layer("layer1")
    assert len(popen_called) == 0
    node.get_logger().error.assert_called()
    assert "ERROR" in node.get_logger().error.call_args[0][0]


def test_lifecycle_t10_stop_during_starting_telemetry_shows_stopping(app, monkeypatch):
    """T10: STOP trong STARTING: Telemetry phải hiển thị STOPPING."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    monkeypatch.setattr("subprocess.Popen", lambda *args, **kwargs: MagicMock())

    node._set_layer_state("layer1", LayerState.STARTING)
    assert node.get_layer_state("layer1") == LayerState.STARTING

    node.stop_layer("layer1")
    assert node.get_layer_state("layer1") == LayerState.STOPPING

    node._publish_system_telemetry()
    last_emit = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_emit.get("layer1") == "STOPPING"
    assert last_emit.get("layer_hardware") == "STOPPING"
    assert last_emit.get("layer1") != "STARTING"
    assert last_emit.get("layer1") != "RUNNING"


def test_lifecycle_t11_telemetry_timer_never_overwrites_error_with_running(app, monkeypatch):
    """T11: Timer cập nhật khi ERROR: Không ghi đè ERROR thành RUNNING."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    mock_stuck = MagicMock()
    mock_stuck.poll.return_value = None

    node._set_layer_state("layer2", LayerState.ERROR)
    node._layer_procs["layer2"] = mock_stuck

    node._publish_system_telemetry()
    last_emit = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_emit.get("layer2") == "ERROR"
    assert last_emit.get("layer_navigation") == "ERROR"
    assert node.get_layer_state("layer2") == LayerState.ERROR


def test_lifecycle_t12_unexpected_process_death_detected_and_updated(app, monkeypatch):
    """T12: Process chết ngoài ý muốn: Phát hiện exit và cập nhật trạng thái."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    mock_crashed = MagicMock()
    mock_crashed.poll.return_value = 139  # Segmentation fault exit code

    node._set_layer_state("layer3", LayerState.RUNNING)
    node._layer_procs["layer3"] = mock_crashed
    node._layer_procs["layer_slam"] = mock_crashed

    node._publish_system_telemetry()
    assert node.get_layer_state("layer3") == LayerState.ERROR
    assert "layer3" not in node._layer_procs
    last_emit = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_emit.get("layer3") == "ERROR"
    assert last_emit.get("layer_slam") == "ERROR"

    # Trường hợp thoát sạch (exit code 0)
    mock_clean = MagicMock()
    mock_clean.poll.return_value = 0
    node._set_layer_state("layer4", LayerState.RUNNING)
    node._layer_procs["layer4"] = mock_clean

    node._publish_system_telemetry()
    assert node.get_layer_state("layer4") == LayerState.STOPPED
    assert "layer4" not in node._layer_procs
    last_emit2 = node.signals.layer_status_changed.emit.call_args[0][0]
    assert last_emit2.get("layer4") == "STOPPED"


def test_lifecycle_t13_canonical_and_alias_keys_share_same_state_machine(app, monkeypatch):
    """T13: Nhận alias layer1 và layer_hardware: Cùng tham chiếu một layer, không spawn trùng."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()
    popen_called = []
    launch_gate = threading.Event()

    def mock_popen(*args, **kwargs):
        popen_called.append(args)
        launch_gate.wait(timeout=2.0)
        p = MagicMock()
        p.pid = 1300
        p.poll.return_value = None
        p.stdout = None
        return p

    monkeypatch.setattr("subprocess.Popen", mock_popen)

    # Start bằng alias
    node.start_layer("layer_hardware")
    assert node.get_layer_state("layer1") == LayerState.STARTING
    assert node.get_layer_state("layer_hardware") == LayerState.STARTING

    # Start bằng canonical name trong khi đang starting -> bị từ chối
    node.start_layer("layer1")
    node.get_logger().warn.assert_called()
    assert len(popen_called) == 1

    launch_gate.set()
    time.sleep(0.1)
    assert node.get_layer_state("layer1") == LayerState.RUNNING
    assert node.get_layer_state("layer_hardware") == LayerState.RUNNING


def test_lifecycle_t14_stale_launch_thread_does_not_corrupt_newer_generation(app, monkeypatch):
    """T14: Luồng launch cũ kết thúc sau generation mới: Không xóa state hoặc handle của generation mới."""
    from robot_ui.kiosk.ros_node import LayerState

    node = MockLifecycleRosNode()

    # Gen 2 đang RUNNING với process mới
    proc_gen2 = MagicMock()
    proc_gen2.pid = 9999
    proc_gen2.poll.return_value = None
    token_gen2 = "token_gen_2"

    node._set_layer_state("layer1", LayerState.RUNNING)
    node._layer_procs["layer1"] = proc_gen2
    node._active_launch_tokens["layer1"] = token_gen2

    # Giả lập worker gen 1 cũ (token_gen_1_stale) chạy _stop_worker
    proc_gen1_old = MagicMock()
    proc_gen1_old.pid = 1111
    token_gen1_stale = "token_gen_1_stale"

    # Gọi _stop_worker với token cũ
    node._stop_worker("layer1", "layer1", "layer_hardware", proc_gen1_old, token=token_gen1_stale)

    # Đảm bảo state và handle của gen 2 KHÔNG bị ảnh hưởng
    assert node.get_layer_state("layer1") == LayerState.RUNNING
    assert node._layer_procs.get("layer1") is proc_gen2
    assert node._active_launch_tokens.get("layer1") == token_gen2
    node.get_logger().warn.assert_called()
    assert "Generation cũ" in node.get_logger().warn.call_args[0][0]
