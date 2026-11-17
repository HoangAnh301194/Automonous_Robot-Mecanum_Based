"""test_kiosk_p0_p1_hotfixes.py - Verification tests for P0/P1 fixes on simulator_interface.

Covers:
- P0-1: Elimination of duplicate AMCL/Nav2/RViz2 processes and unified layer2 lifecycle.
- P0-2: Subprocess environment sanitization and unread pipe deadlock prevention.
- P0-3: UI render throttling (Pose 20 Hz, Scan 10 Hz, Obs 8 Hz), hidden map widget deactivation, log batching.
- P1-1: Qt/OpenCV xcb plugin path sanitization for RViz2.
- P1-2: Non-blocking SLAM stop and background workers.
"""
from __future__ import annotations

import os
import subprocess
import time
from unittest.mock import MagicMock, patch

import pytest
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.navigation_page import InteractiveNavMapWidget, NavigationPage
from robot_ui.kiosk.pages.system_status.ros2_system import ROS2SystemDetailScreen
from robot_ui.kiosk.ros_node import KioskRosNode, LayerState
from robot_ui.kiosk.signals import KioskSignals


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_p1_1_qt_qpa_plugin_path_sanitized():
    """P1-1: Kiểm tra biến QT_QPA_PLATFORM_PLUGIN_PATH bị loại bỏ để tránh crash RViz2."""
    assert os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH") is None

    # Giả lập môi trường có bị inject bởi cv2
    with patch.dict(os.environ, {"QT_QPA_PLATFORM_PLUGIN_PATH": "/some/fake/cv2/qt/plugins"}):
        clean_env = KioskRosNode._get_clean_env()
        assert "QT_QPA_PLATFORM_PLUGIN_PATH" not in clean_env


def test_p0_3_pose_emission_throttled_to_20hz(app):
    """P0-3: Tín hiệu pose_changed từ _emit_pose bị giới hạn tối đa 20 Hz (interval >= 0.05s)."""
    signals = KioskSignals()
    node = KioskRosNode.__new__(KioskRosNode)
    node.signals = signals
    node._pose = {}
    node._last_pose_emit_time = 0.0

    emitted_poses = []
    signals.pose_changed.connect(lambda p: emitted_poses.append(p))

    mock_pose = MagicMock()
    mock_pose.position.x = 1.0
    mock_pose.position.y = 2.0
    mock_pose.orientation.x = 0.0
    mock_pose.orientation.y = 0.0
    mock_pose.orientation.z = 0.0
    mock_pose.orientation.w = 1.0

    # Phát liên tục 10 lần trong vòng 5ms
    for _ in range(10):
        node._emit_pose(mock_pose, "map", "amcl")

    # Chỉ có duy nhất 1 emit đầu tiên lọt qua do thời gian < 0.05s
    assert len(emitted_poses) == 1


def test_p0_3_nav_map_widget_repaint_throttling(app):
    """P0-3: InteractiveNavMapWidget giới hạn tần suất repaint khi nhận telemetry dồn dập."""
    widget = InteractiveNavMapWidget()
    widget.set_active(True)

    update_calls = []
    widget.update = lambda: update_calls.append(time.monotonic())

    # 1. Test set_pose (tối đa 15 Hz -> interval >= 0.066s)
    update_calls.clear()
    for _ in range(10):
        widget.set_pose({"x": 1.0, "y": 2.0, "yaw": 0.0})
    assert len(update_calls) == 1

    # 2. Test set_scan_points (tối đa 10 Hz -> interval >= 0.10s)
    update_calls.clear()
    for _ in range(10):
        widget.set_scan_points([(1.0, 1.0), (2.0, 2.0)])
    assert len(update_calls) == 1

    # 3. Test set_obstacle_points (tối đa 8 Hz -> interval >= 0.125s)
    update_calls.clear()
    for _ in range(10):
        widget.set_obstacle_points("front", [(0.5, 0.5)])
    assert len(update_calls) == 1


def test_p0_3_navigation_page_inactive_tab_deactivated(app):
    """P0-3: Chỉ có widget bản đồ của tab đang hiển thị mới ở trạng thái _is_active=True."""
    page = NavigationPage(icons={})
    page.set_active(True)

    # Mặc định ở tab 0 (Single Nav)
    assert page.map_widget._is_active is True
    assert page.patrol_map_widget._is_active is False

    # Chuyển sang tab 1 (Patrol)
    page._set_active_mode(1)
    assert page.map_widget._is_active is False
    assert page.patrol_map_widget._is_active is True

    # Khi cả NavigationPage bị ẩn (set_active=False)
    page.set_active(False)
    assert page.map_widget._is_active is False
    assert page.patrol_map_widget._is_active is False


def test_p0_3_live_log_batching(app):
    """P0-3: ROS2SystemDetailScreen gom các dòng log vào _pending_render_lines và flush theo lô."""
    screen = ROS2SystemDetailScreen()
    screen.set_console_visible(True)
    screen._current_filter = "all"

    # Gọi 5 dòng liên tục
    for i in range(5):
        screen.append_layer_log("layer1", f"Batch line {i}")

    # Các dòng nằm trong hàng đợi chờ timer flush
    assert len(screen._pending_render_lines) == 5
    assert screen._batch_render_timer.isActive() is True

    # Gọi toPlainText() sẽ tự động flush các dòng chờ
    text = screen.txt_layer_log.toPlainText()
    assert "Batch line 0" in text
    assert "Batch line 4" in text
    assert len(screen._pending_render_lines) == 0


def test_p1_2_stop_slam_non_blocking_worker(app):
    """P1-2: stop_slam không block luồng gọi mà chạy ngầm qua background thread."""
    node = KioskRosNode.__new__(KioskRosNode)
    node.signals = KioskSignals()
    node._slam_proc = MagicMock()
    node._slam_proc.poll.return_value = None
    node._slam_proc.pid = 4321
    node._layer_procs = {"layer3": node._slam_proc}

    logger_mock = MagicMock()
    node.get_logger = lambda: logger_mock

    with patch("threading.Thread") as mock_thread_cls, patch("os.getpgid", return_value=4321):
        thread_instance = MagicMock()
        mock_thread_cls.return_value = thread_instance

        node.stop_slam()

        # Kiểm tra _slam_proc đã được gỡ bỏ khỏi main node ngay lập tức
        assert node._slam_proc is None
        assert "layer3" not in node._layer_procs

        # Kiểm tra worker thread được khởi tạo dưới dạng daemon
        mock_thread_cls.assert_called_once()
        _, kwargs = mock_thread_cls.call_args
        assert kwargs.get("daemon") is True
        thread_instance.start.assert_called_once()


def test_p0_1_build_layer_cmd_layer2_with_selected_map():
    """P0-1: _build_layer_cmd cho layer2 truyền chính xác map_yaml, map_graph và use_rviz:=false."""
    node = KioskRosNode.__new__(KioskRosNode)
    node._get_ws_prefix = lambda: "export ROS_DOMAIN_ID=94 && "
    node._selected_backend = "amcl"
    node._selected_map_yaml = "/tmp/test_map.yaml"

    with patch("os.path.exists", return_value=True):
        cmd = node._build_layer_cmd("layer2")

    assert "mo_hinh real_localization_nav2.launch.py" in cmd
    assert "localization_mode:=amcl" in cmd
    assert "map_yaml:=/tmp/test_map.yaml" in cmd
    assert "map_graph:=/tmp/test_map" in cmd
    assert "use_rviz:=false" in cmd


def test_cleanup_ros_processes_non_blocking():
    """Emergency Cleanup không block luồng gọi mà chạy ngầm qua background daemon thread."""
    import threading
    node = KioskRosNode.__new__(KioskRosNode)
    node.signals = KioskSignals()
    node._lifecycle_lock = threading.RLock()
    node._active_launch_tokens = {"layer1": "token1"}
    node._starting_layers = set()
    node._stopping_layers = set()
    node._error_layers = set()
    node._cancel_starting_layers = set()
    node._loc_session_token = ""
    from robot_ui.kiosk.ros_node import LocalizationState
    node._localization_state = LocalizationState.NOT_INITIALIZED
    node._layer_states = {"layer1": LayerState.RUNNING}
    node._layer_procs = {"layer1": MagicMock()}
    node._slam_proc = MagicMock()
    node._localization_proc = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    node._record_layer_log = MagicMock()
    node.LAYER_ALIAS_MAP = {"layer1": "layer_hardware", "layer_hardware": "layer1"}

    with patch("threading.Thread") as mock_thread_cls:
        thread_instance = MagicMock()
        mock_thread_cls.return_value = thread_instance

        node.cleanup_ros_processes()

        # State machine reset ngay lập tức dưới lock
        assert node._layer_states["layer1"] == LayerState.STOPPED
        assert len(node._layer_procs) == 0
        assert node._slam_proc is None
        assert node._localization_proc is None

        # pkill được chạy trong daemon worker thread
        mock_thread_cls.assert_called_once()
        _, kwargs = mock_thread_cls.call_args
        assert kwargs.get("daemon") is True
        thread_instance.start.assert_called_once()


def test_get_detected_ports_cached_non_blocking():
    """get_detected_ports trả về ngay từ cache mà không gọi lại I/O probe."""
    node = KioskRosNode.__new__(KioskRosNode)
    node._cached_serial_ports = ("/dev/ttyUSB1", "/dev/ttyUSB0", "cached_scan")
    node._force_swap_ports = False

    with patch("robot_ui.kiosk.serial_resolver.detect_serial_ports") as mock_detect:
        hb, lidar, src = node.get_detected_ports()
        assert hb == "/dev/ttyUSB1"
        assert lidar == "/dev/ttyUSB0"
        assert src == "cached_scan"
        mock_detect.assert_not_called()


def test_rescan_serial_ports_async():
    """rescan_serial_ports_async quét cổng ngầm trên daemon thread và gọi callback."""
    node = KioskRosNode.__new__(KioskRosNode)
    node._force_swap_ports = False
    node.get_logger = MagicMock(return_value=MagicMock())

    with patch("threading.Thread") as mock_thread_cls:
        thread_instance = MagicMock()
        mock_thread_cls.return_value = thread_instance

        node.rescan_serial_ports_async()
        mock_thread_cls.assert_called_once()
        _, kwargs = mock_thread_cls.call_args
        assert kwargs.get("daemon") is True
        thread_instance.start.assert_called_once()

