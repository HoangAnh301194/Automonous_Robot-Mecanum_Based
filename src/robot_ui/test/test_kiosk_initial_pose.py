import math
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from PyQt5.QtCore import QEvent, QPoint, QPointF, QRectF, Qt
from PyQt5.QtGui import QImage, QMouseEvent
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.navigation_page import (
    InitialPoseDialog,
    InteractiveNavMapWidget,
    NavigationPage,
)
from robot_ui.kiosk.ros_node import KioskRosNode, LayerState, LocalizationState
from robot_ui.kiosk.signals import KioskSignals


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    yield app


@pytest.fixture
def ros_node(qapp):
    if not rclpy.ok():
        rclpy.init(args=None)
    signals = KioskSignals()
    node = KioskRosNode(signals)
    node.check_amcl_lifecycle_active = MagicMock(return_value=(True, "AMCL active"))
    yield node
    node.destroy_node()


def test_ip01_publish_initial_pose_valid_message(ros_node):
    """IP-01: Publish Initial Pose hợp lệ -> Message/frame/quaternion/covariance đúng chuẩn."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    published = []
    ros_node._initial_pose_pub.publish = lambda msg: published.append(msg)

    success = ros_node.publish_initial_pose(1.5, -2.5, 0.785)
    assert success is True
    assert len(published) == 1

    msg = published[0]
    assert isinstance(msg, PoseWithCovarianceStamped)
    assert msg.header.frame_id == "map"
    assert msg.pose.pose.position.x == 1.5
    assert msg.pose.pose.position.y == -2.5
    assert msg.pose.pose.position.z == 0.0

    # Quaternion z = sin(yaw/2), w = cos(yaw/2)
    assert msg.pose.pose.orientation.z == pytest.approx(math.sin(0.785 / 2.0), abs=1e-4)
    assert msg.pose.pose.orientation.w == pytest.approx(math.cos(0.785 / 2.0), abs=1e-4)

    # Covariance 6x6
    cov = msg.pose.covariance
    assert len(cov) == 36
    assert cov[0] == 0.25   # var x
    assert cov[7] == 0.25   # var y
    assert cov[35] == 0.0685 # var yaw


def test_ip02_yaw_extremes_and_special_angles(ros_node):
    """IP-02: Yaw +-pi, 0, pi/2, -pi/2, 3pi -> Quaternion z^2 + w^2 = 1.0."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    test_angles = [0.0, math.pi, -math.pi, math.pi / 2.0, -math.pi / 2.0, 3.0 * math.pi]
    for angle in test_angles:
        published = []
        ros_node._initial_pose_pub.publish = lambda msg: published.append(msg)

        ok = ros_node.publish_initial_pose(0.0, 0.0, angle)
        assert ok is True
        msg = published[0]
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        assert (z * z + w * w) == pytest.approx(1.0, abs=1e-5)


def test_ip03_no_start_waypoint_rejects_auto_pose(ros_node, tmp_path):
    """IP-03: Bản đồ không có waypoint xuất phát 'khoihanh' -> Không publish bừa."""
    empty_map_yaml = tmp_path / "custom_map.yaml"
    empty_map_yaml.write_text("image: test.pgm\nresolution: 0.05\n", encoding="utf-8")

    pose = ros_node.resolve_start_pose_for_map(empty_map_yaml)
    assert pose is None


def test_ip04_waypoint_not_belonging_to_current_map(ros_node, tmp_path):
    """IP-04: Waypoint không thuộc map hiện tại -> Từ chối nạp nhầm điểm của map khác."""
    map_a = tmp_path / "map_a.yaml"
    map_a.write_text("image: a.pgm\nresolution: 0.05\n", encoding="utf-8")

    wp_b = tmp_path / "map_b_waypoints.yaml"
    wp_b.write_text("locations:\n  khoihanh:\n    x: 10.0\n    y: 20.0\n    yaw: 0.0\n", encoding="utf-8")

    # Map A không có file map_a_waypoints.yaml
    pose = ros_node.resolve_start_pose_for_map(map_a)
    if pose:
        assert pose["x"] != 10.0  # Không lấy điểm của map B


def test_ip05_amcl_inactive_rejects_initial_pose(ros_node):
    """IP-05: AMCL inactive (Layer 2 STOPPED) -> Không publish, chuyển ERROR."""
    ros_node._layer_states["layer2"] = LayerState.STOPPED
    ready, reason = ros_node.check_localization_readiness()
    assert ready is False
    assert "Layer 2" in reason

    published = []
    ros_node._initial_pose_pub.publish = lambda msg: published.append(msg)

    ok = ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    assert ok is False
    assert len(published) == 0
    assert ros_node._localization_state == LocalizationState.ERROR


def test_ip06_amcl_active_valid_sensors_accepts_pose(ros_node):
    """IP-06: AMCL active, dữ liệu hợp lệ -> Sẵn sàng chấp nhận pose."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ready, reason = ros_node.check_localization_readiness()
    assert ready is True


def test_ip07_map_switch_cancels_pending_pose_task(ros_node):
    """IP-07: Đổi map trong lúc đang chờ -> Hủy session token cũ."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    old_token = ros_node._loc_session_token
    assert old_token != ""

    ros_node.cancel_pending_initial_pose()
    assert ros_node._loc_session_token == ""
    assert ros_node._localization_state == LocalizationState.NOT_INITIALIZED


def test_ip08_stop_layer2_resets_localization(ros_node):
    """IP-08: Dừng Layer 2 bằng stop_layer -> Hủy phiên định vị và đưa về NOT_INITIALIZED."""
    ros_node._localization_state = LocalizationState.LOCALIZED
    ros_node._loc_session_token = "abc123"
    ros_node._have_localized_pose = True
    ros_node._layer_states["layer2"] = LayerState.RUNNING

    mock_proc = MagicMock()
    mock_proc.poll.return_value = 0
    ros_node._layer_procs["layer2"] = mock_proc
    ros_node._terminate_process_group = MagicMock(return_value=True)

    ros_node.stop_layer("layer2")
    time.sleep(0.15)

    assert ros_node._localization_state == LocalizationState.NOT_INITIALIZED
    assert ros_node._loc_session_token == ""


def test_ip09_published_without_tf_not_localized(ros_node):
    """IP-09: Publish thành công nhưng chưa có TF map -> odom -> Không báo LOCALIZED."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()

    def _mock_can_transform(target_frame, source_frame, *args, **kwargs):
        if target_frame == "odom" and source_frame in ("base_footprint", "base_link"):
            return True
        return False  # Không có TF map -> odom

    ros_node._tf_buffer.can_transform = MagicMock(side_effect=_mock_can_transform)

    ok = ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    assert ok is True
    assert ros_node._localization_state != LocalizationState.LOCALIZED
    assert ros_node._localization_state in (
        LocalizationState.WAITING_FOR_LOCALIZATION,
        LocalizationState.PUBLISHING,
    )


def test_ip10_stale_tf_not_accepted(ros_node):
    """IP-10: TF timeout hoặc không tạo được -> Chuyển sang ERROR."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=False)

    token = "test_tok"
    ros_node._loc_session_token = token
    ros_node._localization_state = LocalizationState.WAITING_FOR_LOCALIZATION

    # Chạy worker timeout
    ros_node._verify_localization_worker(token)
    assert ros_node._localization_state == LocalizationState.ERROR
    assert "Hết thời gian chờ" in ros_node._localization_message


def test_ip11_amcl_pose_and_tf_transitions_to_localized(ros_node):
    """IP-11: Nhận /amcl_pose mới và TF map->odom hợp lệ -> Chuyển sang LOCALIZED và emit signal."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    confirmed_events = []
    ros_node.signals.initial_pose_confirmed.connect(lambda d: confirmed_events.append(d))

    ok = ros_node.publish_initial_pose(2.0, 3.0, 1.57)
    assert ok is True
    tok = ros_node._loc_session_token

    # Giả lập nhận /amcl_pose với covariance nhỏ
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 2.05
    msg.pose.pose.position.y = 2.98
    msg.pose.pose.orientation.z = math.sin(1.57 / 2.0)
    msg.pose.pose.orientation.w = math.cos(1.57 / 2.0)
    cov = [0.0] * 36
    cov[0] = 0.05
    cov[7] = 0.05
    msg.pose.covariance = cov

    ros_node._on_amcl_pose(msg)
    # Xác nhận nguyên tử qua verify worker
    ros_node._verify_localization_worker(tok)
    assert ros_node._localization_state == LocalizationState.LOCALIZED
    assert len(confirmed_events) == 1
    assert confirmed_events[0]["token"] == tok


def test_ip12_pixel_to_map_coordinate_conversion(qapp):
    """IP-12: Test chuyển đổi tọa độ Pixel <-> Map trong InteractiveNavMapWidget."""
    widget = InteractiveNavMapWidget()
    mock_map = {
        "preview_width": 200,
        "preview_height": 200,
        "resolution": 0.05,
        "sample_step": 1,
        "origin_x": -5.0,
        "origin_y": -5.0,
        "origin_yaw": 0.0,
        "cells": [0] * 40000,
    }
    widget._map = mock_map
    target_rect = QRectF(0, 0, 400, 400)

    # 1. Tâm bản đồ thế giới (0.0, 0.0)
    screen_pt = widget._world_to_screen(0.0, 0.0, target_rect)
    assert screen_pt is not None
    assert screen_pt.x() == pytest.approx(200.0, abs=1.0)
    assert screen_pt.y() == pytest.approx(200.0, abs=1.0)

    # 2. Ngược lại: screen sang world
    world_coord = widget._screen_to_world(screen_pt, target_rect)
    assert world_coord is not None
    assert world_coord[0] == pytest.approx(0.0, abs=0.1)
    assert world_coord[1] == pytest.approx(0.0, abs=0.1)


def test_ip13_double_tap_initial_pose_debounce(ros_node):
    """IP-13: Bấm Initial Pose liên tục -> Session token mới thay thế, không duplicate worker."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    t1 = ros_node._loc_session_token

    ros_node.publish_initial_pose(2.0, 2.0, 0.0)
    t2 = ros_node._loc_session_token

    assert t1 != t2
    assert ros_node._loc_session_token == t2


def test_ip14_slam_toolbox_skips_amcl_initial_pose(ros_node):
    """IP-14: Backend slam_toolbox -> Không yêu cầu AMCL Initial Pose."""
    ros_node._selected_backend = "slam_toolbox"
    ready, reason = ros_node.check_localization_readiness()
    assert ready is True

    nav_ready, msg = ros_node.is_navigation_ready()
    assert nav_ready is True


def test_ip15_navigation_gate_blocks_goal_when_not_localized(ros_node):
    """IP-15: Navigation Readiness Gate chặn Goal mới khi chưa định vị."""
    ros_node._selected_backend = "amcl"
    ros_node._localization_state = LocalizationState.NOT_INITIALIZED

    nav_ready, reason = ros_node.is_navigation_ready()
    assert nav_ready is False
    assert "chưa hoàn tất định vị" in reason

    warnings = []
    ros_node.signals.command_status_changed.connect(lambda tone, msg: warnings.append((tone, msg)))

    # Gửi thử điểm đến
    ros_node.navigate_to_pose(1.0, 2.0, 0.0)
    ros_node.navigate_to("diem1")
    ros_node.start_patrol()

    assert len(warnings) == 3
    for tone, msg in warnings:
        assert tone == "warning"
        assert "chưa hoàn tất định vị" in msg


def test_ip16_navigation_gate_allows_goal_when_localized(ros_node):
    """IP-16: Cho phép gửi Goal khi đã LOCALIZED, chặn khi ở TF_AVAILABLE hoặc NOT_INITIALIZED."""
    ros_node._selected_backend = "amcl"
    ros_node._localization_state = LocalizationState.TF_AVAILABLE
    ready, reason = ros_node.is_navigation_ready()
    assert ready is False
    assert "chưa hoàn tất định vị" in reason

    ros_node._localization_state = LocalizationState.LOCALIZED
    ready, _ = ros_node.is_navigation_ready()
    assert ready is True

    # Giả lập phát lệnh navigate_to_pose
    warnings = []
    ros_node.signals.command_status_changed.connect(lambda tone, msg: warnings.append((tone, msg)))
    ros_node._nav_action_client = MagicMock()
    ros_node._nav_action_client.wait_for_server.return_value = True

    ros_node.navigate_to_pose(1.0, 2.0, 0.0)
    # Không có cảnh báo chặn định vị nào được phát ra
    loc_warnings = [m for t, m in warnings if "chưa hoàn tất định vị" in m]
    assert len(loc_warnings) == 0


def test_ip17_close_kiosk_cancels_daemon_workers(ros_node):
    """IP-17: Đóng Kiosk giữa chừng (stop_subprocesses) -> Hủy session token và chuyển NOT_INITIALIZED."""
    ros_node._loc_session_token = "tok_active"
    ros_node._localization_state = LocalizationState.WAITING_FOR_LOCALIZATION
    ros_node.stop_subprocesses()
    assert ros_node._loc_session_token == ""
    assert ros_node._localization_state == LocalizationState.NOT_INITIALIZED


def test_ip18_subpage_switch_preserves_localization_state(qapp):
    """IP-18: Chuyển subpage trong NavigationPage -> Giữ nguyên trạng thái Localization."""
    icons = {}
    page = NavigationPage(icons)
    page.set_localization_status("LOCALIZED", "Đã định vị thành công")
    assert page._localization_state == "LOCALIZED"
    assert "ĐÃ ĐỊNH VỊ" in page.lbl_localization_badge.text()

    # Kích hoạt / hủy kích hoạt trang
    page.set_active(False)
    page.set_active(True)
    assert page._localization_state == "LOCALIZED"
    assert "ĐÃ ĐỊNH VỊ" in page.lbl_localization_badge.text()
    page.close()


def test_ip19_high_covariance_rejected(ros_node):
    """IP-19: /amcl_pose có phương sai lớn (>1.20m) -> Không chuyển LOCALIZED, báo ERROR sau timeout."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ok = ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    assert ok is True
    tok = ros_node._loc_session_token

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    cov = [0.0] * 36
    cov[0] = 1.0   # var x
    cov[7] = 1.0   # var y -> norm = sqrt(2.0) = 1.41m > 1.20m
    msg.pose.covariance = cov

    ros_node._on_amcl_pose(msg)
    ros_node._verify_localization_worker(tok)
    assert ros_node._localization_state != LocalizationState.LOCALIZED
    assert ros_node._localization_state == LocalizationState.ERROR
    assert "Độ phân tán hạt quá lớn" in ros_node._localization_message


def test_ip20_amcl_lifecycle_inactive_rejects_pose(ros_node):
    """IP-20: Layer 2 RUNNING nhưng AMCL node chưa active -> Reject phát pose."""
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ros_node.check_amcl_lifecycle_active = MagicMock(
        return_value=(False, "AMCL chưa active (trạng thái: inactive)")
    )
    ready, reason = ros_node.check_localization_readiness()
    assert ready is False
    assert "chưa active" in reason

    ok = ros_node.publish_initial_pose(1.0, 1.0, 0.0)
    assert ok is False
    assert ros_node._localization_state == LocalizationState.ERROR


def test_ip21_mismatched_map_name_rejected(ros_node):
    """IP-21: map_name yêu cầu không khớp bản đồ active -> Từ chối phát pose."""
    ros_node._selected_map_yaml = "/maps/tang1_cntt.yaml"
    ros_node._layer_states["layer2"] = LayerState.RUNNING
    ros_node._last_odom_time = time.monotonic()
    ros_node._last_scan_time = time.monotonic()
    ros_node._tf_buffer.can_transform = MagicMock(return_value=True)

    ok = ros_node.publish_initial_pose(1.0, 1.0, 0.0, map_name="tang2_dtvt")
    assert ok is False
    assert ros_node._localization_state == LocalizationState.ERROR
    assert "không khớp" in ros_node._localization_message


def test_ip22_stale_session_token_ignored(ros_node):
    """IP-22: Worker nhận token cũ không khớp _loc_session_token -> Thoát ngay không đổi state."""
    ros_node._localization_state = LocalizationState.WAITING_FOR_LOCALIZATION
    ros_node._loc_session_token = "token_current"

    # Chạy worker với token cũ
    ros_node._verify_localization_worker("token_stale")
    assert ros_node._localization_state == LocalizationState.WAITING_FOR_LOCALIZATION


def test_ip23_ui_navigation_gate_blocks_tf_available(qapp, monkeypatch):
    """IP-23: UI Navigation Page chặn tuyệt đối phát lệnh khi ở TF_AVAILABLE."""
    icons = {}
    page = NavigationPage(icons)
    page._goal = (1.0, 2.0, 0.0)
    page.set_localization_status("TF_AVAILABLE", "Chờ hội tụ")

    warning_dialogs = []
    monkeypatch.setattr(
        "PyQt5.QtWidgets.QMessageBox.warning",
        lambda *args, **kwargs: warning_dialogs.append(args),
    )

    signals_emitted = []
    page.start_requested.connect(lambda *args: signals_emitted.append(args))
    page.start_patrol_requested.connect(lambda: signals_emitted.append("patrol"))

    # Thử chạy single navigation
    page._start_single_navigation()
    assert len(signals_emitted) == 0
    assert len(warning_dialogs) == 1

    # Thử chạy patrol
    page._start_patrol()
    assert len(signals_emitted) == 0
    assert len(warning_dialogs) == 2
    page.close()


def test_scan_telemetry_does_not_require_robot_pose(ros_node):
    """LiDAR data must update telemetry even before AMCL/Odom pose is received."""
    assert ros_node._pose == {}

    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 5.0
    scan.angle_min = 0.0
    scan.angle_increment = 0.1
    scan.ranges = [0.8, 1.2, float("inf")]

    ros_node._on_scan(scan)
    assert ros_node._topic_stats["lidar"]["count"] == 1
    assert ros_node._topic_stats["lidar"]["extra"]["min_dist"] == pytest.approx(0.8)

    snapshots = []
    ros_node.signals.system_telemetry_changed.connect(snapshots.append)
    ros_node._publish_system_telemetry()
    assert snapshots[-1]["lidar_min_dist"] == pytest.approx(0.8)


def test_scan_overlay_after_pose_and_rviz_launch_configuration(ros_node):
    """Port simulator pose cache and reuse RViz SetInitialPose with Layer 2."""
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.orientation.w = 1.0
    ros_node._on_odom(odom)
    assert ros_node._pose["x"] == pytest.approx(1.0)

    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 5.0
    scan.angle_min = 0.0
    scan.angle_increment = 0.1
    scan.ranges = [0.8, 1.2]

    overlays = []
    ros_node.signals.scan_points_changed.connect(overlays.append)
    ros_node._last_scan_ui_pub = 0.0
    ros_node._on_scan(scan)
    assert overlays and len(overlays[-1]) == 2

    layer2_command = ros_node._build_layer_cmd("layer2")
    assert "real_localization_nav2.launch.py" in layer2_command
    assert "use_rviz:=false" in layer2_command


def _setup_mock_map(widget: InteractiveNavMapWidget, width: int = 200, height: int = 200) -> None:
    """Thiết lập bản đồ giả lập với kích thước và pixel hợp lệ cho unit test."""
    widget.resize(width, height)
    img = QImage(width, height, QImage.Format_RGB32)
    img.fill(Qt.white)
    widget._map_image = img
    widget._map = {
        "preview_width": width,
        "preview_height": height,
        "resolution": 0.05,
        "origin_x": 0.0,
        "origin_y": 0.0,
        "origin_yaw": 0.0,
        "sample_step": 1,
        "cells": [0] * (width * height),
    }


def test_ip24_2d_pose_estimate_toggle_button(qapp):
    """IP-24: Nút 2D POSE ESTIMATE bật/tắt đúng chế độ và trạng thái giao diện."""
    page = NavigationPage({})
    assert page.btn_initial_pose.text() == "2D POSE ESTIMATE"
    assert not page.map_widget._initial_pose_mode

    # Click để bật chế độ
    page.btn_initial_pose.click()
    assert page.map_widget._initial_pose_mode is True
    assert page.btn_initial_pose.text() == "HỦY 2D POSE"
    assert "Chạm và kéo trên bản đồ" in page.status_label.text()

    # Click lại để hủy
    page.btn_initial_pose.click()
    assert page.map_widget._initial_pose_mode is False
    assert page.btn_initial_pose.text() == "2D POSE ESTIMATE"
    assert "Đã hủy" in page.status_label.text()
    page.close()


def test_ip25_2d_pose_estimate_press_drag_release(qapp):
    """IP-25: Thao tác Press -> Drag -> Release phát sinh Initial Pose chính xác."""
    page = NavigationPage({})
    _setup_mock_map(page.map_widget, 200, 200)

    emitted_poses = []
    page.initial_pose_requested.connect(emitted_poses.append)

    # Bật chế độ 2D Pose
    page._toggle_2d_estimate()
    assert page.map_widget._initial_pose_mode is True

    # 1. Press tại tâm bản đồ (100, 100)
    p_center = QPoint(100, 100)
    press_ev = QMouseEvent(QEvent.MouseButtonPress, p_center, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    page.map_widget.mousePressEvent(press_ev)

    assert page.map_widget._is_dragging_pose is True
    assert page.map_widget._estimate_start_world is not None

    # 2. Drag sang phải (150, 100) -> hướng dương trục X (yaw ~ 0.0 rad)
    p_right = QPoint(150, 100)
    move_ev = QMouseEvent(QEvent.MouseMove, p_right, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    page.map_widget.mouseMoveEvent(move_ev)

    assert abs(page.map_widget._estimate_yaw) < 0.2

    # 3. Release để kết thúc thao tác
    rel_ev = QMouseEvent(QEvent.MouseButtonRelease, p_right, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    page.map_widget.mouseReleaseEvent(rel_ev)

    # Chế độ tự động tắt sau khi thả
    assert page.map_widget._initial_pose_mode is False
    assert page.map_widget._is_dragging_pose is False
    assert page.btn_initial_pose.text() == "2D POSE ESTIMATE"

    # Kiểm tra payload phát đi
    assert len(emitted_poses) == 1
    pose = emitted_poses[0]
    assert pose["source"] == "2d_pose_estimate"
    assert "x" in pose and "y" in pose and "yaw" in pose
    assert abs(pose["yaw"]) < 0.2
    page.close()


def test_ip26_2d_pose_estimate_short_drag_uses_fallback_yaw(qapp):
    """IP-26: Kéo quá ngắn hoặc chỉ click sẽ dùng hướng robot hiện tại làm fallback."""
    page = NavigationPage({})
    _setup_mock_map(page.map_widget, 200, 200)
    page.map_widget.set_pose({"x": 1.0, "y": 2.0, "yaw": 1.25})

    emitted_poses = []
    page.initial_pose_requested.connect(emitted_poses.append)

    page._toggle_2d_estimate()

    # Click mà không kéo (khoảng cách 0px < 8px ngưỡng)
    p = QPoint(100, 100)
    press_ev = QMouseEvent(QEvent.MouseButtonPress, p, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    page.map_widget.mousePressEvent(press_ev)

    rel_ev = QMouseEvent(QEvent.MouseButtonRelease, p, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier)
    page.map_widget.mouseReleaseEvent(rel_ev)

    assert len(emitted_poses) == 1
    assert emitted_poses[0]["yaw"] == pytest.approx(1.25)
    page.close()


def test_ip27_2d_pose_estimate_does_not_emit_goal_or_waypoint(qapp):
    """IP-27: Trong chế độ 2D Pose Estimate không được phát Navigation Goal hay Waypoint."""
    page = NavigationPage({})
    _setup_mock_map(page.map_widget, 200, 200)

    goals = []
    waypoints = []
    page.map_widget.goal_selected.connect(goals.append)
    page.map_widget.waypoint_picked.connect(waypoints.append)

    page._toggle_2d_estimate()

    p1 = QPoint(80, 80)
    p2 = QPoint(120, 80)
    page.map_widget.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, p1, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    page.map_widget.mouseMoveEvent(QMouseEvent(QEvent.MouseMove, p2, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    page.map_widget.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, p2, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))

    assert len(goals) == 0
    assert len(waypoints) == 0
    page.close()


def test_ip28_2d_pose_estimate_click_outside_map_ignored(qapp):
    """IP-28: Click vào vùng ngoài bản đồ sẽ bị bỏ qua và không emit initial pose."""
    page = NavigationPage({})
    _setup_mock_map(page.map_widget, 200, 200)

    emitted_poses = []
    page.initial_pose_requested.connect(emitted_poses.append)

    page._toggle_2d_estimate()

    # Click ở điểm âm (-10, -10) ngoài target rect
    p_outside = QPoint(-10, -10)
    page.map_widget.mousePressEvent(QMouseEvent(QEvent.MouseButtonPress, p_outside, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    assert page.map_widget._is_dragging_pose is False

    page.map_widget.mouseReleaseEvent(QMouseEvent(QEvent.MouseButtonRelease, p_outside, Qt.LeftButton, Qt.LeftButton, Qt.NoModifier))
    assert len(emitted_poses) == 0
    page.close()


def test_ip29_capture_current_pose_as_start(qapp, tmp_path):
    """IP-29: Chụp vị trí hiện tại của robot lưu vào YAML và cập nhật danh sách waypoints."""
    page = NavigationPage({})
    map_yaml = tmp_path / "lab_robot.yaml"
    map_yaml.write_text("image: lab.png\nresolution: 0.05\norigin: [0, 0, 0]\n")
    page._current_map_path = map_yaml
    page._batch_waypoints = []
    page._start_wp = None

    # 1. Khi chưa có tọa độ robot -> Cảnh báo không chụp được
    page._current_robot_pose = None
    page.map_widget._pose = None
    page.map_widget._candidate_initial_pose = None
    page._capture_current_pose_as_start(silent=True)
    assert page._start_wp is None

    # 2. Cung cấp tọa độ robot và chụp thành công
    page.set_pose({"x": 3.1415, "y": -2.7182, "yaw": 0.7853})
    page._capture_current_pose_as_start(silent=True)

    assert page._start_wp is not None
    assert page._start_wp["x"] == pytest.approx(3.142, abs=0.01)
    assert page._start_wp["y"] == pytest.approx(-2.718, abs=0.01)
    assert page._start_wp["yaw"] == pytest.approx(0.785, abs=0.01)

    # Kiểm tra file YAML được tạo đúng chuẩn
    wp_file = tmp_path / "lab_robot_waypoints.yaml"
    assert wp_file.is_file()
    import yaml
    with wp_file.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    assert "locations" in data
    assert "khoihanh" in data["locations"]
    assert data["locations"]["khoihanh"]["x"] == pytest.approx(3.142, abs=0.01)
    assert page._batch_waypoints[0]["name"] == "khoihanh"
    page.close()


def test_ip30_relocalize_to_start_pose(qapp, tmp_path):
    """IP-30: Khôi phục định vị về điểm xuất phát đã lưu (phát /initialpose)."""
    page = NavigationPage({})
    emitted = []
    page.initial_pose_requested.connect(emitted.append)

    # 1. Chưa có điểm xuất phát -> Bị chặn và không emit
    page._start_wp = None
    page._batch_waypoints = []
    page._current_map_path = tmp_path / "empty_map.yaml"
    page._relocalize_to_start_pose(silent=True)
    assert len(emitted) == 0

    # 2. Đã có điểm xuất phát -> Phát /initialpose với tọa độ chuẩn
    page._start_wp = {"name": "khoihanh", "x": 1.25, "y": -4.50, "yaw": 1.57}
    page._relocalize_to_start_pose(silent=True)

    assert len(emitted) == 1
    req = emitted[0]
    assert req["x"] == pytest.approx(1.25)
    assert req["y"] == pytest.approx(-4.50)
    assert req["yaw"] == pytest.approx(1.57)
    assert req["source"] == "restore_start_pose"
    page.close()


def test_ip31_buttons_presence_and_sync(qapp):
    """IP-31: Đảm bảo đầy đủ các nút bấm Chụp & Khôi phục vị trí trên cả 2 Tab giao diện."""
    page = NavigationPage({})
    assert hasattr(page, "btn_capture_start_pose")
    assert hasattr(page, "btn_relocalize_start_pose")
    assert hasattr(page, "btn_quick_relocalize")
    assert hasattr(page, "btn_patrol_capture_start")
    assert hasattr(page, "btn_patrol_relocalize")

    assert page.btn_capture_start_pose.text() == "📸 LƯU ĐIỂM GỐC"
    assert page.btn_relocalize_start_pose.text() == "🎯 VỀ ĐIỂM GỐC"
    assert page.btn_quick_relocalize.text() == "🎯 VỀ ĐIỂM GỐC"
    page.close()

