"""Unit tests for DetectionDebugPage, Process Monitor, RQT launchers, and Map Sync."""
import os
from pathlib import Path
from unittest.mock import MagicMock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.admin_page import AdminPage
from robot_ui.kiosk.pages.detection_debug_page import DetectionDebugPage
from robot_ui.kiosk.pages.navigation_page import NavigationPage
from robot_ui.kiosk.slam_page import SlamMapPage
from robot_ui.kiosk.pages.system_config_page import SystemConfigPage
from robot_ui.kiosk.pages.system_status.contracts import HostTelemetrySnapshot
from robot_ui.kiosk.pages.system_status.hardware import HardwareDetailScreen
from robot_ui.kiosk.pages.system_status.sensors_ai import SensorsAIDetailScreen


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_detection_debug_page_telemetry_and_signals(app):
    icons = {
        "home": Path("/nonexistent/home.png"),
        "back": Path("/nonexistent/back.png"),
        "emergency": Path("/nonexistent/emergency.png"),
        "wifi_connected": Path("/nonexistent/wifi.png"),
        "wifi_disconnected": Path("/nonexistent/wifidisconected.png"),
        "battery_icons": {"100": Path("/nonexistent/bat100.png")},
    }
    page = DetectionDebugPage(icons=icons)

    # 1. Verify Header Buttons & Signals
    home_mock = MagicMock()
    back_mock = MagicMock()
    emg_mock = MagicMock()
    fs_mock = MagicMock()

    page.home_requested.connect(home_mock)
    page.back_requested.connect(back_mock)
    page.emergency_stop_requested.connect(emg_mock)
    page.fullscreen_toggle_requested.connect(fs_mock)

    page.btn_home.click()
    assert home_mock.called
    page.btn_back.click()
    assert back_mock.called
    page.btn_emergency.click()
    assert emg_mock.called
    page.btn_fullscreen.click()
    assert fs_mock.called

    # 2. Verify Fullscreen Toggle Glyph
    page.update_fullscreen_button(is_fullscreen=True)
    assert page.btn_fullscreen.text() == "🗗"
    page.update_fullscreen_button(is_fullscreen=False)
    assert page.btn_fullscreen.text() == "🗖"

    # 3. Verify Telemetry Updates
    telemetry = {
        "yolo_people_count": 3,
        "closest_person_dist": 1.25,
        "hand_wave_detected": True,
        "people_distances": [1.25, 2.10, 3.40],
    }
    page.update_ai_telemetry(telemetry)
    assert "3" in page.lbl_people_count.text()
    assert "1.25 m" in page.lbl_closest_dist.text()
    assert "CÓ VẪY TAY" in page.lbl_wave_state.text()
    assert page.list_people.count() == 3

    # 4. Verify Face Recognition Placeholder
    assert hasattr(page, "btn_register_face")
    assert not page.btn_register_face.isEnabled()

    # 5. Verify RQT Launch
    with patch("subprocess.Popen") as mock_popen:
        page._launch_rqt_pose()
        mock_popen.assert_called_once()
        cmd = mock_popen.call_args[0][0]
        cmd_str = " ".join(cmd) if isinstance(cmd, list) else str(cmd)
        assert "rqt_image_view" in cmd_str
        assert "/ai/pose_image" in cmd_str

    page.close()


def test_hardware_process_monitor(app):
    hw = HardwareDetailScreen()

    # Check initial state: table hidden
    assert hw.table_proc.isHidden()
    assert "Hiện Tiến Trình" in hw.btn_toggle_proc.text()

    # Toggle table visibility
    hw._toggle_proc_table()
    assert not hw.table_proc.isHidden()
    assert "Ẩn Tiến Trình" in hw.btn_toggle_proc.text()

    # Send host telemetry with top processes
    procs = [
        {"pid": 1234, "name": "ros2", "cpu": 45.2, "ram_mb": 512.5, "status": "running"},
        {"pid": 5678, "name": "yolo_node", "cpu": 32.1, "ram_mb": 1024.0, "status": "running"},
    ]
    snap = HostTelemetrySnapshot(top_processes=procs)
    hw.update_host_telemetry(snap)

    assert hw.table_proc.rowCount() == 2
    assert hw.table_proc.item(0, 0).text() == "1234"
    assert hw.table_proc.item(0, 1).text() == "ros2"
    assert "45.2%" in hw.table_proc.item(0, 2).text()
    assert "512" in hw.table_proc.item(0, 3).text()

    hw.close()


def test_sensors_ai_rqt_camera_buttons(app):
    sensors = SensorsAIDetailScreen()

    assert hasattr(sensors, "btn_rqt_cam_f")
    assert hasattr(sensors, "btn_rqt_cam_r")
    assert hasattr(sensors, "btn_rqt_cam_ai")

    with patch("subprocess.Popen") as mock_popen:
        sensors._launch_rqt_image_view("/camera/front/image_raw")
        mock_popen.assert_called_once()
        cmd = mock_popen.call_args[0][0]
        cmd_str = " ".join(cmd) if isinstance(cmd, list) else str(cmd)
        assert "rqt_image_view" in cmd_str
        assert "/camera/front/image_raw" in cmd_str

    sensors.close()


def test_navigation_map_yaml_sync(app, tmp_path):
    icons = {"goal": Path("/nonexistent/goal.png")}
    nav = NavigationPage(icons=icons)

    # 1. Load a2_amcl map and verify its dedicated waypoints are loaded
    map_a2 = Path("my_map/a2_amcl.yaml")
    if map_a2.is_file():
        success = nav.load_map_yaml(map_a2)
        assert success is True
        assert not nav.map_widget._map_image.isNull()
        assert "a2_amcl" in nav.status_label.text()
        assert len(nav._batch_waypoints) == 7
        assert nav._batch_waypoints[0]["name"] == "khoihanh"
        assert nav.table_waypoints.rowCount() == 7

    # 2. Switch to san_b5 map and verify waypoints switch to san_b5's dedicated waypoints
    map_b5 = Path("my_map/san_b5.yaml")
    if map_b5.is_file():
        success = nav.load_map_yaml(map_b5)
        assert success is True
        assert "san_b5" in nav.status_label.text()
        assert len(nav._batch_waypoints) == 1
        assert nav.table_waypoints.rowCount() == 1
        assert "Trạm xuất phát san_b5" in nav._batch_waypoints[0]["note"]

    # 3. Switch to a brand new map without any saved waypoints
    # Create dummy map yaml and 1x1 pgm
    dummy_yaml = tmp_path / "new_clean_map.yaml"
    dummy_pgm = tmp_path / "new_clean_map.pgm"
    # P2 1 1 255 0
    dummy_pgm.write_bytes(b"P5\n1 1\n255\n\x00")
    dummy_yaml.write_text(
        "image: new_clean_map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n"
    )

    success = nav.load_map_yaml(dummy_yaml)
    assert success is True
    # Canvas and waypoints must be completely cleared
    assert len(nav._batch_waypoints) == 0
    assert nav.table_waypoints.rowCount() == 0
    assert "Chưa có điểm" in nav.status_label.text()

    # 4. Add waypoints to the new map and save
    nav._batch_waypoints = [
        {"name": "p1", "x": 1.5, "y": 2.5, "yaw": 0.0, "note": "Point 1"},
        {"name": "p2", "x": 3.0, "y": 4.0, "yaw": 1.0, "note": "Point 2"},
    ]
    nav._sync_views_from_waypoints()
    with patch("PyQt5.QtWidgets.QMessageBox.information"), patch("PyQt5.QtWidgets.QMessageBox.warning"):
        assert nav._save_patrol_to_yaml() is True

    # Check dedicated file was created for new_clean_map
    expected_wp_file = tmp_path / "new_clean_map_waypoints.yaml"
    assert expected_wp_file.is_file()

    # 5. Switch back to a2_amcl and verify its original 7 waypoints return intact
    if map_a2.is_file():
        nav.load_map_yaml(map_a2)
        assert len(nav._batch_waypoints) == 7
        assert nav._batch_waypoints[0]["name"] == "khoihanh"
        assert nav.table_waypoints.rowCount() == 7

    nav.close()


def test_fullscreen_synchronization_across_pages(app):
    dummy = Path("/nonexistent/dummy.png")
    admin = AdminPage(dummy, dummy)
    nav = NavigationPage(icons={})
    slam = SlamMapPage(icons={})
    sys_cfg = SystemConfigPage(icons={})
    det_dbg = DetectionDebugPage(icons={})

    pages = [admin, nav, slam, sys_cfg, det_dbg]

    for p in pages:
        p.update_fullscreen_button(is_fullscreen=True)
        btn = getattr(p, "fullscreen_button", getattr(p, "btn_fullscreen", None))
        assert btn is not None
        assert btn.text() == "🗗"

        p.update_fullscreen_button(is_fullscreen=False)
        assert btn.text() == "🗖"

    sys_cfg.stop()
    for p in pages:
        p.close()
