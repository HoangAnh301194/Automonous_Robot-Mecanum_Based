"""Unit tests for SystemConfigPage and System Status Architecture in Robot UI Kiosk."""
import os
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.system_config_page import SystemConfigPage


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_system_config_page_3level_navigation(app):
    missing = Path("/nonexistent-asset")
    page = SystemConfigPage(home_icon=missing, back_icon=missing, emergency_icon=missing)

    assert page.stack is not None
    # 5 Screens: 0=Overview, 1=Sensors, 2=Hardware, 3=ROS2, 4=Diagnostics
    assert page.stack.count() == 5
    assert page.stack.currentIndex() == 0

    # 1. Navigate to Sensors & AI Detail and use top bar Back button
    page.overview_screen.card_sensors.click()
    assert page.stack.currentIndex() == 1
    page.btn_back.click()  # Top bar back button returns to overview
    assert page.stack.currentIndex() == 0

    # 2. Navigate to Hardware Detail and use screen header Back button
    page.overview_screen.card_hardware.click()
    assert page.stack.currentIndex() == 2
    page.hardware_screen.header.btn_back.click()
    assert page.stack.currentIndex() == 0

    # 3. Navigate to ROS 2 System Detail
    page.overview_screen.card_ros2.click()
    assert page.stack.currentIndex() == 3
    page.ros2_screen.header.btn_back.click()
    assert page.stack.currentIndex() == 0

    # 4. Navigate to Diagnostics Detail
    page.overview_screen.card_diagnostics.click()
    assert page.stack.currentIndex() == 4
    page.diagnostics_screen.header.btn_back.click()
    assert page.stack.currentIndex() == 0

    page.close()


def test_system_config_telemetry_updates(app):
    missing = Path("/nonexistent-asset")
    page = SystemConfigPage(home_icon=missing, back_icon=missing, emergency_icon=missing)

    # Test sensor telemetry update
    snap = {
        "lidar_hz": 10.5,
        "lidar_min_dist": 0.45,
        "odom_hz": 20.0,
        "vx": 0.15,
        "wz": -0.05,
        "cam_front_hz": 15.0,
        "cam_rear_hz": 14.8,
        "cam_ai_hz": 28.0,
        "obs_front_dist": 0.85,
        "obs_rear_dist": 1.20,
        "tf_map_odom_ok": True,
        "tf_odom_base_ok": True,
        "yolo_active": True,
        "people_count": 2,
        "is_waving": True,
        "closest_person_dist": 1.45,
    }
    page.update_sensor_telemetry(snap)

    assert "10.5 Hz" in page.lbl_hz_lidar.text()
    assert "20.0 Hz" in page.lbl_hz_odom.text()
    assert "ĐÃ ĐỊNH VỊ" in page.lbl_tf_map_odom.text()
    assert "ODOM SỐNG" in page.lbl_tf_odom_base.text()
    assert "ĐANG VẪY" in page.lbl_ai_wave_status.text()
    assert "2 người" in page.lbl_ai_yolo_status.text()

    # Test battery and water update
    bat_data = {
        "battery_percent": 82.0,
        "voltage_smooth": 39.4,
        "voltage_raw": 39.2,
        "voltage_source": "hoverboard",
        "water_liter": 2.4,
        "kettle_present": True,
    }
    page.update_battery_water(bat_data)
    assert page.pbar_battery.value() == 82
    assert "82%" in page.lbl_bat_pct_big.text()
    assert "39.40V" in page.lbl_bat_details.text()
    assert "2.4 L" in page.lbl_water_val.text()

    page.close()


def test_system_config_signals(app):
    missing = Path("/nonexistent-asset")
    page = SystemConfigPage(home_icon=missing, back_icon=missing, emergency_icon=missing)

    home_mock = MagicMock()
    back_mock = MagicMock()
    emergency_mock = MagicMock()
    actuator_mock = MagicMock()

    page.home_requested.connect(home_mock)
    page.back_requested.connect(back_mock)
    page.emergency_stop_requested.connect(emergency_mock)
    page.actuator_test_requested.connect(actuator_mock)

    page.btn_home.click()
    assert home_mock.called

    page.btn_back.click()
    assert back_mock.called

    page.btn_emergency.click()
    assert emergency_mock.called

    page._trigger_actuator_test("pump")
    actuator_mock.assert_called_with("pump")

    page.close()


def test_system_config_preflight_and_icons(app):
    icons = {
        "home": Path("/nonexistent/home.png"),
        "back": Path("/nonexistent/back.png"),
        "emergency": Path("/nonexistent/emergency.png"),
        "wifi_connected": Path("/nonexistent/wifi.png"),
        "wifi_disconnected": Path("/nonexistent/wifidisconected.png"),
        "battery_icons": {"100": Path("/nonexistent/bat100.png")},
        "speaker": Path("/nonexistent/speaker.png"),
        "status": Path("/nonexistent/status.png"),
    }
    page = SystemConfigPage(icons=icons)

    # Trigger pre-flight check
    page._start_preflight_check()
    assert page._preflight_timer.isActive()
    assert "[ĐANG KIỂM TRA]" in page.lbl_preflight_banner.text() or "7 hạng mục" in page.lbl_preflight_banner.text()

    # Step through ticks
    for _ in range(8):
        page._on_preflight_tick()

    assert not page._preflight_timer.isActive()
    assert page.btn_run_preflight.isEnabled()

    # Ensure no emojis in preflight text
    for _, (_, lbl) in page.preflight_items.items():
        assert "✔" not in lbl.text()
        assert "❌" not in lbl.text()
        assert "✅" not in lbl.text()
        assert "[ĐẠT]" in lbl.text() or "[CHƯA ĐẠT]" in lbl.text()

    page.close()


def test_ros2_system_live_layer_log_console(app):
    """Test live layer logging and filtering in ROS2SystemDetailScreen."""
    page = SystemConfigPage()
    ros2_screen = page.ros2_screen

    assert hasattr(ros2_screen, "txt_layer_log")
    assert hasattr(ros2_screen, "_filter_buttons")

    # 1. Append log for Layer 1
    page.append_layer_log("layer1", "Hoverboard connected on /dev/ttyUSB0")
    assert "Hoverboard connected" in ros2_screen.txt_layer_log.toPlainText()
    assert "[LAYER1]" in ros2_screen.txt_layer_log.toPlainText()

    # 2. Append log for Layer 2
    page.append_layer_log("layer2", "AMCL localization node initialized")
    assert "AMCL localization node" in ros2_screen.txt_layer_log.toPlainText()

    # 3. Filter by Layer 1 only
    ros2_screen._set_log_filter("layer1")
    assert ros2_screen._current_filter == "layer1"
    content_l1 = ros2_screen.txt_layer_log.toPlainText()
    assert "Hoverboard connected" in content_l1
    assert "AMCL localization node" not in content_l1

    # 4. Filter by Layer 2 only
    ros2_screen._set_log_filter("layer2")
    assert ros2_screen._current_filter == "layer2"
    content_l2 = ros2_screen.txt_layer_log.toPlainText()
    assert "AMCL localization node" in content_l2
    assert "Hoverboard connected" not in content_l2

    # 5. Clear current log
    ros2_screen._clear_current_log()
    assert "Đã xóa" in ros2_screen.txt_layer_log.toPlainText()

    # 6. Switch back to All
    ros2_screen._set_log_filter("all")
    assert ros2_screen._current_filter == "all"

    # 7. Auto-scroll toggle
    initial_scroll = ros2_screen._auto_scroll
    ros2_screen._toggle_autoscroll()
    assert ros2_screen._auto_scroll != initial_scroll

    page.close()


def test_ros2_master_controls_and_serial_signal_flow(app):
    """Master workflow and port buttons are wired through the System Config facade."""
    page = SystemConfigPage()
    started, stopped, rescanned, swapped = [], [], [], []
    page.start_all_requested.connect(lambda mode, map_path, backend: started.append((mode, map_path, backend)))
    page.stop_all_requested.connect(lambda: stopped.append(True))
    page.rescan_ports_requested.connect(lambda: rescanned.append(True))
    page.swap_ports_requested.connect(lambda: swapped.append(True))
    screen = page.ros2_screen

    assert screen.cmb_operation_mode.currentIndex() == 0
    assert not screen.cmb_backend.isEnabled()
    assert not screen.map_row_widget.isVisible()

    screen.btn_start_all.click()
    assert started[-1][0] == "slam_mapping"

    screen.cmb_operation_mode.setCurrentIndex(1)
    assert screen.cmb_backend.isEnabled()
    screen.cmb_backend.setCurrentIndex(0)
    screen.cmb_maps.clear()
    screen.cmb_maps.addItem("Lab", "/tmp/lab_map.yaml")
    screen.btn_start_all.click()
    assert started[-1] == ("localization", "/tmp/lab_map.yaml", "amcl")

    screen.cmb_backend.setCurrentIndex(1)
    screen.btn_start_all.click()
    assert started[-1] == ("localization", "", "slam_toolbox")

    screen.btn_stop_all.click()
    screen.btn_rescan_ports.click()
    screen.btn_swap_ports.click()
    assert stopped == [True]
    assert rescanned == [True]
    assert swapped == [True]
    page.set_detected_ports("/dev/ttyUSB0", "/dev/ttyUSB1", "auto_probed")
    assert "ttyUSB0" in screen.lbl_ports_info.text()
    assert "ttyUSB1" in screen.lbl_ports_info.text()
    page.close()
