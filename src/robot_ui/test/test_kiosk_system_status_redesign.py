"""test_kiosk_system_status_redesign.py - Comprehensive Unit & Regression Tests for Souta System Status Redesign."""
import os
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.system_status import (
    DangerousActionDialog,
    DiagnosticsDetailScreen,
    HardwareDetailScreen,
    HealthStatus,
    HostTelemetrySnapshot,
    KioskTheme,
    MetricCard,
    ModuleCard,
    PowerWaterSnapshot,
    PreflightStatus,
    ROS2SystemDetailScreen,
    SensorsAIDetailScreen,
    StatusPill,
    SystemOverviewScreen,
    SystemTelemetrySnapshot,
    compute_overall_health,
)


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_contracts_and_overall_health():
    sensors = SystemTelemetrySnapshot(lidar_hz=10.0, tf_map_odom_ok=True, tf_odom_base_ok=True)
    host = HostTelemetrySnapshot(cpu_pct=25.0, cpu_temp=55.0)
    power = PowerWaterSnapshot(battery_percent=78.0, voltage_smooth=39.2)

    health, headline, warnings = compute_overall_health(sensors, host, power)
    assert health == HealthStatus.HEALTHY
    assert "bình thường" in headline
    assert len(warnings) == 0

    # Test warning trigger (High CPU temp 78°C)
    host_hot = HostTelemetrySnapshot(cpu_pct=85.0, cpu_temp=78.0)
    health_w, headline_w, warnings_w = compute_overall_health(sensors, host_hot, power)
    assert health_w == HealthStatus.WARNING
    assert any("nhiệt độ" in w.lower() for w in warnings_w)

    # Test critical trigger (Overheat 88°C)
    host_crit = HostTelemetrySnapshot(cpu_pct=95.0, cpu_temp=88.0)
    health_c, headline_c, _ = compute_overall_health(sensors, host_crit, power)
    assert health_c == HealthStatus.CRITICAL
    assert "nguy hiểm" in headline_c.lower()

    # Test unknown trigger (Missing telemetry)
    health_u, _, _ = compute_overall_health(SystemTelemetrySnapshot(), HostTelemetrySnapshot(), PowerWaterSnapshot())
    assert health_u == HealthStatus.UNKNOWN


def test_theme_tokens():
    assert KioskTheme.COLOR_BG == "#000000"
    assert KioskTheme.COLOR_SURFACE == "#111418"
    assert KioskTheme.COLOR_ACCENT == "#F6C16A"
    assert KioskTheme.TOUCH_TARGET_MIN >= 44

    assert KioskTheme.get_status_color(HealthStatus.HEALTHY) == KioskTheme.COLOR_HEALTHY
    assert KioskTheme.get_status_color(HealthStatus.WARNING) == KioskTheme.COLOR_WARNING
    assert KioskTheme.get_status_color(HealthStatus.CRITICAL) == KioskTheme.COLOR_CRITICAL
    assert KioskTheme.get_status_color(HealthStatus.UNKNOWN) == KioskTheme.COLOR_UNKNOWN


def test_components(app):
    pill = StatusPill("HEALTHY", HealthStatus.HEALTHY)
    assert pill.text() == "HEALTHY"

    metric = MetricCard("CPU Load", "45%", "Safe")
    assert metric.lbl_val.text() == "45%"
    metric.update_data("85%", "High", HealthStatus.WARNING)
    assert metric.lbl_val.text() == "85%"

    mod_card = ModuleCard("Sensors", "LiDAR & Cam", status_text="RUNNING", status=HealthStatus.RUNNING)
    assert mod_card.lbl_title.text() == "Sensors"
    mod_card.set_warning_count(2)
    assert "2 cảnh báo" in mod_card.lbl_bottom.text()


def test_overview_navigation_and_warnings(app):
    overview = SystemOverviewScreen()
    nav_mock = MagicMock()
    back_mock = MagicMock()
    overview.navigate_to_module.connect(nav_mock)
    overview.back_to_admin_requested.connect(back_mock)

    overview.card_sensors.click()
    nav_mock.assert_called_with("sensors")

    overview.card_hardware.click()
    nav_mock.assert_called_with("hardware")

    overview.card_ros2.click()
    nav_mock.assert_called_with("ros2")

    overview.card_diagnostics.click()
    nav_mock.assert_called_with("diagnostics")

    overview.header.btn_back.click()
    assert back_mock.called

    # Warning banner should show when warnings present
    s = SystemTelemetrySnapshot(lidar_hz=0.0)
    h = HostTelemetrySnapshot(cpu_temp=78.0)
    p = PowerWaterSnapshot(battery_percent=18.0)
    overview.update_telemetry(s, h, p)
    assert not overview.warning_bar.isHidden()
    assert "cảnh báo" in overview.lbl_warning_text.text().lower()

    overview.close()


def test_detail_screens(app):
    sensors_screen = SensorsAIDetailScreen()
    s_snap = SystemTelemetrySnapshot(lidar_hz=12.0, tf_map_odom_ok=True, tf_odom_base_ok=True, yolo_active=True)
    sensors_screen.update_telemetry(s_snap)
    assert "12.0 Hz" in sensors_screen.card_lidar.lbl_val.text()
    assert "ĐÃ ĐỊNH VỊ" in sensors_screen.lbl_tf_map.text()
    sensors_screen.close()

    hw_screen = HardwareDetailScreen()
    h_snap = HostTelemetrySnapshot(
        cpu_pct=34.0, cpu_temp=52.0, local_ip="192.168.1.100",
        router_ok=True, router_port=20128, stt_ok=True, stt_latency_ms=28,
    )
    p_snap = PowerWaterSnapshot(battery_percent=85.0, water_liter=2.1, kettle_present=True)
    hw_screen.update_host_telemetry(h_snap)
    hw_screen.update_power_water(p_snap)
    assert "85%" in hw_screen.lbl_bat_pct.text()
    assert "2.1 L" in hw_screen.lbl_water_val.text()
    assert "192.168.1.100" in hw_screen.card_ip.lbl_val.text()
    assert "ONLINE" in hw_screen.card_router.lbl_val.text()
    assert "OK (28ms)" in hw_screen.card_stt.lbl_val.text()
    hw_screen.close()

    ros2_screen = ROS2SystemDetailScreen()
    ros2_screen.update_layer_status({"layer_hardware": True, "layer_navigation": False})
    assert ros2_screen.layer_rows["layer_hardware"]["status_lbl"].text() == "RUNNING"
    assert ros2_screen.layer_rows["layer_navigation"]["status_lbl"].text() == "STOPPED"
    ros2_screen.close()

    diag_screen = DiagnosticsDetailScreen()
    diag_screen.cache_telemetry(sensors=s_snap, host=h_snap, power=p_snap)
    diag_screen._start_preflight_check()
    for _ in range(8):
        diag_screen._on_preflight_tick()
    assert "[ĐẠT]" in diag_screen.preflight_items["bat"][1].text()
    diag_screen.close()
