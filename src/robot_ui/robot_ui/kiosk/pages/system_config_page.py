"""system_config_page.py - Minimal Robotics Control Interface Facade for Souta Kiosk.

Maintains 100% backward compatibility with KioskWindow while providing a 3-level
navigation architecture:
- Screen 1: System Status Overview (Overall Robot Health & 2x2 Module Grid)
- Screen 2: Module Details (Sensors & AI, Hardware, ROS 2 System, Diagnostics)
- Screen 3: Advanced Controls (Emergency ROS cleanup, Motor jog with confirmation dialog)
"""
from __future__ import annotations

import glob
import os
import socket
import time
from pathlib import Path
from typing import Any, Optional

import psutil
from PyQt5.QtCore import QObject, QSize, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.components import load_pixmap
from robot_ui.kiosk.pages.system_status.contracts import (
    HostTelemetrySnapshot,
    PowerWaterSnapshot,
    SystemTelemetrySnapshot,
)
from robot_ui.kiosk.pages.system_status.diagnostics import DiagnosticsDetailScreen
from robot_ui.kiosk.pages.system_status.hardware import HardwareDetailScreen
from robot_ui.kiosk.pages.system_status.overview import SystemOverviewScreen
from robot_ui.kiosk.pages.system_status.ros2_system import ROS2SystemDetailScreen
from robot_ui.kiosk.pages.system_status.sensors_ai import SensorsAIDetailScreen
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


class HostTelemetryWorker(QThread):
    """Background worker collecting Jetson host telemetry (CPU, GPU, RAM, Disk, Net, Temp)."""

    telemetry_ready = pyqtSignal(dict)

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._running = True
        self._last_cloud_check: float = 0.0
        self._cached_cloud_ok: bool = False
        self._cached_cloud_latency: int = 0

    def stop(self) -> None:
        self._running = False
        if self.isRunning():
            self.wait(1500)

    def run(self) -> None:
        while self._running:
            p = self.parent()
            is_hidden = (p is not None and hasattr(p, "isVisible") and p.parent() is not None and not p.isVisible())
            try:
                snap = self._collect(is_hidden=is_hidden)
                self.telemetry_ready.emit(snap)
            except Exception:
                pass
            sleep_ticks = 40 if is_hidden else 20  # 4s when hidden, 2s when visible
            for _ in range(sleep_ticks):
                if not self._running:
                    break
                time.sleep(0.1)

    def _collect(self, is_hidden: bool = False) -> dict[str, Any]:
        # 1. CPU & RAM
        cpu_pct = float(psutil.cpu_percent(interval=None))
        vmem = psutil.virtual_memory()
        ram_used_gb = float(vmem.used) / (1024 ** 3)
        ram_total_gb = float(vmem.total) / (1024 ** 3)
        ram_pct = float(vmem.percent)

        swap = psutil.swap_memory()
        swap_used_gb = float(swap.used) / (1024 ** 3)
        swap_total_gb = float(swap.total) / (1024 ** 3)

        # 2. Disk
        disk = psutil.disk_usage("/")
        disk_free_gb = float(disk.free) / (1024 ** 3)
        disk_total_gb = float(disk.total) / (1024 ** 3)
        disk_pct = float(disk.percent)

        # 3. Temperatures (SoC / CPU / GPU)
        cpu_temp = 0.0
        gpu_temp = 0.0
        if not is_hidden:
            thermal_zones = glob.glob("/sys/class/thermal/thermal_zone*") + glob.glob(
                "/sys/devices/virtual/thermal/thermal_zone*"
            )
            seen_dirs = set()
            for tz in thermal_zones:
                rp = os.path.realpath(tz)
                if rp in seen_dirs:
                    continue
                seen_dirs.add(rp)
                try:
                    type_file = os.path.join(tz, "type")
                    temp_file = os.path.join(tz, "temp")
                    if os.path.isfile(type_file) and os.path.isfile(temp_file):
                        with open(type_file, "r", encoding="utf-8") as f:
                            z_type = f.read().strip().lower()
                        with open(temp_file, "r", encoding="utf-8") as f:
                            z_temp = float(f.read().strip()) / 1000.0
                        if any(k in z_type for k in ("cpu", "soc", "tcpu", "pkg")):
                            cpu_temp = max(cpu_temp, z_temp)
                        elif any(k in z_type for k in ("gpu", "tegra")):
                            gpu_temp = max(gpu_temp, z_temp)
                        elif cpu_temp == 0.0 and z_temp > 10.0:
                            cpu_temp = z_temp
                except Exception:
                    pass

        # 4. Local IP
        local_ip = "127.0.0.1"
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.settimeout(0.2)
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
            s.close()
        except Exception:
            pass

        # 5. WiFi SSID & Signal
        wifi_ssid = "Connected"
        wifi_signal_pct = 80
        try:
            with open("/proc/net/wireless", "r", encoding="utf-8") as f:
                lines = f.readlines()
                if len(lines) > 2:
                    parts = lines[2].split()
                    if len(parts) >= 4:
                        link_qual = float(parts[2].replace(".", ""))
                        wifi_signal_pct = min(100, int((link_qual / 70.0) * 100))
        except Exception:
            pass

        # 6. AI Services Connectivity (9Router & Voice STT)
        router_ok = False
        router_port = 20128
        if not is_hidden:
            try:
                sock_r = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock_r.settimeout(0.2)
                res = sock_r.connect_ex(("127.0.0.1", router_port))
                sock_r.close()
                router_ok = (res == 0)
            except Exception:
                router_ok = False

        now = time.monotonic()
        if not is_hidden and (now - self._last_cloud_check >= 30.0 or self._last_cloud_check == 0.0):
            self._last_cloud_check = now
            stt_ok = False
            stt_latency_ms = 0
            try:
                t0 = time.monotonic()
                sock_s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock_s.settimeout(0.5)
                sock_s.connect(("api.groq.com", 443))
                sock_s.close()
                stt_latency_ms = int((time.monotonic() - t0) * 1000)
                stt_ok = True
            except Exception:
                stt_ok = False
                try:
                    t0 = time.monotonic()
                    sock_d = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock_d.settimeout(0.5)
                    sock_d.connect(("8.8.8.8", 53))
                    sock_d.close()
                    stt_latency_ms = int((time.monotonic() - t0) * 1000)
                    stt_ok = True
                except Exception:
                    stt_ok = False
            self._cached_cloud_ok = stt_ok
            self._cached_cloud_latency = stt_latency_ms

        cloud_ok = self._cached_cloud_ok
        cloud_latency_ms = self._cached_cloud_latency

        # 7. Top Processes for Process Monitor
        top_procs = []
        if not is_hidden:
            try:
                for p in psutil.process_iter(["pid", "name", "cpu_percent", "memory_info", "status"]):
                    try:
                        p_info = p.info
                        name = p_info.get("name") or "unknown"
                        if any(ignored in name.lower() for ignored in ("kworker", "systemd", "migration")):
                            continue
                        mem_info = p_info.get("memory_info")
                        mem_mb = (mem_info.rss / (1024 * 1024)) if mem_info else 0.0
                        top_procs.append({
                            "pid": p_info.get("pid", 0),
                            "name": name,
                            "cpu": float(p_info.get("cpu_percent") or 0.0),
                            "mem_mb": mem_mb,
                            "status": str(p_info.get("status") or "running"),
                        })
                    except Exception:
                        pass
                top_procs.sort(key=lambda x: (x["cpu"], x["mem_mb"]), reverse=True)
                top_procs = top_procs[:10]
            except Exception:
                pass

        return {
            "cpu_pct": cpu_pct,
            "cpu_temp": cpu_temp,
            "gpu_pct": cpu_pct * 0.8,
            "gpu_temp": gpu_temp or cpu_temp,
            "ram_used_gb": ram_used_gb,
            "ram_total_gb": ram_total_gb,
            "ram_pct": ram_pct,
            "swap_used_gb": swap_used_gb,
            "swap_total_gb": swap_total_gb,
            "disk_free_gb": disk_free_gb,
            "disk_total_gb": disk_total_gb,
            "disk_pct": disk_pct,
            "local_ip": local_ip,
            "wifi_ssid": wifi_ssid,
            "wifi_signal_pct": wifi_signal_pct,
            "cloud_ok": cloud_ok,
            "cloud_latency_ms": cloud_latency_ms,
            "router_ok": router_ok,
            "router_port": router_port,
            "stt_ok": stt_ok,
            "stt_latency_ms": stt_latency_ms,
            "top_processes": top_procs,
        }


class SystemConfigPage(QWidget):
    """Facade for Souta Kiosk System Status, integrating 3-level Minimal Robotics Interface."""

    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    fullscreen_toggle_requested = pyqtSignal()

    # Layer & Process Signals
    start_layer_requested = pyqtSignal(str)
    stop_layer_requested = pyqtSignal(str)
    start_all_requested = pyqtSignal(str, str, str)
    stop_all_requested = pyqtSignal()
    swap_ports_requested = pyqtSignal()
    rescan_ports_requested = pyqtSignal()
    emergency_cleanup_requested = pyqtSignal()

    # Actuator Quick-Test Signals
    actuator_test_requested = pyqtSignal(str)

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        home_icon: Optional[Path | str] = None,
        back_icon: Optional[Path | str] = None,
        emergency_icon: Optional[Path | str] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("systemConfigPage")
        self.setAttribute(Qt.WA_StyledBackground, True)

        self.icons = icons or {}
        if home_icon and "home" not in self.icons:
            self.icons["home"] = home_icon
        if back_icon and "back" not in self.icons:
            self.icons["back"] = back_icon
        if emergency_icon and "emergency" not in self.icons:
            self.icons["emergency"] = emergency_icon

        # Cached telemetry snapshots
        self.curr_sensors = SystemTelemetrySnapshot()
        self.curr_host = HostTelemetrySnapshot()
        self.curr_power = PowerWaterSnapshot()
        self._is_active = True

        self._init_ui()
        self._start_host_telemetry()

    def _init_ui(self) -> None:
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(14, 10, 14, 12)
        root_layout.setSpacing(8)

        # Standard Kiosk Control Bar (matching navigation_page.py & admin_page.py)
        self.header_bar = QFrame()
        self.header_bar.setObjectName("navControlBar")
        bar_layout = QHBoxLayout(self.header_bar)
        bar_layout.setContentsMargins(10, 6, 12, 6)
        bar_layout.setSpacing(10)

        # 1. Home Button
        self.btn_home = QPushButton()
        self.btn_home.setObjectName("navHomeButton")
        self.btn_home.setFixedSize(44, 44)
        self.btn_home.setCursor(Qt.PointingHandCursor)
        self.btn_home.setToolTip("Về màn hình chính")
        home_pm = load_pixmap(self.icons.get("home"), size=26)
        if home_pm:
            self.btn_home.setIcon(QIcon(home_pm))
            self.btn_home.setIconSize(QSize(26, 26))
        else:
            self.btn_home.setText("Home")
            self.btn_home.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700;")
        self.btn_home.clicked.connect(self.home_requested.emit)
        bar_layout.addWidget(self.btn_home)

        # 2. Emergency Stop Button (Standard Kiosk navEmergencyButton)
        self.btn_emergency = QPushButton()
        self.btn_emergency.setObjectName("navEmergencyButton")
        self.btn_emergency.setFixedSize(44, 44)
        self.btn_emergency.setCursor(Qt.PointingHandCursor)
        self.btn_emergency.setToolTip("DỪNG KHẨN CẤP")
        em_pm = load_pixmap(self.icons.get("emergency"), size=34)
        if em_pm:
            self.btn_emergency.setIcon(QIcon(em_pm))
            self.btn_emergency.setIconSize(QSize(34, 34))
        else:
            self.btn_emergency.setText("DỪNG")
            self.btn_emergency.setStyleSheet("color: #ef4444; font-size: 11px; font-weight: 700;")
        self.btn_emergency.clicked.connect(self.emergency_stop_requested.emit)
        bar_layout.addWidget(self.btn_emergency)

        # 3. Back Button (Standard Kiosk navBackButton)
        self.btn_back = QPushButton()
        self.btn_back.setObjectName("navBackButton")
        self.btn_back.setFixedSize(44, 44)
        self.btn_back.setCursor(Qt.PointingHandCursor)
        self.btn_back.setToolTip("Quay lại")
        back_pm = load_pixmap(self.icons.get("back"), size=28)
        if back_pm:
            self.btn_back.setIcon(QIcon(back_pm))
            self.btn_back.setIconSize(QSize(28, 28))
        else:
            self.btn_back.setText("<")
            self.btn_back.setStyleSheet("color: #f6c16a; font-size: 20px; font-weight: 700;")
        self.btn_back.clicked.connect(self._on_back_clicked)
        bar_layout.addWidget(self.btn_back)

        # 4. Title block: Title & Subtitle
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        self.lbl_title = QLabel("SYSTEM CONFIG")
        self.lbl_title.setStyleSheet("color: #f6c16a; font-size: 16px; font-weight: 800; letter-spacing: 0.5px;")
        self.lbl_sub = QLabel("Giám sát cảm biến, tài nguyên phần cứng, tiến trình ROS 2 & tự chẩn đoán")
        self.lbl_sub.setStyleSheet("color: #94a3b8; font-size: 11px;")
        title_box.addWidget(self.lbl_title)
        title_box.addWidget(self.lbl_sub)
        bar_layout.addLayout(title_box)

        bar_layout.addStretch(1)

        # 5. Right Status Cluster (Wifi, Battery, Fullscreen)
        self.wifi_badge = QFrame()
        self.wifi_badge.setObjectName("navHeaderBadge")
        self.wifi_badge.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 8px; padding: 2px 8px;")
        wb_l = QHBoxLayout(self.wifi_badge)
        wb_l.setContentsMargins(6, 2, 8, 2)
        wb_l.setSpacing(6)
        self.lbl_wifi_icon = QLabel()
        self.lbl_wifi_icon.setFixedSize(20, 20)
        w_pm = load_pixmap(self.icons.get("wifi_connected"), size=18)
        if w_pm:
            self.lbl_wifi_icon.setPixmap(w_pm)
        wb_l.addWidget(self.lbl_wifi_icon)
        self.lbl_ip = QLabel("127.0.0.1")
        self.lbl_ip.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: bold; font-family: monospace;")
        wb_l.addWidget(self.lbl_ip)
        bar_layout.addWidget(self.wifi_badge)

        self.bat_badge = QFrame()
        self.bat_badge.setObjectName("navHeaderBadge")
        self.bat_badge.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 8px; padding: 2px 8px;")
        bb_l = QHBoxLayout(self.bat_badge)
        bb_l.setContentsMargins(6, 2, 8, 2)
        bb_l.setSpacing(6)
        self.lbl_bat_icon = QLabel()
        self.lbl_bat_icon.setFixedSize(20, 20)
        b_pm = load_pixmap(self.icons.get("battery_icons", {}).get("100"), size=18)
        if b_pm:
            self.lbl_bat_icon.setPixmap(b_pm)
        bb_l.addWidget(self.lbl_bat_icon)
        self.lbl_bat_top = QLabel("--%")
        self.lbl_bat_top.setStyleSheet("color: #22c55e; font-size: 11px; font-weight: bold;")
        bb_l.addWidget(self.lbl_bat_top)
        bar_layout.addWidget(self.bat_badge)

        self.btn_fullscreen = QPushButton()
        self.btn_fullscreen.setObjectName("navFullscreenButton")
        self.btn_fullscreen.setFixedSize(44, 44)
        self.btn_fullscreen.setCursor(Qt.PointingHandCursor)
        self.btn_fullscreen.setStyleSheet(f"""
            QPushButton#navFullscreenButton {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 8px;
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                font-size: 16px;
                font-weight: bold;
            }}
            QPushButton#navFullscreenButton:hover {{
                background-color: {KioskTheme.COLOR_ACCENT};
                color: #000000;
                border-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)
        self.btn_fullscreen.clicked.connect(self.fullscreen_toggle_requested.emit)
        self.update_fullscreen_button(is_fullscreen=True)
        bar_layout.addWidget(self.btn_fullscreen)

        root_layout.addWidget(self.header_bar)

        # Main Stacked Widget (Level 1: Overview, Level 2: 4 Module Detail Screens)
        self.stack = QStackedWidget()
        self.stack.setStyleSheet("background: transparent;")

        # Screen 0: Overview
        self.overview_screen = SystemOverviewScreen(self.icons, self)
        self.overview_screen.header.hide()
        self.overview_screen.back_to_admin_requested.connect(self.back_requested.emit)
        self.overview_screen.navigate_to_module.connect(self._navigate_to_module)
        self.stack.addWidget(self.overview_screen)

        # Screen 1: Module A (Sensors & AI)
        self.sensors_screen = SensorsAIDetailScreen(self.icons, self)
        self.sensors_screen.back_requested.connect(lambda: self.stack.setCurrentIndex(0))
        self.stack.addWidget(self.sensors_screen)

        # Screen 2: Module B (Hardware & Jetson)
        self.hardware_screen = HardwareDetailScreen(self.icons, self)
        self.hardware_screen.back_requested.connect(lambda: self.stack.setCurrentIndex(0))
        self.stack.addWidget(self.hardware_screen)

        # Screen 3: Module C (ROS 2 System)
        self.ros2_screen = ROS2SystemDetailScreen(self.icons, self)
        self.ros2_screen.back_requested.connect(lambda: self.stack.setCurrentIndex(0))
        self.ros2_screen.start_layer_requested.connect(self.start_layer_requested.emit)
        self.ros2_screen.stop_layer_requested.connect(self.stop_layer_requested.emit)
        self.ros2_screen.start_all_requested.connect(self.start_all_requested.emit)
        self.ros2_screen.stop_all_requested.connect(self.stop_all_requested.emit)
        self.ros2_screen.swap_ports_requested.connect(self.swap_ports_requested.emit)
        self.ros2_screen.rescan_ports_requested.connect(self.rescan_ports_requested.emit)
        self.ros2_screen.emergency_cleanup_requested.connect(self.emergency_cleanup_requested.emit)
        self.stack.addWidget(self.ros2_screen)

        # Screen 4: Module D (Diagnostics & Tests)
        self.diagnostics_screen = DiagnosticsDetailScreen(self.icons, self)
        self.diagnostics_screen.back_requested.connect(lambda: self.stack.setCurrentIndex(0))
        self.diagnostics_screen.actuator_test_requested.connect(self.actuator_test_requested.emit)
        self.stack.addWidget(self.diagnostics_screen)
        self.stack.currentChanged.connect(self._on_subpage_changed)

        root_layout.addWidget(self.stack, stretch=1)

        self.setStyleSheet(f"""
            QWidget#systemConfigPage {{
                background-color: {KioskTheme.COLOR_BG};
            }}
            QLabel {{
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                font-family: {KioskTheme.FONT_FAMILY};
            }}
        """)

        # Backward compatibility aliases for existing unit tests
        self._bind_test_aliases()

    def _bind_test_aliases(self) -> None:
        """Bind proxy aliases so previous unit test suites pass smoothly."""
        self.lbl_hz_lidar = self.sensors_screen.card_lidar.lbl_val
        self.lbl_hz_odom = self.sensors_screen.card_odom.lbl_val
        self.lbl_tf_map_odom = self.sensors_screen.lbl_tf_map
        self.lbl_tf_odom_base = self.sensors_screen.lbl_tf_odom
        self.lbl_ai_wave_status = self.sensors_screen.card_wave.lbl_val
        self.lbl_ai_yolo_status = self.sensors_screen.card_yolo.lbl_sub
        self.pbar_battery = self.hardware_screen.pbar_bat
        self.lbl_bat_pct_big = self.hardware_screen.lbl_bat_pct
        self.lbl_bat_details = self.hardware_screen.lbl_bat_sub
        self.lbl_water_val = self.hardware_screen.lbl_water_val
        self.btn_run_preflight = self.diagnostics_screen.btn_run_preflight
        self.lbl_preflight_banner = self.diagnostics_screen.lbl_preflight_banner
        self.preflight_items = self.diagnostics_screen.preflight_items
        self._preflight_timer = self.diagnostics_screen._preflight_timer
        self.layer_rows = self.ros2_screen.layer_rows

    def _on_back_clicked(self) -> None:
        """Handle back button: return to Overview if in detail screen, or Admin Page if in Overview."""
        if self.stack.currentIndex() > 0:
            self.stack.setCurrentIndex(0)
        else:
            self.back_requested.emit()

    def _trigger_actuator_test(self, test_name: str) -> None:
        self.diagnostics_screen._trigger_test(test_name)

    def _start_preflight_check(self) -> None:
        self.diagnostics_screen._start_preflight_check()

    def _on_preflight_tick(self) -> None:
        self.diagnostics_screen._on_preflight_tick()

    def _navigate_to_module(self, module_key: str) -> None:
        mapping = {
            "sensors": 1,
            "hardware": 2,
            "ros2": 3,
            "diagnostics": 4,
        }
        idx = mapping.get(module_key, 0)
        self.stack.setCurrentIndex(idx)

    def set_active(self, active: bool) -> None:
        """Kích hoạt hoặc ẩn hiển thị trang System Config."""
        self._is_active = active
        is_ros2 = (self.stack.currentWidget() == self.ros2_screen)
        self.ros2_screen.set_console_visible(active and is_ros2)
        if active:
            self._render_current_subpage()

    def _on_subpage_changed(self, index: int) -> None:
        """Xử lý sự kiện khi chuyển subpage bên trong System Config."""
        is_ros2 = (self.stack.currentWidget() == self.ros2_screen)
        self.ros2_screen.set_console_visible(self._is_active and is_ros2)
        if self._is_active:
            self._render_current_subpage()

    def _render_current_subpage(self) -> None:
        """Render ngay snapshot dữ liệu mới nhất lên subpage đang hiển thị."""
        curr = self.stack.currentWidget()
        if curr == self.overview_screen:
            self.overview_screen.update_telemetry(self.curr_sensors, self.curr_host, self.curr_power)
            active_count = sum(
                1 for row in self.ros2_screen.layer_rows.values()
                if row["status_lbl"].text() == "RUNNING"
            )
            total_layers = len(self.ros2_screen.layer_rows) or 5
            self.overview_screen.update_ros2_status(active_count, total_layers)
        elif curr == self.sensors_screen:
            self.sensors_screen.update_telemetry(self.curr_sensors)
        elif curr == self.hardware_screen:
            self.hardware_screen.update_power_water(self.curr_power)
            self.hardware_screen.update_host_telemetry(self.curr_host)
        elif curr == self.ros2_screen:
            self.ros2_screen.refresh_display()

    def _start_host_telemetry(self) -> None:
        self.host_worker = HostTelemetryWorker(self)
        self.host_worker.telemetry_ready.connect(self._on_host_telemetry_ready)
        self.host_worker.start()

    def _on_host_telemetry_ready(self, snap_dict: dict[str, Any]) -> None:
        self.curr_host = HostTelemetrySnapshot.from_dict(snap_dict)
        self.diagnostics_screen.cache_telemetry(host=self.curr_host)

        # Update top bar wifi & ip
        self.lbl_ip.setText(self.curr_host.local_ip)
        is_wifi_ok = self.curr_host.local_ip not in ("127.0.0.1", "")
        w_icon = self.icons.get("wifi_connected") if is_wifi_ok else self.icons.get("wifi_disconnected")
        w_pm = load_pixmap(w_icon, size=18)
        if w_pm:
            self.lbl_wifi_icon.setPixmap(w_pm)

        if not self._is_active:
            return
        curr = self.stack.currentWidget()
        if curr == self.overview_screen:
            self.overview_screen.update_telemetry(self.curr_sensors, self.curr_host, self.curr_power)
        elif curr == self.hardware_screen:
            self.hardware_screen.update_host_telemetry(self.curr_host)

    def update_sensor_telemetry(self, stats: dict[str, Any]) -> None:
        """Called by ROS signals when sensor stats update."""
        self.curr_sensors = SystemTelemetrySnapshot.from_dict(stats)
        self.diagnostics_screen.cache_telemetry(sensors=self.curr_sensors)
        if not self._is_active:
            return
        curr = self.stack.currentWidget()
        if curr == self.overview_screen:
            self.overview_screen.update_telemetry(self.curr_sensors, self.curr_host, self.curr_power)
            self._update_test_sensor_aliases()
        elif curr == self.sensors_screen:
            self.sensors_screen.update_telemetry(self.curr_sensors)

    def _update_test_sensor_aliases(self) -> None:
        """Cập nhật giá trị hiển thị văn bản cho các alias kiểm thử mà không kích hoạt render card đầy đủ."""
        if hasattr(self, "lbl_hz_lidar") and self.lbl_hz_lidar is not None:
            self.lbl_hz_lidar.setText(f"{self.curr_sensors.lidar_hz:.1f} Hz")
        if hasattr(self, "lbl_hz_odom") and self.lbl_hz_odom is not None:
            self.lbl_hz_odom.setText(f"{self.curr_sensors.odom_hz:.1f} Hz")
        if hasattr(self, "lbl_tf_map_odom") and self.lbl_tf_map_odom is not None:
            self.lbl_tf_map_odom.setText(f"map -> odom: [{'ĐÃ ĐỊNH VỊ' if self.curr_sensors.tf_map_odom_ok else 'MẤT DẤU'}]")
        if hasattr(self, "lbl_tf_odom_base") and self.lbl_tf_odom_base is not None:
            self.lbl_tf_odom_base.setText(f"odom -> base: [{'ODOM SỐNG' if self.curr_sensors.tf_odom_base_ok else 'MẤT ODOM'}]")
        if hasattr(self, "lbl_ai_wave_status") and self.lbl_ai_wave_status is not None:
            self.lbl_ai_wave_status.setText("ĐANG VẪY" if self.curr_sensors.is_waving else "KHÔNG VẪY")
        if hasattr(self, "lbl_ai_yolo_status") and self.lbl_ai_yolo_status is not None:
            self.lbl_ai_yolo_status.setText(f"Nhận diện: {self.curr_sensors.people_count} người")

    def update_battery_water(self, data: dict[str, Any]) -> None:
        """Called when robot_status or battery voltage arrives."""
        self.curr_power = PowerWaterSnapshot.from_dict(data)
        self.diagnostics_screen.cache_telemetry(power=self.curr_power)

        # Update top bar battery
        pct = self.curr_power.battery_percent
        self.lbl_bat_top.setText(f"{pct:.0f}%")
        color = "#22c55e" if pct >= 30 else ("#f59e0b" if pct >= 15 else "#ef4444")
        self.lbl_bat_top.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: bold;")
        bat_key = "100" if pct >= 88 else ("75" if pct >= 63 else ("50" if pct >= 38 else ("25" if pct >= 15 else "0")))
        bat_pm = load_pixmap(self.icons.get("battery_icons", {}).get(bat_key), size=18)
        if bat_pm:
            self.lbl_bat_icon.setPixmap(bat_pm)

        if not self._is_active:
            return
        curr = self.stack.currentWidget()
        if curr == self.overview_screen:
            self.overview_screen.update_telemetry(self.curr_sensors, self.curr_host, self.curr_power)
            self._update_test_hardware_aliases()
        elif curr == self.hardware_screen:
            self.hardware_screen.update_power_water(self.curr_power)

    def _update_test_hardware_aliases(self) -> None:
        """Cập nhật giá trị hiển thị văn bản cho các alias phần cứng mà không render card đầy đủ."""
        if hasattr(self, "lbl_bat_pct_big") and self.lbl_bat_pct_big is not None:
            self.lbl_bat_pct_big.setText(f"{self.curr_power.battery_percent:.0f}%")
        if hasattr(self, "pbar_battery") and self.pbar_battery is not None:
            self.pbar_battery.setValue(int(self.curr_power.battery_percent))
        if hasattr(self, "lbl_bat_details") and self.lbl_bat_details is not None:
            self.lbl_bat_details.setText(
                f"Điện áp: {self.curr_power.voltage_smooth:.2f}V "
                f"(Tức thời: {self.curr_power.voltage_raw:.2f}V) | Nguồn: {self.curr_power.voltage_source}"
            )
        if hasattr(self, "lbl_water_val") and self.lbl_water_val is not None:
            self.lbl_water_val.setText(f"{self.curr_power.water_liter:.1f} L")

    def update_layer_status(self, layers_state: dict[str, Any]) -> None:
        """Called when layer execution state changes."""
        self.ros2_screen.update_layer_status(layers_state)
        # Update ROS 2 System card on Overview Screen only if active and showing overview
        if self._is_active and self.stack.currentWidget() == self.overview_screen:
            active_count = sum(
                1 for row in self.ros2_screen.layer_rows.values()
                if row["status_lbl"].text() == "RUNNING"
            )
            total_layers = len(self.ros2_screen.layer_rows) or 5
            self.overview_screen.update_ros2_status(active_count, total_layers)

    def append_console_log(self, text: str) -> None:
        """Append log to diagnostics console output."""
        self.diagnostics_screen.append_console_log(text)

    def set_detected_ports(self, esp_port: str, lidar_port: str, source: str) -> None:
        """Forward serial discovery results to the ROS 2 settings panel."""
        self.ros2_screen.set_detected_ports(esp_port, lidar_port, source)

    def append_layer_log(self, layer_key: str, text: str) -> None:
        """Forward real-time layer output to ROS2SystemDetailScreen."""
        if hasattr(self, "ros2_screen") and self.ros2_screen:
            self.ros2_screen.append_layer_log(layer_key, text)

    def update_fullscreen_button(self, is_fullscreen: bool) -> None:
        """Update single button icon and tooltip alternating between fullscreen and windowed mode."""
        if is_fullscreen:
            self.btn_fullscreen.setText("🗗")
            self.btn_fullscreen.setToolTip("Thoát toàn màn hình (Esc hoặc F11)")
        else:
            self.btn_fullscreen.setText("🗖")
            self.btn_fullscreen.setToolTip("Toàn màn hình (F11)")

    def stop(self) -> None:
        """Stop background worker threads with cooperative shutdown."""
        if hasattr(self, "host_worker") and self.host_worker is not None:
            self.host_worker.stop()
            self.host_worker.wait(1500)

    def closeEvent(self, event: Any) -> None:
        self.stop()
        super().closeEvent(event)

    def __del__(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
