"""overview.py - Screen 1: System Status Overview.

Features:
- Header with Back button (to Admin Page) & overall health pill
- Overall Robot Status Card with computed status & quick badges (Battery, Localization, Nav2)
- Dynamic Active Warning Bar (auto-hides when 0 warnings)
- 2x2 Module Grid for navigation into Module Detail Screens
- Tailored for 800x480 Kiosk touch display
"""
from __future__ import annotations

from typing import Any, Optional

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.components import (
    ModuleCard,
    PageHeader,
    StatusPill,
    load_pixmap,
)
from robot_ui.kiosk.pages.system_status.contracts import (
    HealthStatus,
    HostTelemetrySnapshot,
    PowerWaterSnapshot,
    SystemTelemetrySnapshot,
    compute_overall_health,
)
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


class SystemOverviewScreen(QWidget):
    """Screen 1: High-level System Status Overview for Souta Kiosk."""

    navigate_to_module = pyqtSignal(str)   # "sensors", "hardware", "ros2", "diagnostics"
    back_to_admin_requested = pyqtSignal()

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("systemOverviewScreen")
        self.icons = icons or {}
        self._current_warnings: list[str] = []

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 12, 18, 16)
        layout.setSpacing(10)

        # 1. Header with Back button (Touch Target >= 44px)
        self.header = PageHeader(
            title="SYSTEM STATUS",
            subtitle="Souta Robot · System Monitor",
            back_callback=self.back_to_admin_requested.emit,
            back_icon_path=self.icons.get("back"),
            status_text="UNKNOWN",
            status=HealthStatus.UNKNOWN,
        )
        layout.addLayout(self.header)

        # 2. Overall Robot Status Card
        self.card_overall = QFrame()
        self.card_overall.setObjectName("overallStatusCard")
        self.card_overall.setStyleSheet(f"""
            QFrame#overallStatusCard {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-left: 5px solid {KioskTheme.COLOR_ACCENT};
                border-radius: {KioskTheme.CARD_RADIUS};
            }}
        """)
        overall_layout = QVBoxLayout(self.card_overall)
        overall_layout.setContentsMargins(16, 12, 16, 12)
        overall_layout.setSpacing(8)

        # Top row: Robot status title & main state
        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        self.lbl_robot_icon = QLabel()
        self.lbl_robot_icon.setFixedSize(32, 32)
        r_pm = load_pixmap(self.icons.get("status"), size=28)
        if r_pm:
            self.lbl_robot_icon.setPixmap(r_pm)
        top_row.addWidget(self.lbl_robot_icon)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        self.lbl_overall_title = QLabel("TRẠNG THÁI TỔNG THỂ ROBOT")
        self.lbl_overall_title.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 11px; font-weight: 700; letter-spacing: 0.5px;")
        self.lbl_overall_headline = QLabel("ĐANG KHỞI TẠO...")
        self.lbl_overall_headline.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 16px; font-weight: 800;")
        title_col.addWidget(self.lbl_overall_title)
        title_col.addWidget(self.lbl_overall_headline)
        top_row.addLayout(title_col)
        top_row.addStretch()

        self.overall_pill = StatusPill("UNKNOWN", HealthStatus.UNKNOWN)
        top_row.addWidget(self.overall_pill)
        overall_layout.addLayout(top_row)

        # Bottom row: 3 Quick Telemetry Badges (Battery, Localization, Nav2)
        badges_row = QHBoxLayout()
        badges_row.setSpacing(8)

        self.badge_bat = self._create_quick_badge("PIN", "--%", HealthStatus.UNKNOWN)
        self.badge_loc = self._create_quick_badge("ĐỊNH VỊ", "CHƯA RÕ", HealthStatus.UNKNOWN)
        self.badge_nav = self._create_quick_badge("NAV2", "SẴN SÀNG", HealthStatus.HEALTHY)

        badges_row.addWidget(self.badge_bat)
        badges_row.addWidget(self.badge_loc)
        badges_row.addWidget(self.badge_nav)
        overall_layout.addLayout(badges_row)

        layout.addWidget(self.card_overall)

        # 3. Dynamic Warning Bar (Initially hidden)
        self.warning_bar = QFrame()
        self.warning_bar.setStyleSheet(f"""
            QFrame {{
                background-color: {KioskTheme.COLOR_WARNING_BG};
                border: 1px solid {KioskTheme.COLOR_WARNING};
                border-radius: 6px;
                padding: 6px 12px;
            }}
        """)
        wb_layout = QHBoxLayout(self.warning_bar)
        wb_layout.setContentsMargins(8, 4, 8, 4)
        self.lbl_warning_text = QLabel("Cảnh báo hệ thống")
        self.lbl_warning_text.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 11px; font-weight: bold;")
        wb_layout.addWidget(self.lbl_warning_text)
        wb_layout.addStretch()
        self.btn_warning_link = QPushButton("Chi tiết")
        self.btn_warning_link.setFixedHeight(28)
        self.btn_warning_link.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE}; color: {KioskTheme.COLOR_ACCENT}; font-size: 11px; font-weight: bold; padding: 2px 8px; border-radius: 4px;")
        self.btn_warning_link.clicked.connect(self._on_warning_detail_clicked)
        wb_layout.addWidget(self.btn_warning_link)
        self.warning_bar.hide()
        layout.addWidget(self.warning_bar)

        # 4. Module Grid 2x2 (Touch Targets >= 44px)
        grid = QGridLayout()
        grid.setSpacing(10)

        # Card 1: Sensors & AI
        self.card_sensors = ModuleCard(
            title="Sensors & AI",
            subtitle="LiDAR · Camera Astra · YOLO",
            icon_path=self.icons.get("status"),
            status_text="HEALTHY",
            status=HealthStatus.HEALTHY,
        )
        self.card_sensors.clicked.connect(lambda: self.navigate_to_module.emit("sensors"))
        grid.addWidget(self.card_sensors, 0, 0)

        # Card 2: Hardware
        self.card_hardware = ModuleCard(
            title="Hardware",
            subtitle="Jetson Orin · Pin 36V · Nước",
            icon_path=self.icons.get("status"),
            status_text="HEALTHY",
            status=HealthStatus.HEALTHY,
        )
        self.card_hardware.clicked.connect(lambda: self.navigate_to_module.emit("hardware"))
        grid.addWidget(self.card_hardware, 0, 1)

        # Card 3: ROS 2 System
        self.card_ros2 = ModuleCard(
            title="ROS 2 System",
            subtitle="Nodes · 5 Layers · Serial Ports",
            icon_path=self.icons.get("settings"),
            status_text="STOPPED",
            status=HealthStatus.STOPPED,
        )
        self.card_ros2.clicked.connect(lambda: self.navigate_to_module.emit("ros2"))
        grid.addWidget(self.card_ros2, 1, 0)

        # Card 4: Diagnostics
        self.card_diagnostics = ModuleCard(
            title="Diagnostics",
            subtitle="Pre-Flight Check · Quick Tests",
            icon_path=self.icons.get("settings"),
            status_text="STANDBY",
            status=HealthStatus.UNKNOWN,
        )
        self.card_diagnostics.clicked.connect(lambda: self.navigate_to_module.emit("diagnostics"))
        grid.addWidget(self.card_diagnostics, 1, 1)

        layout.addLayout(grid)
        layout.addStretch()

    def _create_quick_badge(self, label: str, value: str, status: HealthStatus) -> QFrame:
        badge = QFrame()
        badge.setStyleSheet(f"""
            QFrame {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 4px 10px;
            }}
        """)
        bl = QHBoxLayout(badge)
        bl.setContentsMargins(6, 4, 6, 4)
        bl.setSpacing(6)

        lbl_k = QLabel(label)
        lbl_k.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 10px; font-weight: bold;")
        bl.addWidget(lbl_k)

        lbl_v = QLabel(value)
        color = KioskTheme.get_status_color(status)
        lbl_v.setStyleSheet(f"color: {color}; font-size: 11px; font-weight: 800;")
        bl.addWidget(lbl_v)
        badge.lbl_val = lbl_v  # attach for fast access
        return badge

    def update_telemetry(
        self,
        sensors: SystemTelemetrySnapshot,
        host: HostTelemetrySnapshot,
        power: PowerWaterSnapshot,
    ) -> None:
        """Update Overview cards & calculated overall health status."""
        health, headline, warnings = compute_overall_health(sensors, host, power)

        # Header status
        self.header.set_status(health.value, health)

        # Overall Status Card
        self.lbl_overall_headline.setText(headline)
        self.overall_pill.set_status(health.value, health)
        border_color = KioskTheme.get_status_color(health)
        self.card_overall.setStyleSheet(f"""
            QFrame#overallStatusCard {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-left: 5px solid {border_color};
                border-radius: {KioskTheme.CARD_RADIUS};
            }}
        """)

        # Quick Badges
        # 1. Battery
        bat_color = KioskTheme.COLOR_HEALTHY if power.battery_percent >= 30 else (
            KioskTheme.COLOR_WARNING if power.battery_percent >= 15 else KioskTheme.COLOR_CRITICAL
        )
        self.badge_bat.lbl_val.setText(f"{power.battery_percent:.0f}% ({power.voltage_smooth:.1f}V)")
        self.badge_bat.lbl_val.setStyleSheet(f"color: {bat_color}; font-size: 11px; font-weight: 800;")

        # 2. Localization
        if sensors.tf_map_odom_ok:
            self.badge_loc.lbl_val.setText("ĐÃ ĐỊNH VỊ")
            self.badge_loc.lbl_val.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 11px; font-weight: 800;")
        else:
            self.badge_loc.lbl_val.setText("MẤT DẤU")
            self.badge_loc.lbl_val.setStyleSheet(f"color: {KioskTheme.COLOR_CRITICAL}; font-size: 11px; font-weight: 800;")

        # 3. Dynamic Warning Bar
        self._current_warnings = warnings
        if warnings:
            self.lbl_warning_text.setText(f"Cảnh báo: {warnings[0]}")
            self.warning_bar.show()
        else:
            self.warning_bar.hide()

        # Update Module Cards
        # Module A (Sensors)
        s_health = HealthStatus.HEALTHY if (sensors.lidar_hz > 5.0 and sensors.tf_odom_base_ok) else HealthStatus.WARNING
        self.card_sensors.set_status(s_health.value, s_health, f"LiDAR: {sensors.lidar_hz:.1f} Hz | Odom: {sensors.odom_hz:.1f} Hz")

        # Module B (Hardware)
        h_health = HealthStatus.CRITICAL if host.cpu_temp >= 85 else (
            HealthStatus.WARNING if host.cpu_temp >= 75 else HealthStatus.HEALTHY
        )
        self.card_hardware.set_status(h_health.value, h_health, f"Jetson: {host.cpu_temp:.1f}°C | CPU: {host.cpu_pct:.0f}%")

    def update_ros2_status(self, active_count: int, total_count: int = 5) -> None:
        """Update ROS 2 System Module card status based on active layers count."""
        st = HealthStatus.HEALTHY if active_count >= 1 else HealthStatus.STOPPED
        status_text = f"{active_count}/{total_count} ACTIVE" if active_count > 0 else "STOPPED"
        subtitle = "Tất cả 5 tầng đang chạy" if active_count == total_count else (
            f"{active_count}/{total_count} tầng đang hoạt động" if active_count > 0 else "Các tầng đang dừng"
        )
        self.card_ros2.set_status(status_text, st, subtitle)

    def _on_warning_detail_clicked(self) -> None:
        """Route to appropriate detail screen based on top warning."""
        if not self._current_warnings:
            return
        w = self._current_warnings[0].lower()
        if any(k in w for k in ("pin", "nhiệt độ", "ram", "disk", "nước", "gateway", "bình")):
            self.navigate_to_module.emit("hardware")
        elif any(k in w for k in ("lidar", "camera", "tf", "vật cản", "ai", "yolo")):
            self.navigate_to_module.emit("sensors")
        elif any(k in w for k in ("layer", "tiến trình", "ros")):
            self.navigate_to_module.emit("ros2")
        else:
            self.navigate_to_module.emit("sensors")
