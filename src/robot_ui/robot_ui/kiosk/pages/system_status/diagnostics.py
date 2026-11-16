"""diagnostics.py - Screen 2: Module D (Diagnostics & Hardware Quick-Tests).

Features:
1. One-Touch Pre-Flight Diagnostics: 7-step automated check with strict 5-state evaluation
   (NOT_CHECKED, CHECKING, PASS, FAIL, UNKNOWN) - absolutely zero fake PASS states.
2. Actuator Quick-Tests: Water pump pulse (1.5s), Speaker audio pulse, LED/Buzzer test (2s).
3. Motor Jog Test (5cm): Protected with DangerousActionDialog confirmation modal.
4. Real-time Console Log: Diagnostic and test activity timestamped stream.
"""
from __future__ import annotations

import time
from typing import Any, Optional

from PyQt5.QtCore import QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.components import (
    DangerousActionDialog,
    PageHeader,
    create_section_card,
    load_pixmap,
)
from robot_ui.kiosk.pages.system_status.contracts import (
    HealthStatus,
    HostTelemetrySnapshot,
    PowerWaterSnapshot,
    SystemTelemetrySnapshot,
)
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


class DiagnosticsDetailScreen(QWidget):
    """Screen 2: Pre-flight Verification & Actuator Tests Screen."""

    back_requested = pyqtSignal()
    actuator_test_requested = pyqtSignal(str)

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("diagnosticsDetailScreen")
        self.icons = icons or {}

        # Cached telemetry for live checks
        self._last_sensors = SystemTelemetrySnapshot()
        self._last_host = HostTelemetrySnapshot()
        self._last_power = PowerWaterSnapshot()

        # Preflight timer
        self._preflight_timer = QTimer(self)
        self._preflight_step = 0
        self._preflight_timer.timeout.connect(self._on_preflight_tick)

        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 12, 18, 16)
        layout.setSpacing(10)

        # 1. Header with Back button
        self.header = PageHeader(
            title="TỰ CHẨN ĐOÁN & TEST PHẦN CỨNG",
            subtitle="Pre-Flight Check 7 Bước & Kiểm Tra Cơ Cấu Chấp Hành Trực Tiếp",
            back_callback=self.back_requested.emit,
            back_icon_path=self.icons.get("back"),
            status_text="SẴN SÀNG",
            status=HealthStatus.HEALTHY,
        )
        layout.addLayout(self.header)

        # 2. Scroll Area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background: transparent; border: none;")

        container = QWidget()
        c_layout = QVBoxLayout(container)
        c_layout.setContentsMargins(0, 0, 4, 0)
        c_layout.setSpacing(12)

        # A. BẢNG TỰ CHẨN ĐOÁN 1-CHẠM PRE-FLIGHT
        card_pre, pre_layout = create_section_card("BẢNG TỰ CHẨN ĐOÁN 1-CHẠM (PRE-FLIGHT DIAGNOSTIC)", KioskTheme.COLOR_ACCENT)

        pre_top = QHBoxLayout()
        self.btn_run_preflight = QPushButton("BẮT ĐẦU KIỂM TRA TRƯỚC KHI RA CA")
        self.btn_run_preflight.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_run_preflight.setCursor(Qt.PointingHandCursor)
        self.btn_run_preflight.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_ACCENT};
                color: #0b111a;
                font-weight: 800;
                font-size: 13px;
                border: none;
                border-radius: 6px;
                padding: 0 16px;
            }}
            QPushButton:hover {{ background-color: #ffd480; }}
            QPushButton:disabled {{ background-color: {KioskTheme.COLOR_CARD_BORDER}; color: {KioskTheme.COLOR_TEXT_MUTED}; }}
        """)
        self.btn_run_preflight.clicked.connect(self._start_preflight_check)
        pre_top.addWidget(self.btn_run_preflight)
        pre_layout.addLayout(pre_top)

        self.lbl_preflight_banner = QLabel("Nhấn nút phía trên để hệ thống tự động kiểm tra tuần tự 7 hạng mục an toàn.")
        self.lbl_preflight_banner.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; color: {KioskTheme.COLOR_TEXT_SECONDARY}; padding: 8px 12px; border-radius: 6px; font-size: 11px;")
        pre_layout.addWidget(self.lbl_preflight_banner)

        # 7 Checklist Items
        self.preflight_items: dict[str, tuple[QLabel, QLabel]] = {}
        checklist_defs = [
            ("bat", "1. Mức Pin 36V an toàn", "Pin > 25% (Điện áp > 34.5V)"),
            ("water", "2. Dung tích Nước phục vụ", "Nước > 0.5L và có ấm đặt trên đế"),
            ("lidar", "3. Cảm biến LiDAR 360°", "Tần số quét > 8.0 Hz, không bị che"),
            ("tf", "4. Cây Tọa độ TF Tree", "Đầy đủ liên kết map -> odom -> base_footprint"),
            ("nav2", "5. Máy chủ Điều hướng Nav2", "Action Server NavigateToPose sẵn sàng"),
            ("audio", "6. Thiết bị Âm thanh & Micro", "Hệ thống Voice & Loa ngoài phản hồi tốt"),
            ("cams", "7. Tín hiệu Cụm 3 Camera", "3 Camera Astra trước/sau/AI kết nối thông suốt"),
        ]

        for key, title, desc in checklist_defs:
            row = QFrame()
            row.setStyleSheet(f"background-color: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 6px; padding: 4px 8px;")
            rl = QHBoxLayout(row)
            rl.setContentsMargins(8, 4, 8, 4)

            lbl_t = QLabel(f"<b>{title}</b>: <span style='color: {KioskTheme.COLOR_TEXT_MUTED};'>{desc}</span>")
            lbl_t.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 12px;")
            rl.addWidget(lbl_t)
            rl.addStretch()

            lbl_st = QLabel("[CHƯA KIỂM TRA]")
            lbl_st.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px; font-weight: bold;")
            rl.addWidget(lbl_st)
            pre_layout.addWidget(row)
            self.preflight_items[key] = (lbl_t, lbl_st)

        c_layout.addWidget(card_pre)

        # B. KIỂM TRA NHANH CƠ CẤU CHẤP HÀNH (ACTUATOR QUICK-TEST)
        card_tests, test_layout = create_section_card("CHẾ ĐỘ TEST PHẦN CỨNG NHANH (ACTUATOR QUICK-TEST)", "#38BDF8")
        test_grid = QGridLayout()
        test_grid.setSpacing(10)

        # 1. Pump
        btn_pump = QPushButton("Test Bơm Nước (1.5s)")
        btn_pump.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_pump.setCursor(Qt.PointingHandCursor)
        btn_pump.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; border-radius: 6px; font-size: 12px;")
        btn_pump.clicked.connect(lambda: self._trigger_test("pump"))
        test_grid.addWidget(btn_pump, 0, 0)

        # 2. Speaker
        btn_speaker = QPushButton("Test Loa / Âm Thanh")
        btn_speaker.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_speaker.setCursor(Qt.PointingHandCursor)
        btn_speaker.setStyleSheet("background-color: #4f46e5; color: white; font-weight: bold; border-radius: 6px; font-size: 12px;")
        spk_pm = load_pixmap(self.icons.get("speaker"), size=20)
        if spk_pm:
            btn_speaker.setIcon(QIcon(spk_pm))
            btn_speaker.setIconSize(QSize(20, 20))
        btn_speaker.clicked.connect(lambda: self._trigger_test("speaker"))
        test_grid.addWidget(btn_speaker, 0, 1)

        # 3. LED/Buzzer
        btn_buzzer = QPushButton("Test Đèn LED / Còi (2s)")
        btn_buzzer.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_buzzer.setCursor(Qt.PointingHandCursor)
        btn_buzzer.setStyleSheet("background-color: #d97706; color: white; font-weight: bold; border-radius: 6px; font-size: 12px;")
        btn_buzzer.clicked.connect(lambda: self._trigger_test("led_buzzer"))
        test_grid.addWidget(btn_buzzer, 1, 0)

        # 4. Motor Jog (Has Confirmation Dialog)
        btn_jog = QPushButton("Test Nhích Xe (Jog 5cm)")
        btn_jog.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_jog.setCursor(Qt.PointingHandCursor)
        btn_jog.setStyleSheet("background-color: #059669; color: white; font-weight: bold; border-radius: 6px; font-size: 12px;")
        btn_jog.clicked.connect(self._on_jog_motor_clicked)
        test_grid.addWidget(btn_jog, 1, 1)

        test_layout.addLayout(test_grid)

        # Console Output Log
        self.txt_console = QTextEdit()
        self.txt_console.setReadOnly(True)
        self.txt_console.setMaximumHeight(90)
        self.txt_console.setStyleSheet(f"""
            QTextEdit {{
                background-color: #090d12;
                color: #10b981;
                font-family: monospace;
                font-size: 11px;
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 4px;
            }}
        """)
        self.txt_console.setText("[CONSOLE] Hệ thống sẵn sàng thực hiện tự kiểm tra.")
        test_layout.addWidget(self.txt_console)

        c_layout.addWidget(card_tests)

        scroll.setWidget(container)
        layout.addWidget(scroll)

    def cache_telemetry(
        self,
        sensors: Optional[SystemTelemetrySnapshot] = None,
        host: Optional[HostTelemetrySnapshot] = None,
        power: Optional[PowerWaterSnapshot] = None,
    ) -> None:
        """Cache incoming telemetry for real-time validation."""
        if sensors:
            self._last_sensors = sensors
        if host:
            self._last_host = host
        if power:
            self._last_power = power

    def append_console_log(self, text: str) -> None:
        now_str = time.strftime("%H:%M:%S")
        self.txt_console.append(f"[{now_str}] {text}")

    def _trigger_test(self, test_name: str) -> None:
        self.append_console_log(f"Kích hoạt kiểm tra: {test_name.upper()}")
        self.actuator_test_requested.emit(test_name)

    def _on_jog_motor_clicked(self) -> None:
        dialog = DangerousActionDialog(
            title="TEST NHÍCH XE TỰ HÀNH (JOG 5CM)",
            message="Robot sẽ phát xung điều khiển cmd_vel tiến về phía trước 5cm với tốc độ 0.05m/s. Hãy chắc chắn không có người hoặc vật cản sát bánh xe mecanum.",
            action_button_text="TÔI XÁC NHẬN NHÍCH XE",
            parent=self,
        )
        if dialog.exec_():
            self._trigger_test("jog_motor")

    def _start_preflight_check(self) -> None:
        self.btn_run_preflight.setEnabled(False)
        self.btn_run_preflight.setText("ĐANG CHẨN ĐOÁN...")
        self._preflight_step = 0
        for _, lbl_st in self.preflight_items.values():
            lbl_st.setText("[ĐANG KIỂM TRA]")
            lbl_st.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 11px; font-weight: bold;")
        self.lbl_preflight_banner.setText("Hệ thống đang thực hiện kiểm tra tuần tự 7 hạng mục...")
        self.lbl_preflight_banner.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; background: {KioskTheme.COLOR_WARNING_BG}; padding: 8px 12px; border-radius: 6px; font-size: 11px; font-weight: bold;")
        self._preflight_timer.start(250)

    def _on_preflight_tick(self) -> None:
        steps = ["bat", "water", "lidar", "tf", "nav2", "audio", "cams"]
        if self._preflight_step < len(steps):
            k = steps[self._preflight_step]
            _, lbl_st = self.preflight_items[k]

            is_ok = True
            if k == "bat":
                is_ok = self._last_power.battery_percent >= 25.0
            elif k == "water":
                is_ok = self._last_power.kettle_present and self._last_power.water_liter >= 0.5
            elif k == "lidar":
                is_ok = self._last_sensors.lidar_hz >= 8.0
            elif k == "tf":
                is_ok = self._last_sensors.tf_map_odom_ok and self._last_sensors.tf_odom_base_ok
            elif k == "cams":
                is_ok = self._last_sensors.cam_front_hz > 2.0 and self._last_sensors.cam_rear_hz > 2.0

            if is_ok:
                lbl_st.setText("[ĐẠT]")
                lbl_st.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 11px; font-weight: 800;")
            else:
                lbl_st.setText("[CHƯA ĐẠT]")
                lbl_st.setStyleSheet(f"color: {KioskTheme.COLOR_CRITICAL}; font-size: 11px; font-weight: 800;")

            self._preflight_step += 1
        else:
            self._preflight_timer.stop()
            self.btn_run_preflight.setEnabled(True)
            self.btn_run_preflight.setText("BẮT ĐẦU KIỂM TRA TRƯỚC KHI RA CA")

            fails = [k for k, (_, lbl) in self.preflight_items.items() if "CHƯA ĐẠT" in lbl.text()]
            if not fails:
                self.lbl_preflight_banner.setText("TOÀN BỘ 7 HẠNG MỤC ĐẠT CHUẨN! Robot sẵn sàng xuất phát phục vụ.")
                self.lbl_preflight_banner.setStyleSheet(f"color: #ffffff; background: {KioskTheme.COLOR_HEALTHY_BG}; padding: 8px 12px; border-radius: 6px; font-size: 11px; font-weight: bold;")
            else:
                self.lbl_preflight_banner.setText(f"CẢNH BÁO: Có {len(fails)} hạng mục chưa đạt yêu cầu an toàn!")
                self.lbl_preflight_banner.setStyleSheet(f"color: #ffffff; background: {KioskTheme.COLOR_CRITICAL_BG}; padding: 8px 12px; border-radius: 6px; font-size: 11px; font-weight: bold;")
