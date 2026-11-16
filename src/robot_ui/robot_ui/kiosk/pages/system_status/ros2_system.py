"""ros2_system.py - Screen 2: Module C (ROS 2 System Detail).

Organizes:
1. ROS 2 System Health & Process Hierarchy (Distinguishing Process Running vs Node Available).
2. 5-Layer Process Control: Hardware, Navigation, SLAM, Perception, Voice Interaction.
3. Peripheral Hardware Ports: USB/Serial bus scanner & swap interface.
4. Advanced Controls / Dangerous Actions: Emergency ROS 2 Process Cleanup with Confirmation Dialog.
"""
from __future__ import annotations

import glob
import re
import time
from typing import Any, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QFrame,
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
    StatusPill,
    create_section_card,
)
from robot_ui.kiosk.pages.system_status.contracts import HealthStatus
from robot_ui.kiosk.pages.system_status.theme import KioskTheme

ANSI_REGEX = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')


class ROS2SystemDetailScreen(QWidget):
    """Screen 2: ROS 2 System & Layer Management Screen."""

    back_requested = pyqtSignal()
    start_layer_requested = pyqtSignal(str)
    stop_layer_requested = pyqtSignal(str)
    emergency_cleanup_requested = pyqtSignal()

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("ros2SystemDetailScreen")
        self.icons = icons or {}
        self.layer_rows: dict[str, dict[str, Any]] = {}
        self._log_queue: list[str] = []
        self._init_ui()

        # Throttled log batching: cập nhật UI mỗi 60ms để chống lag/nghẽn Main Thread
        self._log_timer = QTimer(self)
        self._log_timer.setInterval(60)
        self._log_timer.timeout.connect(self._flush_log_queue)
        self._log_timer.start()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 12, 18, 16)
        layout.setSpacing(10)

        # 1. Header with Back button
        self.header = PageHeader(
            title="HỆ THỐNG ROS 2 & TIẾN TRÌNH",
            subtitle="Quản lý 5 Layer Kiến trúc, Cổng Serial & Khôi phục Khẩn cấp",
            back_callback=self.back_requested.emit,
            back_icon_path=self.icons.get("back"),
            status_text="5/5 ACTIVE",
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

        # A. QUẢN LÝ TIẾN TRÌNH 5 LAYER ROS 2
        card_layers, l_layout = create_section_card("QUẢN LÝ 5 LAYER KIẾN TRÚC PHẦN MỀM ROBOT", KioskTheme.COLOR_ACCENT)

        layers_info = [
            ("layer_hardware", "Layer 1: Giao Tiếp Phần Cứng & Động Cơ", "hoverboard_driver, esp32_bridge, bms"),
            ("layer_navigation", "Layer 2: Điều Hướng Tự Hành Nav2", "nav2_bringup, amcl, costmap_2d, controller_server"),
            ("layer_slam", "Layer 3: Lập Bản Đồ SLAM Toolbox", "slam_toolbox, map_saver_server"),
            ("layer_perception", "Layer 4: Thị Giác Máy Tính AI Astra", "astra_camera, yolo_detector, hand_wave_pose"),
            ("layer_voice_interaction", "Layer 5: Giọng Nói & Tương Tác", "wake_word_kws, llm_worker, audio_player"),
        ]

        for key, name, desc in layers_info:
            row_frame = QFrame()
            row_frame.setStyleSheet(f"""
                QFrame {{
                    background-color: {KioskTheme.COLOR_SURFACE_ALT};
                    border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                    border-radius: 6px;
                    padding: 4px 8px;
                }}
            """)
            rl = QHBoxLayout(row_frame)
            rl.setContentsMargins(10, 6, 10, 6)
            rl.setSpacing(10)

            title_box = QVBoxLayout()
            title_box.setSpacing(2)
            lbl_name = QLabel(name)
            lbl_name.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 13px; font-weight: 700;")
            lbl_desc = QLabel(desc)
            lbl_desc.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")
            title_box.addWidget(lbl_name)
            title_box.addWidget(lbl_desc)
            rl.addLayout(title_box)
            rl.addStretch()

            lbl_st = StatusPill("RUNNING", HealthStatus.RUNNING)
            rl.addWidget(lbl_st)

            btn_start = QPushButton("BẬT")
            btn_start.setFixedSize(54, KioskTheme.TOUCH_TARGET_MIN)
            btn_start.setCursor(Qt.PointingHandCursor)
            btn_start.setStyleSheet(f"""
                QPushButton {{
                    background-color: {KioskTheme.COLOR_HEALTHY_BG};
                    color: {KioskTheme.COLOR_HEALTHY};
                    border: 1px solid {KioskTheme.COLOR_HEALTHY};
                    border-radius: 6px;
                    font-weight: 800;
                    font-size: 12px;
                }}
                QPushButton:hover {{ background-color: {KioskTheme.COLOR_HEALTHY}; color: #ffffff; }}
            """)
            btn_start.clicked.connect(lambda _, k=key, n=name: self._on_start_layer_clicked(k, n))
            rl.addWidget(btn_start)

            btn_stop = QPushButton("TẮT")
            btn_stop.setFixedSize(54, KioskTheme.TOUCH_TARGET_MIN)
            btn_stop.setCursor(Qt.PointingHandCursor)
            btn_stop.setStyleSheet(f"""
                QPushButton {{
                    background-color: {KioskTheme.COLOR_CRITICAL_BG};
                    color: {KioskTheme.COLOR_CRITICAL};
                    border: 1px solid {KioskTheme.COLOR_CRITICAL};
                    border-radius: 6px;
                    font-weight: 800;
                    font-size: 12px;
                }}
                QPushButton:hover {{ background-color: {KioskTheme.COLOR_CRITICAL}; color: #ffffff; }}
            """)
            btn_stop.clicked.connect(lambda _, k=key, n=name: self._on_stop_layer_clicked(k, n))
            rl.addWidget(btn_stop)

            l_layout.addWidget(row_frame)
            self.layer_rows[key] = {
                "status_lbl": lbl_st,
                "start_btn": btn_start,
                "stop_btn": btn_stop,
            }

        c_layout.addWidget(card_layers)

        # B. NHẬT KÝ TIẾN TRÌNH ROS 2 HỆ THỐNG (STDOUT / STDERR CONSOLE)
        card_log, log_layout = create_section_card(
            "NHẬT KÝ TIẾN TRÌNH ROS 2 (REAL-TIME CONSOLE LOG)",
            KioskTheme.COLOR_ACCENT,
        )

        log_top = QHBoxLayout()
        log_top.setSpacing(10)

        self.chk_autoscroll = QCheckBox("Tự động cuộn xuống cuối")
        self.chk_autoscroll.setChecked(True)
        self.chk_autoscroll.setStyleSheet(f"""
            QCheckBox {{
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                font-size: 11px;
                font-weight: 700;
            }}
            QCheckBox::indicator {{
                width: 16px;
                height: 16px;
            }}
        """)
        log_top.addWidget(self.chk_autoscroll)
        log_top.addStretch()

        self.btn_clear_log = QPushButton("Xóa Log Hệ Thống")
        self.btn_clear_log.setFixedHeight(28)
        self.btn_clear_log.setCursor(Qt.PointingHandCursor)
        self.btn_clear_log.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                color: {KioskTheme.COLOR_TEXT_MUTED};
                font-size: 11px;
                font-weight: bold;
                padding: 0 12px;
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 4px;
            }}
            QPushButton:hover {{
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                border-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)
        self.btn_clear_log.clicked.connect(self._clear_log)
        log_top.addWidget(self.btn_clear_log)
        log_layout.addLayout(log_top)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.document().setMaximumBlockCount(1000)
        self.txt_log.setMinimumHeight(180)
        self.txt_log.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.txt_log.setStyleSheet(f"""
            QTextEdit {{
                background-color: #090d12;
                color: #10b981;
                font-family: 'DejaVu Sans Mono', Consolas, Monaco, monospace;
                font-size: 11px;
                line-height: 1.4;
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 6px;
            }}
        """)
        self.txt_log.setText("[SYSTEM] Sẵn sàng nhận nhật ký tiến trình ROS 2...")
        log_layout.addWidget(self.txt_log)
        c_layout.addWidget(card_log)

        # C. THIẾT BỊ NGOẠI VI & CỔNG SERIAL
        card_ports, port_layout = create_section_card("THIẾT BỊ NGOẠI VI & ĐƯỜNG TRUYỀN SERIAL", "#38BDF8")
        ports_row = QHBoxLayout()
        ports_row.setSpacing(10)

        self.btn_rescan_ports = QPushButton("Quét Lại Cổng Serial")
        self.btn_rescan_ports.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_rescan_ports.setCursor(Qt.PointingHandCursor)
        self.btn_rescan_ports.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                border-radius: 6px;
                font-weight: bold;
                padding: 0 14px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: {KioskTheme.COLOR_ACCENT}; }}
        """)
        self.btn_rescan_ports.clicked.connect(self._on_rescan_ports)
        ports_row.addWidget(self.btn_rescan_ports)

        self.btn_swap_ports = QPushButton("Hoán Đổi Cổng Hoverboard/ESP32")
        self.btn_swap_ports.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_swap_ports.setCursor(Qt.PointingHandCursor)
        self.btn_swap_ports.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                color: {KioskTheme.COLOR_ACCENT};
                border-radius: 6px;
                font-weight: bold;
                padding: 0 14px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: {KioskTheme.COLOR_ACCENT}; }}
        """)
        self.btn_swap_ports.clicked.connect(self._on_swap_ports)
        ports_row.addWidget(self.btn_swap_ports)
        port_layout.addLayout(ports_row)

        self.lbl_ports_info = QLabel("Cổng phát hiện: Đang quét...")
        self.lbl_ports_info.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px; padding: 4px 0;")
        port_layout.addWidget(self.lbl_ports_info)
        c_layout.addWidget(card_ports)

        # C. ADVANCED CONTROLS / RISKY ACTIONS (SEPARATE SECTION)
        card_danger, danger_layout = create_section_card("THAO TÁC NÂNG CAO & KHÔI PHỤC TIẾN TRÌNH", KioskTheme.COLOR_CRITICAL)

        lbl_danger_desc = QLabel("CẢNH BÁO: Thao tác bên dưới sẽ can thiệp trực tiếp vào toàn bộ tiến trình ROS 2 đang chạy. Chỉ sử dụng khi hệ thống gặp sự cố treo cứng hoặc xung đột node.")
        lbl_danger_desc.setWordWrap(True)
        lbl_danger_desc.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 11px; font-weight: 500;")
        danger_layout.addWidget(lbl_danger_desc)

        self.btn_emergency_cleanup = QPushButton("DỌN DẸP KHẨN CẤP TIẾN TRÌNH ROS 2 (EMERGENCY CLEANUP)")
        self.btn_emergency_cleanup.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_emergency_cleanup.setCursor(Qt.PointingHandCursor)
        self.btn_emergency_cleanup.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_CRITICAL_BG};
                color: {KioskTheme.COLOR_CRITICAL};
                border: 1px solid {KioskTheme.COLOR_CRITICAL};
                border-radius: 6px;
                font-weight: 800;
                font-size: 12px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_CRITICAL}; color: #ffffff; }}
        """)
        self.btn_emergency_cleanup.clicked.connect(self._on_emergency_cleanup_clicked)
        danger_layout.addWidget(self.btn_emergency_cleanup)

        c_layout.addWidget(card_danger)

        scroll.setWidget(container)
        layout.addWidget(scroll)

        # Initial ports scan
        self._on_rescan_ports()

    def update_layer_status(self, layers_state: dict[str, bool]) -> None:
        """Update visual indicators for layer processes."""
        for key, is_on in layers_state.items():
            if key in self.layer_rows:
                lbl = self.layer_rows[key]["status_lbl"]
                if is_on:
                    lbl.set_status("RUNNING", HealthStatus.RUNNING)
                else:
                    lbl.set_status("STOPPED", HealthStatus.STOPPED)

    def _on_rescan_ports(self) -> None:
        ports = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        unique_ports = list(dict.fromkeys(ports))
        ports_str = ", ".join(unique_ports) if unique_ports else "Không phát hiện cổng serial khả dụng"
        self.lbl_ports_info.setText(f"Cổng phát hiện: {ports_str}")

    def _on_swap_ports(self) -> None:
        self.lbl_ports_info.setText("Đã yêu cầu đảo cổng cấu hình Serial Hoverboard/ESP32.")

    def _on_start_layer_clicked(self, key: str, name: str) -> None:
        self.append_process_log(key, f"Gửi yêu cầu khởi động: {name}...")
        self.start_layer_requested.emit(key)

    def _on_stop_layer_clicked(self, key: str, name: str) -> None:
        self.append_process_log(key, f"Gửi yêu cầu dừng: {name}...")
        self.stop_layer_requested.emit(key)

    def append_process_log(self, layer: str, line: str, flush_now: bool = False) -> None:
        """Đưa dòng log vào hàng đợi để gom batch cập nhật UI chống giật lag."""
        clean_line = ANSI_REGEX.sub("", line).strip()
        if not clean_line:
            return
        now_str = time.strftime("%H:%M:%S")
        self._log_queue.append(f"[{now_str}] [{layer}] {clean_line}")
        # Nếu yêu cầu flush ngay hoặc hàng đợi vượt 100 dòng, đẩy ngay ra giao diện
        if flush_now or len(self._log_queue) >= 100:
            self._flush_log_queue()

    def _flush_log_queue(self) -> None:
        """Gom các dòng log đang chờ và ghi 1 lần duy nhất vào QTextEdit."""
        if not self._log_queue:
            return
        batch = "\n".join(self._log_queue)
        self._log_queue.clear()
        self.txt_log.append(batch)
        if self.chk_autoscroll.isChecked():
            sb = self.txt_log.verticalScrollBar()
            sb.setValue(sb.maximum())

    def _clear_log(self) -> None:
        self._log_queue.clear()
        self.txt_log.clear()
        self.txt_log.setText("[SYSTEM] Đã xóa toàn bộ nhật ký hiển thị.")

    def closeEvent(self, event: Any) -> None:
        if hasattr(self, "_log_timer") and self._log_timer.isActive():
            self._log_timer.stop()
        super().closeEvent(event)

    def _on_emergency_cleanup_clicked(self) -> None:
        dialog = DangerousActionDialog(
            title="DỌN DẸP KHẨN CẤP TOÀN BỘ TIẾN TRÌNH ROS 2",
            message="Hành động này sẽ gửi tín hiệu SIGTERM/SIGKILL đến toàn bộ các node ROS 2, Nav2, SLAM và driver phần cứng. Hệ thống sẽ tạm dừng hoạt động và cần khởi động lại các layer.",
            action_button_text="TÔI XÁC NHẬN DỌN DẸP",
            parent=self,
        )
        if dialog.exec_():
            self.append_process_log("EMERGENCY", "Yêu cầu dọn dẹp khẩn cấp toàn bộ tiến trình ROS 2...")
            self.emergency_cleanup_requested.emit()
