"""ros2_system.py - Screen 2: Module C (ROS 2 System Detail).

Organizes:
1. ROS 2 System Health & Process Hierarchy (Distinguishing Process Running vs Node Available).
2. 5-Layer Process Control: Hardware, Navigation, SLAM, Perception, Voice Interaction.
3. Peripheral Hardware Ports: USB/Serial bus scanner & swap interface.
4. Advanced Controls / Dangerous Actions: Emergency ROS 2 Process Cleanup with Confirmation Dialog.
"""
from __future__ import annotations

import glob
import time
from typing import Any, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QTextCursor
from PyQt5.QtWidgets import (
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
        self._logs_by_layer: dict[str, list[str]] = {
            "all": [],
            "layer1": [],
            "layer2": [],
            "layer3": [],
            "layer4": [],
            "layer5": [],
            "system": [],
        }
        self._current_filter: str = "all"
        self._auto_scroll: bool = True
        self._filter_buttons: dict[str, QPushButton] = {}
        self._init_ui()

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
            status_text="0/5 ACTIVE",
            status=HealthStatus.STOPPED,
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

            lbl_st = StatusPill("STOPPED", HealthStatus.STOPPED)
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
            btn_start.clicked.connect(lambda _, k=key: self.start_layer_requested.emit(k))
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
            btn_stop.clicked.connect(lambda _, k=key: self.stop_layer_requested.emit(k))
            rl.addWidget(btn_stop)

            l_layout.addWidget(row_frame)
            self.layer_rows[key] = {
                "status_lbl": lbl_st,
                "start_btn": btn_start,
                "stop_btn": btn_stop,
            }

        c_layout.addWidget(card_layers)

        # B. NHẬT KÝ TIẾN TRÌNH LAYER (LIVE LOG CONSOLE)
        card_log, log_layout = create_section_card("NHẬT KÝ TIẾN TRÌNH LAYER (LIVE LOG CONSOLE)", "#10B981")

        # Filter and control toolbar
        filter_bar = QHBoxLayout()
        filter_bar.setSpacing(6)

        filters = [
            ("all", "Tất Cả"),
            ("layer1", "L1: Phần Cứng"),
            ("layer2", "L2: Điều Hướng"),
            ("layer3", "L3: SLAM"),
            ("layer4", "L4: AI Vision"),
            ("layer5", "L5: Giọng Nói"),
        ]

        for f_key, f_label in filters:
            btn_f = QPushButton(f_label)
            btn_f.setFixedHeight(28)
            btn_f.setCursor(Qt.PointingHandCursor)
            btn_f.clicked.connect(lambda _, k=f_key: self._set_log_filter(k))
            filter_bar.addWidget(btn_f)
            self._filter_buttons[f_key] = btn_f

        filter_bar.addStretch()

        self.btn_autoscroll = QPushButton("Cuộn: BẬT")
        self.btn_autoscroll.setFixedHeight(28)
        self.btn_autoscroll.setCursor(Qt.PointingHandCursor)
        self.btn_autoscroll.clicked.connect(self._toggle_autoscroll)
        self._update_autoscroll_style()
        filter_bar.addWidget(self.btn_autoscroll)

        btn_clear = QPushButton("Xóa Log")
        btn_clear.setFixedHeight(28)
        btn_clear.setCursor(Qt.PointingHandCursor)
        btn_clear.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                color: {KioskTheme.COLOR_TEXT_MUTED};
                border-radius: 4px;
                padding: 0 10px;
                font-size: 11px;
                font-weight: 600;
            }}
            QPushButton:hover {{
                color: #ffffff;
                background-color: {KioskTheme.COLOR_CRITICAL_BG};
                border-color: {KioskTheme.COLOR_CRITICAL};
            }}
        """)
        btn_clear.clicked.connect(self._clear_current_log)
        filter_bar.addWidget(btn_clear)

        log_layout.addLayout(filter_bar)

        self.txt_layer_log = QTextEdit()
        self.txt_layer_log.setReadOnly(True)
        self.txt_layer_log.setMinimumHeight(150)
        self.txt_layer_log.setMaximumHeight(230)
        self.txt_layer_log.setStyleSheet(f"""
            QTextEdit {{
                background-color: #06090e;
                color: #10b981;
                font-family: 'JetBrains Mono', 'Fira Code', 'DejaVu Sans Mono', monospace;
                font-size: 11px;
                line-height: 1.4;
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 8px;
            }}
        """)
        self.txt_layer_log.setText("[CONSOLE] Hệ thống sẵn sàng. Bấm BẬT ở một Layer để xem luồng khởi động tiến trình...")
        log_layout.addWidget(self.txt_layer_log)

        c_layout.addWidget(card_log)
        self._update_filter_button_styles()

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

    LAYER_NAME_MAP = {
        "layer1": "layer_hardware",
        "layer2": "layer_navigation",
        "layer3": "layer_slam",
        "layer4": "layer_perception",
        "layer5": "layer_voice_interaction",
    }

    def update_layer_status(self, layers_state: dict[str, bool]) -> None:
        """Cập nhật trạng thái hiển thị cho từng Layer và Header counter."""
        for raw_k, is_on in layers_state.items():
            k = self.LAYER_NAME_MAP.get(raw_k, raw_k)
            if k in self.layer_rows:
                lbl = self.layer_rows[k]["status_lbl"]
                if is_on:
                    lbl.set_status("RUNNING", HealthStatus.RUNNING)
                else:
                    lbl.set_status("STOPPED", HealthStatus.STOPPED)
                if "start_btn" in self.layer_rows[k]:
                    self.layer_rows[k]["start_btn"].setEnabled(not is_on)
                if "stop_btn" in self.layer_rows[k]:
                    self.layer_rows[k]["stop_btn"].setEnabled(is_on)

        active_count = sum(
            1 for row in self.layer_rows.values()
            if row["status_lbl"].text() == "RUNNING"
        )
        total_layers = len(self.layer_rows)
        header_st = HealthStatus.RUNNING if active_count > 0 else HealthStatus.STOPPED
        self.header.set_status(f"{active_count}/{total_layers} ACTIVE", header_st)

    def _normalize_layer_key(self, raw_key: str) -> str:
        k = raw_key.lower().strip()
        if k in self.LAYER_NAME_MAP:
            return k
        rev_map = {v: k for k, v in self.LAYER_NAME_MAP.items()}
        if k in rev_map:
            return rev_map[k]
        if k.startswith("layer") and k[5:].isdigit():
            return k
        return "all" if k in ("all", "system") else k

    def append_layer_log(self, layer_key: str, text: str) -> None:
        """Thêm dòng log mới từ tiến trình Layer và hiển thị lên console."""
        norm_key = self._normalize_layer_key(layer_key)
        now_str = time.strftime("%H:%M:%S")
        prefix = norm_key.upper() if norm_key != "all" else "SYSTEM"
        formatted_line = f"[{now_str}] [{prefix}] {text}"

        for target_key in ("all", norm_key):
            if target_key not in self._logs_by_layer:
                self._logs_by_layer[target_key] = []
            buf = self._logs_by_layer[target_key]
            buf.append(formatted_line)
            if len(buf) > 300:
                buf.pop(0)

        if self._current_filter in ("all", norm_key):
            self.txt_layer_log.append(formatted_line)
            if self._auto_scroll:
                cursor = self.txt_layer_log.textCursor()
                cursor.movePosition(QTextCursor.End)
                self.txt_layer_log.setTextCursor(cursor)

    def _set_log_filter(self, filter_key: str) -> None:
        self._current_filter = filter_key
        self._update_filter_button_styles()
        logs = self._logs_by_layer.get(filter_key, [])
        if not logs:
            self.txt_layer_log.setText(f"[CONSOLE] Chưa có log cho {filter_key.upper()}.")
        else:
            self.txt_layer_log.setPlainText("\n".join(logs))
            if self._auto_scroll:
                cursor = self.txt_layer_log.textCursor()
                cursor.movePosition(QTextCursor.End)
                self.txt_layer_log.setTextCursor(cursor)

    def _update_filter_button_styles(self) -> None:
        for k, btn in self._filter_buttons.items():
            if k == self._current_filter:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: #064e3b;
                        border: 1px solid #10b981;
                        color: #10b981;
                        border-radius: 4px;
                        font-weight: 700;
                        font-size: 11px;
                        padding: 0 8px;
                    }}
                """)
            else:
                btn.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {KioskTheme.COLOR_SURFACE_ALT};
                        border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                        color: {KioskTheme.COLOR_TEXT_PRIMARY};
                        border-radius: 4px;
                        font-weight: 500;
                        font-size: 11px;
                        padding: 0 8px;
                    }}
                    QPushButton:hover {{
                        background-color: {KioskTheme.COLOR_SURFACE_ACTIVE};
                        border-color: {KioskTheme.COLOR_ACCENT};
                    }}
                """)

    def _toggle_autoscroll(self) -> None:
        self._auto_scroll = not self._auto_scroll
        self._update_autoscroll_style()

    def _update_autoscroll_style(self) -> None:
        if self._auto_scroll:
            self.btn_autoscroll.setText("Cuộn: BẬT")
            self.btn_autoscroll.setStyleSheet(f"""
                QPushButton {{
                    background-color: #064e3b;
                    border: 1px solid #10b981;
                    color: #10b981;
                    border-radius: 4px;
                    padding: 0 8px;
                    font-size: 11px;
                    font-weight: 700;
                }}
            """)
        else:
            self.btn_autoscroll.setText("Cuộn: TẮT")
            self.btn_autoscroll.setStyleSheet(f"""
                QPushButton {{
                    background-color: {KioskTheme.COLOR_SURFACE_ALT};
                    border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                    color: {KioskTheme.COLOR_TEXT_MUTED};
                    border-radius: 4px;
                    padding: 0 8px;
                    font-size: 11px;
                    font-weight: 600;
                }}
            """)

    def _clear_current_log(self) -> None:
        if self._current_filter == "all":
            for k in self._logs_by_layer:
                self._logs_by_layer[k].clear()
        else:
            if self._current_filter in self._logs_by_layer:
                self._logs_by_layer[self._current_filter].clear()
        self.txt_layer_log.setText("[CONSOLE] Đã xóa nhật ký tiến trình.")

    def _on_rescan_ports(self) -> None:
        try:
            from robot_ui.kiosk.serial_resolver import detect_serial_ports
            hb, lidar, method = detect_serial_ports()
            res_str = f"Hoverboard: {hb} | LiDAR: {lidar} ({method})"
        except Exception:
            res_str = ""
        ports = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        unique_ports = list(dict.fromkeys(ports))
        ports_str = ", ".join(unique_ports) if unique_ports else "Không phát hiện"
        if res_str:
            self.lbl_ports_info.setText(f"{res_str} | Tất cả cổng: {ports_str}")
        else:
            self.lbl_ports_info.setText(f"Cổng phát hiện: {ports_str}")

    def _on_swap_ports(self) -> None:
        self.lbl_ports_info.setText("Đã yêu cầu đảo cổng cấu hình Serial Hoverboard/ESP32.")

    def _on_emergency_cleanup_clicked(self) -> None:
        dialog = DangerousActionDialog(
            title="DỌN DẸP KHẨN CẤP TOÀN BỘ TIẾN TRÌNH ROS 2",
            message="Hành động này sẽ gửi tín hiệu SIGTERM/SIGKILL đến toàn bộ các node ROS 2, Nav2, SLAM và driver phần cứng. Hệ thống sẽ tạm dừng hoạt động và cần khởi động lại các layer.",
            action_button_text="TÔI XÁC NHẬN DỌN DẸP",
            parent=self,
        )
        if dialog.exec_():
            self.emergency_cleanup_requested.emit()
