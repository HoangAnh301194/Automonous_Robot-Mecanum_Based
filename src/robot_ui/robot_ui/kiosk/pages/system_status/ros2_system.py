"""ros2_system.py - Screen 2: Module C (ROS 2 System Detail).

Organizes:
1. ROS 2 System Health & Process Hierarchy (Distinguishing Process Running vs Node Available).
2. 5-Layer Process Control: Hardware, Navigation, SLAM, Perception, Voice Interaction.
3. Peripheral Hardware Ports: USB/Serial bus scanner & swap interface.
4. Advanced Controls / Dangerous Actions: Emergency ROS 2 Process Cleanup with Confirmation Dialog.
"""
from __future__ import annotations

import glob
from typing import Any, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
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
            ("layer_voice_interaction", "Layer 5: Trung Tâm Nhiệm Vụ Backend & Âm Thanh", "mission_controller, config_manager, audio_player"),
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
            btn_start.clicked.connect(lambda _, k=key: self._on_start_layer_clicked(k))
            rl.addWidget(btn_start)

            btn_stop = QPushButton("TẮT")
            btn_stop.setFixedSize(54, KioskTheme.TOUCH_TARGET_MIN)
            btn_stop.setCursor(Qt.PointingHandCursor)
            btn_stop.setEnabled(False)
            btn_stop.setStyleSheet(f"""
                QPushButton {{
                    background-color: {KioskTheme.COLOR_SURFACE};
                    color: {KioskTheme.COLOR_TEXT_MUTED};
                    border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                    border-radius: 6px;
                    font-weight: 700;
                    font-size: 12px;
                }}
            """)
            btn_stop.clicked.connect(lambda _, k=key: self._on_stop_layer_clicked(k))
            rl.addWidget(btn_stop)

            l_layout.addWidget(row_frame)
            self.layer_rows[key] = {
                "status_lbl": lbl_st,
                "start_btn": btn_start,
                "stop_btn": btn_stop,
            }

        c_layout.addWidget(card_layers)

        # B. THIẾT BỊ NGOẠI VI & CỔNG SERIAL
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

    def _normalize_key(self, k: str) -> str:
        mapping = {
            "layer1": "layer_hardware",
            "layer2": "layer_navigation",
            "layer3": "layer_slam",
            "layer4": "layer_perception",
            "layer5": "layer_voice_interaction",
            "layer_backend": "layer_voice_interaction",
        }
        return mapping.get(k, k)

    def _on_start_layer_clicked(self, key: str) -> None:
        target = self._normalize_key(key)
        if target in self.layer_rows:
            self.layer_rows[target]["status_lbl"].set_status("STARTING...", HealthStatus.WARNING)
            self.layer_rows[target]["start_btn"].setEnabled(False)
        self.start_layer_requested.emit(key)

    def _on_stop_layer_clicked(self, key: str) -> None:
        target = self._normalize_key(key)
        if target in self.layer_rows:
            self.layer_rows[target]["status_lbl"].set_status("STOPPING...", HealthStatus.WARNING)
            self.layer_rows[target]["stop_btn"].setEnabled(False)
        self.stop_layer_requested.emit(key)

    def update_layer_status(self, layers_state: dict[str, bool]) -> None:
        """Cập nhật trạng thái hiển thị và nút bấm tương ứng cho các tầng ROS 2."""
        active_count = 0
        total_layers = len(self.layer_rows)
        for raw_key, is_on in layers_state.items():
            key = self._normalize_key(raw_key)
            if key in self.layer_rows:
                row = self.layer_rows[key]
                lbl = row["status_lbl"]
                btn_start = row["start_btn"]
                btn_stop = row["stop_btn"]
                if is_on:
                    active_count += 1
                    lbl.set_status("RUNNING", HealthStatus.RUNNING)
                    btn_start.setEnabled(False)
                    btn_start.setStyleSheet(f"""
                        QPushButton {{
                            background-color: {KioskTheme.COLOR_SURFACE};
                            color: {KioskTheme.COLOR_TEXT_MUTED};
                            border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                            border-radius: 6px;
                            font-weight: 700;
                            font-size: 12px;
                        }}
                    """)
                    btn_stop.setEnabled(True)
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
                else:
                    lbl.set_status("STOPPED", HealthStatus.STOPPED)
                    btn_start.setEnabled(True)
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
                    btn_stop.setEnabled(False)
                    btn_stop.setStyleSheet(f"""
                        QPushButton {{
                            background-color: {KioskTheme.COLOR_SURFACE};
                            color: {KioskTheme.COLOR_TEXT_MUTED};
                            border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                            border-radius: 6px;
                            font-weight: 700;
                            font-size: 12px;
                        }}
                    """)

        header_status = HealthStatus.RUNNING if active_count > 0 else HealthStatus.STOPPED
        self.header.set_status(f"{active_count}/{total_layers} ACTIVE", header_status)

    def _on_rescan_ports(self) -> None:
        ports = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        unique_ports = list(dict.fromkeys(ports))
        ports_str = ", ".join(unique_ports) if unique_ports else "Không phát hiện cổng serial khả dụng"
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
