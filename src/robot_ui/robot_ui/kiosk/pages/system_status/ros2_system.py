"""ros2_system.py - Screen 2: Module C (ROS 2 System Detail).

Organizes:
1. Master Controls (Workflow gui_v3): Start All / Stop All with Mode & Map selection.
2. 5-Layer Process Control: Hardware, Navigation, SLAM, Perception, Voice Interaction.
3. Peripheral Hardware Ports: USB/Serial bus scanner & swap interface (Hoverboard ⇄ LiDAR).
4. Real-time Console Log with 60ms batching anti-lag.
5. Advanced Controls / Dangerous Actions: Emergency ROS 2 Process Cleanup with Confirmation Dialog.
"""
from __future__ import annotations

from pathlib import Path
import re
import time
from typing import Any, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
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
    """Screen 2: ROS 2 System & Layer Management Screen (gui_v3 integration)."""

    back_requested = pyqtSignal()
    start_layer_requested = pyqtSignal(str)
    stop_layer_requested = pyqtSignal(str)
    emergency_cleanup_requested = pyqtSignal()
    start_all_requested = pyqtSignal(str, str, str)  # (mode, map_path, backend)
    stop_all_requested = pyqtSignal()
    swap_ports_requested = pyqtSignal()
    rescan_ports_requested = pyqtSignal()

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
            subtitle="Tích hợp Vận hành Chuẩn gui_v3, Quản lý Layer & Cổng Serial",
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

        # A. CỤM ĐIỀU KHIỂN HỆ THỐNG TỔNG THỂ (WORKFLOW GUI V3 - START ALL / STOP ALL)
        card_master, m_layout = create_section_card(
            "CỤM ĐIỀU KHIỂN VẬN HÀNH TỔNG THỂ (RUN ALL - WORKFLOW GUI V3)",
            "#10b981",
        )

        # 1. Hàng chọn Chế độ vận hành Layer 2 & Backend định vị
        mode_row = QHBoxLayout()
        mode_row.setSpacing(10)

        lbl_mode = QLabel("Chế độ Layer 2:")
        lbl_mode.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-weight: 700; font-size: 12px;")
        mode_row.addWidget(lbl_mode)

        self.cmb_operation_mode = QComboBox()
        self.cmb_operation_mode.addItems([
            "Lập Bản Đồ Mới (SLAM Mapping)",
            "Định Vị & Điều Hướng (Localization Nav2)",
        ])
        self.cmb_operation_mode.setFixedHeight(36)
        self.cmb_operation_mode.setStyleSheet(f"""
            QComboBox {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 4px 10px;
                font-weight: 700;
                font-size: 12px;
            }}
            QComboBox::drop-down {{ border: none; }}
            QComboBox QAbstractItemView {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                selection-background-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)
        self.cmb_operation_mode.currentIndexChanged.connect(self._on_mode_changed)
        mode_row.addWidget(self.cmb_operation_mode, stretch=1)

        lbl_backend = QLabel("Backend định vị:")
        lbl_backend.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-weight: 700; font-size: 12px;")
        mode_row.addWidget(lbl_backend)

        self.cmb_backend = QComboBox()
        self.cmb_backend.addItems(["AMCL", "SLAM Toolbox"])
        self.cmb_backend.setFixedHeight(36)
        self.cmb_backend.setStyleSheet(self.cmb_operation_mode.styleSheet())
        self.cmb_backend.setEnabled(False)  # Mặc định tắt nếu đang là SLAM Mapping
        mode_row.addWidget(self.cmb_backend)

        m_layout.addLayout(mode_row)

        # 2. Hàng chọn Bản đồ (Map selector) khi ở chế độ Localization
        self.map_row_widget = QWidget()
        map_row = QHBoxLayout(self.map_row_widget)
        map_row.setContentsMargins(0, 0, 0, 0)
        map_row.setSpacing(10)

        lbl_map = QLabel("Bản đồ mục tiêu:")
        lbl_map.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-weight: 700; font-size: 12px;")
        map_row.addWidget(lbl_map)

        self.cmb_maps = QComboBox()
        self.cmb_maps.setFixedHeight(36)
        self.cmb_maps.setStyleSheet(self.cmb_operation_mode.styleSheet())
        map_row.addWidget(self.cmb_maps, stretch=1)

        self.btn_refresh_maps = QPushButton("Làm mới Map")
        self.btn_refresh_maps.setFixedHeight(36)
        self.btn_refresh_maps.setCursor(Qt.PointingHandCursor)
        self.btn_refresh_maps.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                color: {KioskTheme.COLOR_TEXT_MUTED};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 0 12px;
                font-weight: 700;
                font-size: 11px;
            }}
            QPushButton:hover {{ color: {KioskTheme.COLOR_TEXT_PRIMARY}; border-color: {KioskTheme.COLOR_ACCENT}; }}
        """)
        self.btn_refresh_maps.clicked.connect(self.refresh_map_list)
        map_row.addWidget(self.btn_refresh_maps)

        self.map_row_widget.setVisible(False)  # Mặc định ẩn nếu đang là SLAM Mapping
        m_layout.addWidget(self.map_row_widget)

        # 3. Hàng 2 Nút Tổng Quan: START ALL & STOP ALL
        actions_row = QHBoxLayout()
        actions_row.setSpacing(12)

        self.btn_start_all = QPushButton("▶ CHẠY TOÀN BỘ (START ALL)")
        self.btn_start_all.setFixedHeight(44)
        self.btn_start_all.setCursor(Qt.PointingHandCursor)
        self.btn_start_all.setToolTip(
            "Khởi động Layer 1 (Hardware/Odom), sau 1.2s tự động khởi động tiếp Layer 2 (theo Mode đã chọn) giống gui_v3"
        )
        self.btn_start_all.setStyleSheet("""
            QPushButton {
                background-color: #10b981;
                color: #ffffff;
                border: none;
                border-radius: 6px;
                font-size: 13px;
                font-weight: 900;
                letter-spacing: 0.5px;
            }
            QPushButton:hover {
                background-color: #059669;
            }
            QPushButton:pressed {
                background-color: #047857;
            }
        """)
        self.btn_start_all.clicked.connect(self._on_start_all_clicked)
        actions_row.addWidget(self.btn_start_all, stretch=1)

        self.btn_stop_all = QPushButton("⏹ DỪNG TOÀN BỘ (STOP ALL)")
        self.btn_stop_all.setFixedHeight(44)
        self.btn_stop_all.setCursor(Qt.PointingHandCursor)
        self.btn_stop_all.setToolTip("Dừng tất cả các tiến trình Layer 1..5 và SLAM/Nav2 đang chạy")
        self.btn_stop_all.setStyleSheet("""
            QPushButton {
                background-color: #ef4444;
                color: #ffffff;
                border: none;
                border-radius: 6px;
                font-size: 13px;
                font-weight: 900;
                letter-spacing: 0.5px;
            }
            QPushButton:hover {
                background-color: #dc2626;
            }
            QPushButton:pressed {
                background-color: #b91c1c;
            }
        """)
        self.btn_stop_all.clicked.connect(self._on_stop_all_clicked)
        actions_row.addWidget(self.btn_stop_all, stretch=1)

        m_layout.addLayout(actions_row)
        c_layout.addWidget(card_master)

        # B. QUẢN LÝ TIẾN TRÌNH 5 LAYER ROS 2 (ĐIỀU KHIỂN CHI TIẾT TỪNG TẦNG)
        card_layers, l_layout = create_section_card(
            "QUẢN LÝ 5 LAYER KIẾN TRÚC PHẦN MỀM ROBOT (BẬT / TẮT TỪNG PHẦN)",
            KioskTheme.COLOR_ACCENT,
        )

        layers_info = [
            ("layer_hardware", "Layer 1: Giao Tiếp Phần Cứng & Động Cơ", "hoverboard_driver, sllidar_node, real_odom"),
            ("layer_navigation", "Layer 2: Điều Hướng Tự Hành Nav2", "nav2_bringup, amcl / slam_toolbox localization"),
            ("layer_slam", "Layer 3: Lập Bản Đồ SLAM Toolbox", "slam_toolbox (real_slam.launch.py)"),
            ("layer_perception", "Layer 4: Thị Giác Máy Tính AI Astra", "astra_camera, yolo_detector, hand_wave_pose"),
            ("layer_voice_interaction", "Layer 5: Giọng Nói & Tương Tác", "wake_word_kws, llm_worker, backend.launch.py"),
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

        # C. NHẬT KÝ TIẾN TRÌNH ROS 2 HỆ THỐNG (STDOUT / STDERR CONSOLE)
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

        # D. THIẾT BỊ NGOẠI VI & CỔNG SERIAL (ESP32 & LIDAR)
        card_ports, port_layout = create_section_card(
            "THIẾT BỊ NGOẠI VI & ĐƯỜNG TRUYỀN SERIAL (ESP32 & LIDAR)",
            "#38BDF8",
        )

        ports_desc_frame = QFrame()
        ports_desc_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                padding: 6px 10px;
            }}
        """)
        pdf_layout = QVBoxLayout(ports_desc_frame)
        pdf_layout.setContentsMargins(6, 6, 6, 6)
        pdf_layout.setSpacing(4)

        self.lbl_esp_port = QLabel("ESP32 / Hoverboard: Đang quét...")
        self.lbl_esp_port.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 12px; font-weight: bold;")
        self.lbl_lidar_port = QLabel("LiDAR SLAMTEC: Đang quét...")
        self.lbl_lidar_port.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 12px; font-weight: bold;")
        self.lbl_ports_info = QLabel("Cơ chế nhận diện: Chờ khởi tạo...")
        self.lbl_ports_info.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")

        pdf_layout.addWidget(self.lbl_esp_port)
        pdf_layout.addWidget(self.lbl_lidar_port)
        pdf_layout.addWidget(self.lbl_ports_info)
        port_layout.addWidget(ports_desc_frame)

        ports_btn_row = QHBoxLayout()
        ports_btn_row.setSpacing(10)

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
        ports_btn_row.addWidget(self.btn_rescan_ports)

        self.btn_swap_ports = QPushButton("Hoán Đổi Cổng (Hoverboard ⇄ LiDAR)")
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
        ports_btn_row.addWidget(self.btn_swap_ports)

        port_layout.addLayout(ports_btn_row)
        c_layout.addWidget(card_ports)

        # E. ADVANCED CONTROLS / RISKY ACTIONS (SEPARATE SECTION)
        card_danger, danger_layout = create_section_card(
            "THAO TÁC NÂNG CAO & KHÔI PHỤC TIẾN TRÌNH",
            KioskTheme.COLOR_CRITICAL,
        )

        lbl_danger_desc = QLabel(
            "CẢNH BÁO: Thao tác bên dưới sẽ can thiệp trực tiếp vào toàn bộ tiến trình ROS 2 đang chạy. "
            "Chỉ sử dụng khi hệ thống gặp sự cố treo cứng hoặc xung đột node."
        )
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

        # Khởi tạo nạp danh sách bản đồ
        self.refresh_map_list()

    def _on_mode_changed(self, index: int) -> None:
        """Bật/tắt các widget phụ thuộc theo chế độ SLAM hoặc Localization."""
        is_loc = index == 1
        self.cmb_backend.setEnabled(is_loc)
        self.map_row_widget.setVisible(is_loc)

    def refresh_map_list(self) -> None:
        """Quét và làm mới danh sách các file bản đồ YAML có sẵn."""
        self.cmb_maps.blockSignals(True)
        self.cmb_maps.clear()
        found_maps: list[str] = []
        search_dirs = [
            Path.home() / "ros2_ws" / "my_map",
            Path.home() / "ros2_ws" / "src" / "mo_hinh" / "maps",
            Path.home() / "ros2_ws" / "install" / "mo_hinh" / "share" / "mo_hinh" / "maps",
        ]
        for sdir in search_dirs:
            if sdir.is_dir():
                for p in sorted(sdir.glob("*.yaml")):
                    if p.stem in ("cam_truoc", "cam_sau", "nav2_params", "mapper_params_online_async"):
                        continue
                    sp = str(p.resolve())
                    if sp not in found_maps:
                        found_maps.append(sp)
                        self.cmb_maps.addItem(f"{p.name} ({p.parent.name})", sp)

        self.cmb_maps.blockSignals(False)

    def set_detected_ports(self, esp_port: str, lidar_port: str, detect_src: str) -> None:
        """Cập nhật thông tin cổng Serial hiển thị lên giao diện."""
        self.lbl_esp_port.setText(f"ESP32 / Hoverboard: {esp_port}")
        self.lbl_lidar_port.setText(f"LiDAR SLAMTEC: {lidar_port}")
        self.lbl_ports_info.setText(f"Cơ chế nhận diện: {detect_src}")

    def update_layer_status(self, layers_state: dict[str, bool]) -> None:
        """Cập nhật badge trạng thái các tầng tiến trình."""
        for key, is_on in layers_state.items():
            if key in self.layer_rows:
                lbl = self.layer_rows[key]["status_lbl"]
                if is_on:
                    lbl.set_status("RUNNING", HealthStatus.RUNNING)
                else:
                    lbl.set_status("STOPPED", HealthStatus.STOPPED)

    def _on_start_all_clicked(self) -> None:
        """Bắt đầu chạy toàn bộ hệ thống theo chuẩn gui_v3."""
        mode_idx = self.cmb_operation_mode.currentIndex()
        mode = "slam_mapping" if mode_idx == 0 else "localization"
        backend = self.cmb_backend.currentText().strip().lower()
        if "slam" in backend:
            backend = "slam_toolbox"
        else:
            backend = "amcl"
        selected_map = self.cmb_maps.currentData() or ""
        self.append_process_log(
            "SYSTEM",
            f"Bắt đầu quy trình CHẠY TOÀN BỘ (Start All)... Mode: {self.cmb_operation_mode.currentText()}",
            flush_now=True,
        )
        self.start_all_requested.emit(mode, selected_map, backend)

    def _on_stop_all_clicked(self) -> None:
        """Dừng toàn bộ hệ thống."""
        self.append_process_log("SYSTEM", "Gửi lệnh DỪNG TOÀN BỘ (Stop All)...", flush_now=True)
        self.stop_all_requested.emit()

    def _on_swap_ports(self) -> None:
        """Gửi yêu cầu hoán đổi cổng Hoverboard và LiDAR."""
        self.append_process_log("SERIAL", "Yêu cầu hoán đổi cổng Hoverboard và LiDAR...", flush_now=True)
        self.swap_ports_requested.emit()

    def _on_rescan_ports(self) -> None:
        """Quét lại danh sách cổng kết nối."""
        self.append_process_log("SERIAL", "Đang quét lại các cổng kết nối Serial...", flush_now=True)
        self.rescan_ports_requested.emit()

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
            message=(
                "Hành động này sẽ gửi tín hiệu SIGTERM/SIGKILL đến toàn bộ các node ROS 2, "
                "Nav2, SLAM và driver phần cứng. Hệ thống sẽ tạm dừng hoạt động và cần khởi động lại các layer."
            ),
            action_button_text="TÔI XÁC NHẬN DỌN DẸP",
            parent=self,
        )
        if dialog.exec_():
            self.append_process_log("EMERGENCY", "Yêu cầu dọn dẹp khẩn cấp toàn bộ tiến trình ROS 2...")
            self.emergency_cleanup_requested.emit()
