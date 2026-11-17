"""detection_debug_page.py - Real-time AI Vision & Pose Debugging Page for Robot UI Kiosk.

Provides 2 dedicated modular panels:
1. Pose Debug:
   - Button to launch rqt_image_view for live pose/YOLO skeleton visualization.
   - Live telemetry panel: Number of people detected, hand-waving gesture state,
     closest person distance, and individual distance details.
2. Face Recognition (Placeholder / Coming Soon):
   - Face registration and recognition debug slot for future model integration.
"""
from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any, Optional

from PyQt5.QtCore import QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QIcon, QPixmap
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.theme import KioskTheme


def _load_pixmap(icon_path: Path | str | None, size: int = 24) -> Optional[QPixmap]:
    if not icon_path:
        return None
    p = Path(icon_path)
    if not p.is_file():
        return None
    pm = QPixmap(str(p))
    if pm.isNull():
        return None
    return pm.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


class DetectionDebugPage(QWidget):
    """Touch-friendly AI Detection, Pose, and Vision debugging page."""

    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    fullscreen_toggle_requested = pyqtSignal()

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("detectionDebugPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.icons = icons or {}
        self._is_active = True
        self._cached_telemetry: Optional[dict[str, Any]] = None

        self._init_ui()

    def _init_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(10)

        # 1. Header Bar matching navigation_page & system_config_page
        header_bar = QFrame()
        header_bar.setObjectName("navControlBar")
        bar_layout = QHBoxLayout(header_bar)
        bar_layout.setContentsMargins(10, 6, 12, 6)
        bar_layout.setSpacing(10)

        # Home Button
        self.btn_home = QPushButton()
        self.btn_home.setObjectName("navHomeButton")
        self.btn_home.setFixedSize(44, 44)
        self.btn_home.setCursor(Qt.PointingHandCursor)
        self.btn_home.setToolTip("Về màn hình chính")
        h_pm = _load_pixmap(self.icons.get("home"), size=26)
        if h_pm:
            self.btn_home.setIcon(QIcon(h_pm))
            self.btn_home.setIconSize(QSize(26, 26))
        else:
            self.btn_home.setText("Home")
            self.btn_home.setStyleSheet("color: #5c6e80; font-size: 11px; font-weight: 700;")
        self.btn_home.clicked.connect(self.home_requested.emit)
        bar_layout.addWidget(self.btn_home)

        # Emergency Button (navEmergencyButton)
        self.btn_emergency = QPushButton()
        self.btn_emergency.setObjectName("navEmergencyButton")
        self.btn_emergency.setFixedSize(44, 44)
        self.btn_emergency.setCursor(Qt.PointingHandCursor)
        self.btn_emergency.setToolTip("DỪNG KHẨN CẤP")
        em_pm = _load_pixmap(self.icons.get("emergency"), size=34)
        if em_pm:
            self.btn_emergency.setIcon(QIcon(em_pm))
            self.btn_emergency.setIconSize(QSize(34, 34))
        else:
            self.btn_emergency.setText("DỪNG")
            self.btn_emergency.setStyleSheet("color: #ef4444; font-size: 11px; font-weight: 700;")
        self.btn_emergency.clicked.connect(self.emergency_stop_requested.emit)
        bar_layout.addWidget(self.btn_emergency)

        # Back Button (navBackButton)
        self.btn_back = QPushButton()
        self.btn_back.setObjectName("navBackButton")
        self.btn_back.setFixedSize(44, 44)
        self.btn_back.setCursor(Qt.PointingHandCursor)
        self.btn_back.setToolTip("Quay lại trang Quản trị")
        b_pm = _load_pixmap(self.icons.get("back"), size=28)
        if b_pm:
            self.btn_back.setIcon(QIcon(b_pm))
            self.btn_back.setIconSize(QSize(28, 28))
        else:
            self.btn_back.setText("<")
            self.btn_back.setStyleSheet("color: #b32632; font-size: 20px; font-weight: 700;")
        self.btn_back.clicked.connect(self.back_requested.emit)
        bar_layout.addWidget(self.btn_back)

        # Title Block
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        lbl_title = QLabel("DETECTION & AI VISION DEBUG")
        lbl_title.setStyleSheet("color: #b32632; font-size: 16px; font-weight: 800; letter-spacing: 0.5px;")
        lbl_sub = QLabel("Chẩn đoán Nhận diện Tư thế Pose, Khung xương & Khuôn mặt thời gian thực")
        lbl_sub.setStyleSheet("color: #5c6e80; font-size: 11px;")
        title_box.addWidget(lbl_title)
        title_box.addWidget(lbl_sub)
        bar_layout.addLayout(title_box)

        bar_layout.addStretch(1)

        # Badges (Wifi & Battery)
        self.wifi_badge = QWidget()
        self.wifi_badge.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 8px;")
        wb_l = QHBoxLayout(self.wifi_badge)
        wb_l.setContentsMargins(8, 4, 10, 4)
        wb_l.setSpacing(6)
        self.lbl_wifi_icon = QLabel()
        self.lbl_wifi_icon.setFixedSize(20, 20)
        w_icon = self.icons.get("wifi_connected")
        if w_icon and Path(w_icon).is_file():
            self.lbl_wifi_icon.setPixmap(QPixmap(str(w_icon)).scaled(18, 18, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.lbl_wifi_top = QLabel("Online")
        self.lbl_wifi_top.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 11px; font-weight: bold;")
        wb_l.addWidget(self.lbl_wifi_icon)
        wb_l.addWidget(self.lbl_wifi_top)
        bar_layout.addWidget(self.wifi_badge)

        self.bat_badge = QWidget()
        self.bat_badge.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 8px;")
        bb_l = QHBoxLayout(self.bat_badge)
        bb_l.setContentsMargins(8, 4, 10, 4)
        bb_l.setSpacing(6)
        self.lbl_bat_icon = QLabel()
        self.lbl_bat_icon.setFixedSize(20, 20)
        b_icon = (self.icons.get("battery_icons") or {}).get("100")
        if b_icon and Path(b_icon).is_file():
            self.lbl_bat_icon.setPixmap(QPixmap(str(b_icon)).scaled(18, 18, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.lbl_bat_top = QLabel("--%")
        self.lbl_bat_top.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 11px; font-weight: bold;")
        bb_l.addWidget(self.lbl_bat_icon)
        bb_l.addWidget(self.lbl_bat_top)
        bar_layout.addWidget(self.bat_badge)

        # Fullscreen Button (navFullscreenButton)
        self.btn_fullscreen = QPushButton()
        self.btn_fullscreen.setObjectName("navFullscreenButton")
        self.btn_fullscreen.setFixedSize(44, 44)
        self.btn_fullscreen.setCursor(Qt.PointingHandCursor)
        self.btn_fullscreen.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 8px;
                color: #263747;
                font-size: 18px;
                font-weight: 700;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ALT}; color: #b32632; }}
        """)
        self.btn_fullscreen.clicked.connect(self.fullscreen_toggle_requested.emit)
        self.update_fullscreen_button(is_fullscreen=True)
        bar_layout.addWidget(self.btn_fullscreen)

        root.addWidget(header_bar)

        # 2. Main 2-Column Split
        split_row = QHBoxLayout()
        split_row.setSpacing(12)

        # -------------------------------------------------------------
        # BOX 1: POSE DEBUG
        # -------------------------------------------------------------
        card_pose = QFrame()
        card_pose.setStyleSheet(f"""
            QFrame {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-top: 3px solid #a855f7;
                border-radius: {KioskTheme.CARD_RADIUS};
            }}
        """)
        pose_layout = QVBoxLayout(card_pose)
        pose_layout.setContentsMargins(14, 12, 14, 12)
        pose_layout.setSpacing(10)

        lbl_pose_head = QLabel("1. KHỐI POSE & HUMAN TRACKING (TƯ THẾ & NGƯỜI)")
        lbl_pose_head.setStyleSheet("color: #7f39b2; font-size: 13px; font-weight: 800; letter-spacing: 0.5px;")
        pose_layout.addWidget(lbl_pose_head)

        # Button to launch RQT image view
        self.btn_launch_rqt_pose = QPushButton("MỞ CỬA SỔ RQT POSE DEBUG")
        self.btn_launch_rqt_pose.setFixedHeight(44)
        self.btn_launch_rqt_pose.setCursor(Qt.PointingHandCursor)
        self.btn_launch_rqt_pose.setStyleSheet("""
            QPushButton {{
                background-color: #6b21a8;
                color: #ffffff;
                font-weight: 800;
                font-size: 13px;
                border: none;
                border-radius: 8px;
            }}
            QPushButton:hover {{ background-color: #7e22ce; }}
            QPushButton:pressed {{ background-color: #581c87; }}
        """)
        self.btn_launch_rqt_pose.clicked.connect(self._launch_rqt_pose)
        pose_layout.addWidget(self.btn_launch_rqt_pose)

        # Telemetry metrics grid
        t_grid = QGridLayout()
        t_grid.setSpacing(8)

        # People count
        box_cnt = QFrame()
        box_cnt.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border-radius: 6px; padding: 8px;")
        bc_l = QVBoxLayout(box_cnt)
        bc_l.setContentsMargins(4, 4, 4, 4)
        bc_l.addWidget(QLabel("Số Người Nhận Diện", styleSheet="color: #5c6e80; font-size: 10px; font-weight: bold;"))
        self.lbl_people_count = QLabel("0 người")
        self.lbl_people_count.setStyleSheet("color: #126da8; font-size: 18px; font-weight: 800;")
        bc_l.addWidget(self.lbl_people_count)
        t_grid.addWidget(box_cnt, 0, 0)

        # Wave state
        box_wave = QFrame()
        box_wave.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border-radius: 6px; padding: 8px;")
        bw_l = QVBoxLayout(box_wave)
        bw_l.setContentsMargins(4, 4, 4, 4)
        bw_l.addWidget(QLabel("Cử Chỉ Vẫy Tay", styleSheet="color: #5c6e80; font-size: 10px; font-weight: bold;"))
        self.lbl_wave_state = QLabel("KHÔNG VẪY")
        self.lbl_wave_state.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 18px; font-weight: 800;")
        bw_l.addWidget(self.lbl_wave_state)
        t_grid.addWidget(box_wave, 0, 1)

        # Closest person distance
        box_dist = QFrame()
        box_dist.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border-radius: 6px; padding: 8px;")
        bd_l = QVBoxLayout(box_dist)
        bd_l.setContentsMargins(4, 4, 4, 4)
        bd_l.addWidget(QLabel("Cự Ly Người Gần Nhất", styleSheet="color: #5c6e80; font-size: 10px; font-weight: bold;"))
        self.lbl_closest_dist = QLabel("-- m")
        self.lbl_closest_dist.setStyleSheet(f"color: {KioskTheme.COLOR_ACCENT}; font-size: 18px; font-weight: 800;")
        bd_l.addWidget(self.lbl_closest_dist)
        t_grid.addWidget(box_dist, 1, 0, 1, 2)

        pose_layout.addLayout(t_grid)

        # Details list of detected people
        lbl_list_head = QLabel("Danh Sách Người Trong Tầm Quan Sát:")
        lbl_list_head.setStyleSheet("color: #263747; font-size: 11px; font-weight: bold;")
        pose_layout.addWidget(lbl_list_head)

        self.list_people = QListWidget()
        self.list_people.setStyleSheet(f"""
            QListWidget {{
                background-color: #ffffff;
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                font-family: monospace;
                font-size: 11px;
                padding: 4px;
            }}
            QListWidget::item {{
                padding: 6px 8px;
                border-bottom: 1px solid #1e2633;
            }}
        """)
        self.list_people.addItem("Chưa phát hiện mục tiêu người trong khung hình...")
        pose_layout.addWidget(self.list_people, stretch=1)

        split_row.addWidget(card_pose, stretch=1)

        # -------------------------------------------------------------
        # BOX 2: FACE RECOGNITION (COMING SOON)
        # -------------------------------------------------------------
        card_face = QFrame()
        card_face.setStyleSheet(f"""
            QFrame {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-top: 3px solid #38bdf8;
                border-radius: {KioskTheme.CARD_RADIUS};
            }}
        """)
        face_layout = QVBoxLayout(card_face)
        face_layout.setContentsMargins(14, 12, 14, 12)
        face_layout.setSpacing(12)

        lbl_face_head = QLabel("2. KHỐI FACE RECOGNITION (NHẬN DIỆN KHUÔN MẶT)")
        lbl_face_head.setStyleSheet("color: #126da8; font-size: 13px; font-weight: 800; letter-spacing: 0.5px;")
        face_layout.addWidget(lbl_face_head)

        status_box = QFrame()
        status_box.setStyleSheet(f"background-color: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid #23354d; border-radius: 8px; padding: 14px;")
        sb_l = QVBoxLayout(status_box)
        sb_l.setSpacing(8)

        lbl_badge = QLabel("[ TÍNH NĂNG ĐANG PHÁT TRIỂN / COMING SOON ]")
        lbl_badge.setStyleSheet("color: #126da8; font-size: 12px; font-weight: 800;")
        sb_l.addWidget(lbl_badge)

        lbl_desc = QLabel(
            "Mô-đun nhận diện khuôn mặt cho phép đăng ký khuôn mặt khách hàng VIP, "
            "nhân viên vận hành, nhận diện danh tính và tự động chào hỏi thông minh qua hệ thống LLM Souta."
        )
        lbl_desc.setWordWrap(True)
        lbl_desc.setStyleSheet("color: #5c6e80; font-size: 12px; line-height: 1.4;")
        sb_l.addWidget(lbl_desc)
        face_layout.addWidget(status_box)

        # Action Buttons (Disabled placeholders)
        self.btn_register_face = QPushButton("Đăng Ký Khuôn Mặt Mới (Face Enrollment)")
        self.btn_register_face.setFixedHeight(42)
        self.btn_register_face.setEnabled(False)
        self.btn_register_face.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px dashed {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: #64748b;
                font-weight: bold;
                font-size: 12px;
            }}
        """)
        face_layout.addWidget(self.btn_register_face)

        self.btn_test_db = QPushButton("Kiểm Tra Cơ Sở Dữ Liệu Khuôn Mặt")
        self.btn_test_db.setFixedHeight(42)
        self.btn_test_db.setEnabled(False)
        self.btn_test_db.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px dashed {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: #64748b;
                font-weight: bold;
                font-size: 12px;
            }}
        """)
        face_layout.addWidget(self.btn_test_db)

        face_layout.addStretch(1)

        # Notice label
        lbl_hint = QLabel("Kỹ thuật viên có thể bổ sung API DeepFace / InsightFace vào đây.")
        lbl_hint.setStyleSheet("color: #5a6e85; font-size: 11px; font-style: italic;")
        face_layout.addWidget(lbl_hint)

        split_row.addWidget(card_face, stretch=1)
        root.addLayout(split_row, stretch=1)

    def _launch_rqt_pose(self) -> None:
        """Launch rqt_image_view for pose debug stream detached."""
        cmd = "source /opt/ros/humble/setup.bash && rqt_image_view /ai/pose_image"
        try:
            subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def set_active(self, active: bool) -> None:
        """Kích hoạt hoặc tắt cập nhật UI khi trang hiển thị."""
        self._is_active = active
        if active and self._cached_telemetry is not None:
            self._render_ai_telemetry(self._cached_telemetry)

    def update_ai_telemetry(self, stats: dict[str, Any]) -> None:
        """Update live telemetry from AI pipeline (cache when hidden)."""
        self._cached_telemetry = stats
        if self._is_active:
            self._render_ai_telemetry(stats)

    def _render_ai_telemetry(self, stats: dict[str, Any]) -> None:
        """Thực tế render telemetry lên các widget của Detection Debug."""
        cnt = int(stats.get("people_count", stats.get("yolo_people_count", 0)))
        self.lbl_people_count.setText(f"{cnt} người")

        is_w = bool(stats.get("is_waving", stats.get("hand_wave_detected", False)))
        if is_w:
            self.lbl_wave_state.setText("CÓ VẪY TAY")
            self.lbl_wave_state.setStyleSheet(f"color: {KioskTheme.COLOR_CRITICAL}; font-size: 18px; font-weight: 800;")
        else:
            self.lbl_wave_state.setText("KHÔNG VẪY")
            self.lbl_wave_state.setStyleSheet(f"color: {KioskTheme.COLOR_HEALTHY}; font-size: 18px; font-weight: 800;")

        dist = stats.get("closest_person_dist")
        if dist is not None:
            self.lbl_closest_dist.setText(f"{float(dist):.2f} m")
            color = KioskTheme.COLOR_CRITICAL if float(dist) < 1.8 else KioskTheme.COLOR_HEALTHY
            self.lbl_closest_dist.setStyleSheet(f"color: {color}; font-size: 18px; font-weight: 800;")
        else:
            self.lbl_closest_dist.setText("-- m")
            self.lbl_closest_dist.setStyleSheet(f"color: {KioskTheme.COLOR_ACCENT}; font-size: 18px; font-weight: 800;")

        # Update people list
        self.list_people.clear()
        if cnt == 0:
            self.list_people.addItem("Không có người trong phạm vi phát hiện.")
        else:
            for idx in range(1, cnt + 1):
                if idx == 1 and dist is not None:
                    flag = " [CẢN ĐƯỜNG]" if dist < 1.8 else " [AN TOÀN]"
                    item_text = f"Mục tiêu #{idx}: Cự ly ~{dist:.2f} m{flag}"
                    item = QListWidgetItem(item_text)
                    item.setForeground(QColor("#ef4444" if dist < 1.8 else "#22c55e"))
                else:
                    item_text = f"Mục tiêu #{idx}: Cự ly -- m (Không có cự ly 3D)"
                    item = QListWidgetItem(item_text)
                    item.setForeground(QColor("#94a3b8"))
                self.list_people.addItem(item)

    def update_fullscreen_button(self, is_fullscreen: bool) -> None:
        if is_fullscreen:
            self.btn_fullscreen.setText("🗗")
            self.btn_fullscreen.setToolTip("Thoát toàn màn hình (Esc hoặc F11)")
        else:
            self.btn_fullscreen.setText("🗖")
            self.btn_fullscreen.setToolTip("Toàn màn hình (F11)")

    def update_wifi(self, connected: bool) -> None:
        self.lbl_wifi_top.setText("Online" if connected else "Offline")
        self.lbl_wifi_top.setStyleSheet(
            f"color: {KioskTheme.COLOR_HEALTHY if connected else KioskTheme.COLOR_CRITICAL}; font-size: 11px; font-weight: bold;"
        )
        icon_key = "wifi_connected" if connected else "wifi_disconnected"
        icon_path = self.icons.get(icon_key)
        if icon_path and Path(icon_path).is_file():
            self.lbl_wifi_icon.setPixmap(
                QPixmap(str(icon_path)).scaled(18, 18, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )

    def update_battery(self, percent: float | None) -> None:
        if percent is None:
            self.lbl_bat_top.setText("--%")
            return
        p = max(0.0, min(100.0, float(percent)))
        self.lbl_bat_top.setText(f"{round(p)}%")
        bucket = "100" if p >= 85 else ("75" if p >= 60 else ("50" if p >= 35 else ("25" if p >= 15 else "0")))
        bat_icon = (self.icons.get("battery_icons") or {}).get(bucket)
        if bat_icon and Path(bat_icon).is_file():
            self.lbl_bat_icon.setPixmap(
                QPixmap(str(bat_icon)).scaled(18, 18, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
