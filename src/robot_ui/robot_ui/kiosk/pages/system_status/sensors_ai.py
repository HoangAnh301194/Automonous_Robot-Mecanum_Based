"""sensors_ai.py - Screen 2: Module A (Sensors & AI Perception Detail).

Organizes:
1. Summary: Available sensors & pipeline health.
2. Camera Section: Front, Rear, and AI Astra cameras (Hz & closest obstacle).
3. Core Sensors: LiDAR 360°, Wheel Odometry, and TF Tree (map -> odom -> base_footprint).
4. AI Perception: YOLO Person Tracking, Hand-Wave Detection, and Person Distance.
"""
from __future__ import annotations

import os
import shlex
import subprocess
from typing import Any, Optional

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QScroller,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.components import (
    MetricCard,
    PageHeader,
    create_section_card,
)
from robot_ui.kiosk.pages.system_status.contracts import (
    HealthStatus,
    SystemTelemetrySnapshot,
)
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


class SensorsAIDetailScreen(QWidget):
    """Screen 2: Sensors & AI Detail Screen."""

    back_requested = pyqtSignal()

    def __init__(
        self,
        icons: Optional[dict[str, Any]] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("sensorsAIDetailScreen")
        self.icons = icons or {}
        self._init_ui()

    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 12, 18, 16)
        layout.setSpacing(10)

        # 1. Header with Back button (Touch Target >= 44px)
        self.header = PageHeader(
            title="CẢM BIẾN & AI PERCEPTION",
            subtitle="Theo dõi Tần số Cảm biến, TF Tree & Khối Thị giác Máy tính",
            back_callback=self.back_requested.emit,
            back_icon_path=self.icons.get("back"),
            status_text="ĐANG KIỂM TRA...",
            status=HealthStatus.UNKNOWN,
        )
        layout.addLayout(self.header)

        # 2. Scroll Area for Detail Sections
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background: transparent; border: none;")
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        QScroller.grabGesture(scroll.viewport(), QScroller.LeftMouseButtonGesture)

        container = QWidget()
        c_layout = QVBoxLayout(container)
        c_layout.setContentsMargins(0, 0, 4, 0)
        c_layout.setSpacing(12)

        # A. CỤM 3 CAMERA ASTRA
        card_cams, cams_layout = create_section_card("CỤM 3 CAMERA ASTRA (TRƯỚC / SAU / AI)", KioskTheme.COLOR_ACCENT)
        cams_grid = QGridLayout()
        cams_grid.setSpacing(10)

        self.card_cam_f = MetricCard("Camera Astra Trước", "0.0 Hz", "Vật cản: --")
        self.card_cam_r = MetricCard("Camera Astra Sau", "0.0 Hz", "Vật cản: --")
        self.card_cam_ai = MetricCard("Camera AI Nhận Diện", "0.0 Hz", "Pipeline: YOLOv8")

        cams_grid.addWidget(self.card_cam_f, 0, 0)
        cams_grid.addWidget(self.card_cam_r, 0, 1)
        cams_grid.addWidget(self.card_cam_ai, 0, 2)
        cams_layout.addLayout(cams_grid)

        # RQT Image View Launcher buttons for each camera (Touch Target >= 44px)
        rqt_row = QHBoxLayout()
        rqt_row.setSpacing(10)

        self.btn_rqt_cam_f = QPushButton("Xem RQT Cam Trước")
        self.btn_rqt_cam_f.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_rqt_cam_f.setCursor(Qt.PointingHandCursor)
        self.btn_rqt_cam_f.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: {KioskTheme.COLOR_ACCENT};
                font-size: 11px;
                font-weight: bold;
                padding: 0 8px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: {KioskTheme.COLOR_ACCENT}; }}
        """)
        self.btn_rqt_cam_f.clicked.connect(lambda: self._launch_rqt_image_view("/front_camera/depth/image_raw"))
        rqt_row.addWidget(self.btn_rqt_cam_f)

        self.btn_rqt_cam_r = QPushButton("Xem RQT Cam Sau")
        self.btn_rqt_cam_r.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_rqt_cam_r.setCursor(Qt.PointingHandCursor)
        self.btn_rqt_cam_r.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: {KioskTheme.COLOR_ACCENT};
                font-size: 11px;
                font-weight: bold;
                padding: 0 8px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: {KioskTheme.COLOR_ACCENT}; }}
        """)
        self.btn_rqt_cam_r.clicked.connect(lambda: self._launch_rqt_image_view("/rear_camera/depth/image_raw"))
        rqt_row.addWidget(self.btn_rqt_cam_r)

        self.btn_rqt_cam_ai = QPushButton("Xem RQT Cam AI RGB")
        self.btn_rqt_cam_ai.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_rqt_cam_ai.setCursor(Qt.PointingHandCursor)
        self.btn_rqt_cam_ai.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: #a855f7;
                font-weight: bold;
                font-size: 11px;
                padding: 0 8px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: #a855f7; }}
        """)
        self.btn_rqt_cam_ai.clicked.connect(lambda: self._launch_rqt_image_view("/ai_camera/color/image_raw"))
        rqt_row.addWidget(self.btn_rqt_cam_ai)

        self.btn_rqt_pose = QPushButton("Xem RQT Pose (Skeleton)")
        self.btn_rqt_pose.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        self.btn_rqt_pose.setCursor(Qt.PointingHandCursor)
        self.btn_rqt_pose.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: 6px;
                color: #38bdf8;
                font-weight: bold;
                font-size: 11px;
                padding: 0 8px;
            }}
            QPushButton:hover {{ background-color: {KioskTheme.COLOR_SURFACE_ACTIVE}; border-color: #38bdf8; }}
        """)
        self.btn_rqt_pose.clicked.connect(lambda: self._launch_rqt_image_view("/pose/image_debug"))
        rqt_row.addWidget(self.btn_rqt_pose)

        cams_layout.addLayout(rqt_row)
        c_layout.addWidget(card_cams)

        # B. CẢM BIẾN ĐỊNH VỊ & TF TREE
        card_nav_sensors, nav_layout = create_section_card("CẢM BIẾN CỐT LÕI & CÂY TỌA ĐỘ TF TREE", KioskTheme.COLOR_HEALTHY)
        nav_grid = QGridLayout()
        nav_grid.setSpacing(10)

        self.card_lidar = MetricCard("LiDAR 360° (/scan)", "0.0 Hz", "Cự ly gần nhất: --")
        self.card_odom = MetricCard("Wheel Odometry (/odom)", "0.0 Hz", "vx=0.00 wz=0.00")
        nav_grid.addWidget(self.card_lidar, 0, 0)
        nav_grid.addWidget(self.card_odom, 0, 1)
        nav_layout.addLayout(nav_grid)

        # TF Tree Status Row
        tf_row = QHBoxLayout()
        tf_row.setSpacing(10)
        self.lbl_tf_map = QLabel("map -> odom: [ KIỂM TRA... ]")
        self.lbl_tf_map.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 6px; padding: 10px; font-weight: bold; font-size: 12px; color: {KioskTheme.COLOR_WARNING};")
        self.lbl_tf_odom = QLabel("odom -> base_footprint: [ KIỂM TRA... ]")
        self.lbl_tf_odom.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 6px; padding: 10px; font-weight: bold; font-size: 12px; color: {KioskTheme.COLOR_WARNING};")
        tf_row.addWidget(self.lbl_tf_map)
        tf_row.addWidget(self.lbl_tf_odom)
        nav_layout.addLayout(tf_row)

        self.lbl_tf_summary = QLabel("Đánh giá: Hệ thống đang chờ dữ liệu TF từ bộ định vị và odometry...")
        self.lbl_tf_summary.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px; font-style: italic;")
        nav_layout.addWidget(self.lbl_tf_summary)
        c_layout.addWidget(card_nav_sensors)

        # C. KHỐI AI PERCEPTION (YOLO & POSE)
        card_ai, ai_layout = create_section_card("KHỐI AI PERCEPTION (YOLO & POSE RECOGNITION)", "#A855F7")
        ai_grid = QGridLayout()
        ai_grid.setSpacing(10)

        self.card_yolo = MetricCard("YOLO Detection", "TẮT", "Nhận diện: 0 người")
        self.card_wave = MetricCard("Cử chỉ Vẫy tay", "KHÔNG VẪY", "Tương tác: Sẵn sàng")
        self.card_person = MetricCard("Người gần nhất", "-- m", "Cự ly an toàn")
        ai_grid.addWidget(self.card_yolo, 0, 0)
        ai_grid.addWidget(self.card_wave, 0, 1)
        ai_grid.addWidget(self.card_person, 0, 2)
        ai_layout.addLayout(ai_grid)
        c_layout.addWidget(card_ai)
        c_layout.addStretch(1)

        scroll.setWidget(container)
        layout.addWidget(scroll)

    def update_telemetry(self, snap: SystemTelemetrySnapshot) -> None:
        """Update metrics across all sensor and AI indicators."""
        # 1. Cameras
        cf_hz = snap.cam_front_hz
        cf_st = HealthStatus.HEALTHY if cf_hz > 2.0 else HealthStatus.UNKNOWN
        cf_sub = f"Vật cản: {snap.obs_front_dist:.2f}m" if snap.obs_front_dist else "Cự ly: --"
        self.card_cam_f.update_data(f"{cf_hz:.1f} Hz", cf_sub, cf_st)

        cr_hz = snap.cam_rear_hz
        cr_st = HealthStatus.HEALTHY if cr_hz > 2.0 else HealthStatus.UNKNOWN
        cr_sub = f"Vật cản: {snap.obs_rear_dist:.2f}m" if snap.obs_rear_dist else "Cự ly: --"
        self.card_cam_r.update_data(f"{cr_hz:.1f} Hz", cr_sub, cr_st)

        cai_hz = snap.cam_ai_hz
        cai_st = HealthStatus.HEALTHY if cai_hz > 2.0 else HealthStatus.UNKNOWN
        self.card_cam_ai.update_data(f"{cai_hz:.1f} Hz", "Pipeline: YOLOv8", cai_st)

        # 2. LiDAR & Odometry
        lidar_st = HealthStatus.HEALTHY if snap.lidar_hz > 5.0 else HealthStatus.CRITICAL
        l_sub = f"Cự ly gần nhất: {snap.lidar_min_dist:.2f}m" if snap.lidar_min_dist else "Cự ly: --"
        self.card_lidar.update_data(f"{snap.lidar_hz:.1f} Hz", l_sub, lidar_st)

        odom_st = HealthStatus.HEALTHY if snap.odom_hz > 5.0 else HealthStatus.CRITICAL
        self.card_odom.update_data(f"{snap.odom_hz:.1f} Hz", f"vx={snap.vx:+.2f} wz={snap.wz:+.2f}", odom_st)

        # TF Tree
        tf_m = snap.tf_map_odom_ok
        tf_o = snap.tf_odom_base_ok
        self.lbl_tf_map.setText(f"map -> odom: [{'ĐÃ ĐỊNH VỊ' if tf_m else 'MẤT DẤU'}]")
        self.lbl_tf_map.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 6px; padding: 10px; font-weight: bold; font-size: 12px; color: {'#22c55e' if tf_m else '#ef4444'};")

        self.lbl_tf_odom.setText(f"odom -> base: [{'ODOM SỐNG' if tf_o else 'MẤT ODOM'}]")
        self.lbl_tf_odom.setStyleSheet(f"background: {KioskTheme.COLOR_SURFACE_ALT}; border: 1px solid {KioskTheme.COLOR_CARD_BORDER}; border-radius: 6px; padding: 10px; font-weight: bold; font-size: 12px; color: {'#22c55e' if tf_o else '#ef4444'};")

        if tf_m and tf_o:
            self.lbl_tf_summary.setText("Đánh giá: Cây TF hoàn chỉnh, robot sẵn sàng điều hướng.")
            self.lbl_tf_summary.setStyleSheet("color: #22c55e; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_tf_summary.setText("CẢNH BÁO: Mất liên kết TF, robot chưa thể tự hành an toàn!")
            self.lbl_tf_summary.setStyleSheet("color: #ef4444; font-size: 11px; font-weight: bold;")

        # 3. AI Perception
        y_val = "BẬT" if snap.yolo_active else "TẮT"
        y_st = HealthStatus.HEALTHY if snap.yolo_active else HealthStatus.UNKNOWN
        self.card_yolo.update_data(y_val, f"Nhận diện: {snap.people_count} người", y_st)

        w_val = "ĐANG VẪY" if snap.is_waving else "KHÔNG VẪY"
        w_st = HealthStatus.WARNING if snap.is_waving else HealthStatus.HEALTHY
        self.card_wave.update_data(w_val, "Tương tác: Sẵn sàng", w_st)

        if snap.closest_person_dist is not None:
            p_val = f"{snap.closest_person_dist:.2f} m"
            p_sub = "CẢN ĐƯỜNG TIẾN" if snap.closest_person_dist < 1.8 else "Cự ly an toàn"
            self.card_person.update_data(p_val, p_sub, HealthStatus.WARNING if snap.closest_person_dist < 1.8 else HealthStatus.HEALTHY)
        else:
            self.card_person.update_data("-- m", "Không phát hiện người", HealthStatus.UNKNOWN)

        all_ok = bool(snap.lidar_hz >= 5.0 and snap.odom_hz >= 5.0 and snap.tf_map_odom_ok and snap.tf_odom_base_ok)
        if all_ok:
            self.header.set_status("HEALTHY", HealthStatus.HEALTHY)
        elif snap.lidar_hz > 0:
            self.header.set_status("CẢNH BÁO", HealthStatus.WARNING)
        else:
            self.header.set_status("CHƯA SẴN SÀNG", HealthStatus.CRITICAL)

    def _launch_rqt_image_view(self, topic: str) -> None:
        """Launch rqt_image_view process detached in background."""
        ros_domain = os.environ.get("ROS_DOMAIN_ID", "94")
        disp = os.environ.get("DISPLAY", ":0")
        cmd = (
            f"source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID={ros_domain} "
            f"&& export DISPLAY={disp} && ros2 run rqt_image_view rqt_image_view {shlex.quote(topic)}"
        )
        try:
            subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
