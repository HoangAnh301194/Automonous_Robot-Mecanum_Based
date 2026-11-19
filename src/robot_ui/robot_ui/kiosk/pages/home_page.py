"""Standby idle page playing facial expressions and video loops."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget


class VideoBackground(QWidget):
    """Loop a small MP4 as a borderless, cover-scaled Qt background."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._capture: cv2.VideoCapture | None = None
        self._pixmap = QPixmap()
        self._static_image = QPixmap()
        self._path: Path | None = None
        self._display_scale = 0.9
        self._vertical_offset_px = 0
        self._cached_target_w = 0
        self._cached_target_h = 0
        self._frame_aspect = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._read_frame)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def _update_target_dims(self) -> None:
        """Tính toán và lưu cache kích thước hiển thị mục tiêu (giữ tỷ lệ khung hình)."""
        max_w = max(1, round(self.width() * self._display_scale))
        max_h = max(1, round(self.height() * self._display_scale))
        if self._frame_aspect > 0:
            target_w = max_w
            target_h = round(target_w / self._frame_aspect)
            if target_h > max_h:
                target_h = max_h
                target_w = round(target_h * self._frame_aspect)
            self._cached_target_w = max(1, target_w)
            self._cached_target_h = max(1, target_h)
        else:
            self._cached_target_w = max_w
            self._cached_target_h = max_h

    def resizeEvent(self, event: Any) -> None:
        self._update_target_dims()
        super().resizeEvent(event)

    def set_display_scale(self, scale: float) -> None:
        self._display_scale = max(0.5, min(1.0, scale))
        self._update_target_dims()
        self.update()

    def set_vertical_offset(self, offset_px: int) -> None:
        self._vertical_offset_px = offset_px
        self.update()

    def set_static_image(self, path: Path) -> None:
        """Optional static PTIT mascot preview, without MP4 decoding."""
        self.stop()
        self._static_image = QPixmap(str(path))
        self.update()

    def set_video(self, path: Path) -> None:
        if not self._static_image.isNull():
            return
        if self._path == path and self._capture is not None:
            return
        self.stop()
        self._path = path
        capture = cv2.VideoCapture(str(path))
        if not capture.isOpened():
            self._capture = None
            self.update()
            return
        self._capture = capture
        fw = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        fh = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self._frame_aspect = (fw / fh) if (fw > 0 and fh > 0) else 0.0
        self._update_target_dims()
        fps = capture.get(cv2.CAP_PROP_FPS)
        interval_ms = round(1000.0 / fps) if fps and fps > 1 else 40
        self._timer.start(max(16, interval_ms))
        self._read_frame()

    def start(self) -> None:
        if self._capture is not None and not self._timer.isActive():
            fps = self._capture.get(cv2.CAP_PROP_FPS)
            self._timer.start(max(16, round(1000.0 / fps) if fps and fps > 1 else 40))

    def pause(self) -> None:
        self._timer.stop()

    def stop(self) -> None:
        self._timer.stop()
        if self._capture is not None:
            self._capture.release()
        self._capture = None

    def _read_frame(self) -> None:
        if self._capture is None:
            return
        success, frame = self._capture.read()
        if not success:
            self._capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            success, frame = self._capture.read()
        if not success or frame is None:
            return

        target_w = self._cached_target_w
        target_h = self._cached_target_h
        if target_w <= 0 or target_h <= 0:
            self._update_target_dims()
            target_w = self._cached_target_w
            target_h = self._cached_target_h

        # 1. Scale bằng OpenCV SIMD trực tiếp trên numpy frame trước khi đổi hệ màu
        if target_w > 0 and target_h > 0 and (frame.shape[1] != target_w or frame.shape[0] != target_h):
            proc_frame = cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        else:
            proc_frame = frame

        # 2. Chuyển hệ màu trên frame đã thu nhỏ để tiết kiệm xung nhịp CPU
        rgb_frame = cv2.cvtColor(proc_frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape

        # 3. Chuyển đổi trực tiếp sang QImage / QPixmap không qua QImage.scaled()
        image = QImage(rgb_frame.data, w, h, ch * w, QImage.Format_RGB888)
        self._pixmap = QPixmap.fromImage(image)
        self.update()

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        # This screen is intentionally light even when the rest of Kiosk is dark.
        # Match the white background baked into mochi_idle_light.jpg so there
        # are no black bands at wide screen aspect ratios.
        painter.fillRect(self.rect(), QColor("#FFFFFF"))
        if not self._static_image.isNull():
            image = self._static_image.scaled(
                max(1, round(self.width() * self._display_scale)),
                max(1, round(self.height() * self._display_scale)),
                Qt.KeepAspectRatio, Qt.SmoothTransformation,
            )
            x = (self.width() - image.width()) // 2
            y = (self.height() - image.height()) // 2 + self._vertical_offset_px
            painter.drawPixmap(x, y, image)
            return
        if not self._pixmap.isNull():
            x = (self.width() - self._pixmap.width()) // 2
            y = (self.height() - self._pixmap.height()) // 2 + self._vertical_offset_px
            painter.drawPixmap(x, y, self._pixmap)
        # Restore a subtle dark overlay for MP4 expressions.
        painter.fillRect(self.rect(), QColor(0, 0, 0, 22))

    def closeEvent(self, event: Any) -> None:
        self.stop()
        super().closeEvent(event)


from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)
import yaml


class MissionTimelineWidget(QFrame):
    """Thanh hiển thị lộ trình tuần tra thời gian thực và cơ chế rẽ nhánh sự kiện (mời khách 5s)."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("missionTimelineCard")
        self.setAttribute(Qt.WA_StyledBackground, True)

        self._patrol_route: list[str] = []
        self._waypoint_notes: dict[str, str] = {}
        self._current_goal_name: str = ""
        self._distance_remaining: float = 0.0
        self._nav_status: str = "IDLE"
        self._is_branching_event: bool = False

        # Event countdown timer
        self._event_timer = QTimer(self)
        self._event_timer.setInterval(100)  # 100ms for smooth progress bar
        self._event_timer.timeout.connect(self._on_event_tick)
        self._event_total_ms: int = 5000
        self._event_remaining_ms: int = 0
        self._event_title: str = "Mời khách tương tác"
        self._event_description: str = ""

        self._init_ui()
        self._load_fallback_route()
        self._render_timeline()

    def _init_ui(self) -> None:
        self.setStyleSheet("""
            QFrame#missionTimelineCard {
                background-color: rgba(11, 18, 29, 0.92);
                border: 1.5px solid rgba(56, 189, 248, 0.35);
                border-radius: 18px;
            }
            QLabel {
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            }
            QLabel#timelineHeaderStatus {
                background-color: rgba(14, 165, 233, 0.2);
                color: #38bdf8;
                border: 1px solid rgba(56, 189, 248, 0.4);
                border-radius: 10px;
                padding: 4px 12px;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#timelineHeaderStatus[mode="patrol"] {
                background-color: rgba(34, 197, 94, 0.2);
                color: #4ade80;
                border: 1px solid rgba(74, 222, 128, 0.4);
            }
            QLabel#timelineHeaderStatus[mode="branch"] {
                background-color: rgba(245, 158, 11, 0.25);
                color: #fbbf24;
                border: 1px solid rgba(251, 191, 36, 0.5);
            }
            QLabel#timelineHeaderGoal {
                color: #f8fafc;
                font-size: 15px;
                font-weight: 700;
            }
            QLabel#timelineHeaderDist {
                color: #94a3b8;
                font-size: 13px;
                font-weight: 600;
            }
            /* Event Branch Card */
            QFrame#eventBranchBox {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 rgba(180, 83, 9, 0.95), stop:1 rgba(146, 64, 14, 0.95));
                border: 2px solid #fbbf24;
                border-radius: 14px;
                padding: 8px 14px;
            }
            QLabel#eventBranchTitle {
                color: #ffffff;
                font-size: 15px;
                font-weight: 800;
            }
            QLabel#eventBranchCountdown {
                color: #fef08a;
                font-size: 14px;
                font-weight: 800;
            }
            QLabel#eventBranchDesc {
                color: #fef3c7;
                font-size: 13px;
                font-weight: 500;
            }
            QPushButton#btnDismissBranch {
                background-color: rgba(0, 0, 0, 0.35);
                color: #fef08a;
                border: 1px solid rgba(254, 240, 138, 0.5);
                border-radius: 8px;
                padding: 4px 10px;
                font-size: 12px;
                font-weight: 700;
            }
            QPushButton#btnDismissBranch:hover {
                background-color: rgba(0, 0, 0, 0.55);
            }
            QProgressBar#eventProgressBar {
                border: none;
                border-radius: 4px;
                background-color: rgba(0, 0, 0, 0.3);
                height: 6px;
                text-align: center;
            }
            QProgressBar#eventProgressBar::chunk {
                border-radius: 4px;
                background-color: #fbbf24;
            }
            /* Timeline Chips */
            QFrame#chipCompleted {
                background-color: rgba(51, 65, 85, 0.65);
                border: 1px solid rgba(148, 163, 184, 0.3);
                border-radius: 10px;
                padding: 6px 10px;
            }
            QLabel#chipTextCompleted {
                color: #94a3b8;
                font-size: 13px;
                font-weight: 600;
            }
            QFrame#chipActive {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #0284c7, stop:1 #0369a1);
                border: 2px solid #38bdf8;
                border-radius: 10px;
                padding: 6px 12px;
            }
            QLabel#chipTextActive {
                color: #ffffff;
                font-size: 14px;
                font-weight: 800;
            }
            QFrame#chipPaused {
                background-color: rgba(71, 85, 105, 0.45);
                border: 1.5px dashed #f59e0b;
                border-radius: 10px;
                padding: 6px 12px;
            }
            QLabel#chipTextPaused {
                color: #cbd5e1;
                font-size: 13px;
                font-weight: 700;
            }
            QFrame#chipPending {
                background-color: rgba(30, 41, 59, 0.45);
                border: 1px solid rgba(148, 163, 184, 0.15);
                border-radius: 10px;
                padding: 6px 10px;
            }
            QLabel#chipTextPending {
                color: #64748b;
                font-size: 13px;
                font-weight: 600;
            }
            QLabel#chipArrow {
                color: #475569;
                font-size: 13px;
                font-weight: 700;
            }
            QLabel#chipArrowActive {
                color: #38bdf8;
                font-size: 14px;
                font-weight: 800;
            }
        """)

        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(16, 12, 16, 12)
        main_layout.setSpacing(10)

        # 1. Top Header Row: Status Badge | Target Info | Distance
        top_row = QHBoxLayout()
        top_row.setSpacing(12)

        self.lbl_status = QLabel("🚏 SẴN SÀNG")
        self.lbl_status.setObjectName("timelineHeaderStatus")
        self.lbl_status.setProperty("mode", "idle")
        top_row.addWidget(self.lbl_status)

        self.lbl_goal_info = QLabel("Lộ trình tuần tra")
        self.lbl_goal_info.setObjectName("timelineHeaderGoal")
        top_row.addWidget(self.lbl_goal_info, 1)

        self.lbl_dist_info = QLabel("")
        self.lbl_dist_info.setObjectName("timelineHeaderDist")
        top_row.addWidget(self.lbl_dist_info)

        main_layout.addLayout(top_row)

        # 2. Event Branch Box (Hidden by default, shown during 5s invite / interrupt)
        self.event_branch_box = QFrame()
        self.event_branch_box.setObjectName("eventBranchBox")
        self.event_branch_box.setVisible(False)
        branch_layout = QVBoxLayout(self.event_branch_box)
        branch_layout.setContentsMargins(10, 8, 10, 8)
        branch_layout.setSpacing(6)

        event_header = QHBoxLayout()
        self.lbl_event_title = QLabel("⏳ [RẼ NHÁNH SỰ KIỆN] Mời khách tương tác")
        self.lbl_event_title.setObjectName("eventBranchTitle")
        event_header.addWidget(self.lbl_event_title, 1)

        self.lbl_event_countdown = QLabel("Còn 5s...")
        self.lbl_event_countdown.setObjectName("eventBranchCountdown")
        event_header.addWidget(self.lbl_event_countdown)

        self.btn_dismiss_event = QPushButton("✕ Bỏ qua")
        self.btn_dismiss_event.setObjectName("btnDismissBranch")
        self.btn_dismiss_event.clicked.connect(self.clear_event_branch)
        event_header.addWidget(self.btn_dismiss_event)
        branch_layout.addLayout(event_header)

        self.lbl_event_desc = QLabel("Đang chào mời khách tương tác trước mặt robot...")
        self.lbl_event_desc.setObjectName("eventBranchDesc")
        branch_layout.addWidget(self.lbl_event_desc)

        self.event_progress = QProgressBar()
        self.event_progress.setObjectName("eventProgressBar")
        self.event_progress.setRange(0, 100)
        self.event_progress.setValue(100)
        self.event_progress.setTextVisible(False)
        branch_layout.addWidget(self.event_progress)

        main_layout.addWidget(self.event_branch_box)

        # 3. Horizontal Waypoint Timeline Flow
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFixedHeight(50)
        self.scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_area.setStyleSheet("QScrollArea { background: transparent; border: none; }")

        self.timeline_container = QWidget()
        self.timeline_container.setStyleSheet("background: transparent;")
        self.timeline_flow_layout = QHBoxLayout(self.timeline_container)
        self.timeline_flow_layout.setContentsMargins(4, 2, 4, 2)
        self.timeline_flow_layout.setSpacing(8)
        self.scroll_area.setWidget(self.timeline_container)

        main_layout.addWidget(self.scroll_area)

    def _load_fallback_route(self) -> None:
        """Load default patrol route from config/routes.yaml if available."""
        candidates = [
            Path.cwd() / "config",
            Path(__file__).resolve().parents[5] / "config",
            Path.home() / "ros2_ws" / "config",
        ]
        for c in candidates:
            route_f = c / "routes.yaml"
            loc_f = c / "locations.yaml"
            if route_f.is_file():
                try:
                    with open(route_f, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f) or {}
                        self._patrol_route = data.get("patrol_route", [])
                    if loc_f.is_file():
                        with open(loc_f, "r", encoding="utf-8") as lf:
                            loc_data = yaml.safe_load(lf) or {}
                            for k, v in loc_data.items():
                                if isinstance(v, dict) and "note" in v:
                                    self._waypoint_notes[k] = str(v["note"])
                    break
                except Exception:
                    pass

    def set_patrol_route(self, route: list[Any]) -> None:
        """Update list of waypoints from UI or ROS2 node."""
        cleaned: list[str] = []
        for item in route:
            if isinstance(item, str):
                cleaned.append(item)
            elif isinstance(item, dict) and "name" in item:
                cleaned.append(str(item["name"]))
                if "note" in item and item["note"]:
                    self._waypoint_notes[str(item["name"])] = str(item["note"])
        if cleaned:
            self._patrol_route = cleaned
            self._render_timeline()

    def set_navigation_status(self, status: dict[str, Any]) -> None:
        """Update current goal name, distance remaining, and patrol state."""
        self._nav_status = str(status.get("status", "IDLE")).upper()
        goal = str(status.get("current_goal_name", "")).strip()
        if goal:
            self._current_goal_name = goal
        dist = status.get("distance_remaining")
        if isinstance(dist, (int, float)):
            self._distance_remaining = float(dist)

        # Update top header text
        if self._is_branching_event:
            self.lbl_status.setText("⏳ RẼ NHÁNH")
            self.lbl_status.setProperty("mode", "branch")
            self.lbl_goal_info.setText(f"Tạm hoãn: {self._display_name(self._current_goal_name)}")
            self.lbl_dist_info.setText(f"Cách đích: {self._distance_remaining:.1f}m")
        elif self._nav_status in {"MOVING", "WAITING_FOR_NAV2", "FETCHING_LOCATION"}:
            self.lbl_status.setText("🟢 ĐANG TUẦN TRA")
            self.lbl_status.setProperty("mode", "patrol")
            target_disp = self._display_name(self._current_goal_name)
            self.lbl_goal_info.setText(f"Đang di chuyển đến: {target_disp}")
            self.lbl_dist_info.setText(f"Còn lại: {self._distance_remaining:.1f}m")
        elif self._nav_status == "REACHED_GOAL":
            self.lbl_status.setText("✓ ĐÃ ĐẾN NƠI")
            self.lbl_status.setProperty("mode", "patrol")
            self.lbl_goal_info.setText(f"Đã đến: {self._display_name(self._current_goal_name)}")
            self.lbl_dist_info.setText("0.0m")
        elif self._nav_status in {"PAUSED", "CANCELLED"}:
            self.lbl_status.setText("⏸️ TẠM DỪNG")
            self.lbl_status.setProperty("mode", "idle")
            self.lbl_goal_info.setText("Tạm dừng lộ trình")
            self.lbl_dist_info.setText("")
        else:
            self.lbl_status.setText("🚏 SẴN SÀNG")
            self.lbl_status.setProperty("mode", "idle")
            self.lbl_goal_info.setText("Lộ trình tuần tra sẵn sàng")
            self.lbl_dist_info.setText("")

        self.lbl_status.style().unpolish(self.lbl_status)
        self.lbl_status.style().polish(self.lbl_status)
        self._render_timeline()

    def set_nav_feedback(self, feedback: dict[str, Any]) -> None:
        dist = feedback.get("dist_remain")
        if isinstance(dist, (int, float)):
            self._distance_remaining = float(dist)
            if self._nav_status in {"MOVING", "WAITING_FOR_NAV2"} and not self._is_branching_event:
                self.lbl_dist_info.setText(f"Còn lại: {self._distance_remaining:.1f}m")
                self._render_timeline()

    def trigger_event_branch(
        self,
        title: str = "Mời khách tương tác",
        duration_sec: int = 5,
        description: str = "Đang chào mời khách tương tác trước mặt...",
    ) -> None:
        """Kích hoạt rẽ nhánh sự kiện: làm xám điểm đích và chạy đếm ngược lùi."""
        self._is_branching_event = True
        self._event_title = title
        self._event_description = description
        self._event_total_ms = max(1000, duration_sec * 1000)
        self._event_remaining_ms = self._event_total_ms

        self.lbl_event_title.setText(f"⏳ [RẼ NHÁNH SỰ KIỆN] {title}")
        self.lbl_event_desc.setText(description)
        self.lbl_event_countdown.setText(f"Còn {duration_sec}s...")
        self.event_progress.setValue(100)
        self.event_branch_box.setVisible(True)

        self._event_timer.start()
        # Refresh header and timeline with grayed-out target
        self.set_navigation_status({"status": self._nav_status, "current_goal_name": self._current_goal_name})

    def clear_event_branch(self) -> None:
        """Đóng nhánh rẽ sự kiện, khôi phục điểm đích sáng xanh và tiếp tục lộ trình."""
        self._event_timer.stop()
        self._is_branching_event = False
        self.event_branch_box.setVisible(False)
        self.set_navigation_status({"status": self._nav_status, "current_goal_name": self._current_goal_name})

    def _on_event_tick(self) -> None:
        self._event_remaining_ms -= 100
        if self._event_remaining_ms <= 0:
            self.clear_event_branch()
            return
        sec = max(1, round(self._event_remaining_ms / 1000.0))
        self.lbl_event_countdown.setText(f"Còn {sec}s...")
        pct = max(0, min(100, round((self._event_remaining_ms / self._event_total_ms) * 100)))
        self.event_progress.setValue(pct)

    def _display_name(self, name: str) -> str:
        if not name:
            return "Chưa chọn điểm"
        if name in self._waypoint_notes and self._waypoint_notes[name]:
            return f"{name} ({self._waypoint_notes[name]})"
        return name

    def _render_timeline(self) -> None:
        """Dựng giao diện chuỗi waypoint dạng Timeline."""
        # Clean current layout
        while self.timeline_flow_layout.count():
            item = self.timeline_flow_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()

        if not self._patrol_route:
            placeholder = QLabel("Chưa cấu hình danh sách điểm tuần tra")
            placeholder.setStyleSheet("color: #64748b; font-size: 13px; font-style: italic;")
            self.timeline_flow_layout.addWidget(placeholder)
            return

        # Determine target index in route
        target_idx = -1
        if self._current_goal_name:
            try:
                target_idx = self._patrol_route.index(self._current_goal_name)
            except ValueError:
                # Fuzzy match
                for i, wp in enumerate(self._patrol_route):
                    if wp in self._current_goal_name or self._current_goal_name in wp:
                        target_idx = i
                        break

        total_pts = len(self._patrol_route)
        for i, wp_name in enumerate(self._patrol_route):
            disp = self._waypoint_notes.get(wp_name, wp_name)

            chip = QFrame()
            chip_layout = QHBoxLayout(chip)
            chip_layout.setContentsMargins(8, 4, 8, 4)
            chip_layout.setSpacing(4)

            # State evaluation
            if target_idx != -1 and i < target_idx:
                # COMPLETED POINT (Grayed out with checkmark)
                chip.setObjectName("chipCompleted")
                lbl = QLabel(f"✓ {disp}")
                lbl.setObjectName("chipTextCompleted")
                chip_layout.addWidget(lbl)
            elif i == target_idx:
                # CURRENT ACTIVE TARGET
                if self._is_branching_event:
                    # GRAYED OUT / PAUSED DUE TO EVENT
                    chip.setObjectName("chipPaused")
                    lbl = QLabel(f"⏸️ {disp} [Tạm hoãn]")
                    lbl.setObjectName("chipTextPaused")
                    chip_layout.addWidget(lbl)
                else:
                    # HIGHLIGHT ACTIVE TARGET
                    chip.setObjectName("chipActive")
                    dist_txt = f" ({self._distance_remaining:.1f}m)" if self._distance_remaining > 0 else ""
                    lbl = QLabel(f"📍 {disp}{dist_txt}")
                    lbl.setObjectName("chipTextActive")
                    chip_layout.addWidget(lbl)
            else:
                # FUTURE PENDING POINT
                chip.setObjectName("chipPending")
                lbl = QLabel(disp)
                lbl.setObjectName("chipTextPending")
                chip_layout.addWidget(lbl)

            self.timeline_flow_layout.addWidget(chip)

            # Connector Arrow
            if i < total_pts - 1:
                arrow = QLabel("➜")
                if target_idx != -1 and i == target_idx - 1 and not self._is_branching_event:
                    arrow.setObjectName("chipArrowActive")
                else:
                    arrow.setObjectName("chipArrow")
                self.timeline_flow_layout.addWidget(arrow)

        self.timeline_flow_layout.addStretch(1)


class HomePage(QWidget):
    """Màn hình chờ chính (Idle / Standby screen) hiển thị biểu cảm Souta Robot qua video nền."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("idleHomePage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet("QWidget#idleHomePage { background-color: #FFFFFF; }")
        self.video = VideoBackground(self)
        self.overlay = QWidget(self)
        self.overlay.setAttribute(Qt.WA_TranslucentBackground)

        layout = QVBoxLayout(self.overlay)
        layout.setContentsMargins(24, 20, 24, 125)
        layout.setSpacing(10)

        top_row = QHBoxLayout()
        top_row.addStretch(1)
        self.battery_label = QLabel("PIN --%")
        self.battery_label.setObjectName("homeTelemetryBadge")
        top_row.addWidget(self.battery_label)
        layout.addLayout(top_row)

        # Thanh tiến trình lộ trình tuần tra và rẽ nhánh sự kiện trực quan
        self.mission_timeline = MissionTimelineWidget(self)
        layout.addWidget(self.mission_timeline)

        layout.addStretch(1)

    def resizeEvent(self, event: Any) -> None:
        self.video.setGeometry(self.rect())
        self.overlay.setGeometry(self.rect())
        self.video.lower()
        self.overlay.raise_()
        super().resizeEvent(event)

    def set_robot_status(self, status: dict[str, Any]) -> None:
        battery = status.get("battery_percent")
        self.battery_label.setText(f"PIN {battery:.0f}%" if isinstance(battery, float) else "PIN --%")

    def set_patrol_route(self, route: list[Any]) -> None:
        """Đồng bộ danh sách waypoint lộ trình vào thanh timeline."""
        if hasattr(self, "mission_timeline"):
            self.mission_timeline.set_patrol_route(route)

    def set_navigation_status(self, status: dict[str, Any]) -> None:
        """Cập nhật trạng thái di chuyển, điểm mục tiêu và khoảng cách."""
        if hasattr(self, "mission_timeline"):
            self.mission_timeline.set_navigation_status(status)

    def set_nav_feedback(self, feedback: dict[str, Any]) -> None:
        """Cập nhật cự ly còn lại thời gian thực."""
        if hasattr(self, "mission_timeline"):
            self.mission_timeline.set_nav_feedback(feedback)

    def trigger_event_branch(
        self,
        title: str = "Mời khách tương tác",
        duration_sec: int = 5,
        description: str = "Đang chào mời khách tương tác trước mặt...",
    ) -> None:
        """Kích hoạt rẽ nhánh sự kiện (mời khách 5s) và làm xám điểm mục tiêu."""
        if hasattr(self, "mission_timeline"):
            self.mission_timeline.trigger_event_branch(title, duration_sec, description)

    def clear_event_branch(self) -> None:
        """Thu gọn nhánh rẽ sự kiện và tiếp tục lộ trình."""
        if hasattr(self, "mission_timeline"):
            self.mission_timeline.clear_event_branch()
