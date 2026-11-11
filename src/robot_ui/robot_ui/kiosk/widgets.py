from __future__ import annotations

from enum import Enum
import math
import re
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt5.QtGui import (
    QColor,
    QFont,
    QIcon,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtCore import QPointF, QRectF, QSize
from PyQt5.QtWidgets import (
    QFrame,
    QGraphicsOpacityEffect,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class VideoBackground(QWidget):
    """Loop a small MP4 as a borderless, cover-scaled Qt background."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._capture: cv2.VideoCapture | None = None
        self._pixmap = QPixmap()
        self._path: Path | None = None
        self._display_scale = 0.9
        self._vertical_offset_px = 0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._read_frame)
        self.setAttribute(Qt.WA_OpaquePaintEvent)

    def set_display_scale(self, scale: float) -> None:
        self._display_scale = max(0.5, min(1.0, scale))
        self.update()

    def set_vertical_offset(self, offset_px: int) -> None:
        self._vertical_offset_px = offset_px
        self.update()

    def set_video(self, path: Path) -> None:
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
        if not success:
            return
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        image = QImage(
            rgb_frame.data,
            width,
            height,
            channels * width,
            QImage.Format_RGB888,
        ).copy()
        self._pixmap = QPixmap.fromImage(image)
        self.update()

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#000000"))
        if not self._pixmap.isNull():
            target_size = QSize(
                max(1, round(self.width() * self._display_scale)),
                max(1, round(self.height() * self._display_scale)),
            )
            scaled = self._pixmap.scaled(
                target_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
            x = (self.width() - scaled.width()) // 2
            y = (self.height() - scaled.height()) // 2 + self._vertical_offset_px
            painter.drawPixmap(x, y, scaled)
        painter.fillRect(self.rect(), QColor(0, 0, 0, 22))

    def closeEvent(self, event: Any) -> None:
        self.stop()
        super().closeEvent(event)


class HomePage(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.video = VideoBackground(self)
        self.overlay = QWidget(self)
        self.overlay.setAttribute(Qt.WA_TranslucentBackground)

        layout = QVBoxLayout(self.overlay)
        layout.setContentsMargins(24, 22, 24, 125)
        layout.setSpacing(8)

        top_row = QHBoxLayout()
        top_row.addStretch(1)
        self.battery_label = QLabel("PIN --%")
        self.battery_label.setObjectName("homeTelemetryBadge")
        top_row.addWidget(self.battery_label)
        layout.addLayout(top_row)
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


def monochrome_icon(path: Path, color: QColor) -> QIcon:
    """Tint a monochrome icon using its alpha channel, or white luminance fallback."""
    image_data = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image_data is None:
        return QIcon(str(path))
    if image_data.ndim == 3 and image_data.shape[2] == 4 and (image_data[:, :, 3] < 255).any():
        alpha = image_data[:, :, 3].copy()
    else:
        grayscale = (
            image_data
            if image_data.ndim == 2
            else cv2.cvtColor(image_data[:, :, :3], cv2.COLOR_BGR2GRAY)
        )
        alpha = 255 - grayscale
        alpha[alpha < 24] = 0
    rgba = np.empty((alpha.shape[0], alpha.shape[1], 4), dtype=np.uint8)
    rgba[:, :, 0] = color.red()
    rgba[:, :, 1] = color.green()
    rgba[:, :, 2] = color.blue()
    rgba[:, :, 3] = alpha
    rgba = np.ascontiguousarray(rgba)
    height, width, channels = rgba.shape
    image = QImage(
        rgba.data,
        width,
        height,
        channels * width,
        QImage.Format_RGBA8888,
    ).copy()
    return QIcon(QPixmap.fromImage(image))


class MapWidget(QWidget):
    goal_selected = pyqtSignal(float, float, float)
    invalid_goal = pyqtSignal(str)

    def __init__(self, goal_icon: Path | None = None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._map: dict[str, Any] | None = None
        self._pose: dict[str, Any] | None = None
        self._path: list[list[float]] = []
        self._locations: list[dict[str, Any]] = []
        self._goal: dict[str, float] | None = None
        self._selection_enabled = True
        self._map_image = QImage()
        self._goal_icon = QPixmap(str(goal_icon)) if goal_icon is not None else QPixmap()
        if not self._goal_icon.isNull():
            self._goal_icon = self._goal_icon.scaled(
                34,
                34,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        self.setMinimumSize(300, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_map(self, map_data: dict[str, Any]) -> None:
        self._map = map_data
        width = int(map_data.get("preview_width", 0))
        height = int(map_data.get("preview_height", 0))
        cells = np.asarray(map_data.get("cells", []), dtype=np.int16)
        if width <= 0 or height <= 0 or cells.size < width * height:
            self._map_image = QImage()
            self.update()
            return
        cells = cells[: width * height].reshape((height, width))
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[cells < 0] = (49, 54, 60)
        rgb[(cells >= 0) & (cells < 50)] = (231, 234, 229)
        rgb[cells >= 50] = (27, 31, 34)
        rgb = np.ascontiguousarray(np.flipud(rgb))
        self._map_image = QImage(
            rgb.data,
            width,
            height,
            width * 3,
            QImage.Format_RGB888,
        ).copy()
        self.update()

    def set_pose(self, pose: dict[str, Any]) -> None:
        self._pose = pose
        self.update()

    def set_path(self, path: list[list[float]]) -> None:
        self._path = path
        self.update()

    def set_locations(self, locations: list[dict[str, Any]]) -> None:
        self._locations = [item for item in locations if isinstance(item, dict)]
        self.update()

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal = {"x": x, "y": y, "yaw": yaw}
        self.update()

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = enabled
        self.setCursor(Qt.PointingHandCursor if enabled else Qt.ArrowCursor)

    def _map_rect(self) -> QRectF:
        if self._map_image.isNull():
            return QRectF()
        # The rounded frame belongs to the map itself. Keeping only a two-pixel
        # inset prevents a larger coloured panel from appearing around maps
        # whose aspect ratio differs from the display.
        available = QRectF(self.rect()).adjusted(2, 2, -2, -2)
        ratio = min(
            available.width() / self._map_image.width(),
            available.height() / self._map_image.height(),
        )
        width = self._map_image.width() * ratio
        height = self._map_image.height() * ratio
        return QRectF(
            available.center().x() - width / 2,
            available.center().y() - height / 2,
            width,
            height,
        )

    def _world_to_screen(self, x: float, y: float, target: QRectF) -> QPointF | None:
        if not self._map:
            return None
        resolution = float(self._map.get("resolution", 0.0))
        if resolution <= 0:
            return None
        origin_x = float(self._map.get("origin_x", 0.0))
        origin_y = float(self._map.get("origin_y", 0.0))
        origin_yaw = float(self._map.get("origin_yaw", 0.0))
        step = int(self._map.get("sample_step", 1))
        delta_x = x - origin_x
        delta_y = y - origin_y
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        cell_x = (cosine * delta_x + sine * delta_y) / resolution / step
        cell_y = (-sine * delta_x + cosine * delta_y) / resolution / step
        preview_width = float(self._map.get("preview_width", 1))
        preview_height = float(self._map.get("preview_height", 1))
        return QPointF(
            target.left() + cell_x / preview_width * target.width(),
            target.bottom() - cell_y / preview_height * target.height(),
        )

    def _screen_to_world(self, point: QPointF, target: QRectF) -> tuple[float, float] | None:
        if not self._map or not target.contains(point):
            return None
        preview_width = int(self._map.get("preview_width", 0))
        preview_height = int(self._map.get("preview_height", 0))
        resolution = float(self._map.get("resolution", 0.0))
        step = int(self._map.get("sample_step", 1))
        if preview_width <= 0 or preview_height <= 0 or resolution <= 0:
            return None
        cell_x = (point.x() - target.left()) / target.width() * preview_width
        cell_y = (target.bottom() - point.y()) / target.height() * preview_height
        index_x = min(preview_width - 1, max(0, int(cell_x)))
        index_y = min(preview_height - 1, max(0, int(cell_y)))
        cells = self._map.get("cells", [])
        cell_index = index_y * preview_width + index_x
        if cell_index >= len(cells) or int(cells[cell_index]) < 0 or int(cells[cell_index]) >= 50:
            self.invalid_goal.emit("Hãy chọn một vùng trống trên bản đồ")
            return None
        distance_x = cell_x * resolution * step
        distance_y = cell_y * resolution * step
        origin_yaw = float(self._map.get("origin_yaw", 0.0))
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        world_x = float(self._map.get("origin_x", 0.0)) + cosine * distance_x - sine * distance_y
        world_y = float(self._map.get("origin_y", 0.0)) + sine * distance_x + cosine * distance_y
        return world_x, world_y

    def mousePressEvent(self, event: Any) -> None:
        if event.button() != Qt.LeftButton or not self._selection_enabled:
            super().mousePressEvent(event)
            return
        world = self._screen_to_world(QPointF(event.pos()), self._map_rect())
        if world is None:
            return
        x, y = world
        if self._pose:
            yaw = math.atan2(y - float(self._pose.get("y", y)), x - float(self._pose.get("x", x)))
        else:
            yaw = 0.0
        self.set_goal(x, y, yaw)
        self.goal_selected.emit(x, y, yaw)

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#000000"))
        if self._map_image.isNull():
            painter.setPen(QColor("#888888"))
            painter.setFont(QFont("Sans Serif", 13, QFont.DemiBold))
            painter.drawText(self.rect(), Qt.AlignCenter, "Đang chờ dữ liệu bản đồ…")
            return

        target = self._map_rect()
        map_clip = QPainterPath()
        map_clip.addRoundedRect(target, 10, 10)
        painter.save()
        painter.setClipPath(map_clip)
        painter.drawImage(target, self._map_image)
        painter.restore()
        painter.setPen(QPen(QColor("#3a3a3a"), 2))
        painter.drawRoundedRect(target, 10, 10)

        if len(self._path) > 1:
            points = [
                self._world_to_screen(float(item[0]), float(item[1]), target)
                for item in self._path
            ]
            points = [point for point in points if point is not None]
            if len(points) > 1:
                painter.setPen(QPen(QColor("#ffb32c"), 4, Qt.DotLine, Qt.RoundCap, Qt.RoundJoin))
                painter.drawPolyline(QPolygonF(points))

        painter.setFont(QFont("Sans Serif", 8, QFont.DemiBold))
        for location in self._locations:
            point = self._world_to_screen(
                float(location.get("x", 0.0)),
                float(location.get("y", 0.0)),
                target,
            )
            if point is None or not target.contains(point):
                continue
            name = str(location.get("name", ""))
            text_width = painter.fontMetrics().horizontalAdvance(name)
            label_rect = QRectF(point.x() - text_width / 2 - 7, point.y() - 25, text_width + 14, 18)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(242, 242, 242, 230))
            painter.drawRoundedRect(label_rect, 5, 5)
            painter.setPen(QColor("#242424"))
            painter.drawText(label_rect, Qt.AlignCenter, name)

        if self._pose:
            robot = self._world_to_screen(
                float(self._pose.get("x", 0.0)),
                float(self._pose.get("y", 0.0)),
                target,
            )
            if robot is not None and target.contains(robot):
                painter.save()
                painter.translate(robot)
                painter.rotate(-math.degrees(float(self._pose.get("yaw", 0.0))))
                robot_pen = QPen(QColor("#ffffff"))
                robot_pen.setWidthF(2.5)
                painter.setPen(robot_pen)
                painter.setBrush(QColor("#ff7048"))
                painter.drawPolygon(
                    QPolygonF(
                        [
                            QPointF(18, 0),
                            QPointF(-12, -11),
                            QPointF(-7, 0),
                            QPointF(-12, 11),
                        ]
                    )
                )
                painter.restore()

        if self._goal:
            goal = self._world_to_screen(self._goal["x"], self._goal["y"], target)
            if goal is not None and target.contains(goal):
                if not self._goal_icon.isNull():
                    painter.drawPixmap(
                        round(goal.x() - self._goal_icon.width() / 2),
                        round(goal.y() - self._goal_icon.height()),
                        self._goal_icon,
                    )
                else:
                    painter.setPen(QPen(QColor("#ffffff"), 2))
                    painter.setBrush(QColor("#ff3158"))
                    painter.drawEllipse(goal, 8, 8)


class NavigationPage(QWidget):
    home_requested = pyqtSignal()
    start_requested = pyqtSignal(float, float, float)
    cancel_requested = pyqtSignal()

    def __init__(self, icons: dict[str, Path], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._goal: tuple[float, float, float] | None = None
        self._navigation_active = False
        self.setObjectName("navigationPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        root = QVBoxLayout(self)
        root.setContentsMargins(14, 10, 14, 12)
        root.setSpacing(8)

        top_layout = QHBoxLayout()
        top_layout.setContentsMargins(0, 0, 0, 0)

        home = QPushButton()
        home.setObjectName("navHomeButton")
        home.setToolTip("Quay lại")
        home.setAccessibleName("Quay lại")
        back_icon = icons.get("back")
        if back_icon and Path(back_icon).is_file():
            home.setIcon(QIcon(str(back_icon)))
            home.setIconSize(QSize(28, 28))
            home.setFixedSize(42, 42)
        else:
            home.setIcon(monochrome_icon(icons.get("home", ""), QColor("#ffb32c")))
            home.setIconSize(QSize(22, 22))
            home.setFixedSize(34, 34)
        home.clicked.connect(self.home_requested)
        top_layout.addWidget(home)
        top_layout.addStretch(1)

        self.time_label = QLabel()
        self.time_label.setObjectName("navClock")
        self.time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        top_layout.addWidget(self.time_label)
        root.addLayout(top_layout)

        self.map_widget = MapWidget(icons["goal"])
        self.map_widget.setObjectName("mapWidget")
        self.map_widget.goal_selected.connect(self._on_goal_selected)
        self.map_widget.invalid_goal.connect(self._show_warning)
        root.addWidget(self.map_widget, 1)

        controls = QFrame()
        controls.setObjectName("navControlBar")
        controls_layout = QHBoxLayout(controls)
        controls_layout.setContentsMargins(14, 5, 10, 5)
        controls_layout.setSpacing(10)

        prompt_icon = QLabel()
        prompt_pixmap = QPixmap(str(icons["click_goal"]))
        if not prompt_pixmap.isNull():
            prompt_icon.setPixmap(
                prompt_pixmap.scaled(210, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        prompt_icon.setFixedSize(214, 72)
        prompt_icon.setAlignment(Qt.AlignCenter)
        controls_layout.addWidget(prompt_icon)

        self.status_label = QLabel("")
        self.status_label.setObjectName("navPrompt")
        controls_layout.addWidget(self.status_label, 1)

        self.start_button = QPushButton()
        self.start_button.setObjectName("navStartButton")
        self.start_button.setIcon(QIcon(str(icons["start"])))
        self.start_button.setIconSize(QSize(68, 68))
        self.start_button.setFixedSize(72, 72)
        self.start_button.clicked.connect(self._start_navigation)
        controls_layout.addWidget(self.start_button)

        self.stop_button = QPushButton()
        self.stop_button.setObjectName("navStopButton")
        self.stop_button.setIcon(QIcon(str(icons["stop"])))
        self.stop_button.setIconSize(QSize(68, 68))
        self.stop_button.setFixedSize(72, 72)
        self.stop_button.clicked.connect(self._stop_navigation)
        controls_layout.addWidget(self.stop_button)
        root.addWidget(controls)

        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)
        self._update_clock()
        self.set_navigation_active(False)

    def _set_button_strength(self, button: QPushButton, strong: bool) -> None:
        effect = button.graphicsEffect()
        if not isinstance(effect, QGraphicsOpacityEffect):
            effect = QGraphicsOpacityEffect(button)
            button.setGraphicsEffect(effect)
        effect.setOpacity(1.0 if strong else 0.28)
        button.setEnabled(strong)

    def set_navigation_active(self, active: bool) -> None:
        self._navigation_active = active
        self._set_button_strength(self.start_button, not active)
        self._set_button_strength(self.stop_button, active)
        self.map_widget.set_selection_enabled(not active)

    def _on_goal_selected(self, x: float, y: float, yaw: float) -> None:
        self._goal = (x, y, yaw)
        self.status_label.setText(f"Điểm đến: X {x:.2f} · Y {y:.2f} m")

    def _show_warning(self, message: str) -> None:
        self.status_label.setText(message)

    def _start_navigation(self) -> None:
        if self._goal is None:
            self._show_warning("Hãy chọn điểm đến trên bản đồ")
            return
        self.set_navigation_active(True)
        self.start_requested.emit(*self._goal)

    def _stop_navigation(self) -> None:
        self.set_navigation_active(False)
        self.cancel_requested.emit()

    def _update_clock(self) -> None:
        now = datetime.now()
        self.time_label.setText(now.strftime("%H:%M\n%d/%m"))

    def set_robot_status(self, status: dict[str, Any]) -> None:
        # Status data is still received by the app, but this page deliberately
        # keeps the top area visual-only with a single Home control.
        return

    def set_locations(self, locations: list[Any]) -> None:
        self.map_widget.set_locations(locations)

    def set_pose(self, pose: dict[str, Any]) -> None:
        self.map_widget.set_pose(pose)

    def set_navigation_status(self, status: dict[str, Any]) -> None:
        state = str(status.get("status", "IDLE")).upper()
        if state in {"MOVING", "FETCHING_LOCATION", "WAITING_FOR_NAV2"}:
            self.set_navigation_active(True)
        elif state in {"REACHED_GOAL", "CANCELLED", "FAILED"}:
            self.set_navigation_active(False)

    def set_command_status(self, tone: str, message: str) -> None:
        self.status_label.setText(message)
        self.status_label.setProperty("tone", tone)
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
        if tone in {"success", "error", "warning"}:
            self.set_navigation_active(False)


class MicState(Enum):
    READY = "ready"
    LISTENING = "listening"
    UNAVAILABLE = "unavailable"
    STT_PROCESSING = "stt_processing"
    LLM_PROCESSING = "llm_processing"
    TTS_PROCESSING = "tts_processing"
    SPEAKING = "speaking"
    ERROR = "error"


EMOJI_PATTERN = re.compile(
    r"[\U00010000-\U0010ffff\u2600-\u26ff\u2700-\u27bf\ufe00-\ufe0f\u200d]+",
    flags=re.UNICODE,
)


def strip_emojis(text: str) -> str:
    """Loại bỏ emoji/icon Unicode để tránh lỗi font (ô vuông rỗng) trên Linux Qt5."""
    if not text:
        return text
    cleaned = EMOJI_PATTERN.sub("", text)
    cleaned = re.sub(r" +([,.\?!;:])", r"\1", cleaned)
    lines = [re.sub(r"[ \t]+", " ", line).strip() for line in cleaned.splitlines()]
    return "\n".join(lines).strip()


class SpeakerButton(QPushButton):
    """Nút loa hỗ trợ: Bấm nhanh = Mute toggle; Bấm giữ + Kéo = Điều chỉnh âm lượng."""

    long_press_started = pyqtSignal()
    drag_moved = pyqtSignal(int, int)  # (global_x, global_y)
    drag_finished = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._press_pos: QPoint | None = None
        self._is_long_press = False
        self._drag_active = False
        self._long_timer = QTimer(self)
        self._long_timer.setSingleShot(True)
        self._long_timer.setInterval(250)
        self._long_timer.timeout.connect(self._on_long_press_timeout)

    def _on_long_press_timeout(self) -> None:
        self._is_long_press = True
        self._drag_active = True
        self.long_press_started.emit()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._press_pos = event.globalPos()
            self._is_long_press = False
            self._drag_active = False
            self._long_timer.start()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_active:
            gpos = event.globalPos()
            self.drag_moved.emit(gpos.x(), gpos.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        self._long_timer.stop()
        if self._drag_active or self._is_long_press:
            self._drag_active = False
            self.drag_finished.emit()
            self.setDown(False)
            event.accept()
            return
        super().mouseReleaseEvent(event)


class VolumeSliderWidget(QFrame):
    """Thanh trượt âm lượng pop-up ngắn gọn nằm cạnh nút loa."""

    volume_changed = pyqtSignal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("volumeSliderWidget")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setFixedHeight(44)
        self.setMinimumWidth(150)
        self.setMaximumWidth(185)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(8)

        self._label = QLabel("80%")
        self._label.setObjectName("volumeLabel")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setFixedWidth(36)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setObjectName("volumeSlider")
        self._slider.setRange(0, 100)
        self._slider.setValue(80)
        self._slider.setCursor(Qt.PointingHandCursor)
        self._slider.valueChanged.connect(self._on_slider_value_changed)

        layout.addWidget(self._label)
        layout.addWidget(self._slider, 1)

        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.setInterval(2500)
        self._hide_timer.timeout.connect(self.hide)

        self.setStyleSheet("""
            QFrame#volumeSliderWidget {
                background: #1c1c1c;
                border: 1px solid #383838;
                border-radius: 12px;
            }
            QLabel#volumeLabel {
                color: #ffb32c;
                font-size: 12px;
                font-weight: 700;
            }
            QSlider#volumeSlider::groove:horizontal {
                height: 6px;
                background: #2b2b2b;
                border-radius: 3px;
            }
            QSlider#volumeSlider::sub-page:horizontal {
                background: #ffb32c;
                border-radius: 3px;
            }
            QSlider#volumeSlider::handle:horizontal {
                background: #ffffff;
                border: 2px solid #ffb32c;
                width: 18px;
                height: 18px;
                margin: -6px 0;
                border-radius: 9px;
            }
        """)

    @property
    def value(self) -> int:
        return self._slider.value()

    def set_volume(self, value: int, notify: bool = True) -> None:
        val = max(0, min(100, int(value)))
        self._label.setText(f"{val}%")
        self._slider.blockSignals(not notify)
        self._slider.setValue(val)
        self._slider.blockSignals(False)
        if notify:
            self.volume_changed.emit(val)

    def show_slider(self) -> None:
        self.show()
        self.raise_()
        self._hide_timer.stop()

    def start_hide_timer(self) -> None:
        self._hide_timer.start(2500)

    def update_from_global_point(self, gx: int, gy: int) -> None:
        self.show_slider()
        pt_in_slider = self._slider.mapFromGlobal(QPoint(gx, gy))
        w = self._slider.width()
        if w > 0:
            ratio = max(0.0, min(1.0, pt_in_slider.x() / float(w)))
            val = int(round(ratio * 100))
            self.set_volume(val, notify=True)

    def _on_slider_value_changed(self, value: int) -> None:
        self._label.setText(f"{value}%")
        self.volume_changed.emit(value)
        self._hide_timer.start(2500)


class ChatPage(QWidget):
    chat_submitted = pyqtSignal(str)  # emitted khi user nhấn Gửi
    home_requested = pyqtSignal()
    mic_clicked = pyqtSignal()
    speaker_toggled = pyqtSignal(bool)
    stop_speaking_requested = pyqtSignal()
    volume_changed = pyqtSignal(int)

    DEFAULT_SUGGESTIONS = (
        "Giới thiệu về PTIT",
        "Giới thiệu về Robot",
        "Giới thiệu về Khoa Kỹ thuật Điện tử 1",
        "Giới thiệu về nhóm nghiên cứu OpenWorkShop",
    )

    def __init__(
        self,
        chat_icon: Path,
        parent: QWidget | None = None,
        back_icon: Path | None = None,
        voice_icons: dict[str, Path] | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("chatPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._voice_icons = voice_icons or {}
        self._speaker_enabled = True
        self._is_speaking = False
        self._mic_state = MicState.UNAVAILABLE
        self._typing_bubble: QWidget | None = None
        self._typing_dots = 1
        self._typing_timer = QTimer(self)
        self._typing_timer.timeout.connect(self._update_typing_dots)
        self._suggestions_widget: QWidget | None = None

        # SSE Streaming Bubble support with coalesced throttling
        self._current_streaming_row: QWidget | None = None
        self._current_streaming_body: QLabel | None = None
        self._current_streaming_text: str = ""
        self._streaming_turn_id: int | None = None
        self._stream_pending_text: str = ""
        self._stream_throttle_timer = QTimer(self)
        self._stream_throttle_timer.setInterval(40)
        self._stream_throttle_timer.timeout.connect(self._on_stream_throttle_timeout)

        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._animate_listening_pulse)
        self._pulse_state = False

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 12, 18, 14)
        root.setSpacing(10)

        # A. Header (64-72px)
        header = QFrame()
        header.setObjectName("chatHeader")
        header.setMinimumHeight(64)
        header.setMaximumHeight(72)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(10)

        back = QPushButton()
        back.setObjectName("navHomeButton")
        back.setToolTip("Quay lại")
        back.setAccessibleName("Quay lại")
        back.setFixedSize(46, 46)
        if back_icon and Path(back_icon).is_file():
            back.setIcon(QIcon(str(back_icon)))
            back.setIconSize(QSize(28, 28))
        else:
            back.setText("‹")
            back.setStyleSheet("color: #ffb32c; font-size: 29px; font-weight: 700;")
        back.clicked.connect(self.home_requested)
        header_layout.addWidget(back)

        icon_label = QLabel()
        icon_label.setObjectName("chatHeaderIcon")
        icon_label.setPixmap(
            monochrome_icon(chat_icon, QColor("#ffb32c")).pixmap(QSize(26, 26))
        )
        icon_label.setFixedSize(36, 36)
        icon_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(icon_label)

        heading = QVBoxLayout()
        heading.setSpacing(2)
        title = QLabel("Souta Assistant")
        title.setObjectName("chatTitle")
        self._status_label = QLabel("Sẵn sàng trò chuyện")
        self._status_label.setObjectName("chatSubtitle")
        heading.addWidget(title)
        heading.addWidget(self._status_label)
        header_layout.addLayout(heading)

        header_layout.addStretch(1)

        # LLM Status badge (compact & basic in top-right)
        self._llm_status_label = QLabel("● LLM")
        self._llm_status_label.setObjectName("llmStatus")
        self._llm_status_label.setToolTip("Trạng thái AI LLM")
        header_layout.addWidget(self._llm_status_label)

        # Stop speaking button (hidden unless speaking)
        self._stop_speaking_btn = QPushButton("Dừng nói")
        self._stop_speaking_btn.setObjectName("chatStopSpeakingButton")
        self._stop_speaking_btn.setMinimumHeight(44)
        self._stop_speaking_btn.setIcon(self._get_icon("stop_speaking"))
        self._stop_speaking_btn.setIconSize(QSize(22, 22))
        self._stop_speaking_btn.setCursor(Qt.PointingHandCursor)
        self._stop_speaking_btn.hide()
        self._stop_speaking_btn.clicked.connect(self._on_stop_speaking_clicked)
        header_layout.addWidget(self._stop_speaking_btn)

        # Volume slider popup/bar (hidden by default)
        self._volume_widget = VolumeSliderWidget()
        self._volume_widget.hide()
        self._volume_widget.volume_changed.connect(self._on_volume_widget_changed)
        header_layout.addWidget(self._volume_widget)

        # Speaker toggle button (min 48x48)
        self._speaker_btn = SpeakerButton()
        self._speaker_btn.setObjectName("chatSpeakerButton")
        self._speaker_btn.setFixedSize(48, 48)
        self._speaker_btn.setIconSize(QSize(26, 26))
        self._speaker_btn.setCursor(Qt.PointingHandCursor)
        self._speaker_btn.clicked.connect(self._toggle_speaker)
        self._speaker_btn.long_press_started.connect(self._on_speaker_long_pressed)
        self._speaker_btn.drag_moved.connect(self._on_speaker_drag_moved)
        self._speaker_btn.drag_finished.connect(self._on_speaker_drag_finished)
        header_layout.addWidget(self._speaker_btn)
        self._update_speaker_ui()

        root.addWidget(header)

        # B. Conversation Area
        conversation = QFrame()
        conversation.setObjectName("chatConversation")
        conversation_layout = QVBoxLayout(conversation)
        conversation_layout.setContentsMargins(6, 6, 6, 6)

        self.messages = QScrollArea()
        self.messages.setObjectName("chatScroll")
        self.messages.setWidgetResizable(True)
        self.messages.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.messages.setFrameShape(QFrame.NoFrame)

        self._message_container = QWidget()
        self._message_container.setObjectName("chatMessageContainer")
        self._messages_layout = QVBoxLayout(self._message_container)
        self._messages_layout.setContentsMargins(10, 10, 10, 10)
        self._messages_layout.setSpacing(10)
        self._messages_layout.addStretch(1)
        self.messages.setWidget(self._message_container)

        self.messages.verticalScrollBar().rangeChanged.connect(
            lambda _min, _max: self.messages.verticalScrollBar().setValue(_max)
        )
        conversation_layout.addWidget(self.messages)
        root.addWidget(conversation, 1)

        # C. Input Bar (72-84px)
        input_bar = QFrame()
        input_bar.setObjectName("chatInputBar")
        input_bar.setMinimumHeight(72)
        input_bar.setMaximumHeight(84)
        input_row = QHBoxLayout(input_bar)
        input_row.setContentsMargins(10, 10, 10, 10)
        input_row.setSpacing(10)

        self.input = QLineEdit()
        self.input.setObjectName("chatInput")
        self.input.setPlaceholderText("Nhập câu hỏi của bạn...")
        self.input.setMinimumHeight(52)
        self.input.returnPressed.connect(self._on_send)
        input_row.addWidget(self.input, 1)

        self.mic_btn = QPushButton()
        self.mic_btn.setObjectName("chatMicButton")
        self.mic_btn.setFixedSize(54, 52)
        self.mic_btn.setIconSize(QSize(28, 28))
        self.mic_btn.setCursor(Qt.PointingHandCursor)
        self.mic_btn.clicked.connect(self._on_mic_clicked)
        input_row.addWidget(self.mic_btn)

        self._send_btn = QPushButton("Gửi")
        self._send_btn.setObjectName("chatSendButton")
        self._send_btn.setFixedSize(76, 52)
        self._send_btn.setCursor(Qt.PointingHandCursor)
        self._send_btn.clicked.connect(self._on_send)
        input_row.addWidget(self._send_btn)

        root.addWidget(input_bar)

        # Initial state setup
        self.set_mic_state(MicState.UNAVAILABLE)
        self._add_message("Souta Robot", "Xin chào! Tôi là Souta Robot. Tôi có thể giúp gì cho bạn?")
        self._add_suggestions()

    def _get_icon(self, name: str) -> QIcon:
        path = self._voice_icons.get(name)
        if path is not None and Path(path).is_file():
            return QIcon(str(path))
        try:
            from robot_ui.kiosk.assets import icon_path
            p = icon_path(f"{name}.png")
            if p.is_file():
                return QIcon(str(p))
        except Exception:
            pass
        return QIcon()

    def _add_message(self, sender: str, message: str, from_user: bool = False) -> None:
        display_message = message if from_user else (strip_emojis(message) or message)
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("userBubble" if from_user else "robotBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel(sender)
        sender_label.setObjectName("userBubbleSender" if from_user else "robotBubbleSender")
        body = QLabel(display_message)
        body.setObjectName("bubbleText")
        body.setWordWrap(True)
        body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(body)

        if from_user:
            row_layout.addStretch(1)
            row_layout.addWidget(bubble)
        else:
            row_layout.addWidget(bubble)
            row_layout.addStretch(1)

        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)

    def _add_suggestions(self) -> None:
        self._hide_suggestions()
        container = QFrame()
        container.setObjectName("suggestionContainer")
        grid = QGridLayout(container)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setSpacing(8)
        self._suggestion_buttons: list[QPushButton] = []
        for i, text in enumerate(self.DEFAULT_SUGGESTIONS):
            btn = QPushButton(text)
            btn.setObjectName("suggestionChip")
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked=False, query=text: self._on_suggestion_clicked(query))
            grid.addWidget(btn, i // 2, i % 2)
            self._suggestion_buttons.append(btn)
        self._suggestions_widget = container
        self._messages_layout.insertWidget(self._messages_layout.count() - 1, container)

    def _hide_suggestions(self) -> None:
        if self._suggestions_widget is not None:
            self._suggestions_widget.hide()

    def _on_suggestion_clicked(self, prompt: str) -> None:
        self._hide_suggestions()
        self.input.setText(prompt)
        self._on_send()

    def _show_typing_indicator(self) -> None:
        if self._typing_bubble is not None:
            return
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("typingBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel("Souta Robot")
        sender_label.setObjectName("robotBubbleSender")
        self._typing_label = QLabel("Thinking.")
        self._typing_label.setObjectName("typingText")
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(self._typing_label)

        row_layout.addWidget(bubble)
        row_layout.addStretch(1)

        self._typing_bubble = row
        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)
        self._typing_dots = 1
        self._typing_timer.start(450)

    def _update_typing_dots(self) -> None:
        if self._typing_bubble is not None and hasattr(self, "_typing_label"):
            self._typing_dots = (self._typing_dots % 3) + 1
            dots = "." * self._typing_dots
            self._typing_label.setText(f"Thinking{dots}")

    def _remove_typing_indicator(self) -> None:
        self._typing_timer.stop()
        if self._typing_bubble is not None:
            self._messages_layout.removeWidget(self._typing_bubble)
            self._typing_bubble.deleteLater()
            self._typing_bubble = None

    def _on_send(self) -> None:
        text = self.input.text().strip()
        if not text or not self.input.isEnabled():
            return
        self._hide_suggestions()
        self.input.clear()
        self._add_message("Bạn", text, from_user=True)
        self.set_busy(True)
        self.chat_submitted.emit(text)

    def _animate_listening_pulse(self) -> None:
        self._pulse_state = not self._pulse_state
        border = "#ff4d4d" if self._pulse_state else "#ffb32c"
        self.mic_btn.setStyleSheet(f"#chatMicButton {{ border: 2px solid {border}; background-color: rgba(255, 77, 77, 0.25); }}")

    def submit_voice_transcript(self, text: str) -> None:
        """Submit voice transcript as user message and trigger LLM processing."""
        text = text.strip()
        if not text or not self.input.isEnabled():
            return
        self._hide_suggestions()
        self.input.clear()
        self._add_message("Bạn", text, from_user=True)
        self.set_busy(True)
        self.chat_submitted.emit(text)

    # ------------------------------------------------------------------
    # Voice Chat & Speaker State Management
    # ------------------------------------------------------------------

    def set_mic_state(self, state: MicState) -> None:
        self._mic_state = state
        if state != MicState.LISTENING:
            self._pulse_timer.stop()
            self.mic_btn.setStyleSheet("")

        if state not in (MicState.LLM_PROCESSING, MicState.TTS_PROCESSING, MicState.SPEAKING):
            self._stop_speaking_btn.hide()

        if state == MicState.READY:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(True)
            self.mic_btn.setToolTip("Bắt đầu nói")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Sẵn sàng trò chuyện")
        elif state == MicState.LISTENING:
            self.mic_btn.setIcon(self._get_icon("mic_listening"))
            self.mic_btn.setEnabled(True)
            self.mic_btn.setToolTip("Đang ghi âm (chạm để dừng)")
            self.mic_btn.setProperty("listening", "true")
            self._pulse_timer.start(450)
            self.set_status("Đang nghe...")
        elif state == MicState.UNAVAILABLE:
            self.mic_btn.setIcon(self._get_icon("mic_off"))
            self.mic_btn.setEnabled(True)  # click shows friendly message
            self.mic_btn.setToolTip("Trò chuyện bằng giọng nói chưa khả dụng")
            self.mic_btn.setProperty("listening", "false")
        elif state == MicState.STT_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang nhận dạng giọng nói...")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Đang nhận dạng giọng nói...")
        elif state == MicState.LLM_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang xử lý câu hỏi...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng phản hồi")
            self._stop_speaking_btn.setToolTip("Dừng câu hỏi đang xử lý")
            self._stop_speaking_btn.show()
            self.set_status("Thinking...")
        elif state == MicState.TTS_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang tạo giọng nói...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng")
            self._stop_speaking_btn.setToolTip("Hủy tạo âm thanh")
            self._stop_speaking_btn.show()
            self.set_status("Đang tạo giọng nói...")
        elif state == MicState.SPEAKING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Souta đang nói...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng nói")
            self._stop_speaking_btn.setToolTip("Dừng phát âm thanh")
            self._stop_speaking_btn.show()
            self.set_status("Souta đang nói...")
        elif state == MicState.ERROR:
            self.mic_btn.setIcon(self._get_icon("mic_off"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Lỗi microphone")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Lỗi microphone")
        self.mic_btn.style().unpolish(self.mic_btn)
        self.mic_btn.style().polish(self.mic_btn)

    def set_speaker_enabled(self, enabled: bool) -> None:
        self._speaker_enabled = enabled
        self._update_speaker_ui()

    def set_voice_status(self, message: str) -> None:
        self.set_status(message)

    def _on_mic_clicked(self) -> None:
        self.mic_clicked.emit()
        if self._mic_state == MicState.UNAVAILABLE:
            self.set_status("Trò chuyện bằng giọng nói chưa khả dụng")
            QTimer.singleShot(
                2500,
                lambda: self.set_status("Sẵn sàng trò chuyện")
                if self._mic_state == MicState.UNAVAILABLE and not self._is_speaking
                else None,
            )

    def _on_speaker_long_pressed(self) -> None:
        self._volume_widget.show_slider()

    def _on_speaker_drag_moved(self, gx: int, gy: int) -> None:
        self._volume_widget.update_from_global_point(gx, gy)

    def _on_speaker_drag_finished(self) -> None:
        self._volume_widget.start_hide_timer()

    def _on_volume_widget_changed(self, val: int) -> None:
        if val == 0 and self._speaker_enabled:
            self._speaker_enabled = False
            self._update_speaker_ui()
            self.speaker_toggled.emit(False)
        elif val > 0 and not self._speaker_enabled:
            self._speaker_enabled = True
            self._update_speaker_ui()
            self.speaker_toggled.emit(True)
        self.volume_changed.emit(val)

    def set_volume(self, val: int) -> None:
        self._volume_widget.set_volume(val, notify=False)

    def _toggle_speaker(self) -> None:
        self._speaker_enabled = not self._speaker_enabled
        if self._speaker_enabled and self._volume_widget.value == 0:
            self._volume_widget.set_volume(80, notify=True)
        self._update_speaker_ui()
        if not self._speaker_enabled and self._is_speaking:
            self.stop_speaking()
        self.speaker_toggled.emit(self._speaker_enabled)

    def _update_speaker_ui(self) -> None:
        icon_name = "speaker_on" if self._speaker_enabled else "speaker_off"
        self._speaker_btn.setIcon(self._get_icon(icon_name))
        self._speaker_btn.setToolTip("Bật âm thanh" if not self._speaker_enabled else "Tắt âm thanh")

    def set_speaker_speaking(self, is_speaking: bool) -> None:
        self._is_speaking = is_speaking
        if is_speaking:
            self._stop_speaking_btn.setText("Dừng nói")
            self._stop_speaking_btn.show()
            self.set_status("Đang nói...")
        else:
            if self._mic_state not in (MicState.LLM_PROCESSING, MicState.TTS_PROCESSING):
                self._stop_speaking_btn.hide()
            if self._mic_state != MicState.LISTENING and self.input.isEnabled():
                self.set_status("Sẵn sàng trò chuyện")

    def _on_stop_speaking_clicked(self) -> None:
        self.stop_speaking()

    def stop_speaking(self) -> None:
        self.set_speaker_speaking(False)
        self.stop_speaking_requested.emit()

    def set_status(self, status: str) -> None:
        self._status_label.setText(status)

    # ------------------------------------------------------------------
    # Public API – được gọi từ KioskWindow để wire LLM
    # ------------------------------------------------------------------

    def reset_conversation(self) -> None:
        self.stop_speaking()
        self._stream_throttle_timer.stop()
        self._remove_typing_indicator()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._current_streaming_text = ""
        self._stream_pending_text = ""
        self._streaming_turn_id = None
        self._last_finalized_text = None
        while self._messages_layout.count() > 1:
            item = self._messages_layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self._suggestions_widget = None
        self.set_busy(False)
        self.input.clear()
        self.set_mic_state(MicState.UNAVAILABLE)
        self.set_status("Sẵn sàng trò chuyện")
        self._llm_status_label.setText("● LLM")
        self._llm_status_label.setProperty("state", "ready")
        self._add_message("Souta Robot", "Xin chào! Tôi là Souta Robot. Tôi có thể giúp gì cho bạn?")
        self._add_suggestions()

    def set_llm_status(self, status: str) -> None:
        """Ghi log kỹ thuật và cập nhật UI thân thiện cho khách."""
        print(f"[robot_ui] LLM status: {status}", flush=True)
        self._llm_status_label.setToolTip(f"Trạng thái: {status}")
        if "ĐANG XỬ LÝ" in status:
            self._llm_status_label.setText("● LLM...")
            self._llm_status_label.setProperty("state", "busy")
            self.set_status("Thinking...")
            self.set_mic_state(MicState.LLM_PROCESSING)
        elif "LỖI" in status or "⚠️" in status:
            self._llm_status_label.setText("⚠ LLM")
            self._llm_status_label.setProperty("state", "error")
            self.set_status(status)
        elif "CHƯA KẾT NỐI" in status or "OFFLINE" in status:
            self._llm_status_label.setText("○ LLM")
            self._llm_status_label.setProperty("state", "offline")
            if not self._is_speaking and self._mic_state != MicState.LISTENING:
                self.set_status("Chưa kết nối LLM")
        else:
            self._llm_status_label.setText("● LLM")
            self._llm_status_label.setProperty("state", "ready")
            if not self._is_speaking and self._mic_state != MicState.LISTENING:
                self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state == MicState.LLM_PROCESSING:
                self.set_mic_state(MicState.READY)
        self._llm_status_label.style().unpolish(self._llm_status_label)
        self._llm_status_label.style().polish(self._llm_status_label)

    def _create_streaming_bubble(self, sender: str) -> tuple[QWidget, QLabel]:
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("robotBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel(sender)
        sender_label.setObjectName("robotBubbleSender")
        body = QLabel("...")
        body.setObjectName("bubbleText")
        body.setWordWrap(True)
        body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(body)

        row_layout.addWidget(bubble)
        row_layout.addStretch(1)

        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)
        return row, body

    def receive_stream_delta(self, session_id: int, turn_id: int, delta: str) -> None:
        """Receive incremental SSE token from LLM, displaying smoothly with coalesce throttling."""
        if self._current_streaming_row is None or self._streaming_turn_id != turn_id:
            self._remove_typing_indicator()
            row, body = self._create_streaming_bubble("Souta Robot")
            self._current_streaming_row = row
            self._current_streaming_body = body
            self._current_streaming_text = ""
            self._stream_pending_text = ""
            self._streaming_turn_id = turn_id
            self._stop_speaking_btn.setText("Dừng phản hồi")
            self._stop_speaking_btn.show()

        self._stream_pending_text += delta
        if not self._stream_throttle_timer.isActive():
            self._stream_throttle_timer.start(40)

    def _on_stream_throttle_timeout(self) -> None:
        self._stream_throttle_timer.stop()
        if self._current_streaming_body is None:
            return
        self._current_streaming_text += self._stream_pending_text
        self._stream_pending_text = ""
        display = strip_emojis(self._current_streaming_text) or self._current_streaming_text
        self._current_streaming_body.setText(display)
        self.messages.verticalScrollBar().setValue(self.messages.verticalScrollBar().maximum())

    def finalize_stream(self, session_id: int, turn_id: int, full_text: str, interrupted: bool = False) -> None:
        """Finalize streamed message bubble on stream complete or user stop."""
        self._stream_throttle_timer.stop()
        if self._stream_pending_text:
            self._current_streaming_text += self._stream_pending_text
            self._stream_pending_text = ""
        self._remove_typing_indicator()

        final_content = full_text.strip() if full_text.strip() else self._current_streaming_text.strip()
        if interrupted and final_content:
            final_content += " (Đã dừng)"

        if self._current_streaming_body is not None:
            display = strip_emojis(final_content) or final_content
            self._current_streaming_body.setText(display)
        elif final_content:
            display = strip_emojis(final_content) or final_content
            self._add_message("Souta Robot", display)

        self._last_finalized_text = final_content.strip()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._current_streaming_text = ""
        self._streaming_turn_id = None
        self.set_busy(False)

    def receive_response(self, response: str) -> None:
        """Hiển thị response từ LLM và mở khóa input."""
        if self._current_streaming_body is not None:
            self.finalize_stream(0, 0, response, interrupted=False)
            return
        last_finalized = getattr(self, "_last_finalized_text", None)
        if last_finalized and last_finalized == response.strip():
            self._last_finalized_text = None
            self.set_busy(False)
            return
        self.set_busy(False)
        clean_text = strip_emojis(response)
        self._add_message("Souta Robot", clean_text or response)

    def receive_error(self, error_msg: str) -> None:
        """Hiển thị lỗi và mở khóa input."""
        self._stream_throttle_timer.stop()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._streaming_turn_id = None
        self._last_finalized_text = None
        self.set_busy(False)
        clean_msg = strip_emojis(error_msg) or error_msg
        self._add_message("Souta Robot", f"Lỗi: {clean_msg}")
        self.set_status(f"Lỗi: {clean_msg}")

    def set_busy(self, busy: bool) -> None:
        """Khoá/mở input trong khi đang chờ LLM."""
        self.input.setEnabled(not busy)
        self._send_btn.setEnabled(not busy)
        if busy:
            self._send_btn.setText("...")
            self.set_status("Thinking...")
            self._show_typing_indicator()
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(False)
        else:
            self._send_btn.setText("Gửi")
            self._remove_typing_indicator()
            self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(True)
            self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(True)


class NavButton(QPushButton):
    def __init__(self, text: str, icon: Path, page_id: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.page_id = page_id
        self.icon_path = icon
        self.setObjectName("kioskNavButton")
        self.setToolTip(text)
        self.setAccessibleName(text)
        self.setCheckable(True)
        self.setIconSize(QSize(25, 25))
        self.setFixedSize(70, 52)

    def refresh_appearance(self, dark_background: bool, active: bool) -> None:
        self.setProperty("dark", dark_background)
        if active:
            color = QColor("#ffb32c" if dark_background else "#d97800")
        else:
            color = QColor("#ffffff" if dark_background else "#26343c")
        # Preserve full-colour icons (Settings, Service, Emergency) instead of monochrome tinting.
        if self.page_id in {"settings", "service", "emergency"} or self.icon_path.suffix.lower() in {".jpg", ".jpeg"}:
            self.setIcon(QIcon(str(self.icon_path)))
        else:
            self.setIcon(monochrome_icon(self.icon_path, color))
        self.style().unpolish(self)
        self.style().polish(self)


class BottomNavigation(QFrame):
    page_requested = pyqtSignal(str)

    def __init__(self, icons: dict[str, Path], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("bottomNavigation")
        self._buttons: dict[str, NavButton] = {}
        self._dark_background = True
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 7, 10, 7)
        layout.setSpacing(8)
        labels = {
            "home": "Trang chủ",
            "chat": "Trò chuyện",
            "service": "Dịch vụ",
            "emergency": "Dừng khẩn",
            "settings": "Quản trị",
            "navigation": "Dẫn đường",
        }
        for page_id, path in icons.items():
            button = NavButton(labels.get(page_id, page_id), path, page_id)
            button.clicked.connect(lambda checked=False, name=page_id: self.page_requested.emit(name))
            layout.addWidget(button)
            self._buttons[page_id] = button
        self.set_active("home")

    def set_active(self, page_id: str) -> None:
        for name, button in self._buttons.items():
            active = name == page_id
            button.setChecked(active)
            button.refresh_appearance(self._dark_background, active)

    def set_dark_background(self, enabled: bool) -> None:
        self._dark_background = enabled
        for button in self._buttons.values():
            button.refresh_appearance(enabled, button.isChecked())
