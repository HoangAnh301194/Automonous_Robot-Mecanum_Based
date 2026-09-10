from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
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
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
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
    """Use image luminance as alpha so the opaque white asset becomes transparent."""
    grayscale = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if grayscale is None:
        return QIcon(str(path))
    alpha = 255 - grayscale
    alpha[alpha < 24] = 0
    rgba = np.empty((*grayscale.shape, 4), dtype=np.uint8)
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
        home.setToolTip("Về trang chủ")
        home.setAccessibleName("Về trang chủ")
        home.setIcon(monochrome_icon(icons["home"], QColor("#ffb32c")))
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


class ChatPage(QWidget):
    chat_submitted = pyqtSignal(str)  # emitted khi user nhấn Gửi
    def __init__(self, chat_icon: Path, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("chatPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 14, 20, 84)
        root.setSpacing(10)

        header = QFrame()
        header.setObjectName("chatHeader")
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 7, 12, 7)
        header_layout.setSpacing(10)

        icon_label = QLabel()
        icon_label.setObjectName("chatHeaderIcon")
        icon_label.setPixmap(
            monochrome_icon(chat_icon, QColor("#ffb32c")).pixmap(QSize(26, 26))
        )
        icon_label.setFixedSize(32, 32)
        icon_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(icon_label)

        heading = QVBoxLayout()
        heading.setSpacing(0)
        title = QLabel("TRÒ CHUYỆN")
        title.setObjectName("chatTitle")
        subtitle = QLabel("Trợ lý Dasai Mochi")
        subtitle.setObjectName("chatSubtitle")
        heading.addWidget(title)
        heading.addWidget(subtitle)
        header_layout.addLayout(heading)
        header_layout.addStretch(1)

        llm_status = QLabel("●  LLM: CHƯA KẾT NỐI")
        llm_status.setObjectName("llmStatus")
        header_layout.addWidget(llm_status)
        root.addWidget(header)

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
        self._messages_layout.setContentsMargins(8, 8, 8, 8)
        self._messages_layout.setSpacing(8)
        self._messages_layout.addStretch(1)
        self.messages.setWidget(self._message_container)
        # Auto-scroll: khi content mới thêm vào làm range thay đổi → cuộn xuống cuối
        self.messages.verticalScrollBar().rangeChanged.connect(
            lambda _min, _max: self.messages.verticalScrollBar().setValue(_max)
        )
        conversation_layout.addWidget(self.messages)
        root.addWidget(conversation, 1)

        input_bar = QFrame()
        input_bar.setObjectName("chatInputBar")
        input_row = QHBoxLayout(input_bar)
        input_row.setContentsMargins(8, 7, 7, 7)
        input_row.setSpacing(8)
        self.input = QLineEdit()
        self.input.setObjectName("chatInput")
        self.input.setPlaceholderText("Nhập nội dung...")
        self.input.setMinimumHeight(48)
        self._send_btn = QPushButton("Gửi")
        self._send_btn.setObjectName("primaryButton")
        self._send_btn.setMinimumSize(90, 48)
        self._send_btn.clicked.connect(self._on_send)
        self.input.returnPressed.connect(self._on_send)
        input_row.addWidget(self.input, 1)
        input_row.addWidget(self._send_btn)
        root.addWidget(input_bar)

        self._llm_status_label = llm_status  # giữ tham chiếu để cập nhật
        self._add_message("Souta", "Xin chào! Tôi là Souta. Tôi có thể giúp gì cho bạn?")

    def _add_message(self, sender: str, message: str, from_user: bool = False) -> None:
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("userBubble" if from_user else "robotBubble")
        bubble.setMaximumWidth(560)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(13, 8, 13, 9)
        bubble_layout.setSpacing(3)

        sender_label = QLabel(sender)
        sender_label.setObjectName("userBubbleSender" if from_user else "robotBubbleSender")
        body = QLabel(message)
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

    def _on_send(self) -> None:
        text = self.input.text().strip()
        if not text:
            return
        self.input.clear()
        self._add_message("Bạn", text, from_user=True)
        self.set_busy(True)
        self.chat_submitted.emit(text)

    # ------------------------------------------------------------------
    # Public API – được gọi từ KioskWindow để wire LLM
    # ------------------------------------------------------------------

    def set_llm_status(self, status: str) -> None:
        """Cập nhật label trạng thái LLM ở header."""
        self._llm_status_label.setText(status)

    def receive_response(self, response: str) -> None:
        """Hiển thị response từ LLM và mở khóa input."""
        self.set_busy(False)
        self._add_message("Souta", response)

    def receive_error(self, error_msg: str) -> None:
        """Hiển thị lỗi và mở khóa input."""
        self.set_busy(False)
        self._add_message("Souta", f"⚠️ Lỗi: {error_msg}")

    def set_busy(self, busy: bool) -> None:
        """Khoá/mở input trong khi đang chờ LLM."""
        self.input.setEnabled(not busy)
        self._send_btn.setEnabled(not busy)
        if busy:
            self._send_btn.setText("...")
        else:
            self._send_btn.setText("Gửi")


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
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setSpacing(6)
        for page_id, label in (("home", "Trang chủ"), ("navigation", "Dẫn đường"), ("chat", "Trò chuyện")):
            button = NavButton(label, icons[page_id], page_id)
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
