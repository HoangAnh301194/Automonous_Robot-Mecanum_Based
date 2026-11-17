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
        """Static light-theme mascot, without OpenCV video decode/loop."""
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
        painter.fillRect(self.rect(), QColor("#ffffff"))
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
        # Light theme: no dark overlay on idle mascot.

    def closeEvent(self, event: Any) -> None:
        self.stop()
        super().closeEvent(event)


class HomePage(QWidget):
    """Màn hình chờ chính (Idle / Standby screen) hiển thị biểu cảm Souta Robot qua video nền."""

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
