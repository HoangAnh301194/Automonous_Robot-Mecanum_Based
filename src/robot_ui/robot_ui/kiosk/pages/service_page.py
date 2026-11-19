"""White-backed PTIT Mochi Service page inside the dark-mode Kiosk."""
from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt, QRect, QRectF, QSize, QEasingCurve, QVariantAnimation, pyqtSignal
from PyQt5.QtGui import QColor, QIcon, QPainter, QPainterPath, QPixmap
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget


class ServiceMascotCanvas(QWidget):
    """Draw the artwork without distortion and align hit regions to its eye buttons."""

    def __init__(self, image_file: Path | None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._image = QPixmap(str(image_file)) if image_file and image_file.is_file() else QPixmap()
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.setMinimumHeight(290)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.wait_button = self._make_button("Phục vụ", "wait")
        self.continue_button = self._make_button("Hoàn tất", "continue")
        self._selected_mode: str | None = None
        self._press_scale = 1.0
        self._zoom_animation = QVariantAnimation(self)
        self._zoom_animation.setDuration(120)
        self._zoom_animation.setEasingCurve(QEasingCurve.OutCubic)
        self._zoom_animation.valueChanged.connect(self._on_zoom_value)
        self.set_selected(None)

    def _make_button(self, label: str, mode: str) -> QPushButton:
        btn = QPushButton(self)
        btn.setObjectName("serviceMascotButton")
        btn.setProperty("mode", mode)
        btn.setAccessibleName(label)
        btn.setToolTip(label)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFocusPolicy(Qt.StrongFocus)
        if self._image.isNull():
            btn.setText(label)
        return btn

    def _art_rect(self) -> QRect:
        side = max(1, min(self.width(), self.height()))
        return QRect((self.width()-side)//2, (self.height()-side)//2, side, side)

    def resizeEvent(self, event) -> None:
        # Use the same tight bounds for clickable overlays and press animation.
        for mode, button in (("wait", self.wait_button), ("continue", self.continue_button)):
            button.setGeometry(self._button_rect(mode).toAlignedRect())
            button.raise_()
        super().resizeEvent(event)

    def mousePressEvent(self, event) -> None:
        pos = event.pos()
        art = self._art_rect()
        if art.contains(pos):
            # Nhận diện chạm toàn vùng: nửa trái bức tranh là Phục vụ, nửa phải là Hoàn tất
            if pos.x() < art.center().x():
                self.wait_button.click()
            else:
                self.continue_button.click()
        super().mousePressEvent(event)

    def _art_crop_rect(self, mode: str) -> QRectF:
        """Visual crop bounds matching the eye artwork for zoom animation."""
        art = self._art_rect()
        center_x = .329 if mode == "wait" else .680
        center_y = .490
        width = .245
        height = .185
        return QRectF(
            art.x() + art.width() * (center_x - width / 2),
            art.y() + art.height() * (center_y - height / 2),
            art.width() * width,
            art.height() * height,
        )

    def _button_rect(self, mode: str) -> QRectF:
        """Generous hit area around each button, ensuring effortless touch interaction."""
        art = self._art_rect()
        center_x = .329 if mode == "wait" else .680
        center_y = .490
        # Mở rộng vùng bấm lên 0.360 x 0.280 (tăng hơn 50% diện tích bấm)
        width = .360
        height = .280
        return QRectF(
            art.x() + art.width() * (center_x - width / 2),
            art.y() + art.height() * (center_y - height / 2),
            art.width() * width,
            art.height() * height,
        )

    def _on_zoom_value(self, value) -> None:
        self._press_scale = float(value)
        self.update()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        # The image is white-backed: match it exactly to remove dark margins.
        painter.fillRect(self.rect(), QColor("#FFFFFF"))
        if self._image.isNull():
            return
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        painter.drawPixmap(self._art_rect(), self._image)
        # The buttons are part of the source JPG. Zoom the selected image crop,
        # not merely the transparent hitbox on top of it.
        if self._selected_mode and self._press_scale > 1.001:
            target = self._art_crop_rect(self._selected_mode)
            art = self._art_rect()
            source = QRectF(
                (target.x() - art.x()) / art.width() * self._image.width(),
                (target.y() - art.y()) / art.height() * self._image.height(),
                target.width() / art.width() * self._image.width(),
                target.height() / art.height() * self._image.height(),
            )
            scale = self._press_scale
            destination = QRectF(
                target.center().x() - target.width() * scale / 2,
                target.center().y() - target.height() * scale / 2,
                target.width() * scale, target.height() * scale,
            )
            # Keep the zoomed artwork strictly inside its own button. The
            # white background of the source JPEG must not cover the face.
            clip = QPainterPath()
            clip.addRoundedRect(target, target.height() * .17, target.height() * .17)
            painter.save()
            painter.setClipPath(clip)
            painter.drawPixmap(destination, self._image, source)
            painter.restore()

    def set_selected(self, mode: str | None) -> None:
        if mode not in {"wait", "continue", None}:
            return
        self._selected_mode = mode
        self._zoom_animation.stop()
        if mode is None:
            self._press_scale = 1.0
        else:
            self._press_scale = 1.0
            self._zoom_animation.setStartValue(1.0)
            self._zoom_animation.setEndValue(1.055)
            self._zoom_animation.start()
        for button, button_mode, fill in (
            (self.wait_button, "wait", "#0865b3"),
            (self.continue_button, "continue", "#168a40"),
        ):
            button.setProperty("selected", mode == button_mode)
            if self._image.isNull():
                button.setStyleSheet(
                    "QPushButton { background: " + fill +
                    "; color: white; border: none; border-radius: 14px;"
                    " font-size: " + ("20px" if mode == button_mode else "18px") +
                    "; font-weight: 700; }"
                )
            else:
                button.setStyleSheet(
                    "QPushButton { background: transparent; border: none; }"
                    "QPushButton:focus { background: transparent; border: none; }"
                )
        self.update()


class ServicePage(QWidget):
    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    patrol_toggle_requested = pyqtSignal()
    mode_selected = pyqtSignal(str)

    def __init__(
        self,
        back_icon: Path | None = None, wait_icon: Path | None = None,
        continue_icon: Path | None = None, service_icon: Path | None = None,
        home_icon: Path | None = None, emergency_icon: Path | None = None,
        parent: QWidget | None = None, mascot_image: Path | None = None,
        start_icon: Path | None = None, stop_icon: Path | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("servicePage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._selected_mode: str | None = None
        self.start_icon = start_icon
        self.stop_icon = stop_icon

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 12, 20, 14)
        root.setSpacing(8)
        header = QFrame(self)
        header.setObjectName("serviceHeader")
        header.setFixedHeight(62)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 7, 12, 7)
        header_layout.setSpacing(9)

        def action_button(obj: str, label: str, icon: Path | None) -> QPushButton:
            btn = QPushButton()
            btn.setObjectName(obj)
            btn.setFixedSize(46, 46)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setAccessibleName(label)
            btn.setToolTip(label)
            if icon and icon.is_file():
                glyph = QPixmap(str(icon))
                if not glyph.isNull():
                    if obj in {"serviceHomeButton", "serviceBackButton"}:
                        # Tint the alpha shape black, retaining transparent edges.
                        black = QPixmap(glyph.size())
                        black.fill(Qt.transparent)
                        painter = QPainter(black)
                        painter.drawPixmap(0, 0, glyph)
                        painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
                        painter.fillRect(black.rect(), QColor("#000000"))
                        painter.end()
                        glyph = black
                    btn.setIcon(QIcon(glyph))
                    btn.setIconSize(QSize(30, 30))
                else:
                    btn.setText("‹" if label == "Quay lại" else ("!" if label == "Dừng khẩn cấp" else "⌂"))
            else:
                btn.setText("‹" if label == "Quay lại" else ("!" if label == "Dừng khẩn cấp" else "⌂"))
            return btn

        self.home_button = action_button("serviceHomeButton", "Trang chủ", home_icon)
        self.home_button.clicked.connect(self.home_requested.emit)
        header_layout.addWidget(self.home_button)
        self.back_button = action_button("serviceBackButton", "Quay lại", back_icon)
        self.back_button.clicked.connect(self.back_requested.emit)
        header_layout.addWidget(self.back_button)
        heading = QLabel("DỊCH VỤ PHỤC VỤ NƯỚC")
        heading.setObjectName("serviceHeaderTitle")
        header_layout.addWidget(heading)
        header_layout.addStretch(1)
        self.emergency_button = action_button("serviceEmergencyButton", "Dừng khẩn cấp", emergency_icon)
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        header_layout.addWidget(self.emergency_button)
        self.patrol_button = action_button("servicePatrolButton", "Bắt đầu tuần tra", start_icon)
        self.patrol_button.clicked.connect(self.patrol_toggle_requested.emit)
        header_layout.addWidget(self.patrol_button)
        root.addWidget(header)

        self.mascot_canvas = ServiceMascotCanvas(mascot_image, self)
        self.btn_wait = self.mascot_canvas.wait_button
        self.btn_continue = self.mascot_canvas.continue_button
        self.btn_wait.clicked.connect(lambda: self._select_mode("wait"))
        self.btn_continue.clicked.connect(lambda: self._select_mode("continue"))
        root.addWidget(self.mascot_canvas, 1)

        # Service is a deliberate light-mode exception to the dark Kiosk.
        # All surrounding surfaces match the white-backed mascot image.
        self.setStyleSheet("""
            QWidget#servicePage { background-color: #FFFFFF; }
            QFrame#serviceHeader {
                background-color: #FFFFFF; border: 1px solid #e1e6ee;
                border-radius: 12px;
            }
            QLabel#serviceHeaderTitle {
                color: #b32632; font-size: 19px; font-weight: 800;
            }
            QPushButton#serviceHomeButton, QPushButton#serviceBackButton {
                color: #000000; background: #f2f5fa;
                border: 1px solid #dbe4ef; border-radius: 10px;
            }
            QPushButton#serviceHomeButton:hover, QPushButton#serviceBackButton:hover {
                background: #e9eff7; border-color: #b32632;
            }
            QPushButton#serviceEmergencyButton {
                background: #fff0f2; border: 1px solid #f4c9d0;
                border-radius: 10px;
            }
        """)

    def _select_mode(self, mode: str, emit_signal: bool = True) -> None:
        if mode not in {"wait", "continue"}:
            return
        self._selected_mode = mode
        self.mascot_canvas.set_selected(mode)
        if emit_signal:
            self.mode_selected.emit(mode)

    def get_selected_mode(self) -> str | None:
        return self._selected_mode

    def select_mode_programmatically(self, mode: str, emit_signal: bool = True) -> None:
        self._select_mode(mode, emit_signal=emit_signal)

    def reset_selection(self) -> None:
        self._selected_mode = None
        self.mascot_canvas.set_selected(None)

    def update_patrol_state(self, is_patrolling: bool) -> None:
        if not hasattr(self, "patrol_button"):
            return
        icon_path = self.stop_icon if is_patrolling else self.start_icon
        tooltip = "Tạm dừng tuần tra" if is_patrolling else "Bắt đầu tuần tra"
        self.patrol_button.setToolTip(tooltip)
        self.patrol_button.setAccessibleName(tooltip)
        if icon_path and Path(icon_path).is_file():
            self.patrol_button.setIcon(QIcon(str(icon_path)))
            self.patrol_button.setText("")
        else:
            self.patrol_button.setText("⏸" if is_patrolling else "▶")
            self.patrol_button.setStyleSheet(
                "color: #f59e0b; font-size: 16px; font-weight: 700;" if is_patrolling
                else "color: #10b981; font-size: 16px; font-weight: 700;"
            )
