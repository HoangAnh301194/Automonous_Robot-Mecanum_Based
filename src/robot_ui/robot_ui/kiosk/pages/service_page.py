"""PTIT Mochi light-theme Service page with native, accessible Qt touch controls."""
from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt, QRect, QSize, pyqtSignal
from PyQt5.QtGui import QColor, QIcon, QPainter, QPixmap
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QSizePolicy, QVBoxLayout, QWidget


class ServiceMascotCanvas(QWidget):
    """Draw the artwork without distortion and align hit regions to its eye buttons."""

    def __init__(self, image_file: Path | None, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._image = QPixmap(str(image_file)) if image_file and image_file.is_file() else QPixmap()
        self.setMinimumHeight(290)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.wait_button = self._make_button("Phục vụ", "wait")
        self.continue_button = self._make_button("Hoàn tất", "continue")
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
        rect = self._art_rect()
        for button, x_fraction in ((self.wait_button, .184), (self.continue_button, .535)):
            button.setGeometry(
                rect.x() + round(rect.width()*x_fraction),
                rect.y() + round(rect.height()*.360),
                max(44, round(rect.width()*.29)),
                max(44, round(rect.height()*.260)),
            )
            button.raise_()
        super().resizeEvent(event)

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        if not self._image.isNull():
            painter.setRenderHint(QPainter.SmoothPixmapTransform)
            painter.drawPixmap(self._art_rect(), self._image)

    def set_selected(self, mode: str | None) -> None:
        for btn, ident, base in (
            (self.wait_button, "wait", "#0764b2"),
            (self.continue_button, "continue", "#198b40"),
        ):
            selected = mode == ident
            btn.setProperty("selected", selected)
            if self._image.isNull():
                btn.setStyleSheet(
                    "QPushButton { background: " + base + "; color: white;"
                    " border: 3px solid " + ("#b32632" if selected else base) +
                    "; border-radius: 16px; font-size: 19px; font-weight: 700; }"
                )
            else:
                btn.setStyleSheet(
                    "QPushButton { background: transparent; border: " +
                    ("3px solid #b32632" if selected else "0px solid transparent") +
                    "; border-radius: 18px; }"
                    "QPushButton:focus { border: 3px solid #b32632; }"
                )


class ServicePage(QWidget):
    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    mode_selected = pyqtSignal(str)

    def __init__(
        self,
        back_icon: Path | None = None, wait_icon: Path | None = None,
        continue_icon: Path | None = None, service_icon: Path | None = None,
        home_icon: Path | None = None, emergency_icon: Path | None = None,
        parent: QWidget | None = None, mascot_image: Path | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("servicePage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._selected_mode: str | None = None

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
                btn.setIcon(QIcon(str(icon)))
                btn.setIconSize(QSize(30, 30))
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
        root.addWidget(header)

        self.mascot_canvas = ServiceMascotCanvas(mascot_image, self)
        self.btn_wait = self.mascot_canvas.wait_button
        self.btn_continue = self.mascot_canvas.continue_button
        self.btn_wait.clicked.connect(lambda: self._select_mode("wait"))
        self.btn_continue.clicked.connect(lambda: self._select_mode("continue"))
        root.addWidget(self.mascot_canvas, 1)
        footer = QLabel("Phục vụ: bắt đầu phục vụ nước    ·    Hoàn tất: kết thúc phục vụ")
        footer.setObjectName("serviceFooter")
        footer.setAlignment(Qt.AlignCenter)
        root.addWidget(footer)

        self.setStyleSheet("""
            QWidget#servicePage { background-color: #ffffff; }
            QFrame#serviceHeader { background: #ffffff; border: 1px solid #dbe4ef; border-radius: 12px; }
            QLabel#serviceHeaderTitle { color: #b32632; font-size: 19px; font-weight: 800; }
            QPushButton#serviceHomeButton, QPushButton#serviceBackButton {
                background: #f2f5fa; border: 1px solid #dbe4ef; border-radius: 10px;
            }
            QPushButton#serviceHomeButton:hover, QPushButton#serviceBackButton:hover {
                background: #e9eff7; border-color: #b32632;
            }
            QPushButton#serviceEmergencyButton {
                background: #fff0f2; border: 1px solid #f4c9d0; border-radius: 10px;
            }
            QLabel#serviceFooter { color: #627185; font-size: 13px; }
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
