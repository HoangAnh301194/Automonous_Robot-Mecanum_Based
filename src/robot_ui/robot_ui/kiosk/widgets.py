"""Reusable UI widgets and controls for the touchscreen kiosk interface."""
from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import QSize, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QIcon
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QPushButton, QWidget

# Re-export page components and helpers for 100% backward compatibility
from robot_ui.kiosk.pages.base import monochrome_icon, strip_emojis
from robot_ui.kiosk.pages.chat_page import (
    ChatPage,
    MicState,
    SpeakerButton,
    VolumeSliderWidget,
)
from robot_ui.kiosk.pages.home_page import HomePage, VideoBackground
from robot_ui.kiosk.pages.navigation_page import MapWidget, NavigationPage


class NavButton(QPushButton):
    def __init__(self, text: str, icon: Path, page_id: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.page_id = page_id
        self.icon_path = icon
        self.setObjectName("kioskNavButton")
        self.setToolTip(text)
        self.setAccessibleName(text)
        self.setCheckable(True)
        self.setIconSize(QSize(39, 39))
        self.setFixedSize(110, 81)

    def refresh_appearance(self, dark_background: bool, active: bool) -> None:
        self.setProperty("dark", dark_background)
        if active:
            color = QColor("#ffb32c" if dark_background else "#d97800")
        else:
            color = QColor("#ffffff" if dark_background else "#26343c")
        # Preserve full-colour icons (Settings, Service, Emergency, Patrol) instead of monochrome tinting.
        if self.page_id in {"settings", "service", "emergency", "patrol"} or self.icon_path.suffix.lower() in {".jpg", ".jpeg"}:
            self.setIcon(QIcon(str(self.icon_path)))
        else:
            self.setIcon(monochrome_icon(self.icon_path, color))
        self.style().unpolish(self)
        self.style().polish(self)

    def update_icon(self, icon_path: Path, tooltip: str = "") -> None:
        self.icon_path = icon_path
        if tooltip:
            self.setToolTip(tooltip)
            self.setAccessibleName(tooltip)
        self.refresh_appearance(self.property("dark") if self.property("dark") is not None else True, self.isChecked())


class BottomNavigation(QFrame):
    page_requested = pyqtSignal(str)

    def __init__(self, icons: dict[str, Path], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("bottomNavigation")
        self._buttons: dict[str, NavButton] = {}
        self._dark_background = True
        layout = QHBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(10)
        labels = {
            "home": "Trang chủ",
            "chat": "Trò chuyện",
            "service": "Dịch vụ",
            "patrol": "Tuần tra",
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

    def update_patrol_state(self, is_patrolling: bool, start_icon: Path, stop_icon: Path) -> None:
        if "patrol" in self._buttons:
            btn = self._buttons["patrol"]
            new_icon = stop_icon if is_patrolling else start_icon
            tooltip = "Tạm dừng tuần tra" if is_patrolling else "Bắt đầu tuần tra"
            btn.update_icon(new_icon, tooltip)

    def set_active(self, page_id: str) -> None:
        for name, button in self._buttons.items():
            active = name == page_id
            button.setChecked(active)
            button.refresh_appearance(self._dark_background, active)

    def set_dark_background(self, enabled: bool) -> None:
        self._dark_background = enabled
        for button in self._buttons.values():
            button.refresh_appearance(enabled, button.isChecked())


__all__ = [
    "BottomNavigation",
    "ChatPage",
    "HomePage",
    "MapWidget",
    "MicState",
    "NavButton",
    "NavigationPage",
    "SpeakerButton",
    "VideoBackground",
    "VolumeSliderWidget",
    "monochrome_icon",
    "strip_emojis",
]
