"""components.py - Shared Reusable UI Components for Souta Kiosk System Status.

Implements:
- StatusPill: semantic status badge
- MetricCard: compact hardware/sensor metric presentation
- ModuleCard: 2x2 grid entry card with touch target >= 44px
- PageHeader: unified minimal header with Back button & title
- SectionCard: themed container for detail sections
- DangerousActionDialog: modal confirmation for risky operations (Motor Jog, ROS cleanup)
"""
from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import (
    QDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.system_status.contracts import HealthStatus
from robot_ui.kiosk.pages.system_status.theme import KioskTheme


def load_pixmap(icon_path: Path | str | None, size: int = 24) -> Optional[QPixmap]:
    """Load a QPixmap scaled smoothly if the file exists."""
    if not icon_path:
        return None
    p = Path(icon_path)
    if not p.is_file():
        return None
    pm = QPixmap(str(p))
    if pm.isNull():
        return None
    return pm.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)


class StatusPill(QLabel):
    """Semantic status pill badge."""

    def __init__(
        self,
        text: str = "UNKNOWN",
        status: HealthStatus | str = HealthStatus.UNKNOWN,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.set_status(text, status)

    def set_status(self, text: str, status: HealthStatus | str) -> None:
        color = KioskTheme.get_status_color(status)
        bg = KioskTheme.get_status_bg(status)
        self.setText(text)
        self.setStyleSheet(f"""
            QLabel {{
                color: {color};
                background-color: {bg};
                border: 1px solid {color}44;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 800;
                letter-spacing: 0.5px;
            }}
        """)


class MetricCard(QFrame):
    """Compact metric card displaying key indicator and status."""

    def __init__(
        self,
        title: str,
        value: str = "--",
        subtitle: str = "",
        status: HealthStatus | str = HealthStatus.UNKNOWN,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("metricCard")
        self.setStyleSheet(f"""
            QFrame#metricCard {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: {KioskTheme.CARD_RADIUS};
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(4)

        self.lbl_title = QLabel(title)
        self.lbl_title.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 11px; font-weight: 600;")
        layout.addWidget(self.lbl_title)

        self.lbl_val = QLabel(value)
        self.lbl_val.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 18px; font-weight: 800;")
        layout.addWidget(self.lbl_val)

        self.lbl_sub = QLabel(subtitle)
        self.lbl_sub.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 10px;")
        layout.addWidget(self.lbl_sub)

    def update_data(self, value: str, subtitle: str = "", status: HealthStatus | str = HealthStatus.HEALTHY) -> None:
        color = KioskTheme.get_status_color(status)
        self.lbl_val.setText(value)
        self.lbl_val.setStyleSheet(f"color: {color}; font-size: 18px; font-weight: 800;")
        if subtitle:
            self.lbl_sub.setText(subtitle)


class ModuleCard(QPushButton):
    """Clickable 2x2 module navigation card on the Overview screen."""

    def __init__(
        self,
        title: str,
        subtitle: str,
        icon_path: Optional[Path | str] = None,
        status_text: str = "READY",
        status: HealthStatus | str = HealthStatus.HEALTHY,
        warning_count: int = 0,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("moduleCard")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(105)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)

        # Top row: Icon + Title + Status Pill
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        self.lbl_icon = QLabel()
        self.lbl_icon.setFixedSize(28, 28)
        pm = load_pixmap(icon_path, size=24)
        if pm:
            self.lbl_icon.setPixmap(pm)
        top_row.addWidget(self.lbl_icon)

        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        self.lbl_title = QLabel(title)
        self.lbl_title.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_PRIMARY}; font-size: 14px; font-weight: 800;")
        self.lbl_sub = QLabel(subtitle)
        self.lbl_sub.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 11px;")
        title_box.addWidget(self.lbl_title)
        title_box.addWidget(self.lbl_sub)
        top_row.addLayout(title_box)
        top_row.addStretch()

        self.pill = StatusPill(status_text, status)
        top_row.addWidget(self.pill)
        layout.addLayout(top_row)

        # Bottom row: Summary / Warning count
        self.lbl_bottom = QLabel("Tất cả thành phần sẵn sàng")
        self.lbl_bottom.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")
        layout.addWidget(self.lbl_bottom)

        self.set_warning_count(warning_count)
        self._apply_style()

    def set_status(self, text: str, status: HealthStatus | str, bottom_hint: str = "") -> None:
        self.pill.set_status(text, status)
        if bottom_hint:
            self.lbl_bottom.setText(bottom_hint)

    def set_warning_count(self, count: int) -> None:
        if count > 0:
            self.lbl_bottom.setText(f"{count} cảnh báo đáng chú ý")
            self.lbl_bottom.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 11px; font-weight: bold;")
        else:
            self.lbl_bottom.setText("Tất cả thành phần sẵn sàng")
            self.lbl_bottom.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_MUTED}; font-size: 11px;")

    def _apply_style(self) -> None:
        self.setStyleSheet(f"""
            QPushButton#moduleCard {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: {KioskTheme.CARD_RADIUS};
                text-align: left;
            }}
            QPushButton#moduleCard:hover {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border-color: {KioskTheme.COLOR_ACCENT}88;
            }}
            QPushButton#moduleCard:pressed {{
                background-color: {KioskTheme.COLOR_SURFACE_ACTIVE};
                border-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)


class PageHeader(QHBoxLayout):
    """Minimal header with Back button, title, and status pill."""

    def __init__(
        self,
        title: str,
        subtitle: str = "Souta Robot · System Monitor",
        back_callback: Optional[Callable[[], None]] = None,
        back_icon_path: Optional[Path | str] = None,
        status_text: str = "HEALTHY",
        status: HealthStatus | str = HealthStatus.HEALTHY,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__()
        self.setSpacing(12)

        # Back button with Touch Target >= 44px
        self.btn_back = QPushButton()
        self.btn_back.setFixedSize(46, 46)
        self.btn_back.setCursor(Qt.PointingHandCursor)
        self.btn_back.setToolTip("Quay lại")
        self.btn_back.setStyleSheet(f"""
            QPushButton {{
                background-color: {KioskTheme.COLOR_SURFACE};
                border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
                border-radius: {KioskTheme.BUTTON_RADIUS};
            }}
            QPushButton:hover {{
                background-color: {KioskTheme.COLOR_SURFACE_ALT};
                border-color: {KioskTheme.COLOR_ACCENT};
            }}
        """)
        pm = load_pixmap(back_icon_path, size=22)
        if pm:
            self.btn_back.setIcon(QIcon(pm))
            self.btn_back.setIconSize(QSize(22, 22))
        else:
            self.btn_back.setText("<")
            self.btn_back.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {KioskTheme.COLOR_ACCENT}; background: {KioskTheme.COLOR_SURFACE}; border-radius: 8px;")
        if back_callback:
            self.btn_back.clicked.connect(back_callback)
        self.addWidget(self.btn_back)

        # Title block
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        self.lbl_title = QLabel(title)
        self.lbl_title.setStyleSheet(f"color: {KioskTheme.COLOR_ACCENT}; font-size: 18px; font-weight: 800; letter-spacing: 0.5px;")
        self.lbl_sub = QLabel(subtitle)
        self.lbl_sub.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 11px;")
        title_box.addWidget(self.lbl_title)
        title_box.addWidget(self.lbl_sub)
        self.addLayout(title_box)
        self.addStretch()

        # Status Pill
        self.status_pill = StatusPill(status_text, status)
        self.addWidget(self.status_pill)

    def set_status(self, text: str, status: HealthStatus | str) -> None:
        self.status_pill.set_status(text, status)

    def setVisible(self, visible: bool) -> None:
        for i in range(self.count()):
            item = self.itemAt(i)
            w = item.widget()
            if w:
                w.setVisible(visible)
            child_layout = item.layout()
            if child_layout:
                for j in range(child_layout.count()):
                    cw = child_layout.itemAt(j).widget()
                    if cw:
                        cw.setVisible(visible)

    def hide(self) -> None:
        self.setVisible(False)

    def show(self) -> None:
        self.setVisible(True)


def create_section_card(title: str, accent_color: str = KioskTheme.COLOR_ACCENT) -> tuple[QFrame, QVBoxLayout]:
    """Helper to create a themed section container with subtle top accent line."""
    card = QFrame()
    card.setObjectName("sectionCard")
    card.setStyleSheet(f"""
        QFrame#sectionCard {{
            background-color: {KioskTheme.COLOR_SURFACE};
            border: 1px solid {KioskTheme.COLOR_CARD_BORDER};
            border-top: 3px solid {accent_color};
            border-radius: {KioskTheme.CARD_RADIUS};
        }}
    """)
    layout = QVBoxLayout(card)
    layout.setContentsMargins(14, 12, 14, 12)
    layout.setSpacing(8)

    lbl_title = QLabel(title)
    lbl_title.setStyleSheet(f"color: {accent_color}; font-size: 12px; font-weight: 800; letter-spacing: 0.5px;")
    layout.addWidget(lbl_title)
    return card, layout


class DangerousActionDialog(QDialog):
    """Modal confirmation dialog before executing risky robot operations."""

    def __init__(
        self,
        title: str,
        message: str,
        action_button_text: str = "XÁC NHẬN THỰC HIỆN",
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(True)
        self.setFixedSize(380, 200)
        self.setStyleSheet(f"""
            QDialog {{
                background-color: {KioskTheme.COLOR_BG};
                border: 2px solid {KioskTheme.COLOR_CRITICAL};
                border-radius: 10px;
            }}
            QLabel {{
                color: {KioskTheme.COLOR_TEXT_PRIMARY};
                font-family: {KioskTheme.FONT_FAMILY};
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        lbl_warn = QLabel(f"<b>CẢNH BÁO AN TOÀN:</b><br>{title}")
        lbl_warn.setStyleSheet(f"color: {KioskTheme.COLOR_WARNING}; font-size: 14px;")
        layout.addWidget(lbl_warn)

        lbl_msg = QLabel(message)
        lbl_msg.setWordWrap(True)
        lbl_msg.setStyleSheet(f"color: {KioskTheme.COLOR_TEXT_SECONDARY}; font-size: 12px;")
        layout.addWidget(lbl_msg)

        btn_row = QHBoxLayout()
        btn_cancel = QPushButton("HỦY BỎ")
        btn_cancel.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_cancel.setStyleSheet(f"background-color: {KioskTheme.COLOR_SURFACE}; color: {KioskTheme.COLOR_TEXT_PRIMARY}; border-radius: 6px; font-weight: bold;")
        btn_cancel.clicked.connect(self.reject)

        btn_confirm = QPushButton(action_button_text)
        btn_confirm.setFixedHeight(KioskTheme.TOUCH_TARGET_MIN)
        btn_confirm.setStyleSheet(f"background-color: {KioskTheme.COLOR_CRITICAL}; color: #1d2938; border-radius: 6px; font-weight: bold;")
        btn_confirm.clicked.connect(self.accept)

        btn_row.addWidget(btn_cancel)
        btn_row.addWidget(btn_confirm)
        layout.addLayout(btn_row)
