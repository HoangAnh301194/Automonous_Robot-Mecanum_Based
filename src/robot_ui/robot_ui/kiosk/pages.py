"""Public welcome, touchscreen admin PIN and control overview pages."""
from __future__ import annotations

import hmac
from pathlib import Path
from typing import Any

from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QColor, QIcon, QImageReader, QLinearGradient, QPainter, QPixmap
from PyQt5.QtWidgets import (
    QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QVBoxLayout, QWidget,
)


def picture(path: Path, size: int = 70) -> QLabel:
    label = QLabel()
    label.setFixedSize(size, size)
    label.setAlignment(Qt.AlignCenter)
    pixmap = QPixmap(str(path))
    if not pixmap.isNull():
        label.setPixmap(pixmap.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation))
    return label


class WelcomePage(QWidget):
    chat_requested = pyqtSignal()

    def __init__(self, background: Path, ptit: Path, faculty: Path, chat: Path):
        super().__init__()
        self.setObjectName("welcomePage")
        self.background_file = background
        self._cached_size = None
        self._background = QPixmap()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(28, 20, 28, 100)
        layout.setSpacing(12)

        header = QHBoxLayout()
        header.addWidget(picture(ptit))
        header.addWidget(picture(faculty))
        school = QLabel("HỌC VIỆN CÔNG NGHỆ BƯU CHÍNH VIỄN THÔNG\nKHOA KỸ THUẬT ĐIỆN TỬ 1")
        school.setObjectName("welcomeSchool")
        school.setWordWrap(True)
        header.addWidget(school, 1)
        layout.addLayout(header)
        layout.addStretch(2)

        title = QLabel("XIN CHÀO QUÝ THẦY CÔ")
        title.setObjectName("welcomeHeading")
        title.setAlignment(Qt.AlignCenter)
        title.setWordWrap(True)
        layout.addWidget(title)
        subtitle = QLabel("Dasai Mochi  ·  Trợ lý robot tại Học viện")
        subtitle.setObjectName("welcomeSubtitle")
        subtitle.setAlignment(Qt.AlignCenter)
        layout.addWidget(subtitle)
        layout.addSpacing(14)

        row = QHBoxLayout()
        row.addStretch()
        btn = QPushButton("  Trò chuyện với Dasai Mochi")
        btn.setObjectName("welcomeChatButton")
        btn.setAccessibleName("Mở trò chuyện")
        btn.setMinimumSize(320, 78)
        btn.setMaximumWidth(480)
        btn.setIcon(QIcon(str(chat)))
        btn.setIconSize(QSize(46, 46))
        btn.clicked.connect(self.chat_requested)
        row.addWidget(btn)
        row.addStretch()
        layout.addLayout(row)
        layout.addStretch(3)
        self.setStyleSheet("""
            QLabel#welcomeSchool { color: white; font-size: 14px; font-weight: 700; }
            QLabel#welcomeHeading { color: white; font-size: 32px; font-weight: 800; }
            QLabel#welcomeSubtitle { color: #eef2f4; font-size: 17px; }
            QPushButton#welcomeChatButton {
                color: white; background: rgba(12, 25, 36, 210);
                border: 2px solid #f2bf6e; border-radius: 17px;
                font-size: 17px; font-weight: 700; padding: 8px;
            }
            QPushButton#welcomeChatButton:pressed { background: #3b4b59; }
        """)

    def _read_background(self):
        size = (max(1, self.width()), max(1, self.height()))
        if size == self._cached_size:
            return
        self._cached_size = size
        reader = QImageReader(str(self.background_file))
        reader.setAutoTransform(True)
        original = reader.size()
        if original.isValid() and original.width() and original.height():
            scale = max(size[0] / original.width(), size[1] / original.height())
            reader.setScaledSize(QSize(max(1, round(original.width() * scale)),
                                       max(1, round(original.height() * scale))))
        image = reader.read()
        self._background = QPixmap.fromImage(image) if not image.isNull() else QPixmap()

    def paintEvent(self, event: Any) -> None:
        self._read_background()
        p = QPainter(self)
        p.fillRect(self.rect(), QColor("#142130"))
        if not self._background.isNull():
            p.drawPixmap((self.width() - self._background.width()) // 2,
                         (self.height() - self._background.height()) // 2,
                         self._background)
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(3, 10, 17, 190))
        gradient.setColorAt(0.6, QColor(3, 10, 17, 100))
        gradient.setColorAt(1, QColor(3, 10, 17, 200))
        p.fillRect(self.rect(), gradient)


class AdminPinDialog(QDialog):
    """Touch keypad. Default PIN '1' is for prototyping, not deployed security."""

    def __init__(self, expected: str, parent: QWidget | None = None):
        super().__init__(parent)
        self.expected = expected
        self.attempts = 0
        self.setWindowTitle("Đăng nhập quản trị")
        self.setMinimumWidth(340)
        self.setObjectName("adminPinDialog")
        root = QVBoxLayout(self)
        root.setContentsMargins(24, 20, 24, 20)
        title = QLabel("QUẢN TRỊ ROBOT")
        title.setObjectName("adminPinTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)
        root.addWidget(QLabel("Nhập mã PIN để truy cập"))

        self.entry = QLineEdit()
        self.entry.setEchoMode(QLineEdit.Password)
        self.entry.setInputMethodHints(Qt.ImhDigitsOnly | Qt.ImhSensitiveData)
        self.entry.setMaxLength(12)
        self.entry.setMinimumHeight(48)
        self.entry.setAlignment(Qt.AlignCenter)
        self.entry.returnPressed.connect(self._check)
        root.addWidget(self.entry)
        self.error = QLabel("")
        self.error.setObjectName("adminPinError")
        self.error.setMinimumHeight(24)
        root.addWidget(self.error)

        grid = QGridLayout()
        for row, digits in enumerate((("1", "2", "3"), ("4", "5", "6"), ("7", "8", "9"))):
            for col, digit in enumerate(digits):
                button = QPushButton(digit)
                button.setMinimumHeight(51)
                button.clicked.connect(lambda checked=False, d=digit: self.entry.insert(d))
                grid.addWidget(button, row, col)
        back = QPushButton("Xóa")
        back.clicked.connect(self.entry.backspace)
        grid.addWidget(back, 3, 0)
        zero = QPushButton("0")
        zero.clicked.connect(lambda: self.entry.insert("0"))
        grid.addWidget(zero, 3, 1)
        self.confirm = QPushButton("OK")
        self.confirm.setObjectName("adminPinConfirm")
        self.confirm.clicked.connect(self._check)
        grid.addWidget(self.confirm, 3, 2)
        root.addLayout(grid)
        cancel = QPushButton("Hủy")
        cancel.setMinimumHeight(43)
        cancel.clicked.connect(self.reject)
        root.addWidget(cancel)
        self.setStyleSheet("""
            QDialog#adminPinDialog { background: #151d27; color: white; }
            QLabel { color: #e3e7ed; font-size: 14px; }
            QLabel#adminPinTitle { color: #f6c16a; font-size: 20px; font-weight: 700; }
            QLabel#adminPinError { color: #ff8585; }
            QLineEdit { color: white; background: #293849; border-radius: 8px; font-size: 22px; }
            QPushButton { color: white; background: #344454; border: none;
                          border-radius: 9px; font-size: 17px; font-weight: 700; }
            QPushButton#adminPinConfirm { color: #161b23; background: #f6c16a; }
        """)

    def _check(self):
        if hmac.compare_digest(self.entry.text(), self.expected):
            self.entry.clear()
            self.accept()
        else:
            self.attempts += 1
            self.error.setText("Mã PIN không đúng")
            self.entry.clear()
            if self.attempts >= 5:
                self.error.setText("Đã nhập sai 5 lần; đóng hộp thoại để thử lại.")
                self.entry.setEnabled(False)
                self.confirm.setEnabled(False)


class AdminPage(QWidget):
    navigation_requested = pyqtSignal()
    logout_requested = pyqtSignal()

    def __init__(self, nav_icon: Path, slam_icon: Path):
        super().__init__()
        self.setObjectName("adminPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        root = QVBoxLayout(self)
        root.setContentsMargins(28, 22, 28, 28)
        root.setSpacing(13)
        header = QHBoxLayout()
        title = QLabel("QUẢN TRỊ ROBOT")
        title.setObjectName("adminTitle")
        header.addWidget(title)
        header.addStretch()
        logout = QPushButton("Đăng xuất")
        logout.setObjectName("adminLogout")
        logout.setMinimumSize(115, 50)
        logout.clicked.connect(self.logout_requested)
        header.addWidget(logout)
        root.addLayout(header)
        root.addWidget(QLabel("Trạng thái hệ thống và công cụ điều hướng"))
        self.connection = self._add_status(root, "Kết nối ROS 2: Đang chờ")
        self.robot = self._add_status(root, "Trạng thái robot: Chưa có dữ liệu")
        self.nav = self._add_status(root, "Điều hướng: Chưa có dữ liệu")
        self.battery = self._add_status(root, "Pin: --")
        root.addSpacing(12)
        actions = QHBoxLayout()
        actions.setSpacing(16)
        nav_btn = QPushButton("  Bản đồ & Nav2")
        nav_btn.setObjectName("adminNavButton")
        nav_btn.setIcon(QIcon(str(nav_icon)))
        nav_btn.setIconSize(QSize(60, 60))
        nav_btn.setMinimumHeight(92)
        nav_btn.clicked.connect(self.navigation_requested)
        actions.addWidget(nav_btn)
        slam_label = QLabel()
        slam_label.setPixmap(QPixmap(str(slam_icon)).scaled(58, 58, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        actions.addWidget(slam_label)
        actions.addWidget(QLabel("SLAM\nTích hợp ở giai đoạn tiếp theo"))
        root.addLayout(actions)
        root.addStretch()
        root.addWidget(QLabel("Nhiệm vụ · AI/Camera · Developer: sẽ tích hợp sau."))
        self.setStyleSheet("""
            QWidget#adminPage { background: #141c25; }
            QLabel { color: #dce2e7; font-size: 14px; }
            QLabel#adminTitle { color: #f6c16a; font-size: 25px; font-weight: 800; }
            QLabel#adminStatus { background: #283441; color: white;
                                 border-radius: 9px; padding: 10px 13px; }
            QPushButton#adminLogout { background: #344250; color: white;
                                      border-radius: 10px; font-weight: 700; }
            QPushButton#adminNavButton { color: white; background: #344250;
                border: 2px solid #f6c16a; border-radius: 14px;
                font-size: 18px; font-weight: 700; }
        """)

    @staticmethod
    def _add_status(layout: QVBoxLayout, value: str) -> QLabel:
        label = QLabel(value)
        label.setObjectName("adminStatus")
        label.setMinimumHeight(43)
        layout.addWidget(label)
        return label

    def set_connection(self, connected: bool):
        self.connection.setText("Kết nối ROS 2: Có phản hồi" if connected
                                else "Kết nối ROS 2: Chưa kết nối")

    def set_robot_status(self, status: dict[str, Any]):
        self.robot.setText(f"Trạng thái robot: {status.get('robot_state', 'Chưa có dữ liệu')}")
        battery = status.get("battery_percent")
        self.battery.setText(f"Pin: {float(battery):.0f}%" if isinstance(battery, (int, float)) else "Pin: --")

    def set_navigation_status(self, status: dict[str, Any]):
        self.nav.setText(f"Điều hướng: {status.get('status', 'Chưa có dữ liệu')}")
