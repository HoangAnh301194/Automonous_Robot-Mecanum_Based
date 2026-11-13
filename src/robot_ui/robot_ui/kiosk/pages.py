"""Public welcome, touchscreen admin PIN and control overview pages."""
from __future__ import annotations

import hmac
from pathlib import Path
from typing import Any

from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QIcon, QImageReader, QLinearGradient, QPainter, QPixmap
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


def check_wifi_connected() -> bool:
    """Check wireless network connectivity from sysfs and procfs."""
    try:
        sys_net = Path("/sys/class/net")
        if sys_net.is_dir():
            for iface in sys_net.iterdir():
                if iface.name.startswith(("wl", "ra", "wlan")):
                    operstate = (iface / "operstate").read_text().strip()
                    carrier_file = iface / "carrier"
                    carrier = carrier_file.read_text().strip() if carrier_file.exists() else ""
                    if operstate == "up" or carrier == "1":
                        return True
        proc_wireless = Path("/proc/net/wireless")
        if proc_wireless.exists():
            lines = [l for l in proc_wireless.read_text().strip().splitlines() if ":" in l]
            if lines:
                return True
    except Exception:
        pass
    return False


class WelcomePage(QWidget):
    chat_requested = pyqtSignal()
    service_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()

    def __init__(
        self,
        background: Path,
        ptit: Path,
        faculty: Path,
        service_icon: Path | None = None,
        emergency_icon: Path | None = None,
        chat: Path | None = None,
    ):
        super().__init__()
        self.setObjectName("welcomePage")
        self.background_file = background
        self._cached_size = None
        self._background = QPixmap()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(28, 20, 28, 100)
        layout.setSpacing(12)

        header = QHBoxLayout()
        header.addWidget(picture(ptit), 0, Qt.AlignTop)

        school_box = QVBoxLayout()
        school_box.setContentsMargins(8, 6, 8, 0)
        school_box.setSpacing(3)
        school_ptit = QLabel("HỌC VIỆN CÔNG NGHỆ BƯU CHÍNH VIỄN THÔNG")
        school_ptit.setObjectName("welcomeSchoolPtit")
        school_ptit.setAlignment(Qt.AlignCenter)
        school_ptit.setWordWrap(True)
        school_fee1 = QLabel("KHOA KỸ THUẬT ĐIỆN TỬ 1")
        school_fee1.setObjectName("welcomeSchoolFee1")
        school_fee1.setAlignment(Qt.AlignCenter)
        school_fee1.setWordWrap(True)
        school_box.addWidget(school_ptit)
        school_box.addWidget(school_fee1)
        header.addLayout(school_box, 1)

        header.addWidget(picture(faculty), 0, Qt.AlignTop)
        layout.addLayout(header)
        layout.addStretch(1)

        title = QLabel("Chào mừng ngày Nhà giáo Việt Nam 20-11")
        title.setObjectName("welcomeHeading")
        title.setAlignment(Qt.AlignCenter)
        title.setWordWrap(True)
        layout.addWidget(title)
        subtitle = QLabel("Kính chào quý Thầy Cô Khoa Kỹ thuật Điện tử 1")
        subtitle.setObjectName("welcomeSubtitle")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setWordWrap(True)
        layout.addWidget(subtitle)
        layout.addStretch(3)

        self.setStyleSheet("""
            QLabel#welcomeSchoolPtit { color: #ff4545; font-size: 15px; font-weight: 800; }
            QLabel#welcomeSchoolFee1 { color: #48a9ff; font-size: 14px; font-weight: 800; }
            QLabel#welcomeHeading { color: white; font-size: 29px; font-weight: 800; }
            QLabel#welcomeSubtitle { color: #eef2f4; font-size: 19px; font-weight: 600; }
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
        p.fillRect(self.rect(), QBrush(gradient))


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
        root.setSpacing(10)
        title = QLabel("QUẢN TRỊ ROBOT")
        title.setObjectName("adminPinTitle")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)
        prompt = QLabel("Nhập mã PIN để truy cập")
        prompt.setAlignment(Qt.AlignCenter)
        root.addWidget(prompt)

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
        self.error.setAlignment(Qt.AlignCenter)
        root.addWidget(self.error)

        grid = QGridLayout()
        grid.setSpacing(10)
        grid.setAlignment(Qt.AlignCenter)
        btn_w, btn_h = 72, 60
        for row, digits in enumerate((("1", "2", "3"), ("4", "5", "6"), ("7", "8", "9"))):
            for col, digit in enumerate(digits):
                button = QPushButton(digit)
                button.setFixedSize(btn_w, btn_h)
                button.clicked.connect(lambda checked=False, d=digit: self.entry.insert(d))
                grid.addWidget(button, row, col)
        back = QPushButton("Xóa")
        back.setFixedSize(btn_w, btn_h)
        back.clicked.connect(self.entry.backspace)
        grid.addWidget(back, 3, 0)
        zero = QPushButton("0")
        zero.setFixedSize(btn_w, btn_h)
        zero.clicked.connect(lambda: self.entry.insert("0"))
        grid.addWidget(zero, 3, 1)
        self.confirm = QPushButton("OK")
        self.confirm.setObjectName("adminPinConfirm")
        self.confirm.setFixedSize(btn_w, btn_h)
        self.confirm.clicked.connect(self._check)
        grid.addWidget(self.confirm, 3, 2)
        root.addLayout(grid)
        cancel = QPushButton("Hủy")
        cancel.setMinimumHeight(44)
        cancel.clicked.connect(self.reject)
        root.addWidget(cancel)
        self.setStyleSheet("""
            QDialog#adminPinDialog { background: #151d27; color: white; }
            QLabel { color: #e3e7ed; font-size: 14px; }
            QLabel#adminPinTitle { color: #f6c16a; font-size: 20px; font-weight: 700; }
            QLabel#adminPinError { color: #ff8585; }
            QLineEdit { color: white; background: #293849; border-radius: 8px; font-size: 22px; }
            QPushButton { color: white; background: #344454; border: none;
                          border-radius: 10px; font-size: 18px; font-weight: 700; }
            QPushButton#adminPinConfirm { color: #161b23; background: #f6c16a; font-size: 18px; }
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
    back_requested = pyqtSignal()
    home_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    detection_debug_requested = pyqtSignal()
    slam_requested = pyqtSignal()
    start_nav_requested = pyqtSignal()
    stop_nav_requested = pyqtSignal()
    system_status_requested = pyqtSignal()

    def __init__(
        self,
        nav_icon: Path,
        slam_icon: Path,
        detection_icon: Path | None = None,
        start_nav_icon: Path | None = None,
        stop_nav_icon: Path | None = None,
        status_icon: Path | None = None,
        wifi_connected_icon: Path | None = None,
        wifi_disconnected_icon: Path | None = None,
        battery_icons: dict[str, Path] | None = None,
        back_icon: Path | None = None,
        home_icon: Path | None = None,
        emergency_icon: Path | None = None,
    ):
        super().__init__()
        self.setObjectName("adminPage")
        self.setAttribute(Qt.WA_StyledBackground, True)

        self.wifi_connected_icon = wifi_connected_icon
        self.wifi_disconnected_icon = wifi_disconnected_icon
        self.battery_icons = battery_icons or {}
        self.back_icon = back_icon
        self.home_icon = home_icon
        self.emergency_icon = emergency_icon

        # Fallback status labels for backwards compatibility
        self.connection = QLabel("Kết nối ROS 2: Đang chờ")
        self.robot = QLabel("Trạng thái robot: Chưa có dữ liệu")
        self.nav = QLabel("Điều hướng: Chưa có dữ liệu")
        self.battery = QLabel("Pin: --")

        root = QVBoxLayout(self)
        root.setContentsMargins(28, 18, 28, 24)
        root.setSpacing(14)

        # Header: Control cluster (Home, E-Stop, Back) + Title on Left, Status Bar on Right
        header = QHBoxLayout()
        header.setSpacing(10)

        # 1. Home button in top-left
        self.home_button = QPushButton()
        self.home_button.setObjectName("adminHomeButton")
        self.home_button.setCursor(Qt.PointingHandCursor)
        self.home_button.setFixedSize(46, 46)
        self.home_button.setToolTip("Về màn hình chính")
        if home_icon and Path(home_icon).is_file():
            self.home_button.setIcon(QIcon(str(home_icon)))
            self.home_button.setIconSize(QSize(28, 28))
        else:
            self.home_button.setText("🏠")
            self.home_button.setStyleSheet("font-size: 22px;")
        self.home_button.clicked.connect(self.home_requested.emit)
        header.addWidget(self.home_button)

        # 2. Emergency stop button in top-left
        self.emergency_button = QPushButton()
        self.emergency_button.setObjectName("adminEmergencyButton")
        self.emergency_button.setCursor(Qt.PointingHandCursor)
        self.emergency_button.setFixedSize(46, 46)
        self.emergency_button.setToolTip("DỪNG KHẨN CẤP")
        if emergency_icon and Path(emergency_icon).is_file():
            self.emergency_button.setIcon(QIcon(str(emergency_icon)))
            self.emergency_button.setIconSize(QSize(36, 36))
        else:
            self.emergency_button.setText("🛑")
            self.emergency_button.setStyleSheet("font-size: 24px;")
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        header.addWidget(self.emergency_button)

        # 3. Back button in top-left
        self.back_button = QPushButton()
        self.back_button.setObjectName("adminBackButton")
        self.back_button.setCursor(Qt.PointingHandCursor)
        self.back_button.setFixedSize(46, 46)
        self.back_button.setToolTip("Quay lại")
        if back_icon and Path(back_icon).is_file():
            self.back_button.setIcon(QIcon(str(back_icon)))
            self.back_button.setIconSize(QSize(32, 32))
        else:
            self.back_button.setText("‹")
            self.back_button.setStyleSheet("color: #ffb32c; font-size: 26px; font-weight: 700;")
        self.back_button.clicked.connect(self.back_requested)
        self.back_button.clicked.connect(self.logout_requested)
        header.addWidget(self.back_button)

        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        title = QLabel("QUẢN TRỊ ROBOT")
        title.setObjectName("adminTitle")
        subtitle = QLabel("Trung tâm điều khiển và cấu hình chuyên sâu")
        subtitle.setObjectName("adminSubtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch()

        # Status Bar on Top Right (Wifi, Battery)
        status_bar = QHBoxLayout()
        status_bar.setSpacing(12)

        # Wifi Badge
        self.wifi_widget = QWidget()
        self.wifi_widget.setObjectName("adminHeaderBadge")
        wifi_layout = QHBoxLayout(self.wifi_widget)
        wifi_layout.setContentsMargins(10, 5, 12, 5)
        wifi_layout.setSpacing(8)
        self.wifi_icon_label = QLabel()
        self.wifi_icon_label.setFixedSize(28, 28)
        self.wifi_icon_label.setAlignment(Qt.AlignCenter)
        self.wifi_text_label = QLabel("Wifi")
        self.wifi_text_label.setObjectName("adminBadgeText")
        wifi_layout.addWidget(self.wifi_icon_label)
        wifi_layout.addWidget(self.wifi_text_label)
        status_bar.addWidget(self.wifi_widget)

        # Battery Badge
        self.battery_widget = QWidget()
        self.battery_widget.setObjectName("adminHeaderBadge")
        battery_layout = QHBoxLayout(self.battery_widget)
        battery_layout.setContentsMargins(10, 5, 12, 5)
        battery_layout.setSpacing(8)
        self.battery_icon_label = QLabel()
        self.battery_icon_label.setFixedSize(28, 28)
        self.battery_icon_label.setAlignment(Qt.AlignCenter)
        self.battery_text_label = QLabel("--%")
        self.battery_text_label.setObjectName("adminBadgeText")
        battery_layout.addWidget(self.battery_icon_label)
        battery_layout.addWidget(self.battery_text_label)
        status_bar.addWidget(self.battery_widget)

        header.addLayout(status_bar)
        root.addLayout(header)

        # Status Banner
        self.status_banner = QLabel("Trạng thái: Sẵn sàng")
        self.status_banner.setObjectName("adminStatusBanner")
        self.status_banner.setMinimumHeight(36)
        root.addWidget(self.status_banner)

        # Grid of 6 launcher buttons (Icon on top, Name below icon)
        grid = QGridLayout()
        grid.setSpacing(24)
        grid.setAlignment(Qt.AlignCenter)

        def make_launcher_icon(
            title_text: str,
            icon_file: Path | None,
            obj_name: str,
            callback,
        ) -> QPushButton:
            btn = QPushButton()
            btn.setObjectName(obj_name)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedSize(168, 135)
            layout = QVBoxLayout(btn)
            layout.setContentsMargins(10, 14, 10, 12)
            layout.setSpacing(8)
            layout.setAlignment(Qt.AlignCenter)

            icon_lbl = QLabel()
            icon_lbl.setFixedSize(68, 68)
            icon_lbl.setAlignment(Qt.AlignCenter)
            if icon_file and Path(icon_file).is_file():
                pm = QPixmap(str(icon_file))
                if not pm.isNull():
                    icon_lbl.setPixmap(pm.scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            layout.addWidget(icon_lbl, 0, Qt.AlignCenter)

            t = QLabel(title_text)
            t.setObjectName("launcherTitle")
            t.setAlignment(Qt.AlignCenter)
            t.setWordWrap(True)
            layout.addWidget(t, 0, Qt.AlignCenter)

            btn.clicked.connect(callback)
            return btn

        # 1. Detection Debug
        self.btn_detection = make_launcher_icon(
            "Detection Debug",
            detection_icon,
            "adminDetectionButton",
            self.detection_debug_requested,
        )
        grid.addWidget(self.btn_detection, 0, 0)

        # 2. SLAM Config
        self.btn_slam = make_launcher_icon(
            "SLAM Config",
            slam_icon,
            "adminSlamButton",
            self.slam_requested,
        )
        grid.addWidget(self.btn_slam, 0, 1)

        # 3. Navigation Config
        self.btn_nav_config = make_launcher_icon(
            "Navigation Config",
            nav_icon,
            "adminNavButton",
            self.navigation_requested,
        )
        grid.addWidget(self.btn_nav_config, 0, 2)

        # 4. Start Navigation
        self.btn_start_nav = make_launcher_icon(
            "Bắt đầu Điều hướng",
            start_nav_icon,
            "adminStartNavButton",
            self.start_nav_requested,
        )
        grid.addWidget(self.btn_start_nav, 1, 0)

        # 5. Stop Navigation
        self.btn_stop_nav = make_launcher_icon(
            "Dừng Điều hướng",
            stop_nav_icon,
            "adminStopNavButton",
            self.stop_nav_requested,
        )
        grid.addWidget(self.btn_stop_nav, 1, 1)

        # 6. System Status
        self.btn_status = make_launcher_icon(
            "System Status",
            status_icon,
            "adminSystemStatusButton",
            self.system_status_requested,
        )
        grid.addWidget(self.btn_status, 1, 2)

        root.addLayout(grid)
        root.addStretch()

        self.setStyleSheet("""
            QWidget#adminPage { background: #000000; }
            QLabel { color: #dce2e7; }
            QLabel#adminTitle { color: #f6c16a; font-size: 26px; font-weight: 800; }
            QLabel#adminSubtitle { color: #8896a6; font-size: 13px; font-weight: 500; }
            QWidget#adminHeaderBadge {
                background: #0f1318;
                border: 1px solid #202732;
                border-radius: 10px;
            }
            QLabel#adminBadgeText { color: #f0f4f8; font-size: 14px; font-weight: 700; }
            QLabel#adminStatusBanner {
                background: #0d1217;
                color: #a4b3c4;
                border: 1px solid #1f2733;
                border-radius: 9px;
                padding: 6px 14px;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton#adminBackButton {
                background: #0f141a;
                border: 1px solid #232e3d;
                border-radius: 12px;
            }
            QPushButton#adminBackButton:hover {
                background: #17202c;
                border-color: #3b4e64;
            }
            QPushButton#adminBackButton:pressed {
                background: #1c2738;
                border: 2px solid #f6c16a;
            }
            QPushButton#adminDetectionButton,
            QPushButton#adminSlamButton,
            QPushButton#adminNavButton,
            QPushButton#adminStartNavButton,
            QPushButton#adminStopNavButton,
            QPushButton#adminSystemStatusButton {
                background: #0d1218;
                border: 1px solid #1c2532;
                border-radius: 18px;
            }
            QPushButton#adminDetectionButton:hover,
            QPushButton#adminSlamButton:hover,
            QPushButton#adminNavButton:hover,
            QPushButton#adminStartNavButton:hover,
            QPushButton#adminStopNavButton:hover,
            QPushButton#adminSystemStatusButton:hover {
                background: #141d27;
                border-color: #3b4e64;
            }
            QPushButton#adminDetectionButton:pressed,
            QPushButton#adminSlamButton:pressed,
            QPushButton#adminNavButton:pressed,
            QPushButton#adminStartNavButton:pressed,
            QPushButton#adminStopNavButton:pressed,
            QPushButton#adminSystemStatusButton:pressed {
                background: #192432;
                border: 2px solid #f6c16a;
            }
            QLabel#launcherTitle {
                color: #ffffff;
                font-size: 14px;
                font-weight: 700;
            }
        """)

        self.update_wifi()
        self.set_battery(None)

    def update_wifi(self) -> None:
        self.set_wifi(check_wifi_connected())

    def set_wifi(self, connected: bool) -> None:
        icon = self.wifi_connected_icon if connected else self.wifi_disconnected_icon
        if icon and Path(icon).is_file():
            pm = QPixmap(str(icon))
            if not pm.isNull():
                self.wifi_icon_label.setPixmap(
                    pm.scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        self.wifi_text_label.setText("Online" if connected else "Offline")
        self.wifi_text_label.setStyleSheet("color: #4cd964;" if connected else "color: #ff5252;")

    def set_battery(self, percent: float | None) -> None:
        if percent is None or percent < 0:
            key = "0"
            text = "--%"
        else:
            text = f"{percent:.0f}%"
            if percent >= 85:
                key = "100"
            elif percent >= 60:
                key = "75"
            elif percent >= 35:
                key = "50"
            elif percent >= 15:
                key = "25"
            else:
                key = "0"
        icon = self.battery_icons.get(key)
        if icon and Path(icon).is_file():
            pm = QPixmap(str(icon))
            if not pm.isNull():
                self.battery_icon_label.setPixmap(
                    pm.scaled(24, 24, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        self.battery_text_label.setText(text)
        self.battery.setText(f"Pin: {text}")

    def set_connection(self, connected: bool):
        self.connection.setText("Kết nối ROS 2: Có phản hồi" if connected else "Kết nối ROS 2: Chưa kết nối")
        if not connected:
            self.status_banner.setText("ROS 2: Chưa kết nối")

    def set_robot_status(self, status: dict[str, Any]):
        battery = status.get("battery_percent")
        if isinstance(battery, (int, float)):
            self.set_battery(float(battery))
        else:
            self.set_battery(None)
        state = status.get("robot_state", "Bình thường")
        self.robot.setText(f"Trạng thái robot: {state}")
        self.status_banner.setText(f"Trạng thái robot: {state}")

    def set_navigation_status(self, status: dict[str, Any]):
        nav_state = status.get("status", "Chưa có dữ liệu")
        self.nav.setText(f"Điều hướng: {nav_state}")
        self.status_banner.setText(f"Điều hướng: {nav_state}")


class ServicePage(QWidget):
    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    mode_selected = pyqtSignal(str)

    def __init__(
        self,
        back_icon: Path | None = None,
        wait_icon: Path | None = None,
        continue_icon: Path | None = None,
        service_icon: Path | None = None,
        home_icon: Path | None = None,
        emergency_icon: Path | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("servicePage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._selected_mode: str | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(20, 14, 20, 16)
        root.setSpacing(10)

        # 1. Header (Nền đen #080808)
        header = QFrame()
        header.setObjectName("serviceHeader")
        header.setMinimumHeight(64)
        header.setMaximumHeight(72)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(8)

        # Nút Home bên trái
        self.home_button = QPushButton()
        self.home_button.setObjectName("serviceHomeButton")
        self.home_button.setCursor(Qt.PointingHandCursor)
        self.home_button.setFixedSize(46, 46)
        self.home_button.setToolTip("Về màn hình chính")
        if home_icon and Path(home_icon).is_file():
            self.home_button.setIcon(QIcon(str(home_icon)))
            self.home_button.setIconSize(QSize(28, 28))
        else:
            self.home_button.setText("🏠")
            self.home_button.setStyleSheet("font-size: 22px;")
        self.home_button.clicked.connect(self.home_requested.emit)
        header_layout.addWidget(self.home_button)

        # Nút Dừng khẩn cấp
        self.emergency_button = QPushButton()
        self.emergency_button.setObjectName("serviceEmergencyButton")
        self.emergency_button.setCursor(Qt.PointingHandCursor)
        self.emergency_button.setFixedSize(46, 46)
        self.emergency_button.setToolTip("DỪNG KHẨN CẤP")
        if emergency_icon and Path(emergency_icon).is_file():
            self.emergency_button.setIcon(QIcon(str(emergency_icon)))
            self.emergency_button.setIconSize(QSize(36, 36))
        else:
            self.emergency_button.setText("🛑")
            self.emergency_button.setStyleSheet("font-size: 24px;")
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        header_layout.addWidget(self.emergency_button)

        # Nút Back bên trái
        self.back_button = QPushButton()
        self.back_button.setObjectName("serviceBackButton")
        self.back_button.setCursor(Qt.PointingHandCursor)
        self.back_button.setFixedSize(46, 46)
        self.back_button.setToolTip("Quay lại")
        if back_icon and Path(back_icon).is_file():
            self.back_button.setIcon(QIcon(str(back_icon)))
            self.back_button.setIconSize(QSize(28, 28))
        else:
            self.back_button.setText("‹")
            self.back_button.setStyleSheet("color: #ffb32c; font-size: 28px; font-weight: 700;")
        self.back_button.clicked.connect(self.back_requested.emit)
        header_layout.addWidget(self.back_button)

        # Tiêu đề & Subtitle
        heading = QVBoxLayout()
        heading.setSpacing(2)
        title = QLabel("SERVICE")
        title.setObjectName("serviceHeaderTitle")
        subtitle = QLabel("Souta Robot · Phục vụ nước")
        subtitle.setObjectName("serviceHeaderSubtitle")
        heading.addWidget(title)
        heading.addWidget(subtitle)
        header_layout.addLayout(heading)

        header_layout.addStretch(1)

        # Icon ly nước nhỏ bên phải nếu có service.png
        if service_icon and Path(service_icon).is_file():
            service_lbl = QLabel()
            service_lbl.setObjectName("serviceHeaderIcon")
            service_lbl.setFixedSize(36, 36)
            service_lbl.setAlignment(Qt.AlignCenter)
            pm = QPixmap(str(service_icon))
            if not pm.isNull():
                service_lbl.setPixmap(pm.scaled(28, 28, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            header_layout.addWidget(service_lbl)

        root.addWidget(header)

        # 2. Main Content
        root.addStretch(1)

        center_vbox = QVBoxLayout()
        center_vbox.setSpacing(6)
        center_vbox.setAlignment(Qt.AlignCenter)

        main_title = QLabel("Dịch vụ phục vụ nước")
        main_title.setObjectName("serviceMainTitle")
        main_title.setAlignment(Qt.AlignCenter)
        center_vbox.addWidget(main_title)

        instruction = QLabel("Vui lòng lựa chọn chức năng bên dưới")
        instruction.setObjectName("serviceInstruction")
        instruction.setAlignment(Qt.AlignCenter)
        center_vbox.addWidget(instruction)

        root.addLayout(center_vbox)
        root.addSpacing(16)

        # Hai button lớn nằm ngang, kích thước tương đương nhau, căn giữa cửa sổ
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(24)
        cards_layout.setAlignment(Qt.AlignCenter)

        def make_service_card(
            icon_file: Path | None,
            title_text: str,
            desc_text: str,
            mode_id: str,
        ) -> QPushButton:
            btn = QPushButton()
            btn.setObjectName("serviceCard")
            btn.setProperty("mode", mode_id)
            btn.setProperty("selected", False)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedSize(280, 180)

            btn_layout = QVBoxLayout(btn)
            btn_layout.setContentsMargins(16, 16, 16, 16)
            btn_layout.setSpacing(8)
            btn_layout.setAlignment(Qt.AlignCenter)

            icon_lbl = QLabel()
            icon_lbl.setObjectName("serviceCardIcon")
            icon_lbl.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            icon_lbl.setFixedSize(72, 72)
            icon_lbl.setAlignment(Qt.AlignCenter)
            if icon_file and Path(icon_file).is_file():
                pm = QPixmap(str(icon_file))
                if not pm.isNull():
                    icon_lbl.setPixmap(pm.scaled(68, 68, Qt.KeepAspectRatio, Qt.SmoothTransformation))
            btn_layout.addWidget(icon_lbl, 0, Qt.AlignCenter)

            t_lbl = QLabel(title_text)
            t_lbl.setObjectName("serviceCardTitle")
            t_lbl.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            t_lbl.setAlignment(Qt.AlignCenter)
            btn_layout.addWidget(t_lbl, 0, Qt.AlignCenter)

            d_lbl = QLabel(desc_text)
            d_lbl.setObjectName("serviceCardDesc")
            d_lbl.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            d_lbl.setAlignment(Qt.AlignCenter)
            btn_layout.addWidget(d_lbl, 0, Qt.AlignCenter)

            btn.clicked.connect(lambda: self._select_mode(mode_id))
            return btn

        # Button 1 — Chờ phục vụ
        self.btn_wait = make_service_card(
            wait_icon,
            "CHỜ PHỤC VỤ",
            "Chế độ chờ phục vụ khách",
            "wait",
        )

        # Button 2 — Hoàn tất & tiếp tục
        self.btn_continue = make_service_card(
            continue_icon,
            "HOÀN TẤT & TIẾP TỤC",
            "Xác nhận kết thúc phục vụ",
            "continue",
        )

        cards_layout.addWidget(self.btn_wait)
        cards_layout.addWidget(self.btn_continue)
        root.addLayout(cards_layout)

        root.addStretch(2)

        # 3. Footer
        footer = QLabel("Đã kết nối điều khiển robot · Nhấn nút để kích hoạt")
        footer.setObjectName("serviceFooter")
        footer.setAlignment(Qt.AlignCenter)
        root.addWidget(footer)

        self.setStyleSheet("""
            QWidget#servicePage {
                background-color: #080808;
            }
            QFrame#serviceHeader {
                background-color: #080808;
                border-bottom: 1px solid #1c1c1c;
                border-radius: 8px;
            }
            QPushButton#serviceBackButton {
                background: #171717;
                border: 1px solid #3a3a3a;
                border-radius: 11px;
            }
            QPushButton#serviceBackButton:hover {
                background: #242424;
                border-color: #4f4f4f;
            }
            QPushButton#serviceBackButton:pressed {
                background: #2b2b2b;
            }
            QLabel#serviceHeaderTitle {
                color: #ffffff;
                font-size: 16px;
                font-weight: 700;
            }
            QLabel#serviceHeaderSubtitle {
                color: #8d8d8d;
                font-size: 11px;
            }
            QLabel#serviceMainTitle {
                color: #ffffff;
                font-size: 20px;
                font-weight: 700;
            }
            QLabel#serviceInstruction {
                color: #9a9a9a;
                font-size: 13px;
            }
            QPushButton#serviceCard {
                background-color: #1C1C1C;
                border: 1px solid #404040;
                border-radius: 14px;
            }
            QPushButton#serviceCard:hover {
                background-color: #242424;
                border-color: #555555;
            }
            QPushButton#serviceCard[selected="true"] {
                background-color: #262626;
                border: 2px solid #ffb32c;
            }
            QLabel#serviceCardTitle {
                color: #ffffff;
                font-size: 16px;
                font-weight: 700;
            }
            QLabel#serviceCardDesc {
                color: #a0a0a0;
                font-size: 12px;
            }
            QLabel#serviceFooter {
                color: #777777;
                font-size: 12px;
            }
        """)

    def _select_mode(self, mode: str) -> None:
        self._selected_mode = mode
        self.btn_wait.setProperty("selected", mode == "wait")
        self.btn_continue.setProperty("selected", mode == "continue")
        self.btn_wait.style().unpolish(self.btn_wait)
        self.btn_wait.style().polish(self.btn_wait)
        self.btn_continue.style().unpolish(self.btn_continue)
        self.btn_continue.style().polish(self.btn_continue)
        self.mode_selected.emit(mode)

    def get_selected_mode(self) -> str | None:
        return self._selected_mode

    def select_mode_programmatically(self, mode: str) -> None:
        self._select_mode(mode)

    def reset_selection(self) -> None:
        self._selected_mode = None
        self.btn_wait.setProperty("selected", False)
        self.btn_continue.setProperty("selected", False)
        self.btn_wait.style().unpolish(self.btn_wait)
        self.btn_wait.style().polish(self.btn_wait)
        self.btn_continue.style().unpolish(self.btn_continue)
        self.btn_continue.style().polish(self.btn_continue)

