"""Robot dining and beverage service selection page."""
from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QIcon, QPixmap
from PyQt5.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


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
