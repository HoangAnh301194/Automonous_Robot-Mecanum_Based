"""Public welcome and greeting page."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QImageReader, QLinearGradient, QPainter, QPixmap
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from robot_ui.kiosk.pages.base import picture


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
            QLabel#welcomeHeading { color: #FFFFFF; background: transparent; padding: 0px; border: none; font-size: 29px; font-weight: 800; }
            QLabel#welcomeSubtitle { color: #FFFFFF; background: transparent; padding: 0px; border: none; font-size: 19px; font-weight: 700; }
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
        p.setRenderHint(QPainter.SmoothPixmapTransform)
        p.fillRect(self.rect(), QColor("#f8fafc"))
        if not self._background.isNull():
            p.drawPixmap((self.width() - self._background.width()) // 2,
                         (self.height() - self._background.height()) // 2,
                         self._background)
        gradient = QLinearGradient(0, 0, 0, self.height())
        gradient.setColorAt(0, QColor(255, 255, 255, 12))
        gradient.setColorAt(0.6, QColor(255, 255, 255, 0))
        gradient.setColorAt(1, QColor(248, 250, 252, 44))
        p.fillRect(self.rect(), QBrush(gradient))
