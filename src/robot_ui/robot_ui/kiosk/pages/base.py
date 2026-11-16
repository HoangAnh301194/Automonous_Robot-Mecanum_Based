"""Common UI utilities and helpers for Kiosk pages."""
from __future__ import annotations

from pathlib import Path
import re

import cv2
import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QIcon, QImage, QPixmap
from PyQt5.QtWidgets import QLabel

EMOJI_PATTERN = re.compile(
    r"[\U00010000-\U0010ffff\u2600-\u26ff\u2700-\u27bf\ufe00-\ufe0f\u200d]+",
    flags=re.UNICODE,
)


def picture(path: Path, size: int = 70) -> QLabel:
    """Create a fixed-size QLabel containing an image scaled to keep aspect ratio."""
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


def monochrome_icon(path: Path, color: QColor) -> QIcon:
    """Tint a monochrome icon using its alpha channel, or white luminance fallback."""
    image_data = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image_data is None:
        return QIcon(str(path))
    if image_data.ndim == 3 and image_data.shape[2] == 4 and (image_data[:, :, 3] < 255).any():
        alpha = image_data[:, :, 3].copy()
    else:
        grayscale = (
            image_data
            if image_data.ndim == 2
            else cv2.cvtColor(image_data[:, :, :3], cv2.COLOR_BGR2GRAY)
        )
        alpha = 255 - grayscale
        alpha[alpha < 24] = 0
    rgba = np.empty((alpha.shape[0], alpha.shape[1], 4), dtype=np.uint8)
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


def strip_emojis(text: str) -> str:
    """Remove emoji/Unicode pictographs to avoid font rendering issues (empty boxes) on Linux Qt5."""
    if not text:
        return text
    cleaned = EMOJI_PATTERN.sub("", text)
    cleaned = re.sub(r" +([,.\?!;:])", r"\1", cleaned)
    lines = [re.sub(r"[ \t]+", " ", line).strip() for line in cleaned.splitlines()]
    return "\n".join(lines).strip()
