from __future__ import annotations

import json
import tempfile
from functools import lru_cache
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QImage, QPainter

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory


def package_share() -> Path:
    """Return installed assets, with a source-tree fallback for development."""
    try:
        return Path(get_package_share_directory("robot_ui"))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[2]


# White-on-transparent glyphs designed for the old dark theme. Keep brand,
# service and emergency graphics in their original semantic colours.
_LIGHT_GLYPH_COLORS = {
    "back.png": "#243447",
    "homeIcon.png": "#243447",
    "chatIcon.png": "#243447",
    "setting_icon.png": "#243447",
    "wifi.png": "#16804a",
    "wifidisconected.png": "#be3446",
    "mic.png": "#243447",
    "mic_listening.png": "#b32632",
    "mic_off.png": "#243447",
    "microphone.png": "#243447",
    "speaker_on.png": "#243447",
    "speaker_off.png": "#243447",
    "stop_speaking.png": "#b32632",
    "status.png": "#243447",
    "detection.png": "#243447",
    "locolization.png": "#243447",
    "manual_control.png": "#243447",
    "service_wait.png": "#24558a",
    "service_continue.png": "#16804a",
    **{f"battery_{pct}.png": "#243447" for pct in (0, 25, 50, 75, 100)},
}


@lru_cache(maxsize=64)
def icon_path(filename: str) -> Path:
    """Return a colour-corrected monochrome icon, preserving its alpha channel.

    Tint only the known white glyph files. Source images are not overwritten;
    the display variants are cached in a writable temporary directory.
    """
    source_path = package_share() / "souta_robot" / filename
    color = _LIGHT_GLYPH_COLORS.get(filename)
    if color is None or not source_path.is_file():
        return source_path
    try:
        stamp = source_path.stat().st_mtime_ns
        cache_dir = Path(tempfile.gettempdir()) / "souta_light_icons"
        cache_dir.mkdir(parents=True, exist_ok=True)
        cached = cache_dir / f"{source_path.stem}_{stamp}.png"
        if cached.is_file():
            return cached
        source = QImage(str(source_path))
        if source.isNull() or not source.hasAlphaChannel():
            return source_path
        image = QImage(source.size(), QImage.Format_ARGB32_Premultiplied)
        image.fill(Qt.transparent)
        painter = QPainter(image)
        painter.drawImage(0, 0, source)
        painter.setCompositionMode(QPainter.CompositionMode_SourceIn)
        painter.fillRect(image.rect(), QColor(color))
        painter.end()
        return cached if image.save(str(cached), "PNG") else source_path
    except (OSError, ValueError, RuntimeError):
        return source_path


class EmotionCatalog:
    def __init__(self) -> None:
        config_path = package_share() / "config" / "emotions.json"
        with config_path.open("r", encoding="utf-8") as config_file:
            config = json.load(config_file)
        self.default = str(config.get("default", "idle"))
        self.display_scale = max(0.5, min(1.0, float(config.get("display_scale", 0.9))))
        self.vertical_offset_px = int(config.get("vertical_offset_px", 0))
        self._emotions = dict(config.get("emotions", {}))

    def path(self, name: str) -> Path:
        filename = self._emotions.get(name) or self._emotions[self.default]
        return package_share() / "souta_robot" / "videomp4" / filename

    def has(self, name: str) -> bool:
        return name in self._emotions
