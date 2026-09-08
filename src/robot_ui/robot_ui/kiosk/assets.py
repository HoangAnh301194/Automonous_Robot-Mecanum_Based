from __future__ import annotations

import json
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory


def package_share() -> Path:
    """Return installed assets, with a source-tree fallback for development."""
    try:
        return Path(get_package_share_directory("robot_ui"))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[2]


def icon_path(filename: str) -> Path:
    return package_share() / "emotion_dasaimochi" / filename


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
        return package_share() / "emotion_dasaimochi" / "videomp4" / filename

    def has(self, name: str) -> bool:
        return name in self._emotions
