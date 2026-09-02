from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "server": {
        "host": "0.0.0.0",
        "port": 8000,
        "telemetry_interval_ms": 500,
        "cors_origins": [],
    },
    "ros": {
        "node_name": "robot_ui_bridge",
        "graph": {
            "refresh_seconds": 2.0,
            "include_hidden": False,
            "max_topics": 120,
            "exclude_topic_prefixes": ["/parameter_events", "/rosout"],
        },
        "topics": {
            "battery": "/battery",
            "encoders": "/dataenc",
            "map": "/map",
            "odom": "/odom",
            "scan": "/scan",
            "goal": "/goal_pose",
            "global_path": "/plan",
            "local_path": "/local_plan",
            "global_costmap": "/global_costmap/costmap_raw",
            "local_costmap": "/local_costmap/costmap_raw",
            "nav2_status": "/navigate_to_pose/_action/status",
            "diagnostics": "/diagnostics",
            "rosout": "/rosout",
            "mission_status": "/mission/status",
            "camera_color_info": "/camera/color/camera_info",
            "camera_depth_info": "/camera/depth/camera_info",
            "yolo_detections": "/yolo/detections",
            "wave_detected": "/pose/wave_detected",
            "wave_status": "/pose/wave_status",
            "scan_obstacles": "/scan_obstacles",
        },
        "expected_rates_hz": {
            "battery": 10.0,
            "encoders": 10.0,
            "odom": 20.0,
            "scan": 5.0,
            "global_path": 1.0,
            "local_path": 5.0,
            "global_costmap": 1.0,
            "local_costmap": 5.0,
            "diagnostics": 1.0,
            "mission_status": 2.0,
            "camera_color_info": 5.0,
            "camera_depth_info": 5.0,
            "yolo_detections": 5.0,
            "scan_obstacles": 5.0,
        },
        "navigation": {
            "map_frame": "map",
            "base_frame": "base_footprint",
            "tf_refresh_seconds": 0.2,
            "max_scan_points": 360,
            "max_path_points": 300,
            "costmap_preview_max_dimension": 160,
        },
        "diagnostics": {
            "health_refresh_seconds": 1.0,
            "stale_multiplier": 3.0,
            "minimum_stale_seconds": 2.0,
            "warn_rate_ratio": 0.5,
            "max_statuses": 200,
            "max_rosout_entries": 500,
        },
        "vision": {
            "max_detection_samples": 40,
        },
    },
    "features": {
        "read_only": True,
        "navigation_controls": False,
        "process_controls": False,
        "teleoperation": False,
    },
}


def _merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge(result[key], value)
        else:
            result[key] = value
    return result


def default_config_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("robot_ui")) / "config" / "robot_ui.yaml"
    except Exception:
        return Path(__file__).resolve().parents[1] / "config" / "robot_ui.yaml"


def default_web_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory("robot_ui")) / "web_dist"
    except Exception:
        path = Path(__file__).resolve().parent / "web_dist"

    index_file = path / "index.html"
    if index_file.exists():
        return index_file.resolve().parent
    return path


@dataclass(frozen=True)
class AppConfig:
    raw: dict[str, Any]
    source: Path

    @property
    def server(self) -> dict[str, Any]:
        return self.raw["server"]

    @property
    def ros(self) -> dict[str, Any]:
        return self.raw["ros"]

    @property
    def features(self) -> dict[str, Any]:
        return self.raw["features"]


def load_config(path: str | Path | None = None) -> AppConfig:
    config_path = Path(path).expanduser().resolve() if path else default_config_path()
    override: dict[str, Any] = {}
    if config_path.exists():
        with config_path.open("r", encoding="utf-8") as handle:
            override = yaml.safe_load(handle) or {}
    return AppConfig(raw=_merge(DEFAULT_CONFIG, override), source=config_path)
