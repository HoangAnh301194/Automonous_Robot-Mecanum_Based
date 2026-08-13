from __future__ import annotations

import os
from pathlib import Path
import shutil
import socket
import threading
from typing import Any

from robot_ui.state_store import StateStore


class SystemMonitor:
    def __init__(self, store: StateStore, interval_seconds: float = 1.0):
        self._store = store
        self._interval_seconds = interval_seconds
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="robot-ui-system", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:
        try:
            import psutil
        except ImportError:
            self._store.append_event("WARN", "system", "psutil is not installed")
            return

        while not self._stop_event.wait(self._interval_seconds):
            disk = shutil.disk_usage("/")
            memory = psutil.virtual_memory()
            temperatures = self._read_temperatures(psutil)
            self._store.patch_section(
                "system",
                {
                    "hostname": socket.gethostname(),
                    "cpu_percent": psutil.cpu_percent(interval=None),
                    "cpu_per_core_percent": psutil.cpu_percent(
                        interval=None, percpu=True
                    ),
                    "load_average": self._read_load_average(),
                    "memory_percent": memory.percent,
                    "memory_used_bytes": memory.used,
                    "memory_total_bytes": memory.total,
                    "disk_percent": (disk.used / disk.total * 100.0) if disk.total else 0.0,
                    "disk_free_bytes": disk.free,
                    "boot_time": psutil.boot_time(),
                    "gpu_percent": self._read_jetson_gpu_percent(),
                    "temperature_celsius": max(
                        (
                            item["current_celsius"]
                            for item in temperatures
                        ),
                        default=None,
                    ),
                    "temperatures": temperatures,
                },
            )

    @staticmethod
    def _read_load_average() -> dict[str, float] | None:
        try:
            one, five, fifteen = os.getloadavg()
        except (AttributeError, OSError):
            return None
        return {"one": one, "five": five, "fifteen": fifteen}

    @staticmethod
    def _read_temperatures(psutil: Any) -> list[dict[str, Any]]:
        try:
            sensor_groups = psutil.sensors_temperatures(fahrenheit=False)
        except (AttributeError, NotImplementedError, OSError):
            return []

        readings: list[dict[str, Any]] = []
        for group_name, sensors in sensor_groups.items():
            for index, sensor in enumerate(sensors):
                readings.append(
                    {
                        "label": sensor.label or f"{group_name}:{index}",
                        "current_celsius": float(sensor.current),
                        "high_celsius": (
                            float(sensor.high)
                            if sensor.high is not None
                            else None
                        ),
                        "critical_celsius": (
                            float(sensor.critical)
                            if sensor.critical is not None
                            else None
                        ),
                    }
                )
        return readings

    @staticmethod
    def _read_jetson_gpu_percent() -> float | None:
        candidates = (
            Path("/sys/devices/gpu.0/load"),
            Path("/sys/devices/platform/gpu.0/load"),
            Path("/sys/devices/platform/17000000.gpu/load"),
        )
        for path in candidates:
            try:
                raw_value = float(path.read_text(encoding="utf-8").strip())
            except (OSError, ValueError):
                continue
            percentage = raw_value / 10.0 if raw_value > 100.0 else raw_value
            return max(0.0, min(100.0, percentage))
        return None
