"""Serial port resolver for Hoverboard motor controller and RPLiDAR sensors.

Aligns with the proven detection mechanism from gui_v3.py:
- Auto-detects udev aliases (/dev/hoverboard, /dev/rplidar, /dev/esp32, /dev/lidar)
- Scans system serial devices (/dev/ttyUSB*, /dev/ttyACM*)
- Probes RPLiDAR sensor using standard SDK protocol bytes (0xA5 0x52)
- Automatically resolves optimal port assignments for Layer 1 bringup.
"""
from __future__ import annotations

import glob
import logging
import os
from pathlib import Path
from typing import Tuple

LOG = logging.getLogger(__name__)


def list_serial_ports() -> list[str]:
    """Return all available serial port device paths on Linux."""
    ports: list[str] = []
    # by-id symlinks
    by_id = "/dev/serial/by-id"
    if os.path.isdir(by_id):
        for name in os.listdir(by_id):
            full = os.path.join(by_id, name)
            if os.path.islink(full):
                try:
                    resolved = os.path.realpath(full)
                    if resolved not in ports:
                        ports.append(resolved)
                except Exception:
                    pass

    # direct ttyUSB / ttyACM nodes
    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for p in sorted(glob.glob(pattern)):
            if p not in ports:
                ports.append(p)

    return ports


def probe_rplidar(port: str, baudrate: int = 115200, timeout: float = 0.4) -> bool:
    """Probe whether the given serial port responds to RPLiDAR protocol."""
    try:
        import serial  # type: ignore
    except ImportError:
        LOG.debug("pyserial is not installed; skipping active probe.")
        return False

    ser = None
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        ser.reset_input_buffer()
        # RPLiDAR GET_HEALTH command: 0xA5 0x52
        ser.write(bytes([0xA5, 0x52]))
        resp = ser.read(10)
        # RPLiDAR response descriptor starts with 0xA5 0x5A
        if resp and b"\xa5\x5a" in resp:
            return True
    except Exception as exc:
        LOG.debug("Error probing RPLiDAR on %s: %s", port, exc)
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    return False


def detect_serial_ports() -> Tuple[str, str, str]:
    """Detect and return (hoverboard_port, lidar_port, detection_method).

    Checks udev aliases first; if not present, scans /dev/ttyUSB* and probes
    for RPLiDAR to cleanly pair Hoverboard and LiDAR ports.
    """
    # 1. Check udev aliases
    hb_alias = "/dev/hoverboard" if os.path.exists("/dev/hoverboard") else (
        "/dev/esp32" if os.path.exists("/dev/esp32") else None
    )
    lidar_alias = "/dev/rplidar" if os.path.exists("/dev/rplidar") else (
        "/dev/lidar" if os.path.exists("/dev/lidar") else None
    )

    if hb_alias and lidar_alias:
        return hb_alias, lidar_alias, "udev_alias"

    ports = list_serial_ports()
    if not ports:
        return "/dev/ttyUSB0", "/dev/ttyUSB1", "fallback_defaults"

    hoverboard_port: str | None = hb_alias
    lidar_port: str | None = lidar_alias

    # 2. Check by filename hints (e.g. symlinks in by-id)
    for p in ports:
        key = Path(p).name.lower()
        if lidar_port is None and any(tag in key for tag in ("rplidar", "slamtec", "lidar")):
            lidar_port = p
        if hoverboard_port is None and any(tag in key for tag in ("hoverboard", "esp32", "esp")):
            hoverboard_port = p

    # 3. Active hardware probe for RPLiDAR
    if lidar_port is None:
        for p in ports:
            if p != hoverboard_port and probe_rplidar(p):
                lidar_port = p
                LOG.info(f"[SerialResolver] Probed RPLiDAR found at {p}")
                break

    # 4. Pair remaining port
    if hoverboard_port is None and lidar_port is not None:
        for p in ports:
            if p != lidar_port:
                hoverboard_port = p
                break

    if lidar_port is None and hoverboard_port is not None:
        for p in ports:
            if p != hoverboard_port:
                lidar_port = p
                break

    # 5. Final fallback assignments
    if hoverboard_port is None:
        hoverboard_port = "/dev/ttyUSB0" if "/dev/ttyUSB0" in ports else ports[0]

    if lidar_port is None or lidar_port == hoverboard_port:
        candidates = [p for p in ports if p != hoverboard_port]
        lidar_port = candidates[0] if candidates else "/dev/ttyUSB1"

    method = "auto_probed" if (hoverboard_port in ports or lidar_port in ports) else "fallback_defaults"
    return hoverboard_port, lidar_port, method
