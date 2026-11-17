"""Serial port resolver for Hoverboard motor controller and RPLiDAR sensors.

Faithfully implements the proven port scanning logic from gui_v3.py and simulator:
- Auto-detects udev aliases (/dev/hoverboard, /dev/rplidar, /dev/esp32, /dev/lidar).
- Preserves /dev/serial/by-id/* symlinks to recognize hardware chipsets (Silicon Labs CP2102, RPLiDAR, ESP32).
- Passive read probe for ESP32 / Hoverboard telemetry and bootloader headers (0xCD 0xAB, V:, E, ets Jun).
- Optional active probe for RPLiDAR health protocol (0xA5 0x52).
- Automatic physical device deduplication and port pairing.
- Supports force_swap to quickly invert port assignments.
"""
from __future__ import annotations

import glob
import logging
import os
import time
from typing import Tuple

LOG = logging.getLogger(__name__)


def list_serial_ports() -> list[str]:
    """Return all available serial port device paths on Linux, prioritizing by-id symlinks."""
    raw_ports = (
        sorted(glob.glob("/dev/serial/by-id/*"))
        + sorted(glob.glob("/dev/ttyUSB*"))
        + sorted(glob.glob("/dev/ttyACM*"))
    )
    dedup: list[str] = []
    seen_real: set[str] = set()
    for p in raw_ports:
        try:
            real = os.path.realpath(p)
        except Exception:
            real = p
        if real not in seen_real:
            seen_real.add(real)
            dedup.append(p)
    return dedup


def is_esp32_port(port: str, baudrate: int = 115200, timeout: float = 0.1) -> bool:
    """Passive probe for ESP32/Hoverboard telemetry packets and bootloader messages."""
    try:
        import serial  # type: ignore
    except ImportError:
        return False
    ser = None
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        ser.reset_input_buffer()
        start_time = time.monotonic()
        while time.monotonic() - start_time < 0.6:
            raw = ser.read(64)
            if not raw:
                continue
            if b"\xcd\xab" in raw or b"\xab\xcd" in raw:
                return True
            if b"ets Jun" in raw or b"rst:0x" in raw or b"boot:" in raw:
                return True
            if b"V:" in raw or b"E," in raw:
                return True
    except Exception:
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    return False


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
        # Send STOP first in case LiDAR is continuously streaming scan data
        ser.write(bytes([0xA5, 0x25]))
        time.sleep(0.02)
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


def detect_serial_ports(force_swap: bool = False) -> Tuple[str, str, str]:
    """Tự động phát hiện cổng ESP32/Hoverboard và LiDAR chuẩn xác như gui_v3 và simulator.

    Returns:
        (hoverboard_port, lidar_port, detection_method)
    """
    # 1. Kiểm tra udev aliases trước
    hb_alias = "/dev/hoverboard" if os.path.exists("/dev/hoverboard") else (
        "/dev/esp32" if os.path.exists("/dev/esp32") else None
    )
    lidar_alias = "/dev/rplidar" if os.path.exists("/dev/rplidar") else (
        "/dev/lidar" if os.path.exists("/dev/lidar") else None
    )
    if hb_alias and lidar_alias:
        if force_swap:
            return lidar_alias, hb_alias, "swap_override"
        return hb_alias, lidar_alias, "udev_alias"

    ports = list_serial_ports()
    if not ports:
        if force_swap:
            return "/dev/ttyUSB1", "/dev/ttyUSB0", "swap_override"
        return "/dev/ttyUSB0", "/dev/ttyUSB1", "fallback_defaults"

    esp_port: str | None = hb_alias
    lidar_port: str | None = lidar_alias

    # 2. Nhận diện qua tên symlink / chipset (đặc biệt chip Silicon Labs CP2102 của RPLiDAR trên Jetson)
    for p in ports:
        key = os.path.basename(p).lower()
        if lidar_port is None and any(tag in key for tag in ("rplidar", "slamtec", "lidar", "cp210", "silicon_labs")):
            lidar_port = p
        if esp_port is None and any(tag in key for tag in ("hoverboard", "esp32", "esp")):
            esp_port = p

    # 3. Thăm dò byte thực tế ESP32/Hoverboard (thụ động như gui_v3)
    if esp_port is None:
        for p in ports:
            if p != lidar_port and is_esp32_port(p):
                esp_port = p
                LOG.info(f"[SerialResolver] Probed ESP32/Hoverboard found at {p}")
                break

    # 4. Thăm dò RPLiDAR nếu chưa phát hiện
    if lidar_port is None:
        for p in ports:
            if p != esp_port and probe_rplidar(p):
                lidar_port = p
                LOG.info(f"[SerialResolver] Probed RPLiDAR found at {p}")
                break

    # 5. Phân bổ cổng còn lại
    if esp_port is None:
        for p in ports:
            if p.endswith("ttyUSB0") or p.endswith("ttyACM0"):
                esp_port = p
                break
    if esp_port is None:
        esp_port = ports[0]

    # Đảm bảo lidar_port trỏ tới thiết bị vật lý KHÁC esp_port
    try:
        esp_real = os.path.realpath(esp_port)
    except Exception:
        esp_real = esp_port

    if lidar_port is None or os.path.realpath(lidar_port) == esp_real:
        lidar_port = None
        for p in ports:
            try:
                p_real = os.path.realpath(p)
            except Exception:
                p_real = p
            if p_real != esp_real:
                lidar_port = p
                break

    # Fallback nếu chỉ có 1 cổng
    if lidar_port is None or os.path.realpath(lidar_port) == esp_real:
        lidar_port = "/dev/ttyUSB1" if esp_real != "/dev/ttyUSB1" else "/dev/ttyUSB0"

    if force_swap:
        return lidar_port, esp_port, "swap_override"

    method = "scan" if (esp_port in ports or lidar_port in ports) else "fallback_defaults"
    return esp_port, lidar_port, method
