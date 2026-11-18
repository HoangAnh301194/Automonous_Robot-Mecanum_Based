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
        while time.monotonic() - start_time < 0.25:
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


def get_device_id_path(port: str) -> str | None:
    """Trích xuất thuộc tính ID_PATH của thiết bị serial qua udevadm."""
    try:
        import subprocess
        real_port = os.path.realpath(port)
        out = subprocess.check_output(
            ["udevadm", "info", "-q", "property", "-n", real_port],
            text=True,
            timeout=2.0,
            stderr=subprocess.DEVNULL,
        )
        for line in out.splitlines():
            if line.startswith("ID_PATH="):
                val = line.split("=", 1)[1].strip()
                if val:
                    return val
    except Exception as exc:
        LOG.debug("Không thể lấy ID_PATH cho cổng %s: %s", port, exc)
    return None


def rebind_udev_rules(lidar_port: str, hoverboard_port: str) -> bool:
    """Tự động ghi đè /etc/udev/rules.d/99-robot-serial.rules và reload udev.

    Cố định vĩnh viễn /dev/rplidar và /dev/hoverboard theo ID_PATH của cổng vật lý.
    """
    try:
        import subprocess
        lidar_canon = os.path.realpath(lidar_port)
        hover_canon = os.path.realpath(hoverboard_port)

        if not os.path.exists(lidar_canon) or not os.path.exists(hover_canon):
            LOG.warning("rebind_udev_rules: Cổng thiết bị không tồn tại (%s, %s)", lidar_canon, hover_canon)
            return False

        if lidar_canon == hover_canon:
            LOG.warning("rebind_udev_rules: Trùng lặp cổng (%s == %s)", lidar_canon, hover_canon)
            return False

        # 1. Thử gọi script helper nếu đã cài đặt
        helper_candidates = [
            "/usr/local/bin/update_robot_udev.sh",
            os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "update_robot_udev.sh"),
            "/home/a/ros2_ws/src/robot_ui/scripts/update_robot_udev.sh",
        ]
        for script in helper_candidates:
            if os.path.exists(script) and os.access(script, os.X_OK):
                cmd = ["sudo", "-n", script, lidar_canon, hover_canon] if os.geteuid() != 0 else [script, lidar_canon, hover_canon]
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=5.0)
                if res.returncode == 0:
                    LOG.info("[SerialResolver] Đã cập nhật udev rules qua helper: %s", script)
                    return True
                else:
                    LOG.debug("Helper %s thất bại: %s", script, res.stderr)

        # 2. Phương án trực tiếp: trích xuất ID_PATH và ghi file rule
        lidar_path = get_device_id_path(lidar_canon)
        hover_path = get_device_id_path(hover_canon)
        if not lidar_path or not hover_path or lidar_path == hover_path:
            LOG.warning("rebind_udev_rules: Không xác định được ID_PATH hợp lệ (LiDAR: %s, Hover: %s)", lidar_path, hover_path)
            return False

        rules_content = (
            f"# Udev rules for Robot Serial Devices (Auto-generated by Robot Kiosk UI)\n"
            f'SUBSYSTEM=="tty", ENV{{ID_PATH}}=="{lidar_path}", MODE:="0777", SYMLINK+="rplidar"\n'
            f'SUBSYSTEM=="tty", ENV{{ID_PATH}}=="{hover_path}", MODE:="0777", SYMLINK+="hoverboard"\n'
        )

        rules_file = "/etc/udev/rules.d/99-robot-serial.rules"
        if os.access("/etc/udev/rules.d", os.W_OK) or os.geteuid() == 0:
            with open(rules_file, "w") as f:
                f.write(rules_content)
        else:
            p = subprocess.Popen(["sudo", "-n", "tee", rules_file], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            _, err = p.communicate(input=rules_content, timeout=3.0)
            if p.returncode != 0:
                LOG.warning("rebind_udev_rules: Ghi file %s thất bại: %s", rules_file, err)
                return False

        # Reload & trigger udev
        cmd_prefix = [] if os.geteuid() == 0 else ["sudo", "-n"]
        subprocess.run(cmd_prefix + ["udevadm", "control", "--reload-rules"], timeout=3.0, check=False)
        subprocess.run(cmd_prefix + ["udevadm", "trigger", "-c", "add", "-s", "tty"], timeout=3.0, check=False)
        time.sleep(0.1)

        success = os.path.exists("/dev/rplidar") and os.path.exists("/dev/hoverboard")
        if success:
            LOG.info("[SerialResolver] Cố định udev thành công: /dev/rplidar -> %s (%s), /dev/hoverboard -> %s (%s)", lidar_canon, lidar_path, hover_canon, hover_path)
        return success
    except Exception as exc:
        LOG.warning("[SerialResolver] Lỗi khi thực hiện rebind_udev_rules: %s", exc)
        return False


def detect_serial_ports(force_swap: bool = False, active_rescan: bool = False) -> Tuple[str, str, str]:
    """Tự động phát hiện cổng ESP32/Hoverboard và LiDAR chuẩn xác như gui_v3 và simulator.

    Khi active_rescan=True hoặc khi phát hiện cắm ngược cổng, hàm sẽ tự động kích hoạt
    rebind_udev_rules để cố định lại /dev/rplidar và /dev/hoverboard cho hệ thống.

    Returns:
        (hoverboard_port, lidar_port, detection_method)
    """
    # 1. Kiểm tra udev aliases trước (nếu không yêu cầu quét sâu active_rescan)
    hb_alias = "/dev/hoverboard" if os.path.exists("/dev/hoverboard") else (
        "/dev/esp32" if os.path.exists("/dev/esp32") else None
    )
    lidar_alias = "/dev/rplidar" if os.path.exists("/dev/rplidar") else (
        "/dev/lidar" if os.path.exists("/dev/lidar") else None
    )

    if not active_rescan and hb_alias and lidar_alias:
        # Tự bảo vệ thông minh: Kiểm tra nếu cổng hoverboard đang phản hồi giao thức RPLiDAR -> Đã bị cắm ngược!
        hb_real = os.path.realpath(hb_alias)
        lidar_real = os.path.realpath(lidar_alias)
        if hb_real != lidar_real and probe_rplidar(hb_real):
            LOG.warning("[SerialResolver] Phát hiện cắm ngược cổng! /dev/hoverboard là RPLiDAR. Tự động sửa lại udev...")
            rebind_udev_rules(lidar_port=hb_real, hoverboard_port=lidar_real)
            if force_swap:
                return lidar_alias, hb_alias, "swap_override"
            return hb_alias, lidar_alias, "udev_rebound"

        if force_swap:
            return lidar_alias, hb_alias, "swap_override"
        return hb_alias, lidar_alias, "udev_alias"

    ports = list_serial_ports()
    if not ports:
        if force_swap:
            return "/dev/ttyUSB1", "/dev/ttyUSB0", "swap_override"
        return "/dev/ttyUSB0", "/dev/ttyUSB1", "fallback_defaults"

    esp_port: str | None = None if active_rescan else hb_alias
    lidar_port: str | None = None if active_rescan else lidar_alias

    if not active_rescan:
        # Nhận diện qua tên symlink / chipset (đặc biệt chip Silicon Labs CP2102 của RPLiDAR trên Jetson)
        for p in ports:
            key = os.path.basename(p).lower()
            if lidar_port is None and any(tag in key for tag in ("rplidar", "slamtec", "lidar", "cp210", "silicon_labs")):
                lidar_port = p
            if esp_port is None and any(tag in key for tag in ("hoverboard", "esp32", "esp")):
                esp_port = p

        # Nếu 1 thiết bị đã xác định rõ và chỉ còn đúng 1 cổng vật lý khác, ghép đôi tức thì (<0.1ms, không I/O probe)
        if (lidar_port is not None) ^ (esp_port is not None):
            known = lidar_port or esp_port
            try:
                known_real = os.path.realpath(known)
            except Exception:
                known_real = known
            remaining_candidates = [
                p for p in ports
                if p != known and (os.path.realpath(p) != known_real if hasattr(os.path, "realpath") else True)
            ]
            if len(remaining_candidates) == 1:
                if lidar_port is not None:
                    esp_port = remaining_candidates[0]
                    LOG.info(f"[SerialResolver] Ghép đôi tức thì cổng còn lại cho Hoverboard/ESP32: {esp_port}")
                else:
                    lidar_port = remaining_candidates[0]
                    LOG.info(f"[SerialResolver] Ghép đôi tức thì cổng còn lại cho RPLiDAR: {lidar_port}")

    # 2. Thăm dò byte thực tế nếu active_rescan=True hoặc chưa xác định được cổng
    if lidar_port is None:
        for p in ports:
            if probe_rplidar(p):
                lidar_port = p
                LOG.info(f"[SerialResolver] Probed RPLiDAR confirmed at {p}")
                break

    # 3. Thăm dò byte thực tế ESP32/Hoverboard (thụ động như gui_v3)
    if esp_port is None:
        for p in ports:
            if p != lidar_port and is_esp32_port(p):
                esp_port = p
                LOG.info(f"[SerialResolver] Probed ESP32/Hoverboard confirmed at {p}")
                break

    # 4. Nhận diện qua tên symlink / chipset nếu probe chưa ra (đặc biệt khi thiết bị offline)
    if lidar_port is None or esp_port is None:
        for p in ports:
            key = os.path.basename(p).lower()
            if lidar_port is None and p != esp_port and any(tag in key for tag in ("rplidar", "slamtec", "lidar")):
                lidar_port = p
            if esp_port is None and p != lidar_port and any(tag in key for tag in ("hoverboard", "esp32", "esp")):
                esp_port = p

    # 5. Nếu 1 thiết bị đã xác định rõ và chỉ còn đúng 1 cổng vật lý khác sau probe, ghép đôi
    if (lidar_port is not None) ^ (esp_port is not None):
        known = lidar_port or esp_port
        try:
            known_real = os.path.realpath(known)
        except Exception:
            known_real = known
        remaining_candidates = [
            p for p in ports
            if p != known and (os.path.realpath(p) != known_real if hasattr(os.path, "realpath") else True)
        ]
        if len(remaining_candidates) == 1:
            if lidar_port is not None:
                esp_port = remaining_candidates[0]
            else:
                lidar_port = remaining_candidates[0]

    # Fallback phân bổ cổng nếu vẫn còn thiếu
    if esp_port is None:
        for p in ports:
            if (p.endswith("ttyUSB0") or p.endswith("ttyACM0")) and p != lidar_port:
                esp_port = p
                break
    if esp_port is None:
        esp_port = ports[0] if ports[0] != lidar_port else (ports[1] if len(ports) > 1 else ports[0])

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

    # Fallback cuối cùng nếu chỉ có 1 cổng
    if lidar_port is None or os.path.realpath(lidar_port) == esp_real:
        lidar_port = "/dev/ttyUSB1" if esp_real != "/dev/ttyUSB1" else "/dev/ttyUSB0"

    # 6. Tự động cố định lại UDEV Rules nếu active_rescan=True
    method = "scan"
    if active_rescan and esp_port and lidar_port:
        rebound = rebind_udev_rules(lidar_port, esp_port)
        if rebound:
            method = "udev_rebound"
            esp_alias = "/dev/hoverboard" if os.path.exists("/dev/hoverboard") else esp_port
            lidar_alias = "/dev/rplidar" if os.path.exists("/dev/rplidar") else lidar_port
            if force_swap:
                return lidar_alias, esp_alias, "swap_override"
            return esp_alias, lidar_alias, method

    if force_swap:
        return lidar_port, esp_port, "swap_override"

    if method == "scan" and (esp_port in ports or lidar_port in ports):
        method = "scan"
    else:
        method = "fallback_defaults"

    return esp_port, lidar_port, method

