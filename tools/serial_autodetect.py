#!/usr/bin/env python3
"""Fail-closed ESP32 + SLAMTEC RPLIDAR serial discovery for the ROS 2 robot.

The default mode only prints a proposed ROS 2 launch command. It never starts
ROS processes, and the probe never sends an ESP32 motor-velocity command.

Caveat: opening CP210x USB serial ports can affect modem-control lines (DTR/RTS)
and may reset a microcontroller or affect an RPLIDAR A1 motor enable circuit.
Perform discovery with the robot safely immobilized and no serial owners.
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import shlex
import sys
import time
from dataclasses import dataclass
from typing import Callable, Sequence


# Matches exactly the packet parsed by src/odom_pub/odom_pub/odom_publisher.py.
NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
TELEMETRY = re.compile(
    rf"V:{NUMBER}\s+A:{NUMBER}\s+RC:\d+/\d+\s+"
    rf"TL:[+-]?\d+\s+TR:[+-]?\d+\s+EL:[+-]?\d+\s+ER:[+-]?\d+\s+"
    rf"B:{NUMBER}\s+T:{NUMBER}"
)
# Official SLAMTEC RPLIDAR GET_INFO request / descriptor (20-byte payload).
LIDAR_GET_INFO = b"\xA5\x50"
LIDAR_DESCRIPTOR = b"\xA5\x5A\x14\x00\x00\x00\x04"
LIDAR_INFO_BYTES = 20


class DiscoveryError(RuntimeError):
    """Refuse to guess a serial port on ambiguous or missing evidence."""


@dataclass(frozen=True)
class IdentifiedPorts:
    esp: str
    lidar: str
    lidar_model: int
    lidar_firmware: str


def candidate_ports(explicit: Sequence[str] | None = None) -> list[str]:
    sources = (list(explicit) if explicit is not None else
               sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')))
    out: list[str] = []
    unique_devices: set[str] = set()
    for port in sources:
        real = os.path.realpath(port)
        if real not in unique_devices:
            unique_devices.add(real)
            out.append(port)
    return out


def is_esp_telemetry(line: bytes) -> bool:
    try:
        value = line.decode('ascii', errors='strict').strip()
    except UnicodeDecodeError:
        return False
    return TELEMETRY.fullmatch(value) is not None


def serial_open(port: str, baudrate: int):
    """PySerial default DTR=asserted; never call DTR-low motor-enable here.

    Exclusive opens prevent concurrently using ports on Linux. The driver itself
    may briefly change modem-control lines as the port is opened/closed.
    """
    import serial  # type: ignore[import-not-found]
    try:
        return serial.Serial(port=port, baudrate=baudrate, timeout=0.12,
                             write_timeout=0.4, exclusive=True)
    except TypeError as exc:
        raise DiscoveryError('pyserial with exclusive=True support is required') from exc


def passive_esp_probe(port: str, baudrate: int, seconds: float,
                      open_port: Callable = serial_open) -> bool:
    """Read-only inspection: absolutely no serial.write and no buffer reset."""
    evidence = 0
    deadline = time.monotonic() + seconds
    remainder = b''
    with open_port(port, baudrate) as ser:
        while time.monotonic() < deadline:
            payload = ser.read(256)
            if not payload:
                continue
            remainder += payload
            if len(remainder) > 4096:
                remainder = remainder[-4096:]
            while b'\n' in remainder:
                line, remainder = remainder.split(b'\n', 1)
                if is_esp_telemetry(line):
                    evidence += 1
                    if evidence >= 2:
                        return True
    return False


def read_exact(ser, wanted: int, deadline: float) -> bytes:
    out = bytearray()
    while len(out) < wanted and time.monotonic() < deadline:
        part = ser.read(wanted - len(out))
        if part:
            out.extend(part)
    return bytes(out)


def lidar_info_probe(port: str, baudrate: int, seconds: float,
                     open_port: Callable = serial_open) -> tuple[int, str] | None:
    """Only issue GET_INFO, not scan/start/stop/motor commands."""
    with open_port(port, baudrate) as ser:
        # After ESP has been positively identified on the OTHER port.
        # Clear stale received samples only; no data is sent by this call.
        ser.reset_input_buffer()
        ser.write(LIDAR_GET_INFO)
        ser.flush()
        deadline = time.monotonic() + seconds
        descriptor = read_exact(ser, len(LIDAR_DESCRIPTOR), deadline)
        if descriptor != LIDAR_DESCRIPTOR:
            return None
        payload = read_exact(ser, LIDAR_INFO_BYTES, deadline)
        if len(payload) != LIDAR_INFO_BYTES:
            return None
        model, minor, major, _hardware = payload[:4]
        return model, f'{major}.{minor}'


def identify(ports: Sequence[str], esp_baud: int = 115200,
             lidar_baud: int = 115200, esp_seconds: float = 3.0,
             lidar_seconds: float = 2.0,
             esp_probe: Callable = passive_esp_probe,
             lidar_probe: Callable = lidar_info_probe) -> IdentifiedPorts:
    ports = candidate_ports(ports)
    # Fail closed. Additional USB serial peripherals require an explicit, reviewed
    # candidate pair; we never probe unknown connected motor controllers.
    if len(ports) != 2:
        raise DiscoveryError(
            f'Exactly two distinct candidate ports are required, found {len(ports)}: {ports}. '
            'Specify --ports with the exact expected pair when other adapters are connected.'
        )
    verified_esp = []
    for port in ports:
        try:
            if esp_probe(port, esp_baud, esp_seconds):
                verified_esp.append(port)
        except (OSError, ValueError, RuntimeError) as exc:
            raise DiscoveryError(f'ESP passive probe failed on {port}: {exc}') from exc
    if len(verified_esp) != 1:
        raise DiscoveryError(
            f'Expected exactly one ESP32 with >=2 valid V: telemetry frames; '
            f'found {len(verified_esp)}. No LiDAR command was sent.'
        )
    esp = verified_esp[0]
    lidar = next(p for p in ports if p != esp)
    try:
        info = lidar_probe(lidar, lidar_baud, lidar_seconds)
    except (OSError, ValueError, RuntimeError) as exc:
        raise DiscoveryError(f'RPLIDAR GET_INFO failed on {lidar}: {exc}') from exc
    if info is None:
        raise DiscoveryError(
            f'{lidar} did not return the expected RPLIDAR GET_INFO descriptor and payload; '
            'refusing to assume it is LiDAR.'
        )
    return IdentifiedPorts(esp=esp, lidar=lidar,
                           lidar_model=info[0], lidar_firmware=info[1])


def launch_argv(ports: IdentifiedPorts, mode: str,
                esp_baud: int, lidar_baud: int) -> list[str]:
    if mode not in ('esp', 'rf2o'):
        raise DiscoveryError('Auto-detection only supports esp or rf2o; not hoverboard.')
    return ['ros2', 'launch', 'mo_hinh', 'real_odom.launch.py',
            f'odom_source:={mode}', f'esp_port:={ports.esp}',
            f'lidar_port:={ports.lidar}', f'esp_baudrate:={esp_baud}',
            f'lidar_baudrate:={lidar_baud}', 'use_sim_time:=false']


def cli(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ports', nargs=2, metavar=('PORT_A', 'PORT_B'),
                        help='Explicitly scope discovery to these two devices.')
    parser.add_argument('--esp-baudrate', type=int, default=115200)
    parser.add_argument('--lidar-baudrate', type=int, default=115200)
    parser.add_argument('--esp-seconds', type=float, default=3.0)
    parser.add_argument('--lidar-seconds', type=float, default=2.0)
    parser.add_argument('--mode', choices=('esp', 'rf2o'), default='esp')
    parser.add_argument('--launch', action='store_true',
                        help='Explicitly start hardware ROS 2 nodes AFTER discovery (may drive motors).')
    args = parser.parse_args(argv)
    if args.esp_baudrate <= 0 or args.lidar_baudrate <= 0 or not (0.3 <= args.esp_seconds <= 20) or not (0.3 <= args.lidar_seconds <= 20):
        parser.error('Baudrate must be positive; probe timeouts must be within 0.3..20 seconds.')
    try:
        import serial  # type: ignore[import-not-found] # noqa: F401
    except ImportError:
        print('ERROR: pyserial missing; install python3-serial (sudo apt install python3-serial).', file=sys.stderr)
        return 2
    try:
        found = identify(candidate_ports(args.ports), args.esp_baudrate,
                         args.lidar_baudrate, args.esp_seconds, args.lidar_seconds)
    except DiscoveryError as exc:
        print(f'DETECTION FAILED: {exc}', file=sys.stderr)
        return 1
    cmd = launch_argv(found, args.mode, args.esp_baudrate, args.lidar_baudrate)
    print(f'ESP32:   {found.esp} (passive telemetry confirmed)')
    print(f'RPLIDAR: {found.lidar} (GET_INFO confirmed, model={found.lidar_model}, fw={found.lidar_firmware})')
    print('ROS 2 command:')
    print(shlex.join(cmd))
    if not args.launch:
        print('DRY RUN: no ROS 2 nodes started; no motor velocity commands sent.')
        return 0
    print('WARNING: --launch starts hardware nodes; the ESP32 bridge can publish motor commands.', file=sys.stderr)
    sys.stdout.flush()
    os.execvp(cmd[0], cmd)
    return 0


if __name__ == '__main__':
    raise SystemExit(cli())
