"""Unit tests for serial port auto-detection and resolver."""
from pathlib import Path
from unittest.mock import MagicMock, patch

from robot_ui.kiosk.serial_resolver import detect_serial_ports, list_serial_ports, probe_rplidar


def test_list_serial_ports_empty():
    with patch("os.path.isdir", return_value=False), patch("glob.glob", return_value=[]):
        ports = list_serial_ports()
        assert isinstance(ports, list)


def test_detect_serial_ports_udev_alias(tmp_path: Path):
    with patch("os.path.exists") as mock_exists:
        mock_exists.side_effect = lambda p: p in ("/dev/hoverboard", "/dev/rplidar")
        hb, lidar, method = detect_serial_ports()
        assert hb == "/dev/hoverboard"
        assert lidar == "/dev/rplidar"
        assert method == "udev_alias"


def test_detect_serial_ports_probed():
    with patch("os.path.exists", return_value=False), \
         patch("robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"]), \
         patch("robot_ui.kiosk.serial_resolver.probe_rplidar") as mock_probe:
        # ttyUSB1 is lidar, ttyUSB0 is hoverboard
        mock_probe.side_effect = lambda p: p == "/dev/ttyUSB1"
        hb, lidar, method = detect_serial_ports()
        assert lidar == "/dev/ttyUSB1"
        assert hb == "/dev/ttyUSB0"
        assert method == "auto_probed"


def test_detect_serial_ports_fallback():
    with patch("os.path.exists", return_value=False), \
         patch("robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=[]):
        hb, lidar, method = detect_serial_ports()
        assert hb == "/dev/ttyUSB0"
        assert lidar == "/dev/ttyUSB1"
        assert method == "fallback_defaults"
