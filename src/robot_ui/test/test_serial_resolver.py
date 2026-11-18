"""Unit tests for serial port auto-detection, resolver, and force_swap."""
from unittest.mock import MagicMock, patch

from robot_ui.kiosk.serial_resolver import (
    detect_serial_ports,
    is_esp32_port,
    list_serial_ports,
)


def test_list_serial_ports_empty():
    with patch("glob.glob", return_value=[]):
        ports = list_serial_ports()
        assert isinstance(ports, list)
        assert len(ports) == 0


def test_list_serial_ports_dedup_and_priority():
    def fake_realpath(p: str) -> str:
        if "by-id" in p:
            return "/dev/ttyUSB1"
        return p

    with patch(
        "glob.glob",
        side_effect=lambda pattern: (
            ["/dev/serial/by-id/usb-Silicon_Labs_CP2102-port0"]
            if "by-id" in pattern
            else (["/dev/ttyUSB0", "/dev/ttyUSB1"] if "ttyUSB" in pattern else [])
        ),
    ), patch("os.path.realpath", side_effect=fake_realpath):
        ports = list_serial_ports()
        # by-id symlink comes first and deduplicates /dev/ttyUSB1
        assert "/dev/serial/by-id/usb-Silicon_Labs_CP2102-port0" in ports
        assert "/dev/ttyUSB0" in ports
        assert "/dev/ttyUSB1" not in ports


def test_is_esp32_port_telemetry():
    fake_ser = MagicMock()
    fake_ser.read.return_value = b"\x00\x01\xcd\xab\x10"
    with patch("serial.Serial", return_value=fake_ser):
        assert is_esp32_port("/dev/ttyUSB0") is True


def test_is_esp32_port_bootloader():
    fake_ser = MagicMock()
    fake_ser.read.return_value = b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\nets Jun"
    with patch("serial.Serial", return_value=fake_ser):
        assert is_esp32_port("/dev/ttyUSB0") is True


def test_is_esp32_port_not_found():
    fake_ser = MagicMock()
    fake_ser.read.return_value = b"\x00\x00\x00"
    with patch("serial.Serial", return_value=fake_ser):
        assert is_esp32_port("/dev/ttyUSB0") is False


def test_detect_serial_ports_udev_alias():
    with patch("os.path.exists") as mock_exists:
        mock_exists.side_effect = lambda p: p in ("/dev/hoverboard", "/dev/rplidar")
        hb, lidar, method = detect_serial_ports()
        assert hb == "/dev/hoverboard"
        assert lidar == "/dev/rplidar"
        assert method == "udev_alias"

        # Force swap with udev alias
        hb_s, lidar_s, method_s = detect_serial_ports(force_swap=True)
        assert hb_s == "/dev/rplidar"
        assert lidar_s == "/dev/hoverboard"
        assert method_s == "swap_override"


def test_detect_serial_ports_cp2102_and_esp_symlinks():
    ports = [
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0001-port0",
        "/dev/serial/by-id/usb-Espressif_ESP32_Device-port0",
    ]
    with patch("os.path.exists", return_value=False), patch(
        "robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=ports
    ), patch("os.path.realpath", side_effect=lambda p: f"/dev/ttyUSB{ports.index(p)}"):
        hb, lidar, method = detect_serial_ports()
        assert "ESP32" in hb
        assert "CP2102" in lidar
        assert method == "scan"

        hb_swapped, lidar_swapped, method_swapped = detect_serial_ports(force_swap=True)
        assert "CP2102" in hb_swapped
        assert "ESP32" in lidar_swapped
        assert method_swapped == "swap_override"


def test_detect_serial_ports_probed():
    with patch("os.path.exists", return_value=False), patch(
        "robot_ui.kiosk.serial_resolver.list_serial_ports",
        return_value=["/dev/ttyUSB0", "/dev/ttyUSB1"],
    ), patch("robot_ui.kiosk.serial_resolver.is_esp32_port", return_value=False), patch(
        "robot_ui.kiosk.serial_resolver.probe_rplidar"
    ) as mock_probe:
        # ttyUSB1 is lidar, ttyUSB0 is hoverboard
        mock_probe.side_effect = lambda p: p == "/dev/ttyUSB1"
        hb, lidar, method = detect_serial_ports()
        assert lidar == "/dev/ttyUSB1"
        assert hb == "/dev/ttyUSB0"
        assert method == "scan"


def test_detect_serial_ports_fallback():
    with patch("os.path.exists", return_value=False), patch(
        "robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=[]
    ):
        hb, lidar, method = detect_serial_ports()
        assert hb == "/dev/ttyUSB0"
        assert lidar == "/dev/ttyUSB1"
        assert method == "fallback_defaults"

        hb_s, lidar_s, method_s = detect_serial_ports(force_swap=True)
        assert hb_s == "/dev/ttyUSB1"
        assert lidar_s == "/dev/ttyUSB0"
        assert method_s == "swap_override"


def test_ros_node_swap_serial_ports():
    from robot_ui.kiosk.ros_node import KioskRosNode

    with patch("rclpy.node.Node.__init__"), patch(
        "robot_ui.kiosk.ros_node.KioskSignals"
    ), patch("robot_ui.kiosk.ros_node.tf2_ros.TransformListener"), patch(
        "robot_ui.kiosk.ros_node.tf2_ros.Buffer"
    ):
        node = KioskRosNode.__new__(KioskRosNode)
        node._force_swap_ports = False
        node._logger = MagicMock()
        node.get_logger = MagicMock(return_value=node._logger)
        node.signals = MagicMock()
        node._layer_log_buffers = {"layer1": []}
        node._canonical_layer_map = {}
        node._record_layer_log = MagicMock()

        with patch(
            "robot_ui.kiosk.serial_resolver.detect_serial_ports"
        ) as mock_detect:
            mock_detect.side_effect = [
                ("/dev/ttyUSB1", "/dev/ttyUSB0", "swap_override"),
                ("/dev/ttyUSB0", "/dev/ttyUSB1", "scan"),
            ]
            esp, lidar, src = node.swap_serial_ports()
            assert node._force_swap_ports is True
            assert esp == "/dev/ttyUSB1"
            assert lidar == "/dev/ttyUSB0"

            esp2, lidar2, src2 = node.swap_serial_ports()
            assert node._force_swap_ports is False
            assert esp2 == "/dev/ttyUSB0"
            assert lidar2 == "/dev/ttyUSB1"


def test_detect_serial_ports_fast_pairing_without_probing():
    """Kiểm tra tối ưu hóa ghép đôi tức thì khi có 1 thiết bị định danh và 1 cổng duy nhất còn lại."""
    ports = [
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0001-port0",
        "/dev/ttyUSB1",
    ]
    with patch("os.path.exists", return_value=False), patch(
        "robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=ports
    ), patch("os.path.realpath", side_effect=lambda p: p), patch(
        "robot_ui.kiosk.serial_resolver.is_esp32_port"
    ) as mock_is_esp32, patch(
        "robot_ui.kiosk.serial_resolver.probe_rplidar"
    ) as mock_probe_lidar:
        hb, lidar, method = detect_serial_ports()
        assert "CP2102" in lidar
        assert hb == "/dev/ttyUSB1"
        assert method == "scan"
        # Tuyệt đối không gọi hàm probe gây delay
        mock_is_esp32.assert_not_called()
        mock_probe_lidar.assert_not_called()


def test_detect_serial_ports_inverted_alias_rebind():
    """Kiểm tra khi bị cắm ngược cổng (/dev/hoverboard là RPLiDAR), hệ thống tự đảo và rebind udev."""
    with patch("os.path.exists", return_value=True), patch(
        "os.path.realpath", side_effect=lambda p: "/dev/ttyUSB0" if "hoverboard" in p else "/dev/ttyUSB1"
    ), patch("robot_ui.kiosk.serial_resolver.probe_rplidar", return_value=True) as mock_probe, patch(
        "robot_ui.kiosk.serial_resolver.rebind_udev_rules", return_value=True
    ) as mock_rebind:
        hb, lidar, method = detect_serial_ports()
        mock_probe.assert_called_with("/dev/ttyUSB0")
        mock_rebind.assert_called_once_with(lidar_port="/dev/ttyUSB0", hoverboard_port="/dev/ttyUSB1")
        assert hb == "/dev/hoverboard"
        assert lidar == "/dev/rplidar"
        assert method == "udev_rebound"


def test_detect_serial_ports_active_rescan_rebind():
    """Kiểm tra active_rescan=True thăm dò byte và tự động gọi rebind_udev_rules."""
    ports = ["/dev/ttyUSB0", "/dev/ttyUSB1"]
    with patch("os.path.exists", side_effect=lambda p: p in ("/dev/hoverboard", "/dev/rplidar")), patch(
        "robot_ui.kiosk.serial_resolver.list_serial_ports", return_value=ports
    ), patch("os.path.realpath", side_effect=lambda p: p), patch(
        "robot_ui.kiosk.serial_resolver.probe_rplidar", side_effect=lambda p: p == "/dev/ttyUSB1"
    ), patch(
        "robot_ui.kiosk.serial_resolver.is_esp32_port", side_effect=lambda p: p == "/dev/ttyUSB0"
    ), patch(
        "robot_ui.kiosk.serial_resolver.rebind_udev_rules", return_value=True
    ) as mock_rebind:
        hb, lidar, method = detect_serial_ports(active_rescan=True)
        mock_rebind.assert_called_once_with("/dev/ttyUSB1", "/dev/ttyUSB0")
        assert hb == "/dev/hoverboard"
        assert lidar == "/dev/rplidar"
        assert method == "udev_rebound"


def test_rebind_udev_rules_helper_success():
    """Kiểm tra hàm rebind_udev_rules khi helper script thành công."""
    from robot_ui.kiosk.serial_resolver import rebind_udev_rules
    with patch("os.path.exists", return_value=True), patch(
        "os.access", return_value=True
    ), patch("subprocess.run") as mock_run:
        mock_run.return_value.returncode = 0
        success = rebind_udev_rules("/dev/ttyUSB1", "/dev/ttyUSB0")
        assert success is True
        mock_run.assert_called()


