#!/usr/bin/env python3
"""Pure unit tests; never opens a real serial port or transmits bytes to hardware."""
import importlib.util
import pathlib
import unittest
from unittest.mock import patch

SOURCE = pathlib.Path(__file__).with_name('serial_autodetect.py')
spec = importlib.util.spec_from_file_location('serial_autodetect', SOURCE)
module = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = module
spec.loader.exec_module(module)

TELEM = b'V:0.000 A:0.001 RC:1/2 TL:1 TR:-2 EL:123 ER:456 B:12.5 T:30.0\n'
HEADER = module.LIDAR_DESCRIPTOR
INFO = bytes([24, 8, 1, 3]) + bytes(range(1, 17))


class FakeSerial:
    def __init__(self, incoming=b''):
        self.incoming = bytearray(incoming)
        self.writes = []
        self.flushed = False
        self.cleared = False

    def __enter__(self): return self
    def __exit__(self, *_): return False
    def read(self, n):
        if not self.incoming:
            return b''
        out = bytes(self.incoming[:n])
        del self.incoming[:n]
        return out
    def write(self, msg):
        self.writes.append(msg)
        return len(msg)
    def flush(self): self.flushed = True
    def reset_input_buffer(self): self.cleared = True


class Tests(unittest.TestCase):
    def test_telemetry_matching(self):
        self.assertTrue(module.is_esp_telemetry(TELEM))
        self.assertFalse(module.is_esp_telemetry(b'E,123,456\n'))
        self.assertFalse(module.is_esp_telemetry(b'V:10 A:1\n'))
        self.assertFalse(module.is_esp_telemetry(b'garbage\n'))

    def test_passive_probe_never_writes(self):
        serial = FakeSerial(TELEM * 2)
        self.assertTrue(module.passive_esp_probe('/dev/ttyUSB1', 115200, 0.02,
                        open_port=lambda *args: serial))
        self.assertEqual(serial.writes, [])
        self.assertFalse(serial.cleared)

    def test_lidar_get_info_is_only_write(self):
        serial = FakeSerial(HEADER + INFO)
        serial.reset_input_buffer = lambda: None
        actual = module.lidar_info_probe('/dev/ttyUSB0', 115200, 0.02,
                                         open_port=lambda *args: serial)
        self.assertEqual(actual, (24, '1.8'))
        self.assertEqual(serial.writes, [b'\xa5\x50'])

    def test_lidar_bad_descriptor_is_rejected(self):
        serial = FakeSerial(b'wrong!!' + INFO)
        serial.reset_input_buffer = lambda: None
        self.assertIsNone(module.lidar_info_probe('/dev/ttyUSB0', 115200, 0.02,
                                                  open_port=lambda *args: serial))

    def test_swapped_ports_and_fail_closed(self):
        for esp in ('/dev/ttyUSB0', '/dev/ttyUSB1'):
            lidar = '/dev/ttyUSB1' if esp.endswith('0') else '/dev/ttyUSB0'
            actions = []
            def ep(p, *_):
                actions.append(('read', p))
                return p == esp
            def lp(p, *_):
                actions.append(('info', p))
                return (24, '1.8') if p == lidar else None
            result = module.identify(['/dev/ttyUSB0', '/dev/ttyUSB1'],
                                     esp_probe=ep, lidar_probe=lp)
            self.assertEqual((result.esp, result.lidar), (esp, lidar))
            self.assertEqual(actions[-1], ('info', lidar))

        actions = []
        with self.assertRaises(module.DiscoveryError):
            module.identify(['/dev/ttyUSB0', '/dev/ttyUSB1'],
                            esp_probe=lambda *_: False,
                            lidar_probe=lambda *x: actions.append(x))
        self.assertEqual(actions, [])

    def test_duplicate_and_too_many_ports(self):
        with self.assertRaises(module.DiscoveryError):
            module.identify(['/dev/ttyUSB0', '/dev/ttyUSB0'])
        with self.assertRaises(module.DiscoveryError):
            module.identify(['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2'])

    def test_command_never_defaults_hoverboard(self):
        found = module.IdentifiedPorts('/dev/ttyUSB1', '/dev/ttyUSB0', 24, '1.8')
        cmd = module.launch_argv(found, 'esp', 115200, 115200)
        self.assertIn('odom_source:=esp', cmd)
        self.assertIn('esp_port:=/dev/ttyUSB1', cmd)
        self.assertIn('lidar_port:=/dev/ttyUSB0', cmd)
        with self.assertRaises(module.DiscoveryError):
            module.launch_argv(found, 'hoverboard', 115200, 115200)


if __name__ == '__main__':
    unittest.main()
