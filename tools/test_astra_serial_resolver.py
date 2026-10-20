#!/usr/bin/env python3
"""Offline USB topology tests for serial->RGB pairing (no ROS or camera needed)."""

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/mo_hinh/launch/_astra_serial_resolver.py"
spec = importlib.util.spec_from_file_location("_astra_serial_resolver", SOURCE)
module = importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class FakeThreeAstraUSB:
    """OpenNI and UVC are siblings below each camera's internal USB hub."""

    def __init__(self, root):
        self.root = Path(root)
        self.usb = self.root / "bus" / "usb" / "devices"
        self.videos = self.root / "class" / "video4linux"
        self.dev = self.root / "dev"
        self.usb.mkdir(parents=True)
        self.videos.mkdir(parents=True)
        self.dev.mkdir()
        self.depths = []
        self.rgb = []
        self.devices = []
        self.video_by_serial = {}
        for i, serial in enumerate(("17110310855", "17122514440",
                                     "17122611013"), start=1):
            hub = self.root / "hardware" / ("internal-hub-" + str(i))
            hub.mkdir(parents=True)
            depth = hub / ("depth-" + str(i))
            rgb = hub / ("rgb-" + str(i))
            depth.mkdir()
            rgb.mkdir()
            (depth / "busnum").write_text("1")
            (depth / "devnum").write_text(str(i * 2))
            (rgb / "busnum").write_text("1")
            (rgb / "devnum").write_text(str(i * 2 + 1))
            (self.usb / ("depth" + str(i))).symlink_to(depth)
            (self.usb / ("rgb" + str(i))).symlink_to(rgb)
            face = rgb / ("rgb{}:1.0".format(i))
            face.mkdir()
            # Intentionally nonsequential /dev/video indices to ensure that
            # numeric order is never used to choose the RGB camera.
            index = 9 - i * 2
            video = self.videos / ("video" + str(index))
            video.mkdir()
            (video / "device").symlink_to(face)
            (video / "index").write_text("0")
            self.depths.append(depth)
            self.rgb.append(rgb)
            self.devices.append(module.OpenNIDevice(
                serial=serial,
                uri="2bc5/0403@1/{}".format(i * 2),
                usb_bus=1,
                usb_device=i * 2,
            ))
            self.video_by_serial[serial] = str(self.dev / ("video" + str(index)))

    def resolve(self, serial="17122611013", serials=None, devices=None):
        return module.resolve_video_from_serial(
            serial,
            serials or [d.serial for d in self.devices],
            openni_devices=self.devices if devices is None else devices,
            sys_usb_root=self.usb,
            sys_video_root=self.videos,
            dev_root=self.dev,
        )


class SerialRGBResolverTests(unittest.TestCase):
    def test_exact_pairing_for_all_three_serials(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            for serial in (d.serial for d in fixture.devices):
                path, report = fixture.resolve(serial)
                self.assertEqual(path, fixture.video_by_serial[serial])
                self.assertEqual(len(report), 3)

    def test_reject_duplicate_role_serials(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            with self.assertRaisesRegex(module.CameraIdentityError, "unique"):
                fixture.resolve(serials=["17122611013"] * 3)

    def test_reject_unplugged_serial(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            with self.assertRaisesRegex(module.CameraIdentityError, "not found"):
                fixture.resolve(devices=fixture.devices[:2])

    def test_reject_shared_usb_hub_without_unique_rgb(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            # Move a second RGB interface under the same camera hub: ambiguous.
            first_rgb = fixture.rgb[0]
            second_rgb = first_rgb.parent / "extra_rgb"
            second_rgb.mkdir()
            (second_rgb / "busnum").write_text("1")
            (second_rgb / "devnum").write_text("39")
            extra = fixture.videos / "video44"
            extra.mkdir()
            (extra / "index").write_text("0")
            (extra / "device").symlink_to(second_rgb)
            with self.assertRaisesRegex(module.CameraIdentityError,
                                        "exactly one RGB"):
                fixture.resolve()

    def test_ignore_metadata_video_nodes(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            metadata = fixture.videos / "video42"
            metadata.mkdir()
            (metadata / "index").write_text("1")
            (metadata / "device").symlink_to(fixture.rgb[2])
            path, _ = fixture.resolve()
            self.assertEqual(path, fixture.video_by_serial["17122611013"])

    def test_missing_usb_depth_mapping_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = FakeThreeAstraUSB(temp)
            changed = list(fixture.devices)
            changed[2] = module.OpenNIDevice(
                serial=changed[2].serial,
                uri="2bc5/0403@1/87",
                usb_bus=1,
                usb_device=87,
            )
            with self.assertRaisesRegex(module.CameraIdentityError,
                                        "no matching Linux USB"):
                fixture.resolve(devices=changed)

    def test_launch_uses_all_three_serials_no_fixed_usb_path(self):
        launch = (
            ROOT / "src/mo_hinh/launch/three_astra.launch.py"
        ).read_text(encoding="utf-8")
        for item in ("front_serial", "rear_serial", "ai_serial",
                     "resolve_video_from_serial",
                     "launch_verified_cameras", "namespace='ai_camera/color'"):
            self.assertIn(item, launch)
        self.assertNotIn("ai_video_device", launch)
        self.assertNotIn("platform-3610000.usb", launch)


if __name__ == "__main__":
    unittest.main()
