"""Resolve Astra Pro OpenNI serials to their physically paired UVC RGB node.

The Depth and RGB interfaces are separate USB devices on Astra Pro. Use the
OpenNI serial to locate the Depth USB device, then choose the sole index-0
V4L2 interface under the SAME immediate USB hub as that Depth device.
Never guess by /dev/videoN, enumeration order, or the nearest USB port.

Deliberately fail closed when the hardware does not expose an unambiguous
pairing. This module runs BEFORE launching any Astra Depth nodes.
"""

import ctypes
import ctypes.util
from dataclasses import dataclass
import os
from pathlib import Path
import re


class CameraIdentityError(RuntimeError):
    """A requested serial cannot be resolved to a unique physical RGB camera."""


@dataclass(frozen=True)
class OpenNIDevice:
    serial: str
    uri: str
    usb_bus: int
    usb_device: int


class _OniDeviceInfo(ctypes.Structure):
    # OpenNI2's OniCTypes.h (ONI_MAX_STR == 256).
    _fields_ = [
        ("uri", ctypes.c_char * 256),
        ("vendor", ctypes.c_char * 256),
        ("name", ctypes.c_char * 256),
        ("usbVendorId", ctypes.c_uint16),
        ("usbProductId", ctypes.c_uint16),
    ]


def _openni_library():
    """Load the vendor OpenNI2 library WITH its matching Orbbec plugin.

    The ROS astra_camera package installs these under:
      <astra_camera-prefix>/lib/libOpenNI2.so
      <astra_camera-prefix>/lib/OpenNI2/Drivers/liborbbec.so

    A generic system OpenNI2 may initialize successfully yet enumerate zero
    Astra devices. Never silently fall back to a library without liborbbec.
    """
    candidates = []
    explicit = os.environ.get("OPENNI2_LIBRARY", "").strip()
    if explicit:
        candidates.append(explicit)
    for key in ("OPENNI2_REDIST", "OPENNI2_REDIST64"):
        if os.environ.get(key):
            candidates.append(str(Path(os.environ[key]) / "libOpenNI2.so"))
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix("astra_camera"))
        candidates.append(str(prefix / "lib" / "libOpenNI2.so"))
    except (ImportError, LookupError):
        pass
    # Direct source-tree fallback for unbuilt or partially-sourced workspaces.
    for pkg in (
        Path.home() / "ros2_ws" / "install" / "astra_camera" / "lib",
        Path.home() / "ros2_ws" / "src" / "ros2_astra_camera"
            / "astra_camera" / "openni2_redist" / "arm64",
        Path.home() / "ros2_ws" / "src" / "ros2_astra_camera"
            / "astra_camera" / "openni2_redist" / "x64",
    ):
        candidates.append(str(pkg / "libOpenNI2.so"))
    candidates.extend([
        "/usr/lib/aarch64-linux-gnu/libOpenNI2.so",
        "/usr/local/lib/libOpenNI2.so",
        "/usr/lib/x86_64-linux-gnu/libOpenNI2.so",
        "/opt/ros/humble/lib/libOpenNI2.so",
    ])
    failures = []
    for name in dict.fromkeys(candidates):
        location = Path(name).expanduser()
        if not location.is_file():
            failures.append("{} (not found)".format(location))
            continue
        plugin = location.parent / "OpenNI2" / "Drivers" / "liborbbec.so"
        if not plugin.is_file():
            failures.append("{} (no Orbbec driver at {})".format(location, plugin))
            continue
        try:
            return ctypes.CDLL(str(location.resolve()))
        except OSError as exc:
            failures.append("{} ({})".format(location, exc))
    raise CameraIdentityError(
        "No compatible Orbbec OpenNI2 runtime found. The Python resolver "
        "requires libOpenNI2.so and OpenNI2/Drivers/liborbbec.so from the "
        "same Astra SDK install. Check "
        "'ros2 pkg prefix astra_camera' / 'ls <prefix>/lib/OpenNI2/Drivers' "
        "or set OPENNI2_LIBRARY to the vendor libOpenNI2.so path. "
        "Candidates: {}".format("; ".join(failures[:8]))
    )


def enumerate_openni_devices():
    """Read real OpenNI hardware serial and URI using the C API (no pip binding)."""
    lib = _openni_library()
    lib.oniInitialize.argtypes = [ctypes.c_int]
    lib.oniInitialize.restype = ctypes.c_int
    lib.oniGetDeviceList.argtypes = [
        ctypes.POINTER(ctypes.POINTER(_OniDeviceInfo)),
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.oniGetDeviceList.restype = ctypes.c_int
    lib.oniReleaseDeviceList.argtypes = [ctypes.POINTER(_OniDeviceInfo)]
    lib.oniReleaseDeviceList.restype = ctypes.c_int
    lib.oniDeviceOpen.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_void_p)]
    lib.oniDeviceOpen.restype = ctypes.c_int
    lib.oniDeviceGetProperty.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
    ]
    lib.oniDeviceGetProperty.restype = ctypes.c_int
    lib.oniDeviceClose.argtypes = [ctypes.c_void_p]
    lib.oniDeviceClose.restype = ctypes.c_int
    lib.oniShutdown.argtypes = []
    lib.oniShutdown.restype = None

    # Orbbec SDK in this repo uses OpenNI2 2.3 (ONI_API_VERSION=2003).
    if lib.oniInitialize(2003) != 0:
        raise CameraIdentityError(
            "OpenNI2 could not initialize; check Orbbec drivers, permissions, "
            "or OPENNI2_REDIST for its driver plugins."
        )
    result = []
    try:
        ptr = ctypes.POINTER(_OniDeviceInfo)()
        count = ctypes.c_int(0)
        if lib.oniGetDeviceList(ctypes.byref(ptr), ctypes.byref(count)) != 0:
            raise CameraIdentityError("OpenNI2 failed to enumerate Depth devices.")
        try:
            for index in range(count.value):
                info = ptr[index]
                uri = info.uri.decode("utf-8", errors="replace")
                # Orbbec/OpenNI2 URI: 2bc5/0403@BUS/DEV.
                match = re.search(r"@(\d+)/(\d+)$", uri)
                if match is None:
                    raise CameraIdentityError(
                        "Cannot extract USB bus/device from OpenNI URI {!r}; "
                        "refusing an unverified RGB pairing.".format(uri)
                    )
                handle = ctypes.c_void_p()
                if lib.oniDeviceOpen(info.uri, ctypes.byref(handle)) != 0:
                    raise CameraIdentityError(
                        "Cannot open OpenNI device {!r} to check serial. "
                        "Stop duplicate camera drivers and retry.".format(uri)
                    )
                try:
                    buffer = ctypes.create_string_buffer(256)
                    size = ctypes.c_int(ctypes.sizeof(buffer))
                    # Orbbec astra_camera/src/list_devices_node.cpp uses
                    # openni::OBEXTENSION_ID_SERIALNUMBER = 16, not the
                    # generic DEVICE_PROPERTY_SERIAL_NUMBER = 3.
                    if lib.oniDeviceGetProperty(
                        handle, 16, buffer, ctypes.byref(size)
                    ) != 0:
                        raise CameraIdentityError(
                            "Orbbec device {!r} does not expose its serial "
                            "via OBEXTENSION_ID_SERIALNUMBER (16).".format(uri)
                        )
                    serial = buffer.value.decode("utf-8", errors="replace").strip()
                    if not serial:
                        raise CameraIdentityError(
                            "OpenNI device {!r} returned an empty serial.".format(uri)
                        )
                    result.append(
                        OpenNIDevice(serial, uri, int(match[1]), int(match[2]))
                    )
                finally:
                    lib.oniDeviceClose(handle)
        finally:
            if ptr:
                lib.oniReleaseDeviceList(ptr)
    finally:
        lib.oniShutdown()
    if not result:
        raise CameraIdentityError(
            "Orbbec OpenNI2 found zero Depth devices using {}. "
            "Confirm that all camera launches are stopped, USB devices "
            "are connected, and the Orbbec OpenNI2 plugin can enumerate "
            "them. Run 'ros2 run astra_camera list_devices_node' "
            "to compare.".format(lib._name)
        )
    if len({d.serial for d in result}) != len(result):
        raise CameraIdentityError(
            "Duplicate OpenNI serials were reported; cannot safely bind RGB."
        )
    return result


def _attribute(directory, name):
    try:
        return (directory / name).read_text(encoding="ascii").strip()
    except (OSError, UnicodeError):
        return None


def _usb_device_at(path):
    """Find the nearest actual USB device ancestor, not an interface or a hub."""
    for ancestor in (path.resolve(), *path.resolve().parents):
        if (_attribute(ancestor, "busnum") is not None
                and _attribute(ancestor, "devnum") is not None):
            return ancestor
    return None


def _usb_devices(sys_usb_root):
    """Index real USB devices by (busnum, devnum); ignore interface symlinks."""
    devices = {}
    for candidate in sys_usb_root.glob("*"):
        node = _usb_device_at(candidate)
        if node is None or node != candidate.resolve():
            continue
        try:
            key = (int(_attribute(node, "busnum")),
                   int(_attribute(node, "devnum")))
        except (ValueError, TypeError):
            continue
        if key in devices and devices[key] != node:
            raise CameraIdentityError("Ambiguous USB device number {}".format(key))
        devices[key] = node
    return devices


def _video_nodes(sys_video_root, dev_root):
    """Return only index-0 V4L2 devices; ignore UVC metadata (index > 0)."""
    videos = []
    for video in sorted(sys_video_root.glob("video*")):
        if not re.fullmatch(r"video\d+", video.name):
            continue
        if _attribute(video, "index") != "0":
            continue
        device_link = video / "device"
        if not device_link.exists():
            continue
        usb = _usb_device_at(device_link)
        if usb is not None:
            videos.append((usb, dev_root / video.name))
    return videos


def resolve_video_from_serial(
    ai_serial, required_serials, openni_devices=None,
    sys_usb_root=Path("/sys/bus/usb/devices"),
    sys_video_root=Path("/sys/class/video4linux"),
    dev_root=Path("/dev"),
):
    """Return (absolute /dev/videoN, mapping report), or raise without guessing.

    Every required serial must be uniquely enumerated. Astra Pro Depth and RGB
    must be USB siblings under an internal per-camera hub. If devices share a
    generic external hub instead, they cannot be distinguished; reject them.
    """
    serials = [str(x).strip() for x in required_serials]
    if not serials or not ai_serial or len(set(serials)) != len(serials):
        raise CameraIdentityError("Three camera serials must be nonempty and unique.")
    devices = (
        enumerate_openni_devices()
        if openni_devices is None else list(openni_devices)
    )
    available = {d.serial: d for d in devices}
    missing = sorted(set(serials) - available.keys())
    if missing:
        raise CameraIdentityError(
            "OpenNI serial(s) not found: {}. Found: {}. "
            "Check all three Astra Pro USB Depth interfaces.".format(
                ", ".join(missing), ", ".join(sorted(available))
            )
        )
    by_bus = _usb_devices(Path(sys_usb_root))
    videos = _video_nodes(Path(sys_video_root), Path(dev_root))
    grouped = {}
    report = []
    for serial in serials:
        depth = available[serial]
        usb = by_bus.get((depth.usb_bus, depth.usb_device))
        if usb is None:
            raise CameraIdentityError(
                "Depth serial {} at OpenNI {} has no matching Linux USB bus/device "
                "({},{})".format(serial, depth.uri, depth.usb_bus, depth.usb_device)
            )
        hub = usb.parent
        # Only accept devices from the same immediate USB hub (internal hub
        # of the Astra Pro). Do NOT fall back to a nearby USB device.
        matching = sorted({str(video) for uv, video in videos
                           if uv != usb and uv.parent == hub})
        if len(matching) != 1:
            raise CameraIdentityError(
                "Depth serial {} at {}: expected exactly one RGB video index-0 "
                "sibling under USB hub {}, found {}. Cannot prove RGB identity; "
                "check lsusb -t / v4l2-ctl --list-devices.".format(
                    serial, usb, hub, matching
                )
            )
        video = matching[0]
        if video in grouped:
            raise CameraIdentityError(
                "RGB {} maps to multiple OpenNI serials ({}, {}); "
                "refusing cross-camera pairing.".format(video, grouped[video], serial)
            )
        grouped[video] = serial
        report.append("{} [{}] -> {} [{}]".format(serial, usb.name, video, hub.name))
    match = next((v for v, s in grouped.items() if s == ai_serial), None)
    if not match:
        raise CameraIdentityError("AI serial {} did not resolve.".format(ai_serial))
    return match, report


def _cli():
    import argparse
    parser = argparse.ArgumentParser(
        description="Verify Astra Pro OpenNI serials and their paired RGB UVC devices."
    )
    parser.add_argument("--front-serial", default="17110310855")
    parser.add_argument("--rear-serial", default="17122514440")
    parser.add_argument("--ai-serial", default="17122611013")
    args = parser.parse_args()
    print("Checking three OpenNI serials and the USB RGB pairing...")
    try:
        video, mapping = resolve_video_from_serial(
            ai_serial=args.ai_serial,
            required_serials=[
                args.front_serial, args.rear_serial, args.ai_serial,
            ],
        )
    except CameraIdentityError as exc:
        parser.exit(2, "ERROR: {}\n".format(exc))
    print("Verified serial -> UVC mapping:")
    for item in mapping:
        print("  {}".format(item))
    print("AI RGB device: {}".format(video))
    print("Note: open the ROS AI image to verify physical orientation.")


if __name__ == "__main__":
    _cli()
