"""Resolve Astra Pro AI Camera OpenNI serial to its PulseAudio microphone source."""
from __future__ import annotations

import importlib.util
import logging
import os
import subprocess
from pathlib import Path
from typing import Any

LOG = logging.getLogger(__name__)

DEFAULT_AI_CAMERA_SERIAL = "17122611013"


def _load_astra_serial_resolver() -> Any | None:
    """Dynamically load _astra_serial_resolver from mo_hinh launch or workspace paths."""
    # 1. Try ament package share if in ROS 2 environment
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("mo_hinh"))
        cand = share_dir / "launch" / "_astra_serial_resolver.py"
        if cand.is_file():
            spec = importlib.util.spec_from_file_location("_astra_serial_resolver", str(cand))
            if spec and spec.loader:
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                return mod
    except Exception:
        pass

    # 2. Try common relative and workspace paths
    search_paths = [
        Path(__file__).resolve().parents[4] / "mo_hinh" / "launch" / "_astra_serial_resolver.py",
        Path(__file__).resolve().parents[5] / "src" / "mo_hinh" / "launch" / "_astra_serial_resolver.py",
        Path.home() / "ros2_ws" / "src" / "mo_hinh" / "launch" / "_astra_serial_resolver.py",
        Path.home() / "ros2_ws" / "install" / "mo_hinh" / "share" / "mo_hinh" / "launch" / "_astra_serial_resolver.py",
    ]
    for cand in search_paths:
        if cand.is_file():
            try:
                spec = importlib.util.spec_from_file_location("_astra_serial_resolver", str(cand))
                if spec and spec.loader:
                    mod = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(mod)
                    return mod
            except Exception as exc:
                LOG.debug("Failed loading %s: %s", cand, exc)
    return None


def find_pulseaudio_source_for_usb_node(target_node: Path, pactl_output: str | None = None) -> str | None:
    """Match a physical USB device node to its corresponding PulseAudio source name."""
    if pactl_output is None:
        try:
            res = subprocess.run(
                ["pactl", "list", "sources"],
                capture_output=True,
                text=True,
                check=False,
                timeout=2.0,
            )
            if res.returncode != 0:
                return None
            pactl_output = res.stdout
        except Exception as exc:
            LOG.debug("Failed to query pactl list sources: %s", exc)
            return None

    target_str = str(target_node.resolve())
    target_rel = target_str.removeprefix("/sys")
    usb_port_id = target_node.name

    current_source = None
    for line in pactl_output.splitlines():
        line = line.strip()
        if line.startswith("Name: "):
            current_source = line.split("Name: ", 1)[1].strip()
        elif "sysfs.path = " in line and current_source:
            sysfs = line.split("sysfs.path = ", 1)[1].strip('"')
            # Match either full relative sysfs path or port segment
            if target_rel in sysfs or f"/{usb_port_id}/" in sysfs or f"/{usb_port_id}:" in sysfs:
                return current_source
    return None


def resolve_ai_camera_mic_source(
    target_serial: str = DEFAULT_AI_CAMERA_SERIAL,
    sys_usb_root: Path | None = None,
) -> str | None:
    """Find the PulseAudio source name matching the physical Astra AI camera serial.

    1. Enumerate OpenNI devices to find the Depth USB device with `target_serial`.
    2. Identify its immediate physical USB port / device node in sysfs.
    3. Query PulseAudio sources via `pactl list sources` and find the ALSA source
       whose `sysfs.path` belongs to the SAME physical USB device.
    """
    resolver_mod = _load_astra_serial_resolver()
    if resolver_mod is None:
        LOG.debug("Astra serial resolver module could not be loaded.")
        return None

    try:
        devs = resolver_mod.enumerate_openni_devices()
        root = sys_usb_root or Path("/sys/bus/usb/devices")
        usb_map = resolver_mod._usb_devices(root)
        target_dev = next((d for d in devs if str(d.serial).strip() == str(target_serial).strip()), None)
        if target_dev is None:
            LOG.info("AI camera serial %s not found among connected OpenNI devices.", target_serial)
            return None

        target_node = usb_map.get((target_dev.usb_bus, target_dev.usb_device))
        if target_node is None:
            LOG.warning("Could not map OpenNI device %s to a physical USB sysfs node.", target_dev)
            return None

        source_name = find_pulseaudio_source_for_usb_node(target_node)
        if source_name:
            LOG.info(
                "Resolved AI Camera (serial=%s, usb_port=%s) -> PulseAudio source: %s",
                target_serial,
                target_node.name,
                source_name,
            )
            return source_name
    except Exception as exc:
        LOG.warning("Error resolving AI camera microphone: %s", exc)

    return None


def sync_system_default_mic(source_name: str) -> bool:
    """Synchronize PulseAudio default source so all tools (including wakeword) use it."""
    if not source_name or source_name in ("@DEFAULT_SOURCE@", "auto", "default"):
        return False
    try:
        subprocess.run(
            ["pactl", "set-default-source", source_name],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=1.5,
            check=False,
        )
        return True
    except Exception as exc:
        LOG.debug("Failed to set default source via pactl: %s", exc)
        return False


def get_effective_mic_source(configured_source: str | None = None) -> str:
    """Return the active microphone source name for Kiosk.

    - If an explicit hardware source is configured (not empty or 'auto'), returns it.
    - If set to 'auto' or empty, attempts dynamic resolution of the AI camera serial.
    - Falls back to '@DEFAULT_SOURCE@' if resolution fails.
    """
    source = (configured_source or os.getenv("KIOSK_MIC_SOURCE", "")).strip()
    if source and source not in ("auto", "default", "@DEFAULT_SOURCE@"):
        return source

    target_serial = os.getenv("AI_CAMERA_SERIAL", DEFAULT_AI_CAMERA_SERIAL).strip()
    resolved = resolve_ai_camera_mic_source(target_serial)
    if resolved:
        sync_system_default_mic(resolved)
        return resolved

    return "@DEFAULT_SOURCE@"


if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.INFO)
    serial = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_AI_CAMERA_SERIAL
    source = resolve_ai_camera_mic_source(serial)
    if source:
        sync_system_default_mic(source)
        print(f"Successfully resolved and set AI Camera ({serial}) microphone: {source}")
    else:
        print(f"Could not resolve microphone for serial: {serial} (fallback to system default)")
