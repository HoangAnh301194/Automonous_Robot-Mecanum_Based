"""Unit tests for Astra AI Camera microphone auto-resolver."""
from __future__ import annotations

import subprocess
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest
from robot_ui.kiosk.camera_mic_resolver import (
    find_pulseaudio_source_for_usb_node,
    get_effective_mic_source,
    resolve_ai_camera_mic_source,
    sync_system_default_mic,
)

SAMPLE_PACTL_OUTPUT = """
Source #1
	State: SUSPENDED
	Name: alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo
	Description: Astra Pro Analog Stereo
	Driver: module-alsa-card.c
	Sample Specification: s16le 2ch 16000Hz
	Channel Map: front-left,front-right
	Owner Module: 8
	Properties:
		alsa.card = "2"
		alsa.card_name = "ASTRA Pro"
		sysfs.path = "/devices/platform/bus@0/3610000.usb/usb1/1-1/1-1.2/1-1.2.2/1-1.2.2:1.1/sound/card2"
		device.bus_path = "platform-3610000.usb-usb-0:1.2.2:1.1"

Source #2
	State: RUNNING
	Name: alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo.2
	Description: Astra Pro Analog Stereo
	Driver: module-alsa-card.c
	Sample Specification: s16le 2ch 16000Hz
	Channel Map: front-left,front-right
	Owner Module: 9
	Properties:
		alsa.card = "3"
		alsa.card_name = "ASTRA Pro"
		sysfs.path = "/devices/platform/bus@0/3610000.usb/usb1/1-1/1-1.3/1-1.3.2/1-1.3.2:1.1/sound/card3"
		device.bus_path = "platform-3610000.usb-usb-0:1.3.2:1.1"
"""


def test_find_pulseaudio_source_matching_usb_nodes():
    # 1. Match AI camera on port 1-1.3.2
    node_ai = Path("/sys/devices/platform/bus@0/3610000.usb/usb1/1-1/1-1.3/1-1.3.2")
    source_ai = find_pulseaudio_source_for_usb_node(node_ai, pactl_output=SAMPLE_PACTL_OUTPUT)
    assert source_ai == "alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo.2"

    # 2. Match Front camera on port 1-1.2.2
    node_front = Path("/sys/devices/platform/bus@0/3610000.usb/usb1/1-1/1-1.2/1-1.2.2")
    source_front = find_pulseaudio_source_for_usb_node(node_front, pactl_output=SAMPLE_PACTL_OUTPUT)
    assert source_front == "alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo"

    # 3. Non-existent port returns None
    node_unknown = Path("/sys/devices/platform/bus@0/3610000.usb/usb1/1-1/1-1.9/1-1.9.9")
    source_unknown = find_pulseaudio_source_for_usb_node(node_unknown, pactl_output=SAMPLE_PACTL_OUTPUT)
    assert source_unknown is None


def test_get_effective_mic_source_explicit_and_auto():
    # Explicit source takes precedence
    assert get_effective_mic_source("custom_mic_source") == "custom_mic_source"

    # Auto resolves via resolve_ai_camera_mic_source
    with patch(
        "robot_ui.kiosk.camera_mic_resolver.resolve_ai_camera_mic_source",
        return_value="alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo.2",
    ), patch("robot_ui.kiosk.camera_mic_resolver.sync_system_default_mic") as mock_sync:
        res = get_effective_mic_source("auto")
        assert res == "alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo.2"
        mock_sync.assert_called_once_with("alsa_input.usb-ORBBEC_ASTRA_Pro-01.analog-stereo.2")

    # If resolver fails, fallback to @DEFAULT_SOURCE@
    with patch("robot_ui.kiosk.camera_mic_resolver.resolve_ai_camera_mic_source", return_value=None):
        res = get_effective_mic_source("")
        assert res == "@DEFAULT_SOURCE@"


def test_sync_system_default_mic():
    # Default source or empty does nothing
    assert sync_system_default_mic("@DEFAULT_SOURCE@") is False
    assert sync_system_default_mic("") is False

    # Valid source runs pactl set-default-source
    with patch("subprocess.run") as mock_run:
        mock_run.return_value = MagicMock(returncode=0)
        assert sync_system_default_mic("alsa_input.test_source") is True
        mock_run.assert_called_once_with(
            ["pactl", "set-default-source", "alsa_input.test_source"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=1.5,
            check=False,
        )
