"""Kiosk page components."""
from __future__ import annotations

from robot_ui.kiosk.pages.admin_page import AdminPage, AdminPinDialog
from robot_ui.kiosk.pages.base import (
    check_wifi_connected,
    monochrome_icon,
    picture,
    strip_emojis,
)
from robot_ui.kiosk.pages.chat_page import (
    ChatPage,
    MicState,
    SpeakerButton,
    VolumeSliderWidget,
)
from robot_ui.kiosk.pages.home_page import HomePage, VideoBackground
from robot_ui.kiosk.pages.navigation_page import MapWidget, NavigationPage
from robot_ui.kiosk.pages.service_page import ServicePage
from robot_ui.kiosk.pages.system_config_page import SystemConfigPage
from robot_ui.kiosk.pages.welcome_page import WelcomePage

__all__ = [
    "AdminPage",
    "AdminPinDialog",
    "ChatPage",
    "HomePage",
    "MapWidget",
    "MicState",
    "NavigationPage",
    "ServicePage",
    "SlamMapPage",
    "SpeakerButton",
    "SystemConfigPage",
    "VideoBackground",
    "VolumeSliderWidget",
    "WelcomePage",
    "check_wifi_connected",
    "monochrome_icon",
    "picture",
    "strip_emojis",
]


def __getattr__(name: str):
    if name == "SlamMapPage":
        from robot_ui.kiosk.slam_page import SlamMapPage
        return SlamMapPage
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
