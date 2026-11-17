"""theme.py - Design Tokens & Stylesheet for Minimal Robotics Control Interface.

Standardizes light theme styling matching Souta Robot Kiosk:
- Background: #0B111A
- Card Surface: #152130
- Accent Amber: #F6C16A
- Primary Text: #EAF0F7
- Secondary Text: #91A0B5
- Semantic Status: Healthy (#22C55E), Warning (#F59E0B), Critical (#EF4444), Unknown (#64748B)
"""
from __future__ import annotations

from robot_ui.kiosk.pages.system_status.contracts import HealthStatus, PreflightStatus


class KioskTheme:
    """Core design tokens for Souta Kiosk System Status."""

    # Background & Surfaces (OLED Pure Black & Dark Charcoal Theme)
    COLOR_BG = "#f7f9fc"
    COLOR_SURFACE = "#ffffff"         # Dark Charcoal Surface (matching Admin & Service cards)
    COLOR_SURFACE_ALT = "#f1f5f9"     # Slightly lighter dark surface
    COLOR_SURFACE_ACTIVE = "#e6edf6"
    COLOR_CARD_BORDER = "#dce4ec"
    COLOR_BORDER_LIGHT = "#cbd5e1"

    # Accent & Typography
    COLOR_ACCENT = "#b32632"          # Souta Amber Accent
    COLOR_TEXT_PRIMARY = "#1d2938"
    COLOR_TEXT_SECONDARY = "#506176"
    COLOR_TEXT_MUTED = "#657589"

    # Semantic Status Colors
    COLOR_HEALTHY = "#16804a"
    COLOR_HEALTHY_BG = "#eaf8f0"
    COLOR_WARNING = "#a86600"
    COLOR_WARNING_BG = "#fff4df"
    COLOR_CRITICAL = "#c42c44"
    COLOR_CRITICAL_BG = "#fff0f2"
    COLOR_UNKNOWN = "#66778b"
    COLOR_UNKNOWN_BG = "#eef2f6"

    # Typography & Touch Targets
    FONT_FAMILY = '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "DejaVu Sans", sans-serif'
    TOUCH_TARGET_MIN = 44
    CARD_RADIUS = "10px"
    BUTTON_RADIUS = "8px"

    @classmethod
    def get_status_color(cls, status: HealthStatus | str) -> str:
        s_val = status.value if isinstance(status, HealthStatus) else str(status).upper()
        if s_val in ("HEALTHY", "READY", "RUNNING", "PASS"):
            return cls.COLOR_HEALTHY
        if s_val in ("WARNING", "CHECKING"):
            return cls.COLOR_WARNING
        if s_val in ("CRITICAL", "FAIL", "ERROR", "STOPPED"):
            return cls.COLOR_CRITICAL
        return cls.COLOR_UNKNOWN

    @classmethod
    def get_status_bg(cls, status: HealthStatus | str) -> str:
        s_val = status.value if isinstance(status, HealthStatus) else str(status).upper()
        if s_val in ("HEALTHY", "READY", "RUNNING", "PASS"):
            return cls.COLOR_HEALTHY_BG
        if s_val in ("WARNING", "CHECKING"):
            return cls.COLOR_WARNING_BG
        if s_val in ("CRITICAL", "FAIL", "ERROR", "STOPPED"):
            return cls.COLOR_CRITICAL_BG
        return cls.COLOR_UNKNOWN_BG

    @classmethod
    def get_preflight_color(cls, status: PreflightStatus) -> str:
        if status == PreflightStatus.PASS:
            return cls.COLOR_HEALTHY
        if status == PreflightStatus.FAIL:
            return cls.COLOR_CRITICAL
        if status == PreflightStatus.CHECKING:
            return cls.COLOR_WARNING
        return cls.COLOR_UNKNOWN
