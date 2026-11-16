"""theme.py - Design Tokens & Stylesheet for Minimal Robotics Control Interface.

Standardizes dark theme styling matching Souta Robot Kiosk:
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
    COLOR_BG = "#000000"
    COLOR_SURFACE = "#111418"         # Dark Charcoal Surface (matching Admin & Service cards)
    COLOR_SURFACE_ALT = "#171c22"     # Slightly lighter dark surface
    COLOR_SURFACE_ACTIVE = "#202730"
    COLOR_CARD_BORDER = "#242c36"
    COLOR_BORDER_LIGHT = "#323b47"

    # Accent & Typography
    COLOR_ACCENT = "#F6C16A"          # Souta Amber Accent
    COLOR_TEXT_PRIMARY = "#FFFFFF"
    COLOR_TEXT_SECONDARY = "#94A3B8"
    COLOR_TEXT_MUTED = "#64748B"

    # Semantic Status Colors
    COLOR_HEALTHY = "#22C55E"
    COLOR_HEALTHY_BG = "#0C2818"
    COLOR_WARNING = "#F59E0B"
    COLOR_WARNING_BG = "#2E1C0A"
    COLOR_CRITICAL = "#EF4444"
    COLOR_CRITICAL_BG = "#300D0F"
    COLOR_UNKNOWN = "#71717A"
    COLOR_UNKNOWN_BG = "#18181B"

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
