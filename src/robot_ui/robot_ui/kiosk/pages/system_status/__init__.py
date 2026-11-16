"""system_status package - Minimal Robotics Control Interface for Souta Robot Kiosk."""

from robot_ui.kiosk.pages.system_status.contracts import (
    HealthStatus,
    HostTelemetrySnapshot,
    PowerWaterSnapshot,
    PreflightStatus,
    SystemTelemetrySnapshot,
    compute_overall_health,
)
from robot_ui.kiosk.pages.system_status.theme import KioskTheme
from robot_ui.kiosk.pages.system_status.components import (
    DangerousActionDialog,
    MetricCard,
    ModuleCard,
    PageHeader,
    StatusPill,
    create_section_card,
    load_pixmap,
)
from robot_ui.kiosk.pages.system_status.overview import SystemOverviewScreen
from robot_ui.kiosk.pages.system_status.sensors_ai import SensorsAIDetailScreen
from robot_ui.kiosk.pages.system_status.hardware import HardwareDetailScreen
from robot_ui.kiosk.pages.system_status.ros2_system import ROS2SystemDetailScreen
from robot_ui.kiosk.pages.system_status.diagnostics import DiagnosticsDetailScreen

__all__ = [
    "HealthStatus",
    "PreflightStatus",
    "SystemTelemetrySnapshot",
    "HostTelemetrySnapshot",
    "PowerWaterSnapshot",
    "compute_overall_health",
    "KioskTheme",
    "PageHeader",
    "MetricCard",
    "ModuleCard",
    "StatusPill",
    "DangerousActionDialog",
    "create_section_card",
    "load_pixmap",
    "SystemOverviewScreen",
    "SensorsAIDetailScreen",
    "HardwareDetailScreen",
    "ROS2SystemDetailScreen",
    "DiagnosticsDetailScreen",
]
