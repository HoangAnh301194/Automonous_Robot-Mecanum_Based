"""contracts.py - System Status Redesign Interface Contract & Data Definitions.

Defines standardized telemetry snapshots, health status enums, and utility functions
for the Souta Robot Minimal Robotics Control Interface.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional


class HealthStatus(Enum):
    """Semantic health states for robot components and the overall system."""
    UNKNOWN = "UNKNOWN"          # Missing data or topic not yet received
    HEALTHY = "HEALTHY"          # Fully operational, frequencies within threshold
    WARNING = "WARNING"          # Minor degradation, low battery, or elevated temperature
    CRITICAL = "CRITICAL"        # Disconnected, TF loss, or emergency stop active
    RUNNING = "RUNNING"          # Process is actively executing
    STOPPED = "STOPPED"          # Process has terminated or is inactive


class PreflightStatus(Enum):
    """Strict 5-state evaluation for Pre-flight verification checklist items."""
    NOT_CHECKED = "NOT_CHECKED"  # Initial state before diagnosis begins
    CHECKING = "CHECKING"        # Currently evaluating criteria
    PASS = "PASS"                # Validated and meets safety standards
    FAIL = "FAIL"                # Failed safety verification threshold
    UNKNOWN = "UNKNOWN"          # Backend check not supported or sensor missing


@dataclass
class SystemTelemetrySnapshot:
    """Snapshot of real-time sensor frequencies and perception states."""
    lidar_hz: float = 0.0
    lidar_min_dist: Optional[float] = None
    odom_hz: float = 0.0
    vx: float = 0.0
    wz: float = 0.0
    cam_front_hz: float = 0.0
    cam_rear_hz: float = 0.0
    cam_ai_hz: float = 0.0
    obs_front_dist: Optional[float] = None
    obs_rear_dist: Optional[float] = None
    tf_map_odom_ok: bool = False
    tf_odom_base_ok: bool = False
    yolo_active: bool = False
    people_count: int = 0
    is_waving: bool = False
    closest_person_dist: Optional[float] = None

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> SystemTelemetrySnapshot:
        return cls(
            lidar_hz=float(data.get("lidar_hz", 0.0)),
            lidar_min_dist=data.get("lidar_min_dist"),
            odom_hz=float(data.get("odom_hz", 0.0)),
            vx=float(data.get("vx", 0.0)),
            wz=float(data.get("wz", 0.0)),
            cam_front_hz=float(data.get("cam_front_hz", 0.0)),
            cam_rear_hz=float(data.get("cam_rear_hz", 0.0)),
            cam_ai_hz=float(data.get("cam_ai_hz", 0.0)),
            obs_front_dist=data.get("obs_front_dist"),
            obs_rear_dist=data.get("obs_rear_dist"),
            tf_map_odom_ok=bool(data.get("tf_map_odom_ok", False)),
            tf_odom_base_ok=bool(data.get("tf_odom_base_ok", False)),
            yolo_active=bool(data.get("yolo_active", False)),
            people_count=int(data.get("people_count", 0)),
            is_waving=bool(data.get("is_waving", False)),
            closest_person_dist=data.get("closest_person_dist"),
        )


@dataclass
class HostTelemetrySnapshot:
    """Snapshot of Jetson Orin Nano hardware telemetry and networking."""
    cpu_pct: float = 0.0
    cpu_temp: float = 0.0
    gpu_pct: float = 0.0
    gpu_temp: float = 0.0
    ram_used_gb: float = 0.0
    ram_total_gb: float = 0.0
    ram_pct: float = 0.0
    swap_used_gb: float = 0.0
    swap_total_gb: float = 0.0
    disk_free_gb: float = 0.0
    disk_total_gb: float = 0.0
    disk_pct: float = 0.0
    local_ip: str = "127.0.0.1"
    wifi_ssid: str = "Connected"
    wifi_signal_pct: int = 0
    cloud_ok: bool = False
    cloud_latency_ms: int = 0
    router_ok: bool = False
    router_port: int = 20128
    stt_ok: bool = False
    stt_latency_ms: int = 0
    top_processes: list[dict[str, Any]] = field(default_factory=list)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> HostTelemetrySnapshot:
        return cls(
            cpu_pct=float(data.get("cpu_pct", 0.0)),
            cpu_temp=float(data.get("cpu_temp", 0.0)),
            gpu_pct=float(data.get("gpu_pct", 0.0)),
            gpu_temp=float(data.get("gpu_temp", 0.0)),
            ram_used_gb=float(data.get("ram_used_gb", 0.0)),
            ram_total_gb=float(data.get("ram_total_gb", 0.0)),
            ram_pct=float(data.get("ram_pct", 0.0)),
            swap_used_gb=float(data.get("swap_used_gb", 0.0)),
            swap_total_gb=float(data.get("swap_total_gb", 0.0)),
            disk_free_gb=float(data.get("disk_free_gb", 0.0)),
            disk_total_gb=float(data.get("disk_total_gb", 0.0)),
            disk_pct=float(data.get("disk_pct", 0.0)),
            local_ip=str(data.get("local_ip", "127.0.0.1")),
            wifi_ssid=str(data.get("wifi_ssid", "Connected")),
            wifi_signal_pct=int(data.get("wifi_signal_pct", 0)),
            cloud_ok=bool(data.get("cloud_ok", False)),
            cloud_latency_ms=int(data.get("cloud_latency_ms", 0)),
            router_ok=bool(data.get("router_ok", False)),
            router_port=int(data.get("router_port", 20128)),
            stt_ok=bool(data.get("stt_ok", False)),
            stt_latency_ms=int(data.get("stt_latency_ms", 0)),
            top_processes=list(data.get("top_processes", [])),
        )


@dataclass
class PowerWaterSnapshot:
    """Snapshot of deep 36V battery and water service metrics."""
    battery_percent: float = 0.0
    voltage_smooth: float = 36.0
    voltage_raw: float = 36.0
    voltage_source: str = "hoverboard"
    water_liter: float = 0.0
    kettle_present: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> PowerWaterSnapshot:
        return cls(
            battery_percent=float(data.get("battery_percent", 0.0)),
            voltage_smooth=float(data.get("voltage_smooth", 36.0)),
            voltage_raw=float(data.get("voltage_raw", 36.0)),
            voltage_source=str(data.get("voltage_source", "hoverboard")),
            water_liter=float(data.get("water_liter", 0.0)),
            kettle_present=bool(data.get("kettle_present", True)),
        )


def compute_overall_health(
    sensors: SystemTelemetrySnapshot,
    host: HostTelemetrySnapshot,
    power: PowerWaterSnapshot,
) -> tuple[HealthStatus, str, list[str]]:
    """Compute overall robot health based strictly on grounded telemetry."""
    warnings: list[str] = []
    criticals: list[str] = []

    # 1. Critical checks
    if host.cpu_temp >= 85.0:
        criticals.append("Nhiệt độ SoC vượt ngưỡng nguy hiểm (>=85°C)")
    if power.battery_percent > 0 and power.battery_percent < 15.0:
        criticals.append(f"Mức Pin quá thấp ({power.battery_percent:.0f}%), cần sạc ngay")

    # 2. Warning checks
    if host.cpu_temp >= 75.0 and host.cpu_temp < 85.0:
        warnings.append(f"Cảnh báo nhiệt độ Jetson cao ({host.cpu_temp:.1f}°C)")
    if 15.0 <= power.battery_percent < 25.0:
        warnings.append(f"Mức Pin yếu ({power.battery_percent:.0f}%)")
    if not sensors.tf_map_odom_ok and sensors.lidar_hz > 1.0:
        warnings.append("Mất liên kết tọa độ map -> odom (Chưa định vị)")
    if not sensors.tf_odom_base_ok and sensors.odom_hz > 1.0:
        warnings.append("Mất liên kết odom -> base_footprint")
    if host.disk_pct >= 90.0:
        warnings.append(f"Dung lượng ổ SSD sắp đầy ({host.disk_pct:.0f}% đã dùng)")
    if host.ram_pct >= 90.0:
        warnings.append(f"RAM Jetson sắp đầy ({host.ram_pct:.0f}% đã dùng)")

    # 3. Overall health conclusion
    if criticals:
        return HealthStatus.CRITICAL, criticals[0], criticals + warnings
    if warnings:
        return HealthStatus.WARNING, warnings[0], warnings
    
    # Chuẩn xác minh nghiêm ngặt trước khi tuyên bố robot HEALTHY
    has_lidar = sensors.lidar_hz >= 5.0
    has_odom = (sensors.odom_hz >= 5.0) or sensors.tf_odom_base_ok
    has_tf = sensors.tf_map_odom_ok and sensors.tf_odom_base_ok
    has_power = power.battery_percent >= 25.0
    has_host = (host.cpu_temp == 0.0 or 0 < host.cpu_temp < 75.0) and host.ram_pct < 85.0

    if has_lidar and has_odom and has_tf and has_power and has_host:
        return HealthStatus.HEALTHY, "Hệ thống hoạt động bình thường, toàn bộ cảm biến & phần cứng sẵn sàng", []
    return HealthStatus.UNKNOWN, "Hệ thống đang khởi tạo hoặc chưa đủ dữ liệu telemetry cảm biến", []
