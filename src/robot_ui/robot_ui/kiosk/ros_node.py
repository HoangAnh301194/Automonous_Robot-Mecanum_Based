from __future__ import annotations

from enum import Enum
import math
import os
import shlex
import signal
import subprocess
import threading
import time
import uuid
from functools import partial
from pathlib import Path as FilePath
from typing import Any

# Khắc phục xung đột môi trường Qt do opencv-python tự động inject cv2/qt/plugins gây crash RViz2
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import yaml

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose
from collections import deque
import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger

try:
    from lifecycle_msgs.srv import GetState
except ImportError:
    GetState = None

from robot_interfaces.action import DirectGo
from robot_interfaces.msg import NavigationStatus, RobotStatus
from robot_interfaces.srv import GetLocation, GetLocationList

try:
    from robot_interfaces.msg import TrackedPersonArray
except ImportError:
    TrackedPersonArray = None

from robot_ui.kiosk.signals import KioskSignals


_BATTERY_36V_CURVE = [
    (31.5, 0.0),
    (33.0, 5.0),
    (34.2, 10.0),
    (35.4, 20.0),
    (36.0, 30.0),
    (36.6, 40.0),
    (37.2, 50.0),
    (37.8, 60.0),
    (38.4, 70.0),
    (39.0, 80.0),
    (39.9, 90.0),
    (41.5, 100.0),
]


def voltage_to_percentage_36v(voltage_v: float) -> float:
    if voltage_v <= _BATTERY_36V_CURVE[0][0]:
        return 0.0
    if voltage_v >= _BATTERY_36V_CURVE[-1][0]:
        return 100.0
    for (v0, p0), (v1, p1) in zip(_BATTERY_36V_CURVE, _BATTERY_36V_CURVE[1:]):
        if voltage_v <= v1:
            return p0 + (voltage_v - v0) / (v1 - v0) * (p1 - p0)
    return 100.0


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class LayerState(str, Enum):
    """Máy trạng thái chuẩn cho các tầng tiến trình ROS 2 (Single Source of Truth)."""
    STOPPED = "STOPPED"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    ERROR = "ERROR"


class LocalizationState(str, Enum):
    """Máy trạng thái chuẩn cho AMCL Localization (Single Source of Truth)."""
    NOT_INITIALIZED = "NOT_INITIALIZED"          # Chưa có ước lượng vị trí
    WAITING_FOR_AMCL = "WAITING_FOR_AMCL"        # AMCL chưa active hoặc thiếu sensor
    READY_TO_SET_POSE = "READY_TO_SET_POSE"      # Đủ điều kiện nhận pose ban đầu
    PUBLISHING = "PUBLISHING"                    # Đang gửi dữ liệu lên /initialpose
    WAITING_FOR_LOCALIZATION = "WAITING_FOR_LOCALIZATION" # Đã gửi, chờ AMCL hội tụ
    TF_AVAILABLE = "TF_AVAILABLE"                # Đã có TF map -> odom hợp lệ
    LOCALIZED = "LOCALIZED"                      # Đã định vị chính xác (TF + Covariance đạt)
    ERROR = "ERROR"                              # Lỗi (Timeout, sensor stale, sai map)


class KioskRosNode(Node):
    """Small ROS adapter dedicated to the native customer UI."""

    MAP_PREVIEW_MAX_DIMENSION = 280
    PATH_PREVIEW_MAX_POINTS = 400

    def __init__(self, signals: KioskSignals) -> None:
        super().__init__("robot_ui_kiosk")
        self.signals = signals
        self._pose: dict[str, Any] = {}
        self._last_pose_emit_time: float = 0.0
        self._cached_serial_ports: tuple[str, str, str] | None = None
        self._direct_go_client = ActionClient(
            self,
            DirectGo,
            "/navigation/direct_go",
        )
        self._nav2_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cancel_nav2_client = self.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.cancel_direct_go_client = self.create_client(
            CancelGoal, "/navigation/direct_go/_action/cancel_goal"
        )
        self.emergency_stop_client = self.create_client(
            Trigger, "/mission/emergency_stop"
        )
        self.reset_emergency_client = self.create_client(
            Trigger, "/mission/reset_emergency"
        )
        self.pause_patrol_client = self.create_client(
            Trigger, "/mission/pause_patrol"
        )
        self.start_patrol_client = self.create_client(
            Trigger, "/mission/start_patrol"
        )
        self.reload_route_client = self.create_client(
            Trigger, "/mission/reload_route"
        )
        self.trigger_serve_client = self.create_client(
            Trigger, "/mission/trigger_serve"
        )
        self.trigger_finish_client = self.create_client(
            Trigger, "/mission/trigger_finish"
        )
        self.serve_trigger_pub = self.create_publisher(Bool, "/mission/serve_trigger", 10)
        self.finish_serving_pub = self.create_publisher(Bool, "/mission/finish_serving", 10)
        self._initial_goal_dist: float | None = None
        self._last_scan_ui_pub = 0.0
        self._last_obs_ui_pub = {"front": 0.0, "rear": 0.0}
        self._location_client = self.create_client(
            GetLocationList,
            "/config/get_location_list",
        )
        self._get_location_client = self.create_client(
            GetLocation,
            "/config/get_location",
        )
        self._location_batch = 0
        self._pending_locations: dict[str, dict[str, Any]] = {}
        self._remaining_locations = 0
        self._goal_handle: Any = None
        self._have_localized_pose = False
        self._last_robot_status_at: float | None = None
        self._connected = False
        self._slam_proc: subprocess.Popen | None = None
        self._localization_proc: subprocess.Popen | None = None
        self._selected_map_yaml: str = ""
        self._selected_backend: str = "amcl"

        # AMCL Initial Pose & Localization Tracking
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        if GetState is not None:
            self._amcl_get_state_client = self.create_client(GetState, "/amcl/get_state")
        else:
            self._amcl_get_state_client = None
        self._localization_state: LocalizationState = LocalizationState.NOT_INITIALIZED
        self._localization_message: str = "Chưa thiết lập vị trí ban đầu"
        self._loc_session_token: str = ""
        self._last_initial_pose_sent_at: float = 0.0
        self._last_amcl_pose_at: float = 0.0
        self._last_amcl_pose_msg: PoseWithCovarianceStamped | None = None
        self._amcl_covariance_norm: float = 999.0
        self._last_scan_time: float = 0.0
        self._last_odom_time: float = 0.0
        self._auto_initial_pose_enabled: bool = False

        # System telemetry & Configuration tracking
        self._dbg_lock = threading.Lock()
        self._topic_stats: dict[str, dict[str, Any]] = {}
        self._battery_voltages: deque[float] = deque(maxlen=25)
        self._battery_pct = 0.0
        self._battery_smooth_v = 36.0
        self._battery_raw_v = 36.0
        self._battery_source = "robot_status"
        self._water_liter = 0.0
        self._kettle_present = False
        self._is_waving = False
        self._yolo_active = False
        self._people_count = 0
        self._closest_person_dist: float | None = None
        self._last_ai_rx = 0.0
        # Thread-safe Process Lifecycle State Machine
        self._lifecycle_lock = threading.RLock()
        self._layer_states: dict[str, LayerState] = {
            "layer1": LayerState.STOPPED,
            "layer2": LayerState.STOPPED,
            "layer3": LayerState.STOPPED,
            "layer4": LayerState.STOPPED,
            "layer5": LayerState.STOPPED,
        }
        self._layer_procs: dict[str, subprocess.Popen] = {}
        self._starting_layers: set[str] = set()
        self._cancel_starting_layers: set[str] = set()
        self._stopping_layers: set[str] = set()
        self._error_layers: set[str] = set()
        self._active_launch_tokens: dict[str, str] = {}
        self._layer_log_buffers: dict[str, deque[str]] = {
            "all": deque(maxlen=400),
            "layer1": deque(maxlen=400),
            "layer2": deque(maxlen=400),
            "layer3": deque(maxlen=400),
            "layer4": deque(maxlen=400),
            "layer5": deque(maxlen=400),
        }
        self._console_active: bool = False
        self._force_swap_ports: bool = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(RobotStatus, "/robot_status", self._on_robot_status, 10)
        self.create_subscription(
            NavigationStatus,
            "/navigation/status",
            self._on_navigation_status,
            10,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            10,
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(String, "/mission/service_popup_cmd", self._on_service_popup_cmd, 10)

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, best_effort_qos)
        self.create_subscription(
            LaserScan,
            "/scan_obstacles",
            partial(self._on_obstacle_scan, "front"),
            best_effort_qos,
        )
        self.create_subscription(
            LaserScan,
            "/rear_scan_obstacle",
            partial(self._on_obstacle_scan, "rear"),
            best_effort_qos,
        )

        # Raw camera image subscribers for Hz tracking
        for key, topic in (
            ("cam_front", "/front_camera/depth/image_raw"),
            ("cam_rear", "/rear_camera/depth/image_raw"),
            ("cam_ai", "/ai_camera/color/image_raw"),
        ):
            self.create_subscription(
                Image,
                topic,
                lambda _msg, k=key: self._mark_topic_rx(k),
                10,
                raw=True,
            )

        self.create_subscription(
            Float64,
            "/hoverboard/battery_voltage",
            self._on_hoverboard_voltage,
            10,
        )
        self.create_subscription(
            BatteryState,
            "/battery",
            self._on_battery_state,
            10,
        )
        self.create_subscription(
            Bool,
            "/pose/wave_detected",
            self._on_wave_detected,
            10,
        )
        if TrackedPersonArray is not None:
            self.create_subscription(
                TrackedPersonArray,
                "/perception/tracked_people",
                self._on_tracked_people,
                10,
            )
        self.create_subscription(
            Float32,
            "/mission/person_distance_preview",
            self._on_person_distance_preview,
            10,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )

        self.create_timer(1.0, self._check_backend_connection)
        self.create_timer(1.0, self._publish_system_telemetry)

        self.get_logger().info("Native touchscreen UI ROS adapter started.")

    def _on_robot_status(self, msg: RobotStatus) -> None:
        self._last_robot_status_at = time.monotonic()
        if not self._connected:
            self._connected = True
            self.get_logger().info("Đã kết nối với Robot backend (nhận được /robot_status).")
            self.signals.connection_changed.emit(True)
        if hasattr(msg, "water_liter"):
            self._water_liter = float(msg.water_liter)
        if hasattr(msg, "kettle_present"):
            self._kettle_present = bool(msg.kettle_present)
        self.signals.robot_status_changed.emit(
            {
                "battery_percent": float(msg.battery_percent),
                "battery_low": bool(msg.battery_low),
                "lidar_ok": bool(msg.lidar_ok),
                "localization_ok": bool(msg.localization_ok),
                "odom_ok": bool(msg.odom_ok),
                "water_low": bool(msg.water_low),
                "water_liter": self._water_liter,
                "kettle_present": self._kettle_present,
                "robot_state": str(msg.robot_state),
                "current_location": str(msg.current_location),
            }
        )

    def _check_backend_connection(self) -> None:
        connected = (
            self._last_robot_status_at is not None
            and time.monotonic() - self._last_robot_status_at < 4.0
        )
        if connected != self._connected:
            self._connected = connected
            if not connected:
                self.get_logger().warn("Mất kết nối với Robot backend (quá 4s không nhận được /robot_status)!")
            else:
                self.get_logger().info("Đã kết nối lại với Robot backend.")
            self.signals.connection_changed.emit(connected)

    def _on_navigation_status(self, msg: NavigationStatus) -> None:
        self.get_logger().debug(
            f"[NAV STATUS] status={msg.status}, goal='{msg.current_goal_name}', dist_remain={msg.distance_remaining:.2f}m"
        )
        self.signals.navigation_status_changed.emit(
            {
                "status": str(msg.status),
                "current_goal_name": str(msg.current_goal_name),
                "distance_remaining": float(msg.distance_remaining),
            }
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        self.get_logger().info(
            f"[KIOSK MAP] Received /map: {width}x{height}, "
            f"cells={len(msg.data)}, frame={msg.header.frame_id}"
        )
        if width <= 0 or height <= 0:
            self.get_logger().warning("[KIOSK MAP] Ignoring empty map")
            return

        step = max(1, math.ceil(max(width, height) / self.MAP_PREVIEW_MAX_DIMENSION))
        preview_width = math.ceil(width / step)
        preview_height = math.ceil(height / step)
        cells = [
            int(msg.data[source_y * width + source_x])
            for source_y in range(0, height, step)
            for source_x in range(0, width, step)
        ]
        orientation = msg.info.origin.orientation
        self.signals.map_changed.emit(
            {
                "frame_id": str(msg.header.frame_id),
                "width": width,
                "height": height,
                "preview_width": preview_width,
                "preview_height": preview_height,
                "sample_step": step,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "origin_yaw": quaternion_to_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                ),
                "cells": cells,
            }
        )

    def _emit_pose(self, pose: Any, frame_id: str, source: str) -> None:
        orientation = pose.orientation
        pose_data = {
            "source": source,
            "frame_id": frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quaternion_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        }
        self._pose = pose_data
        now = time.monotonic()
        if now - self._last_pose_emit_time >= 0.05:  # Throttling tối đa 20 Hz lên Qt UI
            self._last_pose_emit_time = now
            self.signals.pose_changed.emit(pose_data)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._have_localized_pose = True
        self._last_amcl_pose_at = time.monotonic()
        self._last_amcl_pose_msg = msg
        cov = msg.pose.covariance
        cov_norm = math.sqrt(float(cov[0]) + float(cov[7]))
        self._amcl_covariance_norm = cov_norm
        # Khi Kiosk chủ động phát initial pose, việc chuyển sang LOCALIZED được thực hiện nguyên tử qua _verify_localization_worker.
        # Nếu người dùng dùng RViz2 (2D Pose Estimate) độc lập, tự động đồng bộ khi đạt chuẩn hội tụ và TF:
        if not self._loc_session_token and self._localization_state != LocalizationState.LOCALIZED:
            if cov_norm <= 1.20:
                tf_ok = False
                try:
                    tf_ok = self._tf_buffer.can_transform("map", "odom", rclpy.time.Time())
                except Exception:
                    pass
                if tf_ok:
                    self._set_localization_state(
                        LocalizationState.LOCALIZED,
                        f"Đã định vị thành công từ RViz (sai số: {cov_norm:.2f}m)"
                    )
        self._emit_pose(msg.pose.pose, str(msg.header.frame_id), "amcl")

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom_time = time.monotonic()
        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)
        self._mark_topic_rx("odom", {"vx": vx, "wz": wz})
        if not self._have_localized_pose:
            self._emit_pose(msg.pose.pose, str(msg.header.frame_id), "odom")

    def _on_path(self, msg: Path) -> None:
        poses = msg.poses
        step = max(1, math.ceil(len(poses) / self.PATH_PREVIEW_MAX_POINTS))
        self.signals.path_changed.emit(
            [
                [float(pose.pose.position.x), float(pose.pose.position.y)]
                for pose in poses[::step]
            ]
        )

    def request_locations(self) -> None:
        if not self._location_client.service_is_ready():
            self.get_logger().warn("[CONFIG] Service /config/get_location_list chưa sẵn sàng!")
            self.signals.command_status_changed.emit(
                "warning",
                "Dịch vụ danh sách địa điểm chưa sẵn sàng",
            )
            return
        self.get_logger().info("[CONFIG] Đang yêu cầu danh sách địa điểm từ /config/get_location_list...")
        future = self._location_client.call_async(GetLocationList.Request())
        future.add_done_callback(self._on_locations_response)

    def _on_locations_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[CONFIG] Không thể tải danh sách địa điểm: {error}")
            self.signals.command_status_changed.emit("error", "Không tải được địa điểm")
            return
        if response.success:
            loc_names = list(response.locations)
            self.get_logger().info(f"[CONFIG] Đã nhận {len(loc_names)} địa điểm: {loc_names}")
            self._request_location_coordinates(loc_names)
        else:
            self.get_logger().error(f"[CONFIG] Service trả về thất bại: {response.message}")
            self.signals.command_status_changed.emit("error", str(response.message))

    def _request_location_coordinates(self, names: list[str]) -> None:
        self._location_batch += 1
        batch = self._location_batch
        self._pending_locations = {}
        self._remaining_locations = len(names)
        if not names:
            self.signals.locations_changed.emit([])
            return
        if not self._get_location_client.service_is_ready():
            self.get_logger().warn("[CONFIG] Service /config/get_location chưa sẵn sàng lấy tọa độ!")
            self.signals.command_status_changed.emit(
                "warning",
                "Dịch vụ tọa độ địa điểm chưa sẵn sàng",
            )
            return
        for name in names:
            request = GetLocation.Request()
            request.name = name
            future = self._get_location_client.call_async(request)
            future.add_done_callback(partial(self._on_location_coordinate, batch, name))

    def _on_location_coordinate(self, batch: int, name: str, future: Any) -> None:
        if batch != self._location_batch:
            return
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"[CONFIG] Tọa độ '{name}': X={response.x:.2f}, Y={response.y:.2f}, Yaw={response.yaw:.2f}"
                )
                self._pending_locations[name] = {
                    "name": name,
                    "x": float(response.x),
                    "y": float(response.y),
                    "yaw": float(response.yaw),
                }
            else:
                self.get_logger().warn(f"[CONFIG] Không lấy được tọa độ điểm '{name}': {response.message}")
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().warning(f"[CONFIG] Lỗi khi gọi tọa độ '{name}': {error}")
        self._remaining_locations -= 1
        if self._remaining_locations == 0:
            self.get_logger().info(f"[CONFIG] Hoàn tất nạp tọa độ cho {len(self._pending_locations)} địa điểm.")
            self.signals.locations_changed.emit(
                [self._pending_locations[key] for key in sorted(self._pending_locations)]
            )

    def navigate_to(self, location: str) -> None:
        target = location.strip()
        if not target:
            return
        is_ready, reason = self.is_navigation_ready()
        if not is_ready:
            self.get_logger().error(f"[NAV][ERROR] Từ chối navigate_to: {reason}")
            self.signals.command_status_changed.emit("warning", f"[ERROR] {reason}")
            return
        client = self._direct_go_client
        server_ok = False
        if client is not None:
            if hasattr(client, "server_is_ready") and client.server_is_ready():
                server_ok = True
            elif hasattr(client, "wait_for_server") and client.wait_for_server(timeout_sec=1.5):
                server_ok = True

        if not server_ok:
            self.get_logger().error(f"[NAV][ERROR] Action server /navigation/direct_go chưa sẵn sàng (target='{target}')!")
            self.signals.command_status_changed.emit(
                "warning",
                "[ERROR] Navigation Manager chưa sẵn sàng (Layer 5)",
            )
            return

        self.get_logger().info(f"[NAV][INFO] Gửi mục tiêu địa điểm: '{target}'")
        self.signals.navigation_active_changed.emit(True)
        goal = DirectGo.Goal()
        goal.target_location = target
        self.signals.command_status_changed.emit("progress", f"[INFO] Đang gửi điểm đến {target}...")
        future = self._direct_go_client.send_goal_async(
            goal,
            feedback_callback=self._on_navigation_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[NAV][ERROR] Lỗi khi gửi mục tiêu: {error}")
            self.signals.command_status_changed.emit("error", f"[ERROR] Không gửi được điểm đến: {error}")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.get_logger().error("[NAV][ERROR] Mục tiêu bị Action server TỪ CHỐI!")
            self.signals.command_status_changed.emit("error", "[ERROR] Điểm đến bị từ chối")
            self.signals.navigation_active_changed.emit(False)
            return
        self.get_logger().info("[NAV][INFO] Mục tiêu đã được CHẤP THUẬN (ACCEPTED), bắt đầu di chuyển.")
        self._goal_handle = goal_handle
        self.signals.command_status_changed.emit("progress", "[INFO] Robot bắt đầu di chuyển")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        distance = getattr(feedback, "distance_remaining", None)
        if distance is not None:
            self.get_logger().info(f"[NAV][DEBUG] Khoảng cách còn lại: {float(distance):.2f} m")
            self.signals.command_status_changed.emit(
                "progress",
                f"[INFO] Còn {float(distance):.1f} m",
            )

    def _on_navigation_result(self, future: Any) -> None:
        self._goal_handle = None
        self.signals.navigation_active_changed.emit(False)
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[NAV][ERROR] Lỗi khi nhận kết quả hành trình: {error}")
            self.signals.command_status_changed.emit("error", f"[ERROR] Mất kết quả dẫn đường: {error}")
            return
        tone = "success" if result.success else "error"
        tag = "[INFO]" if result.success else "[ERROR]"
        self.get_logger().info(f"[NAV][{tag[1:-1]}] Kết quả hành trình: success={result.success}, msg='{result.finish_message}'")
        self.signals.command_status_changed.emit(tone, f"{tag} {result.finish_message}")

    def navigate_to_pose(self, x: float, y: float, yaw: float) -> None:
        is_ready, reason = self.is_navigation_ready()
        if not is_ready:
            self.get_logger().error(f"[NAV][ERROR] Từ chối navigate_to_pose: {reason}")
            self.signals.command_status_changed.emit("warning", f"[ERROR] {reason}")
            self.signals.navigation_active_changed.emit(False)
            return

        client = getattr(self, "_nav2_client", None) or getattr(self, "_nav_action_client", None)
        server_ok = False
        if client is not None:
            if hasattr(client, "server_is_ready") and client.server_is_ready():
                server_ok = True
            elif hasattr(client, "wait_for_server") and client.wait_for_server(timeout_sec=1.5):
                server_ok = True

        if not server_ok:
            self.get_logger().error(
                f"[NAV][ERROR] Nav2 Action Server /navigate_to_pose chưa online (target: X={x:.2f}, Y={y:.2f})! Hãy kiểm tra Layer 2."
            )
            self.signals.command_status_changed.emit(
                "error",
                "[ERROR] Nav2 Action Server chưa online (Hãy kiểm tra Layer 2)",
            )
            self.signals.navigation_active_changed.emit(False)
            return

        self.get_logger().info(f"[NAV][INFO] Bắt đầu điều hướng tới tọa độ: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

        self.signals.navigation_active_changed.emit(True)
        self.signals.command_status_changed.emit(
            "progress",
            f"[INFO] Đang đi tới X {x:.2f}, Y {y:.2f}",
        )
        future = self._nav2_client.send_goal_async(
            goal,
            feedback_callback=self._on_nav2_feedback,
        )
        future.add_done_callback(self._on_nav2_goal_response)

    def _on_nav2_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[NAV][ERROR] Lỗi khi gửi mục tiêu Nav2: {error}")
            self.signals.command_status_changed.emit("error", f"[ERROR] Lỗi gửi mục tiêu Nav2: {error}")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.get_logger().error(
                "[NAV][ERROR] Mục tiêu bị Nav2 server TỪ CHỐI (Goal Rejected - có thể điểm đến rơi vào vật cản costmap hoặc ngoài bản đồ)!"
            )
            self.signals.command_status_changed.emit(
                "error",
                "[ERROR] Nav2 từ chối mục tiêu (Rơi vào vật cản hoặc ngoài bản đồ)",
            )
            self.signals.navigation_active_changed.emit(False)
            return
        self.get_logger().info("[NAV][INFO] Mục tiêu đã được Nav2 CHẤP THUẬN (Goal Accepted). Robot bắt đầu di chuyển.")
        self._goal_handle = goal_handle
        self.signals.command_status_changed.emit("progress", "[INFO] Nav2 đã chấp nhận. Robot đang di chuyển...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav2_result)

    def _on_nav2_feedback(self, feedback_message: Any) -> None:
        fb = feedback_message.feedback
        distance = float(fb.distance_remaining)
        recov = int(getattr(fb, "number_of_recoveries", 0))
        nav_time = getattr(fb, "navigation_time", None)
        nav_sec = float(nav_time.sec + nav_time.nanosec * 1e-9) if nav_time else 0.0
        est_time = getattr(fb, "estimated_time_remaining", None)
        est_sec = float(est_time.sec + est_time.nanosec * 1e-9) if est_time else 0.0

        if self._initial_goal_dist is None or distance > self._initial_goal_dist:
            self._initial_goal_dist = distance

        self.get_logger().info(
            f"[NAV][DEBUG] Tiến độ: Còn lại {distance:.2f} m | Thời gian chạy: {nav_sec:.1f} s | Ước tính: {est_sec:.1f} s | Tự cứu: {recov}"
        )
        self.signals.command_status_changed.emit("progress", f"[INFO] Còn {distance:.1f} m (Tự cứu: {recov})")
        self.signals.nav_feedback_changed.emit({
            "distance_remaining": distance,
            "initial_distance": self._initial_goal_dist,
            "navigation_time": nav_sec,
            "estimated_time_remaining": est_sec,
            "number_of_recoveries": recov,
        })

    def _on_nav2_result(self, future: Any) -> None:
        self._goal_handle = None
        self._initial_goal_dist = None
        self.signals.navigation_active_changed.emit(False)
        try:
            status = int(future.result().status)
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[NAV][ERROR] Lỗi nhận kết quả từ Nav2: {error}")
            self.signals.command_status_changed.emit("error", f"[ERROR] Mất kết quả dẫn đường: {error}")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[NAV][INFO] Đã tới điểm đến thành công (STATUS_SUCCEEDED).")
            self.signals.command_status_changed.emit("success", "[INFO] Đã tới điểm đến thành công")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("[NAV][WARN] Đã hủy hành trình điều hướng (STATUS_CANCELED).")
            self.signals.command_status_changed.emit("warning", "[WARN] Đã dừng hành trình điều hướng")
        else:
            self.get_logger().error(
                f"[NAV][ERROR] Điều hướng kết thúc với mã lỗi: {status} (Có thể robot bị kẹt vật cản hoặc hết thời gian tìm đường)"
            )
            self.signals.command_status_changed.emit("error", f"[ERROR] Không thể tới điểm đến (Status code: {status})")

    def cancel_navigation(self) -> None:
        """Hủy hành trình điều hướng hiện tại một cách an toàn mà không kích hoạt Emergency Stop."""
        self.get_logger().info("[NAV][INFO] Nhận yêu cầu hủy hành trình điều hướng từ UI.")
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"[NAV][WARN] Lỗi khi gửi cancel goal: {exc}")
            self._goal_handle = None
        # Dừng robot êm bằng cách phát vận tốc 0 vào /cmd_vel
        try:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
        except Exception:
            pass
        self.signals.navigation_status_changed.emit({"status": "CANCELLED", "state": "CANCELLED"})
        self.signals.command_status_changed.emit("warning", "[WARN] Đã dừng hành trình điều hướng")

    def trigger_emergency_stop(self) -> None:
        """Dừng khẩn cấp toàn diện: phanh tức thì, hủy mọi Goal Nav2/DirectGo và kích hoạt P0 Emergency trong BT."""
        self.get_logger().warn("[EMERGENCY][WARN] EMERGENCY STOP TRIGGERED FROM UI!")

        # 1. Phát liên tục vận tốc 0.0 vào /cmd_vel để phanh dừng bánh lập tức
        stop_twist = Twist()
        for _ in range(10):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.01)

        # 2. Hủy goal nội bộ Kiosk nếu đang có
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None

        # 3. Hủy toàn bộ goal trên Nav2 server
        req = CancelGoal.Request()
        if self.cancel_nav2_client.service_is_ready():
            self.cancel_nav2_client.call_async(req)
        elif self.cancel_nav2_client.wait_for_service(timeout_sec=0.2):
            self.cancel_nav2_client.call_async(req)

        # 4. Hủy toàn bộ goal trên DirectGo server (/navigation/direct_go)
        if self.cancel_direct_go_client.service_is_ready():
            self.cancel_direct_go_client.call_async(req)
        elif self.cancel_direct_go_client.wait_for_service(timeout_sec=0.2):
            self.cancel_direct_go_client.call_async(req)

        # 5. Gọi Service Dừng Khẩn Cấp toàn hệ thống tới Mission Controller
        if self.emergency_stop_client.service_is_ready():
            self.emergency_stop_client.call_async(Trigger.Request())
        elif self.emergency_stop_client.wait_for_service(timeout_sec=0.2):
            self.emergency_stop_client.call_async(Trigger.Request())
        else:
            if self.pause_patrol_client.service_is_ready():
                self.pause_patrol_client.call_async(Trigger.Request())
            elif self.pause_patrol_client.wait_for_service(timeout_sec=0.2):
                self.pause_patrol_client.call_async(Trigger.Request())

        self.signals.command_status_changed.emit("error", "[ERROR] DA KICH HOAT DUNG KHAN CAP!")
        self.signals.navigation_active_changed.emit(False)

    def reset_emergency_stop(self) -> None:
        """Mở khóa dừng khẩn cấp trên Mission Controller."""
        if self.reset_emergency_client.service_is_ready():
            self.reset_emergency_client.call_async(Trigger.Request())
        elif self.reset_emergency_client.wait_for_service(timeout_sec=0.2):
            self.reset_emergency_client.call_async(Trigger.Request())
        self.get_logger().info("[EMERGENCY][INFO] Da mo khoa dung khan cap.")
        self.signals.command_status_changed.emit("success", "[INFO] Da mo khoa dung khan cap")

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan_time = time.monotonic()
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        min_lidar = min(valid_ranges) if valid_ranges else None
        self._mark_topic_rx("lidar", {"min_dist": min_lidar, "count": len(valid_ranges)})

        now = time.monotonic()
        if now - self._last_scan_ui_pub < 0.1:
            return
        self._last_scan_ui_pub = now
        if not self._pose:
            return
        ranges = msg.ranges
        if not ranges:
            return
        robot_x = float(self._pose.get("x", 0.0))
        robot_y = float(self._pose.get("y", 0.0))
        robot_yaw = float(self._pose.get("yaw", 0.0))
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        lx_base, ly_base = 0.175, 0.0
        origin_x = robot_x + cos_yaw * lx_base - sin_yaw * ly_base
        origin_y = robot_y + sin_yaw * lx_base + cos_yaw * ly_base
        step = max(1, len(ranges) // 360)
        points: list[tuple[float, float]] = []
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            beam = robot_yaw - angle
            points.append((origin_x + r * math.cos(beam), origin_y + r * math.sin(beam)))
        self.signals.scan_points_changed.emit(points)

    def _on_obstacle_scan(self, side: str, msg: LaserScan) -> None:
        valid_obs = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        min_obs = min(valid_obs) if valid_obs else None
        self._mark_topic_rx(f"obs_{side}", {"min_dist": min_obs})

        now = time.monotonic()
        if now - self._last_obs_ui_pub.get(side, 0.0) < 0.15:
            return
        self._last_obs_ui_pub[side] = now
        if not self._pose:
            return
        ranges = msg.ranges
        if not ranges:
            return
        robot_x = float(self._pose.get("x", 0.0))
        robot_y = float(self._pose.get("y", 0.0))
        robot_yaw = float(self._pose.get("yaw", 0.0))
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        lx_base = 0.20 if side == "front" else -0.20
        ly_base = 0.0
        origin_x = robot_x + cos_yaw * lx_base - sin_yaw * ly_base
        origin_y = robot_y + sin_yaw * lx_base + cos_yaw * ly_base
        step = max(1, len(ranges) // 120)
        points: list[tuple[float, float]] = []
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            beam = robot_yaw - angle
            points.append((origin_x + r * math.cos(beam), origin_y + r * math.sin(beam)))
        self.signals.obstacle_points_changed.emit(side, points)

    def start_patrol(self) -> None:
        """Kích hoạt bắt đầu tuần tra từ Mission Controller."""
        is_ready, reason = self.is_navigation_ready()
        if not is_ready:
            self.get_logger().error(f"[PATROL][ERROR] Từ chối start_patrol: {reason}")
            self.signals.command_status_changed.emit("warning", f"[ERROR] {reason}")
            return

        self.get_logger().info("[PATROL][INFO] Kích hoạt bắt đầu tuần tra (/mission/start_patrol)")
        client = getattr(self, "start_patrol_client", None)
        service_ok = False
        if client is not None:
            if hasattr(client, "service_is_ready") and client.service_is_ready():
                service_ok = True
            elif hasattr(client, "wait_for_service") and client.wait_for_service(timeout_sec=1.5):
                service_ok = True

        if not service_ok:
            self.get_logger().error("[PATROL][ERROR] Service /mission/start_patrol chưa sẵn sàng (Hãy kiểm tra Layer 5)!")
            self.signals.command_status_changed.emit(
                "warning",
                "[ERROR] Chưa kết nối Mission Controller (Layer 5)",
            )
            return
        req = Trigger.Request()
        future = self.start_patrol_client.call_async(req)
        future.add_done_callback(self._on_start_patrol_response)

    def _on_start_patrol_response(self, future: Any) -> None:
        try:
            res = future.result()
            tone = "success" if res.success else "warning"
            tag = "[INFO]" if res.success else "[ERROR]"
            if res.success:
                self.get_logger().info(f"[PATROL][INFO] Khởi động tuần tra thành công: {res.message}")
            else:
                self.get_logger().error(f"[PATROL][ERROR] Khởi động tuần tra thất bại: {res.message}")
            self.signals.command_status_changed.emit(tone, f"{tag} Tuần tra: {res.message}")
        except Exception as e:
            self.get_logger().error(f"[PATROL][ERROR] Lỗi gọi start_patrol: {e}")
            self.signals.command_status_changed.emit("error", f"[ERROR] Lỗi gọi tuần tra: {e}")

    def pause_patrol(self) -> None:
        """Tạm dừng hành trình tuần tra."""
        self.get_logger().info("[PATROL][INFO] Kích hoạt tạm dừng tuần tra (/mission/pause_patrol)")
        if not self.pause_patrol_client.service_is_ready():
            if not self.pause_patrol_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn("[PATROL][WARN] Service /mission/pause_patrol chưa sẵn sàng!")
                return
        req = Trigger.Request()
        self.pause_patrol_client.call_async(req)
        self.signals.command_status_changed.emit("warning", "[WARN] Đã tạm dừng tuần tra")

    def reload_route(self) -> None:
        """Yêu cầu Mission Controller nạp lại cấu hình routes.yaml."""
        self.get_logger().info("[MISSION] Nạp lại lộ trình tuần tra (/mission/reload_route)")
        if not self.reload_route_client.service_is_ready():
            if not self.reload_route_client.wait_for_service(timeout_sec=0.5):
                return
        req = Trigger.Request()
        self.reload_route_client.call_async(req)

    def _on_service_popup_cmd(self, msg: String) -> None:
        cmd = str(msg.data).strip()
        self.get_logger().info(f"Received service popup command: {cmd}")
        self.signals.service_popup_cmd.emit(cmd)

    def request_serve(self) -> None:
        """Kích hoạt trạng thái phục vụ nước (khóa bánh đứng yên)."""
        self.get_logger().info("Kích hoạt chế độ phục vụ nước...")
        self.serve_trigger_pub.publish(Bool(data=True))
        if self.trigger_serve_client.service_is_ready():
            self.trigger_serve_client.call_async(Trigger.Request())
        elif self.trigger_serve_client.wait_for_service(timeout_sec=0.2):
            self.trigger_serve_client.call_async(Trigger.Request())

    def finish_serving(self) -> None:
        """Xác nhận hoàn tất phục vụ để robot tiếp tục rời đi."""
        self.get_logger().info("Xác nhận hoàn tất phục vụ nước, tiếp tục di chuyển...")
        self.finish_serving_pub.publish(Bool(data=True))
        if self.trigger_finish_client.service_is_ready():
            self.trigger_finish_client.call_async(Trigger.Request())
        elif self.trigger_finish_client.wait_for_service(timeout_sec=0.2):
            self.trigger_finish_client.call_async(Trigger.Request())

    def _get_ws_root(self) -> FilePath:
        return FilePath(__file__).resolve().parents[4]

    def is_layer_running(self, layer_key: str) -> bool:
        """Kiểm tra xem tầng tiến trình có đang ở trạng thái RUNNING hay không."""
        canonical = self._canonical_layer_key(layer_key)
        if not (canonical.startswith("layer") and canonical[5:].isdigit()):
            canonical = self.LAYER_ALIAS_MAP.get(layer_key, "layer1")
        with self._lifecycle_lock:
            return self._layer_states.get(canonical) == LayerState.RUNNING

    @staticmethod
    def _get_clean_env() -> dict[str, str]:
        """Tạo môi trường thực thi sạch cho tiến trình con ROS 2/RViz, loại bỏ các biến xung đột từ OpenCV."""
        env = os.environ.copy()
        env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
        return env

    def is_sim_time(self) -> bool:
        """Kiểm tra xem node Kiosk có đang chạy ở chế độ use_sim_time (mô phỏng) không."""
        try:
            if hasattr(self, "has_parameter") and self.has_parameter("use_sim_time"):
                return bool(self.get_parameter("use_sim_time").value)
        except Exception:
            pass
        return False

    def _get_ws_prefix(self) -> str:
        ws_root = self._get_ws_root()
        ros_setup = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
        ws_setup = ws_root / "install" / "setup.bash"
        domain_id = os.environ.get("ROS_DOMAIN_ID", "94")
        cmd = f"export ROS_DOMAIN_ID={domain_id} && source '{ros_setup}'"
        if ws_setup.is_file():
            cmd += f" && source '{ws_setup}'"
        return cmd + " && "

    def start_slam(self) -> None:
        """Khởi chạy node SLAM mapping qua launch file real_slam.launch.py.

        Tự động kiểm tra và khởi chạy Layer 1 (phần cứng LiDAR & Odometry) nếu chưa chạy,
        đảm bảo slam_toolbox có dữ liệu /scan và TF để xây dựng map (chuẩn hóa theo gui_v3).
        """
        if self._slam_proc and self._slam_proc.poll() is None:
            self.get_logger().info("[KIOSK SLAM] Tiến trình SLAM đang chạy rồi.")
            self.signals.slam_status_changed.emit(True, "SLAM đang chạy")
            return

        def _slam_startup_worker() -> None:
            # 1. Tự động kiểm tra và kích hoạt Layer 1 nếu chưa chạy
            l1_proc = self._layer_procs.get("layer1")
            if not l1_proc or l1_proc.poll() is not None:
                self.get_logger().info(
                    "[KIOSK SLAM] Layer 1 (Hardware) chưa chạy. Tự động khởi động Layer 1 trước..."
                )
                self.signals.slam_status_changed.emit(True, "Đang khởi động phần cứng LiDAR & Odometry...")
                self.start_layer("layer1")
                # Đợi phần cứng khởi động và kích hoạt controller (tối thiểu 4.0s)
                time.sleep(4.0)

            sim_time_str = "true" if (hasattr(self, "is_sim_time") and self.is_sim_time()) else "false"
            cmd = self._get_ws_prefix() + f"ros2 launch mo_hinh real_slam.launch.py use_sim_time:={sim_time_str} use_rviz:=false"
            self.get_logger().info(f"[KIOSK SLAM] START: {cmd}")
            try:
                log_path = "/tmp/ros_slam.log"
                with open(log_path, "a", encoding="utf-8") as f_init:
                    f_init.write(f"\n--- BẮT ĐẦU SLAM MAPPING ({time.strftime('%Y-%m-%d %H:%M:%S')}) ---\n")
                    f_init.write(f"Lệnh: {cmd}\n")
                    f_init.flush()

                log_out = open(log_path, "a", encoding="utf-8")
                clean_env = self._get_clean_env() if hasattr(self, "_get_clean_env") else os.environ.copy()
                clean_env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
                self._slam_proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    env=clean_env,
                    stdout=log_out,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                self._layer_procs["layer3"] = self._slam_proc
                self._layer_procs["layer_slam"] = self._slam_proc
                self.signals.slam_status_changed.emit(True, "Đã khởi động SLAM Mapping")
            except Exception as exc:
                self.get_logger().error(f"[KIOSK SLAM] Lỗi khởi chạy SLAM: {exc}")
                self.signals.slam_status_changed.emit(False, f"Lỗi khởi động SLAM: {exc}")

        threading.Thread(target=_slam_startup_worker, daemon=True).start()

    def stop_slam(self) -> None:
        """Dừng tiến trình SLAM an toàn trên luồng nền, không chặn UI thread."""
        proc = self._slam_proc
        if not proc or proc.poll() is not None:
            self.get_logger().info("[KIOSK SLAM] SLAM chưa chạy hoặc đã dừng.")
            self._slam_proc = None
            self._layer_procs.pop("layer3", None)
            self._layer_procs.pop("layer_slam", None)
            self.signals.slam_status_changed.emit(False, "SLAM đã dừng")
            return

        self.get_logger().info("[KIOSK SLAM] STOP SLAM process group (background)")
        self._slam_proc = None
        self._layer_procs.pop("layer3", None)
        self._layer_procs.pop("layer_slam", None)

        def _stop_slam_worker(target_proc: subprocess.Popen) -> None:
            try:
                pgid = os.getpgid(target_proc.pid)
                os.killpg(pgid, signal.SIGINT)
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if target_proc.poll() is not None:
                        break
                    time.sleep(0.05)
                if target_proc.poll() is None:
                    os.killpg(pgid, signal.SIGTERM)
            except Exception as exc:
                self.get_logger().warning(f"[KIOSK SLAM] Dừng SLAM gặp lỗi: {exc}")
            finally:
                self.signals.slam_status_changed.emit(False, "SLAM đã dừng")

        threading.Thread(target=_stop_slam_worker, args=(proc,), daemon=True).start()

    def save_map(self, map_stem: str) -> None:
        """Thực hiện lưu bản đồ kép (AMCL YAML + PoseGraph serialize) trên background thread."""
        def _save_worker():
            raw_name = map_stem.strip() or "new_map"
            ws_root = FilePath(__file__).resolve().parents[4]
            map_dir = ws_root / "my_map"
            map_dir.mkdir(parents=True, exist_ok=True)

            target_stem = str((map_dir / raw_name).resolve())
            amcl_stem = f"{target_stem}_amcl"
            localization_stem = f"{target_stem}_localization"
            serialize_req = shlex.quote(f'{{filename: "{localization_stem}"}}')

            cmd = (
                self._get_ws_prefix()
                + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(amcl_stem)} && "
                + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(localization_stem)} && "
                + f"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph {serialize_req}"
            )
            self.get_logger().info(f"[KIOSK MAP SAVE] Chạy lệnh: {cmd}")
            try:
                proc = subprocess.run(
                    ["bash", "-c", cmd],
                    capture_output=True,
                    text=True,
                    timeout=25,
                )
                if proc.returncode == 0:
                    msg = f"Đã lưu bản đồ: {raw_name}_amcl.yaml và {raw_name}_localization.posegraph"
                    self.get_logger().info(f"[KIOSK MAP SAVE] Thành công: {msg}")
                    self.signals.map_saved_result.emit(True, msg)
                else:
                    err_msg = f"Lỗi khi lưu (mã lỗi {proc.returncode}): {proc.stderr[:100]}"
                    self.get_logger().error(f"[KIOSK MAP SAVE] {err_msg}")
                    self.signals.map_saved_result.emit(False, err_msg)
            except Exception as exc:
                self.get_logger().error(f"[KIOSK MAP SAVE] Ngoại lệ: {exc}")
                self.signals.map_saved_result.emit(False, f"Lỗi: {exc}")

        threading.Thread(target=_save_worker, daemon=True).start()

    def apply_localization_map(self, map_yaml_path: str, backend: str = "amcl") -> None:
        """Lưu cấu hình và khởi chạy chế độ định vị với bản đồ được chọn qua layer2 duy nhất."""
        self._selected_map_yaml = map_yaml_path
        self._selected_backend = backend
        if hasattr(self, "cancel_pending_initial_pose"):
            self.cancel_pending_initial_pose()
        if hasattr(self, "_set_localization_state"):
            self._set_localization_state(
                LocalizationState.NOT_INITIALIZED,
                f"Đã nạp bản đồ: {FilePath(map_yaml_path).name}"
            )
        self.get_logger().info(
            f"[KIOSK LOCALIZATION] Áp dụng bản đồ: {map_yaml_path} (Backend: {backend})"
        )
        self.stop_slam()

        def _loc_worker():
            # 1. Dừng Layer 2 cũ nếu đang chạy để tránh duplicate localization stack
            if hasattr(self, "stop_layer") and hasattr(self, "is_layer_running"):
                if self.is_layer_running("layer2") or (
                    self._layer_procs.get("layer2") and self._layer_procs["layer2"].poll() is None
                ):
                    self.get_logger().info("[KIOSK LOCALIZATION] Dừng phiên Layer 2 cũ trước khi nạp map...")
                    self.stop_layer("layer2")
                    deadline = time.monotonic() + 3.0
                    while time.monotonic() < deadline and self.is_layer_running("layer2"):
                        time.sleep(0.05)

            if hasattr(self, "_localization_proc") and self._localization_proc and self._localization_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._localization_proc.pid), signal.SIGINT)
                    deadline = time.monotonic() + 1.5
                    while time.monotonic() < deadline and self._localization_proc.poll() is None:
                        time.sleep(0.05)
                    if self._localization_proc.poll() is None:
                        os.killpg(os.getpgid(self._localization_proc.pid), signal.SIGTERM)
                except Exception:
                    pass
                self._localization_proc = None

            # 2. Khởi chạy Layer 2 đồng bộ qua start_layer nếu chưa chạy
            if hasattr(self, "start_layer") and hasattr(self, "_lifecycle_lock"):
                amcl_already_active = False
                if hasattr(self, "check_amcl_lifecycle_active"):
                    amcl_already_active, _ = self.check_amcl_lifecycle_active()
                layer2_already_running = self.is_layer_running("layer2") or self.is_layer_running("layer_navigation")
                if not amcl_already_active and not layer2_already_running:
                    self.get_logger().info("[KIOSK LOCALIZATION] Khởi động Layer 2 đồng nhất qua start_layer...")
                    self.start_layer("layer2")
                else:
                    self.get_logger().info("[KIOSK LOCALIZATION] AMCL / Layer 2 đã đang chạy, tái sử dụng không khởi chạy trùng lặp.")
            else:
                # Fallback cho unit test mock độc lập (DummyRosNode)
                graph_path = os.path.splitext(map_yaml_path)[0]
                if backend == "slam_toolbox":
                    posegraph_file = graph_path + ".posegraph"
                    if not os.path.exists(posegraph_file):
                        self.get_logger().warn(
                            f"[KIOSK LOCALIZATION] File posegraph '{posegraph_file}' không tồn tại trên đĩa. "
                            "SLAM Toolbox có thể cần file posegraph hợp lệ để định vị!"
                        )
                sim_time_str = "true" if (hasattr(self, "is_sim_time") and self.is_sim_time()) else "false"
                cmd = (
                    self._get_ws_prefix()
                    + "ros2 launch mo_hinh real_localization_nav2.launch.py "
                    + f"localization_mode:={shlex.quote(backend)} use_sim_time:={sim_time_str} "
                    + f"map_yaml:={shlex.quote(map_yaml_path)} "
                    + f"map_graph:={shlex.quote(graph_path)} use_rviz:=false"
                )
                self.get_logger().info(f"[KIOSK LOCALIZATION] START fallback: {cmd}")
                clean_env = self._get_clean_env() if hasattr(self, "_get_clean_env") else os.environ.copy()
                clean_env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
                try:
                    log_path = "/tmp/ros_layer_layer2.log"
                    log_out = open(log_path, "a", encoding="utf-8")
                    self._localization_proc = subprocess.Popen(
                        ["bash", "-c", cmd],
                        env=clean_env,
                        stdout=log_out,
                        stderr=subprocess.STDOUT,
                        text=True,
                        preexec_fn=os.setsid,
                    )
                except Exception as exc:
                    self.get_logger().error(f"[KIOSK LOCALIZATION] Lỗi: {exc}")

            if getattr(self, "_auto_initial_pose_enabled", False) and backend != "slam_toolbox":
                def _auto_pose_worker():
                    time.sleep(2.0)
                    for _ in range(15):
                        if getattr(self, "_selected_map_yaml", "") != map_yaml_path:
                            return
                        if hasattr(self, "check_localization_readiness"):
                            ready, _ = self.check_localization_readiness()
                            if ready:
                                if hasattr(self, "trigger_auto_initial_pose_if_enabled"):
                                    self.trigger_auto_initial_pose_if_enabled()
                                return
                        time.sleep(1.0)
                threading.Thread(target=_auto_pose_worker, daemon=True).start()

        threading.Thread(target=_loc_worker, daemon=True).start()

    # -------------------------------------------------------------------------
    # AMCL Initial Pose & Localization Management
    # -------------------------------------------------------------------------
    def _set_localization_state(self, state: LocalizationState, message: str = "") -> None:
        self._localization_state = state
        self._localization_message = message
        self.signals.localization_status_changed.emit(state.value, message)

    def get_localization_status(self) -> dict[str, Any]:
        """Trả về snapshot chi tiết trạng thái Localization hiện tại."""
        return {
            "state": self._localization_state.value,
            "message": self._localization_message,
            "covariance_norm": self._amcl_covariance_norm,
            "last_initial_pose_sent_at": self._last_initial_pose_sent_at,
            "last_amcl_pose_at": self._last_amcl_pose_at,
            "have_localized_pose": self._have_localized_pose,
            "auto_initial_pose_enabled": self._auto_initial_pose_enabled,
        }

    def set_auto_initial_pose_enabled(self, enabled: bool) -> None:
        """Bật/tắt chế độ tự động định vị robot vào điểm xuất phát khi nạp map."""
        self._auto_initial_pose_enabled = bool(enabled)
        self.get_logger().info(f"[LOCALIZATION] Auto Initial Pose: {self._auto_initial_pose_enabled}")

    def check_amcl_lifecycle_active(self, timeout_sec: float = 0.5) -> tuple[bool, str]:
        """Kiểm tra lifecycle state thực tế của node /amcl qua GetState service."""
        if self._selected_backend == "slam_toolbox":
            return True, "Chế độ SLAM Toolbox không yêu cầu AMCL"

        # Tối ưu hóa phản hồi: Nếu gần đây (< 5s) đã nhận /amcl_pose thì AMCL chắc chắn đang active
        now = time.monotonic()
        if self._last_amcl_pose_at > 0 and (now - self._last_amcl_pose_at) < 5.0:
            return True, "AMCL đang ở trạng thái active (đang phát /amcl_pose)"

        if not hasattr(self, "_amcl_get_state_client") or self._amcl_get_state_client is None:
            return True, "Không có client lifecycle_msgs, bỏ qua kiểm tra lifecycle"

        if not self._amcl_get_state_client.service_is_ready():
            return False, "Dịch vụ /amcl/get_state chưa sẵn sàng (AMCL node chưa online)"

        req = GetState.Request()
        future = self._amcl_get_state_client.call_async(req)
        start_t = time.monotonic()
        # Nếu đang ở Main Thread, dùng timeout ngắn (80ms) để không block Qt event loop
        actual_timeout = 0.08 if threading.current_thread() is threading.main_thread() else timeout_sec
        while not future.done() and (time.monotonic() - start_t) < actual_timeout:
            time.sleep(0.01)

        if not future.done():
            return False, "Hết thời gian chờ phản hồi từ /amcl/get_state (timeout)"

        try:
            res = future.result()
            curr_id = getattr(res.current_state, "id", None)
            curr_label = str(getattr(res.current_state, "label", "")).lower()
            if curr_id == 3 or curr_label == "active":
                return True, "AMCL đang ở trạng thái active"
            return False, f"AMCL chưa active (trạng thái: {curr_label or curr_id})"
        except Exception as exc:
            return False, f"Lỗi khi đọc trạng thái AMCL: {exc}"

    def check_localization_readiness(self) -> tuple[bool, str]:
        """Kiểm tra điều kiện tiên quyết trước khi phát Initial Pose hoặc chạy AMCL."""
        if self._selected_backend == "slam_toolbox":
            return True, "Chế độ SLAM Toolbox không yêu cầu AMCL Initial Pose"

        # 1. Kiểm tra Layer 2 (Nav2 & AMCL) hoặc node /amcl đang chạy bên ngoài
        layer2_running = (
            self.is_layer_running("layer2")
            or self.is_layer_running("layer_navigation")
            or (self._localization_proc is not None and self._localization_proc.poll() is None)
        )
        layer2_explicitly_stopped = self._layer_states.get("layer2") == LayerState.STOPPED
        external_amcl_online = (
            not layer2_explicitly_stopped
            and hasattr(self, "_amcl_get_state_client")
            and self._amcl_get_state_client is not None
            and self._amcl_get_state_client.service_is_ready()
        )
        if not layer2_running and not external_amcl_online:
            return False, "Layer 2 (Nav2 & AMCL) chưa khởi chạy hoặc chưa ở trạng thái RUNNING"

        # 2. Kiểm tra AMCL lifecycle state thực tế
        amcl_ok, amcl_msg = self.check_amcl_lifecycle_active()
        if not amcl_ok:
            return False, amcl_msg

        # 3. Kiểm tra dữ liệu cảm biến (freshness < 2.0s)
        now = time.monotonic()
        if self._last_odom_time == 0.0 or (now - self._last_odom_time) > 2.0:
            return False, "Chưa nhận được dữ liệu Odometry (/odom) hợp lệ hoặc bị gián đoạn (>2s)"

        if self._last_scan_time == 0.0 or (now - self._last_scan_time) > 2.0:
            return False, "Chưa nhận được dữ liệu LiDAR (/scan) hợp lệ hoặc bị gián đoạn (>2s)"

        # 4. Kiểm tra TF odom -> base_footprint (hoặc base_link)
        odom_tf_ok = False
        for target_child in ("base_footprint", "base_link"):
            try:
                if self._tf_buffer.can_transform("odom", target_child, rclpy.time.Time()):
                    odom_tf_ok = True
                    break
            except Exception:
                pass
        if not odom_tf_ok:
            return False, "Chưa có liên kết TF từ odom -> base_footprint"

        return True, "Sẵn sàng nhận vị trí ban đầu (Ready to set pose)"

    def trigger_auto_initial_pose_if_enabled(self) -> bool:
        """Tự động kích hoạt Initial Pose nếu chế độ auto được bật và đủ điều kiện."""
        if not self._auto_initial_pose_enabled:
            return False
        if self._selected_backend == "slam_toolbox":
            return False
        start_pose = self.resolve_start_pose_for_map(self._selected_map_yaml)
        if not start_pose:
            self.get_logger().info("[AUTO INITIAL POSE] Không tìm thấy điểm xuất phát khoihanh cho bản đồ hiện tại.")
            return False
        ready, reason = self.check_localization_readiness()
        if not ready:
            self.get_logger().warn(f"[AUTO INITIAL POSE] Chưa sẵn sàng: {reason}")
            return False
        self.get_logger().info(f"[AUTO INITIAL POSE] Tự động phát Initial Pose: {start_pose}")
        return self.publish_initial_pose(
            x=start_pose["x"],
            y=start_pose["y"],
            yaw=start_pose["yaw"],
            map_name=self._selected_map_yaml,
        )

    def is_navigation_ready(self) -> tuple[bool, str]:
        """Localization Readiness Gate: Chặn Navigation Goal / Patrol nếu robot chưa sẵn sàng."""
        if self._selected_backend == "slam_toolbox":
            return True, "Sẵn sàng điều hướng (SLAM Toolbox)"

        if self._localization_state != LocalizationState.LOCALIZED:
            return False, (
                "Robot chưa hoàn tất định vị ban đầu (Initial Pose).\n"
                f"Trạng thái hiện tại: {self._localization_state.value} ({self._localization_message}).\n"
                "Vui lòng vào 'ĐỊNH VỊ ROBOT' để xác nhận vị trí trước khi chạy điều hướng!"
            )
        return True, "Hệ thống định vị sẵn sàng"

    def cancel_pending_initial_pose(self) -> None:
        """Hủy phiên định vị đang chờ (dùng khi đổi map, dừng Layer 2 hoặc người dùng hủy)."""
        with self._lifecycle_lock:
            self._loc_session_token = ""
            if self._localization_state in (
                LocalizationState.PUBLISHING,
                LocalizationState.WAITING_FOR_LOCALIZATION,
                LocalizationState.TF_AVAILABLE,
            ):
                self._set_localization_state(
                    LocalizationState.NOT_INITIALIZED,
                    "Đã hủy phiên định vị ban đầu"
                )

    def resolve_start_pose_for_map(self, map_path: str | FilePath | None = None) -> dict[str, Any] | None:
        """Đọc waypoint 'khoihanh' từ file <map_stem>_waypoints.yaml của bản đồ tương ứng."""
        target_path = map_path or self._selected_map_yaml
        if not target_path:
            return None
        p = FilePath(target_path).resolve()
        stem = p.stem
        candidates = [
            p.parent / f"{stem}_waypoints.yaml",
            self._get_ws_root() / "my_map" / f"{stem}_waypoints.yaml",
        ]
        for c in candidates:
            if c.is_file():
                try:
                    with c.open("r", encoding="utf-8") as f:
                        data = yaml.safe_load(f) or {}
                    locs = data.get("locations", {}) if "locations" in data else data
                    if isinstance(locs, dict) and "khoihanh" in locs:
                        kh = locs["khoihanh"]
                        if isinstance(kh, dict) and "x" in kh and "y" in kh:
                            return {
                                "name": "khoihanh",
                                "x": float(kh.get("x", 0.0)),
                                "y": float(kh.get("y", 0.0)),
                                "yaw": float(kh.get("yaw", 0.0)),
                                "note": str(kh.get("note", "Điểm xuất phát")),
                                "source_file": str(c),
                            }
                except Exception:
                    pass
        return None

    def publish_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        cov_xy: float = 0.25,
        cov_yaw: float = 0.0685,
        map_name: str = "",
    ) -> bool:
        """Phát tọa độ PoseWithCovarianceStamped lên /initialpose để AMCL định vị robot."""
        if not math.isfinite(x) or not math.isfinite(y) or not math.isfinite(yaw):
            self.get_logger().error(f"[ESTIMATE][ERROR] Tọa độ không hợp lệ: x={x}, y={y}, yaw={yaw}")
            self._set_localization_state(LocalizationState.ERROR, "[ERROR] Tọa độ hoặc góc yaw không hợp lệ (NaN/Inf)")
            return False

        if map_name and not getattr(self, "_selected_map_yaml", None):
            self._selected_map_yaml = map_name

        if map_name:
            req_stem = FilePath(map_name).stem.lower()
            curr_stem = FilePath(self._selected_map_yaml).stem.lower() if self._selected_map_yaml else ""
            if curr_stem and req_stem != curr_stem:
                reason = f"Bản đồ yêu cầu ({req_stem}) không khớp với bản đồ đang mở ({curr_stem})"
                self.get_logger().error(f"[ESTIMATE][ERROR] {reason}")
                self._set_localization_state(LocalizationState.ERROR, f"[ERROR] {reason}")
                return False

        ready, reason = self.check_localization_readiness()
        if not ready:
            self.get_logger().warn(f"[ESTIMATE][WARN] Không thể phát Initial Pose: {reason}")
            self._set_localization_state(LocalizationState.ERROR, f"[ERROR] {reason}")
            return False

        normalized_yaw = math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(normalized_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(normalized_yaw / 2.0)

        cov = [0.0] * 36
        cov[0] = float(cov_xy)
        cov[7] = float(cov_xy)
        cov[35] = float(cov_yaw)
        msg.pose.covariance = cov

        token = uuid.uuid4().hex[:8]
        self._loc_session_token = token
        self._last_initial_pose_sent_at = time.monotonic()

        self._set_localization_state(LocalizationState.PUBLISHING, "[INFO] Đang gửi tọa độ tới AMCL...")
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"[ESTIMATE][INFO] Đã phát /initialpose (token={token}): x={x:.3f}, y={y:.3f}, yaw={normalized_yaw:.3f} rad"
        )
        self._set_localization_state(
            LocalizationState.WAITING_FOR_LOCALIZATION,
            f"[INFO] Đã phát Initial Pose ({x:.2f}, {y:.2f}), chờ AMCL xác nhận..."
        )

        threading.Thread(
            target=self._verify_localization_worker,
            args=(token,),
            daemon=True,
        ).start()
        return True

    def _verify_localization_worker(self, token: str) -> None:
        """Worker nền kiểm tra TF map -> odom từ AMCL sau khi gửi Initial Pose (chuẩn gui_v3)."""
        start_t = time.monotonic()
        timeout = 10.0
        tf_detected = False
        MAX_COVARIANCE_NORM = 1.20

        while time.monotonic() - start_t < timeout:
            if self._loc_session_token != token:
                return

            tf_ok = False
            try:
                tf_ok = self._tf_buffer.can_transform("map", "odom", rclpy.time.Time())
            except Exception:
                pass

            if tf_ok:
                tf_detected = True
                cov_ok = self._amcl_covariance_norm <= MAX_COVARIANCE_NORM

                if cov_ok:
                    self._loc_session_token = ""
                    self.get_logger().info(
                        f"[ESTIMATE][INFO] Định vị thành công (TF map->odom OK, sai số: {self._amcl_covariance_norm:.2f}m)"
                    )
                    self._set_localization_state(
                        LocalizationState.LOCALIZED,
                        f"[INFO] Đã định vị thành công (TF map->odom OK, sai số: {self._amcl_covariance_norm:.2f}m)"
                    )
                    self.signals.initial_pose_confirmed.emit({
                        "token": token,
                        "covariance_norm": self._amcl_covariance_norm,
                        "timestamp": self._last_amcl_pose_at or time.monotonic(),
                    })
                    return
                else:
                    self.get_logger().warn(
                        f"[ESTIMATE][WARN] Có liên kết TF nhưng độ phân tán hạt quá lớn ({self._amcl_covariance_norm:.2f}m > {MAX_COVARIANCE_NORM}m)"
                    )
                    self._set_localization_state(
                        LocalizationState.TF_AVAILABLE,
                        f"[WARN] Có liên kết TF nhưng độ phân tán hạt quá lớn ({self._amcl_covariance_norm:.2f}m > {MAX_COVARIANCE_NORM}m)"
                    )
            else:
                if self._localization_state == LocalizationState.WAITING_FOR_LOCALIZATION:
                    self._set_localization_state(
                        LocalizationState.WAITING_FOR_LOCALIZATION,
                        "[INFO] Đang chờ liên kết TF map -> odom từ AMCL..."
                    )

            if self._localization_state == LocalizationState.LOCALIZED:
                self._loc_session_token = ""
                return

            time.sleep(0.10)

        if self._loc_session_token == token and self._localization_state != LocalizationState.LOCALIZED:
            self._loc_session_token = ""
            if tf_detected:
                if self._amcl_covariance_norm > MAX_COVARIANCE_NORM:
                    self.get_logger().error(
                        f"[ESTIMATE][ERROR] Định vị thất bại: Độ phân tán hạt quá lớn ({self._amcl_covariance_norm:.2f}m > {MAX_COVARIANCE_NORM}m)"
                    )
                    self._set_localization_state(
                        LocalizationState.ERROR,
                        f"[ERROR] Định vị thất bại: Độ phân tán hạt quá lớn ({self._amcl_covariance_norm:.2f}m > {MAX_COVARIANCE_NORM}m)"
                    )
                else:
                    self.get_logger().error(
                        "[ESTIMATE][ERROR] Hết thời gian chờ (10s): Chưa nhận được ước lượng hội tụ từ /amcl_pose"
                    )
                    self._set_localization_state(
                        LocalizationState.ERROR,
                        "[ERROR] Hết thời gian chờ (10s): Chưa nhận được ước lượng hội tụ từ /amcl_pose"
                    )
            else:
                self.get_logger().error(
                    "[ESTIMATE][ERROR] Hết thời gian chờ (10s): AMCL không tạo được liên kết TF map -> odom"
                )
                self._set_localization_state(
                    LocalizationState.ERROR,
                    "[ERROR] Hết thời gian chờ (10s): AMCL không tạo được liên kết TF map -> odom"
                )

    def stop_subprocesses(self) -> None:
        """Dọn dẹp các tiến trình nền khi đóng ứng dụng."""
        self.cancel_pending_initial_pose()
        self.stop_slam()
        if self._localization_proc and self._localization_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._localization_proc.pid), signal.SIGTERM)
            except Exception:
                pass
        for p in self._layer_procs.values():
            if p and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass

    # -------------------------------------------------------------------------
    # System Telemetry & Configuration Methods
    # -------------------------------------------------------------------------
    def _mark_topic_rx(self, key: str, extra: dict[str, Any] | None = None) -> None:
        now = time.monotonic()
        with self._dbg_lock:
            st = self._topic_stats.get(key)
            if st is None:
                self._topic_stats[key] = {
                    "last_ts": now,
                    "count": 1,
                    "hz": 0.0,
                    "last_hz_calc": now,
                    "period_count": 1,
                    "extra": extra or {},
                }
            else:
                st["count"] += 1
                st["period_count"] += 1
                st["last_ts"] = now
                if extra is not None:
                    st["extra"].update(extra)
                dt = now - st["last_hz_calc"]
                if dt >= 1.0:
                    st["hz"] = st["period_count"] / dt
                    st["period_count"] = 0
                    st["last_hz_calc"] = now

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._mark_topic_rx("cmd_vel", {"vx": float(msg.linear.x), "wz": float(msg.angular.z)})

    def _on_hoverboard_voltage(self, msg: Float64) -> None:
        raw_v = float(msg.data)
        if 20.0 <= raw_v <= 50.0:
            self._battery_voltages.append(raw_v)
            smooth_v = sum(self._battery_voltages) / len(self._battery_voltages)
            self._battery_raw_v = raw_v
            self._battery_smooth_v = smooth_v
            self._battery_pct = voltage_to_percentage_36v(smooth_v)
            self._battery_source = "hoverboard (/hoverboard/battery_voltage)"
            self._emit_battery_water()

    def _on_battery_state(self, msg: BatteryState) -> None:
        raw_v = float(msg.voltage)
        if 20.0 <= raw_v <= 50.0:
            self._battery_voltages.append(raw_v)
            smooth_v = sum(self._battery_voltages) / len(self._battery_voltages)
            self._battery_raw_v = raw_v
            self._battery_smooth_v = smooth_v
            self._battery_pct = voltage_to_percentage_36v(smooth_v)
            self._battery_source = "battery (/battery)"
            self._emit_battery_water()

    def _on_wave_detected(self, msg: Bool) -> None:
        self._is_waving = bool(msg.data)

    def _on_tracked_people(self, msg: Any) -> None:
        self._last_ai_rx = time.monotonic()
        self._yolo_active = True
        people = getattr(msg, "people", [])
        self._people_count = len(people)
        distances: list[float] = []
        for p in people:
            d = getattr(p, "distance", 0.0)
            if d > 0.0:
                distances.append(float(d))
        self._closest_person_dist = min(distances) if distances else None

    def _on_person_distance_preview(self, msg: Float32) -> None:
        self._last_ai_rx = time.monotonic()
        d = float(msg.data)
        if d > 0.0:
            self._closest_person_dist = d

    def _emit_battery_water(self) -> None:
        self.signals.robot_status_changed.emit(
            {
                "battery_percent": self._battery_pct,
                "voltage_smooth": self._battery_smooth_v,
                "voltage_raw": self._battery_raw_v,
                "voltage_source": self._battery_source,
                "water_liter": self._water_liter,
                "kettle_present": self._kettle_present,
            }
        )

    def _publish_system_telemetry(self) -> None:
        now = time.monotonic()
        if (now - self._last_ai_rx) > 3.0:
            self._yolo_active = False
            self._people_count = 0
            self._closest_person_dist = None
        with self._dbg_lock:
            def _get_hz(k: str) -> float:
                entry = self._topic_stats.get(k)
                if not entry or (now - entry["last_ts"]) > 2.0:
                    return 0.0
                return float(entry.get("hz", 0.0))

            lidar_hz = _get_hz("lidar")
            odom_hz = _get_hz("odom")
            cam_front_hz = _get_hz("cam_front")
            cam_rear_hz = _get_hz("cam_rear")
            cam_ai_hz = _get_hz("cam_ai")

            lidar_extra = self._topic_stats.get("lidar", {}).get("extra", {})
            odom_extra = self._topic_stats.get("odom", {}).get("extra", {})
            obs_f_extra = self._topic_stats.get("obs_front", {}).get("extra", {})
            obs_r_extra = self._topic_stats.get("obs_rear", {}).get("extra", {})

        # TF checks
        tf_map_ok = False
        tf_odom_ok = False
        try:
            tf_map_ok = self._tf_buffer.can_transform("map", "odom", rclpy.time.Time())
        except Exception:
            pass
        try:
            tf_odom_ok = self._tf_buffer.can_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception:
            pass

        snap = {
            "lidar_hz": lidar_hz,
            "lidar_min_dist": lidar_extra.get("min_dist"),
            "odom_hz": odom_hz,
            "vx": odom_extra.get("vx", 0.0),
            "wz": odom_extra.get("wz", 0.0),
            "cam_front_hz": cam_front_hz,
            "cam_rear_hz": cam_rear_hz,
            "cam_ai_hz": cam_ai_hz,
            "obs_front_dist": obs_f_extra.get("min_dist"),
            "obs_rear_dist": obs_r_extra.get("min_dist"),
            "tf_map_odom_ok": tf_map_ok,
            "tf_odom_base_ok": tf_odom_ok,
            "yolo_active": self._yolo_active,
            "people_count": self._people_count,
            "is_waving": self._is_waving,
            "closest_person_dist": self._closest_person_dist,
        }
        self.signals.system_telemetry_changed.emit(snap)

        # Single Source of Truth: đọc trực tiếp từ self._layer_states dưới lock, bảo đảm không bao giờ xung đột
        with self._lifecycle_lock:
            # Phát hiện các tiến trình chết ngoài ý muốn (T12)
            for c_k in ("layer1", "layer2", "layer3", "layer4", "layer5"):
                st = self._layer_states.get(c_k, LayerState.STOPPED)
                if st == LayerState.RUNNING:
                    pr = self._layer_procs.get(c_k)
                    if pr is not None and pr.poll() is not None:
                        exit_code = pr.poll()
                        new_st = LayerState.STOPPED if exit_code == 0 else LayerState.ERROR
                        self._set_layer_state(c_k, new_st)
                        self._layer_procs.pop(c_k, None)
                        alt = self.LAYER_ALIAS_MAP.get(c_k, "")
                        if alt:
                            self._layer_procs.pop(alt, None)

            # Lấy snapshot trạng thái tầng đồng nhất
            layer_states = {}
            for c_k in ("layer1", "layer2", "layer3", "layer4", "layer5"):
                s_val = self._layer_states.get(c_k, LayerState.STOPPED).value
                alt_k = self.LAYER_ALIAS_MAP.get(c_k, "")
                layer_states[c_k] = s_val
                if alt_k:
                    layer_states[alt_k] = s_val

        self.signals.layer_status_changed.emit(layer_states)

    def test_actuator(self, test_name: str) -> None:
        """Kích hoạt kiểm tra an toàn cơ cấu chấp hành (đã loại bỏ Pump và LED/Buzzer)."""
        self.get_logger().info(f"[TEST ACTUATOR] Kích hoạt: {test_name}")
        if test_name == "speaker":
            def _sound_worker():
                sound_file = "/usr/share/sounds/freedesktop/stereo/complete.oga"
                try:
                    res = subprocess.run(
                        ["paplay", sound_file],
                        capture_output=True,
                        text=True,
                        timeout=3.0,
                    )
                    if res.returncode == 0:
                        self.signals.actuator_test_result.emit("speaker", True, "Đã phát âm thanh kiểm tra loa ngoài thành công.")
                    else:
                        self.signals.actuator_test_result.emit("speaker", False, f"Lỗi paplay (mã {res.returncode}): {res.stderr.strip()}")
                except Exception as exc:
                    self.signals.actuator_test_result.emit("speaker", False, f"Không thể phát loa: {exc}")

            threading.Thread(target=_sound_worker, daemon=True).start()
        elif test_name == "jog_motor":
            def _jog_worker():
                try:
                    tw = Twist()
                    tw.linear.x = 0.05
                    self.cmd_vel_pub.publish(tw)
                    time.sleep(1.0)
                    stop_tw = Twist()
                    self.cmd_vel_pub.publish(stop_tw)
                    self.signals.actuator_test_result.emit("jog_motor", True, "Đã nhích xe 5cm an toàn và phanh dừng.")
                except Exception as exc:
                    self.signals.actuator_test_result.emit("jog_motor", False, f"Lỗi nhích xe: {exc}")

            threading.Thread(target=_jog_worker, daemon=True).start()
        else:
            self.get_logger().warn(f"[TEST ACTUATOR] Bỏ qua test không hợp lệ hoặc đã loại bỏ: {test_name}")

    CANONICAL_LAYER_MAP = {
        "layer_hardware": "layer1",
        "layer_navigation": "layer2",
        "layer_slam": "layer3",
        "layer_perception": "layer4",
        "layer_voice_interaction": "layer5",
        "layer_backend": "layer5",
        "layer_mission": "layer5",
    }

    LAYER_ALIAS_MAP = {
        "layer_hardware": "layer1",
        "layer_navigation": "layer2",
        "layer_slam": "layer3",
        "layer_perception": "layer4",
        "layer_voice_interaction": "layer5",
        "layer_backend": "layer5",
        "layer_mission": "layer5",
        "layer1": "layer_hardware",
        "layer2": "layer_navigation",
        "layer3": "layer_slam",
        "layer4": "layer_perception",
        "layer5": "layer_voice_interaction",
    }

    def _canonical_layer_key(self, layer_key: str) -> str:
        k = layer_key.lower().strip()
        return self.CANONICAL_LAYER_MAP.get(k, k)

    def _record_layer_log(self, layer_key: str, text: str) -> None:
        """Lưu log vào bounded buffer và chỉ phát Qt signal nếu console đang mở."""
        norm_key = self._canonical_layer_key(layer_key)
        now_str = time.strftime("%H:%M:%S")
        prefix = norm_key.upper() if norm_key != "all" else "SYSTEM"
        formatted_line = f"[{now_str}] [{prefix}] {text}"

        if norm_key not in self._layer_log_buffers:
            self._layer_log_buffers[norm_key] = deque(maxlen=400)
        self._layer_log_buffers[norm_key].append(formatted_line)
        if norm_key != "all":
            self._layer_log_buffers["all"].append(formatted_line)
        if self._console_active:
            self.signals.layer_log_received.emit(norm_key, text)

    def set_console_active(self, active: bool) -> None:
        """Bật/tắt trạng thái hiển thị của Live Log Console."""
        self._console_active = active

    def get_layer_log_snapshot(self, layer_key: str = "all") -> list[str]:
        """Lấy snapshot danh sách các dòng log gần nhất từ bounded buffer theo layer."""
        norm_key = self._canonical_layer_key(layer_key)
        buf = self._layer_log_buffers.get(norm_key)
        return list(buf) if buf else []

    def _set_layer_state(self, canonical_key: str, state: LayerState) -> None:
        """Cập nhật trạng thái duy nhất cho layer và đồng bộ các tập phụ trợ (phải gọi dưới _lifecycle_lock)."""
        self._layer_states[canonical_key] = state
        alt_key = self.LAYER_ALIAS_MAP.get(canonical_key, "")

        if state == LayerState.STARTING:
            self._starting_layers.add(canonical_key)
            if alt_key:
                self._starting_layers.add(alt_key)
        else:
            self._starting_layers.discard(canonical_key)
            if alt_key:
                self._starting_layers.discard(alt_key)

        if state == LayerState.STOPPING:
            self._stopping_layers.add(canonical_key)
            if alt_key:
                self._stopping_layers.add(alt_key)
        else:
            self._stopping_layers.discard(canonical_key)
            if alt_key:
                self._stopping_layers.discard(alt_key)

        if state == LayerState.ERROR:
            self._error_layers.add(canonical_key)
            if alt_key:
                self._error_layers.add(alt_key)
        else:
            self._error_layers.discard(canonical_key)
            if alt_key:
                self._error_layers.discard(alt_key)

    def get_layer_state(self, layer_key: str) -> LayerState:
        """Lấy trạng thái chuẩn hiện tại của layer bảo đảm thread-safe."""
        canonical = self._canonical_layer_key(layer_key)
        with self._lifecycle_lock:
            return self._layer_states.get(canonical, LayerState.STOPPED)

    @staticmethod
    def _terminate_process_group(proc: subprocess.Popen | None, timeout: float = 1.5) -> bool:
        """Dừng an toàn process group: SIGINT -> chờ timeout -> SIGKILL -> chờ xác nhận đã chết hoàn toàn."""
        if proc is None or proc.poll() is not None:
            return True
        try:
            pgid = os.getpgid(proc.pid)
        except (ProcessLookupError, OSError):
            return proc.poll() is not None

        # 1. Gửi SIGINT
        try:
            os.killpg(pgid, signal.SIGINT)
        except (ProcessLookupError, OSError):
            pass

        # Chờ có timeout cho SIGINT
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.05)

        # 2. Nếu vẫn chưa thoát, gửi SIGKILL
        try:
            os.killpg(pgid, signal.SIGKILL)
        except (ProcessLookupError, OSError):
            pass

        # Chờ có timeout cho SIGKILL
        kill_deadline = time.monotonic() + 1.0
        while time.monotonic() < kill_deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.05)

        return proc.poll() is not None

    def _build_layer_cmd(self, launch_key: str) -> str:
        """Sinh chuỗi lệnh khởi chạy cho tầng tiến trình."""
        prefix = self._get_ws_prefix()
        sim_time_str = "true" if (hasattr(self, "is_sim_time") and self.is_sim_time()) else "false"
        if launch_key == "layer1":
            hb_port, lidar_port, detect_method = self.get_detected_ports()
            self.get_logger().info(
                f"[LAYER 1] Auto-detected serial ports ({detect_method}): Hoverboard={hb_port}, LiDAR={lidar_port}"
            )
            return (
                prefix
                + f"ros2 launch mo_hinh real_odom.launch.py odom_source:=hoverboard use_sim_time:={sim_time_str} "
                + f"hoverboard_port:={shlex.quote(hb_port)} esp_port:={shlex.quote(hb_port)} lidar_port:={shlex.quote(lidar_port)}"
            )
        elif launch_key == "layer2":
            backend = getattr(self, "_selected_backend", "amcl") or "amcl"
            cmd_parts = [
                prefix + "ros2 launch mo_hinh real_localization_nav2.launch.py",
                f"localization_mode:={shlex.quote(backend)}",
                f"use_sim_time:={sim_time_str}",
                "use_rviz:=false",
            ]
            map_yaml = getattr(self, "_selected_map_yaml", None)
            if map_yaml and os.path.exists(map_yaml):
                graph_path = os.path.splitext(map_yaml)[0]
                cmd_parts.append(f"map_yaml:={shlex.quote(map_yaml)}")
                cmd_parts.append(f"map_graph:={shlex.quote(graph_path)}")
            return " ".join(cmd_parts)
        elif launch_key == "layer3":
            return prefix + f"ros2 launch mo_hinh real_slam.launch.py use_sim_time:={sim_time_str} use_rviz:=false"
        elif launch_key == "layer4":
            return prefix + f"ros2 launch robot_bringup ai_perception.launch.py use_sim_time:={sim_time_str}"
        elif launch_key == "layer5":
            return prefix + "ros2 launch robot_bringup backend.launch.py"
        return ""

    def swap_serial_ports(self) -> tuple[str, str, str]:
        """Hoán đổi vị trí giữa cổng Hoverboard/ESP32 và cổng LiDAR."""
        self._force_swap_ports = not self._force_swap_ports
        from robot_ui.kiosk.serial_resolver import detect_serial_ports

        esp, lidar, src = detect_serial_ports(force_swap=self._force_swap_ports)
        self._cached_serial_ports = (esp, lidar, src)
        self.get_logger().info(f"[SERIAL] Đảo cổng thành công: ESP={esp}, LiDAR={lidar} (nguồn: {src})")
        self._record_layer_log("layer1", f"[SERIAL] Đã đảo cổng cấu hình: ESP={esp}, LiDAR={lidar} (nguồn: {src})")
        return esp, lidar, src

    def get_detected_ports(self) -> tuple[str, str, str]:
        """Trả về tuple (esp_port, lidar_port, detect_src) từ cache hoặc quét nhanh."""
        if getattr(self, "_cached_serial_ports", None) is not None:
            return self._cached_serial_ports
        from robot_ui.kiosk.serial_resolver import detect_serial_ports

        self._cached_serial_ports = detect_serial_ports(force_swap=self._force_swap_ports)
        return self._cached_serial_ports

    def rescan_serial_ports_async(self, callback: Any = None) -> None:
        """Quét và làm mới thông tin cổng Serial trên luồng nền (non-blocking)."""
        def _scan_worker():
            from robot_ui.kiosk.serial_resolver import detect_serial_ports
            ports = detect_serial_ports(force_swap=self._force_swap_ports)
            self._cached_serial_ports = ports
            self.get_logger().info(f"[SERIAL] Đã quét cổng nền xong ({ports[2]}): ESP={ports[0]}, LiDAR={ports[1]}")
            if callback:
                try:
                    callback(*ports)
                except Exception:
                    pass
        threading.Thread(target=_scan_worker, daemon=True).start()

    def start_layer(self, layer_key: str) -> None:
        """Khởi động tầng tiến trình tương ứng bảo đảm thread-safe và nguyên tử."""
        canonical = self._canonical_layer_key(layer_key)
        if not (canonical.startswith("layer") and canonical[5:].isdigit()):
            canonical = self.LAYER_ALIAS_MAP.get(layer_key, "layer1")
        alt_key = self.LAYER_ALIAS_MAP.get(canonical, "")

        with self._lifecycle_lock:
            curr_state = self._layer_states.get(canonical, LayerState.STOPPED)

            if curr_state == LayerState.RUNNING:
                self.get_logger().warn(
                    f"[LAYER START] Tầng {layer_key} ({canonical}) đang RUNNING, từ chối khởi động trùng."
                )
                return

            if curr_state == LayerState.STARTING:
                self.get_logger().warn(
                    f"[LAYER START] Tầng {layer_key} ({canonical}) đang trong quá trình STARTING, bỏ qua."
                )
                return

            if curr_state == LayerState.STOPPING:
                self.get_logger().warn(
                    f"[LAYER START] Tầng {layer_key} ({canonical}) đang trong trạng thái STOPPING. "
                    "Không thể khởi động lại cho đến khi tiến trình dừng hoàn toàn!"
                )
                return

            if curr_state == LayerState.ERROR:
                self.get_logger().error(
                    f"[LAYER START] Tầng {layer_key} ({canonical}) đang ở trạng thái ERROR (tiến trình cũ chưa dừng sạch). "
                    "Vui lòng bấm STOP hoặc Emergency Cleanup trước khi khởi động lại!"
                )
                return

            existing_proc = (
                self._layer_procs.get(canonical)
                or (self._layer_procs.get(alt_key) if alt_key else None)
            )
            if existing_proc and existing_proc.poll() is None:
                self.get_logger().warn(
                    f"[LAYER START] Tầng {layer_key} ({canonical}) đang có tiến trình PID {existing_proc.pid} còn chạy! "
                    "Từ chối khởi động trùng."
                )
                return

            # Gán token định danh duy nhất cho launch generation này
            launch_token = str(time.time_ns())
            self._active_launch_tokens[canonical] = launch_token
            if alt_key:
                self._active_launch_tokens[alt_key] = launch_token
            self._cancel_starting_layers.discard(canonical)

            # Chuyển trạng thái sang STARTING
            self._set_layer_state(canonical, LayerState.STARTING)

            starting_dict = {canonical: LayerState.STARTING.value, layer_key: LayerState.STARTING.value}
            if alt_key:
                starting_dict[alt_key] = LayerState.STARTING.value

        # Phát signal trạng thái STARTING ngoài lock
        self.signals.layer_status_changed.emit(starting_dict)
        self.get_logger().info(f"[LAYER START] Khởi động tầng: {layer_key} (canonical={canonical}, token={launch_token})")

        cmd = self._build_layer_cmd(canonical)
        if not cmd:
            with self._lifecycle_lock:
                self._set_layer_state(canonical, LayerState.ERROR)
                self._active_launch_tokens.pop(canonical, None)
            self.signals.layer_status_changed.emit({canonical: LayerState.ERROR.value})
            return

        threading.Thread(
            target=self._launch_worker,
            args=(canonical, layer_key, alt_key, cmd, launch_token),
            daemon=True,
        ).start()

    def _launch_worker(self, canonical: str, orig_key: str, alt_key: str, cmd: str, token: str) -> None:
        """Worker thread khởi tạo tiến trình và bảo vệ tính nguyên tử của token & process registration."""
        # Bước 1: Kiểm tra trước Popen dưới lock
        with self._lifecycle_lock:
            if (
                self._active_launch_tokens.get(canonical) != token
                or self._layer_states.get(canonical) != LayerState.STARTING
                or canonical in self._cancel_starting_layers
            ):
                self.get_logger().info(f"[LAYER LAUNCH] Hủy khởi động {canonical} trước khi Popen (token mismatch hoặc cờ hủy).")
                if self._layer_states.get(canonical) in (LayerState.STARTING, LayerState.STOPPING):
                    self._set_layer_state(canonical, LayerState.STOPPED)
                    self._active_launch_tokens.pop(canonical, None)
                    self._cancel_starting_layers.discard(canonical)
                    emit_stopped = True
                else:
                    emit_stopped = False
            else:
                emit_stopped = False

        if emit_stopped:
            self.signals.layer_status_changed.emit({canonical: LayerState.STOPPED.value})
            return

        # Bước 2: Thực hiện Popen ngoài lock
        proc = None
        spawn_exc = None
        log_path = f"/tmp/ros_layer_{canonical}.log"
        self._record_layer_log(canonical, f"--- BẮT ĐẦU KHỞI ĐỘNG [{canonical.upper()}] ---")
        self._record_layer_log(canonical, f"Lệnh thực thi: {cmd}")

        clean_env = self._get_clean_env() if hasattr(self, "_get_clean_env") else os.environ.copy()
        clean_env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
        try:
            proc = subprocess.Popen(
                ["bash", "-c", cmd],
                env=clean_env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            spawn_exc = exc

        # Bước 3: Kiểm tra token & đăng ký tiến trình NGUYÊN TỬ DƯỚI LOCK (P0-1)
        abort_proc = False
        emit_dict = None
        with self._lifecycle_lock:
            if spawn_exc is not None:
                self.get_logger().error(f"[LAYER LAUNCH] Lỗi khi tạo tiến trình {canonical}: {spawn_exc}")
                self._set_layer_state(canonical, LayerState.ERROR)
                self._active_launch_tokens.pop(canonical, None)
                emit_dict = {canonical: LayerState.ERROR.value}
            elif (
                self._active_launch_tokens.get(canonical) != token
                or self._layer_states.get(canonical) != LayerState.STARTING
                or canonical in self._cancel_starting_layers
            ):
                self.get_logger().info(f"[LAYER LAUNCH] Token bị thu hồi hoặc layer đã STOPPING trong lúc Popen {canonical}.")
                abort_proc = True
                self._set_layer_state(canonical, LayerState.STOPPING)
                emit_dict = {canonical: LayerState.STOPPING.value}
            else:
                self._layer_procs[canonical] = proc
                if orig_key:
                    self._layer_procs[orig_key] = proc
                if alt_key:
                    self._layer_procs[alt_key] = proc
                if canonical == "layer3":
                    self._slam_proc = proc
                elif canonical == "layer2":
                    self._localization_proc = proc

                self._set_layer_state(canonical, LayerState.RUNNING)
                emit_dict = {canonical: LayerState.RUNNING.value, orig_key: LayerState.RUNNING.value}
                if alt_key:
                    emit_dict[alt_key] = LayerState.RUNNING.value

        if emit_dict:
            self.signals.layer_status_changed.emit(emit_dict)
            if canonical == "layer3" and not abort_proc and spawn_exc is None:
                self.signals.slam_status_changed.emit(True, "SLAM đang chạy")

        if spawn_exc is not None:
            self._record_layer_log(canonical, f"[LỖI KHỞI ĐỘNG]: {spawn_exc}")
            return

        # Bước 4: Xử lý hủy tiến trình nếu token bị thu hồi (P0-2)
        if abort_proc and proc is not None:
            self.get_logger().info(f"[LAYER LAUNCH] Đang dừng tiến trình {canonical} (PID: {proc.pid}) do yêu cầu hủy...")
            is_dead = self._terminate_process_group(proc, timeout=1.5)
            with self._lifecycle_lock:
                if self._active_launch_tokens.get(canonical) in (token, None):
                    if is_dead:
                        self._set_layer_state(canonical, LayerState.STOPPED)
                        self._active_launch_tokens.pop(canonical, None)
                        self._cancel_starting_layers.discard(canonical)
                        post_emit = {canonical: LayerState.STOPPED.value}
                        self.get_logger().info(f"[LAYER LAUNCH] Tiến trình hủy {canonical} đã dừng hoàn toàn -> STOPPED.")
                    else:
                        self.get_logger().error(f"[LAYER LAUNCH] Không thể dừng tiến trình {canonical} PID {proc.pid} -> ERROR.")
                        self._set_layer_state(canonical, LayerState.ERROR)
                        self._layer_procs[canonical] = proc
                        post_emit = {canonical: LayerState.ERROR.value}
                else:
                    post_emit = None

            if post_emit:
                self.signals.layer_status_changed.emit(post_emit)
            return

        # Bước 5: Đọc log stream và phát hiện tiến trình kết thúc
        def _read_layer_stream(p=proc, key=canonical, path=log_path, tok=token):
            last_flush = time.monotonic()
            try:
                with open(path, "a", encoding="utf-8") as f_out:
                    if p.stdout:
                        for line in iter(p.stdout.readline, ""):
                            if not line:
                                break
                            clean_line = line.rstrip()
                            f_out.write(clean_line + "\n")
                            now = time.monotonic()
                            if now - last_flush > 1.0:
                                f_out.flush()
                                last_flush = now
                            self._record_layer_log(key, clean_line)
                    f_out.flush()
            except Exception:
                pass
            finally:
                try:
                    if p.stdout:
                        p.stdout.close()
                    p.wait(timeout=2.0)
                except Exception:
                    pass
                exit_code = p.poll()
                if exit_code is None:
                    return
                self._record_layer_log(
                    key, f"--- TIẾN TRÌNH [{key.upper()}] ĐÃ DỪNG (Mã thoát: {exit_code}) ---"
                )

                with self._lifecycle_lock:
                    if self._layer_procs.get(key) is p and self._layer_states.get(key) == LayerState.RUNNING:
                        self._layer_procs.pop(key, None)
                        a_k = self.LAYER_ALIAS_MAP.get(key, "")
                        if a_k:
                            self._layer_procs.pop(a_k, None)
                        if key == "layer3":
                            self._slam_proc = None
                        elif key == "layer2":
                            self._localization_proc = None

                        new_st = LayerState.STOPPED if exit_code == 0 else LayerState.ERROR
                        self._set_layer_state(key, new_st)
                        self._active_launch_tokens.pop(key, None)
                        stream_emit = {key: new_st.value}
                        if a_k:
                            stream_emit[a_k] = new_st.value
                    else:
                        stream_emit = None

                if stream_emit:
                    self.signals.layer_status_changed.emit(stream_emit)
                    if key == "layer3":
                        self.signals.slam_status_changed.emit(False, "SLAM đã dừng")

        threading.Thread(target=_read_layer_stream, daemon=True).start()

    def stop_layer(self, layer_key: str) -> None:
        """Dừng tầng tiến trình tương ứng bảo đảm thread-safe và xác nhận dừng trước khi chuyển STOPPED."""
        canonical = self._canonical_layer_key(layer_key)
        if not (canonical.startswith("layer") and canonical[5:].isdigit()):
            canonical = self.LAYER_ALIAS_MAP.get(layer_key, "layer1")
        alt_key = self.LAYER_ALIAS_MAP.get(canonical, "")

        proc_to_kill = None
        target_token = None

        with self._lifecycle_lock:
            curr_state = self._layer_states.get(canonical, LayerState.STOPPED)

            if curr_state == LayerState.STOPPING:
                self.get_logger().warn(
                    f"[LAYER STOP] Tầng {layer_key} ({canonical}) đang ở trạng thái STOPPING, bỏ qua yêu cầu trùng lặp."
                )
                return

            if curr_state == LayerState.STOPPED:
                proc = self._layer_procs.get(canonical)
                if not proc or proc.poll() is not None:
                    self.get_logger().info(f"[LAYER STOP] Tầng {layer_key} ({canonical}) đã STOPPED.")
                    self.signals.layer_status_changed.emit({canonical: LayerState.STOPPED.value})
                    return

            # Thu hồi launch token của generation hiện tại ngay lập tức
            target_token = self._active_launch_tokens.pop(canonical, None)
            if alt_key:
                self._active_launch_tokens.pop(alt_key, None)
            self._cancel_starting_layers.add(canonical)

            # Chuyển trạng thái sang STOPPING
            self._set_layer_state(canonical, LayerState.STOPPING)

            # Lấy handle tiến trình nếu có
            proc_to_kill = self._layer_procs.get(canonical) or (self._layer_procs.get(alt_key) if alt_key else None)

            stopping_dict = {canonical: LayerState.STOPPING.value, layer_key: LayerState.STOPPING.value}
            if alt_key:
                stopping_dict[alt_key] = LayerState.STOPPING.value

        # Báo ngay trạng thái STOPPING lên UI ngoài lock để khóa cả hai nút Start/Stop
        self.signals.layer_status_changed.emit(stopping_dict)
        self.get_logger().info(f"[LAYER STOP] Bắt đầu dừng tầng: {layer_key} (canonical={canonical}, STOPPING)")
        self._record_layer_log(canonical, f"Đang gửi tín hiệu dừng tới {layer_key} (STOPPING)...")

        # Nếu đang STARTING nhưng chưa có process handle trong _layer_procs:
        # Launch worker sẽ phát hiện token bị thu hồi và tự hủy an toàn ở Bước 4 của nó.
        if proc_to_kill is None:
            self.get_logger().info(f"[LAYER STOP] Tầng {layer_key} chưa có process handle, đã thu hồi token và chờ launch worker xử lý.")
            return

        threading.Thread(
            target=self._stop_worker,
            args=(canonical, layer_key, alt_key, proc_to_kill, target_token),
            daemon=True,
        ).start()

    def _stop_worker(self, canonical: str, orig_key: str, alt_key: str, proc: subprocess.Popen, token: str | None) -> None:
        """Worker thread thực thi terminate process group và xác nhận kết thúc trước khi cập nhật state."""
        is_dead = self._terminate_process_group(proc, timeout=1.5)

        with self._lifecycle_lock:
            # Kiểm tra xem có launch generation mới nào đã thay thế chưa (T14)
            current_token = self._active_launch_tokens.get(canonical)
            if token is not None and current_token is not None and current_token != token:
                self.get_logger().warn(f"[LAYER STOP] Generation cũ ({token}) kết thúc sau khi generation mới ({current_token}) đã bắt đầu. Bỏ qua cập nhật.")
                return

            if is_dead:
                # ĐÃ DỪNG HOÀN TOÀN: Xóa process handle và chuyển sang STOPPED
                self._layer_procs.pop(canonical, None)
                if orig_key:
                    self._layer_procs.pop(orig_key, None)
                if alt_key:
                    self._layer_procs.pop(alt_key, None)
                if canonical == "layer3":
                    self._slam_proc = None
                elif canonical == "layer2":
                    self._localization_proc = None
                    if hasattr(self, "cancel_pending_initial_pose"):
                        self.cancel_pending_initial_pose()
                    if hasattr(self, "_set_localization_state"):
                        self._set_localization_state(
                            LocalizationState.NOT_INITIALIZED,
                            "Layer 2 (Nav2 & AMCL) đã dừng"
                        )

                self._set_layer_state(canonical, LayerState.STOPPED)
                self._cancel_starting_layers.discard(canonical)

                emit_payload = {canonical: LayerState.STOPPED.value, orig_key: LayerState.STOPPED.value}
                if alt_key:
                    emit_payload[alt_key] = LayerState.STOPPED.value
                self._record_layer_log(canonical, f"--- TIẾN TRÌNH [{canonical.upper()}] ĐÃ DỪNG HOÀN TOÀN ---")
                self.get_logger().info(f"[LAYER STOP] Đã dừng hoàn toàn: {canonical}")
            else:
                # KHÔNG THỂ DỪNG: Giữ process handle và chuyển sang ERROR
                self.get_logger().error(f"[LAYER STOP] Tiến trình {canonical} (PID: {proc.pid}) không phản hồi! Giữ trạng thái ERROR.")
                self._set_layer_state(canonical, LayerState.ERROR)
                self._layer_procs[canonical] = proc
                if alt_key:
                    self._layer_procs[alt_key] = proc

                emit_payload = {canonical: LayerState.ERROR.value, orig_key: LayerState.ERROR.value}
                if alt_key:
                    emit_payload[alt_key] = LayerState.ERROR.value
                self._record_layer_log(canonical, f"[LỖI DỪNG]: Tiến trình PID {proc.pid} không phản hồi tín hiệu dừng.")

        self.signals.layer_status_changed.emit(emit_payload)
        if canonical == "layer3" and is_dead:
            self.signals.slam_status_changed.emit(False, "SLAM đã dừng")

    def cleanup_ros_processes(self) -> None:
        """Dọn dẹp triệt để tất cả các tiến trình ROS 2 nền và giải phóng tài nguyên cổng/bộ nhớ."""
        self.get_logger().info("[EMERGENCY CLEANUP] Quét và đóng sạch toàn bộ tiến trình ROS 2...")
        self._record_layer_log("all", "[EMERGENCY CLEANUP] Quét và đóng sạch toàn bộ tiến trình ROS 2...")
        patterns = [
            "robot_state_publisher",
            "scan_to_scan_filter_chain",
            "real_odom",
            "real_slam",
            "real_localization_nav2",
            "slam_toolbox",
            "async_slam_toolbox",
            "nav2_bringup",
            "component_container",
            "nav2_container",
            "bt_navigator",
            "controller_server",
            "planner_server",
            "behavior_server",
            "lifecycle_manager",
            "amcl",
            "map_server",
            "hoverboard_driver",
            "ros2_control_node",
            "controller_manager",
            "spawner",
            "sllidar_node",
            "esp_bridge_node",
            "ai_perception",
            "three_astra",
            "dual_camera_obstacles",
            "obstacle_detector",
            "debugger_gui",
            "astra_camera_node",
            "usb_cam_node_exe",
            "yolo_node",
            "tracking_node",
            "hand_wave_detector",
            "rough_localizer",
            "backend.launch.py",
            "config_manager_node",
            "system_monitor_node",
            "navigation_manager_node",
            "mission_controller_node",
            "audio_player_node",
            "smoother_server",
            "waypoint_follower",
            "velocity_smoother",
            "front_astra_optical_alias",
            "rqt_image_view",
        ]
        # 1. Cập nhật state machine ngay lập tức dưới lock (non-blocking)
        with self._lifecycle_lock:
            self._active_launch_tokens.clear()
            self._cancel_starting_layers.clear()
            for c_k in ("layer1", "layer2", "layer3", "layer4", "layer5"):
                self._set_layer_state(c_k, LayerState.STOPPED)
            self._layer_procs.clear()
            self._slam_proc = None
            self._localization_proc = None
            if hasattr(self, "cancel_pending_initial_pose"):
                self.cancel_pending_initial_pose()
            if hasattr(self, "_set_localization_state"):
                self._set_localization_state(
                    LocalizationState.NOT_INITIALIZED,
                    "Đã dừng toàn bộ tiến trình"
                )

            stopped_states = {}
            for k in self.LAYER_ALIAS_MAP:
                stopped_states[k] = LayerState.STOPPED.value

        self.signals.layer_status_changed.emit(stopped_states)
        self.signals.slam_status_changed.emit(False, "SLAM đã dừng")

        # 2. Thực thi pkill trên background thread để không bao giờ block Qt UI main loop
        def _cleanup_worker():
            regex = "|".join(patterns)
            try:
                subprocess.run(["pkill", "-9", "-f", regex], capture_output=True, timeout=2.0)
            except Exception:
                for p in patterns:
                    try:
                        subprocess.run(["pkill", "-9", "-f", p], capture_output=True, timeout=0.2)
                    except Exception:
                        pass
            self.get_logger().info("[EMERGENCY CLEANUP] Hoàn tất quét và đóng tiến trình ROS 2.")
            self._record_layer_log("all", "[EMERGENCY CLEANUP] Hoàn tất quét và đóng tiến trình ROS 2.")

        threading.Thread(target=_cleanup_worker, daemon=True).start()

