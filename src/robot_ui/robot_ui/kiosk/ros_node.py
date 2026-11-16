from __future__ import annotations

import glob
import math
import os
import shlex
import signal
import subprocess
import threading
import time
from functools import partial
from pathlib import Path
from typing import Any

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from collections import deque
import tf2_ros
import rclpy
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from robot_interfaces.action import DirectGo
from robot_interfaces.msg import NavigationStatus, RobotStatus
from robot_interfaces.srv import GetLocation, GetLocationList

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


class KioskRosNode(Node):
    """Small ROS adapter dedicated to the native customer UI."""

    MAP_PREVIEW_MAX_DIMENSION = 280
    PATH_PREVIEW_MAX_POINTS = 400

    def __init__(self, signals: KioskSignals) -> None:
        super().__init__("robot_ui_kiosk")
        self.signals = signals
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
        self._pose: dict[str, Any] = {}
        self._last_robot_status_at: float | None = None
        self._connected = False
        self._slam_proc: subprocess.Popen | None = None
        self._localization_proc: subprocess.Popen | None = None
        self._selected_map_yaml: str = ""
        self._selected_backend: str = "amcl"

        # System telemetry & Configuration tracking
        self._dbg_lock = threading.Lock()
        self._topic_stats: dict[str, dict[str, Any]] = {}
        self._battery_voltages: deque[float] = deque(maxlen=25)
        self._battery_pct = 0.0
        self._battery_smooth_v = 36.0
        self._battery_raw_v = 36.0
        self._battery_source = "robot_status"
        self._water_liter = 0.0
        self._kettle_present = True
        self._is_waving = False
        self._yolo_active = False
        self._people_count = 0
        self._closest_person_dist: float | None = None
        self._layer_procs: dict[str, subprocess.Popen] = {}
        self._force_swap_ports: bool = False
        self._selected_operation_mode: str = "slam_mapping"
        self._selected_odom_source: str = "hoverboard"

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
        self.create_subscription(NavPath, "/plan", self._on_path, 10)
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
                qos_profile_sensor_data,
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
        self.signals.robot_status_changed.emit(
            {
                "battery_percent": float(msg.battery_percent),
                "battery_low": bool(msg.battery_low),
                "lidar_ok": bool(msg.lidar_ok),
                "localization_ok": bool(msg.localization_ok),
                "odom_ok": bool(msg.odom_ok),
                "water_low": bool(msg.water_low),
                "water_liter": float(msg.water_liter),
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
        p_data = {
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
        self._pose = p_data
        self.signals.pose_changed.emit(p_data)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._have_localized_pose = True
        self._emit_pose(msg.pose.pose, str(msg.header.frame_id), "amcl")

    def _on_odom(self, msg: Odometry) -> None:
        vx = float(msg.twist.twist.linear.x)
        wz = float(msg.twist.twist.angular.z)
        self._mark_topic_rx("odom", {"vx": vx, "wz": wz})
        if not self._have_localized_pose:
            self._emit_pose(msg.pose.pose, str(msg.header.frame_id), "odom")

    def _on_path(self, msg: NavPath) -> None:
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
        if not self._direct_go_client.server_is_ready():
            self.get_logger().warn(f"[DIRECT_GO] Action server /navigation/direct_go chưa sẵn sàng (target='{target}')!")
            self.signals.command_status_changed.emit(
                "warning",
                "Navigation Manager chưa sẵn sàng",
            )
            return

        self.get_logger().info(f"[DIRECT_GO] Gửi mục tiêu địa điểm: '{target}'")
        self.signals.navigation_active_changed.emit(True)
        goal = DirectGo.Goal()
        goal.target_location = target
        self.signals.command_status_changed.emit("progress", f"Đang gửi điểm đến {target}")
        future = self._direct_go_client.send_goal_async(
            goal,
            feedback_callback=self._on_navigation_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[DIRECT_GO] Lỗi khi gửi mục tiêu: {error}")
            self.signals.command_status_changed.emit("error", "Không gửi được điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.get_logger().error("[DIRECT_GO] Mục tiêu bị Action server TỪ CHỐI!")
            self.signals.command_status_changed.emit("error", "Điểm đến bị từ chối")
            self.signals.navigation_active_changed.emit(False)
            return
        self.get_logger().info("[DIRECT_GO] Mục tiêu đã được CHẤP THUẬN (ACCEPTED), bắt đầu di chuyển.")
        self._goal_handle = goal_handle
        self.signals.command_status_changed.emit("progress", "Robot bắt đầu di chuyển")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        distance = getattr(feedback, "distance_remaining", None)
        if distance is not None:
            self.get_logger().info(f"[DIRECT_GO Feedback] Khoảng cách còn lại: {float(distance):.2f} m")
            self.signals.command_status_changed.emit(
                "progress",
                f"Còn {float(distance):.1f} m",
            )

    def _on_navigation_result(self, future: Any) -> None:
        self._goal_handle = None
        self.signals.navigation_active_changed.emit(False)
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"[DIRECT_GO] Lỗi khi nhận kết quả hành trình: {error}")
            self.signals.command_status_changed.emit("error", "Mất kết quả dẫn đường")
            return
        tone = "success" if result.success else "error"
        self.get_logger().info(f"[DIRECT_GO Result] Kết quả: success={result.success}, msg='{result.finish_message}'")
        self.signals.command_status_changed.emit(tone, str(result.finish_message))

    def navigate_to_pose(self, x: float, y: float, yaw: float) -> None:
        if not self._nav2_client.server_is_ready():
            self.get_logger().warn(f"[NAV2] Action server /navigate_to_pose chưa sẵn sàng (x={x:.2f}, y={y:.2f})!")
            self.signals.command_status_changed.emit(
                "warning",
                "Nav2 chưa sẵn sàng nhận điểm đến",
            )
            self.signals.navigation_active_changed.emit(False)
            return

        self.get_logger().info(f"[NAV2] Gửi mục tiêu tọa độ: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f} rad")
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
            f"Đang đi tới X {x:.2f}, Y {y:.2f}",
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
            self.get_logger().error(f"[NAV2] Lỗi khi gửi mục tiêu Nav2: {error}")
            self.signals.command_status_changed.emit("error", "Không gửi được điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.get_logger().error("[NAV2] Mục tiêu bị Nav2 server TỪ CHỐI!")
            self.signals.command_status_changed.emit("error", "Nav2 từ chối điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        self.get_logger().info("[NAV2] Mục tiêu đã được Nav2 CHẤP THUẬN (ACCEPTED), bắt đầu di chuyển.")
        self._goal_handle = goal_handle
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

        self.get_logger().debug(f"[NAV2 Feedback] Khoảng cách còn lại: {distance:.2f} m | Tự cứu: {recov}")
        self.signals.command_status_changed.emit("progress", f"Còn {distance:.1f} m")
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
            self.get_logger().error(f"[NAV2] Lỗi nhận kết quả từ Nav2: {error}")
            self.signals.command_status_changed.emit("error", "Mất kết quả dẫn đường")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[NAV2 Result] Đã tới điểm đến thành công (STATUS_SUCCEEDED).")
            self.signals.command_status_changed.emit("success", "Đã tới điểm đến")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("[NAV2 Result] Đã hủy hành trình (STATUS_CANCELED).")
            self.signals.command_status_changed.emit("warning", "Đã dừng hành trình")
        else:
            self.get_logger().error(f"[NAV2 Result] Dẫn đường kết thúc với mã trạng thái: {status}")
            self.signals.command_status_changed.emit("error", "Không thể tới điểm đến")

    def cancel_navigation(self) -> None:
        """Hủy hành trình điều hướng hiện tại."""
        self.get_logger().info("[NAV] Yêu cầu hủy hành trình điều hướng từ UI.")
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self.trigger_emergency_stop()

    def trigger_emergency_stop(self) -> None:
        """Dừng khẩn cấp toàn diện: phanh tức thì, hủy mọi Goal Nav2/DirectGo và kích hoạt P0 Emergency trong BT."""
        self.get_logger().warn("🚨 EMERGENCY STOP TRIGGERED FROM UI!")

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

        self.signals.command_status_changed.emit("error", "🚨 ĐÃ KÍCH HOẠT DỪNG KHẨN CẤP!")
        self.signals.navigation_active_changed.emit(False)

    def reset_emergency_stop(self) -> None:
        """Mở khóa dừng khẩn cấp trên Mission Controller."""
        if self.reset_emergency_client.service_is_ready():
            self.reset_emergency_client.call_async(Trigger.Request())
        elif self.reset_emergency_client.wait_for_service(timeout_sec=0.2):
            self.reset_emergency_client.call_async(Trigger.Request())
        self.signals.command_status_changed.emit("success", "Đã mở khóa dừng khẩn cấp")

    def _on_scan(self, msg: LaserScan) -> None:
        valid_ranges = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        min_lidar = min(valid_ranges) if valid_ranges else None
        self._mark_topic_rx("lidar", {"min_dist": min_lidar, "count": len(valid_ranges)})

        now = time.monotonic()
        if now - self._last_scan_ui_pub < 0.1:
            return
        self._last_scan_ui_pub = now
        if not getattr(self, "_pose", None):
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
        if not getattr(self, "_pose", None):
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
        self.get_logger().info("[MISSION] Kích hoạt bắt đầu tuần tra (/mission/start_patrol)")
        if not self.start_patrol_client.service_is_ready():
            if not self.start_patrol_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn("[MISSION] Service /mission/start_patrol chưa sẵn sàng!")
                self.signals.command_status_changed.emit("warning", "Chưa kết nối Mission Controller")
                return
        req = Trigger.Request()
        future = self.start_patrol_client.call_async(req)
        future.add_done_callback(self._on_start_patrol_response)

    def _on_start_patrol_response(self, future: Any) -> None:
        try:
            res = future.result()
            tone = "success" if res.success else "warning"
            self.signals.command_status_changed.emit(tone, str(res.message))
        except Exception as e:
            self.get_logger().error(f"[MISSION] Lỗi gọi start_patrol: {e}")

    def pause_patrol(self) -> None:
        """Tạm dừng hành trình tuần tra."""
        self.get_logger().info("[MISSION] Kích hoạt tạm dừng tuần tra (/mission/pause_patrol)")
        if not self.pause_patrol_client.service_is_ready():
            if not self.pause_patrol_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn("[MISSION] Service /mission/pause_patrol chưa sẵn sàng!")
                return
        req = Trigger.Request()
        self.pause_patrol_client.call_async(req)
        self.signals.command_status_changed.emit("warning", "Đã tạm dừng tuần tra")

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

    def _get_ws_root(self) -> Path:
        env_ws = os.environ.get("ROS2_WS")
        if env_ws and Path(env_ws).is_dir():
            return Path(env_ws)
        colcon_prefix = os.environ.get("COLCON_PREFIX_PATH")
        if colcon_prefix:
            for p in colcon_prefix.split(os.pathsep):
                cand = Path(p).parent
                if (cand / "install" / "setup.bash").is_file():
                    return cand
        cur = Path(__file__).resolve()
        for parent in cur.parents:
            if (parent / "install" / "setup.bash").is_file():
                return parent
            if (parent / "src").is_dir() and (parent / "build").is_dir():
                return parent
        home_ws = Path.home() / "ros2_ws"
        if home_ws.is_dir():
            return home_ws
        return cur.parents[4]

    def _get_ws_prefix(self) -> str:
        ws_root = self._get_ws_root()
        ros_setup = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
        ws_setup = ws_root / "install" / "setup.bash"
        cmd = f"source '{ros_setup}'"
        if ws_setup.is_file():
            cmd += f" && source '{ws_setup}'"
        return cmd + " && "

    @staticmethod
    def _list_serial_ports() -> list[str]:
        raw_ports = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
        dedup: list[str] = []
        seen_real: set[str] = set()
        for p in raw_ports:
            try:
                real = os.path.realpath(p)
            except Exception:
                real = p
            if real not in seen_real:
                seen_real.add(real)
                dedup.append(p)
        return dedup

    @staticmethod
    def _is_esp32_port(port: str, baudrate: int = 115200, timeout: float = 0.1) -> bool:
        """Thăm dò dữ liệu serial thực tế giống hệt gui_v3 để nhận diện ESP32/Hoverboard."""
        try:
            import serial
        except ImportError:
            return False
        ser = None
        try:
            ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            ser.reset_input_buffer()
            start_time = time.time()
            while time.time() - start_time < 0.6:
                raw = ser.read(64)
                if not raw:
                    continue
                if b"\xcd\xab" in raw or b"\xab\xcd" in raw:
                    return True
                if b"ets Jun" in raw or b"rst:0x" in raw or b"boot:" in raw:
                    return True
                if b"V:" in raw or b"E," in raw:
                    return True
        except Exception:
            return False
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
        return False

    def _detect_esp_lidar_ports(self) -> tuple[str, str, str]:
        """Tự động phát hiện cổng ESP32 và LiDAR chuẩn xác như gui_v3."""
        esp_alias, lidar_alias = "/dev/esp32", "/dev/lidar"
        if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
            esp_p, lidar_p = esp_alias, lidar_alias
            if self._force_swap_ports:
                return lidar_p, esp_p, "swap_override"
            return esp_p, lidar_p, "udev_alias"

        ports = self._list_serial_ports()
        if not ports:
            esp_p, lidar_p = "/dev/ttyUSB0", "/dev/ttyUSB1"
            if self._force_swap_ports:
                return lidar_p, esp_p, "swap_override"
            return esp_p, lidar_p, "fallback_defaults"

        esp_port, lidar_port = None, None
        for port in ports:
            key = os.path.basename(port).lower()
            # SLAMTEC RPLIDAR trên Jetson dùng chip Silicon Labs CP2102
            if lidar_port is None and (
                "rplidar" in key or "slamtec" in key or "lidar" in key or "cp210" in key or "silicon_labs" in key
            ):
                lidar_port = port
            if esp_port is None and ("esp32" in key or "esp" in key):
                esp_port = port

        # Thăm dò byte thực tế nếu chưa phát hiện bằng tên udev (giống gui_v3)
        if esp_port is None:
            for port in ports:
                if self._is_esp32_port(port):
                    esp_port = port
                    break

        if esp_port is None:
            for port in ports:
                if port.endswith("ttyUSB0") or port.endswith("ttyACM0"):
                    esp_port = port
                    break
        if esp_port is None:
            esp_port = ports[0]

        # Đảm bảo lidar_port trỏ tới thiết bị vật lý KHÁC esp_port
        esp_real = os.path.realpath(esp_port)
        if lidar_port is None or os.path.realpath(lidar_port) == esp_real:
            lidar_port = None
            for port in ports:
                if os.path.realpath(port) != esp_real:
                    lidar_port = port
                    break

        # Fallback nếu chỉ có 1 cổng
        if lidar_port is None or os.path.realpath(lidar_port) == esp_real:
            lidar_port = "/dev/ttyUSB1" if esp_real != "/dev/ttyUSB1" else "/dev/ttyUSB0"

        if self._force_swap_ports:
            return lidar_port, esp_port, "swap_override"

        return esp_port, lidar_port, "scan"

    def swap_serial_ports(self) -> tuple[str, str, str]:
        """Hoán đổi vị trí giữa cổng Hoverboard/ESP32 và cổng LiDAR."""
        self._force_swap_ports = not self._force_swap_ports
        esp, lidar, src = self._detect_esp_lidar_ports()
        self.get_logger().info(f"[SERIAL] Đảo cổng thành công: ESP={esp}, LiDAR={lidar} (nguồn: {src})")
        self.signals.process_log_received.emit("SERIAL", f"[SERIAL] Đã đảo cổng cấu hình: ESP={esp}, LiDAR={lidar}")
        return esp, lidar, src

    def get_detected_ports(self) -> tuple[str, str, str]:
        """Trả về tuple (esp_port, lidar_port, detect_src)."""
        return self._detect_esp_lidar_ports()

    def start_slam(self) -> None:
        """Khởi chạy node SLAM mapping qua launch file real_slam.launch.py."""
        if self._slam_proc and self._slam_proc.poll() is None:
            self.get_logger().info("[KIOSK SLAM] Tiến trình SLAM đang chạy rồi.")
            self.signals.slam_status_changed.emit(True, "SLAM đang chạy")
            self.signals.process_log_received.emit("layer_slam", "[layer_slam] Tiến trình đang chạy rồi.")
            return

        cmd = self._get_ws_prefix() + "ros2 launch mo_hinh real_slam.launch.py use_sim_time:=false use_rviz:=false"
        self.get_logger().info(f"[KIOSK SLAM] START: {cmd}")
        self.signals.process_log_received.emit("layer_slam", f"[layer_slam] START: {cmd}")
        try:
            self._slam_proc = subprocess.Popen(
                ["bash", "-c", cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid,
            )
            self._layer_procs["layer_slam"] = self._slam_proc
            self.signals.slam_status_changed.emit(True, "Đã khởi động SLAM Mapping")

            def _stream_slam():
                if self._slam_proc and self._slam_proc.stdout:
                    for line in self._slam_proc.stdout:
                        cl = line.rstrip()
                        if cl:
                            self.signals.process_log_received.emit("layer_slam", cl)

            threading.Thread(target=_stream_slam, daemon=True).start()
        except Exception as exc:
            self.get_logger().error(f"[KIOSK SLAM] Lỗi khởi chạy SLAM: {exc}")
            self.signals.slam_status_changed.emit(False, f"Lỗi khởi động SLAM: {exc}")
            self.signals.process_log_received.emit("layer_slam", f"[layer_slam] LỖI: {exc}")

    def stop_slam(self) -> None:
        """Dừng tiến trình SLAM một cách an toàn."""
        if not self._slam_proc or self._slam_proc.poll() is not None:
            self.get_logger().info("[KIOSK SLAM] SLAM chưa chạy hoặc đã dừng.")
            self.signals.slam_status_changed.emit(False, "SLAM đã dừng")
            return

        self.get_logger().info("[KIOSK SLAM] STOP SLAM process group")
        self.signals.process_log_received.emit("layer_slam", "[layer_slam] Dừng tiến trình (SIGINT)...")
        try:
            pgid = os.getpgid(self._slam_proc.pid)
            os.killpg(pgid, signal.SIGINT)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if self._slam_proc.poll() is not None:
                    break
                time.sleep(0.05)
            if self._slam_proc.poll() is None:
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(0.5)
            if self._slam_proc.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except Exception as exc:
            self.get_logger().warning(f"[KIOSK SLAM] Dừng SLAM gặp lỗi: {exc}")
        finally:
            self._slam_proc = None
            self._layer_procs.pop("layer_slam", None)
            self.signals.slam_status_changed.emit(False, "Đã dừng SLAM Mapping")
            self.signals.process_log_received.emit("layer_slam", "[layer_slam] Đã dừng SLAM Mapping.")

    def save_map(self, map_stem: str) -> None:
        """Thực hiện lưu bản đồ kép (AMCL YAML + PoseGraph serialize) trên background thread."""
        def _save_worker():
            raw_name = map_stem.strip() or "new_map"
            ws_root = self._get_ws_root()
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
            self.signals.process_log_received.emit("layer_slam", f"[KIOSK MAP SAVE] Chạy lệnh: {cmd}")
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
                    self.signals.process_log_received.emit("layer_slam", f"[KIOSK MAP SAVE] {msg}")
                else:
                    err_msg = f"Lỗi khi lưu (mã lỗi {proc.returncode}): {proc.stderr[:100]}"
                    self.get_logger().error(f"[KIOSK MAP SAVE] {err_msg}")
                    self.signals.map_saved_result.emit(False, err_msg)
                    self.signals.process_log_received.emit("layer_slam", f"[KIOSK MAP SAVE] {err_msg}")
            except Exception as exc:
                self.get_logger().error(f"[KIOSK MAP SAVE] Ngoại lệ: {exc}")
                self.signals.map_saved_result.emit(False, f"Lỗi: {exc}")
                self.signals.process_log_received.emit("layer_slam", f"[KIOSK MAP SAVE] Lỗi: {exc}")

        threading.Thread(target=_save_worker, daemon=True).start()

    def apply_localization_map(self, map_yaml_path: str, backend: str = "amcl") -> None:
        """Lưu cấu hình và khởi chạy chế độ định vị với bản đồ được chọn."""
        self._selected_map_yaml = map_yaml_path
        self._selected_backend = backend
        self.get_logger().info(
            f"[KIOSK LOCALIZATION] Áp dụng bản đồ: {map_yaml_path} (Backend: {backend})"
        )
        self.signals.process_log_received.emit(
            "layer_navigation",
            f"[layer_navigation] Áp dụng bản đồ: {map_yaml_path} (Backend: {backend})",
        )
        self.stop_slam()

        def _loc_worker():
            if self._localization_proc and self._localization_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self._localization_proc.pid), signal.SIGINT)
                    time.sleep(1.0)
                except Exception:
                    pass

            cmd = (
                self._get_ws_prefix()
                + "ros2 launch mo_hinh real_localization_nav2.launch.py "
                + f"localization_mode:={shlex.quote(backend)} use_sim_time:=false "
                + f"map_yaml:={shlex.quote(map_yaml_path)} use_rviz:=false"
            )
            self.get_logger().info(f"[KIOSK LOCALIZATION] START: {cmd}")
            self.signals.process_log_received.emit("layer_navigation", f"[layer_navigation] START: {cmd}")
            try:
                self._localization_proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                self._layer_procs["layer_navigation"] = self._localization_proc

                def _stream_loc():
                    if self._localization_proc and self._localization_proc.stdout:
                        for line in self._localization_proc.stdout:
                            cl = line.rstrip()
                            if cl:
                                self.signals.process_log_received.emit("layer_navigation", cl)

                threading.Thread(target=_stream_loc, daemon=True).start()
            except Exception as exc:
                self.get_logger().error(f"[KIOSK LOCALIZATION] Lỗi: {exc}")
                self.signals.process_log_received.emit("layer_navigation", f"[layer_navigation] LỖI: {exc}")

        threading.Thread(target=_loc_worker, daemon=True).start()

    def stop_subprocesses(self) -> None:
        """Dọn dẹp các tiến trình nền khi đóng ứng dụng."""
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

        # Layer states
        def _is_running(k: str) -> bool:
            p = self._layer_procs.get(k)
            return bool(p and p.poll() is None)

        is_hw = _is_running("layer_hardware") or _is_running("layer1")
        is_nav = _is_running("layer_navigation") or _is_running("layer2") or bool(self._localization_proc and self._localization_proc.poll() is None)
        is_slam = _is_running("layer_slam") or _is_running("layer3") or bool(self._slam_proc and self._slam_proc.poll() is None)
        is_ai = _is_running("layer_perception") or _is_running("layer4")
        is_voice = _is_running("layer_voice_interaction") or _is_running("layer5")

        layer_states = {
            "layer_hardware": is_hw,
            "layer_navigation": is_nav,
            "layer_slam": is_slam,
            "layer_perception": is_ai,
            "layer_voice_interaction": is_voice,
            # Backward compatibility aliases
            "layer1": is_hw,
            "layer2": is_nav,
            "layer3": is_slam,
            "layer4": is_ai,
            "layer5": is_voice,
        }
        self.signals.layer_status_changed.emit(layer_states)

    def test_actuator(self, test_name: str) -> None:
        """Kích hoạt kiểm tra nhanh cơ cấu chấp hành."""
        self.get_logger().info(f"[TEST ACTUATOR] Kích hoạt: {test_name}")
        if test_name == "pump":
            msg = Bool()
            msg.data = True
            self.serve_trigger_pub.publish(msg)

            def _turn_off():
                time.sleep(1.5)
                m2 = Bool()
                m2.data = False
                self.serve_trigger_pub.publish(m2)
                self.signals.actuator_test_result.emit("pump", True, "Đã xả bơm 1.5 giây hoàn tất.")

            threading.Thread(target=_turn_off, daemon=True).start()
        elif test_name == "speaker":
            def _sound_worker():
                try:
                    subprocess.run(
                        ["paplay", "/usr/share/sounds/freedesktop/stereo/complete.oga"],
                        capture_output=True,
                        timeout=2.0,
                    )
                except Exception:
                    pass
                self.signals.actuator_test_result.emit("speaker", True, "Đã phát âm thanh kiểm tra loa ngoài.")

            threading.Thread(target=_sound_worker, daemon=True).start()
        elif test_name == "led_buzzer":
            self.signals.actuator_test_result.emit("led_buzzer", True, "Đã phát tín hiệu đèn LED / còi cảnh báo.")
        elif test_name == "jog_motor":
            def _jog_worker():
                tw = Twist()
                tw.linear.x = 0.05
                self.cmd_vel_pub.publish(tw)
                time.sleep(1.0)
                stop_tw = Twist()
                self.cmd_vel_pub.publish(stop_tw)
                self.signals.actuator_test_result.emit("jog_motor", True, "Đã nhích xe 5cm an toàn và phanh dừng.")

            threading.Thread(target=_jog_worker, daemon=True).start()

    def _terminate_proc_group(self, name: str, proc: subprocess.Popen) -> None:
        """Dừng tiến trình nhẹ nhàng bằng SIGINT trước (giống gui_v3) rồi SIGTERM / SIGKILL nếu cần."""
        if not proc or proc.poll() is not None:
            self.signals.process_log_received.emit(name, f"[{name}] Chưa chạy hoặc đã dừng.")
            return
        try:
            pgid = os.getpgid(proc.pid)
            self.signals.process_log_received.emit(name, f"[{name}] Dừng tiến trình (SIGINT)...")
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        except Exception as exc:
            self.get_logger().error(f"[{name}] Lỗi gửi SIGINT: {exc}")
            return

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                self.signals.process_log_received.emit(name, f"[{name}] Đã dừng thành công.")
                return
            time.sleep(0.05)

        if proc.poll() is None:
            try:
                pgid = os.getpgid(proc.pid)
                self.signals.process_log_received.emit(name, f"[{name}] Tiến trình chưa thoát, gửi SIGTERM...")
                os.killpg(pgid, signal.SIGTERM)
            except Exception:
                pass
            time.sleep(0.5)

        if proc.poll() is None:
            try:
                pgid = os.getpgid(proc.pid)
                self.signals.process_log_received.emit(name, f"[{name}] Buộc dừng (SIGKILL)!")
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass

    def start_layer(self, layer_key: str) -> None:
        """Khởi động tầng tiến trình tương ứng giống gui_v3."""
        canonical_map = {
            "layer1": "layer_hardware",
            "layer2": "layer_navigation",
            "layer3": "layer_slam",
            "layer4": "layer_perception",
            "layer5": "layer_voice_interaction",
        }
        target_key = canonical_map.get(layer_key, layer_key)

        existing_proc = self._layer_procs.get(target_key)
        if existing_proc and existing_proc.poll() is None:
            self.get_logger().info(f"[LAYER START] Tầng {target_key} đang chạy rồi.")
            self.signals.process_log_received.emit(target_key, f"[{target_key}] Tiến trình đang chạy rồi.")
            return

        def _bg_launcher():
            prefix = self._get_ws_prefix()
            cmd = ""
            if target_key == "layer_hardware":
                esp_port, lidar_port, detect_src = self._detect_esp_lidar_ports()
                self.get_logger().info(f"[LAYER START] Cổng Serial ({detect_src}): ESP={esp_port}, LiDAR={lidar_port}")
                self.signals.process_log_received.emit(
                    target_key,
                    f"[{target_key}] Phát hiện cổng ({detect_src}): ESP={esp_port}, LiDAR={lidar_port}",
                )
                cmd = (
                    prefix
                    + "ros2 launch mo_hinh real_odom.launch.py odom_source:=hoverboard use_sim_time:=false "
                    + f"hoverboard_port:={shlex.quote(esp_port)} esp_port:={shlex.quote(esp_port)} lidar_port:={shlex.quote(lidar_port)}"
                )
            elif target_key == "layer_navigation":
                map_arg = ""
                if self._selected_map_yaml and os.path.exists(self._selected_map_yaml):
                    map_arg = f" map_yaml:={shlex.quote(self._selected_map_yaml)}"
                else:
                    ws_root = self._get_ws_root()
                    default_yaml = ws_root / "my_map" / "my_map.yaml"
                    if default_yaml.is_file():
                        map_arg = f" map_yaml:={shlex.quote(str(default_yaml))}"
                cmd = (
                    prefix
                    + f"ros2 launch mo_hinh real_localization_nav2.launch.py localization_mode:={shlex.quote(self._selected_backend)} "
                    + f"use_sim_time:=false use_rviz:=false{map_arg}"
                )
            elif target_key == "layer_slam":
                cmd = prefix + "ros2 launch mo_hinh real_slam.launch.py use_sim_time:=false use_rviz:=false"
            elif target_key == "layer_perception":
                ws_root = self._get_ws_root()
                front_cfg = ws_root / "my_map" / "cam_truoc.yaml"
                rear_cfg = ws_root / "my_map" / "cam_sau.yaml"
                extra_cfgs = ""
                if front_cfg.is_file():
                    extra_cfgs += f" front_obstacle_config:={shlex.quote(str(front_cfg))}"
                if rear_cfg.is_file():
                    extra_cfgs += f" rear_obstacle_config:={shlex.quote(str(rear_cfg))}"
                cmd = (
                    prefix
                    + "ros2 launch robot_bringup ai_perception.launch.py enable_cameras:=true "
                    + f"enable_obstacle_detectors:=true enable_ai_nodes:=true{extra_cfgs}"
                )
            elif target_key == "layer_voice_interaction":
                cmd = prefix + "ros2 launch robot_bringup backend.launch.py"

            if not cmd:
                self.get_logger().warning(f"[LAYER START] Không tìm thấy cấu hình cho tầng: {layer_key}")
                return

            self.get_logger().info(f"[LAYER START] [{target_key}] CMD: {cmd}")
            self.signals.process_log_received.emit(target_key, f"[{target_key}] START: {cmd}")

            try:
                proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                self._layer_procs[target_key] = proc
                if target_key == "layer_navigation":
                    self._localization_proc = proc
                elif target_key == "layer_slam":
                    self._slam_proc = proc

                if proc.stdout:
                    buf: list[str] = []
                    last_flush = time.monotonic()
                    for line in proc.stdout:
                        clean_line = line.rstrip()
                        if clean_line:
                            buf.append(clean_line)
                        now = time.monotonic()
                        if buf and (len(buf) >= 15 or (now - last_flush) >= 0.08):
                            chunk = "\n".join(buf)
                            buf.clear()
                            last_flush = now
                            self.signals.process_log_received.emit(target_key, chunk)
                    if buf:
                        self.signals.process_log_received.emit(target_key, "\n".join(buf))
            except Exception as exc:
                self.get_logger().error(f"[LAYER START] Lỗi khởi chạy {target_key}: {exc}")
                self.signals.process_log_received.emit(target_key, f"[{target_key}] LỖI: {exc}")

        threading.Thread(target=_bg_launcher, daemon=True).start()

    def stop_layer(self, layer_key: str) -> None:
        """Dừng tầng tiến trình tương ứng."""
        canonical_map = {
            "layer1": "layer_hardware",
            "layer2": "layer_navigation",
            "layer3": "layer_slam",
            "layer4": "layer_perception",
            "layer5": "layer_voice_interaction",
        }
        target_key = canonical_map.get(layer_key, layer_key)
        self.get_logger().info(f"[LAYER STOP] Yêu cầu dừng tầng: {target_key}")

        proc = self._layer_procs.get(target_key)
        if proc and proc.poll() is None:
            self._terminate_proc_group(target_key, proc)
        else:
            self.signals.process_log_received.emit(target_key, f"[{target_key}] Tầng chưa chạy.")

        if target_key == "layer_navigation":
            if self._localization_proc and self._localization_proc != proc:
                self._terminate_proc_group("layer_navigation", self._localization_proc)
            self._localization_proc = None
        elif target_key == "layer_slam":
            if self._slam_proc and self._slam_proc != proc:
                self._terminate_proc_group("layer_slam", self._slam_proc)
            self._slam_proc = None
            self.signals.slam_status_changed.emit(False, "Đã dừng SLAM Mapping")

    def start_all_layers(self, mode: str = "slam_mapping", map_path: str = "", backend: str = "amcl") -> None:
        """Khởi động toàn bộ hệ thống giống hệt nút Start All trong gui_v3 trên background thread."""
        self.get_logger().info(f"[START ALL] Bắt đầu khởi động toàn bộ: mode={mode}, backend={backend}, map={map_path}")
        self.signals.process_log_received.emit("SYSTEM", f"[START ALL] Bắt đầu quy trình khởi động toàn bộ (Mode: {mode})...")

        if map_path:
            self._selected_map_yaml = map_path
        if backend:
            self._selected_backend = backend

        def _bg_start_all():
            # Bước 1: Khởi động Layer 1 (Hardware / Odom)
            self.start_layer("layer_hardware")
            # Chờ 2.5 giây để Layer 1 nạp xong driver, sllidar và controller trước khi bật Layer 2
            time.sleep(2.5)
            clean_mode = mode.lower()
            if "slam" in clean_mode or "mapping" in clean_mode:
                self.get_logger().info("[START ALL] Bước 2: Tự động khởi động Layer 2 (SLAM Mapping)...")
                self.signals.process_log_received.emit("SYSTEM", "[START ALL] Bước 2: Tự động khởi động Layer 2 (SLAM Mapping)...")
                self.start_layer("layer_slam")
            else:
                self.get_logger().info(f"[START ALL] Bước 2: Tự động khởi động Layer 2 (Localization Nav2 - Backend: {self._selected_backend})...")
                self.signals.process_log_received.emit("SYSTEM", f"[START ALL] Bước 2: Tự động khởi động Layer 2 (Localization Nav2 - Backend: {self._selected_backend})...")
                self.start_layer("layer_navigation")

        threading.Thread(target=_bg_start_all, daemon=True).start()

    def stop_all_layers(self) -> None:
        """Dừng toàn bộ mọi tầng tiến trình giống nút Stop All trong gui_v3."""
        self.get_logger().info("[STOP ALL] Yêu cầu dừng toàn bộ các tầng tiến trình...")
        self.signals.process_log_received.emit("SYSTEM", "[STOP ALL] Đang dừng toàn bộ các tầng tiến trình...")
        for key in list(self._layer_procs.keys()):
            self.stop_layer(key)
        self.stop_slam()
        if self._localization_proc and self._localization_proc.poll() is None:
            self._terminate_proc_group("layer_navigation", self._localization_proc)
            self._localization_proc = None
        self.signals.process_log_received.emit("SYSTEM", "[STOP ALL] Đã dừng toàn bộ tiến trình.")

    def cleanup_ros_processes(self) -> None:
        """Dọn dẹp triệt để tất cả các tiến trình ROS 2 nền."""
        self.get_logger().info("[EMERGENCY CLEANUP] Quét và đóng sạch toàn bộ tiến trình ROS 2...")
        self.signals.process_log_received.emit("EMERGENCY", "[EMERGENCY CLEANUP] Đang quét và đóng sạch toàn bộ tiến trình ROS 2...")
        for key in list(self._layer_procs.keys()):
            proc = self._layer_procs.pop(key, None)
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        self._slam_proc = None
        self._localization_proc = None

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
        ]
        for p in patterns:
            try:
                subprocess.run(["pkill", "-9", "-f", p], capture_output=True, timeout=1.0)
            except Exception:
                pass
        self.signals.process_log_received.emit("EMERGENCY", "[EMERGENCY CLEANUP] Hoàn tất dọn dẹp các tiến trình.")
