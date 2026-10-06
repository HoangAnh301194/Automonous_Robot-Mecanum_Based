from __future__ import annotations

import math
import time
from functools import partial
from typing import Any

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from robot_interfaces.action import DirectGo
from robot_interfaces.msg import NavigationStatus, RobotStatus
from robot_interfaces.srv import GetLocation, GetLocationList

from robot_ui.kiosk.signals import KioskSignals


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
        self.create_timer(1.0, self._check_backend_connection)

        self.get_logger().info("Native touchscreen UI ROS adapter started.")

    def _on_robot_status(self, msg: RobotStatus) -> None:
        self._last_robot_status_at = time.monotonic()
        if not self._connected:
            self._connected = True
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
            self.signals.connection_changed.emit(connected)

    def _on_navigation_status(self, msg: NavigationStatus) -> None:
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
        if width <= 0 or height <= 0:
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
        self.signals.pose_changed.emit(
            {
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
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._have_localized_pose = True
        self._emit_pose(msg.pose.pose, str(msg.header.frame_id), "amcl")

    def _on_odom(self, msg: Odometry) -> None:
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
            self.signals.command_status_changed.emit(
                "warning",
                "Dịch vụ danh sách địa điểm chưa sẵn sàng",
            )
            return
        future = self._location_client.call_async(GetLocationList.Request())
        future.add_done_callback(self._on_locations_response)

    def _on_locations_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"Cannot load location list: {error}")
            self.signals.command_status_changed.emit("error", "Không tải được địa điểm")
            return
        if response.success:
            self._request_location_coordinates(list(response.locations))
        else:
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
                self._pending_locations[name] = {
                    "name": name,
                    "x": float(response.x),
                    "y": float(response.y),
                    "yaw": float(response.yaw),
                }
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().warning(f"Cannot load location {name}: {error}")
        self._remaining_locations -= 1
        if self._remaining_locations == 0:
            self.signals.locations_changed.emit(
                [self._pending_locations[key] for key in sorted(self._pending_locations)]
            )

    def navigate_to(self, location: str) -> None:
        target = location.strip()
        if not target:
            return
        if not self._direct_go_client.server_is_ready():
            self.signals.command_status_changed.emit(
                "warning",
                "Navigation Manager chưa sẵn sàng",
            )
            return

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
            self.get_logger().error(f"Cannot send navigation goal: {error}")
            self.signals.command_status_changed.emit("error", "Không gửi được điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.signals.command_status_changed.emit("error", "Điểm đến bị từ chối")
            self.signals.navigation_active_changed.emit(False)
            return
        self._goal_handle = goal_handle
        self.signals.command_status_changed.emit("progress", "Robot bắt đầu di chuyển")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        distance = getattr(feedback, "distance_remaining", None)
        if distance is not None:
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
            self.get_logger().error(f"Navigation result failed: {error}")
            self.signals.command_status_changed.emit("error", "Mất kết quả dẫn đường")
            return
        tone = "success" if result.success else "error"
        self.signals.command_status_changed.emit(tone, str(result.finish_message))

    def navigate_to_pose(self, x: float, y: float, yaw: float) -> None:
        if not self._nav2_client.server_is_ready():
            self.signals.command_status_changed.emit(
                "warning",
                "Nav2 chưa sẵn sàng nhận điểm đến",
            )
            self.signals.navigation_active_changed.emit(False)
            return

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
            self.get_logger().error(f"Cannot send Nav2 goal: {error}")
            self.signals.command_status_changed.emit("error", "Không gửi được điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        if not goal_handle.accepted:
            self.signals.command_status_changed.emit("error", "Nav2 từ chối điểm đến")
            self.signals.navigation_active_changed.emit(False)
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav2_result)

    def _on_nav2_feedback(self, feedback_message: Any) -> None:
        distance = float(feedback_message.feedback.distance_remaining)
        self.signals.command_status_changed.emit("progress", f"Còn {distance:.1f} m")

    def _on_nav2_result(self, future: Any) -> None:
        self._goal_handle = None
        self.signals.navigation_active_changed.emit(False)
        try:
            status = int(future.result().status)
        except Exception as error:  # pragma: no cover - depends on ROS transport
            self.get_logger().error(f"Nav2 result failed: {error}")
            self.signals.command_status_changed.emit("error", "Mất kết quả dẫn đường")
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.signals.command_status_changed.emit("success", "Đã tới điểm đến")
        elif status == GoalStatus.STATUS_CANCELED:
            self.signals.command_status_changed.emit("warning", "Đã dừng hành trình")
        else:
            self.signals.command_status_changed.emit("error", "Không thể tới điểm đến")

    def cancel_navigation(self) -> None:
        if self._goal_handle is None:
            self.signals.command_status_changed.emit("warning", "Không có hành trình để hủy")
            self.signals.navigation_active_changed.emit(False)
            return
        self._goal_handle.cancel_goal_async()
        self.signals.command_status_changed.emit("progress", "Đang hủy hành trình")
        self.signals.navigation_active_changed.emit(False)
