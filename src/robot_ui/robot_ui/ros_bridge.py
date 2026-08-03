from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_action_status_default,
)
from rclpy.time import Time
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState, CameraInfo, LaserScan
from std_msgs.msg import Bool, Int32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from yolo_msgs.msg import DetectionArray

from robot_ui.state_store import StateStore

MAP_PREVIEW_MAX_DIMENSION = 240
GOAL_STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}
DIAGNOSTIC_LEVEL_LABELS = {
    DiagnosticStatus.OK: "OK",
    DiagnosticStatus.WARN: "WARN",
    DiagnosticStatus.ERROR: "ERROR",
    DiagnosticStatus.STALE: "STALE",
}
ROS_LOG_LEVEL_LABELS = {
    Log.DEBUG: "DEBUG",
    Log.INFO: "INFO",
    Log.WARN: "WARN",
    Log.ERROR: "ERROR",
    Log.FATAL: "FATAL",
}


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def build_occupancy_preview(
    data: Any,
    width: int,
    height: int,
    max_dimension: int = MAP_PREVIEW_MAX_DIMENSION,
) -> dict[str, Any]:
    if width <= 0 or height <= 0:
        return {
            "preview_width": 0,
            "preview_height": 0,
            "sample_step": 1,
            "cells": [],
        }

    sample_step = max(1, math.ceil(max(width, height) / max_dimension))
    preview_width = math.ceil(width / sample_step)
    preview_height = math.ceil(height / sample_step)
    cells: list[int] = []

    for preview_y in range(preview_height):
        start_y = preview_y * sample_step
        end_y = min(height, start_y + sample_step)
        for preview_x in range(preview_width):
            start_x = preview_x * sample_step
            end_x = min(width, start_x + sample_step)
            preview_value = -1
            for source_y in range(start_y, end_y):
                row_offset = source_y * width
                for source_x in range(start_x, end_x):
                    value = int(data[row_offset + source_x])
                    if value >= 50:
                        preview_value = max(preview_value, value)
                    elif value >= 0 and preview_value < 0:
                        preview_value = value
            cells.append(preview_value)

    return {
        "preview_width": preview_width,
        "preview_height": preview_height,
        "sample_step": sample_step,
        "cells": cells,
    }


class RobotUiBridgeNode(Node):
    def __init__(self, store: StateStore, config: dict[str, Any]):
        super().__init__(config.get("node_name", "robot_ui_bridge"))
        self._store = store
        self._topics = config.get("topics", {})
        self._expected_rates = {
            self._topics[key]: float(value)
            for key, value in config.get("expected_rates_hz", {}).items()
            if key in self._topics
        }
        self._graph_config = config.get("graph", {})
        self._navigation_config = config.get("navigation", {})
        self._diagnostics_config = config.get("diagnostics", {})
        self._vision_config = config.get("vision", {})
        self._map_frame = str(self._navigation_config.get("map_frame", "map"))
        self._base_frame = str(
            self._navigation_config.get("base_frame", "base_footprint")
        )
        self._max_scan_points = max(
            1, int(self._navigation_config.get("max_scan_points", 360))
        )
        self._max_path_points = max(
            1, int(self._navigation_config.get("max_path_points", 300))
        )
        self._costmap_preview_max_dimension = max(
            32,
            int(
                self._navigation_config.get(
                    "costmap_preview_max_dimension", 160
                )
            ),
        )
        self._last_graph_error = ""
        self._last_tf_error = ""
        self._last_scan_tf_error = ""
        self._nav2_status_by_goal: dict[str, int] = {}
        self._nav2_status_history: list[dict[str, Any]] = []
        self._diagnostic_levels: dict[str, int] = {}
        self._last_mission_mode = ""
        self._last_mission_status_error = ""
        self._last_mission_error = ""
        self._vision_pose_state: dict[str, Any] = {}
        self._wave_detected = False
        self._last_wave_status = ""
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        expected_rate_keys = set(config.get("expected_rates_hz", {}))
        for topic_key, topic_name in self._topics.items():
            self._store.ensure_topic(
                topic_name,
                self._expected_rates.get(topic_name),
                health_monitored=topic_key in expected_rate_keys,
            )

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            BatteryState, self._topics["battery"], self._on_battery, 10
        )
        self.create_subscription(
            Int32MultiArray, self._topics["encoders"], self._on_encoders, 20
        )
        self.create_subscription(
            OccupancyGrid, self._topics["map"], self._on_map, map_qos
        )
        self.create_subscription(Odometry, self._topics["odom"], self._on_odom, 20)
        self.create_subscription(
            LaserScan, self._topics["scan"], self._on_scan, sensor_qos
        )
        self.create_subscription(PoseStamped, self._topics["goal"], self._on_goal, 10)
        self.create_subscription(
            Path, self._topics["global_path"], self._on_global_path, 10
        )
        self.create_subscription(
            Path, self._topics["local_path"], self._on_local_path, 10
        )
        self.create_subscription(
            OccupancyGrid,
            self._topics["global_costmap"],
            self._on_global_costmap,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self._topics["local_costmap"],
            self._on_local_costmap,
            map_qos,
        )
        self.create_subscription(
            GoalStatusArray,
            self._topics["nav2_status"],
            self._on_nav2_status,
            qos_profile_action_status_default,
        )
        self.create_subscription(
            DiagnosticArray,
            self._topics["diagnostics"],
            self._on_diagnostics,
            10,
        )
        self.create_subscription(Log, self._topics["rosout"], self._on_rosout, 100)
        self.create_subscription(
            String,
            self._topics["mission_status"],
            self._on_mission_status,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self._topics["camera_color_info"],
            self._on_color_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            CameraInfo,
            self._topics["camera_depth_info"],
            self._on_depth_camera_info,
            sensor_qos,
        )
        self.create_subscription(
            DetectionArray,
            self._topics["yolo_detections"],
            self._on_yolo_detections,
            10,
        )
        self.create_subscription(
            Bool,
            self._topics["wave_detected"],
            self._on_wave_detected,
            10,
        )
        self.create_subscription(
            String,
            self._topics["wave_status"],
            self._on_wave_status,
            10,
        )
        self.create_subscription(
            LaserScan,
            self._topics["scan_obstacles"],
            self._on_scan_obstacles,
            sensor_qos,
        )
        self.create_timer(
            max(
                0.05,
                float(self._navigation_config.get("tf_refresh_seconds", 0.2)),
            ),
            self._refresh_tf_pose,
        )
        self.create_timer(
            max(
                0.25,
                float(
                    self._diagnostics_config.get(
                        "health_refresh_seconds", 1.0
                    )
                ),
            ),
            self._refresh_topic_health,
        )
        self.create_timer(
            max(0.5, float(self._graph_config.get("refresh_seconds", 2.0))),
            self._refresh_graph,
        )

        self._store.append_event(
            "INFO", "ros", f"ROS bridge node started: {self.get_name()}"
        )

    def _on_battery(self, message: BatteryState) -> None:
        percentage = message.percentage if math.isfinite(message.percentage) else None
        battery_state = {
            "voltage": message.voltage,
            "temperature": message.temperature,
            "percentage": percentage,
            "present": message.present,
        }
        self._store.patch_section(
            "hardware",
            {"battery": battery_state},
        )
        self._store.update_topic(
            self._topics["battery"],
            {
                "message_type": "sensor_msgs/BatteryState",
                "latest": battery_state,
            },
            self._expected_rates.get(self._topics["battery"]),
        )

    def _on_encoders(self, message: Int32MultiArray) -> None:
        encoder_values = list(message.data)
        self._store.patch_section("hardware", {"encoders": encoder_values})
        self._store.update_topic(
            self._topics["encoders"],
            {
                "message_type": "std_msgs/Int32MultiArray",
                "latest": encoder_values[:32],
            },
            self._expected_rates.get(self._topics["encoders"]),
        )

    @staticmethod
    def _occupancy_grid_state(
        message: OccupancyGrid, max_dimension: int
    ) -> dict[str, Any]:
        return {
            "frame_id": message.header.frame_id,
            "width": message.info.width,
            "height": message.info.height,
            "resolution": message.info.resolution,
            "origin_x": message.info.origin.position.x,
            "origin_y": message.info.origin.position.y,
            "origin_yaw": quaternion_to_yaw(
                message.info.origin.orientation.x,
                message.info.origin.orientation.y,
                message.info.origin.orientation.z,
                message.info.origin.orientation.w,
            ),
            **build_occupancy_preview(
                message.data,
                message.info.width,
                message.info.height,
                max_dimension,
            ),
        }

    def _on_map(self, message: OccupancyGrid) -> None:
        map_state = self._occupancy_grid_state(
            message, MAP_PREVIEW_MAX_DIMENSION
        )
        self._store.patch_section(
            "navigation",
            {"map": map_state},
        )
        self._store.update_topic(
            self._topics["map"],
            {
                "message_type": "nav_msgs/OccupancyGrid",
                "latest": {
                    "frame_id": map_state["frame_id"],
                    "width": map_state["width"],
                    "height": map_state["height"],
                    "resolution": map_state["resolution"],
                    "origin_x": map_state["origin_x"],
                    "origin_y": map_state["origin_y"],
                },
            },
            self._expected_rates.get(self._topics["map"]),
        )

    def _on_odom(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        odom_state = {
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            "linear_x": twist.linear.x,
            "linear_y": twist.linear.y,
            "angular_z": twist.angular.z,
        }
        self._store.patch_section(
            "navigation",
            {"odom": odom_state},
        )
        self._store.update_topic(
            self._topics["odom"],
            {
                "message_type": "nav_msgs/Odometry",
                "latest": odom_state,
            },
            self._expected_rates.get(self._topics["odom"]),
        )

    def _on_scan(self, message: LaserScan) -> None:
        valid_ranges = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        scan_points, transform_error = self._scan_points_in_map(message)
        scan_state = {
            "frame_id": message.header.frame_id,
            "target_frame_id": self._map_frame,
            "point_count": len(message.ranges),
            "valid_point_count": len(valid_ranges),
            "nearest_range": min(valid_ranges) if valid_ranges else None,
            "range_min": message.range_min,
            "range_max": message.range_max,
            "points_xy": scan_points,
            "transform_state": "ERROR" if transform_error else "OK",
            "transform_error": transform_error,
        }
        self._store.patch_section("navigation", {"scan": scan_state})
        self._store.update_topic(
            self._topics["scan"],
            {
                "message_type": "sensor_msgs/LaserScan",
                "latest": scan_state,
            },
            self._expected_rates.get(self._topics["scan"]),
        )

    def _scan_points_in_map(
        self, message: LaserScan
    ) -> tuple[list[list[float]], str]:
        source_frame = message.header.frame_id or self._base_frame
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.03),
            )
        except TransformException:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.03),
                )
            except TransformException as exc:
                error = str(exc)
                if error != self._last_scan_tf_error:
                    self._store.append_event("WARN", "scan_tf", error)
                    self._last_scan_tf_error = error
                return [], error

        self._last_scan_tf_error = ""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotation_xx = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        rotation_xy = 2.0 * (
            rotation.x * rotation.y - rotation.z * rotation.w
        )
        rotation_yx = 2.0 * (
            rotation.x * rotation.y + rotation.z * rotation.w
        )
        rotation_yy = 1.0 - 2.0 * (
            rotation.x * rotation.x + rotation.z * rotation.z
        )
        sample_step = max(
            1, math.ceil(len(message.ranges) / self._max_scan_points)
        )
        points: list[list[float]] = []
        for range_index in range(0, len(message.ranges), sample_step):
            range_value = message.ranges[range_index]
            if (
                not math.isfinite(range_value)
                or range_value < message.range_min
                or range_value > message.range_max
            ):
                continue
            angle = message.angle_min + range_index * message.angle_increment
            local_x = range_value * math.cos(angle)
            local_y = range_value * math.sin(angle)
            world_x = (
                rotation_xx * local_x
                + rotation_xy * local_y
                + translation.x
            )
            world_y = (
                rotation_yx * local_x
                + rotation_yy * local_y
                + translation.y
            )
            points.append([round(world_x, 3), round(world_y, 3)])
        return points, ""

    def _refresh_tf_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.03),
            )
        except TransformException as exc:
            error = str(exc)
            if error != self._last_tf_error:
                self._store.patch_section(
                    "navigation",
                    {
                        "tf": {
                            "state": "ERROR",
                            "map_frame": self._map_frame,
                            "base_frame": self._base_frame,
                            "error": error,
                        }
                    },
                )
                self._store.append_event("WARN", "tf", error)
                self._last_tf_error = error
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose_state = {
            "source": "tf2",
            "frame_id": self._map_frame,
            "child_frame_id": self._base_frame,
            "x": translation.x,
            "y": translation.y,
            "yaw": quaternion_to_yaw(
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            ),
        }
        self._store.patch_section(
            "navigation",
            {
                "pose": pose_state,
                "tf": {
                    "state": "OK",
                    "map_frame": self._map_frame,
                    "base_frame": self._base_frame,
                    "last_success_at": time.time(),
                    "error": "",
                },
            },
        )
        self._last_tf_error = ""

    def _on_goal(self, message: PoseStamped) -> None:
        pose = message.pose
        goal_state = {
            "frame_id": message.header.frame_id,
            "x": pose.position.x,
            "y": pose.position.y,
            "yaw": quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            "received_at": time.time(),
        }
        self._store.patch_section("navigation", {"goal": goal_state})
        self._store.update_topic(
            self._topics["goal"],
            {
                "message_type": "geometry_msgs/PoseStamped",
                "latest": goal_state,
            },
            self._expected_rates.get(self._topics["goal"]),
        )

    def _path_state(self, message: Path) -> dict[str, Any]:
        point_count = len(message.poses)
        sample_step = max(1, math.ceil(point_count / self._max_path_points))
        points = [
            [round(stamped_pose.pose.position.x, 3), round(stamped_pose.pose.position.y, 3)]
            for stamped_pose in message.poses[::sample_step]
        ]
        return {
            "frame_id": message.header.frame_id,
            "point_count": point_count,
            "sample_step": sample_step,
            "points_xy": points,
        }

    def _on_global_path(self, message: Path) -> None:
        path_state = self._path_state(message)
        self._store.patch_section("navigation", {"global_path": path_state})
        self._store.update_topic(
            self._topics["global_path"],
            {
                "message_type": "nav_msgs/Path",
                "latest": {
                    "frame_id": path_state["frame_id"],
                    "point_count": path_state["point_count"],
                },
            },
            self._expected_rates.get(self._topics["global_path"]),
        )

    def _on_local_path(self, message: Path) -> None:
        path_state = self._path_state(message)
        self._store.patch_section("navigation", {"local_path": path_state})
        self._store.update_topic(
            self._topics["local_path"],
            {
                "message_type": "nav_msgs/Path",
                "latest": {
                    "frame_id": path_state["frame_id"],
                    "point_count": path_state["point_count"],
                },
            },
            self._expected_rates.get(self._topics["local_path"]),
        )

    def _on_global_costmap(self, message: OccupancyGrid) -> None:
        self._on_costmap("global_costmap", message)

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        self._on_costmap("local_costmap", message)

    def _on_costmap(self, key: str, message: OccupancyGrid) -> None:
        costmap_state = self._occupancy_grid_state(
            message, self._costmap_preview_max_dimension
        )
        self._store.patch_section("navigation", {key: costmap_state})
        topic = self._topics[key]
        self._store.update_topic(
            topic,
            {
                "message_type": "nav_msgs/OccupancyGrid",
                "latest": {
                    "frame_id": costmap_state["frame_id"],
                    "width": costmap_state["width"],
                    "height": costmap_state["height"],
                    "resolution": costmap_state["resolution"],
                },
            },
            self._expected_rates.get(topic),
        )

    def _on_nav2_status(self, message: GoalStatusArray) -> None:
        statuses: list[dict[str, Any]] = []
        for status_entry in message.status_list:
            goal_id = bytes(status_entry.goal_info.goal_id.uuid).hex()
            status_code = int(status_entry.status)
            status_label = GOAL_STATUS_LABELS.get(status_code, str(status_code))
            status_state = {
                "goal_id": goal_id,
                "status_code": status_code,
                "status": status_label,
                "accepted_at": (
                    status_entry.goal_info.stamp.sec
                    + status_entry.goal_info.stamp.nanosec / 1_000_000_000
                ),
            }
            statuses.append(status_state)
            if self._nav2_status_by_goal.get(goal_id) != status_code:
                self._nav2_status_history.append(
                    {**status_state, "transition_at": time.time()}
                )
                self._nav2_status_history = self._nav2_status_history[-30:]
                self._nav2_status_by_goal[goal_id] = status_code

        active_codes = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active_statuses = [
            status
            for status in statuses
            if status["status_code"] in active_codes
        ]
        primary = active_statuses[-1] if active_statuses else (
            statuses[-1]
            if statuses
            else {"goal_id": "", "status_code": 0, "status": "IDLE"}
        )
        nav2_state = {
            **primary,
            "active_goal_count": len(active_statuses),
            "status_count": len(statuses),
            "history": list(reversed(self._nav2_status_history)),
        }
        self._store.patch_section("navigation", {"nav2": nav2_state})
        self._store.update_topic(
            self._topics["nav2_status"],
            {
                "message_type": "action_msgs/GoalStatusArray",
                "latest": {
                    "status": nav2_state["status"],
                    "goal_id": nav2_state["goal_id"],
                    "active_goal_count": nav2_state["active_goal_count"],
                },
            },
            self._expected_rates.get(self._topics["nav2_status"]),
        )

    def _on_mission_status(self, message: String) -> None:
        received_at = time.time()
        topic = self._topics["mission_status"]
        try:
            if len(message.data) > 65_536:
                raise ValueError("mission status payload exceeds 65536 characters")
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("mission status payload must be a JSON object")
        except (json.JSONDecodeError, ValueError) as exc:
            error = str(exc)
            if error != self._last_mission_status_error:
                self._store.append_event("ERROR", "mission_status", error)
                self._last_mission_status_error = error
            self._store.patch_section(
                "mission",
                {
                    "mode": "ERROR",
                    "active": False,
                    "last_error": error,
                    "source_topic": topic,
                    "received_at": received_at,
                },
            )
            self._store.update_topic(
                topic,
                {
                    "message_type": "std_msgs/String",
                    "latest": {"error": error},
                },
                self._expected_rates.get(topic),
            )
            return

        self._last_mission_status_error = ""
        mode = str(payload.get("mode", "UNKNOWN"))
        if self._last_mission_mode and mode != self._last_mission_mode:
            self._store.append_event(
                "INFO",
                "mission",
                f"Mode changed from {self._last_mission_mode} to {mode}",
            )
        self._last_mission_mode = mode

        mission_error = str(payload.get("last_error", ""))
        if mission_error and mission_error != self._last_mission_error:
            self._store.append_event("WARN", "mission", mission_error[:1024])
        self._last_mission_error = mission_error

        self._store.patch_section(
            "mission",
            {
                **payload,
                "source_topic": topic,
                "received_at": received_at,
            },
        )
        waypoint = payload.get("waypoint", {})
        nav2 = payload.get("nav2", {})
        self._store.update_topic(
            topic,
            {
                "message_type": "std_msgs/String",
                "latest": {
                    "mode": mode,
                    "active": bool(payload.get("active", False)),
                    "waypoint": waypoint if isinstance(waypoint, dict) else {},
                    "nav2": nav2 if isinstance(nav2, dict) else {},
                },
            },
            self._expected_rates.get(topic),
        )

    @staticmethod
    def _camera_info_state(message: CameraInfo) -> dict[str, Any]:
        timestamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec / 1_000_000_000
        ) or time.time()
        return {
            "frame_id": message.header.frame_id,
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "fx": float(message.k[0]) if len(message.k) > 0 else None,
            "fy": float(message.k[4]) if len(message.k) > 4 else None,
            "cx": float(message.k[2]) if len(message.k) > 2 else None,
            "cy": float(message.k[5]) if len(message.k) > 5 else None,
            "timestamp": timestamp,
        }

    def _on_color_camera_info(self, message: CameraInfo) -> None:
        self._on_camera_info("color_camera", "camera_color_info", message)

    def _on_depth_camera_info(self, message: CameraInfo) -> None:
        self._on_camera_info("depth_camera", "camera_depth_info", message)

    def _on_camera_info(
        self,
        section_key: str,
        topic_key: str,
        message: CameraInfo,
    ) -> None:
        camera_state = self._camera_info_state(message)
        self._store.patch_section("vision", {section_key: camera_state})
        topic = self._topics[topic_key]
        self._store.update_topic(
            topic,
            {
                "message_type": "sensor_msgs/CameraInfo",
                "latest": camera_state,
            },
            self._expected_rates.get(topic),
        )

    def _on_yolo_detections(self, message: DetectionArray) -> None:
        max_samples = max(
            1, int(self._vision_config.get("max_detection_samples", 40))
        )
        class_counts: dict[str, int] = {}
        for detection in message.detections:
            class_name = detection.class_name or str(detection.class_id)
            class_counts[class_name] = class_counts.get(class_name, 0) + 1

        samples = sorted(
            message.detections,
            key=lambda detection: float(detection.score),
            reverse=True,
        )[:max_samples]
        detections = [
            {
                "id": detection.id,
                "class_id": int(detection.class_id),
                "class_name": detection.class_name,
                "score": float(detection.score),
                "center_x": float(detection.bbox.center.position.x),
                "center_y": float(detection.bbox.center.position.y),
                "width": float(detection.bbox.size.x),
                "height": float(detection.bbox.size.y),
                "keypoint_count": len(detection.keypoints.data),
            }
            for detection in samples
        ]
        timestamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec / 1_000_000_000
        ) or time.time()
        yolo_state = {
            "frame_id": message.header.frame_id,
            "timestamp": timestamp,
            "detection_count": len(message.detections),
            "person_count": class_counts.get("person", 0),
            "tracked_count": sum(
                1 for detection in message.detections if detection.id
            ),
            "class_counts": class_counts,
            "detections": detections,
            "truncated": len(message.detections) > max_samples,
        }
        self._store.patch_section("vision", {"yolo": yolo_state})
        topic = self._topics["yolo_detections"]
        self._store.update_topic(
            topic,
            {
                "message_type": "yolo_msgs/DetectionArray",
                "latest": {
                    "frame_id": yolo_state["frame_id"],
                    "detection_count": yolo_state["detection_count"],
                    "person_count": yolo_state["person_count"],
                    "class_counts": class_counts,
                },
            },
            self._expected_rates.get(topic),
        )

    def _on_wave_detected(self, message: Bool) -> None:
        detected = bool(message.data)
        if detected and not self._wave_detected:
            self._store.append_event("INFO", "vision", "Wave detected")
        self._wave_detected = detected
        self._vision_pose_state["wave_detected"] = detected
        self._vision_pose_state["updated_at"] = time.time()
        if detected:
            self._vision_pose_state["last_detected_at"] = time.time()
        self._store.patch_section(
            "vision", {"pose": dict(self._vision_pose_state)}
        )
        topic = self._topics["wave_detected"]
        self._store.update_topic(
            topic,
            {
                "message_type": "std_msgs/Bool",
                "latest": {"wave_detected": detected},
            },
            self._expected_rates.get(topic),
        )

    def _on_wave_status(self, message: String) -> None:
        status = message.data[:1024]
        if status and status != self._last_wave_status:
            self._store.append_event("INFO", "vision", status)
        self._last_wave_status = status
        self._vision_pose_state["status"] = status
        self._vision_pose_state["status_updated_at"] = time.time()
        self._store.patch_section(
            "vision", {"pose": dict(self._vision_pose_state)}
        )
        topic = self._topics["wave_status"]
        self._store.update_topic(
            topic,
            {
                "message_type": "std_msgs/String",
                "latest": {"status": status},
            },
            self._expected_rates.get(topic),
        )

    def _on_scan_obstacles(self, message: LaserScan) -> None:
        valid_ranges = [
            float(value)
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        obstacle_state = {
            "frame_id": message.header.frame_id,
            "point_count": len(message.ranges),
            "obstacle_point_count": len(valid_ranges),
            "nearest_range": min(valid_ranges) if valid_ranges else None,
            "range_min": float(message.range_min),
            "range_max": float(message.range_max),
        }
        self._store.patch_section("vision", {"obstacle": obstacle_state})
        topic = self._topics["scan_obstacles"]
        self._store.update_topic(
            topic,
            {
                "message_type": "sensor_msgs/LaserScan",
                "latest": obstacle_state,
            },
            self._expected_rates.get(topic),
        )

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        timestamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec / 1_000_000_000
        ) or time.time()
        max_statuses = max(
            1, int(self._diagnostics_config.get("max_statuses", 200))
        )
        statuses: list[dict[str, Any]] = []
        summary = {
            "ok_count": 0,
            "warn_count": 0,
            "error_count": 0,
            "stale_count": 0,
            "total_count": 0,
        }
        current_levels: dict[str, int] = {}

        for status in message.status[:max_statuses]:
            level = int(status.level)
            level_name = DIAGNOSTIC_LEVEL_LABELS.get(level, str(level))
            name = status.name or "unnamed diagnostic"
            values = {
                item.key[:256]: item.value[:512]
                for item in status.values
            }
            statuses.append(
                {
                    "name": name,
                    "level": level,
                    "level_name": level_name,
                    "message": status.message[:1024],
                    "hardware_id": status.hardware_id[:256],
                    "values": values,
                    "timestamp": timestamp,
                }
            )
            summary_key = f"{level_name.lower()}_count"
            if summary_key in summary:
                summary[summary_key] += 1
            summary["total_count"] += 1
            current_levels[name] = level

            previous_level = self._diagnostic_levels.get(name)
            if (
                previous_level != level
                and level
                in (
                    DiagnosticStatus.WARN,
                    DiagnosticStatus.ERROR,
                    DiagnosticStatus.STALE,
                )
            ):
                self._store.append_event(
                    "WARN" if level == DiagnosticStatus.WARN else "ERROR",
                    "diagnostics",
                    f"{name}: {level_name} - {status.message[:512]}",
                )

        self._diagnostic_levels = current_levels
        diagnostics_state = {
            "summary": summary,
            "statuses": statuses,
            "message_timestamp": timestamp,
            "truncated": len(message.status) > max_statuses,
        }
        self._store.patch_section("diagnostics", diagnostics_state)
        self._store.update_topic(
            self._topics["diagnostics"],
            {
                "message_type": "diagnostic_msgs/DiagnosticArray",
                "latest": summary,
            },
            self._expected_rates.get(self._topics["diagnostics"]),
        )

    def _on_rosout(self, message: Log) -> None:
        timestamp = (
            message.stamp.sec + message.stamp.nanosec / 1_000_000_000
        ) or time.time()
        level = int(message.level)
        level_name = ROS_LOG_LEVEL_LABELS.get(level, str(level))
        entry = {
            "timestamp": timestamp,
            "received_at": time.time(),
            "level": level,
            "level_name": level_name,
            "name": message.name[:256],
            "message": message.msg[:4096],
            "file": message.file[:512],
            "function": message.function[:256],
            "line": int(message.line),
        }
        self._store.append_ros_log(
            entry,
            max(
                1,
                int(
                    self._diagnostics_config.get(
                        "max_rosout_entries", 500
                    )
                ),
            ),
        )
        if level >= Log.WARN:
            self._store.append_event(
                level_name,
                message.name or "rosout",
                message.msg[:1024],
            )
        self._store.update_topic(
            self._topics["rosout"],
            {
                "message_type": "rcl_interfaces/Log",
                "latest": {
                    "level": level_name,
                    "name": message.name,
                    "message": message.msg[:1024],
                },
            },
            self._expected_rates.get(self._topics["rosout"]),
        )

    def _refresh_topic_health(self) -> None:
        transitions = self._store.refresh_topic_health(
            stale_multiplier=max(
                1.0,
                float(
                    self._diagnostics_config.get("stale_multiplier", 3.0)
                ),
            ),
            minimum_stale_seconds=max(
                0.1,
                float(
                    self._diagnostics_config.get(
                        "minimum_stale_seconds", 2.0
                    )
                ),
            ),
            warn_rate_ratio=max(
                0.0,
                float(
                    self._diagnostics_config.get("warn_rate_ratio", 0.5)
                ),
            ),
        )
        for transition in transitions:
            state = transition["state"]
            if state not in ("WARN", "STALE"):
                continue
            self._store.append_event(
                "WARN" if state == "WARN" else "ERROR",
                "topic_health",
                (
                    f"{transition['topic']} changed from "
                    f"{transition['previous']} to {state}"
                ),
            )

    @staticmethod
    def _full_node_name(name: str, namespace: str) -> str:
        if namespace in ("", "/"):
            return f"/{name}"
        return f"{namespace.rstrip('/')}/{name}"

    @staticmethod
    def _is_hidden(name: str) -> bool:
        return any(part.startswith("_") for part in name.split("/") if part)

    @staticmethod
    def _qos_policy_name(value: Any) -> str:
        name = getattr(value, "name", None)
        if name:
            return str(name)
        return str(value).rsplit(".", maxsplit=1)[-1]

    @classmethod
    def _qos_summary(cls, endpoint: Any) -> dict[str, Any]:
        profile = endpoint.qos_profile
        return {
            "reliability": cls._qos_policy_name(profile.reliability),
            "durability": cls._qos_policy_name(profile.durability),
            "history": cls._qos_policy_name(profile.history),
            "depth": int(profile.depth),
        }

    def _refresh_graph(self) -> None:
        try:
            include_hidden = bool(self._graph_config.get("include_hidden", False))
            excluded_prefixes = tuple(
                self._graph_config.get("exclude_topic_prefixes", [])
            )
            max_topics = max(1, int(self._graph_config.get("max_topics", 120)))

            graph_nodes: dict[str, dict[str, Any]] = {}
            graph_edges: list[dict[str, Any]] = []
            edge_ids: set[str] = set()

            discovered_nodes = self.get_node_names_and_namespaces()
            for node_name, namespace in discovered_nodes:
                full_name = self._full_node_name(node_name, namespace)
                if not include_hidden and self._is_hidden(full_name):
                    continue
                graph_nodes[f"node:{full_name}"] = {
                    "id": f"node:{full_name}",
                    "label": full_name,
                    "kind": "node",
                    "namespace": namespace,
                }

            topic_entries = []
            for topic_name, topic_types in sorted(self.get_topic_names_and_types()):
                if excluded_prefixes and topic_name.startswith(excluded_prefixes):
                    continue
                if not include_hidden and self._is_hidden(topic_name):
                    continue
                topic_entries.append((topic_name, topic_types))
                if len(topic_entries) >= max_topics:
                    break

            publisher_count = 0
            subscription_count = 0
            for topic_name, topic_types in topic_entries:
                topic_id = f"topic:{topic_name}"
                graph_nodes[topic_id] = {
                    "id": topic_id,
                    "label": topic_name,
                    "kind": "topic",
                    "message_types": topic_types,
                }

                for endpoint in self.get_publishers_info_by_topic(topic_name):
                    full_name = self._full_node_name(
                        endpoint.node_name, endpoint.node_namespace
                    )
                    node_id = f"node:{full_name}"
                    if not include_hidden and self._is_hidden(full_name):
                        continue
                    graph_nodes.setdefault(
                        node_id,
                        {
                            "id": node_id,
                            "label": full_name,
                            "kind": "node",
                            "namespace": endpoint.node_namespace,
                        },
                    )
                    edge_id = f"publish:{node_id}:{topic_id}"
                    if edge_id not in edge_ids:
                        graph_edges.append(
                            {
                                "id": edge_id,
                                "source": node_id,
                                "target": topic_id,
                                "kind": "publish",
                                "message_types": topic_types,
                                "qos": self._qos_summary(endpoint),
                            }
                        )
                        edge_ids.add(edge_id)
                        publisher_count += 1

                for endpoint in self.get_subscriptions_info_by_topic(topic_name):
                    full_name = self._full_node_name(
                        endpoint.node_name, endpoint.node_namespace
                    )
                    node_id = f"node:{full_name}"
                    if not include_hidden and self._is_hidden(full_name):
                        continue
                    graph_nodes.setdefault(
                        node_id,
                        {
                            "id": node_id,
                            "label": full_name,
                            "kind": "node",
                            "namespace": endpoint.node_namespace,
                        },
                    )
                    edge_id = f"subscribe:{topic_id}:{node_id}"
                    if edge_id not in edge_ids:
                        graph_edges.append(
                            {
                                "id": edge_id,
                                "source": topic_id,
                                "target": node_id,
                                "kind": "subscribe",
                                "message_types": topic_types,
                                "qos": self._qos_summary(endpoint),
                            }
                        )
                        edge_ids.add(edge_id)
                        subscription_count += 1

            node_count = sum(
                1 for item in graph_nodes.values() if item["kind"] == "node"
            )
            topic_count = sum(
                1 for item in graph_nodes.values() if item["kind"] == "topic"
            )
            self._store.patch_section(
                "ros_graph",
                {
                    "nodes": list(graph_nodes.values()),
                    "edges": graph_edges,
                    "summary": {
                        "node_count": node_count,
                        "topic_count": topic_count,
                        "publisher_count": publisher_count,
                        "subscription_count": subscription_count,
                        "edge_count": len(graph_edges),
                    },
                },
            )
            self._last_graph_error = ""
        except Exception as exc:
            error = str(exc)
            if error != self._last_graph_error:
                self._store.append_event("WARN", "ros_graph", error)
                self._last_graph_error = error


class RosBridge:
    def __init__(self, store: StateStore, config: dict[str, Any]):
        self._store = store
        self._config = config
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node: RobotUiBridgeNode | None = None
        self._executor: MultiThreadedExecutor | None = None
        self._owns_rclpy = False

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="robot-ui-ros", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._owns_rclpy = True

            self._node = RobotUiBridgeNode(self._store, self._config)
            self._executor = MultiThreadedExecutor(num_threads=2)
            self._executor.add_node(self._node)
            self._store.patch_section(
                "robot", {"state": "ONLINE", "ros_connected": True}
            )

            while not self._stop_event.is_set() and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.2)
        except Exception as exc:
            self._store.patch_section(
                "robot", {"state": "FAULT", "ros_connected": False}
            )
            self._store.append_event("ERROR", "ros", str(exc))
        finally:
            if self._executor and self._node:
                self._executor.remove_node(self._node)
            if self._node:
                self._node.destroy_node()
            if self._owns_rclpy and rclpy.ok():
                rclpy.shutdown()
            self._store.patch_section(
                "robot", {"state": "OFFLINE", "ros_connected": False}
            )
