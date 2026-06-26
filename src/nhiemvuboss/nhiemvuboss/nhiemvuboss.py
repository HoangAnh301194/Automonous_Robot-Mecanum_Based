#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from sensor_msgs.msg import CameraInfo, Image
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray, Detection

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
from tf2_ros import TransformException

from cv_bridge import CvBridge


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str


@dataclass
class WaveTrackState:
    raised_since: Optional[float] = None
    last_seen: float = 0.0
    active_hand: str = ""


class Mode(Enum):
    WAIT_FOR_B = 0
    GO_TO_B = 1
    GO_TO_PERSON = 2
    WAIT_10S = 3
    DONE = 4
    WAIT_AT_WAYPOINT = 5


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class MissionAB(Node):
    """
    Success-first mission:
      - wait for user to set goal B in RViz (PoseStamped), or fall back to fixed params
      - go to B with Nav2
      - during A->B, intercept at most ONE nearest person:
          cancel B, approach to target_distance, wait 10s, then resume B
    """
    def __init__(self):
        super().__init__("mission_ab_person_once")

        self.bridge = CvBridge()

        # ---------- Params ----------
        self.declare_parameter("detections_topic", "/yolo/detections")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("aligned_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")

        self.declare_parameter("class_name", "person")
        self.declare_parameter("min_confidence", 0.6)

        self.declare_parameter("depth_roi_size", 20)
        self.declare_parameter("max_depth_m", 6.0)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        self.declare_parameter("target_distance", 0.4)
        self.declare_parameter("wait_seconds", 10.0)

        self.declare_parameter("navigate_action_name", "/navigate_to_pose")

        # Goal B: có hai cách
        # 1) Ng??i dùng ch?n trên RViz (khuy?n ngh?) qua topic goal_input_topic.
        # 2) Dùng giá tr? tham s? t?nh (use_fixed_goal_B=true).
        self.declare_parameter("goal_input_topic", "/goal_pose")
        self.declare_parameter("waypoints_input_topic", "/nhiemvuboss/waypoints_json")
        self.declare_parameter("use_fixed_goal_B", False)
        self.declare_parameter("goal_B_x", 0.0)
        self.declare_parameter("goal_B_y", 0.0)
        self.declare_parameter("goal_B_yaw", 0.0)
        self.declare_parameter("wait_at_waypoint_seconds", 0.0)

        # Behavior tuning (success-first)
        self.declare_parameter("intercept_enabled", True)
        self.declare_parameter("intercept_min_range_m", 0.4)   # quá g?n thì b?
        self.declare_parameter("intercept_max_range_m", 3.0)   # xa quá thì b?
        self.declare_parameter("require_map_tf", True)         # n?u true: ch? intercept khi cam->map OK
        self.declare_parameter("wave_hold_seconds", 3.0)
        self.declare_parameter("wave_margin_px", 20.0)
        self.declare_parameter("wave_keypoint_confidence", 0.4)
        self.declare_parameter("wave_track_timeout_seconds", 0.8)
        self.declare_parameter("wave_track_bin_px", 48.0)

        # Ctrl+C cancel
        self.declare_parameter("cancel_on_shutdown", True)
        self.declare_parameter("stop_cmd_vel_on_shutdown", True)

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- Data buffers ----------
        self.intr: Optional[Intrinsics] = None
        self.depth_m: Optional[np.ndarray] = None

        # nearest person candidate
        self.nearest_det: Optional[Detection] = None
        self.nearest_z: float = 1e9  # meters
        self.wave_tracks: Dict[str, WaveTrackState] = {}
        self.wave_candidate_key: Optional[str] = None
        self.wave_candidate_hand: str = ""
        self.wave_candidate_held_s: float = 0.0
        self.wave_candidate_seen_at: float = 0.0

        # ---------- Nav2 ----------
        self.action_name = self.get_parameter("navigate_action_name").value
        self.client = ActionClient(self, NavigateToPose, self.action_name)

        self.goal_handle = None
        self.mode: Mode = Mode.WAIT_FOR_B
        self.intercept_done_this_trip = False

        self._wait_end_time: Optional[float] = None

        # pubs
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        # subs
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value, self.on_caminfo, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("aligned_depth_topic").value, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(DetectionArray, self.get_parameter("detections_topic").value, self.on_detections, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_input_topic").value, self.on_goal_pose, 10)
        self.create_subscription(
            String,
            self.get_parameter("waypoints_input_topic").value,
            self.on_waypoints_json,
            10,
        )

        # loop
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info(
            "Mission: GO_TO_B; intercept only when hand-raise is held long enough.\n"
            f"Nav2 action={self.action_name} map_frame={self.get_parameter('map_frame').value} "
            f"base_frame={self.get_parameter('base_frame').value} "
            f"target_distance={self.get_parameter('target_distance').value}m "
            f"wave_hold={self.get_parameter('wave_hold_seconds').value}s"
        )

        # start
        self.goal_B_pose: Optional[PoseStamped] = None
        self.waypoint_sequence: List[Tuple[str, PoseStamped]] = []
        self.current_waypoint_index: int = -1
        if bool(self.get_parameter("use_fixed_goal_B").value):
            self.goal_B_pose = self._goal_B_pose_from_params()
            self.mode = Mode.GO_TO_B
            self._send_goal_B_once()
        else:
            self.mode = Mode.WAIT_FOR_B
            self.get_logger().info(
                "Waiting for goal B from RViz: use 'Nav2 Goal' tool to publish PoseStamped to "
                f"{self.get_parameter('goal_input_topic').value}. "
                "Batch waypoints topic: "
                f"{self.get_parameter('waypoints_input_topic').value}"
            )

    def _reset_wave_state(self) -> None:
        self.wave_tracks.clear()
        self.wave_candidate_key = None
        self.wave_candidate_hand = ""
        self.wave_candidate_held_s = 0.0
        self.wave_candidate_seen_at = 0.0
        self.nearest_det = None
        self.nearest_z = 1e9

    def _track_key_for_detection(self, det: Detection) -> str:
        det_id = det.id.strip() if det.id else ""
        if det_id:
            return f"id:{det_id}"

        bin_px = max(1.0, float(self.get_parameter("wave_track_bin_px").value))
        u = float(det.bbox.center.position.x)
        v = float(det.bbox.center.position.y)
        return f"anon:{int(u // bin_px)}:{int(v // bin_px)}"

    def _is_hand_raised_from_keypoints(self, det: Detection) -> Tuple[bool, str]:
        conf_th = float(self.get_parameter("wave_keypoint_confidence").value)
        margin = float(self.get_parameter("wave_margin_px").value)

        nose = None
        left_wrist = None
        right_wrist = None

        for kp in det.keypoints.data:
            if kp.id == 1:
                nose = (float(kp.point.x), float(kp.point.y), float(kp.score))
            elif kp.id == 10:
                left_wrist = (float(kp.point.x), float(kp.point.y), float(kp.score))
            elif kp.id == 11:
                right_wrist = (float(kp.point.x), float(kp.point.y), float(kp.score))

        if nose is None or nose[2] < conf_th:
            return False, ""

        nose_y = nose[1]
        left_up = False
        right_up = False

        if left_wrist is not None and left_wrist[2] >= conf_th:
            left_up = left_wrist[1] < (nose_y - margin)
        if right_wrist is not None and right_wrist[2] >= conf_th:
            right_up = right_wrist[1] < (nose_y - margin)

        if left_up and right_up:
            return True, "BOTH"
        if left_up:
            return True, "LEFT"
        if right_up:
            return True, "RIGHT"
        return False, ""

    # ---------------- callbacks ----------------
    def on_caminfo(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0]); fy = float(msg.k[4])
        cx = float(msg.k[2]); cy = float(msg.k[5])
        self.intr = Intrinsics(fx=fx, fy=fy, cx=cx, cy=cy, frame_id=msg.header.frame_id)

    def on_depth(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        arr = np.array(depth)
        if arr.dtype == np.uint16:
            self.depth_m = arr.astype(np.float32) / 1000.0
        else:
            self.depth_m = arr.astype(np.float32)

    def on_goal_pose(self, msg: PoseStamped) -> None:
        """Receive goal B from RViz (PoseStamped)."""
        self.waypoint_sequence = []
        self.current_waypoint_index = -1
        self.goal_B_pose = msg
        self.intercept_done_this_trip = False
        self._reset_wave_state()

        # cancel current goal (if any) before sending new one
        self._cancel_current_goal()

        self.mode = Mode.GO_TO_B
        self.get_logger().warn(
            f"New goal B from RViz: frame={msg.header.frame_id} x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}"
        )
        self._send_goal_B_once()

    def _send_current_waypoint(self) -> None:
        if not self.waypoint_sequence:
            self.mode = Mode.WAIT_FOR_B
            return
        if self.current_waypoint_index < 0 or self.current_waypoint_index >= len(self.waypoint_sequence):
            self.mode = Mode.WAIT_FOR_B
            return

        name, pose = self.waypoint_sequence[self.current_waypoint_index]
        self.goal_B_pose = pose
        self.intercept_done_this_trip = False
        self._reset_wave_state()
        self.goal_B_pose.header.stamp = self.get_clock().now().to_msg()
        self.mode = Mode.GO_TO_B
        self._send_goal(self.goal_B_pose, f"WP:{name}")

    def on_waypoints_json(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid waypoints JSON: {e}")
            return

        raw_wps = payload.get("waypoints", [])
        if not isinstance(raw_wps, list) or len(raw_wps) == 0:
            self.get_logger().warn("Waypoints JSON không có danh sách 'waypoints'.")
            return

        seq: List[Tuple[str, PoseStamped]] = []
        default_frame = self.get_parameter("map_frame").value
        for i, item in enumerate(raw_wps):
            if not isinstance(item, dict):
                continue
            try:
                name = str(item.get("name", f"W{i+1}"))
                x = float(item["x"])
                y = float(item["y"])
                yaw = float(item.get("yaw", 0.0))
                frame_id = str(item.get("frame_id", default_frame))
            except Exception:
                continue

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = yaw_to_quat(yaw)
            seq.append((name, pose))

        if not seq:
            self.get_logger().warn("Waypoints JSON không có waypoint h?p l?.")
            return

        self._cancel_current_goal()
        self.waypoint_sequence = seq
        self.current_waypoint_index = 0
        self.intercept_done_this_trip = False
        self._reset_wave_state()
        self.mode = Mode.GO_TO_B
        self.get_logger().warn(
            f"Received waypoint sequence: {len(seq)} ?i?m. Start t? {seq[0][0]}"
        )
        self._send_current_waypoint()

    def on_detections(self, msg: DetectionArray) -> None:
        # only care while going to B and intercept not yet done
        if self.mode != Mode.GO_TO_B:
            self._reset_wave_state()
            return
        if self.intercept_done_this_trip:
            self._reset_wave_state()
            return
        if not bool(self.get_parameter("intercept_enabled").value):
            self._reset_wave_state()
            return

        cname = self.get_parameter("class_name").value
        min_conf = float(self.get_parameter("min_confidence").value)
        wave_hold_s = float(self.get_parameter("wave_hold_seconds").value)
        track_timeout_s = float(self.get_parameter("wave_track_timeout_seconds").value)
        can_estimate_depth = self.intr is not None and self.depth_m is not None
        now = time.monotonic()

        best: Optional[Detection] = None
        best_z = 1e9
        best_key: Optional[str] = None
        best_hand = ""
        best_hold_s = 0.0

        for det in msg.detections:
            if det.class_name != cname:
                continue
            if float(det.score) < min_conf:
                continue

            track_key = self._track_key_for_detection(det)
            track_state = self.wave_tracks.get(track_key)
            if track_state is None:
                track_state = WaveTrackState()
                self.wave_tracks[track_key] = track_state

            track_state.last_seen = now
            raised, active_hand = self._is_hand_raised_from_keypoints(det)
            if raised:
                if track_state.raised_since is None:
                    track_state.raised_since = now
                track_state.active_hand = active_hand
            else:
                track_state.raised_since = None
                track_state.active_hand = ""

            hold_s = 0.0
            if track_state.raised_since is not None:
                hold_s = now - track_state.raised_since

            if hold_s < wave_hold_s:
                continue

            if not can_estimate_depth:
                continue

            try:
                z = self._depth_at_center(det)
            except Exception:
                continue

            if z < best_z:
                best = det
                best_z = z
                best_key = track_key
                best_hand = track_state.active_hand
                best_hold_s = hold_s

        stale_before = now - track_timeout_s
        for key in list(self.wave_tracks.keys()):
            if self.wave_tracks[key].last_seen < stale_before:
                del self.wave_tracks[key]

        self.nearest_det = best
        self.nearest_z = best_z
        self.wave_candidate_key = best_key
        self.wave_candidate_hand = best_hand
        self.wave_candidate_held_s = best_hold_s
        self.wave_candidate_seen_at = now if best is not None else 0.0

    # ---------------- depth + geometry ----------------
    def _depth_at_center(self, det: Detection) -> float:
        if self.depth_m is None:
            raise RuntimeError("no depth")
        depth = self.depth_m

        u = int(round(det.bbox.center.position.x))
        v = int(round(det.bbox.center.position.y))

        roi_size = int(self.get_parameter("depth_roi_size").value)
        half = max(1, roi_size // 2)

        h, w = depth.shape[:2]
        u0 = int(clamp(u - half, 0, w - 1)); u1 = int(clamp(u + half, 0, w - 1))
        v0 = int(clamp(v - half, 0, h - 1)); v1 = int(clamp(v + half, 0, h - 1))

        roi = depth[v0:v1 + 1, u0:u1 + 1].reshape(-1)
        roi = roi[np.isfinite(roi)]
        roi = roi[(roi > 0.05)]
        max_d = float(self.get_parameter("max_depth_m").value)
        roi = roi[(roi < max_d)]
        if roi.size < 10:
            raise RuntimeError("not enough valid depth")
        return float(np.median(roi))

    def _person_point_in_map(self, det: Detection) -> PointStamped:
        if self.intr is None or self.depth_m is None:
            raise RuntimeError("no intr/depth")

        Z = self._depth_at_center(det)

        u = float(det.bbox.center.position.x)
        v = float(det.bbox.center.position.y)

        X = (u - self.intr.cx) * Z / self.intr.fx
        Y = (v - self.intr.cy) * Z / self.intr.fy

        p_cam = PointStamped()
        p_cam.header.frame_id = self.intr.frame_id
        p_cam.header.stamp = rclpy.time.Time().to_msg()  # latest tf
        p_cam.point.x = float(X)
        p_cam.point.y = float(Y)
        p_cam.point.z = float(Z)

        map_frame = self.get_parameter("map_frame").value

        if not self.tf_buffer.can_transform(map_frame, p_cam.header.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.2)):
            raise RuntimeError(f"No TF {p_cam.header.frame_id}->{map_frame}")

        return self.tf_buffer.transform(p_cam, map_frame, timeout=Duration(seconds=0.2))

    def _robot_xy_in_map(self) -> Tuple[float, float]:
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        tf = self.tf_buffer.lookup_transform(map_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        return float(tf.transform.translation.x), float(tf.transform.translation.y)

    def _compute_approach_goal(self, det: Detection) -> PoseStamped:
        p = self._person_point_in_map(det)
        px, py = float(p.point.x), float(p.point.y)

        rx, ry = self._robot_xy_in_map()

        d = float(self.get_parameter("target_distance").value)

        dx, dy = rx - px, ry - py
        norm = math.hypot(dx, dy)

        if norm < 0.25:
            gx, gy = rx, ry
        else:
            ux, uy = dx / norm, dy / norm
            gx, gy = px + d * ux, py + d * uy

        yaw = math.atan2(py - gy, px - gx)

        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter("map_frame").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(gx)
        goal.pose.position.y = float(gy)
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quat(yaw)
        return goal

    # ---------------- nav helpers ----------------
    def _goal_B_pose_from_params(self) -> PoseStamped:
        x = float(self.get_parameter("goal_B_x").value)
        y = float(self.get_parameter("goal_B_y").value)
        yaw = float(self.get_parameter("goal_B_yaw").value)

        goal = PoseStamped()
        goal.header.frame_id = self.get_parameter("map_frame").value
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = yaw_to_quat(yaw)
        return goal

    def _send_goal(self, pose: PoseStamped, label: str):
        if not self.client.server_is_ready():
            self.get_logger().warn("Nav2 server not ready yet...")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().warn(f"SEND GOAL ({label}): frame={pose.header.frame_id} x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}")

        fut = self.client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        fut.add_done_callback(lambda f: self._on_goal_response(f, label))

    def _send_goal_B_once(self):
        if self.goal_B_pose is None:
            self.get_logger().warn("No goal B yet. Click Nav2 Goal in RViz to set it.")
            self.mode = Mode.WAIT_FOR_B
            return
        # refresh timestamp for current send
        self.goal_B_pose.header.stamp = self.get_clock().now().to_msg()
        self.mode = Mode.GO_TO_B
        self._send_goal(self.goal_B_pose, "B")

    def _cancel_current_goal(self):
        if self.goal_handle is None:
            return
        try:
            fut = self.goal_handle.cancel_goal_async()
            # best-effort, don't block long
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            self.get_logger().warn("Cancel requested.")
        except Exception as e:
            self.get_logger().warn(f"Cancel failed: {e}")

    # ---------------- action callbacks ----------------
    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            d = float(fb.distance_remaining)
        except Exception:
            return
        # lightweight log
        if int(time.time() * 2) % 10 == 0:  # approx every ~5s
            self.get_logger().info(f"NAV2 distance_remaining={d:.2f}")

    def _on_goal_response(self, future, label: str):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Nav2 goal rejected ({label})")
            return
        self.get_logger().info(f"Nav2 goal accepted ({label})")
        self.goal_handle = goal_handle
        res_fut = goal_handle.get_result_async()
        res_fut.add_done_callback(lambda f: self._on_result(f, label))

    def _on_result(self, future, label: str):
        status = future.result().status
        self.get_logger().warn(f"Nav2 result ({label}) status={status} (4=SUCCEEDED,6=ABORTED,5=CANCELED)")
        self.goal_handle = None

        if label != "PERSON":
            # If we intentionally canceled B to intercept a person, don't retry.
            if status == 5 and (self.mode == Mode.GO_TO_PERSON or self.intercept_done_this_trip):
                self.get_logger().info("B goal was canceled for intercept; skipping auto-retry.")
                return
            if status == 4:
                if self.waypoint_sequence:
                    wp_name, _ = self.waypoint_sequence[self.current_waypoint_index]
                    self.get_logger().info(f"Reached waypoint {wp_name}")
                    if self.current_waypoint_index >= len(self.waypoint_sequence) - 1:
                        self.get_logger().info("Waypoint sequence completed. Waiting for new goal.")
                        self.waypoint_sequence = []
                        self.current_waypoint_index = -1
                        self.mode = Mode.WAIT_FOR_B
                    else:
                        wait_wp = float(self.get_parameter("wait_at_waypoint_seconds").value)
                        if wait_wp > 0.0:
                            self.mode = Mode.WAIT_AT_WAYPOINT
                            self._wait_end_time = time.time() + wait_wp
                        else:
                            self.current_waypoint_index += 1
                            self._send_current_waypoint()
                elif bool(self.get_parameter("use_fixed_goal_B").value):
                    self.mode = Mode.DONE
                else:
                    self.mode = Mode.WAIT_FOR_B
                    self.get_logger().info("Reached B. Waiting for next goal from RViz.")
            else:
                # success-first: if B failed, just try again after short delay
                self.get_logger().warn("B failed -> retry in 1s")
                time.sleep(1.0)
                if self.waypoint_sequence:
                    self._send_current_waypoint()
                else:
                    self._send_goal_B_once()

        elif label == "PERSON":
            # no matter succeed/abort, continue mission to B
            self.mode = Mode.WAIT_10S
            self._wait_end_time = time.time() + float(self.get_parameter("wait_seconds").value)

    # ---------------- main loop ----------------
    def tick(self):
        if self.mode == Mode.DONE:
            return

        if self.mode == Mode.WAIT_FOR_B:
            return

        if self.mode == Mode.WAIT_AT_WAYPOINT:
            tw = Twist()
            self.pub_cmd_vel.publish(tw)
            if self._wait_end_time is not None and time.time() >= self._wait_end_time:
                self._wait_end_time = None
                self.current_waypoint_index += 1
                self._send_current_waypoint()
            return

        if self.mode == Mode.WAIT_10S:
            # stop robot
            tw = Twist()
            self.pub_cmd_vel.publish(tw)
            if self._wait_end_time is not None and time.time() >= self._wait_end_time:
                self.get_logger().warn("Wait done -> resume goal B")
                self._wait_end_time = None
                self._send_goal_B_once()
            return

        if self.mode != Mode.GO_TO_B:
            return

        # GO_TO_B: decide intercept once
        if self.intercept_done_this_trip:
            return

        det = self.nearest_det
        z = self.nearest_z

        if det is None:
            return
        if self.wave_candidate_key is None:
            return
        max_stale = float(self.get_parameter("wave_track_timeout_seconds").value)
        if (time.monotonic() - self.wave_candidate_seen_at) > max_stale:
            return

        zmin = float(self.get_parameter("intercept_min_range_m").value)
        zmax = float(self.get_parameter("intercept_max_range_m").value)
        if not (zmin <= z <= zmax):
            return

        if bool(self.get_parameter("require_map_tf").value):
            if self.intr is None:
                return
            map_frame = self.get_parameter("map_frame").value
            if not self.tf_buffer.can_transform(map_frame, self.intr.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.05)):
                return

        # Intercept now (success-first)
        self.get_logger().warn(
            f"INTERCEPT: wave held {self.wave_candidate_held_s:.1f}s "
            f"({self.wave_candidate_hand or 'UNKNOWN'}) z={z:.2f}m -> cancel B and approach"
        )
        self.intercept_done_this_trip = True

        # cancel current B goal and send person goal
        self._cancel_current_goal()

        try:
            goal_person = self._compute_approach_goal(det)
        except Exception as e:
            self.get_logger().warn(f"Compute person goal failed: {e}. Resume B.")
            self._send_goal_B_once()
            return

        self.mode = Mode.GO_TO_PERSON
        self._send_goal(goal_person, "PERSON")

    # ---------------- shutdown behavior ----------------
    def shutdown_cleanup(self):
        if bool(self.get_parameter("cancel_on_shutdown").value):
            self.get_logger().warn("Shutdown: canceling Nav2 goal (best-effort)")
            self._cancel_current_goal()

        if bool(self.get_parameter("stop_cmd_vel_on_shutdown").value):
            self.get_logger().warn("Shutdown: publishing cmd_vel=0 for 0.5s")
            tw = Twist()
            end = time.time() + 0.5
            while rclpy.ok() and time.time() < end:
                self.pub_cmd_vel.publish(tw)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.05)


def main():
    rclpy.init()
    node = MissionAB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_cleanup()
        except Exception:
            pass
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

