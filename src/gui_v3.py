from collections import deque
import copy
import glob
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time

import yaml
try:
    import serial
except ImportError:
    serial = None

from PyQt5.QtCore import QPoint, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QFileDialog,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
    QProgressBar, QPushButton, QRadioButton, QScrollArea, QSlider, QTableWidget, QTableWidgetItem,
    QTabWidget, QTextEdit, QVBoxLayout, QWidget,
)

import rclpy
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, Image, LaserScan
from std_msgs.msg import Bool, Float64, Int32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

try:
    from robot_interfaces.msg import TrackedPerson, TrackedPersonArray
    HAS_TRACKED_PERSON_MSG = True
except ImportError:
    HAS_TRACKED_PERSON_MSG = False

try:
    from yolo_msgs.msg import DetectionArray
    HAS_YOLO_MSG = True
except ImportError:
    HAS_YOLO_MSG = False

try:
    from robot_interfaces.msg import RobotStatus
    HAS_ROBOT_STATUS_MSG = True
except ImportError:
    HAS_ROBOT_STATUS_MSG = False

try:
    from robot_interfaces.srv import SaveLocation, SaveRoute
    HAS_CONFIG_SRVS = True
except ImportError:
    HAS_CONFIG_SRVS = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
CONFIG_DIR = os.path.join(WS_ROOT, "config")
MO_HINH_CONFIG_DIR = os.path.join(WS_ROOT, "src", "mo_hinh", "config")
MO_HINH_MAP_DIR = os.path.join(WS_ROOT, "src", "mo_hinh", "maps")
DEFAULT_MAP_DIR = os.path.join(WS_ROOT, "my_map")
MAP_SUFFIX_AMCL = "_amcl"
MAP_SUFFIX_LOCALIZATION = "_localization"
MAP_MODE_GOAL = "goal"
MAP_MODE_DRAW_OBSTACLE = "draw_obstacle"
MAP_MODE_ERASE = "erase"

# Đường cong xả 3 bình ắc quy chì Yamato 12V 30Ah ghép nối tiếp (36V)
_BATTERY_36V_CURVE = (
    (33.60, 0.0),    # 11.20V/bình (ngưỡng ngắt bảo vệ)
    (34.80, 10.0),   # 11.60V/bình
    (35.70, 25.0),   # 11.90V/bình
    (36.45, 50.0),   # 12.15V/bình
    (37.20, 75.0),   # 12.40V/bình
    (37.80, 90.0),   # 12.60V/bình
    (38.40, 100.0),  # 12.80V/bình (đầy)
)

def compute_battery_percent_36v(voltage_v: float) -> float:
    """Interpolate State-of-Charge (%) for 3x12V lead-acid batteries in series."""
    if voltage_v <= _BATTERY_36V_CURVE[0][0]:
        return 0.0
    if voltage_v >= _BATTERY_36V_CURVE[-1][0]:
        return 100.0
    for (v0, p0), (v1, p1) in zip(_BATTERY_36V_CURVE, _BATTERY_36V_CURVE[1:]):
        if voltage_v <= v1:
            return p0 + (voltage_v - v0) / (v1 - v0) * (p1 - p0)
    return 100.0

def _pick_first_existing(paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]

def _bool_to_ros(value):
    return "true" if value else "false"

def _replace_key_recursive(node, target_key, value):
    count = 0
    if isinstance(node, dict):
        for key, child in node.items():
            if key == target_key:
                node[key] = value
                count += 1
            else:
                count += _replace_key_recursive(child, target_key, value)
    elif isinstance(node, list):
        for child in node:
            count += _replace_key_recursive(child, target_key, value)
    return count

def _map_stem_from_yaml(map_yaml_path):
    return os.path.splitext(map_yaml_path)[0]

def _list_serial_ports():
    preferred = sorted(glob.glob("/dev/serial/by-id/*"))
    fallback = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    ordered = preferred + fallback
    dedup, seen_real = [], set()
    for port in ordered:
        real_port = os.path.realpath(port)
        if real_port in seen_real:
            continue
        seen_real.add(real_port)
        dedup.append(port)
    return dedup

def _is_esp32_port(port, baudrate=115200, timeout=0.1):
    if serial is None:
        return False
    ser = None
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        ser.reset_input_buffer()
        start_time = time.time()
        while time.time() - start_time < 1.0:
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

def _detect_esp_lidar_ports():
    esp_alias, lidar_alias = "/dev/esp32", "/dev/lidar"
    if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
        return esp_alias, lidar_alias, "udev_alias"
    ports = _list_serial_ports()
    if not ports:
        return "/dev/ttyUSB0", "/dev/ttyUSB1", "fallback_defaults"
    esp_port, lidar_port = None, None
    for port in ports:
        key = os.path.basename(port).lower()
        if lidar_port is None and ("rplidar" in key or "slamtec" in key or "lidar" in key):
            lidar_port = port
        if esp_port is None and ("esp32" in key or "esp" in key):
            esp_port = port
    if esp_port is None:
        for port in ports:
            if _is_esp32_port(port):
                esp_port = port
                break
    if esp_port is None:
        for port in ports:
            if port.endswith("ttyUSB0") or port.endswith("ttyACM0"):
                esp_port = port
                break
    if esp_port is None:
        esp_port = ports[0]
    if lidar_port is None:
        for port in ports:
            if port != esp_port:
                lidar_port = port
                break
    if lidar_port is None or lidar_port == esp_port:
        lidar_port = "/dev/ttyUSB1" if esp_port != "/dev/ttyUSB1" else "/dev/ttyUSB0"
    return esp_port, lidar_port, "scan"

ROS_SETUP = os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash")
WS_SETUP = os.path.join(WS_ROOT, "install", "setup.bash")
NAV2_PARAMS_BASE = os.environ.get("NAV2_PARAMS", _pick_first_existing([os.path.join(MO_HINH_CONFIG_DIR, "nav2_params.yaml"), os.path.join(CONFIG_DIR, "nav2_params.yaml")]))
SLAM_MAPPING_PARAMS_BASE = os.environ.get("SLAM_MAPPING_PARAMS", _pick_first_existing([os.path.join(MO_HINH_CONFIG_DIR, "mapper_params_online_async.yaml"), os.path.join(CONFIG_DIR, "mapper_params_online_async.yaml")]))
SLAM_LOCALIZATION_PARAMS_BASE = os.environ.get("SLAM_LOCALIZATION_PARAMS", _pick_first_existing([os.path.join(MO_HINH_CONFIG_DIR, "slam_localization.yaml"), os.path.join(CONFIG_DIR, "slam_localization.yaml")]))

def recompute_cyclic_waypoint_yaws(waypoints):
    """Set each patrol waypoint's yaw (rad) to face toward the next waypoint in the cycle."""
    n = len(waypoints)
    if n < 2:
        if n == 1:
            waypoints[0]["yaw"] = 0.0
        return waypoints
    for i, wp in enumerate(waypoints):
        nxt = waypoints[(i + 1) % n]
        dx = float(nxt.get("x", 0.0)) - float(wp.get("x", 0.0))
        dy = float(nxt.get("y", 0.0)) - float(wp.get("y", 0.0))
        wp["yaw"] = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
    return waypoints

class RosInterface(Node):
    def __init__(self, log_callback=None, map_callback=None, pose_callback=None, scan_callback=None, dataenc_callback=None, nav_feedback_callback=None, nav_status_callback=None, human_callback=None, mission_log_callback=None, popup_cmd_callback=None, obstacle_scan_callback=None, operator_alert_callback=None):
        super().__init__("desktop_nav_gui_v3")
        self.log_callback = log_callback
        self.map_callback = map_callback
        self.pose_callback = pose_callback
        self.scan_callback = scan_callback
        self.obstacle_scan_callback = obstacle_scan_callback
        self.dataenc_callback = dataenc_callback
        self.nav_feedback_callback = nav_feedback_callback
        self.nav_status_callback = nav_status_callback
        self.human_callback = human_callback
        self.mission_log_callback = mission_log_callback
        self.popup_cmd_callback = popup_cmd_callback
        self.operator_alert_callback = operator_alert_callback
        self.current_goal_x = None
        self.current_goal_y = None
        self.initial_goal_dist = None
        self.last_scan_ui_pub = 0.0
        self.last_tf_warn_time = 0.0
        self._dbg_lock = threading.Lock()
        self._topic_stats = {}
        self._last_obs_ui_pub = {"front": 0.0, "rear": 0.0}
        self.target_map_frame = "map"
        self.target_base_frame = "base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self._map_sub = self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, best_effort_qos)
        self.dataenc_sub = self.create_subscription(Int32MultiArray, "/dataenc", self.on_dataenc, 20)
        for dbg_key, topic_name in (
            ("cam_front_depth", "/front_camera/depth/image_raw"),
            ("cam_rear_depth", "/rear_camera/depth/image_raw"),
            ("cam_ai_color", "/ai_camera/color/image_raw"),
            ("cam_ai_depth", "/ai_camera/depth/image_raw"),
        ):
            # Skip CDR pixel deserialization in Python since only arrival timestamps/Hz are tracked.
            self.create_subscription(
                Image,
                topic_name,
                lambda _msg, k=dbg_key: self._mark_topic_rx(k),
                qos_profile_sensor_data,
                raw=True,
            )
        for side, topic_name in (("front", "/scan_obstacles"), ("rear", "/rear_scan_obstacle")):
            self.create_subscription(LaserScan, topic_name, lambda msg, s=side: self._on_obstacle_scan(s, msg), best_effort_qos)
        self._battery_voltages = deque(maxlen=25)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Float64, "/hoverboard/battery_voltage", self._on_hoverboard_voltage, 10)
        self.create_subscription(BatteryState, "/battery", self._on_battery_state, 10)
        if HAS_ROBOT_STATUS_MSG:
            self.create_subscription(RobotStatus, "/robot_status", self._on_robot_status, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_goal_handle = None
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cancel_nav_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.cancel_through_poses_client = self.create_client(CancelGoal, "/navigate_through_poses/_action/cancel_goal")
        self.human_detected_pub = self.create_publisher(Bool, "/human_detected_in_path", 10)
        self.human_detected_sub = self.create_subscription(Bool, "/human_detected_in_path", self.on_human_detected_msg, 10)

        # Mission Control interfaces
        self.mission_log_sub = self.create_subscription(String, "/mission/status_log", self.on_mission_log_msg, 10)
        self.popup_cmd_sub = self.create_subscription(String, "/mission/service_popup_cmd", self.on_popup_cmd_msg, 10)
        self.operator_alert_sub = self.create_subscription(KeyValue, "/navigation/operator_alert", self.on_operator_alert_msg, 10)
        self.serve_trigger_pub = self.create_publisher(Bool, "/mission/serve_trigger", 10)
        self.finish_serving_pub = self.create_publisher(Bool, "/mission/finish_serving", 10)
        self.wave_detected_pub = self.create_publisher(Bool, "/pose/wave_detected", 10)
        self.wave_status_pub = self.create_publisher(String, "/pose/wave_status", 10)
        self.create_subscription(Bool, "/pose/wave_detected", self._on_wave_detected, 10)
        self.create_subscription(String, "/pose/wave_status", self._on_wave_status, 10)
        if HAS_TRACKED_PERSON_MSG:
            self.tracked_people_pub = self.create_publisher(TrackedPersonArray, "/perception/tracked_people", 10)
            self.create_subscription(TrackedPersonArray, "/perception/tracked_people", self._on_tracked_people, 10)
        else:
            self.tracked_people_pub = None
        if HAS_YOLO_MSG:
            self.create_subscription(DetectionArray, "/yolo/tracking", self._on_yolo_tracking, 10)

        self.cli_start_patrol = self.create_client(Trigger, "/mission/start_patrol")
        self.cli_pause_patrol = self.create_client(Trigger, "/mission/pause_patrol")
        self.cli_trigger_serve = self.create_client(Trigger, "/mission/trigger_serve")
        self.cli_trigger_finish = self.create_client(Trigger, "/mission/trigger_finish")
        self.cli_reload_route = self.create_client(Trigger, "/mission/reload_route")
        if HAS_CONFIG_SRVS:
            self.cli_save_location = self.create_client(SaveLocation, "/config/save_location")
            self.cli_save_route = self.create_client(SaveRoute, "/config/save_route")
        else:
            self.cli_save_location = None
            self.cli_save_route = None

        self.pose_timer = self.create_timer(0.05, self.publish_pose_from_tf)

    def publish_mock_human(self, is_human: bool):
        msg = Bool()
        msg.data = bool(is_human)
        self.human_detected_pub.publish(msg)
        if HAS_TRACKED_PERSON_MSG and self.tracked_people_pub is not None:
            arr = TrackedPersonArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.header.frame_id = "base_footprint"
            if is_human:
                p = TrackedPerson()
                p.track_id = "mock_human_1"
                p.position.x = 1.2
                p.position.y = 0.0
                p.position.z = 0.0
                p.distance_xy_m = 1.2
                p.depth_m = 1.2
                p.position_valid = True
                p.depth_valid = True
                p.is_waving = False
                p.wave_valid = True
                p.detection_confidence = 0.99
                arr.people.append(p)
            self.tracked_people_pub.publish(arr)

    def publish_mock_waving(self, is_waving: bool):
        msg = Bool()
        msg.data = bool(is_waving)
        self.wave_detected_pub.publish(msg)
        if is_waving:
            st = String()
            st.data = "[person_99] Waving with Right hand!"
            self.wave_status_pub.publish(st)
        if HAS_TRACKED_PERSON_MSG and self.tracked_people_pub is not None:
            arr = TrackedPersonArray()
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.header.frame_id = "base_footprint"
            if is_waving:
                p = TrackedPerson()
                p.track_id = "99"
                p.position.x = 2.2
                p.position.y = 0.4
                p.position.z = 0.0
                p.distance_xy_m = math.hypot(2.2, 0.4)
                p.depth_m = 2.2
                p.position_valid = True
                p.depth_valid = True
                p.is_waving = True
                p.wave_valid = True
                p.detection_confidence = 0.99
                arr.people.append(p)
            self.tracked_people_pub.publish(arr)

    def call_mission_trigger(self, client, label: str):
        if client.service_is_ready():
            client.call_async(Trigger.Request())
            self.log(f"[Mission] Đã gửi lệnh: {label}")
            return True
        if client.wait_for_service(timeout_sec=0.3):
            client.call_async(Trigger.Request())
            self.log(f"[Mission] Đã gửi lệnh: {label}")
            return True
        self.log(f"[Mission] Lỗi: Service {label} chưa bật (Hãy chạy backend.launch.py)!")
        return False

    def sync_patrol_config(self, waypoints, route_name="patrol_route"):
        synced = False
        recompute_cyclic_waypoint_yaws(waypoints)
        if HAS_CONFIG_SRVS and self.cli_save_location is not None and self.cli_save_route is not None:
            if self.cli_save_location.wait_for_service(timeout_sec=0.2):
                for wp in waypoints:
                    req = SaveLocation.Request()
                    req.name = str(wp["name"])
                    req.x = float(wp["x"])
                    req.y = float(wp["y"])
                    req.yaw = float(wp.get("yaw", 0.0))
                    self.cli_save_location.call_async(req)
                synced = True
            if self.cli_save_route.wait_for_service(timeout_sec=0.2):
                r_req = SaveRoute.Request()
                r_req.route_name = route_name
                r_req.waypoints = [str(wp["name"]) for wp in waypoints]
                self.cli_save_route.call_async(r_req)
                synced = True
        if self.cli_reload_route.wait_for_service(timeout_sec=0.2):
            self.cli_reload_route.call_async(Trigger.Request())
            self.log("[Mission] Đã gửi tín hiệu reload lộ trình sang Mission Controller.")
        return synced

    def on_mission_log_msg(self, msg):
        if self.mission_log_callback:
            self.mission_log_callback(msg.data)

    def on_popup_cmd_msg(self, msg):
        if self.popup_cmd_callback:
            self.popup_cmd_callback(msg.data)

    def on_operator_alert_msg(self, msg):
        if self.operator_alert_callback:
            self.operator_alert_callback(str(msg.key), str(msg.value))

    def on_human_detected_msg(self, msg):
        if self.human_callback:
            self.human_callback(bool(msg.data))

    def log(self, msg):
        self.get_logger().info(msg)
        if self.log_callback:
            self.log_callback(msg)

    def on_map(self, msg):
        if self.map_callback:
            self.map_callback(msg)

    def _pose_to_xyyaw(self, qx, qy, qz, qw):
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _warn_tf(self, message):
        now = time.monotonic()
        if now - self.last_tf_warn_time < 10.0:
            return
        self.last_tf_warn_time = now
        self.get_logger().warn(message)

    def _get_active_target_frame(self):
        all_frames = self.tf_buffer.all_frames_as_string()
        if self.target_map_frame in all_frames:
            return self.target_map_frame
        if "odom" in all_frames:
            return "odom"
        return None

    def publish_pose_from_tf(self):
        if not self.pose_callback:
            return
        target_frame = self._get_active_target_frame()
        if not target_frame or not self.tf_buffer.can_transform(target_frame, self.target_base_frame, Time(), timeout=Duration(seconds=0.02)):
            return
        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, self.target_base_frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self._warn_tf(f"TF pose lookup lỗi: {exc}")
            return
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        self.pose_callback(target_frame, t.x, t.y, self._pose_to_xyyaw(q.x, q.y, q.z, q.w))

    def _scan_to_world_points(self, msg):
        ranges = msg.ranges
        if not ranges:
            return []
        target_frame = self._get_active_target_frame()
        if not target_frame:
            return []
        source_frame = msg.header.frame_id if msg.header.frame_id else self.target_base_frame
        tf_msg = None
        scan_time = Time.from_msg(msg.header.stamp)
        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, scan_time, timeout=Duration(seconds=0.03))
        except TransformException:
            try:
                tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.03))
            except TransformException:
                return []
        if tf_msg is None:
            return []
        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        step = max(1, len(ranges) // 720)
        world_points = []
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lx, ly = r * math.cos(angle), r * math.sin(angle)
            world_points.append((r00 * lx + r01 * ly + tr.x, r10 * lx + r11 * ly + tr.y))
        return world_points

    @staticmethod
    def _min_scan_range(msg):
        return min(
            (r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max),
            default=None,
        )

    def _mark_topic_rx(self, key, extra=None):
        """Record message arrival timestamp and smoothed Hz under _dbg_lock."""
        now = time.monotonic()
        with self._dbg_lock:
            entry = self._topic_stats.get(key)
            if entry is None:
                self._topic_stats[key] = {"last_ts": now, "hz": 0.0, "extra": extra}
            else:
                dt = now - entry["last_ts"]
                if dt > 0.002:
                    inst_hz = 1.0 / dt
                    prev_hz = entry["hz"]
                    entry["hz"] = inst_hz if prev_hz <= 0.0 else (0.7 * prev_hz + 0.3 * inst_hz)
                entry["last_ts"] = now
                if extra is not None:
                    entry["extra"] = extra
        return now

    def _on_obstacle_scan(self, side, msg):
        now = self._mark_topic_rx(f"obs_{side}", {"min_dist": self._min_scan_range(msg)})
        if not self.obstacle_scan_callback:
            return
        if now - self._last_obs_ui_pub.get(side, 0.0) < 0.15:
            return
        self._last_obs_ui_pub[side] = now
        self.obstacle_scan_callback(side, self._scan_to_world_points(msg))

    def _on_odom(self, msg):
        self._mark_topic_rx("odom", (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)))

    def _on_cmd_vel(self, msg):
        self._mark_topic_rx("cmd_vel", (float(msg.linear.x), float(msg.angular.z)))

    def _record_battery_voltage(self, voltage_v: float, source: str):
        if not math.isfinite(voltage_v) or voltage_v <= 1.0:
            return
        with self._dbg_lock:
            now = time.monotonic()
            hb = self._topic_stats.get("bat_hoverboard")
            if source == "/battery" and hb and (now - hb["last_ts"]) < 5.0:
                return
            if getattr(self, "_battery_source", None) != source:
                self._battery_voltages.clear()
                self._battery_source = source
            self._battery_voltages.append(voltage_v)
            smooth_v = sum(self._battery_voltages) / len(self._battery_voltages)
        key = "bat_hoverboard" if source == "/hoverboard/battery_voltage" else "bat_state"
        self._mark_topic_rx(key, {
            "raw_v": voltage_v,
            "smooth_v": smooth_v,
            "percent": compute_battery_percent_36v(smooth_v),
            "source": source,
        })

    def _on_hoverboard_voltage(self, msg):
        self._record_battery_voltage(float(msg.data), "/hoverboard/battery_voltage")

    def _on_battery_state(self, msg):
        self._record_battery_voltage(float(msg.voltage), "/battery")

    def _on_robot_status(self, msg):
        self._mark_topic_rx("robot_status", {
            "battery_percent": float(msg.battery_percent),
            "water_liter": float(msg.water_liter),
            "lidar_ok": bool(msg.lidar_ok),
            "odom_ok": bool(msg.odom_ok),
        })

    def _on_wave_detected(self, msg):
        self._mark_topic_rx("wave_detected", bool(msg.data))

    def _on_wave_status(self, msg):
        self._mark_topic_rx("wave_status", str(msg.data))

    def _on_yolo_tracking(self, msg):
        dets = getattr(msg, "detections", ()) or ()
        persons = sum(
            1 for d in dets
            if str(getattr(d, "class_name", "")).lower() == "person" or getattr(d, "class_id", -1) == 0
        )
        self._mark_topic_rx("yolo_tracking", {"total": len(dets), "persons": persons})

    def _on_tracked_people(self, msg):
        people = [
            {
                "track_id": str(p.track_id or "?"),
                "dist_xy": float(p.distance_xy_m),
                "depth_m": float(p.depth_m),
                "pos_valid": bool(p.position_valid),
                "depth_valid": bool(p.depth_valid),
                "is_waving": bool(p.is_waving),
                "wave_valid": bool(p.wave_valid),
                "conf": float(p.detection_confidence),
                "x": float(p.position.x),
                "y": float(p.position.y),
            }
            for p in (getattr(msg, "people", ()) or ())
        ]
        people.sort(key=lambda item: item["dist_xy"] if item["dist_xy"] > 0.0 else 999.0)
        self._mark_topic_rx("tracked_people", {
            "frame_id": str(msg.header.frame_id or "base"),
            "calibrated": bool(msg.geometry_calibrated),
            "people": people,
        })

    def get_debug_snapshot(self):
        """Return thread-safe sensor telemetry snapshot and TF tree availability."""
        with self._dbg_lock:
            stats_copy = {k: dict(v) for k, v in self._topic_stats.items()}
        try:
            odom_tf_ok = bool(self.tf_buffer.can_transform("odom", self.target_base_frame, Time(), timeout=Duration(seconds=0.0)))
        except Exception:
            odom_tf_ok = False
        try:
            map_tf_ok = bool(self.tf_buffer.can_transform(self.target_map_frame, "odom", Time(), timeout=Duration(seconds=0.0)))
        except Exception:
            map_tf_ok = False
        return {
            "now": time.monotonic(),
            "stats": stats_copy,
            "odom_tf_ok": odom_tf_ok,
            "map_tf_ok": map_tf_ok,
        }

    def on_scan(self, msg):
        now = self._mark_topic_rx(
            "lidar_scan",
            {"min_dist": self._min_scan_range(msg), "count": len(msg.ranges)},
        )
        if not self.scan_callback:
            return
        if now - self.last_scan_ui_pub < 0.1:
            return
        self.last_scan_ui_pub = now
        self.scan_callback(msg, self._scan_to_world_points(msg))

    def on_dataenc(self, msg):
        if self.dataenc_callback:
            self.dataenc_callback(list(msg.data))

    def emit_nav_status(self, title, desc, color="#3b82f6"):
        if self.nav_status_callback:
            self.nav_status_callback(title, desc, color)

    def _on_nav_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = float(fb.distance_remaining)
        recov = int(fb.number_of_recoveries)
        nav_sec = float(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9)
        est_sec = float(fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec * 1e-9)
        if self.initial_goal_dist is None or dist > self.initial_goal_dist:
            self.initial_goal_dist = dist
        if self.nav_feedback_callback:
            self.nav_feedback_callback(dist, self.initial_goal_dist, nav_sec, est_sec, recov)

    def _on_goal_result(self, fut):
        try:
            status = fut.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.log("🏁 Đã đến đích thành công!")
                self.emit_nav_status("🏁 ĐÃ ĐẾN ĐÍCH THÀNH CÔNG!", "Robot đã tới đúng vị trí mục tiêu và phanh dừng an toàn.", "#10b981")
            elif status == GoalStatus.STATUS_CANCELED:
                self.log("⛔ Mục tiêu đã bị hủy.")
                self.emit_nav_status("⛔ ĐÃ HỦY MỤC TIÊU", "Đã phanh xe và dừng toàn bộ nhiệm vụ điều hướng theo yêu cầu.", "#6b7280")
            elif status == GoalStatus.STATUS_ABORTED:
                self.log("❌ Điều hướng bị hủy bỏ (Aborted).")
                self.emit_nav_status("❌ BẾ TẮC / HỦY BỎ", "Robot không thể đến đích vì bị kẹt hoặc đường đi bị vật cản chặn hoàn toàn.", "#ef4444")
        except Exception as e:
            self.log(f"Lỗi khi nhận kết quả goal: {e}")

    def send_goal(self, x, y, yaw=0.0):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.log("NavigateToPose action server chưa sẵn sàng.")
            self.emit_nav_status("⚠️ SERVER CHƯA SẴN SÀNG", "NavigateToPose action server chưa phản hồi. Hãy kiểm tra xem Nav2 đã Start chưa.", "#ef4444")
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.current_goal_x = float(x)
        self.current_goal_y = float(y)
        self.initial_goal_dist = None
        self.log(f"Gửi goal Nav2: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        self.emit_nav_status("🎯 ĐÃ GỬI MỤC TIÊU MỚI", f"Mục tiêu ({x:.2f}, {y:.2f}). Đang gửi tới Nav2 planner để tính toán đường đi...", "#3b82f6")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self._on_nav_feedback)

        def _on_goal_response(fut):
            try:
                self.current_goal_handle = fut.result()
                if not self.current_goal_handle.accepted:
                    self.log("Goal bị từ chối bởi Nav2 server.")
                    self.emit_nav_status("❌ MỤC TIÊU BỊ TỪ CHỐI", "Nav2 server từ chối mục tiêu. Điểm đến có thể rơi vào vùng cấm hoặc ngoài bản đồ.", "#ef4444")
                    self.current_goal_handle = None
                else:
                    self.log("Goal đã được Nav2 chấp nhận.")
                    self.emit_nav_status("🚗 ĐANG BÁM LỘ TRÌNH MPPI", "Nav2 đã chấp nhận mục tiêu. Đang tính toán quỹ đạo điều khiển MPPI để di chuyển.", "#16a34a")
                    result_future = self.current_goal_handle.get_result_async()
                    result_future.add_done_callback(self._on_goal_result)
            except Exception as e:
                self.log(f"Lỗi khi gửi goal Nav2: {e}")

        send_goal_future.add_done_callback(_on_goal_response)
        return True

    def cancel_nav_goal(self):
        """Hủy ngay lập tức mục tiêu Nav2 đang chạy và phanh dừng robot"""
        self.log("⛔ [CANCEL GOAL] Đang gửi lệnh dừng khẩn cấp và hủy mục tiêu Nav2...")
        self.emit_nav_status("⛔ ĐÃ PHANH DỪNG / HỦY GOAL", "Đã phanh xe khẩn cấp và gửi lệnh hủy mục tiêu tới Nav2.", "#ef4444")
        # 1. Phát liên tục vận tốc 0.0 vào /cmd_vel để phanh dừng bánh lập tức
        stop_twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.01)

        # 2. Hủy goal qua action handle nếu có
        if hasattr(self, 'current_goal_handle') and self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.log(f"Lỗi cancel goal handle: {e}")
            self.current_goal_handle = None

        # 3. Hủy toàn bộ goal qua service cancel_goal của Action Server
        req = CancelGoal.Request()
        if self.cancel_nav_client.service_is_ready():
            self.cancel_nav_client.call_async(req)
        elif self.cancel_nav_client.wait_for_service(timeout_sec=0.2):
            self.cancel_nav_client.call_async(req)

        if hasattr(self, 'cancel_through_poses_client'):
            if self.cancel_through_poses_client.service_is_ready():
                self.cancel_through_poses_client.call_async(req)
            elif self.cancel_through_poses_client.wait_for_service(timeout_sec=0.2):
                self.cancel_through_poses_client.call_async(req)

        # 4. Tạm dừng tuần tra Behavior Tree nếu Mission Controller đang chạy
        if hasattr(self, 'cli_pause_patrol') and self.cli_pause_patrol is not None:
            req_pause = Trigger.Request()
            if self.cli_pause_patrol.service_is_ready():
                self.cli_pause_patrol.call_async(req_pause)
            elif self.cli_pause_patrol.wait_for_service(timeout_sec=0.2):
                self.cli_pause_patrol.call_async(req_pause)

        self.log("✅ Đã phanh xe, gửi lệnh CancelGoal tới Nav2 và Tạm dừng tuần tra.")
        return True


class MapViewer(QLabel):
    def __init__(self, ros_node_getter, on_map_click=None, on_map_drag=None, parent=None):
        super().__init__(parent)
        self.get_ros_node = ros_node_getter
        self.on_map_click = on_map_click
        self.on_map_drag = on_map_drag
        self.setMinimumSize(220, 180)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #555;")
        self.setText("Chưa có dữ liệu /map")
        self.setMouseTracking(True)
        self.map_pixmap = None
        self.map_resolution = None
        self.map_origin_x = self.map_origin_y = 0.0
        self.map_width = self.map_height = 0
        self.scale_factor = 1.0
        self.offset_x = self.offset_y = 0
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._is_panning = False
        self._last_pan_pos = None
        self.robot_x = self.robot_y = None
        self.robot_yaw = 0.0
        self.pose_source = "none"
        self.latest_scan = None
        self.scan_points_world = []
        self.scan_points_from_tf = False
        self.front_obs_points_world = []
        self.rear_obs_points_world = []
        self.batch_waypoints = []
        self.is_dragging = False

    def reset_view(self):
        """Reset map zoom and pan back to fit-to-window view."""
        self.zoom_factor = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.update_display()

    def _compute_view_transform(self, label_w, label_h, zoom_factor=None, pan_x=None, pan_y=None):
        """Compute effective scale factor and top-left offset for the current zoom and pan."""
        zf = self.zoom_factor if zoom_factor is None else zoom_factor
        px = self.pan_x if pan_x is None else pan_x
        py = self.pan_y if pan_y is None else pan_y
        fit_scale = min(label_w / max(1, self.map_width), label_h / max(1, self.map_height))
        scale = fit_scale * zf
        ox = (label_w - self.map_width * scale) / 2.0 + px
        oy = (label_h - self.map_height * scale) / 2.0 + py
        return fit_scale, scale, ox, oy

    def update_map(self, occ_grid):
        info = occ_grid.info
        self.map_resolution, self.map_origin_x, self.map_origin_y = info.resolution, info.origin.position.x, info.origin.position.y
        self.map_width, self.map_height = info.width, info.height
        img = QImage(self.map_width, self.map_height, QImage.Format_RGB888)
        data = occ_grid.data
        for y in range(self.map_height):
            for x in range(self.map_width):
                i = x + (self.map_height - 1 - y) * self.map_width
                val = data[i]
                c = 127 if val == -1 else max(0, min(255, int(255 - (val * 255 / 100))))
                img.setPixel(x, y, QColor(c, c, c).rgb())
        self.map_pixmap = QPixmap.fromImage(img)
        self.update_display()

    def update_edited_map(self, edited_data, width, height):
        self.map_width, self.map_height = width, height
        img = QImage(width, height, QImage.Format_RGB888)
        for y in range(height):
            for x in range(width):
                i = x + (height - 1 - y) * width
                val = edited_data[i]
                c = 127 if val == -1 else max(0, min(255, int(255 - (val * 255 / 100))))
                img.setPixel(x, y, QColor(c, c, c).rgb())
        self.map_pixmap = QPixmap.fromImage(img)
        self.update_display()

    def update_pose(self, x, y, yaw, source):
        self.robot_x, self.robot_y, self.robot_yaw, self.pose_source = x, y, yaw, source
        if not self.scan_points_from_tf:
            self._recompute_scan_points()
        self.update_display()

    def update_scan(self, scan_msg, scan_points_world=None):
        self.latest_scan = scan_msg
        if not scan_points_world:
            self.scan_points_from_tf = False
            self._recompute_scan_points()
        else:
            self.scan_points_from_tf = True
            self.scan_points_world = list(scan_points_world)
        self.update_display()

    def update_waypoint_batch(self, waypoints):
        self.batch_waypoints = list(waypoints)
        self.update_display()

    def update_obstacle_scan(self, side, points_world):
        """Update cached world-frame obstacle scan points for 'front' or 'rear' camera."""
        pts = list(points_world) if points_world else []
        if side == "front":
            self.front_obs_points_world = pts
        elif side == "rear":
            self.rear_obs_points_world = pts
        self.update_display()

    def _draw_world_points(self, painter, points, color, width, label_w, label_h):
        """Draw world-frame (wx, wy) points onto the map canvas."""
        if not points:
            return
        painter.setPen(QPen(color, width))
        for wx, wy in points:
            sx, sy = self.world_to_widget(wx, wy)
            if sx is not None and 0 <= sx < label_w and 0 <= sy < label_h:
                painter.drawPoint(sx, sy)

    def _recompute_scan_points(self):
        self.scan_points_world = []
        if self.latest_scan is None or self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            return
        ranges = self.latest_scan.ranges
        if not ranges:
            return
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        # laser_link offset in base_footprint coordinate: (+0.175, 0.0)
        lx_base = 0.175
        ly_base = 0.0
        laser_origin_x = self.robot_x + cos_yaw * lx_base - sin_yaw * ly_base
        laser_origin_y = self.robot_y + sin_yaw * lx_base + cos_yaw * ly_base

        step = max(1, len(ranges) // 720)
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < self.latest_scan.range_min or r > self.latest_scan.range_max:
                continue
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
            beam = self.robot_yaw - angle
            self.scan_points_world.append((laser_origin_x + r * math.cos(beam), laser_origin_y + r * math.sin(beam)))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_display()

    def update_display(self):
        if self.map_pixmap is None:
            return
        label_w, label_h = max(1, self.width()), max(1, self.height())
        _, self.scale_factor, self.offset_x, self.offset_y = self._compute_view_transform(label_w, label_h)
        scaled_w = self.map_width * self.scale_factor
        scaled_h = self.map_height * self.scale_factor
        canvas = QPixmap(label_w, label_h)
        canvas.fill(QColor("#1e1e1e"))
        painter = QPainter(canvas)
        painter.drawPixmap(
            QRectF(self.offset_x, self.offset_y, scaled_w, scaled_h),
            self.map_pixmap,
            QRectF(0, 0, self.map_width, self.map_height),
        )
        self._draw_world_points(painter, self.scan_points_world, QColor(0, 255, 255), 1, label_w, label_h)
        self._draw_world_points(painter, self.front_obs_points_world, QColor(245, 158, 11), 3, label_w, label_h)
        self._draw_world_points(painter, self.rear_obs_points_world, QColor(232, 121, 249), 3, label_w, label_h)
        if self.batch_waypoints:
            painter.setFont(QFont("Arial", 10))
            for i, wp in enumerate(self.batch_waypoints):
                sx, sy = self.world_to_widget(wp["x"], wp["y"])
                if sx is None:
                    continue
                color = QColor(255, 210, 0) if i < len(self.batch_waypoints) - 1 else QColor(255, 120, 0)
                painter.setPen(QPen(color, 2))
                painter.setBrush(color)
                painter.drawEllipse(QPoint(sx, sy), 5, 5)
                wyaw = float(wp.get("yaw", 0.0))
                painter.drawLine(sx, sy, sx + int(14 * math.cos(wyaw)), sy - int(14 * math.sin(wyaw)))
                painter.drawText(sx + 8, sy - 8, wp["name"])
                if i > 0:
                    px, py = self.world_to_widget(self.batch_waypoints[i - 1]["x"], self.batch_waypoints[i - 1]["y"])
                    if px is not None:
                        painter.drawLine(px, py, sx, sy)
        if self.robot_x is not None and self.robot_y is not None:
            px, py = self.world_to_widget(self.robot_x, self.robot_y)
            if px is not None:
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                painter.setBrush(QColor(255, 0, 0))
                painter.drawEllipse(QPoint(px, py), 6, 6)
                ax = px + int(18 * math.cos(self.robot_yaw))
                ay = py - int(18 * math.sin(self.robot_yaw))
                painter.drawLine(px, py, ax, ay)
        painter.setPen(QPen(QColor(0, 255, 0), 1))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(10, 20, f"Click trái: Goal/Point | Cuộn: Zoom ({self.zoom_factor:.1f}x) | Kéo chuột phải: Di chuyển (Đúp: Reset)")
        painter.drawText(10, 38, f"Pose source: {self.pose_source}")
        painter.drawText(10, 56, f"/scan: {len(self.scan_points_world)} | FrontObs: {len(self.front_obs_points_world)} | RearObs: {len(self.rear_obs_points_world)}")
        painter.end()
        self.setPixmap(canvas)

    def wheelEvent(self, event):
        if self.map_pixmap is None or self.map_width <= 0 or self.map_height <= 0:
            super().wheelEvent(event)
            return
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.2 if delta > 0 else (1.0 / 1.2)
        new_zoom = max(0.5, min(15.0, self.zoom_factor * factor))
        if abs(new_zoom - self.zoom_factor) < 1e-6:
            return
        label_w, label_h = max(1, self.width()), max(1, self.height())
        _, old_scale, old_ox, old_oy = self._compute_view_transform(label_w, label_h)
        mx, my = event.pos().x(), event.pos().y()
        map_px = (mx - old_ox) / old_scale
        map_py = (my - old_oy) / old_scale
        fit_scale, new_scale, _, _ = self._compute_view_transform(label_w, label_h, zoom_factor=new_zoom, pan_x=0.0, pan_y=0.0)
        del fit_scale
        self.pan_x = (mx - map_px * new_scale) - (label_w - self.map_width * new_scale) / 2.0
        self.pan_y = (my - map_py * new_scale) - (label_h - self.map_height * new_scale) / 2.0
        self.zoom_factor = new_zoom
        self.update_display()
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if event.button() in (Qt.RightButton, Qt.MiddleButton) and self.map_pixmap is not None:
            self.reset_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event):
        if event.button() in (Qt.RightButton, Qt.MiddleButton) and self.map_pixmap is not None:
            self._is_panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        if event.button() == Qt.LeftButton and self.map_pixmap is not None:
            world = self.widget_to_world(event.pos().x(), event.pos().y())
            if world is None:
                return
            wx, wy = world
            self.is_dragging = True
            if self.on_map_click is not None and self.on_map_click(wx, wy):
                return
            ros_node = self.get_ros_node()
            if ros_node is not None and ros_node.send_goal(wx, wy):
                ros_node.log(f"Đã click goal trên map: ({wx:.3f}, {wy:.3f})")

    def mouseMoveEvent(self, event):
        if self._is_panning and self._last_pan_pos is not None:
            delta = event.pos() - self._last_pan_pos
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self._last_pan_pos = event.pos()
            self.update_display()
            event.accept()
            return
        if self.is_dragging and self.on_map_drag is not None:
            world = self.widget_to_world(event.pos().x(), event.pos().y())
            if world is not None:
                self.on_map_drag(world[0], world[1])
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() in (Qt.RightButton, Qt.MiddleButton) and self._is_panning:
            self._is_panning = False
            self._last_pan_pos = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        if event.button() == Qt.LeftButton:
            if self.is_dragging and self.on_map_drag is not None:
                self.on_map_drag(None, None)
            self.is_dragging = False
        super().mouseReleaseEvent(event)

    def widget_to_world(self, px, py):
        if self.map_pixmap is None:
            return None
        x_in, y_in = px - self.offset_x, py - self.offset_y
        scaled_w, scaled_h = self.map_width * self.scale_factor, self.map_height * self.scale_factor
        if x_in < 0 or y_in < 0 or x_in >= scaled_w or y_in >= scaled_h:
            return None
        mx = x_in / self.scale_factor
        my = self.map_height - 1 - y_in / self.scale_factor
        return self.map_origin_x + mx * self.map_resolution, self.map_origin_y + my * self.map_resolution

    def world_to_widget(self, wx, wy):
        if self.map_pixmap is None:
            return None, None
        mx = (wx - self.map_origin_x) / self.map_resolution
        my = (wy - self.map_origin_y) / self.map_resolution
        return int(self.offset_x + mx * self.scale_factor), int(self.offset_y + (self.map_height - 1 - my) * self.scale_factor)

class ProcessManager:
    def __init__(self, log_callback):
        self.log_callback = log_callback
        self.processes = {}

    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)

    def start(self, name, cmd):
        if name in self.processes and self.is_running(name):
            self.log(f"[{name}] đang chạy rồi.")
            return
        self.log(f"[{name}] START\nCMD: {cmd}")
        proc = subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
        self.processes[name] = proc
        threading.Thread(target=self._read_output, args=(name, proc), daemon=True).start()

    def _read_output(self, name, proc):
        try:
            for line in proc.stdout:
                self.log(f"[{name}] {line.rstrip()}")
        except Exception as exc:
            self.log(f"[{name}] read output error: {exc}")

    def _terminate_groups(self, entries):
        """Gracefully stop process groups with SIGINT first so ros2 launch shuts down camera streams cleanly."""
        pending = []
        for name, proc in entries:
            if not proc or proc.poll() is not None:
                continue
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGINT)
                pending.append((name, proc, pgid))
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.log(f"[{name}] stop error: {exc}")

        if not pending:
            return

        for sig, timeout_s, label in (
            (None, 2.0, None),
            (signal.SIGTERM, 0.5, "SIGTERM"),
        ):
            still_alive = []
            for name, proc, pgid in pending:
                if proc.poll() is not None:
                    continue
                if sig is not None:
                    try:
                        os.killpg(pgid, sig)
                        self.log(f"[{name}] {label}")
                    except ProcessLookupError:
                        continue
                    except Exception as exc:
                        self.log(f"[{name}] stop error: {exc}")
                still_alive.append((name, proc, pgid))
            pending = still_alive
            if not pending:
                return

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if all(proc.poll() is not None for _, proc, _ in pending):
                    return
                time.sleep(0.05)

        for name, proc, pgid in pending:
            if proc.poll() is None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    self.log(f"[{name}] FORCE KILL (SIGKILL)")
                except ProcessLookupError:
                    pass
                except Exception as exc:
                    self.log(f"[{name}] stop error: {exc}")

    def stop(self, name):
        proc = self.processes.get(name)
        if not proc:
            self.log(f"[{name}] chưa chạy.")
            return
        if proc.poll() is None:
            self.log(f"[{name}] STOP")
            self._terminate_groups([(name, proc)])
        else:
            self.log(f"[{name}] đã dừng.")

    def is_running(self, name):
        proc = self.processes.get(name)
        return bool(proc and proc.poll() is None)

    def stop_all(self):
        active = [
            (name, proc)
            for name, proc in list(self.processes.items())
            if proc and proc.poll() is None
        ]
        if not active:
            return
        for name, _ in active:
            self.log(f"[{name}] STOP (SIGINT)")
        self._terminate_groups(active)

class MainWindow(QWidget):
    log_signal = pyqtSignal(str)
    map_signal = pyqtSignal(object)
    pose_signal = pyqtSignal(float, float, float, str)
    scan_signal = pyqtSignal(object, object)
    obstacle_scan_signal = pyqtSignal(str, object)
    dataenc_signal = pyqtSignal(object)
    nav_feedback_signal = pyqtSignal(float, float, float, float, int)
    nav_thought_signal = pyqtSignal(str, str, str)
    human_signal = pyqtSignal(bool)
    mission_log_signal = pyqtSignal(str)
    mission_popup_signal = pyqtSignal(str)
    operator_alert_signal = pyqtSignal(str, str)
    LAYER1_REAL = "layer1_real"
    SLAM_MAPPING, SLAM_LOCALIZATION = "slam_mapping", "slam_localization"
    AMCL, NAV2 = "amcl", "nav2"
    CAMERAS = "cameras"
    OBSTACLE_DEBUGGER = "obstacle_debugger"
    AI_PERCEPTION, BACKEND, KIOSK_UI = "ai_perception", "backend", "kiosk_ui"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control GUI v3 - Autonomous Patrol & Map Console")
        self._fit_window_to_screen()
        self.temp_files = []
        self.map_dir = _pick_first_existing([DEFAULT_MAP_DIR, MO_HINH_MAP_DIR, WS_ROOT])
        self.selected_map_file = ""
        self.batch_waypoint_mode = False
        self.patrol_waypoint_mode = False
        self.batch_waypoints = []
        self.is_shutting_down = False
        self.one_shot_procs = []
        self.one_shot_lock = threading.Lock()
        self.map_mode = MAP_MODE_GOAL
        self.proc_mgr = ProcessManager(self.append_log)
        self.last_thought_title = ""
        self.last_thought_time = 0.0
        # Tự động dọn dẹp các tiến trình ROS2 chạy ngầm từ trước
        self.force_kill_known_ros_processes()
        self.ros_node = None
        self.ros_spin_thread = None
        self.map_editor = None
        self.log_signal.connect(self._append_log_ui)
        self.map_signal.connect(self._on_new_map_ui)
        self.pose_signal.connect(self._on_new_pose_ui)
        self.scan_signal.connect(self._on_new_scan_ui)
        self.obstacle_scan_signal.connect(self._on_new_obstacle_scan_ui)
        self.dataenc_signal.connect(self._on_new_dataenc_ui)
        self.nav_feedback_signal.connect(self._on_nav_feedback_ui)
        self.nav_thought_signal.connect(self._on_nav_thought_ui)
        self.human_signal.connect(self._on_human_detected_ui)
        self.mission_log_signal.connect(self._on_mission_log_ui)
        self.mission_popup_signal.connect(self._on_mission_popup_ui)
        self.operator_alert_signal.connect(self._on_operator_alert_ui)
        self.init_ui()
        self.load_patrol_config_from_yaml(silent=True)
        self.append_log("Đã tự động quét và dọn dẹp sạch sẽ các tiến trình ROS2 cũ trước khi khởi động.")
        self.init_ros()
        self.refresh_map_list()
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(500)
        self.mock_timer = QTimer(self)
        self.mock_timer.timeout.connect(self._on_mock_timer_tick)
        self.mock_timer.start(400)

    def _fit_window_to_screen(self):
        """Clamp initial window size and position to fit smaller displays (e.g. Jetson VNC)."""
        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1024, 680)
            return
        avail = screen.availableGeometry()
        w = min(1320, max(640, int(avail.width() * 0.94)))
        h = min(820, max(480, int(avail.height() * 0.92)))
        self.resize(w, h)
        x = avail.x() + max(0, (avail.width() - w) // 2)
        y = avail.y() + max(0, (avail.height() - h) // 2)
        self.move(x, y)

    def init_ui(self):
        self.lbl_robot_mode_val = QLabel("Robot thật (Jetson Orin Nano)")
        self.lbl_robot_mode_val.setStyleSheet("font-weight: bold; color: #1d4ed8;")
        self.cmb_odom_source = QComboBox()
        self.cmb_odom_source.addItem("Hoverboard ros2_control", "hoverboard")
        self.cmb_odom_source.addItem("ESP encoder odom", "esp")
        self.cmb_odom_source.addItem("LiDAR RF2O odom", "rf2o")
        self.cmb_odom_source.currentIndexChanged.connect(self.on_odom_source_changed)
        self.cmb_operation_mode = QComboBox()
        self.cmb_operation_mode.addItems(["Localization", "SLAM Mapping"])
        self.cmb_operation_mode.currentIndexChanged.connect(self.on_operation_mode_changed)
        self.cmb_localization_backend = QComboBox()
        self.cmb_localization_backend.addItems(["AMCL", "SLAM Toolbox"])
        self.chk_auto_nav2 = QCheckBox("Tự start Nav2 khi Start All")
        self.chk_auto_nav2.setChecked(False)
        self.chk_auto_nav2.setEnabled(False)
        self.on_operation_mode_changed()
        system_group = QGroupBox("Cấu hình hệ thống")
        system_layout = QGridLayout()
        system_layout.addWidget(QLabel("Robot mode:"), 0, 0); system_layout.addWidget(self.lbl_robot_mode_val, 0, 1, 1, 2)
        system_layout.addWidget(QLabel("Odom source:"), 1, 0); system_layout.addWidget(self.cmb_odom_source, 1, 1, 1, 2)
        system_layout.addWidget(QLabel("Operation mode:"), 2, 0); system_layout.addWidget(self.cmb_operation_mode, 2, 1, 1, 2)
        system_layout.addWidget(QLabel("Localization backend:"), 3, 0); system_layout.addWidget(self.cmb_localization_backend, 3, 1, 1, 2)
        system_layout.addWidget(self.chk_auto_nav2, 4, 0, 1, 3)

        self.cmb_esp_port = QComboBox(); self.cmb_esp_port.setEditable(True)
        self.cmb_esp_port.setToolTip("Cổng Serial ESP32 / Hoverboard (ví dụ: /dev/ttyUSB0 hoặc /dev/esp32)")
        self.cmb_lidar_port = QComboBox(); self.cmb_lidar_port.setEditable(True)
        self.cmb_lidar_port.setToolTip("Cổng Serial LiDAR (ví dụ: /dev/ttyUSB1 hoặc /dev/lidar)")
        self.btn_refresh_ports = QPushButton("🔄 Quét cổng"); self.btn_refresh_ports.clicked.connect(self.refresh_serial_ports)
        self.btn_swap_ports = QPushButton("⇄ Đổi ESP ↔ LiDAR"); self.btn_swap_ports.clicked.connect(self.swap_serial_ports)

        system_layout.addWidget(QLabel("Cổng ESP32:"), 5, 0); system_layout.addWidget(self.cmb_esp_port, 5, 1, 1, 2)
        system_layout.addWidget(QLabel("Cổng LiDAR:"), 6, 0); system_layout.addWidget(self.cmb_lidar_port, 6, 1, 1, 2)
        port_btns = QHBoxLayout(); port_btns.addWidget(self.btn_refresh_ports); port_btns.addWidget(self.btn_swap_ports)
        system_layout.addLayout(port_btns, 7, 0, 1, 3)

        self.cmb_yolo_device = QComboBox()
        self.cmb_yolo_device.addItems(["cuda:0", "cpu"])
        self.cmb_yolo_device.setToolTip("Thiết bị chạy YOLO tracking (mặc định cuda:0 trên Jetson Orin Nano)")
        self.cmb_pose_device = QComboBox()
        self.cmb_pose_device.addItems(["cpu", "cuda:0"])
        self.cmb_pose_device.setToolTip("Thiết bị chạy HandWave RTMPose (mặc định cpu để tránh nghẽn VRAM với YOLO)")
        self.chk_ai_cameras = QCheckBox("Bật 3 camera Astra cùng AI (three_astra)")
        self.chk_ai_cameras.setChecked(True)
        self.chk_obstacle_detectors = QCheckBox("Bật detect vật cản 2 cam (/scan_obstacles)")
        self.chk_obstacle_detectors.setChecked(True)
        self.chk_obstacle_detectors.setToolTip("Chạy depth_obstacle_detector cho cam trước và cam sau khi bật Cam hoặc AI")
        system_layout.addWidget(QLabel("AI YOLO / Pose:"), 8, 0)
        system_layout.addWidget(self.cmb_yolo_device, 8, 1)
        system_layout.addWidget(self.cmb_pose_device, 8, 2)
        system_layout.addWidget(self.chk_ai_cameras, 9, 0, 1, 3)
        system_layout.addWidget(self.chk_obstacle_detectors, 10, 0, 1, 3)
        system_group.setLayout(system_layout)
        self.lbl_map_folder = QLabel("Map folder: "); self.lbl_map_folder.setWordWrap(True)
        self.cmb_map_files = QComboBox(); self.cmb_map_files.currentIndexChanged.connect(self.on_selected_map_changed)
        self.btn_choose_map_folder = QPushButton("Chọn folder map"); self.btn_choose_map_folder.clicked.connect(self.choose_map_folder)
        self.btn_refresh_maps = QPushButton("Refresh map list"); self.btn_refresh_maps.clicked.connect(self.refresh_map_list)
        self.btn_choose_map_file = QPushButton("Chọn map YAML thủ công"); self.btn_choose_map_file.clicked.connect(self.choose_map_file)
        self.lbl_selected_map = QLabel("Map đang dùng: chưa chọn"); self.lbl_selected_map.setWordWrap(True)
        self.edt_save_map_name = QLineEdit("my_map/new_map")
        self.btn_save_map = QPushButton("Lưu map (Serialize)"); self.btn_save_map.clicked.connect(self.save_map)
        self.btn_batch_mode = QPushButton("Waypoint Mode: OFF"); self.btn_batch_mode.clicked.connect(self.toggle_batch_waypoint_mode)
        self.btn_batch_clear = QPushButton("Clear Điểm"); self.btn_batch_clear.clicked.connect(self.clear_batch_waypoints)
        self.btn_batch_undo = QPushButton("Undo"); self.btn_batch_undo.clicked.connect(self.undo_batch_waypoint)
        self.btn_batch_send = QPushButton("🚀 Lưu & Mở Tab Mission"); self.btn_batch_send.clicked.connect(self.open_mission_tab_from_batch)
        self.lbl_batch_info = QLabel("Patrol Route: 0 điểm"); self.lbl_batch_info.setWordWrap(True)
        map_group = QGroupBox("Quản lý map"); map_layout = QGridLayout()
        map_layout.addWidget(self.lbl_map_folder, 0, 0, 1, 3)
        map_layout.addWidget(self.btn_choose_map_folder, 1, 0); map_layout.addWidget(self.btn_refresh_maps, 1, 1); map_layout.addWidget(self.btn_choose_map_file, 1, 2)
        map_layout.addWidget(self.cmb_map_files, 2, 0, 1, 3); map_layout.addWidget(self.lbl_selected_map, 3, 0, 1, 3)
        map_layout.addWidget(QLabel("Tên file save (không cần .yaml):"), 4, 0, 1, 3)
        map_layout.addWidget(self.edt_save_map_name, 5, 0, 1, 2); map_layout.addWidget(self.btn_save_map, 5, 2)
        map_layout.addWidget(self.btn_batch_mode, 6, 0); map_layout.addWidget(self.btn_batch_undo, 6, 1); map_layout.addWidget(self.btn_batch_clear, 6, 2)
        map_layout.addWidget(self.btn_batch_send, 7, 0, 1, 3); map_layout.addWidget(self.lbl_batch_info, 8, 0, 1, 3)
        map_group.setLayout(map_layout)
        self.btn_start_platform = QPushButton("Start Layer 1 (Odom)"); self.btn_stop_platform = QPushButton("Stop Layer 1 (Odom)")
        self.btn_start_mode = QPushButton("Start Layer 2 (Mode)"); self.btn_stop_mode = QPushButton("Stop Layer 2 (Mode)")
        self.btn_switch_mode = QPushButton("Switch Layer 2 Mode")
        self.btn_start_nav2 = QPushButton("Start Nav2"); self.btn_stop_nav2 = QPushButton("Stop Nav2")
        self.btn_start_cameras = QPushButton("Start Cam (3 Cam + Vật cản)"); self.btn_stop_cameras = QPushButton("Stop Cam")
        self.btn_start_ai = QPushButton("Start AI (Cam + YOLO)"); self.btn_stop_ai = QPushButton("Stop AI")
        self.btn_start_backend = QPushButton("Start Backend (Mission)"); self.btn_stop_backend = QPushButton("Stop Backend")
        self.btn_start_kiosk = QPushButton("Start Kiosk UI"); self.btn_stop_kiosk = QPushButton("Stop Kiosk UI")
        self.btn_cancel_nav2 = QPushButton("⛔ DỪNG KHẨN CẤP / HỦY GOAL")
        self.btn_cancel_nav2.setStyleSheet("background-color: #dc2626; color: white; font-weight: bold; padding: 7px; border-radius: 4px;")
        self.btn_cancel_nav2.setToolTip("Hủy mục tiêu Nav2 lập tức và phanh robot đứng im tại chỗ (không tắt Nav2)")
        self.btn_start_all = QPushButton("Start All"); self.btn_stop_all = QPushButton("Stop All")
        self.edt_kill_node = QLineEdit("rviz2"); self.btn_kill_node = QPushButton("Kill Node")
        self.btn_cleanup_all = QPushButton("Dọn sạch ROS2")
        self.btn_cleanup_all.setStyleSheet("background-color: #fee2e2; color: #b91c1c; font-weight: bold;")
        self.btn_cleanup_all.setToolTip("Quét và đóng sạch tất cả tiến trình ROS 2 và reset daemon")
        self.btn_start_platform.clicked.connect(self.start_platform); self.btn_stop_platform.clicked.connect(self.stop_platform)
        self.btn_start_mode.clicked.connect(self.start_operation_mode); self.btn_stop_mode.clicked.connect(self.stop_operation_mode)
        self.btn_switch_mode.clicked.connect(self.switch_operation_mode)
        self.btn_start_nav2.clicked.connect(self.start_nav2); self.btn_stop_nav2.clicked.connect(self.stop_nav2)
        self.btn_start_cameras.clicked.connect(self.start_cameras); self.btn_stop_cameras.clicked.connect(self.stop_cameras)
        self.btn_start_ai.clicked.connect(self.start_ai_perception); self.btn_stop_ai.clicked.connect(self.stop_ai_perception)
        self.btn_start_backend.clicked.connect(self.start_backend); self.btn_stop_backend.clicked.connect(self.stop_backend)
        self.btn_start_kiosk.clicked.connect(self.start_kiosk); self.btn_stop_kiosk.clicked.connect(self.stop_kiosk)
        self.btn_cancel_nav2.clicked.connect(self.cancel_nav2_goal)
        self.btn_start_all.clicked.connect(self.start_all); self.btn_stop_all.clicked.connect(self.stop_all)
        self.btn_kill_node.clicked.connect(self.kill_node); self.btn_cleanup_all.clicked.connect(self.manual_cleanup)
        self.lbl_platform_status = QLabel("Layer1 Odom: OFF"); self.lbl_mode_status = QLabel("Layer2 Mode: OFF")
        self.lbl_nav2_status = QLabel("Nav2: OFF"); self.lbl_cameras_status = QLabel("Cam + Vật cản: OFF")
        self.lbl_ai_status = QLabel("AI Perception: OFF"); self.lbl_backend_status = QLabel("Backend: OFF")
        self.lbl_kiosk_status = QLabel("Kiosk UI: OFF")
        self.lbl_odom_status = QLabel("Odom source: esp -> /odom"); self.lbl_dataenc = QLabel("Encoder /dataenc: -")
        self.lbl_sensor_summary = QLabel("LiDAR: 0.0Hz | Odom: 0.0Hz | TF: MISS")
        self.lbl_sensor_summary.setStyleSheet("font-size: 11px; color: #334155;")
        self.lbl_cam_summary = QLabel("Cam F/R/AI: -/-/- | Obs(m): F=- R=-")
        self.lbl_cam_summary.setStyleSheet("font-size: 11px; color: #334155;")
        self.lbl_battery_summary = QLabel("Pin 36V (3×12V 30Ah): --% (-- V)")
        self.lbl_battery_summary.setStyleSheet("font-size: 11px; font-weight: bold; color: #334155;")
        control_group = QGroupBox("Điều khiển"); control_layout = QGridLayout()
        control_layout.addWidget(self.btn_start_platform, 0, 0); control_layout.addWidget(self.btn_stop_platform, 0, 1)
        control_layout.addWidget(self.btn_start_mode, 1, 0); control_layout.addWidget(self.btn_stop_mode, 1, 1)
        control_layout.addWidget(self.btn_switch_mode, 2, 0, 1, 2)
        control_layout.addWidget(self.btn_start_nav2, 3, 0); control_layout.addWidget(self.btn_stop_nav2, 3, 1)
        control_layout.addWidget(self.btn_start_cameras, 4, 0); control_layout.addWidget(self.btn_stop_cameras, 4, 1)
        control_layout.addWidget(self.btn_start_ai, 5, 0); control_layout.addWidget(self.btn_stop_ai, 5, 1)
        control_layout.addWidget(self.btn_start_backend, 6, 0); control_layout.addWidget(self.btn_stop_backend, 6, 1)
        control_layout.addWidget(self.btn_start_kiosk, 7, 0); control_layout.addWidget(self.btn_stop_kiosk, 7, 1)
        control_layout.addWidget(self.btn_cancel_nav2, 8, 0, 1, 2)
        control_layout.addWidget(self.btn_start_all, 9, 0); control_layout.addWidget(self.btn_stop_all, 9, 1)
        control_layout.addWidget(QLabel("Node/pattern to kill:"), 10, 0); control_layout.addWidget(self.edt_kill_node, 10, 1)
        control_layout.addWidget(self.btn_kill_node, 11, 0); control_layout.addWidget(self.btn_cleanup_all, 11, 1)
        control_layout.addWidget(self.lbl_platform_status, 12, 0, 1, 2); control_layout.addWidget(self.lbl_mode_status, 13, 0, 1, 2)
        control_layout.addWidget(self.lbl_nav2_status, 14, 0, 1, 2); control_layout.addWidget(self.lbl_cameras_status, 15, 0, 1, 2)
        control_layout.addWidget(self.lbl_ai_status, 16, 0, 1, 2); control_layout.addWidget(self.lbl_backend_status, 17, 0, 1, 2)
        control_layout.addWidget(self.lbl_kiosk_status, 18, 0, 1, 2)
        control_layout.addWidget(self.lbl_odom_status, 19, 0, 1, 2); control_layout.addWidget(self.lbl_dataenc, 20, 0, 1, 2)
        control_layout.addWidget(self.lbl_sensor_summary, 21, 0, 1, 2); control_layout.addWidget(self.lbl_cam_summary, 22, 0, 1, 2)
        control_layout.addWidget(self.lbl_battery_summary, 23, 0, 1, 2)
        self.chk_mock_human = QCheckBox("👤 Giả lập: Có người trước mặt")
        self.chk_mock_human.setStyleSheet("color: #0f172a; font-weight: bold;")
        self.chk_mock_human.setToolTip("Bật để giả lập có người chắn trước mũi xe, kiểm tra phản ứng của robot")
        self.chk_mock_human.toggled.connect(self.on_mock_human_toggled)
        control_layout.addWidget(self.chk_mock_human, 24, 0, 1, 2)
        control_group.setLayout(control_layout)
        # QTabWidget chứa Tab 1 (Log gốc) và Tab 2 (Trạng thái & Suy nghĩ Nav2)
        self.tabs_info = QTabWidget()
        self.tabs_info.setMinimumHeight(180)

        # TAB 1: Log gốc hệ thống
        tab_log = QWidget()
        tab_log_layout = QVBoxLayout()
        tab_log_layout.setContentsMargins(4, 4, 4, 4)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(140)
        tab_log_layout.addWidget(self.log_box)
        tab_log.setLayout(tab_log_layout)
        self.tabs_info.addTab(tab_log, "📜 Log Hệ Thống")

        # TAB 2: Chi tiết & Lịch sử điều hướng Nav2 (banner đã đưa lên thanh Não Robot chung phía trên)
        tab_mind = QWidget()
        tab_mind_layout = QVBoxLayout()
        tab_mind_layout.setContentsMargins(6, 6, 6, 6)
        tab_mind_layout.setSpacing(6)

        # 1. Nhóm thông số lộ trình Nav2
        nav_info_group = QGroupBox("Tiến trình điều hướng Nav2")
        nav_info_layout = QGridLayout()
        nav_info_layout.setContentsMargins(6, 6, 6, 6)
        nav_info_layout.setSpacing(4)

        self.lbl_mind_goal = QLabel("🎯 Mục tiêu: Chưa nhận")
        self.lbl_mind_goal.setStyleSheet("font-weight: bold; color: #0f172a;")
        self.lbl_mind_dist = QLabel("📏 Còn lại: -- m")
        self.lbl_mind_time = QLabel("⏱ Thời gian: --")
        self.lbl_mind_recovery = QLabel("🔄 Tự cứu (Recovery): 0 lần")

        self.pbar_nav = QProgressBar()
        self.pbar_nav.setRange(0, 100)
        self.pbar_nav.setValue(0)
        self.pbar_nav.setTextVisible(True)
        self.pbar_nav.setFormat("Tiến độ: %p%")
        self.pbar_nav.setStyleSheet(
            "QProgressBar { border: 1px solid #cbd5e1; border-radius: 4px; text-align: center; background-color: #ffffff; color: #0f172a; font-weight: bold; height: 18px; }"
            "QProgressBar::chunk { background-color: #22c55e; border-radius: 3px; }"
        )

        nav_info_layout.addWidget(self.lbl_mind_goal, 0, 0, 1, 2)
        nav_info_layout.addWidget(self.pbar_nav, 1, 0, 1, 2)
        nav_info_layout.addWidget(self.lbl_mind_dist, 2, 0)
        nav_info_layout.addWidget(self.lbl_mind_time, 2, 1)
        nav_info_layout.addWidget(self.lbl_mind_recovery, 3, 0, 1, 2)
        nav_info_group.setLayout(nav_info_layout)
        tab_mind_layout.addWidget(nav_info_group)

        # 2. Nhật ký suy nghĩ & tự cứu gần nhất
        thought_hist_header = QHBoxLayout()
        lbl_hist_title = QLabel("📝 Lịch sử phản ứng Nav2 & Tự cứu gần nhất:")
        lbl_hist_title.setStyleSheet("font-weight: bold; font-size: 11px; color: #334155;")
        self.btn_clear_thoughts = QPushButton("Xóa")
        self.btn_clear_thoughts.setFixedSize(45, 20)
        self.btn_clear_thoughts.setStyleSheet("font-size: 10px; padding: 0px;")
        self.btn_clear_thoughts.clicked.connect(self.clear_thought_history)
        thought_hist_header.addWidget(lbl_hist_title)
        thought_hist_header.addStretch()
        thought_hist_header.addWidget(self.btn_clear_thoughts)
        tab_mind_layout.addLayout(thought_hist_header)

        self.thought_history_box = QTextEdit()
        self.thought_history_box.setReadOnly(True)
        self.thought_history_box.setMinimumHeight(100)
        self.thought_history_box.setStyleSheet(
            "background-color: #ffffff; color: #000000; font-family: Monospace, Consolas, monospace; font-size: 10px; border: 1px solid #cbd5e1; border-radius: 4px;"
        )
        tab_mind_layout.addWidget(self.thought_history_box, stretch=1)
        tab_mind.setLayout(tab_mind_layout)
        self.tabs_info.addTab(tab_mind, "🧭 Chi Tiết Nav2")

        # TAB 3: Debug Cảm biến, Camera, TF & Chuyển động
        tab_debug = QWidget()
        tab_debug_layout = QVBoxLayout()
        tab_debug_layout.setContentsMargins(6, 6, 6, 6)
        tab_debug_layout.setSpacing(6)

        dbg_hw_group = QGroupBox("Cảm biến, TF & Chuyển động")
        dbg_hw_layout = QVBoxLayout()
        dbg_hw_layout.setSpacing(3)
        self.lbl_dbg_lidar = QLabel("LiDAR (/scan): OFF")
        self.lbl_dbg_odom_cmd = QLabel("Odom (0.0Hz): vx=+0.00 wz=+0.00 | CmdVel: vx=+0.00 wz=+0.00")
        self.lbl_dbg_tf = QLabel("TF: map->odom [MISS] | odom->base_footprint [MISS]")
        self.lbl_dbg_battery = QLabel("Ắc quy 36V (3×12V 30Ah): Chưa có điện áp")
        self.lbl_dbg_robot_status = QLabel("/robot_status: Chưa có dữ liệu (cần Start Backend)")
        for lbl in (self.lbl_dbg_lidar, self.lbl_dbg_odom_cmd, self.lbl_dbg_tf, self.lbl_dbg_battery, self.lbl_dbg_robot_status):
            lbl.setWordWrap(True)
            lbl.setStyleSheet("font-size: 11px; color: #0f172a;")
            dbg_hw_layout.addWidget(lbl)
        dbg_hw_group.setLayout(dbg_hw_layout)

        dbg_cam_group = QGroupBox("3 Camera & Vật cản (160×120)")
        dbg_cam_layout = QVBoxLayout()
        dbg_cam_layout.setSpacing(3)
        self.lbl_dbg_cam_front = QLabel("Cam Trước Depth: OFF")
        self.lbl_dbg_cam_rear = QLabel("Cam Sau Depth: OFF")
        self.lbl_dbg_cam_ai = QLabel("Cam AI RGB/Depth: OFF / OFF")
        self.lbl_dbg_obs_front = QLabel("/scan_obstacles: OFF | min=-")
        self.lbl_dbg_obs_rear = QLabel("/rear_scan_obstacle: OFF | min=-")
        self.btn_open_cam_debugger = QPushButton("🛠 Mở Debugger 2 Cam (ROI / Mặt sàn)")
        self.btn_open_cam_debugger.setToolTip("Chạy depth_obstacle_detector debugger_gui để hiệu chuẩn cam trước/sau")
        self.btn_open_cam_debugger.clicked.connect(self.launch_obstacle_debugger)
        for lbl in (
            self.lbl_dbg_cam_front, self.lbl_dbg_cam_rear, self.lbl_dbg_cam_ai,
            self.lbl_dbg_obs_front, self.lbl_dbg_obs_rear,
        ):
            lbl.setWordWrap(True)
            lbl.setStyleSheet("font-size: 11px; color: #0f172a;")
            dbg_cam_layout.addWidget(lbl)
        dbg_cam_layout.addWidget(self.btn_open_cam_debugger)
        dbg_cam_group.setLayout(dbg_cam_layout)

        dbg_ai_group = QGroupBox("Debug AI (Người / Khoảng cách / Vẫy tay)")
        dbg_ai_layout = QVBoxLayout()
        dbg_ai_layout.setSpacing(3)
        self.lbl_dbg_yolo = QLabel("YOLO (/yolo/tracking): OFF")
        self.lbl_dbg_people_count = QLabel("Đã phát hiện người: Chưa có dữ liệu")
        self.lbl_dbg_wave = QLabel("Vẫy tay (/pose/wave_detected): OFF")
        self.lbl_dbg_people_detail = QLabel("Chi tiết: --")
        for lbl in (self.lbl_dbg_yolo, self.lbl_dbg_people_count, self.lbl_dbg_wave, self.lbl_dbg_people_detail):
            lbl.setWordWrap(True)
            lbl.setStyleSheet("font-size: 11px; color: #0f172a;")
            dbg_ai_layout.addWidget(lbl)
        dbg_ai_group.setLayout(dbg_ai_layout)

        tab_debug_layout.addWidget(dbg_hw_group)
        tab_debug_layout.addWidget(dbg_cam_group)
        tab_debug_layout.addWidget(dbg_ai_group)
        tab_debug_layout.addStretch()
        tab_debug.setLayout(tab_debug_layout)
        self.tabs_info.addTab(tab_debug, "📡 Debug Sensor")

        left_layout = QVBoxLayout()
        left_layout.addWidget(system_group); left_layout.addWidget(map_group)
        left_layout.addWidget(control_group); left_layout.addWidget(self.tabs_info, stretch=1)
        left_widget = QWidget(); left_widget.setLayout(left_layout)
        left_scroll = QScrollArea(); left_scroll.setWidgetResizable(True); left_scroll.setWidget(left_widget)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff); left_scroll.setMinimumWidth(290)
        self.map_view = MapViewer(self.get_ros_node, on_map_click=self.on_map_clicked, on_map_drag=self.on_map_dragged)
        right_layout = QVBoxLayout(); right_layout.addWidget(self.map_view)
        right_widget = QWidget(); right_widget.setLayout(right_layout); right_widget.setMinimumWidth(260)

        # === TAB LỚN 1: BẢN ĐỒ & VẬN HÀNH HỆ THỐNG ===
        self.tab_system = QWidget()
        tab1_layout = QHBoxLayout(self.tab_system)
        tab1_layout.addWidget(left_scroll, stretch=0)
        tab1_layout.addWidget(right_widget, stretch=1)
        tab1_layout.setContentsMargins(6, 6, 6, 6)
        tab1_layout.setSpacing(6)

        # === TAB LỚN 2: TRUNG TÂM TUẦN TRA & PHỤC VỤ (MISSION DASHBOARD) ===
        mission_container = QWidget()
        mission_root = QHBoxLayout(mission_container)
        mission_root.setContentsMargins(8, 8, 8, 8)
        mission_root.setSpacing(10)

        # --- CỘT TRÁI TAB 2: THIẾT LẬP LỘ TRÌNH & BẢN ĐỒ CHẤM ĐIỂM ---
        mission_left_widget = QWidget()
        mission_left = QVBoxLayout(mission_left_widget)
        mission_left.setContentsMargins(0, 0, 0, 0)
        mission_left.setSpacing(6)

        setup_group = QGroupBox("📍 Thiết Lập Lộ Trình Tuần Tra (locations.yaml & routes.yaml)")
        setup_layout = QVBoxLayout()
        toolbar_row1 = QHBoxLayout()
        self.btn_patrol_pick_mode = QPushButton("🎯 Chấm điểm trên Map: OFF")
        self.btn_patrol_pick_mode.setStyleSheet("font-weight: bold; padding: 6px 10px;")
        self.btn_patrol_pick_mode.clicked.connect(self.toggle_patrol_waypoint_mode)
        self.btn_patrol_undo = QPushButton("↩ Hoàn tác")
        self.btn_patrol_undo.setStyleSheet("padding: 6px 10px;")
        self.btn_patrol_undo.clicked.connect(self.undo_batch_waypoint)
        self.btn_patrol_clear = QPushButton("🗑️ Xóa hết")
        self.btn_patrol_clear.setStyleSheet("padding: 6px 10px;")
        self.btn_patrol_clear.clicked.connect(self.clear_batch_waypoints)
        toolbar_row1.addWidget(self.btn_patrol_pick_mode)
        toolbar_row1.addWidget(self.btn_patrol_undo)
        toolbar_row1.addWidget(self.btn_patrol_clear)

        toolbar_row2 = QHBoxLayout()
        self.btn_patrol_load = QPushButton("📂 Nạp từ Config")
        self.btn_patrol_load.setStyleSheet("padding: 7px 10px;")
        self.btn_patrol_load.clicked.connect(lambda: self.load_patrol_config_from_yaml(silent=False))
        self.btn_patrol_save = QPushButton("💾 LƯU & ÁP DỤNG LỘ TRÌNH")
        self.btn_patrol_save.setStyleSheet("background-color: #2563eb; color: white; font-weight: bold; padding: 7px 12px; border-radius: 4px;")
        self.btn_patrol_save.clicked.connect(self.save_patrol_config_to_yaml)
        toolbar_row2.addWidget(self.btn_patrol_load)
        toolbar_row2.addWidget(self.btn_patrol_save, stretch=1)

        self.lbl_patrol_hint = QLabel(
            "Bật 'Chấm điểm trên Map: ON' rồi click lên bản đồ bên dưới để thêm lần lượt Trạm nước (khoihanh) → diem1 → diem2 → diem3..."
        )
        self.lbl_patrol_hint.setWordWrap(True)
        self.lbl_patrol_hint.setStyleSheet("color: #475569; font-size: 11px;")

        setup_layout.addLayout(toolbar_row1)
        setup_layout.addLayout(toolbar_row2)
        setup_layout.addWidget(self.lbl_patrol_hint)
        setup_group.setLayout(setup_layout)
        mission_left.addWidget(setup_group)

        self.patrol_map_view = MapViewer(self.get_ros_node, on_map_click=self.on_patrol_map_clicked)
        self.patrol_map_view.setMinimumHeight(200)
        mission_left.addWidget(self.patrol_map_view, stretch=3)

        wp_table_group = QGroupBox("📋 Danh sách các điểm tuần tra (có thể sửa trực tiếp X, Y trước khi Lưu)")
        wp_table_layout = QVBoxLayout()
        self.table_waypoints = QTableWidget(0, 4)
        self.table_waypoints.setHorizontalHeaderLabels(["Tên điểm", "X (m)", "Y (m)", "Ghi chú"])
        self.table_waypoints.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_waypoints.setMinimumHeight(110)
        wp_table_layout.addWidget(self.table_waypoints)
        wp_table_group.setLayout(wp_table_layout)
        mission_left.addWidget(wp_table_group, stretch=2)

        # --- CỘT PHẢI TAB 2: ĐIỀU KHIỂN MISSION, LỘ TRÌNH & SANDBOX ---
        mission_right_widget = QWidget()
        mission_right = QVBoxLayout(mission_right_widget)
        mission_right.setContentsMargins(0, 0, 0, 0)
        mission_right.setSpacing(8)

        ctrl_mission_group = QGroupBox("🎮 Điều khiển Nhiệm vụ Tuần Tra")
        ctrl_mission_layout = QHBoxLayout()
        self.btn_fsm_start_patrol = QPushButton("▶ BẮT ĐẦU TUẦN TRA")
        self.btn_fsm_start_patrol.setStyleSheet(
            "background-color: #16a34a; color: white; font-size: 14px; font-weight: bold; padding: 12px; border-radius: 6px;"
        )
        self.btn_fsm_start_patrol.clicked.connect(self.on_fsm_start_patrol)
        self.btn_fsm_pause_patrol = QPushButton("⏸ TẠM DỪNG TUẦN TRA")
        self.btn_fsm_pause_patrol.setStyleSheet(
            "background-color: #d97706; color: white; font-size: 14px; font-weight: bold; padding: 12px; border-radius: 6px;"
        )
        self.btn_fsm_pause_patrol.clicked.connect(self.on_fsm_pause_patrol)
        ctrl_mission_layout.addWidget(self.btn_fsm_start_patrol)
        ctrl_mission_layout.addWidget(self.btn_fsm_pause_patrol)
        ctrl_mission_group.setLayout(ctrl_mission_layout)
        mission_right.addWidget(ctrl_mission_group)

        route_group = QGroupBox("🗺️ Lộ trình tuần tra hiện tại")
        route_layout = QVBoxLayout()
        self.lbl_fsm_route_seq = QLabel("Chưa có điểm nào.")
        self.lbl_fsm_route_seq.setWordWrap(True)
        self.lbl_fsm_route_seq.setStyleSheet("font-weight: bold; color: #0f172a; font-size: 12px;")
        self.lbl_fsm_current_target = QLabel("🎯 Mục tiêu hiện tại: Chưa xác định")
        self.lbl_fsm_current_target.setStyleSheet("color: #1d4ed8; font-weight: bold; font-size: 12px;")
        route_layout.addWidget(self.lbl_fsm_route_seq)
        route_layout.addWidget(self.lbl_fsm_current_target)
        route_group.setLayout(route_layout)
        mission_right.addWidget(route_group)

        sandbox_group = QGroupBox("🧪 Developer Sandbox (Giả lập Cảm biến & Phục vụ không cần Camera)")
        sandbox_layout = QGridLayout()
        self.chk_fsm_mock_human = QCheckBox("👤 Giả lập: Người cản đường (<1.8m)")
        self.chk_fsm_mock_human.setStyleSheet("font-weight: bold; color: #0f172a;")
        self.chk_fsm_mock_human.toggled.connect(self.on_fsm_mock_human_toggled)
        self.chk_fsm_mock_wave = QCheckBox("👋 Giả lập: Người giơ tay vẫy")
        self.chk_fsm_mock_wave.setStyleSheet("font-weight: bold; color: #0f172a;")
        self.chk_fsm_mock_wave.toggled.connect(self.on_fsm_mock_wave_toggled)
        self.btn_fsm_trigger_serve = QPushButton("☕ Bấm Giả Lập: Lấy Nước")
        self.btn_fsm_trigger_serve.setStyleSheet("background-color: #0284c7; color: white; font-weight: bold; padding: 8px; border-radius: 4px;")
        self.btn_fsm_trigger_serve.clicked.connect(self.on_fsm_trigger_serve)
        self.btn_fsm_trigger_finish = QPushButton("✅ Bấm Giả Lập: Đã Hài Lòng")
        self.btn_fsm_trigger_finish.setStyleSheet("background-color: #059669; color: white; font-weight: bold; padding: 8px; border-radius: 4px;")
        self.btn_fsm_trigger_finish.clicked.connect(self.on_fsm_trigger_finish)
        sandbox_layout.addWidget(self.chk_fsm_mock_human, 0, 0)
        sandbox_layout.addWidget(self.chk_fsm_mock_wave, 0, 1)
        sandbox_layout.addWidget(self.btn_fsm_trigger_serve, 1, 0)
        sandbox_layout.addWidget(self.btn_fsm_trigger_finish, 1, 1)
        sandbox_group.setLayout(sandbox_layout)
        mission_right.addWidget(sandbox_group)

        fsm_log_group = QGroupBox("📜 Nhật ký hành vi tự động (/mission/status_log)")
        fsm_log_layout = QVBoxLayout()
        self.fsm_log_box = QTextEdit()
        self.fsm_log_box.setReadOnly(True)
        self.fsm_log_box.setStyleSheet(
            "background-color: #ffffff; color: #000000; font-family: Monospace, Consolas, monospace; font-size: 11px; border: 1px solid #cbd5e1; border-radius: 4px;"
        )
        self.btn_clear_fsm_log = QPushButton("Xóa log FSM")
        self.btn_clear_fsm_log.clicked.connect(lambda: self.fsm_log_box.clear())
        fsm_log_layout.addWidget(self.fsm_log_box, stretch=1)
        fsm_log_layout.addWidget(self.btn_clear_fsm_log)
        fsm_log_group.setLayout(fsm_log_layout)
        mission_right.addWidget(fsm_log_group, stretch=1)

        mission_root.addWidget(mission_left_widget, stretch=1)
        mission_root.addWidget(mission_right_widget, stretch=1)

        self.tab_mission = QScrollArea()
        self.tab_mission.setWidgetResizable(True)
        self.tab_mission.setFrameShape(QFrame.NoFrame)
        self.tab_mission.setWidget(mission_container)

        # === THANH "NÃO ROBOT" HỢP NHẤT (LUÔN HIỂN THỊ Ở CẢ 2 TAB LỚN) ===
        brain_bar = QHBoxLayout()
        brain_bar.setContentsMargins(0, 0, 0, 2)
        brain_bar.setSpacing(6)

        # Ô trái: Trạng thái Mission / Phục vụ (BT cấp cao)
        self.fsm_banner_frame = QFrame()
        self.fsm_banner_frame.setStyleSheet(
            "QFrame { background-color: #ffffff; border: 2px solid #2563eb; border-radius: 6px; padding: 5px 8px; }"
        )
        fsm_banner_layout = QVBoxLayout(self.fsm_banner_frame)
        fsm_banner_layout.setContentsMargins(2, 2, 2, 2)
        fsm_banner_layout.setSpacing(2)
        self.lbl_fsm_title = QLabel("🎯 NHIỆM VỤ & PHỤC VỤ (MISSION)")
        self.lbl_fsm_title.setStyleSheet("font-weight: bold; font-size: 10px; color: #475569; letter-spacing: 0.4px;")
        self.lbl_fsm_state = QLabel("CHỜ LỆNH / ĐANG KHỞI ĐỘNG")
        self.lbl_fsm_state.setStyleSheet("font-size: 13px; font-weight: bold; color: #1d4ed8;")
        self.lbl_fsm_state.setWordWrap(True)
        self.lbl_fsm_detail = QLabel("Kết nối với mission_controller_node để xem hành vi tuần tra & phục vụ.")
        self.lbl_fsm_detail.setStyleSheet("font-size: 11px; color: #0f172a;")
        self.lbl_fsm_detail.setWordWrap(True)
        fsm_banner_layout.addWidget(self.lbl_fsm_title)
        fsm_banner_layout.addWidget(self.lbl_fsm_state)
        fsm_banner_layout.addWidget(self.lbl_fsm_detail)

        # Ô phải: Suy nghĩ & Tự cứu Nav2 (MPPI / CustomBackUp)
        self.mind_banner_frame = QFrame()
        self.mind_banner_frame.setStyleSheet(
            "QFrame { background-color: #ffffff; border: 2px solid #0f766e; border-radius: 6px; padding: 5px 8px; }"
        )
        mind_banner_layout = QVBoxLayout(self.mind_banner_frame)
        mind_banner_layout.setContentsMargins(2, 2, 2, 2)
        mind_banner_layout.setSpacing(2)
        mind_hdr_row = QHBoxLayout()
        mind_hdr_row.setContentsMargins(0, 0, 0, 0)
        self.lbl_mind_title = QLabel("🧭 ĐIỀU HƯỚNG & TỰ CỨU (NAV2)")
        self.lbl_mind_title.setStyleSheet("font-weight: bold; font-size: 10px; color: #475569; letter-spacing: 0.4px;")
        self.lbl_top_nav_metrics = QLabel("📏 -- m | 🔄 Recovery: 0 lần")
        self.lbl_top_nav_metrics.setStyleSheet("font-size: 10px; font-weight: bold; color: #334155;")
        mind_hdr_row.addWidget(self.lbl_mind_title)
        mind_hdr_row.addStretch()
        mind_hdr_row.addWidget(self.lbl_top_nav_metrics)
        self.lbl_mind_status = QLabel("Chưa có lệnh điều hướng")
        self.lbl_mind_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #0f172a;")
        self.lbl_mind_status.setWordWrap(True)
        self.lbl_mind_desc = QLabel("Robot đang ở trạng thái chờ. Click mục tiêu trên bản đồ hoặc Bắt đầu tuần tra.")
        self.lbl_mind_desc.setStyleSheet("font-size: 11px; color: #0f172a;")
        self.lbl_mind_desc.setWordWrap(True)
        mind_banner_layout.addLayout(mind_hdr_row)
        mind_banner_layout.addWidget(self.lbl_mind_status)
        mind_banner_layout.addWidget(self.lbl_mind_desc)

        brain_bar.addWidget(self.fsm_banner_frame, stretch=1)
        brain_bar.addWidget(self.mind_banner_frame, stretch=1)

        # === THANH TAB LỚN TOÀN CỬA SỔ ===
        self.main_tabs = QTabWidget()
        self.main_tabs.setStyleSheet(
            "QTabBar::tab { background-color: #f1f5f9; color: #334155; font-size: 13px; font-weight: bold; padding: 8px 14px; border: 1px solid #cbd5e1; }"
            "QTabBar::tab:selected { background-color: #ffffff; color: #000000; border-bottom: 3px solid #2563eb; }"
        )
        self.main_tabs.addTab(self.tab_system, "🗺️ BẢN ĐỒ & VẬN HÀNH HỆ THỐNG")
        self.main_tabs.addTab(self.tab_mission, "🚀 TRUNG TÂM TUẦN TRA & PHỤC VỤ (MISSION)")

        root_layout = QVBoxLayout()
        root_layout.setContentsMargins(4, 4, 4, 4)
        root_layout.setSpacing(4)
        root_layout.addLayout(brain_bar)
        root_layout.addWidget(self.main_tabs, stretch=1)
        self.setLayout(root_layout)
        self.on_operation_mode_changed(); self.on_odom_source_changed(); self.refresh_batch_waypoint_views(); self.refresh_serial_ports()

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros_node = RosInterface(
            log_callback=self.append_log,
            map_callback=self.on_new_map,
            pose_callback=self.on_new_pose,
            scan_callback=self.on_new_scan,
            obstacle_scan_callback=self.on_new_obstacle_scan,
            dataenc_callback=self.on_new_dataenc,
            nav_feedback_callback=self.on_new_nav_feedback,
            nav_status_callback=self.on_new_nav_status,
            human_callback=self.on_human_detected,
            mission_log_callback=self.on_mission_log,
            popup_cmd_callback=self.on_popup_cmd,
            operator_alert_callback=self.on_operator_alert,
        )
        self.ros_spin_thread = threading.Thread(target=self.spin_ros, daemon=True); self.ros_spin_thread.start()
        self.append_log("ROS2 node v3 đã khởi động.")

    def spin_ros(self):
        try:
            rclpy.spin(self.ros_node)
        except Exception as exc:
            if not self.is_shutting_down:
                self.append_log(f"ROS spin error: {exc}")

    def get_ros_node(self):
        return self.ros_node

    def append_log(self, text):
        self.log_signal.emit(str(text))

    def on_mock_human_toggled(self, checked):
        if self.ros_node:
            self.ros_node.publish_mock_human(checked)
            self.append_log(f"[MOCK_HUMAN] Giả lập có người phía trước: {'BẬT (True)' if checked else 'TẮT (False)'}")

    def on_human_detected(self, is_human: bool):
        self.human_signal.emit(is_human)

    def _on_human_detected_ui(self, is_human: bool):
        now_str = time.strftime("%H:%M:%S")
        if is_human:
            self.lbl_mind_status.setText("👤 PHÁT HIỆN CÓ NGƯỜI TRƯỚC MẶT")
            self.lbl_mind_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #0284c7;")
            self.lbl_mind_desc.setText("Camera AI phát hiện có người trong cự ly an toàn (<1.8m). Robot đang chủ động phanh dừng nhường đường (tạm khóa tự lùi/xoay).")
            self.lbl_mind_desc.setStyleSheet("font-size: 11px; color: #0f172a; line-height: 1.3;")
            self.mind_banner_frame.setStyleSheet(
                "QFrame { background-color: #ffffff; border: 2px solid #0284c7; border-radius: 6px; padding: 6px; }"
            )
            self.thought_history_box.append(f"<span style='color: #64748b;'>[{now_str}]</span> <b style='color: #0284c7;'>👤 PHÁT HIỆN NGƯỜI</b>: Đang phanh dừng chờ nhường đường.")
        else:
            if "NGƯỜI" in self.lbl_mind_status.text():
                self.lbl_mind_status.setText("🚗 ĐƯỜNG ĐÃ THÔNG THOÁNG")
                self.lbl_mind_status.setStyleSheet("font-weight: bold; font-size: 13px; color: #16a34a;")
                self.lbl_mind_desc.setText("Không còn người chắn trước mũi xe. Robot tiếp tục di chuyển theo lộ trình.")
                self.lbl_mind_desc.setStyleSheet("font-size: 11px; color: #0f172a; line-height: 1.3;")
                self.mind_banner_frame.setStyleSheet(
                    "QFrame { background-color: #ffffff; border: 2px solid #16a34a; border-radius: 6px; padding: 6px; }"
                )
                self.thought_history_box.append(f"<span style='color: #64748b;'>[{now_str}]</span> <b style='color: #16a34a;'>ĐƯỜNG THOÁNG</b>: Người đã rời đi, tiếp tục hành trình.")

    def on_new_nav_feedback(self, dist, total_dist, nav_sec, est_sec, recov):
        self.nav_feedback_signal.emit(float(dist), float(total_dist), float(nav_sec), float(est_sec), int(recov))

    def on_new_nav_status(self, title, desc, color):
        self.nav_thought_signal.emit(str(title), str(desc), str(color))

    def on_operator_alert(self, code: str, message: str):
        self.operator_alert_signal.emit(str(code), str(message))

    def _on_operator_alert_ui(self, code: str, message: str):
        c = code.strip().upper()
        if c == "PRE_BACKUP":
            title, color = "⏪ CHUẨN BỊ LÙI TỰ CỨU (CustomBackUp)", "#a855f7"
        elif c == "BACKUP_WAIT":
            title, color = "🛑 TẠM DỪNG LÙI (CHỜ AN TOÀN PHÍA SAU)", "#f59e0b"
        elif c == "REPLANNING":
            title, color = "🔄 ĐÃ LÙI XONG — ĐANG TÍNH LẠI ĐƯỜNG", "#2563eb"
        elif c == "WAITING":
            title, color = "⏳ SỰ CỐ ĐIỀU HƯỚNG — ĐANG CHỜ XỬ LÝ", "#f97316"
        else:
            title, color = f"⚠️ CẢNH BÁO TỰ CỨU ({c or 'NAV2'})", "#ea580c"
        self._on_nav_thought_ui(title, message, color)

    def clear_thought_history(self):
        self.thought_history_box.clear()

    def _on_nav_feedback_ui(self, dist, total_dist, nav_sec, est_sec, recov):
        self.lbl_mind_dist.setText(f"📏 Còn lại: {dist:.2f} m")
        self.lbl_mind_time.setText(f"⏱ Đi: {int(nav_sec)}s | Dự kiến: {int(est_sec)}s")
        self.lbl_mind_recovery.setText(f"🔄 Tự cứu (Recovery): {recov} lần")
        if hasattr(self, "lbl_top_nav_metrics"):
            self.lbl_top_nav_metrics.setText(f"📏 {dist:.2f} m | 🔄 Recovery: {recov} lần")
        if total_dist > 0.05:
            progress = max(0, min(100, int((1.0 - (dist / total_dist)) * 100)))
            self.pbar_nav.setValue(progress)

    def _on_nav_thought_ui(self, title, desc, color):
        self.lbl_mind_status.setText(title)
        self.lbl_mind_status.setStyleSheet(f"font-weight: bold; font-size: 13px; color: {color};")
        self.lbl_mind_desc.setText(desc)
        self.lbl_mind_desc.setStyleSheet("font-size: 11px; color: #0f172a;")
        self.mind_banner_frame.setStyleSheet(
            f"QFrame {{ background-color: #ffffff; border: 2px solid {color}; border-radius: 6px; padding: 5px 8px; }}"
        )
        now_str = time.strftime("%H:%M:%S")
        self.thought_history_box.append(f"<span style='color: #64748b;'>[{now_str}]</span> <b style='color: {color};'>{title}</b>: {desc}")
        if hasattr(self, 'ros_node') and self.ros_node and hasattr(self.ros_node, 'current_goal_x') and self.ros_node.current_goal_x is not None:
            self.lbl_mind_goal.setText(f"🎯 Mục tiêu: ({self.ros_node.current_goal_x:.2f}, {self.ros_node.current_goal_y:.2f})")

    def _parse_log_for_mind(self, text):
        now = time.monotonic()
        lower = text.lower()
        title = None
        desc = None
        color = "#3b82f6"

        if "[custom_backup] safety_stop" in lower:
            title = "🛑 DỪNG LÙI KHẨN (CẢM BIẾN PHÍA SAU)"
            desc = "CustomBackUp dừng lùi để đảm bảo an toàn (vật cản/mất tín hiệu phía sau)."
            color = "#dc2626"
        elif "[custom_backup] partial_success" in lower:
            title = "✅ LÙI MỘT PHẦN ĐẠT YÊU CẦU (PARTIAL SUCCESS)"
            desc = "Đã lùi được khoảng an toàn tối thiểu trước khi gặp vật cản sau, chuyển sang tính lại đường."
            color = "#0d9488"
        elif "[custom_backup] start" in lower:
            title = "⏪ ĐANG LÙI TỰ CỨU CÓ GIÁM SÁT (CustomBackUp)"
            desc = "Đang lùi chậm kết hợp quét camera sau và local costmap."
            color = "#a855f7"
        elif "checkrearsafety:" in lower and "safe" not in lower:
            title = "🚫 KHÔNG ĐỦ AN TOÀN ĐỂ LÙI (CheckRearSafety)"
            desc = "Phía sau có vật cản hoặc thiếu dữ liệu camera sau (/rear_scan_obstacle)."
            color = "#f59e0b"
        elif "controller failed to compute command velocity" in lower or "failed to make progress" in lower:
            title = "⚠️ ĐANG DỪNG TRÁNH VẬT CẢN (MPPI)"
            desc = "Bộ điều khiển MPPI phát hiện vật cản quá sát thân xe. Robot đang phanh dừng an toàn để chờ đường mở hoặc tính đường lách."
            color = "#f97316"
        elif "failed to create plan" in lower or "failed to generate a valid path" in lower or "start pose is in collision" in lower or "goal is in collision" in lower or "could not compute path" in lower:
            title = "❌ BẾ TẮC: KHÔNG THỂ LẬP ĐƯỜNG ĐI"
            desc = "Bộ lập kế hoạch không tìm thấy đường đi an toàn. Vị trí đích hoặc điểm đứng của robot đang bị vật cản/tường chặn kín."
            color = "#ef4444"
        elif "backup" in lower or "backing up" in lower:
            title = "⏪ ĐANG TỰ ĐỘNG LÙI XE THOÁT KẸT"
            desc = "Đang lùi xe một khoảng ngắn để tạo cự ly an toàn trước mũi xe."
            color = "#a855f7"
        elif "spinning" in lower or ("spin" in lower and "behavior" in lower):
            title = "🔄 ĐANG XOAY TẠI CHỖ TÌM ĐƯỜNG"
            desc = "Robot đang xoay thân xe để cảm biến quét lại không gian xung quanh và tìm lối thoát."
            color = "#8b5cf6"
        elif "wait" in lower and "behavior" in lower:
            title = "⏳ ĐANG TẠM DỪNG CHỜ ĐƯỜNG THOÁNG"
            desc = "Đang tạm dừng tĩnh vài giây để chờ người hoặc vật cản phía trước rời đi."
            color = "#3b82f6"
        elif "clearing" in lower and "costmap" in lower:
            title = "🧹 ĐANG XÓA BÓNG MA CẢM BIẾN"
            desc = "Đang tự động dọn sạch bản đồ vật cản cục bộ để cảm biến LiDAR & Depth Camera cập nhật lại thực tế."
            color = "#eab308"
        elif "passing new path" in lower:
            title = "🚗 ĐÃ CÓ LỘ TRÌNH THÔNG THOÁNG"
            desc = "Đường đi đã thông suốt. MPPI đang điều khiển động cơ bám sát quỹ đạo."
            color = "#16a34a"
        elif "reached the goal" in lower or "goal succeeded" in lower:
            title = "🏁 ĐÃ ĐẾN ĐÍCH THÀNH CÔNG!"
            desc = "Robot đã hoàn thành lộ trình và đến đúng điểm hẹn."
            color = "#10b981"
            self.pbar_nav.setValue(100)
        elif "extrapolation into future" in lower or "extrapolation into past" in lower:
            title = "⚠️ LỆCH THỜI GIAN TF (TF JITTER)"
            desc = "Có độ trễ nhẹ giữa TF odom và máy tính. Nav2 đang tự động bù trừ dữ liệu."
            color = "#f59e0b"

        if title is not None:
            if title == self.last_thought_title and (now - self.last_thought_time) < 2.0:
                return
            self.last_thought_title = title
            self.last_thought_time = now
            self.nav_thought_signal.emit(title, desc, color)

    def _append_log_ui(self, text):
        self.log_box.append(text)
        self._parse_log_for_mind(text)

    def on_new_map(self, msg):
        self.map_signal.emit(msg)
    def on_new_pose(self, source, x, y, yaw):
        self.pose_signal.emit(float(x), float(y), float(yaw), source)
    def on_new_scan(self, msg, scan_points_world):
        self.scan_signal.emit(msg, scan_points_world)
    def on_new_obstacle_scan(self, side, points_world):
        self.obstacle_scan_signal.emit(str(side), list(points_world))
    def on_new_dataenc(self, values):
        self.dataenc_signal.emit(values)

    def _on_new_map_ui(self, msg):
        self.map_view.update_map(msg)
        if hasattr(self, "patrol_map_view"):
            self.patrol_map_view.update_map(msg)

    def _on_new_pose_ui(self, x, y, yaw, source):
        self.map_view.update_pose(x, y, yaw, source)
        if hasattr(self, "patrol_map_view"):
            self.patrol_map_view.update_pose(x, y, yaw, source)

    def _on_new_scan_ui(self, msg, scan_points_world):
        self.map_view.update_scan(msg, scan_points_world)
        if hasattr(self, "patrol_map_view"):
            self.patrol_map_view.update_scan(msg, scan_points_world)

    def _on_new_obstacle_scan_ui(self, side, points_world):
        self.map_view.update_obstacle_scan(side, points_world)
        if hasattr(self, "patrol_map_view"):
            self.patrol_map_view.update_obstacle_scan(side, points_world)

    def _on_new_dataenc_ui(self, values):
        if not values:
            self.lbl_dataenc.setText("Encoder: [No Data]")
        elif len(values) == 2:
            self.lbl_dataenc.setText(f"Encoder: L={values[0]} | R={values[1]}")
        else:
            self.lbl_dataenc.setText(f"Encoder: {values}")

    def on_map_dragged(self, wx, wy):
        pass

    def _add_patrol_waypoint_at(self, wx, wy):
        idx = len(self.batch_waypoints)
        name = self.waypoint_name_from_index(idx)
        note = "Trạm nước / Khởi hành" if name == "khoihanh" else f"Điểm tuần tra {idx}"
        self.batch_waypoints.append({
            "name": name,
            "x": float(wx),
            "y": float(wy),
            "yaw": 0.0,
            "note": note,
            "frame_id": "map",
        })
        self.refresh_batch_waypoint_views()
        self.append_log(f"Thêm điểm tuần tra {name}: x={wx:.3f}, y={wy:.3f}")

    def on_map_clicked(self, wx, wy):
        if self.batch_waypoint_mode:
            self._add_patrol_waypoint_at(wx, wy)
            return True
        return False

    def on_patrol_map_clicked(self, wx, wy):
        if self.patrol_waypoint_mode:
            self._add_patrol_waypoint_at(wx, wy)
            return True
        return False

    def waypoint_name_from_index(self, idx):
        if idx == 0:
            return "khoihanh"
        return f"diem{idx}"

    def refresh_batch_waypoint_views(self):
        recompute_cyclic_waypoint_yaws(self.batch_waypoints)
        self.btn_batch_mode.setText(f"Waypoint Mode: {'ON' if self.batch_waypoint_mode else 'OFF'}")
        self.lbl_batch_info.setText(f"Patrol Route: {len(self.batch_waypoints)} điểm")
        self.map_view.update_waypoint_batch(self.batch_waypoints)
        if hasattr(self, "patrol_map_view"):
            self.patrol_map_view.update_waypoint_batch(self.batch_waypoints)
        if hasattr(self, "btn_patrol_pick_mode"):
            self.btn_patrol_pick_mode.setText(
                f"🎯 Chấm điểm trên Map: {'ON' if self.patrol_waypoint_mode else 'OFF'}"
            )
            if self.patrol_waypoint_mode:
                self.btn_patrol_pick_mode.setStyleSheet(
                    "background-color: #16a34a; color: white; font-weight: bold; padding: 6px 10px; border-radius: 4px;"
                )
            else:
                self.btn_patrol_pick_mode.setStyleSheet("font-weight: bold; padding: 6px 10px;")
        if hasattr(self, "table_waypoints"):
            self.table_waypoints.blockSignals(True)
            self.table_waypoints.setRowCount(len(self.batch_waypoints))
            for row, wp in enumerate(self.batch_waypoints):
                self.table_waypoints.setItem(row, 0, QTableWidgetItem(str(wp.get("name", f"diem{row}"))))
                self.table_waypoints.setItem(row, 1, QTableWidgetItem(f"{float(wp.get('x', 0.0)):.3f}"))
                self.table_waypoints.setItem(row, 2, QTableWidgetItem(f"{float(wp.get('y', 0.0)):.3f}"))
                note = wp.get("note", "Trạm nước / Khởi hành" if wp.get("name") == "khoihanh" else f"Điểm tuần tra {row}")
                self.table_waypoints.setItem(row, 3, QTableWidgetItem(str(note)))
            self.table_waypoints.blockSignals(False)
        if hasattr(self, "lbl_fsm_route_seq"):
            if self.batch_waypoints:
                names = [str(wp.get("name", "?")) for wp in self.batch_waypoints]
                self.lbl_fsm_route_seq.setText(" ➔ ".join(names) + " ➔ (lặp lại)")
            else:
                self.lbl_fsm_route_seq.setText("Chưa có điểm tuần tra nào.")

    def toggle_batch_waypoint_mode(self):
        self.batch_waypoint_mode = not self.batch_waypoint_mode
        self.refresh_batch_waypoint_views()
        self.append_log("Waypoint mode (Tab 1) bật: click map để thêm điểm." if self.batch_waypoint_mode else "Waypoint mode (Tab 1) tắt.")

    def toggle_patrol_waypoint_mode(self):
        self.patrol_waypoint_mode = not self.patrol_waypoint_mode
        self.refresh_batch_waypoint_views()
        self.append_log("Chế độ chấm điểm tuần tra (Tab 2) BẬT: click lên bản đồ để đặt điểm." if self.patrol_waypoint_mode else "Chế độ chấm điểm tuần tra (Tab 2) TẮT.")

    def undo_batch_waypoint(self):
        if not self.batch_waypoints:
            return
        removed = self.batch_waypoints.pop()
        self.refresh_batch_waypoint_views()
        self.append_log(f"Hoàn tác điểm tuần tra: {removed['name']}")

    def clear_batch_waypoints(self):
        if not self.batch_waypoints:
            return
        self.batch_waypoints = []
        self.refresh_batch_waypoint_views()
        self.append_log("Đã xóa toàn bộ danh sách điểm tuần tra.")

    def open_mission_tab_from_batch(self):
        if self.batch_waypoints:
            self.save_patrol_config_to_yaml(show_dialog=False)
        if hasattr(self, "main_tabs"):
            self.main_tabs.setCurrentIndex(1)

    def load_patrol_config_from_yaml(self, silent=False):
        loc_file = os.path.join(CONFIG_DIR, "locations.yaml")
        route_file = os.path.join(CONFIG_DIR, "routes.yaml")
        locations = {}
        routes = {}
        try:
            if os.path.exists(loc_file):
                with open(loc_file, "r", encoding="utf-8") as f:
                    locations = yaml.safe_load(f) or {}
            if os.path.exists(route_file):
                with open(route_file, "r", encoding="utf-8") as f:
                    routes = yaml.safe_load(f) or {}
        except Exception as exc:
            if not silent:
                QMessageBox.warning(self, "Lỗi đọc Config", f"Không thể đọc file config YAML:\n{exc}")
            self.append_log(f"[Config] Lỗi đọc YAML: {exc}")
            return False

        patrol_names = routes.get("patrol_route")
        if not isinstance(patrol_names, list) or not patrol_names:
            patrol_names = list(locations.keys())

        loaded = []
        for idx, name in enumerate(patrol_names):
            coords = locations.get(name)
            if not isinstance(coords, dict):
                continue
            note = "Trạm nước / Khởi hành" if name == "khoihanh" else f"Điểm tuần tra ({name})"
            loaded.append({
                "name": str(name),
                "x": float(coords.get("x", 0.0)),
                "y": float(coords.get("y", 0.0)),
                "yaw": float(coords.get("yaw", 0.0)),
                "note": note,
                "frame_id": "map",
            })

        self.batch_waypoints = loaded
        self.refresh_batch_waypoint_views()
        self.append_log(f"[Config] Đã nạp {len(loaded)} điểm tuần tra từ {CONFIG_DIR}.")
        if not silent:
            QMessageBox.information(self, "Nạp cấu hình thành công", f"Đã nạp {len(loaded)} điểm tuần tra từ locations.yaml & routes.yaml.")
        return True

    def _sync_waypoints_from_table(self):
        if not hasattr(self, "table_waypoints"):
            return
        rows = self.table_waypoints.rowCount()
        updated = []
        for row in range(rows):
            item_name = self.table_waypoints.item(row, 0)
            item_x = self.table_waypoints.item(row, 1)
            item_y = self.table_waypoints.item(row, 2)
            item_note = self.table_waypoints.item(row, 3)
            name = (item_name.text().strip() if item_name else "") or self.waypoint_name_from_index(row)
            prev_wp = self.batch_waypoints[row] if row < len(self.batch_waypoints) else {}
            try:
                x = float(item_x.text().strip()) if item_x else float(prev_wp.get("x", 0.0))
                y = float(item_y.text().strip()) if item_y else float(prev_wp.get("y", 0.0))
            except ValueError:
                x = float(prev_wp.get("x", 0.0))
                y = float(prev_wp.get("y", 0.0))
            note = item_note.text().strip() if item_note else prev_wp.get("note", "")
            updated.append({
                "name": name,
                "x": x,
                "y": y,
                "yaw": float(prev_wp.get("yaw", 0.0)),
                "note": note,
                "frame_id": "map",
            })
        self.batch_waypoints = updated

    def save_patrol_config_to_yaml(self, show_dialog=True):
        self._sync_waypoints_from_table()
        recompute_cyclic_waypoint_yaws(self.batch_waypoints)
        if not self.batch_waypoints:
            if show_dialog:
                QMessageBox.warning(self, "Chưa có điểm nào", "Hãy chấm ít nhất 1 điểm trên bản đồ trước khi lưu lộ trình.")
            return False

        os.makedirs(CONFIG_DIR, exist_ok=True)
        loc_file = os.path.join(CONFIG_DIR, "locations.yaml")
        route_file = os.path.join(CONFIG_DIR, "routes.yaml")

        try:
            locations = {}
            if os.path.exists(loc_file):
                with open(loc_file, "r", encoding="utf-8") as f:
                    locations = yaml.safe_load(f) or {}
            routes = {}
            if os.path.exists(route_file):
                with open(route_file, "r", encoding="utf-8") as f:
                    routes = yaml.safe_load(f) or {}

            route_names = []
            for wp in self.batch_waypoints:
                name = str(wp["name"])
                route_names.append(name)
                locations[name] = {
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                }
            routes["patrol_route"] = route_names

            with open(loc_file, "w", encoding="utf-8") as f:
                yaml.safe_dump(locations, f, allow_unicode=True, sort_keys=False)
            with open(route_file, "w", encoding="utf-8") as f:
                yaml.safe_dump(routes, f, allow_unicode=True, sort_keys=False)
        except Exception as exc:
            if show_dialog:
                QMessageBox.critical(self, "Lỗi lưu Config", f"Không thể ghi YAML:\n{exc}")
            self.append_log(f"[Config] Lỗi ghi YAML: {exc}")
            return False

        if self.ros_node is not None:
            self.ros_node.sync_patrol_config(self.batch_waypoints, route_name="patrol_route")

        self.refresh_batch_waypoint_views()
        self.append_log(f"[Config] Đã lưu {len(self.batch_waypoints)} điểm vào locations.yaml & routes.yaml.")
        if show_dialog:
            QMessageBox.information(
                self,
                "Đã lưu lộ trình",
                f"Đã lưu {len(self.batch_waypoints)} điểm vào:\n- {loc_file}\n- {route_file}\n\nLộ trình: {' ➔ '.join(route_names)}",
            )
        return True

    def _set_fsm_banner(self, title: str, detail: str, border_color: str, bg_color: str = "#ffffff"):
        if not hasattr(self, "fsm_banner_frame"):
            return
        self.lbl_fsm_state.setText(title)
        self.lbl_fsm_state.setStyleSheet(f"font-size: 13px; font-weight: bold; color: {border_color};")
        self.lbl_fsm_detail.setText(detail)
        self.lbl_fsm_detail.setStyleSheet("font-size: 11px; color: #0f172a;")
        self.fsm_banner_frame.setStyleSheet(
            f"QFrame {{ background-color: {bg_color}; border: 2px solid {border_color}; border-radius: 6px; padding: 5px 8px; }}"
        )

    def on_mission_log(self, text: str):
        self.mission_log_signal.emit(str(text))

    def on_popup_cmd(self, cmd: str):
        self.mission_popup_signal.emit(str(cmd))

    def _on_mission_log_ui(self, text: str):
        now_str = time.strftime("%H:%M:%S")
        if hasattr(self, "fsm_log_box"):
            self.fsm_log_box.append(f"<span style='color:#64748b;'>[{now_str}]</span> {text}")

        lower = text.lower()
        if "khẩn cấp" in lower or "emergency" in lower:
            self._set_fsm_banner("🚨 KHẨN CẤP (EMERGENCY STOP)", text, "#dc2626", "#ffffff")
        elif "vẫy" in lower or "tiếp cận" in lower:
            self._set_fsm_banner("👋 ĐANG TIẾP CẬN NGƯỜI VẪY TAY", text, "#7c3aed", "#ffffff")
        elif "phục vụ" in lower or "lấy nước" in lower or "hài lòng" in lower:
            self._set_fsm_banner("☕ ĐANG PHỤC VỤ KHÁCH", text, "#2563eb", "#ffffff")
        elif "chắn đường" in lower or "gặp người" in lower:
            self._set_fsm_banner("🧍 GẶP NGƯỜI CẢN ĐƯỜNG (TẠM DỪNG)", text, "#ea580c", "#ffffff")
        elif "tạm dừng tuần tra" in lower:
            self._set_fsm_banner("⏸ ĐÃ TẠM DỪNG TUẦN TRA", text, "#475569", "#ffffff")
        elif "[tuần tra]" in lower or "bắt đầu tuần tra" in lower or "lộ trình tuần tra" in lower:
            self._set_fsm_banner("🚀 ĐANG TUẦN TRA TỰ ĐỘNG", text, "#16a34a", "#ffffff")
            if "đến điểm:" in text and hasattr(self, "lbl_fsm_current_target"):
                pt = text.split("đến điểm:", 1)[-1].strip()
                self.lbl_fsm_current_target.setText(f"🎯 Mục tiêu hiện tại: {pt}")

    def _on_mission_popup_ui(self, cmd: str):
        now_str = time.strftime("%H:%M:%S")
        if hasattr(self, "fsm_log_box"):
            self.fsm_log_box.append(f"<span style='color:#0284c7;'>[{now_str}] [POPUP KIOSK] {cmd}</span>")
        if cmd == "SHOW_5S_INVITE":
            self._set_fsm_banner("☕ MỜI KHÁCH LẤY NƯỚC (ĐẾM NGƯỢC 5S)", "Phát hiện người đứng trước robot. Bấm 'Lấy Nước' nếu muốn nhận nước.", "#0284c7", "#ffffff")
        elif cmd == "SHOW_SERVING":
            self._set_fsm_banner("☕ ĐANG PHỤC VỤ NƯỚC (KHÓA PHANH)", "Khách đang lấy nước. Bấm 'Đã Hài Lòng' khi hoàn tất.", "#2563eb", "#ffffff")
        elif cmd == "HIDE":
            self._set_fsm_banner("🚗 ĐÃ ĐÓNG POPUP PHỤC VỤ", "Quay trở lại trạng thái tuần tra bình thường.", "#16a34a", "#ffffff")

    def on_fsm_start_patrol(self):
        if self.ros_node is None:
            QMessageBox.warning(self, "ROS chưa sẵn sàng", "ROS node chưa khởi động.")
            return
        self.ros_node.call_mission_trigger(self.ros_node.cli_start_patrol, "Bắt đầu Tuần Tra (/mission/start_patrol)")

    def on_fsm_pause_patrol(self):
        if self.ros_node is None:
            QMessageBox.warning(self, "ROS chưa sẵn sàng", "ROS node chưa khởi động.")
            return
        self.ros_node.call_mission_trigger(self.ros_node.cli_pause_patrol, "Tạm dừng Tuần Tra (/mission/pause_patrol)")

    def on_fsm_mock_human_toggled(self, checked):
        if self.ros_node is None:
            return
        if checked and hasattr(self, "chk_fsm_mock_wave") and self.chk_fsm_mock_wave.isChecked():
            self.chk_fsm_mock_wave.setChecked(False)
        self.ros_node.publish_mock_human(checked)
        self.append_log(f"[Sandbox] Giả lập người cản đường: {'BẬT' if checked else 'TẮT'}")

    def on_fsm_mock_wave_toggled(self, checked):
        if self.ros_node is None:
            return
        if checked and hasattr(self, "chk_fsm_mock_human") and self.chk_fsm_mock_human.isChecked():
            self.chk_fsm_mock_human.setChecked(False)
        self.ros_node.publish_mock_waving(checked)
        self.append_log(f"[Sandbox] Giả lập người giơ tay vẫy: {'BẬT' if checked else 'TẮT'}")

    def _on_mock_timer_tick(self):
        if self.ros_node is None:
            return
        if hasattr(self, "chk_fsm_mock_wave") and self.chk_fsm_mock_wave.isChecked():
            self.ros_node.publish_mock_waving(True)
        elif hasattr(self, "chk_fsm_mock_human") and self.chk_fsm_mock_human.isChecked():
            self.ros_node.publish_mock_human(True)

    def on_fsm_trigger_serve(self):
        if self.ros_node is None:
            return
        msg = Bool(); msg.data = True
        self.ros_node.serve_trigger_pub.publish(msg)
        self.ros_node.call_mission_trigger(self.ros_node.cli_trigger_serve, "Lấy Nước (/mission/trigger_serve)")

    def on_fsm_trigger_finish(self):
        if self.ros_node is None:
            return
        msg = Bool(); msg.data = True
        self.ros_node.finish_serving_pub.publish(msg)
        self.ros_node.call_mission_trigger(self.ros_node.cli_trigger_finish, "Đã Hài Lòng (/mission/trigger_finish)")

    def choose_map_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Chọn thư mục chứa map YAML", self.map_dir if os.path.exists(self.map_dir) else os.path.expanduser("~"))
        if folder: self.map_dir = folder; self.refresh_map_list()
    def choose_map_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Chọn file map YAML", self.map_dir if os.path.exists(self.map_dir) else os.path.expanduser("~"), "YAML files (*.yaml *.yml)")
        if file_path: self.selected_map_file = file_path; self.lbl_selected_map.setText(f"Map đang dùng: {file_path}"); self.append_log(f"Đã chọn map thủ công: {file_path}")
    def refresh_map_list(self):
        os.makedirs(self.map_dir, exist_ok=True); self.lbl_map_folder.setText(f"Map folder: {self.map_dir}")
        files = sorted(glob.glob(os.path.join(self.map_dir, "*.yaml"))) + sorted(glob.glob(os.path.join(self.map_dir, "*.yml")))
        prev = self.selected_map_file
        self.cmb_map_files.blockSignals(True); self.cmb_map_files.clear()
        for fpath in files: self.cmb_map_files.addItem(os.path.basename(fpath), fpath)
        self.cmb_map_files.blockSignals(False)
        if files:
            pick_index = files.index(prev) if prev in files else 0
            self.cmb_map_files.setCurrentIndex(pick_index)
            self.selected_map_file = self.cmb_map_files.currentData()
            self.lbl_selected_map.setText(f"Map đang dùng: {self.selected_map_file}")
        else:
            self.selected_map_file = ""; self.lbl_selected_map.setText("Map đang dùng: chưa có file YAML trong folder")
        self.append_log(f"Refresh map list: tìm thấy {len(files)} file.")
    def on_selected_map_changed(self):
        fpath = self.cmb_map_files.currentData()
        if fpath: self.selected_map_file = fpath; self.lbl_selected_map.setText(f"Map đang dùng: {fpath}")

    def ros_prefix(self):
        return f"source '{ROS_SETUP}' && source '{WS_SETUP}' && "
    def get_selected_odom_source(self): return self.cmb_odom_source.currentData() or "esp"
    def get_selected_odom_topic(self): return "/odom"
    def on_odom_source_changed(self):
        self.lbl_odom_status.setText(f"Odom source: {self.get_selected_odom_source()} -> /odom")
    def on_operation_mode_changed(self):
        is_loc = self.cmb_operation_mode.currentText() == "Localization"
        self.cmb_localization_backend.setEnabled(is_loc)
        if is_loc: self.chk_auto_nav2.setChecked(False); self.chk_auto_nav2.setEnabled(False)
        else: self.chk_auto_nav2.setEnabled(True)

    def refresh_serial_ports(self):
        detected_esp, detected_lidar, method = _detect_esp_lidar_ports()
        all_ports = _list_serial_ports()
        standard_ports = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyACM0", "/dev/ttyACM1", "/dev/esp32", "/dev/lidar"]
        for sp in standard_ports:
            if sp not in all_ports and os.path.exists(sp):
                all_ports.append(sp)

        curr_esp = self.cmb_esp_port.currentText().strip() if hasattr(self, "cmb_esp_port") else ""
        curr_lidar = self.cmb_lidar_port.currentText().strip() if hasattr(self, "cmb_lidar_port") else ""

        self.cmb_esp_port.blockSignals(True); self.cmb_lidar_port.blockSignals(True)
        self.cmb_esp_port.clear(); self.cmb_lidar_port.clear()

        combined = []
        for p in [detected_esp, detected_lidar] + all_ports + standard_ports:
            if p and p not in combined:
                combined.append(p)

        for p in combined:
            self.cmb_esp_port.addItem(p)
            self.cmb_lidar_port.addItem(p)

        # Tự động chọn cổng phát hiện tốt nhất
        if detected_esp:
            self.cmb_esp_port.setCurrentText(detected_esp)
        if detected_lidar:
            self.cmb_lidar_port.setCurrentText(detected_lidar)

        # Đảm bảo ESP và LiDAR không bị trùng cùng 1 cổng vật lý
        real_esp = os.path.realpath(self.cmb_esp_port.currentText().strip())
        real_lidar = os.path.realpath(self.cmb_lidar_port.currentText().strip())
        if real_esp and real_esp == real_lidar:
            for p in combined:
                if os.path.realpath(p) != real_esp:
                    self.cmb_lidar_port.setCurrentText(p)
                    break

        self.cmb_esp_port.blockSignals(False); self.cmb_lidar_port.blockSignals(False)
        if hasattr(self, "append_log"):
            self.append_log(f"Tự động nhận diện cổng ({method}): ESP={self.cmb_esp_port.currentText()}, LiDAR={self.cmb_lidar_port.currentText()}")

    def swap_serial_ports(self):
        esp = self.cmb_esp_port.currentText().strip()
        lidar = self.cmb_lidar_port.currentText().strip()
        self.cmb_esp_port.setCurrentText(lidar); self.cmb_lidar_port.setCurrentText(esp)
        self.append_log(f"Đã hoán đổi cổng: ESP={lidar}, LiDAR={esp}")

    def _map_base_stem_from_selected(self):
        stem = os.path.splitext(self.selected_map_file)[0] if self.selected_map_file else os.path.join(self.map_dir, "my_map")
        for suffix in (MAP_SUFFIX_AMCL, MAP_SUFFIX_LOCALIZATION):
            if stem.endswith(suffix): return stem[: -len(suffix)]
        return stem
    def _pick_existing_file(self, candidates):
        for path in candidates:
            if os.path.exists(path): return path
        return candidates[0]
    def get_localization_map_targets(self):
        base_stem = self._map_base_stem_from_selected()
        amcl_yaml_candidates = [f"{base_stem}{MAP_SUFFIX_AMCL}.yaml", f"{base_stem}{MAP_SUFFIX_AMCL}.yml", f"{base_stem}.yaml", f"{base_stem}.yml", f"{base_stem}{MAP_SUFFIX_LOCALIZATION}.yaml", f"{base_stem}{MAP_SUFFIX_LOCALIZATION}.yml"]
        if self.selected_map_file: amcl_yaml_candidates.insert(0, self.selected_map_file)
        amcl_yaml = self._pick_existing_file(amcl_yaml_candidates)
        slam_graph_candidates = [f"{base_stem}{MAP_SUFFIX_LOCALIZATION}", base_stem, f"{base_stem}{MAP_SUFFIX_AMCL}"]
        slam_graph = slam_graph_candidates[0]
        for candidate in slam_graph_candidates:
            if os.path.exists(candidate + ".posegraph") or os.path.exists(candidate + ".data"):
                slam_graph = candidate; break
        return amcl_yaml, slam_graph
    def ensure_map_selected(self, backend):
        amcl_yaml, slam_graph = self.get_localization_map_targets()
        if backend == "AMCL":
            if not os.path.exists(amcl_yaml):
                QMessageBox.warning(self, "Thiếu map AMCL", "Không tìm thấy file YAML map cho AMCL.\nHãy Save map trước hoặc chọn file map YAML hợp lệ."); return False
            return True
        if not (os.path.exists(slam_graph + ".posegraph") or os.path.exists(slam_graph + ".data")):
            QMessageBox.warning(self, "Thiếu map localization", "Không tìm thấy file posegraph/data cho SLAM localization.\nHãy Save map trước để tạo bản localization."); return False
        return True
    def is_layer1_running(self): return self.proc_mgr.is_running(self.LAYER1_REAL)
    def _write_temp_yaml(self, data, prefix):
        fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml"); os.close(fd)
        with open(path, "w", encoding="utf-8") as f: yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.temp_files.append(path); return path
    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as f: return yaml.safe_load(f)
    def build_nav2_params_for_selected_odom(self):
        data = self._load_yaml(NAV2_PARAMS_BASE)
        topic = self.get_selected_odom_topic()
        changed = _replace_key_recursive(data, "odom_topic", topic)

        # Tự động thay thế mọi đường dẫn chứa /home/.../ros2_ws thành đường dẫn thực tế của máy
        def _fix_user_paths(obj):
            if isinstance(obj, dict):
                return {k: _fix_user_paths(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [_fix_user_paths(v) for v in obj]
            elif isinstance(obj, str):
                import re
                return re.sub(r"/home/[^/]+/(?:ros2_ws|robot_ws)", WS_ROOT, obj)
            return obj

        data = _fix_user_paths(data)

        if "behavior_server" in data and "ros__parameters" in data["behavior_server"]:
            bs_params = data["behavior_server"]["ros__parameters"]
            bs_params.setdefault(
                "behavior_plugins",
                ["spin", "backup", "custom_backup", "drive_on_heading", "assisted_teleop", "wait"],
            )
            bs_params.setdefault("spin", {"plugin": "nav2_behaviors/Spin"})
            bs_params.setdefault("backup", {"plugin": "nav2_behaviors/BackUp"})
            if "custom_backup" in bs_params.get("behavior_plugins", []):
                bs_params.setdefault("custom_backup", {"plugin": "nav2_custom_plugins/CustomBackUp"})
            bs_params.setdefault("drive_on_heading", {"plugin": "nav2_behaviors/DriveOnHeading"})
            bs_params.setdefault("wait", {"plugin": "nav2_behaviors/Wait"})
            bs_params.setdefault("assisted_teleop", {"plugin": "nav2_behaviors/AssistedTeleop"})

        temp_path = self._write_temp_yaml(data, "nav2_gui_v3_")
        self.append_log(f"Tạo params Nav2 tạm: {temp_path} (odom_topic='{topic}', cập nhật {changed} vị trí, giữ cấu hình BT/Recovery từ YAML)")
        return temp_path
    def build_slam_mapping_params(self):
        data = self._load_yaml(SLAM_MAPPING_PARAMS_BASE)
        if "slam_toolbox" not in data: data["slam_toolbox"] = {"ros__parameters": {}}
        if "ros__parameters" not in data["slam_toolbox"]: data["slam_toolbox"]["ros__parameters"] = {}
        data["slam_toolbox"]["ros__parameters"]["map_topic"] = "/map"
        temp_path = self._write_temp_yaml(data, "slam_mapping_gui_v3_")
        self.append_log(f"Tạo SLAM mapping params tạm: {temp_path} (map_topic=/map)"); return temp_path
    def build_slam_localization_params(self, map_graph_stem=None):
        data = self._load_yaml(SLAM_LOCALIZATION_PARAMS_BASE)
        if "slam_toolbox" not in data: data["slam_toolbox"] = {"ros__parameters": {}}
        if "ros__parameters" not in data["slam_toolbox"]: data["slam_toolbox"]["ros__parameters"] = {}
        params = data["slam_toolbox"]["ros__parameters"]
        params["mode"] = "localization"; params["map_file_name"] = map_graph_stem or _map_stem_from_yaml(self.selected_map_file)
        temp_path = self._write_temp_yaml(data, "slam_loc_gui_v3_")
        self.append_log(f"Tạo params SLAM localization tạm: {temp_path}"); return temp_path

    def start_platform(self):
        esp_port = self.cmb_esp_port.currentText().strip() if hasattr(self, "cmb_esp_port") else ""
        lidar_port = self.cmb_lidar_port.currentText().strip() if hasattr(self, "cmb_lidar_port") else ""
        if not esp_port or not lidar_port:
            detected_esp, detected_lidar, detect_method = _detect_esp_lidar_ports()
            esp_port = esp_port or detected_esp
            lidar_port = lidar_port or detected_lidar
        else:
            detect_method = "user_selection"
        self.append_log(f"Khởi động Serial ({detect_method}): ESP={esp_port}, LiDAR={lidar_port}")
        self.proc_mgr.start(
            self.LAYER1_REAL,
            self.ros_prefix()
            + f"ros2 launch mo_hinh real_odom.launch.py odom_source:={self.get_selected_odom_source()} use_sim_time:=false hoverboard_port:={shlex.quote(esp_port)} esp_port:={shlex.quote(esp_port)} lidar_port:={shlex.quote(lidar_port)}",
        )
    def stop_platform(self): self.proc_mgr.stop(self.LAYER1_REAL)
    def start_slam_mapping(self):
        self.append_log("Layer2 mode: SLAM Mapping")
        self.proc_mgr.stop(self.AMCL); self.proc_mgr.stop(self.SLAM_LOCALIZATION)
        slam_params = self.build_slam_mapping_params()
        self.proc_mgr.start(self.SLAM_MAPPING, self.ros_prefix() + f"ros2 launch mo_hinh real_slam.launch.py use_sim_time:=false slam_params_file:='{slam_params}' use_rviz:=true")
    def start_localization_backend(self):
        self.append_log(f"Layer2 mode: Localization ({self.cmb_localization_backend.currentText()})")
        backend = self.cmb_localization_backend.currentText()
        if not self.ensure_map_selected(backend): return
        amcl_yaml, slam_graph = self.get_localization_map_targets()
        self.proc_mgr.stop(self.SLAM_MAPPING)
        if backend == "SLAM Toolbox":
            self.proc_mgr.stop(self.AMCL)
            slam_loc_params = self.build_slam_localization_params(slam_graph)
            self.proc_mgr.start(self.SLAM_LOCALIZATION, self.ros_prefix() + f"ros2 launch mo_hinh real_localization_nav2.launch.py localization_mode:=slam_toolbox use_sim_time:=false map_graph:='{slam_graph}' slam_params_file:='{slam_loc_params}' use_rviz:=true")
            self.append_log(f"Localization backend=SLAM Toolbox -> map_graph: {slam_graph}")
        else:
            self.proc_mgr.stop(self.SLAM_LOCALIZATION)
            nav2_params = self.build_nav2_params_for_selected_odom()
            self.proc_mgr.start(self.AMCL, self.ros_prefix() + f"ros2 launch mo_hinh real_localization_nav2.launch.py localization_mode:=amcl use_sim_time:=false map_yaml:='{amcl_yaml}' nav2_params_file:='{nav2_params}' use_rviz:=true")
            self.append_log(f"Localization backend=AMCL -> map_yaml: {amcl_yaml}")
    def start_operation_mode(self):
        if self.cmb_operation_mode.currentText() == "SLAM Mapping": self.start_slam_mapping()
        else: self.start_localization_backend()
    def switch_operation_mode(self):
        self.append_log("Switch Layer 2 mode..."); self.stop_operation_mode(); QTimer.singleShot(600, self.start_operation_mode)
    def stop_operation_mode(self): self.proc_mgr.stop(self.SLAM_MAPPING); self.proc_mgr.stop(self.SLAM_LOCALIZATION); self.proc_mgr.stop(self.AMCL)
    def start_nav2(self):
        if self.cmb_operation_mode.currentText() == "Localization":
            self.append_log("Localization mode (real) đã bao gồm Nav2, bỏ qua Start Nav2 riêng."); return
        nav2_params = self.build_nav2_params_for_selected_odom()
        self.proc_mgr.start(self.NAV2, self.ros_prefix() + f"ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:='{nav2_params}'")
    def stop_nav2(self): self.proc_mgr.stop(self.NAV2)
    def _build_ai_launch_cmd(self, enable_cameras: bool, enable_obstacles: bool, enable_ai_nodes: bool) -> str:
        yolo_dev = self.cmb_yolo_device.currentText().strip() or "cuda:0"
        pose_dev = self.cmb_pose_device.currentText().strip() or "cpu"
        front_cfg = os.path.join(DEFAULT_MAP_DIR, "cam_truoc.yaml")
        rear_cfg = os.path.join(DEFAULT_MAP_DIR, "cam_sau.yaml")
        return (
            self.ros_prefix()
            + "ros2 launch robot_bringup ai_perception.launch.py"
            + f" enable_cameras:={_bool_to_ros(enable_cameras)}"
            + f" enable_obstacle_detectors:={_bool_to_ros(enable_obstacles)}"
            + f" enable_ai_nodes:={_bool_to_ros(enable_ai_nodes)}"
            + f" front_obstacle_config:={shlex.quote(front_cfg)}"
            + f" rear_obstacle_config:={shlex.quote(rear_cfg)}"
            + f" yolo_device:={shlex.quote(yolo_dev)}"
            + f" pose_device:={shlex.quote(pose_dev)}"
        )
    def start_cameras(self):
        if self.proc_mgr.is_running(self.AI_PERCEPTION):
            self.append_log("AI Perception đang chạy cùng camera/vật cản; bỏ qua Start Cam riêng để tránh trùng thiết bị.")
            return
        enable_obs = self.chk_obstacle_detectors.isChecked()
        cmd = self._build_ai_launch_cmd(enable_cameras=True, enable_obstacles=enable_obs, enable_ai_nodes=False)
        self.proc_mgr.start(self.CAMERAS, cmd)
    def stop_cameras(self): self.proc_mgr.stop(self.CAMERAS)
    def launch_obstacle_debugger(self):
        self.proc_mgr.start(
            self.OBSTACLE_DEBUGGER,
            self.ros_prefix() + "ros2 run depth_obstacle_detector debugger_gui",
        )
    def start_ai_perception(self):
        if self.proc_mgr.is_running(self.CAMERAS):
            enable_cams = False
            enable_obs = False
            self.append_log("Cam + Vật cản đang chạy riêng; Start AI chỉ bật node AI (YOLO / HandWave / RoughLocalizer).")
        else:
            enable_cams = self.chk_ai_cameras.isChecked()
            enable_obs = self.chk_obstacle_detectors.isChecked()
        cmd = self._build_ai_launch_cmd(enable_cameras=enable_cams, enable_obstacles=enable_obs, enable_ai_nodes=True)
        self.proc_mgr.start(self.AI_PERCEPTION, cmd)
    def stop_ai_perception(self): self.proc_mgr.stop(self.AI_PERCEPTION)
    def start_backend(self):
        self.proc_mgr.start(self.BACKEND, self.ros_prefix() + "ros2 launch robot_bringup backend.launch.py")
    def stop_backend(self): self.proc_mgr.stop(self.BACKEND)
    def start_kiosk(self):
        self.proc_mgr.start(self.KIOSK_UI, self.ros_prefix() + "ros2 launch robot_ui kiosk.launch.py windowed:=true hide_cursor:=false")
    def stop_kiosk(self): self.proc_mgr.stop(self.KIOSK_UI)
    def cancel_nav2_goal(self):
        ros_node = self.get_ros_node()
        if ros_node is not None:
            ros_node.cancel_nav_goal()
            self.append_log("⛔ Đã kích hoạt DỪNG KHẨN CẤP / HỦY GOAL Nav2!")
        else:
            self.append_log("ROS Interface chưa sẵn sàng.")
    def kill_node(self):
        pattern = self.edt_kill_node.text().strip()
        if not pattern: QMessageBox.warning(self, "Thiếu tên node", "Bạn cần nhập tên node/pattern."); return
        self.append_log(f"Kill node/pattern: {pattern}"); self.run_one_shot_command("kill_node", self.ros_prefix() + f"pkill -f {shlex.quote(pattern)}")
    def start_all(self): self.start_platform(); QTimer.singleShot(1200, self._start_all_layer2)
    def _start_all_layer2(self): self.start_operation_mode()
    def stop_all(self): self.proc_mgr.stop_all(); self.stop_one_shot_processes()

    def save_map(self):
        raw_name = self.edt_save_map_name.text().strip()
        if not raw_name: QMessageBox.warning(self, "Thiếu tên", "Bạn chưa nhập tên map cần lưu."); return
        if not (self.proc_mgr.is_running(self.SLAM_MAPPING) or self.proc_mgr.is_running(self.SLAM_LOCALIZATION)):
            QMessageBox.warning(self, "SLAM chưa chạy", "Bạn cần chạy SLAM trước khi lưu map."); return
        target_stem = os.path.splitext(os.path.join(self.map_dir, raw_name) if not os.path.isabs(raw_name) else raw_name)[0]
        target_dir = os.path.dirname(target_stem)
        if target_dir: os.makedirs(target_dir, exist_ok=True)
        amcl_stem = f"{target_stem}{MAP_SUFFIX_AMCL}"; localization_stem = f"{target_stem}{MAP_SUFFIX_LOCALIZATION}"
        serialize_req = shlex.quote(f'{{filename: "{localization_stem}"}}')
        cmd = (self.ros_prefix() + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(amcl_stem)}" + " && " + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(localization_stem)}" + " && " + f"ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph {serialize_req}")
        self.append_log(f"Lưu map 2 bản: AMCL='{amcl_stem}.yaml/.pgm', Localization='{localization_stem}.yaml/.pgm + .posegraph/.data'")
        def _after_save():
            self.refresh_map_list()
            selected = amcl_stem + ".yaml" if os.path.exists(amcl_stem + ".yaml") else amcl_stem + ".yml"
            if os.path.exists(selected):
                self.selected_map_file = selected
                idx = self.cmb_map_files.findData(selected)
                if idx >= 0: self.cmb_map_files.setCurrentIndex(idx)
                self.lbl_selected_map.setText(f"Map đang dùng: {selected}"); self.append_log(f"Auto chọn map AMCL: {selected}")
        self.run_one_shot_command("save_map", cmd, _after_save)

    def run_one_shot_command(self, name, cmd, done_callback=None):
        def _worker():
            self.append_log(f"[{name}] RUN\nCMD: {cmd}"); proc = None
            try:
                proc = subprocess.Popen(["bash", "-c", cmd], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
                with self.one_shot_lock: self.one_shot_procs.append(proc)
                for line in iter(proc.stdout.readline, ""):
                    if line == "" and proc.poll() is not None: break
                    self.append_log(f"[{name}] {line.rstrip()}")
                ret = proc.wait(); self.append_log(f"[{name}] exit_code={ret}")
            except Exception as exc:
                self.append_log(f"[{name}] error: {exc}")
            finally:
                if proc is not None:
                    with self.one_shot_lock:
                        if proc in self.one_shot_procs: self.one_shot_procs.remove(proc)
            if done_callback and not self.is_shutting_down: QTimer.singleShot(0, done_callback)
        threading.Thread(target=_worker, daemon=True).start()

    def _set_status_label(self, label, name, is_on):
        label.setText(f"{name}: {'ON' if is_on else 'OFF'}"); label.setStyleSheet(f"color: {'#16a34a' if is_on else '#dc2626'}; font-weight: bold;")
    def refresh_status(self):
        platform_on = self.proc_mgr.is_running(self.LAYER1_REAL)
        mode_on = self.proc_mgr.is_running(self.SLAM_MAPPING) or self.proc_mgr.is_running(self.SLAM_LOCALIZATION) or self.proc_mgr.is_running(self.AMCL)
        nav2_on = self.proc_mgr.is_running(self.NAV2)
        if self.cmb_operation_mode.currentText() == "Localization":
            nav2_on = nav2_on or self.proc_mgr.is_running(self.SLAM_LOCALIZATION) or self.proc_mgr.is_running(self.AMCL)
        ai_on = self.proc_mgr.is_running(self.AI_PERCEPTION)
        cam_on = self.proc_mgr.is_running(self.CAMERAS)
        backend_on = self.proc_mgr.is_running(self.BACKEND)
        kiosk_on = self.proc_mgr.is_running(self.KIOSK_UI)
        self._set_status_label(self.lbl_platform_status, "Layer1 Odom", platform_on)
        self._set_status_label(self.lbl_mode_status, "Layer2 Mode", mode_on)
        self._set_status_label(self.lbl_nav2_status, "Nav2", nav2_on)
        self._set_status_label(self.lbl_cameras_status, "Cam + Vật cản", cam_on)
        self._set_status_label(self.lbl_ai_status, "AI Perception", ai_on)
        self._set_status_label(self.lbl_backend_status, "Backend", backend_on)
        self._set_status_label(self.lbl_kiosk_status, "Kiosk UI", kiosk_on)
        self.lbl_odom_status.setText(f"Odom source: {self.get_selected_odom_source()} -> /odom")
        self._update_sensor_debug_ui()

    def _stream_state(self, stats, key, now, timeout=2.0):
        entry = stats.get(key)
        if not entry:
            return False, 0.0, None
        age = now - entry["last_ts"]
        if age > timeout:
            return False, 0.0, entry.get("extra")
        return True, entry.get("hz", 0.0), entry.get("extra")

    def _update_sensor_debug_ui(self):
        if not self.ros_node:
            return
        snap = self.ros_node.get_debug_snapshot()
        now = snap["now"]
        stats = snap["stats"]

        lidar_ok, lidar_hz, lidar_extra = self._stream_state(stats, "lidar_scan", now)
        odom_ok, odom_hz, odom_extra = self._stream_state(stats, "odom", now)
        cmd_ok, _, cmd_extra = self._stream_state(stats, "cmd_vel", now, timeout=1.0)
        cf_ok, cf_hz, _ = self._stream_state(stats, "cam_front_depth", now)
        cr_ok, cr_hz, _ = self._stream_state(stats, "cam_rear_depth", now)
        ca_rgb_ok, ca_rgb_hz, _ = self._stream_state(stats, "cam_ai_color", now)
        ca_dep_ok, ca_dep_hz, _ = self._stream_state(stats, "cam_ai_depth", now)
        of_ok, of_hz, of_extra = self._stream_state(stats, "obs_front", now)
        or_ok, or_hz, or_extra = self._stream_state(stats, "obs_rear", now)
        rs_ok, _, rs_extra = self._stream_state(stats, "robot_status", now, timeout=3.0)

        odom_tf_ok = snap["odom_tf_ok"]
        map_tf_ok = snap["map_tf_ok"]
        tf_short = "map+odom" if (map_tf_ok and odom_tf_ok) else ("odom" if odom_tf_ok else "MISS")
        self.lbl_sensor_summary.setText(
            f"LiDAR: {lidar_hz:.1f}Hz | Odom: {odom_hz:.1f}Hz | TF: {tf_short}"
        )
        f_min = f"{of_extra['min_dist']:.2f}" if (of_ok and of_extra and of_extra.get("min_dist") is not None) else ("inf" if of_ok else "-")
        r_min = f"{or_extra['min_dist']:.2f}" if (or_ok and or_extra and or_extra.get("min_dist") is not None) else ("inf" if or_ok else "-")
        ai_on_flag = ca_rgb_ok or ca_dep_ok
        self.lbl_cam_summary.setText(
            f"Cam F/R/AI: {'ON' if cf_ok else '-'}/{'ON' if cr_ok else '-'}/{'ON' if ai_on_flag else '-'} | Obs(m): F={f_min} R={r_min}"
        )

        if hasattr(self, "lbl_dbg_lidar"):
            l_min = f"{lidar_extra['min_dist']:.2f}m" if (lidar_ok and lidar_extra and lidar_extra.get("min_dist") is not None) else "--"
            l_cnt = lidar_extra.get("count", 0) if (lidar_ok and lidar_extra) else 0
            self.lbl_dbg_lidar.setText(
                f"LiDAR (/scan): {'ON' if lidar_ok else 'OFF'} | {lidar_hz:.1f} Hz | gần nhất: {l_min} ({l_cnt} tia)"
            )
            self.lbl_dbg_lidar.setStyleSheet(f"color: {'#16a34a' if lidar_ok else '#dc2626'}; font-weight: bold;")

            ovx, owz = odom_extra if (odom_ok and odom_extra) else (0.0, 0.0)
            cvx, cwz = cmd_extra if (cmd_ok and cmd_extra) else (0.0, 0.0)
            self.lbl_dbg_odom_cmd.setText(
                f"Odom ({odom_hz:.1f}Hz): vx={ovx:+.2f} wz={owz:+.2f} | CmdVel: vx={cvx:+.2f} wz={cwz:+.2f}"
            )
            self.lbl_dbg_tf.setText(
                f"TF: map->odom [{'OK' if map_tf_ok else 'MISS'}] | odom->base_footprint [{'OK' if odom_tf_ok else 'MISS'}]"
            )
            self.lbl_dbg_tf.setStyleSheet(
                f"color: {'#16a34a' if (map_tf_ok and odom_tf_ok) else ('#d97706' if odom_tf_ok else '#dc2626')}; font-weight: bold;"
            )
            if rs_ok and rs_extra:
                self.lbl_dbg_robot_status.setText(
                    f"/robot_status: Pin {rs_extra['battery_percent']:.0f}% | Nước {rs_extra['water_liter']:.1f}L | lidar_ok={rs_extra['lidar_ok']} odom_ok={rs_extra['odom_ok']}"
                )
            else:
                self.lbl_dbg_robot_status.setText("/robot_status: Chưa có dữ liệu (cần Start Backend)")

            self.lbl_dbg_cam_front.setText(f"Cam Trước Depth: {'ON (' + f'{cf_hz:.1f}Hz)' if cf_ok else 'OFF'}")
            self.lbl_dbg_cam_front.setStyleSheet(f"color: {'#16a34a' if cf_ok else '#64748b'}; font-weight: bold;")
            self.lbl_dbg_cam_rear.setText(f"Cam Sau Depth: {'ON (' + f'{cr_hz:.1f}Hz)' if cr_ok else 'OFF'}")
            self.lbl_dbg_cam_rear.setStyleSheet(f"color: {'#16a34a' if cr_ok else '#64748b'}; font-weight: bold;")
            self.lbl_dbg_cam_ai.setText(
                f"Cam AI RGB/Depth: {'ON' if ca_rgb_ok else 'OFF'} ({ca_rgb_hz:.1f}Hz) / {'ON' if ca_dep_ok else 'OFF'} ({ca_dep_hz:.1f}Hz)"
            )
            self.lbl_dbg_cam_ai.setStyleSheet(f"color: {'#16a34a' if ai_on_flag else '#64748b'}; font-weight: bold;")
            self.lbl_dbg_obs_front.setText(f"/scan_obstacles: {'ON (' + f'{of_hz:.1f}Hz)' if of_ok else 'OFF'} | min={f_min}")
            self.lbl_dbg_obs_rear.setText(f"/rear_scan_obstacle: {'ON (' + f'{or_hz:.1f}Hz)' if or_ok else 'OFF'} | min={r_min}")
            self._update_battery_debug_ui(stats, now)
            self._update_ai_debug_ui(stats, now)

    def _update_battery_debug_ui(self, stats, now):
        hb_ok, hb_hz, hb_extra = self._stream_state(stats, "bat_hoverboard", now, timeout=5.0)
        bs_ok, bs_hz, bs_extra = self._stream_state(stats, "bat_state", now, timeout=5.0)
        bat_ok, bat_hz, bat_extra = (hb_ok, hb_hz, hb_extra) if hb_ok else (bs_ok, bs_hz, bs_extra)

        if bat_ok and bat_extra:
            pct = bat_extra["percent"]
            smooth_v = bat_extra["smooth_v"]
            raw_v = bat_extra["raw_v"]
            src = bat_extra["source"]
            color = "#16a34a" if pct >= 50.0 else ("#d97706" if pct >= 20.0 else "#dc2626")
            low_tag = " ⚠️ YẾU" if pct < 20.0 else ""
            self.lbl_battery_summary.setText(f"Pin 36V (3×12V 30Ah): {pct:.0f}% ({smooth_v:.2f}V){low_tag}")
            self.lbl_battery_summary.setStyleSheet(f"font-size: 11px; font-weight: bold; color: {color};")
            self.lbl_dbg_battery.setText(
                f"Ắc quy 36V ({bat_hz:.1f}Hz): {pct:.1f}% | {smooth_v:.2f}V (tức thời {raw_v:.2f}V | {src}){low_tag}"
            )
            self.lbl_dbg_battery.setStyleSheet(f"font-size: 11px; font-weight: bold; color: {color};")
        else:
            self.lbl_battery_summary.setText("Pin 36V (3×12V 30Ah): --% (-- V)")
            self.lbl_battery_summary.setStyleSheet("font-size: 11px; font-weight: bold; color: #334155;")
            self.lbl_dbg_battery.setText("Ắc quy 36V (3×12V 30Ah): Chưa có điện áp (/hoverboard/battery_voltage hoặc /battery)")
            self.lbl_dbg_battery.setStyleSheet("font-size: 11px; color: #64748b;")

    def _update_ai_debug_ui(self, stats, now):
        yolo_ok, yolo_hz, yolo_extra = self._stream_state(stats, "yolo_tracking", now)
        tp_ok, tp_hz, tp_extra = self._stream_state(stats, "tracked_people", now)
        wave_ok, wave_hz, wave_extra = self._stream_state(stats, "wave_detected", now)
        wstat_ok, _, wstat_extra = self._stream_state(stats, "wave_status", now, timeout=3.0)

        if yolo_ok and yolo_extra is not None:
            self.lbl_dbg_yolo.setText(
                f"YOLO (/yolo/tracking): ON ({yolo_hz:.1f}Hz) | {yolo_extra['persons']} người ({yolo_extra['total']} obj)"
            )
            self.lbl_dbg_yolo.setStyleSheet(
                f"color: {'#16a34a' if yolo_extra['persons'] > 0 else '#0f172a'}; font-weight: bold;"
            )
        else:
            self.lbl_dbg_yolo.setText(
                "YOLO (/yolo/tracking): OFF" if HAS_YOLO_MSG else "YOLO (/yolo/tracking): Thiếu yolo_msgs"
            )
            self.lbl_dbg_yolo.setStyleSheet("color: #64748b; font-weight: bold;")

        is_waving = bool(wave_extra) if wave_ok else False
        wstat_txt = f" ({wstat_extra})" if (wstat_ok and wstat_extra) else ""
        if wave_ok:
            self.lbl_dbg_wave.setText(
                f"Vẫy tay (/pose/wave_detected): {'ĐANG VẪY' if is_waving else 'Không'} ({wave_hz:.1f}Hz){wstat_txt}"
            )
            self.lbl_dbg_wave.setStyleSheet(
                f"color: {'#dc2626' if is_waving else '#16a34a'}; font-weight: bold;"
            )
        else:
            self.lbl_dbg_wave.setText("Vẫy tay (/pose/wave_detected): OFF")
            self.lbl_dbg_wave.setStyleSheet("color: #64748b; font-weight: bold;")

        people = tp_extra.get("people", []) if (tp_ok and tp_extra) else []
        if tp_ok and tp_extra is not None:
            n_people = len(people)
            cal_txt = "cal=OK" if tp_extra.get("calibrated") else "uncal"
            self.lbl_dbg_people_count.setText(
                f"Đã phát hiện người (/perception/tracked_people): {n_people} người ({tp_hz:.1f}Hz | {cal_txt})"
            )
            self.lbl_dbg_people_count.setStyleSheet(
                f"color: {'#16a34a' if n_people > 0 else '#0f172a'}; font-weight: bold;"
            )
            if people:
                frame_id = tp_extra.get("frame_id", "base")
                lines = []
                for p in people[:5]:
                    dist = f"{p['dist_xy']:.2f}m" if p["dist_xy"] > 0.0 else "--"
                    depth = f"{p['depth_m']:.2f}m" if p["depth_valid"] else "--"
                    pos = f"({p['x']:+.2f},{p['y']:+.2f})" if p["pos_valid"] else "(invalid)"
                    w_lbl = "VẪY" if p["is_waving"] else ("không" if p["wave_valid"] else "?")
                    lines.append(
                        f"• ID {p['track_id']}: cách {dist} (depth {depth}) | {frame_id}={pos} | conf={p['conf']:.2f} | vẫy={w_lbl}"
                    )
                if n_people > 5:
                    lines.append(f"... (+{n_people - 5} người khác)")
                self.lbl_dbg_people_detail.setText("\n".join(lines))
            else:
                self.lbl_dbg_people_detail.setText("Chi tiết: Không có người trong khung hình")

            wave_tag = " | ĐANG VẪY!" if (is_waving or any(p["is_waving"] for p in people)) else ""
            self.lbl_ai_status.setText(f"{self.lbl_ai_status.text()} | Người: {n_people}{wave_tag}")
        else:
            self.lbl_dbg_people_count.setText(
                "Đã phát hiện người (/perception/tracked_people): OFF"
                if HAS_TRACKED_PERSON_MSG
                else "Đã phát hiện người: Thiếu robot_interfaces"
            )
            self.lbl_dbg_people_count.setStyleSheet("color: #64748b; font-weight: bold;")
            self.lbl_dbg_people_detail.setText("Chi tiết: --")
            if yolo_ok and yolo_extra is not None:
                self.lbl_ai_status.setText(f"{self.lbl_ai_status.text()} | YOLO người: {yolo_extra['persons']}")

    def stop_one_shot_processes(self):
        with self.one_shot_lock:
            procs = list(self.one_shot_procs)
            self.one_shot_procs = []

        if not procs:
            return

        for proc in procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass

        time.sleep(0.15)
        for proc in procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

    def force_kill_known_ros_processes(self):
        # Direct pkill without login bash subshells to avoid freezing the GUI event loop
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
            "yolo_ros/debug_node",
            "hand_wave_detector",
            "rough_localizer",
            "backend.launch.py",
            "config_manager_node",
            "system_monitor_node",
            "navigation_manager_node",
            "mission_controller_node",
            "rviz2",
        ]
        combined = "|".join(patterns)
        try:
            subprocess.run(
                ["pkill", "-TERM", "-f", combined],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.5,
            )
        except Exception:
            pass

        time.sleep(0.15)

        try:
            subprocess.run(
                ["pkill", "-KILL", "-f", combined],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.5,
            )
        except Exception:
            pass

        # Stop ROS 2 daemon to refresh discovery
        try:
            subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0)
        except Exception:
            pass

    def manual_cleanup(self):
        self.stop_all()
        self.force_kill_known_ros_processes()
        self.append_log("Đã hoàn tất dọn dẹp sạch sẽ toàn bộ các tiến trình ROS2.")

    def cleanup_temp_files(self):
        for path in self.temp_files:
            try:
                if os.path.exists(path):
                    os.remove(path)
            except Exception:
                pass
        self.temp_files = []

    def closeEvent(self, event):
        if self.is_shutting_down:
            event.accept()
            return
        self.is_shutting_down = True

        # 1. Hide window immediately so the desktop UI closes instantly
        self.hide()
        QApplication.processEvents()

        # 2. Stop GUI status timers
        try:
            if hasattr(self, "status_timer") and self.status_timer.isActive():
                self.status_timer.stop()
        except Exception:
            pass

        # 3. Stop background managed processes
        try:
            self.stop_all()
        except Exception:
            pass

        # 4. Clean up lingering ROS 2 launch processes
        try:
            self.force_kill_known_ros_processes()
        except Exception:
            pass

        # 5. Clean up temporary files
        try:
            self.cleanup_temp_files()
        except Exception:
            pass

        # 6. Shut down ROS 2 node cleanly
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            if self.ros_spin_thread and self.ros_spin_thread.is_alive():
                self.ros_spin_thread.join(timeout=0.2)
        except Exception:
            pass

        try:
            if self.ros_node is not None:
                self.ros_node.destroy_node()
        except Exception:
            pass

        event.accept()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    # Enable responsive Ctrl+C in terminal:
    def sigint_handler(sig, frame):
        if window.is_shutting_down:
            os._exit(0)
        window.close()

    signal.signal(signal.SIGINT, sigint_handler)

    sig_timer = QTimer()
    sig_timer.timeout.connect(lambda: None)
    sig_timer.start(200)

    exit_code = app.exec_()
    os._exit(exit_code)


if __name__ == "__main__":
    main()