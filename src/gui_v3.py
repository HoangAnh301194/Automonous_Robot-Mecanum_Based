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

from PyQt5.QtCore import QPoint, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QFileDialog,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox,
    QPushButton, QRadioButton, QScrollArea, QSlider, QTextEdit,
    QVBoxLayout, QWidget,
)

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray, String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

def _is_esp32_port(port, baudrate=115200, timeout=1.0):
    if serial is None:
        return False
    ser = None
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        time.sleep(1.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        for _ in range(5):
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("V:") and "EL:" in line:
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

class RosInterface(Node):
    def __init__(self, log_callback=None, map_callback=None, pose_callback=None, scan_callback=None, dataenc_callback=None):
        super().__init__("desktop_nav_gui_v3")
        self.log_callback = log_callback
        self.map_callback = map_callback
        self.pose_callback = pose_callback
        self.scan_callback = scan_callback
        self.dataenc_callback = dataenc_callback
        self.last_scan_ui_pub = 0.0
        self.last_tf_warn_time = 0.0
        self.target_map_frame = "map"
        self.target_base_frame = "base_footprint"
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        self._map_sub = self.create_subscription(OccupancyGrid, "/map1", self.on_map, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=10))
        self.dataenc_sub = self.create_subscription(Int32MultiArray, "/dataenc", self.on_dataenc, 20)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.waypoints_pub = self.create_publisher(String, "/nhiemvuboss/waypoints_json", 10)
        self.edited_map_pub = self.create_publisher(OccupancyGrid, "/map", QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.pose_timer = self.create_timer(0.05, self.publish_pose_from_tf)

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

    def publish_pose_from_tf(self):
        if not self.pose_callback:
            return
        if self.target_map_frame not in self.tf_buffer.all_frames_as_string():
            return
        if not self.tf_buffer.can_transform(self.target_map_frame, self.target_base_frame, Time(), timeout=Duration(seconds=0.02)):
            return
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.target_map_frame, self.target_base_frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException as exc:
            self._warn_tf(f"TF pose lookup lỗi: {exc}")
            return
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        self.pose_callback("tf2", t.x, t.y, self._pose_to_xyyaw(q.x, q.y, q.z, q.w))

    def _scan_to_world_points(self, msg):
        ranges = msg.ranges
        if not ranges:
            return []
        if self.target_map_frame not in self.tf_buffer.all_frames_as_string():
            return []
        source_frame = msg.header.frame_id if msg.header.frame_id else self.target_base_frame
        scan_time = Time.from_msg(msg.header.stamp)
        try:
            tf_msg = self.tf_buffer.lookup_transform(self.target_map_frame, source_frame, scan_time, timeout=Duration(seconds=0.05))
        except TransformException:
            try:
                tf_msg = self.tf_buffer.lookup_transform(self.target_map_frame, source_frame, Time(), timeout=Duration(seconds=0.05))
            except TransformException as exc:
                self._warn_tf(f"TF scan lookup lỗi: {exc}")
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

    def on_scan(self, msg):
        if not self.scan_callback:
            return
        now = time.monotonic()
        if now - self.last_scan_ui_pub < 0.1:
            return
        self.last_scan_ui_pub = now
        self.scan_callback(msg, self._scan_to_world_points(msg))

    def on_dataenc(self, msg):
        if self.dataenc_callback:
            self.dataenc_callback(list(msg.data))

    def send_goal(self, x, y, yaw=0.0):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.log("NavigateToPose action server chưa sẵn sàng.")
            return False
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.log(f"Gửi goal Nav2: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        self.nav_to_pose_client.send_goal_async(goal_msg)
        return True

    def publish_edited_map(self, occ_grid_msg):
        self.edited_map_pub.publish(occ_grid_msg)

    def switch_map_subscription(self, new_topic):
        pass  # Không cần nữa - subscribe /map1 từ đầu

    def send_waypoint_batch(self, waypoints):
        msg = String()
        msg.data = json.dumps({"waypoints": waypoints}, ensure_ascii=False)
        self.waypoints_pub.publish(msg)
        self.log(f"Gửi batch waypoint cho nhiemvuboss: {len(waypoints)} điểm")

class MapEditor:
    def __init__(self, ros_node_getter, log_callback=None):
        self.get_ros_node = ros_node_getter
        self.log = log_callback or (lambda msg: None)
        self.original_map_msg = None
        self.edited_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.edit_enabled = False
        self.history = []
        self.redo_stack = []
        self._stroke_start = 0

    def load_from_grid(self, occ_grid):
        info = occ_grid.info
        self.map_width, self.map_height = info.width, info.height
        self.map_resolution = info.resolution
        self.map_origin_x, self.map_origin_y = info.origin.position.x, info.origin.position.y
        self.original_map_msg = occ_grid
        if self.edited_data is None or len(self.edited_data) != len(occ_grid.data):
            self.edited_data = list(occ_grid.data)
        if not self.edit_enabled:
            self.edited_data = list(occ_grid.data)

    def enable_edit(self):
        if self.original_map_msg is None:
            self.log("Chưa có bản đồ gốc để chỉnh sửa.")
            return False
        if self.edited_data is None or len(self.edited_data) != len(self.original_map_msg.data):
            self.edited_data = list(self.original_map_msg.data)
        self.edit_enabled = True
        self.history.clear()
        self.redo_stack.clear()
        self.log("Đã bật chế độ chỉnh sửa map.")
        return True

    def disable_edit(self):
        self.edit_enabled = False
        self.log("Đã tắt chế độ chỉnh sửa map.")

    def reset(self):
        if self.original_map_msg is not None:
            self.edited_data = list(self.original_map_msg.data)
            self.history.clear()
            self.redo_stack.clear()
            self.log("Đã reset map về bản gốc.")

    def paint_cell(self, mx, my, value, brush_radius=0):
        if not self.edit_enabled or self.edited_data is None:
            return False
        changed = False
        for dy in range(-brush_radius, brush_radius + 1):
            for dx in range(-brush_radius, brush_radius + 1):
                if brush_radius > 0 and dx * dx + dy * dy > brush_radius * brush_radius:
                    continue
                cx, cy = mx + dx, my + dy
                if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
                    idx = cx + cy * self.map_width
                    old_val = self.edited_data[idx]
                    if old_val != value and old_val != -1:
                        self.history.append((idx, old_val, value))
                        self.redo_stack.clear()
                        self.edited_data[idx] = value
                        changed = True
        return changed

    def paint_world(self, wx, wy, value, brush_radius_cells=0):
        if self.map_resolution <= 0:
            return False
        mx = int((wx - self.map_origin_x) / self.map_resolution)
        my = int((wy - self.map_origin_y) / self.map_resolution)
        return self.paint_cell(mx, my, value, brush_radius_cells)

    def start_stroke(self):
        self._stroke_start = len(self.history)

    def end_stroke(self):
        pass

    def undo(self):
        if not self.history:
            return False
        start = getattr(self, '_stroke_start', max(0, len(self.history) - 1))
        if len(self.history) <= start:
            group = list(self.history)
            self.history.clear()
        else:
            group = self.history[start:]
            self.history = self.history[:start]
        for idx, old_val, new_val in group:
            self.edited_data[idx] = old_val
            self.redo_stack.append((idx, old_val, new_val))
        self._stroke_start = len(self.history)
        self.log(f"Undo: {len(group)} ô.")
        return True

    def redo(self):
        if not self.redo_stack:
            return False
        idx, old_val, new_val = self.redo_stack.pop()
        self.edited_data[idx] = new_val
        self.history.append((idx, old_val, new_val))
        self.log("Redo.")
        return True

    def get_edited_grid(self):
        if self.original_map_msg is None or self.edited_data is None:
            return None
        msg = copy.deepcopy(self.original_map_msg)
        msg.data = list(self.edited_data)
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        return msg

    def publish_edited_map(self):
        node = self.get_ros_node()
        if node is None:
            self.log("ROS node chưa sẵn sàng.")
            return False
        msg = self.get_edited_grid()
        if msg is None:
            self.log("Không có dữ liệu map để publish.")
            return False
        node.publish_edited_map(msg)
        self.log("Đã publish bản đồ chỉnh sửa lên /map.")
        return True

class MapViewer(QLabel):
    def __init__(self, ros_node_getter, on_map_click=None, on_map_drag=None, parent=None):
        super().__init__(parent)
        self.get_ros_node = ros_node_getter
        self.on_map_click = on_map_click
        self.on_map_drag = on_map_drag
        self.setMinimumSize(260, 220)
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
        self.robot_x = self.robot_y = None
        self.robot_yaw = 0.0
        self.pose_source = "none"
        self.latest_scan = None
        self.scan_points_world = []
        self.scan_points_from_tf = False
        self.batch_waypoints = []
        self.is_dragging = False

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
        if scan_points_world is None:
            self.scan_points_from_tf = False
            self._recompute_scan_points()
        else:
            self.scan_points_from_tf = True
            self.scan_points_world = list(scan_points_world)
        self.update_display()

    def update_waypoint_batch(self, waypoints):
        self.batch_waypoints = list(waypoints)
        self.update_display()

    def _recompute_scan_points(self):
        self.scan_points_world = []
        if self.latest_scan is None or self.robot_x is None or self.robot_y is None:
            return
        ranges = self.latest_scan.ranges
        if not ranges:
            return
        step = max(1, len(ranges) // 720)
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < self.latest_scan.range_min or r > self.latest_scan.range_max:
                continue
            beam = self.latest_scan.angle_min + i * self.latest_scan.angle_increment + self.robot_yaw
            self.scan_points_world.append((self.robot_x + r * math.cos(beam), self.robot_y + r * math.sin(beam)))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_display()

    def update_display(self):
        if self.map_pixmap is None:
            return
        label_w, label_h = max(1, self.width()), max(1, self.height())
        scaled = self.map_pixmap.scaled(label_w, label_h, Qt.KeepAspectRatio, Qt.FastTransformation)
        self.scale_factor = scaled.width() / self.map_width
        self.offset_x = (label_w - scaled.width()) // 2
        self.offset_y = (label_h - scaled.height()) // 2
        canvas = QPixmap(label_w, label_h)
        canvas.fill(QColor("#1e1e1e"))
        painter = QPainter(canvas)
        painter.drawPixmap(self.offset_x, self.offset_y, scaled)
        if self.scan_points_world:
            painter.setPen(QPen(QColor(0, 255, 255), 1))
            for wx, wy in self.scan_points_world:
                sx, sy = self.world_to_widget(wx, wy)
                if sx is not None and 0 <= sx < label_w and 0 <= sy < label_h:
                    painter.drawPoint(sx, sy)
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
        painter.drawText(10, 20, "Click trái để gửi goal Nav2")
        painter.drawText(10, 38, f"Pose source: {self.pose_source}")
        painter.drawText(10, 56, f"/scan points: {len(self.scan_points_world)}")
        painter.end()
        self.setPixmap(canvas)

    def mousePressEvent(self, event):
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
        if self.is_dragging and self.on_map_drag is not None:
            world = self.widget_to_world(event.pos().x(), event.pos().y())
            if world is not None:
                self.on_map_drag(world[0], world[1])
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
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

    def stop(self, name):
        proc = self.processes.get(name)
        if not proc:
            self.log(f"[{name}] chưa chạy.")
            return
        if proc.poll() is None:
            self.log(f"[{name}] STOP")
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                for _ in range(3):
                    time.sleep(0.1)
                    if proc.poll() is not None:
                        break
                if proc.poll() is None:
                    os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except Exception as exc:
                self.log(f"[{name}] stop error: {exc}")
        else:
            self.log(f"[{name}] đã dừng.")

    def is_running(self, name):
        proc = self.processes.get(name)
        return bool(proc and proc.poll() is None)

    def stop_all(self):
        active = []
        for name, proc in list(self.processes.items()):
            if proc and proc.poll() is None:
                try:
                    pgid = os.getpgid(proc.pid)
                    active.append((name, proc, pgid))
                    os.killpg(pgid, signal.SIGTERM)
                    self.log(f"[{name}] STOP (SIGTERM)")
                except Exception:
                    pass

        if not active:
            return

        for _ in range(3):
            time.sleep(0.1)
            if all(proc.poll() is not None for _, proc, _ in active):
                break

        for name, proc, pgid in active:
            if proc.poll() is None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                    self.log(f"[{name}] FORCE KILL (SIGKILL)")
                except Exception:
                    pass

class MainWindow(QWidget):
    log_signal = pyqtSignal(str)
    map_signal = pyqtSignal(object)
    pose_signal = pyqtSignal(float, float, float, str)
    scan_signal = pyqtSignal(object, object)
    dataenc_signal = pyqtSignal(object)
    LAYER1_SIM, LAYER1_REAL = "layer1_sim", "layer1_real"
    SLAM_MAPPING, SLAM_LOCALIZATION = "slam_mapping", "slam_localization"
    AMCL, NAV2 = "amcl", "nav2"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control GUI v3 - Map Editor")
        self.resize(1200, 760)
        self.temp_files = []
        self.map_dir = _pick_first_existing([DEFAULT_MAP_DIR, MO_HINH_MAP_DIR, WS_ROOT])
        self.selected_map_file = ""
        self.batch_waypoint_mode = False
        self.batch_waypoints = []
        self.is_shutting_down = False
        self.one_shot_procs = []
        self.one_shot_lock = threading.Lock()
        self.map_mode = MAP_MODE_GOAL
        self.proc_mgr = ProcessManager(self.append_log)
        self.ros_node = None
        self.ros_spin_thread = None
        self.map_editor = None
        self.log_signal.connect(self._append_log_ui)
        self.map_signal.connect(self._on_new_map_ui)
        self.pose_signal.connect(self._on_new_pose_ui)
        self.scan_signal.connect(self._on_new_scan_ui)
        self.dataenc_signal.connect(self._on_new_dataenc_ui)
        self.init_ui()
        self.init_ros()
        self.refresh_map_list()
        self.map_editor = MapEditor(self.get_ros_node, self.append_log)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(500)

    def init_ui(self):
        self.rb_sim = QRadioButton("Simulation (Gazebo)")
        self.rb_real = QRadioButton("Robot thật")
        self.rb_real.setChecked(True)
        self.robot_mode_group = QButtonGroup(self)
        self.robot_mode_group.setExclusive(True)
        self.robot_mode_group.addButton(self.rb_sim)
        self.robot_mode_group.addButton(self.rb_real)
        self.rb_sim.toggled.connect(self.on_robot_mode_changed)
        self.rb_real.toggled.connect(self.on_robot_mode_changed)
        self.cmb_odom_source = QComboBox()
        self.cmb_odom_source.addItem("Hoverboard ros2_control", "hoverboard")
        self.cmb_odom_source.addItem("ESP encoder odom", "esp")
        self.cmb_odom_source.addItem("LiDAR RF2O odom", "rf2o")
        self.cmb_odom_source.currentIndexChanged.connect(self.on_odom_source_changed)
        self.cmb_operation_mode = QComboBox()
        self.cmb_operation_mode.addItems(["SLAM Mapping", "Localization"])
        self.cmb_operation_mode.currentIndexChanged.connect(self.on_operation_mode_changed)
        self.cmb_localization_backend = QComboBox()
        self.cmb_localization_backend.addItems(["SLAM Toolbox", "AMCL"])
        self.chk_auto_nav2 = QCheckBox("Tự start Nav2 khi Start All")
        self.chk_auto_nav2.setChecked(True)
        system_group = QGroupBox("Cấu hình hệ thống")
        system_layout = QGridLayout()
        system_layout.addWidget(QLabel("Robot mode:"), 0, 0); system_layout.addWidget(self.rb_sim, 0, 1); system_layout.addWidget(self.rb_real, 0, 2)
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
        self.btn_batch_clear = QPushButton("Clear A.."); self.btn_batch_clear.clicked.connect(self.clear_batch_waypoints)
        self.btn_batch_undo = QPushButton("Undo"); self.btn_batch_undo.clicked.connect(self.undo_batch_waypoint)
        self.btn_batch_send = QPushButton("Gửi A.. cho nhiemvuboss"); self.btn_batch_send.clicked.connect(self.send_batch_waypoints_to_boss)
        self.lbl_batch_info = QLabel("Batch: 0 điểm"); self.lbl_batch_info.setWordWrap(True)
        map_group = QGroupBox("Quản lý map"); map_layout = QGridLayout()
        map_layout.addWidget(self.lbl_map_folder, 0, 0, 1, 3)
        map_layout.addWidget(self.btn_choose_map_folder, 1, 0); map_layout.addWidget(self.btn_refresh_maps, 1, 1); map_layout.addWidget(self.btn_choose_map_file, 1, 2)
        map_layout.addWidget(self.cmb_map_files, 2, 0, 1, 3); map_layout.addWidget(self.lbl_selected_map, 3, 0, 1, 3)
        map_layout.addWidget(QLabel("Tên file save (không cần .yaml):"), 4, 0, 1, 3)
        map_layout.addWidget(self.edt_save_map_name, 5, 0, 1, 2); map_layout.addWidget(self.btn_save_map, 5, 2)
        map_layout.addWidget(self.btn_batch_mode, 6, 0); map_layout.addWidget(self.btn_batch_undo, 6, 1); map_layout.addWidget(self.btn_batch_clear, 6, 2)
        map_layout.addWidget(self.btn_batch_send, 7, 0, 1, 3); map_layout.addWidget(self.lbl_batch_info, 8, 0, 1, 3)
        map_group.setLayout(map_layout)
        self.rb_map_goal = QRadioButton("Gửi Goal"); self.rb_map_draw = QRadioButton("Vẽ vật cản"); self.rb_map_erase = QRadioButton("Xóa vật cản")
        self.rb_map_goal.setChecked(True)
        self.map_mode_group = QButtonGroup(self); self.map_mode_group.setExclusive(True)
        self.map_mode_group.addButton(self.rb_map_goal); self.map_mode_group.addButton(self.rb_map_draw); self.map_mode_group.addButton(self.rb_map_erase)
        self.rb_map_goal.toggled.connect(self.on_map_mode_changed); self.rb_map_draw.toggled.connect(self.on_map_mode_changed); self.rb_map_erase.toggled.connect(self.on_map_mode_changed)
        self.sld_brush = QSlider(Qt.Horizontal); self.sld_brush.setMinimum(0); self.sld_brush.setMaximum(10); self.sld_brush.setValue(2); self.sld_brush.setTickPosition(QSlider.TicksBelow)
        self.lbl_brush = QLabel("Brush: 2 ô"); self.sld_brush.valueChanged.connect(lambda v: self.lbl_brush.setText(f"Brush: {v} ô"))
        self.btn_edit_undo = QPushButton("Undo"); self.btn_edit_undo.clicked.connect(self.on_edit_undo)
        self.btn_edit_redo = QPushButton("Redo"); self.btn_edit_redo.clicked.connect(self.on_edit_redo)
        self.btn_edit_reset = QPushButton("Reset map gốc"); self.btn_edit_reset.clicked.connect(self.on_edit_reset)
        self.btn_edit_apply = QPushButton("Áp dụng & Publish"); self.btn_edit_apply.clicked.connect(self.on_edit_apply)
        self.lbl_edit_status = QLabel("Chỉnh sửa: OFF"); self.lbl_edit_status.setStyleSheet("color: gray; font-weight: bold;")
        edit_group = QGroupBox("Chỉnh sửa map"); edit_layout = QGridLayout()
        edit_layout.addWidget(self.rb_map_goal, 0, 0); edit_layout.addWidget(self.rb_map_draw, 0, 1); edit_layout.addWidget(self.rb_map_erase, 0, 2)
        edit_layout.addWidget(self.lbl_brush, 1, 0); edit_layout.addWidget(self.sld_brush, 1, 1, 1, 2)
        edit_layout.addWidget(self.btn_edit_undo, 2, 0); edit_layout.addWidget(self.btn_edit_redo, 2, 1); edit_layout.addWidget(self.btn_edit_reset, 2, 2)
        edit_layout.addWidget(self.btn_edit_apply, 3, 0, 1, 3); edit_layout.addWidget(self.lbl_edit_status, 4, 0, 1, 3)
        edit_group.setLayout(edit_layout)
        self.btn_start_platform = QPushButton("Start Layer 1 (Odom)"); self.btn_stop_platform = QPushButton("Stop Layer 1 (Odom)")
        self.btn_start_mode = QPushButton("Start Layer 2 (Mode)"); self.btn_stop_mode = QPushButton("Stop Layer 2 (Mode)")
        self.btn_switch_mode = QPushButton("Switch Layer 2 Mode")
        self.btn_start_nav2 = QPushButton("Start Nav2"); self.btn_stop_nav2 = QPushButton("Stop Nav2")
        self.btn_start_all = QPushButton("Start All"); self.btn_stop_all = QPushButton("Stop All")
        self.edt_kill_node = QLineEdit("rviz2"); self.btn_kill_node = QPushButton("Kill Node")
        self.btn_start_platform.clicked.connect(self.start_platform); self.btn_stop_platform.clicked.connect(self.stop_platform)
        self.btn_start_mode.clicked.connect(self.start_operation_mode); self.btn_stop_mode.clicked.connect(self.stop_operation_mode)
        self.btn_switch_mode.clicked.connect(self.switch_operation_mode)
        self.btn_start_nav2.clicked.connect(self.start_nav2); self.btn_stop_nav2.clicked.connect(self.stop_nav2)
        self.btn_start_all.clicked.connect(self.start_all); self.btn_stop_all.clicked.connect(self.stop_all); self.btn_kill_node.clicked.connect(self.kill_node)
        self.lbl_platform_status = QLabel("Layer1 Odom: OFF"); self.lbl_mode_status = QLabel("Layer2 Mode: OFF")
        self.lbl_nav2_status = QLabel("Nav2: OFF"); self.lbl_odom_status = QLabel("Odom source: esp -> /odom"); self.lbl_dataenc = QLabel("Encoder /dataenc: -")
        control_group = QGroupBox("Điều khiển"); control_layout = QGridLayout()
        control_layout.addWidget(self.btn_start_platform, 0, 0); control_layout.addWidget(self.btn_stop_platform, 0, 1)
        control_layout.addWidget(self.btn_start_mode, 1, 0); control_layout.addWidget(self.btn_stop_mode, 1, 1)
        control_layout.addWidget(self.btn_switch_mode, 2, 0, 1, 2)
        control_layout.addWidget(self.btn_start_nav2, 3, 0); control_layout.addWidget(self.btn_stop_nav2, 3, 1)
        control_layout.addWidget(self.btn_start_all, 4, 0); control_layout.addWidget(self.btn_stop_all, 4, 1)
        control_layout.addWidget(QLabel("Node/pattern to kill:"), 5, 0); control_layout.addWidget(self.edt_kill_node, 5, 1)
        control_layout.addWidget(self.btn_kill_node, 6, 0, 1, 2)
        control_layout.addWidget(self.lbl_platform_status, 7, 0, 1, 2); control_layout.addWidget(self.lbl_mode_status, 8, 0, 1, 2)
        control_layout.addWidget(self.lbl_nav2_status, 9, 0, 1, 2); control_layout.addWidget(self.lbl_odom_status, 10, 0, 1, 2)
        control_layout.addWidget(self.lbl_dataenc, 11, 0, 1, 2)
        control_group.setLayout(control_layout)
        self.log_box = QTextEdit(); self.log_box.setReadOnly(True); self.log_box.setMinimumHeight(140)
        log_group = QGroupBox("Log"); log_layout = QVBoxLayout(); log_layout.addWidget(self.log_box); log_group.setLayout(log_layout)
        left_layout = QVBoxLayout()
        left_layout.addWidget(system_group); left_layout.addWidget(map_group); left_layout.addWidget(edit_group)
        left_layout.addWidget(control_group); left_layout.addWidget(log_group, stretch=1)
        left_widget = QWidget(); left_widget.setLayout(left_layout)
        left_scroll = QScrollArea(); left_scroll.setWidgetResizable(True); left_scroll.setWidget(left_widget)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff); left_scroll.setMinimumWidth(320)
        self.map_view = MapViewer(self.get_ros_node, on_map_click=self.on_map_clicked, on_map_drag=self.on_map_dragged)
        right_layout = QVBoxLayout(); right_layout.addWidget(self.map_view)
        right_widget = QWidget(); right_widget.setLayout(right_layout); right_widget.setMinimumWidth(260)
        root_layout = QHBoxLayout(); root_layout.addWidget(left_scroll, stretch=0); root_layout.addWidget(right_widget, stretch=1)
        root_layout.setContentsMargins(6, 6, 6, 6); root_layout.setSpacing(6); self.setLayout(root_layout)
        self.on_robot_mode_changed(); self.refresh_batch_waypoint_views(); self.refresh_serial_ports()

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros_node = RosInterface(log_callback=self.append_log, map_callback=self.on_new_map, pose_callback=self.on_new_pose, scan_callback=self.on_new_scan, dataenc_callback=self.on_new_dataenc)
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

    def _append_log_ui(self, text):
        self.log_box.append(text)

    def on_new_map(self, msg):
        self.map_signal.emit(msg)
    def on_new_pose(self, source, x, y, yaw):
        self.pose_signal.emit(float(x), float(y), float(yaw), source)
    def on_new_scan(self, msg, scan_points_world):
        self.scan_signal.emit(msg, scan_points_world)
    def on_new_dataenc(self, values):
        self.dataenc_signal.emit(values)

    def _on_new_map_ui(self, msg):
        if self.map_editor:
            self.map_editor.load_from_grid(msg)
        if self.map_editor and self.map_editor.edit_enabled:
            # Edit mode: KHÔNG redraw khi SLAM publish (tránh nhấp nháy)
            pass
        else:
            # Hiển thị map bình thường
            self.map_view.update_map(msg)
            # Nhận map đầu tiên → tự động bật edit mode
            if self.map_editor and self.map_editor.original_map_msg and not self.map_editor.edit_enabled:
                self._auto_activate_map_proxy()

    def _auto_activate_map_proxy(self):
        """Nhận map đầu tiên từ /map1 → bật edit mode ngay."""
        self.append_log("Nhận map đầu tiên từ /map1 → bật chỉnh sửa map...")
        QTimer.singleShot(500, self._activate_edit_mode)

    def _activate_edit_mode(self):
        if self.map_editor:
            self.map_editor.enable_edit()
            self.lbl_edit_status.setText("Chỉnh sửa: ON (SLAM→/map1, GUI→/map)")
            self.lbl_edit_status.setStyleSheet("color: lime; font-weight: bold;")
            if self.map_editor.edited_data:
                self.map_view.update_edited_map(self.map_editor.edited_data, self.map_editor.map_width, self.map_editor.map_height)
            self.append_log("Đã tự động bật chỉnh sửa map.")

    def _on_new_pose_ui(self, x, y, yaw, source):
        self.map_view.update_pose(x, y, yaw, source)
    def _on_new_scan_ui(self, msg, scan_points_world):
        self.map_view.update_scan(msg, scan_points_world)
    def _on_new_dataenc_ui(self, values):
        if not values:
            self.lbl_dataenc.setText("Encoder: [No Data]")
        elif len(values) == 2:
            self.lbl_dataenc.setText(f"Encoder: L={values[0]} | R={values[1]}")
        else:
            self.lbl_dataenc.setText(f"Encoder: {values}")

    def on_map_mode_changed(self):
        if self.rb_map_goal.isChecked(): self.map_mode = MAP_MODE_GOAL
        elif self.rb_map_draw.isChecked(): self.map_mode = MAP_MODE_DRAW_OBSTACLE
        elif self.rb_map_erase.isChecked(): self.map_mode = MAP_MODE_ERASE
    def on_edit_undo(self):
        if self.map_editor and self.map_editor.undo(): self._refresh_edit_map_display()
    def on_edit_redo(self):
        if self.map_editor and self.map_editor.redo(): self._refresh_edit_map_display()
    def on_edit_reset(self):
        if self.map_editor: self.map_editor.reset(); self._refresh_edit_map_display()
    def on_edit_apply(self):
        if not self.map_editor: return
        if not self.map_editor.edit_enabled:
            QMessageBox.warning(self, "Chưa bật edit", "Chưa có map để áp dụng."); return
        self.map_editor.publish_edited_map()
    def _refresh_edit_map_display(self):
        if self.map_editor and self.map_editor.edited_data:
            self.map_view.update_edited_map(self.map_editor.edited_data, self.map_editor.map_width, self.map_editor.map_height)

    def on_map_dragged(self, wx, wy):
        if wx is None or wy is None:
            if self.map_editor: self.map_editor.end_stroke()
            if self.map_editor and self.map_editor.edit_enabled: self._refresh_edit_map_display()
            return
        if self.map_mode not in (MAP_MODE_DRAW_OBSTACLE, MAP_MODE_ERASE): return
        if not self.map_editor or not self.map_editor.edit_enabled: return
        brush = self.sld_brush.value()
        paint_value = 100 if self.map_mode == MAP_MODE_DRAW_OBSTACLE else 0
        if self.map_editor.paint_world(wx, wy, paint_value, brush): self._refresh_edit_map_display()

    def on_map_clicked(self, wx, wy):
        if self.map_mode == MAP_MODE_DRAW_OBSTACLE:
            if self.map_editor and self.map_editor.edit_enabled:
                brush = self.sld_brush.value()
                self.map_editor.start_stroke(); self.map_editor.paint_world(wx, wy, 100, brush); self.map_editor.end_stroke()
                self._refresh_edit_map_display(); return True
            return False
        if self.map_mode == MAP_MODE_ERASE:
            if self.map_editor and self.map_editor.edit_enabled:
                brush = self.sld_brush.value()
                self.map_editor.start_stroke(); self.map_editor.paint_world(wx, wy, 0, brush); self.map_editor.end_stroke()
                self._refresh_edit_map_display(); return True
            return False
        if self.batch_waypoint_mode:
            name = self.waypoint_name_from_index(len(self.batch_waypoints))
            self.batch_waypoints.append({"name": name, "x": float(wx), "y": float(wy), "yaw": 0.0, "frame_id": "map"})
            self.refresh_batch_waypoint_views(); self.append_log(f"Thêm waypoint {name}: x={wx:.3f}, y={wy:.3f}"); return True
        return False

    def waypoint_name_from_index(self, idx):
        alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        if idx < len(alphabet): return alphabet[idx]
        return f"{alphabet[idx % len(alphabet)]}{idx // len(alphabet)}"
    def refresh_batch_waypoint_views(self):
        self.btn_batch_mode.setText(f"Waypoint Mode: {'ON' if self.batch_waypoint_mode else 'OFF'}")
        self.lbl_batch_info.setText(f"Batch: {len(self.batch_waypoints)} điểm")
        self.map_view.update_waypoint_batch(self.batch_waypoints)
    def toggle_batch_waypoint_mode(self):
        self.batch_waypoint_mode = not self.batch_waypoint_mode
        self.refresh_batch_waypoint_views()
        self.append_log("Waypoint mode bật: click map để thêm A/B/C..." if self.batch_waypoint_mode else "Waypoint mode tắt.")
    def undo_batch_waypoint(self):
        if not self.batch_waypoints: return
        removed = self.batch_waypoints.pop(); self.refresh_batch_waypoint_views(); self.append_log(f"Undo waypoint {removed['name']}")
    def clear_batch_waypoints(self):
        if not self.batch_waypoints: return
        self.batch_waypoints = []; self.refresh_batch_waypoint_views(); self.append_log("Đã clear toàn bộ waypoint batch.")
    def send_batch_waypoints_to_boss(self):
        if not self.batch_waypoints: QMessageBox.warning(self, "Thiếu waypoint", "Bạn chưa có waypoint A/B/C/... nào."); return
        node = self.get_ros_node()
        if node is None: QMessageBox.warning(self, "ROS chưa sẵn sàng", "ROS node của GUI chưa chạy."); return
        node.send_waypoint_batch(self.batch_waypoints); self.append_log("Đã gửi waypoint batch cho nhiemvuboss.")

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

    def is_sim_mode(self): return self.rb_sim.isChecked()
    def ros_prefix(self): return f"source '{ROS_SETUP}' && source '{WS_SETUP}' && "
    def get_selected_odom_source(self): return self.cmb_odom_source.currentData() or "esp"
    def get_selected_odom_topic(self): return "/odom"
    def on_odom_source_changed(self):
        if self.is_sim_mode(): self.lbl_odom_status.setText("Odom source: gazebo -> /odom"); return
        self.lbl_odom_status.setText(f"Odom source: {self.get_selected_odom_source()} -> /odom")
    def on_operation_mode_changed(self):
        is_loc = self.cmb_operation_mode.currentText() == "Localization"
        self.cmb_localization_backend.setEnabled(is_loc)
        if (not self.is_sim_mode()) and is_loc: self.chk_auto_nav2.setChecked(False); self.chk_auto_nav2.setEnabled(False)
        else: self.chk_auto_nav2.setEnabled(True)
    def on_robot_mode_changed(self):
        if self.ros_node:
            sim_active = self.is_sim_mode()
            param = rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, sim_active)
            self.ros_node.set_parameters([param])
        is_real = not self.is_sim_mode()
        self.cmb_odom_source.setEnabled(is_real)
        if hasattr(self, "cmb_esp_port"):
            self.cmb_esp_port.setEnabled(is_real)
            self.cmb_lidar_port.setEnabled(is_real)
            self.btn_refresh_ports.setEnabled(is_real)
            self.btn_swap_ports.setEnabled(is_real)
        self.on_operation_mode_changed()
        self.on_odom_source_changed()

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

        if curr_esp and curr_esp in combined:
            self.cmb_esp_port.setCurrentText(curr_esp)
        elif detected_esp:
            self.cmb_esp_port.setCurrentText(detected_esp)

        if curr_lidar and curr_lidar in combined:
            self.cmb_lidar_port.setCurrentText(curr_lidar)
        elif detected_lidar:
            self.cmb_lidar_port.setCurrentText(detected_lidar)

        self.cmb_esp_port.blockSignals(False); self.cmb_lidar_port.blockSignals(False)
        if hasattr(self, "append_log"):
            self.append_log(f"Quét cổng serial ({method}): ESP={self.cmb_esp_port.currentText()}, LiDAR={self.cmb_lidar_port.currentText()}")

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
    def is_layer1_running(self): return self.proc_mgr.is_running(self.LAYER1_SIM) or self.proc_mgr.is_running(self.LAYER1_REAL)
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
        temp_path = self._write_temp_yaml(data, "nav2_gui_v3_")
        self.append_log(f"Tạo params Nav2 tạm: {temp_path} (odom_topic='{topic}', cập nhật {changed} vị trí)"); return temp_path
    def build_slam_mapping_params(self):
        data = self._load_yaml(SLAM_MAPPING_PARAMS_BASE)
        if "slam_toolbox" not in data: data["slam_toolbox"] = {"ros__parameters": {}}
        if "ros__parameters" not in data["slam_toolbox"]: data["slam_toolbox"]["ros__parameters"] = {}
        data["slam_toolbox"]["ros__parameters"]["map_topic"] = "/map1"
        temp_path = self._write_temp_yaml(data, "slam_mapping_gui_v3_")
        self.append_log(f"Tạo SLAM mapping params tạm: {temp_path} (map_topic=/map1)"); return temp_path
    def build_slam_localization_params(self, map_graph_stem=None):
        data = self._load_yaml(SLAM_LOCALIZATION_PARAMS_BASE)
        if "slam_toolbox" not in data: data["slam_toolbox"] = {"ros__parameters": {}}
        if "ros__parameters" not in data["slam_toolbox"]: data["slam_toolbox"]["ros__parameters"] = {}
        params = data["slam_toolbox"]["ros__parameters"]
        params["mode"] = "localization"; params["map_file_name"] = map_graph_stem or _map_stem_from_yaml(self.selected_map_file)
        temp_path = self._write_temp_yaml(data, "slam_loc_gui_v3_")
        self.append_log(f"Tạo params SLAM localization tạm: {temp_path}"); return temp_path

    def start_platform(self):
        sim_time = _bool_to_ros(self.is_sim_mode())
        if self.is_sim_mode():
            self.proc_mgr.stop(self.LAYER1_REAL)
            self.proc_mgr.start(self.LAYER1_SIM, self.ros_prefix() + f"ros2 launch mo_hinh virtual_robot_gazebo.launch.py use_sim_time:={sim_time}")
            QTimer.singleShot(3500, self.unpause_gazebo_physics)
        else:
            self.proc_mgr.stop(self.LAYER1_SIM)
            esp_port = self.cmb_esp_port.currentText().strip() if hasattr(self, "cmb_esp_port") else ""
            lidar_port = self.cmb_lidar_port.currentText().strip() if hasattr(self, "cmb_lidar_port") else ""
            if not esp_port or not lidar_port:
                detected_esp, detected_lidar, detect_method = _detect_esp_lidar_ports()
                esp_port = esp_port or detected_esp
                lidar_port = lidar_port or detected_lidar
            else:
                detect_method = "user_selection"
            self.append_log(f"Khởi động Serial ({detect_method}): ESP={esp_port}, LiDAR={lidar_port}")
            self.proc_mgr.start(self.LAYER1_REAL, self.ros_prefix() + f"ros2 launch mo_hinh real_odom.launch.py odom_source:={self.get_selected_odom_source()} use_sim_time:={sim_time} hoverboard_port:={shlex.quote(esp_port)} esp_port:={shlex.quote(esp_port)} lidar_port:={shlex.quote(lidar_port)}")
    def stop_platform(self): self.proc_mgr.stop(self.LAYER1_SIM); self.proc_mgr.stop(self.LAYER1_REAL)
    def start_slam_mapping(self):
        self.append_log("Layer2 mode: SLAM Mapping")
        sim_time = _bool_to_ros(self.is_sim_mode())
        self.proc_mgr.stop(self.AMCL); self.proc_mgr.stop(self.SLAM_LOCALIZATION)
        slam_params = self.build_slam_mapping_params()
        if self.is_sim_mode():
            self.proc_mgr.start(self.SLAM_MAPPING, self.ros_prefix() + f"ros2 launch slam_toolbox online_async_launch.py use_sim_time:={sim_time} slam_params_file:='{slam_params}'")
            self.append_log("SLAM mô phỏng: cần robot di chuyển thì map mới mở rộng.")
        else:
            self.proc_mgr.start(self.SLAM_MAPPING, self.ros_prefix() + f"ros2 launch mo_hinh real_slam.launch.py use_sim_time:={sim_time} slam_params_file:='{slam_params}' use_rviz:=true")
    def start_localization_backend(self):
        self.append_log(f"Layer2 mode: Localization ({self.cmb_localization_backend.currentText()})")
        sim_time = _bool_to_ros(self.is_sim_mode())
        backend = self.cmb_localization_backend.currentText()
        if not self.ensure_map_selected(backend): return
        amcl_yaml, slam_graph = self.get_localization_map_targets()
        self.proc_mgr.stop(self.SLAM_MAPPING)
        if backend == "SLAM Toolbox":
            self.proc_mgr.stop(self.AMCL)
            slam_loc_params = self.build_slam_localization_params(slam_graph)
            if self.is_sim_mode():
                self.proc_mgr.start(self.SLAM_LOCALIZATION, self.ros_prefix() + f"ros2 launch slam_toolbox localization_launch.py use_sim_time:={sim_time} slam_params_file:='{slam_loc_params}'")
            else:
                self.proc_mgr.start(self.SLAM_LOCALIZATION, self.ros_prefix() + f"ros2 launch mo_hinh real_localization_nav2.launch.py localization_mode:=slam_toolbox use_sim_time:={sim_time} map_graph:='{slam_graph}' slam_params_file:='{slam_loc_params}' use_rviz:=true")
            self.append_log(f"Localization backend=SLAM Toolbox -> map_graph: {slam_graph}")
        else:
            self.proc_mgr.stop(self.SLAM_LOCALIZATION)
            nav2_params = self.build_nav2_params_for_selected_odom()
            if self.is_sim_mode():
                self.proc_mgr.start(self.AMCL, self.ros_prefix() + f"ros2 launch nav2_bringup localization_launch.py use_sim_time:={sim_time} map:='{amcl_yaml}' params_file:='{nav2_params}'")
            else:
                self.proc_mgr.start(self.AMCL, self.ros_prefix() + f"ros2 launch mo_hinh real_localization_nav2.launch.py localization_mode:=amcl use_sim_time:={sim_time} map_yaml:='{amcl_yaml}' nav2_params_file:='{nav2_params}' use_rviz:=true")
            self.append_log(f"Localization backend=AMCL -> map_yaml: {amcl_yaml}")
    def start_operation_mode(self):
        if self.cmb_operation_mode.currentText() == "SLAM Mapping": self.start_slam_mapping()
        else: self.start_localization_backend()
    def switch_operation_mode(self):
        self.append_log("Switch Layer 2 mode..."); self.stop_operation_mode(); QTimer.singleShot(600, self.start_operation_mode)
    def stop_operation_mode(self): self.proc_mgr.stop(self.SLAM_MAPPING); self.proc_mgr.stop(self.SLAM_LOCALIZATION); self.proc_mgr.stop(self.AMCL)
    def start_nav2(self):
        sim_time = _bool_to_ros(self.is_sim_mode())
        if (not self.is_sim_mode()) and self.cmb_operation_mode.currentText() == "Localization":
            self.append_log("Localization mode (real) đã bao gồm Nav2, bỏ qua Start Nav2 riêng."); return
        nav2_params = self.build_nav2_params_for_selected_odom()
        self.proc_mgr.start(self.NAV2, self.ros_prefix() + f"ros2 launch nav2_bringup navigation_launch.py use_sim_time:={sim_time} params_file:='{nav2_params}'")
    def stop_nav2(self): self.proc_mgr.stop(self.NAV2)
    def unpause_gazebo_physics(self):
        if self.proc_mgr.is_running(self.LAYER1_SIM):
            self.run_one_shot_command("unpause_physics", self.ros_prefix() + 'ros2 service call /unpause_physics std_srvs/srv/Empty "{}"')
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
        label.setText(f"{name}: {'ON' if is_on else 'OFF'}"); label.setStyleSheet(f"color: {'lime' if is_on else 'red'}; font-weight: bold;")
    def refresh_status(self):
        platform_on = self.proc_mgr.is_running(self.LAYER1_SIM) or self.proc_mgr.is_running(self.LAYER1_REAL)
        mode_on = self.proc_mgr.is_running(self.SLAM_MAPPING) or self.proc_mgr.is_running(self.SLAM_LOCALIZATION) or self.proc_mgr.is_running(self.AMCL)
        nav2_on = self.proc_mgr.is_running(self.NAV2)
        if (not self.is_sim_mode()) and self.cmb_operation_mode.currentText() == "Localization":
            nav2_on = nav2_on or self.proc_mgr.is_running(self.SLAM_LOCALIZATION) or self.proc_mgr.is_running(self.AMCL)
        self._set_status_label(self.lbl_platform_status, "Layer1 Odom", platform_on)
        self._set_status_label(self.lbl_mode_status, "Layer2 Mode", mode_on)
        self._set_status_label(self.lbl_nav2_status, "Nav2", nav2_on)
        self.lbl_odom_status.setText(f"Odom source: {'gazebo' if self.is_sim_mode() else self.get_selected_odom_source()} -> /odom")

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
            "virtual_robot_gazebo",
            "real_odom",
            "real_slam",
            "real_localization_nav2",
            "slam_toolbox",
            "nav2_bringup",
            "hoverboard_driver",
            "unpause_physics",
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