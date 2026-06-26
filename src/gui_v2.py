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
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QRadioButton,
    QScrollArea,
    QTextEdit,
    QVBoxLayout,
    QWidget,
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

    dedup = []
    seen_real = set()
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
        for _ in range(3):
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line.startswith("E,"):
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
    esp_alias = "/dev/esp32"
    lidar_alias = "/dev/lidar"
    if os.path.exists(esp_alias) and os.path.exists(lidar_alias):
        return esp_alias, lidar_alias, "udev_alias"

    ports = _list_serial_ports()
    if not ports:
        return "/dev/ttyUSB0", "/dev/ttyUSB1", "fallback_defaults"

    esp_port = None
    lidar_port = None

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
WS_SETUP = os.environ.get("WS_SETUP", os.path.join(WS_ROOT, "install", "setup.bash"))
NAV2_PARAMS_BASE = os.environ.get(
    "NAV2_PARAMS",
    _pick_first_existing(
        [
            os.path.join(CONFIG_DIR, "nav2_params.yaml"),
            os.path.join(MO_HINH_CONFIG_DIR, "nav2_params.yaml"),
        ]
    ),
)
SLAM_MAPPING_PARAMS_BASE = os.environ.get(
    "SLAM_MAPPING_PARAMS",
    _pick_first_existing(
        [
            os.path.join(CONFIG_DIR, "mapper_params_online_async.yaml"),
            os.path.join(MO_HINH_CONFIG_DIR, "mapper_params_online_async.yaml"),
        ]
    ),
)
SLAM_LOCALIZATION_PARAMS_BASE = os.environ.get(
    "SLAM_LOCALIZATION_PARAMS",
    _pick_first_existing(
        [
            os.path.join(CONFIG_DIR, "slam_localization.yaml"),
            os.path.join(MO_HINH_CONFIG_DIR, "slam_localization.yaml"),
        ]
    ),
)


class RosInterface(Node):
    def __init__(
        self,
        log_callback=None,
        map_callback=None,
        pose_callback=None,
        scan_callback=None,
        dataenc_callback=None,
    ):
        super().__init__("desktop_nav_gui_v2")
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

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            map_qos,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.on_scan,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.dataenc_sub = self.create_subscription(
            Int32MultiArray,
            "/dataenc",
            self.on_dataenc,
            20,
        )

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.waypoints_pub = self.create_publisher(String, "/nhiemvuboss/waypoints_json", 10)
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
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _warn_tf(self, message):
        now = time.monotonic()
        if now - self.last_tf_warn_time < 2.0:
            return
        self.last_tf_warn_time = now
        self.log(message)

    def publish_pose_from_tf(self):
        if not self.pose_callback:
            return
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_map_frame,
                self.target_base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self._warn_tf(f"TF pose lookup l?i ({self.target_map_frame} <- {self.target_base_frame}): {exc}")
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        yaw = self._pose_to_xyyaw(q.x, q.y, q.z, q.w)
        self.pose_callback("tf2", t.x, t.y, yaw)

    def _scan_to_world_points(self, msg):
        ranges = msg.ranges
        if not ranges:
            return []

        source_frame = msg.header.frame_id if msg.header.frame_id else self.target_base_frame
        scan_time = Time.from_msg(msg.header.stamp)

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.target_map_frame,
                source_frame,
                scan_time,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.target_map_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                self._warn_tf(
                    f"TF scan lookup l?i ({self.target_map_frame} <- {source_frame}): {exc}"
                )
                return []

        tr = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w

        # 2D projection from quaternion rotation matrix.
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)

        step = max(1, len(ranges) // 720)
        angle_min = msg.angle_min
        inc = msg.angle_increment
        min_range = msg.range_min
        max_range = msg.range_max
        world_points = []

        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            if r < min_range or r > max_range:
                continue

            angle = angle_min + i * inc
            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            wx = r00 * lx + r01 * ly + tr.x
            wy = r10 * lx + r11 * ly + tr.y
            world_points.append((wx, wy))

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
        if not self.dataenc_callback:
            return
        self.dataenc_callback(list(msg.data))

    def send_goal(self, x, y, yaw=0.0):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.log("NavigateToPose action server ch?a s?n sàng.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.log(f"G?i goal Nav2: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
        self.nav_to_pose_client.send_goal_async(goal_msg)
        return True

    def send_waypoint_batch(self, waypoints):
        msg = String()
        msg.data = json.dumps({"waypoints": waypoints}, ensure_ascii=False)
        self.waypoints_pub.publish(msg)
        self.log(
            f"G?i batch waypoint cho nhiemvuboss: {len(waypoints)} ?i?m lên /nhiemvuboss/waypoints_json"
        )


class MapViewer(QLabel):
    def __init__(self, ros_node_getter, on_map_click=None, parent=None):
        super().__init__(parent)
        self.get_ros_node = ros_node_getter
        self.on_map_click = on_map_click
        self.setMinimumSize(260, 220)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #555;")
        self.setText("Ch?a có d? li?u /map")

        self.map_pixmap = None
        self.map_resolution = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.scale_factor = 1.0
        self.offset_x = 0
        self.offset_y = 0

        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = 0.0
        self.pose_source = "none"
        self.latest_scan = None
        self.scan_points_world = []
        self.scan_points_from_tf = False
        self.batch_waypoints = []

    def update_map(self, occ_grid):
        info = occ_grid.info
        self.map_resolution = info.resolution
        self.map_origin_x = info.origin.position.x
        self.map_origin_y = info.origin.position.y
        self.map_width = info.width
        self.map_height = info.height

        img = QImage(self.map_width, self.map_height, QImage.Format_RGB888)
        data = occ_grid.data
        for y in range(self.map_height):
            for x in range(self.map_width):
                i = x + (self.map_height - 1 - y) * self.map_width
                val = data[i]
                if val == -1:
                    c = 127
                else:
                    c = int(255 - (val * 255 / 100))
                    c = max(0, min(255, c))
                img.setPixel(x, y, QColor(c, c, c).rgb())
        self.map_pixmap = QPixmap.fromImage(img)
        self.update_display()

    def update_pose(self, x, y, yaw, source):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.pose_source = source
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
        if self.latest_scan is None:
            return
        if self.robot_x is None or self.robot_y is None:
            return

        ranges = self.latest_scan.ranges
        if not ranges:
            return

        step = max(1, len(ranges) // 720)
        angle = self.latest_scan.angle_min
        inc = self.latest_scan.angle_increment
        min_range = self.latest_scan.range_min
        max_range = self.latest_scan.range_max

        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r):
                continue
            if r < min_range or r > max_range:
                continue
            beam = angle + i * inc + self.robot_yaw
            wx = self.robot_x + r * math.cos(beam)
            wy = self.robot_y + r * math.sin(beam)
            self.scan_points_world.append((wx, wy))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_display()

    def update_display(self):
        if self.map_pixmap is None:
            return

        label_w = max(1, self.width())
        label_h = max(1, self.height())
        scaled = self.map_pixmap.scaled(
            label_w, label_h, Qt.KeepAspectRatio, Qt.FastTransformation
        )
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
                if sx is None:
                    continue
                if 0 <= sx < label_w and 0 <= sy < label_h:
                    painter.drawPoint(sx, sy)

        if self.batch_waypoints:
            painter.setFont(QFont("Arial", 10))
            for i, wp in enumerate(self.batch_waypoints):
                sx, sy = self.world_to_widget(wp["x"], wp["y"])
                if sx is None:
                    continue
                is_last = i == len(self.batch_waypoints) - 1
                color = QColor(255, 210, 0) if not is_last else QColor(255, 120, 0)
                painter.setPen(QPen(color, 2))
                painter.setBrush(color)
                painter.drawEllipse(QPoint(sx, sy), 5, 5)
                painter.drawText(sx + 8, sy - 8, wp["name"])
                if i > 0:
                    px, py = self.world_to_widget(
                        self.batch_waypoints[i - 1]["x"], self.batch_waypoints[i - 1]["y"]
                    )
                    if px is not None:
                        painter.drawLine(px, py, sx, sy)

        if self.robot_x is not None and self.robot_y is not None:
            px, py = self.world_to_widget(self.robot_x, self.robot_y)
            if px is not None:
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                painter.setBrush(QColor(255, 0, 0))
                painter.drawEllipse(QPoint(px, py), 6, 6)
                arrow_len = 18
                ax = px + int(arrow_len * math.cos(self.robot_yaw))
                ay = py - int(arrow_len * math.sin(self.robot_yaw))
                painter.drawLine(px, py, ax, ay)

        painter.setPen(QPen(QColor(0, 255, 0), 1))
        painter.setFont(QFont("Arial", 10))
        painter.drawText(10, 20, "Click trái ?? g?i goal Nav2")
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
            if self.on_map_click is not None:
                consumed = bool(self.on_map_click(wx, wy))
                if consumed:
                    return
            ros_node = self.get_ros_node()
            if ros_node is not None:
                ok = ros_node.send_goal(wx, wy, yaw=0.0)
                if ok:
                    ros_node.log(f"?ã click goal trên map: ({wx:.3f}, {wy:.3f})")

    def widget_to_world(self, px, py):
        if self.map_pixmap is None:
            return None
        x_in = px - self.offset_x
        y_in = py - self.offset_y
        scaled_w = self.map_width * self.scale_factor
        scaled_h = self.map_height * self.scale_factor
        if x_in < 0 or y_in < 0 or x_in >= scaled_w or y_in >= scaled_h:
            return None
        mx = x_in / self.scale_factor
        my_img = y_in / self.scale_factor
        my = self.map_height - 1 - my_img
        wx = self.map_origin_x + mx * self.map_resolution
        wy = self.map_origin_y + my * self.map_resolution
        return wx, wy

    def world_to_widget(self, wx, wy):
        if self.map_pixmap is None:
            return None, None
        mx = (wx - self.map_origin_x) / self.map_resolution
        my = (wy - self.map_origin_y) / self.map_resolution
        px_map = mx
        py_map = self.map_height - 1 - my
        px = int(self.offset_x + px_map * self.scale_factor)
        py = int(self.offset_y + py_map * self.scale_factor)
        return px, py


class ProcessManager:
    def __init__(self, log_callback):
        self.log_callback = log_callback
        self.processes = {}

    def log(self, msg):
        if self.log_callback:
            self.log_callback(msg)

    def start(self, name, cmd):
        if name in self.processes and self.is_running(name):
            self.log(f"[{name}] ?ang ch?y r?i.")
            return

        self.log(f"[{name}] START")
        self.log(f"CMD: {cmd}")
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        self.processes[name] = proc
        thread = threading.Thread(
            target=self._read_output,
            args=(name, proc),
            daemon=True,
        )
        thread.start()

    def _read_output(self, name, proc):
        try:
            for line in proc.stdout:
                self.log(f"[{name}] {line.rstrip()}")
        except Exception as exc:
            self.log(f"[{name}] read output error: {exc}")

    def stop(self, name):
        proc = self.processes.get(name)
        if not proc:
            self.log(f"[{name}] ch?a ch?y.")
            return
        if proc.poll() is None:
            self.log(f"[{name}] STOP")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                time.sleep(1.0)
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception as exc:
                self.log(f"[{name}] stop error: {exc}")
        else:
            self.log(f"[{name}] ?ã d?ng.")

    def is_running(self, name):
        proc = self.processes.get(name)
        return bool(proc and proc.poll() is None)

    def stop_all(self):
        for name in list(self.processes.keys()):
            self.stop(name)


class MainWindow(QWidget):
    log_signal = pyqtSignal(str)
    map_signal = pyqtSignal(object)
    pose_signal = pyqtSignal(float, float, float, str)
    scan_signal = pyqtSignal(object, object)
    dataenc_signal = pyqtSignal(object)

    LAYER1_SIM = "layer1_sim"
    LAYER1_REAL = "layer1_real"
    SLAM_MAPPING = "slam_mapping"
    SLAM_LOCALIZATION = "slam_localization"
    AMCL = "amcl"
    NAV2 = "nav2"

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Control GUI v2 - Layer1 Odom / Layer2 Mode")
        self.resize(1200, 760)

        self.temp_files = []
        self.map_dir = _pick_first_existing([DEFAULT_MAP_DIR, MO_HINH_MAP_DIR, WS_ROOT])
        self.selected_map_file = ""
        self.batch_waypoint_mode = False
        self.batch_waypoints = []
        self.is_shutting_down = False
        self.one_shot_procs = []
        self.one_shot_lock = threading.Lock()

        self.proc_mgr = ProcessManager(self.append_log)
        self.ros_node = None
        self.ros_spin_thread = None

        self.log_signal.connect(self._append_log_ui)
        self.map_signal.connect(self._on_new_map_ui)
        self.pose_signal.connect(self._on_new_pose_ui)
        self.scan_signal.connect(self._on_new_scan_ui)
        self.dataenc_signal.connect(self._on_new_dataenc_ui)

        self.init_ui()
        self.init_ros()
        self.refresh_map_list()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.refresh_status)
        self.status_timer.start(500)

    def init_ui(self):
        self.rb_sim = QRadioButton("Simulation (Gazebo)")
        self.rb_real = QRadioButton("Robot th?t")
        self.rb_real.setChecked(True)
        self.robot_mode_group = QButtonGroup(self)
        self.robot_mode_group.setExclusive(True)
        self.robot_mode_group.addButton(self.rb_sim)
        self.robot_mode_group.addButton(self.rb_real)
        self.rb_sim.toggled.connect(self.on_robot_mode_changed)
        self.rb_real.toggled.connect(self.on_robot_mode_changed)

        self.cmb_odom_source = QComboBox()
        self.cmb_odom_source.addItem("ESP encoder odom", "esp")
        self.cmb_odom_source.addItem("LiDAR RF2O odom", "rf2o")
        self.cmb_odom_source.currentIndexChanged.connect(self.on_odom_source_changed)
        self.cmb_esp_wheel_mode = QComboBox()
        self.cmb_esp_wheel_mode.addItem("All wheels (1,2,3,4)", "all_4")
        self.cmb_esp_wheel_mode.addItem("Only wheels 1 + 4", "wheels_1_4")
        default_wheel_mode_idx = self.cmb_esp_wheel_mode.findData("wheels_1_4")
        if default_wheel_mode_idx >= 0:
            self.cmb_esp_wheel_mode.setCurrentIndex(default_wheel_mode_idx)
        self.cmb_esp_wheel_mode.currentIndexChanged.connect(self.on_odom_source_changed)

        self.cmb_operation_mode = QComboBox()
        self.cmb_operation_mode.addItems(["SLAM Mapping", "Localization"])
        self.cmb_operation_mode.currentIndexChanged.connect(self.on_operation_mode_changed)

        self.cmb_localization_backend = QComboBox()
        self.cmb_localization_backend.addItems(["SLAM Toolbox", "AMCL"])

        self.chk_auto_nav2 = QCheckBox("T? start Nav2 khi Start All")
        self.chk_auto_nav2.setChecked(True)

        system_group = QGroupBox("C?u hình h? th?ng")
        system_layout = QGridLayout()
        system_layout.addWidget(QLabel("Robot mode:"), 0, 0)
        system_layout.addWidget(self.rb_sim, 0, 1)
        system_layout.addWidget(self.rb_real, 0, 2)
        system_layout.addWidget(QLabel("Odom source:"), 1, 0)
        system_layout.addWidget(self.cmb_odom_source, 1, 1, 1, 2)
        system_layout.addWidget(QLabel("ESP wheel odom mode:"), 2, 0)
        system_layout.addWidget(self.cmb_esp_wheel_mode, 2, 1, 1, 2)
        system_layout.addWidget(QLabel("Operation mode:"), 3, 0)
        system_layout.addWidget(self.cmb_operation_mode, 3, 1, 1, 2)
        system_layout.addWidget(QLabel("Localization backend:"), 4, 0)
        system_layout.addWidget(self.cmb_localization_backend, 4, 1, 1, 2)
        system_layout.addWidget(self.chk_auto_nav2, 5, 0, 1, 3)
        system_group.setLayout(system_layout)

        self.lbl_map_folder = QLabel("Map folder: ")
        self.lbl_map_folder.setWordWrap(True)
        self.cmb_map_files = QComboBox()
        self.cmb_map_files.currentIndexChanged.connect(self.on_selected_map_changed)

        self.btn_choose_map_folder = QPushButton("Ch?n folder map")
        self.btn_refresh_maps = QPushButton("Refresh map list")
        self.btn_choose_map_file = QPushButton("Ch?n map YAML th? công")
        self.btn_choose_map_folder.clicked.connect(self.choose_map_folder)
        self.btn_refresh_maps.clicked.connect(self.refresh_map_list)
        self.btn_choose_map_file.clicked.connect(self.choose_map_file)

        self.lbl_selected_map = QLabel("Map ?ang dùng: ch?a ch?n")
        self.lbl_selected_map.setWordWrap(True)
        self.edt_save_map_name = QLineEdit("my_map/new_map")
        self.btn_save_map = QPushButton("L?u map (Serialize)")
        self.btn_save_map.clicked.connect(self.save_map)
        self.btn_batch_mode = QPushButton("Waypoint Mode: OFF")
        self.btn_batch_clear = QPushButton("Clear A..")
        self.btn_batch_undo = QPushButton("Undo")
        self.btn_batch_send = QPushButton("G?i A.. cho nhiemvuboss")
        self.btn_batch_mode.clicked.connect(self.toggle_batch_waypoint_mode)
        self.btn_batch_clear.clicked.connect(self.clear_batch_waypoints)
        self.btn_batch_undo.clicked.connect(self.undo_batch_waypoint)
        self.btn_batch_send.clicked.connect(self.send_batch_waypoints_to_boss)
        self.lbl_batch_info = QLabel("Batch: 0 ?i?m")
        self.lbl_batch_info.setWordWrap(True)

        map_group = QGroupBox("Qu?n lý map")
        map_layout = QGridLayout()
        map_layout.addWidget(self.lbl_map_folder, 0, 0, 1, 3)
        map_layout.addWidget(self.btn_choose_map_folder, 1, 0)
        map_layout.addWidget(self.btn_refresh_maps, 1, 1)
        map_layout.addWidget(self.btn_choose_map_file, 1, 2)
        map_layout.addWidget(self.cmb_map_files, 2, 0, 1, 3)
        map_layout.addWidget(self.lbl_selected_map, 3, 0, 1, 3)
        map_layout.addWidget(QLabel("Tên file save (không c?n .yaml):"), 4, 0, 1, 3)
        map_layout.addWidget(self.edt_save_map_name, 5, 0, 1, 2)
        map_layout.addWidget(self.btn_save_map, 5, 2)
        map_layout.addWidget(self.btn_batch_mode, 6, 0)
        map_layout.addWidget(self.btn_batch_undo, 6, 1)
        map_layout.addWidget(self.btn_batch_clear, 6, 2)
        map_layout.addWidget(self.btn_batch_send, 7, 0, 1, 3)
        map_layout.addWidget(self.lbl_batch_info, 8, 0, 1, 3)
        map_group.setLayout(map_layout)

        self.btn_start_platform = QPushButton("Start Layer 1 (Odom)")
        self.btn_stop_platform = QPushButton("Stop Layer 1 (Odom)")
        self.btn_start_mode = QPushButton("Start Layer 2 (Mode)")
        self.btn_stop_mode = QPushButton("Stop Layer 2 (Mode)")
        self.btn_switch_mode = QPushButton("Switch Layer 2 Mode")
        self.btn_start_nav2 = QPushButton("Start Nav2")
        self.btn_stop_nav2 = QPushButton("Stop Nav2")
        self.btn_start_all = QPushButton("Start All")
        self.btn_stop_all = QPushButton("Stop All")
        self.edt_kill_node = QLineEdit("rviz2")
        self.btn_kill_node = QPushButton("Kill Node")

        self.btn_start_platform.clicked.connect(self.start_platform)
        self.btn_stop_platform.clicked.connect(self.stop_platform)
        self.btn_start_mode.clicked.connect(self.start_operation_mode)
        self.btn_stop_mode.clicked.connect(self.stop_operation_mode)
        self.btn_switch_mode.clicked.connect(self.switch_operation_mode)
        self.btn_start_nav2.clicked.connect(self.start_nav2)
        self.btn_stop_nav2.clicked.connect(self.stop_nav2)
        self.btn_start_all.clicked.connect(self.start_all)
        self.btn_stop_all.clicked.connect(self.stop_all)
        self.btn_kill_node.clicked.connect(self.kill_node)

        self.lbl_platform_status = QLabel("Layer1 Odom: OFF")
        self.lbl_mode_status = QLabel("Layer2 Mode: OFF")
        self.lbl_nav2_status = QLabel("Nav2: OFF")
        self.lbl_odom_status = QLabel("Odom source: esp -> /odom")
        self.lbl_dataenc = QLabel("Encoder /dataenc: -")

        control_group = QGroupBox("?i?u khi?n")
        control_layout = QGridLayout()
        control_layout.addWidget(self.btn_start_platform, 0, 0)
        control_layout.addWidget(self.btn_stop_platform, 0, 1)
        control_layout.addWidget(self.btn_start_mode, 1, 0)
        control_layout.addWidget(self.btn_stop_mode, 1, 1)
        control_layout.addWidget(self.btn_switch_mode, 2, 0, 1, 2)
        control_layout.addWidget(self.btn_start_nav2, 3, 0)
        control_layout.addWidget(self.btn_stop_nav2, 3, 1)
        control_layout.addWidget(self.btn_start_all, 4, 0)
        control_layout.addWidget(self.btn_stop_all, 4, 1)
        control_layout.addWidget(QLabel("Node/pattern to kill:"), 5, 0)
        control_layout.addWidget(self.edt_kill_node, 5, 1)
        control_layout.addWidget(self.btn_kill_node, 6, 0, 1, 2)
        control_layout.addWidget(self.lbl_platform_status, 7, 0, 1, 2)
        control_layout.addWidget(self.lbl_mode_status, 8, 0, 1, 2)
        control_layout.addWidget(self.lbl_nav2_status, 9, 0, 1, 2)
        control_layout.addWidget(self.lbl_odom_status, 10, 0, 1, 2)
        control_layout.addWidget(self.lbl_dataenc, 11, 0, 1, 2)
        control_group.setLayout(control_layout)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMinimumHeight(140)
        log_group = QGroupBox("Log")
        log_layout = QVBoxLayout()
        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)

        left_layout = QVBoxLayout()
        left_layout.addWidget(system_group)
        left_layout.addWidget(map_group)
        left_layout.addWidget(control_group)
        left_layout.addWidget(log_group, stretch=1)
        left_widget = QWidget()
        left_widget.setLayout(left_layout)

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setWidget(left_widget)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setMinimumWidth(320)

        self.map_view = MapViewer(self.get_ros_node, self.on_map_clicked)
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.map_view)
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        right_widget.setMinimumWidth(260)

        root_layout = QHBoxLayout()
        root_layout.addWidget(left_scroll, stretch=0)
        root_layout.addWidget(right_widget, stretch=1)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(6)
        self.setLayout(root_layout)

        self.on_robot_mode_changed()
        self.refresh_batch_waypoint_views()

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros_node = RosInterface(
            log_callback=self.append_log,
            map_callback=self.on_new_map,
            pose_callback=self.on_new_pose,
            scan_callback=self.on_new_scan,
            dataenc_callback=self.on_new_dataenc,
        )
        self.ros_spin_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_spin_thread.start()
        self.append_log("ROS2 node v2 ?ã kh?i ??ng.")

    def spin_ros(self):
        try:
            rclpy.spin(self.ros_node)
        except Exception as exc:
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
        self.map_view.update_map(msg)

    def _on_new_pose_ui(self, x, y, yaw, source):
        self.map_view.update_pose(x, y, yaw, source)

    def _on_new_scan_ui(self, msg, scan_points_world):
        self.map_view.update_scan(msg, scan_points_world)

    def _on_new_dataenc_ui(self, values):
        if not values:
            self.lbl_dataenc.setText("Encoder /dataenc: []")
            return
        self.lbl_dataenc.setText(f"Encoder /dataenc: {values}")

    def on_odom_source_changed(self):
        if self.is_sim_mode():
            self.cmb_esp_wheel_mode.setEnabled(False)
            self.lbl_odom_status.setText("Odom source: gazebo -> /odom")
            return
        odom_source = self.get_selected_odom_source()
        wheel_mode = self.get_selected_esp_wheel_odom_mode()
        self.cmb_esp_wheel_mode.setEnabled(odom_source == "esp")
        if odom_source == "esp":
            self.lbl_odom_status.setText(f"Odom source: esp/{wheel_mode} -> /odom")
            return
        self.lbl_odom_status.setText(f"Odom source: {odom_source} -> /odom")

    def on_operation_mode_changed(self):
        is_localization = self.cmb_operation_mode.currentText() == "Localization"
        self.cmb_localization_backend.setEnabled(is_localization)
        if (not self.is_sim_mode()) and is_localization:
            self.chk_auto_nav2.setChecked(False)
            self.chk_auto_nav2.setEnabled(False)
        else:
            self.chk_auto_nav2.setEnabled(True)

    def on_robot_mode_changed(self):
        self.cmb_odom_source.setEnabled(not self.is_sim_mode())
        self.on_operation_mode_changed()
        self.on_odom_source_changed()

    def waypoint_name_from_index(self, idx):
        alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        if idx < len(alphabet):
            return alphabet[idx]
        group = idx // len(alphabet)
        pos = idx % len(alphabet)
        return f"{alphabet[pos]}{group}"

    def refresh_batch_waypoint_views(self):
        mode_text = "ON" if self.batch_waypoint_mode else "OFF"
        self.btn_batch_mode.setText(f"Waypoint Mode: {mode_text}")
        self.lbl_batch_info.setText(f"Batch: {len(self.batch_waypoints)} ?i?m")
        self.map_view.update_waypoint_batch(self.batch_waypoints)

    def toggle_batch_waypoint_mode(self):
        self.batch_waypoint_mode = not self.batch_waypoint_mode
        self.refresh_batch_waypoint_views()
        self.append_log(
            "Waypoint mode b?t: click map ?? thêm A/B/C..." if self.batch_waypoint_mode
            else "Waypoint mode t?t: click map g?i goal Nav2 nh? c?."
        )

    def on_map_clicked(self, wx, wy):
        if not self.batch_waypoint_mode:
            return False
        name = self.waypoint_name_from_index(len(self.batch_waypoints))
        self.batch_waypoints.append(
            {
                "name": name,
                "x": float(wx),
                "y": float(wy),
                "yaw": 0.0,
                "frame_id": "map",
            }
        )
        self.refresh_batch_waypoint_views()
        self.append_log(f"Thêm waypoint {name}: x={wx:.3f}, y={wy:.3f}")
        return True

    def undo_batch_waypoint(self):
        if not self.batch_waypoints:
            return
        removed = self.batch_waypoints.pop()
        self.refresh_batch_waypoint_views()
        self.append_log(f"Undo waypoint {removed['name']}")

    def clear_batch_waypoints(self):
        if not self.batch_waypoints:
            return
        self.batch_waypoints = []
        self.refresh_batch_waypoint_views()
        self.append_log("?ã clear toàn b? waypoint batch.")

    def send_batch_waypoints_to_boss(self):
        if not self.batch_waypoints:
            QMessageBox.warning(self, "Thi?u waypoint", "B?n ch?a có waypoint A/B/C/... nào.")
            return
        node = self.get_ros_node()
        if node is None:
            QMessageBox.warning(self, "ROS ch?a s?n sàng", "ROS node c?a GUI ch?a ch?y.")
            return
        node.send_waypoint_batch(self.batch_waypoints)
        self.append_log(
            "?ã g?i waypoint batch cho nhiemvuboss. Hãy ??m b?o node nhiemvuboss.py ?ang ch?y."
        )

    def choose_map_folder(self):
        folder = QFileDialog.getExistingDirectory(
            self,
            "Ch?n th? m?c ch?a map YAML",
            self.map_dir if os.path.exists(self.map_dir) else os.path.expanduser("~"),
        )
        if folder:
            self.map_dir = folder
            self.refresh_map_list()

    def choose_map_file(self):
        start_dir = self.map_dir if os.path.exists(self.map_dir) else os.path.expanduser("~")
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Ch?n file map YAML",
            start_dir,
            "YAML files (*.yaml *.yml)",
        )
        if file_path:
            self.selected_map_file = file_path
            self.lbl_selected_map.setText(f"Map ?ang dùng: {file_path}")
            self.append_log(f"?ã ch?n map th? công: {file_path}")

    def refresh_map_list(self):
        os.makedirs(self.map_dir, exist_ok=True)
        self.lbl_map_folder.setText(f"Map folder: {self.map_dir}")
        files = sorted(glob.glob(os.path.join(self.map_dir, "*.yaml"))) + sorted(
            glob.glob(os.path.join(self.map_dir, "*.yml"))
        )

        prev = self.selected_map_file
        self.cmb_map_files.blockSignals(True)
        self.cmb_map_files.clear()
        for fpath in files:
            self.cmb_map_files.addItem(os.path.basename(fpath), fpath)
        self.cmb_map_files.blockSignals(False)

        if files:
            pick_index = 0
            if prev in files:
                pick_index = files.index(prev)
            self.cmb_map_files.setCurrentIndex(pick_index)
            self.selected_map_file = self.cmb_map_files.currentData()
            self.lbl_selected_map.setText(f"Map ?ang dùng: {self.selected_map_file}")
        else:
            self.selected_map_file = ""
            self.lbl_selected_map.setText("Map ?ang dùng: ch?a có file YAML trong folder")
        self.append_log(f"Refresh map list: tìm th?y {len(files)} file.")

    def on_selected_map_changed(self):
        fpath = self.cmb_map_files.currentData()
        if fpath:
            self.selected_map_file = fpath
            self.lbl_selected_map.setText(f"Map ?ang dùng: {fpath}")

    def is_sim_mode(self):
        return self.rb_sim.isChecked()

    def ros_prefix(self):
        return f"source '{ROS_SETUP}' && source '{WS_SETUP}' && "

    def get_selected_odom_source(self):
        return self.cmb_odom_source.currentData() or "esp"

    def get_selected_esp_wheel_odom_mode(self):
        return self.cmb_esp_wheel_mode.currentData() or "wheels_1_4"

    def get_selected_odom_topic(self):
        return "/odom"

    def _map_base_stem_from_selected(self):
        if self.selected_map_file:
            stem = os.path.splitext(self.selected_map_file)[0]
        else:
            stem = os.path.join(self.map_dir, "my_map")

        for suffix in (MAP_SUFFIX_AMCL, MAP_SUFFIX_LOCALIZATION):
            if stem.endswith(suffix):
                return stem[: -len(suffix)]
        return stem

    def _pick_existing_file(self, candidates):
        for path in candidates:
            if os.path.exists(path):
                return path
        return candidates[0]

    def get_localization_map_targets(self):
        base_stem = self._map_base_stem_from_selected()

        amcl_yaml_candidates = [
            f"{base_stem}{MAP_SUFFIX_AMCL}.yaml",
            f"{base_stem}{MAP_SUFFIX_AMCL}.yml",
            f"{base_stem}.yaml",
            f"{base_stem}.yml",
            f"{base_stem}{MAP_SUFFIX_LOCALIZATION}.yaml",
            f"{base_stem}{MAP_SUFFIX_LOCALIZATION}.yml",
        ]
        if self.selected_map_file:
            amcl_yaml_candidates.insert(0, self.selected_map_file)
        amcl_yaml = self._pick_existing_file(amcl_yaml_candidates)

        slam_graph_candidates = [
            f"{base_stem}{MAP_SUFFIX_LOCALIZATION}",
            base_stem,
            f"{base_stem}{MAP_SUFFIX_AMCL}",
        ]
        slam_graph = slam_graph_candidates[0]
        for candidate in slam_graph_candidates:
            posegraph = candidate + ".posegraph"
            data = candidate + ".data"
            if os.path.exists(posegraph) or os.path.exists(data):
                slam_graph = candidate
                break

        return amcl_yaml, slam_graph

    def ensure_map_selected(self, backend):
        amcl_yaml, slam_graph = self.get_localization_map_targets()
        if backend == "AMCL":
            if not os.path.exists(amcl_yaml):
                QMessageBox.warning(
                    self,
                    "Thi?u map AMCL",
                    "Không tìm th?y file YAML map cho AMCL.\n"
                    "Hãy Save map tr??c ho?c ch?n file map YAML h?p l?.",
                )
                return False
            return True

        posegraph = slam_graph + ".posegraph"
        data = slam_graph + ".data"
        if not (os.path.exists(posegraph) or os.path.exists(data)):
            QMessageBox.warning(
                self,
                "Thi?u map localization",
                "Không tìm th?y file posegraph/data cho SLAM localization.\n"
                "Hãy Save map tr??c ?? t?o b?n localization.",
            )
            return False
        return True

    def is_layer1_running(self):
        return self.proc_mgr.is_running(self.LAYER1_SIM) or self.proc_mgr.is_running(
            self.LAYER1_REAL
        )

    def _write_temp_yaml(self, data, prefix):
        fd, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
        os.close(fd)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        self.temp_files.append(path)
        return path

    def _load_yaml(self, path):
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def build_nav2_params_for_selected_odom(self):
        data = self._load_yaml(NAV2_PARAMS_BASE)
        topic = self.get_selected_odom_topic()
        changed = _replace_key_recursive(data, "odom_topic", topic)
        temp_path = self._write_temp_yaml(data, "nav2_gui_v2_")
        self.append_log(
            f"T?o params Nav2 t?m: {temp_path} (odom_topic='{topic}', c?p nh?t {changed} v? trí)"
        )
        return temp_path

    def build_slam_localization_params(self, map_graph_stem=None):
        data = self._load_yaml(SLAM_LOCALIZATION_PARAMS_BASE)
        if "slam_toolbox" not in data:
            data["slam_toolbox"] = {"ros__parameters": {}}
        if "ros__parameters" not in data["slam_toolbox"]:
            data["slam_toolbox"]["ros__parameters"] = {}

        params = data["slam_toolbox"]["ros__parameters"]
        params["mode"] = "localization"
        if map_graph_stem:
            params["map_file_name"] = map_graph_stem
        else:
            params["map_file_name"] = _map_stem_from_yaml(self.selected_map_file)
        temp_path = self._write_temp_yaml(data, "slam_loc_gui_v2_")
        self.append_log(
            f"T?o params SLAM localization t?m: {temp_path} (map_file_name='{params['map_file_name']}')"
        )
        return temp_path

    def start_platform(self):
        sim_time = _bool_to_ros(self.is_sim_mode())
        if self.is_sim_mode():
            self.proc_mgr.stop(self.LAYER1_REAL)
            cmd = (
                self.ros_prefix()
                + "ros2 launch mo_hinh virtual_robot_gazebo.launch.py "
                + f"use_sim_time:={sim_time} gui:=true"
            )
            self.proc_mgr.start(self.LAYER1_SIM, cmd)
            QTimer.singleShot(3500, self.unpause_gazebo_physics)
        else:
            self.proc_mgr.stop(self.LAYER1_SIM)
            odom_source = self.get_selected_odom_source()
            esp_wheel_mode = self.get_selected_esp_wheel_odom_mode()
            esp_port, lidar_port, detect_method = _detect_esp_lidar_ports()
            self.append_log(
                f"Auto-detect serial ({detect_method}): ESP={esp_port}, LiDAR={lidar_port}"
            )
            cmd = (
                self.ros_prefix()
                + "ros2 launch mo_hinh real_odom.launch.py "
                + f"odom_source:={odom_source} use_sim_time:={sim_time} "
                + f"esp_wheel_odom_mode:={esp_wheel_mode} "
                + f"esp_port:={shlex.quote(esp_port)} lidar_port:={shlex.quote(lidar_port)}"
            )
            self.proc_mgr.start(self.LAYER1_REAL, cmd)

    def stop_platform(self):
        self.proc_mgr.stop(self.LAYER1_SIM)
        self.proc_mgr.stop(self.LAYER1_REAL)

    def start_slam_mapping(self):
        self.append_log("Layer2 mode: SLAM Mapping")
        if not self.is_layer1_running():
            QMessageBox.warning(
                self,
                "Thi?u Layer 1",
                "B?n c?n start Layer 1 (Odom) tr??c.",
            )
            return

        sim_time = _bool_to_ros(self.is_sim_mode())
        self.proc_mgr.stop(self.AMCL)
        self.proc_mgr.stop(self.SLAM_LOCALIZATION)
        if self.is_sim_mode():
            cmd = (
                self.ros_prefix()
                + "ros2 launch slam_toolbox online_async_launch.py "
                + f"use_sim_time:={sim_time} slam_params_file:='{SLAM_MAPPING_PARAMS_BASE}'"
            )
            self.append_log(
                "SLAM mô ph?ng: c?n robot di chuy?n thì map m?i m? r?ng (dùng teleop /cmd_vel)."
            )
        else:
            cmd = (
                self.ros_prefix()
                + "ros2 launch mo_hinh real_slam.launch.py "
                + f"use_sim_time:={sim_time} slam_params_file:='{SLAM_MAPPING_PARAMS_BASE}' "
                + "use_rviz:=true"
            )
        self.proc_mgr.start(self.SLAM_MAPPING, cmd)

    def start_localization_backend(self):
        self.append_log(
            f"Layer2 mode: Localization ({self.cmb_localization_backend.currentText()})"
        )
        if not self.is_layer1_running():
            QMessageBox.warning(
                self,
                "Thi?u Layer 1",
                "B?n c?n start Layer 1 (Odom) tr??c.",
            )
            return
        sim_time = _bool_to_ros(self.is_sim_mode())
        backend = self.cmb_localization_backend.currentText()
        if not self.ensure_map_selected(backend):
            return

        amcl_yaml, slam_graph = self.get_localization_map_targets()
        self.proc_mgr.stop(self.SLAM_MAPPING)

        if backend == "SLAM Toolbox":
            self.proc_mgr.stop(self.AMCL)
            graph_stem = slam_graph
            slam_loc_params = self.build_slam_localization_params(graph_stem)
            if self.is_sim_mode():
                cmd = (
                    self.ros_prefix()
                    + "ros2 launch slam_toolbox localization_launch.py "
                    + f"use_sim_time:={sim_time} slam_params_file:='{slam_loc_params}'"
                )
            else:
                cmd = (
                    self.ros_prefix()
                    + "ros2 launch mo_hinh real_localization_nav2.launch.py "
                    + "localization_mode:=slam_toolbox "
                    + f"use_sim_time:={sim_time} map_graph:='{graph_stem}' "
                    + f"slam_params_file:='{slam_loc_params}' use_rviz:=true"
                )
            self.append_log(f"Localization backend=SLAM Toolbox -> map_graph: {graph_stem}")
            self.proc_mgr.start(self.SLAM_LOCALIZATION, cmd)
        else:
            self.proc_mgr.stop(self.SLAM_LOCALIZATION)
            nav2_params = self.build_nav2_params_for_selected_odom()
            if self.is_sim_mode():
                cmd = (
                    self.ros_prefix()
                    + "ros2 launch nav2_bringup localization_launch.py "
                    + f"use_sim_time:={sim_time} map:='{amcl_yaml}' "
                    + f"params_file:='{nav2_params}'"
                )
            else:
                cmd = (
                    self.ros_prefix()
                    + "ros2 launch mo_hinh real_localization_nav2.launch.py "
                    + "localization_mode:=amcl "
                    + f"use_sim_time:={sim_time} map_yaml:='{amcl_yaml}' "
                    + f"nav2_params_file:='{nav2_params}' use_rviz:=true"
                )
            self.append_log(f"Localization backend=AMCL -> map_yaml: {amcl_yaml}")
            self.proc_mgr.start(self.AMCL, cmd)

    def start_operation_mode(self):
        mode = self.cmb_operation_mode.currentText()
        if mode == "SLAM Mapping":
            self.start_slam_mapping()
        else:
            self.start_localization_backend()

    def switch_operation_mode(self):
        self.append_log("Switch Layer 2 mode...")
        self.stop_operation_mode()
        QTimer.singleShot(600, self.start_operation_mode)

    def stop_operation_mode(self):
        self.proc_mgr.stop(self.SLAM_MAPPING)
        self.proc_mgr.stop(self.SLAM_LOCALIZATION)
        self.proc_mgr.stop(self.AMCL)

    def start_nav2(self):
        if not self.is_layer1_running():
            QMessageBox.warning(
                self,
                "Thi?u Layer 1",
                "B?n c?n start Layer 1 (Odom) tr??c.",
            )
            return
        sim_time = _bool_to_ros(self.is_sim_mode())
        if (not self.is_sim_mode()) and self.cmb_operation_mode.currentText() == "Localization":
            self.append_log(
                "Localization mode (real) ?ã bao g?m Nav2 trong file4, b? qua Start Nav2 riêng."
            )
            return
        nav2_params = self.build_nav2_params_for_selected_odom()
        cmd = (
            self.ros_prefix()
            + "ros2 launch nav2_bringup navigation_launch.py "
            + f"use_sim_time:={sim_time} params_file:='{nav2_params}'"
        )
        self.proc_mgr.start(self.NAV2, cmd)

    def stop_nav2(self):
        self.proc_mgr.stop(self.NAV2)

    def unpause_gazebo_physics(self):
        if not self.proc_mgr.is_running(self.LAYER1_SIM):
            return
        cmd = (
            self.ros_prefix()
            + "ros2 service call /unpause_physics std_srvs/srv/Empty \"{}\""
        )
        self.run_one_shot_command("unpause_physics", cmd)

    def kill_node(self):
        pattern = self.edt_kill_node.text().strip()
        if not pattern:
            QMessageBox.warning(self, "Thi?u tên node", "B?n c?n nh?p tên node/pattern.")
            return
        safe_pattern = shlex.quote(pattern)
        cmd = self.ros_prefix() + f"pkill -f {safe_pattern}"
        self.append_log(f"Kill node/pattern: {pattern}")
        self.run_one_shot_command("kill_node", cmd)

    def start_all(self):
        self.start_platform()
        QTimer.singleShot(1200, self._start_all_layer2)

    def _start_all_layer2(self):
        self.start_operation_mode()
        if self.chk_auto_nav2.isChecked():
            self.start_nav2()

    def stop_all(self):
        self.proc_mgr.stop_all()
        self.stop_one_shot_processes()

    def save_map(self):
        raw_name = self.edt_save_map_name.text().strip()
        if not raw_name:
            QMessageBox.warning(self, "Thi?u tên", "B?n ch?a nh?p tên map c?n l?u.")
            return
        if not (
            self.proc_mgr.is_running(self.SLAM_MAPPING)
            or self.proc_mgr.is_running(self.SLAM_LOCALIZATION)
        ):
            QMessageBox.warning(
                self,
                "SLAM ch?a ch?y",
                "B?n c?n ch?y SLAM tr??c khi l?u map.",
            )
            return

        if os.path.isabs(raw_name):
            target_stem = os.path.splitext(raw_name)[0]
        else:
            target_stem = os.path.splitext(os.path.join(self.map_dir, raw_name))[0]
        target_dir = os.path.dirname(target_stem)
        if target_dir:
            os.makedirs(target_dir, exist_ok=True)

        amcl_stem = f"{target_stem}{MAP_SUFFIX_AMCL}"
        localization_stem = f"{target_stem}{MAP_SUFFIX_LOCALIZATION}"
        serialize_req = shlex.quote(f'{{filename: "{localization_stem}"}}')

        cmd = (
            self.ros_prefix()
            + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(amcl_stem)}"
            + " && "
            + f"ros2 run nav2_map_server map_saver_cli -f {shlex.quote(localization_stem)}"
            + " && "
            + "ros2 service call /slam_toolbox/serialize_map "
            + "slam_toolbox/srv/SerializePoseGraph "
            + serialize_req
        )
        self.append_log(
            "L?u map 2 b?n:"
            f" AMCL='{amcl_stem}.yaml/.pgm',"
            f" Localization='{localization_stem}.yaml/.pgm + .posegraph/.data'"
        )

        def _after_save():
            self.refresh_map_list()
            amcl_yaml = f"{amcl_stem}.yaml"
            amcl_yml = f"{amcl_stem}.yml"
            selected = amcl_yaml if os.path.exists(amcl_yaml) else amcl_yml
            if os.path.exists(selected):
                self.selected_map_file = selected
                idx = self.cmb_map_files.findData(selected)
                if idx >= 0:
                    self.cmb_map_files.setCurrentIndex(idx)
                self.lbl_selected_map.setText(f"Map ?ang dùng: {selected}")
                self.append_log(f"Auto ch?n map AMCL: {selected}")

        self.run_one_shot_command("save_map", cmd, _after_save)

    def run_one_shot_command(self, name, cmd, done_callback=None):
        def _worker():
            self.append_log(f"[{name}] RUN")
            self.append_log(f"CMD: {cmd}")
            proc = None
            try:
                proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    preexec_fn=os.setsid,
                )
                with self.one_shot_lock:
                    self.one_shot_procs.append(proc)

                for line in iter(proc.stdout.readline, ""):
                    if line == "" and proc.poll() is not None:
                        break
                    self.append_log(f"[{name}] {line.rstrip()}")
                ret = proc.wait()
                self.append_log(f"[{name}] exit_code={ret}")
            except Exception as exc:
                self.append_log(f"[{name}] error: {exc}")
            finally:
                if proc is not None:
                    with self.one_shot_lock:
                        if proc in self.one_shot_procs:
                            self.one_shot_procs.remove(proc)
            if done_callback:
                if not self.is_shutting_down:
                    QTimer.singleShot(0, done_callback)

        threading.Thread(target=_worker, daemon=True).start()

    def _set_status_label(self, label, name, is_on):
        label.setText(f"{name}: {'ON' if is_on else 'OFF'}")
        label.setStyleSheet(f"color: {'lime' if is_on else 'red'}; font-weight: bold;")

    def refresh_status(self):
        platform_on = self.proc_mgr.is_running(self.LAYER1_SIM) or self.proc_mgr.is_running(
            self.LAYER1_REAL
        )
        mode_on = (
            self.proc_mgr.is_running(self.SLAM_MAPPING)
            or self.proc_mgr.is_running(self.SLAM_LOCALIZATION)
            or self.proc_mgr.is_running(self.AMCL)
        )
        nav2_on = self.proc_mgr.is_running(self.NAV2)
        if (not self.is_sim_mode()) and self.cmb_operation_mode.currentText() == "Localization":
            nav2_on = nav2_on or self.proc_mgr.is_running(self.SLAM_LOCALIZATION) or self.proc_mgr.is_running(
                self.AMCL
            )

        self._set_status_label(self.lbl_platform_status, "Layer1 Odom", platform_on)
        self._set_status_label(self.lbl_mode_status, "Layer2 Mode", mode_on)
        self._set_status_label(self.lbl_nav2_status, "Nav2", nav2_on)
        if self.is_sim_mode():
            self.lbl_odom_status.setText("Odom source: gazebo -> /odom")
        else:
            odom_source = self.get_selected_odom_source()
            wheel_mode = self.get_selected_esp_wheel_odom_mode()
            if odom_source == "esp":
                self.lbl_odom_status.setText(f"Odom source: esp/{wheel_mode} -> /odom")
            else:
                self.lbl_odom_status.setText(f"Odom source: {odom_source} -> /odom")

    def stop_one_shot_processes(self):
        with self.one_shot_lock:
            procs = list(self.one_shot_procs)

        for proc in procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass

        time.sleep(0.4)
        for proc in procs:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass

        with self.one_shot_lock:
            self.one_shot_procs = []

    def force_kill_known_ros_processes(self):
        # Fallback cleanup: ensure no launch/node started by GUI remains alive.
        patterns = [
            "ros2 launch mo_hinh virtual_robot_gazebo.launch.py",
            "ros2 launch mo_hinh real_odom.launch.py",
            "ros2 launch mo_hinh real_slam.launch.py",
            "ros2 launch mo_hinh real_localization_nav2.launch.py",
            "ros2 launch slam_toolbox online_async_launch.py",
            "ros2 launch slam_toolbox localization_launch.py",
            "ros2 launch nav2_bringup localization_launch.py",
            "ros2 launch nav2_bringup navigation_launch.py",
            "ros2 service call /unpause_physics",
        ]

        for sig in ("TERM", "KILL"):
            for pat in patterns:
                safe_pat = shlex.quote(pat)
                subprocess.run(
                    ["bash", "-lc", f"pkill -{sig} -f {safe_pat} >/dev/null 2>&1 || true"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    text=True,
                )
            if sig == "TERM":
                time.sleep(0.6)

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

        try:
            if self.status_timer.isActive():
                self.status_timer.stop()
        except Exception:
            pass
        try:
            self.stop_all()
        except Exception:
            pass
        try:
            self.stop_one_shot_processes()
        except Exception:
            pass
        try:
            self.force_kill_known_ros_processes()
        except Exception:
            pass
        try:
            self.cleanup_temp_files()
        except Exception:
            pass
        try:
            if self.ros_node is not None:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

