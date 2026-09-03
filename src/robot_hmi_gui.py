#!/usr/bin/env python3

import sys
import threading
import time
import os
import yaml

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QPoint
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, 
    QPushButton, QLabel, QGroupBox, QComboBox, QLineEdit, QProgressBar, QTextEdit
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QPen
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from robot_interfaces.msg import RobotStatus, CustomerStatus
from robot_interfaces.srv import SetLanguage, FinishSetup, SaveLocation, DeleteLocation, SaveRoute, GetLocationList
from robot_interfaces.action import DirectGo

class MapViewer(QLabel):
    def __init__(self, ros_node_getter, parent=None):
        super().__init__(parent)
        self.get_ros_node = ros_node_getter
        self.setMinimumSize(400, 300)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("background-color: #1e1e1e; border: 1px solid #555; color: white;")
        self.setText("Chưa có dữ liệu /map")

        self.map_pixmap = None
        self.map_resolution = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.scale_factor = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.saved_locations = {}

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
        
        # Vẽ các điểm đã lưu
        if self.saved_locations and self.map_resolution is not None:
            painter.setRenderHint(QPainter.Antialiasing)
            pen = QPen(QColor(0, 255, 0)) # Xanh lá cho text
            pen.setWidth(2)
            painter.setPen(pen)
            painter.setBrush(QColor(255, 0, 0, 200)) # Đỏ cho điểm
            
            for name, data in self.saved_locations.items():
                if 'x' in data and 'y' in data:
                    wx = data['x']
                    wy = data['y']
                    # Chuyển đổi world -> map grid
                    mx = (wx - self.map_origin_x) / self.map_resolution
                    my = (wy - self.map_origin_y) / self.map_resolution
                    
                    # Chuyển đổi map grid -> widget pixel
                    my_img = self.map_height - 1 - my
                    px = mx * self.scale_factor + self.offset_x
                    py = my_img * self.scale_factor + self.offset_y
                    
                    if 0 <= px < self.width() and 0 <= py < self.height():
                        painter.drawEllipse(QPoint(int(px), int(py)), 4, 4)
                        painter.drawText(int(px) + 7, int(py) + 4, name)
                        
        painter.end()
        self.setPixmap(canvas)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.map_pixmap is not None:
            world = self.widget_to_world(event.pos().x(), event.pos().y())
            if world is None:
                return
            wx, wy = world
            ros_node = self.get_ros_node()
            if ros_node is not None:
                ros_node.nav2_go_xy(wx, wy)

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

class HmiRosNode(Node):
    """Node ROS 2 ngầm để giao tiếp với 6 Phase backend"""

    def __init__(self, log_callback=None, gui_callback=None, map_callback=None, loc_list_callback=None):
        super().__init__('robot_hmi_gui_node')
        self.log_callback = log_callback
        self.gui_callback = gui_callback
        self.map_callback = map_callback
        self.loc_list_callback = loc_list_callback
        
        # --- Phase 2: Config Manager ---
        self.cli_set_lang = self.create_client(SetLanguage, '/config/set_language')
        self.cli_finish_setup = self.create_client(FinishSetup, '/config/finish_setup')
        self.cli_save_location = self.create_client(SaveLocation, '/config/save_location')
        self.cli_del_location = self.create_client(DeleteLocation, '/config/delete_location')
        self.cli_save_route = self.create_client(SaveRoute, '/config/save_route')
        self.cli_get_loc_list = self.create_client(GetLocationList, '/config/get_location_list')
        
        # TF2 Listener for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Phase 3: System Monitor ---
        self.sub_status = self.create_subscription(RobotStatus, '/robot_status', self.cb_status, 10)
        
        # --- Phase 4: Navigation Manager ---
        self.act_direct_go = ActionClient(self, DirectGo, '/navigation/direct_go')
        self.act_nav2 = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # --- Phase 5: Mission Controller ---
        self.cli_start_patrol = self.create_client(Trigger, '/mission/start_patrol')
        self.cli_pause_patrol = self.create_client(Trigger, '/mission/pause_patrol')
        self.pub_customer = self.create_publisher(CustomerStatus, '/hmi/customer_status', 10)
        
        # --- Phase 6: Mapping ---
        self.cli_start_slam = self.create_client(Trigger, '/mapping/start_slam')
        self.cli_save_map = self.create_client(Trigger, '/mapping/save_map')
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.cb_map, map_qos)
        
        # --- Logging from Backend ---
        self.sub_mission_log = self.create_subscription(String, '/mission/status_log', self.cb_mission_log, 10)

    def cb_mission_log(self, msg):
        self.log(msg.data)

    def cb_map(self, msg):
        if self.map_callback:
            self.map_callback(msg)

    def log(self, msg):
        self.get_logger().info(msg)
        if self.log_callback:
            self.log_callback(msg)

    def cb_status(self, msg):
        # Khi nhận được status từ ROS, đẩy nó qua GUI callback
        if self.gui_callback:
            self.gui_callback(msg)

    def cb_get_location_list_done(self, future):
        try:
            res = future.result()
            if res.success:
                if self.loc_list_callback:
                    self.loc_list_callback(res.locations)
        except Exception as e:
            self.log(f"Lỗi khi lấy danh sách điểm: {e}")

    def get_location_list(self):
        if self.cli_get_loc_list.service_is_ready():
            req = GetLocationList.Request()
            future = self.cli_get_loc_list.call_async(req)
            future.add_done_callback(self.cb_get_location_list_done)

    # --- Các hàm gọi Service Phase 2 ---
    def set_language(self, lang_code):
        if self.cli_set_lang.wait_for_service(timeout_sec=1.0):
            req = SetLanguage.Request()
            req.language = lang_code
            self.cli_set_lang.call_async(req)
            self.log(f"Đã gửi yêu cầu đổi ngôn ngữ thành: {lang_code}")
        else:
            self.log("Lỗi: Service /config/set_language chưa bật.")

    def finish_setup(self):
        if self.cli_finish_setup.wait_for_service(timeout_sec=1.0):
            req = FinishSetup.Request()
            self.cli_finish_setup.call_async(req)
            self.log("Đã gửi yêu cầu Hoàn tất Setup")
        else:
            self.log("Lỗi: Service /config/finish_setup chưa bật.")

    def save_current_location(self, name):
        if not self.cli_save_location.wait_for_service(timeout_sec=1.0):
            self.log("Lỗi: Service /config/save_location chưa sẵn sàng.")
            return
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            
            # Quat to Yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            req = SaveLocation.Request()
            req.name = name
            req.x = float(x)
            req.y = float(y)
            req.yaw = float(yaw)
            self.cli_save_location.call_async(req)
            self.log(f"Đã lưu điểm '{name}' tại: X={x:.2f}, Y={y:.2f}, Yaw={yaw:.2f}")
        except TransformException as ex:
            self.log(f"Lỗi đọc TF (map -> base_footprint): {ex}")

    def delete_location(self, name):
        if not self.cli_del_location.wait_for_service(timeout_sec=1.0):
            self.log("Lỗi: Service /config/delete_location chưa bật.")
            return
        req = DeleteLocation.Request()
        req.name = name
        self.cli_del_location.call_async(req)
        self.log(f"Đã gửi lệnh xóa điểm: {name}")

    def save_route(self, route_name, waypoints):
        if not self.cli_save_route.wait_for_service(timeout_sec=1.0):
            self.log("Lỗi: Service /config/save_route chưa bật.")
            return
        req = SaveRoute.Request()
        req.route_name = route_name
        req.waypoints = waypoints
        self.cli_save_route.call_async(req)
        self.log(f"Đã lưu lộ trình '{route_name}': {waypoints}")

    # --- Actions Phase 4 ---
    def direct_go(self, location):
        if not self.act_direct_go.wait_for_server(timeout_sec=1.0):
            self.log("Lỗi: Action Server /navigation/direct_go chưa sẵn sàng.")
            return
        goal_msg = DirectGo.Goal()
        goal_msg.target_location = location
        self.act_direct_go.send_goal_async(goal_msg)
        self.log(f"Đã gửi lệnh: Đi tới điểm '{location}'")

    def nav2_go_xy(self, x, y):
        if not self.act_nav2.wait_for_server(timeout_sec=1.0):
            self.log("Lỗi: Action Server /navigate_to_pose chưa sẵn sàng.")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0 # Hướng mặc định
        self.act_nav2.send_goal_async(goal)
        self.log(f"Đã click Map: Đi tới tọa độ X={x:.2f}, Y={y:.2f} bằng Nav2")

    # --- Actions Phase 5 ---
    def start_patrol(self):
        if self.cli_start_patrol.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.cli_start_patrol.call_async(req)
            self.log("Đã gửi lệnh: Bắt đầu Tuần tra")
        else:
            self.log("Lỗi: Service /mission/start_patrol chưa bật.")
            
    def pause_patrol(self):
        if self.cli_pause_patrol.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.cli_pause_patrol.call_async(req)
            self.log("Đã gửi lệnh: Dừng/Tạm ngưng Tuần tra")
            
    def serve_customer(self):
        msg = CustomerStatus()
        msg.customer_detected = True
        self.pub_customer.publish(msg)
        self.log("Đã gửi tín hiệu: Khách hàng cần phục vụ (Customer detected = True)")

    # --- Actions Phase 6 ---
    def start_slam(self):
        if self.cli_start_slam.wait_for_service(timeout_sec=1.0):
            req = Trigger.Request()
            self.cli_start_slam.call_async(req)
            self.log("Đã gửi lệnh Start SLAM")
        else:
            self.log("Lỗi: Service /mapping/start_slam chưa sẵn sàng.")

class RobotHmiGui(QWidget):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(object)
    map_signal = pyqtSignal(object)
    location_list_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot HMI - Control Panel (6 Phases)")
        self.resize(1100, 750)
        
        # ROS Node
        self.ros_node = None
        self.ros_thread = None
        
        self.init_ui()
        self.init_ros()
        
        self.log_signal.connect(self.append_log)
        self.status_signal.connect(self.update_monitor_ui)
        self.map_signal.connect(self.update_map_ui)
        self.location_list_signal.connect(self.update_locations_ui)
        
        # Timer for refreshing location list
        self.loc_timer = QTimer(self)
        self.loc_timer.timeout.connect(self.refresh_locations)
        self.loc_timer.start(2000)

    def get_ros_node(self):
        return self.ros_node

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        # Tiêu đề
        title = QLabel("🤖 Hệ thống Điều khiển Robot Dịch vụ")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 24px; font-weight: bold; margin: 10px;")
        main_layout.addWidget(title)
        
        # Tabs
        self.tabs = QTabWidget()
        main_layout.addWidget(self.tabs)
        
        self.setup_tab_config()
        self.setup_tab_monitor()
        self.setup_tab_mapping_nav()
        self.setup_tab_mission()
        
        # Khung Log
        log_group = QGroupBox("Terminal / Logs")
        log_layout = QVBoxLayout()
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;")
        log_layout.addWidget(self.txt_log)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group, stretch=1)

    def setup_tab_config(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        group = QGroupBox("⚙️ Phase 2: Cấu hình (Config Manager)")
        g_layout = QVBoxLayout()
        
        # Ngôn ngữ
        h1 = QHBoxLayout()
        h1.addWidget(QLabel("Ngôn ngữ hiện tại:"))
        self.cmb_lang = QComboBox()
        self.cmb_lang.addItems(["vi", "en"])
        h1.addWidget(self.cmb_lang)
        btn_set_lang = QPushButton("Lưu Ngôn ngữ")
        btn_set_lang.clicked.connect(self.on_set_language)
        h1.addWidget(btn_set_lang)
        g_layout.addLayout(h1)
        
        # Lưu / Xóa Vị Trí Hiện Tại
        h2 = QHBoxLayout()
        h2.addWidget(QLabel("Tên điểm (vd: point1):"))
        self.txt_loc_name = QLineEdit("point1")
        h2.addWidget(self.txt_loc_name)
        
        btn_save_loc = QPushButton("Lưu Tọa Độ")
        btn_save_loc.setStyleSheet("background-color: #2196F3; color: white; padding: 8px;")
        btn_save_loc.clicked.connect(self.on_save_location)
        h2.addWidget(btn_save_loc)
        
        btn_del_loc = QPushButton("❌ Xóa Điểm Này")
        btn_del_loc.setStyleSheet("background-color: #F44336; color: white; padding: 8px;")
        btn_del_loc.clicked.connect(self.on_delete_location)
        h2.addWidget(btn_del_loc)
        
        g_layout.addLayout(h2)
        
        # Setup
        btn_finish = QPushButton("✅ Hoàn tất Setup (Finish Setup)")
        btn_finish.setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;")
        btn_finish.clicked.connect(self.on_finish_setup)
        g_layout.addWidget(btn_finish)
        
        group.setLayout(g_layout)
        layout.addWidget(group)
        layout.addStretch()
        self.tabs.addTab(tab, "Config & Setup")

    def setup_tab_monitor(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        group = QGroupBox("🩺 Phase 3: Giám sát Hệ thống (System Monitor)")
        g_layout = QVBoxLayout()
        
        # Battery
        h_bat = QHBoxLayout()
        h_bat.addWidget(QLabel("🔋 Pin (Battery):"))
        self.bar_battery = QProgressBar()
        self.bar_battery.setValue(100)
        h_bat.addWidget(self.bar_battery)
        g_layout.addLayout(h_bat)
        
        # Water
        h_water = QHBoxLayout()
        h_water.addWidget(QLabel("💧 Mức nước (Water Level):"))
        self.lbl_water = QLabel("OK (Đầy)")
        self.lbl_water.setStyleSheet("color: green; font-weight: bold;")
        h_water.addWidget(self.lbl_water)
        g_layout.addLayout(h_water)
        
        # Sensors
        self.lbl_sensors = QLabel("Cảm biến: Lidar [OK] | Odom [OK]")
        g_layout.addWidget(self.lbl_sensors)
        
        group.setLayout(g_layout)
        layout.addWidget(group)
        layout.addStretch()
        self.tabs.addTab(tab, "System Monitor")

    def setup_tab_mapping_nav(self):
        tab = QWidget()
        layout = QHBoxLayout(tab)
        
        # Left Panel: Controls
        left_panel = QVBoxLayout()
        
        group_map = QGroupBox("🗺️ Phase 6: Mapping")
        g_map_layout = QVBoxLayout()
        btn_start_slam = QPushButton("Bắt đầu SLAM (Start)")
        btn_start_slam.clicked.connect(self.on_start_slam)
        btn_save_map = QPushButton("Lưu Bản đồ (Save)")
        g_map_layout.addWidget(btn_start_slam)
        g_map_layout.addWidget(btn_save_map)
        group_map.setLayout(g_map_layout)
        left_panel.addWidget(group_map)
        
        group_nav = QGroupBox("📍 Phase 4: Navigation")
        g_nav_layout = QVBoxLayout()
        self.cmb_locations = QComboBox()
        self.cmb_locations.addItems(["Đang tải..."])
        btn_go = QPushButton("🚀 Đi tới điểm này (Direct Go)")
        btn_go.clicked.connect(self.on_direct_go)
        g_nav_layout.addWidget(QLabel("Chọn điểm đến:"))
        g_nav_layout.addWidget(self.cmb_locations)
        g_nav_layout.addWidget(btn_go)
        group_nav.setLayout(g_nav_layout)
        left_panel.addWidget(group_nav)
        
        left_panel.addStretch()
        
        # Right Panel: Map Viewer
        right_panel = QVBoxLayout()
        self.lbl_map = MapViewer(ros_node_getter=self.get_ros_node)
        right_panel.addWidget(self.lbl_map)
        
        layout.addLayout(left_panel, stretch=1)
        layout.addLayout(right_panel, stretch=2)
        self.tabs.addTab(tab, "Mapping & Navigation")

    def setup_tab_mission(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        group = QGroupBox("🧠 Phase 5: Bộ não Trung tâm (Mission Control)")
        g_layout = QVBoxLayout()
        
        # Route Input
        h_route = QHBoxLayout()
        h_route.addWidget(QLabel("Lộ trình tuần tra (cách nhau bởi phẩy):"))
        self.txt_route = QLineEdit("point1, point2")
        h_route.addWidget(self.txt_route)
        g_layout.addLayout(h_route)
        
        btn_patrol = QPushButton("🔂 Lưu Lộ Trình & Bắt đầu Tuần tra")
        btn_patrol.setStyleSheet("padding: 15px; font-size: 16px;")
        btn_patrol.clicked.connect(self.on_start_patrol)
        
        btn_patrol_pause = QPushButton("⏸️ Tạm dừng Tuần tra (Pause Patrol)")
        btn_patrol_pause.setStyleSheet("padding: 15px; font-size: 16px;")
        btn_patrol_pause.clicked.connect(self.on_pause_patrol)
        
        btn_serve = QPushButton("☕ Phục vụ Khách hàng (Serve Customer)")
        btn_serve.setStyleSheet("padding: 15px; font-size: 16px;")
        btn_serve.clicked.connect(self.on_serve_customer)
        
        btn_estop = QPushButton("🛑 DỪNG KHẨN CẤP (Cancel Goals)")
        btn_estop.setStyleSheet("background-color: red; color: white; padding: 20px; font-size: 20px; font-weight: bold;")
        
        g_layout.addWidget(btn_patrol)
        g_layout.addWidget(btn_patrol_pause)
        g_layout.addWidget(btn_serve)
        g_layout.addWidget(btn_estop)
        
        group.setLayout(g_layout)
        layout.addWidget(group)
        layout.addStretch()
        self.tabs.addTab(tab, "Mission Control")

    def append_log(self, text):
        self.txt_log.append(text)
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def emit_log(self, text):
        self.log_signal.emit(text)

    def init_ros(self):
        try:
            rclpy.init(args=sys.argv)
            self.ros_node = HmiRosNode(
                log_callback=self.emit_log, 
                gui_callback=self.status_signal.emit,
                map_callback=self.map_signal.emit,
                loc_list_callback=self.location_list_signal.emit
            )
            self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.ros_thread.start()
            self.emit_log("ROS 2 Node khởi tạo thành công.")
        except Exception as e:
            self.emit_log(f"Lỗi khởi tạo ROS 2: {e}")

    def spin_ros(self):
        if self.ros_node:
            rclpy.spin(self.ros_node)

    # --- Phase 3 Monitor Update ---
    def update_monitor_ui(self, status_msg):
        # Cập nhật thanh Pin
        self.bar_battery.setValue(int(status_msg.battery_percent))
        
        # Cập nhật nước
        if status_msg.water_low:
            self.lbl_water.setText("CẢNH BÁO: HẾT NƯỚC!")
            self.lbl_water.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.lbl_water.setText("OK (Có nước)")
            self.lbl_water.setStyleSheet("color: green; font-weight: bold;")
            
        # Cập nhật cảm biến
        l_status = "OK" if status_msg.lidar_ok else "LỖI"
        o_status = "OK" if status_msg.odom_ok else "LỖI"
        self.lbl_sensors.setText(f"Cảm biến: Lidar [{l_status}] | Odom [{o_status}]")

    def update_map_ui(self, occ_grid):
        self.lbl_map.update_map(occ_grid)

    def update_locations_ui(self, loc_list):
        current_items = [self.cmb_locations.itemText(i) for i in range(self.cmb_locations.count())]
        
        if not loc_list:
            if current_items != ["Không có điểm nào"] and current_items != ["Đang tải..."]:
                self.cmb_locations.clear()
                self.cmb_locations.addItem("Không có điểm nào")
            return
            
        if current_items != loc_list:
            current_text = self.cmb_locations.currentText()
            self.cmb_locations.clear()
            self.cmb_locations.addItems(loc_list)
            idx = self.cmb_locations.findText(current_text)
            if idx >= 0:
                self.cmb_locations.setCurrentIndex(idx)

    def refresh_locations(self):
        if self.ros_node:
            self.ros_node.get_location_list()
            
        # Đọc locations.yaml để vẽ lên bản đồ
        yaml_path = os.path.join(os.path.expanduser('~'), 'robot_ws', 'config', 'locations.yaml')
        try:
            if os.path.exists(yaml_path):
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f) or {}
                    if data != self.lbl_map.saved_locations:
                        self.lbl_map.saved_locations = data
                        self.lbl_map.update_display()
        except Exception:
            pass

    # --- Actions Phase 2 ---
    def on_set_language(self):
        if self.ros_node:
            lang = self.cmb_lang.currentText()
            self.ros_node.set_language(lang)

    def on_finish_setup(self):
        if self.ros_node:
            self.ros_node.finish_setup()

    def on_save_location(self):
        if self.ros_node:
            name = self.txt_loc_name.text().strip()
            if name:
                self.ros_node.save_current_location(name)
            else:
                self.emit_log("Vui lòng nhập tên điểm trước khi lưu.")
                
    def on_delete_location(self):
        if self.ros_node:
            name = self.txt_loc_name.text().strip()
            if name:
                self.ros_node.delete_location(name)
            else:
                self.emit_log("Vui lòng nhập tên điểm để xóa.")

    # --- Actions Phase 4 ---
    def on_direct_go(self):
        if self.ros_node:
            loc = self.cmb_locations.currentText()
            self.ros_node.direct_go(loc)

    # --- Actions Phase 5 ---
    def on_start_patrol(self):
        if self.ros_node:
            route_str = self.txt_route.text().strip()
            if route_str:
                waypoints = [w.strip() for w in route_str.split(',')]
                self.ros_node.save_route("patrol_route", waypoints)
                # Đợi một chút để service kịp lưu trước khi trigger patrol
                QTimer.singleShot(500, self.ros_node.start_patrol)
            else:
                self.emit_log("Vui lòng nhập lộ trình trước khi tuần tra.")
            
    def on_pause_patrol(self):
        if self.ros_node:
            self.ros_node.pause_patrol()
            
    def on_serve_customer(self):
        if self.ros_node:
            self.ros_node.serve_customer()

    # --- Actions Phase 6 ---
    def on_start_slam(self):
        if self.ros_node:
            self.ros_node.start_slam()

    def closeEvent(self, event):
        if self.ros_node:
            self.ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotHmiGui()
    gui.show()
    sys.exit(app.exec_())
