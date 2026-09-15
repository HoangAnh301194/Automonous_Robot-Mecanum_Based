#!/usr/bin/env python3

import sys
import os
import time
import threading
import numpy as np
import cv2
import yaml

from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QSlider, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QFileDialog, QStatusBar, QMessageBox, QComboBox, QScrollArea,
    QTabWidget, QLineEdit, QCheckBox, QFormLayout
)
from PyQt5.QtGui import QImage, QPixmap, QFont

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs

class ImageSignalEmitter(QObject):
    image_received = pyqtSignal(np.ndarray, object)

class CameraProfile:
    def __init__(self, name="front"):
        self.name = name
        self.enabled = True
        if name == "front":
            self.depth_topic = "/front_camera/depth/image_raw"
            self.info_topic = "/front_camera/depth/camera_info"
            self.output_topic = "/scan_obstacles"
            self.min_range = 0.20
            self.max_range = 2.50
        else:
            self.depth_topic = "/rear_camera/depth/image_raw"
            self.info_topic = "/rear_camera/depth/camera_info"
            self.output_topic = "/rear_scan"
            self.min_range = 0.15
            self.max_range = 1.50
            
        self.target_frame = "base_footprint"
        self.pitch_deg = 50.0
        self.dist_offset = 0.0
        
        self.roi_x = 0
        self.roi_y = 0
        self.roi_w = 320
        self.roi_h = 240
        
        self.threshold = 80
        self.min_area = 150
        self.median_filter = 3
        self.morph_size = 3
        
        self.ground_frame = None
        self.ground_file_path = "None"
        
        self.raw_width = 0
        self.raw_height = 0

class DepthSubscriberNode(Node):
    def __init__(self, signal_emitter):
        super().__init__('depth_obstacle_debugger_node')
        self.signal_emitter = signal_emitter
        self.camera_info = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.info_sub = None
        self.image_sub = None
        self.laser_pub = None
        
        self.bridge = CvBridge()
        
        # Default empty
        self.current_depth_topic = ""
        self.current_info_topic = ""
        self.current_output_topic = ""
        
        self.get_logger().info("Dual-Camera Debugger ROS 2 Node initialized.")

    def switch_camera(self, depth_topic, info_topic, output_topic):
        self.get_logger().info(f"Switching topics -> Depth: {depth_topic}, Info: {info_topic}, Scan: {output_topic}")
        
        if self.info_sub:
            self.destroy_subscription(self.info_sub)
        if self.image_sub:
            self.destroy_subscription(self.image_sub)
        if self.laser_pub:
            self.destroy_publisher(self.laser_pub)
            
        self.camera_info = None
        self.current_depth_topic = depth_topic
        self.current_info_topic = info_topic
        self.current_output_topic = output_topic
        
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, 10)
        self.image_sub = self.create_subscription(Image, depth_topic, self.listener_callback, 10)
        self.laser_pub = self.create_publisher(LaserScan, output_topic, 10)

    def info_callback(self, msg):
        self.camera_info = msg

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            info_dict = None
            if self.camera_info is not None:
                info_dict = {
                    'fx': float(self.camera_info.k[0]),
                    'fy': float(self.camera_info.k[4]),
                    'cx': float(self.camera_info.k[2]),
                    'cy': float(self.camera_info.k[5]),
                    'frame_id': str(self.camera_info.header.frame_id)
                }
            self.signal_emitter.image_received.emit(cv_image, info_dict)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")

    def publish_scan(self, scan_msg):
        if self.laser_pub:
            self.laser_pub.publish(scan_msg)


class ClickableLabel(QLabel):
    mouse_moved = pyqtSignal(int, int)
    mouse_pressed = pyqtSignal(int, int)
    mouse_dragged = pyqtSignal(int, int, int, int)
    mouse_released = pyqtSignal(int, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.drag_start = None

    def mouseMoveEvent(self, event):
        x, y = event.x(), event.y()
        self.mouse_moved.emit(x, y)
        if self.drag_start is not None and event.buttons() & Qt.LeftButton:
            self.mouse_dragged.emit(self.drag_start[0], self.drag_start[1], x, y)
        super().mouseMoveEvent(event)

    def mousePressEvent(self, event):
        x, y = event.x(), event.y()
        if event.button() == Qt.LeftButton:
            self.drag_start = (x, y)
            self.mouse_pressed.emit(x, y)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        x, y = event.x(), event.y()
        if event.button() == Qt.LeftButton and self.drag_start is not None:
            self.mouse_released.emit(self.drag_start[0], self.drag_start[1], x, y)
            self.drag_start = None
        super().mouseReleaseEvent(event)


class ProfileWidget(QWidget):
    param_changed = pyqtSignal()
    preview_requested = pyqtSignal()
    
    def __init__(self, profile: CameraProfile):
        super().__init__()
        self.profile = profile
        self.init_ui()
        self.update_ui_from_profile()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        
        # 1. Topics Config
        grp_topics = QGroupBox(f"{self.profile.name.capitalize()} Topics")
        form_topics = QFormLayout(grp_topics)
        
        self.chk_enable = QCheckBox("Enable Camera in Pipeline")
        self.chk_enable.stateChanged.connect(self.on_change)
        form_topics.addRow(self.chk_enable)
        
        self.edit_depth = QLineEdit()
        self.edit_depth.textChanged.connect(self.on_change)
        form_topics.addRow("Depth Topic:", self.edit_depth)
        
        self.edit_info = QLineEdit()
        self.edit_info.textChanged.connect(self.on_change)
        form_topics.addRow("Camera Info:", self.edit_info)
        
        self.edit_output = QLineEdit()
        self.edit_output.textChanged.connect(self.on_change)
        form_topics.addRow("Output Scan:", self.edit_output)
        
        self.cmb_frame = QComboBox()
        self.cmb_frame.setEditable(True)
        self.cmb_frame.addItems([
            'base_footprint',
            'base_link',
            'camera_link',
            'camera_depth_optical_frame',
            'front_camera_link',
            'rear_camera_link'
        ])
        self.cmb_frame.currentTextChanged.connect(self.on_change)
        form_topics.addRow("Target Frame:", self.cmb_frame)
        
        layout.addWidget(grp_topics)
        
        # 2. ROI Config
        grp_roi = QGroupBox("Region of Interest (ROI)")
        grid_roi = QGridLayout(grp_roi)
        
        self.sld_roi_x, self.lbl_rx = self.add_slider(grid_roi, 0, "ROI X:", 0, 1000)
        self.sld_roi_y, self.lbl_ry = self.add_slider(grid_roi, 1, "ROI Y:", 0, 1000)
        self.sld_roi_w, self.lbl_rw = self.add_slider(grid_roi, 2, "Width:", 1, 1000)
        self.sld_roi_h, self.lbl_rh = self.add_slider(grid_roi, 3, "Height:", 1, 1000)
        layout.addWidget(grp_roi)
        
        # 3. Tuning Parameters
        grp_tune = QGroupBox("Parameters")
        grid_tune = QGridLayout(grp_tune)
        
        self.sld_min_range, self.lbl_min_r = self.add_slider(grid_tune, 0, "Min Range(cm):", 0, 1000)
        self.sld_max_range, self.lbl_max_r = self.add_slider(grid_tune, 1, "Max Range(cm):", 10, 1000)
        self.sld_thresh, self.lbl_thresh = self.add_slider(grid_tune, 2, "Threshold(mm):", 5, 500)
        self.sld_min_area, self.lbl_area = self.add_slider(grid_tune, 3, "Min Area(px):", 10, 5000)
        self.sld_median, self.lbl_median = self.add_slider(grid_tune, 4, "Median Filter:", 1, 15, step=2)
        self.sld_morph, self.lbl_morph = self.add_slider(grid_tune, 5, "Morph Size:", 0, 15)
        self.sld_pitch, self.lbl_pitch = self.add_slider(grid_tune, 6, "Pitch (deg):", -90, 90)
        layout.addWidget(grp_tune)
        
        # 4. Actions
        grp_actions = QGroupBox("Actions")
        vbox = QVBoxLayout(grp_actions)
        
        self.lbl_ground = QLabel("Ground Loaded: No")
        self.lbl_ground.setStyleSheet("color: #ff5555; font-weight: bold;")
        vbox.addWidget(self.lbl_ground)
        
        self.btn_capture = QPushButton("Capture Ground Reference")
        self.btn_load_ground = QPushButton("Load Ground from File")
        self.btn_preview = QPushButton(f"Preview {self.profile.name.upper()} Camera")
        self.btn_save_yaml = QPushButton("Save Config (YAML)")
        self.btn_load_yaml = QPushButton("Load Config (YAML)")
        
        self.btn_preview.setStyleSheet("background-color: #f39c12; color: #fff;")
        
        vbox.addWidget(self.btn_capture)
        vbox.addWidget(self.btn_load_ground)
        vbox.addWidget(self.btn_preview)
        vbox.addWidget(self.btn_save_yaml)
        vbox.addWidget(self.btn_load_yaml)
        
        layout.addWidget(grp_actions)
        layout.addStretch()
        
    def add_slider(self, grid, row, label, vmin, vmax, step=1):
        grid.addWidget(QLabel(label), row, 0)
        sld = QSlider(Qt.Horizontal)
        sld.setRange(vmin, vmax)
        sld.setSingleStep(step)
        sld.valueChanged.connect(self.on_change)
        grid.addWidget(sld, row, 1)
        lbl = QLabel(str(vmin))
        grid.addWidget(lbl, row, 2)
        return sld, lbl

    def on_change(self):
        # Force odd median
        if self.sld_median.value() > 1 and self.sld_median.value() % 2 == 0:
            self.sld_median.blockSignals(True)
            self.sld_median.setValue(self.sld_median.value() + 1)
            self.sld_median.blockSignals(False)
            
        self.lbl_rx.setText(str(self.sld_roi_x.value()))
        self.lbl_ry.setText(str(self.sld_roi_y.value()))
        self.lbl_rw.setText(str(self.sld_roi_w.value()))
        self.lbl_rh.setText(str(self.sld_roi_h.value()))
        
        self.lbl_min_r.setText(f"{self.sld_min_range.value()/100.0:.2f}m")
        self.lbl_max_r.setText(f"{self.sld_max_range.value()/100.0:.2f}m")
        self.lbl_thresh.setText(str(self.sld_thresh.value()))
        self.lbl_area.setText(str(self.sld_min_area.value()))
        self.lbl_median.setText(str(self.sld_median.value()))
        self.lbl_morph.setText(str(self.sld_morph.value()))
        self.lbl_pitch.setText(str(self.sld_pitch.value()))
        
        self.commit_to_profile()
        self.param_changed.emit()

    def commit_to_profile(self):
        self.profile.enabled = self.chk_enable.isChecked()
        self.profile.depth_topic = self.edit_depth.text()
        self.profile.info_topic = self.edit_info.text()
        self.profile.output_topic = self.edit_output.text()
        self.profile.target_frame = self.cmb_frame.currentText()
        
        self.profile.roi_x = self.sld_roi_x.value()
        self.profile.roi_y = self.sld_roi_y.value()
        self.profile.roi_w = self.sld_roi_w.value()
        self.profile.roi_h = self.sld_roi_h.value()
        
        self.profile.min_range = self.sld_min_range.value() / 100.0
        self.profile.max_range = self.sld_max_range.value() / 100.0
        
        self.profile.threshold = self.sld_thresh.value()
        self.profile.min_area = self.sld_min_area.value()
        self.profile.median_filter = self.sld_median.value()
        self.profile.morph_size = self.sld_morph.value()
        self.profile.pitch_deg = self.sld_pitch.value()

    def update_ui_from_profile(self):
        self.chk_enable.blockSignals(True)
        self.edit_depth.blockSignals(True)
        self.edit_info.blockSignals(True)
        self.edit_output.blockSignals(True)
        self.cmb_frame.blockSignals(True)
        
        self.sld_roi_x.blockSignals(True)
        self.sld_roi_y.blockSignals(True)
        self.sld_roi_w.blockSignals(True)
        self.sld_roi_h.blockSignals(True)
        self.sld_min_range.blockSignals(True)
        self.sld_max_range.blockSignals(True)
        self.sld_thresh.blockSignals(True)
        self.sld_min_area.blockSignals(True)
        self.sld_median.blockSignals(True)
        self.sld_morph.blockSignals(True)
        self.sld_pitch.blockSignals(True)

        self.chk_enable.setChecked(self.profile.enabled)
        self.edit_depth.setText(self.profile.depth_topic)
        self.edit_info.setText(self.profile.info_topic)
        self.edit_output.setText(self.profile.output_topic)
        self.cmb_frame.setCurrentText(self.profile.target_frame)
        
        if self.profile.raw_width > 0:
            self.sld_roi_x.setRange(0, self.profile.raw_width - 1)
            self.sld_roi_w.setRange(1, self.profile.raw_width)
            self.sld_roi_y.setRange(0, self.profile.raw_height - 1)
            self.sld_roi_h.setRange(1, self.profile.raw_height)
            
        self.sld_roi_x.setValue(int(self.profile.roi_x))
        self.sld_roi_y.setValue(int(self.profile.roi_y))
        self.sld_roi_w.setValue(int(self.profile.roi_w))
        self.sld_roi_h.setValue(int(self.profile.roi_h))
        
        self.sld_min_range.setValue(int(self.profile.min_range * 100))
        self.sld_max_range.setValue(int(self.profile.max_range * 100))
        self.sld_thresh.setValue(int(self.profile.threshold))
        self.sld_min_area.setValue(int(self.profile.min_area))
        self.sld_median.setValue(int(self.profile.median_filter))
        self.sld_morph.setValue(int(self.profile.morph_size))
        self.sld_pitch.setValue(int(self.profile.pitch_deg))

        self.chk_enable.blockSignals(False)
        self.edit_depth.blockSignals(False)
        self.edit_info.blockSignals(False)
        self.edit_output.blockSignals(False)
        self.cmb_frame.blockSignals(False)
        
        self.sld_roi_x.blockSignals(False)
        self.sld_roi_y.blockSignals(False)
        self.sld_roi_w.blockSignals(False)
        self.sld_roi_h.blockSignals(False)
        self.sld_min_range.blockSignals(False)
        self.sld_max_range.blockSignals(False)
        self.sld_thresh.blockSignals(False)
        self.sld_min_area.blockSignals(False)
        self.sld_median.blockSignals(False)
        self.sld_morph.blockSignals(False)
        self.sld_pitch.blockSignals(False)
        
        self.on_change()
        self.update_ground_label()
        
    def update_ground_label(self):
        if self.profile.ground_frame is not None:
            self.lbl_ground.setText(f"Ground Loaded: Yes ({os.path.basename(self.profile.ground_file_path)})")
            self.lbl_ground.setStyleSheet("color: #00ff66; font-weight: bold;")
        else:
            self.lbl_ground.setText("Ground Loaded: No")
            self.lbl_ground.setStyleSheet("color: #ff5555; font-weight: bold;")


class DebuggerGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Dual-Camera Obstacle Debugger GUI")
        self.resize(1500, 900)

        # Profiles
        self.profiles = {
            "front": CameraProfile("front"),
            "rear": CameraProfile("rear")
        }
        self.active_profile_key = "front"
        
        # State
        self.current_frame = None
        self.camera_intrinsics = None
        self.zoom = 1.0
        self.temp_drag_roi = None
        
        # Stats
        self.fps = 0.0
        self.last_frame_time = time.time()
        self.processing_time_ms = 0.0
        
        self.init_ui()
        self.apply_stylesheet()
        
        # Initial ROS bind
        self.switch_to_profile("front")

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        
        # Left: Preview Panels
        left_layout = QVBoxLayout()
        
        self.lbl_status_overlay = QLabel("PREVIEW: FRONT")
        self.lbl_status_overlay.setFont(QFont("Arial", 16, QFont.Bold))
        self.lbl_status_overlay.setStyleSheet("color: #f39c12; background: transparent;")
        self.lbl_status_overlay.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.lbl_status_overlay)
        
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.lbl_depth_viewer = ClickableLabel()
        self.lbl_depth_viewer.setAlignment(Qt.AlignCenter)
        self.lbl_depth_viewer.setText("Waiting for Camera Stream...")
        self.scroll_area.setWidget(self.lbl_depth_viewer)
        left_layout.addWidget(self.scroll_area, stretch=3)
        
        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(QLabel("Zoom:"))
        self.cmb_zoom = QComboBox()
        self.cmb_zoom.addItems(["50%", "75%", "100%", "125%", "150%", "200%"])
        self.cmb_zoom.setCurrentText("100%")
        self.cmb_zoom.currentIndexChanged.connect(self.on_zoom_changed)
        zoom_layout.addWidget(self.cmb_zoom)
        zoom_layout.addStretch()
        left_layout.addLayout(zoom_layout)
        
        # Lower monitors
        lower_monitors = QHBoxLayout()
        self.lbl_diff_viewer = QLabel("Diff")
        self.lbl_mask_viewer = QLabel("Mask")
        self.lbl_overlay_viewer = QLabel("Overlay")
        for lbl in (self.lbl_diff_viewer, self.lbl_mask_viewer, self.lbl_overlay_viewer):
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet("background-color: #1a1a1f;")
            lower_monitors.addWidget(lbl)
            
        left_layout.addLayout(lower_monitors, stretch=2)
        main_layout.addLayout(left_layout, stretch=3)
        
        # Right: Tabs for Front/Rear controls
        self.tabs = QTabWidget()
        self.tabs.setMinimumWidth(400)
        
        self.tab_front = ProfileWidget(self.profiles["front"])
        self.tab_rear = ProfileWidget(self.profiles["rear"])
        
        self.tabs.addTab(self.tab_front, "Front Camera")
        self.tabs.addTab(self.tab_rear, "Rear Camera")
        
        main_layout.addWidget(self.tabs, stretch=1)
        
        # Connect signals
        for tab in (self.tab_front, self.tab_rear):
            tab.param_changed.connect(self.process_pipeline)
            tab.btn_capture.clicked.connect(lambda t=tab: self.on_capture_ground(t))
            tab.btn_save_yaml.clicked.connect(lambda t=tab: self.on_save_yaml(t))
            tab.btn_load_yaml.clicked.connect(lambda t=tab: self.on_load_yaml(t))
            tab.btn_load_ground.clicked.connect(lambda t=tab: self.on_load_ground(t))
            
        self.tab_front.btn_preview.clicked.connect(lambda: self.switch_to_profile("front"))
        self.tab_rear.btn_preview.clicked.connect(lambda: self.switch_to_profile("rear"))
        
        self.lbl_depth_viewer.mouse_pressed.connect(self.on_mouse_pressed)
        self.lbl_depth_viewer.mouse_dragged.connect(self.on_mouse_dragged)
        self.lbl_depth_viewer.mouse_released.connect(self.on_mouse_released)
        
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

    def apply_stylesheet(self):
        stylesheet = """
        QMainWindow { background-color: #121214; color: #e0e0e6; }
        QWidget { color: #e0e0e6; font-family: 'Segoe UI', Arial, sans-serif; font-size: 13px; }
        QGroupBox { border: 2px solid #2d2d35; border-radius: 8px; margin-top: 10px; font-weight: bold; color: #00adb5; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
        QPushButton { background-color: #00adb5; color: #ffffff; border: none; border-radius: 4px; padding: 6px; font-weight: bold; }
        QPushButton:hover { background-color: #00cfd8; }
        QSlider::groove:horizontal { border: 1px solid #2c2c35; height: 6px; background: #1e1e24; border-radius: 3px; }
        QSlider::handle:horizontal { background: #00adb5; width: 14px; height: 14px; margin: -4px 0; border-radius: 7px; }
        QComboBox, QLineEdit { background-color: #1e1e24; border: 1px solid #2d2d35; border-radius: 4px; padding: 4px; color: #e0e0e6; }
        QTabWidget::pane { border: 1px solid #2d2d35; background: #121214; }
        QTabBar::tab { background: #1a1a1f; color: #888; padding: 8px 20px; border: 1px solid #2d2d35; }
        QTabBar::tab:selected { background: #00adb5; color: #fff; }
        """
        self.setStyleSheet(stylesheet)

    def switch_to_profile(self, key):
        self.active_profile_key = key
        prof = self.profiles[key]
        self.lbl_status_overlay.setText(f"PREVIEW: {key.upper()} CAMERA")
        self.tabs.setCurrentIndex(0 if key == "front" else 1)
        self.current_frame = None
        self.ros_node.switch_camera(prof.depth_topic, prof.info_topic, prof.output_topic)

    def get_active_tab(self):
        return self.tab_front if self.active_profile_key == "front" else self.tab_rear

    @pyqtSlot(np.ndarray, object)
    def handle_new_frame(self, cv_image, info_dict):
        self.current_frame = cv_image
        self.camera_intrinsics = info_dict
        
        prof = self.profiles[self.active_profile_key]
        h, w = cv_image.shape
        
        if prof.raw_width != w or prof.raw_height != h:
            prof.raw_width = w
            prof.raw_height = h
            self.get_active_tab().update_ui_from_profile()
            
        t_now = time.time()
        self.fps = 1.0 / max(1e-5, (t_now - self.last_frame_time))
        self.last_frame_time = t_now
        
        t_start = time.time()
        self.process_pipeline()
        self.processing_time_ms = (time.time() - t_start) * 1000.0
        
        self.update_status_bar()

    def update_status_bar(self):
        prof = self.profiles[self.active_profile_key]
        stat_text = (f"[{prof.name.upper()}] FPS: {self.fps:.1f} | Res: {prof.raw_width}x{prof.raw_height} | "
                     f"Proc: {self.processing_time_ms:.1f} ms | Target: {prof.target_frame}")
        self.status_bar.showMessage(stat_text)

    def on_zoom_changed(self):
        txt = self.cmb_zoom.currentText().replace("%", "")
        self.zoom = float(txt) / 100.0
        self.process_pipeline()

    # --- Mouse Events for ROI Drawing ---
    def map_to_raw(self, lbl_x, lbl_y):
        prof = self.profiles[self.active_profile_key]
        if self.zoom <= 0 or prof.raw_width == 0:
            return 0, 0
        rx = int(lbl_x / self.zoom)
        ry = int(lbl_y / self.zoom)
        return max(0, min(rx, prof.raw_width - 1)), max(0, min(ry, prof.raw_height - 1))

    def on_mouse_pressed(self, x, y):
        rx, ry = self.map_to_raw(x, y)
        self.temp_drag_roi = (rx, ry, rx, ry)

    def on_mouse_dragged(self, sx, sy, cx, cy):
        rsx, rsy = self.map_to_raw(sx, sy)
        rcx, rcy = self.map_to_raw(cx, cy)
        self.temp_drag_roi = (rsx, rsy, rcx, rcy)
        self.process_pipeline()

    def on_mouse_released(self, sx, sy, ex, ey):
        rsx, rsy = self.map_to_raw(sx, sy)
        rex, rey = self.map_to_raw(ex, ey)
        rx, ry = min(rsx, rex), min(rsy, rey)
        rw, rh = abs(rsx - rex), abs(rsy - rey)
        
        if rw > 5 and rh > 5:
            tab = self.get_active_tab()
            tab.sld_roi_x.setValue(rx)
            tab.sld_roi_y.setValue(ry)
            tab.sld_roi_w.setValue(rw)
            tab.sld_roi_h.setValue(rh)
            
        self.temp_drag_roi = None
        self.process_pipeline()

    # --- Config / Ground Actions ---
    def on_capture_ground(self, tab: ProfileWidget):
        if self.current_frame is None:
            QMessageBox.warning(self, "Warning", "No active camera stream!")
            return
        tab.profile.ground_frame = self.current_frame.copy()
        tab.profile.ground_file_path = "Memory"
        tab.update_ground_label()
        self.process_pipeline()

    def on_load_ground(self, tab: ProfileWidget):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Ground", "", "Numpy (*.npy)")
        if filename:
            try:
                tab.profile.ground_frame = np.load(filename).copy()
                tab.profile.ground_file_path = filename
                tab.update_ground_label()
                self.process_pipeline()
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def on_save_yaml(self, tab: ProfileWidget):
        name = tab.profile.name
        filename, _ = QFileDialog.getSaveFileName(self, f"Save {name.upper()} Config", f"{name}_camera.yaml", "YAML (*.yaml)")
        if filename:
            if not filename.endswith('.yaml'): filename += '.yaml'
            p = tab.profile
            data = {
                f'{name}_camera': {
                    'enabled': p.enabled,
                    'depth_topic': p.depth_topic,
                    'camera_info_topic': p.info_topic,
                    'output_scan_topic': p.output_topic,
                    'target_frame': p.target_frame,
                    'width': p.raw_width,
                    'height': p.raw_height,
                    'pitch_deg': p.pitch_deg,
                    'min_range': p.min_range,
                    'max_range': p.max_range,
                    'roi_top': p.roi_y,
                    'roi_bottom': p.roi_y + p.roi_h,
                    'roi_left': p.roi_x,
                    'roi_right': p.roi_x + p.roi_w,
                    'threshold': p.threshold,
                    'min_area': p.min_area,
                    'median_filter': p.median_filter,
                    'morphology_size': p.morph_size,
                    'ground_file_path': p.ground_file_path
                }
            }
            try:
                with open(filename, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False)
                QMessageBox.information(self, "Saved", f"Config saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    def on_load_yaml(self, tab: ProfileWidget):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Config", "", "YAML (*.yaml)")
        if filename:
            try:
                with open(filename, 'r') as f:
                    d = yaml.safe_load(f)
                name = tab.profile.name
                if f'{name}_camera' in d:
                    cfg = d[f'{name}_camera']
                    p = tab.profile
                    p.enabled = cfg.get('enabled', p.enabled)
                    p.depth_topic = cfg.get('depth_topic', p.depth_topic)
                    p.info_topic = cfg.get('camera_info_topic', p.info_topic)
                    p.output_topic = cfg.get('output_scan_topic', p.output_topic)
                    p.target_frame = cfg.get('target_frame', p.target_frame)
                    p.pitch_deg = cfg.get('pitch_deg', p.pitch_deg)
                    p.min_range = cfg.get('min_range', p.min_range)
                    p.max_range = cfg.get('max_range', p.max_range)
                    
                    p.roi_x = cfg.get('roi_left', p.roi_x)
                    p.roi_y = cfg.get('roi_top', p.roi_y)
                    p.roi_w = cfg.get('roi_right', p.roi_x + p.roi_w) - p.roi_x
                    p.roi_h = cfg.get('roi_bottom', p.roi_y + p.roi_h) - p.roi_y
                    
                    p.threshold = cfg.get('threshold', p.threshold)
                    p.min_area = cfg.get('min_area', p.min_area)
                    p.median_filter = cfg.get('median_filter', p.median_filter)
                    p.morph_size = cfg.get('morphology_size', p.morph_size)
                    
                    g_path = cfg.get('ground_file_path', 'None')
                    if g_path != "None" and g_path != "Memory":
                        if os.path.exists(g_path):
                            p.ground_frame = np.load(g_path).copy()
                            p.ground_file_path = g_path
                        else:
                            # Try relative
                            rel_path = os.path.join(os.path.dirname(filename), os.path.basename(g_path))
                            if os.path.exists(rel_path):
                                p.ground_frame = np.load(rel_path).copy()
                                p.ground_file_path = rel_path
                    
                    tab.update_ui_from_profile()
                    
                    # If this is the active tab, resubscribe if topics changed
                    if self.active_profile_key == name:
                        self.switch_to_profile(name)
                        
            except Exception as e:
                QMessageBox.critical(self, "Error", str(e))

    # --- Core Pipeline ---
    def process_pipeline(self):
        if self.current_frame is None: return
        p = self.profiles[self.active_profile_key]
        
        rx, ry, rw, rh = int(p.roi_x), int(p.roi_y), int(p.roi_w), int(p.roi_h)
        rx = max(0, min(rx, p.raw_width - 1))
        ry = max(0, min(ry, p.raw_height - 1))
        rw = max(1, min(rw, p.raw_width - rx))
        rh = max(1, min(rh, p.raw_height - ry))
        
        current_roi = self.current_frame[ry:ry+rh, rx:rx+rw]
        
        depth_8u = np.clip(self.current_frame, 0, 4000) / 4000.0 * 255.0
        depth_color = cv2.applyColorMap(depth_8u.astype(np.uint8), cv2.COLORMAP_JET)
        depth_color[self.current_frame == 0] = [0, 0, 0]
        
        diff_disp = np.zeros((rh, rw), dtype=np.uint8)
        mask_disp = np.zeros((rh, rw), dtype=np.uint8)
        overlay_color = depth_color.copy()
        
        if p.ground_frame is not None and p.ground_frame.shape == self.current_frame.shape:
            ground_roi = p.ground_frame[ry:ry+rh, rx:rx+rw]
            valid_mask = (current_roi > 0) & (ground_roi > 0)
            diff = ground_roi.astype(np.int32) - current_roi.astype(np.int32)
            
            diff_vis = np.clip(diff, 0, 1000) / 1000.0 * 255.0
            diff_disp = diff_vis.astype(np.uint8)
            diff_disp[~valid_mask] = 0
            
            obstacle_mask = np.zeros_like(diff, dtype=np.uint8)
            obstacle_mask[valid_mask & (diff > p.threshold)] = 255
            
            if p.median_filter > 1:
                obstacle_mask = cv2.medianBlur(obstacle_mask, p.median_filter)
            if p.morph_size > 0:
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (p.morph_size, p.morph_size))
                obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
                obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
                
            mask_disp = obstacle_mask.copy()
            contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                if cv2.contourArea(cnt) >= p.min_area:
                    bx, by, bw, bh = cv2.boundingRect(cnt)
                    ox, oy = bx + rx, by + ry
                    cv2.rectangle(overlay_color, (ox, oy), (ox+bw, oy+bh), (0, 255, 255), 2)
            
            # Generate LaserScan
            if p.enabled:
                self.generate_scan(obstacle_mask, rx, ry, current_roi, p)
                
        cv2.rectangle(depth_color, (rx, ry), (rx+rw, ry+rh), (0, 255, 0), 2)
        if self.temp_drag_roi:
            tsx, tsy, tcx, tcy = self.temp_drag_roi
            tx, ty = min(tsx, tcx), min(tsy, tcy)
            tw, th = abs(tsx - tcx), abs(tsy - tcy)
            cv2.rectangle(depth_color, (tx, ty), (tx+tw, ty+th), (255, 255, 255), 1)
            
        self.display_main_depth(depth_color)
        self.display_bottom_images(diff_disp, mask_disp, overlay_color)

    def generate_scan(self, mask, rx, ry, current_roi, p: CameraProfile):
        if self.camera_intrinsics:
            fx, fy = self.camera_intrinsics['fx'], self.camera_intrinsics['fy']
            cx, cy = self.camera_intrinsics['cx'], self.camera_intrinsics['cy']
            frame_id = self.camera_intrinsics['frame_id']
        else:
            fx = fy = 570.0
            cx = p.raw_width / 2.0
            cy = p.raw_height / 2.0
            frame_id = 'camera_depth_optical_frame'
            
        fov = 2.0 * np.arctan(p.raw_width / (2.0 * fx))
        scan = LaserScan()
        scan.header.stamp = self.ros_node.get_clock().now().to_msg()
        scan.header.frame_id = p.target_frame
        scan.angle_min = -fov / 2.0
        scan.angle_max = fov / 2.0
        scan.angle_increment = fov / p.raw_width
        scan.range_min = p.min_range
        scan.range_max = p.max_range
        
        ranges = np.full(p.raw_width, np.inf, dtype=np.float32)
        
        y_idx, x_idx = np.where(mask == 255)
        if len(y_idx) > 0:
            u, v = rx + x_idx, ry + y_idx
            depths_m = current_roi[y_idx, x_idx] / 1000.0
            valid = depths_m > 0
            u, v, depths_m = u[valid], v[valid], depths_m[valid]
            
            if len(depths_m) > 0:
                x_3d = (u - cx) * depths_m / fx
                y_3d = (v - cy) * depths_m / fy
                z_3d = depths_m
                
                # Manual pitch since TF might be complex in debugger
                alpha_rad = -p.pitch_deg * np.pi / 180.0
                cos_a, sin_a = np.cos(alpha_rad), np.sin(alpha_rad)
                
                x_t = x_3d
                y_t = y_3d * cos_a - z_3d * sin_a
                z_t = y_3d * sin_a + z_3d * cos_a
                
                thetas = np.arctan2(-x_t, z_t)
                planar = np.sqrt(x_t**2 + z_t**2)
                
                # Filter by range
                valid_r = (planar >= p.min_range) & (planar <= p.max_range)
                thetas = thetas[valid_r]
                planar = planar[valid_r]
                
                bin_idx = ((thetas - scan.angle_min) / scan.angle_increment).astype(np.int32)
                in_bounds = (bin_idx >= 0) & (bin_idx < p.raw_width)
                bin_idx = bin_idx[in_bounds]
                planar = planar[in_bounds]
                
                if len(planar) > 0:
                    np.minimum.at(ranges, bin_idx, planar)
                    
        scan.ranges = [float(r) for r in ranges]
        self.ros_node.publish_scan(scan)

    def display_main_depth(self, img):
        target_w = int(img.shape[1] * self.zoom)
        target_h = int(img.shape[0] * self.zoom)
        resized = cv2.resize(img, (target_w, target_h))
        self.lbl_depth_viewer.setPixmap(self.numpy_to_pixmap(resized))
        self.lbl_depth_viewer.setFixedSize(target_w, target_h)

    def display_bottom_images(self, diff, mask, overlay):
        tw = 320
        th = int(overlay.shape[0] * (tw / max(1, overlay.shape[1])))
        
        diff_col = cv2.applyColorMap(cv2.resize(diff, (tw, th)), cv2.COLORMAP_BONE)
        mask_rgb = cv2.cvtColor(cv2.resize(mask, (tw, th)), cv2.COLOR_GRAY2BGR)
        over_res = cv2.resize(overlay, (tw, th))
        
        self.lbl_diff_viewer.setPixmap(self.numpy_to_pixmap(diff_col))
        self.lbl_mask_viewer.setPixmap(self.numpy_to_pixmap(mask_rgb))
        self.lbl_overlay_viewer.setPixmap(self.numpy_to_pixmap(over_res))
        
        for lbl in (self.lbl_diff_viewer, self.lbl_mask_viewer, self.lbl_overlay_viewer):
            lbl.setFixedSize(tw, th)

    def numpy_to_pixmap(self, arr):
        if arr is None or arr.size == 0: return QPixmap()
        if len(arr.shape) == 3:
            h, w, c = arr.shape
            rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            return QPixmap.fromImage(QImage(rgb.data, w, h, w * c, QImage.Format_RGB888)).copy()
        else:
            h, w = arr.shape
            return QPixmap.fromImage(QImage(arr.data, w, h, w, QImage.Format_Grayscale8)).copy()

    def closeEvent(self, event):
        super().closeEvent(event)

def main(args=None):
    rclpy.init(args=args)
    emitter = ImageSignalEmitter()
    node = DepthSubscriberNode(emitter)
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    
    app = QApplication(sys.argv)
    gui = DebuggerGUI(node)
    emitter.image_received.connect(gui.handle_new_frame)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
