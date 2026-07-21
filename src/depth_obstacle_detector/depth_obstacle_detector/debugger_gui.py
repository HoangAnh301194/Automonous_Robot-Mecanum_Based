#!/usr/bin/env python3

import sys
import os
import time
import threading
import numpy as np
import cv2

from PyQt5.QtCore import Qt, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton,
    QSlider, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QFileDialog, QStatusBar, QMessageBox, QComboBox, QScrollArea
)
from PyQt5.QtGui import QImage, QPixmap, QFont

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


class ImageSignalEmitter(QObject):
    """
    Thread-safe signal emitter to pass image data and camera info parameters
    from the ROS 2 background thread to the PyQt5 GUI main thread.
    """
    image_received = pyqtSignal(np.ndarray, object)


class DepthSubscriberNode(Node):
    """
    ROS 2 Node that subscribes to the depth camera stream and camera info topics,
    and publishes the filtered obstacle LaserScan.
    """
    def __init__(self, signal_emitter):
        super().__init__('depth_obstacle_debugger_node')
        self.signal_emitter = signal_emitter
        self.camera_info = None
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to camera info
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.info_callback,
            10
        )
        
        # Subscribe to depth raw topic (typically 16-bit unsigned in millimeters)
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_callback,
            10
        )
        
        # LaserScan publisher for detected obstacles
        self.laser_pub = self.create_publisher(LaserScan, '/scan_obstacles', 10)
        
        self.bridge = CvBridge()
        self.get_logger().info("Depth Obstacle Debugger ROS 2 Node initialized.")
        self.get_logger().info("Subscribed to /camera/depth/image_raw & /camera/depth/camera_info")
        self.get_logger().info("Publishing LaserScan on /scan_obstacles")

    def info_callback(self, msg):
        self.camera_info = msg

    def listener_callback(self, msg):
        try:
            # Convert depth image (passthrough retains 16-bit depth values)
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
        self.laser_pub.publish(scan_msg)


class ClickableLabel(QLabel):
    """
    Custom QLabel supporting mouse tracking, click-and-drag for ROI selection,
    and hover events for the Pixel Inspector.
    """
    mouse_moved = pyqtSignal(int, int)
    mouse_pressed = pyqtSignal(int, int)
    mouse_dragged = pyqtSignal(int, int, int, int)  # start_x, start_y, current_x, current_y
    mouse_released = pyqtSignal(int, int, int, int) # start_x, start_y, end_x, end_y

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


class DebuggerGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Depth Obstacle Debugger GUI")
        self.resize(1400, 850)

        # Core Data State
        self.current_frame = None
        self.ground_frame = None
        self.camera_intrinsics = None
        self.ground_file_path = "None"
        self.target_frame = "base_link"
        
        # Last known resolution
        self.raw_width = 0
        self.raw_height = 0
        
        # GUI Settings
        self.zoom = 1.0
        
        # Statistics
        self.fps = 0.0
        self.last_frame_time = time.time()
        self.processing_time_ms = 0.0
        self.obstacle_count = 0
        self.nearest_obstacle_dist = 0.0

        # Click-and-drag temp ROI state
        self.temp_drag_roi = None  # (start_x, start_y, current_x, current_y)

        # Setup GUI layout
        self.init_ui()
        self.apply_stylesheet()

    def init_ui(self):
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main Layout (Vertical)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Upper row Layout (Horizontal): Main Viewer + Controls
        upper_layout = QHBoxLayout()
        upper_layout.setSpacing(15)

        # 1. Left Section: Main Depth Viewer
        viewer_group = QGroupBox("Depth Viewer (Color Map)")
        viewer_layout = QVBoxLayout(viewer_group)
        viewer_layout.setContentsMargins(5, 5, 5, 5)
        
        # Scroll area for zooming
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setAlignment(Qt.AlignCenter)
        
        self.lbl_depth_viewer = ClickableLabel()
        self.lbl_depth_viewer.setAlignment(Qt.AlignCenter)
        self.lbl_depth_viewer.setText("Waiting for Camera Stream...")
        self.lbl_depth_viewer.setFont(QFont("Arial", 14, QFont.Bold))
        self.lbl_depth_viewer.setStyleSheet("color: #888888; background-color: #1e1e24;")
        self.scroll_area.setWidget(self.lbl_depth_viewer)
        viewer_layout.addWidget(self.scroll_area)
        
        # Depth zoom controls
        zoom_layout = QHBoxLayout()
        lbl_zoom = QLabel("Zoom:")
        self.cmb_zoom = QComboBox()
        self.cmb_zoom.addItems(["50%", "75%", "100%", "125%", "150%", "200%"])
        self.cmb_zoom.setCurrentText("100%")
        self.cmb_zoom.currentIndexChanged.connect(self.on_zoom_changed)
        zoom_layout.addWidget(lbl_zoom)
        zoom_layout.addWidget(self.cmb_zoom)
        zoom_layout.addStretch()
        viewer_layout.addLayout(zoom_layout)

        upper_layout.addWidget(viewer_group, stretch=3)

        # 2. Right Section: Scrollable Controls Panel to prevent cutoff
        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.controls_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.controls_scroll.setMinimumWidth(320)
        self.controls_scroll.setMaximumWidth(420)
        self.controls_scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")

        controls_panel = QWidget()
        controls_layout = QVBoxLayout(controls_panel)
        controls_layout.setContentsMargins(0, 0, 5, 0)
        controls_layout.setSpacing(10)

        # Group 2.1: Pixel Inspector (With 3D coordinates!)
        grp_inspector = QGroupBox("Pixel Inspector (3D)")
        inspector_grid = QGridLayout(grp_inspector)
        inspector_grid.setSpacing(8)
        
        inspector_grid.addWidget(QLabel("Cursor X, Y:"), 0, 0)
        self.lbl_inspect_xy = QLabel("-")
        inspector_grid.addWidget(self.lbl_inspect_xy, 0, 1)

        inspector_grid.addWidget(QLabel("Depth (mm):"), 1, 0)
        self.lbl_inspect_depth_mm = QLabel("-")
        inspector_grid.addWidget(self.lbl_inspect_depth_mm, 1, 1)

        inspector_grid.addWidget(QLabel("Point X (m):"), 2, 0)
        self.lbl_inspect_3dx = QLabel("-")
        inspector_grid.addWidget(self.lbl_inspect_3dx, 2, 1)

        inspector_grid.addWidget(QLabel("Point Y (m):"), 3, 0)
        self.lbl_inspect_3dy = QLabel("-")
        inspector_grid.addWidget(self.lbl_inspect_3dy, 3, 1)

        inspector_grid.addWidget(QLabel("Point Z (m):"), 4, 0)
        self.lbl_inspect_3dz = QLabel("-")
        inspector_grid.addWidget(self.lbl_inspect_3dz, 4, 1)

        controls_layout.addWidget(grp_inspector)

        # Group 2.2: ROI Config
        grp_roi = QGroupBox("Region of Interest (ROI)")
        roi_grid = QGridLayout(grp_roi)
        roi_grid.setSpacing(6)

        # X
        roi_grid.addWidget(QLabel("ROI X:"), 0, 0)
        self.sld_roi_x = QSlider(Qt.Horizontal)
        self.sld_roi_x.setRange(0, 100)
        self.sld_roi_x.valueChanged.connect(self.on_param_changed)
        roi_grid.addWidget(self.sld_roi_x, 0, 1)
        self.lbl_roi_x_val = QLabel("0")
        roi_grid.addWidget(self.lbl_roi_x_val, 0, 2)

        # Y
        roi_grid.addWidget(QLabel("ROI Y:"), 1, 0)
        self.sld_roi_y = QSlider(Qt.Horizontal)
        self.sld_roi_y.setRange(0, 100)
        self.sld_roi_y.valueChanged.connect(self.on_param_changed)
        roi_grid.addWidget(self.sld_roi_y, 1, 1)
        self.lbl_roi_y_val = QLabel("0")
        roi_grid.addWidget(self.lbl_roi_y_val, 1, 2)

        # Width
        roi_grid.addWidget(QLabel("ROI Width:"), 2, 0)
        self.sld_roi_w = QSlider(Qt.Horizontal)
        self.sld_roi_w.setRange(1, 100)
        self.sld_roi_w.setValue(50)
        self.sld_roi_w.valueChanged.connect(self.on_param_changed)
        roi_grid.addWidget(self.sld_roi_w, 2, 1)
        self.lbl_roi_w_val = QLabel("50")
        roi_grid.addWidget(self.lbl_roi_w_val, 2, 2)

        # Height
        roi_grid.addWidget(QLabel("ROI Height:"), 3, 0)
        self.sld_roi_h = QSlider(Qt.Horizontal)
        self.sld_roi_h.setRange(1, 100)
        self.sld_roi_h.setValue(50)
        self.sld_roi_h.valueChanged.connect(self.on_param_changed)
        roi_grid.addWidget(self.sld_roi_h, 3, 1)
        self.lbl_roi_h_val = QLabel("50")
        roi_grid.addWidget(self.lbl_roi_h_val, 3, 2)

        controls_layout.addWidget(grp_roi)

        # Group 2.3: Algorithmic Parameters
        grp_params = QGroupBox("Algorithmic Parameters")
        params_grid = QGridLayout(grp_params)
        params_grid.setSpacing(6)

        # Threshold
        params_grid.addWidget(QLabel("Threshold (mm):"), 0, 0)
        self.sld_threshold = QSlider(Qt.Horizontal)
        self.sld_threshold.setRange(5, 500)
        self.sld_threshold.setValue(80)
        self.sld_threshold.valueChanged.connect(self.on_param_changed)
        params_grid.addWidget(self.sld_threshold, 0, 1)
        self.lbl_threshold_val = QLabel("80")
        params_grid.addWidget(self.lbl_threshold_val, 0, 2)

        # Min Area
        params_grid.addWidget(QLabel("Min Area (px):"), 1, 0)
        self.sld_min_area = QSlider(Qt.Horizontal)
        self.sld_min_area.setRange(10, 5000)
        self.sld_min_area.setValue(150)
        self.sld_min_area.valueChanged.connect(self.on_param_changed)
        params_grid.addWidget(self.sld_min_area, 1, 1)
        self.lbl_min_area_val = QLabel("150")
        params_grid.addWidget(self.lbl_min_area_val, 1, 2)

        # Median Filter Size
        params_grid.addWidget(QLabel("Median Filter:"), 2, 0)
        self.sld_median = QSlider(Qt.Horizontal)
        self.sld_median.setRange(1, 15)
        self.sld_median.setValue(3)
        self.sld_median.setSingleStep(2)
        self.sld_median.valueChanged.connect(self.on_param_changed)
        params_grid.addWidget(self.sld_median, 2, 1)
        self.lbl_median_val = QLabel("3")
        params_grid.addWidget(self.lbl_median_val, 2, 2)

        # Morphological Operations Kernel Size
        params_grid.addWidget(QLabel("Morphology Size:"), 3, 0)
        self.sld_morph = QSlider(Qt.Horizontal)
        self.sld_morph.setRange(0, 15)
        self.sld_morph.setValue(3)
        self.sld_morph.valueChanged.connect(self.on_param_changed)
        params_grid.addWidget(self.sld_morph, 3, 1)
        self.lbl_morph_val = QLabel("3")
        params_grid.addWidget(self.lbl_morph_val, 3, 2)

        controls_layout.addWidget(grp_params)

        # Group 2.4: Configuration & Ground Reference Controls
        grp_ground = QGroupBox("Config & Ground Reference")
        ground_layout = QVBoxLayout(grp_ground)
        ground_layout.setSpacing(8)

        # Target Frame selection dropdown (editable to allow custom TF names)
        tf_layout = QHBoxLayout()
        tf_layout.addWidget(QLabel("Target Frame:"))
        self.cmb_target_frame = QComboBox()
        self.cmb_target_frame.setEditable(True)
        self.cmb_target_frame.addItems([
            'camera_depth_optical_frame',
            'camera_link',
            'base_link',
            'base_footprint'
        ])
        self.cmb_target_frame.setCurrentText("base_link")
        self.cmb_target_frame.currentTextChanged.connect(self.on_target_frame_changed)
        tf_layout.addWidget(self.cmb_target_frame)
        ground_layout.addLayout(tf_layout)

        # Manual Tilt Angle Slider
        tilt_layout = QHBoxLayout()
        tilt_layout.addWidget(QLabel("Camera Tilt (deg):"))
        self.sld_tilt_angle = QSlider(Qt.Horizontal)
        self.sld_tilt_angle.setRange(-45, 45)
        self.sld_tilt_angle.setValue(0)
        self.sld_tilt_angle.valueChanged.connect(self.on_param_changed)
        tilt_layout.addWidget(self.sld_tilt_angle)
        self.lbl_tilt_val = QLabel("0°")
        tilt_layout.addWidget(self.lbl_tilt_val)
        ground_layout.addLayout(tilt_layout)

        self.btn_capture_ground = QPushButton("Capture Ground")
        self.btn_capture_ground.clicked.connect(self.on_capture_ground)
        ground_layout.addWidget(self.btn_capture_ground)

        btn_files_layout = QHBoxLayout()
        self.btn_save_ground = QPushButton("Save Ground")
        self.btn_save_ground.clicked.connect(self.on_save_ground)
        btn_files_layout.addWidget(self.btn_save_ground)

        self.btn_load_ground = QPushButton("Load Ground")
        self.btn_load_ground.clicked.connect(self.on_load_ground)
        btn_files_layout.addWidget(self.btn_load_ground)
        ground_layout.addLayout(btn_files_layout)

        self.lbl_ground_status = QLabel("Ground Loaded: No")
        self.lbl_ground_status.setStyleSheet("color: #ff5555; font-weight: bold;")
        ground_layout.addWidget(self.lbl_ground_status)

        # Config YAML Buttons
        btn_config_layout = QHBoxLayout()
        self.btn_save_config = QPushButton("Save Config")
        self.btn_save_config.clicked.connect(self.on_save_config)
        btn_config_layout.addWidget(self.btn_save_config)

        self.btn_load_config = QPushButton("Load Config")
        self.btn_load_config.clicked.connect(self.on_load_config)
        btn_config_layout.addWidget(self.btn_load_config)
        ground_layout.addLayout(btn_config_layout)

        controls_layout.addWidget(grp_ground)

        # Set panel inside the scrollable area
        self.controls_scroll.setWidget(controls_panel)
        upper_layout.addWidget(self.controls_scroll)

        main_layout.addLayout(upper_layout, stretch=3)

        # 3. Lower Row Layout (Horizontal): Difference, Binary Mask, Detection Overlay
        lower_layout = QHBoxLayout()
        lower_layout.setSpacing(10)

        # 3.1 Difference
        grp_diff = QGroupBox("Difference (Current - Ground)")
        layout_diff = QVBoxLayout(grp_diff)
        self.lbl_diff_viewer = QLabel()
        self.lbl_diff_viewer.setAlignment(Qt.AlignCenter)
        self.lbl_diff_viewer.setStyleSheet("background-color: #1a1a1f;")
        self.lbl_diff_viewer.setText("N/A")
        layout_diff.addWidget(self.lbl_diff_viewer)
        lower_layout.addWidget(grp_diff)

        # 3.2 Binary Mask
        grp_mask = QGroupBox("Binary Mask (Thresholded)")
        layout_mask = QVBoxLayout(grp_mask)
        self.lbl_mask_viewer = QLabel()
        self.lbl_mask_viewer.setAlignment(Qt.AlignCenter)
        self.lbl_mask_viewer.setStyleSheet("background-color: #1a1a1f;")
        self.lbl_mask_viewer.setText("N/A")
        layout_mask.addWidget(self.lbl_mask_viewer)
        lower_layout.addWidget(grp_mask)

        # 3.3 Detection Overlay
        grp_overlay = QGroupBox("Detection Overlay (Output)")
        layout_overlay = QVBoxLayout(grp_overlay)
        self.lbl_overlay_viewer = QLabel()
        self.lbl_overlay_viewer.setAlignment(Qt.AlignCenter)
        self.lbl_overlay_viewer.setStyleSheet("background-color: #1a1a1f;")
        self.lbl_overlay_viewer.setText("N/A")
        layout_overlay.addWidget(self.lbl_overlay_viewer)
        lower_layout.addWidget(grp_overlay)

        main_layout.addLayout(lower_layout, stretch=2)

        # 4. Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.update_status_bar()

        # Wire up mouse interactions
        self.lbl_depth_viewer.mouse_moved.connect(self.on_mouse_moved)
        self.lbl_depth_viewer.mouse_pressed.connect(self.on_mouse_pressed)
        self.lbl_depth_viewer.mouse_dragged.connect(self.on_mouse_dragged)
        self.lbl_depth_viewer.mouse_released.connect(self.on_mouse_released)

    def apply_stylesheet(self):
        """
        Applies a modern, premium Dark Theme stylesheet.
        """
        stylesheet = """
        QMainWindow {
            background-color: #121214;
        }
        QWidget {
            color: #e0e0e6;
            font-family: 'Segoe UI', Arial, sans-serif;
            font-size: 13px;
        }
        QGroupBox {
            border: 2px solid #2d2d35;
            border-radius: 8px;
            margin-top: 10px;
            font-weight: bold;
            color: #00adb5;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QLabel {
            color: #b0b0ba;
        }
        QPushButton {
            background-color: #00adb5;
            color: #ffffff;
            border: none;
            border-radius: 4px;
            padding: 6px 12px;
            font-weight: bold;
        }
        QPushButton:hover {
            background-color: #00cfd8;
        }
        QPushButton:pressed {
            background-color: #008a90;
        }
        QSlider::groove:horizontal {
            border: 1px solid #2c2c35;
            height: 6px;
            background: #1e1e24;
            border-radius: 3px;
        }
        QSlider::handle:horizontal {
            background: #00adb5;
            border: 1px solid #00adb5;
            width: 14px;
            height: 14px;
            margin: -4px 0;
            border-radius: 7px;
        }
        QSlider::handle:horizontal:hover {
            background: #00cfd8;
            border-color: #00cfd8;
        }
        QComboBox {
            background-color: #1e1e24;
            border: 1px solid #2d2d35;
            border-radius: 4px;
            padding: 4px 8px;
            color: #e0e0e6;
        }
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 15px;
            border-left-width: 1px;
            border-left-color: #2d2d35;
            border-left-style: solid;
        }
        QStatusBar {
            background-color: #1a1a1f;
            color: #88888e;
            font-size: 12px;
        }
        QScrollArea {
            border: 1px solid #2d2d35;
            border-radius: 6px;
            background-color: #1a1a1f;
        }
        """
        self.setStyleSheet(stylesheet)

    # ==========================================
    # Callbacks & Events
    # ==========================================

    @pyqtSlot(np.ndarray, object)
    def handle_new_frame(self, cv_image, info_dict):
        """
        Invoked on the GUI thread whenever a new frame arrives via the ROS 2 subscriber.
        """
        self.current_frame = cv_image
        self.camera_intrinsics = info_dict
        self.raw_height, self.raw_width = cv_image.shape

        # Reset loaded ground frame if resolution differs
        if self.ground_frame is not None and self.ground_frame.shape != cv_image.shape:
            self.ros_node.get_logger().warn(
                f"Resetting ground reference because shape {self.ground_frame.shape} "
                f"does not match new stream resolution {cv_image.shape}!"
            )
            self.ground_frame = None
            self.ground_file_path = "None"
            self.lbl_ground_status.setText("Ground Loaded: No (Res Mismatch)")
            self.lbl_ground_status.setStyleSheet("color: #ff5555; font-weight: bold;")

        # Initialize sliders boundaries on first frame
        self.initialize_sliders_boundaries()

        # Calculate FPS
        t_now = time.time()
        self.fps = 1.0 / max(1e-5, (t_now - self.last_frame_time))
        self.last_frame_time = t_now

        # Execute processing pipeline
        t_start = time.time()
        self.process_pipeline()
        self.processing_time_ms = (time.time() - t_start) * 1000.0

        # Update stats on status bar
        self.update_status_bar()

    def initialize_sliders_boundaries(self):
        """
        Dynamically adjusts the ranges of ROI sliders to fit the received image resolution.
        """
        if self.sld_roi_x.maximum() != self.raw_width - 1:
            # Block signals temporarily to prevent infinite loop of updates
            self.sld_roi_x.blockSignals(True)
            self.sld_roi_y.blockSignals(True)
            self.sld_roi_w.blockSignals(True)
            self.sld_roi_h.blockSignals(True)

            self.sld_roi_x.setRange(0, self.raw_width - 1)
            self.sld_roi_y.setRange(0, self.raw_height - 1)
            self.sld_roi_w.setRange(1, self.raw_width)
            self.sld_roi_h.setRange(1, self.raw_height)

            # Set defaults (middle 50% ROI)
            self.sld_roi_x.setValue(self.raw_width // 4)
            self.sld_roi_y.setValue(self.raw_height // 4)
            self.sld_roi_w.setValue(self.raw_width // 2)
            self.sld_roi_h.setValue(self.raw_height // 2)

            self.sld_roi_x.blockSignals(False)
            self.sld_roi_y.blockSignals(False)
            self.sld_roi_w.blockSignals(False)
            self.sld_roi_h.blockSignals(False)

            self.update_slider_labels()

    def update_slider_labels(self):
        self.lbl_roi_x_val.setText(str(self.sld_roi_x.value()))
        self.lbl_roi_y_val.setText(str(self.sld_roi_y.value()))
        self.lbl_roi_w_val.setText(str(self.sld_roi_w.value()))
        self.lbl_roi_h_val.setText(str(self.sld_roi_h.value()))
        self.lbl_threshold_val.setText(str(self.sld_threshold.value()))
        self.lbl_min_area_val.setText(str(self.sld_min_area.value()))
        self.lbl_median_val.setText(str(self.sld_median.value()))
        self.lbl_morph_val.setText(str(self.sld_morph.value()))
        self.lbl_tilt_val.setText(f"{self.sld_tilt_angle.value()}°")

    def on_param_changed(self):
        # Force odd median kernel size
        val = self.sld_median.value()
        if val > 1 and val % 2 == 0:
            self.sld_median.setValue(val + 1)
            
        self.update_slider_labels()
        if self.current_frame is not None:
            # Force reprocessing of last frame with new parameters
            self.process_pipeline()

    def on_zoom_changed(self):
        txt = self.cmb_zoom.currentText().replace("%", "")
        self.zoom = float(txt) / 100.0
        if self.current_frame is not None:
            self.process_pipeline()

    def on_target_frame_changed(self, text):
        self.target_frame = text.strip()
        if self.current_frame is not None:
            self.process_pipeline()

    def update_status_bar(self):
        stat_text = (
            f"FPS: {self.fps:.1f}  |  "
            f"Resolution: {self.raw_width}x{self.raw_height}  |  "
            f"Obstacles: {self.obstacle_count}  |  "
            f"Nearest: {self.nearest_obstacle_dist:.2f}m  |  "
            f"Processing: {self.processing_time_ms:.1f} ms  |  "
            f"Ground Ref: {self.ground_file_path}"
        )
        self.status_bar.showMessage(stat_text)

    # ==========================================
    # Ground Reference & Config Actions
    # ==========================================

    def on_capture_ground(self):
        if self.current_frame is None:
            QMessageBox.warning(self, "Warning", "No active camera stream to capture ground reference!")
            return
        self.ground_frame = self.current_frame.copy()
        self.ground_file_path = "Memory (Unsaved)"
        self.lbl_ground_status.setText("Ground Loaded: Yes (Memory)")
        self.lbl_ground_status.setStyleSheet("color: #00ff66; font-weight: bold;")
        self.update_status_bar()
        self.process_pipeline()

    def on_save_ground(self):
        if self.ground_frame is None:
            QMessageBox.warning(self, "Warning", "No ground reference captured to save!")
            return
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Ground Reference", "ground_reference.npy", "Numpy Files (*.npy);;All Files (*)"
        )
        if filename:
            # Auto-append .npy if no extension is present in name
            if not filename.endswith('.npy') and '.' not in os.path.basename(filename):
                filename += '.npy'
            try:
                np.save(filename, self.ground_frame)
                self.ground_file_path = os.path.abspath(filename)
                self.lbl_ground_status.setText(f"Ground Loaded: Yes ({os.path.basename(filename)})")
                self.lbl_ground_status.setStyleSheet("color: #00ff66; font-weight: bold;")
                self.update_status_bar()
                self.process_pipeline()
                QMessageBox.information(self, "Success", f"Ground reference saved to {self.ground_file_path} successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save ground reference: {str(e)}")

    def on_load_ground(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Ground Reference", "", "Numpy Files (*.npy);;All Files (*)"
        )
        if filename:
            try:
                self.ros_node.get_logger().info(f"Attempting to load ground reference from: {filename}")
                loaded = np.load(filename)
                
                # Check shape compatibility
                self.ros_node.get_logger().info(f"Loaded array shape: {loaded.shape}, dtype: {loaded.dtype}")
                
                if self.current_frame is not None and loaded.shape != self.current_frame.shape:
                    QMessageBox.critical(
                        self, "Error", 
                        f"Ground frame resolution {loaded.shape} does not match camera stream {self.current_frame.shape}!"
                    )
                    return
                
                # Copy array safely to prevent memory access issues
                self.ground_frame = loaded.copy()
                self.ground_file_path = os.path.abspath(filename)
                
                self.lbl_ground_status.setText(f"Ground Loaded: Yes ({os.path.basename(filename)})")
                self.lbl_ground_status.setStyleSheet("color: #00ff66; font-weight: bold;")
                self.update_status_bar()
                
                # Update processing pipeline immediately
                self.process_pipeline()
                QMessageBox.information(self, "Success", "Ground reference loaded successfully!")
            except Exception as e:
                self.ros_node.get_logger().error(f"Failed to load ground: {str(e)}")
                QMessageBox.critical(self, "Error", f"Failed to load ground reference: {str(e)}")

    def on_save_config(self):
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "obstacle_config.yaml", "YAML Files (*.yaml *.yml);;All Files (*)"
        )
        if filename:
            if not filename.endswith('.yaml') and not filename.endswith('.yml') and '.' not in os.path.basename(filename):
                filename += '.yaml'
            try:
                import yaml
                config_data = {
                    'roi_x': int(self.sld_roi_x.value()),
                    'roi_y': int(self.sld_roi_y.value()),
                    'roi_width': int(self.sld_roi_w.value()),
                    'roi_height': int(self.sld_roi_h.value()),
                    'threshold': int(self.sld_threshold.value()),
                    'min_area': int(self.sld_min_area.value()),
                    'median_filter': int(self.sld_median.value()),
                    'morphology_size': int(self.sld_morph.value()),
                    'ground_file_path': str(self.ground_file_path) if self.ground_file_path else "None",
                    'target_frame': str(self.target_frame),
                    'camera_tilt': int(self.sld_tilt_angle.value())
                }
                with open(filename, 'w') as f:
                    yaml.dump(config_data, f, default_flow_style=False)
                QMessageBox.information(self, "Success", f"Configuration saved successfully to {os.path.basename(filename)}!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save configuration: {str(e)}")

    def on_load_config(self):
        filename, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "YAML Files (*.yaml *.yml);;All Files (*)"
        )
        if filename:
            try:
                import yaml
                with open(filename, 'r') as f:
                    config_data = yaml.safe_load(f)
                
                # Update GUI parameters (Block signals to update without multiple refreshes)
                self.sld_roi_x.blockSignals(True)
                self.sld_roi_y.blockSignals(True)
                self.sld_roi_w.blockSignals(True)
                self.sld_roi_h.blockSignals(True)
                self.sld_threshold.blockSignals(True)
                self.sld_min_area.blockSignals(True)
                self.sld_median.blockSignals(True)
                self.sld_morph.blockSignals(True)
                self.sld_tilt_angle.blockSignals(True)

                self.sld_roi_x.setValue(config_data.get('roi_x', 0))
                self.sld_roi_y.setValue(config_data.get('roi_y', 0))
                self.sld_roi_w.setValue(config_data.get('roi_width', 100))
                self.sld_roi_h.setValue(config_data.get('roi_height', 100))
                self.sld_threshold.setValue(config_data.get('threshold', 80))
                self.sld_min_area.setValue(config_data.get('min_area', 150))
                self.sld_median.setValue(config_data.get('median_filter', 3))
                self.sld_morph.setValue(config_data.get('morphology_size', 3))
                self.sld_tilt_angle.setValue(config_data.get('camera_tilt', 0))

                self.sld_roi_x.blockSignals(False)
                self.sld_roi_y.blockSignals(False)
                self.sld_roi_w.blockSignals(False)
                self.sld_roi_h.blockSignals(False)
                self.sld_threshold.blockSignals(False)
                self.sld_min_area.blockSignals(False)
                self.sld_median.blockSignals(False)
                self.sld_morph.blockSignals(False)
                self.sld_tilt_angle.blockSignals(False)

                self.update_slider_labels()

                # Update target frame dropdown
                loaded_tf = config_data.get('target_frame', 'base_link')
                self.cmb_target_frame.blockSignals(True)
                self.cmb_target_frame.setCurrentText(loaded_tf)
                self.target_frame = loaded_tf
                self.cmb_target_frame.blockSignals(False)

                # Attempt to load ground reference if specified
                g_path = config_data.get('ground_file_path', 'None')
                if g_path and g_path != "None" and g_path != "Memory (Unsaved)":
                    if os.path.exists(g_path):
                        self.ros_node.get_logger().info(f"Loading ground from config path: {g_path}")
                        loaded = np.load(g_path)
                        self.ground_frame = loaded.copy()
                        self.ground_file_path = g_path
                        self.lbl_ground_status.setText(f"Ground Loaded: Yes ({os.path.basename(g_path)})")
                        self.lbl_ground_status.setStyleSheet("color: #00ff66; font-weight: bold;")
                    else:
                        # Try relative to the yaml file location
                        yaml_dir = os.path.dirname(filename)
                        rel_path = os.path.abspath(os.path.join(yaml_dir, os.path.basename(g_path)))
                        if os.path.exists(rel_path):
                            self.ros_node.get_logger().info(f"Loading ground from relative config path: {rel_path}")
                            loaded = np.load(rel_path)
                            self.ground_frame = loaded.copy()
                            self.ground_file_path = rel_path
                            self.lbl_ground_status.setText(f"Ground Loaded: Yes ({os.path.basename(rel_path)})")
                            self.lbl_ground_status.setStyleSheet("color: #00ff66; font-weight: bold;")
                        else:
                            self.ros_node.get_logger().warn(f"Ground reference file not found: {g_path}")
                            QMessageBox.warning(self, "Warning", f"Ground reference file not found at: {g_path}")
                
                self.update_status_bar()
                self.process_pipeline()
                QMessageBox.information(self, "Success", "Configuration loaded successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load configuration: {str(e)}")

    # ==========================================
    # Mouse Event Responders & 3D Projection
    # ==========================================

    def map_to_raw(self, lbl_x, lbl_y):
        """
        Maps GUI coordinate (scaled) back to raw depth image coordinates.
        """
        if self.zoom <= 0 or self.raw_width == 0 or self.raw_height == 0:
            return 0, 0
        raw_x = int(lbl_x / self.zoom)
        raw_y = int(lbl_y / self.zoom)
        raw_x = max(0, min(raw_x, self.raw_width - 1))
        raw_y = max(0, min(raw_y, self.raw_height - 1))
        return raw_x, raw_y

    def on_mouse_moved(self, x, y):
        raw_x, raw_y = self.map_to_raw(x, y)
        self.lbl_inspect_xy.setText(f"{raw_x}, {raw_y}")
        
        if self.current_frame is not None:
            depth_val = self.current_frame[raw_y, raw_x]
            if depth_val > 0:
                self.lbl_inspect_depth_mm.setText(f"{depth_val} mm")
                
                # Retrieve camera intrinsics for 3D projection
                if self.camera_intrinsics is not None:
                    fx = self.camera_intrinsics['fx']
                    fy = self.camera_intrinsics['fy']
                    cx = self.camera_intrinsics['cx']
                    cy = self.camera_intrinsics['cy']
                else:
                    # Fallback default values (approximate)
                    fx = 570.0
                    fy = 570.0
                    cx = self.raw_width / 2.0
                    cy = self.raw_height / 2.0
                
                # Compute 3D Coordinates in Camera Optical Frame
                z_m = depth_val / 1000.0
                x_m = (raw_x - cx) * z_m / fx
                y_m = (raw_y - cy) * z_m / fy
                
                self.lbl_inspect_3dx.setText(f"{x_m:.3f} m")
                self.lbl_inspect_3dy.setText(f"{y_m:.3f} m")
                self.lbl_inspect_3dz.setText(f"{z_m:.3f} m")
            else:
                self.lbl_inspect_depth_mm.setText("0 (Invalid)")
                self.lbl_inspect_3dx.setText("N/A")
                self.lbl_inspect_3dy.setText("N/A")
                self.lbl_inspect_3dz.setText("N/A")

    def on_mouse_pressed(self, x, y):
        raw_x, raw_y = self.map_to_raw(x, y)
        self.temp_drag_roi = (raw_x, raw_y, raw_x, raw_y)

    def on_mouse_dragged(self, sx, sy, cx, cy):
        raw_sx, raw_sy = self.map_to_raw(sx, sy)
        raw_cx, raw_cy = self.map_to_raw(cx, cy)
        self.temp_drag_roi = (raw_sx, raw_sy, raw_cx, raw_cy)
        # Update display dynamically while dragging
        self.process_pipeline()

    def on_mouse_released(self, sx, sy, ex, ey):
        raw_sx, raw_sy = self.map_to_raw(sx, sy)
        raw_ex, raw_ey = self.map_to_raw(ex, ey)
        
        # Calculate new ROI parameters
        rx = min(raw_sx, raw_ex)
        ry = min(raw_sy, raw_ey)
        rw = abs(raw_sx - raw_ex)
        rh = abs(raw_sy - raw_ey)

        # Minimum size requirement for mouse ROI selection
        if rw > 5 and rh > 5:
            # Temporarily block signals to update all sliders together
            self.sld_roi_x.blockSignals(True)
            self.sld_roi_y.blockSignals(True)
            self.sld_roi_w.blockSignals(True)
            self.sld_roi_h.blockSignals(True)

            self.sld_roi_x.setValue(rx)
            self.sld_roi_y.setValue(ry)
            self.sld_roi_w.setValue(rw)
            self.sld_roi_h.setValue(rh)

            self.sld_roi_x.blockSignals(False)
            self.sld_roi_y.blockSignals(False)
            self.sld_roi_w.blockSignals(False)
            self.sld_roi_h.blockSignals(False)

            self.update_slider_labels()
            
        self.temp_drag_roi = None
        self.process_pipeline()

    # ==========================================
    # Algorithmic Pipeline & LaserScan Generation
    # ==========================================

    def process_pipeline(self):
        if self.current_frame is None:
            return

        try:
            # 1. Fetch current control parameters
            rx = self.sld_roi_x.value()
            ry = self.sld_roi_y.value()
            rw = self.sld_roi_w.value()
            rh = self.sld_roi_h.value()
            
            # Clamp parameters to current resolution bounds
            rx = max(0, min(rx, self.raw_width - 1))
            ry = max(0, min(ry, self.raw_height - 1))
            rw = max(1, min(rw, self.raw_width - rx))
            rh = max(1, min(rh, self.raw_height - ry))

            thresh_val = self.sld_threshold.value()
            min_area = self.sld_min_area.value()
            k_median = self.sld_median.value()
            k_morph = self.sld_morph.value()

            # 2. Extract ROI from Current Frame
            current_roi = self.current_frame[ry:ry+rh, rx:rx+rw]

            # 3. Create Colorized Raw Depth Viewer base
            # 16-bit to 8-bit mapping (max distance 4.0 meters / 4000 mm)
            depth_8u = np.clip(self.current_frame, 0, 4000) / 4000.0 * 255.0
            depth_8u = depth_8u.astype(np.uint8)
            # Apply Jet Color Map
            depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
            # Force invalid pixels (0 depth value) to black color
            depth_color[self.current_frame == 0] = [0, 0, 0]

            # Initialize default lower view placeholders
            diff_disp = np.zeros((rh, rw), dtype=np.uint8)
            mask_disp = np.zeros((rh, rw), dtype=np.uint8)
            overlay_color = depth_color.copy()

            obstacles = []
            self.obstacle_count = 0
            self.nearest_obstacle_dist = 0.0

            if self.ground_frame is not None:
                # 4. Compare with Ground Reference inside ROI
                ground_roi = self.ground_frame[ry:ry+rh, rx:rx+rw]
                
                # Mask representing pixels that are non-zero in both current and ground frame
                valid_mask = (current_roi > 0) & (ground_roi > 0)
                
                # Subtract (Ground - Current). If current is smaller, object is closer.
                # Convert to int32 to prevent overflow wrapping in uint16 math
                diff = ground_roi.astype(np.int32) - current_roi.astype(np.int32)
                
                # Difference visualization (positive values up to 1000mm mapped to 0-255 grayscale)
                diff_vis = np.clip(diff, 0, 1000) / 1000.0 * 255.0
                diff_disp = diff_vis.astype(np.uint8)
                # Fill invalid pixels with black in visualization
                diff_disp[~valid_mask] = 0

                # 5. Threshold to create Binary Mask
                obstacle_mask = np.zeros_like(diff, dtype=np.uint8)
                obstacle_mask[valid_mask & (diff > thresh_val)] = 255

                # 6. Apply Morphology Filters (Noise Reduction)
                # A. Median filter (must be odd size)
                if k_median > 1:
                    obstacle_mask = cv2.medianBlur(obstacle_mask, k_median)
                
                # B. Opening and Closing Morphology operations
                if k_morph > 0:
                    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k_morph, k_morph))
                    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
                    obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

                mask_disp = obstacle_mask.copy()

                # 7. Contour/Connected Components Detection
                contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                min_dist_overall = float('inf')
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area >= min_area:
                        # Obstacle is qualified
                        bx, by, bw, bh = cv2.boundingRect(cnt)
                        
                        # Compute nearest distance to this obstacle
                        # Mask the depth frame within bounding box using the contour shape
                        mask_box = np.zeros((bh, bw), dtype=np.uint8)
                        # Adjust contour coordinates to box coordinates
                        cnt_adjusted = cnt - np.array([bx, by])
                        cv2.drawContours(mask_box, [cnt_adjusted], -1, 255, -1)
                        
                        # Extract depth values belonging to this obstacle
                        box_depth = current_roi[by:by+bh, bx:bx+bw]
                        obstacle_depths = box_depth[mask_box > 0]
                        
                        # Filter invalid (0) depth values
                        valid_depths = obstacle_depths[obstacle_depths > 0]
                        if len(valid_depths) > 0:
                            min_depth_mm = np.min(valid_depths)
                            dist_m = min_depth_mm / 1000.0
                        else:
                            dist_m = 0.0

                        if dist_m > 0 and dist_m < min_dist_overall:
                            min_dist_overall = dist_m

                        # Store obstacles relative to full frame coordinates
                        obstacles.append({
                            'x': bx + rx,
                            'y': by + ry,
                            'w': bw,
                            'h': bh,
                            'dist': dist_m
                        })

                self.obstacle_count = len(obstacles)
                if min_dist_overall != float('inf'):
                    self.nearest_obstacle_dist = min_dist_overall

                # 8. Render bounding boxes and distances on Overlay
                for obs in obstacles:
                    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                    d_val = obs['dist']
                    
                    # Check if this is the nearest obstacle
                    is_nearest = (d_val == self.nearest_obstacle_dist)
                    color = (0, 0, 255) if is_nearest else (0, 255, 255) # Red for nearest, Yellow for others
                    thickness = 3 if is_nearest else 2
                    
                    # Draw bounding box
                    cv2.rectangle(overlay_color, (ox, oy), (ox+ow, oy+oh), color, thickness)
                    
                    # Draw distance text
                    txt = f"{d_val:.2f}m"
                    cv2.putText(overlay_color, txt, (ox, max(oy - 5, 15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

            # 9. Draw current ROI rectangle on the Main Depth colorized stream
            cv2.rectangle(depth_color, (rx, ry), (rx+rw, ry+rh), (0, 255, 0), 2) # Green ROI box

            # If user is currently dragging a new ROI rectangle, draw it in dashed/white
            if self.temp_drag_roi is not None:
                tsx, tsy, tcx, tcy = self.temp_drag_roi
                tx = min(tsx, tcx)
                ty = min(tsy, tcy)
                tw = abs(tsx - tcx)
                th = abs(tsy - tcy)
                cv2.rectangle(depth_color, (tx, ty), (tx+tw, ty+th), (255, 255, 255), 1, cv2.LINE_4)

            # 10. Generate and Publish LaserScan
            self.generate_and_publish_laserscan(mask_disp, rx, ry, current_roi)

            # 11. Display Image conversions and resizing
            self.display_main_depth(depth_color)
            self.display_bottom_images(diff_disp, mask_disp, overlay_color)

        except Exception as e:
            self.ros_node.get_logger().error(f"Error in image processing pipeline: {str(e)}")

    def generate_and_publish_laserscan(self, obstacle_mask, rx, ry, current_roi):
        """
        Performs high-performance vectorized conversion from the 2D obstacle binary mask 
        into a 2D LaserScan message, publishing it for Nav2/RViz consumption.
        Optionally uses TF2 to project coordinates to a target frame (e.g. camera_link)
        or falls back to manual pitch angle rotation.
        """
        # Fetch or fallback camera intrinsic parameters
        if self.camera_intrinsics is not None:
            fx = self.camera_intrinsics['fx']
            fy = self.camera_intrinsics['fy']
            cx = self.camera_intrinsics['cx']
            cy = self.camera_intrinsics['cy']
            frame_id = self.camera_intrinsics['frame_id']
        else:
            fx = 570.0
            fy = 570.0
            cx = self.raw_width / 2.0
            cy = self.raw_height / 2.0
            frame_id = 'camera_depth_optical_frame'

        # Compute Horizontal Field of View (FOV) in Radians
        fov = 2.0 * np.arctan(self.raw_width / (2.0 * fx))

        # Check if we should use TF projection to target frame
        use_tf = (self.target_frame != frame_id)
        
        translation = np.array([0.0, 0.0, 0.0])
        R = np.eye(3)
        actual_frame_id = frame_id

        if use_tf:
            try:
                # Lookup transform from camera optical frame to target frame (e.g. camera_link)
                trans = self.ros_node.tf_buffer.lookup_transform(
                    self.target_frame,
                    frame_id,
                    rclpy.time.Time()
                )
                
                # Get Translation
                translation = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])
                
                # Get Rotation Quaternion
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w
                
                # Compute Rotation Matrix
                R = np.array([
                    [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                    [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
                ])
                actual_frame_id = self.target_frame
            except Exception as e:
                self.ros_node.get_logger().warn(
                    f"TF lookup failed to {self.target_frame}: {str(e)}. Falling back to manual tilt."
                )
                use_tf = False
                if self.sld_tilt_angle.value() != 0:
                    actual_frame_id = self.target_frame
                else:
                    actual_frame_id = frame_id

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        scan_msg.header.frame_id = actual_frame_id
        scan_msg.angle_min = -fov / 2.0
        scan_msg.angle_max = fov / 2.0
        scan_msg.angle_increment = fov / self.raw_width
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.033  # ~30 FPS
        scan_msg.range_min = 0.4
        scan_msg.range_max = 5.0

        # Default all scan points to infinity (no obstacle detected)
        ranges = [float('inf')] * self.raw_width

        # Process obstacle coordinates if Ground Reference is loaded and active
        if self.ground_frame is not None and obstacle_mask is not None:
            # Find row (y) and column (x) indices of all obstacle pixels
            y_idx, x_idx = np.where(obstacle_mask == 255)
            
            if len(y_idx) > 0:
                # Map coordinates back to the full image frame
                u_coords = rx + x_idx
                v_coords = ry + y_idx
                
                # Fetch depth values and convert to meters
                depths_m = current_roi[y_idx, x_idx] / 1000.0
                
                # Filter out invalid depth pixels
                valid = depths_m > 0
                u_coords = u_coords[valid]
                v_coords = v_coords[valid]
                depths_m = depths_m[valid]
                
                if len(depths_m) > 0:
                    # De-project 2D pixels to 3D space in Optical Frame
                    x_3d = (u_coords - cx) * depths_m / fx
                    y_3d = (v_coords - cy) * depths_m / fy
                    z_3d = depths_m
                    
                    if use_tf:
                        # Stack optical points: shape (N, 3)
                        pts_optical = np.vstack([x_3d, y_3d, z_3d]).T
                        # Rotate and translate to target frame
                        pts_target = pts_optical @ R.T + translation
                        
                        # Extract projected coords (X_t is forward, Y_t is left)
                        x_t = pts_target[:, 0]
                        y_t = pts_target[:, 1]
                        
                        # Calculate angles and 2D planar ranges in target frame
                        thetas = np.arctan2(y_t, x_t)
                        planar_ranges = np.sqrt(x_t**2 + y_t**2)
                    else:
                        # Apply manual camera tilt pitch rotation around X-axis
                        tilt_deg = self.sld_tilt_angle.value()
                        if tilt_deg != 0:
                            alpha_rad = -tilt_deg * np.pi / 180.0
                            cos_a = np.cos(alpha_rad)
                            sin_a = np.sin(alpha_rad)
                            
                            # Pitch rotation
                            x_t = x_3d
                            y_t = y_3d * cos_a - z_3d * sin_a
                            z_t = y_3d * sin_a + z_3d * cos_a
                            
                            # In horizontal alignment, Z_t is forward, X_t is horizontal left/right
                            # Map to ROS frame conventions: forward = z_t, left = -x_t
                            thetas = np.arctan2(-x_t, z_t)
                            planar_ranges = np.sqrt(x_t**2 + z_t**2)
                        else:
                            # Standard optical frame angle/range calculations
                            thetas = np.arctan2(x_3d, z_3d)
                            planar_ranges = np.sqrt(x_3d**2 + z_3d**2)
                    
                    # Determine range bin indexes
                    bin_idx = ((thetas - scan_msg.angle_min) / scan_msg.angle_increment).astype(np.int32)
                    
                    # Filter bins lying within image width bounds
                    in_bounds = (bin_idx >= 0) & (bin_idx < self.raw_width)
                    planar_ranges = planar_ranges[in_bounds]
                    bin_idx = bin_idx[in_bounds]
                    
                    if len(planar_ranges) > 0:
                        # Vectorized minimum search to get closest obstacle for each angular bin
                        ranges_arr = np.full(self.raw_width, np.inf, dtype=np.float32)
                        np.minimum.at(ranges_arr, bin_idx, planar_ranges)
                        
                        # Populate final message values
                        ranges = [float(val) for val in ranges_arr]

        scan_msg.ranges = ranges
        self.ros_node.publish_scan(scan_msg)

    # ==========================================
    # Canvas Renderers
    # ==========================================

    def display_main_depth(self, depth_color):
        """
        Scales and displays the main colorized depth image onto the ClickableLabel.
        """
        target_w = int(self.raw_width * self.zoom)
        target_h = int(self.raw_height * self.zoom)
        
        resized = cv2.resize(depth_color, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        pixmap = self.numpy_to_pixmap(resized)
        
        self.lbl_depth_viewer.setPixmap(pixmap)
        self.lbl_depth_viewer.setFixedSize(target_w, target_h)

    def display_bottom_images(self, diff, mask, overlay):
        """
        Renders the three smaller status monitors at the bottom of the GUI.
        """
        # We scale bottom views to a fixed width of 360px to maintain clean alignment
        target_w = 360
        ratio = target_w / max(1, self.raw_width)
        target_h = int(self.raw_height * ratio)

        # Convert difference (single channel) to display
        if len(diff.shape) == 2:
            resized_diff = cv2.resize(diff, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
            diff_color = cv2.applyColorMap(resized_diff, cv2.COLORMAP_BONE)
            pix_diff = self.numpy_to_pixmap(diff_color)
        else:
            pix_diff = QPixmap()

        # Convert mask (binary single channel) to display
        resized_mask = cv2.resize(mask, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
        mask_rgb = cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2BGR)
        pix_mask = self.numpy_to_pixmap(mask_rgb)

        # Convert final overlay to display
        resized_overlay = cv2.resize(overlay, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        pix_overlay = self.numpy_to_pixmap(resized_overlay)

        # Update QLabels
        self.lbl_diff_viewer.setPixmap(pix_diff)
        self.lbl_diff_viewer.setFixedSize(target_w, target_h)

        self.lbl_mask_viewer.setPixmap(pix_mask)
        self.lbl_mask_viewer.setFixedSize(target_w, target_h)

        self.lbl_overlay_viewer.setPixmap(pix_overlay)
        self.lbl_overlay_viewer.setFixedSize(target_w, target_h)

    def numpy_to_pixmap(self, arr):
        """
        Safe conversion utility from OpenCV numpy array to QPixmap.
        """
        if arr is None or arr.size == 0:
            return QPixmap()
            
        if len(arr.shape) == 3: # BGR Color format
            h, w, c = arr.shape
            rgb = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
            qimg = QImage(rgb.data, w, h, w * c, QImage.Format_RGB888)
            return QPixmap.fromImage(qimg).copy()
        else: # Grayscale format
            h, w = arr.shape
            qimg = QImage(arr.data, w, h, w, QImage.Format_Grayscale8)
            return QPixmap.fromImage(qimg).copy()

    def closeEvent(self, event):
        """
        Ensures clean shutdown of ROS 2 resources when the GUI window is closed.
        """
        self.ros_node.get_logger().info("Shutting down Debugger GUI...")
        super().closeEvent(event)


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Signal emitter for thread-safe cross-talk
    signal_emitter = ImageSignalEmitter()

    # Create the ROS 2 subscriber node
    ros_node = DepthSubscriberNode(signal_emitter)

    # Spin ROS 2 executor in a separate background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Start the standard PyQt5 App event loop
    app = QApplication(sys.argv)
    gui = DebuggerGUI(ros_node)
    
    # Connect the signal emitter to the GUI slot
    signal_emitter.image_received.connect(gui.handle_new_frame)
    
    # Show GUI
    gui.show()

    # Block on GUI thread exit
    sys.exit(app.exec_())

    # Shutdown ROS
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
