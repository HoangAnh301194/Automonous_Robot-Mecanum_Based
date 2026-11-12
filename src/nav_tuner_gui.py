#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 Navigation Realtime Parameter Tuner (Clean White Theme)
Cho phép điều chỉnh trực tiếp các tham số cốt lõi của AMCL, Nav2 Costmap,
Controller và SLAM Toolbox ngay khi robot đang chạy mà không cần khởi động lại.
"""

import os
import sys
import time
import threading
import yaml

# Đảm bảo không bị xung đột Qt plugin xcb từ OpenCV pip
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QTabWidget, QVBoxLayout,
    QHBoxLayout, QGridLayout, QGroupBox, QLabel, QSlider, QPushButton,
    QCheckBox, QTextEdit, QStatusBar, QMessageBox, QScrollArea, QFrame,
    QFileDialog, QComboBox
)

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType


# ==============================================================================
# BỘ THIẾT KẾ GIAO DIỆN TRẮNG HIỆN ĐẠI (CLEAN WHITE / LIGHT THEME STYLESHEET)
# ==============================================================================
CLEAN_WHITE_QSS = """
QMainWindow {
    background-color: #F8F9FA;
}
QWidget {
    color: #1E293B;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 13px;
}
QTabWidget::pane {
    border: 1px solid #E2E8F0;
    background-color: #FFFFFF;
    border-radius: 8px;
    top: -1px;
}
QTabBar::tab {
    background-color: #F1F5F9;
    color: #64748B;
    padding: 10px 24px;
    margin-right: 4px;
    border-top-left-radius: 8px;
    border-top-right-radius: 8px;
    font-weight: 600;
}
QTabBar::tab:selected {
    background-color: #FFFFFF;
    color: #2563EB;
    border: 1px solid #E2E8F0;
    border-bottom: 2px solid #2563EB;
}
QTabBar::tab:hover:!selected {
    background-color: #E2E8F0;
    color: #1E293B;
}
QGroupBox {
    background-color: #FFFFFF;
    border: 1px solid #E2E8F0;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 10px;
    font-weight: bold;
    color: #0F172A;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #2563EB;
    font-size: 13px;
    font-weight: 700;
}
QLabel {
    color: #334155;
}
QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background: #E2E8F0;
    border-radius: 3px;
}
QSlider::sub-page:horizontal {
    background: #3B82F6;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #2563EB;
    border: 2px solid #FFFFFF;
    width: 18px;
    height: 18px;
    margin: -6px 0;
    border-radius: 9px;
}
QSlider::handle:horizontal:hover {
    background: #1D4ED8;
}
QPushButton {
    background-color: #FFFFFF;
    color: #334155;
    border: 1px solid #CBD5E1;
    border-radius: 6px;
    padding: 7px 16px;
    font-weight: 600;
}
QPushButton:hover {
    background-color: #F8F9FA;
    border-color: #94A3B8;
}
QPushButton:pressed {
    background-color: #E2E8F0;
}
QPushButton#btn_primary {
    background-color: #2563EB;
    color: #FFFFFF;
    border: none;
}
QPushButton#btn_primary:hover {
    background-color: #1D4ED8;
}
QPushButton#btn_success {
    background-color: #10B981;
    color: #FFFFFF;
    border: none;
}
QPushButton#btn_success:hover {
    background-color: #059669;
}
QTextEdit {
    background-color: #F8FAFC;
    border: 1px solid #E2E8F0;
    border-radius: 6px;
    color: #0F172A;
    font-family: 'Consolas', 'Courier New', monospace;
    font-size: 12px;
}
QCheckBox {
    color: #334155;
    font-weight: 600;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border-radius: 4px;
    border: 1px solid #CBD5E1;
}
QCheckBox::indicator:checked {
    background-color: #2563EB;
    border-color: #2563EB;
}
QScrollArea {
    border: none;
    background-color: transparent;
}
"""


class ParamCommunicator(QObject):
    log_signal = pyqtSignal(str, str)  # msg, level
    node_status_signal = pyqtSignal(dict)  # {node_name: bool}
    sync_value_signal = pyqtSignal(str, str, object)  # node, param_name, value


class RosParamWorker(Node):
    def __init__(self, comm):
        super().__init__('nav_realtime_tuner_node')
        self.comm = comm
        self.param_clients = {}

    def get_client(self, node_name, service_type, service_suffix):
        srv_name = f'/{node_name}/{service_suffix}'.replace('//', '/')
        if srv_name not in self.param_clients:
            self.param_clients[srv_name] = self.create_client(service_type, srv_name)
        return self.param_clients[srv_name]

    def set_param_async(self, node_name, param_name, param_val, val_type):
        client = self.get_client(node_name, SetParameters, 'set_parameters')
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.2):
                self.comm.log_signal.emit(f"Node [{node_name}] chưa bật hoặc không khả dụng!", "ERROR")
                return

        param_msg = ParamMsg()
        param_msg.name = param_name
        pval = ParameterValue()

        if val_type == 'int':
            pval.type = ParameterType.PARAMETER_INTEGER
            pval.integer_value = int(param_val)
        elif val_type == 'double':
            pval.type = ParameterType.PARAMETER_DOUBLE
            pval.double_value = float(param_val)
        elif val_type == 'double_array':
            pval.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pval.double_array_value = [float(x) for x in param_val]
        elif val_type == 'bool':
            pval.type = ParameterType.PARAMETER_BOOL
            pval.bool_value = bool(param_val)
        else:
            pval.type = ParameterType.PARAMETER_STRING
            pval.string_value = str(param_val)

        param_msg.value = pval

        req = SetParameters.Request()
        req.parameters = [param_msg]

        t0 = time.perf_counter()
        future = client.call_async(req)

        def done_callback(fut):
            dt = (time.perf_counter() - t0) * 1000.0
            try:
                res = fut.result()
                if res and res.results and res.results[0].successful:
                    self.comm.log_signal.emit(
                        f"[{node_name}] {param_name} -> {param_val} ({dt:.1f}ms)", "SUCCESS"
                    )
                else:
                    reason = res.results[0].reason if res and res.results else "Unknown"
                    self.comm.log_signal.emit(
                        f"Thất bại [{node_name}] {param_name}: {reason}", "WARN"
                    )
            except Exception as e:
                self.comm.log_signal.emit(f"Lỗi gọi dịch vụ: {str(e)}", "ERROR")

        future.add_done_callback(done_callback)

    def query_param_async(self, node_name, param_name):
        if not rclpy.ok():
            return
        try:
            client = self.get_client(node_name, GetParameters, 'get_parameters')
            if not client.service_is_ready():
                if not client.wait_for_service(timeout_sec=0.15):
                    return

            req = GetParameters.Request()
            req.names = [param_name]
            future = client.call_async(req)

            def done_callback(fut):
                try:
                    res = fut.result()
                    if res and res.values:
                        val_msg = res.values[0]
                        val = None
                        if val_msg.type == ParameterType.PARAMETER_INTEGER:
                            val = val_msg.integer_value
                        elif val_msg.type == ParameterType.PARAMETER_DOUBLE:
                            val = val_msg.double_value
                        elif val_msg.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                            val = list(val_msg.double_array_value)
                        elif val_msg.type == ParameterType.PARAMETER_BOOL:
                            val = val_msg.bool_value
                        elif val_msg.type == ParameterType.PARAMETER_STRING:
                            val = val_msg.string_value
                        if val is not None:
                            self.comm.sync_value_signal.emit(node_name, param_name, val)
                except Exception:
                    pass

            future.add_done_callback(done_callback)
        except Exception:
            pass

    def check_nodes_liveness(self, target_nodes):
        if not rclpy.ok():
            return
        try:
            active = self.get_node_names()
            status = {}
            for target in target_nodes:
                status[target] = any(target in name for name in active)
            self.comm.node_status_signal.emit(status)
        except Exception:
            pass


class NavTunerGUI(QMainWindow):
    def __init__(self, ros_worker, comm):
        super().__init__()
        self.ros_worker = ros_worker
        self.comm = comm

        self.setWindowTitle("ROS 2 Nav2 & SLAM Realtime Tuner")
        self.resize(1060, 860)
        self.setStyleSheet(CLEAN_WHITE_QSS)

        # Paths to config files
        self.ws_dir = os.path.expanduser('~/ros2_ws')
        self.nav2_yaml_path = os.path.join(self.ws_dir, 'src/mo_hinh/config/nav2_params.yaml')
        self.slam_yaml_path = os.path.join(self.ws_dir, 'src/mo_hinh/config/mapper_params_online_async.yaml')

        # Dictionary of registered controls: key = (node, param_name) -> dict(...)
        self.param_controls = {}
        self.debounce_timers = {}

        # Array parameter groups: (node, param_name) -> [component_keys_in_order]
        self.array_param_groups = {
            ('velocity_smoother', 'max_velocity'): [
                ('velocity_smoother', 'max_velocity[0]'),
                ('velocity_smoother', 'max_velocity[1]'),
                ('velocity_smoother', 'max_velocity[2]'),
            ],
            ('velocity_smoother', 'min_velocity'): [
                ('velocity_smoother', 'min_velocity[0]'),
                ('velocity_smoother', 'min_velocity[1]'),
                ('velocity_smoother', 'min_velocity[2]'),
            ],
            ('velocity_smoother', 'max_accel'): [
                ('velocity_smoother', 'max_accel[0]'),
                ('velocity_smoother', 'max_accel[1]'),
                ('velocity_smoother', 'max_accel[2]'),
            ],
            ('velocity_smoother', 'max_decel'): [
                ('velocity_smoother', 'max_decel[0]'),
                ('velocity_smoother', 'max_decel[1]'),
                ('velocity_smoother', 'max_decel[2]'),
            ],
        }

        self.comm.log_signal.connect(self.append_log)
        self.comm.node_status_signal.connect(self.update_node_status)
        self.comm.sync_value_signal.connect(self.handle_synced_value)

        self.init_ui()

        # Load local YAML defaults into UI immediately so sliders match disk config even offline
        if os.path.exists(self.nav2_yaml_path):
            self.load_yaml_to_gui(self.nav2_yaml_path, silent=True, apply_to_robot=False)
        if os.path.exists(self.slam_yaml_path):
            self.load_yaml_to_gui(self.slam_yaml_path, silent=True, apply_to_robot=False)

        # Heartbeat timer to monitor running nodes
        self.timer_liveness = QTimer(self)
        self.timer_liveness.timeout.connect(self.poll_liveness)
        self.timer_liveness.start(2000)

        # Initial Sync from running robot nodes
        QTimer.singleShot(1000, self.on_sync_all)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(18, 16, 18, 16)
        main_layout.setSpacing(12)

        # ======================================================================
        # 1. HEADER BAR: Title, Live Status Badges, Action Buttons
        # ======================================================================
        header_card = QFrame()
        header_card.setStyleSheet("background-color: #FFFFFF; border: 1px solid #E2E8F0; border-radius: 8px;")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 10, 16, 10)

        title_layout = QVBoxLayout()
        lbl_app_title = QLabel("⚡ ROS 2 NAV TUNER")
        lbl_app_title.setStyleSheet("font-size: 17px; font-weight: 800; color: #1E40AF;")
        lbl_subtitle = QLabel("Chỉnh sửa tham số Realtime & YAML cho AMCL, Costmap, MPPI, Smoother, Safety & SLAM")
        lbl_subtitle.setStyleSheet("font-size: 11px; color: #64748B;")
        title_layout.addWidget(lbl_app_title)
        title_layout.addWidget(lbl_subtitle)
        header_layout.addLayout(title_layout)

        header_layout.addStretch()

        # Node Status Badges
        self.badge_amcl = self.create_badge("AMCL")
        self.badge_controller = self.create_badge("Controller")
        self.badge_costmap = self.create_badge("Costmap")
        self.badge_smoother = self.create_badge("Smoother")
        self.badge_slam = self.create_badge("SLAM")

        badges_layout = QHBoxLayout()
        badges_layout.setSpacing(6)
        badges_layout.addWidget(self.badge_amcl)
        badges_layout.addWidget(self.badge_controller)
        badges_layout.addWidget(self.badge_costmap)
        badges_layout.addWidget(self.badge_smoother)
        badges_layout.addWidget(self.badge_slam)
        header_layout.addLayout(badges_layout)

        header_layout.addSpacing(12)

        # Action Buttons: Preset, Load YAML, Sync, Save
        self.combo_preset = QComboBox()
        self.combo_preset.setStyleSheet("""
            QComboBox {
                background-color: #EFF6FF;
                color: #1E40AF;
                border: 1px solid #BFDBFE;
                border-radius: 6px;
                padding: 6px 12px;
                font-weight: 700;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #FFFFFF;
                border: 1px solid #CBD5E1;
                selection-background-color: #DBEAFE;
                selection-color: #1E40AF;
            }
        """)
        self.combo_preset.addItem("⚡ Chọn Preset mẫu...")
        self.combo_preset.addItem("🏢 Khoa A1 (Orin Nano Tối ưu)")
        self.combo_preset.addItem("🐢 Chế độ An toàn (Đông người)")
        self.combo_preset.addItem("🔄 Trả về Mặc định (Default)")
        self.combo_preset.currentIndexChanged.connect(self.on_preset_selected)
        header_layout.addWidget(self.combo_preset)

        btn_load = QPushButton("📂 Nạp file YAML")
        btn_load.setToolTip("Chọn một file YAML cấu hình để nạp vào các thanh trượt")
        btn_load.clicked.connect(self.on_load_yaml_file)
        header_layout.addWidget(btn_load)

        btn_sync = QPushButton("🔄 Nạp từ Robot")
        btn_sync.setToolTip("Đọc toàn bộ giá trị đang chạy trên robot về thanh trượt")
        btn_sync.clicked.connect(self.on_sync_all)
        header_layout.addWidget(btn_sync)

        btn_save = QPushButton("💾 Lưu vào YAML")
        btn_save.setObjectName("btn_success")
        btn_save.setToolTip("Ghi đè thông số đang chỉnh vào nav2_params.yaml & mapper_params_online_async.yaml")
        btn_save.clicked.connect(self.on_save_to_yaml)
        header_layout.addWidget(btn_save)

        main_layout.addWidget(header_card)

        # ======================================================================
        # 2. MAIN TABS
        # ======================================================================
        self.tabs = QTabWidget()
        self.tabs.addTab(self.create_amcl_tab(), "🎯 AMCL Định vị")
        self.tabs.addTab(self.create_costmap_tab(), "🗺️ Costmap Né cản")
        self.tabs.addTab(self.create_controller_tab(), "🚀 MPPI Controller")
        self.tabs.addTab(self.create_velocity_smoother_tab(), "⚡ Velocity Smoother")
        self.tabs.addTab(self.create_safety_planner_tab(), "🛡️ Planner & An toàn")
        self.tabs.addTab(self.create_slam_tab(), "🏗️ SLAM Toolbox")
        main_layout.addWidget(self.tabs, stretch=1)

        # ======================================================================
        # 3. BOTTOM PANEL: Live Toggle & Console Log
        # ======================================================================
        bottom_layout = QHBoxLayout()
        self.chk_auto_apply = QCheckBox("Tự động áp dụng Realtime khi kéo thanh trượt")
        self.chk_auto_apply.setChecked(True)
        bottom_layout.addWidget(self.chk_auto_apply)

        bottom_layout.addStretch()

        btn_clear_log = QPushButton("Xóa Log")
        btn_clear_log.setStyleSheet("font-size: 11px; padding: 4px 10px;")
        btn_clear_log.clicked.connect(lambda: self.txt_log.clear())
        bottom_layout.addWidget(btn_clear_log)
        main_layout.addLayout(bottom_layout)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFixedHeight(120)
        main_layout.addWidget(self.txt_log)

    def create_badge(self, name):
        lbl = QLabel(f"● {name}")
        lbl.setStyleSheet(
            "background-color: #FEE2E2; color: #DC2626; border-radius: 12px; "
            "padding: 4px 10px; font-weight: 700; font-size: 11px;"
        )
        return lbl

    def set_badge_state(self, badge, name, is_active):
        if is_active:
            badge.setText(f"● {name}")
            badge.setStyleSheet(
                "background-color: #DCFCE7; color: #16A34A; border-radius: 12px; "
                "padding: 4px 10px; font-weight: 700; font-size: 11px;"
            )
        else:
            badge.setText(f"○ {name}")
            badge.setStyleSheet(
                "background-color: #FEE2E2; color: #DC2626; border-radius: 12px; "
                "padding: 4px 10px; font-weight: 700; font-size: 11px;"
            )

    # ==========================================================================
    # CÁC TAB CHUYÊN DỤNG
    # ==========================================================================

    def create_amcl_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        node = 'amcl'

        # Grp 1: Particle Limits
        grp_particles = QGroupBox("1. Bộ lọc phân tán hạt (Particle Population)")
        g1 = QGridLayout(grp_particles)
        self.add_param_slider(g1, 0, node, "min_particles", "Số hạt tối thiểu:", 100, 3000, 500, scale=1, val_type='int')
        self.add_param_slider(g1, 1, node, "max_particles", "Số hạt tối đa:", 500, 8000, 2000, scale=1, val_type='int')
        self.add_param_slider(g1, 2, node, "update_min_d", "Ngưỡng di chuyển d (m):", 0.02, 0.8, 0.25, scale=100, val_type='double')
        self.add_param_slider(g1, 3, node, "update_min_a", "Ngưỡng góc xoay a (rad):", 0.02, 1.0, 0.2, scale=100, val_type='double')
        layout.addWidget(grp_particles)

        # Grp 2: Odometry Alpha Noise
        grp_alpha = QGroupBox("2. Mô hình sai số bánh xe (Odometry Noise Alpha 1 - 5)")
        g2 = QGridLayout(grp_alpha)
        self.add_param_slider(g2, 0, node, "alpha1", "Alpha 1 (Xoay khi đang xoay):", 0.0, 1.0, 0.02, scale=100, val_type='double')
        self.add_param_slider(g2, 1, node, "alpha2", "Alpha 2 (Xoay khi chạy thẳng):", 0.0, 1.0, 0.02, scale=100, val_type='double')
        self.add_param_slider(g2, 2, node, "alpha3", "Alpha 3 (Trượt khi chạy thẳng):", 0.0, 1.0, 0.02, scale=100, val_type='double')
        self.add_param_slider(g2, 3, node, "alpha4", "Alpha 4 (Trượt khi đang xoay):", 0.0, 1.0, 0.02, scale=100, val_type='double')
        self.add_param_slider(g2, 4, node, "alpha5", "Alpha 5 (Nhiễu dịch chuyển Omni):", 0.0, 1.0, 0.01, scale=100, val_type='double')
        layout.addWidget(grp_alpha)

        # Grp 3: Laser Likelihood Sensor
        grp_sensor = QGroupBox("3. Mô hình Cảm biến Laser (Laser Sensor Model)")
        g3 = QGridLayout(grp_sensor)
        self.add_param_slider(g3, 0, node, "laser_max_range", "Tầm quét Laser tối đa (m):", 2.0, 100.0, 100.0, scale=10, val_type='double')
        self.add_param_slider(g3, 1, node, "sigma_hit", "Độ lệch chuẩn tia trúng sigma_hit (m):", 0.02, 0.5, 0.12, scale=100, val_type='double')
        self.add_param_slider(g3, 2, node, "z_hit", "Xác suất quét trúng tường z_hit:", 0.1, 1.0, 0.85, scale=100, val_type='double')
        self.add_param_slider(g3, 3, node, "z_rand", "Xác suất nhiễu ngẫu nhiên z_rand:", 0.0, 0.8, 0.15, scale=100, val_type='double')
        layout.addWidget(grp_sensor)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    def create_costmap_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        local_node = 'local_costmap/local_costmap'
        global_node = 'global_costmap/global_costmap'

        # Grp 1: Local Costmap (VoxelLayer + Inflation)
        grp_local = QGroupBox("1. Bản đồ né cản cục bộ (Local Costmap - VoxelLayer: LiDAR & Camera)")
        g1 = QGridLayout(grp_local)
        self.add_param_slider(g1, 0, local_node, "inflation_layer.inflation_radius", "Bán kính đệm an toàn (m):", 0.1, 1.5, 0.40, scale=100, val_type='double')
        self.add_param_slider(g1, 1, local_node, "inflation_layer.cost_scaling_factor", "Độ dốc né cản (Scaling):", 0.5, 15.0, 3.0, scale=10, val_type='double')
        self.add_param_slider(g1, 2, local_node, "voxel_layer.scan.obstacle_max_range", "LiDAR - Tầm bắt cản tối đa (m):", 0.5, 8.0, 2.5, scale=10, val_type='double')
        self.add_param_slider(g1, 3, local_node, "voxel_layer.scan.raytrace_max_range", "LiDAR - Tầm xóa cản tối đa (m):", 0.5, 8.0, 3.0, scale=10, val_type='double')
        self.add_param_slider(g1, 4, local_node, "voxel_layer.scan_camera.obstacle_max_range", "Camera - Tầm bắt cản tối đa (m):", 0.5, 6.0, 2.5, scale=10, val_type='double')
        self.add_param_slider(g1, 5, local_node, "voxel_layer.scan_camera.raytrace_max_range", "Camera - Tầm xóa cản tối đa (m):", 0.5, 6.0, 3.0, scale=10, val_type='double')
        self.add_param_slider(g1, 6, local_node, "update_frequency", "Tần số cập nhật Local Costmap (Hz):", 1.0, 20.0, 5.0, scale=10, val_type='double')
        layout.addWidget(grp_local)

        # Grp 2: Global Costmap (ObstacleLayer + Inflation)
        grp_global = QGroupBox("2. Bản đồ quy hoạch toàn cục (Global Costmap - ObstacleLayer & Inflation)")
        g2 = QGridLayout(grp_global)
        self.add_param_slider(g2, 0, global_node, "inflation_layer.inflation_radius", "Bán kính đệm toàn cục (m):", 0.1, 1.5, 0.40, scale=100, val_type='double')
        self.add_param_slider(g2, 1, global_node, "inflation_layer.cost_scaling_factor", "Độ dốc né cản toàn cục:", 0.5, 15.0, 3.0, scale=10, val_type='double')
        self.add_param_slider(g2, 2, global_node, "obstacle_layer.scan.obstacle_max_range", "LiDAR Global - Tầm bắt cản (m):", 0.5, 8.0, 2.5, scale=10, val_type='double')
        self.add_param_slider(g2, 3, global_node, "obstacle_layer.scan.raytrace_max_range", "LiDAR Global - Tầm xóa cản (m):", 0.5, 8.0, 3.0, scale=10, val_type='double')
        layout.addWidget(grp_global)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    def create_controller_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        node = 'controller_server'

        # Grp 1: MPPI Velocity & Motion Model
        grp_vel = QGroupBox("1. Vận tốc, Mô hình chuyển động & Prune (FollowPath)")
        g1 = QGridLayout(grp_vel)
        self.add_param_combo(g1, 0, node, "FollowPath.motion_model", "Mô hình động học (motion_model):", ["DiffDrive", "Omni"], "DiffDrive")
        self.add_param_slider(g1, 1, node, "FollowPath.vx_max", "Vận tốc tiến tối đa vx_max (m/s):", 0.05, 1.2, 0.4, scale=100, val_type='double')
        self.add_param_slider(g1, 2, node, "FollowPath.vx_min", "Vận tốc lùi vx_min (m/s):", -0.6, 0.0, 0.0, scale=100, val_type='double')
        self.add_param_slider(g1, 3, node, "FollowPath.vy_max", "Vận tốc ngang tối đa vy_max (Omni, m/s):", 0.0, 0.8, 0.5, scale=100, val_type='double')
        self.add_param_slider(g1, 4, node, "FollowPath.wz_max", "Tốc độ xoay tối đa wz_max (rad/s):", 0.1, 2.0, 0.5, scale=100, val_type='double')
        self.add_param_slider(g1, 5, node, "FollowPath.prune_distance", "Khoảng cách tỉa đường prune_distance (m):", 0.2, 3.0, 1.7, scale=100, val_type='double')
        layout.addWidget(grp_vel)

        # Grp 2: MPPI Dynamics, Noise & Performance
        grp_perf = QGroupBox("2. Hiệu năng, Gia tốc & Lấy mẫu MPPI (Orin Nano)")
        g2 = QGridLayout(grp_perf)
        self.add_param_slider(g2, 0, node, "controller_frequency", "Tần số Controller Server (Hz):", 2.0, 30.0, 5.0, scale=10, val_type='double')
        self.add_param_slider(g2, 1, node, "FollowPath.batch_size", "Số mẫu quỹ đạo (batch_size):", 300, 3000, 1500, scale=1, val_type='int')
        self.add_param_slider(g2, 2, node, "FollowPath.time_steps", "Số bước dự đoán (time_steps):", 5, 40, 10, scale=1, val_type='int')
        self.add_param_slider(g2, 3, node, "FollowPath.model_dt", "Bước thời gian mô hình (model_dt, s):", 0.05, 0.5, 0.2, scale=100, val_type='double')
        self.add_param_slider(g2, 4, node, "FollowPath.iteration_count", "Số vòng lặp tối ưu (iteration_count):", 1, 4, 1, scale=1, val_type='int')
        self.add_param_slider(g2, 5, node, "FollowPath.ax_max", "Gia tốc tiến ax_max (m/s²):", 0.2, 3.0, 0.6, scale=100, val_type='double')
        self.add_param_slider(g2, 6, node, "FollowPath.ax_min", "Giảm tốc ax_min (m/s²):", -3.0, -0.2, -0.6, scale=100, val_type='double')
        self.add_param_slider(g2, 7, node, "FollowPath.az_max", "Gia tốc xoay az_max (rad/s²):", 0.2, 3.2, 0.4, scale=100, val_type='double')
        self.add_param_slider(g2, 8, node, "FollowPath.vx_std", "Độ lệch chuẩn mẫu vx_std:", 0.02, 0.5, 0.15, scale=100, val_type='double')
        self.add_param_slider(g2, 9, node, "FollowPath.vy_std", "Độ lệch chuẩn mẫu ngang vy_std (Omni):", 0.0, 0.5, 0.0, scale=100, val_type='double')
        self.add_param_slider(g2, 10, node, "FollowPath.wz_std", "Độ lệch chuẩn mẫu xoay wz_std:", 0.05, 1.0, 0.3, scale=100, val_type='double')
        self.add_param_slider(g2, 11, node, "FollowPath.temperature", "Nhiệt độ lấy mẫu (temperature):", 0.05, 1.0, 0.3, scale=100, val_type='double')
        self.add_param_slider(g2, 12, node, "FollowPath.gamma", "Hệ số gamma:", 0.001, 0.2, 0.015, scale=1000, val_type='double', precision=3)
        layout.addWidget(grp_perf)

        # Grp 3: Goal Checker & Progress Checker
        grp_goal = QGroupBox("3. Ngưỡng đạt đích & Phát hiện kẹt (Goal & Progress Checker)")
        g_goal = QGridLayout(grp_goal)
        self.add_param_slider(g_goal, 0, node, "precise_goal_checker.xy_goal_tolerance", "Sai số vị trí tới đích xy_goal_tolerance (m):", 0.05, 0.60, 0.25, scale=100, val_type='double')
        self.add_param_slider(g_goal, 1, node, "precise_goal_checker.yaw_goal_tolerance", "Sai số góc tới đích yaw_goal_tolerance (rad):", 0.05, 3.14, 0.25, scale=100, val_type='double')
        self.add_param_slider(g_goal, 2, node, "progress_checker.required_movement_radius", "Bán kính di chuyển tối thiểu (m):", 0.10, 1.50, 0.50, scale=100, val_type='double')
        self.add_param_slider(g_goal, 3, node, "progress_checker.movement_time_allowance", "Thời gian cho phép trước khi báo kẹt (s):", 2.0, 30.0, 10.0, scale=10, val_type='double')
        layout.addWidget(grp_goal)

        # Grp 4: MPPI Critics Tuning (Đủ 8 Critics)
        grp_critics = QGroupBox("4. Trọng số đánh giá quỹ đạo MPPI (Critics Weights)")
        g3 = QGridLayout(grp_critics)
        self.add_param_slider(g3, 0, node, "FollowPath.ConstraintCritic.cost_weight", "Ràng buộc động học (ConstraintCritic):", 0.0, 20.0, 4.0, scale=10, val_type='double')
        self.add_param_slider(g3, 1, node, "FollowPath.CostCritic.cost_weight", "Né vật cản bản đồ (CostCritic):", 0.0, 20.0, 3.81, scale=100, val_type='double')
        self.add_param_slider(g3, 2, node, "FollowPath.GoalCritic.cost_weight", "Ưu tiên tiến tới đích (GoalCritic):", 0.0, 20.0, 5.0, scale=10, val_type='double')
        self.add_param_slider(g3, 3, node, "FollowPath.GoalAngleCritic.cost_weight", "Xoay đúng góc đích (GoalAngleCritic):", 0.0, 20.0, 3.0, scale=10, val_type='double')
        self.add_param_slider(g3, 4, node, "FollowPath.PathAlignCritic.cost_weight", "Bám sát đường đi (PathAlignCritic):", 0.0, 25.0, 14.0, scale=10, val_type='double')
        self.add_param_slider(g3, 5, node, "FollowPath.PathFollowCritic.cost_weight", "Theo tuyến đường (PathFollowCritic):", 0.0, 20.0, 5.0, scale=10, val_type='double')
        self.add_param_slider(g3, 6, node, "FollowPath.PathAngleCritic.cost_weight", "Giữ hướng theo đường (PathAngleCritic):", 0.0, 20.0, 2.0, scale=10, val_type='double')
        self.add_param_slider(g3, 7, node, "FollowPath.PreferForwardCritic.cost_weight", "Ưu tiên chạy tiến (PreferForwardCritic):", 0.0, 20.0, 5.0, scale=10, val_type='double')
        layout.addWidget(grp_critics)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    def create_velocity_smoother_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        lbl_note = QLabel(
            "Lưu ý: velocity_smoother nằm giữa controller_server và collision_monitor. "
            "Nếu giá trị kẹp ở đây nhỏ hơn MPPI (ví dụ max_velocity[2] = 0.3), tốc độ thực tế của xe sẽ bị giới hạn theo mức này."
        )
        lbl_note.setWordWrap(True)
        lbl_note.setStyleSheet(
            "background-color: #EFF6FF; color: #1E40AF; border: 1px solid #BFDBFE; "
            "border-radius: 6px; padding: 8px 12px; font-weight: 600;"
        )
        layout.addWidget(lbl_note)

        node = 'velocity_smoother'

        grp_max = QGroupBox("1. Vận tốc tối đa (max_velocity: [vx, vy, wz])")
        g1 = QGridLayout(grp_max)
        self.add_param_slider(g1, 0, node, "max_velocity[0]", "Tiến tối đa vx (m/s):", 0.05, 1.2, 0.4, scale=100, val_type='double', array_param='max_velocity')
        self.add_param_slider(g1, 1, node, "max_velocity[1]", "Ngang tối đa vy (m/s):", 0.0, 0.8, 0.0, scale=100, val_type='double', array_param='max_velocity')
        self.add_param_slider(g1, 2, node, "max_velocity[2]", "Xoay tối đa wz (rad/s):", 0.1, 2.0, 0.3, scale=100, val_type='double', array_param='max_velocity')
        layout.addWidget(grp_max)

        grp_min = QGroupBox("2. Vận tốc tối thiểu / lùi (min_velocity: [vx, vy, wz])")
        g2 = QGridLayout(grp_min)
        self.add_param_slider(g2, 0, node, "min_velocity[0]", "Lùi tối đa vx_min (m/s):", -0.8, 0.0, -0.35, scale=100, val_type='double', array_param='min_velocity')
        self.add_param_slider(g2, 1, node, "min_velocity[1]", "Ngang trái vy_min (m/s):", -0.8, 0.0, 0.0, scale=100, val_type='double', array_param='min_velocity')
        self.add_param_slider(g2, 2, node, "min_velocity[2]", "Xoay ngược wz_min (rad/s):", -2.0, -0.1, -0.3, scale=100, val_type='double', array_param='min_velocity')
        layout.addWidget(grp_min)

        grp_acc = QGroupBox("3. Gia tốc & Giảm tốc (max_accel / max_decel)")
        g3 = QGridLayout(grp_acc)
        self.add_param_slider(g3, 0, node, "max_accel[0]", "Gia tốc thẳng max_accel_x (m/s²):", 0.2, 4.0, 2.5, scale=100, val_type='double', array_param='max_accel')
        self.add_param_slider(g3, 1, node, "max_accel[1]", "Gia tốc ngang max_accel_y (m/s²):", 0.0, 3.0, 0.0, scale=100, val_type='double', array_param='max_accel')
        self.add_param_slider(g3, 2, node, "max_accel[2]", "Gia tốc xoay max_accel_z (rad/s²):", 0.1, 4.0, 0.4, scale=100, val_type='double', array_param='max_accel')
        self.add_param_slider(g3, 3, node, "max_decel[0]", "Giảm tốc thẳng max_decel_x (m/s²):", -4.0, -0.2, -2.5, scale=100, val_type='double', array_param='max_decel')
        self.add_param_slider(g3, 4, node, "max_decel[1]", "Giảm tốc ngang max_decel_y (m/s²):", -3.0, 0.0, 0.0, scale=100, val_type='double', array_param='max_decel')
        self.add_param_slider(g3, 5, node, "max_decel[2]", "Giảm tốc xoay max_decel_z (rad/s²):", -4.0, -0.2, -3.2, scale=100, val_type='double', array_param='max_decel')
        layout.addWidget(grp_acc)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    def create_safety_planner_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        # Grp 1: Planner Server
        grp_plan = QGroupBox("1. Bộ lập tuyến đường toàn cục (Planner Server - Navfn)")
        g1 = QGridLayout(grp_plan)
        self.add_param_slider(g1, 0, 'planner_server', "expected_planner_frequency", "Tần số lập đường đi (Hz):", 0.0, 30.0, 20.0, scale=10, val_type='double')
        self.add_param_slider(g1, 1, 'planner_server', "GridBased.tolerance", "Dung sai điểm đích Planner (m):", 0.05, 1.5, 0.5, scale=100, val_type='double')
        self.add_param_checkbox(g1, 2, 'planner_server', "GridBased.use_astar", "Dùng A* thay cho Dijkstra (use_astar)", False)
        layout.addWidget(grp_plan)

        # Grp 2: Collision Monitor
        grp_cm = QGroupBox("2. Phanh an toàn khẩn cấp (Collision Monitor)")
        g2 = QGridLayout(grp_cm)
        self.add_param_checkbox(g2, 0, 'collision_monitor', "FootprintApproach.enabled", "Bật vùng phanh FootprintApproach", True)
        self.add_param_slider(g2, 1, 'collision_monitor', "FootprintApproach.time_before_collision", "Thời gian dự báo va chạm (s):", 0.2, 3.0, 1.2, scale=10, val_type='double')
        self.add_param_slider(g2, 2, 'collision_monitor', "FootprintApproach.min_points", "Số điểm tối thiểu kích hoạt phanh:", 1, 30, 6, scale=1, val_type='int')
        self.add_param_slider(g2, 3, 'collision_monitor', "scan.min_height", "LiDAR - Chiều cao lọc tối thiểu (m):", 0.0, 1.0, 0.15, scale=100, val_type='double')
        self.add_param_slider(g2, 4, 'collision_monitor', "scan.max_height", "LiDAR - Chiều cao lọc tối đa (m):", 0.5, 3.0, 2.0, scale=10, val_type='double')
        self.add_param_slider(g2, 5, 'collision_monitor', "scan_camera.min_height", "Camera - Chiều cao lọc tối thiểu (m):", 0.0, 1.0, 0.15, scale=100, val_type='double')
        self.add_param_slider(g2, 6, 'collision_monitor', "scan_camera.max_height", "Camera - Chiều cao lọc tối đa (m):", 0.5, 3.0, 2.0, scale=10, val_type='double')
        layout.addWidget(grp_cm)

        # Grp 3: Behavior Server (Recovery)
        grp_beh = QGroupBox("3. Hành vi phục hồi khi kẹt (Behavior Server: Spin / BackUp)")
        g3 = QGridLayout(grp_beh)
        self.add_param_slider(g3, 0, 'behavior_server', "max_rotational_vel", "Tốc độ xoay phục hồi tối đa (rad/s):", 0.2, 2.0, 1.0, scale=100, val_type='double')
        self.add_param_slider(g3, 1, 'behavior_server', "min_rotational_vel", "Tốc độ xoay phục hồi tối thiểu (rad/s):", 0.05, 1.0, 0.4, scale=100, val_type='double')
        self.add_param_slider(g3, 2, 'behavior_server', "rotational_acc_lim", "Gia tốc xoay phục hồi (rad/s²):", 0.4, 5.0, 3.2, scale=100, val_type='double')
        layout.addWidget(grp_beh)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    def create_slam_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(12)

        node = 'slam_toolbox'

        grp_slam = QGroupBox("1. Thông số tạo bản đồ trực tiếp (SLAM Mapping Parameters)")
        g1 = QGridLayout(grp_slam)
        self.add_param_slider(g1, 0, node, "minimum_travel_distance", "Quãng đường chèn quét mới (m):", 0.05, 1.0, 0.2, scale=100, val_type='double')
        self.add_param_slider(g1, 1, node, "minimum_travel_heading", "Góc xoay chèn quét mới (rad):", 0.05, 1.2, 0.3, scale=100, val_type='double')
        self.add_param_slider(g1, 2, node, "loop_search_maximum_distance", "Bán kính tìm vòng lặp (m):", 1.0, 15.0, 6.0, scale=10, val_type='double')
        self.add_param_slider(g1, 3, node, "link_match_minimum_response_fine", "Ngưỡng khớp khớp nối Fine:", 0.1, 1.0, 0.65, scale=100, val_type='double')
        layout.addWidget(grp_slam)

        layout.addStretch()
        scroll.setWidget(content)
        return scroll

    # ==========================================================================
    # TIỆN ÍCH TẠO ĐIỀU KHIỂN VÀ LIÊN KẾT REALTIME
    # ==========================================================================

    def add_param_slider(self, grid, row, node_name, param_name, label_text, min_val, max_val, default_val, scale=1, val_type='double', precision=2, array_param=None):
        lbl_name = QLabel(label_text)
        lbl_name.setStyleSheet("font-weight: 500;")

        slider = QSlider(Qt.Horizontal)
        slider_min = int(round(min_val * scale))
        slider_max = int(round(max_val * scale))
        slider_def = int(round(default_val * scale))

        slider.setRange(slider_min, slider_max)
        slider.setValue(slider_def)

        txt_def = str(int(default_val)) if val_type == 'int' else f"{float(default_val):.{precision}f}"
        lbl_val = QLabel(txt_def)
        lbl_val.setStyleSheet("font-weight: 700; color: #2563EB; min-width: 60px;")
        lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        grid.addWidget(lbl_name, row, 0)
        grid.addWidget(slider, row, 1)
        grid.addWidget(lbl_val, row, 2)

        key = (node_name, param_name)
        self.param_controls[key] = {
            'kind': 'slider',
            'slider': slider,
            'label': lbl_val,
            'scale': scale,
            'precision': precision,
            'type': val_type,
            'node': node_name,
            'param': param_name,
            'default': default_val,
            'array_param': array_param,
        }

        slider.valueChanged.connect(lambda val, k=key: self.on_control_changed(k))

    def add_param_checkbox(self, grid, row, node_name, param_name, label_text, default_val=False):
        chk = QCheckBox(label_text)
        chk.setChecked(bool(default_val))
        grid.addWidget(chk, row, 0, 1, 3)

        key = (node_name, param_name)
        self.param_controls[key] = {
            'kind': 'checkbox',
            'widget': chk,
            'type': 'bool',
            'node': node_name,
            'param': param_name,
            'default': bool(default_val),
            'array_param': None,
        }

        chk.stateChanged.connect(lambda _st, k=key: self.on_control_changed(k))

    def add_param_combo(self, grid, row, node_name, param_name, label_text, options, default_val):
        lbl_name = QLabel(label_text)
        lbl_name.setStyleSheet("font-weight: 500;")
        combo = QComboBox()
        for opt in options:
            combo.addItem(opt)
        idx = combo.findText(str(default_val))
        if idx >= 0:
            combo.setCurrentIndex(idx)

        grid.addWidget(lbl_name, row, 0)
        grid.addWidget(combo, row, 1, 1, 2)

        key = (node_name, param_name)
        self.param_controls[key] = {
            'kind': 'combo',
            'widget': combo,
            'type': 'string',
            'node': node_name,
            'param': param_name,
            'default': str(default_val),
            'array_param': None,
        }

        combo.currentTextChanged.connect(lambda _txt, k=key: self.on_control_changed(k))

    def _format_slider_text(self, info, val):
        if info['type'] == 'int':
            return str(int(val))
        return f"{float(val):.{info.get('precision', 2)}f}"

    def on_control_changed(self, key):
        info = self.param_controls[key]
        if info['kind'] == 'slider':
            val = self.get_control_value(key)
            info['label'].setText(self._format_slider_text(info, val))

        if not self.chk_auto_apply.isChecked():
            return

        array_param = info.get('array_param')
        if array_param:
            debounce_key = (info['node'], array_param)
            send_node = info['node']
            send_param = array_param
            send_type = 'double_array'
            get_payload = lambda n=send_node, p=send_param: [
                float(self.get_control_value(ck)) for ck in self.array_param_groups[(n, p)]
            ]
        else:
            debounce_key = key
            send_node = info['node']
            send_param = info['param']
            send_type = info['type']
            get_payload = lambda k=key: self.get_control_value(k)

        if debounce_key not in self.debounce_timers:
            timer = QTimer(self)
            timer.setSingleShot(True)
            self.debounce_timers[debounce_key] = timer

        timer = self.debounce_timers[debounce_key]
        try:
            timer.timeout.disconnect()
        except Exception:
            pass

        timer.timeout.connect(lambda: self.ros_worker.set_param_async(
            send_node, send_param, get_payload(), send_type
        ))
        timer.start(80)

    # ==========================================================================
    # ĐỒNG BỘ VÀ LƯU TRỮ (SYNC & SAVE TO YAML)
    # ==========================================================================

    def on_sync_all(self):
        self.append_log("Đang đồng bộ tham số thực tế từ các node ROS 2...", "INFO")
        queried_arrays = set()
        for (node_name, param_name), info in self.param_controls.items():
            array_param = info.get('array_param')
            if array_param:
                arr_key = (node_name, array_param)
                if arr_key not in queried_arrays:
                    queried_arrays.add(arr_key)
                    self.ros_worker.query_param_async(node_name, array_param)
            else:
                self.ros_worker.query_param_async(node_name, param_name)

    def handle_synced_value(self, node_name, param_name, val):
        arr_key = (node_name, param_name)
        if arr_key in self.array_param_groups and isinstance(val, (list, tuple)):
            for idx, comp_key in enumerate(self.array_param_groups[arr_key]):
                if idx < len(val):
                    self.set_control_value(comp_key, val[idx], apply_to_robot=False)
            return

        key = (node_name, param_name)
        if key in self.param_controls:
            self.set_control_value(key, val, apply_to_robot=False)

    def _get_yaml_mappings(self):
        """Declarative (control_key, yaml_path_tuple) mappings for nav2_params.yaml & SLAM."""
        nav2_scalars = [
            # AMCL
            (('amcl', 'min_particles'), ('amcl', 'ros__parameters', 'min_particles')),
            (('amcl', 'max_particles'), ('amcl', 'ros__parameters', 'max_particles')),
            (('amcl', 'update_min_d'), ('amcl', 'ros__parameters', 'update_min_d')),
            (('amcl', 'update_min_a'), ('amcl', 'ros__parameters', 'update_min_a')),
            (('amcl', 'alpha1'), ('amcl', 'ros__parameters', 'alpha1')),
            (('amcl', 'alpha2'), ('amcl', 'ros__parameters', 'alpha2')),
            (('amcl', 'alpha3'), ('amcl', 'ros__parameters', 'alpha3')),
            (('amcl', 'alpha4'), ('amcl', 'ros__parameters', 'alpha4')),
            (('amcl', 'alpha5'), ('amcl', 'ros__parameters', 'alpha5')),
            (('amcl', 'laser_max_range'), ('amcl', 'ros__parameters', 'laser_max_range')),
            (('amcl', 'sigma_hit'), ('amcl', 'ros__parameters', 'sigma_hit')),
            (('amcl', 'z_hit'), ('amcl', 'ros__parameters', 'z_hit')),
            (('amcl', 'z_rand'), ('amcl', 'ros__parameters', 'z_rand')),
            # Local Costmap (VoxelLayer + Inflation)
            (('local_costmap/local_costmap', 'update_frequency'), ('local_costmap', 'local_costmap', 'ros__parameters', 'update_frequency')),
            (('local_costmap/local_costmap', 'inflation_layer.inflation_radius'), ('local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius')),
            (('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor'), ('local_costmap', 'local_costmap', 'ros__parameters', 'inflation_layer', 'cost_scaling_factor')),
            (('local_costmap/local_costmap', 'voxel_layer.scan.obstacle_max_range'), ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'scan', 'obstacle_max_range')),
            (('local_costmap/local_costmap', 'voxel_layer.scan.raytrace_max_range'), ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'scan', 'raytrace_max_range')),
            (('local_costmap/local_costmap', 'voxel_layer.scan_camera.obstacle_max_range'), ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'scan_camera', 'obstacle_max_range')),
            (('local_costmap/local_costmap', 'voxel_layer.scan_camera.raytrace_max_range'), ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'scan_camera', 'raytrace_max_range')),
            # Global Costmap (ObstacleLayer + Inflation)
            (('global_costmap/global_costmap', 'inflation_layer.inflation_radius'), ('global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer', 'inflation_radius')),
            (('global_costmap/global_costmap', 'inflation_layer.cost_scaling_factor'), ('global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer', 'cost_scaling_factor')),
            (('global_costmap/global_costmap', 'obstacle_layer.scan.obstacle_max_range'), ('global_costmap', 'global_costmap', 'ros__parameters', 'obstacle_layer', 'scan', 'obstacle_max_range')),
            (('global_costmap/global_costmap', 'obstacle_layer.scan.raytrace_max_range'), ('global_costmap', 'global_costmap', 'ros__parameters', 'obstacle_layer', 'scan', 'raytrace_max_range')),
            # Controller Server + Goal/Progress Checkers + FollowPath (MPPI)
            (('controller_server', 'controller_frequency'), ('controller_server', 'ros__parameters', 'controller_frequency')),
            (('controller_server', 'precise_goal_checker.xy_goal_tolerance'), ('controller_server', 'ros__parameters', 'precise_goal_checker', 'xy_goal_tolerance')),
            (('controller_server', 'precise_goal_checker.yaw_goal_tolerance'), ('controller_server', 'ros__parameters', 'precise_goal_checker', 'yaw_goal_tolerance')),
            (('controller_server', 'progress_checker.required_movement_radius'), ('controller_server', 'ros__parameters', 'progress_checker', 'required_movement_radius')),
            (('controller_server', 'progress_checker.movement_time_allowance'), ('controller_server', 'ros__parameters', 'progress_checker', 'movement_time_allowance')),
            (('controller_server', 'FollowPath.motion_model'), ('controller_server', 'ros__parameters', 'FollowPath', 'motion_model')),
            (('controller_server', 'FollowPath.vx_max'), ('controller_server', 'ros__parameters', 'FollowPath', 'vx_max')),
            (('controller_server', 'FollowPath.vx_min'), ('controller_server', 'ros__parameters', 'FollowPath', 'vx_min')),
            (('controller_server', 'FollowPath.vy_max'), ('controller_server', 'ros__parameters', 'FollowPath', 'vy_max')),
            (('controller_server', 'FollowPath.wz_max'), ('controller_server', 'ros__parameters', 'FollowPath', 'wz_max')),
            (('controller_server', 'FollowPath.prune_distance'), ('controller_server', 'ros__parameters', 'FollowPath', 'prune_distance')),
            (('controller_server', 'FollowPath.batch_size'), ('controller_server', 'ros__parameters', 'FollowPath', 'batch_size')),
            (('controller_server', 'FollowPath.time_steps'), ('controller_server', 'ros__parameters', 'FollowPath', 'time_steps')),
            (('controller_server', 'FollowPath.model_dt'), ('controller_server', 'ros__parameters', 'FollowPath', 'model_dt')),
            (('controller_server', 'FollowPath.iteration_count'), ('controller_server', 'ros__parameters', 'FollowPath', 'iteration_count')),
            (('controller_server', 'FollowPath.ax_max'), ('controller_server', 'ros__parameters', 'FollowPath', 'ax_max')),
            (('controller_server', 'FollowPath.ax_min'), ('controller_server', 'ros__parameters', 'FollowPath', 'ax_min')),
            (('controller_server', 'FollowPath.az_max'), ('controller_server', 'ros__parameters', 'FollowPath', 'az_max')),
            (('controller_server', 'FollowPath.vx_std'), ('controller_server', 'ros__parameters', 'FollowPath', 'vx_std')),
            (('controller_server', 'FollowPath.vy_std'), ('controller_server', 'ros__parameters', 'FollowPath', 'vy_std')),
            (('controller_server', 'FollowPath.wz_std'), ('controller_server', 'ros__parameters', 'FollowPath', 'wz_std')),
            (('controller_server', 'FollowPath.temperature'), ('controller_server', 'ros__parameters', 'FollowPath', 'temperature')),
            (('controller_server', 'FollowPath.gamma'), ('controller_server', 'ros__parameters', 'FollowPath', 'gamma')),
            (('controller_server', 'FollowPath.ConstraintCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'ConstraintCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.CostCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'CostCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.GoalCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'GoalCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.GoalAngleCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'GoalAngleCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.PathAlignCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'PathAlignCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.PathFollowCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'PathFollowCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.PathAngleCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'PathAngleCritic', 'cost_weight')),
            (('controller_server', 'FollowPath.PreferForwardCritic.cost_weight'), ('controller_server', 'ros__parameters', 'FollowPath', 'PreferForwardCritic', 'cost_weight')),
            # Planner Server
            (('planner_server', 'expected_planner_frequency'), ('planner_server', 'ros__parameters', 'expected_planner_frequency')),
            (('planner_server', 'GridBased.tolerance'), ('planner_server', 'ros__parameters', 'GridBased', 'tolerance')),
            (('planner_server', 'GridBased.use_astar'), ('planner_server', 'ros__parameters', 'GridBased', 'use_astar')),
            # Collision Monitor
            (('collision_monitor', 'FootprintApproach.enabled'), ('collision_monitor', 'ros__parameters', 'FootprintApproach', 'enabled')),
            (('collision_monitor', 'FootprintApproach.time_before_collision'), ('collision_monitor', 'ros__parameters', 'FootprintApproach', 'time_before_collision')),
            (('collision_monitor', 'FootprintApproach.min_points'), ('collision_monitor', 'ros__parameters', 'FootprintApproach', 'min_points')),
            (('collision_monitor', 'scan.min_height'), ('collision_monitor', 'ros__parameters', 'scan', 'min_height')),
            (('collision_monitor', 'scan.max_height'), ('collision_monitor', 'ros__parameters', 'scan', 'max_height')),
            (('collision_monitor', 'scan_camera.min_height'), ('collision_monitor', 'ros__parameters', 'scan_camera', 'min_height')),
            (('collision_monitor', 'scan_camera.max_height'), ('collision_monitor', 'ros__parameters', 'scan_camera', 'max_height')),
            # Behavior Server
            (('behavior_server', 'max_rotational_vel'), ('behavior_server', 'ros__parameters', 'max_rotational_vel')),
            (('behavior_server', 'min_rotational_vel'), ('behavior_server', 'ros__parameters', 'min_rotational_vel')),
            (('behavior_server', 'rotational_acc_lim'), ('behavior_server', 'ros__parameters', 'rotational_acc_lim')),
        ]
        nav2_arrays = [
            (('velocity_smoother', 'max_velocity'), ('velocity_smoother', 'ros__parameters', 'max_velocity')),
            (('velocity_smoother', 'min_velocity'), ('velocity_smoother', 'ros__parameters', 'min_velocity')),
            (('velocity_smoother', 'max_accel'), ('velocity_smoother', 'ros__parameters', 'max_accel')),
            (('velocity_smoother', 'max_decel'), ('velocity_smoother', 'ros__parameters', 'max_decel')),
        ]
        slam_scalars = [
            (('slam_toolbox', 'minimum_travel_distance'), ('slam_toolbox', 'ros__parameters', 'minimum_travel_distance')),
            (('slam_toolbox', 'minimum_travel_heading'), ('slam_toolbox', 'ros__parameters', 'minimum_travel_heading')),
            (('slam_toolbox', 'loop_search_maximum_distance'), ('slam_toolbox', 'ros__parameters', 'loop_search_maximum_distance')),
            (('slam_toolbox', 'link_match_minimum_response_fine'), ('slam_toolbox', 'ros__parameters', 'link_match_minimum_response_fine')),
        ]
        return nav2_scalars, nav2_arrays, slam_scalars

    @staticmethod
    def _yaml_get_nested(data, path):
        cur = data
        for key in path:
            if not isinstance(cur, dict) or key not in cur:
                return None
            cur = cur[key]
        return cur

    @staticmethod
    def _yaml_set_nested(data, path, value):
        cur = data
        for key in path[:-1]:
            if not isinstance(cur, dict) or key not in cur:
                return False
            cur = cur[key]
        if isinstance(cur, dict):
            cur[path[-1]] = value
            return True
        return False

    def on_save_to_yaml(self):
        """Ghi đè trực tiếp các tham số vừa tinh chỉnh vào các file YAML gốc"""
        try:
            nav2_scalars, nav2_arrays, slam_scalars = self._get_yaml_mappings()

            # 1. Update nav2_params.yaml
            if os.path.exists(self.nav2_yaml_path):
                with open(self.nav2_yaml_path, 'r', encoding='utf-8') as f:
                    nav2_data = yaml.safe_load(f)

                for ctrl_key, ypath in nav2_scalars:
                    if ctrl_key in self.param_controls:
                        self._yaml_set_nested(nav2_data, ypath, self.get_control_value(ctrl_key))

                for arr_key, ypath in nav2_arrays:
                    if arr_key in self.array_param_groups:
                        vec = [float(self.get_control_value(ck)) for ck in self.array_param_groups[arr_key]]
                        self._yaml_set_nested(nav2_data, ypath, vec)

                with open(self.nav2_yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(nav2_data, f, default_flow_style=False, sort_keys=False)

                self.append_log(f"Đã lưu thành công vào: {self.nav2_yaml_path}", "SUCCESS")

            # 2. Update mapper_params_online_async.yaml (SLAM)
            if os.path.exists(self.slam_yaml_path):
                with open(self.slam_yaml_path, 'r', encoding='utf-8') as f:
                    slam_data = yaml.safe_load(f)

                for ctrl_key, ypath in slam_scalars:
                    if ctrl_key in self.param_controls:
                        self._yaml_set_nested(slam_data, ypath, self.get_control_value(ctrl_key))

                with open(self.slam_yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(slam_data, f, default_flow_style=False, sort_keys=False)

                self.append_log(f"Đã lưu thành công vào: {self.slam_yaml_path}", "SUCCESS")

            QMessageBox.information(self, "Lưu thành công", "Đã ghi đè toàn bộ tham số vào file YAML gốc!")

        except Exception as e:
            self.append_log(f"Lỗi khi lưu YAML: {str(e)}", "ERROR")
            QMessageBox.critical(self, "Lỗi", f"Không thể lưu file YAML: {str(e)}")

    def set_control_value(self, key, val, apply_to_robot=True):
        if key not in self.param_controls or val is None:
            return
        info = self.param_controls[key]
        kind = info['kind']

        if kind == 'slider':
            slider = info['slider']
            scale = info['scale']
            slider.blockSignals(True)
            slider.setValue(int(round(float(val) * scale)))
            info['label'].setText(self._format_slider_text(info, val))
            slider.blockSignals(False)
        elif kind == 'checkbox':
            chk = info['widget']
            chk.blockSignals(True)
            chk.setChecked(bool(val))
            chk.blockSignals(False)
        elif kind == 'combo':
            combo = info['widget']
            combo.blockSignals(True)
            idx = combo.findText(str(val))
            if idx >= 0:
                combo.setCurrentIndex(idx)
            combo.blockSignals(False)

        if apply_to_robot and self.chk_auto_apply.isChecked():
            array_param = info.get('array_param')
            if array_param:
                vec = [float(self.get_control_value(ck)) for ck in self.array_param_groups[(info['node'], array_param)]]
                self.ros_worker.set_param_async(info['node'], array_param, vec, 'double_array')
            else:
                self.ros_worker.set_param_async(info['node'], info['param'], self.get_control_value(key), info['type'])

    def on_preset_selected(self, index):
        if index == 1:
            self.apply_preset("khoaa1")
        elif index == 2:
            self.apply_preset("safe")
        elif index == 3:
            self.apply_preset("default")

    def apply_preset(self, preset_name):
        self.append_log(f"Đang kích hoạt Preset: [{preset_name}]...", "INFO")
        preset_values = {}
        if preset_name == "khoaa1":
            preset_values = {
                ('planner_server', 'expected_planner_frequency'): 1.0,
                ('controller_server', 'FollowPath.batch_size'): 800,
                ('controller_server', 'FollowPath.prune_distance'): 0.6,
                ('controller_server', 'FollowPath.vx_max'): 0.4,
                ('controller_server', 'FollowPath.vx_min'): -0.15,
                ('controller_server', 'FollowPath.wz_max'): 0.5,
                ('controller_server', 'FollowPath.PathAlignCritic.cost_weight'): 4.0,
                ('controller_server', 'FollowPath.GoalCritic.cost_weight'): 5.0,
                ('controller_server', 'FollowPath.CostCritic.cost_weight'): 3.8,
                ('controller_server', 'FollowPath.PreferForwardCritic.cost_weight'): 5.0,
                ('local_costmap/local_costmap', 'inflation_layer.inflation_radius'): 0.32,
                ('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor'): 4.5,
                ('global_costmap/global_costmap', 'inflation_layer.inflation_radius'): 0.45,
                ('global_costmap/global_costmap', 'inflation_layer.cost_scaling_factor'): 3.5,
                ('collision_monitor', 'FootprintApproach.time_before_collision'): 0.8,
                ('collision_monitor', 'FootprintApproach.min_points'): 12,
                ('amcl', 'update_min_d'): 0.1,
                ('amcl', 'update_min_a'): 0.15,
                ('amcl', 'laser_max_range'): 16.0,
            }
        elif preset_name == "safe":
            preset_values = {
                ('planner_server', 'expected_planner_frequency'): 1.0,
                ('controller_server', 'FollowPath.vx_max'): 0.25,
                ('controller_server', 'FollowPath.vx_min'): -0.1,
                ('controller_server', 'FollowPath.wz_max'): 0.4,
                ('controller_server', 'FollowPath.prune_distance'): 0.5,
                ('controller_server', 'FollowPath.batch_size'): 600,
                ('controller_server', 'FollowPath.PathAlignCritic.cost_weight'): 5.0,
                ('velocity_smoother', 'max_velocity[0]'): 0.25,
                ('velocity_smoother', 'max_velocity[2]'): 0.4,
                ('local_costmap/local_costmap', 'inflation_layer.inflation_radius'): 0.35,
                ('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor'): 5.0,
                ('collision_monitor', 'FootprintApproach.time_before_collision'): 1.0,
                ('collision_monitor', 'FootprintApproach.min_points'): 10,
            }
        elif preset_name == "default":
            for key, info in self.param_controls.items():
                preset_values[key] = info.get('default', 0)

        for key, val in preset_values.items():
            if key in self.param_controls and val is not None:
                self.set_control_value(key, val, apply_to_robot=True)

        self.append_log(f"Đã nạp và gửi thành công Preset [{preset_name}] tới Robot!", "SUCCESS")

    def on_load_yaml_file(self):
        init_dir = os.path.join(self.ws_dir, 'src/mo_hinh/config/nav_profiles')
        if not os.path.exists(init_dir):
            init_dir = os.path.join(self.ws_dir, 'src/mo_hinh/config')
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Chọn file YAML cấu hình Nav2 / SLAM",
            init_dir,
            "YAML Files (*.yaml *.yml);;All Files (*)"
        )
        if not file_path:
            return
        self.load_yaml_to_gui(file_path, silent=False, apply_to_robot=self.chk_auto_apply.isChecked())

    def load_yaml_to_gui(self, file_path, silent=False, apply_to_robot=True):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                raise ValueError("Nội dung file YAML không hợp lệ.")

            nav2_scalars, nav2_arrays, slam_scalars = self._get_yaml_mappings()
            loaded_count = 0

            for ctrl_key, ypath in nav2_scalars + slam_scalars:
                val = self._yaml_get_nested(data, ypath)
                if val is not None and ctrl_key in self.param_controls:
                    self.set_control_value(ctrl_key, val, apply_to_robot=apply_to_robot)
                    loaded_count += 1

            # Fallback for older profiles that used obstacle_layer in local_costmap
            for scan_p in ('obstacle_max_range', 'raytrace_max_range'):
                ctrl_key = ('local_costmap/local_costmap', f'voxel_layer.scan.{scan_p}')
                if self._yaml_get_nested(data, ('local_costmap', 'local_costmap', 'ros__parameters', 'voxel_layer', 'scan', scan_p)) is None:
                    legacy_val = self._yaml_get_nested(data, ('local_costmap', 'local_costmap', 'ros__parameters', 'obstacle_layer', 'scan', scan_p))
                    if legacy_val is not None:
                        self.set_control_value(ctrl_key, legacy_val, apply_to_robot=apply_to_robot)
                        loaded_count += 1

            for arr_key, ypath in nav2_arrays:
                arr_val = self._yaml_get_nested(data, ypath)
                if isinstance(arr_val, (list, tuple)) and arr_key in self.array_param_groups:
                    for idx, comp_key in enumerate(self.array_param_groups[arr_key]):
                        if idx < len(arr_val):
                            self.set_control_value(comp_key, arr_val[idx], apply_to_robot=apply_to_robot)
                            loaded_count += 1

            base_name = os.path.basename(file_path)
            self.append_log(f"Đã nạp {loaded_count} tham số từ file: {base_name}", "SUCCESS")
            if not silent:
                QMessageBox.information(self, "Nạp thành công", f"Đã nạp thành công {loaded_count} tham số từ:\n{base_name}")
        except Exception as e:
            self.append_log(f"Lỗi khi đọc file YAML: {str(e)}", "ERROR")
            if not silent:
                QMessageBox.critical(self, "Lỗi đọc file", f"Không thể đọc file YAML:\n{str(e)}")

    def get_control_value(self, key):
        info = self.param_controls[key]
        kind = info['kind']
        if kind == 'checkbox':
            return bool(info['widget'].isChecked())
        if kind == 'combo':
            return str(info['widget'].currentText())
        val = info['slider'].value() / info['scale']
        return int(round(val)) if info['type'] == 'int' else float(val)

    # ==========================================================================
    # LOG & TRẠNG THÁI
    # ==========================================================================

    def poll_liveness(self):
        targets = ['amcl', 'controller_server', 'local_costmap', 'velocity_smoother', 'slam_toolbox']
        self.ros_worker.check_nodes_liveness(targets)

    def update_node_status(self, status):
        self.set_badge_state(self.badge_amcl, "AMCL", status.get('amcl', False))
        self.set_badge_state(self.badge_controller, "Controller", status.get('controller_server', False))
        self.set_badge_state(self.badge_costmap, "Costmap", status.get('local_costmap', False))
        self.set_badge_state(self.badge_smoother, "Smoother", status.get('velocity_smoother', False))
        self.set_badge_state(self.badge_slam, "SLAM", status.get('slam_toolbox', False))

    def append_log(self, msg, level="INFO"):
        now = time.strftime("%H:%M:%S")
        color = "#2563EB"
        if level == "SUCCESS":
            color = "#16A34A"
        elif level == "WARN":
            color = "#D97706"
        elif level == "ERROR":
            color = "#DC2626"

        html = f"<span style='color: #94A3B8;'>[{now}]</span> <b style='color: {color};'>[{level}]</b> {msg}"
        self.txt_log.append(html)

    def closeEvent(self, event):
        self.timer_liveness.stop()
        super().closeEvent(event)


# ==============================================================================
# MAIN ENTRY POINT
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    comm = ParamCommunicator()
    ros_worker = RosParamWorker(comm)

    # Spin ROS 2 in background thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_worker,), daemon=True)
    ros_thread.start()

    gui = NavTunerGUI(ros_worker, comm)
    gui.show()

    exit_code = app.exec_()

    ros_worker.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
