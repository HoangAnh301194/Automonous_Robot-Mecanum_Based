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

        self.setWindowTitle("ROS 2 Nav & SLAM Realtime Tuner (Modern White Edition)")
        self.resize(980, 820)
        self.setStyleSheet(CLEAN_WHITE_QSS)

        # Paths to config files
        self.ws_dir = os.path.expanduser('~/ros2_ws')
        self.nav2_yaml_path = os.path.join(self.ws_dir, 'src/mo_hinh/config/nav2_params.yaml')
        self.slam_yaml_path = os.path.join(self.ws_dir, 'src/mo_hinh/config/mapper_params_online_async.yaml')

        # Dictionary of registered controls: key = (node, param_name) -> dict(slider, lbl, type, scale, ...)
        self.param_controls = {}
        self.debounce_timers = {}

        self.comm.log_signal.connect(self.append_log)
        self.comm.node_status_signal.connect(self.update_node_status)
        self.comm.sync_value_signal.connect(self.handle_synced_value)

        self.init_ui()

        # Heartbeat timer to monitor running nodes
        self.timer_liveness = QTimer(self)
        self.timer_liveness.timeout.connect(self.poll_liveness)
        self.timer_liveness.start(2000)

        # Initial Sync
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
        lbl_subtitle = QLabel("Chỉnh sửa tham số Realtime cho AMCL, Costmap, Controller & SLAM")
        lbl_subtitle.setStyleSheet("font-size: 11px; color: #64748B;")
        title_layout.addWidget(lbl_app_title)
        title_layout.addWidget(lbl_subtitle)
        header_layout.addLayout(title_layout)

        header_layout.addStretch()

        # Node Status Badges
        self.badge_amcl = self.create_badge("AMCL")
        self.badge_controller = self.create_badge("Controller")
        self.badge_costmap = self.create_badge("Costmap")
        self.badge_slam = self.create_badge("SLAM")

        badges_layout = QHBoxLayout()
        badges_layout.setSpacing(6)
        badges_layout.addWidget(self.badge_amcl)
        badges_layout.addWidget(self.badge_controller)
        badges_layout.addWidget(self.badge_costmap)
        badges_layout.addWidget(self.badge_slam)
        header_layout.addLayout(badges_layout)

        header_layout.addSpacing(16)

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
        self.add_param_slider(g1, 2, node, "update_min_d", "Ngưỡng di chuyển d (m):", 0.02, 0.8, 0.1, scale=100, val_type='double')
        self.add_param_slider(g1, 3, node, "update_min_a", "Ngưỡng góc xoay a (rad):", 0.02, 1.0, 0.2, scale=100, val_type='double')
        layout.addWidget(grp_particles)

        # Grp 2: Odometry Alpha Noise
        grp_alpha = QGroupBox("2. Mô hình sai số bánh xe (Odometry Noise Alpha 1 - 4)")
        g2 = QGridLayout(grp_alpha)
        self.add_param_slider(g2, 0, node, "alpha1", "Alpha 1 (Xoay khi đang xoay):", 0.0, 1.0, 0.2, scale=100, val_type='double')
        self.add_param_slider(g2, 1, node, "alpha2", "Alpha 2 (Xoay khi chạy thẳng):", 0.0, 1.0, 0.1, scale=100, val_type='double')
        self.add_param_slider(g2, 2, node, "alpha3", "Alpha 3 (Trượt khi chạy thẳng):", 0.0, 1.0, 0.2, scale=100, val_type='double')
        self.add_param_slider(g2, 3, node, "alpha4", "Alpha 4 (Trượt khi đang xoay):", 0.0, 1.0, 0.1, scale=100, val_type='double')
        layout.addWidget(grp_alpha)

        # Grp 3: Laser Likelihood Sensor
        grp_sensor = QGroupBox("3. Mô hình Cảm biến Laser (Laser Sensor Model)")
        g3 = QGridLayout(grp_sensor)
        self.add_param_slider(g3, 0, node, "laser_max_range", "Tầm quét Laser tối đa (m):", 2.0, 25.0, 12.0, scale=10, val_type='double')
        self.add_param_slider(g3, 1, node, "z_hit", "Xác suất quét trúng tường z_hit:", 0.1, 1.0, 0.85, scale=100, val_type='double')
        self.add_param_slider(g3, 2, node, "z_rand", "Xác suất nhiễu ngẫu nhiên z_rand:", 0.0, 0.8, 0.15, scale=100, val_type='double')
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

        # Grp 1: Local Costmap Inflation
        grp_local = QGroupBox("1. Bản đồ né cản cục bộ (Local Costmap - Dành cho Camera & LiDAR)")
        g1 = QGridLayout(grp_local)
        self.add_param_slider(g1, 0, local_node, "inflation_layer.inflation_radius", "Bán kính đệm an toàn (m):", 0.1, 1.5, 0.45, scale=100, val_type='double')
        self.add_param_slider(g1, 1, local_node, "inflation_layer.cost_scaling_factor", "Độ dốc né cản (Scaling):", 0.5, 15.0, 4.0, scale=10, val_type='double')
        self.add_param_slider(g1, 2, local_node, "obstacle_layer.scan.obstacle_max_range", "Tầm bắt cản tối đa (m):", 1.0, 6.0, 2.5, scale=10, val_type='double')
        self.add_param_slider(g1, 3, local_node, "obstacle_layer.scan.raytrace_max_range", "Tầm xóa cản tối đa (m):", 1.0, 6.0, 3.0, scale=10, val_type='double')
        layout.addWidget(grp_local)

        # Grp 2: Global Costmap Inflation
        grp_global = QGroupBox("2. Bản đồ quy hoạch toàn cục (Global Costmap - Vạch tuyến đường)")
        g2 = QGridLayout(grp_global)
        self.add_param_slider(g2, 0, global_node, "inflation_layer.inflation_radius", "Bán kính đệm toàn cục (m):", 0.1, 1.5, 0.55, scale=100, val_type='double')
        self.add_param_slider(g2, 1, global_node, "inflation_layer.cost_scaling_factor", "Độ dốc né cản toàn cục:", 0.5, 15.0, 3.0, scale=10, val_type='double')
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

        # Grp 1: MPPI Velocity & Pruning
        grp_vel = QGroupBox("1. Vận tốc & Ôm cua MPPI (MPPI Velocity & Pruning)")
        g1 = QGridLayout(grp_vel)
        self.add_param_slider(g1, 0, node, "FollowPath.vx_max", "Vận tốc tiến tối đa vx_max (m/s):", 0.05, 1.2, 0.4, scale=100, val_type='double')
        self.add_param_slider(g1, 1, node, "FollowPath.vx_min", "Vận tốc lùi vx_min (m/s):", -0.5, 0.0, -0.15, scale=100, val_type='double')
        self.add_param_slider(g1, 2, node, "FollowPath.wz_max", "Tốc độ xoay tối đa wz_max (rad/s):", 0.1, 2.0, 0.5, scale=100, val_type='double')
        self.add_param_slider(g1, 3, node, "FollowPath.prune_distance", "Khoảng cách tỉa đường (Prune) (m):", 0.2, 2.5, 0.6, scale=100, val_type='double')
        layout.addWidget(grp_vel)

        # Grp 2: MPPI Jetson Orin Nano Performance
        grp_perf = QGroupBox("2. Hiệu năng tính toán MPPI (Jetson Orin Nano Tuning)")
        g2 = QGridLayout(grp_perf)
        self.add_param_slider(g2, 0, node, "FollowPath.batch_size", "Số mẫu quỹ đạo (Batch size):", 300, 2500, 800, scale=1, val_type='int')
        self.add_param_slider(g2, 1, node, "FollowPath.time_steps", "Số bước dự đoán (Time steps):", 5, 30, 10, scale=1, val_type='int')
        self.add_param_slider(g2, 2, node, "FollowPath.model_dt", "Bước thời gian mô hình (Model dt):", 0.05, 0.5, 0.2, scale=100, val_type='double')
        layout.addWidget(grp_perf)

        # Grp 3: MPPI Critics Tuning
        grp_critics = QGroupBox("3. Trọng số đánh giá thuật toán MPPI (Critics Weights)")
        g3 = QGridLayout(grp_critics)
        self.add_param_slider(g3, 0, node, "FollowPath.PathAlignCritic.cost_weight", "Độ ép góc quay (PathAlign - chống lắc đầu):", 0.0, 20.0, 4.0, scale=10, val_type='double')
        self.add_param_slider(g3, 1, node, "FollowPath.GoalCritic.cost_weight", "Ưu tiên tiến tới đích (Goal):", 0.0, 20.0, 5.0, scale=10, val_type='double')
        self.add_param_slider(g3, 2, node, "FollowPath.CostCritic.cost_weight", "Né vật cản bản đồ (Cost):", 0.0, 20.0, 3.8, scale=10, val_type='double')
        self.add_param_slider(g3, 3, node, "FollowPath.PreferForwardCritic.cost_weight", "Ưu tiên chạy tiến (PreferForward):", 0.0, 20.0, 5.0, scale=10, val_type='double')
        layout.addWidget(grp_critics)

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
        grp_plan = QGroupBox("1. Bộ lập tuyến đường toàn cục (Planner Server - A* / Dijkstra)")
        g1 = QGridLayout(grp_plan)
        self.add_param_slider(g1, 0, 'planner_server', "expected_planner_frequency", "Tần số lập đường đi (Hz - giảm tải CPU):", 0.0, 20.0, 1.0, scale=10, val_type='double')
        layout.addWidget(grp_plan)

        # Grp 2: Collision Monitor
        grp_cm = QGroupBox("2. Cảm biến phanh an toàn khẩn cấp (Collision Monitor)")
        g2 = QGridLayout(grp_cm)
        self.add_param_slider(g2, 0, 'collision_monitor', "FootprintApproach.time_before_collision", "Thời gian dự báo va chạm (s):", 0.2, 3.0, 0.8, scale=10, val_type='double')
        self.add_param_slider(g2, 1, 'collision_monitor', "FootprintApproach.min_points", "Số điểm tối thiểu kích hoạt phanh (lọc nhiễu):", 2, 30, 12, scale=1, val_type='int')
        layout.addWidget(grp_cm)

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
    # TIỆN ÍCH TẠO THANH TRƯỢT VÀ LIÊN KẾT REALTIME
    # ==========================================================================

    def add_param_slider(self, grid, row, node_name, param_name, label_text, min_val, max_val, default_val, scale=1, val_type='double'):
        lbl_name = QLabel(label_text)
        lbl_name.setStyleSheet("font-weight: 500;")

        slider = QSlider(Qt.Horizontal)
        slider_min = int(round(min_val * scale))
        slider_max = int(round(max_val * scale))
        slider_def = int(round(default_val * scale))

        slider.setRange(slider_min, slider_max)
        slider.setValue(slider_def)

        lbl_val = QLabel(f"{default_val}")
        lbl_val.setStyleSheet("font-weight: 700; color: #2563EB; min-width: 55px;")
        lbl_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        grid.addWidget(lbl_name, row, 0)
        grid.addWidget(slider, row, 1)
        grid.addWidget(lbl_val, row, 2)

        key = (node_name, param_name)
        control_info = {
            'slider': slider,
            'label': lbl_val,
            'scale': scale,
            'type': val_type,
            'node': node_name,
            'param': param_name,
            'default': default_val,
        }
        self.param_controls[key] = control_info

        # Connect event
        slider.valueChanged.connect(lambda val, k=key: self.on_slider_moved(k, val))

    def on_slider_moved(self, key, slider_int_val):
        info = self.param_controls[key]
        scale = info['scale']
        val_type = info['type']

        if val_type == 'int':
            real_val = int(slider_int_val // scale)
            info['label'].setText(str(real_val))
        else:
            real_val = float(slider_int_val / scale)
            info['label'].setText(f"{real_val:.2f}")

        if not self.chk_auto_apply.isChecked():
            return

        # Debounce: tránh bắn liên tục quá nhiều request khi đang rê chuột
        if key not in self.debounce_timers:
            timer = QTimer(self)
            timer.setSingleShot(True)
            self.debounce_timers[key] = timer

        timer = self.debounce_timers[key]
        try:
            timer.timeout.disconnect()
        except Exception:
            pass

        timer.timeout.connect(lambda: self.ros_worker.set_param_async(
            info['node'], info['param'], real_val, val_type
        ))
        timer.start(80)  # Gửi sau 80ms khi dừng ngón tay

    # ==========================================================================
    # ĐỒNG BỘ VÀ LƯU TRỮ (SYNC & SAVE TO YAML)
    # ==========================================================================

    def on_sync_all(self):
        self.append_log("Đang đồng bộ tham số thực tế từ các node ROS 2...", "INFO")
        for (node_name, param_name), info in self.param_controls.items():
            self.ros_worker.query_param_async(node_name, param_name)

    def handle_synced_value(self, node_name, param_name, val):
        key = (node_name, param_name)
        if key in self.param_controls:
            info = self.param_controls[key]
            slider = info['slider']
            scale = info['scale']
            val_type = info['type']

            slider.blockSignals(True)
            if val_type == 'int':
                slider.setValue(int(round(val * scale)))
                info['label'].setText(str(val))
            else:
                slider.setValue(int(round(val * scale)))
                info['label'].setText(f"{val:.2f}")
            slider.blockSignals(False)

    def on_save_to_yaml(self):
        """Ghi đè trực tiếp các tham số vừa tinh chỉnh vào các file YAML gốc"""
        try:
            # 1. Update nav2_params.yaml
            if os.path.exists(self.nav2_yaml_path):
                with open(self.nav2_yaml_path, 'r', encoding='utf-8') as f:
                    nav2_data = yaml.safe_load(f)

                # AMCL
                if 'amcl' in nav2_data and 'ros__parameters' in nav2_data['amcl']:
                    p = nav2_data['amcl']['ros__parameters']
                    for name in ['min_particles', 'max_particles', 'update_min_d', 'update_min_a',
                                 'alpha1', 'alpha2', 'alpha3', 'alpha4', 'laser_max_range', 'z_hit', 'z_rand']:
                        key = ('amcl', name)
                        if key in self.param_controls:
                            p[name] = self.get_control_value(key)

                # Local Costmap Inflation
                if 'local_costmap' in nav2_data:
                    lc = nav2_data['local_costmap'].get('local_costmap', {}).get('ros__parameters', {})
                    if 'inflation_layer' in lc:
                        lc['inflation_layer']['inflation_radius'] = self.get_control_value(('local_costmap/local_costmap', 'inflation_layer.inflation_radius'))
                        lc['inflation_layer']['cost_scaling_factor'] = self.get_control_value(('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor'))

                # Global Costmap Inflation
                if 'global_costmap' in nav2_data:
                    gc = nav2_data['global_costmap'].get('global_costmap', {}).get('ros__parameters', {})
                    if 'inflation_layer' in gc:
                        gc['inflation_layer']['inflation_radius'] = self.get_control_value(('global_costmap/global_costmap', 'inflation_layer.inflation_radius'))
                        gc['inflation_layer']['cost_scaling_factor'] = self.get_control_value(('global_costmap/global_costmap', 'inflation_layer.cost_scaling_factor'))

                # Controller Server (MPPI)
                if 'controller_server' in nav2_data and 'ros__parameters' in nav2_data['controller_server']:
                    ctrl_p = nav2_data['controller_server']['ros__parameters']
                    if 'FollowPath' in ctrl_p:
                        fp = ctrl_p['FollowPath']
                        for p_name in ['vx_max', 'vx_min', 'wz_max', 'prune_distance', 'batch_size', 'time_steps', 'model_dt']:
                            key = ('controller_server', f'FollowPath.{p_name}')
                            if key in self.param_controls:
                                fp[p_name] = self.get_control_value(key)
                        for critic_name in ['PathAlignCritic', 'GoalCritic', 'CostCritic', 'PreferForwardCritic']:
                            key = ('controller_server', f'FollowPath.{critic_name}.cost_weight')
                            if key in self.param_controls and critic_name in fp:
                                fp[critic_name]['cost_weight'] = self.get_control_value(key)

                # Planner Server
                if 'planner_server' in nav2_data and 'ros__parameters' in nav2_data['planner_server']:
                    ps_p = nav2_data['planner_server']['ros__parameters']
                    key = ('planner_server', 'expected_planner_frequency')
                    if key in self.param_controls:
                        ps_p['expected_planner_frequency'] = self.get_control_value(key)

                # Collision Monitor
                if 'collision_monitor' in nav2_data and 'ros__parameters' in nav2_data['collision_monitor']:
                    cm_p = nav2_data['collision_monitor']['ros__parameters']
                    if 'FootprintApproach' in cm_p:
                        fa = cm_p['FootprintApproach']
                        key_t = ('collision_monitor', 'FootprintApproach.time_before_collision')
                        if key_t in self.param_controls:
                            fa['time_before_collision'] = self.get_control_value(key_t)
                        key_p = ('collision_monitor', 'FootprintApproach.min_points')
                        if key_p in self.param_controls:
                            fa['min_points'] = self.get_control_value(key_p)

                with open(self.nav2_yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(nav2_data, f, default_flow_style=False, sort_keys=False)

                self.append_log(f"Đã lưu thành công vào: {self.nav2_yaml_path}", "SUCCESS")

            # 2. Update mapper_params_online_async.yaml (SLAM)
            if os.path.exists(self.slam_yaml_path):
                with open(self.slam_yaml_path, 'r', encoding='utf-8') as f:
                    slam_data = yaml.safe_load(f)

                if 'slam_toolbox' in slam_data and 'ros__parameters' in slam_data['slam_toolbox']:
                    sp = slam_data['slam_toolbox']['ros__parameters']
                    for name in ['minimum_travel_distance', 'minimum_travel_heading',
                                 'loop_search_maximum_distance', 'link_match_minimum_response_fine']:
                        key = ('slam_toolbox', name)
                        if key in self.param_controls:
                            sp[name] = self.get_control_value(key)

                with open(self.slam_yaml_path, 'w', encoding='utf-8') as f:
                    yaml.dump(slam_data, f, default_flow_style=False, sort_keys=False)

                self.append_log(f"Đã lưu thành công vào: {self.slam_yaml_path}", "SUCCESS")

            QMessageBox.information(self, "Lưu thành công", "Đã ghi đè toàn bộ tham số vào file YAML gốc!")

        except Exception as e:
            self.append_log(f"Lỗi khi lưu YAML: {str(e)}", "ERROR")
            QMessageBox.critical(self, "Lỗi", f"Không thể lưu file YAML: {str(e)}")

    def set_control_value(self, key, val, apply_to_robot=True):
        if key in self.param_controls and val is not None:
            info = self.param_controls[key]
            slider = info['slider']
            scale = info['scale']
            val_type = info['type']

            slider.blockSignals(True)
            if val_type == 'int':
                slider.setValue(int(round(float(val) * scale)))
                info['label'].setText(str(int(val)))
            else:
                slider.setValue(int(round(float(val) * scale)))
                info['label'].setText(f"{float(val):.2f}")
            slider.blockSignals(False)

            if apply_to_robot and self.chk_auto_apply.isChecked():
                real_val = int(val) if val_type == 'int' else float(val)
                self.ros_worker.set_param_async(info['node'], info['param'], real_val, val_type)

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
        self.load_yaml_to_gui(file_path)

    def load_yaml_to_gui(self, file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            if not isinstance(data, dict):
                raise ValueError("Nội dung file YAML không hợp lệ.")

            loaded_count = 0

            # 1. AMCL
            if 'amcl' in data and 'ros__parameters' in data['amcl']:
                amcl_p = data['amcl']['ros__parameters']
                for p_name in ['min_particles', 'max_particles', 'update_min_d', 'update_min_a',
                               'alpha1', 'alpha2', 'alpha3', 'alpha4', 'laser_max_range', 'z_hit', 'z_rand']:
                    if p_name in amcl_p:
                        self.set_control_value(('amcl', p_name), amcl_p[p_name])
                        loaded_count += 1

            # 2. Local Costmap
            if 'local_costmap' in data:
                lc = data['local_costmap'].get('local_costmap', {}).get('ros__parameters', {})
                infl = lc.get('inflation_layer', {})
                if 'inflation_radius' in infl:
                    self.set_control_value(('local_costmap/local_costmap', 'inflation_layer.inflation_radius'), infl['inflation_radius'])
                    loaded_count += 1
                if 'cost_scaling_factor' in infl:
                    self.set_control_value(('local_costmap/local_costmap', 'inflation_layer.cost_scaling_factor'), infl['cost_scaling_factor'])
                    loaded_count += 1
                obst = lc.get('obstacle_layer', {}).get('scan', {})
                if 'obstacle_max_range' in obst:
                    self.set_control_value(('local_costmap/local_costmap', 'obstacle_layer.scan.obstacle_max_range'), obst['obstacle_max_range'])
                    loaded_count += 1
                if 'raytrace_max_range' in obst:
                    self.set_control_value(('local_costmap/local_costmap', 'obstacle_layer.scan.raytrace_max_range'), obst['raytrace_max_range'])
                    loaded_count += 1

            # 3. Global Costmap
            if 'global_costmap' in data:
                gc = data['global_costmap'].get('global_costmap', {}).get('ros__parameters', {})
                infl = gc.get('inflation_layer', {})
                if 'inflation_radius' in infl:
                    self.set_control_value(('global_costmap/global_costmap', 'inflation_layer.inflation_radius'), infl['inflation_radius'])
                    loaded_count += 1
                if 'cost_scaling_factor' in infl:
                    self.set_control_value(('global_costmap/global_costmap', 'inflation_layer.cost_scaling_factor'), infl['cost_scaling_factor'])
                    loaded_count += 1

            # 4. Controller (FollowPath - MPPI)
            if 'controller_server' in data:
                ctrl = data['controller_server'].get('ros__parameters', {})
                fp = ctrl.get('FollowPath', {})
                for p_name in ['vx_max', 'vx_min', 'wz_max', 'batch_size', 'prune_distance', 'time_steps', 'model_dt']:
                    if p_name in fp:
                        self.set_control_value(('controller_server', f'FollowPath.{p_name}'), fp[p_name])
                        loaded_count += 1
                for critic_name in ['PathAlignCritic', 'GoalCritic', 'CostCritic', 'PreferForwardCritic']:
                    if critic_name in fp and 'cost_weight' in fp[critic_name]:
                        self.set_control_value(('controller_server', f'FollowPath.{critic_name}.cost_weight'), fp[critic_name]['cost_weight'])
                        loaded_count += 1

            # 5. Planner Server
            if 'planner_server' in data:
                ps = data['planner_server'].get('ros__parameters', {})
                if 'expected_planner_frequency' in ps:
                    self.set_control_value(('planner_server', 'expected_planner_frequency'), ps['expected_planner_frequency'])
                    loaded_count += 1

            # 6. Collision Monitor
            if 'collision_monitor' in data:
                cm = data['collision_monitor'].get('ros__parameters', {})
                fa = cm.get('FootprintApproach', {})
                if 'time_before_collision' in fa:
                    self.set_control_value(('collision_monitor', 'FootprintApproach.time_before_collision'), fa['time_before_collision'])
                    loaded_count += 1
                if 'min_points' in fa:
                    self.set_control_value(('collision_monitor', 'FootprintApproach.min_points'), fa['min_points'])
                    loaded_count += 1

            # 7. SLAM Toolbox
            if 'slam_toolbox' in data:
                sp = data['slam_toolbox'].get('ros__parameters', {})
                for p_name in ['minimum_travel_distance', 'minimum_travel_heading',
                               'loop_search_maximum_distance', 'link_match_minimum_response_fine']:
                    if p_name in sp:
                        self.set_control_value(('slam_toolbox', p_name), sp[p_name])
                        loaded_count += 1

            base_name = os.path.basename(file_path)
            self.append_log(f"Đã nạp {loaded_count} tham số từ file: {base_name}", "SUCCESS")
            QMessageBox.information(self, "Nạp thành công", f"Đã nạp thành công {loaded_count} tham số từ:\n{base_name}")
        except Exception as e:
            self.append_log(f"Lỗi khi đọc file YAML: {str(e)}", "ERROR")
            QMessageBox.critical(self, "Lỗi đọc file", f"Không thể đọc file YAML:\n{str(e)}")

    def get_control_value(self, key):
        info = self.param_controls[key]
        val = info['slider'].value() / info['scale']
        return int(val) if info['type'] == 'int' else float(val)

    # ==========================================================================
    # LOG & TRẠNG THÁI
    # ==========================================================================

    def poll_liveness(self):
        targets = ['amcl', 'controller_server', 'local_costmap', 'slam_toolbox']
        self.ros_worker.check_nodes_liveness(targets)

    def update_node_status(self, status):
        self.set_badge_state(self.badge_amcl, "AMCL", status.get('amcl', False))
        self.set_badge_state(self.badge_controller, "Controller", status.get('controller_server', False))
        self.set_badge_state(self.badge_costmap, "Costmap", status.get('local_costmap', False))
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
