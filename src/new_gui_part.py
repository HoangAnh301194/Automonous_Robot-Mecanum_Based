class RobotHmiGui(QWidget):
    log_signal = pyqtSignal(str)
    status_signal = pyqtSignal(object)
    map_signal = pyqtSignal(object)
    location_list_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Service - Advanced HMI")
        self.resize(1100, 800)
        self.setStyleSheet('''
            QWidget { background-color: #121212; color: #ffffff; font-family: 'Segoe UI', Arial, sans-serif; }
            QPushButton { border-radius: 6px; padding: 12px; font-weight: bold; background-color: #2c2c2c; border: 1px solid #444; font-size: 14px; }
            QPushButton:hover { background-color: #3a3a3a; }
            QPushButton:pressed { background-color: #1a1a1a; }
            QGroupBox { font-size: 16px; font-weight: bold; border: 1px solid #4CAF50; border-radius: 8px; margin-top: 15px; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 10px; color: #4CAF50; top: -10px; }
            QLineEdit, QComboBox { padding: 10px; border: 1px solid #555; border-radius: 4px; background-color: #1e1e1e; font-size: 14px; }
        ''')
        
        self.ros_node = None
        self.ros_thread = None
        self.battery_percent = 100
        self.patrol_state = "READY"
        self.wifi_name = "Robot_Net_5G"
        
        self.init_ui()
        self.init_ros()
        
        self.log_signal.connect(self.append_log)
        self.status_signal.connect(self.update_monitor_ui)
        self.map_signal.connect(self.update_map_ui)
        self.location_list_signal.connect(self.update_locations_ui)
        
        self.loc_timer = QTimer(self)
        self.loc_timer.timeout.connect(self.refresh_locations)
        self.loc_timer.start(2000)

    def get_ros_node(self):
        return self.ros_node

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        self.stack = QStackedWidget()
        main_layout.addWidget(self.stack, stretch=1)
        
        # Log Box at bottom
        log_group = QGroupBox("System Logs")
        log_group.setStyleSheet("QGroupBox { border: 1px solid #333; margin-top: 10px; } QGroupBox::title { color: #888; }")
        l_lay = QVBoxLayout(log_group)
        l_lay.setContentsMargins(5, 5, 5, 5)
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(100)
        self.txt_log.setStyleSheet("background-color: #000; color: #0f0; font-family: monospace; font-size: 12px; border: none;")
        l_lay.addWidget(self.txt_log)
        main_layout.addWidget(log_group)

        self.stack.addWidget(self.build_setup_wizard())   # 0
        self.stack.addWidget(self.build_main_screen())     # 1
        self.stack.addWidget(self.build_operator_menu())   # 2
        self.stack.addWidget(self.build_direct_go())       # 3
        self.stack.addWidget(self.build_map_view())        # 4
        
        self.stack.setCurrentIndex(0)

    # --- UI BUILDERS ---
    def build_setup_wizard(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; }")
        
        content = QWidget()
        c_layout = QVBoxLayout(content)
        
        lbl_title = QLabel("Initial Setup Wizard")
        lbl_title.setStyleSheet("font-size: 24px; font-weight: bold; color: #4CAF50; margin-bottom: 10px;")
        lbl_title.setAlignment(Qt.AlignCenter)
        c_layout.addWidget(lbl_title)
        
        # 1. Lang & Battery
        g1 = QGroupBox("1. Language & Battery Check")
        l1 = QVBoxLayout(g1)
        h1 = QHBoxLayout()
        h1.addWidget(QLabel("Language:"))
        self.cmb_lang = QComboBox()
        self.cmb_lang.addItems(["en", "vi"])
        h1.addWidget(self.cmb_lang)
        l1.addLayout(h1)
        
        self.lbl_setup_battery = QLabel("Battery: Checking...")
        self.lbl_setup_battery.setStyleSheet("color: yellow; font-weight: bold; font-size: 16px;")
        l1.addWidget(self.lbl_setup_battery)
        c_layout.addWidget(g1)
        
        # 2. Network
        g2 = QGroupBox("2. Network Configuration")
        l2 = QHBoxLayout(g2)
        l2.addWidget(QLabel("SSID:"))
        cmb_wifi = QComboBox()
        cmb_wifi.addItems(["Robot_Net_5G", "Guest_Wifi", "Lab_Network"])
        l2.addWidget(cmb_wifi)
        l2.addWidget(QLabel("Pass:"))
        l2.addWidget(QLineEdit("********"))
        btn_connect = QPushButton("Connect")
        btn_connect.setStyleSheet("background-color: #2196F3;")
        l2.addWidget(btn_connect)
        c_layout.addWidget(g2)
        
        # 3. Mapping
        g3 = QGroupBox("3. Mapping")
        l3 = QHBoxLayout(g3)
        btn_slam = QPushButton("Start SLAM")
        btn_slam.clicked.connect(self.on_start_slam)
        l3.addWidget(btn_slam)
        l3.addWidget(QPushButton("Load Existing Map"))
        c_layout.addWidget(g3)
        
        # 4. Function Locations
        g4 = QGroupBox("4. Function Locations")
        l4 = QVBoxLayout(g4)
        
        h4a = QHBoxLayout()
        h4a.addWidget(QLabel("Save Current Pose As:"))
        self.cmb_loc_type = QComboBox()
        self.cmb_loc_type.addItems(["Departure", "Greeting", "Charging Pile", "Temporary Stop Point", "Table 01", "Table 02"])
        h4a.addWidget(self.cmb_loc_type)
        
        self.txt_loc_custom = QLineEdit()
        self.txt_loc_custom.setPlaceholderText("Or custom name...")
        h4a.addWidget(self.txt_loc_custom)
        
        btn_save_loc = QPushButton("Save Location")
        btn_save_loc.setStyleSheet("background-color: #FF9800;")
        btn_save_loc.clicked.connect(self.on_setup_save_location)
        h4a.addWidget(btn_save_loc)
        l4.addLayout(h4a)
        
        h4b = QHBoxLayout()
        h4b.addWidget(QLabel("Cruise Route (comma-separated):"))
        self.txt_route = QLineEdit("Departure, Table 01, Greeting")
        h4b.addWidget(self.txt_route)
        btn_save_route = QPushButton("Save Route")
        btn_save_route.clicked.connect(self.on_setup_save_route)
        h4b.addWidget(btn_save_route)
        l4.addLayout(h4b)
        c_layout.addWidget(g4)
        
        # 5. Virtual Walls & Path
        g5 = QGroupBox("5. Advanced Navigation (Virtual Walls & Paths)")
        l5 = QVBoxLayout(g5)
        l5.addWidget(QLabel("<i>Note: Virtual Walls, Restricted Areas, and Path Configs are managed via Costmap Editor Plugin (Coming soon).</i>"))
        c_layout.addWidget(g5)
        
        # Finish
        self.btn_finish_setup = QPushButton("✅ FINISH SETUP & START OPERATION")
        self.btn_finish_setup.setStyleSheet("background-color: #4CAF50; padding: 20px; font-size: 18px;")
        self.btn_finish_setup.clicked.connect(self.on_finish_setup)
        c_layout.addWidget(self.btn_finish_setup)
        
        c_layout.addStretch()
        scroll.setWidget(content)
        layout.addWidget(scroll)
        return widget

    def build_main_screen(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Header
        header = QHBoxLayout()
        lbl_title = QLabel("ROBOT SERVICE")
        lbl_title.setStyleSheet("font-size: 24px; font-weight: bold; color: #4CAF50;")
        self.lbl_main_top_status = QLabel("100% 🔋  |  WiFi: Robot_Net_5G 📶")
        self.lbl_main_top_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_main_top_status.setStyleSheet("font-size: 18px;")
        header.addWidget(lbl_title)
        header.addWidget(self.lbl_main_top_status)
        layout.addLayout(header)
        
        layout.addSpacing(40)
        
        # Center Status
        self.lbl_main_center_status = QLabel("Status: READY")
        self.lbl_main_center_status.setAlignment(Qt.AlignCenter)
        self.lbl_main_center_status.setStyleSheet("font-size: 28px; font-weight: bold; color: #2196F3;")
        layout.addWidget(self.lbl_main_center_status)
        
        layout.addSpacing(20)
        
        # Giant Button
        self.btn_main_patrol = QPushButton("START PATROL")
        self.btn_main_patrol.setStyleSheet("background-color: #4CAF50; color: white; font-size: 36px; font-weight: bold; border-radius: 20px; min-height: 120px;")
        self.btn_main_patrol.clicked.connect(self.on_main_patrol_clicked)
        layout.addWidget(self.btn_main_patrol)
        
        layout.addSpacing(20)
        
        # Details
        self.lbl_main_details = QLabel("Current Point: Home\nLocalization: OK\nWater System: Ready\nRoute: -")
        self.lbl_main_details.setAlignment(Qt.AlignCenter)
        self.lbl_main_details.setStyleSheet("font-size: 18px; color: #aaa; line-height: 1.5;")
        layout.addWidget(self.lbl_main_details)
        
        layout.addStretch()
        
        # Footer Menu
        self.btn_menu = QPushButton("☰ Menu")
        self.btn_menu.setStyleSheet("background-color: #333; font-size: 20px; padding: 15px;")
        self.btn_menu.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        layout.addWidget(self.btn_menu)
        
        return widget

    def build_operator_menu(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        lbl = QLabel("ROBOT MENU")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size: 28px; font-weight: bold; margin-bottom: 20px;")
        layout.addWidget(lbl)
        
        grid = QGridLayout()
        grid.setSpacing(15)
        
        btn_style = "QPushButton { font-size: 20px; padding: 30px; background-color: #222; border: 2px solid #555; } QPushButton:hover { background-color: #333; border: 2px solid #4CAF50; }"
        
        btn_dg = QPushButton("🎯 Direct Go")
        btn_dg.setStyleSheet(btn_style); btn_dg.clicked.connect(lambda: self.stack.setCurrentIndex(3))
        
        btn_home = QPushButton("🏠 Return Home")
        btn_home.setStyleSheet(btn_style); btn_home.clicked.connect(self.on_menu_return_home)
        
        btn_srv = QPushButton("👤 Manual Service")
        btn_srv.setStyleSheet(btn_style); btn_srv.clicked.connect(self.on_menu_serve)
        
        btn_chg = QPushButton("🔋 Go Charging")
        btn_chg.setStyleSheet(btn_style); btn_chg.clicked.connect(self.on_menu_charge)
        
        btn_wat = QPushButton("☕ Water System")
        btn_wat.setStyleSheet(btn_style)
        
        btn_map = QPushButton("🗺 Map")
        btn_map.setStyleSheet(btn_style); btn_map.clicked.connect(lambda: self.stack.setCurrentIndex(4))
        
        btn_diag = QPushButton("🛠 Diagnostics")
        btn_diag.setStyleSheet(btn_style)
        
        btn_set = QPushButton("⚙ Settings")
        btn_set.setStyleSheet(btn_style); btn_set.clicked.connect(lambda: self.stack.setCurrentIndex(0))
        
        grid.addWidget(btn_dg, 0, 0)
        grid.addWidget(btn_home, 0, 1)
        grid.addWidget(btn_srv, 1, 0)
        grid.addWidget(btn_chg, 1, 1)
        grid.addWidget(btn_wat, 2, 0)
        grid.addWidget(btn_diag, 2, 1)
        grid.addWidget(btn_map, 3, 0)
        grid.addWidget(btn_set, 3, 1)
        
        layout.addLayout(grid)
        layout.addStretch()
        
        btn_back = QPushButton("🔙 Back to Main Screen")
        btn_back.setStyleSheet("background-color: #555; padding: 15px; font-size: 18px;")
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(1))
        layout.addWidget(btn_back)
        
        return widget

    def build_direct_go(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        lbl = QLabel("🎯 DIRECT GO")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(lbl)
        
        h = QHBoxLayout()
        h.addWidget(QLabel("Select Destination:"))
        self.cmb_direct_dest = QComboBox()
        self.cmb_direct_dest.addItems(["Đang tải..."])
        h.addWidget(self.cmb_direct_dest)
        
        btn_go = QPushButton("🚀 GO!")
        btn_go.setStyleSheet("background-color: #E91E63; font-size: 18px; padding: 10px 30px;")
        btn_go.clicked.connect(self.on_direct_go_exec)
        h.addWidget(btn_go)
        layout.addLayout(h)
        
        self.lbl_map_direct = MapViewer(ros_node_getter=self.get_ros_node)
        layout.addWidget(self.lbl_map_direct, stretch=1)
        
        btn_back = QPushButton("🔙 Back to Menu")
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        layout.addWidget(btn_back)
        
        return widget

    def build_map_view(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        lbl = QLabel("🗺 GLOBAL MAP VIEW")
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("font-size: 24px; font-weight: bold;")
        layout.addWidget(lbl)
        
        self.lbl_map_full = MapViewer(ros_node_getter=self.get_ros_node)
        layout.addWidget(self.lbl_map_full, stretch=1)
        
        btn_back = QPushButton("🔙 Back to Menu")
        btn_back.clicked.connect(lambda: self.stack.setCurrentIndex(2))
        layout.addWidget(btn_back)
        return widget

    # --- LOGIC & CALLBACKS ---
    def append_log(self, text):
        self.txt_log.append(text)
        self.txt_log.verticalScrollBar().setValue(self.txt_log.verticalScrollBar().maximum())

    def update_monitor_ui(self, status_msg):
        self.battery_percent = int(status_msg.battery_percent)
        bat_str = f"Battery: {self.battery_percent}% "
        if self.battery_percent < 35:
            self.lbl_setup_battery.setText(bat_str + "(LOW - Please Charge!)")
            self.lbl_setup_battery.setStyleSheet("color: red; font-weight: bold; font-size: 16px;")
            self.btn_finish_setup.setEnabled(False)
            self.btn_finish_setup.setText("❌ BATTERY TOO LOW TO START")
            self.btn_finish_setup.setStyleSheet("background-color: #555; padding: 20px; font-size: 18px; color: #888;")
        else:
            self.lbl_setup_battery.setText(bat_str + "(OK)")
            self.lbl_setup_battery.setStyleSheet("color: #4CAF50; font-weight: bold; font-size: 16px;")
            self.btn_finish_setup.setEnabled(True)
            self.btn_finish_setup.setText("✅ FINISH SETUP & START OPERATION")
            self.btn_finish_setup.setStyleSheet("background-color: #4CAF50; padding: 20px; font-size: 18px;")
            
        # Update Main Screen Header
        self.lbl_main_top_status.setText(f"{self.battery_percent}% 🔋  |  WiFi: {self.wifi_name} 📶")
        
        # Update Main Screen Details
        water = "Low!" if status_msg.water_low else "Ready"
        loc = "OK" if status_msg.odom_ok else "Error"
        route_txt = self.txt_route.text()
        self.lbl_main_details.setText(f"Current Point: Unknown\nLocalization: {loc}\nWater System: {water}\nRoute: {route_txt}")

    def update_map_ui(self, occ_grid):
        self.lbl_map_direct.update_map(occ_grid)
        self.lbl_map_full.update_map(occ_grid)

    def update_locations_ui(self, loc_list):
        current_items = [self.cmb_direct_dest.itemText(i) for i in range(self.cmb_direct_dest.count())]
        if not loc_list:
            if current_items != ["Không có điểm nào"] and current_items != ["Đang tải..."]:
                self.cmb_direct_dest.clear()
                self.cmb_direct_dest.addItem("Không có điểm nào")
            return
        if current_items != loc_list:
            current_text = self.cmb_direct_dest.currentText()
            self.cmb_direct_dest.clear()
            self.cmb_direct_dest.addItems(loc_list)
            idx = self.cmb_direct_dest.findText(current_text)
            if idx >= 0:
                self.cmb_direct_dest.setCurrentIndex(idx)

    def refresh_locations(self):
        if self.ros_node:
            self.ros_node.get_location_list()

    def on_start_slam(self):
        if self.ros_node: self.ros_node.start_slam()

    def on_setup_save_location(self):
        if not self.ros_node: return
        name = self.txt_loc_custom.text().strip()
        if not name:
            name = self.cmb_loc_type.currentText()
        self.ros_node.save_current_location(name)

    def on_setup_save_route(self):
        if not self.ros_node: return
        route_str = self.txt_route.text().strip()
        if route_str:
            waypoints = [w.strip() for w in route_str.split(',')]
            self.ros_node.save_route("patrol_route", waypoints)

    def on_finish_setup(self):
        if self.ros_node:
            self.ros_node.set_language(self.cmb_lang.currentText())
            self.on_setup_save_route() # Auto save route
            self.ros_node.finish_setup()
            # Transition to Main Screen
            self.stack.setCurrentIndex(1)
            self.append_log("Đã chuyển sang màn hình Vận hành (Main Screen).")

    def on_main_patrol_clicked(self):
        if not self.ros_node: return
        if self.patrol_state == "READY" or self.patrol_state == "PAUSED":
            self.ros_node.start_patrol()
            self.patrol_state = "PATROLLING"
            self.btn_main_patrol.setText("PAUSE PATROL")
            self.btn_main_patrol.setStyleSheet("background-color: #FFC107; color: black; font-size: 36px; font-weight: bold; border-radius: 20px; min-height: 120px;")
            self.lbl_main_center_status.setText("Status: PATROLLING")
            self.lbl_main_center_status.setStyleSheet("font-size: 28px; font-weight: bold; color: #FFC107;")
        elif self.patrol_state == "PATROLLING":
            self.ros_node.pause_patrol()
            self.patrol_state = "PAUSED"
            self.btn_main_patrol.setText("RESUME PATROL")
            self.btn_main_patrol.setStyleSheet("background-color: #4CAF50; color: white; font-size: 36px; font-weight: bold; border-radius: 20px; min-height: 120px;")
            self.lbl_main_center_status.setText("Status: PAUSED")
            self.lbl_main_center_status.setStyleSheet("font-size: 28px; font-weight: bold; color: #4CAF50;")

    def on_menu_return_home(self):
        if self.ros_node:
            self.ros_node.direct_go("Departure")
            self.stack.setCurrentIndex(1)
            self.lbl_main_center_status.setText("Status: RETURNING HOME")
            
    def on_menu_charge(self):
        if self.ros_node:
            self.ros_node.direct_go("Charging Pile")
            self.stack.setCurrentIndex(1)
            self.lbl_main_center_status.setText("Status: GOING TO CHARGE")

    def on_menu_serve(self):
        if self.ros_node:
            self.ros_node.serve_customer()
            self.stack.setCurrentIndex(1)

    def on_direct_go_exec(self):
        if self.ros_node:
            loc = self.cmb_direct_dest.currentText()
            self.ros_node.direct_go(loc)
            self.stack.setCurrentIndex(1)
            self.lbl_main_center_status.setText(f"Status: GOING TO {loc.upper()}")

    def init_ros(self):
        try:
            rclpy.init(args=sys.argv)
            self.ros_node = HmiRosNode(
                log_callback=self.log_signal.emit, 
                gui_callback=self.status_signal.emit,
                map_callback=self.map_signal.emit,
                loc_list_callback=self.location_list_signal.emit
            )
            self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
            self.ros_thread.start()
            self.append_log("ROS 2 Node khởi tạo thành công.")
        except Exception as e:
            self.append_log(f"Lỗi khởi tạo ROS 2: {e}")

    def spin_ros(self):
        if self.ros_node:
            rclpy.spin(self.ros_node)

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
