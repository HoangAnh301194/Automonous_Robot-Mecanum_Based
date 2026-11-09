#!/usr/bin/env python3

import threading
import time
import tkinter as tk
from tkinter import font as tkfont
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty


class GuiTeleop(Node):
    def __init__(self):
        super().__init__('gui_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.uwb_mode_pub = self.create_publisher(String, '/uwb/mode_control', 10)
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        # Current velocities (ramped with acceleration)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Target velocities from user input
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Max velocity settings
        self.speed = 0.8           # Max linear speed (m/s)
        self.turn = 0.8            # Max angular speed (rad/s)
        
        # Acceleration settings
        self.accel_linear = 0.8    # Linear acceleration (m/s^2)
        self.accel_angular = 2.0   # Angular acceleration (rad/s^2)
        
        self.keys = {'w': False, 's': False, 'a': False, 'd': False}
        self.dt = 0.05
        self.last_time = time.time()
        
        # 20 Hz ROS timer for cmd_vel publishing with acceleration ramping
        self.timer = self.create_timer(self.dt, self.publish_velocity)

    def set_uwb_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.uwb_mode_pub.publish(msg)
        self.get_logger().info(f"Đã gửi lệnh chuyển chế độ UWB: {mode_str}")

    def trigger_global_localization(self):
        try:
            if self.global_loc_client.service_is_ready():
                req = Empty.Request()
                self.global_loc_client.call_async(req)
                self.get_logger().info("Đã gọi Service /reinitialize_global_localization -> Rải hạt thành công!")
                return True
            else:
                self.get_logger().warn("Service /reinitialize_global_localization chưa sẵn sàng (AMCL chưa khởi động xong)!")
                return False
        except Exception as e:
            self.get_logger().error(f"Lỗi khi gọi Global Localization: {e}")
            return False

    def publish_velocity(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0.0 or dt > 0.2:
            dt = self.dt
        self.last_time = current_time

        # 1. Determine target linear velocity
        if self.keys['w'] and not self.keys['s']:
            self.target_linear_vel = self.speed
        elif self.keys['s'] and not self.keys['w']:
            self.target_linear_vel = -self.speed
        else:
            self.target_linear_vel = 0.0
            
        # 2. Determine target angular velocity
        if self.keys['a'] and not self.keys['d']:
            self.target_angular_vel = self.turn
        elif self.keys['d'] and not self.keys['a']:
            self.target_angular_vel = -self.turn
        else:
            self.target_angular_vel = 0.0

        # 3. Apply linear acceleration ramping
        max_lin_step = self.accel_linear * dt
        if self.linear_vel < self.target_linear_vel:
            self.linear_vel = min(self.linear_vel + max_lin_step, self.target_linear_vel)
        elif self.linear_vel > self.target_linear_vel:
            self.linear_vel = max(self.linear_vel - max_lin_step, self.target_linear_vel)

        # 4. Apply angular acceleration ramping
        max_ang_step = self.accel_angular * dt
        if self.angular_vel < self.target_angular_vel:
            self.angular_vel = min(self.angular_vel + max_ang_step, self.target_angular_vel)
        elif self.angular_vel > self.target_angular_vel:
            self.angular_vel = max(self.angular_vel - max_ang_step, self.target_angular_vel)

        # Deadband / snap to zero if very close
        if abs(self.linear_vel) < 1e-4:
            self.linear_vel = 0.0
        if abs(self.angular_vel) < 1e-4:
            self.angular_vel = 0.0

        # 5. Publish Twist msg
        msg = Twist()
        msg.linear.x = float(self.linear_vel)
        msg.angular.z = float(self.angular_vel)
        self.publisher_.publish(msg)


class App:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("Robot Teleop Control (With Acceleration)")
        self.root.geometry("420x720")
        self.root.configure(bg="#f0f2f5")
        
        # Make the window always on top if supported
        try:
            self.root.attributes('-topmost', True)
        except Exception:
            pass

        # Header Title
        title_frame = tk.Frame(root, bg="#1e293b", pady=10)
        title_frame.pack(fill="x")
        title_lbl = tk.Label(
            title_frame, text="BỘ ĐIỀU KHIỂN ROBOT (TELEOP + GIA TỐC)",
            font=("Helvetica", 11, "bold"), fg="#ffffff", bg="#1e293b"
        )
        title_lbl.pack()

        # AMCL Global Localization Frame
        amcl_frame = tk.LabelFrame(
            root, text=" AMCL Particle Control ",
            font=("Helvetica", 9, "bold"), bg="#ffffff", fg="#334155", padx=10, pady=6
        )
        amcl_frame.pack(fill="x", padx=15, pady=(8, 4))

        self.btn_global_loc = tk.Button(
            amcl_frame, text="[+] RẢI HẠT TOÀN BẢN ĐỒ (Global Localization)",
            font=("Helvetica", 8, "bold"), bg="#eff6ff", fg="#1d4ed8", activebackground="#bfdbfe",
            relief="solid", bd=1, height=2, cursor="hand2",
            command=self.on_global_localization
        )
        self.btn_global_loc.pack(fill="x")

        # UWB Mode Debug Control Frame
        uwb_frame = tk.LabelFrame(
            root, text=" Chế độ UWB (Debug NLOS / LOS) ",
            font=("Helvetica", 9, "bold"), bg="#ffffff", fg="#334155", padx=10, pady=6
        )
        uwb_frame.pack(fill="x", padx=15, pady=(4, 4))

        uwb_btn_frame = tk.Frame(uwb_frame, bg="#ffffff")
        uwb_btn_frame.pack(fill="x")

        self.btn_mode_auto = tk.Button(
            uwb_btn_frame, text="[Auto]\nTự động", font=("Helvetica", 8, "bold"),
            bg="#dcfce7", fg="#166534", relief="solid", bd=1, height=2,
            command=lambda: self.on_select_uwb_mode("auto")
        )
        self.btn_mode_auto.pack(side="left", padx=2, expand=True, fill="x")

        self.btn_mode_nlos = tk.Button(
            uwb_btn_frame, text="[NLOS]\nÉp NLOS", font=("Helvetica", 8, "bold"),
            bg="#f1f5f9", fg="#475569", relief="solid", bd=1, height=2,
            command=lambda: self.on_select_uwb_mode("force_nlos")
        )
        self.btn_mode_nlos.pack(side="left", padx=2, expand=True, fill="x")

        self.btn_mode_los = tk.Button(
            uwb_btn_frame, text="[LOS]\nÉp LOS", font=("Helvetica", 8, "bold"),
            bg="#f1f5f9", fg="#475569", relief="solid", bd=1, height=2,
            command=lambda: self.on_select_uwb_mode("force_los")
        )
        self.btn_mode_los.pack(side="left", padx=2, expand=True, fill="x")

        self.uwb_status_lbl = tk.Label(
            uwb_frame, text="Trạng thái: Tự động (3D Raycasting vật cản)",
            font=("Helvetica", 8, "bold"), bg="#ffffff", fg="#16a34a"
        )
        self.uwb_status_lbl.pack(pady=(4, 2))

        # Speed & Acceleration Settings Frame
        settings_frame = tk.LabelFrame(
            root, text=" Cài đặt Tốc độ & Gia tốc ",
            font=("Helvetica", 9, "bold"), bg="#ffffff", fg="#334155", padx=10, pady=6
        )
        settings_frame.pack(fill="x", padx=15, pady=4)

        # 1. Linear speed (m/s)
        tk.Label(settings_frame, text="Tốc độ thẳng (m/s):", font=("Helvetica", 8, "bold"), bg="#ffffff").grid(row=0, column=0, sticky="w", pady=2)
        self.linear_var = tk.DoubleVar(value=self.node.speed)
        self.linear_slider = tk.Scale(
            settings_frame, from_=0.1, to=3.0, resolution=0.05,
            orient="horizontal", variable=self.linear_var,
            command=self.on_params_change, bg="#ffffff", highlightthickness=0, length=160
        )
        self.linear_slider.grid(row=0, column=1, padx=5)
        self.linear_val_lbl = tk.Label(settings_frame, text=f"{self.node.speed:.2f}", font=("Helvetica", 8, "bold"), fg="#0284c7", bg="#ffffff", width=5)
        self.linear_val_lbl.grid(row=0, column=2)

        # 2. Angular speed (rad/s)
        tk.Label(settings_frame, text="Tốc độ quay (rad/s):", font=("Helvetica", 8, "bold"), bg="#ffffff").grid(row=1, column=0, sticky="w", pady=2)
        self.angular_var = tk.DoubleVar(value=self.node.turn)
        self.angular_slider = tk.Scale(
            settings_frame, from_=0.1, to=4.0, resolution=0.1,
            orient="horizontal", variable=self.angular_var,
            command=self.on_params_change, bg="#ffffff", highlightthickness=0, length=160
        )
        self.angular_slider.grid(row=1, column=1, padx=5)
        self.angular_val_lbl = tk.Label(settings_frame, text=f"{self.node.turn:.2f}", font=("Helvetica", 8, "bold"), fg="#0284c7", bg="#ffffff", width=5)
        self.angular_val_lbl.grid(row=1, column=2)

        # 3. Linear acceleration (m/s^2)
        tk.Label(settings_frame, text="Gia tốc thẳng (m/s²):", font=("Helvetica", 8, "bold"), bg="#ffffff", fg="#059669").grid(row=2, column=0, sticky="w", pady=2)
        self.accel_lin_var = tk.DoubleVar(value=self.node.accel_linear)
        self.accel_lin_slider = tk.Scale(
            settings_frame, from_=0.1, to=4.0, resolution=0.05,
            orient="horizontal", variable=self.accel_lin_var,
            command=self.on_params_change, bg="#ffffff", highlightthickness=0, length=160
        )
        self.accel_lin_slider.grid(row=2, column=1, padx=5)
        self.accel_lin_val_lbl = tk.Label(settings_frame, text=f"{self.node.accel_linear:.2f}", font=("Helvetica", 8, "bold"), fg="#059669", bg="#ffffff", width=5)
        self.accel_lin_val_lbl.grid(row=2, column=2)

        # 4. Angular acceleration (rad/s^2)
        tk.Label(settings_frame, text="Gia tốc quay (rad/s²):", font=("Helvetica", 8, "bold"), bg="#ffffff", fg="#059669").grid(row=3, column=0, sticky="w", pady=2)
        self.accel_ang_var = tk.DoubleVar(value=self.node.accel_angular)
        self.accel_ang_slider = tk.Scale(
            settings_frame, from_=0.2, to=6.0, resolution=0.1,
            orient="horizontal", variable=self.accel_ang_var,
            command=self.on_params_change, bg="#ffffff", highlightthickness=0, length=160
        )
        self.accel_ang_slider.grid(row=3, column=1, padx=5)
        self.accel_ang_val_lbl = tk.Label(settings_frame, text=f"{self.node.accel_angular:.2f}", font=("Helvetica", 8, "bold"), fg="#059669", bg="#ffffff", width=5)
        self.accel_ang_val_lbl.grid(row=3, column=2)

        # Quick Presets
        preset_frame = tk.Frame(settings_frame, bg="#ffffff")
        preset_frame.grid(row=4, column=0, columnspan=3, pady=(6, 0))
        tk.Label(preset_frame, text="Cấu hình nhanh:", font=("Helvetica", 8), bg="#ffffff", fg="#64748b").pack(side="left", padx=4)
        tk.Button(preset_frame, text="Mượt mà", font=("Helvetica", 8), command=lambda: self.set_preset(0.3, 0.8, 0.4, 1.0), bg="#e2e8f0").pack(side="left", padx=2)
        tk.Button(preset_frame, text="Chuẩn", font=("Helvetica", 8), command=lambda: self.set_preset(0.8, 0.8, 0.8, 2.0), bg="#e2e8f0").pack(side="left", padx=2)
        tk.Button(preset_frame, text="Thể thao", font=("Helvetica", 8), command=lambda: self.set_preset(1.5, 2.0, 1.5, 3.5), bg="#e2e8f0").pack(side="left", padx=2)

        # D-pad Virtual Buttons
        dpad_frame = tk.LabelFrame(
            root, text=" Điều khiển hướng (W/A/S/D) ",
            font=("Helvetica", 9, "bold"), bg="#ffffff", fg="#334155", padx=10, pady=6
        )
        dpad_frame.pack(fill="x", padx=15, pady=4)

        btn_grid = tk.Frame(dpad_frame, bg="#ffffff")
        btn_grid.pack(pady=2)

        btn_style = {"width": 8, "height": 2, "font": ("Helvetica", 8, "bold"), "relief": "raised", "cursor": "hand2"}

        self.btn_fwd = tk.Button(btn_grid, text="▲ Tiến\n(W)", bg="#dbeafe", activebackground="#93c5fd", **btn_style)
        self.btn_fwd.grid(row=0, column=1, padx=4, pady=2)
        self.btn_fwd.bind('<ButtonPress-1>', lambda e: self.set_key_state('w', True))
        self.btn_fwd.bind('<ButtonRelease-1>', lambda e: self.set_key_state('w', False))

        self.btn_left = tk.Button(btn_grid, text="◄ Trái\n(A)", bg="#dbeafe", activebackground="#93c5fd", **btn_style)
        self.btn_left.grid(row=1, column=0, padx=4, pady=2)
        self.btn_left.bind('<ButtonPress-1>', lambda e: self.set_key_state('a', True))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self.set_key_state('a', False))

        self.btn_stop = tk.Button(btn_grid, text="■ PHANH\n(Space)", bg="#fee2e2", fg="#dc2626", activebackground="#fca5a5", **btn_style, command=self.emergency_stop)
        self.btn_stop.grid(row=1, column=1, padx=4, pady=2)

        self.btn_right = tk.Button(btn_grid, text="Phải ►\n(D)", bg="#dbeafe", activebackground="#93c5fd", **btn_style)
        self.btn_right.grid(row=1, column=2, padx=4, pady=2)
        self.btn_right.bind('<ButtonPress-1>', lambda e: self.set_key_state('d', True))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self.set_key_state('d', False))

        self.btn_back = tk.Button(btn_grid, text="▼ Lùi\n(S)", bg="#dbeafe", activebackground="#93c5fd", **btn_style)
        self.btn_back.grid(row=2, column=1, padx=4, pady=2)
        self.btn_back.bind('<ButtonPress-1>', lambda e: self.set_key_state('s', True))
        self.btn_back.bind('<ButtonRelease-1>', lambda e: self.set_key_state('s', False))

        # Status output
        self.status_lbl = tk.Label(
            root, text="Trạng thái: Đứng yên\nv = 0.00 m/s | w = 0.00 rad/s",
            font=("Courier", 8, "bold"), bg="#f1f5f9", fg="#0f172a", relief="groove", pady=6
        )
        self.status_lbl.pack(fill="x", padx=15, pady=(4, 8))

        # Key Bindings
        self.root.bind('<KeyPress>', self.key_press)
        self.root.bind('<KeyRelease>', self.key_release)
        
        # Focus window on start
        self.root.focus_set()
        self.update_ui_loop()

    def set_key_state(self, key, state):
        if key in self.node.keys:
            self.node.keys[key] = state

    def on_params_change(self, val=None):
        self.node.speed = float(self.linear_var.get())
        self.node.turn = float(self.angular_var.get())
        self.node.accel_linear = float(self.accel_lin_var.get())
        self.node.accel_angular = float(self.accel_ang_var.get())
        
        self.linear_val_lbl.config(text=f"{self.node.speed:.2f}")
        self.angular_val_lbl.config(text=f"{self.node.turn:.2f}")
        self.accel_lin_val_lbl.config(text=f"{self.node.accel_linear:.2f}")
        self.accel_ang_val_lbl.config(text=f"{self.node.accel_angular:.2f}")

    def on_global_localization(self):
        success = self.node.trigger_global_localization()
        if success:
            self.btn_global_loc.config(bg="#dcfce7", fg="#166534", text="[OK] ĐÃ GỬI LỆNH RẢI HẠT TOÀN BẢN ĐỒ!")
            self.root.after(1500, lambda: self.btn_global_loc.config(
                bg="#eff6ff", fg="#1d4ed8", text="[+] RẢI HẠT TOÀN BẢN ĐỒ (Global Localization)"
            ))
        else:
            self.btn_global_loc.config(bg="#fee2e2", fg="#b91c1c", text="[!] CHƯA KẾT NỐI ĐƯỢC AMCL!")
            self.root.after(1500, lambda: self.btn_global_loc.config(
                bg="#eff6ff", fg="#1d4ed8", text="[+] RẢI HẠT TOÀN BẢN ĐỒ (Global Localization)"
            ))

    def on_select_uwb_mode(self, mode_str):
        self.node.set_uwb_mode(mode_str)
        
        # Reset colors
        self.btn_mode_auto.config(bg="#f1f5f9", fg="#475569")
        self.btn_mode_nlos.config(bg="#f1f5f9", fg="#475569")
        self.btn_mode_los.config(bg="#f1f5f9", fg="#475569")

        if mode_str == "auto":
            self.btn_mode_auto.config(bg="#dcfce7", fg="#166534")
            self.uwb_status_lbl.config(text="Trạng thái: Tự động (3D Raycasting vật cản)", fg="#16a34a")
        elif mode_str == "force_nlos":
            self.btn_mode_nlos.config(bg="#fee2e2", fg="#b91c1c")
            self.uwb_status_lbl.config(text="Trạng thái: ÉP BUỘC NLOS (Tất cả Anchor bị suy hao & nhiễu lớn)", fg="#dc2626")
        elif mode_str == "force_los":
            self.btn_mode_los.config(bg="#dbeafe", fg="#1d4ed8")
            self.uwb_status_lbl.config(text="Trạng thái: ÉP BUỘC LOS (Tất cả Anchor trực thông sạch)", fg="#2563eb")

    def set_preset(self, lin, ang, a_lin, a_ang):
        self.linear_var.set(lin)
        self.angular_var.set(ang)
        self.accel_lin_var.set(a_lin)
        self.accel_ang_var.set(a_ang)
        self.on_params_change()

    def emergency_stop(self):
        for k in self.node.keys:
            self.node.keys[k] = False
        self.node.target_linear_vel = 0.0
        self.node.target_angular_vel = 0.0
        self.node.linear_vel = 0.0
        self.node.angular_vel = 0.0

    def key_press(self, event):
        key = event.keysym.lower()
        if key == 'space':
            self.emergency_stop()
        elif key in ['up']:
            self.node.keys['w'] = True
        elif key in ['down']:
            self.node.keys['s'] = True
        elif key in ['left']:
            self.node.keys['a'] = True
        elif key in ['right']:
            self.node.keys['d'] = True
        elif key in self.node.keys:
            self.node.keys[key] = True

    def key_release(self, event):
        key = event.keysym.lower()
        if key in ['up']:
            self.node.keys['w'] = False
        elif key in ['down']:
            self.node.keys['s'] = False
        elif key in ['left']:
            self.node.keys['a'] = False
        elif key in ['right']:
            self.node.keys['d'] = False
        elif key in self.node.keys:
            self.node.keys[key] = False

    def update_ui_loop(self):
        lin = self.node.linear_vel
        ang = self.node.angular_vel
        t_lin = self.node.target_linear_vel
        t_ang = self.node.target_angular_vel
        
        if abs(lin) > 0.001 or abs(ang) > 0.001 or abs(t_lin) > 0.001 or abs(t_ang) > 0.001:
            self.status_lbl.config(
                text=f"Thực tế: v = {lin:+.2f} m/s | w = {ang:+.2f} rad/s\nMục tiêu: v = {t_lin:+.2f} m/s | w = {t_ang:+.2f} rad/s\n(Gia tốc: a = {self.node.accel_linear:.2f} m/s², α = {self.node.accel_angular:.2f} rad/s²)",
                fg="#16a34a"
            )
        else:
            self.status_lbl.config(
                text=f"Trạng thái: Đứng yên\nv = 0.00 m/s | w = 0.00 rad/s\n(Gia tốc: a = {self.node.accel_linear:.2f} m/s², α = {self.node.accel_angular:.2f} rad/s²)",
                fg="#64748b"
            )
            
        self.root.after(50, self.update_ui_loop)


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GuiTeleop()
    
    root = tk.Tk()
    app = App(root, node)
    
    # Start ROS spin in a background daemon thread
    ros_t = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_t.start()
    
    def on_closing():
        root.destroy()
        node.destroy_node()
        rclpy.shutdown()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
