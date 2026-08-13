import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math
import re

# ============================================
# H?NG S? V?T LÝ (Ph?i kh?p v?i main.cpp trên ESP32)
# ============================================
TICK_PER_METER = 173.91     # 90 ticks/vòng ÷ 0.5175m chu vi bánh (6.5 inch)
WHEEL_BASE     = 0.58       # Kho?ng cách gi?a 2 tâm bánh xe (mét)


def yaw_to_quaternion(yaw):
    """Chuy?n góc yaw (radian) sang Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    """Chuy?n Quaternion sang góc yaw (radian)."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class HoverbotOdomNode(Node):
    """
    ROS2 Node giao ti?p v?i ESP32 Hoverbot qua USB Serial.
    
    - Subscribe /cmd_vel ? g?i l?nh V <linear> <angular> xu?ng ESP32
    - Subscribe /imu/data ? l?y hu?ng (yaw) c?a robot
    - Ð?c telemetry t? ESP32 ? tính toán 2-wheel Differential Drive Odometry d?a trên IMU Yaw
    - Publish /odom, /tf (odom ? base_footprint), /battery
    """

    def __init__(self):
        super().__init__('odom_publisher')  # Gi? nguyên tên node cu d? tránh l?i launch file

        # ============================================
        # ROS2 Parameters
        # ============================================
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('esp_port', '/dev/ttyUSB0')
        self.declare_parameter('esp_baudrate', 115200)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        self.declare_parameter('ticks_per_meter', TICK_PER_METER)
        self.declare_parameter('filter_alpha', 0.9)    # H? s? l?c Yaw c?a IMU
        self.declare_parameter('cmd_vel_rate', 10.0)   # T?n su?t g?i cmd_vel (Hz)

        self.publish_tf      = self.get_parameter('publish_tf').value
        self.esp_port        = self.get_parameter('esp_port').value
        self.esp_baudrate    = self.get_parameter('esp_baudrate').value
        self.wheel_base      = self.get_parameter('wheel_base').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.filter_alpha    = self.get_parameter('filter_alpha').value
        self.cmd_vel_rate    = self.get_parameter('cmd_vel_rate').value

        self.get_logger().info(f"=== Hoverbot Differential Drive Odom Node (With IMU) ===")
        self.get_logger().info(f"  Port: {self.esp_port} @ {self.esp_baudrate}")
        self.get_logger().info(f"  Wheel Base: {self.wheel_base} m")
        self.get_logger().info(f"  Ticks/Meter: {self.ticks_per_meter}")
        self.get_logger().info(f"  IMU Filter Alpha: {self.filter_alpha}")

        # ============================================
        # Publishers
        # ============================================
        self.odom_pub    = self.create_publisher(Odometry, '/odom', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)
        self.dataenc_pub = self.create_publisher(Int32MultiArray, '/dataenc', 10) # Cho gui_ros_debug.py hi?n th? thô

        # ============================================
        # Subscribers
        # ============================================
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ============================================
        # TF Broadcaster
        # ============================================
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ============================================
        # Serial Connection
        # ============================================
        self.ser = None
        try:
            self.ser = serial.Serial(self.esp_port, self.esp_baudrate, timeout=0.05)
            self.get_logger().info(f"Serial: K?t n?i thành công {self.esp_port}")
        except Exception as e:
            self.get_logger().error(f"Serial: Không th? m? c?ng {self.esp_port}: {e}")
            return

        # ============================================
        # Tr?ng thái Odometry
        # ============================================
        self.x  = 0.0   # V? trí X trong frame odom (mét)
        self.y  = 0.0   # V? trí Y trong frame odom (mét)
        self.th = 0.0   # Góc quay Yaw (radian)

        # IMU state
        self.imu_yaw_offset = None
        self.current_imu_wz = 0.0

        self.last_enc_L = None   # Encoder trái l?n d?c tru?c
        self.last_enc_R = None   # Encoder ph?i l?n d?c tru?c
        self.last_time  = self.get_clock().now().nanoseconds / 1e9

        # Luu l?nh cmd_vel m?i nh?t d? g?i liên t?c
        self.cmd_linear  = 0.0
        self.cmd_angular = 0.0

        # Regex d? parse chu?i telemetry t? ESP32
        self.telemetry_pattern = re.compile(
            r"V:([-\d.]+)\s+A:([-\d.]+)\s+"
            r"RC:(\d+)/(\d+)\s+"
            r"TL:([-\d]+)\s+TR:([-\d]+)\s+"
            r"EL:([-\d]+)\s+ER:([-\d]+)\s+"
            r"B:([-\d.]+)\s+T:([-\d.]+)"
        )

        # ============================================
        # Timers
        # ============================================
        # Timer d?c Serial + publish odom (20 Hz ? kh?p v?i ESP32)
        self.create_timer(0.05, self.serial_read_callback)

        # Timer g?i cmd_vel liên t?c xu?ng ESP32
        if self.cmd_vel_rate > 0:
            self.create_timer(1.0 / self.cmd_vel_rate, self.cmd_vel_send_callback)


    # ============================================
    # CALLBACK: Nh?n d? li?u góc quay t? IMU
    # ============================================
    def imu_callback(self, msg: Imu):
        q = msg.orientation
        current_yaw = quaternion_to_yaw(q)

        # Ð?t di?m b?t d?u yaw b?ng 0 t?i v? trí b?t robot
        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = current_yaw
            self.th = 0.0
            return

        # Tính góc yaw m?c tiêu d?a trên offset
        target_yaw = current_yaw - self.imu_yaw_offset

        # Chu?n hóa d? l?ch góc
        diff = target_yaw - self.th
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        # L?c thông th?p góc Yaw d? tránh nhi?u
        self.th += diff * self.filter_alpha
        
        # L?y v?n t?c góc tr?c ti?p t? IMU
        self.current_imu_wz = msg.angular_velocity.z

    # ============================================
    # CALLBACK: Nh?n l?nh /cmd_vel t? ROS2
    # ============================================
    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear  = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    # ============================================
    # TIMER: G?i l?nh cmd_vel liên t?c xu?ng ESP32
    # ============================================
    def cmd_vel_send_callback(self):
        if self.ser is None or not self.ser.is_open:
            return
        
        command = f"V {self.cmd_linear:.3f} {self.cmd_angular:.3f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    # ============================================
    # TIMER: Ð?c telemetry t? ESP32 + Tính Odometry
    # ============================================
    def serial_read_callback(self):
        try:
            if self.ser is None or not self.ser.is_open:
                return

            # Ð?c t?t c? dòng có s?n, ch? l?y dòng cu?i cùng
            latest_telemetry = None
            while self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        latest_telemetry = line
                except Exception as e:
                    pass

            if latest_telemetry is None:
                return

            # Parse chu?i telemetry b?ng regex
            match = self.telemetry_pattern.search(latest_telemetry)
            if not match:
                return

            # Trích xu?t d? li?u
            v_measured  = float(match.group(1))   # V?n t?c tâm xe (m/s)
            a_measured  = float(match.group(2))   # Gia t?c (m/s²)

            
            enc_L       = int(match.group(7))     # T?ng xung encoder trái
            enc_R       = int(match.group(8))     # T?ng xung encoder ph?i
            bat_voltage = float(match.group(9))   # Ði?n áp pin
            temperature = float(match.group(10))  # Nhi?t d? board

            # Publish raw encoder data to /dataenc
            enc_msg = Int32MultiArray()
            enc_msg.data = [enc_L, enc_R]
            self.dataenc_pub.publish(enc_msg)

            current_ros_time = self.get_clock().now()
            now_sec = current_ros_time.nanoseconds / 1e9

            # ============================================
            # TÍNH TOÁN 2 BÁNH VI SAI K?T H?P GÓC IMU YAW
            # ============================================
            if self.last_enc_L is not None and self.last_enc_R is not None:
                dt = now_sec - self.last_time
                if dt <= 0 or dt > 1.0:
                    self.last_enc_L = enc_L
                    self.last_enc_R = enc_R
                    self.last_time = now_sec
                    return

                # Delta encoder (ticks)
                delta_L = enc_L - self.last_enc_L
                delta_R = enc_R - self.last_enc_R

                # B? qua n?u delta nh?y d?t ng?t (l?i d? li?u)
                if abs(delta_L) > 5000 or abs(delta_R) > 5000:
                    self.last_enc_L = enc_L
                    self.last_enc_R = enc_R
                    self.last_time = now_sec
                    return

                # Chuy?n ticks ? mét quãng du?ng c?a t?ng bánh
                dist_L = delta_L / self.ticks_per_meter
                dist_R = delta_R / self.ticks_per_meter

                # Quãng du?ng di chuy?n c?a tâm xe (m)
                dist_center = (dist_L + dist_R) / 2.0
                vx = dist_center / dt   # V?n t?c tuy?n tính th?c t? (m/s)

                # Tích h?p hu?ng di chuy?n s? d?ng góc quay Yaw t? IMU (self.th)
                dx = vx * math.cos(self.th) * dt
                dy = vx * math.sin(self.th) * dt

                self.x += dx
                self.y += dy

                # ============================================
                # PUBLISH /odom
                # ============================================
                odom_msg = Odometry()
                odom_msg.header.stamp = current_ros_time.to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_footprint'

                # Pose
                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = yaw_to_quaternion(self.th)

                # Twist (trong frame base_footprint)
                odom_msg.twist.twist.linear.x = vx
                odom_msg.twist.twist.linear.y = 0.0   # Không có thành ph?n tru?t ngang
                odom_msg.twist.twist.angular.z = self.current_imu_wz # L?y t? c?m bi?n IMU

                # Covariance
                pose_cov = [0.0] * 36
                pose_cov[0]  = 0.01    # x
                pose_cov[7]  = 0.01    # y
                pose_cov[14] = 1e6     # z
                pose_cov[21] = 1e6     # roll
                pose_cov[28] = 1e6     # pitch
                pose_cov[35] = 0.01    # yaw (Ð? chính xác cao nh? IMU)
                odom_msg.pose.covariance = pose_cov

                twist_cov = [0.0] * 36
                twist_cov[0]  = 0.01   # vx
                twist_cov[7]  = 1e6    # vy
                twist_cov[14] = 1e6    # vz
                twist_cov[21] = 1e6    # wx
                twist_cov[28] = 1e6    # wy
                twist_cov[35] = 0.01   # wz
                odom_msg.twist.covariance = twist_cov

                self.odom_pub.publish(odom_msg)

                # ============================================
                # BROADCAST TF: odom ? base_footprint
                # ============================================
                if self.publish_tf:
                    t = TransformStamped()
                    t.header.stamp = current_ros_time.to_msg()
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_footprint'
                    t.transform.translation.x = self.x
                    t.transform.translation.y = self.y
                    t.transform.translation.z = 0.0
                    t.transform.rotation = yaw_to_quaternion(self.th)
                    self.tf_broadcaster.sendTransform(t)

            # C?p nh?t encoder cho l?n sau
            self.last_enc_L = enc_L
            self.last_enc_R = enc_R
            self.last_time = now_sec

            # ============================================
            # PUBLISH /battery
            # ============================================
            bat_msg = BatteryState()
            bat_msg.header.stamp = current_ros_time.to_msg()
            bat_msg.voltage = bat_voltage
            bat_msg.temperature = temperature
            bat_msg.present = True
            self.battery_pub.publish(bat_msg)

        except Exception as e:
            self.get_logger().error(f"Error in serial_read_callback: {e}")

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial: Ðã dóng k?t n?i.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HoverbotOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

