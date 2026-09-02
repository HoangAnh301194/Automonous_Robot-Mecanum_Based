import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist, Quaternion
import serial
import math
import re

# Physical parameters matching ESP32 firmware
TICK_PER_METER = 173.91     # 90 ticks/rev ÷ 0.5175m circumference (6.5 inch wheel)
WHEEL_BASE     = 0.58       # Distance between left/right wheel centers (m)


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class EspBridgeNode(Node):
    """
    ROS 2 Hardware Serial Bridge Node for ESP32 Motor Controller.
    - Subscribes /cmd_vel and sends "V <linear> <angular>\n" to ESP32.
    - Reads ESP32 serial telemetry string.
    - Calculates raw 2-wheel differential drive encoder odometry.
    - Publishes /odom_encoder, /battery, and /dataenc.
    - Does NOT publish TF (TF is published by robot_localization EKF node).
    """

    def __init__(self):
        super().__init__('esp_bridge')

        # Parameters
        self.declare_parameter('esp_port', '/dev/ttyUSB0')
        self.declare_parameter('esp_baudrate', 115200)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        self.declare_parameter('ticks_per_meter', TICK_PER_METER)
        self.declare_parameter('cmd_vel_rate', 10.0)

        self.esp_port        = self.get_parameter('esp_port').value
        self.esp_baudrate    = self.get_parameter('esp_baudrate').value
        self.wheel_base      = self.get_parameter('wheel_base').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.cmd_vel_rate    = self.get_parameter('cmd_vel_rate').value

        self.get_logger().info("=== ESP32 Hardware Bridge Node Starting ===")
        self.get_logger().info(f"  Serial Port: {self.esp_port} @ {self.esp_baudrate}")
        self.get_logger().info(f"  Wheel Base: {self.wheel_base} m | Ticks/Meter: {self.ticks_per_meter}")

        # Publishers
        self.odom_encoder_pub = self.create_publisher(Odometry, '/odom_encoder', 10)
        self.battery_pub      = self.create_publisher(BatteryState, '/battery', 10)
        self.dataenc_pub      = self.create_publisher(Int32MultiArray, '/dataenc', 10)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Serial Connection
        self.ser = None
        try:
            self.ser = serial.Serial(self.esp_port, self.esp_baudrate, timeout=0.05)
            self.get_logger().info(f"Serial connected successfully on {self.esp_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.esp_port}: {e}")

        # Odometry state (Encoder accumulation)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_enc_L = None
        self.last_enc_R = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # Telemetry regex parser
        self.telemetry_pattern = re.compile(
            r"V:([-\d.]+)\s+A:([-\d.]+)\s+"
            r"RC:(\d+)/(\d+)\s+"
            r"TL:([-\d]+)\s+TR:([-\d]+)\s+"
            r"EL:([-\d]+)\s+ER:([-\d]+)\s+"
            r"B:([-\d.]+)\s+T:([-\d.]+)"
        )

        # Timers
        self.create_timer(0.05, self.serial_read_callback)  # 20 Hz telemetry loop

        if self.cmd_vel_rate > 0:
            self.create_timer(1.0 / self.cmd_vel_rate, self.cmd_vel_send_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_linear  = float(msg.linear.x)
        self.cmd_angular = float(msg.angular.z)

    def cmd_vel_send_callback(self):
        if self.ser is None or not self.ser.is_open:
            return
        command = f"V {self.cmd_linear:.3f} {self.cmd_angular:.3f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def serial_read_callback(self):
        try:
            if self.ser is None or not self.ser.is_open:
                return

            latest_telemetry = None
            while self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        latest_telemetry = line
                except Exception:
                    pass

            if latest_telemetry is None:
                return

            match = self.telemetry_pattern.search(latest_telemetry)
            if not match:
                return

            enc_L       = int(match.group(7))
            enc_R       = int(match.group(8))
            bat_voltage = float(match.group(9))
            temperature = float(match.group(10))

            # Publish raw encoder counts
            enc_msg = Int32MultiArray()
            enc_msg.data = [enc_L, enc_R]
            self.dataenc_pub.publish(enc_msg)

            current_ros_time = self.get_clock().now()
            now_sec = current_ros_time.nanoseconds / 1e9

            # Calculate Odometry from Encoder Ticks
            if self.last_enc_L is not None and self.last_enc_R is not None:
                dt = now_sec - self.last_time
                if dt <= 0 or dt > 1.0:
                    self.last_enc_L = enc_L
                    self.last_enc_R = enc_R
                    self.last_time = now_sec
                    return

                delta_L = enc_L - self.last_enc_L
                delta_R = enc_R - self.last_enc_R

                # Sanity check for corrupt spikes
                if abs(delta_L) > 5000 or abs(delta_R) > 5000:
                    self.last_enc_L = enc_L
                    self.last_enc_R = enc_R
                    self.last_time = now_sec
                    return

                dist_L = delta_L / self.ticks_per_meter
                dist_R = delta_R / self.ticks_per_meter

                dist_center = (dist_L + dist_R) / 2.0
                d_th = (dist_R - dist_L) / self.wheel_base

                vx = dist_center / dt
                wz = d_th / dt

                dx = dist_center * math.cos(self.th + d_th / 2.0)
                dy = dist_center * math.sin(self.th + d_th / 2.0)

                self.x += dx
                self.y += dy
                self.th += d_th

                # Publish /odom_encoder
                odom_msg = Odometry()
                odom_msg.header.stamp = current_ros_time.to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_footprint'

                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0
                odom_msg.pose.pose.orientation = yaw_to_quaternion(self.th)

                odom_msg.twist.twist.linear.x = vx
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.angular.z = wz

                # Covariance matrices
                pose_cov = [0.0] * 36
                pose_cov[0]  = 0.02    # x
                pose_cov[7]  = 0.02    # y
                pose_cov[14] = 1e6     # z
                pose_cov[21] = 1e6     # roll
                pose_cov[28] = 1e6     # pitch
                pose_cov[35] = 0.05    # yaw
                odom_msg.pose.covariance = pose_cov

                twist_cov = [0.0] * 36
                twist_cov[0]  = 0.02   # vx
                twist_cov[7]  = 1e6    # vy
                twist_cov[14] = 1e6    # vz
                twist_cov[21] = 1e6    # wx
                twist_cov[28] = 1e6    # wy
                twist_cov[35] = 0.05   # wz
                odom_msg.twist.covariance = twist_cov

                self.odom_encoder_pub.publish(odom_msg)

            self.last_enc_L = enc_L
            self.last_enc_R = enc_R
            self.last_time = now_sec

            # Publish Battery State
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
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
