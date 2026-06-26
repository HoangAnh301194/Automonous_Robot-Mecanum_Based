import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
import serial
import math

# --- CONFIGURATION ---
WHEEL_RADIUS = 0.05      # Wheel radius in meters
TICKS_PER_REV = 200      # Encoder ticks per revolution
FILTER_ALPHA = 0.3       # Low-pass filter alpha for IMU yaw smoothing
ODOM_MODE_ALL_4 = 'all_4'
ODOM_MODE_WHEELS_1_4 = 'wheels_1_4'
VALID_ODOM_MODES = (ODOM_MODE_ALL_4, ODOM_MODE_WHEELS_1_4)


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('esp_port', '/dev/ttyUSB0')
        self.declare_parameter('esp_baudrate', 115200)
        self.declare_parameter('wheel_odom_mode', ODOM_MODE_ALL_4)

        self.publish_tf = self.get_parameter('publish_tf').value
        self.esp_port = self.get_parameter('esp_port').value
        self.esp_baudrate = self.get_parameter('esp_baudrate').value
        self.wheel_odom_mode = self.get_parameter('wheel_odom_mode').value

        if self.wheel_odom_mode not in VALID_ODOM_MODES:
            self.get_logger().warn(
                f"Invalid wheel_odom_mode='{self.wheel_odom_mode}', fallback to '{ODOM_MODE_ALL_4}'."
            )
            self.wheel_odom_mode = ODOM_MODE_ALL_4

        self.get_logger().info(f"wheel_odom_mode = {self.wheel_odom_mode}")

        # Publishers / Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.dataenc_pub = self.create_publisher(Int32MultiArray, '/dataenc', 10)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Serial
        self.ser = None
        try:
            self.ser = serial.Serial(self.esp_port, self.esp_baudrate, timeout=0.1)
            self.get_logger().info(
                f"Connected to ESP32 successfully on {self.esp_port} @ {self.esp_baudrate}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to open Serial port {self.esp_port}: {e}"
            )
            return

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # IMU
        self.imu_yaw_offset = None
        self.current_imu_wz = 0.0

        # Encoder / Time
        self.last_enc = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Timer 20 Hz
        self.create_timer(0.05, self.timer_callback)

    def _compute_body_velocity(self, v_wheel):
        if self.wheel_odom_mode == ODOM_MODE_WHEELS_1_4:
            # Use only wheel #1 and #4 encoders (ignore #2 and #3).
            # With two wheels only, keep a conservative lateral estimate.
            vx = (v_wheel[0] + v_wheel[3]) / 2.0
            vy = 0.0
            return vx, vy

        vx = (v_wheel[0] + v_wheel[1] + v_wheel[2] + v_wheel[3]) / 4.0
        vy = -(v_wheel[0] - v_wheel[1] - v_wheel[2] + v_wheel[3]) / 4.0
        return vx, vy

    def cmd_vel_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return

        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = -float(msg.angular.z)

        command = f"V,{vx:.3f},{vy:.3f},{wz:.3f}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def imu_callback(self, msg):
        q = msg.orientation
        current_yaw = quaternion_to_yaw(q)

        if self.imu_yaw_offset is None:
            self.imu_yaw_offset = current_yaw
            self.th = 0.0
            return

        target_yaw = current_yaw - self.imu_yaw_offset

        diff = target_yaw - self.th
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        self.th += diff * FILTER_ALPHA
        self.current_imu_wz = msg.angular_velocity.z

    def timer_callback(self):
        try:
            if self.ser is None or not self.ser.is_open:
                return

            current_ros_time = self.get_clock().now()
            now_sec = current_ros_time.nanoseconds / 1e9

            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                if line.startswith("E,"):
                    parts = line.split(",")

                    if len(parts) == 5:
                        try:
                            enc = [int(p) for p in parts[1:]]
                        except ValueError:
                            return

                        # Publish raw encoder data to /dataenc
                        enc_msg = Int32MultiArray()
                        enc_msg.data = enc
                        self.dataenc_pub.publish(enc_msg)

                        dt = now_sec - self.last_time if self.last_enc is not None else 0.05

                        if self.last_enc is not None and 0 < dt < 1.0:
                            delta = [enc[i] - self.last_enc[i] for i in range(4)]

                            if all(abs(d) < 5000 for d in delta):
                                delta_rev = [d / TICKS_PER_REV for d in delta]
                                delta_s = [2 * math.pi * WHEEL_RADIUS * dr for dr in delta_rev]
                                v_wheel = [ds / dt for ds in delta_s]

                                vx, vy = self._compute_body_velocity(v_wheel)

                                dx = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
                                dy = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt

                                self.x += dx
                                self.y += dy

                                odom_msg = Odometry()
                                odom_msg.header.stamp = current_ros_time.to_msg()
                                odom_msg.header.frame_id = 'odom'
                                odom_msg.child_frame_id = 'base_footprint'

                                odom_msg.pose.pose.position.x = self.x
                                odom_msg.pose.pose.position.y = self.y
                                odom_msg.pose.pose.orientation = yaw_to_quaternion(self.th)

                                odom_msg.twist.twist.linear.x = vx
                                odom_msg.twist.twist.linear.y = vy
                                odom_msg.twist.twist.angular.z = self.current_imu_wz

                                odom_msg.pose.covariance = [
                                    0.01 if i in [0, 7, 35] else 1e6 for i in range(36)
                                ]

                                self.odom_pub.publish(odom_msg)

                        self.last_enc = enc
                        self.last_time = now_sec

            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = yaw_to_quaternion(self.th)
                self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Error in main timer loop: {e}")

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
