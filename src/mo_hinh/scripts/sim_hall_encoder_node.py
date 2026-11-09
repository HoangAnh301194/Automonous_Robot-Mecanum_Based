#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mô phỏng chính xác hiện trạng phần cứng và thuật toán Odometry của xe thật:
1. Đọc góc quay bánh xe từ Gazebo (/joint_states).
2. Lượng tử hóa thành xung Hall encoder 90 ticks/vòng (độ phân giải 5.75 mm/tick).
3. Đọc dữ liệu IMU (/imu/data) và cập nhật yaw với bộ lọc filter_alpha = 0.9.
4. Tính toán Odometry ở chu kỳ 20 Hz theo đúng thuật toán hiện tại của xe thật (odom_publisher.py):
   - vx thô (không có LPF, để tái hiện đúng hiện tượng giật lượng tử hóa ở tốc độ thấp)
   - Tích phân Euler: dx = vx * cos(th) * dt, dy = vx * sin(th) * dt
5. Publish /dataenc, /odom và broadcast trực tiếp TF odom -> base_footprint (không qua EKF).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

TICK_PER_METER = 173.91     # 90 ticks/rev ÷ 0.5175m chu vi bánh 6.5 inch
WHEEL_BASE     = 0.56       # 0.56m khoảng cách 2 bánh xe


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimHallEncoderNode(Node):
    def __init__(self):
        super().__init__('sim_hall_encoder_node')

        self.declare_parameter('left_joint_name', 'wheel_left_joint')
        self.declare_parameter('right_joint_name', 'wheel_right_joint')
        self.declare_parameter('ticks_per_rev', 90)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        self.declare_parameter('ticks_per_meter', TICK_PER_METER)
        self.declare_parameter('filter_alpha', 0.9)    # Khớp hoàn toàn với odom_publisher.py
        self.declare_parameter('publish_rate', 20.0)   # 20 Hz khớp với ESP32
        self.declare_parameter('publish_tf', True)

        self.left_joint_name = self.get_parameter('left_joint_name').value
        self.right_joint_name = self.get_parameter('right_joint_name').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value
        self.filter_alpha = self.get_parameter('filter_alpha').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.rad_to_ticks = self.ticks_per_rev / (2.0 * math.pi)

        self.get_logger().info("=== Mô phỏng Phần cứng Encoder Hall & Odometry Xe Thật ===")
        self.get_logger().info(f"  Độ phân giải: {self.ticks_per_rev} ticks/vòng ({self.ticks_per_meter} ticks/m)")
        self.get_logger().info(f"  Bước lượng tử: {1000.0 / self.ticks_per_meter:.2f} mm/tick")
        self.get_logger().info(f"  Tần số lấy mẫu: {self.publish_rate} Hz | IMU Alpha: {self.filter_alpha}")
        self.get_logger().info(f"  Phát TF odom -> base_footprint: {self.publish_tf}")

        # Publishers (giống hệt xe thật)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.dataenc_pub = self.create_publisher(Int32MultiArray, '/dataenc', 10)
        self.battery_pub = self.create_publisher(BatteryState, '/battery', 10)

        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Trạng thái Encoder & IMU
        self.current_enc_L = 0
        self.current_enc_R = 0
        self.has_joint_data = False

        self.last_enc_L = None
        self.last_enc_R = None
        self.last_time = None

        self.imu_yaw_offset = None
        self.target_imu_yaw = None
        self.has_imu_orientation = False
        self.current_imu_wz = 0.0

        # Trạng thái Odometry tích lũy
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Timer 20 Hz (0.05s)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

    def joint_state_callback(self, msg: JointState):
        if self.left_joint_name not in msg.name or self.right_joint_name not in msg.name:
            return

        idx_l = msg.name.index(self.left_joint_name)
        idx_r = msg.name.index(self.right_joint_name)

        pos_l = msg.position[idx_l]
        pos_r = msg.position[idx_r]

        # Lượng tử hóa rời rạc thành số xung nguyên (như cảm biến Hall ESP32)
        self.current_enc_L = int(round(pos_l * self.rad_to_ticks))
        self.current_enc_R = int(round(pos_r * self.rad_to_ticks))
        self.has_joint_data = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w

        if q_norm_sq > 0.5:
            current_yaw = quaternion_to_yaw(q)
            if self.imu_yaw_offset is None:
                self.imu_yaw_offset = current_yaw
                self.th = 0.0
            self.target_imu_yaw = current_yaw - self.imu_yaw_offset
            self.has_imu_orientation = True
        else:
            self.has_imu_orientation = False

        self.current_imu_wz = msg.angular_velocity.z

    def timer_callback(self):
        if not self.has_joint_data:
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        enc_L = self.current_enc_L
        enc_R = self.current_enc_R

        # 1. Publish /dataenc
        enc_msg = Int32MultiArray()
        enc_msg.data = [enc_L, enc_R]
        self.dataenc_pub.publish(enc_msg)

        # 2. Tính Odometry vi sai 2 bánh + IMU
        if self.last_enc_L is not None and self.last_enc_R is not None:
            dt = now_sec - self.last_time
            if dt <= 0.0 or dt > 1.0:
                self.last_enc_L = enc_L
                self.last_enc_R = enc_R
                self.last_time = now_sec
                return

            delta_L = enc_L - self.last_enc_L
            delta_R = enc_R - self.last_enc_R

            if abs(delta_L) > 5000 or abs(delta_R) > 5000:
                self.last_enc_L = enc_L
                self.last_enc_R = enc_R
                self.last_time = now_sec
                return

            dist_L = delta_L / self.ticks_per_meter
            dist_R = delta_R / self.ticks_per_meter

            dist_center = (dist_L + dist_R) / 2.0
            dth_enc = (dist_R - dist_L) / self.wheel_base

            # Tính biến thiên góc quay Yaw (kết hợp vi sai encoder và IMU)
            if self.has_imu_orientation and self.target_imu_yaw is not None:
                self.th += dth_enc
                diff = self.target_imu_yaw - self.th
                while diff > math.pi:
                    diff -= 2.0 * math.pi
                while diff < -math.pi:
                    diff += 2.0 * math.pi
                dth = dth_enc + diff * self.filter_alpha
                self.th += diff * self.filter_alpha
            elif abs(self.current_imu_wz) > 1e-5:
                dth_imu = self.current_imu_wz * dt
                dth = dth_enc * (1.0 - self.filter_alpha) + dth_imu * self.filter_alpha
                self.th += dth
            else:
                dth = dth_enc
                self.th += dth

            # Chuẩn hóa góc quay Yaw trong khoảng [-pi, pi]
            while self.th > math.pi:
                self.th -= 2.0 * math.pi
            while self.th < -math.pi:
                self.th += 2.0 * math.pi

            # Vận tốc
            vx = dist_center / dt
            wz = self.current_imu_wz if abs(self.current_imu_wz) > 1e-4 else (dth_enc / dt)

            # Tích phân toạ độ x, y
            th_mid = self.th - 0.5 * dth
            dx = dist_center * math.cos(th_mid)
            dy = dist_center * math.sin(th_mid)

            self.x += dx
            self.y += dy

            # Publish /odom
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_footprint'

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = yaw_to_quaternion(self.th)

            odom_msg.twist.twist.linear.x = vx
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.angular.z = wz

            # Covariance giống odom_publisher.py
            pose_cov = [0.0] * 36
            pose_cov[0] = 0.01
            pose_cov[7] = 0.01
            pose_cov[14] = 1e6
            pose_cov[21] = 1e6
            pose_cov[28] = 1e6
            pose_cov[35] = 0.01
            odom_msg.pose.covariance = pose_cov

            twist_cov = [0.0] * 36
            twist_cov[0] = 0.01
            twist_cov[7] = 1e6
            twist_cov[14] = 1e6
            twist_cov[21] = 1e6
            twist_cov[28] = 1e6
            twist_cov[35] = 0.01
            odom_msg.twist.covariance = twist_cov

            self.odom_pub.publish(odom_msg)

            # Broadcast TF: odom -> base_footprint (giống xe thật khi chạy odom_pub)
            if self.publish_tf:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = yaw_to_quaternion(self.th)
                self.tf_broadcaster.sendTransform(t)

        self.last_enc_L = enc_L
        self.last_enc_R = enc_R
        self.last_time = now_sec

        # Publish Battery State
        bat_msg = BatteryState()
        bat_msg.header.stamp = now.to_msg()
        bat_msg.voltage = 25.2
        bat_msg.temperature = 30.0
        bat_msg.present = True
        self.battery_pub.publish(bat_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimHallEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
