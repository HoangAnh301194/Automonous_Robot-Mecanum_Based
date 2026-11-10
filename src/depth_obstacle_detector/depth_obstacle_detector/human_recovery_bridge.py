#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
human_recovery_bridge.py
Node cầu nối phát hiện người chắn đường từ Camera AI YOLO & Depth:
- Lắng nghe /yolo/detections (DetectionArray) từ camera AI.
- Lọc đối tượng "person" trước mũi xe ở cự ly an toàn (0.4m - 1.8m).
- Xuất cờ True/False ra topic /human_detected_in_path (std_msgs/Bool).
- Khi có người đứng chắn quá 3.5s, tự động bắn thông điệp xin đường ra /navigation/speech_request.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

try:
    from yolo_msgs.msg import DetectionArray
except ImportError:
    DetectionArray = None


class HumanRecoveryBridge(Node):
    def __init__(self):
        super().__init__('human_recovery_bridge')

        # Khai báo parameters
        self.declare_parameter('max_human_distance', 1.8)
        self.declare_parameter('min_human_distance', 0.35)
        self.declare_parameter('speak_warning_delay', 3.5)

        self.max_dist = self.get_parameter('max_human_distance').value
        self.min_dist = self.get_parameter('min_human_distance').value
        self.speak_delay = self.get_parameter('speak_warning_delay').value

        # Publisher trạng thái người chắn đường
        self.pub_human = self.create_publisher(Bool, '/human_detected_in_path', 10)
        self.pub_speech = self.create_publisher(String, '/navigation/speech_request', 10)

        # Trạng thái theo dõi
        self.human_blocking = False
        self.human_first_seen_time = 0.0
        self.last_speech_time = 0.0

        # Subscriber nhận diện YOLO
        if DetectionArray is not None:
            self.sub_yolo = self.create_subscription(
                DetectionArray, '/yolo/detections', self.on_yolo_detections, 10
            )
            self.get_logger().info("Đã kết nối với topic /yolo/detections.")
        else:
            self.get_logger().warn("Chưa import được yolo_msgs. Sẵn sàng hoạt động ở chế độ chờ.")

        # Timer chu kỳ kiểm tra và xuất trạng thái (10Hz)
        self.timer = self.create_timer(0.1, self.publish_status)
        self.get_logger().info("HumanRecoveryBridge đã khởi động. Cổng chờ AI sẵn sàng!")

    def on_yolo_detections(self, msg):
        now = time.monotonic()
        found_human_near = False

        for det in msg.detections:
            class_name = getattr(det, 'class_name', '').lower()
            if class_name == 'person' or det.class_id == 0:
                # Kiểm tra cự ly nếu có thông tin 3D position
                z_dist = 1.0  # Mặc định nằm trong cự ly nếu chỉ có 2D bbox
                if hasattr(det, 'bbox3d') and hasattr(det.bbox3d, 'center'):
                    z_dist = det.bbox3d.center.position.z
                elif hasattr(det, 'position') and hasattr(det.position, 'z'):
                    z_dist = det.position.z

                if self.min_dist <= z_dist <= self.max_dist:
                    found_human_near = True
                    break

        if found_human_near:
            if not self.human_blocking:
                self.human_blocking = True
                self.human_first_seen_time = now
        else:
            self.human_blocking = False
            self.human_first_seen_time = 0.0

    def publish_status(self):
        msg = Bool()
        msg.data = self.human_blocking
        self.pub_human.publish(msg)

        # Nếu người đứng chắn đường quá lâu -> phát thông điệp xin đường
        if self.human_blocking:
            now = time.monotonic()
            if (now - self.human_first_seen_time >= self.speak_delay) and (now - self.last_speech_time >= 6.0):
                self.last_speech_time = now
                speech_msg = String()
                speech_msg.data = "Dạ xin lỗi, vui lòng cho robot xin phép đi qua ạ!"
                self.pub_speech.publish(speech_msg)
                self.get_logger().info("Đã phát thông điệp xin đường: Dạ xin lỗi, vui lòng cho robot xin phép đi qua ạ!")


def main(args=None):
    rclpy.init(args=args)
    node = HumanRecoveryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
