#!/usr/bin/env python3
"""Resilient USB Camera Node with Automatic Hotplug Reconnection for ROS 2.

Unlike standard usb_cam which crashes when a USB cable is unplugged, this node:
- Safely catches USB disconnection exceptions without crashing.
- Automatically transitions to RECONNECTING state and releases old device handles.
- Polls the device path or fallback V4L2 devices periodically (default 1.0s).
- Seamlessly re-opens and resumes publishing Image and CameraInfo when replugged.
"""
from __future__ import annotations

import os
import threading
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class ResilientCameraNode(Node):
    """ROS 2 Node that streams UVC camera frames with fault-tolerant auto-reconnect."""

    def __init__(self) -> None:
        super().__init__("resilient_camera_node")

        # Khai báo các tham số cấu hình
        self.declare_parameter("video_device", "/dev/video2")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("framerate", 30.0)
        self.declare_parameter("camera_name", "ai_camera")
        self.declare_parameter("frame_id", "ai_camera_color_optical_frame")
        self.declare_parameter("reconnect_interval", 1.0)
        self.declare_parameter("auto_detect_fallback", True)

        self.video_device_param = str(self.get_parameter("video_device").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.framerate = float(self.get_parameter("framerate").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.reconnect_interval = float(self.get_parameter("reconnect_interval").value)
        self.auto_detect_fallback = bool(self.get_parameter("auto_detect_fallback").value)

        # Publishers
        self.pub_image = self.create_publisher(Image, "image_raw", 10)
        self.pub_info = self.create_publisher(CameraInfo, "camera_info", 10)

        # Trạng thái kết nối
        self._cap: Optional[cv2.VideoCapture] = None
        self._running = True
        self._connected = False
        self._current_device = self.video_device_param
        self._lock = threading.Lock()

        # Thread thu nhận hình ảnh liên tục
        self._worker_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._worker_thread.start()

        self.get_logger().info(
            f"[RESILIENT_CAM] Khởi tạo Node thành công cho thiết bị {self.video_device_param} "
            f"({self.image_width}x{self.image_height} @ {self.framerate}fps, topic: ~/image_raw)"
        )

    def _find_available_video_device(self) -> str:
        """Tìm cổng video V4L2 khả dụng nếu cổng cấu hình ban đầu bị đổi số khi cắm lại."""
        if os.path.exists(self.video_device_param):
            return self.video_device_param

        # Nếu không thấy cổng chỉ định và cho phép fallback, quét /dev/video*
        # Ưu tiên index 2 (Cam AI) trước
        if self.auto_detect_fallback:
            for idx in [2, 0, 4, 6, 8, 1, 3, 5]:
                cand = f"/dev/video{idx}"
                if os.path.exists(cand):
                    # Thử mở kiểm tra nhanh
                    test_cap = cv2.VideoCapture(cand, cv2.CAP_V4L2)
                    if test_cap.isOpened():
                        ret, _ = test_cap.read()
                        test_cap.release()
                        if ret:
                            return cand
        return self.video_device_param

    def _open_camera(self) -> bool:
        """Thử mở kết nối tới thiết bị camera với cấu hình độ phân giải và FPS."""
        target_dev = self._find_available_video_device()
        self._current_device = target_dev

        if not os.path.exists(target_dev) and not target_dev.isdigit():
            return False

        try:
            dev_idx = int(target_dev.replace("/dev/video", "")) if target_dev.startswith("/dev/video") else target_dev
            cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                return False

            # Cấu hình định dạng và độ phân giải
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.image_width))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.image_height))
            cap.set(cv2.CAP_PROP_FPS, float(self.framerate))

            # Đọc thử 1 frame xác nhận
            ret, frame = cap.read()
            if not ret or frame is None:
                cap.release()
                return False

            with self._lock:
                self._cap = cap
                self._connected = True

            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(
                f"[RESILIENT_CAM] Đã KẾT NỐI camera thành công tại {target_dev} "
                f"({actual_w}x{actual_h} @ {actual_fps:.1f}fps). Bắt đầu truyền luồng ảnh!"
            )
            return True
        except Exception as exc:
            self.get_logger().debug(f"[RESILIENT_CAM] Lỗi khi mở {target_dev}: {exc}")
            return False

    def _close_camera(self) -> None:
        """Giải phóng camera an toàn khi mất kết nối để không kẹt cổng."""
        with self._lock:
            self._connected = False
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None

    def _capture_loop(self) -> None:
        """Vòng lặp thu nhận hình ảnh liên tục với khả năng tự phục hồi khi rút cáp."""
        warn_logged = False

        while self._running and rclpy.ok():
            if not self._connected or self._cap is None:
                # Đang ở trạng thái mất kết nối -> thử kết nối lại
                if not warn_logged:
                    self.get_logger().warn(
                        f"[RESILIENT_CAM] Mất kết nối camera ({self._current_device}). "
                        f"Đang chờ cắm lại (tự động thử lại mỗi {self.reconnect_interval:.1f}s)..."
                    )
                    warn_logged = True

                if self._open_camera():
                    warn_logged = False
                else:
                    time.sleep(self.reconnect_interval)
                    continue

            # Đang kết nối -> Đọc khung hình
            try:
                ret, frame = self._cap.read()
                if not ret or frame is None:
                    # Rút cáp nóng hoặc lỗi luồng đọc
                    self.get_logger().warn(
                        f"[RESILIENT_CAM] Không nhận được frame từ {self._current_device} (cáp bị rút nóng). "
                        f"Đang giải phóng cổng và chuyển sang chế độ tự phục hồi..."
                    )
                    self._close_camera()
                    time.sleep(0.5)
                    continue

                # Tạo message ROS 2 và publish
                now = self.get_clock().now().to_msg()

                # 1. Image message (BGR8)
                img_msg = Image()
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.frame_id
                img_msg.height = int(frame.shape[0])
                img_msg.width = int(frame.shape[1])
                img_msg.encoding = "bgr8"
                img_msg.is_bigendian = 0
                img_msg.step = int(frame.shape[1] * 3)
                img_msg.data = frame.tobytes()
                self.pub_image.publish(img_msg)

                # 2. CameraInfo message
                info_msg = CameraInfo()
                info_msg.header.stamp = now
                info_msg.header.frame_id = self.frame_id
                info_msg.height = img_msg.height
                info_msg.width = img_msg.width
                self.pub_info.publish(info_msg)

            except Exception as exc:
                self.get_logger().warn(f"[RESILIENT_CAM] Ngoại lệ khi đọc frame: {exc}")
                self._close_camera()
                time.sleep(0.5)

    def destroy_node(self) -> None:
        self._running = False
        self._close_camera()
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ResilientCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
