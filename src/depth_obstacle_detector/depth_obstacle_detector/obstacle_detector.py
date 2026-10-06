import os
import sys
import yaml
import numpy as np
import cv2
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2

from depth_obstacle_detector.config_paths import resolve_ground_path


class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_detector_node')
        
        self.declare_parameter('config_file', '')
        self.declare_parameter('enable_debug', True)
        
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.debug_enabled = self.get_parameter('enable_debug').get_parameter_value().bool_value
        
        if not config_path:
            self.get_logger().error("Parameter 'config_file' is empty! Please provide a configuration file.")
            sys.exit(1)
            
        self.get_logger().info(f"Loading configuration from: {config_path}")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.bridge = CvBridge()
        
        self.camera_info = None
        self.cached_tf = None
        
        self.load_configuration(config_path)

        if self.cam_name == "Rear" and self.debug_enabled:
            self.pc_pub = self.create_publisher(PointCloud2, '/rear_obstacle_points', 1)

    def load_configuration(self, config_path):
        try:
            with open(config_path, 'r') as f:
                config_full = yaml.safe_load(f)
                
            if 'front_camera' in config_full:
                self.config = config_full['front_camera']
                self.cam_name = "Front"
            elif 'rear_camera' in config_full:
                self.config = config_full['rear_camera']
                self.cam_name = "Rear"
            else:
                self.config = config_full
                target = self.config.get('target_frame', '')
                if 'rear' in target.lower():
                    self.cam_name = "Rear"
                else:
                    self.cam_name = "Front"
                
            self.enabled = self.config.get('enabled', True)
            if not self.enabled:
                self.get_logger().warn(f"{self.cam_name} Camera is disabled in config. Node will exit.")
                sys.exit(0)
                
            self.depth_topic = self.config.get('depth_topic', '/camera/depth/image_raw')
            self.info_topic = self.config.get('camera_info_topic', '/camera/depth/camera_info')
            self.output_topic = self.config.get('output_scan_topic', '/scan_obstacles')
            
            self.target_frame = self.config.get('target_frame', 'base_link')
            
            if self.cam_name == "Rear":
                self.target_frame = "rear_scan_frame"
                
            self.pitch_deg = self.config.get('pitch_deg', 0.0)
            self.min_range = self.config.get('min_range', 0.2)
            self.max_range = self.config.get('max_range', 2.5)
            
            self.roi_x = int(self.config.get('roi_x', 0))
            self.roi_y = int(self.config.get('roi_y', 0))
            self.roi_w = int(self.config.get('roi_width', 640))
            self.roi_h = int(self.config.get('roi_height', 480))
            
            self.thresh_val = int(self.config.get('threshold', 80))
            self.min_area = int(self.config.get('min_area', 150))
            self.k_median = int(self.config.get('median_filter', 3))
            self.k_morph = int(self.config.get('morphology_size', 3))
            
            ground_path = self.config.get('ground_file_path', 'None')
            
            if ground_path and ground_path != "None" and ground_path != "Memory":
                ground_path = resolve_ground_path(config_path, ground_path)
                self.get_logger().info(f"Loading ground reference from: {ground_path}")
                self.ground_frame = np.load(ground_path)
            else:
                self.ground_frame = None
                self.get_logger().warn("No valid ground reference loaded. Obstacle detection will be inactive.")
                
            self.get_logger().info(f"[{self.cam_name}] Topics -> Depth: {self.depth_topic}, Scan: {self.output_topic}")
            self.get_logger().info(f"[{self.cam_name}] Target Frame: {self.target_frame}, Pitch: {self.pitch_deg} deg")

            qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1
            )

            self.info_sub = self.create_subscription(CameraInfo, self.info_topic, self.info_callback, qos_profile)
            self.subscription = self.create_subscription(Image, self.depth_topic, self.listener_callback, qos_profile)
            self.laser_pub = self.create_publisher(LaserScan, self.output_topic, 1)

        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {str(e)}")
            sys.exit(1)

    def info_callback(self, msg):
        self.camera_info = msg

    def listener_callback(self, msg):
        t0 = time.perf_counter()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_image.dtype == np.float32 or cv_image.dtype == np.float64:
                cv_image = np.nan_to_num(cv_image, posinf=0.0, neginf=0.0)
                cv_image = (cv_image * 1000.0).astype(np.uint16)
            self.process_frame(cv_image, msg.header.stamp, t0)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {str(e)}")

    def process_frame(self, cv_image, stamp, t0):
        if self.ground_frame is None or self.ground_frame.shape != cv_image.shape:
            return
            
        h, w = cv_image.shape

        if self.camera_info is not None:
            fx, fy = self.camera_info.k[0], self.camera_info.k[4]
            cx, cy = self.camera_info.k[2], self.camera_info.k[5]
            frame_id = self.camera_info.header.frame_id
            if frame_id == 'rear_camera_link':
                frame_id = 'rear_camera_depth_optical_frame'
            elif frame_id == 'camera_link' or frame_id == 'front_camera_link':
                frame_id = 'camera_depth_optical_frame'
        else:
            if self.cam_name == "Rear":
                return
            fx = fy = 570.0
            cx, cy = w / 2.0, h / 2.0
            frame_id = 'camera_depth_optical_frame'

        fov = 2.0 * np.arctan(w / (2.0 * fx))
        is_rear = (self.cam_name == "Rear")
        
        scan_msg = LaserScan()
        scan_msg.header.stamp = stamp
        scan_msg.range_min = float(self.min_range)
        scan_msg.range_max = float(self.max_range)

        if is_rear:
            actual_frame_id = self.target_frame
            use_tf = True
            scan_msg.header.frame_id = actual_frame_id
            scan_msg.angle_min = -np.pi / 2.0
            scan_msg.angle_max = np.pi / 2.0
            num_bins = 360
            scan_msg.angle_increment = np.pi / num_bins
            ranges = np.full(num_bins, np.inf, dtype=np.float32)
            
            if self.cached_tf is None:
                try:
                    trans = self.tf_buffer.lookup_transform(actual_frame_id, frame_id, rclpy.time.Time())
                    translation = np.array([
                        trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z
                    ])
                    qx, qy = trans.transform.rotation.x, trans.transform.rotation.y
                    qz, qw = trans.transform.rotation.z, trans.transform.rotation.w
                    R = np.array([
                        [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
                    ])
                    self.cached_tf = (translation, R)
                except Exception as e:
                    self.get_logger().warn(f"TF failed: {str(e)}", throttle_duration_sec=2.0)
                    return
            translation, R = self.cached_tf
        else:
            use_tf = (self.target_frame != frame_id)
            translation = np.array([0.0, 0.0, 0.0])
            R = np.eye(3)
            actual_frame_id = frame_id
            
            if use_tf:
                if self.cached_tf is None:
                    try:
                        trans = self.tf_buffer.lookup_transform(self.target_frame, frame_id, rclpy.time.Time())
                        translation = np.array([
                            trans.transform.translation.x,
                            trans.transform.translation.y,
                            trans.transform.translation.z
                        ])
                        qx, qy = trans.transform.rotation.x, trans.transform.rotation.y
                        qz, qw = trans.transform.rotation.z, trans.transform.rotation.w
                        R = np.array([
                            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
                        ])
                        self.cached_tf = (translation, R)
                    except Exception:
                        use_tf = False
                else:
                    translation, R = self.cached_tf

            if use_tf:
                actual_frame_id = self.target_frame
            elif self.pitch_deg != 0:
                actual_frame_id = self.target_frame
                
            scan_msg.header.frame_id = actual_frame_id
            scan_msg.angle_min = -fov / 2.0
            scan_msg.angle_max = fov / 2.0
            scan_msg.angle_increment = fov / w
            ranges = np.full(w, np.inf, dtype=np.float32)

        rx, ry = max(0, min(self.roi_x, w-1)), max(0, min(self.roi_y, h-1))
        rw, rh = max(1, min(self.roi_w, w-rx)), max(1, min(self.roi_h, h-ry))
        
        current_roi = cv_image[ry:ry+rh, rx:rx+rw]
        ground_roi = self.ground_frame[ry:ry+rh, rx:rx+rw]

        valid_mask = (current_roi > 0) & (ground_roi > 0)
        diff = ground_roi.astype(np.int32) - current_roi.astype(np.int32)
        
        obstacle_mask = np.zeros_like(diff, dtype=np.uint8)
        obstacle_mask[valid_mask & (diff > self.thresh_val)] = 255
        
        if self.k_median > 1:
            obstacle_mask = cv2.medianBlur(obstacle_mask, self.k_median)
        if self.k_morph > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_morph, self.k_morph))
            obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
            obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        refined_mask = np.zeros_like(obstacle_mask)
        for cnt in contours:
            if cv2.contourArea(cnt) >= self.min_area:
                cv2.drawContours(refined_mask, [cnt], -1, 255, -1)

        refined_mask = cv2.bitwise_and(refined_mask, obstacle_mask)

        y_idx, x_idx = np.where(refined_mask == 255)
        
        if len(y_idx) > 0:
            u_coords = rx + x_idx
            v_coords = ry + y_idx
            depths_m = current_roi[y_idx, x_idx] / 1000.0
            
            valid = depths_m > 0
            u_coords, v_coords, depths_m = u_coords[valid], v_coords[valid], depths_m[valid]
            
            if len(depths_m) > 0:
                x_3d = (u_coords - cx) * depths_m / fx
                y_3d = (v_coords - cy) * depths_m / fy
                z_3d = depths_m
                
                if use_tf:
                    pts_optical = np.vstack([x_3d, y_3d, z_3d]).T
                    pts_target = pts_optical @ R.T + translation
                    x_t, y_t, z_t = pts_target[:, 0], pts_target[:, 1], pts_target[:, 2]
                    
                    if is_rear:
                        thetas = np.arctan2(y_t, x_t)
                        planar_ranges = np.sqrt(x_t**2 + y_t**2)
                    else:
                        thetas = np.arctan2(y_t, x_t)
                        planar_ranges = np.sqrt(x_t**2 + y_t**2)
                else:
                    tilt_deg = self.pitch_deg
                    if tilt_deg != 0:
                        alpha_rad = -tilt_deg * np.pi / 180.0
                        cos_a, sin_a = np.cos(alpha_rad), np.sin(alpha_rad)
                        x_t = x_3d
                        y_t = y_3d * cos_a - z_3d * sin_a
                        z_t = y_3d * sin_a + z_3d * cos_a
                        thetas = np.arctan2(-x_t, z_t)
                        planar_ranges = np.sqrt(x_t**2 + z_t**2)
                    else:
                        thetas = np.arctan2(x_3d, z_3d)
                        planar_ranges = np.sqrt(x_3d**2 + z_3d**2)

                valid_r = (planar_ranges >= self.min_range) & (planar_ranges <= self.max_range)
                thetas = thetas[valid_r]
                planar_ranges = planar_ranges[valid_r]

                bin_idx = ((thetas - scan_msg.angle_min) / scan_msg.angle_increment).astype(np.int32)
                
                max_bins = num_bins if is_rear else w
                in_bounds = (bin_idx >= 0) & (bin_idx < max_bins)
                
                bin_idx = bin_idx[in_bounds]
                planar_ranges = planar_ranges[in_bounds]
                
                if len(planar_ranges) > 0:
                    np.minimum.at(ranges, bin_idx, planar_ranges)
                    
        scan_msg.ranges = ranges.tolist()
        self.laser_pub.publish(scan_msg)
        
        t1 = time.perf_counter()
        
        # Calculate latency
        now = self.get_clock().now()
        stamp_time = rclpy.time.Time.from_msg(stamp)
        total_latency = (now - stamp_time).nanoseconds / 1e9
        
        self.get_logger().info(
            f"[{self.cam_name}] ProcTime: {(t1-t0)*1000:.1f}ms | TotalLatency: {total_latency*1000:.1f}ms",
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
