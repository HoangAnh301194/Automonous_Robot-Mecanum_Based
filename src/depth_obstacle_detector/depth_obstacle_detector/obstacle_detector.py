#!/usr/bin/env python3

import os
import sys
import yaml
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


class ObstacleDetectorNode(Node):
    """
    Lightweight, headless ROS 2 Node that loads a YAML configuration file 
    and performs real-time depth obstacle detection at maximum performance,
    publishing the result as a LaserScan.
    Supports TF2 coordinate transformations to target frames (e.g. camera_link).
    """
    def __init__(self):
        super().__init__('depth_obstacle_detector_node')
        
        # Declare parameter for configuration file path
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_path:
            self.get_logger().error("Parameter 'config_file' is empty! Please provide a configuration file.")
            sys.exit(1)
            
        self.get_logger().info(f"Loading configuration from: {config_path}")
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.load_configuration(config_path)

        # Subscriber state
        self.camera_info = None
        self.bridge = CvBridge()

        # Subscribe to camera info
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.info_callback,
            10
        )
        
        # Subscribe to depth raw topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.listener_callback,
            10
        )
        
        # LaserScan publisher for detected obstacles
        self.laser_pub = self.create_publisher(LaserScan, '/scan_obstacles', 10)
        self.get_logger().info("Lightweight Obstacle Detector Node initialized successfully.")

    def load_configuration(self, config_path):
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
                
            # Extract parameters
            self.roi_x = int(self.config.get('roi_x', 0))
            self.roi_y = int(self.config.get('roi_y', 0))
            self.roi_w = int(self.config.get('roi_width', 100))
            self.roi_h = int(self.config.get('roi_height', 100))
            self.thresh_val = int(self.config.get('threshold', 80))
            self.min_area = int(self.config.get('min_area', 150))
            self.k_median = int(self.config.get('median_filter', 3))
            self.k_morph = int(self.config.get('morphology_size', 3))
            self.target_frame = str(self.config.get('target_frame', 'base_link'))
            self.camera_tilt = int(self.config.get('camera_tilt', 0))
            
            ground_path = self.config.get('ground_file_path', 'None')
            
            # Resolve relative ground file path if needed
            if ground_path and ground_path != "None" and ground_path != "Memory (Unsaved)":
                if not os.path.isabs(ground_path):
                    config_dir = os.path.dirname(config_path)
                    ground_path = os.path.abspath(os.path.join(config_dir, ground_path))
                
                self.get_logger().info(f"Loading ground reference from: {ground_path}")
                self.ground_frame = np.load(ground_path)
                self.get_logger().info(f"Loaded ground reference shape: {self.ground_frame.shape}")
            else:
                self.ground_frame = None
                self.get_logger().warn("No valid ground reference loaded. Obstacle detection will be inactive.")

            # Print parameters for verification
            self.get_logger().info(f"Target Frame: {self.target_frame}")
            self.get_logger().info(f"ROI: X={self.roi_x}, Y={self.roi_y}, W={self.roi_w}, H={self.roi_h}")
            self.get_logger().info(f"Filters: Threshold={self.thresh_val}mm, MinArea={self.min_area}px, Median={self.k_median}, Morph={self.k_morph}")

        except Exception as e:
            self.get_logger().error(f"Failed to load configuration or ground file: {str(e)}")
            sys.exit(1)

    def info_callback(self, msg):
        self.camera_info = msg

    def listener_callback(self, msg):
        try:
            # Convert depth image (passthrough retains 16-bit depth values)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.process_frame(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to process incoming image frame: {str(e)}")

    def process_frame(self, cv_image):
        h, w = cv_image.shape

        # Fetch or fallback camera intrinsic parameters
        if self.camera_info is not None:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            frame_id = self.camera_info.header.frame_id
        else:
            # Fallback approximate parameters
            fx = 570.0
            fy = 570.0
            cx = w / 2.0
            cy = h / 2.0
            frame_id = 'camera_depth_optical_frame'

        # Compute Horizontal Field of View (FOV) in Radians
        fov = 2.0 * np.arctan(w / (2.0 * fx))

        # Check if we should use TF projection to target frame
        use_tf = (self.target_frame != frame_id)
        
        translation = np.array([0.0, 0.0, 0.0])
        R = np.eye(3)
        actual_frame_id = frame_id

        if use_tf:
            try:
                # Lookup transform from camera optical frame to target frame
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    frame_id,
                    rclpy.time.Time()
                )
                
                # Get Translation
                translation = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])
                
                # Get Rotation Quaternion
                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w
                
                # Compute Rotation Matrix
                R = np.array([
                    [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                    [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                    [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
                ])
                actual_frame_id = self.target_frame
            except Exception as e:
                self.get_logger().warn(
                    f"TF lookup failed to {self.target_frame}: {str(e)}. Falling back to manual tilt."
                )
                use_tf = False
                if self.camera_tilt != 0:
                    actual_frame_id = self.target_frame
                else:
                    actual_frame_id = frame_id

        # Build ROS 2 LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = actual_frame_id
        scan_msg.angle_min = -fov / 2.0
        scan_msg.angle_max = fov / 2.0
        scan_msg.angle_increment = fov / w
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.033  # ~30 FPS
        scan_msg.range_min = 0.4
        scan_msg.range_max = 5.0

        # Initialize ranges to infinity
        ranges = [float('inf')] * w

        if self.ground_frame is not None:
            # Reset ground if resolution differs
            if self.ground_frame.shape != cv_image.shape:
                self.get_logger().error(f"Ground reference shape {self.ground_frame.shape} doesn't match stream resolution {cv_image.shape}!")
                return

            # Clamp ROI bounds to current resolution
            rx = max(0, min(self.roi_x, w - 1))
            ry = max(0, min(self.roi_y, h - 1))
            rw = max(1, min(self.roi_w, w - rx))
            rh = max(1, min(self.roi_h, h - ry))

            # Extract ROIs
            current_roi = cv_image[ry:ry+rh, rx:rx+rw]
            ground_roi = self.ground_frame[ry:ry+rh, rx:rx+rw]

            # Compare and create valid pixel mask
            valid_mask = (current_roi > 0) & (ground_roi > 0)
            diff = ground_roi.astype(np.int32) - current_roi.astype(np.int32)

            # Threshold
            obstacle_mask = np.zeros_like(diff, dtype=np.uint8)
            obstacle_mask[valid_mask & (diff > self.thresh_val)] = 255

            # Apply Median filter
            if self.k_median > 1:
                obstacle_mask = cv2.medianBlur(obstacle_mask, self.k_median)

            # Apply Morphological filter
            if self.k_morph > 0:
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.k_morph, self.k_morph))
                obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
                obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)

            # Check for qualified contour areas
            contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create a refined mask where only objects of area >= min_area are preserved
            refined_mask = np.zeros_like(obstacle_mask)
            for cnt in contours:
                if cv2.contourArea(cnt) >= self.min_area:
                    cv2.drawContours(refined_mask, [cnt], -1, 255, -1)

            # Perform NumPy vectorized conversion from mask to LaserScan
            y_idx, x_idx = np.where(refined_mask == 255)
            
            if len(y_idx) > 0:
                u_coords = rx + x_idx
                v_coords = ry + y_idx
                
                depths_m = current_roi[y_idx, x_idx] / 1000.0
                
                # Filter invalid depth values
                valid = depths_m > 0
                u_coords = u_coords[valid]
                v_coords = v_coords[valid]
                depths_m = depths_m[valid]
                
                if len(depths_m) > 0:
                    # Project to 3D space in optical frame
                    x_3d = (u_coords - cx) * depths_m / fx
                    y_3d = (v_coords - cy) * depths_m / fy
                    z_3d = depths_m
                    
                    if use_tf:
                        # Stack points
                        pts_optical = np.vstack([x_3d, y_3d, z_3d]).T
                        # Rotate and translate
                        pts_target = pts_optical @ R.T + translation
                        
                        x_t = pts_target[:, 0]
                        y_t = pts_target[:, 1]
                        
                        thetas = np.arctan2(y_t, x_t)
                        planar_ranges = np.sqrt(x_t**2 + y_t**2)
                    else:
                        # Apply manual camera tilt pitch rotation around X-axis
                        tilt_deg = self.camera_tilt
                        if tilt_deg != 0:
                            alpha_rad = -tilt_deg * np.pi / 180.0
                            cos_a = np.cos(alpha_rad)
                            sin_a = np.sin(alpha_rad)
                            
                            # Pitch rotation
                            x_t = x_3d
                            y_t = y_3d * cos_a - z_3d * sin_a
                            z_t = y_3d * sin_a + z_3d * cos_a
                            
                            # Map to ROS frame conventions: forward = z_t, left = -x_t
                            thetas = np.arctan2(-x_t, z_t)
                            planar_ranges = np.sqrt(x_t**2 + z_t**2)
                        else:
                            # Standard optical frame angle/range calculations
                            thetas = np.arctan2(x_3d, z_3d)
                            planar_ranges = np.sqrt(x_3d**2 + z_3d**2)
                    
                    bin_idx = ((thetas - scan_msg.angle_min) / scan_msg.angle_increment).astype(np.int32)
                    
                    # Filter bins lying within image width bounds
                    in_bounds = (bin_idx >= 0) & (bin_idx < w)
                    planar_ranges = planar_ranges[in_bounds]
                    bin_idx = bin_idx[in_bounds]
                    
                    if len(planar_ranges) > 0:
                        ranges_arr = np.full(w, np.inf, dtype=np.float32)
                        np.minimum.at(ranges_arr, bin_idx, planar_ranges)
                        ranges = [float(val) for val in ranges_arr]

        scan_msg.ranges = ranges
        self.laser_pub.publish(scan_msg)


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
