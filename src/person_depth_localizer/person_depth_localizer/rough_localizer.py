"""Uncalibrated AI RGB + Depth person preview: NEVER use output as a Nav2 goal.

The RGB bbox and Depth ROI are joined by normalized image coordinates.
This is NOT a calibrated RGB/Depth registration. Intended to prove that
tracking, depth, TF2 and RViz messages work before calibration.
Now includes Stage 4 HandWave association per track_id.
"""
from collections import deque
import json
import math
import re
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformException, TransformListener
from .geometry import backproject_depth_pixel, planar_xy, transform_xyz

from robot_interfaces.msg import TrackedPerson, TrackedPersonArray


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class RoughPersonLocalizer(Node):
    def __init__(self):
        super().__init__('rough_person_localizer')
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('max_time_skew_s', 0.25)
        self.declare_parameter('min_depth_m', 0.45)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('min_samples', 4)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('depth_frame_override', '')
        self.declare_parameter('fallback_hfov_deg', 58.0)
        self.declare_parameter('fallback_vfov_deg', 45.0)

        self.bridge = CvBridge()
        self.rgb_w = int(self.get_parameter('rgb_width').value)
        self.rgb_h = int(self.get_parameter('rgb_height').value)
        self.info = None
        self.depth_buffer = deque(maxlen=15)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Stage 4: Hand wave tracking integration
        self.waving_track_ids = set()
        self.last_wave_time = 0.0
        self.global_wave_detected = False

        self.create_subscription(
            Image, '/ai_camera/color/image_raw', self.rgb_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, '/ai_camera/depth/image_raw', self.depth_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/ai_camera/depth/camera_info', self.info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            DetectionArray, '/yolo/tracking', self.tracking_cb, 10)

        # HandWave subscribers
        self.create_subscription(
            Bool, '/pose/wave_detected', self.wave_detected_cb, 10)
        self.create_subscription(
            String, '/pose/wave_status', self.wave_status_cb, 10)

        # Multi-person array topic
        self.tracked_people_pub = self.create_publisher(
            TrackedPersonArray, '/perception/tracked_people', 10)

        # Legacy / Preview single-person topics
        self.preview_base_pub = self.create_publisher(
            PointStamped, '/mission/person_preview_base', 10)
        self.preview_map_pub = self.create_publisher(
            PointStamped, '/mission/person_preview_map', 10)
        self.distance_pub = self.create_publisher(
            Float32, '/mission/person_distance_preview', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/mission/person_marker', 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, '/mission/person_marker_array', 10)
        self.status_pub = self.create_publisher(
            String, '/mission/person_preview_status', 10)

        self.get_logger().warn(
            'UNCALIBRATED PREVIEW ONLY. Multi-person + HandWave tracking active on /perception/tracked_people. '
            'Do not subscribe to autonomous navigation until calibrated.')

    def wave_detected_cb(self, msg: Bool):
        self.last_wave_time = time.time()
        self.global_wave_detected = bool(msg.data)
        if not self.global_wave_detected:
            self.waving_track_ids.clear()

    def wave_status_cb(self, msg: String):
        self.last_wave_time = time.time()
        # Parse format: "[person_41] Waving with Right hand! | [person_12] ..."
        matches = re.findall(r'\[person_([-\w]+)\]', msg.data)
        if matches:
            self.waving_track_ids = set(matches)
            self.global_wave_detected = True

    def rgb_cb(self, msg):
        self.rgb_w, self.rgb_h = msg.width, msg.height

    def info_cb(self, msg):
        self.info = msg

    def depth_cb(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if image.ndim != 2:
                raise ValueError('Expected mono depth image')
            if msg.encoding.upper() == '16UC1':
                image = image.astype(np.float32) * 0.001
            elif msg.encoding.upper() == '32FC1':
                image = image.astype(np.float32)
            else:
                raise ValueError('Unsupported depth encoding: ' + msg.encoding)
            self.depth_buffer.append((stamp_s(msg.header), msg.header.frame_id, image, msg.header.stamp))
        except Exception as exc:
            self.report('depth_error', error=str(exc))

    def report(self, state, **values):
        payload = dict(state=state, calibrated=False, navigation_ready=False)
        payload.update(values)
        self.status_pub.publish(String(data=json.dumps(payload)))

    def camera_intrinsics(self, w, h):
        if self.info is not None and len(self.info.k) == 9 and self.info.k[0] > 0.0:
            return (float(self.info.k[0]), float(self.info.k[4]),
                    float(self.info.k[2]), float(self.info.k[5]), False)
        hfov = math.radians(float(self.get_parameter('fallback_hfov_deg').value))
        vfov = math.radians(float(self.get_parameter('fallback_vfov_deg').value))
        fx = (w * 0.5) / math.tan(hfov * 0.5)
        fy = (h * 0.5) / math.tan(vfov * 0.5)
        return fx, fy, w * 0.5, h * 0.5, True

    def tracking_cb(self, msg):
        people = [
            d for d in msg.detections
            if str(d.class_name).lower() == 'person' or d.class_id == 0
        ]
        if not people:
            self.report('no_detections')
            # Publish empty array
            empty_arr = TrackedPersonArray()
            empty_arr.header = msg.header
            empty_arr.header.frame_id = str(self.get_parameter('base_frame').value)
            empty_arr.geometry_calibrated = False
            self.tracked_people_pub.publish(empty_arr)
            return

        if not self.depth_buffer:
            self.report('waiting_for_depth')
            return

        t_yolo = stamp_s(msg.header)
        match = min(self.depth_buffer, key=lambda entry: abs(entry[0] - t_yolo))
        t_depth, frame_id, depth, depth_stamp = match
        skew = abs(t_yolo - t_depth)
        max_skew = float(self.get_parameter('max_time_skew_s').value)
        if skew > max_skew:
            self.report('depth_stale', skew_s=round(skew, 3), max_skew_s=max_skew)
            return

        h, w = depth.shape
        fx, fy, cx, cy, intrinsics_estimated = self.camera_intrinsics(w, h)
        source = str(self.get_parameter('depth_frame_override').value) or frame_id
        if not source:
            self.report('missing_depth_frame')
            return

        # Use the same depth acquisition time for camera->base and base->map TF2.
        # Camera height (1.20 m) and LiDAR height (0.30 m) live in the
        # robot URDF / TF tree. They must NEVER be added as offsets here.
        base_frame = str(self.get_parameter('base_frame').value)
        map_frame = str(self.get_parameter('map_frame').value)
        sensor_time = rclpy.time.Time.from_msg(depth_stamp)
        try:
            tf_base = self.tf_buffer.lookup_transform(base_frame, source, sensor_time)
        except (TransformException, ValueError) as exc:
            self.report('tf_base_unavailable', source_frame=source,
                        target_frame=base_frame, error=str(exc))
            return

        map_available = False
        tf_map = None
        if map_frame != base_frame:
            try:
                tf_map = self.tf_buffer.lookup_transform(
                    map_frame, base_frame, sensor_time)
                map_available = True
            except (TransformException, ValueError):
                pass

        now = self.get_clock().now().to_msg()
        lo = float(self.get_parameter('min_depth_m').value)
        hi = float(self.get_parameter('max_depth_m').value)
        min_samp = int(self.get_parameter('min_samples').value)

        # Stage 4: Check if HandWave node is active (< 2.5s)
        now_sec = time.time()
        wave_alive = (now_sec - self.last_wave_time) < 2.5

        tracked_people_list = []
        markers = []
        primary_person_data = None

        for idx, person in enumerate(people):
            track_id = str(person.id)
            bx, by = float(person.bbox.center.position.x), float(person.bbox.center.position.y)
            bw, bh = float(person.bbox.size.x), float(person.bbox.size.y)

            # Map from RGB normalized coordinate to Depth image coordinate
            nx = max(0.0, min(1.0, bx / max(1.0, float(self.rgb_w))))
            ny = max(0.0, min(1.0, by / max(1.0, float(self.rgb_h))))
            nw = max(0.0, min(1.0, bw / max(1.0, float(self.rgb_w))))
            nh = max(0.0, min(1.0, bh / max(1.0, float(self.rgb_h))))

            ud = max(0, min(w - 1, int(round(nx * w))))
            vd = max(0, min(h - 1, int(round(ny * h))))
            half_w = max(1, int(round(nw * w * 0.18)))
            half_h = max(1, int(round(nh * h * 0.18)))
            x0, x1 = max(0, ud - half_w), min(w, ud + half_w + 1)
            y0, y1 = max(0, vd - half_h), min(h, vd + half_h + 1)

            if x0 >= x1 or y0 >= y1:
                continue

            sample = depth[y0:y1, x0:x1]
            valid_sample = sample[np.isfinite(sample) & (sample >= lo) & (sample <= hi)]
            if valid_sample.size < min_samp:
                continue

            q25 = float(np.percentile(valid_sample, 25))
            foreground = valid_sample[valid_sample <= q25 + 0.35]
            if foreground.size < min_samp:
                continue

            depth_m = float(np.median(foreground))
            try:
                # Depth pixel -> 3D in depth optical frame (meters).
                point_camera = backproject_depth_pixel(
                    ud, vd, depth_m, fx, fy, cx, cy)
                # Full 3D extrinsic transform: P_base = R_base_camera * P_camera + t.
                point_base = transform_xyz(point_camera, tf_base)
                # Keep (x,y) on the robot's horizontal plane (Z is not range).
                # This is a body-point projection, NOT a calibrated foot location.
                x_base, y_base, dist_xy = planar_xy(point_base)
            except ValueError:
                continue

            # Stage 4: Check if this specific person is waving
            is_waving = False
            if wave_alive and self.global_wave_detected:
                clean_tid = track_id.strip()
                if (clean_tid in self.waving_track_ids or
                    str(clean_tid.split('_')[-1]) in self.waving_track_ids or
                    (len(people) == 1 and self.global_wave_detected)):
                    is_waving = True

            # Populate TrackedPerson
            tp = TrackedPerson()
            tp.track_id = track_id
            tp.position = Point(x=x_base, y=y_base, z=0.0)
            tp.distance_xy_m = float(dist_xy)
            tp.depth_m = float(depth_m)
            tp.position_valid = True
            tp.depth_valid = True
            tp.is_waving = bool(is_waving)
            tp.wave_valid = bool(wave_alive)
            tp.detection_confidence = float(person.score)
            tp.rgb_stamp = msg.header.stamp
            tp.depth_stamp = depth_stamp

            tracked_people_list.append(tp)

            # RViz Marker for each person
            # Sphere / Cylinder at position
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = base_frame
            marker.ns = 'tracked_people'
            marker.id = idx * 2
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(point_base[0])
            marker.pose.position.y = float(point_base[1])
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.lifetime.sec = 1

            label = Marker()
            label.header = marker.header
            label.ns = 'tracked_people'
            label.id = idx * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(point_base[0])
            label.pose.position.y = float(point_base[1])
            label.pose.position.z = 1.2
            label.pose.orientation.w = 1.0
            label.lifetime.sec = 1

            if is_waving:
                # Bright green cylinder and text with WAVING indicator
                marker.scale.x = 0.42
                marker.scale.y = 0.42
                marker.scale.z = 1.1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.2
                marker.color.a = 0.95

                label.scale.z = 0.26
                label.color.r = 0.2
                label.color.g = 1.0
                label.color.b = 0.2
                label.color.a = 1.0
                label.text = f'ID:{track_id} | d:{dist_xy:.2f}m | [WAVING] 👋'
            else:
                # Orange cylinder for normal person
                marker.scale.x = 0.35
                marker.scale.y = 0.35
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.55
                marker.color.b = 0.0
                marker.color.a = 0.85

                label.scale.z = 0.22
                label.color.r = 1.0
                label.color.g = 0.9
                label.color.b = 0.2
                label.color.a = 1.0
                label.text = f'ID:{track_id} | d:{dist_xy:.2f}m'

            markers.append(marker)
            markers.append(label)

            # Choose primary person (prefer waving person, else largest bbox)
            area = bw * bh
            score = area + (1e6 if is_waving else 0.0)
            if primary_person_data is None or score > primary_person_data['score']:
                primary_person_data = {
                    'score': score,
                    'point_base': point_base,
                    'dist_xy': dist_xy,
                    'depth_m': depth_m,
                    'track_id': track_id,
                    'is_waving': is_waving,
                }

        # Publish TrackedPersonArray
        arr_msg = TrackedPersonArray()
        arr_msg.header.stamp = msg.header.stamp
        arr_msg.header.frame_id = base_frame
        # Intrinsics, RGB-depth registration and extrinsics are not yet
        # verified against real measurements; never mark this navigable.
        arr_msg.geometry_calibrated = False
        arr_msg.people = tracked_people_list
        self.tracked_people_pub.publish(arr_msg)

        # Publish Markers to RViz
        if markers:
            self.markers_pub.publish(MarkerArray(markers=markers))
            self.marker_pub.publish(markers[0])

        # Backward compatibility for single-person preview topics
        if primary_person_data is not None:
            base_msg = PointStamped()
            base_msg.header.stamp = depth_stamp
            base_msg.header.frame_id = base_frame
            base_msg.point.x = float(primary_person_data['point_base'][0])
            base_msg.point.y = float(primary_person_data['point_base'][1])
            base_msg.point.z = 0.0
            self.preview_base_pub.publish(base_msg)
            self.distance_pub.publish(Float32(data=float(primary_person_data['dist_xy'])))

            if map_available and tf_map is not None:
                try:
                    point_map = transform_xyz(primary_person_data['point_base'], tf_map)
                    map_msg = PointStamped()
                    map_msg.header.stamp = depth_stamp
                    map_msg.header.frame_id = map_frame
                    map_msg.point.x = float(point_map[0])
                    map_msg.point.y = float(point_map[1])
                    map_msg.point.z = 0.0
                    self.preview_map_pub.publish(map_msg)
                except Exception:
                    pass

        waving_count = sum(1 for p in tracked_people_list if p.is_waving)
        self.report('preview_only',
                    tracked_people_count=len(tracked_people_list),
                    waving_people_count=waving_count,
                    wave_module_active=wave_alive,
                    people_detected=len(people),
                    map_available=map_available,
                    source_frame=source,
                    target_frame=base_frame,
                    geometry_projection='camera_3d_tf2_base_xy',
                    point_is_ground_contact=False,
                    intrinsics_estimated=intrinsics_estimated,
                    rgb_depth_registered=False,
                    skew_s=round(skew, 3))


def main(args=None):
    rclpy.init(args=args)
    node = RoughPersonLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
