"""Uncalibrated AI RGB + Depth person preview: NEVER use output as a Nav2 goal.

The RGB bbox and Depth ROI are joined by normalized image coordinates.
This is NOT a calibrated RGB/Depth registration. Intended to prove that
tracking, depth, TF2 and RViz messages work before calibration.
"""
from collections import deque
import json
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from yolo_msgs.msg import DetectionArray
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def transform_xyz(point, tf):
    """Transform a numpy xyz using a unit quaternion from geometry_msgs."""
    t = tf.transform.translation
    q = tf.transform.rotation
    xyz = np.asarray(point, dtype=np.float64)
    qxyz = np.array([q.x, q.y, q.z], dtype=np.float64)
    norm = math.sqrt(float(np.dot(qxyz, qxyz) + q.w * q.w))
    if norm <= 1e-9:
        raise ValueError("invalid TF quaternion")
    qxyz /= norm
    w = q.w / norm
    uv = np.cross(qxyz, xyz)
    uuv = np.cross(qxyz, uv)
    return xyz + 2.0 * (w * uv + uuv) + np.array([t.x, t.y, t.z])


class RoughPersonLocalizer(Node):
    def __init__(self):
        super().__init__('rough_person_localizer')
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('max_time_skew_s', 0.25)
        self.declare_parameter('min_depth_m', 0.45)
        self.declare_parameter('max_depth_m', 4.5)
        self.declare_parameter('min_samples', 6)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('depth_frame_override', '')
        self.declare_parameter('fallback_hfov_deg', 58.0)
        self.declare_parameter('fallback_vfov_deg', 45.0)

        self.bridge = CvBridge()
        self.rgb_w = int(self.get_parameter('rgb_width').value)
        self.rgb_h = int(self.get_parameter('rgb_height').value)
        self.info = None
        self.depth_buffer = deque(maxlen=12)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
            'UNCALIBRATED PREVIEW ONLY. Approximate RGB-to-Depth pixel ratio; '
            'do not subscribe these topics to autonomous navigation.')

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
            self.depth_buffer.append((stamp_s(msg.header), msg.header.frame_id, image))
        except Exception as exc:
            self.report('depth_error', error=str(exc))

    def report(self, state, **values):
        payload = dict(state=state, calibrated=False, navigation_ready=False)
        payload.update(values)
        self.status_pub.publish(String(data=json.dumps(payload)))

    def camera_intrinsics(self, width, height):
        if self.info is not None and self.info.k[0] > 0 and self.info.k[4] > 0:
            fx, fy = float(self.info.k[0]), float(self.info.k[4])
            cx, cy = float(self.info.k[2]), float(self.info.k[5])
            if self.info.width and self.info.height:
                sx, sy = width / self.info.width, height / self.info.height
                fx, fy, cx, cy = fx * sx, fy * sy, cx * sx, cy * sy
            return fx, fy, cx, cy, False
        hfov = math.radians(float(self.get_parameter('fallback_hfov_deg').value))
        vfov = math.radians(float(self.get_parameter('fallback_vfov_deg').value))
        return (width / (2.0 * math.tan(hfov / 2.0)),
                height / (2.0 * math.tan(vfov / 2.0)),
                (width - 1) / 2.0, (height - 1) / 2.0, True)

    def tracking_cb(self, msg):
        people = [
            d for d in msg.detections
            if (d.class_id == 0 or d.class_name.lower() == 'person')
            and d.score > 0.4 and d.bbox.size.x > 0 and d.bbox.size.y > 0
        ]
        if not people:
            self.report('no_person')
            return
        if not self.depth_buffer or not self.rgb_w or not self.rgb_h:
            self.report('waiting_for_depth')
            return
        # Multi-person association is unsafe before RGB/Depth registration:
        # select only one dominant bbox for this single-person bench test.
        person = max(people, key=lambda d: d.bbox.size.x * d.bbox.size.y)
        ts = stamp_s(msg.header)
        depth_ts, frame_id, depth = min(
            self.depth_buffer, key=lambda item: abs(item[0] - ts))
        skew = abs(depth_ts - ts)
        if skew > float(self.get_parameter('max_time_skew_s').value):
            self.report('unsynchronized', skew_s=round(skew, 3))
            return

        h, w = depth.shape
        bb = person.bbox
        # Bbox is in *original RGB pixel coordinates*, not YOLO tensor pixels.
        # Select upper torso and a small patch to reduce background.
        ud = (bb.center.position.x / self.rgb_w) * w
        vd = ((bb.center.position.y - 0.12 * bb.size.y) / self.rgb_h) * h
        half_w = max(2, int(bb.size.x / self.rgb_w * w * 0.12))
        half_h = max(2, int(bb.size.y / self.rgb_h * h * 0.10))
        u, v = int(round(ud)), int(round(vd))
        x0, x1 = max(0, u-half_w), min(w, u+half_w+1)
        y0, y1 = max(0, v-half_h), min(h, v+half_h+1)
        if x0 >= x1 or y0 >= y1:
            self.report('roi_outside_depth')
            return

        sample = depth[y0:y1, x0:x1]
        lo = float(self.get_parameter('min_depth_m').value)
        hi = float(self.get_parameter('max_depth_m').value)
        sample = sample[np.isfinite(sample) & (sample >= lo) & (sample <= hi)]
        if sample.size < int(self.get_parameter('min_samples').value):
            self.report('insufficient_depth', sample_count=int(sample.size))
            return
        # Prefer a compact foreground cluster over background pixels.
        q25 = float(np.percentile(sample, 25))
        foreground = sample[sample <= q25 + 0.35]
        if foreground.size < int(self.get_parameter('min_samples').value):
            self.report('insufficient_foreground', sample_count=int(foreground.size))
            return
        depth_m = float(np.median(foreground))
        fx, fy, cx, cy, intrinsics_estimated = self.camera_intrinsics(w, h)
        xyz = np.array([(ud-cx)*depth_m/fx,
                        (vd-cy)*depth_m/fy, depth_m], dtype=np.float64)
        source = str(self.get_parameter('depth_frame_override').value) or frame_id
        if not source:
            self.report('missing_depth_frame')
            return
        base_frame = str(self.get_parameter('base_frame').value)
        map_frame = str(self.get_parameter('map_frame').value)
        try:
            tf_base = self.tf_buffer.lookup_transform(
                base_frame, source, rclpy.time.Time())
            point_base = transform_xyz(xyz, tf_base)
        except (TransformException, ValueError) as exc:
            self.report('tf_base_unavailable', source_frame=source, error=str(exc))
            return

        now = self.get_clock().now().to_msg()
        base_msg = PointStamped()
        base_msg.header.stamp = now
        base_msg.header.frame_id = base_frame
        base_msg.point.x = float(point_base[0])
        base_msg.point.y = float(point_base[1])
        base_msg.point.z = 0.0  # ground-plane proxy of the measured torso
        self.preview_base_pub.publish(base_msg)
        distance = math.hypot(point_base[0], point_base[1])
        self.distance_pub.publish(Float32(data=float(distance)))

        selected_frame, planar = base_frame, point_base.copy()
        mapped = False
        try:
            tf_map = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time())
            planar = transform_xyz(point_base, tf_map)
            selected_frame, mapped = map_frame, True
            map_msg = PointStamped()
            map_msg.header.stamp = now
            map_msg.header.frame_id = map_frame
            map_msg.point.x, map_msg.point.y = float(planar[0]), float(planar[1])
            map_msg.point.z = 0.0
            self.preview_map_pub.publish(map_msg)
        except (TransformException, ValueError):
            pass

        marker = Marker()
        marker.header.stamp, marker.header.frame_id = now, selected_frame
        marker.ns, marker.id = 'uncalibrated_person_preview', 0
        marker.type, marker.action = Marker.SPHERE, Marker.ADD
        marker.pose.position.x, marker.pose.position.y = float(planar[0]), float(planar[1])
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = 0.28, 0.28, 0.24
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.55, 0.0, 0.9
        marker.lifetime.sec = 1
        label = Marker()
        label.header, label.ns, label.id = marker.header, marker.ns, 1
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y = marker.pose.position.x, marker.pose.position.y
        label.pose.position.z, label.pose.orientation.w = 0.5, 1.0
        label.scale.z = 0.18
        label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 0.7, 0.1, 1.0
        label.lifetime.sec = 1
        label.text = f'ROUGH person {person.id} ~{distance:.2f}m'
        self.marker_pub.publish(marker)
        self.markers_pub.publish(MarkerArray(markers=[marker, label]))
        self.report('preview_only', track_id=str(person.id),
                    depth_m=round(depth_m, 3),
                    planar_distance_m=round(distance, 3),
                    approx_registration=True,
                    approx_intrinsics=intrinsics_estimated,
                    map_available=mapped, source_frame=source,
                    people_detected=len(people), skew_s=round(skew, 3))


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
