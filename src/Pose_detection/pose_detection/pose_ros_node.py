#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
from yolo_msgs.msg import DetectionArray

from gesture.logic import NearestTrackSelector, RaisedHandRule, TemporalRaisedHandFilter
from gesture.types import PersonPose
from gesture.visualization import draw_people


class PoseRosNode(Node):
    def __init__(self):
        super().__init__('pose_ros_node')

        # ROS Parameters
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('margin_ratio', 0.02)
        self.declare_parameter('history_size', 7)
        self.declare_parameter('on_votes', 3)
        self.declare_parameter('off_votes', 3)
        self.declare_parameter('max_people', 5)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('enable_window', True)

        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        margin_ratio = float(self.get_parameter('margin_ratio').value)
        history_size = int(self.get_parameter('history_size').value)
        on_votes = int(self.get_parameter('on_votes').value)
        off_votes = int(self.get_parameter('off_votes').value)
        max_people = int(self.get_parameter('max_people').value)
        image_topic = str(self.get_parameter('image_topic').value)
        self.enable_window = bool(self.get_parameter('enable_window').value)

        # Pipeline Components from HandWaveDetection_Pose
        self.rule = RaisedHandRule(
            keypoint_threshold=self.conf_threshold, margin_ratio=margin_ratio
        )
        self.filter = TemporalRaisedHandFilter(
            history_size=history_size, on_votes=on_votes, off_votes=off_votes
        )
        self.selector = NearestTrackSelector(max_people=max_people)
        self.bridge = CvBridge()
        self.latest_frame: np.ndarray | None = None
        self.latest_ranked_people: list[PersonPose] = []
        self.any_waving: bool = False

        # FPS calculation state
        self.last_frame_time: float | None = None
        self.smoothed_fps: float = 0.0

        # Subscribe to Camera Image with SensorData QoS for high-speed camera compatibility
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Subscribe to YOLO Detections
        self.subscription = self.create_subscription(
            DetectionArray, '/yolo/detections', self.detection_callback, 10
        )

        # Publishers (Use depth=10 for standard RViz & Web UI compatibility)
        self.publisher = self.create_publisher(String, '/pose/wave_status', 10)
        self.bool_pub = self.create_publisher(Bool, '/pose/wave_detected', 10)
        self.marker_pub = self.create_publisher(Marker, '/pose/wave_markers', 10)
        self.image_pub = self.create_publisher(Image, '/pose/image_debug', 10)

        if self.enable_window:
            cv2.namedWindow('Pose Detection & Hand-Wave Debug', cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f'Pose ROS Node active. Subscribed to /yolo/detections & {image_topic}. Publishing to /pose/image_debug'
        )

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.render_and_publish_debug()
        except Exception as e:
            self.get_logger().warn(f'Failed to convert camera image: {e}')

    def render_and_publish_debug(self):
        if self.latest_frame is None:
            return

        # Calculate FPS
        current_time = time.perf_counter()
        if self.last_frame_time is not None:
            dt = current_time - self.last_frame_time
            if dt > 0:
                inst_fps = 1.0 / dt
                self.smoothed_fps = 0.9 * self.smoothed_fps + 0.1 * inst_fps
        else:
            self.smoothed_fps = 0.0
        self.last_frame_time = current_time

        debug_frame = draw_people(
            self.latest_frame.copy(),
            self.latest_ranked_people,
            keypoint_threshold=self.conf_threshold,
            stage='raised_hand',
        )

        # Overlay status banner
        banner_text = (
            "WAVE DETECTED!" if self.any_waving else "SCANNING FOR HAND WAVE..."
        )
        banner_color = (0, 0, 255) if self.any_waving else (0, 255, 0)
        cv2.putText(
            debug_frame,
            banner_text,
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            banner_color,
            2,
            cv2.LINE_AA,
        )

        # Overlay FPS on Top Right
        fps_text = f"FPS: {self.smoothed_fps:.1f}"
        frame_width = debug_frame.shape[1]
        cv2.putText(
            debug_frame,
            fps_text,
            (frame_width - 150, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

        if self.enable_window:
            cv2.imshow('Pose Detection & Hand-Wave Debug', debug_frame)
            cv2.waitKey(1)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'camera_color_optical_frame'
            self.image_pub.publish(image_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')

    def detection_callback(self, msg: DetectionArray):
        people: list[PersonPose] = []
        active_ids: set[int] = set()

        if self.latest_frame is not None:
            frame_height, frame_width = self.latest_frame.shape[:2]
        else:
            frame_height, frame_width = 480, 640

        frame_shape = (frame_height, frame_width, 3)

        for i, detection in enumerate(msg.detections):
            if detection.class_name != 'person':
                continue

            raw_id = detection.id
            if raw_id and str(raw_id).isdigit():
                track_id = int(raw_id)
            else:
                track_id = i + 1
            active_ids.add(track_id)

            # Bounding box
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y
            size_x = detection.bbox.size.x
            size_y = detection.bbox.size.y
            bbox = np.array(
                [
                    center_x - size_x / 2.0,
                    center_y - size_y / 2.0,
                    center_x + size_x / 2.0,
                    center_y + size_y / 2.0,
                ],
                dtype=np.float32,
            )

            # Keypoints: Map to 17 COCO keypoints (0..16)
            keypoints = np.zeros((17, 2), dtype=np.float32)
            keypoint_scores = np.zeros(17, dtype=np.float32)

            for kp in detection.keypoints.data:
                idx = kp.id - 1 if 1 <= kp.id <= 17 else kp.id
                if 0 <= idx < 17:
                    keypoints[idx] = [kp.point.x, kp.point.y]
                    keypoint_scores[idx] = kp.score

            person = PersonPose(
                bbox=bbox,
                confidence=float(detection.score),
                track_id=track_id,
                keypoints=keypoints,
                keypoint_scores=keypoint_scores,
            )
            people.append(person)

        # 1. Rank & select nearest people
        ranked_people = self.selector.select(people, frame_shape)
        self.filter.retain({p.track_id for p in ranked_people})

        self.any_waving = False
        waving_events = []

        # 2. Process each person through HandWaveDetection_Pose pipeline
        for person in ranked_people:
            left_obs, right_obs = self.rule.classify(person)
            left_state, right_state = self.filter.update(
                person.track_id, left_obs, right_obs
            )

            person.left_raised = left_state
            person.right_raised = right_state

            if left_state or right_state:
                self.any_waving = True
                hand_state = person.state
                event_str = f'[person_{person.track_id}] Waving with {hand_state} hand!'
                waving_events.append(event_str)

                # RViz Marker (Text above head)
                marker = Marker()
                marker.header.frame_id = 'camera_color_optical_frame'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'wave_detection'
                marker.id = person.track_id
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = 0.0
                marker.pose.position.y = 0.0
                marker.pose.position.z = 1.0
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.text = f'WAVING! ({hand_state})'
                marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
                self.marker_pub.publish(marker)

        self.latest_ranked_people = ranked_people

        # Publish global wave boolean status
        bool_msg = Bool()
        bool_msg.data = self.any_waving
        self.bool_pub.publish(bool_msg)

        # Publish event strings
        if waving_events:
            combined_event = ' | '.join(waving_events)
            self.get_logger().info(combined_event)
            string_msg = String()
            string_msg.data = combined_event
            self.publisher.publish(string_msg)

    def destroy_node(self):
        if self.enable_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
