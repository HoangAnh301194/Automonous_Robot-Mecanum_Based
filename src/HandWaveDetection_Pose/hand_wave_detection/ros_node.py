import time
from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from raised_hand.app import create_backend
from raised_hand.config import load_config
from raised_hand.logic import RaisedHandRule, TemporalRaisedHandFilter
from raised_hand.visualization import draw_people


class HandWaveDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('hand_wave_detection')

        package_share = Path(get_package_share_directory('hand_wave_detection'))
        default_config = package_share / 'config.yaml'
        self.config = self._load_config(default_config)
        self.bridge = CvBridge()
        self.backend = create_backend(self.config)
        self.classifier = RaisedHandRule(
            keypoint_threshold=self.config.processing.keypoint_threshold,
            margin_ratio=self.config.processing.head_margin_ratio,
        )
        self.temporal_filter = TemporalRaisedHandFilter()
        self.last_error_time = 0.0

        image_topic = self._string_parameter(
            'image_topic', '/camera/color/image_raw'
        )
        debug_topic = self._string_parameter(
            'debug_topic', '/pose/image_debug'
        )
        wave_detected_topic = self._string_parameter(
            'wave_detected_topic', '/pose/wave_detected'
        )
        wave_status_topic = self._string_parameter(
            'wave_status_topic', '/pose/wave_status'
        )

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.wave_detected_pub = self.create_publisher(
            Bool, wave_detected_topic, 10
        )
        self.wave_status_pub = self.create_publisher(String, wave_status_topic, 10)

        self.get_logger().info(
            'HandWaveDetection active: '
            f'{image_topic} -> {wave_detected_topic}, {wave_status_topic}'
        )

    def _load_config(self, default_config: Path):
        config_path = self._string_parameter('config', str(default_config))
        config = load_config(config_path)

        backend = self._string_parameter('backend', '')
        device = self._string_parameter('device', '')
        detector_model = self._string_parameter('detector_model', '')
        yolo_pose_model = self._string_parameter('yolo_pose_model', '')
        rtmpose_model = self._string_parameter('rtmpose_model', '')

        if backend:
            config.backend = backend
        if device:
            config.device = device
        if detector_model:
            config.models.detector = detector_model
        if yolo_pose_model:
            config.models.yolo_pose = yolo_pose_model
        if rtmpose_model:
            config.models.rtmpose = rtmpose_model

        config.stage = 'raised_hand'
        config.output.display = False
        config.output.save_video = False
        config.output.save_jsonl = False
        config.validate()
        return config

    def _string_parameter(self, name: str, default: str) -> str:
        return str(self.declare_parameter(name, default).value)

    def image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            inference = self.backend.infer(frame)
            people = inference.people
            self._classify_people(people)
            self._publish_wave_status(people)
            self._publish_debug_image(frame, people, inference.backend, message)
        except Exception as error:
            self._report_error(error)

    def _classify_people(self, people) -> None:
        self.temporal_filter.retain({person.track_id for person in people})
        for person in people:
            left_raised, right_raised = self.classifier.classify(person)
            person.left_raised, person.right_raised = self.temporal_filter.update(
                person.track_id,
                left_raised,
                right_raised,
            )

    def _publish_wave_status(self, people) -> None:
        events = []
        for person in people:
            if person.left_raised or person.right_raised:
                events.append(
                    f'[person_{person.track_id}] Waving with {person.state} hand!'
                )

        self.wave_detected_pub.publish(Bool(data=bool(events)))
        if events:
            self.wave_status_pub.publish(String(data=' | '.join(events)))

    def _publish_debug_image(self, frame, people, backend: str, source: Image) -> None:
        annotated = draw_people(
            frame,
            people,
            self.config.processing.keypoint_threshold,
            stage=self.config.stage,
        )
        cv2.putText(
            annotated,
            f'{self.config.backend} | {backend} | {self.config.device}',
            (16, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        debug_message = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        debug_message.header = source.header
        self.debug_pub.publish(debug_message)

    def _report_error(self, error: Exception) -> None:
        now = time.monotonic()
        if now - self.last_error_time >= 5.0:
            self.get_logger().error(f'Hand-wave frame processing failed: {error}')
            self.last_error_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandWaveDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
