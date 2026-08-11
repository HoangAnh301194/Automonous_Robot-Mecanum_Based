import sys
import time
from pathlib import Path
from typing import List

import cv2
import numpy as np
import rclpy
import message_filters
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from yolo_msgs.msg import DetectionArray

# Add parent package path so 'raised_hand' is always resolvable
package_dir = Path(__file__).resolve().parent.parent
if str(package_dir) not in sys.path:
    sys.path.insert(0, str(package_dir))

from raised_hand.backends import (
    empty_pose,
    expand_bbox,
    normalize_device,
    validate_onnxruntime_device,
    validate_torch_device,
)
from raised_hand.config import load_config
from raised_hand.logic import RaisedHandRule, TemporalRaisedHandFilter
from raised_hand.rtmpose_batch import BatchedRTMPose
from raised_hand.types import PersonPose
from raised_hand.visualization import draw_people


class HandWaveDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('hand_wave_detection')

        package_share = Path(get_package_share_directory('hand_wave_detection'))
        default_config = package_share / 'config.yaml'
        self.config = self._load_config(default_config)
        self.bridge = CvBridge()

        # Initialize BatchedRTMPose directly without loading YOLO detector model
        device = normalize_device(self.config.device)
        validate_torch_device(device)
        validate_onnxruntime_device(device)
        self.device = device

        self.pose = BatchedRTMPose(
            onnx_model=self.config.models.rtmpose,
            model_input_size=(
                self.config.processing.pose_input_width,
                self.config.processing.pose_input_height,
            ),
            backend='onnxruntime',
            device=self.device,
        )

        self.classifier = RaisedHandRule(
            keypoint_threshold=self.config.processing.keypoint_threshold,
            margin_ratio=self.config.processing.head_margin_ratio,
        )
        self.temporal_filter = TemporalRaisedHandFilter()
        self.last_error_time = 0.0

        image_topic = self._string_parameter(
            'image_topic', '/camera/color/image_raw'
        )
        tracking_topic = self._string_parameter(
            'tracking_topic', '/yolo/tracking'
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

        # Subscribers using message_filters for timestamp synchronization
        self.image_sub = message_filters.Subscriber(
            self,
            Image,
            image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.tracking_sub = message_filters.Subscriber(
            self,
            DetectionArray,
            tracking_topic,
            qos_profile=10,
        )

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.tracking_sub],
            queue_size=10,
            slop=0.1,
        )
        self.sync.registerCallback(self.sync_callback)

        # Publishers
        self.debug_pub = self.create_publisher(Image, debug_topic, 10)
        self.wave_detected_pub = self.create_publisher(
            Bool, wave_detected_topic, 10
        )
        self.wave_status_pub = self.create_publisher(String, wave_status_topic, 10)

        self.get_logger().info(
            f'HandWaveDetection active: {image_topic} + {tracking_topic} '
            f'-> {wave_detected_topic}, {wave_status_topic}'
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

    def sync_callback(
        self, image_msg: Image, tracking_msg: DetectionArray
    ) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            confidence_threshold = self.config.processing.confidence

            raw_bboxes = []
            track_ids = []
            confidences = []

            for det in tracking_msg.detections:
                # Filter person class and confidence score threshold
                is_person = (
                    det.class_name.lower() == 'person'
                    or det.class_id == 0
                    or 'person' in det.class_name.lower()
                )
                if is_person and det.score >= confidence_threshold:
                    cx = det.bbox.center.position.x
                    cy = det.bbox.center.position.y
                    w = det.bbox.size.x
                    h = det.bbox.size.y

                    x1 = cx - w / 2.0
                    y1 = cy - h / 2.0
                    x2 = cx + w / 2.0
                    y2 = cy + h / 2.0

                    raw_bboxes.append([x1, y1, x2, y2])

                    # Parse track ID from Detection.id
                    tid_str = str(det.id).strip()
                    if tid_str.isdigit():
                        tid = int(tid_str)
                    elif tid_str:
                        try:
                            tid = int(tid_str.split('_')[-1])
                        except ValueError:
                            tid = abs(hash(tid_str)) % (10**8)
                    else:
                        tid = len(track_ids) + 1

                    track_ids.append(tid)
                    confidences.append(float(det.score))

            people: List[PersonPose] = []
            if raw_bboxes:
                expanded_bboxes = [
                    expand_bbox(
                        bbox,
                        frame.shape,
                        self.config.processing.pose_bbox_margin_ratio,
                    ).tolist()
                    for bbox in raw_bboxes
                ]

                keypoints_arr, scores_arr = self.pose(frame, bboxes=expanded_bboxes)
                keypoints_arr = np.asarray(keypoints_arr, dtype=np.float32)
                scores_arr = np.asarray(scores_arr, dtype=np.float32)

                if keypoints_arr.ndim == 2:
                    keypoints_arr = keypoints_arr[None, ...]
                if scores_arr.ndim == 1:
                    scores_arr = scores_arr[None, ...]

                for i in range(len(raw_bboxes)):
                    kpts = (
                        keypoints_arr[i]
                        if i < len(keypoints_arr)
                        else empty_pose()[0]
                    )
                    kpts_scores = (
                        scores_arr[i]
                        if i < len(scores_arr)
                        else empty_pose()[1]
                    )

                    person = PersonPose(
                        bbox=np.asarray(raw_bboxes[i], dtype=np.float32),
                        confidence=confidences[i],
                        track_id=track_ids[i],
                        keypoints=kpts,
                        keypoint_scores=kpts_scores,
                    )
                    people.append(person)

            self._classify_people(people)
            self._publish_wave_status(people)
            self._publish_debug_image(frame, people, 'rtmpose_external_bbox', image_msg)
        except Exception as error:
            self._report_error(error)

    def _classify_people(self, people: List[PersonPose]) -> None:
        self.temporal_filter.retain({person.track_id for person in people})
        for person in people:
            left_raised, right_raised = self.classifier.classify(person)
            person.left_raised, person.right_raised = self.temporal_filter.update(
                person.track_id,
                left_raised,
                right_raised,
            )

    def _publish_wave_status(self, people: List[PersonPose]) -> None:
        events = []
        for person in people:
            if person.left_raised or person.right_raised:
                events.append(
                    f'[person_{person.track_id}] Waving with {person.state} hand!'
                )

        self.wave_detected_pub.publish(Bool(data=bool(events)))
        if events:
            self.wave_status_pub.publish(String(data=' | '.join(events)))

    def _publish_debug_image(
        self, frame: np.ndarray, people: List[PersonPose], backend_name: str, source: Image
    ) -> None:
        annotated = draw_people(
            frame,
            people,
            self.config.processing.keypoint_threshold,
            stage=self.config.stage,
        )
        cv2.putText(
            annotated,
            f'{self.config.backend} | {backend_name} | {self.device}',
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


