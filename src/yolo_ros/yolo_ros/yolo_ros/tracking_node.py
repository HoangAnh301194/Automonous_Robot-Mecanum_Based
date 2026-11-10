# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import time

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import cv2
import numpy as np
import message_filters
from cv_bridge import CvBridge

from ultralytics.engine.results import Boxes
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.utils import IterableSimpleNamespace, YAML
from ultralytics.utils.checks import check_requirements, check_yaml

from sensor_msgs.msg import Image
from yolo_msgs.msg import Detection
from yolo_msgs.msg import DetectionArray


class TrackingNode(LifecycleNode):
    """
    ROS 2 Lifecycle Node for object tracking.

    This node tracks detected objects across frames using BYTE or BOT-SORT algorithms.
    ByteTrack subscribes only to detections; BoT-SORT retains image synchronization.
    """

    def __init__(self) -> None:
        """
        Initialize the tracking node.

        Declares ROS parameters for tracker configuration.
        """
        super().__init__("tracking_node")

        # ── Patch 0: instrumentation ──
        self._metric_convert_ms_sum = 0.0
        self._metric_tracker_ms_sum = 0.0
        self._metric_input_det_sum = 0
        self._metric_output_track_sum = 0
        self._metric_frame_count = 0
        self._metric_last_log = time.monotonic()
        self._metric_log_interval = 3.0

        # Params
        self.declare_parameter("tracker", "bytetrack.yaml")
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)
        # ORIGINAL camera resolution (not YOLO TensorRT inference imgsz).
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.tracker_type = None
        self.image_sub = None
        self.detections_sub = None
        self._synchronizer = None
        self.cv_bridge = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure lifecycle callback.

        Retrieves parameters, creates the tracker instance, and sets up publishers.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        tracker_name = self.get_parameter("tracker").get_parameter_value().string_value

        self.image_reliability = (
            self.get_parameter("image_reliability").get_parameter_value().integer_value
        )
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        if self.image_width <= 0 or self.image_height <= 0:
            self.get_logger().error("image_width and image_height must be positive")
            return TransitionCallbackReturn.ERROR

        try:
            self.tracker = self.create_tracker(tracker_name)
        except Exception as e:
            self.get_logger().error(f"Failed to create tracker: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

        self._pub = self.create_publisher(DetectionArray, "tracking", 10)

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Subscribe to detections only for ByteTrack; keep RGB for BoT-SORT."""
        self.image_sub = None
        self.detections_sub = None
        self._synchronizer = None

        if self.tracker_type == "bytetrack":
            # DetectionArray already carries original-frame bbox coordinates,
            # class, confidence and acquisition timestamp. Avoid image transport,
            # CvBridge, BGR->RGB and the image/detection sync queue entirely.
            self.detections_sub = self.create_subscription(
                DetectionArray, "detections", self.detections_only_cb, 10
            )
            self.get_logger().info(
                "ByteTrack detections-only mode: no RGB subscription, "
                f"original camera size={self.image_width}x{self.image_height}"
            )
        else:
            # BoT-SORT may require the actual frame for GMC and ReID.
            self.cv_bridge = CvBridge()
            image_qos_profile = QoSProfile(
                reliability=self.image_reliability,
                history=QoSHistoryPolicy.KEEP_LAST,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=1,
            )
            self.image_sub = message_filters.Subscriber(
                self, Image, "image_raw", qos_profile=image_qos_profile
            )
            self.detections_sub = message_filters.Subscriber(
                self, DetectionArray, "detections", qos_profile=10
            )
            self._synchronizer = message_filters.ApproximateTimeSynchronizer(
                (self.image_sub, self.detections_sub), 10, 0.5
            )
            self._synchronizer.registerCallback(self.detections_cb)
            self.get_logger().info("BoT-SORT RGB + detection synchronization mode")

        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Remove subscriptions for the active tracker backend."""
        if self.tracker_type == "bytetrack":
            if self.detections_sub is not None:
                self.destroy_subscription(self.detections_sub)
        else:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub.sub)
            if self.detections_sub is not None:
                self.destroy_subscription(self.detections_sub.sub)
        self.image_sub = None
        self.detections_sub = None
        self._synchronizer = None
        self.cv_bridge = None

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Cleanup lifecycle callback.

        Destroys the tracker instance and cleans up resources.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        del self.tracker

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Shutdown lifecycle callback.

        Performs final cleanup before node shutdown.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        super().on_shutdown(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def create_tracker(self, tracker_yaml: str) -> BaseTrack:
        """
        Create a tracker instance from configuration.

        Loads tracker configuration from YAML file and instantiates the appropriate tracker.

        @param tracker_yaml Path to tracker configuration YAML file
        @return Initialized tracker instance
        """

        TRACKER_MAP = {"bytetrack": BYTETracker, "botsort": BOTSORT}
        check_requirements("lap")  # For linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**YAML.load(tracker))

        assert cfg.tracker_type in [
            "bytetrack",
            "botsort",
        ], f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        self.tracker_type = cfg.tracker_type
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg)
        return tracker

    def detections_only_cb(self, detections_msg: DetectionArray) -> None:
        """ByteTrack: update from detections, no RGB image received or decoded."""
        self._process_detections(
            detections_msg, image_shape=(self.image_height, self.image_width), image=None
        )

    def detections_cb(self, img_msg: Image, detections_msg: DetectionArray) -> None:
        """BoT-SORT legacy path: retain synchronized RGB input for GMC/ReID."""
        t_convert_start = time.perf_counter()
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        convert_ms = (time.perf_counter() - t_convert_start) * 1000.0
        self._process_detections(
            detections_msg, image_shape=(img_msg.height, img_msg.width),
            image=cv_image, convert_ms=convert_ms
        )

    def _process_detections(
        self, detections_msg: DetectionArray, image_shape, image=None,
        convert_ms: float = 0.0
    ) -> None:
        """Publish the same DetectionArray contract for either tracker backend."""
        try:
            stamp = rclpy.time.Time.from_msg(detections_msg.header.stamp)
            input_age_ms = (self.get_clock().now() - stamp).nanoseconds / 1e6
        except Exception:
            input_age_ms = -1.0

        tracked_detections_msg = DetectionArray()
        # Keep original YOLO image timestamp and frame_id for RTMPose and Depth.
        tracked_detections_msg.header = detections_msg.header

        detection_list = []
        for detection in detections_msg.detections:
            bb = detection.bbox
            detection_list.append([
                bb.center.position.x - bb.size.x / 2,
                bb.center.position.y - bb.size.y / 2,
                bb.center.position.x + bb.size.x / 2,
                bb.center.position.y + bb.size.y / 2,
                detection.score,
                detection.class_id,
            ])

        # Empty arrays must also update ByteTrack so lost tracks can expire.
        # Boxes requires a 2-D (N, 6) array even when N == 0.
        det = Boxes(
            np.asarray(detection_list, dtype=np.float32).reshape(-1, 6),
            image_shape,
        )
        t_tracker_start = time.perf_counter()
        tracks = self.tracker.update(det, image)
        tracker_ms = (time.perf_counter() - t_tracker_start) * 1000.0

        for track in tracks:
            source_index = int(track[-1])
            if source_index < 0 or source_index >= len(detections_msg.detections):
                self.get_logger().warn(
                    f"Tracker returned out-of-range detection index {source_index}"
                )
                continue
            tracked_box = Boxes(track[:-1], image_shape)
            tracked_detection = detections_msg.detections[source_index]

            box = tracked_box.xywh[0]
            tracked_detection.bbox.center.position.x = float(box[0])
            tracked_detection.bbox.center.position.y = float(box[1])
            tracked_detection.bbox.size.x = float(box[2])
            tracked_detection.bbox.size.y = float(box[3])
            tracked_detection.id = (
                str(int(tracked_box.id)) if tracked_box.is_track else ""
            )
            tracked_detections_msg.detections.append(tracked_detection)

        self._pub.publish(tracked_detections_msg)

        self._metric_convert_ms_sum += convert_ms
        self._metric_tracker_ms_sum += tracker_ms
        self._metric_input_det_sum += len(detection_list)
        self._metric_output_track_sum += len(tracked_detections_msg.detections)
        self._metric_frame_count += 1

        now = time.monotonic()
        elapsed = now - self._metric_last_log
        if elapsed >= self._metric_log_interval:
            n = self._metric_frame_count
            self.get_logger().info(
                f'[Tracking metrics] fps={n / elapsed:.1f} '
                f'convert={self._metric_convert_ms_sum / n:.1f}ms '
                f'tracker={self._metric_tracker_ms_sum / n:.1f}ms '
                f'in_det={self._metric_input_det_sum / n:.1f} '
                f'out_track={self._metric_output_track_sum / n:.1f} '
                f'input_age={input_age_ms:.0f}ms '
                f'mode={self.tracker_type}'
            )
            self._metric_convert_ms_sum = 0.0
            self._metric_tracker_ms_sum = 0.0
            self._metric_input_det_sum = 0
            self._metric_output_track_sum = 0
            self._metric_frame_count = 0
            self._metric_last_log = now


def main():
    rclpy.init()
    node = TrackingNode()

    # The lifecycle state machine transitions are processed by the executor.
    # We must spin until the state actually changes before triggering the next
    # transition, otherwise trigger_activate() fires while still 'unconfigured'.
    node.trigger_configure()
    while node._state_machine.current_state[1] != 'inactive':
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info("Configure complete, now activating...")

    node.trigger_activate()
    while node._state_machine.current_state[1] != 'active':
        rclpy.spin_once(node, timeout_sec=0.5)
    node.get_logger().info("Activate complete.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

