import time
from typing import List, Dict
from cv_bridge import CvBridge

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import torch
from ultralytics import YOLO, YOLOWorld, YOLOE
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from std_srvs.srv import SetBool
from sensor_msgs.msg import Image
from yolo_msgs.msg import Point2D
from yolo_msgs.msg import BoundingBox2D
from yolo_msgs.msg import Mask
from yolo_msgs.msg import KeyPoint2D
from yolo_msgs.msg import KeyPoint2DArray
from yolo_msgs.msg import Detection
from yolo_msgs.msg import DetectionArray
from yolo_msgs.srv import SetClasses


class YoloNode(LifecycleNode):
    """
    ROS 2 Lifecycle Node for YOLO object detection.

    This node provides real-time object detection using various YOLO models
    (YOLO, YOLOWorld, YOLOE). It subscribes to image topics and publishes
    detection results including bounding boxes, masks, and keypoints.
    """

    def __init__(self) -> None:
        """
        Initialize the YOLO node.

        Declares all ROS parameters for model configuration and inference settings.
        """
        super().__init__("yolo_node")

        # ── Patch 0: instrumentation ──
        self._metric_predict_ms_sum = 0.0
        self._metric_parse_ms_sum = 0.0
        self._metric_det_count_sum = 0
        self._metric_frame_count = 0
        self._metric_last_log = time.monotonic()
        self._metric_log_interval = 3.0  # seconds

        # Params
        self.declare_parameter("model_type", "YOLO")
        self.declare_parameter("model", "yolov8m.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("fuse_model", False)
        self.declare_parameter("yolo_encoding", "bgr8")
        self.declare_parameter("enable", True)
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT)

        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("iou", 0.5)
        self.declare_parameter("imgsz_height", 640)
        self.declare_parameter("imgsz_width", 640)
        self.declare_parameter("half", False)
        self.declare_parameter("max_det", 300)
        self.declare_parameter("augment", False)
        self.declare_parameter("agnostic_nms", False)
        self.declare_parameter("retina_masks", False)
        self.declare_parameter("classes", "")

        self.type_to_model = {"YOLO": YOLO, "World": YOLOWorld, "YOLOE": YOLOE}

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure lifecycle callback.

        Retrieves all parameters and sets up publishers and QoS profiles.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        # Model params
        self.model_type = (
            self.get_parameter("model_type").get_parameter_value().string_value
        )
        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.fuse_model = (
            self.get_parameter("fuse_model").get_parameter_value().bool_value
        )
        self.yolo_encoding = (
            self.get_parameter("yolo_encoding").get_parameter_value().string_value
        )

        # Inference params
        self.threshold = (
            self.get_parameter("threshold").get_parameter_value().double_value
        )
        self.iou = self.get_parameter("iou").get_parameter_value().double_value
        self.imgsz_height = (
            self.get_parameter("imgsz_height").get_parameter_value().integer_value
        )
        self.imgsz_width = (
            self.get_parameter("imgsz_width").get_parameter_value().integer_value
        )
        self.half = self.get_parameter("half").get_parameter_value().bool_value
        self.max_det = self.get_parameter("max_det").get_parameter_value().integer_value
        self.augment = self.get_parameter("augment").get_parameter_value().bool_value
        self.agnostic_nms = (
            self.get_parameter("agnostic_nms").get_parameter_value().bool_value
        )
        self.retina_masks = (
            self.get_parameter("retina_masks").get_parameter_value().bool_value
        )

        # Classes filter (Patch 1)
        classes_value = (
            self.get_parameter("classes")
            .get_parameter_value()
            .string_value
            .strip()
        )
        try:
            self.classes = (
                [int(item.strip()) for item in classes_value.split(",")]
                if classes_value
                else None
            )
        except ValueError:
            self.get_logger().error(
                f"Invalid classes parameter: '{classes_value}'"
            )
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(f"Classes filter: {self.classes}")

        # ROS params
        self.enable = self.get_parameter("enable").get_parameter_value().bool_value
        self.reliability = (
            self.get_parameter("image_reliability").get_parameter_value().integer_value
        )

        # Detection pub
        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._pub = self.create_lifecycle_publisher(DetectionArray, "detections", 10)
        self.cv_bridge = CvBridge()

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activate lifecycle callback.

        Loads the YOLO model, optionally fuses it, and creates subscriptions and services.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Activating...")

        try:
            self.get_logger().info(f"Loading model: {self.model}")
            if 'pose' in self.model:
                self.yolo = self.type_to_model[self.model_type](self.model, task='pose')
            else:
                self.yolo = self.type_to_model[self.model_type](self.model)
            if self.model.endswith('.pt'):
                self.get_logger().info("Moving model to device...")
                self.yolo.to(self.device)
            self.get_logger().info("Model loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"Model file '{self.model}' does not exists")
            return TransitionCallbackReturn.ERROR

        # YOLOE does not support fusing
        if self.fuse_model and (
            isinstance(self.yolo, YOLO) or isinstance(self.yolo, YOLOWorld)
        ):
            try:
                self.get_logger().info("Trying to fuse model...")
                self.yolo.fuse()
            except TypeError as e:
                self.get_logger().warn(f"Error while fuse: {e}")

        self.get_logger().info("Creating enable service...")
        self._enable_srv = self.create_service(SetBool, "enable", self.enable_cb)

        if isinstance(self.yolo, YOLOWorld):
            self._set_classes_srv = self.create_service(
                SetClasses, "set_classes", self.set_classes_cb
            )

        self.get_logger().info("Creating image subscription...")
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb, self.image_qos_profile
        )

        self.get_logger().info("Calling super().on_activate...")
        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Deactivate lifecycle callback.

        Destroys the YOLO model, clears CUDA cache if applicable, and cleans up services and subscriptions.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")

        del self.yolo
        if "cuda" in self.device:
            self.get_logger().info("Clearing CUDA cache")
            torch.cuda.empty_cache()

        self.destroy_service(self._enable_srv)
        self._enable_srv = None

        if isinstance(self.yolo, YOLOWorld):
            self.destroy_service(self._set_classes_srv)
            self._set_classes_srv = None

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Cleanup lifecycle callback.

        Destroys publishers and cleans up resources.

        @param state Current lifecycle state
        @return Transition callback return status
        """
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        self.destroy_publisher(self._pub)
        del self.image_qos_profile

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

    def enable_cb(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """
        Service callback to enable or disable detection.

        @param request Service request containing enable/disable flag
        @param response Service response
        @return Service response with success status
        """
        self.enable = request.data
        response.success = True
        return response

    def parse_hypothesis(self, results: Results) -> List[Dict]:
        """
        Parse detection hypotheses from YOLO results.

        Extracts class IDs, class names, and confidence scores from detection results.

        @param results YOLO detection results
        @return List of dictionaries containing class information and scores
        """

        hypothesis_list = []

        if results.boxes:
            box_data: Boxes
            for box_data in results.boxes:
                hypothesis = {
                    "class_id": int(box_data.cls),
                    "class_name": self.yolo.names[int(box_data.cls)],
                    "score": float(box_data.conf),
                }
                hypothesis_list.append(hypothesis)

        elif results.obb:
            for i in range(results.obb.cls.shape[0]):
                hypothesis = {
                    "class_id": int(results.obb.cls[i]),
                    "class_name": self.yolo.names[int(results.obb.cls[i])],
                    "score": float(results.obb.conf[i]),
                }
                hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:
        """
        Parse bounding boxes from YOLO results.

        Converts YOLO bounding box format to ROS BoundingBox2D messages.
        Supports both regular boxes and oriented bounding boxes (OBB).

        @param results YOLO detection results
        @return List of BoundingBox2D messages
        """

        boxes_list = []

        if results.boxes:
            box_data: Boxes
            for box_data in results.boxes:

                msg = BoundingBox2D()

                # Get boxes values
                box = box_data.xywh[0]
                msg.center.position.x = float(box[0])
                msg.center.position.y = float(box[1])
                msg.size.x = float(box[2])
                msg.size.y = float(box[3])

                # Append msg
                boxes_list.append(msg)

        elif results.obb:
            for i in range(results.obb.cls.shape[0]):
                msg = BoundingBox2D()

                # Get boxes values
                box = results.obb.xywhr[i]
                msg.center.position.x = float(box[0])
                msg.center.position.y = float(box[1])
                msg.center.theta = float(box[4])
                msg.size.x = float(box[2])
                msg.size.y = float(box[3])

                # Append msg
                boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:
        """
        Parse segmentation masks from YOLO results.

        Converts YOLO mask format to ROS Mask messages containing polygon points.

        @param results YOLO detection results
        @return List of Mask messages
        """

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [
                create_point2d(float(ele[0]), float(ele[1]))
                for ele in mask.xy[0].tolist()
            ]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:
        """
        Parse keypoints from YOLO results.

        Extracts keypoint positions and confidence scores, filtering by threshold.

        @param results YOLO detection results
        @return List of KeyPoint2DArray messages
        """

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: Image) -> None:
        """
        Image callback for processing detections.

        Receives images, runs YOLO inference, parses results, and publishes detections.

        @param msg Image message to process
        """

        if self.enable:

            # ── Patch 0: measure frame age ──
            try:
                stamp = rclpy.time.Time.from_msg(msg.header.stamp)
                frame_age_ms = (self.get_clock().now() - stamp).nanoseconds / 1e6
            except Exception:
                frame_age_ms = -1.0

            # Convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding=self.yolo_encoding
            )

            # ── Patch 0: measure predict ──
            t_predict_start = time.perf_counter()
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                iou=self.iou,
                imgsz=(self.imgsz_height, self.imgsz_width),
                half=self.half,
                max_det=self.max_det,
                augment=self.augment,
                agnostic_nms=self.agnostic_nms,
                retina_masks=self.retina_masks,
                classes=self.classes,
                device=self.device,
            )
            predict_ms = (time.perf_counter() - t_predict_start) * 1000.0

            # ── Patch 0: measure GPU→CPU parse ──
            t_parse_start = time.perf_counter()
            results: Results = results[0].cpu()

            if results.boxes or results.obb:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # Create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):

                aux_msg = Detection()

                if (results.boxes or results.obb) and hypothesis and boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks and masks:
                    aux_msg.mask = masks[i]

                if results.keypoints and keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            parse_ms = (time.perf_counter() - t_parse_start) * 1000.0
            det_count = len(detections_msg.detections)

            # Publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)

            # ── Patch 0: accumulate and log throttled ──
            self._metric_predict_ms_sum += predict_ms
            self._metric_parse_ms_sum += parse_ms
            self._metric_det_count_sum += det_count
            self._metric_frame_count += 1

            now = time.monotonic()
            elapsed = now - self._metric_last_log
            if elapsed >= self._metric_log_interval:
                n = self._metric_frame_count
                avg_predict = self._metric_predict_ms_sum / n
                avg_parse = self._metric_parse_ms_sum / n
                avg_det = self._metric_det_count_sum / n
                fps = n / elapsed
                self.get_logger().info(
                    f'[YOLO metrics] fps={fps:.1f} '
                    f'predict={avg_predict:.1f}ms '
                    f'parse={avg_parse:.1f}ms '
                    f'det_count={avg_det:.1f} '
                    f'frame_age={frame_age_ms:.0f}ms'
                )
                self._metric_predict_ms_sum = 0.0
                self._metric_parse_ms_sum = 0.0
                self._metric_det_count_sum = 0
                self._metric_frame_count = 0
                self._metric_last_log = now

            del results
            del cv_image

    def set_classes_cb(
        self,
        req: SetClasses.Request,
        res: SetClasses.Response,
    ) -> SetClasses.Response:
        """
        Service callback to set detection classes (YOLOWorld only).

        Updates the classes that the model should detect.

        @param req Service request containing list of class names
        @param res Service response
        @return Service response
        """
        self.get_logger().info(f"Setting classes: {req.classes}")
        self.yolo.set_classes(req.classes)
        self.get_logger().info(f"New classes: {self.yolo.names}")
        return res


def main():
    rclpy.init()
    node = YoloNode()
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
