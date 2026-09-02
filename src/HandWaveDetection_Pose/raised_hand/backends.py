from abc import ABC, abstractmethod
import time

import numpy as np

from .logic import NearestTrackSelector
from .types import InferenceResult, PersonPose, TrackedBox


DEFAULT_RTMPOSE_MODEL = (
    'https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/'
    'onnx_sdk/rtmpose-s_simcc-body7_pt-body7_420e-256x192-'
    'acd4a1ef_20230504.zip'
)


class PoseBackend(ABC):
    @abstractmethod
    def infer(self, frame: np.ndarray) -> InferenceResult:
        raise NotImplementedError


class YOLOPersonDetectionBackend(PoseBackend):
    def __init__(
        self,
        model_path: str,
        device: str,
        confidence: float,
        iou: float,
        image_size: tuple[int, int],
        use_half: bool,
    ):
        from ultralytics import YOLO

        self.device = normalize_device(device)
        validate_torch_device(self.device)
        self.model = YOLO(model_path)
        self.confidence = confidence
        self.iou = iou
        self.image_size = image_size
        self.use_half = use_half and self.device.startswith('cuda')
        self.selector = NearestTrackSelector(max_people=1)

    def infer(self, frame: np.ndarray) -> InferenceResult:
        total_started = time.perf_counter()
        detector_started = time.perf_counter()
        results = self.model.track(
            frame,
            persist=True,
            tracker='bytetrack.yaml',
            classes=[0],
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.image_size,
            device=self.device,
            quantize=16 if self.use_half else None,
            verbose=False,
        )
        detector_ms = elapsed_ms(detector_started)

        tracked_boxes = extract_tracked_boxes(results[0]) if results else []
        ranked = self.selector.rank(tracked_boxes, frame.shape)
        people = [
            PersonPose(
                bbox=tracked_box.bbox,
                confidence=tracked_box.confidence,
                track_id=tracked_box.track_id,
                keypoints=np.empty((0, 2), dtype=np.float32),
                keypoint_scores=np.empty((0,), dtype=np.float32),
                near_score=tracked_box.near_score,
            )
            for tracked_box in ranked
        ]
        return InferenceResult(
            people=people,
            backend='yolo11_detect',
            timings_ms={
                'detector': detector_ms,
                'backend_total': elapsed_ms(total_started),
            },
        )


class YOLO11TopDownPoseBackend(PoseBackend):
    def __init__(
        self,
        detector_model: str,
        pose_model: str,
        device: str,
        max_people: int,
        confidence: float,
        iou: float,
        detector_image_size: tuple[int, int],
        pose_image_size: tuple[int, int],
        bbox_margin_ratio: float,
        use_half: bool,
    ):
        from ultralytics import YOLO

        self.device = normalize_device(device)
        validate_torch_device(self.device)
        self.detector = YOLO(detector_model)
        self.pose = YOLO(pose_model)
        self.confidence = confidence
        self.iou = iou
        self.detector_image_size = detector_image_size
        self.pose_image_size = pose_image_size
        self.bbox_margin_ratio = bbox_margin_ratio
        self.use_half = use_half and self.device.startswith('cuda')
        self.selector = NearestTrackSelector(max_people=max_people)

    def infer(self, frame: np.ndarray) -> InferenceResult:
        total_started = time.perf_counter()
        detector_started = time.perf_counter()
        results = self.detector.track(
            frame,
            persist=True,
            tracker='bytetrack.yaml',
            classes=[0],
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.detector_image_size,
            device=self.device,
            quantize=16 if self.use_half else None,
            verbose=False,
        )
        detector_ms = elapsed_ms(detector_started)
        if not results:
            return InferenceResult(
                people=[],
                backend='yolo11_detect_yolo11_pose',
                timings_ms={
                    'detector': detector_ms,
                    'pose': 0.0,
                    'backend_total': elapsed_ms(total_started),
                },
            )

        tracked_boxes = extract_tracked_boxes(results[0])
        selected = self.selector.select(tracked_boxes, frame.shape)
        if not selected:
            return InferenceResult(
                people=[],
                backend='yolo11_detect_yolo11_pose',
                timings_ms={
                    'detector': detector_ms,
                    'pose': 0.0,
                    'backend_total': elapsed_ms(total_started),
                },
            )

        pose_inputs = []
        pose_contexts = []
        for tracked_box in selected:
            expanded_bbox = expand_bbox(
                tracked_box.bbox,
                frame.shape,
                self.bbox_margin_ratio,
            )
            crop, crop_bbox = crop_frame(frame, expanded_bbox)
            if crop.size == 0:
                continue
            pose_inputs.append(crop)
            pose_contexts.append((tracked_box, crop_bbox))

        pose_started = time.perf_counter()
        pose_results = (
            self.pose.predict(
                pose_inputs,
                classes=[0],
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.pose_image_size,
                device=self.device,
                quantize=16 if self.use_half else None,
                verbose=False,
            )
            if pose_inputs
            else []
        )
        pose_ms = elapsed_ms(pose_started)

        people = []
        for index, (tracked_box, crop_bbox) in enumerate(pose_contexts):
            keypoints, keypoint_scores = empty_pose()
            if index < len(pose_results):
                keypoints, keypoint_scores = extract_yolo_crop_pose(
                    pose_results[index],
                    tracked_box.bbox,
                    crop_bbox,
                )
            people.append(
                PersonPose(
                    bbox=tracked_box.bbox,
                    confidence=tracked_box.confidence,
                    track_id=tracked_box.track_id,
                    keypoints=keypoints,
                    keypoint_scores=keypoint_scores,
                    near_score=tracked_box.near_score,
                )
            )

        return InferenceResult(
            people=people,
            backend='yolo11_detect_yolo11_pose',
            timings_ms={
                'detector': detector_ms,
                'pose': pose_ms,
                'backend_total': elapsed_ms(total_started),
            },
        )


class RTMPoseBackend(PoseBackend):
    def __init__(
        self,
        detector_model: str,
        pose_model: str,
        device: str,
        max_people: int,
        confidence: float,
        iou: float,
        detector_image_size: tuple[int, int],
        pose_model_input_size: tuple[int, int],
        bbox_margin_ratio: float,
        use_half: bool,
    ):
        from ultralytics import YOLO

        from .rtmpose_batch import BatchedRTMPose

        self.device = normalize_device(device)
        validate_torch_device(self.device)
        validate_onnxruntime_device(self.device)

        self.detector = YOLO(detector_model)
        self.pose = BatchedRTMPose(
            onnx_model=pose_model,
            model_input_size=pose_model_input_size,
            backend='onnxruntime',
            device=self.device,
        )
        self.confidence = confidence
        self.iou = iou
        self.detector_image_size = detector_image_size
        self.bbox_margin_ratio = bbox_margin_ratio
        self.use_half = use_half and self.device.startswith('cuda')
        self.selector = NearestTrackSelector(max_people=max_people)

    def infer(self, frame: np.ndarray) -> InferenceResult:
        total_started = time.perf_counter()
        detector_started = time.perf_counter()
        results = self.detector.track(
            frame,
            persist=True,
            tracker='bytetrack.yaml',
            classes=[0],
            conf=self.confidence,
            iou=self.iou,
            imgsz=self.detector_image_size,
            device=self.device,
            quantize=16 if self.use_half else None,
            verbose=False,
        )
        detector_ms = elapsed_ms(detector_started)
        if not results:
            return InferenceResult(
                people=[],
                backend='yolo11_detect_rtmpose',
                timings_ms={
                    'detector': detector_ms,
                    'pose': 0.0,
                    'backend_total': elapsed_ms(total_started),
                },
            )

        tracked_boxes = extract_tracked_boxes(results[0])
        selected = self.selector.select(tracked_boxes, frame.shape)
        if not selected:
            return InferenceResult(
                people=[],
                backend='yolo11_detect_rtmpose',
                timings_ms={
                    'detector': detector_ms,
                    'pose': 0.0,
                    'backend_total': elapsed_ms(total_started),
                },
            )

        bboxes = [
            expand_bbox(
                person.bbox,
                frame.shape,
                self.bbox_margin_ratio,
            ).tolist()
            for person in selected
        ]
        pose_started = time.perf_counter()
        keypoints, keypoint_scores = self.pose(frame, bboxes=bboxes)
        pose_ms = elapsed_ms(pose_started)
        keypoints = np.asarray(keypoints, dtype=np.float32)
        keypoint_scores = np.asarray(keypoint_scores, dtype=np.float32)

        if keypoints.ndim == 2:
            keypoints = keypoints[None, ...]
        if keypoint_scores.ndim == 1:
            keypoint_scores = keypoint_scores[None, ...]

        people = []
        for index, tracked_box in enumerate(selected):
            person_keypoints, person_scores = empty_pose()
            if index < len(keypoints):
                person_keypoints = keypoints[index]
            if index < len(keypoint_scores):
                person_scores = keypoint_scores[index]
            people.append(
                PersonPose(
                    bbox=tracked_box.bbox,
                    confidence=tracked_box.confidence,
                    track_id=tracked_box.track_id,
                    keypoints=person_keypoints,
                    keypoint_scores=person_scores,
                    near_score=tracked_box.near_score,
                )
            )
        return InferenceResult(
            people=people,
            backend='yolo11_detect_rtmpose',
            timings_ms={
                'detector': detector_ms,
                'pose': pose_ms,
                'backend_total': elapsed_ms(total_started),
            },
        )


def expand_bbox(
    bbox: np.ndarray,
    frame_shape: tuple[int, ...],
    margin_ratio: float,
) -> np.ndarray:
    frame_height, frame_width = frame_shape[:2]
    x1, y1, x2, y2 = np.asarray(bbox, dtype=np.float32)
    margin_x = max(float(x2 - x1), 1.0) * margin_ratio
    margin_y = max(float(y2 - y1), 1.0) * margin_ratio
    return np.asarray(
        [
            np.clip(x1 - margin_x, 0.0, float(frame_width)),
            np.clip(y1 - margin_y, 0.0, float(frame_height)),
            np.clip(x2 + margin_x, 0.0, float(frame_width)),
            np.clip(y2 + margin_y, 0.0, float(frame_height)),
        ],
        dtype=np.float32,
    )


def crop_frame(
    frame: np.ndarray,
    bbox: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    x1, y1, x2, y2 = np.asarray(bbox, dtype=np.float32)
    crop_bbox = np.asarray(
        [np.floor(x1), np.floor(y1), np.ceil(x2), np.ceil(y2)],
        dtype=np.int32,
    )
    crop_x1, crop_y1, crop_x2, crop_y2 = crop_bbox
    crop = np.ascontiguousarray(frame[crop_y1:crop_y2, crop_x1:crop_x2])
    return crop, crop_bbox.astype(np.float32)


def extract_yolo_crop_pose(
    result,
    target_bbox: np.ndarray,
    crop_bbox: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
        return empty_pose()

    pose_boxes = result.boxes.xyxy.cpu().numpy()
    pose_confidences = result.boxes.conf.cpu().numpy()
    crop_x1, crop_y1, _, _ = crop_bbox
    target_local_bbox = np.asarray(target_bbox, dtype=np.float32) - np.asarray(
        [crop_x1, crop_y1, crop_x1, crop_y1],
        dtype=np.float32,
    )
    pose_index = select_matching_pose(
        pose_boxes,
        pose_confidences,
        target_local_bbox,
    )

    all_keypoints = result.keypoints.xy.cpu().numpy()
    if pose_index >= len(all_keypoints):
        return empty_pose()
    if result.keypoints.conf is not None:
        all_scores = result.keypoints.conf.cpu().numpy()
    else:
        all_scores = result.keypoints.data[..., 2].cpu().numpy()
    if pose_index >= len(all_scores):
        return empty_pose()

    keypoints = np.asarray(all_keypoints[pose_index], dtype=np.float32).copy()
    keypoints[:, 0] += crop_x1
    keypoints[:, 1] += crop_y1
    scores = np.asarray(all_scores[pose_index], dtype=np.float32)
    return keypoints, scores


def select_matching_pose(
    pose_boxes: np.ndarray,
    pose_confidences: np.ndarray,
    target_bbox: np.ndarray,
) -> int:
    intersection_x1 = np.maximum(pose_boxes[:, 0], target_bbox[0])
    intersection_y1 = np.maximum(pose_boxes[:, 1], target_bbox[1])
    intersection_x2 = np.minimum(pose_boxes[:, 2], target_bbox[2])
    intersection_y2 = np.minimum(pose_boxes[:, 3], target_bbox[3])
    intersection_width = np.maximum(intersection_x2 - intersection_x1, 0.0)
    intersection_height = np.maximum(intersection_y2 - intersection_y1, 0.0)
    intersection_area = intersection_width * intersection_height

    pose_areas = np.maximum(pose_boxes[:, 2] - pose_boxes[:, 0], 0.0) * np.maximum(
        pose_boxes[:, 3] - pose_boxes[:, 1],
        0.0,
    )
    target_area = max(
        float(target_bbox[2] - target_bbox[0]),
        0.0,
    ) * max(float(target_bbox[3] - target_bbox[1]), 0.0)
    union_area = np.maximum(pose_areas + target_area - intersection_area, 1e-6)
    matching_scores = intersection_area / union_area + 0.05 * pose_confidences
    return int(np.argmax(matching_scores))


def empty_pose() -> tuple[np.ndarray, np.ndarray]:
    return (
        np.empty((0, 2), dtype=np.float32),
        np.empty((0,), dtype=np.float32),
    )


def extract_tracked_boxes(result) -> list[TrackedBox]:
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return []

    coordinates = boxes.xyxy.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    track_ids = boxes.id.cpu().numpy() if boxes.id is not None else None

    tracked_boxes = []
    for index, bbox in enumerate(coordinates):
        track_id = int(track_ids[index]) if track_ids is not None else -(index + 1)
        tracked_boxes.append(
            TrackedBox(
                bbox=np.asarray(bbox, dtype=np.float32),
                confidence=float(confidences[index]),
                track_id=track_id,
            )
        )
    return tracked_boxes


def normalize_device(device: str) -> str:
    normalized = device.strip().lower()
    if normalized.isdigit():
        return f'cuda:{normalized}'
    if normalized == 'cuda':
        return 'cuda:0'
    return normalized


def validate_torch_device(device: str) -> None:
    if not device.startswith('cuda'):
        return

    import torch

    if not torch.cuda.is_available():
        raise RuntimeError(
            'CUDA requested but PyTorch cannot access the GPU. '
            'Install a CUDA-enabled PyTorch build.'
        )


def validate_onnxruntime_device(device: str) -> None:
    if not device.startswith('cuda'):
        return

    import onnxruntime as ort

    providers = ort.get_available_providers()
    if 'CUDAExecutionProvider' not in providers:
        raise RuntimeError(
            'RTMPose GPU requires ONNX Runtime CUDAExecutionProvider. '
            f'Available providers: {providers}'
        )


def elapsed_ms(started: float) -> float:
    return (time.perf_counter() - started) * 1000.0
