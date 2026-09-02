import argparse
import time
from pathlib import Path

import cv2

from .backends import (
    RTMPoseBackend,
    YOLO11TopDownPoseBackend,
    YOLOPersonDetectionBackend,
)
from .config import AppConfig, load_config
from .export import JsonlPredictionWriter
from .logic import RaisedHandRule, TemporalRaisedHandFilter
from .visualization import draw_people


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Raised-hand detector')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--stage', choices=('detect', 'pose', 'raised-hand'))
    parser.add_argument('--backend', choices=('rtmpose', 'yolo11'))
    parser.add_argument('--source', help='Camera index, video, or stream URL')
    parser.add_argument('--device', help='cuda:0, cuda:1, or cpu')
    parser.add_argument('--max-people', type=int)
    parser.add_argument('--confidence', type=float)
    parser.add_argument('--iou', type=float)
    parser.add_argument('--keypoint-threshold', type=float)
    parser.add_argument('--head-margin-ratio', type=float)
    parser.add_argument('--input-width', type=int)
    parser.add_argument('--input-height', type=int)
    parser.add_argument('--pose-input-width', type=int)
    parser.add_argument('--pose-input-height', type=int)
    parser.add_argument('--pose-bbox-margin-ratio', type=float)
    parser.add_argument('--yolo-pose-model')
    parser.add_argument('--detector-model')
    parser.add_argument('--rtmpose-model')
    parser.add_argument('--full-precision', action='store_true', default=None)
    display_group = parser.add_mutually_exclusive_group()
    display_group.add_argument('--display', dest='display', action='store_true')
    display_group.add_argument('--no-display', dest='display', action='store_false')
    parser.set_defaults(display=None)
    parser.add_argument('--output', help='Annotated MP4 output path')
    parser.add_argument('--jsonl', help='Per-frame prediction JSONL output path')
    return parser


def resolve_config(args: argparse.Namespace) -> AppConfig:
    config = load_config(args.config)

    if args.stage is not None:
        config.stage = args.stage.replace('-', '_')
    if args.backend is not None:
        config.backend = args.backend
    if args.source is not None:
        config.source = parse_source(args.source)
    if args.device is not None:
        config.device = args.device
    if args.max_people is not None:
        config.processing.max_people = args.max_people
    if args.confidence is not None:
        config.processing.confidence = args.confidence
    if args.iou is not None:
        config.processing.iou = args.iou
    if args.keypoint_threshold is not None:
        config.processing.keypoint_threshold = args.keypoint_threshold
    if args.head_margin_ratio is not None:
        config.processing.head_margin_ratio = args.head_margin_ratio
    if args.input_width is not None:
        config.processing.input_width = args.input_width
    if args.input_height is not None:
        config.processing.input_height = args.input_height
    if args.pose_input_width is not None:
        config.processing.pose_input_width = args.pose_input_width
    if args.pose_input_height is not None:
        config.processing.pose_input_height = args.pose_input_height
    if args.pose_bbox_margin_ratio is not None:
        config.processing.pose_bbox_margin_ratio = args.pose_bbox_margin_ratio
    if args.yolo_pose_model is not None:
        config.models.yolo_pose = args.yolo_pose_model
    if args.detector_model is not None:
        config.models.detector = args.detector_model
    if args.rtmpose_model is not None:
        config.models.rtmpose = args.rtmpose_model
    if args.full_precision is not None:
        config.processing.full_precision = args.full_precision
    if args.display is not None:
        config.output.display = args.display
    if args.output is not None:
        config.output.save_video = True
        config.output.path = args.output
    if args.jsonl is not None:
        config.output.save_jsonl = True
        config.output.jsonl_path = args.jsonl

    config.validate()
    return config


def create_backend(config: AppConfig):
    processing = config.processing
    detector_image_size = (processing.input_height, processing.input_width)
    pose_image_size = (
        processing.pose_input_height,
        processing.pose_input_width,
    )

    if config.stage == 'detect':
        return YOLOPersonDetectionBackend(
            model_path=config.models.detector,
            device=config.device,
            confidence=processing.confidence,
            iou=processing.iou,
            image_size=detector_image_size,
            use_half=not processing.full_precision,
        )

    common = {
        'device': config.device,
        'max_people': processing.max_people,
        'confidence': processing.confidence,
        'iou': processing.iou,
        'use_half': not processing.full_precision,
    }

    if config.backend == 'yolo11':
        return YOLO11TopDownPoseBackend(
            detector_model=config.models.detector,
            pose_model=config.models.yolo_pose,
            detector_image_size=detector_image_size,
            pose_image_size=pose_image_size,
            bbox_margin_ratio=processing.pose_bbox_margin_ratio,
            **common,
        )

    return RTMPoseBackend(
        detector_model=config.models.detector,
        pose_model=config.models.rtmpose,
        detector_image_size=detector_image_size,
        pose_model_input_size=(
            processing.pose_input_width,
            processing.pose_input_height,
        ),
        bbox_margin_ratio=processing.pose_bbox_margin_ratio,
        **common,
    )


def run(config: AppConfig) -> None:
    backend = create_backend(config)
    processing = config.processing
    classifier = (
        RaisedHandRule(
            keypoint_threshold=processing.keypoint_threshold,
            margin_ratio=processing.head_margin_ratio,
        )
        if config.stage == 'raised_hand'
        else None
    )
    temporal_filter = (
        TemporalRaisedHandFilter() if config.stage == 'raised_hand' else None
    )
    capture = cv2.VideoCapture(config.source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not capture.isOpened():
        raise RuntimeError(f'Cannot open source: {config.source}')

    writer = None
    jsonl_writer = (
        JsonlPredictionWriter(config.output.jsonl_path)
        if config.output.save_jsonl
        else None
    )
    smoothed_fps = 0.0
    previous_time = time.perf_counter()
    run_started = previous_time
    frame_id = 0

    try:
        while True:
            frame_started = time.perf_counter()
            success, frame = capture.read()
            if not success:
                break
            frame_id += 1

            inference = backend.infer(frame)
            people = inference.people
            classification_started = time.perf_counter()

            if classifier is not None and temporal_filter is not None:
                active_ids = {person.track_id for person in people}
                temporal_filter.retain(active_ids)
                for person in people:
                    left, right = classifier.classify(person)
                    person.left_raised, person.right_raised = temporal_filter.update(
                        person.track_id, left, right
                    )
            classification_ms = (time.perf_counter() - classification_started) * 1000.0

            current_time = time.perf_counter()
            elapsed = max(current_time - previous_time, 1e-6)
            previous_time = current_time
            instant_fps = 1.0 / elapsed
            smoothed_fps = (
                instant_fps
                if smoothed_fps == 0.0
                else 0.90 * smoothed_fps + 0.10 * instant_fps
            )

            annotated = draw_people(
                frame,
                people,
                processing.keypoint_threshold,
                stage=config.stage,
            )
            cv2.putText(
                annotated,
                f'{config.stage} | {inference.backend} | '
                f'{smoothed_fps:.1f} FPS | {config.device}',
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if config.output.save_video:
                if writer is None:
                    writer = create_writer(
                        config.output.path, capture, annotated.shape
                    )
                writer.write(annotated)

            if jsonl_writer is not None:
                timestamp_ms = capture.get(cv2.CAP_PROP_POS_MSEC)
                if timestamp_ms <= 0.0:
                    timestamp_ms = (time.perf_counter() - run_started) * 1000.0
                timings_ms = dict(inference.timings_ms)
                if config.stage == 'raised_hand':
                    timings_ms['classification'] = classification_ms
                timings_ms['frame_total'] = (
                    time.perf_counter() - frame_started
                ) * 1000.0
                jsonl_writer.write(
                    frame_id=frame_id,
                    timestamp_ms=timestamp_ms,
                    stage=config.stage,
                    backend=inference.backend,
                    people=people,
                    timings_ms=timings_ms,
                    processing_fps=smoothed_fps,
                    keypoint_threshold=processing.keypoint_threshold,
                    frame_shape=frame.shape,
                )

            if config.output.display:
                cv2.imshow('Raised Hand Detection', annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
    finally:
        capture.release()
        if writer is not None:
            writer.release()
        if jsonl_writer is not None:
            jsonl_writer.close()
        cv2.destroyAllWindows()


def parse_source(source):
    if isinstance(source, str) and source.isdigit():
        return int(source)
    return source


def create_writer(output: str, capture, frame_shape) -> cv2.VideoWriter:
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frame_shape[:2]
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    output_fps = source_fps if source_fps > 0 else 15.0
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*'mp4v'),
        output_fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f'Cannot create output video: {output}')
    return writer
