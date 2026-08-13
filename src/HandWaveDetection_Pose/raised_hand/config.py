from dataclasses import dataclass, field
from pathlib import Path
from typing import Union

import yaml


Source = Union[int, str]


@dataclass(slots=True)
class ModelSettings:
    yolo_pose: str = 'yolo11n-pose.pt'
    detector: str = 'yolo11n.pt'
    rtmpose: str = (
        'https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/'
        'onnx_sdk/rtmpose-s_simcc-body7_pt-body7_420e-256x192-'
        'acd4a1ef_20230504.zip'
    )


@dataclass(slots=True)
class ProcessingSettings:
    max_people: int = 5
    input_width: int = 640
    input_height: int = 384
    pose_input_width: int = 192
    pose_input_height: int = 256
    pose_bbox_margin_ratio: float = 0.12
    confidence: float = 0.25
    iou: float = 0.50
    keypoint_threshold: float = 0.40
    head_margin_ratio: float = 0.02
    full_precision: bool = False


@dataclass(slots=True)
class OutputSettings:
    display: bool = True
    save_video: bool = False
    path: str = 'outputs/result.mp4'
    save_jsonl: bool = False
    jsonl_path: str = 'outputs/predictions.jsonl'


@dataclass(slots=True)
class AppConfig:
    stage: str = 'raised_hand'
    backend: str = 'yolo11'
    device: str = 'cpu'
    source: Source = 0
    models: ModelSettings = field(default_factory=ModelSettings)
    processing: ProcessingSettings = field(default_factory=ProcessingSettings)
    output: OutputSettings = field(default_factory=OutputSettings)

    def validate(self) -> None:
        if self.stage not in {'detect', 'pose', 'raised_hand'}:
            raise ValueError(f'Unsupported stage: {self.stage}')
        if self.backend not in {'yolo11', 'rtmpose'}:
            raise ValueError(f'Unsupported backend: {self.backend}')
        if self.processing.max_people < 1:
            raise ValueError('max_people must be at least 1')
        for name, value in (
            ('input_width', self.processing.input_width),
            ('input_height', self.processing.input_height),
            ('pose_input_width', self.processing.pose_input_width),
            ('pose_input_height', self.processing.pose_input_height),
        ):
            if value < 1:
                raise ValueError(f'{name} must be positive')
        for name, value in (
            ('confidence', self.processing.confidence),
            ('iou', self.processing.iou),
            ('keypoint_threshold', self.processing.keypoint_threshold),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f'{name} must be between 0 and 1')
        if self.processing.head_margin_ratio < 0.0:
            raise ValueError('head_margin_ratio must not be negative')
        if self.processing.pose_bbox_margin_ratio < 0.0:
            raise ValueError('pose_bbox_margin_ratio must not be negative')
        if self.output.save_video and not self.output.path:
            raise ValueError('output path is required when save_video is enabled')
        if self.output.save_jsonl and not self.output.jsonl_path:
            raise ValueError('jsonl_path is required when save_jsonl is enabled')


def load_config(path: str) -> AppConfig:
    config_path = Path(path)
    if not config_path.is_file():
        raise FileNotFoundError(f'Config file not found: {config_path}')

    data = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}
    models_data = data.get('models') or {}
    processing_data = data.get('processing') or {}
    output_data = data.get('output') or {}

    config = AppConfig(
        stage=str(data.get('stage', 'raised_hand')).replace('-', '_'),
        backend=str(data.get('backend', 'yolo11')),
        device=str(data.get('device', 'cpu')),
        source=_parse_source(data.get('source', 0)),
        models=ModelSettings(**models_data),
        processing=ProcessingSettings(**processing_data),
        output=OutputSettings(**output_data),
    )
    config.validate()
    return config


def _parse_source(source) -> Source:
    if isinstance(source, str) and source.isdigit():
        return int(source)
    return source
