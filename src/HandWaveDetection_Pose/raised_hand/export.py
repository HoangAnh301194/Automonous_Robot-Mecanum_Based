import json
from pathlib import Path
from typing import TextIO

import numpy as np

from .types import PersonPose


COCO_KEYPOINT_NAMES = (
    'nose',
    'left_eye',
    'right_eye',
    'left_ear',
    'right_ear',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_hip',
    'right_hip',
    'left_knee',
    'right_knee',
    'left_ankle',
    'right_ankle',
)


class JsonlPredictionWriter:
    def __init__(self, path: str):
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file: TextIO = output_path.open('w', encoding='utf-8')

    def write(
        self,
        *,
        frame_id: int,
        timestamp_ms: float,
        stage: str,
        backend: str,
        people: list[PersonPose],
        timings_ms: dict[str, float],
        processing_fps: float,
        keypoint_threshold: float,
        frame_shape: tuple[int, ...],
    ) -> None:
        frame_height, frame_width = frame_shape[:2]
        payload = {
            'schema_version': 1,
            'frame_id': frame_id,
            'timestamp_ms': round(float(timestamp_ms), 3),
            'stage': stage,
            'backend': backend,
            'keypoint_format': 'coco17',
            'frame_size': {'width': frame_width, 'height': frame_height},
            'processing_fps': round(float(processing_fps), 3),
            'timings_ms': {
                name: round(float(value), 3) for name, value in timings_ms.items()
            },
            'people': [
                serialize_person(person, rank, stage, keypoint_threshold)
                for rank, person in enumerate(people, start=1)
            ],
        }
        self._file.write(json.dumps(payload, ensure_ascii=False) + '\n')

    def close(self) -> None:
        self._file.close()


def serialize_person(
    person: PersonPose,
    rank: int,
    stage: str,
    keypoint_threshold: float,
) -> dict:
    keypoints = np.asarray(person.keypoints, dtype=np.float32)
    scores = np.asarray(person.keypoint_scores, dtype=np.float32)
    serialized_keypoints = []
    for index, keypoint in enumerate(keypoints):
        score = float(scores[index]) if index < len(scores) else 0.0
        serialized_keypoints.append(
            {
                'index': index,
                'name': (
                    COCO_KEYPOINT_NAMES[index]
                    if index < len(COCO_KEYPOINT_NAMES)
                    else f'keypoint_{index}'
                ),
                'x': round(float(keypoint[0]), 3),
                'y': round(float(keypoint[1]), 3),
                'score': round(score, 5),
            }
        )

    result = {
        'rank': rank,
        'track_id': int(person.track_id),
        'bbox': [round(float(value), 3) for value in person.bbox],
        'detector_confidence': round(float(person.confidence), 5),
        'near_score': round(float(person.near_score), 5),
        'keypoint_count': len(serialized_keypoints),
        'valid_keypoint_count': int(np.sum(scores >= keypoint_threshold)),
        'keypoints': serialized_keypoints,
    }
    if stage == 'raised_hand':
        result.update(
            {
                'left_raised': person.left_raised,
                'right_raised': person.right_raised,
                'state': person.state,
            }
        )
    return result
