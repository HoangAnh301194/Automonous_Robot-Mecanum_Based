from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(slots=True)
class TrackedBox:
    bbox: np.ndarray
    confidence: float
    track_id: int
    near_score: float = 0.0


@dataclass(slots=True)
class PersonPose:
    bbox: np.ndarray
    confidence: float
    track_id: int
    keypoints: np.ndarray
    keypoint_scores: np.ndarray
    near_score: float = 0.0
    left_raised: Optional[bool] = None
    right_raised: Optional[bool] = None

    @property
    def state(self) -> str:
        if self.left_raised and self.right_raised:
            return 'BOTH'
        if self.left_raised:
            return 'LEFT'
        if self.right_raised:
            return 'RIGHT'
        if self.left_raised is None or self.right_raised is None:
            return 'UNKNOWN'
        return 'NONE'


@dataclass(slots=True)
class InferenceResult:
    people: list[PersonPose]
    backend: str
    timings_ms: dict[str, float] = field(default_factory=dict)
