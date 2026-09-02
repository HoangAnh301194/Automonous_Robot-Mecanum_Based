from collections import defaultdict, deque
from typing import Optional, Sequence, TypeVar

import numpy as np

from .types import PersonPose


COCO_LEFT_SHOULDER = 5
COCO_RIGHT_SHOULDER = 6
COCO_LEFT_ELBOW = 7
COCO_RIGHT_ELBOW = 8
COCO_LEFT_WRIST = 9
COCO_RIGHT_WRIST = 10
COCO_FACE = (0, 1, 2, 3, 4)

Selectable = TypeVar('Selectable')


class NearestTrackSelector:
    def __init__(self, max_people: int = 5, smoothing: float = 0.45):
        self.max_people = max_people
        self.smoothing = smoothing
        self._scores: dict[int, float] = {}

    def select(
        self,
        people: Sequence[Selectable],
        frame_shape: tuple[int, ...],
    ) -> list[Selectable]:
        return self.rank(people, frame_shape)[: self.max_people]

    def rank(
        self,
        people: Sequence[Selectable],
        frame_shape: tuple[int, ...],
    ) -> list[Selectable]:
        frame_height = max(frame_shape[0], 1)
        active_ids: set[int] = set()

        for person in people:
            _, y1, _, y2 = np.asarray(person.bbox, dtype=np.float32)
            bbox_height = max(float(y2 - y1), 1.0)
            raw_score = (
                0.75 * bbox_height / frame_height
                + 0.25 * float(y2) / frame_height
            )

            track_id = int(person.track_id)
            active_ids.add(track_id)
            previous = self._scores.get(track_id, raw_score)
            smoothed = self.smoothing * raw_score + (1.0 - self.smoothing) * previous
            self._scores[track_id] = smoothed
            person.near_score = smoothed

        stale_ids = set(self._scores) - active_ids
        for track_id in stale_ids:
            del self._scores[track_id]

        return sorted(people, key=lambda person: person.near_score, reverse=True)


class RaisedHandRule:
    def __init__(self, keypoint_threshold: float = 0.4, margin_ratio: float = 0.02):
        self.keypoint_threshold = keypoint_threshold
        self.margin_ratio = margin_ratio

    def classify(self, person: PersonPose) -> tuple[Optional[bool], Optional[bool]]:
        left = self._classify_side(
            person,
            shoulder_index=COCO_LEFT_SHOULDER,
            elbow_index=COCO_LEFT_ELBOW,
            wrist_index=COCO_LEFT_WRIST,
        )
        right = self._classify_side(
            person,
            shoulder_index=COCO_RIGHT_SHOULDER,
            elbow_index=COCO_RIGHT_ELBOW,
            wrist_index=COCO_RIGHT_WRIST,
        )
        return left, right

    def _classify_side(
        self,
        person: PersonPose,
        shoulder_index: int,
        elbow_index: int,
        wrist_index: int,
    ) -> Optional[bool]:
        required = [shoulder_index, elbow_index, wrist_index]
        if len(person.keypoints) <= max(required):
            return None

        scores = np.asarray(person.keypoint_scores)
        if len(scores) <= max(required):
            return None
        if np.min(scores[required]) < self.keypoint_threshold:
            return None

        keypoints = np.asarray(person.keypoints)
        wrist_y = float(keypoints[wrist_index, 1])
        bbox_height = max(float(person.bbox[3] - person.bbox[1]), 1.0)

        visible_face_points = [
            index
            for index in COCO_FACE
            if index < len(scores) and scores[index] >= self.keypoint_threshold
        ]
        if not visible_face_points:
            return None

        head_reference_y = min(
            float(keypoints[index, 1]) for index in visible_face_points
        )

        return wrist_y < head_reference_y - self.margin_ratio * bbox_height


class TemporalRaisedHandFilter:
    def __init__(self, history_size: int = 7, on_votes: int = 3, off_votes: int = 3):
        self.on_votes = on_votes
        self.off_votes = off_votes
        self._left_history: dict[int, deque[bool]] = defaultdict(
            lambda: deque(maxlen=history_size)
        )
        self._right_history: dict[int, deque[bool]] = defaultdict(
            lambda: deque(maxlen=history_size)
        )
        self._left_state: dict[int, Optional[bool]] = {}
        self._right_state: dict[int, Optional[bool]] = {}

    def update(
        self,
        track_id: int,
        left: Optional[bool],
        right: Optional[bool],
    ) -> tuple[Optional[bool], Optional[bool]]:
        left_state = self._update_side(
            track_id, left, self._left_history, self._left_state
        )
        right_state = self._update_side(
            track_id, right, self._right_history, self._right_state
        )
        return left_state, right_state

    def retain(self, active_ids: set[int]) -> None:
        for mapping in (
            self._left_history,
            self._right_history,
            self._left_state,
            self._right_state,
        ):
            for track_id in set(mapping) - active_ids:
                del mapping[track_id]

    def _update_side(
        self,
        track_id: int,
        observation: Optional[bool],
        histories: dict[int, deque[bool]],
        states: dict[int, Optional[bool]],
    ) -> Optional[bool]:
        if observation is not None:
            histories[track_id].append(observation)

        history = histories[track_id]
        current_state = states.get(track_id)
        if sum(history) >= self.on_votes:
            current_state = True
        elif len(history) >= self.off_votes and history.count(False) >= self.off_votes:
            current_state = False
        elif current_state is None and observation is not None:
            current_state = observation

        states[track_id] = current_state
        return current_state
