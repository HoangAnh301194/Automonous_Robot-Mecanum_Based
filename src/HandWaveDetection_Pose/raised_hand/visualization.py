import cv2
import numpy as np

from .types import PersonPose


SKELETON = (
    (5, 6),
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (5, 11),
    (6, 12),
    (11, 12),
)

STATE_COLORS = {
    'UNKNOWN': (128, 128, 128),
    'NONE': (80, 200, 80),
    'LEFT': (0, 165, 255),
    'RIGHT': (255, 180, 0),
    'BOTH': (0, 0, 255),
}

STAGE_COLORS = {
    'detect': (0, 220, 255),
    'pose': (255, 120, 80),
}


def draw_people(
    frame: np.ndarray,
    people: list[PersonPose],
    keypoint_threshold: float,
    stage: str = 'raised_hand',
) -> np.ndarray:
    output = frame.copy()
    for rank, person in enumerate(people, start=1):
        color = (
            STATE_COLORS[person.state]
            if stage == 'raised_hand'
            else STAGE_COLORS[stage]
        )
        x1, y1, x2, y2 = np.asarray(person.bbox, dtype=np.int32)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)

        stage_label = person.state if stage == 'raised_hand' else stage.upper()
        label = f'#{rank} ID:{person.track_id} {stage_label} {person.confidence:.2f}'
        cv2.putText(
            output,
            label,
            (x1, max(y1 - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

        if stage != 'detect':
            _draw_skeleton(output, person, color, keypoint_threshold)
    return output


def _draw_skeleton(
    frame: np.ndarray,
    person: PersonPose,
    color: tuple[int, int, int],
    threshold: float,
) -> None:
    keypoints = np.asarray(person.keypoints)
    scores = np.asarray(person.keypoint_scores)

    for start, end in SKELETON:
        if start >= len(scores) or end >= len(scores):
            continue
        if scores[start] < threshold or scores[end] < threshold:
            continue
        start_point = tuple(np.asarray(keypoints[start], dtype=np.int32))
        end_point = tuple(np.asarray(keypoints[end], dtype=np.int32))
        cv2.line(frame, start_point, end_point, color, 2, cv2.LINE_AA)

    for keypoint, score in zip(keypoints, scores):
        if score < threshold:
            continue
        point = tuple(np.asarray(keypoint, dtype=np.int32))
        cv2.circle(frame, point, 3, color, -1, cv2.LINE_AA)
