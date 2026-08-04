import cv2
import numpy as np

from .types import PersonPose

# COCO 17 Keypoints Skeleton Pairs
SKELETON = (
    # Face
    (0, 1), (0, 2), (1, 3), (2, 4),
    # Upper Body
    (0, 5), (0, 6), (5, 6),
    (5, 7), (7, 9),      # Left Arm
    (6, 8), (8, 10),     # Right Arm
    # Torso
    (5, 11), (6, 12), (11, 12),
    # Lower Body
    (11, 13), (13, 15),  # Left Leg
    (12, 14), (14, 16),  # Right Leg
)

STATE_COLORS = {
    'UNKNOWN': (128, 128, 128),
    'NONE': (0, 255, 0),         # Bright Green
    'LEFT': (0, 165, 255),       # Bright Orange
    'RIGHT': (0, 255, 255),      # Bright Yellow
    'BOTH': (0, 0, 255),         # Red
}

STAGE_COLORS = {
    'detect': (0, 220, 255),
    'pose': (255, 120, 80),
}


def draw_people(
    frame: np.ndarray,
    people: list[PersonPose],
    keypoint_threshold: float = 0.25,
    stage: str = 'raised_hand',
) -> np.ndarray:
    output = frame.copy()
    for rank, person in enumerate(people, start=1):
        color = (
            STATE_COLORS.get(person.state, (128, 128, 128))
            if stage == 'raised_hand'
            else STAGE_COLORS.get(stage, (0, 255, 0))
        )
        x1, y1, x2, y2 = np.asarray(person.bbox, dtype=np.int32)
        cv2.rectangle(output, (x1, y1), (x2, y2), color, 3)

        stage_label = person.state if stage == 'raised_hand' else stage.upper()
        label = f'#{rank} ID:{person.track_id} | STATE: {stage_label} | Conf:{person.confidence:.2f}'
        
        # Bounding box header label with background tag for maximum readability
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        bg_y1 = max(y1 - th - 10, 0)
        cv2.rectangle(output, (x1, bg_y1), (x1 + tw + 10, bg_y1 + th + 10), (0, 0, 0), -1)
        cv2.putText(
            output,
            label,
            (x1 + 5, bg_y1 + th + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )

        if stage != 'detect':
            _draw_skeleton(output, person, color, keypoint_threshold)

        # Highlight raised hand wrists if active
        scores = np.asarray(person.keypoint_scores)
        kps = np.asarray(person.keypoints)

        # Left Wrist: index 9 (0-indexed COCO index 9 = 10th keypoint)
        if person.left_raised and len(kps) > 9 and len(scores) > 9 and scores[9] >= keypoint_threshold:
            lw_pt = tuple(np.asarray(kps[9], dtype=np.int32))
            cv2.circle(output, lw_pt, 12, (0, 165, 255), -1, cv2.LINE_AA)
            cv2.putText(output, "LEFT HAND", (lw_pt[0] - 40, lw_pt[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        # Right Wrist: index 10 (0-indexed COCO index 10 = 11th keypoint)
        if person.right_raised and len(kps) > 10 and len(scores) > 10 and scores[10] >= keypoint_threshold:
            rw_pt = tuple(np.asarray(kps[10], dtype=np.int32))
            cv2.circle(output, rw_pt, 12, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(output, "RIGHT HAND", (rw_pt[0] - 40, rw_pt[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    return output


def _draw_skeleton(
    frame: np.ndarray,
    person: PersonPose,
    color: tuple[int, int, int],
    threshold: float,
) -> None:
    keypoints = np.asarray(person.keypoints)
    scores = np.asarray(person.keypoint_scores)

    if len(keypoints) == 0 or len(scores) == 0:
        return

    # Draw Skeleton Bone Lines
    for start, end in SKELETON:
        if start >= len(scores) or end >= len(scores):
            continue
        if scores[start] < threshold or scores[end] < threshold:
            continue
        start_point = tuple(np.asarray(keypoints[start], dtype=np.int32))
        end_point = tuple(np.asarray(keypoints[end], dtype=np.int32))
        cv2.line(frame, start_point, end_point, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.line(frame, start_point, end_point, color, 2, cv2.LINE_AA)

    # Draw Keypoint Circles
    for idx, (keypoint, score) in enumerate(zip(keypoints, scores)):
        if score < threshold:
            continue
        point = tuple(np.asarray(keypoint, dtype=np.int32))
        cv2.circle(frame, point, 5, (0, 255, 255), -1, cv2.LINE_AA) # Yellow joint
        cv2.circle(frame, point, 6, (0, 0, 0), 1, cv2.LINE_AA)
