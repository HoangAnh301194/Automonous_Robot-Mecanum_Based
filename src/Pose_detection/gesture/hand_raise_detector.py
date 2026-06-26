"""
Pure-logic hand-raise detector.

Given a set of keypoints and a confidence threshold, determines
whether the left wrist, right wrist, or both are above the head
(nose) by at least *margin* pixels.

This module is **stateless** – it evaluates a single frame.
For temporal smoothing see ``state_machine.py``.
"""

from typing import Dict, Tuple

# ── Type aliases ─────────────────────────────────────────────────────
Keypoint = Tuple[float, float, float]   # (x, y, confidence)


def is_hand_raised(
    keypoints: Dict[str, Keypoint],
    margin: int = 20,
    conf_threshold: float = 0.4,
) -> Tuple[bool, bool]:
    """
    Check whether left / right wrist is above the nose.

    Parameters
    ----------
    keypoints : dict
        ``{ "nose": (x,y,c), "left_wrist": (x,y,c), ... }``
    margin : int
        Wrist must be at least *margin* px above nose_y.
    conf_threshold : float
        Ignore keypoints whose confidence is below this value.

    Returns
    -------
    (left_raised, right_raised) : Tuple[bool, bool]
    """
    nose = keypoints.get("nose")
    left_wrist  = keypoints.get("left_wrist")
    right_wrist = keypoints.get("right_wrist")

    # If nose is missing or low-conf, we can't judge → nothing raised
    if nose is None or nose[2] < conf_threshold:
        return False, False

    nose_y = nose[1]

    # Left wrist check
    left_raised = False
    if left_wrist is not None and left_wrist[2] >= conf_threshold:
        left_raised = left_wrist[1] < (nose_y - margin)

    # Right wrist check
    right_raised = False
    if right_wrist is not None and right_wrist[2] >= conf_threshold:
        right_raised = right_wrist[1] < (nose_y - margin)

    return left_raised, right_raised
