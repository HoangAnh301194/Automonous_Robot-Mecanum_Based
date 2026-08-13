import argparse
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Tuple

import cv2
import mediapipe as mp
import numpy as np
from scipy.signal import find_peaks, welch

# MediaPipe Hands index constants
WRIST = 0
INDEX_MCP = 5
MIDDLE_MCP = 9
PINKY_MCP = 17
MIDDLE_TIP = 12
TIPS = [8, 12, 16, 20]
MCP_OF_TIP = {8: 5, 12: 9, 16: 13, 20: 17}


def unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n


@dataclass
class MotionBuf:
    dy: Deque[float] = field(default_factory=lambda: deque(maxlen=60))
    d_pairs: Deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=60))
    last_fire: float = 0.0


class HandWaveDetector:
    """
    Wave detector for a palm-down vertical waving gesture using MediaPipe Hands landmarks.
    """

    def __init__(
        self,
        fps: float = 30.0,
        ny_min: float = 0.45,
        var_thresh: float = 1e-4,
        min_peaks: int = 3,
        fmin: float = 1.5,
        fmax: float = 6.0,
        fft_prominence: float = 5.0,
        cooldown: float = 0.8,
    ) -> None:
        self.fps = fps
        self.ny_min = ny_min
        self.var_thresh = var_thresh
        self.min_peaks = min_peaks
        self.fmin = fmin
        self.fmax = fmax
        self.fft_prominence = fft_prominence
        self.cooldown = cooldown
        self.buf: Dict[str, MotionBuf] = {}

    def _buf(self, key: str) -> MotionBuf:
        if key not in self.buf:
            self.buf[key] = MotionBuf()
        return self.buf[key]

    def update(self, key: str, pts: np.ndarray, now: float) -> Tuple[bool, Dict[str, float]]:
        """
        pts: ndarray shape (21, 3) in normalized image coordinates (x right, y down, z into image)
        """
        buf = self._buf(key)

        v05 = pts[INDEX_MCP] - pts[WRIST]
        v017 = pts[PINKY_MCP] - pts[WRIST]
        n = unit(np.cross(v05, v017))
        if n[1] <= self.ny_min:
            return False, {"ny": float(n[1])}

        # rigid distances between fingertips
        pairs = [(8, 12), (8, 16), (8, 20), (12, 16), (12, 20), (16, 20)]
        d_pairs = np.array([np.linalg.norm(pts[i] - pts[j]) for i, j in pairs], dtype=np.float32)
        buf.d_pairs.append(d_pairs)

        dy = float(pts[MIDDLE_TIP, 1] - pts[WRIST, 1])
        buf.dy.append(dy)

        if len(buf.dy) < buf.dy.maxlen:
            return False, {"ny": float(n[1])}

        # rigidness check
        var_d = float(np.var(np.stack(buf.d_pairs, axis=0), axis=0).mean())
        if var_d > self.var_thresh:
            return False, {"ny": float(n[1]), "var_d": var_d}

        # finger extension (relative to max in window)
        ext = np.array(
            [np.linalg.norm(pts[t] - pts[MCP_OF_TIP[t]]) for t in TIPS], dtype=np.float32
        )
        ext_norm = ext / (np.max(ext) + 1e-6)
        if (ext_norm > 0.8).sum() < 3:
            return False, {"ny": float(n[1]), "var_d": var_d, "ext_ok": float((ext_norm > 0.8).sum())}

        vel = np.diff(np.array(buf.dy, dtype=np.float32))
        peaks, props = find_peaks(np.abs(vel), height=np.std(vel))

        freqs, psd = welch(vel, fs=self.fps, nperseg=min(64, len(vel)))
        dom_idx = int(np.argmax(psd))
        f_dom = float(freqs[dom_idx])
        p_dom = float(psd[dom_idx])
        p_med = float(np.median(psd) + 1e-9)

        ok_peaks = len(peaks) >= self.min_peaks
        ok_fft = self.fmin <= f_dom <= self.fmax and p_dom > self.fft_prominence * p_med
        fire = False
        if ok_peaks and ok_fft and (now - buf.last_fire) >= self.cooldown:
            buf.last_fire = now
            fire = True

        return fire, {
            "ny": float(n[1]),
            "var_d": var_d,
            "f_dom": f_dom,
            "p_dom": p_dom,
            "peaks": int(len(peaks)),
        }


def draw_text(img, text, org, scale=0.7, color=(0, 255, 0)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Palm-down wave detector using MediaPipe Hands")
    parser.add_argument("--source", default="0", help="camera index or video path")
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    cap = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
    if not cap.isOpened():
        raise RuntimeError("Cannot open source")

    hands = mp.solutions.hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        model_complexity=1,
    )
    detector = HandWaveDetector(fps=args.fps)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)
        vis = frame.copy()
        now = time.time()

        if res.multi_hand_landmarks and res.multi_handedness:
            for lm, handed in zip(res.multi_hand_landmarks, res.multi_handedness):
                key = handed.classification[0].label  # "Left"/"Right"
                pts = np.array([[p.x, p.y, p.z] for p in lm.landmark], dtype=np.float32)
                fire, info = detector.update(key, pts, now)

                # draw skeleton
                mp.solutions.drawing_utils.draw_landmarks(
                    vis, lm, mp.solutions.hands.HAND_CONNECTIONS
                )

                x0, y0 = int(pts[WRIST, 0] * vis.shape[1]), int(pts[WRIST, 1] * vis.shape[0])
                draw_text(
                    vis,
                    f"{key}: ny={info.get('ny', 0):.2f} f={info.get('f_dom', 0):.2f}Hz peaks={info.get('peaks', 0)}",
                    (x0, max(20, y0 - 10)),
                    color=(0, 255, 0) if fire else (0, 200, 255),
                )
                if fire:
                    draw_text(vis, "WAVE!", (x0, y0 + 25), scale=1.0, color=(0, 0, 255))

        draw_text(vis, "Press q to quit", (20, 35), scale=0.7)
        cv2.imshow("Palm-down wave detection", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    hands.close()


if __name__ == "__main__":
    main()
