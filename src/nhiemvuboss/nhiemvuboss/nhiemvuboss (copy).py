import argparse
from collections import deque

import cv2
import mediapipe as mp
import numpy as np


mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils


# =========================
# Utility functions
# =========================
def moving_average(arr, k=5):
    arr = np.asarray(arr, dtype=np.float32)
    if len(arr) < 2:
        return arr.copy()
    if len(arr) < k:
        k = max(1, len(arr))
    pad = k // 2
    padded = np.pad(arr, (pad, pad), mode="edge")
    kernel = np.ones(k, dtype=np.float32) / k
    return np.convolve(padded, kernel, mode="valid")


def count_turning_points(y, eps=0.004):
    """
    Đếm số lần đổi hướng của chuỗi y sau khi làm mượt.
    Vì tọa độ y của MediaPipe tăng theo chiều xuống dưới ảnh:
    - y giảm => tay đi lên
    - y tăng => tay đi xuống
    """
    y = np.asarray(y, dtype=np.float32)
    if len(y) < 5:
        return 0

    y_smooth = moving_average(y, k=5)
    dy = np.diff(y_smooth)

    # Bỏ qua nhiễu nhỏ
    signs = []
    prev = 0
    for v in dy:
        if abs(v) < eps:
            continue
        s = 1 if v > 0 else -1
        if s != prev:
            signs.append(s)
            prev = s

    if len(signs) < 2:
        return 0

    return len(signs) - 1


def clamp01(x):
    return float(max(0.0, min(1.0, x)))


def pose_landmarks_to_normalized_dict(landmarks):
    """
    Chuẩn hóa landmark theo tâm vai và độ rộng hai vai.
    Trả về dict:
        {
          "left_shoulder": [x, y, z, vis],
          ...
        }
    """
    ids = {
        "left_shoulder": mp_pose.PoseLandmark.LEFT_SHOULDER.value,
        "right_shoulder": mp_pose.PoseLandmark.RIGHT_SHOULDER.value,
        "left_elbow": mp_pose.PoseLandmark.LEFT_ELBOW.value,
        "right_elbow": mp_pose.PoseLandmark.RIGHT_ELBOW.value,
        "left_wrist": mp_pose.PoseLandmark.LEFT_WRIST.value,
        "right_wrist": mp_pose.PoseLandmark.RIGHT_WRIST.value,
    }

    ls = landmarks[ids["left_shoulder"]]
    rs = landmarks[ids["right_shoulder"]]

    center = np.array([
        (ls.x + rs.x) * 0.5,
        (ls.y + rs.y) * 0.5,
        (ls.z + rs.z) * 0.5
    ], dtype=np.float32)

    shoulder_width = np.linalg.norm(
        np.array([rs.x - ls.x, rs.y - ls.y], dtype=np.float32)
    )

    if shoulder_width < 1e-6:
        return None

    out = {}
    for name, idx in ids.items():
        lm = landmarks[idx]
        p = np.array([lm.x, lm.y, lm.z], dtype=np.float32)
        p = (p - center) / shoulder_width
        out[name] = np.array([p[0], p[1], p[2], lm.visibility], dtype=np.float32)

    return out


# =========================
# Gesture detector
# =========================
class VerticalWaveDetector:
    def __init__(
        self,
        window_size=30,
        min_valid_frames=18,
        visibility_thresh=0.5,
        activation_frames=3
    ):
        self.window_size = window_size
        self.min_valid_frames = min_valid_frames
        self.visibility_thresh = visibility_thresh
        self.activation_frames = activation_frames

        self.history = deque(maxlen=window_size)
        self.activation = 0

    def _analyze_arm(self, side="right"):
        assert side in ("left", "right")

        s_key = f"{side}_shoulder"
        e_key = f"{side}_elbow"
        w_key = f"{side}_wrist"

        shoulders = []
        elbows = []
        wrists = []

        for frame_data in self.history:
            if frame_data is None:
                continue

            s = frame_data[s_key]
            e = frame_data[e_key]
            w = frame_data[w_key]

            if (
                s[3] >= self.visibility_thresh
                and e[3] >= self.visibility_thresh
                and w[3] >= self.visibility_thresh
            ):
                shoulders.append(s[:3])
                elbows.append(e[:3])
                wrists.append(w[:3])

        n = len(wrists)
        if n < self.min_valid_frames:
            return {
                "detected": False,
                "score": 0.0,
                "side": side,
                "metrics": {}
            }

        shoulders = np.asarray(shoulders, dtype=np.float32)
        elbows = np.asarray(elbows, dtype=np.float32)
        wrists = np.asarray(wrists, dtype=np.float32)

        sy, ey, wy = shoulders[:, 1], elbows[:, 1], wrists[:, 1]
        sx, ex, wx = shoulders[:, 0], elbows[:, 0], wrists[:, 0]
        sz, ez, wz = shoulders[:, 2], elbows[:, 2], wrists[:, 2]

        # ===== Điều kiện vị trí =====
        # Tay giơ lên: cổ tay thường cao hơn khuỷu tay
        wrist_above_elbow_ratio = np.mean(wy < (ey - 0.02))

        # Cổ tay ngang vai hoặc cao hơn một chút
        wrist_near_shoulder_ratio = np.mean(wy < (sy + 0.12))

        position_score = 0.5 * wrist_above_elbow_ratio + 0.5 * wrist_near_shoulder_ratio

        # ===== Điều kiện chuyển động =====
        y_amp = float(np.ptp(wy))
        x_amp = float(np.ptp(wx))
        z_amp = float(np.ptp(wz))

        elbow_y_amp = float(np.ptp(ey))
        elbow_x_amp = float(np.ptp(ex))

        turning_points = count_turning_points(wy, eps=0.004)

        # Dao động dọc phải trội hơn ngang và sâu
        vertical_dominance = (
            y_amp > 0.10 and
            y_amp > 1.25 * x_amp and
            y_amp > 1.10 * z_amp
        )

        # Khuỷu tay không nên dao động quá mạnh cùng cổ tay
        elbow_stable = (
            elbow_y_amp < (0.75 * y_amp + 0.03) and
            elbow_x_amp < 0.12
        )

        motion_amp_score = clamp01((y_amp - 0.08) / 0.14)
        turn_score = clamp01((turning_points - 1) / 3.0)
        dominance_score = 1.0 if vertical_dominance else clamp01(
            y_amp / (max(x_amp, z_amp, 1e-6) * 1.8)
        )
        elbow_score = 1.0 if elbow_stable else 0.0

        score = (
            0.30 * position_score +
            0.30 * motion_amp_score +
            0.20 * turn_score +
            0.10 * dominance_score +
            0.10 * elbow_score
        )

        detected = (
            position_score > 0.65 and
            y_amp > 0.10 and
            turning_points >= 2 and
            vertical_dominance and
            elbow_stable
        )

        return {
            "detected": detected,
            "score": float(score),
            "side": side,
            "metrics": {
                "valid_frames": n,
                "wrist_above_elbow_ratio": float(wrist_above_elbow_ratio),
                "wrist_near_shoulder_ratio": float(wrist_near_shoulder_ratio),
                "y_amp": y_amp,
                "x_amp": x_amp,
                "z_amp": z_amp,
                "elbow_y_amp": elbow_y_amp,
                "elbow_x_amp": elbow_x_amp,
                "turning_points": int(turning_points),
                "vertical_dominance": bool(vertical_dominance),
                "elbow_stable": bool(elbow_stable),
            }
        }

    def update(self, frame_pose):
        """
        frame_pose: dict normalized landmark hoặc None
        """
        self.history.append(frame_pose)

        if frame_pose is None:
            self.activation = max(0, self.activation - 2)
            return {
                "detected": False,
                "stable_detected": False,
                "side": None,
                "score": 0.0,
                "metrics": {}
            }

        left_result = self._analyze_arm("left")
        right_result = self._analyze_arm("right")

        best = left_result if left_result["score"] >= right_result["score"] else right_result

        if best["detected"]:
            self.activation = min(self.activation + 1, self.activation_frames + 3)
        else:
            self.activation = max(0, self.activation - 1)

        stable_detected = self.activation >= self.activation_frames

        return {
            "detected": best["detected"],
            "stable_detected": stable_detected,
            "side": best["side"],
            "score": best["score"],
            "metrics": best["metrics"]
        }


# =========================
# Main
# =========================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=str, required=True, help="Path tới video input")
    parser.add_argument("--output", type=str, default="", help="Path lưu video output")
    parser.add_argument("--window", type=int, default=30, help="Số frame trong cửa sổ thời gian")
    parser.add_argument("--show", action="store_true", help="Hiển thị cửa sổ realtime")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError(f"Không mở được video: {args.video}")

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 1e-6:
        fps = 25.0

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.output, fourcc, fps, (width, height))

    detector = VerticalWaveDetector(
        window_size=args.window,
        min_valid_frames=max(12, int(args.window * 0.6)),
        visibility_thresh=0.5,
        activation_frames=3
    )

    with mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2,
        smooth_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as pose:

        frame_idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame_idx += 1

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = pose.process(rgb)

            frame_pose = None
            if results.pose_landmarks:
                frame_pose = pose_landmarks_to_normalized_dict(results.pose_landmarks.landmark)

            result = detector.update(frame_pose)

            # Vẽ skeleton
            if results.pose_landmarks:
                mp_drawing.draw_landmarks(
                    frame,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS
                )

            # Hiển thị kết quả
            label = "NO_WAVE"
            color = (0, 0, 255)

            if result["stable_detected"]:
                label = f"EAST_ASIAN_WAVE ({result['side']})"
                color = (0, 255, 0)
            elif result["detected"]:
                label = f"LIKELY_WAVE ({result['side']})"
                color = (0, 255, 255)

            cv2.putText(
                frame,
                label,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                color,
                2,
                cv2.LINE_AA
            )

            cv2.putText(
                frame,
                f"score={result['score']:.2f}",
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            metrics = result.get("metrics", {})
            if metrics:
                debug_lines = [
                    f"turns={metrics.get('turning_points', 0)}",
                    f"y_amp={metrics.get('y_amp', 0):.3f}",
                    f"x_amp={metrics.get('x_amp', 0):.3f}",
                    f"z_amp={metrics.get('z_amp', 0):.3f}",
                    f"pos={0.5 * metrics.get('wrist_above_elbow_ratio', 0) + 0.5 * metrics.get('wrist_near_shoulder_ratio', 0):.2f}",
                    f"elbow_stable={metrics.get('elbow_stable', False)}"
                ]
                y0 = 110
                for line in debug_lines:
                    cv2.putText(
                        frame,
                        line,
                        (20, y0),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA
                    )
                    y0 += 28

            if writer is not None:
                writer.write(frame)

            if args.show:
                cv2.imshow("Wave Detection", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()