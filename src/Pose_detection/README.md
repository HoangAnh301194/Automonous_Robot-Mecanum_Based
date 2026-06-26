# 🖐️ Real-Time Hand-Raise (Wave) Detection

> **Version**: 1.0 – Inference-only MVP  
> **Backends**: YOLOv8-Pose · MediaPipe Pose (BlazePose)

---

## Overview

Detect when a person raises their hand above their head in real-time via
webcam or video file.  A two-state temporal smoothing machine prevents
flickering – the system only reports **WAVE DETECTED** after the gesture
has been held for several consecutive frames.

---

## Project Structure

```
Pose_detection/
├── main.py                             # Entry point (CLI)
├── detectors/
│   ├── base_pose_detector.py           # ABC + common types
│   ├── yolo_pose_detector.py           # YOLO Pose backend
│   └── mediapipe_pose_detector.py      # MediaPipe Pose backend
├── gesture/
│   ├── hand_raise_detector.py          # Per-frame raise logic
│   └── state_machine.py               # Temporal smoothing (IDLE ↔ WAVING)
├── utils/
│   ├── config.py                       # PipelineConfig dataclass
│   └── drawing.py                      # OpenCV visualisation helpers
├── requirements.txt
└── README.mda
```

---

## Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Run with YOLO backend (webcam)
python main.py --backend yolo --source 0 --device auto --show

# 3. Run with MediaPipe backend (video file)
python main.py --backend mediapipe --source video.mp4 --show
```

---

## CLI Arguments

| Flag          | Default           | Description                              |
| :------------ | :---------------- | :--------------------------------------- |
| `--backend`   | `yolo`            | `yolo` or `mediapipe`                    |
| `--source`    | `0`               | Camera index or path to video file       |
| `--conf`      | `0.4`             | Min keypoint confidence to trust         |
| `--margin`    | `20`              | Wrist must be ≥ margin px above nose     |
| `--model`     | `yolov8n-pose.pt` | YOLO Pose weight file                    |
| `--device`    | `auto`            | YOLO device: `auto`, `cpu`, `cuda:0`    |
| `--show`      | off               | Display the annotated video window       |

Press **q** to quit the window.

## Jetson GPU

For the YOLO backend, the app now defaults to `--device auto`, which picks
`cuda:0` when PyTorch reports CUDA is available and otherwise falls back to
`cpu`.

```bash
python main.py --backend yolo --source 0 --device cuda:0 --show
```

The MediaPipe backend does not use this `--device` flag.

## TensorRT Quantization

For Jetson, the practical deployment path is a TensorRT engine.

FP16 export:

```bash
python3 scripts/quantize_yolo_pose.py \
  --model yolov8n-pose.pt \
  --precision fp16
```

INT8 export:

```bash
python3 scripts/quantize_yolo_pose.py \
  --model yolov8n-pose.pt \
  --precision int8 \
  --data /path/to/your-pose-dataset.yaml
```

Notes:

- The export helper pre-fuses the model on CPU before moving it to CUDA to avoid Jetson memory spikes during export.
- INT8 requires a real local **pose** dataset YAML for calibration, not just a folder of images.
- `examples/pose-calibration.example.yaml` is only a template. Update its `path`, `train`, and `val` entries first.
- Ultralytics recommends roughly 300+ calibration images for stable INT8 results.
- The exported engine can be loaded directly by this app:

```bash
python3 main.py --backend yolo --model yolov8n-pose.engine --source 0 --device auto --show
```

---

## Architecture

```
┌──────────┐      ┌──────────────────┐      ┌───────────────────┐
│  Webcam  │─────▶│  PoseDetector    │─────▶│  HandRaiseDetect  │
│  / Video │      │  (YOLO / MP)     │      │  (per-frame bool) │
└──────────┘      └──────────────────┘      └─────────┬─────────┘
                                                      │
                                              ┌───────▼─────────┐
                                              │  StateMachine    │
                                              │  IDLE ↔ WAVING   │
                                              └───────┬─────────┘
                                                      │
                                              ┌───────▼─────────┐
                                              │  Drawing Utils   │
                                              │  (overlay text)  │
                                              └─────────────────┘
```

### Hand-Raise Logic

A wrist is considered **raised** when:
```
wrist_y < nose_y - margin   AND   confidence ≥ threshold
```

### Temporal Smoothing

| Transition        | Condition                              |
| :---------------- | :------------------------------------- |
| IDLE → WAVING     | Hand raised for ≥ **5** consecutive frames  |
| WAVING → IDLE     | Hand lowered for ≥ **8** consecutive frames |

