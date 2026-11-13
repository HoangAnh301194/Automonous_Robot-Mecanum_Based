# Souta — Custom Wake Word training pipeline

**Training-only experimental pipeline.** It does not modify the working Vosk
probe, Kiosk, ROS topics, or robot launch. The intended future runtime is an
openWakeWord ONNX classifier. Nothing here is a trained Souta model yet.

## Architecture

16 kHz mono PCM -> openWakeWord frozen mel ONNX -> frozen speech embedding ONNX
(16 frames x 96 features per 2 s window) -> custom PyTorch 2-class MLP head
-> ONNX model (wake probability). Only the last classifier is trained.

Reference: https://github.com/dscripka/openWakeWord/blob/main/README.md
and its AudioFeatures API. Extractor transfer to Vietnamese is experimental;
speech-embedding features might not generalize to Vietnamese equally well.

## Installation (training PC, not the production Jetson Python environment)

Use Python 3.10 or 3.11 with an isolated virtual environment. A CUDA GPU speeds
the tiny classifier, but feature extraction currently runs on CPU ONNX.

    cd ~/ros2_ws/src/robot_wakeword/training
    python3 -m venv .venv
    source .venv/bin/activate
    python -m pip install -U pip
    python -m pip install -r requirements-train.txt
    python -m unittest -v test_pipeline.py

First embedding run downloads the upstream frozen shared feature models.
Check licensing before distributing any trained models or third-party data.
The scripts **do not download** large or unclear-license external corpora.

## 1. Collect genuine Vietnamese speech

Save recordings privately under this structure (speaker ID is a pseudonym):

    data/raw/
      positive/speaker_001/*.wav
      positive/speaker_002/*.wav
      negative/speaker_001/*.wav
      negative/speaker_002/*.wav
      hard_negative/speaker_001/*.wav
      ambient/session_001/*.wav

Each WAV must be PCM **16 kHz, 16-bit, mono**. Use “Sô ta” and “Sô ta ơi” as
positives. Record normal speech, phonetically similar terms (e.g. “sổ tay”),
music, robot motors, fans, TV, and unrelated conversations as negatives.
Do **not** relabel arbitrary ASR transcripts as positives.

Record one microphone example on a Linux/PulseAudio device:

    python souta_kws.py record --output data/raw/positive/speaker_001/001.wav --duration 3

Convert existing recordings, preserving only channels/sample rate/format:

    ffmpeg -i input.m4a -ac 1 -ar 16000 -sample_fmt s16 -c:a pcm_s16le output.wav

Plan an initial ~20-30 speakers x 10-20 positives each, multiple distances and
rooms, and genuinely independent continuous negative recordings. This is an
initial target, not evidence of sufficient model accuracy. Obtain consent.
**Never upload raw identifiable voice data to the public GitHub repo.**

## 2. Build a speaker-disjoint manifest

    python souta_kws.py prepare --raw data/raw --manifest data/manifest.csv

Requires >=3 independent speaker groups for positives/negatives. The same
speaker never occurs in train, validation and test simultaneously. Sessions of
ambient noise are independently assigned. If a class is absent from a split,
add groups rather than silently accepting an invalid evaluation.

## 3. Extract frozen features + training-only augmentation

    python souta_kws.py extract --manifest data/manifest.csv \
      --output runs/features --augment 4 --batch 16

Train-only augmentation: random offset, gain, additive ambient noise/SNR.
Long negative recordings contribute up to 100 random 2-second windows per recording.
Validation/test receive no training augmentation. Background noise is drawn
only from TRAIN ambient sessions. The script creates train/val/test .npz
embedding matrices. It checks each feature has shape (16,96).

IMPORTANT: positive recordings must contain the full wake phrase in a 2-second
centered window; trim prolonged leading/trailing silence. For long negatives,
random 2-second windows are sampled; long uncut negatives are also evaluated
as continuous streams in the later steps.

## 4. Train classifier and export ONNX

    python souta_kws.py train --features runs/features \
      --output runs/souta --epochs 80

Artifacts: best.pt, souta.onnx, model_metadata.json. The .onnx contains
the classifier, **not** the frozen feature extractors. It is designed for
openWakeWord's streaming model loader.

## 5. Calibrate only on held-out validation recordings

    python souta_kws.py calibrate \
      --manifest data/manifest.csv --model runs/souta/souta.onnx \
      --target-far 0.5 --min-negative-hours 1 \
      --output runs/souta/calibration.json

Sweep thresholds over streaming model scores and choose the best recall
among thresholds with validation false accepts/hour <= 0.5. If none meets the
criterion, command fails. If validation negative audio is <1 h, the threshold
is marked **preliminary**; do not present as proven at low false-accept rate.

## 6. Final test (never used to select threshold)

    python souta_kws.py evaluate \
      --manifest data/manifest.csv --model runs/souta/souta.onnx \
      --calibration runs/souta/calibration.json \
      --output runs/souta/test_report.json

Record-level recall, total false accepts, continuous negative hours and
false accepts/hour are computed from a genuine streaming inference path with
a threshold and refractory period. Test audio must not have been seen in
training or threshold tuning. If insufficient negative hours, report warns.

Before deployment, separately benchmark wake latency, peak RAM, CPU%, and
false triggers with the robot's fans/YOLO/RTMPose/Nav2 running on Jetson.

## Known limitations

- No automatic Vietnamese synthetic TTS generation; genuine Vietnamese recordings are required.
- openWakeWord's frozen embedding is not guaranteed to be language-neutral.
- For robustness, collect hard negatives and several hours of continuous negatives.
- Model weights are **not present** until user recordings are supplied and training runs.
- No runtime Kiosk integration and no ros2 launch changes in this stage.
- The present script uses a clip-level head and framewise threshold/hysteresis
  on streaming predictions; further production evaluation and debounce tuning are necessary.
