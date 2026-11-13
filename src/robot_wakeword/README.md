# robot_wakeword — Vietnamese-only Wake Word (Souta)

Standalone ROS 2 `ament_python` package for offline Vietnamese Wake Word detection.
Uses **one** Vosk ASR model (`vosk-model-small-vn-0.4`) and matches transcript
aliases for **Souta / Sô ta**. The emitted JSON event uses
`{"event":"wake_detected","keyword":"souta","detector":"vi_souta"}`
(with an additional timestamp). It is not a specially trained KWS model.

**Scope:** The package currently has no Kiosk, Behavior Tree, ROS topic, or
service integration. The old prototype under `src/robot_ui/wakeupword/` is
left untouched while the Kiosk migration is deferred.

## Structure

- `robot_wakeword/probe.py`: standalone `live`, `record`, `evaluate` commands.
- `robot_wakeword/config/keywords.json`: one Vietnamese detector and its aliases.
- `robot_wakeword/download_models.py`: downloads **only** the configured Vietnamese model.
- `launch/wakeword_probe.launch.py`: process launcher, no ROS node or publishers.
- `test/`: offline tests; optional model-load smoke test.

## Build (Jetson Orin Nano, Ubuntu 22.04, ROS 2 Humble)

```bash
cd ~/ros2_ws
git switch simulator_interface
git pull --ff-only origin simulator_interface
sudo apt-get install -y pulseaudio-utils python3-pip
python3 -m pip install --user -r src/robot_wakeword/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_wakeword --symlink-install
source install/setup.bash
```

Download the Vietnamese Vosk model once (outside Git, stored in `~/vosk-models`):

```bash
ros2 run robot_wakeword wakeword_download_models
```

## Standalone operation

```bash
ros2 run robot_wakeword wakeword_probe live
ros2 run robot_wakeword wakeword_probe live --debug-transcripts --debug-partials
ros2 run robot_wakeword wakeword_probe record --output /tmp/souta.wav --duration 4
ros2 run robot_wakeword wakeword_probe evaluate --manifest /tmp/manifest.csv --report /tmp/report.json
ros2 launch robot_wakeword wakeword_probe.launch.py
```

An evaluate manifest has columns `file,expected`, with expected values
`vi_souta` or `none`. Recorded WAV must use PCM S16LE mono 16 kHz.

To customize Vietnamese aliases, copy the JSON and pass a new `--config`
path to `wakeword_probe` and `wakeword_download_models`. The JSON accepts
**only** `vi_souta` and Vietnamese `language: vi`.

```bash
cp src/robot_wakeword/robot_wakeword/config/keywords.json ~/wakeword_keywords.json
ros2 run robot_wakeword wakeword_probe live --config ~/wakeword_keywords.json
```

## Tests (no microphone or model required)

```bash
PYTHONPATH=src/robot_wakeword python3 -m unittest discover -s src/robot_wakeword/test -p 'test_*.py' -v
```

The optional model smoke test skips if the model/Vosk is not installed.
Wake detection uses final ASR segments, so the latency and false-trigger rate
must be measured on the physical microphone before production deployment.

## Pronunciation verification

This is Vosk general ASR with keyword matching, so it will only trigger if the
Vietnamese recognizer actually transcribes the sound as one of the configured
aliases. Test local utterances of **Sô ta** and **Souta** with
`wakeword_probe live --debug-transcripts` and adjust aliases if necessary.
Do not assume reliable detection before testing on the physical microphone.

## Experimental custom training (separate from current Vosk runtime)

See [training/README.md](training/README.md) for a speaker-disjoint, real-recording
Souta KWS pipeline: capture, augment, frozen openWakeWord features, PyTorch
head, ONNX export, threshold calibration and streaming test. This pipeline
is not connected to Kiosk and produces no trained weights until recordings exist.
