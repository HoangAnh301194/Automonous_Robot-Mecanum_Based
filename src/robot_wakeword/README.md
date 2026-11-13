# robot_wakeword — standalone ROS 2 package (Phase 1)

This is a **standalone ament_python ROS 2 package** containing the existing
Vosk ASR + keyword matching prototype (Vietnamese, English and Japanese).
It is deliberately **not integrated** into Kiosk, Behavior Tree, or other
robot services yet. It does **not** publish or subscribe to ROS 2 topics.

The historical prototype in `src/robot_ui/wakeupword/` remains untouched
to avoid modifying the existing Kiosk workflow; this package is the
standalone candidate for future integration. Do not maintain two divergent
configurations when the future migration is performed.

## Files

- `robot_wakeword/probe.py`: `live`, `record`, `evaluate` commands.
- `robot_wakeword/config/keywords.json`: model names, language and aliases.
- `robot_wakeword/download_models.py`: downloads the configured Vosk models.
- `launch/wakeword_probe.launch.py`: optional process launcher (not a ROS node).
- `test/`: keyword/config tests and optional model-load smoke test.

## Install on Jetson Orin Nano / Ubuntu 22.04 / ROS 2 Humble

```bash
cd ~/ros2_ws
git switch simulator_interface
git pull --ff-only origin simulator_interface
sudo apt-get install -y pulseaudio-utils python3-pip
# Use the same Python environment used to run ROS 2 console scripts.
python3 -m pip install --user -r src/robot_wakeword/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_wakeword --symlink-install
source install/setup.bash
python3 -c 'import vosk; print("Vosk import OK")'
```

Vosk models are not committed to Git or embedded in the ROS 2 package.
Download them separately (requires network once):

```bash
ros2 run robot_wakeword wakeword_download_models
```

Default model directory: `~/vosk-models`.

## Standalone commands (no Kiosk required)

```bash
# Listen from the default microphone. Prints JSON wake_detected events.
ros2 run robot_wakeword wakeword_probe live
# Inspect raw ASR segments for debugging (contains potentially private speech).
ros2 run robot_wakeword wakeword_probe live --debug-transcripts --debug-partials
# Record one test utterance.
ros2 run robot_wakeword wakeword_probe record --output /tmp/maica.wav --duration 4
# Evaluate offline. Manifest CSV has columns: file,expected
ros2 run robot_wakeword wakeword_probe evaluate --manifest /tmp/manifest.csv --report /tmp/wakeword_report.json
```

Optional ROS launch wrapper (process only, no rclpy node/topics):

```bash
ros2 launch robot_wakeword wakeword_probe.launch.py \
  mic_source:=@DEFAULT_SOURCE@ models_dir:=$HOME/vosk-models
```

To override the package config without editing the installed file:

```bash
cp src/robot_wakeword/robot_wakeword/config/keywords.json ~/wakeword_keywords.json
# Edit the copy to enable/disable models or customize aliases.
ros2 run robot_wakeword wakeword_probe live --config ~/wakeword_keywords.json
```

Model names are in `keywords.json`; `download_models.py` retrieves the
matching pretrained Vosk model archives. Replacing Vosk with another engine
will require a different inference adapter, not just changing `model`.

## Tests

```bash
# From workspace root; these offline tests do not require a microphone or models.
PYTHONPATH=src/robot_wakeword python3 -m unittest discover -s src/robot_wakeword/test -p 'test_*.py' -v

# Optional pretrained-model load check; skips if models are not installed.
PYTHONPATH=src/robot_wakeword python3 -m unittest discover -s src/robot_wakeword/test -p 'test_smoke_models.py' -v
```

## Current constraints

- Uses three general ASR recognizers, **not** a custom trained KWS model.
- Detection is based on completed ASR segments, so wake latency depends on speech segmentation.
- `wake_detected` JSON currently contains canonical keyword `maica`.
- Existing Kiosk WakeWordWorker is not changed. Its old path/package mismatch
  is **not fixed** as part of this packaging-only change.
- No ROS 2 node, topic, service, parameter or IPC interface is introduced here.
