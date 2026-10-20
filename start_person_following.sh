#!/usr/bin/env bash
# Bring up three cameras, YOLO person tracking (GPU) and RTMPose (CPU).
# Only processes started by this script are stopped on exit.
set -euo pipefail

WS="$(cd -- "$(dirname -- "$0")" && pwd)"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "ROS workspace not built: $WS" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

PERSON_MODEL="${PERSON_MODEL:-yolo11n.pt}"
YOLO_DEVICE="${YOLO_DEVICE:-cuda:0}"
YOLO_IMGSZ_HEIGHT="${YOLO_IMGSZ_HEIGHT:-384}"
YOLO_IMGSZ_WIDTH="${YOLO_IMGSZ_WIDTH:-640}"
YOLO_MAX_DET="${YOLO_MAX_DET:-10}"
YOLO_CLASSES="${YOLO_CLASSES:-0}"
YOLO_IMAGE_RELIABILITY="${YOLO_IMAGE_RELIABILITY:-2}"
HAND_WAVE_DEVICE="${HAND_WAVE_DEVICE:-cpu}"
HAND_WAVE_MAX_PEOPLE="${HAND_WAVE_MAX_PEOPLE:-3}"
FRONT_DEPTH_FPS="${FRONT_DEPTH_FPS:-15}"
REAR_DEPTH_FPS="${REAR_DEPTH_FPS:-15}"
AI_COLOR_FPS="${AI_COLOR_FPS:-30}"

# These topics match mo_hinh/three_astra.launch.py and yolo.launch.py.
AI_IMAGE_TOPIC="/ai_camera/color/image_raw"
TRACKING_TOPIC="/yolo/tracking"

LOG_DIR="${AI_LOG_DIR:-$WS/log/ai_pipeline_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOG_DIR"
pids=()

cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

check_alive() {
  local pid="$1"
  local logfile="$2"
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "Launch process $pid exited unexpectedly; inspect $logfile" >&2
    tail -n 35 "$logfile" >&2 || true
    exit 1
  fi
}

echo "[1/3] Start front/rear Depth and AI RGB"
ros2 launch mo_hinh three_astra.launch.py \
  front_depth_fps:="$FRONT_DEPTH_FPS" \
  rear_depth_fps:="$REAR_DEPTH_FPS" \
  ai_color_fps:="$AI_COLOR_FPS" \
  > "$LOG_DIR/cameras.log" 2>&1 &
camera_pid=$!
pids+=("$camera_pid")
sleep 6
check_alive "$camera_pid" "$LOG_DIR/cameras.log"

echo "[2/3] Start YOLO person detection/tracking on $AI_IMAGE_TOPIC"
ros2 launch yolo_bringup yolo.launch.py \
  model:="$PERSON_MODEL" \
  device:="$YOLO_DEVICE" \
  input_image_topic:="$AI_IMAGE_TOPIC" \
  image_reliability:="$YOLO_IMAGE_RELIABILITY" \
  classes:="$YOLO_CLASSES" \
  imgsz_height:="$YOLO_IMGSZ_HEIGHT" \
  imgsz_width:="$YOLO_IMGSZ_WIDTH" \
  max_det:="$YOLO_MAX_DET" \
  use_tracking:=True \
  use_3d:=False \
  use_debug:=True \
  > "$LOG_DIR/yolo.log" 2>&1 &
yolo_pid=$!
pids+=("$yolo_pid")
sleep 3
check_alive "$yolo_pid" "$LOG_DIR/yolo.log"

echo "[3/3] Start RTMPose hand-wave/skeleton on $AI_IMAGE_TOPIC"
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:="$HAND_WAVE_DEVICE" \
  max_people:="$HAND_WAVE_MAX_PEOPLE" \
  image_topic:="$AI_IMAGE_TOPIC" \
  tracking_topic:="$TRACKING_TOPIC" \
  > "$LOG_DIR/hand_wave.log" 2>&1 &
pose_pid=$!
pids+=("$pose_pid")
sleep 3
check_alive "$pose_pid" "$LOG_DIR/hand_wave.log"

cat <<EOF

AI 2D + skeleton pipeline started.
  RGB source:      $AI_IMAGE_TOPIC
  Person tracking: $TRACKING_TOPIC
  Skeleton debug:  /pose/image_debug
  Hand wave:       /pose/wave_detected
  YOLO device:     $YOLO_DEVICE
  RTMPose device:  $HAND_WAVE_DEVICE
  Logs:            $LOG_DIR
  Image viewer:    ros2 run rqt_image_view rqt_image_view /pose/image_debug

This script does NOT start a 3D person follower: RGB AI and Depth front/rear
are separate cameras and are not registered to each other.
Press Ctrl+C to stop only the processes started here.
EOF

# If any launch ends, stop the other launches as well.
set +e
wait -n "${pids[@]}"
exit_status=$?
set -e
echo "An AI launch process exited (status=$exit_status); shutting down pipeline." >&2
exit "$exit_status"
