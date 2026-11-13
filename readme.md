# AUTONOMOUS MECANUM ROBOT WORKFLOW (ROS 2)

## 0) Build & Source Workspace

Run these commands from any directory inside the cloned repository. Repeat the
workspace export and source commands in every new terminal before using packages
from this workspace.

```bash
export WS="$(git rev-parse --show-toplevel)"
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source "$WS/install/setup.bash"
```

For a source archive without Git metadata, open the workspace root and use:

```bash
export WS="$PWD"
```

Rebuild and source the workspace after moving it. Do not reuse `build/`,
`install/`, or `log/` artifacts generated at the old path.

The legacy `config/nav2_params.yaml` resolves custom behavior trees through
`$(env WS)`, so export `WS` before launching Nav2 with this configuration.
Camera `ground_file_path` entries are relative to their containing YAML file.

## 1) Launch Files for Hardware Mode

### Launch 1: LiDAR + IMU + ESP Communication

```bash
ros2 launch mo_hinh real_hw.launch.py
```

Common options:

```bash
ros2 launch mo_hinh real_hw.launch.py \
  esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
```

### Launch 2: Odometry (ESP or RF2O)

```bash
# Odometry from ESP
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp

# Odometry from LiDAR (RF2O)
ros2 launch mo_hinh real_odom.launch.py odom_source:=rf2o

# Use wheel encoders 1 & 4 only (exclude 2 & 3)
ros2 launch mo_hinh real_odom.launch.py \
  odom_source:=esp esp_wheel_odom_mode:=wheels_1_4
```

Notes:
- Two-stage launch: run Launch 2 first, then launch SLAM or Localization.
- Do not launch Launch 1 simultaneously with Launch 2 to avoid serial port conflicts.

### Launch 3: SLAM Mode (Real Robot)

```bash
# Requirement: Launch 2 must be active
ros2 launch mo_hinh real_slam.launch.py
```

### Launch 4: Localization + Nav2 Mode

```bash
# Option 1: AMCL + Nav2 (using YAML map)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=amcl \
  map_yaml:="$WS/src/mo_hinh/maps/my_map.yaml"

# Option 2: SLAM Toolbox Localization + Nav2 (using PoseGraph)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=slam_toolbox \
  map_graph:="$WS/src/mo_hinh/maps/my_slam_graph"
```

## 2) Recommended Operational Workflow

1. Start `real_odom.launch.py` to publish `/scan`, `/imu/data`, `/odom`, and TF.
2. Run SLAM with `real_slam.launch.py`.
3. Save the PoseGraph map after scanning:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$WS/src/mo_hinh/maps/my_slam_graph'}"
```

4. Save a 2D Occupancy Grid map for AMCL:

```bash
ros2 run nav2_map_server map_saver_cli -f "$WS/src/mo_hinh/maps/my_map"
```

5. Switch to Localization + Nav2 with `real_localization_nav2.launch.py` while
   keeping `real_odom.launch.py` active.

## 3) Important Notes (TF & Goal Success)

- All real robot nodes must use `use_sim_time:=false`.
- Do not run AMCL and SLAM Toolbox Localization simultaneously.
- Do not run two `odom -> base_footprint` TF sources simultaneously.
- Export `WS` before using `config/nav2_params.yaml`, because its custom BT XML
  paths use `$(env WS)`.

## 4) Virtual Robot Simulation (Gazebo)

```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
ros2 launch mo_hinh virtual_slam.launch.py
```

## 5) Layer 1 Bringup & Obstacle Detector

```bash
ros2 launch layer1_bringup layer1.launch.py \
  esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1

ros2 run depth_obstacle_detector obstacle_detector \
  --ros-args -p config_file:="$WS/my_map/cam.yaml"
```

## 6) Jetson AI Pipeline (3 Cameras + YOLO11 + Hand Wave RTMPose)

This is the current, recommended **main**-branch bringup for the real Jetson
Orin robot. Run each launch in a separate terminal (ROS 2 Humble). Keep
previously started processes running.

### 6.1 Build and source the workspace

```bash
export WS="$HOME/ros2_ws"
cd "$WS"
git status --short
git switch main
git pull --ff-only origin main

source /opt/ros/humble/setup.bash
sudo apt install -y ros-humble-usb-cam
colcon build --symlink-install --packages-up-to \
  mo_hinh yolo_bringup hand_wave_detection robot_ui
source "$WS/install/setup.bash"
```

Do not discard uncommitted files if Git reports a conflict when switching
branches. The Astra driver, PyTorch with Jetson CUDA support, Ultralytics,
RTMLib and ONNX Runtime must already be available.

Before any ROS command in **each new terminal**:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 6.2 Terminal 1 — Three cameras

```bash
ros2 launch mo_hinh three_astra.launch.py \
  front_depth_fps:=15 \
  rear_depth_fps:=15 \
  ai_color_fps:=30
```

Camera streams:
- Front Depth: `/front_camera/depth/image_raw` — serial-pinned OpenNI.
- Rear Depth: `/rear_camera/depth/image_raw` — serial-pinned OpenNI.
- AI RGB: `/ai_camera/color/image_raw` — `usb_cam`, MJPEG to RGB.

All three cameras are identified by OpenNI serials, including AI:
- Front: `front_serial:=17110310855`
- Rear: `rear_serial:=17122514440`
- AI: `ai_serial:=17122611013`

**Astra Pro RGB is a separate UVC USB interface:** the launch resolves the
OpenNI Depth device by serial (via `libOpenNI2.so`) and then looks for the
sole UVC video-index0 interface underneath the same per-camera USB hub.
It resolves to the currently active `/dev/videoN` and passes that to
`usb_cam`. It does NOT assume USB enumeration order or a hard-coded port.
If the USB topology cannot prove a unique match, startup **fails closed**
rather than silently substituting RGB from Rear. Camera identity must be
verified once on the physical Jetson hardware.

**Before running three cameras for the first time**, stop all camera launches,
then diagnose the serial-to-RGB pairing:

```bash
cd ~/ros2_ws
python3 src/mo_hinh/launch/_astra_serial_resolver.py \
  --front-serial 17110310855 \
  --rear-serial 17122514440 \
  --ai-serial 17122611013
```

The diagnostic prints three `serial -> /dev/videoN` mappings.
If it reports missing `libOpenNI2.so`, point `OPENNI2_LIBRARY` to the
library already used by `astra_camera` (ensure its OpenNI driver plugins
are installed). If a USB pairing is ambiguous, inspect `lsusb -t` and
`v4l2-ctl --list-devices`; do not force an unverified RGB device.

Unit tests (no hardware required):

```bash
python3 tools/test_astra_serial_resolver.py
python3 tools/test_ai_topic_contract.py
```

You can override the three serials when starting a specific robot:

```bash
ros2 launch mo_hinh three_astra.launch.py \
  front_serial:=17110310855 \
  rear_serial:=17122514440 \
  ai_serial:=17122611013
```

This needs `colcon build --symlink-install --packages-select mo_hinh`
after pulling the changed launch/helper files.

Astra may reject 640x480@15 and fall back to a supported mode such as
640x480@30. Treat 15 FPS as **requested** until actual FPS and driver
`set depth video mode` messages are checked.

To run AI RGB alone for a lightweight test:

```bash
ros2 launch mo_hinh three_astra.launch.py \
  enable_front:=false enable_rear:=false enable_ai:=true
```

### 6.3 Terminal 2 — YOLO11 person detection + tracking on GPU

```bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.pt \
  device:=cuda:0 \
  input_image_topic:=/ai_camera/color/image_raw \
  image_reliability:=2 \
  classes:=0 \
  imgsz_height:=384 \
  imgsz_width:=640 \
  max_det:=10 \
  use_tracking:=True \
  use_3d:=False \
  use_debug:=True
```

Outputs: `/yolo/detections`, `/yolo/tracking`, `/yolo/dbg_image`.
The RGB camera is not registered to a depth camera, so 3D detection is
disabled here. `image_reliability:=2` uses Best Effort QoS on the image
subscriber. Replace `yolo11n.pt` with `yolo11n.engine` only if that
TensorRT engine has already been exported and validated on this Jetson.

### 6.4 Terminal 3 — RTMPose skeleton + hand wave on CPU

```bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  image_topic:=/ai_camera/color/image_raw \
  tracking_topic:=/yolo/tracking \
  max_people:=3
```

**Critical:** YOLO uses the GPU, but RTMPose uses **ONNX Runtime CPU** in
the current Jetson environment. RTMPose `device:=cuda:0` fails because
`CUDAExecutionProvider` is absent (the installed providers were reported
as `AzureExecutionProvider` and `CPUExecutionProvider`). Do not
mistake the PyTorch GPU setting for an ONNX Runtime GPU setting.

Unlike the older backend branch, the main-branch HandWave launch **does**
support `max_people`. The skeleton image is published at
`/pose/image_debug` **only while a subscriber is connected**;
the hand-wave results are `/pose/wave_detected` and
`/pose/wave_status`.

### 6.5 Terminal 4 — View skeleton debug image

For a working graphical desktop/X11:

```bash
ros2 run rqt_image_view rqt_image_view /pose/image_debug
```

For SSH-only/headless operation, start a browser video server on Jetson:

```bash
sudo apt install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

Open from a laptop on the trusted LAN:

```text
http://<JETSON_IP>:8080/stream_viewer?topic=/pose/image_debug
```

Check:

```bash
ros2 topic list | grep -E '^/(ai_camera|yolo|pose)/'
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/image_debug
ros2 topic echo /pose/wave_detected
```

Do not expose the unauthenticated video server directly to the Internet.
A plain SSH session without X forwarding cannot display `rqt_image_view`.

### 6.6 Start the complete AI pipeline (AI Camera -> YOLO11 TensorRT -> ByteTrack -> RTMPose)

The pipeline integrates Astra Pro AI Camera, YOLO11n TensorRT inference, ByteTrack person tracking, and RTMPose skeleton/hand-wave detection. All defaults are pre-configured for low latency and high stability on Jetson Orin Nano.

#### 6.6.1 Default Parameter Specification

| Module | Parameter | Default Value | Notes / Description |
| :--- | :--- | :--- | :--- |
| **YOLO** | `model` | `yolo11n.engine` | TensorRT FP16 engine optimized for Jetson Orin Nano |
| | `device` | `cuda:0` | GPU Jetson |
| | `classes` | `"0"` | Person class only (`0`) |
| | `threshold` | `0.35` | Confidence score threshold (balanced for ByteTrack) |
| | `iou` | `0.7` | NMS IoU threshold |
| | `imgsz_width` | `640` | Input inference width |
| | `imgsz_height` | `384` | Input inference height (384 saves 20% compute over 480) |
| | `half` | `True` | FP16 inference |
| | `max_det` | `5` | Maximum 5 detections per frame |
| | `skip_frames` | `1` | Skips 1 frame between inferences (processes at 15 FPS from 30 FPS stream) |
| | `fuse_model` | `False` | Disabled because TensorRT engine graph is already fused |
| | `use_tracking` | `True` | Enables ByteTrack tracking node |
| | `use_debug` | `False` | Disables debug image publishing by default to save 10-15% CPU |
| | `use_3d` | `False` | Kept False until RGB-Depth extrinsics calibration is finalized |
| **ByteTrack** | `tracker` | `bytetrack.yaml` | Detection-only tracking |
| | `tracking_image_width` | `640` | Original AI RGB frame width |
| | `tracking_image_height` | `480` | Original AI RGB frame height |
| **RTMPose** | `backend` | `rtmpose` | RTMPose-S ONNX SDK |
| | `device` | `cpu` | Multi-threaded CPU execution (intra-op=4, inter-op=2) |
| | `max_people` | `5` | Up to 5 nearest people selected via `NearestTrackSelector` |
| | `pose_input_width` | `192` | Pose input width |
| | `pose_input_height` | `256` | Pose input height |
| | `confidence` | `0.25` | Input bbox score threshold |
| | `keypoint_threshold` | `0.40` | Keypoint visibility threshold |
| **Camera** | `front/rear/ai_depth` | `160x120 @ 15fps` | Ultra-lightweight depth configuration |
| | `ai_color` | `640x480 @ 30fps` | RGB stream resolution and framerate |
| | `device_num` | `3` | Total Astra driver nodes |

---

#### 6.6.2 Quick Start (Lệnh chạy rút gọn - dùng toàn bộ tham số mặc định)

**Cách 1: Khởi động toàn bộ pipeline bằng 1 script duy nhất**

```bash
cd ~/ros2_ws
bash start_person_following.sh
```

**Cách 2: Khởi động từng module bằng ROS 2 launch rút gọn (không cần gõ tham số)**

Terminal 1 (Cameras):
```bash
ros2 launch mo_hinh three_astra.launch.py
```

Terminal 2 (YOLO11 TensorRT & ByteTrack):
```bash
ros2 launch yolo_bringup yolo.launch.py
```

Terminal 3 (RTMPose Skeleton & HandWave):
```bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py
```

---

#### 6.6.3 Full Configuration (Lệnh chạy đầy đủ tham số khi cần tùy biến)

Nếu cần tùy biến hoặc ghi đè tham số trực tiếp trên CLI:

**1. Khởi động 3 Camera Astra Pro đầy đủ tham số:**
```bash
ros2 launch mo_hinh three_astra.launch.py \
  front_serial:=17110310855 \
  rear_serial:=17122514440 \
  ai_serial:=17122611013 \
  front_depth_width:=160 front_depth_height:=120 front_depth_fps:=15 \
  rear_depth_width:=160 rear_depth_height:=120 rear_depth_fps:=15 \
  ai_depth_width:=160 ai_depth_height:=120 ai_depth_fps:=15 \
  ai_color_width:=640 ai_color_height:=480 ai_color_fps:=30 \
  device_num:=3 enable_front:=true enable_rear:=true enable_ai:=true enable_ai_depth:=true
```

**2. Khởi động YOLO11 TensorRT & ByteTrack đầy đủ tham số:**
```bash
ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.engine \
  device:=cuda:0 \
  classes:=0 \
  threshold:=0.35 \
  iou:=0.7 \
  max_det:=5 \
  skip_frames:=1 \
  imgsz_width:=640 \
  imgsz_height:=384 \
  half:=True \
  fuse_model:=False \
  use_tracking:=True \
  use_3d:=False \
  use_debug:=False \
  input_image_topic:=/ai_camera/color/image_raw
```

**3. Khởi động RTMPose Skeleton & HandWave đầy đủ tham số:**
```bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  max_people:=5 \
  detector_model:=yolo11n.engine \
  image_topic:=/ai_camera/color/image_raw \
  tracking_topic:=/yolo/tracking
```

Hoặc ghi đè qua biến môi trường khi chạy `start_person_following.sh`:
```bash
PERSON_MODEL=yolo11n.engine YOLO_MAX_DET=5 YOLO_SKIP_FRAMES=1 HAND_WAVE_MAX_PEOPLE=5 bash start_person_following.sh
```

---

Default AI routing:
- `/ai_camera/color/image_raw` -> YOLO detector + tracker + RTMPose
- `/yolo/tracking` -> RTMPose person bounding boxes
- `/pose/image_debug` -> skeleton visualization
- `/pose/wave_detected` -> raised-hand detection

**Important:** This is an AI RGB / 2D pipeline, not a calibrated 3D
person follower. `person_follower.launch.py` now defaults to 2D YOLO
without publishing `/person_tracking`. To use the 3D follower, explicitly
set `use_3d:=True` and supply registered, calibrated matching RGB and Depth
topics; Front/Rear Depth cannot be silently combined with AI RGB.

To prevent regression to legacy topic names:

```bash
python3 tools/test_ai_topic_contract.py
```

### 6.7 Verify running subscribers and physical camera selection

```bash
ros2 node info /yolo/yolo_node
ros2 node info /yolo/tracking_node
ros2 node info /yolo/debug_node
ros2 node info /hand_wave_detection

ros2 topic info /ai_camera/color/image_raw -v
ros2 param get /ai_camera/color/usb_cam video_device
```

All image subscribers in this AI pipeline must use
`/ai_camera/color/image_raw`. A correct ROS topic does not by itself prove
the UVC camera is the intended physical device: verify the
`ai_video_device` USB-by-path mapping if the image looks like Rear RGB.




## 7) Robot UI

The repository provides one **native PyQt5 touchscreen kiosk**, with Home, Navigation, and Chat/LLM screens.

### 7.1 Install UI Dependencies

On Ubuntu 22.04 / ROS 2 Humble:

~~~bash
sudo apt update
sudo apt install python3-pyqt5 python3-opencv python3-numpy
~~~

### 7.2 Build the UI

Run this after changing UI Python code, launch files, configuration, RAG documents,
or visual assets:

~~~bash
export WS="$(git rev-parse --show-toplevel)"
cd "$WS"

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select robot_ui
source "$WS/install/setup.bash"

ros2 pkg prefix robot_ui
~~~

### 7.3 Run the Native UI on the Real Robot

The real robot must use the system clock:

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch robot_ui kiosk.launch.py \
  windowed:=false \
  hide_cursor:=true \
  use_sim_time:=false
~~~

The launch defaults are already suitable for the deployed touchscreen, so the
short equivalent command is:

~~~bash
ros2 launch robot_ui kiosk.launch.py
~~~

For development on the real robot or laptop, use a resizable window:

~~~bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=false
~~~

### 7.4 Recommended Real-Robot Terminal Layout

Run the main ROS stacks in separate terminals.

**Terminal 1 — Hardware and odometry**

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch mo_hinh real_odom.launch.py odom_source:=esp
~~~

RF2O can be used instead:

~~~bash
ros2 launch mo_hinh real_odom.launch.py odom_source:=rf2o
~~~

**Terminal 2 — Localization and Nav2**

Recommended current configuration for the native UI is AMCL:

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=amcl \
  use_sim_time:=false \
  use_rviz:=false
~~~

To use SLAM Toolbox localization instead:

~~~bash
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=slam_toolbox \
  use_sim_time:=false \
  use_rviz:=false
~~~

**Terminal 3 — Robot backend**

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch robot_bringup backend.launch.py
~~~

**Terminal 4 — Native Robot UI**

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch robot_ui kiosk.launch.py
~~~

The backend provides /robot_status, /navigation/status, saved locations, and
other interfaces consumed by the kiosk.

### 7.5 Run the UI with Gazebo

Start the simulation first:

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch mo_hinh virtual_robot_gazebo.launch.py
~~~

Start Nav2/localization for the virtual robot in another terminal. For example:

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:="$WS/src/mo_hinh/maps/virtual_lab_map.yaml" \
  params_file:="$WS/src/mo_hinh/config/nav2_params.yaml"
~~~

Start the backend:

~~~bash
ros2 launch robot_bringup backend.launch.py
~~~

Then launch the kiosk with the Gazebo clock:

~~~bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=true
~~~

Recommended simulation layout:

~~~text
Terminal 1: Gazebo
Terminal 2: Nav2 + localization
Terminal 3: Robot backend
Terminal 4: Robot UI
~~~


### 7.6 Configure Chat / LLM / RAG — use saved 9router settings

The native kiosk contains the LLM client, but does **not** start the 9router
service. 9router must already be running and reachable.

The Jetson's locally saved, **untracked** configuration is normally at
`/home/orin/ros2_ws/config/llm.local.env`. It should provide
`LLM_BASE_URL`, `LLM_API_KEY`, and `LLM_MODEL`. Never commit that file
or paste the API key into the repository.

If those variables are not exported automatically in the launching terminal,
load the existing file, without entering or printing the key:

~~~bash
set -a
source ~/ros2_ws/config/llm.local.env
set +a

printf 'LLM_BASE_URL=%s\nLLM_MODEL=%s\n' "$LLM_BASE_URL" "$LLM_MODEL"
if [ -n "$LLM_API_KEY" ]; then
  echo "LLM_API_KEY: configured"
else
  echo "LLM_API_KEY: missing"
fi
~~~

If `~/.bashrc` already loads this file, sourcing it again is unnecessary.
A variable in one terminal is **not** automatically inherited by a
separately launched GUI process. Load the file in the **same terminal**
used to launch the kiosk.

Verify the endpoint without exposing the key:

~~~bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
~~~

Then, on a Jetson with a graphical desktop or working X11:

~~~bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=false
~~~

If 9router is on a different machine, the `LLM_BASE_URL` in the env file
must point to that host rather than `localhost`.

Local RAG documents: `src/robot_ui/robot_ui/kiosk/rag_docs/`.
Rebuild `robot_ui` after modifying Python code or RAG documents.

### 7.7 ROS Interfaces Used by the Native UI

The kiosk subscribes to:

~~~text
/robot_status
/navigation/status
/map
/amcl_pose
/odom
/plan
~~~

It loads saved locations through:

~~~text
/config/get_location_list
/config/get_location
~~~

Navigation interfaces:

~~~text
/navigation/direct_go
/navigate_to_pose
~~~

Useful checks:

~~~bash
ros2 topic echo /robot_status --once
ros2 topic echo /navigation/status --once
ros2 topic echo /map --once
ros2 topic echo /amcl_pose --once
ros2 topic echo /odom --once
ros2 topic echo /plan --once

ros2 action list | grep navigate_to_pose
~~~

More UI-specific details are documented in src/robot_ui/README.md.

## 8) Robot Backend

The backend is split into ROS 2 packages with explicit interfaces between mission
logic, configuration, system monitoring, and Nav2.

### Launch the backend

```bash
source "$WS/install/setup.bash"
ros2 launch robot_bringup backend.launch.py
```

`backend.launch.py` starts:
- `robot_config_manager`
- `robot_system_monitor`
- `robot_navigation_manager`
- `robot_mission_control`

### Main backend interfaces

Configuration services:

```text
/config/save_location
/config/get_location
/config/delete_location
/config/get_location_list
/config/save_route
/config/get_route
/config/set_language
/config/finish_setup
```

Navigation:

```text
Action: /navigation/direct_go
Topic : /navigation/status
```

Mission control:

```text
/mission/start_patrol
/mission/pause_patrol
/mission/status_log
```

System status:

```text
/robot_status
```

The mission Behavior Tree uses the priority order:

```text
Emergency -> Battery Low -> Water Low -> Customer -> Patrol
```

The patrol action resolves named route waypoints through Config Manager, sends
`/navigation/direct_go` goals to Navigation Manager, and Navigation Manager
converts semantic locations into `NavigateToPose` goals for Nav2.

### Current backend limitations

- `robot_mapping_nav` exists as a separate package but is not started by
  `backend.launch.py`; its mapping service handlers are currently placeholders.
- Localization health in `robot_system_monitor` is currently mocked as healthy.
- Charging, refill, and customer-service BT actions are currently simulated;
  Patrol is the action integrated with Navigation Manager/Nav2.

## 9) Repository Path Policy

Avoid hard-coded workspace paths such as `/home/orin/ros2_ws`. Use `$WS`,
ROS package-share lookup, or paths relative to the owning configuration file.

## 10) Process Management & Emergency Stop / Kill All Tasks

### 10.1 Quick One-Liner to Kill All Tasks
To immediately terminate all running ROS 2 nodes, launch wrappers, UI kiosks, AI models, cameras, and background processes:

```bash
killall -9 ros2 _ros2_daemon rviz2 robot_ui_kiosk 9router 2>/dev/null || true
pkill -9 -f "(ros2|nav2|robot_|kiosk|9router|rviz2|obstacle_detector|astra_camera|sllidar|hoverboard)" 2>/dev/null || true
ros2 daemon stop && ros2 daemon start
```

### 10.2 Emergency Stop for Motors & Motion
If the robot is moving and needs an immediate velocity stop without killing nodes:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 service call /mission/emergency_stop std_srvs/srv/Trigger "{}"
```

### 10.3 Component-Specific Kill Commands
- **Kill UI & Voice Chat only:**
  ```bash
  pkill -9 -f "(robot_ui_kiosk|9router)"
  ```
- **Kill Navigation & SLAM only:**
  ```bash
  pkill -9 -f "(nav2|slam_toolbox|amcl|controller_server|planner_server|bt_navigator)"
  ```
- **Kill Camera & Sensor Drivers only:**
  ```bash
  pkill -9 -f "(astra_camera|sllidar|obstacle_detector|usb_cam|depthimage_to_laserscan)"
  ```
- **Kill Mission Controller & Backend only:**
  ```bash
  pkill -9 -f "(robot_mission_control|robot_navigation_manager|robot_config_manager|robot_system_monitor)"
  ```

### 10.4 Convenient Shell Alias (Optional)
Add this alias to `~/.bashrc` on both PC and Orin for instant shutdown via `killrobot`:

```bash
alias killrobot='killall -9 ros2 _ros2_daemon rviz2 robot_ui_kiosk 9router 2>/dev/null; pkill -9 -f "(ros2|nav2|robot_|kiosk|9router|rviz2|obstacle_detector|astra_camera|sllidar|hoverboard)" 2>/dev/null; ros2 daemon stop; ros2 daemon start; echo "All robot tasks terminated."'
```

