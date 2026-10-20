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

The RGB camera is selected by physical USB port, not OpenNI serial:
`/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2.1:1.0-video-index0`.
Override `ai_video_device:=...` if the camera is moved to another port.

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

### 6.6 Start the complete AI pipeline with one command

The updated script runs `three_astra.launch.py`, YOLO 2D person tracking
on `/ai_camera/color/image_raw`, and RTMPose skeleton/hand-wave detection
on the same AI RGB topic. No `/camera/color/image_raw` fallback is used.

```bash
cd ~/ros2_ws
bash start_person_following.sh
```

Default AI routing:
- `/ai_camera/color/image_raw` -> YOLO detector + tracker + RTMPose
- `/yolo/tracking` -> RTMPose person bounding boxes
- `/pose/image_debug` -> skeleton visualization
- `/pose/wave_detected` -> raised-hand detection

The script runs YOLO on `cuda:0` and RTMPose on `cpu` by default. Override
non-topic options via environment variables, for example
`HAND_WAVE_MAX_PEOPLE=1 bash start_person_following.sh`. All launch logs
are written under `log/ai_pipeline_YYYYMMDD_HHMMSS/`. Press Ctrl+C to
stop the script's child launches; it does not kill unrelated camera nodes.

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

The web Vision dashboard monitors these camera information topics:
`/ai_camera/color/camera_info`, `/front_camera/depth/camera_info`,
and `/rear_camera/depth/camera_info`.


### 6.8 Refresh web Vision dashboard assets (optional)

Only required when using the web dashboard at port 8000, not for
the camera/YOLO/RTMPose runtime:

```bash
cd ~/ros2_ws/src/robot_ui/frontend
npm ci
npm run build
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_ui
source ~/ros2_ws/install/setup.bash
```

Requires Node.js 20.19+ as described in the UI README. The updated Vision
screen distinguishes AI RGB, Front Depth and Rear Depth CameraInfo topics.
Simply building the Python ROS package without compiling the React frontend
may leave an older web page visible.

## 7) Robot UI

The repository provides two UI modes:

- **Native touchscreen kiosk**: Home, Navigation, and Chat/LLM screens.
- **Web diagnostics/dashboard**: browser-based monitoring and diagnostics.

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

### 7.6 Run the Web Diagnostics UI

The web UI is separate from the native kiosk:

~~~bash
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

ros2 launch robot_ui robot_ui.launch.py
~~~

By default the server binds to:

~~~text
0.0.0.0:8000
~~~

The current web configuration is primarily for monitoring and diagnostics.
Navigation controls, process controls, and teleoperation are disabled in
src/robot_ui/config/robot_ui.yaml.

### 7.7 Configure Chat / LLM / RAG — use saved 9router settings

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

### 7.8 ROS Interfaces Used by the Native UI

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
