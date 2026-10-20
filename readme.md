# AUTONOMOUS MECANUM ROBOT WORKFLOW (ROS 2)

## 0) Build & Source Workspace
```bash
WS=~/ros2_ws
cd ${WS}
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ${WS}/install/setup.bash
```

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
ros2 launch mo_hinh real_odom.launch.py odom_source:=esp esp_wheel_odom_mode:=wheels_1_4
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
  map_yaml:=${WS}/src/mo_hinh/maps/my_map.yaml

# Option 2: SLAM Toolbox Localization + Nav2 (using PoseGraph)
ros2 launch mo_hinh real_localization_nav2.launch.py \
  localization_mode:=slam_toolbox \
  map_graph:=${WS}/src/mo_hinh/maps/my_slam_graph
```

## 2) Recommended Operational Workflow
1. Start `real_odom.launch.py` to publish `/scan`, `/imu/data`, `/odom`, and TF.
2. Run SLAM: `real_slam.launch.py`.
3. Save PoseGraph map after scanning:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${WS}/src/mo_hinh/maps/my_slam_graph'}"
```
4. Save 2D Occupancy Grid map (YAML/PGM) for AMCL:
```bash
ros2 run nav2_map_server map_saver_cli -f ${WS}/src/mo_hinh/maps/my_map
```
5. Switch to Localization + Nav2 using `real_localization_nav2.launch.py` (keep `real_odom.launch.py` running).

## 3) Important Notes (TF & Goal Success)
- All real robot nodes must set `use_sim_time:=false`.
- Do not run two localization stacks simultaneously (AMCL and SLAM Toolbox Localization).
- Do not run two `odom -> base_footprint` TF sources simultaneously (ESP and RF2O).

## 4) Virtual Robot Simulation (Gazebo)
```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
ros2 launch mo_hinh virtual_slam.launch.py
```

## 5) Layer 1 Bringup & Obstacle Detector
```bash
ros2 launch layer1_bringup layer1.launch.py esp_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1
ros2 run depth_obstacle_detector obstacle_detector --ros-args -p config_file:=${WS}/my_map/cam.yaml
```

## 6) Jetson AI Pipeline: 3 Cameras + YOLO11 + RTMPose Skeleton

Use this runbook on branch **backend** with ROS 2 Humble. Each process runs
in its own terminal, and previously started processes stay running.

### Step 0: Update and build

```bash
cd ~/ros2_ws
git status --short
git fetch origin
git switch backend
git pull --ff-only origin backend

source /opt/ros/humble/setup.bash
sudo apt install -y ros-humble-usb-cam
colcon build --symlink-install --packages-up-to \
  mo_hinh yolo_bringup hand_wave_detection robot_ui
source ~/ros2_ws/install/setup.bash
```

Do not discard uncommitted changes to switch branches. Installed dependencies
must also include the Astra driver, Jetson-compatible PyTorch CUDA,
Ultralytics, RTMLib and ONNX Runtime.

**Run in every new terminal before any ROS 2 launch command:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1: Front Depth, Rear Depth and AI RGB

```bash
ros2 launch mo_hinh three_astra.launch.py \
  front_depth_fps:=15 \
  rear_depth_fps:=15 \
  ai_color_fps:=30
```

- Front Astra OpenNI: `/front_camera/depth/image_raw`.
- Rear Astra OpenNI: `/rear_camera/depth/image_raw`.
- AI USB camera: `/ai_camera/color/image_raw`.
- The AI UVC interface is selected by the physical USB port:
  `/dev/v4l/by-path/platform-3610000.usb-usb-0:2.2.1:1.0-video-index0`.
  If connected to another USB port, override `ai_video_device:=...`.
- Depth FPS 15 is **requested**, not guaranteed. Orbbec may fall back to a
  supported video mode such as 640x480@30. Verify real FPS with
  `ros2 topic hz` and `set depth video mode` logs.

For an isolated AI camera test (no Depth):

```bash
ros2 launch mo_hinh three_astra.launch.py \
  enable_front:=false enable_rear:=false enable_ai:=true
```

### Terminal 2: YOLO11 GPU person detection and tracking

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

This produces `/yolo/detections`, `/yolo/tracking`, `/yolo/dbg_image`.
`image_reliability:=2` selects Best Effort QoS for image streams.
AI RGB has no co-registered Depth, so 3D inference is disabled.
A pre-exported TensorRT `.engine` may replace `yolo11n.pt` after it is
validated on this Jetson.

### Terminal 3: RTMPose CPU skeleton / raised-hand detection

```bash
ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  image_topic:=/ai_camera/color/image_raw \
  tracking_topic:=/yolo/tracking
```

**Keep YOLO on CUDA but RTMPose on CPU.** RTMPose uses ONNX Runtime, whose
installed providers on this Jetson are
`['AzureExecutionProvider', 'CPUExecutionProvider']`.
`device:=cuda:0` on RTMPose causes a startup failure because
`CUDAExecutionProvider` is missing. Do not reinstall CUDA to fix a launch
argument mismatch. The backend launch file does not expose
`max_people` as a launch argument; edit the HandWave config YAML if needed.

Outputs: `/pose/image_debug` (bbox + skeleton),
`/pose/wave_detected`, `/pose/wave_status`.

### Terminal 4: View skeleton

If running with a desktop or working X11 forwarding:

```bash
ros2 run rqt_image_view rqt_image_view /pose/image_debug
```

For SSH/headless operation, stream to the laptop browser:

```bash
sudo apt install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server --ros-args -p port:=8080
```

Open on the trusted local network:
`http://<JETSON_IP>:8080/stream_viewer?topic=/pose/image_debug`.

Verification:

```bash
ros2 topic list | grep -E '^/(ai_camera|yolo|pose)/'
ros2 topic hz /yolo/tracking
ros2 topic hz /pose/image_debug
ros2 topic echo /pose/wave_detected
```

Do not expose web_video_server to untrusted networks without authentication.

**Legacy script warning:** `start_person_following.sh` still opens a single
`astra_pro.launch.xml`, uses old `/camera/*` topics, and forcefully
kills existing camera/YOLO processes. Do not run that script with the new
three-camera launch until the script has been migrated.

## 7) Robot Web Dashboard on backend

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch robot_ui robot_ui.launch.py
```

Open `http://<JETSON_IP>:8000`. This is the backend branch's FastAPI/React
administrator dashboard, **not** the LLM chatbot.
The native chatbot's `robot_ui kiosk.launch.py` and
`robot_ui.kiosk.llm_client` exist on main but are **absent from backend**.
They need a separate integration before kiosk can run on a clean backend build.

## 8) LLM/9router: load the saved API key, do not re-enter it

The local, untracked configuration file is
`/home/orin/ros2_ws/config/llm.local.env`, with
`LLM_BASE_URL`, `LLM_API_KEY`, and `LLM_MODEL`.
Never print or commit the secret key.

If the file is already loaded by `~/.bashrc`, no further key entry is needed.
Otherwise load it in the terminal that launches the LLM client:

```bash
set -a
source ~/ros2_ws/config/llm.local.env
set +a

printf 'LLM_BASE_URL=%s\nLLM_MODEL=%s\n' "$LLM_BASE_URL" "$LLM_MODEL"
if [ -n "$LLM_API_KEY" ]; then
  echo 'LLM_API_KEY: configured'
else
  echo 'LLM_API_KEY: missing'
fi

curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
```

The LLM service (9router) must be started separately. A saved env file only
provides credentials; it does not start the service. Once the native kiosk
code has been integrated into backend, it can inherit these variables and
run with `ros2 launch robot_ui kiosk.launch.py`.

