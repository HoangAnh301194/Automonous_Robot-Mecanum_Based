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

## 6) AI Pipeline (Camera + YOLO11 + Hand Wave RTMPose)

### Step 0: Build AI Packages

```bash
cd "$WS"
colcon build --symlink-install \
  --packages-select yolo_msgs yolo_ros yolo_bringup hand_wave_detection
source "$WS/install/setup.bash"
```

### Step 1: Launch Camera Driver (Orbbec Astra Pro)

```bash
source "$WS/install/setup.bash"
ros2 launch astra_camera astra_pro.launch.xml
```

Published topics include:
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

### Step 2: Launch YOLO11 Detection (TensorRT)

Export the TensorRT engine once on the Jetson:

```bash
yolo export model=yolo11n.pt format=engine imgsz=384,640 half=True device=0
```

Launch detection:

```bash
source "$WS/install/setup.bash"

ros2 launch yolo_bringup yolo.launch.py \
  model:=yolo11n.engine \
  use_tracking:=False \
  skip_frames:=0 \
  use_debug:=True \
  device:=cuda:0 \
  classes:=0 \
  imgsz_height:=384 \
  imgsz_width:=640 \
  max_det:=10 \
  namespace:=yolo \
  input_image_topic:=/camera/color/image_raw
```

The optimized path performs person-only detection at 384x640 and publishes
`/yolo/detections`. Debug output is available on `/yolo/dbg_image`.

### Step 3: Launch Hand Wave Detection (RTMPose)

```bash
source "$WS/install/setup.bash"

ros2 launch hand_wave_detection hand_wave_detection.launch.py \
  backend:=rtmpose \
  device:=cpu \
  max_people:=5 \
  image_topic:=/camera/color/image_raw \
  tracking_topic:=/yolo/detections
```

Notes:
- The default RTMPose path uses ONNX Runtime on CPU.
- `max_people` limits the number of people passed to pose inference.
- The main branch keeps the FPS instrumentation, nearest-person selection, and
  asynchronous latest-frame worker used by the optimized hand-wave pipeline.

### Step 4: Verify Output Topics & Visualization

```bash
ros2 topic echo /pose/wave_status
ros2 topic echo /pose/wave_detected
ros2 topic echo /person_tracking
ros2 topic echo /yolo/detections_3d
ros2 run rqt_image_view rqt_image_view /pose/image_debug
```

### Step 5: Run the Automated AI Pipeline

The startup script now resolves the workspace from its own location and defaults
to `yolo11n.engine`, RTMPose on CPU, person class only, and the optimized image
size.

```bash
cd "$WS"
bash start_person_following.sh
```

Example override:

```bash
YOLO_DEVICE=cuda:0 \
HAND_WAVE_DEVICE=cpu \
HAND_WAVE_MAX_PEOPLE=5 \
bash "$WS/start_person_following.sh"
```

## 7) Robot UI

### Web diagnostics/dashboard

```bash
source "$WS/install/setup.bash"
ros2 launch robot_ui robot_ui.launch.py
```

### Native touchscreen kiosk

```bash
source "$WS/install/setup.bash"
ros2 launch robot_ui kiosk.launch.py
```

The kiosk includes Home, Navigation, and Chat screens. Chat configuration and RAG
document setup are documented in `src/robot_ui/README.md`.

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
