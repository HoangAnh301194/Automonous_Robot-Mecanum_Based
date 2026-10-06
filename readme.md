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

### 7.7 Configure Chat / LLM / RAG

The native kiosk contains the LLM client, but it does **not** start the LLM server.
A compatible 9router/OpenAI-style endpoint must already be running.

Set the environment variables in the same terminal that launches the kiosk:

~~~bash
export LLM_BASE_URL="http://localhost:20128/v1"
export LLM_API_KEY="YOUR_9ROUTER_API_KEY"
export LLM_MODEL="hehe"
~~~

If 9router runs on another machine, replace localhost with that machine's IP.

Verify the endpoint:

~~~bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
~~~

Then launch the kiosk:

~~~bash
ros2 launch robot_ui kiosk.launch.py
~~~

Local RAG documents are stored in:

~~~text
src/robot_ui/robot_ui/kiosk/rag_docs/
~~~

Rebuild robot_ui after changing Python code or RAG documents.

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
