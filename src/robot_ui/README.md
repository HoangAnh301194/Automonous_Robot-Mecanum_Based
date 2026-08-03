# Robot UI

Web admin/developer dashboard for the ROS 2 robot stack.

Current scaffold is intentionally read-only. It provides:

- FastAPI server and WebSocket telemetry endpoint.
- ROS 2 bridge for hardware, map, TF, LaserScan, Nav2 paths, costmaps and action status.
- CPU, per-core load, memory, disk, temperature and Jetson GPU monitoring.
- Topic-rate health, `/diagnostics`, `/rosout` and runtime event monitoring.
- Mission state, waypoint/Nav2/intercept status from `/mission/status`.
- CameraInfo, YOLO, wave and depth-obstacle pipeline health.
- React/TypeScript dashboard with six developer pages.
- ROS launch file and deployment service example.

## Directory Layout

```text
robot_ui/
??? config/                 Runtime YAML configuration
??? deploy/                 Deployment examples
??? docs/                   Architecture, API contract, roadmap
??? frontend/               Vite + React + TypeScript source
??? launch/                 ROS 2 launch files
??? resource/               ament package marker
??? robot_ui/               Python backend package
?   ??? api/                REST and WebSocket routers
?   ??? web_dist/           Built frontend output
??? package.xml
??? setup.py
??? requirements.txt
```

## Build Frontend

Requires Node.js `20.19+`.

```bash
cd ~/ros2_ws/src/robot_ui/frontend
npm install
npm run build
```

Vite writes production files to `robot_ui/web_dist`.

## Windows UI Preview (No Jetson)

Requires Node.js `20.19+`. ROS 2, Python backend and Jetson are not required.

From PowerShell:

```powershell
cd D:\PTIT\NCKH\Automonous_Robot-Mecanum_Based\src\robot_ui
.\preview_windows.ps1
```

Or double-click `preview_windows.bat`.

The launcher installs frontend dependencies when needed, opens
`http://127.0.0.1:5173`, and prints LAN URLs for other devices. The Vite
development build uses demo telemetry and a demo ROS graph.

If Windows Firewall asks for access, allow Node.js on private networks. Stop
the preview with `Ctrl+C`.

## Install Backend Dependencies

```bash
cd ~/ros2_ws/src/robot_ui
python3 -m pip install -r requirements.txt
```

## Build ROS Package

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

## Run

```bash
ros2 launch robot_ui robot_ui.launch.py
```

Open from another machine on the same LAN:

```text
http://<robot-ip>:8000
```

Useful endpoints:

```text
GET /api/v1/health
GET /api/v1/state
GET /api/v1/config
GET /api/v1/ros/graph
WS  /ws/telemetry
GET /docs
```

Navigation telemetry defaults:

```text
/map
/odom
/scan
/goal_pose
/plan
/local_plan
/global_costmap/costmap_raw
/local_costmap/costmap_raw
/navigate_to_pose/_action/status
/diagnostics
/rosout
TF: map -> base_footprint
```

Mission and vision telemetry defaults:

```text
/mission/status
/camera/color/camera_info
/camera/depth/camera_info
/yolo/detections
/pose/wave_detected
/pose/wave_status
/scan_obstacles
```

`nhiemvuboss` publishes `/mission/status` as a bounded JSON object inside
`std_msgs/String` at 2 Hz. It includes mission mode, waypoint progress, Nav2
state, distance remaining, wait time and person-intercept state.

Map and costmap grids are downsampled before entering WebSocket snapshots. The
dashboard remains read-only; navigation goal and process controls are disabled.

Only topics listed under `ros.expected_rates_hz` receive rate and stale checks.
Their state is reported as `WAITING`, `OK`, `WARN` or `STALE`. Thresholds and
retention limits live under `ros.diagnostics` in `config/robot_ui.yaml`.

## Development Mode

Run backend on port `8000`, then:

```bash
cd frontend
npm run dev
```

Vite proxies `/api` and `/ws` to the backend.

## Safety Boundary

The scaffold does not expose navigation, process control or teleoperation. These remain disabled in `config/robot_ui.yaml` until authentication, command whitelisting and disconnect safety are implemented.
