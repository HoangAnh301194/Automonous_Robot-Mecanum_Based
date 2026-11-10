# Robot UI
The interface contains three screens:

- **Home:** loops the configured Dasai Mochi emotion video.
- **Navigation:** displays the ROS map, robot pose, planned path and selected
  goal. Press Start to send a `NavigateToPose` goal and Stop to cancel it.
- **Chat:** provides a native chatbot backed by 9router through its
  OpenAI-compatible API. The client loads `.txt` files from `kiosk/rag_docs/`
  and includes the current robot status in the prompt.

## Quick Start: GUI v3 + Souta Kiosk + 9router (simulator branch)

This section is the recommended deployment workflow for Ubuntu 22.04 / ROS 2
Humble, including Jetson Orin Nano. **GUI v3 and the Souta Kiosk are two
separate PyQt5 windows**. GUI v3 manages robot bringup, mapping, localization,
Nav2 and map editing; the Kiosk provides Home, Navigation and AI Chat/RAG.
Launching either GUI does **not** start the 9router server.

### 0. Update and build (one-time, or after pulling code changes)

Run from a **graphical desktop terminal** on the machine hosting both windows.
Do not discard uncommitted files when changing branches.

```bash
cd ~/ros2_ws
git status --short
git fetch origin
git switch simulator
git pull --ff-only origin simulator

export WS="$PWD"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_ui robot_bringup mo_hinh
source "$WS/install/setup.bash"
```

If dependencies are missing, see "Install Native UI Dependencies" below.
For an SSH session, `echo "$DISPLAY"` must refer to a working X11 display;
a plain headless SSH shell cannot show either Qt window. Run on the attached
desktop or use an appropriately configured X11/VNC session.

### 1. Configure and run 9router (before testing Chat)

Ensure the **separate 9router server** is already running, and its
OpenAI-compatible endpoint and selected model/provider work. Start it using
the installation method already set up on the host (service, container or
application); there is **no 9router server startup command in this repository**.

Edit the **untracked local** file `~/ros2_ws/config/llm.local.env`:

```dotenv
LLM_BASE_URL=http://127.0.0.1:20128/v1
LLM_API_KEY=replace-with-your-local-key
LLM_MODEL=replace-with-an-available-model
```

```bash
chmod 600 ~/ros2_ws/config/llm.local.env
```

Use `127.0.0.1` only when 9router runs **on the same machine as the Kiosk**.
If 9router runs on an Ubuntu laptop while Kiosk runs on Jetson, use the
laptop's reachable Hotspot/LAN IP (e.g. `http://10.42.0.1:20128/v1`, only
if that is the laptop's actual IP). The server must listen on that network
interface, and the firewall must allow the connection on the trusted LAN.

**Do not source this file to launch the Kiosk.** The `kiosk.launch.py` entry
point automatically reads these three settings at every launch, before loading
the chatbot. Editing the file requires restarting Kiosk, **not rebuilding**
the package. Never commit the real key.

### 2. Terminal A: start GUI v3 first

```bash
cd ~/ros2_ws
export WS="$PWD"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
python3 src/gui_v3.py
```

In GUI v3, choose `Simulation (Gazebo)` or `Robot thật`, then
`SLAM Mapping` or `Localization` and the localization backend as needed.

- **Simulation / navigation:** select `Simulation (Gazebo)`, `Localization`
  and a valid saved map, then start **Layer 1 (Odom)**, **Layer 2 (Mode)**
  and **Nav2** (if not already active). GUI v3's simulation Localization
  launch supplies localization, while its separate Start Nav2 launches the
  navigation stack.
- **Real robot / navigation:** select `Robot thật`, `Localization` and
  the desired map/backend; start Layer 1 and Layer 2. The real-robot
  localization launch includes Nav2; do **not** start a duplicate Nav2 stack.
- For SLAM mapping, the robot must move to expand the map; do not assume the
  Navigation page can route to a goal before localization/Nav2 is ready.

**Important:** GUI v3 currently cleans up known ROS 2/Gazebo processes at
startup **and on window close**. Therefore **start it before** the backend
and Kiosk. Do not use GUI v3's `Dọn sạch ROS2 / Gazebo` action while a
mission is in progress, and avoid duplicate Nav2/SLAM launches.

### 3. Terminal B: start the robot backend

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup backend.launch.py
```

The backend provides `/robot_status`, `/navigation/status`, location
services and mission components. GUI v3's Layer 1/2 startup does not
automatically replace this backend.

### 4. Terminal C: start the Souta Kiosk

For a **real robot** (system clock):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=false
```

For **Gazebo simulation** (simulation clock):

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=true
```

Expected startup messages include:

```text
[robot_ui] 9router config loaded from .../llm.local.env (URL/model loaded; API key configured)
Native touchscreen UI ROS adapter started.
```

These messages confirm **configuration loading and ROS node startup only**,
not that the 9router model/provider has accepted a chat request. Open the
Kiosk's **Chat** screen and submit a short message to verify it end-to-end.
The initial `LLM: CHƯA KẾT NỐI` label is currently a static default, **not
an active server health check**; do not diagnose connectivity from it alone.

### 5. Terminal D: verify ROS topics and goals

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E '^/(map|map1|amcl_pose|robot_status|navigation/status|plan)
Run these commands from any directory inside the cloned repository in every new
terminal before using ROS 2 packages from this workspace:

```bash
export WS="$(git rev-parse --show-toplevel)"
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For a source archive without Git metadata, open the workspace root and use
`export WS="$PWD"`. After relocating the workspace, rebuild and source it again;
old build/install artifacts can still contain absolute paths.

## Install Native UI Dependencies

On Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-opencv python3-numpy
```

## Configure the Kiosk Chatbot

The native kiosk uses the 9router OpenAI-compatible API, defaulting to
`http://localhost:20128/v1`. **9router must already be running**; launching
the kiosk does not start the LLM server.

On the Jetson, reuse the existing local, untracked configuration
`/home/orin/ros2_ws/config/llm.local.env` with `LLM_BASE_URL`,
`LLM_API_KEY`, and `LLM_MODEL`. Never commit this file or copy the API
key into documentation.

The Kiosk **automatically loads this file on every launch**. No manual
`source` or API key export is required. Both `KEY=value` and
`export KEY=value` lines are supported, including quoted values and comments.
The file must contain `LLM_BASE_URL`, `LLM_API_KEY` and `LLM_MODEL`.

A safe example layout (do not commit actual credentials):

```bash
LLM_BASE_URL=http://localhost:20128/v1
LLM_API_KEY=replace-with-your-local-key
LLM_MODEL=replace-with-an-available-model
```

Edit only this file to change the 9router endpoint/key/model, then **restart
Kiosk** to apply changes. The Kiosk loads the credentials, but does not start
the external 9router server. The server must already be running.

If your file is stored elsewhere, pass its path on launch:

```bash
ros2 launch robot_ui kiosk.launch.py \
  llm_env_file:=/absolute/path/to/llm.local.env
```

Configuration is parsed as plain data, **not executed as shell code**. Shell
expansion and commands inside the file are intentionally unsupported. No
credentials are printed in the Kiosk logs.

Check 9router access **from a terminal where the three LLM variables are loaded**
(for example, using the safe Python check in the Quick Start section). The
following curl command alone does not load the ENV file:

```bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
```

If 9router is on another computer, adjust the saved `LLM_BASE_URL`
to its reachable IP rather than `localhost`. The saved `LLM_MODEL`
must be valid on the connected server.

Launch the kiosk on a machine with a working graphical display:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=false
```

The chatbot loads `.txt` knowledge files from
`robot_ui/kiosk/rag_docs/`. Rebuild the `robot_ui` package after
changing Python code or those documents.

## Build the UI

Build after modifying Python code, launch files, configuration or assets:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# Activate a project virtual environment here only if your deployment uses one.
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

Verify that ROS 2 can find the package:

```bash
ros2 pkg prefix robot_ui
```

## Run a Windowed Simulation Preview

Use this mode during development. The mouse cursor remains visible and the
window can be resized:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=true
```

The Chat screen is available from the same kiosk process. For a complete
chatbot test, ensure 9router is running, launch the Kiosk (which reads the
configuration file automatically), and send a message from the Chat screen.

## Run Fullscreen with Gazebo

Use the Gazebo clock while displaying the UI fullscreen:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=false \
  hide_cursor:=true \
  use_sim_time:=true
```

## Run Fullscreen on the Real Robot

The real robot uses the system clock:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=false \
  hide_cursor:=true \
  use_sim_time:=false
```

The default launch values are already suitable for the real touchscreen, so
the shorter equivalent command is:

```bash
ros2 launch robot_ui kiosk.launch.py
```

## Start the Robot Backend

Open a separate terminal and prepare it as described above, then run:

```bash
ros2 launch robot_bringup backend.launch.py
```

The backend provides robot status, navigation status, saved locations and
other data consumed by the UI.

## Start the Gazebo Simulation

Open a separate prepared terminal:

```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
```

Navigation also requires Nav2 and localization to be running. If using the
saved virtual lab map, start Nav2 from another prepared terminal:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:="$WS/src/mo_hinh/maps/virtual_lab_map.yaml" \
  params_file:="$WS/src/mo_hinh/config/nav2_params.yaml"
```

Start the backend and Robot UI in their own terminals after Gazebo and Nav2 are
running.

## Recommended Simulation Terminal Layout

Use one process per terminal:

```text
Terminal 1: Gazebo simulation
Terminal 2: Nav2 and localization
Terminal 3: Robot backend
Terminal 4: Robot UI
```

Every terminal must source ROS 2 and `install/setup.bash` first.

## Optional Web UI User Service

`deploy/robot-ui.service.example` is a systemd **user** service for the web UI,
not the native kiosk. It uses a stable symlink in your home directory instead of
embedding a disk mount or username in the unit. After setting `WS`:

```bash
mkdir -p "$HOME/.local/share" "$HOME/.config/systemd/user"
ln -s "$WS" "$HOME/.local/share/robot-workspace"
cp "$WS/src/robot_ui/deploy/robot-ui.service.example" \
  "$HOME/.config/systemd/user/robot-ui.service"
systemctl --user daemon-reload
systemctl --user enable --now robot-ui.service
```

Create the symlink only once. If it already exists, check its target before
changing it. When relocating the workspace, stop the service, rebuild at the new
location, update the symlink and restart the service. Do not install this example
as a system-wide service.

## ROS Interfaces Used by the UI

The native UI subscribes to:

```text
/robot_status
/navigation/status
/map
/amcl_pose
/odom
/plan
```

It uses these services to load saved locations:

```text
/config/get_location_list
/config/get_location
```

It sends and cancels goals through the Nav2 action:

```text
/navigate_to_pose
```

## Inspect Live Data

List all active topics and their message types:

```bash
ros2 topic list -t
```

Check robot status:

```bash
ros2 topic echo /robot_status --once
```

Check the localized robot pose:

```bash
ros2 topic echo /amcl_pose --once
```

Check whether a map is being published:

```bash
ros2 topic echo /map --once
```

Check the generated navigation path:

```bash
ros2 topic echo /plan --once
```

Monitor the final velocity sent to Gazebo or the real robot:

```bash
ros2 topic echo /cmd_vel
```

Check whether the Nav2 action server is available:

```bash
ros2 action list | grep navigate_to_pose
```

## Emotions and Icons

Emotion names and video filenames are configured in:

```text
config/emotions.json
```

MP4 emotion files are stored in:

```text
emotion_dasaimochi/videomp4/
```

Navigation and menu image assets are stored in:

```text
emotion_dasaimochi/
```

After adding a new asset, also add it to the `data_files` section in
`setup.py`, rebuild `robot_ui`, and source `install/setup.bash` again.

## Stop the Application

- Press `Esc` while the UI is focused.
- Or press `Ctrl+C` in the terminal that launched it.

## Troubleshooting

### Package `robot_ui` not found

The current terminal has not sourced the workspace overlay. Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix robot_ui
```

If the package is still missing, rebuild it:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

### The UI opens but no map is displayed

Check that Nav2 or SLAM is publishing `/map`:

```bash
ros2 topic info /map
ros2 topic echo /map --once
```

For Gazebo, confirm that every related process uses simulation time:

```bash
ros2 param get /robot_ui_kiosk_process use_sim_time
```

### A goal is selected but the robot does not move

Check the Nav2 action and velocity topics:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

If `/cmd_vel_smoothed` contains commands but `/cmd_vel` is zero or much lower,
the collision monitor may be stopping or slowing the robot near an obstacle.

### The UI still shows an old version

Stop the current UI, rebuild, source the workspace again and relaunch:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash

ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=true
```

### The Chat screen shows an LLM error

Check that the API endpoint is reachable from the same machine running the
kiosk:

```bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
```

Common causes are an unset `LLM_API_KEY`, a wrong `LLM_BASE_URL`, a 9router
server that is not running, or an `LLM_MODEL` value that is not listed by the
`/models` endpoint. The kiosk sends requests to:

```text
$LLM_BASE_URL/chat/completions
```

### The chatbot responds but does not use RAG

Confirm that the knowledge files are present in the installed package and that
they have a `.txt` extension. Rebuild and source the workspace after changing
them:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```


## Kiosk AI upgrades (integrated into simulator)

For visitor-scoped conversation memory, inactivity reset, ROS telemetry freshness,
chunked knowledge search, optional local embeddings, and test commands, see
[the Kiosk AI guide](robot_ui/kiosk/README_AI.md).

ros2 action list | grep navigate_to_pose
ros2 topic info /map -v
```

**Map topic distinction:** GUI v3 subscribes to SLAM/raw `/map1` and
publishes its **edited map to `/map`** through
`Áp dụng & Publish`. Souta Kiosk subscribes to `/map`, not `/map1`.
If GUI v3 displays a map but Kiosk is blank, check for a compatible
publisher on `/map`. In the map-editor workflow, apply/publish the
edited map as appropriate. Avoid creating competing `/map` publishers.

**Navigation:** both GUIs can send goals to `/navigate_to_pose`. Avoid
issuing conflicting goals from both windows; confirm TF, localization,
costmaps and Nav2 action readiness before driving the physical robot.

### 6. Troubleshoot HTTP 503 from Kiosk Chat

If Kiosk logs `RuntimeError: LLM server trả HTTP 503`, the HTTP
request reached a server/proxy, but the chat completion request failed.
It **does not** mean the Kiosk failed to read `llm.local.env`. Investigate
9router's selected model, provider login/session, quota and upstream
availability. A successful `GET /models` alone does not prove
`POST /chat/completions` works.

Use this safe diagnostic from the workspace root: it loads the same local
ENV through the Kiosk parser and tests **the actual chat endpoint**,
without printing the API key.

```bash
cd ~/ros2_ws
PYTHONPATH=src/robot_ui python3 - <<'PY'
import json
import os
import urllib.error
import urllib.request
from robot_ui.kiosk.llm_env import load_llm_env_file

load_llm_env_file("~/ros2_ws/config/llm.local.env")
url = os.environ["LLM_BASE_URL"].rstrip("/") + "/chat/completions"
payload = {
    "model": os.environ["LLM_MODEL"],
    "messages": [{"role": "user", "content": "Xin chao, tra loi ngan."}],
    "stream": False,
}
req = urllib.request.Request(
    url,
    data=json.dumps(payload).encode("utf-8"),
    headers={
        "Content-Type": "application/json",
        "Authorization": "Bearer " + os.environ["LLM_API_KEY"],
    },
    method="POST",
)
try:
    with urllib.request.urlopen(req, timeout=30) as response:
        result = json.load(response)
        print("HTTP:", response.status)
        print("Reply:", result["choices"][0]["message"]["content"])
except urllib.error.HTTPError as exc:
    print("HTTP:", exc.code)
    print("Response:", exc.read().decode("utf-8", "replace")[:1200])
except (urllib.error.URLError, TimeoutError) as exc:
    print("Connection error:", exc)
PY
```

Remove credentials and tokens from any copied error response before sharing
logs. Fix the 9router upstream/provider issue first; restarting GUI v3
or changing ROS 2 topics will not resolve an upstream HTTP 503.

---

## Prepare a Terminal

Run these commands from any directory inside the cloned repository in every new
terminal before using ROS 2 packages from this workspace:

```bash
export WS="$(git rev-parse --show-toplevel)"
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For a source archive without Git metadata, open the workspace root and use
`export WS="$PWD"`. After relocating the workspace, rebuild and source it again;
old build/install artifacts can still contain absolute paths.

## Install Native UI Dependencies

On Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-opencv python3-numpy
```

## Configure the Kiosk Chatbot

The native kiosk uses the 9router OpenAI-compatible API, defaulting to
`http://localhost:20128/v1`. **9router must already be running**; launching
the kiosk does not start the LLM server.

On the Jetson, reuse the existing local, untracked configuration
`/home/orin/ros2_ws/config/llm.local.env` with `LLM_BASE_URL`,
`LLM_API_KEY`, and `LLM_MODEL`. Never commit this file or copy the API
key into documentation.

The Kiosk **automatically loads this file on every launch**. No manual
`source` or API key export is required. Both `KEY=value` and
`export KEY=value` lines are supported, including quoted values and comments.
The file must contain `LLM_BASE_URL`, `LLM_API_KEY` and `LLM_MODEL`.

A safe example layout (do not commit actual credentials):

```bash
LLM_BASE_URL=http://localhost:20128/v1
LLM_API_KEY=replace-with-your-local-key
LLM_MODEL=replace-with-an-available-model
```

Edit only this file to change the 9router endpoint/key/model, then **restart
Kiosk** to apply changes. The Kiosk loads the credentials, but does not start
the external 9router server. The server must already be running.

If your file is stored elsewhere, pass its path on launch:

```bash
ros2 launch robot_ui kiosk.launch.py \
  llm_env_file:=/absolute/path/to/llm.local.env
```

Configuration is parsed as plain data, **not executed as shell code**. Shell
expansion and commands inside the file are intentionally unsupported. No
credentials are printed in the Kiosk logs.

Check 9router access (without printing the key):

```bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
```

If 9router is on another computer, adjust the saved `LLM_BASE_URL`
to its reachable IP rather than `localhost`. The saved `LLM_MODEL`
must be valid on the connected server.

Launch the kiosk on a machine with a working graphical display:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=false
```

The chatbot loads `.txt` knowledge files from
`robot_ui/kiosk/rag_docs/`. Rebuild the `robot_ui` package after
changing Python code or those documents.

## Build the UI

Build after modifying Python code, launch files, configuration or assets:

```bash

source .venv/bin/activate
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

Verify that ROS 2 can find the package:

```bash
ros2 pkg prefix robot_ui
```

## Run a Windowed Simulation Preview

Use this mode during development. The mouse cursor remains visible and the
window can be resized:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=true
```

The Chat screen is available from the same kiosk process. For a complete
chatbot test, ensure 9router is running, launch the Kiosk (which reads the
configuration file automatically), and send a message from the Chat screen.

## Run Fullscreen with Gazebo

Use the Gazebo clock while displaying the UI fullscreen:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=false \
  hide_cursor:=true \
  use_sim_time:=false
```

## Run Fullscreen on the Real Robot

The real robot uses the system clock:

```bash
ros2 launch robot_ui kiosk.launch.py \
  windowed:=false \
  hide_cursor:=true \
  use_sim_time:=false
```

The default launch values are already suitable for the real touchscreen, so
the shorter equivalent command is:

```bash
ros2 launch robot_ui kiosk.launch.py
```

## Start the Robot Backend

Open a separate terminal and prepare it as described above, then run:

```bash
ros2 launch robot_bringup backend.launch.py
```

The backend provides robot status, navigation status, saved locations and
other data consumed by the UI.

## Start the Gazebo Simulation

Open a separate prepared terminal:

```bash
ros2 launch mo_hinh virtual_robot_gazebo.launch.py
```

Navigation also requires Nav2 and localization to be running. If using the
saved virtual lab map, start Nav2 from another prepared terminal:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=true \
  map:="$WS/src/mo_hinh/maps/virtual_lab_map.yaml" \
  params_file:="$WS/src/mo_hinh/config/nav2_params.yaml"
```

Start the backend and Robot UI in their own terminals after Gazebo and Nav2 are
running.

## Recommended Simulation Terminal Layout

Use one process per terminal:

```text
Terminal 1: Gazebo simulation
Terminal 2: Nav2 and localization
Terminal 3: Robot backend
Terminal 4: Robot UI
```

Every terminal must source ROS 2 and `install/setup.bash` first.

## Optional Web UI User Service

`deploy/robot-ui.service.example` is a systemd **user** service for the web UI,
not the native kiosk. It uses a stable symlink in your home directory instead of
embedding a disk mount or username in the unit. After setting `WS`:

```bash
mkdir -p "$HOME/.local/share" "$HOME/.config/systemd/user"
ln -s "$WS" "$HOME/.local/share/robot-workspace"
cp "$WS/src/robot_ui/deploy/robot-ui.service.example" \
  "$HOME/.config/systemd/user/robot-ui.service"
systemctl --user daemon-reload
systemctl --user enable --now robot-ui.service
```

Create the symlink only once. If it already exists, check its target before
changing it. When relocating the workspace, stop the service, rebuild at the new
location, update the symlink and restart the service. Do not install this example
as a system-wide service.

## ROS Interfaces Used by the UI

The native UI subscribes to:

```text
/robot_status
/navigation/status
/map
/amcl_pose
/odom
/plan
```

It uses these services to load saved locations:

```text
/config/get_location_list
/config/get_location
```

It sends and cancels goals through the Nav2 action:

```text
/navigate_to_pose
```

## Inspect Live Data

List all active topics and their message types:

```bash
ros2 topic list -t
```

Check robot status:

```bash
ros2 topic echo /robot_status --once
```

Check the localized robot pose:

```bash
ros2 topic echo /amcl_pose --once
```

Check whether a map is being published:

```bash
ros2 topic echo /map --once
```

Check the generated navigation path:

```bash
ros2 topic echo /plan --once
```

Monitor the final velocity sent to Gazebo or the real robot:

```bash
ros2 topic echo /cmd_vel
```

Check whether the Nav2 action server is available:

```bash
ros2 action list | grep navigate_to_pose
```

## Emotions and Icons

Emotion names and video filenames are configured in:

```text
config/emotions.json
```

MP4 emotion files are stored in:

```text
emotion_dasaimochi/videomp4/
```

Navigation and menu image assets are stored in:

```text
emotion_dasaimochi/
```

After adding a new asset, also add it to the `data_files` section in
`setup.py`, rebuild `robot_ui`, and source `install/setup.bash` again.

## Stop the Application

- Press `Esc` while the UI is focused.
- Or press `Ctrl+C` in the terminal that launched it.

## Troubleshooting

### Package `robot_ui` not found

The current terminal has not sourced the workspace overlay. Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix robot_ui
```

If the package is still missing, rebuild it:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```

### The UI opens but no map is displayed

Check that Nav2 or SLAM is publishing `/map`:

```bash
ros2 topic info /map
ros2 topic echo /map --once
```

For Gazebo, confirm that every related process uses simulation time:

```bash
ros2 param get /robot_ui_kiosk use_sim_time
```

### A goal is selected but the robot does not move

Check the Nav2 action and velocity topics:

```bash
ros2 action list | grep navigate_to_pose
ros2 topic echo /cmd_vel_smoothed
ros2 topic echo /cmd_vel
```

If `/cmd_vel_smoothed` contains commands but `/cmd_vel` is zero or much lower,
the collision monitor may be stopping or slowing the robot near an obstacle.

### The UI still shows an old version

Stop the current UI, rebuild, source the workspace again and relaunch:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash

ros2 launch robot_ui kiosk.launch.py \
  windowed:=true \
  hide_cursor:=false \
  use_sim_time:=true
```

### The Chat screen shows an LLM error

Check that the API endpoint is reachable from the same machine running the
kiosk:

```bash
curl -sS "$LLM_BASE_URL/models" \\
  -H "Authorization: Bearer $LLM_API_KEY"
```

Common causes are an unset `LLM_API_KEY`, a wrong `LLM_BASE_URL`, a 9router
server that is not running, or an `LLM_MODEL` value that is not listed by the
`/models` endpoint. The kiosk sends requests to:

```text
$LLM_BASE_URL/chat/completions
```

### The chatbot responds but does not use RAG

Confirm that the knowledge files are present in the installed package and that
they have a `.txt` extension. Rebuild and source the workspace after changing
them:

```bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash
```


## Kiosk AI upgrades (feature/main_interface)

For visitor-scoped conversation memory, inactivity reset, ROS telemetry freshness,
chunked knowledge search, optional local embeddings, and test commands, see
[the Kiosk AI guide](robot_ui/kiosk/README_AI.md).
