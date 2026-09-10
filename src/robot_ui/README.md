# Robot UI
The interface contains three screens:

- **Home:** loops the configured Dasai Mochi emotion video.
- **Navigation:** displays the ROS map, robot pose, planned path and selected
  goal. Press Start to send a `NavigateToPose` goal and Stop to cancel it.
- **Chat:** provides a native chatbot backed by 9router through its
  OpenAI-compatible API. The client loads `.txt` files from `kiosk/rag_docs/`
  and includes the current robot status in the prompt.

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

The kiosk chatbot expects a running 9router server with an OpenAI-compatible
endpoint. By default it connects to:

```text
http://localhost:20128/v1
```

Set these variables in the same terminal that will launch the kiosk:

```bash
export LLM_BASE_URL="http://localhost:20128/v1"
export LLM_API_KEY="YOUR_9ROUTER_API_KEY"
export LLM_MODEL="hehe"
```

If 9router runs on another computer, replace `localhost` with that computer's
IP address. Do not commit the API key to the repository. The model name must
match one of the models returned by:

```bash
curl "$LLM_BASE_URL/models"
```

Before launching the kiosk, test the endpoint without exposing the key:

```bash
curl -sS "$LLM_BASE_URL/models" \
  -H "Authorization: Bearer $LLM_API_KEY"
```

The chatbot uses the text files in:

```text
robot_ui/kiosk/rag_docs/
```

Add or edit `.txt` files there when updating the robot knowledge base. Rebuild
the `robot_ui` package after changing Python code or RAG documents.

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
chatbot test, start 9router first, export the three `LLM_*` variables above,
then launch the kiosk and send a message from the Chat screen.

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
