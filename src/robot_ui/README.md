# Robot UI

Native ROS 2 touchscreen interface for the robot. The application is built with
PyQt5 and is intended to run fullscreen on a 7-inch Raspberry Pi touchscreen.
It does not require a web browser.

The interface contains three screens:

- **Home:** loops the configured Dasai Mochi emotion video.
- **Navigation:** displays the ROS map, robot pose, planned path and selected
  goal. Press Start to send a `NavigateToPose` goal and Stop to cancel it.
- **Chat:** provides the native chat layout. LLM integration is not connected
  yet.

## Workspace Path

The commands below use the current workspace location:

```bash
cd /media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based
```

## Prepare a Terminal

Run these commands in every new terminal before using ROS 2 packages from this
workspace:

```bash
cd /media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based

source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

If `.venv` is not being used, omit this command:

```bash
source .venv/bin/activate
```

## Install Native UI Dependencies

On Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt update
sudo apt install python3-pyqt5 python3-opencv python3-numpy
```

## Build the UI

Build after modifying Python code, launch files, configuration or assets:

```bash
cd /media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based

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
  map:="$(pwd)/src/mo_hinh/maps/virtual_lab_map.yaml" \
  params_file:="$(pwd)/src/mo_hinh/config/nav2_params.yaml"
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
