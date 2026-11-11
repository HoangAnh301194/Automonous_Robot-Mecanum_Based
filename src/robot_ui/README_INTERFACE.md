# Kiosk interface v2 — simulator_interface

Branch: `simulator_interface` (derived from `simulator`).
Implementation: `robot_ui/kiosk/window.py`, `widgets.py`, `pages.py`.

## Screen map

```text
START
  |
  v
IDLE (Souta Robot face / MP4, full screen, no dock or telemetry)
  | tap anywhere
  v
HOME (background.jpg, PTIT top-left + FEE1 top-right logos)
  |-- Dock Chat icon --> CHAT (existing 9router/RAG) --> Back --> HOME
  |
  `-- Dock Settings icon --> Admin PIN dialog --> ADMIN overview
                                           |-- Map & Nav2 --> NAVIGATION
                                           |                 `-- Back --> ADMIN
                                           `-- Logout --> HOME
```

Only Home, Chat and Settings buttons appear in the **public Home dock**.
Navigation controls are hidden from visitors. The existing ROS subscriptions,
map widget, Nav2 action client and LLM worker remain unchanged.

The SLAM icon is displayed as an **informational future module**, not an
executable SLAM button.

## Run on Jetson Orin

```bash
cd ~/ros2_ws
git fetch origin
git switch simulator_interface  # if absent: git switch --track origin/simulator_interface
git pull --ff-only origin simulator_interface

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_ui
source install/setup.bash

# Development: windowed mode + mouse visible
KIOSK_ADMIN_PIN=1 ros2 launch robot_ui kiosk.launch.py \
  windowed:=true hide_cursor:=false use_sim_time:=false
```

For fullscreen touch mode:

```bash
KIOSK_ADMIN_PIN=1 ros2 launch robot_ui kiosk.launch.py \
  windowed:=false hide_cursor:=true use_sim_time:=false
```

The kiosk still requires a usable Qt display (X11/Wayland/XWayland).
`QT_QPA_PLATFORM=offscreen` is for tests, not physical touch operation.

## Behaviour and settings

| Setting | Default | Purpose |
|---|---|---|
| `KIOSK_ADMIN_PIN` | `1` | **Prototype only.** PIN for opening Admin |
| `KIOSK_IDLE_TIMEOUT_SECONDS` | `120` | Inactivity on Home / Chat before returning to Idle |
| `KIOSK_ADMIN_IDLE_SECONDS` | `90` | Inactivity on Admin / Navigation before locking to Idle |

* Opening Admin requires a PIN every time after logout/timeout.
* Visitor timeout clears the current chat/LLM session for privacy and
  returns to the animated Souta Robot face.
* Screen transitions and timeouts **never cancel an ongoing Nav2 goal**.
* Existing `/map`, robot status, LLM and navigation ROS interfaces are reused.
* `background.jpg`, `logoPtit.png`, `fee1.png`, `setting_icon.png`,
  `SLAMicon.png`, and `navigationIcon.png` are installed via `setup.py`.
* The large background JPEG is decoded to display-scale resolution using
  Qt `QImageReader` to reduce peak image memory.

**Security:** PIN `1` is a test credential, *not production authentication*.
The Admin access check is in the Qt frontend. Any production deployment
must additionally enforce authorization in backend command services,
use a strong PIN / secret storage and consider local OS access control.

## Manual smoke checks

1. Launch Kiosk. It should show only Souta Robot's face, no Home/Map/Chat dock.
2. Tap anywhere. The Home screen should show the institution background,
   PTIT/FEE1 logos and Home + Chat + Settings in the bottom dock.
3. Tap Chat. Confirm 9router conversation works; return via header button.
4. Tap Settings. Wrong PIN stays locked; PIN `1` opens Admin.
5. From Admin open Map & Nav2; use the back arrow to return to Admin.
6. Logout; ensure Map & Nav2 is no longer accessible without PIN.
7. Leave the screen idle; verify return to Souta Robot and reset of Chat.
8. Confirm changing pages **does not** send Nav2 cancel goals.

Offscreen GUI smoke tests:

```bash
QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/robot_ui/test/test_kiosk_interface.py -q
```
