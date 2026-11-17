# Kiosk interface mixed theme — simulator_interface_lord

Branch: `simulator_interface_lord` (derived from `simulator_interface`).
Implementation: `robot_ui/kiosk/window.py`, `widgets.py`, `pages.py`.

## Screen map

```text
START
  |
  v
IDLE (default: static PTIT Mochi image; no dock or telemetry)
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

Home, Chat, Service, Emergency and Settings controls appear in the **public Home dock**.
Navigation controls are hidden from visitors. The existing ROS subscriptions,
map widget, Nav2 action client and LLM worker remain unchanged.

SLAM and Navigation tools remain available from the authenticated Admin screen.

## Run on Jetson Orin / Local Machine

```bash
# Di chuyển vào thư mục workspace (trên Jetson hoặc máy local)
cd <đường_dẫn_workspace>  # ví dụ: cd ~/ros2_ws hoặc cd /path/to/repo
git fetch origin
git switch simulator_interface_lord  # if absent: git switch --track origin/simulator_interface_lord
git pull --ff-only origin simulator_interface_lord

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

Default mascot artwork: `souta_robot/mochi_idle_light.jpg` and
`souta_robot/mochi_service_light.jpg`. Only the Idle and Service pages are light mode (white full-screen
background, white Service canvas and header), matching the white-backed Mochi
artwork. All other Kiosk pages retain the original dark theme.
Service has two real Qt touch
controls aligned over the artwork: **Phục vụ** sends the existing
`request_serve()` ("wait") command; **Hoàn tất** sends the existing
`finish_serving()` ("continue") command. The kiosk does not decode idle
MP4 frames while the static image is active.

The kiosk still requires a usable Qt display (X11/Wayland/XWayland).
`QT_QPA_PLATFORM=offscreen` is for tests, not physical touch operation.

## ROS 2 System workflow (synced from simulator)

The Admin-only **System Config → ROS 2 System** panel now includes:

- **Start All / Stop All** in addition to per-Layer Start / Stop.
- **Operation mode** (SLAM Mapping or Localization Nav2), **backend** (AMCL or
  SLAM Toolbox) and **map YAML** selector for AMCL.
- **Serial Rescan / Swap** controls. They use the existing
  `robot_ui.kiosk.serial_resolver` and display the next-launch assignments.
  Port swapping does **not** hot-swap a driver that is already running.
- The existing Lord bounded per-Layer Live Log Console, STARTING / STOPPING /
  ERROR statuses and launch-token cancellation remain intact.

**Start All safety:** Layer 1 is started first. The workflow starts Layer 3
(SLAM) or Layer 2 (Navigation) only after **fresh /odom and /scan messages**
arrive from Hardware. It times out after 30 seconds without proceeding when
the required data is missing. This checks topic reception, not full Nav2
lifecycle activation: verify Nav2 action server and localization status before
issuing a real movement goal. Never test real Start All/Stop All or motor tests
on an unrestrained robot.

The GUI-to-ROS adapter now caps UI updates at approximately 5 Hz for LiDAR,
10 Hz for odometry pose, and 1 Hz for battery display values. Incoming topic
callbacks remain active. Jetson Host Telemetry reduces expensive work when
System Config is hidden. The audio player prefers mpv, then ffplay, with a
WAV-only aplay fallback.

## Behaviour and settings

| Setting | Default | Purpose |
|---|---|---|
| `KIOSK_ADMIN_PIN` | `1` | **Prototype only.** PIN for opening Admin |
| `KIOSK_MOCHI_STATIC` | `1` | New static PTIT Mochi idle by default; set `0` only to opt into the legacy MP4 expressions |
| `KIOSK_IDLE_TIMEOUT_SECONDS` | `120` | Inactivity on Home / Chat before returning to Idle |
| `KIOSK_ADMIN_IDLE_SECONDS` | `90` | Inactivity on Admin / Navigation before locking to Idle |

* Opening Admin requires a PIN every time after logout/timeout.
* Visitor timeout clears the current chat/LLM session for privacy and
  returns to the new static PTIT Mochi idle screen.
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

1. Launch Kiosk. It should show the new static PTIT Mochi image on a full-white background without black borders or a dock.
2. Tap anywhere. The Home screen should show the institution background,
   PTIT/FEE1 logos and Home + Chat + Settings in the bottom dock.
3. Tap Chat. Confirm dark-theme 9router conversation works; return via header button.
4. Open Service. Confirm the white artwork blends into the white page; tap the blue **Phục vụ** and green **Hoàn tất** controls to check zoom. On a connected robot, these send real ROS 2 commands, so test only when safe.
5. Tap Settings. Wrong PIN stays locked; PIN `1` opens Admin.
6. From Admin open Map & Nav2; use the back arrow to return to Admin.
7. Logout; ensure Map & Nav2 is no longer accessible without PIN.
8. Leave the screen idle; verify return to the new PTIT Mochi and reset of Chat.
9. Confirm changing pages **does not** send Nav2 cancel goals.

## Multilingual Voice Chat (STT & TTS)

For full architecture, voice mappings (Vi/En/Ja/Zh/Ko), hardware requirements and configuration options, see:
* [README_VOICE_CHAT.md](README_VOICE_CHAT.md)

Offscreen GUI smoke tests:

```bash
PYTHONPATH=src/robot_ui QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/robot_ui/test/ -q
```
