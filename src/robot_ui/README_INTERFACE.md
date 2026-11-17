# Kiosk interface dark theme — simulator_interface_lord

Branch: `simulator_interface_lord` (derived from `simulator_interface`).
Implementation: `robot_ui/kiosk/window.py`, `widgets.py`, `pages.py`.

## Screen map

```text
START
  |
  v
IDLE (default: animated dark MP4 emotions; no dock or telemetry)
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

Optional static artwork: `souta_robot/mochi_idle_light.jpg` and
`souta_robot/mochi_service_light.jpg`. Service retains a dark header and
background around its original white-backed Mochi illustration.
Service has two real Qt touch
controls aligned over the artwork: **Phục vụ** sends the existing
`request_serve()` ("wait") command; **Hoàn tất** sends the existing
`finish_serving()` ("continue") command. The kiosk does not decode idle
MP4 frames while the static image is active.

The kiosk still requires a usable Qt display (X11/Wayland/XWayland).
`QT_QPA_PLATFORM=offscreen` is for tests, not physical touch operation.

## Behaviour and settings

| Setting | Default | Purpose |
|---|---|---|
| `KIOSK_ADMIN_PIN` | `1` | **Prototype only.** PIN for opening Admin |
| `KIOSK_MOCHI_STATIC` | `0` | Animated dark-theme MP4 idle by default; set `1` for the PTIT Mochi static preview |
| `KIOSK_IDLE_TIMEOUT_SECONDS` | `120` | Inactivity on Home / Chat before returning to Idle |
| `KIOSK_ADMIN_IDLE_SECONDS` | `90` | Inactivity on Admin / Navigation before locking to Idle |

* Opening Admin requires a PIN every time after logout/timeout.
* Visitor timeout clears the current chat/LLM session for privacy and
  returns to the animated dark Souta Robot face.
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

1. Launch Kiosk. It should show the animated dark idle screen, no Home/Map/Chat dock.
2. Tap anywhere. The Home screen should show the institution background,
   PTIT/FEE1 logos and Home + Chat + Settings in the bottom dock.
3. Tap Chat. Confirm 9router conversation works; return via header button.
4. Tap Settings. Wrong PIN stays locked; PIN `1` opens Admin.
5. From Admin open Map & Nav2; use the back arrow to return to Admin.
6. Logout; ensure Map & Nav2 is no longer accessible without PIN.
7. Leave the screen idle; verify return to Souta Robot and reset of Chat.
8. Confirm changing pages **does not** send Nav2 cancel goals.

## Multilingual Voice Chat (STT & TTS)

For full architecture, voice mappings (Vi/En/Ja/Zh/Ko), hardware requirements and configuration options, see:
* [README_VOICE_CHAT.md](README_VOICE_CHAT.md)

Offscreen GUI smoke tests:

```bash
PYTHONPATH=src/robot_ui QT_QPA_PLATFORM=offscreen python3 -m pytest \
  src/robot_ui/test/ -q
```
