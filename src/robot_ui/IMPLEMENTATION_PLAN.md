# ARCHITECTURE OVERVIEW & DEVELOPMENT ROADMAP FOR RECEPTIONIST ROBOT WEB UI

This document outlines the entire software architecture, categorizing the User Interface (UI) Modes for different target users, and detailing the development phases for the Jetson Orin Nano 8GB + 7-inch touchscreen hardware.

---

## 1. UI Modes Categorization

The Web UI system will run on a single Backend (FastAPI) but serve two independent interface portals (Frontend Apps) for two distinct target audiences.

### 1.1. Guest Mode (For Customers)
- **Device**: Runs full-screen (Kiosk mode) on the 7-inch touchscreen mounted on the robot.
- **Characteristics**: Extremely simple, intuitive, and highly aesthetic interface (Dark/Glassmorphism theme). Large buttons, touch-friendly, featuring eye-catching animations.
- **Key Features**:
  - **Standby Screen**: Displays an animated robot face (blinking/smiling), current time, and weather.
  - **Navigation**: Visual 2D map accompanied by a list of available rooms/locations. Guests tap a button -> Robot navigates automatically.
  - **Multi-language**: Toggle button to switch between Vietnamese / English (dynamic dictionary loading).
  - **Voice AI (Future)**: Mic button allowing guests to voice their requests instead of tapping. The robot responds with audio (TTS).
  - **Recognition**: Greets guests by name if the camera recognizes their face.

### 1.2. Admin / Developer Mode
- **Device**: Accessed via a Web browser (PC/Laptop/Tablet) on the same LAN as the robot.
- **Characteristics**: A professional admin dashboard displaying detailed technical metrics and controls.
- **Key Features**:
  - **System Monitor**: Monitors Jetson Orin RAM/CPU usage, Battery status, Wi-Fi signal strength, and ROS2 node statuses (Nav2, YOLO, Face, Odom).
  - **Map & Waypoint Manager**: Displays a large map layout. Allows users to click or drag-and-drop to create new Waypoints -> Assign names (e.g., "Reception Desk") -> Save directly to the database. Eliminates the need for manual text file editing.
  - **Face Database Manager**: Register new faces directly from the live camera stream, assign employee names, and delete existing faces.
  - **Advanced Debug Modes (Dev Tools)**:
    - *Debug Navigation*: Tool to test sending goals directly, display Costmaps (Global/Local) for obstacle analysis, track Nav2 Feedback (distance remaining), and render the LaserScan stream (`/scan`) overlaid on the map for real-world comparison.
    - *Debug Pose/Vision*: Tool to view the raw video stream complete with YOLO Bounding Boxes, check the frame rate (Hz), and visualize hand-waving keypoints to fine-tune parameters (threshold, detection angles).
    - *Teleoperation*: A virtual joystick to manually control the robot via the web interface (`/cmd_vel`).
  - **Log Viewer**: View live system logs directly on the web interface instead of opening a terminal window.

---

## 2. System Architecture

- **Backend (Python)**: `FastAPI` acts as both the Web Server and WebSocket Server. It runs alongside a ROS2 Node (`rclpy`) in a background thread to seamlessly communicate with the existing ROS2 Humble ecosystem.
- **Frontend (Web)**: Utilizes **Vite + Vanilla JS/TS + Vanilla CSS** to achieve maximum rendering speed on the Orin Nano without heavy framework overhead. Split into 2 entry points:
  - `/` -> Guest UI
  - `/admin` -> Admin Dashboard
- **Database**: `SQLite` (or JSON/YAML files) will be used to store the list of Waypoints, System Settings, and multi-language configurations.

---

## 3. Development Roadmap (Phases)

### Phase 1: Core Navigation & Map UI (Immediate Priority)
> Focuses on replacing the current PyQt5 application with the new Web UI, completing the basic Guest Mode and Admin Waypoint management.
- **Backend Bridge**:
  - Create the `robot_ui` ROS2 package.
  - Subscribes to `/map` and `/tf` (to acquire robot pose).
  - Publishes to `/nhiemvuboss/waypoints_json` (to dispatch movement commands).
- **Guest UI (Phase 1)**:
  - Multi-language framework setup, mock Voice button (UI visualizes recording but does not process yet).
  - Canvas component to render the `/map` and the realtime moving robot icon.
  - Grid layout for predefined location buttons.
- **Admin UI (Phase 1)**:
  - Basic dashboard displaying the map.
  - Click on the map to capture Coordinates -> Assign a Name -> Save to the JSON configuration file.

### Phase 2: Face Recognition & Interaction Integration
> Connects the Web UI with the project's existing Computer Vision modules.
- **Backend**:
  - Stream Camera feed (OAK-D or RealSense) to the Web interface via MJPEG or WebRTC.
  - Subscribe to `/face_recognition` and `/pose/wave_status` topics.
- **Guest UI**:
  - Display a "Hello [Name]" popup upon successful facial recognition.
  - Display a visual notification when a waving gesture is detected.
- **Admin UI**:
  - Interface to capture photos and register new users into the face recognition system.

### Phase 3: AI Voice & LLM Integration
> Connects the robot to the Internet to create a natural communication chatbot.
- **Technology**: Google Speech-to-Text (or Whisper API) & Google TTS. Cloud LLM (OpenAI/Gemini).
- **Workflow**: Guest taps Mic in Guest UI -> Records Audio -> Sends to Backend -> Backend calls STT to get text -> Feeds into LLM (with context regarding the building's layout/state) -> Retrieves answer -> Uses TTS to play audio and displays the text on screen.
- **LLM Skill Integration**: If the LLM parses the user's sentence as "Take me to the Director's room", it automatically triggers the navigation action.

### Phase 4: Analytics, Polish & Deployment
- Generate statistics on visitor counts and movement heatmaps.
- Optimize CSS Animations, Particle effects, and Dark/Light themes for a premium feel.
- Configure a Bash script to automatically launch Chromium in `--kiosk` full-screen mode upon system boot.
