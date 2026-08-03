# Architecture

## Runtime Flow

```text
Browser
  ??? REST /api/v1/*
  ??? WebSocket /ws/telemetry
            ?
            ?
FastAPI application
  ??? StateStore
  ??? SystemMonitor thread
  ??? ROS bridge executor thread
  ??? Static frontend server
            ?
            ?
ROS 2 graph
```

## Backend Modules

- `app.py`: FastAPI creation, lifespan and static frontend mounting.
- `server.py`: CLI entrypoint and Uvicorn startup.
- `config.py`: YAML loading and default paths.
- `runtime.py`: Owns all long-running services.
- `state_store.py`: Thread-safe normalized dashboard state.
- `ros_bridge.py`: ROS subscriptions, normalized callbacks and graph discovery.
- `system_monitor.py`: Host CPU, RAM and disk telemetry.
- `api/health.py`: Read-only REST endpoints.
- `api/websocket.py`: Realtime state snapshots.

## Frontend Modules

- `api/useTelemetry.ts`: WebSocket lifecycle and reconnect.
- `types/telemetry.ts`: Shared frontend contract.
- `components/`: Reusable layout and status widgets.
- `pages/`: Feature boundaries for each debug domain.

## State Rules

- ROS callbacks never touch HTTP or frontend code.
- ROS callbacks only normalize data into `StateStore`.
- HTTP and WebSocket handlers only read `StateStore`.
- High-rate raw payloads such as full maps, images and scans require dedicated binary or streaming endpoints later.
- ROS graph snapshots are refreshed independently from high-rate telemetry callbacks.
- Controls must use explicit service classes and whitelist actions. Never accept arbitrary shell commands from the browser.

## Planned Services

```text
NavigationService   goals, cancel, initial pose, clear costmaps
MissionService      mission start, pause, resume, cancel
TeleopService       control lease, dead-man, zero on disconnect
RecordingService    rosbag presets and incident recording
ProcessService      whitelisted systemd service operations
CameraService       MJPEG first, WebRTC later
```

## Security Model

- `viewer`: read telemetry and logs.
- `operator`: navigation and mission controls.
- `developer`: parameters, recording and service restart.
- Web teleoperation requires a single-owner lease and heartbeat timeout.
- Web software stop is not a hardware emergency stop.
