# Roadmap

## Phase 0 ? Scaffold

- [x] ROS 2 Python package.
- [x] FastAPI lifecycle.
- [x] Thread-safe state store.
- [x] Basic ROS subscriptions.
- [x] Realtime ROS graph discovery.
- [x] WebSocket snapshot stream.
- [x] React/TypeScript frontend shell.
- [x] ROS launch and deployment example.

## Phase 1 ? Read-only Debug Dashboard

- [x] Measure observed topic rate.
- [x] Calculate topic age and stale state.
- [x] Add stale-topic thresholds from YAML.
- [x] Subscribe `/diagnostics` and `/rosout`.
- [x] Send downsampled map through telemetry snapshots.
- [x] Render map, TF robot pose and downsampled `/scan`.
- [x] Add TF health and scan transform status.
- [x] Add global/local path overlays.
- [x] Add global/local costmap previews.
- [x] Add Nav2 action status transition history.
- [x] Add `/scan_obstacles`.
- [x] Add Jetson GPU utilization metrics when sysfs is available.
- [x] Add CPU-core, load-average and temperature metrics when available.
- [ ] Add Jetson power metrics.

## Phase 2 ? Project-specific Status Topics

- [x] Add structured JSON `/mission/status` publisher and dashboard.
- [x] Add vision and obstacle pipeline status.
- [ ] Add Nav2 feedback metrics: distance remaining, ETA and recoveries.
- [ ] Add comparison plots for encoder, RF2O and EKF odometry.

## Phase 3 ? Safe Controls

- [ ] Add authentication and viewer/operator/developer roles.
- [ ] Add navigation goal and cancel endpoints.
- [ ] Add clear-costmap and initial-pose actions.
- [ ] Add mission pause, resume and cancel.
- [ ] Add teleop lease, dead-man and disconnect stop.
- [ ] Add whitelisted systemd service control.

## Phase 4 ? Media and Recording

- [ ] MJPEG camera stream.
- [ ] YOLO overlay and depth diagnostics.
- [ ] Rosbag recording presets.
- [ ] Downloadable incident bundles.
- [ ] Optional Foxglove bridge link.
