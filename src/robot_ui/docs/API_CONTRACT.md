# API Contract

Schema version starts at `0.1.0`.

## REST

### `GET /api/v1/health`

Small readiness response for probes and the top status bar.

```json
{
  "status": "ok",
  "robot_state": "ONLINE",
  "ros_connected": true,
  "read_only": true
}
```

### `GET /api/v1/state`

Returns the same snapshot envelope used by WebSocket.

### `GET /api/v1/config`

Returns active server, ROS topic and feature configuration. Secrets must never be added to this response.

## WebSocket

### `WS /ws/telemetry`

```json
{
  "schema_version": "0.1.0",
  "type": "state.snapshot",
  "sequence": 42,
  "timestamp": 1785700000.25,
  "data": {
    "robot": {},
    "system": {
      "cpu_percent": 38.4,
      "cpu_per_core_percent": [32.1, 41.8, 28.7, 52.4],
      "load_average": {"one": 2.14, "five": 1.87, "fifteen": 1.55},
      "memory_percent": 46.2,
      "disk_percent": 51.7,
      "gpu_percent": 24.8,
      "temperature_celsius": 54.3,
      "temperatures": []
    },
    "navigation": {
      "map": {
        "frame_id": "map",
        "width": 1024,
        "height": 1024,
        "resolution": 0.05,
        "origin_x": -12.8,
        "origin_y": -12.8,
        "origin_yaw": 0.0,
        "preview_width": 205,
        "preview_height": 205,
        "sample_step": 5,
        "cells": [-1, 0, 0, 100]
      },
      "odom": {},
      "pose": {
        "source": "tf2",
        "frame_id": "map",
        "child_frame_id": "base_footprint",
        "x": 1.15,
        "y": 0.6,
        "yaw": 0.42
      },
      "tf": {"state": "OK", "map_frame": "map", "base_frame": "base_footprint"},
      "scan": {
        "frame_id": "laser",
        "target_frame_id": "map",
        "points_xy": [[1.2, 0.7]],
        "transform_state": "OK"
      },
      "goal": {"frame_id": "map", "x": 2.5, "y": 1.4, "yaw": 0.15},
      "global_path": {"frame_id": "map", "point_count": 42, "points_xy": []},
      "local_path": {"frame_id": "map", "point_count": 18, "points_xy": []},
      "global_costmap": {},
      "local_costmap": {},
      "nav2": {
        "status": "EXECUTING",
        "goal_id": "9d74ab2eacb1",
        "active_goal_count": 1,
        "history": []
      }
    },
    "hardware": {},
    "vision": {
      "color_camera": {"frame_id": "camera_color_optical_frame", "width": 1280, "height": 720},
      "depth_camera": {"frame_id": "camera_depth_optical_frame", "width": 848, "height": 480},
      "yolo": {
        "detection_count": 3,
        "person_count": 2,
        "tracked_count": 2,
        "class_counts": {"person": 2, "chair": 1},
        "detections": []
      },
      "pose": {"wave_detected": true, "status": "[person_12] Waving with right hand!"},
      "obstacle": {"frame_id": "base_link", "obstacle_point_count": 63, "nearest_range": 0.82}
    },
    "mission": {
      "mode": "GO_TO_B",
      "active": true,
      "waypoint": {"index": 1, "display_index": 2, "total": 4, "name": "Hall B"},
      "goal": {"frame_id": "map", "x": 2.55, "y": 1.42},
      "nav2": {
        "state": "EXECUTING",
        "label": "WP:Hall B",
        "goal_active": true,
        "distance_remaining": 1.73
      },
      "intercept": {
        "enabled": true,
        "done_this_trip": false,
        "person_detected": true,
        "track_id": "id:person_12",
        "hand": "RIGHT",
        "wave_hold_seconds": 2.4,
        "distance_m": 1.62
      },
      "wait_remaining_seconds": 0.0,
      "last_error": ""
    },
    "diagnostics": {
      "summary": {
        "ok_count": 4,
        "warn_count": 1,
        "error_count": 0,
        "stale_count": 0,
        "total_count": 5
      },
      "statuses": [
        {
          "name": "Local planner frequency",
          "level": 1,
          "level_name": "WARN",
          "message": "Update loop below configured frequency",
          "hardware_id": "controller_server",
          "values": {"expected_rate": "5.0 Hz", "observed_rate": "1.8 Hz"}
        }
      ]
    },
    "ros_logs": [
      {
        "timestamp": 1785700000.18,
        "level": 30,
        "level_name": "WARN",
        "name": "/controller_server",
        "message": "Control loop missed desired update rate",
        "file": "controller_server.cpp",
        "function": "computeControl",
        "line": 538
      }
    ],
    "ros_graph": {
      "nodes": [],
      "edges": [
        {
          "id": "publish:node:/slam_toolbox:topic:/map",
          "source": "node:/slam_toolbox",
          "target": "topic:/map",
          "kind": "publish",
          "message_types": ["nav_msgs/msg/OccupancyGrid"],
          "qos": {
            "reliability": "RELIABLE",
            "durability": "TRANSIENT_LOCAL",
            "history": "KEEP_LAST",
            "depth": 1
          }
        }
      ],
      "summary": {}
    },
    "topics": {
      "/odom": {
        "message_type": "nav_msgs/Odometry",
        "state": "OK",
        "rate_hz": 29.8,
        "expected_rate_hz": 30.0,
        "health_monitored": true,
        "age_seconds": 0.04,
        "stale_after_seconds": 2.0,
        "message_count": 18432,
        "last_message_at": 1785700000.21,
        "latest": {"x": 1.15, "y": 0.6, "yaw": 0.42}
      }
    },
    "events": []
  }
}
```

## Planned Message Types

```text
state.snapshot
health.changed
navigation.pose
navigation.status
hardware.drive
vision.status
mission.status
log.event
```

The initial implementation sends full snapshots. Later revisions may add delta messages while retaining periodic full snapshots for reconnect recovery.

`navigation.map.cells` is a downsampled occupancy preview, capped to roughly
`240 × 240` cells. Values keep ROS OccupancyGrid semantics: `-1` unknown,
`0` free, and `100` occupied. Full-resolution map data remains inside ROS.

Global and local costmaps use the same preview schema with a separately
configurable size cap. LaserScan and Path arrays are sampled before transport.
Nav2 status history records action-status transitions; it does not issue goals
or cancellation commands.

Topic health checks run only for configured expected-rate topics. `WARN` means
the observed rate is below the configured ratio. `STALE` means message age is
past the computed threshold. `/diagnostics` keeps the latest bounded status
snapshot; `/rosout` keeps a bounded newest-first-compatible log buffer.

`/mission/status` is currently a versioned JSON payload carried by
`std_msgs/String`. The bridge validates object shape and payload size before
placing it in telemetry. Vision health subscribes only to CameraInfo, detection,
wave and obstacle-scan messages; raw camera images are not copied into WebSocket
state.
