export type ConnectionState = "connecting" | "online" | "offline";

export interface RobotState {
  state?: string;
  ros_connected?: boolean;
  config_source?: string;
  updated_at?: number;
}

export interface SystemState {
  hostname?: string;
  cpu_percent?: number;
  cpu_per_core_percent?: number[];
  load_average?: {
    one?: number;
    five?: number;
    fifteen?: number;
  } | null;
  memory_percent?: number;
  memory_used_bytes?: number;
  memory_total_bytes?: number;
  disk_percent?: number;
  disk_free_bytes?: number;
  boot_time?: number;
  gpu_percent?: number | null;
  temperature_celsius?: number | null;
  temperatures?: Array<{
    label: string;
    current_celsius: number;
    high_celsius?: number | null;
    critical_celsius?: number | null;
  }>;
  updated_at?: number;
}

export type TopicHealthState = "WAITING" | "OK" | "WARN" | "STALE";

export interface TopicState {
  message_type?: string;
  state?: TopicHealthState | string;
  last_message_at?: number;
  rate_hz?: number;
  expected_rate_hz?: number;
  health_monitored?: boolean;
  age_seconds?: number | null;
  stale_after_seconds?: number;
  health_state_changed_at?: number;
  message_count?: number;
  latest?: unknown;
}

export interface DiagnosticStatusState {
  name: string;
  level: number;
  level_name: string;
  message: string;
  hardware_id?: string;
  values: Record<string, string>;
  timestamp?: number;
}

export interface DiagnosticsState {
  summary: {
    ok_count?: number;
    warn_count?: number;
    error_count?: number;
    stale_count?: number;
    total_count?: number;
  };
  statuses: DiagnosticStatusState[];
  message_timestamp?: number;
  truncated?: boolean;
  updated_at?: number;
}

export interface RosLogEntry {
  timestamp: number;
  received_at?: number;
  level: number;
  level_name: string;
  name: string;
  message: string;
  file?: string;
  function?: string;
  line?: number;
}

export interface RosGraphQos {
  reliability?: string;
  durability?: string;
  history?: string;
  depth?: number;
}

export interface RosGraphNode {
  id: string;
  label: string;
  kind: "node" | "topic";
  namespace?: string;
  message_types?: string[];
}

export interface RosGraphEdge {
  id: string;
  source: string;
  target: string;
  kind: "publish" | "subscribe";
  message_types?: string[];
  qos?: RosGraphQos;
}

export interface RosGraphState {
  nodes: RosGraphNode[];
  edges: RosGraphEdge[];
  summary: {
    node_count?: number;
    topic_count?: number;
    publisher_count?: number;
    subscription_count?: number;
    edge_count?: number;
  };
  updated_at?: number;
}

export interface SlamMapState {
  frame_id?: string;
  width?: number;
  height?: number;
  resolution?: number;
  origin_x?: number;
  origin_y?: number;
  origin_yaw?: number;
  preview_width?: number;
  preview_height?: number;
  sample_step?: number;
  cells?: number[];
}

export interface OdometryState {
  source?: string;
  frame_id?: string;
  child_frame_id?: string;
  x?: number;
  y?: number;
  yaw?: number;
  linear_x?: number;
  linear_y?: number;
  angular_z?: number;
}

export interface LaserScanState {
  frame_id?: string;
  target_frame_id?: string;
  point_count?: number;
  valid_point_count?: number;
  nearest_range?: number;
  range_min?: number;
  range_max?: number;
  points_xy?: Array<[number, number]>;
  transform_state?: string;
  transform_error?: string;
}

export interface NavigationPathState {
  frame_id?: string;
  point_count?: number;
  sample_step?: number;
  points_xy?: Array<[number, number]>;
}

export interface NavigationGoalState {
  frame_id?: string;
  x?: number;
  y?: number;
  yaw?: number;
  received_at?: number;
}

export interface TfState {
  state?: string;
  map_frame?: string;
  base_frame?: string;
  last_success_at?: number;
  error?: string;
}

export interface Nav2HistoryItem {
  goal_id?: string;
  status_code?: number;
  status?: string;
  accepted_at?: number;
  transition_at?: number;
}

export interface Nav2State {
  goal_id?: string;
  status_code?: number;
  status?: string;
  accepted_at?: number;
  active_goal_count?: number;
  status_count?: number;
  history?: Nav2HistoryItem[];
}

export interface NavigationState {
  map?: SlamMapState;
  odom?: OdometryState;
  pose?: OdometryState;
  scan?: LaserScanState;
  goal?: NavigationGoalState;
  global_path?: NavigationPathState;
  local_path?: NavigationPathState;
  global_costmap?: SlamMapState;
  local_costmap?: SlamMapState;
  tf?: TfState;
  nav2?: Nav2State;
  updated_at?: number;
}

export interface MissionState {
  schema_version?: string;
  timestamp?: number;
  mode?: string;
  active?: boolean;
  started_at?: number;
  mode_changed_at?: number;
  waypoint?: {
    index?: number;
    display_index?: number;
    total?: number;
    name?: string;
  };
  goal?: {
    frame_id?: string;
    x?: number;
    y?: number;
  } | null;
  nav2?: {
    state?: string;
    label?: string;
    goal_active?: boolean;
    distance_remaining?: number | null;
    result_status_code?: number | null;
    result_status?: string;
  };
  intercept?: {
    enabled?: boolean;
    done_this_trip?: boolean;
    person_detected?: boolean;
    track_id?: string;
    hand?: string;
    wave_hold_seconds?: number;
    distance_m?: number | null;
  };
  wait_remaining_seconds?: number;
  last_error?: string;
  source_topic?: string;
  received_at?: number;
  updated_at?: number;
}

export interface VisionCameraState {
  frame_id?: string;
  width?: number;
  height?: number;
  distortion_model?: string;
  fx?: number | null;
  fy?: number | null;
  cx?: number | null;
  cy?: number | null;
  timestamp?: number;
}

export interface VisionDetectionState {
  id?: string;
  class_id?: number;
  class_name?: string;
  score?: number;
  center_x?: number;
  center_y?: number;
  width?: number;
  height?: number;
  keypoint_count?: number;
}

export interface VisionState {
  color_camera?: VisionCameraState;
  depth_camera?: VisionCameraState;
  yolo?: {
    frame_id?: string;
    timestamp?: number;
    detection_count?: number;
    person_count?: number;
    tracked_count?: number;
    class_counts?: Record<string, number>;
    detections?: VisionDetectionState[];
    truncated?: boolean;
  };
  pose?: {
    wave_detected?: boolean;
    last_detected_at?: number;
    status?: string;
    updated_at?: number;
    status_updated_at?: number;
  };
  obstacle?: {
    frame_id?: string;
    point_count?: number;
    obstacle_point_count?: number;
    nearest_range?: number | null;
    range_min?: number;
    range_max?: number;
  };
  updated_at?: number;
}

export interface DashboardState {
  schema_version: string;
  robot: RobotState;
  system: SystemState;
  navigation: NavigationState;
  hardware: Record<string, unknown>;
  vision: VisionState;
  mission: MissionState;
  diagnostics: DiagnosticsState;
  ros_logs: RosLogEntry[];
  ros_graph: RosGraphState;
  topics: Record<string, TopicState>;
  events: Array<{
    timestamp: number;
    level: string;
    source: string;
    message: string;
  }>;
}

export interface TelemetryEnvelope {
  schema_version: string;
  type: string;
  sequence: number;
  timestamp: number;
  data: DashboardState;
}
