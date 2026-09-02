import type { DashboardState, RosGraphEdge } from "../types/telemetry";

function buildDemoMap(width: number, height: number): number[] {
  const cells = new Array<number>(width * height).fill(-1);
  const setCell = (column: number, row: number, value: number) => {
    if (column >= 0 && column < width && row >= 0 && row < height) {
      cells[row * width + column] = value;
    }
  };

  for (let row = 8; row < height - 8; row += 1) {
    for (let column = 8; column < width - 8; column += 1) {
      setCell(column, row, 0);
    }
  }

  for (let column = 8; column < width - 8; column += 1) {
    setCell(column, 8, 100);
    setCell(column, height - 9, 100);
  }
  for (let row = 8; row < height - 8; row += 1) {
    setCell(8, row, 100);
    setCell(width - 9, row, 100);
  }

  for (let row = 8; row < height - 30; row += 1) {
    if (row < 39 || row > 48) setCell(58, row, 100);
  }
  for (let column = 58; column < width - 8; column += 1) {
    if (column < 92 || column > 103) setCell(column, 57, 100);
  }
  for (let row = 57; row < height - 8; row += 1) {
    if (row < 69 || row > 76) setCell(116, row, 100);
  }

  for (let row = 24; row < 36; row += 1) {
    for (let column = 91; column < 108; column += 1) setCell(column, row, 100);
  }
  for (let row = 66; row < 75; row += 1) {
    for (let column = 30; column < 45; column += 1) setCell(column, row, 100);
  }

  return cells;
}

function buildDemoScanPoints(
  robotX: number,
  robotY: number,
  count: number,
): Array<[number, number]> {
  return Array.from({ length: count }, (_, pointIndex) => {
    const angle = (pointIndex / count) * Math.PI * 2;
    const range = 1.05 + Math.sin(angle * 3) * 0.18 + Math.cos(angle * 7) * 0.08;
    return [
      robotX + Math.cos(angle) * range,
      robotY + Math.sin(angle) * range,
    ];
  });
}

function buildDemoPath(
  startX: number,
  startY: number,
  endX: number,
  endY: number,
  count: number,
  bend: number,
): Array<[number, number]> {
  return Array.from({ length: count }, (_, pointIndex) => {
    const progress = pointIndex / Math.max(1, count - 1);
    return [
      startX + (endX - startX) * progress,
      startY + (endY - startY) * progress + Math.sin(progress * Math.PI) * bend,
    ];
  });
}

function buildDemoCostmap(width: number, height: number, phase: number): number[] {
  const cells = new Array<number>(width * height).fill(0);
  for (let row = 0; row < height; row += 1) {
    for (let column = 0; column < width; column += 1) {
      const borderDistance = Math.min(column, row, width - column - 1, height - row - 1);
      const obstacleOne = Math.hypot(column - width * 0.38, row - height * 0.42);
      const obstacleTwo = Math.hypot(column - width * 0.7, row - height * 0.63);
      let occupancy = 0;
      if (borderDistance < 3) occupancy = 100;
      else if (obstacleOne < 5 + phase || obstacleTwo < 4 + phase) occupancy = 100;
      else if (obstacleOne < 12 + phase || obstacleTwo < 10 + phase) {
        occupancy = Math.max(10, 90 - Math.round(Math.min(obstacleOne, obstacleTwo) * 6));
      }
      cells[row * width + column] = occupancy;
    }
  }
  return cells;
}

const demoTimestamp = Date.now() / 1000;
const demoMapWidth = 144;
const demoMapHeight = 96;
const demoRobotX = 1.15;
const demoRobotY = 0.6;
const demoGoalX = 2.55;
const demoGoalY = 1.42;
const demoGlobalPath = buildDemoPath(
  demoRobotX,
  demoRobotY,
  demoGoalX,
  demoGoalY,
  42,
  0.45,
);
const demoLocalPath = buildDemoPath(
  demoRobotX,
  demoRobotY,
  1.75,
  0.95,
  18,
  0.12,
);
const demoMessageTypes: Record<string, string[]> = {
  "/odom": ["nav_msgs/msg/Odometry"],
  "/odom_encoder": ["nav_msgs/msg/Odometry"],
  "/imu/data": ["sensor_msgs/msg/Imu"],
  "/scan": ["sensor_msgs/msg/LaserScan"],
  "/cmd_vel": ["geometry_msgs/msg/Twist"],
  "/battery": ["sensor_msgs/msg/BatteryState"],
  "/dataenc": ["std_msgs/msg/Int32MultiArray"],
  "/map": ["nav_msgs/msg/OccupancyGrid"],
  "/goal_pose": ["geometry_msgs/msg/PoseStamped"],
  "/plan": ["nav_msgs/msg/Path"],
  "/local_plan": ["nav_msgs/msg/Path"],
  "/global_costmap/costmap_raw": ["nav_msgs/msg/OccupancyGrid"],
  "/local_costmap/costmap_raw": ["nav_msgs/msg/OccupancyGrid"],
  "/navigate_to_pose/_action/status": ["action_msgs/msg/GoalStatusArray"],
  "/diagnostics": ["diagnostic_msgs/msg/DiagnosticArray"],
  "/rosout": ["rcl_interfaces/msg/Log"],
  "/mission/status": ["std_msgs/msg/String"],
  "/camera/color/camera_info": ["sensor_msgs/msg/CameraInfo"],
  "/camera/depth/camera_info": ["sensor_msgs/msg/CameraInfo"],
  "/yolo/detections": ["yolo_msgs/msg/DetectionArray"],
  "/pose/wave_detected": ["std_msgs/msg/Bool"],
  "/pose/wave_status": ["std_msgs/msg/String"],
  "/scan_obstacles": ["sensor_msgs/msg/LaserScan"],
  "/nhiemvuboss/waypoints_json": ["std_msgs/msg/String"],
};


export const demoState: DashboardState = {
  schema_version: "0.1.0",
  robot: { state: "DEMO", ros_connected: false },
  system: {
    hostname: "jetson-orin",
    cpu_percent: 38.4,
    cpu_per_core_percent: [32.1, 41.8, 28.7, 52.4, 36.6, 38.9],
    load_average: { one: 2.14, five: 1.87, fifteen: 1.55 },
    memory_percent: 46.2,
    disk_percent: 51.7,
    gpu_percent: 24.8,
    temperature_celsius: 54.3,
    temperatures: [
      { label: "CPU", current_celsius: 51.7, high_celsius: 84, critical_celsius: 95 },
      { label: "GPU", current_celsius: 54.3, high_celsius: 84, critical_celsius: 95 },
    ],
  },
  navigation: {
    map: {
      frame_id: "map",
      width: demoMapWidth,
      height: demoMapHeight,
      resolution: 0.05,
      origin_x: -3.6,
      origin_y: -2.4,
      origin_yaw: 0,
      preview_width: demoMapWidth,
      preview_height: demoMapHeight,
      sample_step: 1,
      cells: buildDemoMap(demoMapWidth, demoMapHeight),
    },
    odom: {
      frame_id: "odom",
      child_frame_id: "base_link",
      x: demoRobotX,
      y: demoRobotY,
      yaw: 0.42,
      linear_x: 0.18,
      linear_y: 0.02,
      angular_z: 0.08,
    },
    pose: {
      source: "tf2",
      frame_id: "map",
      child_frame_id: "base_footprint",
      x: demoRobotX,
      y: demoRobotY,
      yaw: 0.42,
    },
    tf: {
      state: "OK",
      map_frame: "map",
      base_frame: "base_footprint",
      last_success_at: demoTimestamp,
      error: "",
    },
    scan: {
      frame_id: "laser",
      target_frame_id: "map",
      point_count: 720,
      valid_point_count: 684,
      nearest_range: 0.73,
      range_min: 0.12,
      range_max: 12,
      points_xy: buildDemoScanPoints(demoRobotX, demoRobotY, 180),
      transform_state: "OK",
      transform_error: "",
    },
    goal: {
      frame_id: "map",
      x: demoGoalX,
      y: demoGoalY,
      yaw: 0.15,
      received_at: demoTimestamp - 7,
    },
    global_path: {
      frame_id: "map",
      point_count: demoGlobalPath.length,
      sample_step: 1,
      points_xy: demoGlobalPath,
    },
    local_path: {
      frame_id: "map",
      point_count: demoLocalPath.length,
      sample_step: 1,
      points_xy: demoLocalPath,
    },
    global_costmap: {
      frame_id: "map",
      width: 120,
      height: 80,
      resolution: 0.08,
      origin_x: -4.8,
      origin_y: -3.2,
      origin_yaw: 0,
      preview_width: 120,
      preview_height: 80,
      sample_step: 1,
      cells: buildDemoCostmap(120, 80, 1),
    },
    local_costmap: {
      frame_id: "odom",
      width: 72,
      height: 72,
      resolution: 0.05,
      origin_x: -0.65,
      origin_y: -1.2,
      origin_yaw: 0,
      preview_width: 72,
      preview_height: 72,
      sample_step: 1,
      cells: buildDemoCostmap(72, 72, 0),
    },
    nav2: {
      goal_id: "9d74ab2eacb1448e9e4fb91d4f3f2341",
      status_code: 2,
      status: "EXECUTING",
      accepted_at: demoTimestamp - 7,
      active_goal_count: 1,
      status_count: 1,
      history: [
        { goal_id: "9d74ab2eacb1448e9e4fb91d4f3f2341", status_code: 2, status: "EXECUTING", transition_at: demoTimestamp - 6.8 },
        { goal_id: "9d74ab2eacb1448e9e4fb91d4f3f2341", status_code: 1, status: "ACCEPTED", transition_at: demoTimestamp - 7 },
        { goal_id: "44fdd1763b834678bc39bd4b57318711", status_code: 4, status: "SUCCEEDED", transition_at: demoTimestamp - 42 },
      ],
    },
  },
  hardware: {},
  vision: {
    color_camera: {
      frame_id: "camera_color_optical_frame",
      width: 1280,
      height: 720,
      distortion_model: "plumb_bob",
      fx: 908.4,
      fy: 907.9,
      cx: 640.2,
      cy: 359.7,
      timestamp: demoTimestamp - 0.03,
    },
    depth_camera: {
      frame_id: "camera_depth_optical_frame",
      width: 848,
      height: 480,
      distortion_model: "brown_conrady",
      fx: 423.6,
      fy: 423.6,
      cx: 421.9,
      cy: 239.8,
      timestamp: demoTimestamp - 0.04,
    },
    yolo: {
      frame_id: "camera_color_optical_frame",
      timestamp: demoTimestamp - 0.08,
      detection_count: 3,
      person_count: 2,
      tracked_count: 2,
      class_counts: { person: 2, chair: 1 },
      truncated: false,
      detections: [
        { id: "person_12", class_id: 0, class_name: "person", score: 0.94, center_x: 522, center_y: 351, width: 182, height: 468, keypoint_count: 17 },
        { id: "person_18", class_id: 0, class_name: "person", score: 0.87, center_x: 906, center_y: 338, width: 146, height: 421, keypoint_count: 17 },
        { id: "chair_4", class_id: 56, class_name: "chair", score: 0.76, center_x: 1112, center_y: 468, width: 224, height: 248, keypoint_count: 0 },
      ],
    },
    pose: {
      wave_detected: true,
      last_detected_at: demoTimestamp - 0.15,
      status: "[person_12] Waving with right hand!",
      updated_at: demoTimestamp - 0.15,
    },
    obstacle: {
      frame_id: "base_link",
      point_count: 848,
      obstacle_point_count: 63,
      nearest_range: 0.82,
      range_min: 0.4,
      range_max: 5,
    },
  },
  mission: {
    schema_version: "0.1.0",
    timestamp: demoTimestamp - 0.1,
    mode: "GO_TO_B",
    active: true,
    started_at: demoTimestamp - 86,
    mode_changed_at: demoTimestamp - 14,
    waypoint: { index: 1, display_index: 2, total: 4, name: "Hall B" },
    goal: { frame_id: "map", x: demoGoalX, y: demoGoalY },
    nav2: {
      state: "EXECUTING",
      label: "WP:Hall B",
      goal_active: true,
      distance_remaining: 1.73,
      result_status_code: null,
      result_status: "",
    },
    intercept: {
      enabled: true,
      done_this_trip: false,
      person_detected: true,
      track_id: "id:person_12",
      hand: "RIGHT",
      wave_hold_seconds: 2.4,
      distance_m: 1.62,
    },
    wait_remaining_seconds: 0,
    last_error: "",
    source_topic: "/mission/status",
    received_at: demoTimestamp - 0.1,
  },
  diagnostics: {
    summary: { ok_count: 1, warn_count: 1, error_count: 1, stale_count: 0, total_count: 3 },
    message_timestamp: demoTimestamp - 0.2,
    truncated: false,
    statuses: [
      {
        name: "Battery monitor",
        level: 0,
        level_name: "OK",
        message: "Battery telemetry healthy",
        hardware_id: "esp32-drive",
        values: { voltage: "23.8 V", percentage: "78%", temperature: "41.2 C" },
        timestamp: demoTimestamp - 0.2,
      },
      {
        name: "Local planner frequency",
        level: 1,
        level_name: "WARN",
        message: "Update loop below configured frequency",
        hardware_id: "controller_server",
        values: { expected_rate: "5.0 Hz", observed_rate: "1.8 Hz", missed_cycles: "12" },
        timestamp: demoTimestamp - 0.2,
      },
      {
        name: "Depth camera",
        level: 2,
        level_name: "ERROR",
        message: "Depth stream is not available",
        hardware_id: "realsense-front",
        values: { device: "/dev/video2", reconnect_attempts: "3" },
        timestamp: demoTimestamp - 0.2,
      },
    ],
  },
  ros_logs: [
    {
      timestamp: demoTimestamp - 9,
      level: 20,
      level_name: "INFO",
      name: "/bt_navigator",
      message: "Begin navigating from current pose to requested goal",
      file: "bt_navigator.cpp",
      function: "navigateToPose",
      line: 284,
    },
    {
      timestamp: demoTimestamp - 6,
      level: 10,
      level_name: "DEBUG",
      name: "/controller_server",
      message: "Computed velocity command vx=0.18 vy=0.02 wz=0.08",
      file: "controller_server.cpp",
      function: "computeControl",
      line: 511,
    },
    {
      timestamp: demoTimestamp - 3,
      level: 30,
      level_name: "WARN",
      name: "/controller_server",
      message: "Control loop missed desired 5 Hz update rate",
      file: "controller_server.cpp",
      function: "computeControl",
      line: 538,
    },
    {
      timestamp: demoTimestamp - 1,
      level: 40,
      level_name: "ERROR",
      name: "/depth_camera",
      message: "Depth stream timeout; retrying device connection",
      file: "camera_node.cpp",
      function: "pollFrames",
      line: 193,
    },
  ],
  ros_graph: {
    summary: { node_count: 10, topic_count: 16, publisher_count: 14, subscription_count: 18, edge_count: 32 },
    nodes: [
      { id: "node:/robot_ui_bridge", label: "/robot_ui_bridge", kind: "node" },
      { id: "node:/ekf_filter_node", label: "/ekf_filter_node", kind: "node" },
      { id: "node:/esp_bridge", label: "/esp_bridge", kind: "node" },
      { id: "node:/sllidar_node", label: "/sllidar_node", kind: "node" },
      { id: "node:/bno055", label: "/bno055", kind: "node" },
      { id: "node:/bt_navigator", label: "/bt_navigator", kind: "node" },
      { id: "node:/controller_server", label: "/controller_server", kind: "node" },
      { id: "node:/planner_server", label: "/planner_server", kind: "node" },
      { id: "node:/yolo_node", label: "/yolo_node", kind: "node" },
      { id: "node:/mission_ab_person_once", label: "/mission_ab_person_once", kind: "node" },
      { id: "topic:/odom", label: "/odom", kind: "topic" },
      { id: "topic:/odom_encoder", label: "/odom_encoder", kind: "topic" },
      { id: "topic:/imu/data", label: "/imu/data", kind: "topic" },
      { id: "topic:/scan", label: "/scan", kind: "topic" },
      { id: "topic:/cmd_vel", label: "/cmd_vel", kind: "topic" },
      { id: "topic:/battery", label: "/battery", kind: "topic" },
      { id: "topic:/dataenc", label: "/dataenc", kind: "topic" },
      { id: "topic:/map", label: "/map", kind: "topic" },
      { id: "topic:/goal_pose", label: "/goal_pose", kind: "topic" },
      { id: "topic:/plan", label: "/plan", kind: "topic" },
      { id: "topic:/local_plan", label: "/local_plan", kind: "topic" },
      { id: "topic:/global_costmap/costmap_raw", label: "/global_costmap/costmap_raw", kind: "topic" },
      { id: "topic:/local_costmap/costmap_raw", label: "/local_costmap/costmap_raw", kind: "topic" },
      { id: "topic:/navigate_to_pose/_action/status", label: "/navigate_to_pose/_action/status", kind: "topic" },
      { id: "topic:/yolo/detections", label: "/yolo/detections", kind: "topic" },
      { id: "topic:/nhiemvuboss/waypoints_json", label: "/nhiemvuboss/waypoints_json", kind: "topic" },
    ],
    edges: ([
      { id: "e1", source: "node:/esp_bridge", target: "topic:/odom_encoder", kind: "publish" },
      { id: "e2", source: "topic:/odom_encoder", target: "node:/ekf_filter_node", kind: "subscribe" },
      { id: "e3", source: "node:/sllidar_node", target: "topic:/scan", kind: "publish" },
      { id: "e4", source: "topic:/scan", target: "node:/ekf_filter_node", kind: "subscribe" },
      { id: "e5", source: "node:/bno055", target: "topic:/imu/data", kind: "publish" },
      { id: "e6", source: "topic:/imu/data", target: "node:/ekf_filter_node", kind: "subscribe" },
      { id: "e7", source: "node:/ekf_filter_node", target: "topic:/odom", kind: "publish" },
      { id: "e8", source: "topic:/odom", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e9", source: "topic:/scan", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e10", source: "node:/esp_bridge", target: "topic:/battery", kind: "publish" },
      { id: "e11", source: "topic:/battery", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e12", source: "node:/esp_bridge", target: "topic:/dataenc", kind: "publish" },
      { id: "e13", source: "topic:/dataenc", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e14", source: "node:/bt_navigator", target: "topic:/cmd_vel", kind: "publish" },
      { id: "e15", source: "topic:/cmd_vel", target: "node:/esp_bridge", kind: "subscribe" },
      { id: "e16", source: "node:/yolo_node", target: "topic:/yolo/detections", kind: "publish" },
      { id: "e17", source: "topic:/yolo/detections", target: "node:/mission_ab_person_once", kind: "subscribe" },
      { id: "e18", source: "node:/mission_ab_person_once", target: "topic:/cmd_vel", kind: "publish" },
      { id: "e19", source: "topic:/map", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e20", source: "topic:/nhiemvuboss/waypoints_json", target: "node:/mission_ab_person_once", kind: "subscribe" },
      { id: "e21", source: "topic:/goal_pose", target: "node:/bt_navigator", kind: "subscribe" },
      { id: "e22", source: "topic:/goal_pose", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e23", source: "node:/planner_server", target: "topic:/plan", kind: "publish" },
      { id: "e24", source: "topic:/plan", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e25", source: "node:/controller_server", target: "topic:/local_plan", kind: "publish" },
      { id: "e26", source: "topic:/local_plan", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e27", source: "node:/planner_server", target: "topic:/global_costmap/costmap_raw", kind: "publish" },
      { id: "e28", source: "topic:/global_costmap/costmap_raw", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e29", source: "node:/controller_server", target: "topic:/local_costmap/costmap_raw", kind: "publish" },
      { id: "e30", source: "topic:/local_costmap/costmap_raw", target: "node:/robot_ui_bridge", kind: "subscribe" },
      { id: "e31", source: "node:/bt_navigator", target: "topic:/navigate_to_pose/_action/status", kind: "publish" },
      { id: "e32", source: "topic:/navigate_to_pose/_action/status", target: "node:/robot_ui_bridge", kind: "subscribe" },
    ] as RosGraphEdge[]).map((edge) => {
      const topicId = edge.kind === "publish" ? edge.target : edge.source;
      const topicName = topicId.replace("topic:", "");
      const sensorTopic = topicName === "/scan" || topicName === "/imu/data";
      return {
        ...edge,
        message_types: demoMessageTypes[topicName] ?? [],
        qos: {
          reliability: sensorTopic ? "BEST_EFFORT" : "RELIABLE",
          durability: topicName === "/map" ? "TRANSIENT_LOCAL" : "VOLATILE",
          history: "KEEP_LAST",
          depth: topicName === "/map" ? 1 : 10,
        },
      };
    }),
  },
  topics: {
    "/odom": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.03,
      stale_after_seconds: 2,
      rate_hz: 29.8,
      expected_rate_hz: 30,
      message_count: 18432,
      last_message_at: demoTimestamp,
      message_type: "nav_msgs/Odometry",
      latest: { x: 1.15, y: 0.6, yaw: 0.42, linear_x: 0.18, angular_z: 0.08 },
    },
    "/scan": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.08,
      stale_after_seconds: 2,
      rate_hz: 9.9,
      expected_rate_hz: 10,
      message_count: 6144,
      last_message_at: demoTimestamp - 0.08,
      message_type: "sensor_msgs/LaserScan",
      latest: { point_count: 720, valid_point_count: 684, nearest_range: 0.73 },
    },
    "/imu/data": {
      state: "STALE",
      health_monitored: true,
      age_seconds: 6.8,
      stale_after_seconds: 2,
      rate_hz: 49.7,
      expected_rate_hz: 50,
      message_count: 30720,
      last_message_at: demoTimestamp - 0.01,
      message_type: "sensor_msgs/Imu",
      latest: { orientation_z: 0.208, orientation_w: 0.978, angular_velocity_z: 0.08 },
    },
    "/battery": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.2,
      stale_after_seconds: 2,
      rate_hz: 2,
      expected_rate_hz: 2,
      message_count: 1228,
      last_message_at: demoTimestamp - 0.2,
      message_type: "sensor_msgs/BatteryState",
      latest: { voltage: 23.8, percentage: 0.78, temperature: 41.2, present: true },
    },
    "/dataenc": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.02,
      stale_after_seconds: 2,
      rate_hz: 29.9,
      expected_rate_hz: 30,
      message_count: 18394,
      last_message_at: demoTimestamp - 0.02,
      message_type: "std_msgs/Int32MultiArray",
      latest: [153802, 154019, 153774, 153991],
    },
    "/map": {
      state: "OK",
      health_monitored: true,
      age_seconds: 1.7,
      stale_after_seconds: 15,
      rate_hz: 0.2,
      expected_rate_hz: 0.2,
      message_count: 126,
      last_message_at: demoTimestamp - 1.7,
      message_type: "nav_msgs/OccupancyGrid",
      latest: { frame_id: "map", width: demoMapWidth, height: demoMapHeight, resolution: 0.05 },
    },
    "/cmd_vel": {
      state: "OK",
      rate_hz: 19.8,
      expected_rate_hz: 20,
      message_count: 8021,
      last_message_at: demoTimestamp - 0.04,
      message_type: "geometry_msgs/Twist",
      latest: { linear: { x: 0.18, y: 0.02 }, angular: { z: 0.08 } },
    },
    "/goal_pose": {
      state: "OK",
      message_count: 4,
      last_message_at: demoTimestamp - 7,
      message_type: "geometry_msgs/PoseStamped",
      latest: { frame_id: "map", x: demoGoalX, y: demoGoalY, yaw: 0.15 },
    },
    "/plan": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.4,
      stale_after_seconds: 3,
      rate_hz: 1,
      expected_rate_hz: 1,
      message_count: 86,
      last_message_at: demoTimestamp - 0.4,
      message_type: "nav_msgs/Path",
      latest: { frame_id: "map", point_count: demoGlobalPath.length },
    },
    "/local_plan": {
      state: "WARN",
      health_monitored: true,
      age_seconds: 0.08,
      stale_after_seconds: 2,
      rate_hz: 1.8,
      expected_rate_hz: 5,
      message_count: 422,
      last_message_at: demoTimestamp - 0.08,
      message_type: "nav_msgs/Path",
      latest: { frame_id: "map", point_count: demoLocalPath.length },
    },
    "/global_costmap/costmap_raw": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.5,
      stale_after_seconds: 3,
      rate_hz: 1,
      expected_rate_hz: 1,
      message_count: 86,
      last_message_at: demoTimestamp - 0.5,
      message_type: "nav_msgs/OccupancyGrid",
      latest: { frame_id: "map", width: 120, height: 80, resolution: 0.08 },
    },
    "/local_costmap/costmap_raw": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.06,
      stale_after_seconds: 2,
      rate_hz: 5,
      expected_rate_hz: 5,
      message_count: 424,
      last_message_at: demoTimestamp - 0.06,
      message_type: "nav_msgs/OccupancyGrid",
      latest: { frame_id: "odom", width: 72, height: 72, resolution: 0.05 },
    },
    "/navigate_to_pose/_action/status": {
      state: "OK",
      rate_hz: 5,
      message_count: 432,
      last_message_at: demoTimestamp - 0.05,
      message_type: "action_msgs/GoalStatusArray",
      latest: { status: "EXECUTING", active_goal_count: 1 },
    },
    "/mission/status": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.1,
      stale_after_seconds: 2,
      rate_hz: 2,
      expected_rate_hz: 2,
      message_count: 172,
      last_message_at: demoTimestamp - 0.1,
      message_type: "std_msgs/String",
      latest: { mode: "GO_TO_B", active: true, waypoint: { display_index: 2, total: 4, name: "Hall B" } },
    },
    "/camera/color/camera_info": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.03,
      stale_after_seconds: 2,
      rate_hz: 29.9,
      expected_rate_hz: 5,
      message_count: 5221,
      last_message_at: demoTimestamp - 0.03,
      message_type: "sensor_msgs/CameraInfo",
      latest: { frame_id: "camera_color_optical_frame", width: 1280, height: 720 },
    },
    "/camera/depth/camera_info": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.04,
      stale_after_seconds: 2,
      rate_hz: 29.8,
      expected_rate_hz: 5,
      message_count: 5218,
      last_message_at: demoTimestamp - 0.04,
      message_type: "sensor_msgs/CameraInfo",
      latest: { frame_id: "camera_depth_optical_frame", width: 848, height: 480 },
    },
    "/yolo/detections": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.08,
      stale_after_seconds: 2,
      rate_hz: 11.6,
      expected_rate_hz: 5,
      message_count: 2031,
      last_message_at: demoTimestamp - 0.08,
      message_type: "yolo_msgs/DetectionArray",
      latest: { detection_count: 3, person_count: 2, class_counts: { person: 2, chair: 1 } },
    },
    "/pose/wave_detected": {
      state: "OK",
      health_monitored: false,
      age_seconds: 0.15,
      rate_hz: 3.2,
      message_count: 87,
      last_message_at: demoTimestamp - 0.15,
      message_type: "std_msgs/Bool",
      latest: { wave_detected: true },
    },
    "/pose/wave_status": {
      state: "OK",
      health_monitored: false,
      age_seconds: 0.15,
      message_count: 16,
      last_message_at: demoTimestamp - 0.15,
      message_type: "std_msgs/String",
      latest: { status: "[person_12] Waving with right hand!" },
    },
    "/scan_obstacles": {
      state: "WARN",
      health_monitored: true,
      age_seconds: 0.21,
      stale_after_seconds: 2,
      rate_hz: 2.1,
      expected_rate_hz: 5,
      message_count: 711,
      last_message_at: demoTimestamp - 0.21,
      message_type: "sensor_msgs/LaserScan",
      latest: { frame_id: "base_link", obstacle_point_count: 63, nearest_range: 0.82 },
    },
    "/diagnostics": {
      state: "OK",
      health_monitored: true,
      age_seconds: 0.2,
      stale_after_seconds: 3,
      rate_hz: 1,
      expected_rate_hz: 1,
      message_count: 621,
      last_message_at: demoTimestamp - 0.2,
      message_type: "diagnostic_msgs/DiagnosticArray",
      latest: { ok_count: 1, warn_count: 1, error_count: 1, stale_count: 0, total_count: 3 },
    },
    "/rosout": {
      state: "OK",
      health_monitored: false,
      age_seconds: 0.04,
      message_count: 2384,
      last_message_at: demoTimestamp - 0.04,
      message_type: "rcl_interfaces/Log",
      latest: { level: "ERROR", name: "/depth_camera", message: "Depth stream timeout" },
    },
  },
  events: [
    { timestamp: Date.now() / 1000 - 1, level: "ERROR", source: "depth_camera", message: "Depth stream timeout; retrying device connection" },
    { timestamp: Date.now() / 1000 - 3, level: "WARN", source: "topic_health", message: "/local_plan changed from OK to WARN" },
    { timestamp: Date.now() / 1000 - 6, level: "ERROR", source: "topic_health", message: "/imu/data changed from OK to STALE" },
    { timestamp: Date.now() / 1000, level: "INFO", source: "ros_graph", message: "Graph discovery completed" },
    { timestamp: Date.now() / 1000 - 4, level: "INFO", source: "ekf", message: "Odometry fusion healthy" },
    { timestamp: Date.now() / 1000 - 9, level: "WARN", source: "vision", message: "Depth stream not connected in demo mode" },
    { timestamp: Date.now() / 1000 - 14, level: "INFO", source: "navigation", message: "Nav2 lifecycle nodes active" },
  ],
};
