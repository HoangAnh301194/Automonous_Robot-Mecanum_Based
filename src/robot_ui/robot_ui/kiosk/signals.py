from PyQt5.QtCore import QObject, pyqtSignal


class KioskSignals(QObject):
    connection_changed = pyqtSignal(bool)
    robot_status_changed = pyqtSignal(dict)
    navigation_status_changed = pyqtSignal(dict)
    map_changed = pyqtSignal(dict)
    pose_changed = pyqtSignal(dict)
    path_changed = pyqtSignal(list)
    locations_changed = pyqtSignal(list)
    command_status_changed = pyqtSignal(str, str)
    navigation_active_changed = pyqtSignal(bool)
    service_popup_cmd = pyqtSignal(str)   # Lệnh popup từ Behavior Tree: SHOW_5S_INVITE, SHOW_SERVING, HIDE
    # Nav2 detailed feedback and sensor overlays
    nav_feedback_changed = pyqtSignal(dict)           # (dist_remain, initial_dist, nav_time, est_time, recoveries)
    scan_points_changed = pyqtSignal(list)            # [(x, y), ...] world coordinates from LiDAR
    obstacle_points_changed = pyqtSignal(str, list)   # ("front"|"rear", [(x, y), ...]) world coordinates
    patrol_route_changed = pyqtSignal(list)           # [{"name": ..., "x": ..., "y": ..., "yaw": ...}, ...]
    # LLM chatbot signals
    llm_response_ready = pyqtSignal(int, str)   # phát khi LLM trả về response (backward compatible)
    llm_stream_delta = pyqtSignal(int, int, str)       # (session_id, turn_id, delta_text)
    llm_stream_completed = pyqtSignal(int, int, str)   # (session_id, turn_id, full_text)
    llm_stream_interrupted = pyqtSignal(int, int)      # (session_id, turn_id)
    llm_status_changed = pyqtSignal(str)   # phát khi trạng thái LLM thay đổi ("connecting", "online", "error: ...")
    # Voice chat signals
    voice_state_changed = pyqtSignal(str, str)            # (state_str, status_msg)
    voice_transcript_ready = pyqtSignal(int, int, str)    # (session_id, turn_id, transcript)
    tts_started = pyqtSignal(int, int)                    # (session_id, turn_id)
    tts_finished = pyqtSignal(int, int)                   # (session_id, turn_id)
    # Wake word signals
    wake_word_detected = pyqtSignal(str)                  # (keyword_id, e.g. "vi_sota_oi")
    # SLAM & Map signals
    slam_status_changed = pyqtSignal(bool, str)           # (is_running, status_message)
    map_saved_result = pyqtSignal(bool, str)              # (success, message)
    # System Configuration & Telemetry signals
    system_telemetry_changed = pyqtSignal(dict)           # Cảm biến, TF, Pin 36V, AI
    host_telemetry_changed = pyqtSignal(dict)             # Jetson CPU, GPU, RAM, Disk, IP, WiFi
    layer_status_changed = pyqtSignal(dict)               # Trạng thái các tầng tiến trình
    layer_log_received = pyqtSignal(str, str)             # (layer_key, log_line)
    serial_ports_changed = pyqtSignal(str, str, str)       # (hoverboard_port, lidar_port, discovery_method)
    preflight_step_updated = pyqtSignal(str, bool, str)   # (step_key, is_ok, message)
    actuator_test_result = pyqtSignal(str, bool, str)     # (test_key, is_ok, message)

