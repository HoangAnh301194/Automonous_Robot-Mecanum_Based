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

