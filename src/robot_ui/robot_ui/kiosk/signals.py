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
    # LLM chatbot signals
    llm_response_ready = pyqtSignal(str)   # phát khi LLM trả về response
    llm_status_changed = pyqtSignal(str)   # phát khi trạng thái LLM thay đổi ("connecting", "online", "error: ...")
