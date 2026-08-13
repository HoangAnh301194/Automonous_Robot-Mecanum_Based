from __future__ import annotations

from robot_ui.config import AppConfig
from robot_ui.ros_bridge import RosBridge
from robot_ui.state_store import StateStore
from robot_ui.system_monitor import SystemMonitor


class Runtime:
    def __init__(self, config: AppConfig):
        self.config = config
        self.store = StateStore(str(config.source))
        self.ros_bridge = RosBridge(self.store, config.ros)
        self.system_monitor = SystemMonitor(self.store)

    def start(self) -> None:
        self.store.append_event("INFO", "server", "Robot UI runtime starting")
        self.system_monitor.start()
        self.ros_bridge.start()

    def stop(self) -> None:
        self.store.append_event("INFO", "server", "Robot UI runtime stopping")
        self.ros_bridge.stop()
        self.system_monitor.stop()
