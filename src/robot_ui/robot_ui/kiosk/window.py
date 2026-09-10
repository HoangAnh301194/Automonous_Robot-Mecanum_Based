from __future__ import annotations

from typing import Any

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QMainWindow, QStackedWidget, QWidget

from robot_ui.kiosk.assets import EmotionCatalog, icon_path
from robot_ui.kiosk.llm_client import LLMWorker
from robot_ui.kiosk.ros_node import KioskRosNode
from robot_ui.kiosk.signals import KioskSignals
from robot_ui.kiosk.widgets import BottomNavigation, ChatPage, HomePage, NavigationPage


KIOSK_STYLESHEET = """
* {
    font-family: "DejaVu Sans", "Noto Sans", sans-serif;
}
QMainWindow, QStackedWidget {
    background: #eef2f4;
}
QLabel#pageTitle {
    color: #172027;
    font-size: 25px;
    font-weight: 700;
}
QLabel#pageSubtitle {
    color: #6c7a84;
    font-size: 13px;
}
QLabel#homeTelemetryBadge {
    color: white;
    background: rgba(185, 54, 54, 205);
    border-radius: 14px;
    padding: 7px 12px;
    font-size: 11px;
    font-weight: 700;
}
QLabel#homeTelemetryBadge {
    background: rgba(15, 22, 27, 185);
}
QFrame#bottomNavigation {
    background: transparent;
    border: none;
}
QPushButton#kioskNavButton {
    color: #58656d;
    background: transparent;
    border: none;
    border-bottom: 3px solid transparent;
    padding: 3px 10px;
}
QPushButton#kioskNavButton:checked {
    color: #d97800;
    background: transparent;
    border-bottom: 3px solid #d97800;
}
QPushButton#kioskNavButton[dark="true"] {
    color: white;
}
QPushButton#kioskNavButton[dark="true"]:checked {
    color: #ffb32c;
    border-bottom: 3px solid #ffb32c;
}
QPushButton#kioskNavButton:pressed {
    background: transparent;
}
QLabel#lightBadge {
    color: #30404a;
    background: white;
    border: 1px solid #d9e0e4;
    border-radius: 14px;
    padding: 8px 12px;
    font-size: 11px;
    font-weight: 700;
}
QWidget#mapWidget {
    border-radius: 16px;
}
QFrame#navigationPanel {
    background: white;
    border: 1px solid #dce3e7;
    border-radius: 16px;
}
QLabel#panelTitle {
    color: #172027;
    font-size: 16px;
    font-weight: 700;
}
QPushButton#textButton {
    color: #2473cd;
    background: transparent;
    border: none;
    padding: 6px;
    font-size: 11px;
    font-weight: 700;
}
QPushButton#destinationButton {
    color: #26343c;
    background: #f3f6f7;
    border: 1px solid #e0e6e9;
    border-radius: 11px;
    padding: 8px 12px;
    text-align: left;
    font-size: 13px;
    font-weight: 700;
}
QPushButton#destinationButton:pressed {
    background: #ffb32c;
}
QLabel#emptyText {
    color: #87949c;
    padding: 16px 4px;
    font-size: 12px;
}
QLabel#navigationStatus {
    color: #40505a;
    background: #edf5ff;
    border-radius: 10px;
    padding: 9px;
    font-size: 11px;
}
QLabel#navigationStatus[tone="error"] {
    color: #982d2d;
    background: #fff0f0;
}
QLabel#navigationStatus[tone="success"] {
    color: #176940;
    background: #eaf8f0;
}
QLabel#navigationStatus[tone="warning"] {
    color: #8a5a00;
    background: #fff6df;
}
QPushButton#dangerButton {
    color: #a52d2d;
    background: #fff0f0;
    border: 1px solid #f2cccc;
    border-radius: 11px;
    font-size: 12px;
    font-weight: 700;
}
QLineEdit#chatInput {
    color: #f2f2f2;
    background: #111111;
    border: 1px solid #383838;
    border-radius: 11px;
    padding: 0 15px;
    font-size: 14px;
}
QLineEdit#chatInput:focus {
    border-color: #ffb32c;
}
QPushButton#primaryButton {
    color: #151515;
    background: #ffb32c;
    border: none;
    border-radius: 11px;
    font-size: 13px;
    font-weight: 700;
}
QPushButton#primaryButton:pressed {
    background: #d98b00;
}
QScrollArea {
    background: transparent;
    border: none;
}
QScrollBar:vertical {
    width: 8px;
    background: transparent;
}
QScrollBar::handle:vertical {
    min-height: 28px;
    background: #cad3d8;
    border-radius: 4px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}
QWidget#navigationPage {
    background: #000000;
}
QFrame#navControlBar {
    background: #0d0d0d;
    border: 1px solid #2d2d2d;
    border-radius: 13px;
}
QPushButton#navHomeButton {
    background: #171717;
    border: 1px solid #3a3a3a;
    border-radius: 9px;
}
QPushButton#navHomeButton:pressed {
    background: #2b2b2b;
}
QLabel#navClock {
    color: #eeeeee;
    font-size: 10px;
    font-weight: 700;
}
QLabel#navPrompt {
    color: #eeeeee;
    font-size: 13px;
    font-weight: 700;
}
QLabel#navPrompt[tone="warning"] {
    color: #ffc45b;
}
QLabel#navPrompt[tone="error"] {
    color: #ff667d;
}
QLabel#navPrompt[tone="success"] {
    color: #4ae1a1;
}
QPushButton#navStartButton, QPushButton#navStopButton {
    background: transparent;
    border: none;
    padding: 0;
}
QWidget#chatPage, QWidget#chatMessageContainer, QWidget#chatMessageRow {
    background: #000000;
}
QFrame#chatHeader, QFrame#chatInputBar {
    background: #0d0d0d;
    border: 1px solid #2d2d2d;
    border-radius: 13px;
}
QLabel#chatHeaderIcon {
    background: #171717;
    border: 1px solid #3a3a3a;
    border-radius: 9px;
}
QLabel#chatTitle {
    color: #f4f4f4;
    font-size: 13px;
    font-weight: 700;
}
QLabel#chatSubtitle {
    color: #8d8d8d;
    font-size: 9px;
}
QLabel#llmStatus {
    color: #ffb32c;
    background: #1d1810;
    border: 1px solid #4b3b20;
    border-radius: 10px;
    padding: 5px 9px;
    font-size: 10px;
    font-weight: 700;
}
QFrame#chatConversation {
    background: #080808;
    border: 1px solid #292929;
    border-radius: 14px;
}
QScrollArea#chatScroll, QScrollArea#chatScroll QWidget#qt_scrollarea_viewport {
    background: #080808;
    border: none;
}
QFrame#robotBubble {
    background: #181818;
    border: 1px solid #353535;
    border-radius: 12px;
}
QFrame#userBubble {
    background: #2a1d08;
    border: 1px solid #8b5b08;
    border-radius: 12px;
}
QLabel#robotBubbleSender {
    color: #ffb32c;
    font-size: 10px;
    font-weight: 700;
}
QLabel#userBubbleSender {
    color: #ffbd45;
    font-size: 10px;
    font-weight: 700;
}
QLabel#bubbleText {
    color: #ededed;
    font-size: 13px;
}
"""


class KioskWindow(QMainWindow):
    def __init__(self, ros_node: KioskRosNode, signals: KioskSignals) -> None:
        super().__init__()
        self.ros_node = ros_node
        self.signals = signals
        self.emotions = EmotionCatalog()
        self._robot_status: dict[str, Any] = {}
        self._navigation_status: dict[str, Any] = {}
        self._connected = False
        self._active_page = "home"

        self.setWindowTitle("Robot Assistant")
        self.setMinimumSize(800, 480)
        self.setStyleSheet(KIOSK_STYLESHEET)

        central = QWidget()
        self.setCentralWidget(central)
        self.stack = QStackedWidget(central)
        self.home_page = HomePage()
        self.navigation_page = NavigationPage(
            {
                "home": icon_path("homeIcon.png"),
                "click_goal": icon_path("clickGoal.png"),
                "goal": icon_path("location.png"),
                "start": icon_path("startNavigation.png"),
                "stop": icon_path("stopNavigation.png"),
            }
        )
        self.chat_page = ChatPage(icon_path("chatIcon.png"))
        self.pages = {
            "home": self.home_page,
            "navigation": self.navigation_page,
            "chat": self.chat_page,
        }
        for page in self.pages.values():
            self.stack.addWidget(page)

        self.bottom_navigation = BottomNavigation(
            {
                "home": icon_path("homeIcon.png"),
                "navigation": icon_path("locolization.png"),
                "chat": icon_path("chatIcon.png"),
            },
            central,
        )
        self.bottom_navigation.page_requested.connect(self.show_page)
        self.navigation_page.home_requested.connect(lambda: self.show_page("home"))
        self.navigation_page.start_requested.connect(self.ros_node.navigate_to_pose)
        self.navigation_page.cancel_requested.connect(self.ros_node.cancel_navigation)

        signals.connection_changed.connect(self._on_connection)
        signals.robot_status_changed.connect(self._on_robot_status)
        signals.navigation_status_changed.connect(self._on_navigation_status)
        signals.map_changed.connect(self.navigation_page.map_widget.set_map)
        signals.pose_changed.connect(self.navigation_page.set_pose)
        signals.path_changed.connect(self.navigation_page.map_widget.set_path)
        signals.locations_changed.connect(self.navigation_page.set_locations)
        signals.command_status_changed.connect(self.navigation_page.set_command_status)
        signals.navigation_active_changed.connect(self.navigation_page.set_navigation_active)
        # LLM signals
        signals.llm_response_ready.connect(self.chat_page.receive_response)
        signals.llm_status_changed.connect(self.chat_page.set_llm_status)

        # LLM worker – thread-safe callbacks emit vào signals (an toàn với Qt)
        def _on_llm_response(text: str) -> None:
            signals.llm_response_ready.emit(text)
            signals.llm_status_changed.emit("●  LLM: SẵN SÀNG")

        def _on_llm_error(err: str) -> None:
            signals.llm_status_changed.emit("●  LLM: LỖI")
            # emit qua signal để gọi receive_error trên main thread
            signals.llm_response_ready.emit(f"⚠️ {err}")

        self._llm_worker = LLMWorker(
            on_response=_on_llm_response,
            on_error=_on_llm_error,
        )
        self.chat_page.chat_submitted.connect(self._on_chat_submitted)

        self.home_page.video.set_display_scale(self.emotions.display_scale)
        self.home_page.video.set_vertical_offset(self.emotions.vertical_offset_px)
        self.home_page.video.set_video(self.emotions.path("idle"))
        self.show_page("home")
        QTimer.singleShot(800, self.ros_node.request_locations)

    def resizeEvent(self, event: Any) -> None:
        central = self.centralWidget()
        self.stack.setGeometry(central.rect())
        nav_width = min(260, max(238, central.width() - 32))
        nav_height = 64
        self.bottom_navigation.setGeometry(
            (central.width() - nav_width) // 2,
            central.height() - nav_height - 12,
            nav_width,
            nav_height,
        )
        self.bottom_navigation.raise_()
        super().resizeEvent(event)

    def show_page(self, page_id: str) -> None:
        page = self.pages.get(page_id)
        if page is None:
            return
        self._active_page = page_id
        self.stack.setCurrentWidget(page)
        self.bottom_navigation.setVisible(page_id != "navigation")
        self.bottom_navigation.set_dark_background(page_id in {"home", "chat"})
        self.bottom_navigation.set_active(page_id)
        if self.bottom_navigation.isVisible():
            self.bottom_navigation.raise_()
        if page_id == "home":
            self.home_page.video.start()
            self._update_emotion()
        else:
            self.home_page.video.pause()
        if page_id == "navigation":
            self.ros_node.request_locations()

    def _on_connection(self, connected: bool) -> None:
        self._connected = connected
        self._update_emotion()

    def _on_robot_status(self, status: dict[str, Any]) -> None:
        self._robot_status = status
        self.home_page.set_robot_status(status)
        self.navigation_page.set_robot_status(status)
        self._update_emotion()

    def _on_navigation_status(self, status: dict[str, Any]) -> None:
        self._navigation_status = status
        self.navigation_page.set_navigation_status(status)
        self._update_emotion()

    def _on_chat_submitted(self, message: str) -> None:
        """Nhận tin nhắn từ ChatPage và gửi cho LLM worker."""
        self.signals.llm_status_changed.emit("●  LLM: ĐANG XỬ LÝ...")
        sent = self._llm_worker.send(message, robot_status=self._robot_status)
        if not sent:
            # Worker bận, trả lời ngay trên UI
            self.signals.llm_response_ready.emit("⏳ Robot đang xử lý tin nhắn trước, vui lòng chờ giây lát.")

    def _update_emotion(self) -> None:
        if self._active_page != "home":
            return
        robot_state = str(self._robot_status.get("robot_state", "IDLE")).upper()
        nav_state = str(self._navigation_status.get("status", "IDLE")).upper()
        sensors_ok = all(
            self._robot_status.get(key, True)
            for key in ("lidar_ok", "localization_ok", "odom_ok")
        )
        if robot_state in {"ERROR", "FAILED"}:
            emotion = "error"
        elif self._robot_status.get("battery_low") or self._robot_status.get("water_low") or not sensors_ok:
            emotion = "warning"
        elif nav_state == "REACHED_GOAL":
            emotion = "arrived"
        elif nav_state in {"FETCHING_LOCATION", "WAITING_FOR_NAV2", "MOVING"}:
            emotion = "navigating"
        else:
            emotion = "idle"
        self.home_page.video.set_video(self.emotions.path(emotion))

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key_Escape:
            self.close()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event: Any) -> None:
        self.home_page.video.stop()
        super().closeEvent(event)
