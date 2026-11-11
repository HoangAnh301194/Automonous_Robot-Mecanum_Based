from __future__ import annotations

from typing import Any
import os
import time

from PyQt5.QtCore import QEvent, QTimer, Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QApplication, QMainWindow, QStackedWidget, QWidget

from robot_ui.kiosk.assets import EmotionCatalog, icon_path
from robot_ui.kiosk.audio_worker import VoiceController, VoiceState
from robot_ui.kiosk.llm_client import LLMWorker, TurnContext
from robot_ui.kiosk.pages import AdminPage, AdminPinDialog, ServicePage, WelcomePage
from robot_ui.kiosk.ros_node import KioskRosNode
from robot_ui.kiosk.signals import KioskSignals
from robot_ui.kiosk.widgets import BottomNavigation, ChatPage, HomePage, MicState, NavigationPage


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
    font-size: 15px;
    font-weight: 700;
}
QLabel#chatSubtitle {
    color: #8d8d8d;
    font-size: 11px;
}
QPushButton#chatSpeakerButton {
    background: #171717;
    border: 1px solid #3a3a3a;
    border-radius: 11px;
}
QPushButton#chatSpeakerButton:pressed {
    background: #2b2b2b;
}
QPushButton#chatStopSpeakingButton {
    color: #ff667d;
    background: #241014;
    border: 1px solid #5a1d28;
    border-radius: 11px;
    padding: 6px 12px;
    font-size: 12px;
    font-weight: 700;
}
QPushButton#chatStopSpeakingButton:pressed {
    background: #38151c;
}
QFrame#suggestionContainer {
    background: transparent;
    border: none;
}
QPushButton#suggestionChip {
    color: #e2e8f0;
    background: #141414;
    border: 1px solid #2e2e2e;
    border-radius: 12px;
    padding: 9px 12px;
    font-size: 13px;
    font-weight: 600;
    text-align: left;
}
QPushButton#suggestionChip:pressed {
    color: #111111;
    background: #ffb32c;
    border-color: #ffb32c;
}
QFrame#typingBubble {
    background: #181818;
    border: 1px solid #282828;
    border-radius: 12px;
}
QLabel#typingText {
    color: #999999;
    font-size: 14px;
    font-style: italic;
}
QPushButton#chatMicButton {
    background: #171717;
    border: 1px solid #3a3a3a;
    border-radius: 12px;
}
QPushButton#chatMicButton:pressed {
    background: #2b2b2b;
}
QPushButton#chatMicButton[listening="true"] {
    background: #2b1c09;
    border: 1px solid #ffb32c;
}
QPushButton#chatMicButton:disabled {
    background: #101010;
    border-color: #222222;
}
QPushButton#chatSendButton {
    color: #151515;
    background: #ffb32c;
    border: none;
    border-radius: 12px;
    font-size: 14px;
    font-weight: 700;
}
QPushButton#chatSendButton:pressed {
    background: #d98b00;
}
QPushButton#chatSendButton:disabled {
    color: #666666;
    background: #28241b;
}
QLabel#llmStatus {
    color: #4ae1a1;
    background: #111a14;
    border: 1px solid #1e3827;
    border-radius: 9px;
    padding: 5px 9px;
    font-size: 11px;
    font-weight: 700;
}
QLabel#llmStatus[state="busy"] {
    color: #ffb32c;
    background: #1d1810;
    border-color: #4b3b20;
}
QLabel#llmStatus[state="offline"] {
    color: #888888;
    background: #161616;
    border-color: #2a2a2a;
}
QLabel#llmStatus[state="error"] {
    color: #ff667d;
    background: #261115;
    border-color: #5a1d28;
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
    border: 1px solid #282828;
    border-radius: 12px;
}
QFrame#userBubble {
    background: #2a1d08;
    border: 1px solid #6b4606;
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
    font-size: 16px;
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
        self._active_page = ""
        self._page_history: list[str] = []
        self._admin_authenticated = False
        # Prototype PIN; override via KIOSK_ADMIN_PIN before deployment.
        self._admin_pin = os.getenv("KIOSK_ADMIN_PIN", "1")
        try:
            self._admin_idle_timeout_seconds = max(
                30, int(os.getenv("KIOSK_ADMIN_IDLE_SECONDS", "90"))
            )
        except ValueError:
            self._admin_idle_timeout_seconds = 90
        self._robot_status_received_at: float | None = None
        try:
            self._idle_timeout_seconds = max(30, int(os.getenv("KIOSK_IDLE_TIMEOUT_SECONDS", "120")))
        except ValueError:
            self._idle_timeout_seconds = 120
        self._last_activity_at = time.monotonic()

        self.setWindowTitle("Souta Robot")
        self.setMinimumSize(800, 480)
        self.setStyleSheet(KIOSK_STYLESHEET)

        central = QWidget()
        self.setCentralWidget(central)
        self.stack = QStackedWidget(central)
        self.idle_page = HomePage()
        # Idle must show only the animated Souta Robot face, without telemetry/menu.
        self.idle_page.battery_label.hide()
        self.idle_page.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.idle_page.video.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.home_page = WelcomePage(
            background=icon_path("background.jpg"),
            ptit=icon_path("logoPtit.png"),
            faculty=icon_path("fee1.png"),
            service_icon=icon_path("service.png"),
            emergency_icon=icon_path("emergency_stop.png"),
        )
        battery_icons = {
            "0": icon_path("battery_0.png"),
            "25": icon_path("battery_25.png"),
            "50": icon_path("battery_50.png"),
            "75": icon_path("battery_75.png"),
            "100": icon_path("battery_100.png"),
        }
        self.admin_page = AdminPage(
            nav_icon=icon_path("navigationIcon.png"),
            slam_icon=icon_path("SLAMicon.png"),
            detection_icon=icon_path("detection.png"),
            start_nav_icon=icon_path("startNavigation.png"),
            stop_nav_icon=icon_path("stopNavigation.png"),
            status_icon=icon_path("status.png"),
            wifi_connected_icon=icon_path("wifi.png"),
            wifi_disconnected_icon=icon_path("wifidisconected.png"),
            battery_icons=battery_icons,
            back_icon=icon_path("back.png"),
        )
        self.navigation_page = NavigationPage(
            {
                "home": icon_path("homeIcon.png"),
                "back": icon_path("back.png"),
                "click_goal": icon_path("clickGoal.png"),
                "goal": icon_path("location.png"),
                "start": icon_path("startNavigation.png"),
                "stop": icon_path("stopNavigation.png"),
            }
        )
        voice_icons = {
            "mic": icon_path("mic.png"),
            "mic_listening": icon_path("mic_listening.png"),
            "mic_off": icon_path("mic_off.png"),
            "speaker_on": icon_path("speaker_on.png"),
            "speaker_off": icon_path("speaker_off.png"),
            "stop_speaking": icon_path("stop_speaking.png"),
        }
        self.chat_page = ChatPage(
            chat_icon=icon_path("chatIcon.png"),
            back_icon=icon_path("back.png"),
            voice_icons=voice_icons,
        )
        self.service_page = ServicePage(
            back_icon=icon_path("back.png"),
            wait_icon=icon_path("service_wait.png"),
            continue_icon=icon_path("service_continue.png"),
            service_icon=icon_path("service.png"),
        )
        self.pages = {
            "idle": self.idle_page,
            "home": self.home_page,
            "chat": self.chat_page,
            "service": self.service_page,
            "admin": self.admin_page,
            "navigation": self.navigation_page,
        }
        for page in self.pages.values():
            self.stack.addWidget(page)

        self.bottom_navigation = BottomNavigation(
            {
                "home": icon_path("homeIcon.png"),
                "chat": icon_path("chatIcon.png"),
                "service": icon_path("service.png"),
                "emergency": icon_path("emergency_stop.png"),
                "settings": icon_path("setting_icon.png"),
            },
            central,
        )
        self.bottom_navigation.hide()
        self.bottom_navigation.page_requested.connect(self._on_bottom_navigation)
        self.home_page.service_requested.connect(self._on_service_requested)
        self.service_page.back_requested.connect(self.go_back)
        self.chat_page.home_requested.connect(self.go_back)
        self.admin_page.back_requested.connect(self.go_back)
        self.admin_page.logout_requested.connect(self.go_back)
        self.admin_page.navigation_requested.connect(lambda: self.show_page("navigation"))
        self.admin_page.start_nav_requested.connect(self._on_admin_start_nav)
        self.admin_page.stop_nav_requested.connect(self._on_admin_stop_nav)
        self.admin_page.slam_requested.connect(self._on_admin_slam)
        self.admin_page.detection_debug_requested.connect(self._on_admin_detection)
        self.admin_page.system_status_requested.connect(self._on_admin_system_status)
        self.navigation_page.home_requested.connect(self.go_back)
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
        signals.llm_response_ready.connect(self._receive_llm_response)
        signals.llm_stream_delta.connect(self._on_llm_stream_delta)
        signals.llm_stream_completed.connect(self._on_llm_stream_completed)
        signals.llm_stream_interrupted.connect(self._on_llm_stream_interrupted)
        signals.llm_status_changed.connect(self.chat_page.set_llm_status)

        # LLM worker – thread-safe callbacks emit vào signals (an toàn với Qt)
        def _on_llm_response(session_id: int, text: str) -> None:
            signals.llm_response_ready.emit(session_id, text)

        def _on_llm_error(session_id: int, err: str) -> None:
            signals.llm_response_ready.emit(session_id, f"⚠️ {err}")

        def _on_llm_stream_delta(ctx: TurnContext, delta: str) -> None:
            signals.llm_stream_delta.emit(ctx.session_id, ctx.turn_id, delta)

        def _on_llm_stream_complete(ctx: TurnContext, full_text: str) -> None:
            signals.llm_stream_completed.emit(ctx.session_id, ctx.turn_id, full_text)

        def _on_llm_stream_interrupted(ctx: TurnContext) -> None:
            signals.llm_stream_interrupted.emit(ctx.session_id, ctx.turn_id)

        self._llm_worker = LLMWorker(
            on_response=_on_llm_response,
            on_error=_on_llm_error,
            on_stream_delta=_on_llm_stream_delta,
            on_stream_completed=_on_llm_stream_complete,
            on_stream_interrupted=_on_llm_stream_interrupted,
        )
        self.chat_page.chat_submitted.connect(self._on_chat_submitted)

        # Voice controller – thread-safe callbacks emit vào Qt signals
        def _on_voice_state(state: VoiceState, msg: str) -> None:
            signals.voice_state_changed.emit(state.value, msg)

        def _on_voice_transcript(session_id: int, turn_id: int, text: str) -> None:
            signals.voice_transcript_ready.emit(session_id, turn_id, text)

        def _on_tts_start(session_id: int, turn_id: int) -> None:
            signals.tts_started.emit(session_id, turn_id)

        def _on_tts_finish(session_id: int, turn_id: int) -> None:
            signals.tts_finished.emit(session_id, turn_id)

        self._voice_controller = VoiceController(
            on_state_changed=_on_voice_state,
            on_transcript_ready=_on_voice_transcript,
            on_tts_started=_on_tts_start,
            on_tts_finished=_on_tts_finish,
        )
        self.chat_page.mic_clicked.connect(self._voice_controller.toggle_listening)
        self.chat_page.stop_speaking_requested.connect(self._on_stop_requested)
        self.chat_page.speaker_toggled.connect(self._voice_controller.set_speaker_enabled)
        self.chat_page.volume_changed.connect(self._voice_controller.set_volume)
        try:
            self.chat_page.set_volume(self._voice_controller.get_volume())
        except Exception:
            pass

        signals.voice_state_changed.connect(self._on_voice_state_changed)
        signals.voice_transcript_ready.connect(self._on_voice_transcript_ready)
        signals.tts_started.connect(lambda _s, _t: self.chat_page.set_speaker_speaking(True))
        signals.tts_finished.connect(lambda _s, _t: self.chat_page.set_speaker_speaking(False))

        self.idle_page.video.set_display_scale(self.emotions.display_scale)
        self.idle_page.video.set_vertical_offset(self.emotions.vertical_offset_px)
        self.idle_page.video.set_video(self.emotions.path("idle"))
        self.show_page("idle")
        # Application-level event filter captures touches/keys in child widgets.
        QApplication.instance().installEventFilter(self)
        self._session_timer = QTimer(self)
        self._session_timer.timeout.connect(self._expire_idle_session)
        self._session_timer.start(1000)
        QTimer.singleShot(800, self.ros_node.request_locations)

    def resizeEvent(self, event: Any) -> None:
        central = self.centralWidget()
        self.stack.setGeometry(central.rect())
        nav_width = min(460, max(390, central.width() - 32))
        nav_height = 64
        self.bottom_navigation.setGeometry(
            (central.width() - nav_width) // 2,
            central.height() - nav_height - 12,
            nav_width,
            nav_height,
        )
        if self.bottom_navigation.isVisible():
            self.bottom_navigation.raise_()
        super().resizeEvent(event)

    def show_page(self, page_id: str, track_history: bool = True) -> None:
        """State-based route guard; protected pages are never guest-accessible."""
        if page_id == self._active_page:
            return
        page = self.pages.get(page_id)
        if page is None:
            return
        if page_id in {"admin", "navigation"} and not self._admin_authenticated:
            self.get_logger_or_warning("Blocked unauthorized admin navigation")
            return

        if self._active_page == "chat" and page_id != "chat":
            self.chat_page.stop_speaking()
            self._voice_controller.stop_speaking()
            self._voice_controller.capture.cancel()
            self._llm_worker.cancel_current_request()

        if track_history and self._active_page not in {"", "idle"}:
            self._page_history.append(self._active_page)

        if page_id == "idle" or (
            page_id == "home" and self._active_page in {"admin", "navigation"}
        ):
            self._admin_authenticated = False

        if page_id == "idle":
            self._page_history.clear()

        self._active_page = page_id
        self.stack.setCurrentWidget(page)
        self.bottom_navigation.setVisible(page_id == "home")
        self.bottom_navigation.set_dark_background(True)
        self.bottom_navigation.set_active("home")
        if self.bottom_navigation.isVisible():
            self.bottom_navigation.raise_()
        if page_id == "idle":
            self.idle_page.video.start()
            self._update_emotion()
        else:
            self.idle_page.video.pause()
        if page_id == "navigation":
            self.ros_node.request_locations()

    def go_back(self) -> None:
        """Navigate back to the previous screen."""
        if self._page_history:
            prev_page = self._page_history.pop()
            self.show_page(prev_page, track_history=False)
        else:
            if self._active_page in {"admin", "chat", "service"}:
                self.show_page("home", track_history=False)
            elif self._active_page == "navigation":
                self.show_page("admin", track_history=False)
            else:
                self.show_page("idle", track_history=False)

    def get_logger_or_warning(self, message: str) -> None:
        # The UI is a Qt window; log through the existing ROS adapter.
        self.ros_node.get_logger().warn(message)

    def _on_bottom_navigation(self, target: str) -> None:
        if target == "settings":
            self._request_admin_login()
        elif target == "chat":
            self.show_page("chat")
        elif target == "home":
            self.show_page("home")
        elif target == "service":
            self._on_service_requested()
        elif target == "emergency":
            self._on_emergency_stop()

    def _request_admin_login(self) -> None:
        dialog = AdminPinDialog(self._admin_pin, self)
        if dialog.exec_() == AdminPinDialog.Accepted:
            self._admin_authenticated = True
            self._last_activity_at = time.monotonic()
            self.show_page("admin")
        else:
            self.bottom_navigation.set_active("home")

    def _logout_admin(self) -> None:
        self._admin_authenticated = False
        self.show_page("home")

    def _on_service_requested(self) -> None:
        self.show_page("service")

    def _on_emergency_stop(self) -> None:
        print("[robot_ui] EMERGENCY STOP TRIGGERED FROM BOTTOM DOCK!", flush=True)
        self.ros_node.trigger_emergency_stop()
        self.signals.command_status_changed.emit("error", "🚨 ĐÃ KÍCH HOẠT DỪNG KHẨN CẤP!")
        self.bottom_navigation.set_active("home" if self._active_page == "home" else self._active_page)

    def _on_admin_start_nav(self) -> None:
        self.show_page("navigation")

    def _on_admin_stop_nav(self) -> None:
        self.ros_node.cancel_navigation()
        self.admin_page.status_banner.setText("Điều hướng: Đã gửi lệnh dừng")

    def _on_admin_slam(self) -> None:
        self.admin_page.status_banner.setText("SLAM: Đang sẵn sàng khởi tạo")

    def _on_admin_detection(self) -> None:
        self.admin_page.status_banner.setText("Detection Debug: Kênh /yolo/detections đang mở")

    def _on_admin_system_status(self) -> None:
        self.admin_page.status_banner.setText("System Status: CPU, Cảm biến & Vi điều khiển bình thường")

    def _on_connection(self, connected: bool) -> None:
        self._connected = connected
        self.admin_page.set_connection(connected)
        self._update_emotion()

    def _on_robot_status(self, status: dict[str, Any]) -> None:
        self._robot_status = status
        self._robot_status_received_at = time.monotonic()
        self.idle_page.set_robot_status(status)
        self.admin_page.set_robot_status(status)
        self.navigation_page.set_robot_status(status)
        self._update_emotion()

    def _on_navigation_status(self, status: dict[str, Any]) -> None:
        self._navigation_status = status
        self.navigation_page.set_navigation_status(status)
        self.admin_page.set_navigation_status(status)
        self._update_emotion()

    def _on_chat_submitted(self, message: str) -> None:
        """Nhận tin nhắn từ ChatPage và gửi cho LLM worker."""
        self._voice_controller.notify_llm_processing()
        self.signals.llm_status_changed.emit("●  LLM: ĐANG XỬ LÝ...")
        sent = self._llm_worker.send(
            message, robot_status=self._robot_status,
            status_updated_at=self._robot_status_received_at,
        )
        if not sent:
            self.chat_page.receive_response("Robot đang xử lý một câu hỏi khác.")

    def _on_voice_state_changed(self, state_str: str, msg: str) -> None:
        try:
            self.chat_page.set_mic_state(MicState(state_str))
        except ValueError:
            pass
        if msg:
            self.chat_page.set_voice_status(msg)

    def _on_voice_transcript_ready(self, session_id: int, turn_id: int, text: str) -> None:
        if session_id != self._llm_worker.session_id:
            return
        self._voice_controller.notify_llm_processing()
        self.chat_page.submit_voice_transcript(text)

    def _on_llm_stream_delta(self, session_id: int, turn_id: int, delta: str) -> None:
        if session_id != self._llm_worker.session_id:
            return
        self.chat_page.receive_stream_delta(session_id, turn_id, delta)
        ctx = TurnContext(session_id, turn_id, 0)
        self._voice_controller.handle_stream_delta(ctx, delta)

    def _on_llm_stream_completed(self, session_id: int, turn_id: int, full_text: str) -> None:
        if session_id != self._llm_worker.session_id:
            return
        self.chat_page.finalize_stream(session_id, turn_id, full_text, interrupted=False)
        self.signals.llm_status_changed.emit("LLM: SẴN SÀNG")
        ctx = TurnContext(session_id, turn_id, 0)
        self._voice_controller.handle_stream_completed(ctx, full_text)

    def _on_llm_stream_interrupted(self, session_id: int, turn_id: int) -> None:
        self.chat_page.finalize_stream(session_id, turn_id, "", interrupted=True)
        ctx = TurnContext(session_id, turn_id, 0)
        self._voice_controller.handle_stream_interrupted(ctx)

    def _on_stop_requested(self) -> None:
        """Hủy xử lý câu hỏi LLM và dừng phát âm thanh ngay lập tức mà không làm mất lịch sử Chat."""
        if self._llm_worker.is_busy:
            self._llm_worker.cancel_current_request()
            self.chat_page.set_busy(False)
            self.signals.llm_status_changed.emit("LLM: SẴN SÀNG")
            self.chat_page.set_status("Đã dừng câu hỏi")

        self._voice_controller.stop_speaking()
        self._voice_controller.capture.cancel()
        self.chat_page.set_mic_state(MicState.READY)

    def _receive_llm_response(self, session_id: int, response: str) -> None:
        # A queued response from a prior visitor must never enter this session.
        if session_id != self._llm_worker.session_id:
            return
        self.chat_page.receive_response(response)
        self.signals.llm_status_changed.emit("LLM: SẴN SÀNG")
        # Route to TTS only if response is valid text (not a technical error)
        if not response.startswith("⚠️"):
            self._voice_controller.handle_llm_response(session_id, response)

    def eventFilter(self, watched: Any, event: QEvent) -> bool:
        input_events = (
            QEvent.MouseButtonPress, QEvent.KeyPress,
            QEvent.TouchBegin, QEvent.Wheel,
        )
        if event.type() in input_events:
            self._last_activity_at = time.monotonic()
            # Only consume taps within this kiosk (not other Qt windows).
            if self._active_page == "idle" and event.type() in (
                QEvent.MouseButtonPress, QEvent.TouchBegin,
            ):
                if isinstance(watched, QWidget) and (
                    watched is self or self.isAncestorOf(watched)
                ):
                    self.show_page("home")
                    return True
        return super().eventFilter(watched, event)

    def _expire_idle_session(self) -> None:
        if self._active_page == "admin":
            self.admin_page.update_wifi()
        if self._active_page == "idle":
            return
        # Do not expire session while visitor is actively recording, waiting for STT/LLM/TTS, or robot is speaking
        if self._voice_controller.is_active or self._llm_worker.is_busy:
            self._last_activity_at = time.monotonic()
            return
        timeout = (
            self._admin_idle_timeout_seconds
            if self._active_page in {"admin", "navigation"}
            else self._idle_timeout_seconds
        )
        if time.monotonic() - self._last_activity_at < timeout:
            return
        self._last_activity_at = time.monotonic()
        self._admin_authenticated = False
        self._llm_worker.reset_session()
        self._voice_controller.reset_session()
        self.chat_page.reset_conversation()
        self.signals.llm_status_changed.emit("LLM: ĐÃ LÀM MỚI PHIÊN")
        # UI timeout only: never cancel an ongoing robot mission/navigation.
        self.show_page("idle")

    def _update_emotion(self) -> None:
        if self._active_page != "idle":
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
        self.idle_page.video.set_video(self.emotions.path(emotion))

    def keyPressEvent(self, event: QKeyEvent) -> None:
        if event.key() == Qt.Key_Escape:
            self.close()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event: Any) -> None:
        QApplication.instance().removeEventFilter(self)
        self.chat_page.stop_speaking()
        self._voice_controller.shutdown()
        self._llm_worker.reset_session()
        self.idle_page.video.stop()
        super().closeEvent(event)
