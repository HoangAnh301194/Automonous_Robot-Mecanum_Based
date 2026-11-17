"""AI chatbot dialogue interface with speech-to-text, LLM streaming, and TTS controls."""
from __future__ import annotations

from enum import Enum
from pathlib import Path
import time

from PyQt5.QtCore import QPoint, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QIcon
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from robot_ui.kiosk.pages.base import monochrome_icon, strip_emojis


class MicState(Enum):
    READY = "ready"
    LISTENING = "listening"
    UNAVAILABLE = "unavailable"
    STT_PROCESSING = "stt_processing"
    LLM_PROCESSING = "llm_processing"
    TTS_PROCESSING = "tts_processing"
    SPEAKING = "speaking"
    ERROR = "error"


class SpeakerButton(QPushButton):
    """Nút loa hỗ trợ: Bấm đơn = Mute/Unmute; Bấm 2 lần = Mở thanh âm lượng; Bấm giữ + Kéo = Trượt âm lượng."""

    single_clicked = pyqtSignal()
    double_clicked = pyqtSignal()
    long_press_started = pyqtSignal()
    drag_moved = pyqtSignal(int, int)  # (global_x, global_y)
    drag_finished = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._press_pos: QPoint | None = None
        self._is_long_press = False
        self._drag_active = False
        self._long_timer = QTimer(self)
        self._long_timer.setSingleShot(True)
        self._long_timer.setInterval(250)
        self._long_timer.timeout.connect(self._on_long_press_timeout)

        self._from_mouse = False
        self._pending_single_click = False
        self._last_click_time = 0.0
        self._last_double_click_time = 0.0
        self._click_timer = QTimer(self)
        self._click_timer.setSingleShot(True)
        self._click_timer.setInterval(320)
        self._click_timer.timeout.connect(self._on_click_timeout)
        self.clicked.connect(self._handle_clicked)

    def _on_long_press_timeout(self) -> None:
        self._is_long_press = True
        self._drag_active = True
        self._pending_single_click = False
        self._click_timer.stop()
        self.long_press_started.emit()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            self._from_mouse = True
            self._press_pos = event.globalPos()
            self._is_long_press = False
            self._drag_active = False
            self._long_timer.start()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._drag_active:
            gpos = event.globalPos()
            self.drag_moved.emit(gpos.x(), gpos.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        self._long_timer.stop()
        if self._drag_active or self._is_long_press:
            self._drag_active = False
            self._pending_single_click = False
            self._click_timer.stop()
            self.drag_finished.emit()
            self.setDown(False)
            event.accept()
            self._from_mouse = False
            return
        super().mouseReleaseEvent(event)
        self._from_mouse = False

    def mouseDoubleClickEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            now = time.monotonic()
            self._long_timer.stop()
            self._drag_active = False
            self._pending_single_click = False
            self._click_timer.stop()
            self._last_double_click_time = now
            self.double_clicked.emit()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def _handle_clicked(self) -> None:
        if not self._from_mouse:
            self.single_clicked.emit()
            return
        now = time.monotonic()
        if self._is_long_press or self._drag_active:
            return
        if now - self._last_double_click_time < 0.4:
            return
        self._pending_single_click = True
        self._last_click_time = now
        self._click_timer.start()

    def _on_click_timeout(self) -> None:
        if self._pending_single_click:
            self._pending_single_click = False
            self.single_clicked.emit()


class VolumeSliderWidget(QFrame):
    """Thanh trượt âm lượng pop-up ngắn gọn nằm cạnh nút loa."""

    volume_changed = pyqtSignal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("volumeSliderWidget")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setFixedHeight(44)
        self.setMinimumWidth(160)
        self.setMaximumWidth(200)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 4, 10, 4)
        layout.setSpacing(8)

        self._label = QLabel("80%")
        self._label.setObjectName("volumeLabel")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setFixedWidth(36)

        self._slider = QSlider(Qt.Horizontal)
        self._slider.setObjectName("volumeSlider")
        self._slider.setRange(0, 100)
        self._slider.setValue(80)
        self._slider.setCursor(Qt.PointingHandCursor)
        self._slider.valueChanged.connect(self._on_slider_value_changed)

        layout.addWidget(self._label)
        layout.addWidget(self._slider, 1)

        self._hide_timer = QTimer(self)
        self._hide_timer.setSingleShot(True)
        self._hide_timer.setInterval(3500)
        self._hide_timer.timeout.connect(self.hide)

        self.setStyleSheet("""
            QFrame#volumeSliderWidget {
                background: #f3f6fa;
                border: 1px solid #383838;
                border-radius: 12px;
            }
            QLabel#volumeLabel {
                color: #b32632;
                font-size: 12px;
                font-weight: 700;
            }
            QSlider#volumeSlider::groove:horizontal {
                height: 8px;
                background: #e8eef4;
                border-radius: 4px;
            }
            QSlider#volumeSlider::sub-page:horizontal {
                background: #ffb32c;
                border-radius: 4px;
            }
            QSlider#volumeSlider::handle:horizontal {
                background: #ffffff;
                border: 2px solid #ffb32c;
                width: 22px;
                height: 22px;
                margin: -7px 0;
                border-radius: 11px;
            }
        """)

    @property
    def value(self) -> int:
        return self._slider.value()

    def set_volume(self, value: int, notify: bool = True) -> None:
        val = max(0, min(100, int(value)))
        self._label.setText(f"{val}%")
        self._slider.blockSignals(not notify)
        self._slider.setValue(val)
        self._slider.blockSignals(False)
        if notify:
            self.volume_changed.emit(val)

    def show_slider(self) -> None:
        self.show()
        self.raise_()
        self._hide_timer.stop()

    def toggle_slider(self) -> None:
        """Bật/tắt hiển thị thanh trượt âm lượng (dùng khi double-click nút loa)."""
        if self.isVisible():
            self._hide_timer.stop()
            self.hide()
        else:
            self.show_slider()
            self.start_hide_timer()

    def start_hide_timer(self) -> None:
        self._hide_timer.start(3500)

    def update_from_global_point(self, gx: int, gy: int) -> None:
        self.show_slider()
        pt_in_slider = self._slider.mapFromGlobal(QPoint(gx, gy))
        w = self._slider.width()
        if w > 0:
            ratio = max(0.0, min(1.0, pt_in_slider.x() / float(w)))
            val = int(round(ratio * 100))
            self.set_volume(val, notify=True)

    def _on_slider_value_changed(self, value: int) -> None:
        self._label.setText(f"{value}%")
        self.volume_changed.emit(value)
        self._hide_timer.start(3500)


class ChatPage(QWidget):
    chat_submitted = pyqtSignal(str)  # emitted khi user nhấn Gửi
    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    mic_clicked = pyqtSignal()
    speaker_toggled = pyqtSignal(bool)
    stop_speaking_requested = pyqtSignal()
    volume_changed = pyqtSignal(int)

    DEFAULT_SUGGESTIONS = (
        "Giới thiệu về PTIT",
        "Giới thiệu về Robot",
        "Giới thiệu về Khoa Kỹ thuật Điện tử 1",
        "Giới thiệu về nhóm nghiên cứu OpenWorkShop",
    )

    def __init__(
        self,
        chat_icon: Path,
        parent: QWidget | None = None,
        back_icon: Path | None = None,
        home_icon: Path | None = None,
        emergency_icon: Path | None = None,
        voice_icons: dict[str, Path] | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("chatPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self._voice_icons = voice_icons or {}
        self._speaker_enabled = True
        self._is_speaking = False
        self._mic_state = MicState.UNAVAILABLE
        self._typing_bubble: QWidget | None = None
        self._typing_dots = 1
        self._typing_timer = QTimer(self)
        self._typing_timer.timeout.connect(self._update_typing_dots)
        self._suggestions_widget: QWidget | None = None

        # SSE Streaming Bubble support with coalesced throttling
        self._current_streaming_row: QWidget | None = None
        self._current_streaming_body: QLabel | None = None
        self._current_streaming_text: str = ""
        self._streaming_turn_id: int | None = None
        self._stream_pending_text: str = ""
        self._stream_throttle_timer = QTimer(self)
        self._stream_throttle_timer.setInterval(40)
        self._stream_throttle_timer.timeout.connect(self._on_stream_throttle_timeout)

        self._pulse_timer = QTimer(self)
        self._pulse_timer.timeout.connect(self._animate_listening_pulse)
        self._pulse_state = False

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 12, 18, 14)
        root.setSpacing(10)

        # A. Header (64-72px)
        header = QFrame()
        header.setObjectName("chatHeader")
        header.setMinimumHeight(64)
        header.setMaximumHeight(72)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(8)

        # 1. Nút Home (Về thẳng màn hình chính)
        home = QPushButton()
        home.setObjectName("navHomeButton")
        home.setToolTip("Về màn hình chính")
        home.setAccessibleName("Về màn hình chính")
        home.setFixedSize(46, 46)
        if home_icon and Path(home_icon).is_file():
            home.setIcon(QIcon(str(home_icon)))
            home.setIconSize(QSize(28, 28))
        else:
            home.setText("🏠")
            home.setStyleSheet("font-size: 22px;")
        home.clicked.connect(self.home_requested.emit)
        header_layout.addWidget(home)

        # 2. Nút Dừng khẩn cấp
        emergency = QPushButton()
        emergency.setObjectName("navEmergencyButton")
        emergency.setToolTip("DỪNG KHẨN CẤP")
        emergency.setAccessibleName("Dừng khẩn cấp")
        emergency.setFixedSize(46, 46)
        if emergency_icon and Path(emergency_icon).is_file():
            emergency.setIcon(QIcon(str(emergency_icon)))
            emergency.setIconSize(QSize(36, 36))
        else:
            emergency.setText("🛑")
            emergency.setStyleSheet("font-size: 24px;")
        emergency.clicked.connect(self.emergency_stop_requested.emit)
        header_layout.addWidget(emergency)

        # 3. Nút Quay lại (Back)
        back = QPushButton()
        back.setObjectName("navBackButton")
        back.setToolTip("Quay lại")
        back.setAccessibleName("Quay lại")
        back.setFixedSize(46, 46)
        if back_icon and Path(back_icon).is_file():
            back.setIcon(QIcon(str(back_icon)))
            back.setIconSize(QSize(28, 28))
        else:
            back.setText("‹")
            back.setStyleSheet("color: #b32632; font-size: 29px; font-weight: 700;")
        back.clicked.connect(self.back_requested.emit)
        header_layout.addWidget(back)

        icon_label = QLabel()
        icon_label.setObjectName("chatHeaderIcon")
        icon_label.setPixmap(
            monochrome_icon(chat_icon, QColor("#1E3A8A")).pixmap(QSize(26, 26))
        )
        icon_label.setFixedSize(36, 36)
        icon_label.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(icon_label)

        heading = QVBoxLayout()
        heading.setSpacing(2)
        title = QLabel("Souta Assistant")
        title.setObjectName("chatTitle")
        self._status_label = QLabel("Sẵn sàng trò chuyện")
        self._status_label.setObjectName("chatSubtitle")
        heading.addWidget(title)
        heading.addWidget(self._status_label)
        header_layout.addLayout(heading)

        header_layout.addStretch(1)

        # LLM Status badge (compact & basic in top-right)
        self._llm_status_label = QLabel("● LLM")
        self._llm_status_label.setObjectName("llmStatus")
        self._llm_status_label.setToolTip("Trạng thái AI LLM")
        header_layout.addWidget(self._llm_status_label)

        # Stop speaking button (hidden unless speaking)
        self._stop_speaking_btn = QPushButton("Dừng nói")
        self._stop_speaking_btn.setObjectName("chatStopSpeakingButton")
        self._stop_speaking_btn.setMinimumHeight(44)
        self._stop_speaking_btn.setIcon(self._get_icon("stop_speaking"))
        self._stop_speaking_btn.setIconSize(QSize(22, 22))
        self._stop_speaking_btn.setCursor(Qt.PointingHandCursor)
        self._stop_speaking_btn.hide()
        self._stop_speaking_btn.clicked.connect(self._on_stop_speaking_clicked)
        header_layout.addWidget(self._stop_speaking_btn)

        # Volume slider popup/bar (hidden by default)
        self._volume_widget = VolumeSliderWidget()
        self._volume_widget.hide()
        self._volume_widget.volume_changed.connect(self._on_volume_widget_changed)
        header_layout.addWidget(self._volume_widget)

        # Speaker toggle button (min 48x48)
        self._speaker_btn = SpeakerButton()
        self._speaker_btn.setObjectName("chatSpeakerButton")
        self._speaker_btn.setFixedSize(48, 48)
        self._speaker_btn.setIconSize(QSize(26, 26))
        self._speaker_btn.setCursor(Qt.PointingHandCursor)
        self._speaker_btn.single_clicked.connect(self._toggle_speaker)
        self._speaker_btn.double_clicked.connect(self._on_speaker_double_clicked)
        self._speaker_btn.long_press_started.connect(self._on_speaker_long_pressed)
        self._speaker_btn.drag_moved.connect(self._on_speaker_drag_moved)
        self._speaker_btn.drag_finished.connect(self._on_speaker_drag_finished)
        header_layout.addWidget(self._speaker_btn)
        self._update_speaker_ui()

        root.addWidget(header)

        # B. Conversation Area
        conversation = QFrame()
        conversation.setObjectName("chatConversation")
        conversation_layout = QVBoxLayout(conversation)
        conversation_layout.setContentsMargins(6, 6, 6, 6)

        self.messages = QScrollArea()
        self.messages.setObjectName("chatScroll")
        self.messages.setWidgetResizable(True)
        self.messages.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.messages.setFrameShape(QFrame.NoFrame)

        self._message_container = QWidget()
        self._message_container.setObjectName("chatMessageContainer")
        self._messages_layout = QVBoxLayout(self._message_container)
        self._messages_layout.setContentsMargins(10, 10, 10, 10)
        self._messages_layout.setSpacing(10)
        self._messages_layout.addStretch(1)
        self.messages.setWidget(self._message_container)

        self.messages.verticalScrollBar().rangeChanged.connect(
            lambda _min, _max: self.messages.verticalScrollBar().setValue(_max)
        )
        conversation_layout.addWidget(self.messages)
        root.addWidget(conversation, 1)

        # C1. Centered Microphone Button (enlarged 3x: icon 84x84, circular button 100x100)
        mic_row = QHBoxLayout()
        mic_row.setContentsMargins(0, 2, 0, 2)
        mic_row.addStretch(1)

        self.mic_btn = QPushButton()
        self.mic_btn.setObjectName("chatMicButton")
        self.mic_btn.setFixedSize(100, 100)
        self.mic_btn.setIconSize(QSize(84, 84))
        self.mic_btn.setCursor(Qt.PointingHandCursor)
        self.mic_btn.clicked.connect(self._on_mic_clicked)
        mic_row.addWidget(self.mic_btn)

        mic_row.addStretch(1)
        root.addLayout(mic_row)

        # C2. Input Bar (64-74px)
        input_bar = QFrame()
        input_bar.setObjectName("chatInputBar")
        input_bar.setMinimumHeight(64)
        input_bar.setMaximumHeight(74)
        input_row = QHBoxLayout(input_bar)
        input_row.setContentsMargins(10, 8, 10, 8)
        input_row.setSpacing(10)

        self.input = QLineEdit()
        self.input.setObjectName("chatInput")
        self.input.setPlaceholderText("Nhập câu hỏi của bạn...")
        self.input.setMinimumHeight(48)
        self.input.returnPressed.connect(self._on_send)
        input_row.addWidget(self.input, 1)

        self._send_btn = QPushButton("Gửi")
        self._send_btn.setObjectName("chatSendButton")
        self._send_btn.setFixedSize(76, 48)
        self._send_btn.setCursor(Qt.PointingHandCursor)
        self._send_btn.clicked.connect(self._on_send)
        input_row.addWidget(self._send_btn)

        root.addWidget(input_bar)

        # Initial state setup
        self.set_mic_state(MicState.UNAVAILABLE)
        self._add_message("Souta Robot", "Xin chào! Tôi là Souta Robot. Tôi có thể giúp gì cho bạn?")
        self._add_suggestions()

    def _get_icon(self, name: str) -> QIcon:
        path = self._voice_icons.get(name)
        if path is not None and Path(path).is_file():
            return QIcon(str(path))
        try:
            from robot_ui.kiosk.assets import icon_path
            p = icon_path(f"{name}.png")
            if p.is_file():
                return QIcon(str(p))
        except Exception:
            pass
        return QIcon()

    def _add_message(self, sender: str, message: str, from_user: bool = False) -> None:
        display_message = message if from_user else (strip_emojis(message) or message)
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("userBubble" if from_user else "robotBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel(sender)
        sender_label.setObjectName("userBubbleSender" if from_user else "robotBubbleSender")
        body = QLabel(display_message)
        body.setObjectName("bubbleText")
        body.setWordWrap(True)
        body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(body)

        if from_user:
            row_layout.addStretch(1)
            row_layout.addWidget(bubble)
        else:
            row_layout.addWidget(bubble)
            row_layout.addStretch(1)

        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)

    def _add_suggestions(self) -> None:
        self._hide_suggestions()
        container = QFrame()
        container.setObjectName("suggestionContainer")
        grid = QGridLayout(container)
        grid.setContentsMargins(4, 4, 4, 4)
        grid.setSpacing(8)
        self._suggestion_buttons: list[QPushButton] = []
        for i, text in enumerate(self.DEFAULT_SUGGESTIONS):
            btn = QPushButton(text)
            btn.setObjectName("suggestionChip")
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked=False, query=text: self._on_suggestion_clicked(query))
            grid.addWidget(btn, i // 2, i % 2)
            self._suggestion_buttons.append(btn)
        self._suggestions_widget = container
        self._messages_layout.insertWidget(self._messages_layout.count() - 1, container)

    def _hide_suggestions(self) -> None:
        if self._suggestions_widget is not None:
            self._suggestions_widget.hide()

    def _on_suggestion_clicked(self, prompt: str) -> None:
        self._hide_suggestions()
        self.input.setText(prompt)
        self._on_send()

    def _show_typing_indicator(self) -> None:
        if self._typing_bubble is not None:
            return
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("typingBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel("Souta Robot")
        sender_label.setObjectName("robotBubbleSender")
        self._typing_label = QLabel("Thinking.")
        self._typing_label.setObjectName("typingText")
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(self._typing_label)

        row_layout.addWidget(bubble)
        row_layout.addStretch(1)

        self._typing_bubble = row
        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)
        self._typing_dots = 1
        self._typing_timer.start(450)

    def _update_typing_dots(self) -> None:
        if self._typing_bubble is not None and hasattr(self, "_typing_label"):
            self._typing_dots = (self._typing_dots % 3) + 1
            dots = "." * self._typing_dots
            self._typing_label.setText(f"Thinking{dots}")

    def _remove_typing_indicator(self) -> None:
        self._typing_timer.stop()
        if self._typing_bubble is not None:
            self._messages_layout.removeWidget(self._typing_bubble)
            self._typing_bubble.deleteLater()
            self._typing_bubble = None

    def _on_send(self) -> None:
        text = self.input.text().strip()
        if not text or not self.input.isEnabled():
            return
        self._hide_suggestions()
        self.input.clear()
        self._add_message("Bạn", text, from_user=True)
        self.set_busy(True)
        self.chat_submitted.emit(text)

    def _animate_listening_pulse(self) -> None:
        self._pulse_state = not self._pulse_state
        border = "#ff4d4d" if self._pulse_state else "#ffb32c"
        self.mic_btn.setStyleSheet(
            f"#chatMicButton {{ border: 3px solid {border}; border-radius: 50px; background-color: rgba(255, 77, 77, 0.25); }}"
        )

    def submit_voice_transcript(self, text: str) -> None:
        """Submit voice transcript as user message and trigger LLM processing."""
        text = text.strip()
        if not text or not self.input.isEnabled():
            return
        self._hide_suggestions()
        self.input.clear()
        self._add_message("Bạn", text, from_user=True)
        self.set_busy(True)
        self.chat_submitted.emit(text)

    # ------------------------------------------------------------------
    # Voice Chat & Speaker State Management
    # ------------------------------------------------------------------

    def set_mic_state(self, state: MicState) -> None:
        self._mic_state = state
        if state != MicState.LISTENING:
            self._pulse_timer.stop()
            self.mic_btn.setStyleSheet("")

        if state not in (MicState.LLM_PROCESSING, MicState.TTS_PROCESSING, MicState.SPEAKING):
            self._stop_speaking_btn.hide()

        if state == MicState.READY:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(True)
            self.mic_btn.setToolTip("Bắt đầu nói")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Sẵn sàng trò chuyện")
        elif state == MicState.LISTENING:
            self.mic_btn.setIcon(self._get_icon("mic_listening"))
            self.mic_btn.setEnabled(True)
            self.mic_btn.setToolTip("Đang ghi âm (chạm để dừng)")
            self.mic_btn.setProperty("listening", "true")
            self._pulse_timer.start(450)
            self.set_status("Đang nghe...")
        elif state == MicState.UNAVAILABLE:
            self.mic_btn.setIcon(self._get_icon("mic_off"))
            self.mic_btn.setEnabled(True)  # click shows friendly message
            self.mic_btn.setToolTip("Trò chuyện bằng giọng nói chưa khả dụng")
            self.mic_btn.setProperty("listening", "false")
        elif state == MicState.STT_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang nhận dạng giọng nói...")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Đang nhận dạng giọng nói...")
        elif state == MicState.LLM_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang xử lý câu hỏi...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng phản hồi")
            self._stop_speaking_btn.setToolTip("Dừng câu hỏi đang xử lý")
            self._stop_speaking_btn.show()
            self.set_status("Thinking...")
        elif state == MicState.TTS_PROCESSING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Đang tạo giọng nói...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng")
            self._stop_speaking_btn.setToolTip("Hủy tạo âm thanh")
            self._stop_speaking_btn.show()
            self.set_status("Đang tạo giọng nói...")
        elif state == MicState.SPEAKING:
            self.mic_btn.setIcon(self._get_icon("mic"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Souta đang nói...")
            self.mic_btn.setProperty("listening", "false")
            self._stop_speaking_btn.setText("Dừng nói")
            self._stop_speaking_btn.setToolTip("Dừng phát âm thanh")
            self._stop_speaking_btn.show()
            self.set_status("Souta đang nói...")
        elif state == MicState.ERROR:
            self.mic_btn.setIcon(self._get_icon("mic_off"))
            self.mic_btn.setEnabled(False)
            self.mic_btn.setToolTip("Lỗi microphone")
            self.mic_btn.setProperty("listening", "false")
            self.set_status("Lỗi microphone")
        self.mic_btn.style().unpolish(self.mic_btn)
        self.mic_btn.style().polish(self.mic_btn)

    def set_speaker_enabled(self, enabled: bool) -> None:
        self._speaker_enabled = enabled
        self._update_speaker_ui()

    def set_voice_status(self, message: str) -> None:
        self.set_status(message)

    def _on_mic_clicked(self) -> None:
        self.mic_clicked.emit()
        if self._mic_state == MicState.UNAVAILABLE:
            self.set_status("Trò chuyện bằng giọng nói chưa khả dụng")
            QTimer.singleShot(
                2500,
                lambda: self.set_status("Sẵn sàng trò chuyện")
                if self._mic_state == MicState.UNAVAILABLE and not self._is_speaking
                else None,
            )

    def _on_speaker_double_clicked(self) -> None:
        self._volume_widget.toggle_slider()

    def _on_speaker_long_pressed(self) -> None:
        self._volume_widget.show_slider()

    def _on_speaker_drag_moved(self, gx: int, gy: int) -> None:
        self._volume_widget.update_from_global_point(gx, gy)

    def _on_speaker_drag_finished(self) -> None:
        self._volume_widget.start_hide_timer()

    def _on_volume_widget_changed(self, val: int) -> None:
        if val == 0 and self._speaker_enabled:
            self._speaker_enabled = False
            self._update_speaker_ui()
            self.speaker_toggled.emit(False)
        elif val > 0 and not self._speaker_enabled:
            self._speaker_enabled = True
            self._update_speaker_ui()
            self.speaker_toggled.emit(True)
        self.volume_changed.emit(val)

    def set_volume(self, val: int) -> None:
        self._volume_widget.set_volume(val, notify=False)

    def _toggle_speaker(self) -> None:
        self._speaker_enabled = not self._speaker_enabled
        if self._speaker_enabled and self._volume_widget.value == 0:
            self._volume_widget.set_volume(80, notify=True)
        self._update_speaker_ui()
        if not self._speaker_enabled and self._is_speaking:
            self.stop_speaking()
        self.speaker_toggled.emit(self._speaker_enabled)

    def _update_speaker_ui(self) -> None:
        icon_name = "speaker_on" if self._speaker_enabled else "speaker_off"
        self._speaker_btn.setIcon(self._get_icon(icon_name))
        tooltip = (
            "Bật âm thanh (Chạm 2 lần để chỉnh âm lượng)"
            if not self._speaker_enabled
            else "Tắt âm thanh (Chạm 2 lần để chỉnh âm lượng)"
        )
        self._speaker_btn.setToolTip(tooltip)

    def set_speaker_speaking(self, is_speaking: bool) -> None:
        self._is_speaking = is_speaking
        if is_speaking:
            self._stop_speaking_btn.setText("Dừng nói")
            self._stop_speaking_btn.show()
            self.set_status("Đang nói...")
        else:
            if self._mic_state not in (MicState.LLM_PROCESSING, MicState.TTS_PROCESSING):
                self._stop_speaking_btn.hide()
            if self._mic_state != MicState.LISTENING and self.input.isEnabled():
                self.set_status("Sẵn sàng trò chuyện")

    def _on_stop_speaking_clicked(self) -> None:
        self.stop_speaking()

    def stop_speaking(self) -> None:
        self.set_speaker_speaking(False)
        self.stop_speaking_requested.emit()

    def set_status(self, status: str) -> None:
        self._status_label.setText(status)

    # ------------------------------------------------------------------
    # Public API – được gọi từ KioskWindow để wire LLM
    # ------------------------------------------------------------------

    def reset_conversation(self) -> None:
        self.stop_speaking()
        self._stream_throttle_timer.stop()
        self._remove_typing_indicator()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._current_streaming_text = ""
        self._stream_pending_text = ""
        self._streaming_turn_id = None
        self._last_finalized_text = None
        while self._messages_layout.count() > 1:
            item = self._messages_layout.takeAt(0)
            if item.widget() is not None:
                item.widget().deleteLater()
        self._suggestions_widget = None
        self.set_busy(False)
        self.input.clear()
        self.set_mic_state(MicState.UNAVAILABLE)
        self.set_status("Sẵn sàng trò chuyện")
        self._llm_status_label.setText("● LLM")
        self._llm_status_label.setProperty("state", "ready")
        self._add_message("Souta Robot", "Xin chào! Tôi là Souta Robot. Tôi có thể giúp gì cho bạn?")
        self._add_suggestions()

    def set_llm_status(self, status: str) -> None:
        """Ghi log kỹ thuật và cập nhật UI thân thiện cho khách."""
        print(f"[robot_ui] LLM status: {status}", flush=True)
        self._llm_status_label.setToolTip(f"Trạng thái: {status}")
        if "ĐANG XỬ LÝ" in status:
            self._llm_status_label.setText("● LLM...")
            self._llm_status_label.setProperty("state", "busy")
            self.set_status("Thinking...")
            self.set_mic_state(MicState.LLM_PROCESSING)
        elif "LỖI" in status or "⚠️" in status:
            self._llm_status_label.setText("⚠ LLM")
            self._llm_status_label.setProperty("state", "error")
            self.set_status(status)
        elif "CHƯA KẾT NỐI" in status or "OFFLINE" in status:
            self._llm_status_label.setText("○ LLM")
            self._llm_status_label.setProperty("state", "offline")
            if not self._is_speaking and self._mic_state != MicState.LISTENING:
                self.set_status("Chưa kết nối LLM")
        else:
            self._llm_status_label.setText("● LLM")
            self._llm_status_label.setProperty("state", "ready")
            if not self._is_speaking and self._mic_state != MicState.LISTENING:
                self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state == MicState.LLM_PROCESSING:
                self.set_mic_state(MicState.READY)
        self._llm_status_label.style().unpolish(self._llm_status_label)
        self._llm_status_label.style().polish(self._llm_status_label)

    def _create_streaming_bubble(self, sender: str) -> tuple[QWidget, QLabel]:
        row = QWidget()
        row.setObjectName("chatMessageRow")
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.setSpacing(0)

        bubble = QFrame()
        bubble.setObjectName("robotBubble")
        bubble.setMaximumWidth(580)
        bubble_layout = QVBoxLayout(bubble)
        bubble_layout.setContentsMargins(14, 10, 14, 10)
        bubble_layout.setSpacing(4)

        sender_label = QLabel(sender)
        sender_label.setObjectName("robotBubbleSender")
        body = QLabel("...")
        body.setObjectName("bubbleText")
        body.setWordWrap(True)
        body.setTextInteractionFlags(Qt.TextSelectableByMouse)
        bubble_layout.addWidget(sender_label)
        bubble_layout.addWidget(body)

        row_layout.addWidget(bubble)
        row_layout.addStretch(1)

        self._messages_layout.insertWidget(self._messages_layout.count() - 1, row)
        return row, body

    def receive_stream_delta(self, session_id: int, turn_id: int, delta: str) -> None:
        """Receive incremental SSE token from LLM, displaying smoothly with coalesce throttling."""
        if self._current_streaming_row is None or self._streaming_turn_id != turn_id:
            self._remove_typing_indicator()
            row, body = self._create_streaming_bubble("Souta Robot")
            self._current_streaming_row = row
            self._current_streaming_body = body
            self._current_streaming_text = ""
            self._stream_pending_text = ""
            self._streaming_turn_id = turn_id
            self._stop_speaking_btn.setText("Dừng phản hồi")
            self._stop_speaking_btn.show()

        self._stream_pending_text += delta
        if not self._stream_throttle_timer.isActive():
            self._stream_throttle_timer.start(40)

    def _on_stream_throttle_timeout(self) -> None:
        self._stream_throttle_timer.stop()
        if self._current_streaming_body is None:
            return
        self._current_streaming_text += self._stream_pending_text
        self._stream_pending_text = ""
        display = strip_emojis(self._current_streaming_text) or self._current_streaming_text
        self._current_streaming_body.setText(display)
        self.messages.verticalScrollBar().setValue(self.messages.verticalScrollBar().maximum())

    def finalize_stream(self, session_id: int, turn_id: int, full_text: str, interrupted: bool = False) -> None:
        """Finalize streamed message bubble on stream complete or user stop."""
        self._stream_throttle_timer.stop()
        if self._stream_pending_text:
            self._current_streaming_text += self._stream_pending_text
            self._stream_pending_text = ""
        self._remove_typing_indicator()

        final_content = full_text.strip() if full_text.strip() else self._current_streaming_text.strip()
        if interrupted and final_content:
            final_content += " (Đã dừng)"

        if self._current_streaming_body is not None:
            display = strip_emojis(final_content) or final_content
            self._current_streaming_body.setText(display)
        elif final_content:
            display = strip_emojis(final_content) or final_content
            self._add_message("Souta Robot", display)

        self._last_finalized_text = final_content.strip()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._current_streaming_text = ""
        self._streaming_turn_id = None
        self.set_busy(False)

    def receive_response(self, response: str) -> None:
        """Hiển thị response từ LLM và mở khóa input."""
        if self._current_streaming_body is not None:
            self.finalize_stream(0, 0, response, interrupted=False)
            return
        last_finalized = getattr(self, "_last_finalized_text", None)
        if last_finalized and last_finalized == response.strip():
            self._last_finalized_text = None
            self.set_busy(False)
            return
        self.set_busy(False)
        clean_text = strip_emojis(response)
        self._add_message("Souta Robot", clean_text or response)

    def receive_error(self, error_msg: str) -> None:
        """Hiển thị lỗi và mở khóa input."""
        self._stream_throttle_timer.stop()
        self._current_streaming_row = None
        self._current_streaming_body = None
        self._streaming_turn_id = None
        self._last_finalized_text = None
        self.set_busy(False)
        clean_msg = strip_emojis(error_msg) or error_msg
        self._add_message("Souta Robot", f"Lỗi: {clean_msg}")
        self.set_status(f"Lỗi: {clean_msg}")

    def set_busy(self, busy: bool) -> None:
        """Khoá/mở input trong khi đang chờ LLM."""
        self.input.setEnabled(not busy)
        self._send_btn.setEnabled(not busy)
        if busy:
            self._send_btn.setText("...")
            self.set_status("Thinking...")
            self._show_typing_indicator()
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(False)
        else:
            self._send_btn.setText("Gửi")
            self._remove_typing_indicator()
            self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(True)
            self.set_status("Sẵn sàng trò chuyện")
            if self._mic_state in (MicState.READY, MicState.UNAVAILABLE):
                self.mic_btn.setEnabled(True)
