"""Smoke tests for the isolated Qt kiosk interface widgets.

Run after sourcing the ROS 2 workspace:
QT_QPA_PLATFORM=offscreen python3 -m pytest src/robot_ui/test/test_kiosk_interface.py -q
"""
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtCore import QEvent, Qt
from PyQt5.QtGui import QKeyEvent
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QLineEdit, QPushButton

from robot_ui.kiosk.pages import AdminPage, AdminPinDialog, ServicePage, WelcomePage
from robot_ui.kiosk.widgets import BottomNavigation


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_pin_accepts_demo_code(app):
    dialog = AdminPinDialog("1")
    dialog.entry.setText("1")
    dialog._check()
    assert dialog.result() == QDialog.Accepted


def test_pin_rejects_wrong_code(app):
    dialog = AdminPinDialog("1")
    dialog.entry.setText("2")
    dialog._check()
    assert dialog.result() != QDialog.Accepted
    assert dialog.error.text()


def test_welcome_greeting_and_dock_chat_action(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = WelcomePage(missing, missing, missing)
    assert page.findChild(QPushButton, "welcomeChatButton") is None
    texts = [label.text() for label in page.findChildren(QLabel)]
    assert any("20-11" in text for text in texts)
    assert any("Khoa Kỹ thuật Điện tử 1" in text for text in texts)
    assert not any("Souta Robot" in text for text in texts)

    dock = BottomNavigation({"home": missing, "chat": missing, "settings": missing})
    seen = []
    dock.page_requested.connect(seen.append)
    dock._buttons["chat"].click()
    assert seen == ["chat"]


def test_admin_nav_and_back_actions(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = AdminPage(missing, missing)
    seen = []
    page.navigation_requested.connect(lambda: seen.append("navigation"))
    page.back_requested.connect(lambda: seen.append("back"))
    nav = page.findChild(QPushButton, "adminNavButton")
    assert nav is not None
    nav.click()
    back = page.findChild(QPushButton, "adminBackButton")
    assert back is not None
    back.click()
    assert seen == ["navigation", "back"]


def test_dock_service_and_emergency_actions(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    dock = BottomNavigation({
        "home": missing,
        "chat": missing,
        "service": missing,
        "emergency": missing,
        "settings": missing,
    })
    seen = []
    dock.page_requested.connect(seen.append)
    dock._buttons["service"].click()
    dock._buttons["emergency"].click()
    assert seen == ["service", "emergency"]


def test_admin_pin_keypad_sizes_synchronized(app):
    dialog = AdminPinDialog("1234")
    buttons = [btn for btn in dialog.findChildren(QPushButton) if btn.text() in {"1", "2", "0", "Xóa", "OK"}]
    assert len(buttons) >= 5
    first_size = buttons[0].size()
    for btn in buttons:
        assert btn.width() == 72
        assert btn.height() == 60


def test_admin_page_grid_and_indicators(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = AdminPage(
        nav_icon=missing,
        slam_icon=missing,
        detection_icon=missing,
        start_nav_icon=missing,
        stop_nav_icon=missing,
        status_icon=missing,
    )
    # Check all 6 function buttons
    for name in (
        "adminDetectionButton",
        "adminSlamButton",
        "adminNavButton",
        "adminStartNavButton",
        "adminStopNavButton",
        "adminSystemStatusButton",
    ):
        btn = page.findChild(QPushButton, name)
        assert btn is not None

    # Check wifi and battery indicators
    page.set_wifi(True)
    assert page.wifi_text_label.text() == "Online"
    page.set_wifi(False)
    assert page.wifi_text_label.text() == "Offline"

    page.set_battery(85.0)
    assert page.battery_text_label.text() == "85%"
    page.set_battery(None)
    assert page.battery_text_label.text() == "--%"


def test_kiosk_window_dock_hidden_on_idle_and_shown_on_home(app):
    from unittest.mock import MagicMock
    try:
        from robot_ui.kiosk.signals import KioskSignals
        from robot_ui.kiosk.window import KioskWindow
    except ImportError:
        pytest.skip("KioskWindow dependencies not in environment")

    node = MagicMock()
    signals = KioskSignals()
    win = KioskWindow(node, signals)
    win.show()
    try:
        assert win._active_page == "idle"
        assert win.bottom_navigation.isHidden()
        assert not win.bottom_navigation.isVisible()

        win.show_page("home")
        assert win._active_page == "home"
        assert win.bottom_navigation.isVisible()

        win.show_page("idle")
        assert win._active_page == "idle"
        assert win.bottom_navigation.isHidden()
    finally:
        win.close()


def test_kiosk_window_fullscreen_toggle_and_window_controls(app):
    from unittest.mock import MagicMock
    try:
        from robot_ui.kiosk.signals import KioskSignals
        from robot_ui.kiosk.window import KioskWindow
    except ImportError:
        pytest.skip("KioskWindow dependencies not in environment")

    node = MagicMock()
    signals = KioskSignals()
    win = KioskWindow(node, signals)
    win.show()
    try:
        assert hasattr(win, "window_controls")
        assert win.window_controls is not None
        assert win._min_btn is not None
        assert win._fullscreen_btn is not None

        # Test toggle_fullscreen
        win.toggle_fullscreen()

        # Test keyPressEvent with Key_Escape (toggles fullscreen instead of closing window)
        key_event = QKeyEvent(QEvent.KeyPress, Qt.Key_Escape, Qt.NoModifier)
        win.keyPressEvent(key_event)
        # Window is still active / not closed
        assert win.isVisible()
    finally:
        win.close()



def test_chat_page_widgets_and_object_names(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)
    page.resize(800, 480)

    # Required buttons and objects
    back = page.findChild(QPushButton, "navHomeButton")
    assert back is not None
    assert back.width() == 46
    assert back.height() == 46

    title = page.findChild(QLabel, "chatTitle")
    assert title is not None
    assert title.text() == "Souta Assistant"

    subtitle = page.findChild(QLabel, "chatSubtitle")
    assert subtitle is not None
    assert subtitle.text() == "Sẵn sàng trò chuyện"

    llm_badge = page.findChild(QLabel, "llmStatus")
    assert llm_badge is not None
    assert "LLM" in llm_badge.text()

    speaker = page.findChild(QPushButton, "chatSpeakerButton")
    assert speaker is not None
    assert speaker.width() == 48
    assert speaker.height() == 48

    stop_speaking = page.findChild(QPushButton, "chatStopSpeakingButton")
    assert stop_speaking is not None
    assert stop_speaking.isHidden()

    input_field = page.findChild(QLineEdit, "chatInput")
    assert input_field is not None
    assert input_field.placeholderText() == "Nhập câu hỏi của bạn..."

    mic = page.findChild(QPushButton, "chatMicButton")
    assert mic is not None
    assert mic.width() >= 52
    assert mic.height() >= 52

    send = page.findChild(QPushButton, "chatSendButton")
    assert send is not None
    assert send.width() >= 52
    assert send.height() >= 48
    assert send.text() == "Gửi"


def test_chat_page_suggestions_and_flow(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    chips = [btn for btn in page.findChildren(QPushButton) if btn.objectName() == "suggestionChip"]
    assert len(chips) == 4
    chip_texts = [btn.text() for btn in chips]
    assert "Giới thiệu về PTIT" in chip_texts
    assert "Giới thiệu về Robot" in chip_texts
    assert "Giới thiệu về Khoa Kỹ thuật Điện tử 1" in chip_texts
    assert "Giới thiệu về nhóm nghiên cứu OpenWorkShop" in chip_texts

    submitted = []
    page.chat_submitted.connect(submitted.append)

    # Click first chip
    chips[0].click()
    assert len(submitted) == 1
    assert submitted[0] == chips[0].text()
    assert page._suggestions_widget.isHidden()


def test_chat_page_mic_states(app):
    from robot_ui.kiosk.widgets import ChatPage, MicState
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    # Initial state is UNAVAILABLE
    assert page._mic_state == MicState.UNAVAILABLE
    assert "chưa khả dụng" in page.mic_btn.toolTip()

    # Click mic when unavailable emits mic_clicked and shows friendly status
    clicked = []
    page.mic_clicked.connect(lambda: clicked.append(True))
    page.mic_btn.click()
    assert len(clicked) == 1
    assert "chưa khả dụng" in page._status_label.text()

    # READY
    page.set_mic_state(MicState.READY)
    assert page.mic_btn.isEnabled()
    assert "Bắt đầu nói" in page.mic_btn.toolTip()
    assert page._status_label.text() == "Sẵn sàng trò chuyện"

    # LISTENING
    page.set_mic_state(MicState.LISTENING)
    assert page.mic_btn.isEnabled()
    assert page.mic_btn.property("listening") == "true"
    assert page._status_label.text() == "Đang nghe..."

    # STT_PROCESSING
    page.set_mic_state(MicState.STT_PROCESSING)
    assert not page.mic_btn.isEnabled()
    assert page._status_label.text() == "Đang nhận dạng giọng nói..."

    # LLM_PROCESSING
    page.set_mic_state(MicState.LLM_PROCESSING)
    assert not page.mic_btn.isEnabled()
    assert page._status_label.text() == "Thinking..."

    # ERROR
    page.set_mic_state(MicState.ERROR)
    assert not page.mic_btn.isEnabled()
    assert page._status_label.text() == "Lỗi microphone"


def test_chat_page_speaker_and_stop_speaking(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    # Initial speaker enabled
    assert page._speaker_enabled is True
    speaker_signals = []
    page.speaker_toggled.connect(speaker_signals.append)

    # Toggle speaker off
    page._speaker_btn.click()
    assert page._speaker_enabled is False
    assert speaker_signals == [False]

    # Toggle speaker on
    page._speaker_btn.click()
    assert page._speaker_enabled is True
    assert speaker_signals == [False, True]

    # Test stop speaking button
    stop_btn = page.findChild(QPushButton, "chatStopSpeakingButton")
    assert stop_btn.isHidden()
    assert page._is_speaking is False

    page.set_speaker_speaking(True)
    assert page._is_speaking is True
    assert not stop_btn.isHidden()
    assert page._status_label.text() == "Đang nói..."

    # Click stop speaking button
    stop_signals = []
    page.stop_speaking_requested.connect(lambda: stop_signals.append(True))
    stop_btn.click()
    assert len(stop_signals) == 1
    assert page._is_speaking is False
    assert stop_btn.isHidden()
    assert page._status_label.text() == "Sẵn sàng trò chuyện"


def test_chat_page_typing_indicator_and_busy_lock(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    submitted = []
    page.chat_submitted.connect(submitted.append)

    page.input.setText("Câu hỏi kiểm tra")
    page._send_btn.click()
    assert submitted == ["Câu hỏi kiểm tra"]

    # Input is locked (busy)
    assert not page.input.isEnabled()
    assert not page._send_btn.isEnabled()
    assert page._send_btn.text() == "..."
    assert page._typing_bubble is not None
    assert "Thinking" in page._typing_label.text()
    assert page._typing_timer.isActive()

    # Attempting to double-send does nothing
    page.input.setText("Gửi lần 2")
    page._on_send()
    assert len(submitted) == 1

    # Receive response unlocks and removes typing indicator
    page.receive_response("Câu trả lời từ Souta")
    assert page.input.isEnabled()
    assert page._send_btn.isEnabled()
    assert page._send_btn.text() == "Gửi"
    assert page._typing_bubble is None
    assert not page._typing_timer.isActive()

    # Check messages contain response
    texts = [lbl.text() for lbl in page.findChildren(QLabel, "bubbleText")]
    assert "Câu hỏi kiểm tra" in texts
    assert "Câu trả lời từ Souta" in texts


def test_chat_page_reset_conversation(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    page.input.setText("Tin nhắn trước reset")
    page._on_send()
    page.receive_response("Phản hồi trước reset")

    page.reset_conversation()

    # Check input and state clean
    assert page.input.text() == ""
    assert page.input.isEnabled()
    assert page._status_label.text() == "Sẵn sàng trò chuyện"

    # Suggestions are restored
    chips = [btn for btn in page.findChildren(QPushButton) if btn.objectName() == "suggestionChip"]
    assert len(chips) == 4
    assert not page._suggestions_widget.isHidden()

    # Only initial greeting message
    texts = [lbl.text() for lbl in page.findChildren(QLabel, "bubbleText")]
    assert len(texts) == 1
    assert "Xin chào! Tôi là Souta Robot" in texts[0]


def test_strip_emojis_function():
    from robot_ui.kiosk.widgets import strip_emojis

    sample_1 = "おはよう！Chào buổi sáng! Mình là Souta 🤖 Rất vui được gặp bạn!"
    assert strip_emojis(sample_1) == "おはよう！Chào buổi sáng! Mình là Souta Rất vui được gặp bạn!"

    sample_2 = "Chào bạn 🤖✨! Mình là robot Souta 🎉."
    assert strip_emojis(sample_2) == "Chào bạn! Mình là robot Souta."

    sample_3 = "Không có emoji ở đây."
    assert strip_emojis(sample_3) == "Không có emoji ở đây."

    sample_4 = "Dòng 1 🤖\nDòng 2 🚀"
    assert strip_emojis(sample_4) == "Dòng 1\nDòng 2"


def test_chat_page_strips_emojis_from_response(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    page.receive_response("おはよう！Chào buổi sáng! Mình là Souta 🤖 Rất vui được gặp bạn!")
    texts = [lbl.text() for lbl in page.findChildren(QLabel, "bubbleText")]
    assert any("Mình là Souta Rất vui được gặp bạn!" in t for t in texts)
    assert not any("🤖" in t for t in texts)


def test_chat_page_streaming_and_response_no_duplicate(app):
    from robot_ui.kiosk.widgets import ChatPage
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ChatPage(chat_icon=missing, back_icon=missing)

    initial_bubble_count = len(page.findChildren(QLabel, "bubbleText"))

    # 1. Stream response deltas
    page.receive_stream_delta(1, 1, "Xin chào! ")
    page.receive_stream_delta(1, 1, "Tôi là Souta.")
    page.finalize_stream(1, 1, "Xin chào! Tôi là Souta.")

    # There should now be exactly 1 new bubble
    assert len(page.findChildren(QLabel, "bubbleText")) == initial_bubble_count + 1

    # 2. Subsequent receive_response with the same text must NOT add a duplicate bubble
    page.receive_response("Xin chào! Tôi là Souta.")
    assert len(page.findChildren(QLabel, "bubbleText")) == initial_bubble_count + 1


def test_nav_button_1_25x_dimensions(app):
    from robot_ui.kiosk.widgets import NavButton
    missing = Path("/nonexistent-kiosk-test-asset")
    btn = NavButton("Trang chủ", missing, "home")
    assert btn.iconSize().width() == 39
    assert btn.iconSize().height() == 39
    assert btn.width() == 110
    assert btn.height() == 81


def test_top_left_home_and_emergency_buttons_on_pages(app):
    """Verify that all main pages provide top-left Home and Emergency Stop buttons."""
    from robot_ui.kiosk.widgets import ChatPage, NavigationPage
    from robot_ui.kiosk.pages import WelcomePage, ServicePage, AdminPage

    missing = Path("/nonexistent-kiosk-test-asset")

    # 1. ChatPage
    chat_page = ChatPage(chat_icon=missing, back_icon=missing, home_icon=missing, emergency_icon=missing)
    chat_home_seen, chat_emg_seen = [], []
    chat_page.home_requested.connect(lambda: chat_home_seen.append(True))
    chat_page.emergency_stop_requested.connect(lambda: chat_emg_seen.append(True))
    chat_home_btn = chat_page.findChild(QPushButton, "navHomeButton")
    chat_emg_btn = chat_page.findChild(QPushButton, "navEmergencyButton")
    assert chat_home_btn is not None
    assert chat_emg_btn is not None
    chat_home_btn.click()
    chat_emg_btn.click()
    assert chat_home_seen == [True]
    assert chat_emg_seen == [True]

    # 2. ServicePage
    service_page = ServicePage(back_icon=missing, home_icon=missing, emergency_icon=missing)
    srv_home_seen, srv_emg_seen = [], []
    service_page.home_requested.connect(lambda: srv_home_seen.append(True))
    service_page.emergency_stop_requested.connect(lambda: srv_emg_seen.append(True))
    assert service_page.home_button is not None
    assert service_page.emergency_button is not None
    service_page.home_button.click()
    service_page.emergency_button.click()
    assert srv_home_seen == [True]
    assert srv_emg_seen == [True]

    # 3. AdminPage
    admin_page = AdminPage(
        nav_icon=missing, slam_icon=missing, back_icon=missing, home_icon=missing, emergency_icon=missing
    )
    admin_home_seen, admin_emg_seen = [], []
    admin_page.home_requested.connect(lambda: admin_home_seen.append(True))
    admin_page.emergency_stop_requested.connect(lambda: admin_emg_seen.append(True))
    assert admin_page.home_button is not None
    assert admin_page.emergency_button is not None
    admin_page.home_button.click()
    admin_page.emergency_button.click()
    assert admin_home_seen == [True]
    assert admin_emg_seen == [True]

    # 4. NavigationPage
    nav_page = NavigationPage({
        "home": missing,
        "back": missing,
        "emergency": missing,
        "goal": missing,
        "click_goal": missing,
        "start": missing,
        "stop": missing,
    })
    nav_home_seen, nav_emg_seen = [], []
    nav_page.home_requested.connect(lambda: nav_home_seen.append(True))
    nav_page.emergency_stop_requested.connect(lambda: nav_emg_seen.append(True))
    nav_home = nav_page.findChild(QPushButton, "navHomeButton")
    nav_emg = nav_page.findChild(QPushButton, "navEmergencyButton")
    assert nav_home is not None
    assert nav_emg is not None
    nav_home.click()
    nav_emg.click()
    assert nav_home_seen == [True]
    assert nav_emg_seen == [True]

    # 5. WelcomePage (uses bottom dock for emergency stop, no duplicate button on top)
    welcome_page = WelcomePage(background=missing, ptit=missing, faculty=missing, emergency_icon=missing)
    assert welcome_page.findChild(QPushButton, "welcomeEmergencyButton") is None




def test_service_page_elements_and_selection(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = ServicePage(
        back_icon=missing,
        wait_icon=missing,
        continue_icon=missing,
        service_icon=missing,
    )

    # 1. Header elements
    assert page.findChild(QLabel, "serviceHeaderTitle").text() == "SERVICE"
    assert "Souta Robot · Phục vụ nước" in page.findChild(QLabel, "serviceHeaderSubtitle").text()

    # 2. Main content elements
    assert page.findChild(QLabel, "serviceMainTitle").text() == "Dịch vụ phục vụ nước"
    assert "Vui lòng lựa chọn chức năng bên dưới" in page.findChild(QLabel, "serviceInstruction").text()

    # 3. Two large cards
    assert page.btn_wait is not None
    assert page.btn_continue is not None
    assert page.btn_wait.property("mode") == "wait"
    assert page.btn_continue.property("mode") == "continue"

    # Initial state
    assert page.get_selected_mode() is None
    assert page.btn_wait.property("selected") is False
    assert page.btn_continue.property("selected") is False

    # Selection 1: Chờ phục vụ
    page.btn_wait.click()
    assert page.get_selected_mode() == "wait"
    assert page.btn_wait.property("selected") is True
    assert page.btn_continue.property("selected") is False

    # Selection 2: Hoàn tất & tiếp tục
    page.btn_continue.click()
    assert page.get_selected_mode() == "continue"
    assert page.btn_wait.property("selected") is False
    assert page.btn_continue.property("selected") is True

    # 4. Footer
    footer = page.findChild(QLabel, "serviceFooter")
    assert footer is not None
    assert "Đã kết nối điều khiển robot" in footer.text()

    # Programmatic selection
    page.select_mode_programmatically("wait")
    assert page.get_selected_mode() == "wait"

    # 5. Back button emits back_requested
    seen_back = []
    page.back_requested.connect(lambda: seen_back.append(True))
    page.back_button.click()
    assert seen_back == [True]


def test_service_page_layout_dimensions(app):
    from robot_ui.kiosk.assets import icon_path
    page = ServicePage(
        back_icon=icon_path("back.png"),
        wait_icon=icon_path("service_wait.png"),
        continue_icon=icon_path("service_continue.png"),
        service_icon=icon_path("service.png"),
    )

    # Test 800x480
    page.resize(800, 480)
    page.show()
    app.processEvents()
    assert page.width() == 800
    assert page.height() == 480
    assert page.btn_wait.width() == 280
    assert page.btn_continue.width() == 280

    # Test 1280x720
    page.resize(1280, 720)
    app.processEvents()
    assert page.width() == 1280
    assert page.height() == 720




