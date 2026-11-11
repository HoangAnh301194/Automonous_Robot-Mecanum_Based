"""Smoke tests for the isolated Qt kiosk interface widgets.

Run after sourcing the ROS 2 workspace:
QT_QPA_PLATFORM=offscreen python3 -m pytest src/robot_ui/test/test_kiosk_interface.py -q
"""
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QPushButton

from robot_ui.kiosk.pages import AdminPage, AdminPinDialog, WelcomePage
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

