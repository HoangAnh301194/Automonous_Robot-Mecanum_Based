"""Smoke tests for the isolated Qt kiosk interface widgets.

Run after sourcing the ROS 2 workspace:
QT_QPA_PLATFORM=offscreen python3 -m pytest src/robot_ui/test/test_kiosk_interface.py -q
"""
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication, QDialog, QPushButton

from robot_ui.kiosk.pages import AdminPage, AdminPinDialog, WelcomePage


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


def test_welcome_chat_action(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = WelcomePage(missing, missing, missing, missing)
    seen = []
    page.chat_requested.connect(lambda: seen.append(True))
    button = page.findChild(QPushButton, "welcomeChatButton")
    assert button is not None
    button.click()
    assert seen == [True]


def test_admin_nav_and_logout_actions(app):
    missing = Path("/nonexistent-kiosk-test-asset")
    page = AdminPage(missing, missing)
    seen = []
    page.navigation_requested.connect(lambda: seen.append("navigation"))
    page.logout_requested.connect(lambda: seen.append("logout"))
    nav = page.findChild(QPushButton, "adminNavButton")
    assert nav is not None
    nav.click()
    logout = page.findChild(QPushButton, "adminLogout")
    assert logout is not None
    logout.click()
    assert seen == ["navigation", "logout"]
