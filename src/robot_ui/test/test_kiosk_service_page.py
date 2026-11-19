import os
import pytest
from PyQt5.QtCore import Qt, QPoint, QSize
from PyQt5.QtGui import QMouseEvent
from PyQt5.QtWidgets import QApplication

from robot_ui.kiosk.pages.service_page import ServicePage, ServiceMascotCanvas


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if app is None:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        app = QApplication([])
    return app


def test_service_page_generous_hitbox(qapp):
    """Verify ServiceMascotCanvas has expanded button hitbox."""
    page = ServicePage()
    canvas = page.mascot_canvas
    canvas.resize(600, 600)

    rect_wait = canvas._button_rect("wait")
    rect_continue = canvas._button_rect("continue")

    # Should have expanded width >= 0.35 and height >= 0.26 relative to art side (600)
    assert rect_wait.width() >= 600 * 0.35
    assert rect_wait.height() >= 600 * 0.26
    assert rect_continue.width() >= 600 * 0.35
    assert rect_continue.height() >= 600 * 0.26


def test_service_page_touch_left_half_triggers_wait(qapp):
    """Verify tapping anywhere on the left half of the mascot canvas selects 'wait' (Phục vụ)."""
    page = ServicePage()
    canvas = page.mascot_canvas
    canvas.resize(600, 600)

    selected_modes = []
    page.mode_selected.connect(selected_modes.append)

    # Click on the left half of art rect (e.g. x = 200, y = 300)
    art = canvas._art_rect()
    left_x = art.center().x() - 50
    center_y = art.center().y()

    mouse_event = QMouseEvent(
        QMouseEvent.MouseButtonPress,
        QPoint(left_x, center_y),
        Qt.LeftButton,
        Qt.LeftButton,
        Qt.NoModifier
    )
    canvas.mousePressEvent(mouse_event)

    assert len(selected_modes) == 1
    assert selected_modes[0] == "wait"
    assert page.get_selected_mode() == "wait"


def test_service_page_touch_right_half_triggers_continue(qapp):
    """Verify tapping anywhere on the right half of the mascot canvas selects 'continue' (Hoàn tất)."""
    page = ServicePage()
    canvas = page.mascot_canvas
    canvas.resize(600, 600)

    selected_modes = []
    page.mode_selected.connect(selected_modes.append)

    # Click on the right half of art rect (e.g. x = center + 50)
    art = canvas._art_rect()
    right_x = art.center().x() + 50
    center_y = art.center().y()

    mouse_event = QMouseEvent(
        QMouseEvent.MouseButtonPress,
        QPoint(right_x, center_y),
        Qt.LeftButton,
        Qt.LeftButton,
        Qt.NoModifier
    )
    canvas.mousePressEvent(mouse_event)

    assert len(selected_modes) == 1
    assert selected_modes[0] == "continue"
    assert page.get_selected_mode() == "continue"
