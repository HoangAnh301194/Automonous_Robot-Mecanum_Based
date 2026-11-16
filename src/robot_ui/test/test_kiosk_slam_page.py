"""Unit tests for SlamMapPage in Robot UI Kiosk."""
import os
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PyQt5.QtWidgets")
from PyQt5.QtWidgets import QApplication, QPushButton, QLineEdit, QListWidget

from robot_ui.kiosk.slam_page import (
    SlamMapPage,
    StaticMapPreviewWidget,
    TouchMapNameDialog,
    check_map_name_exists,
)


@pytest.fixture(scope="module")
def app():
    application = QApplication.instance()
    if application is None:
        application = QApplication([])
    return application


def test_slam_page_elements_and_initial_state(app):
    missing = Path("/nonexistent-asset")
    icons = {
        "home": missing,
        "back": missing,
        "emergency": missing,
        "manual_control": missing,
        "slam": missing,
        "localization": missing,
        "goal": missing,
    }
    page = SlamMapPage(icons)

    assert page.btn_tab_slam is not None
    assert page.btn_tab_amcl is not None
    assert page.stack.currentIndex() == 0  # Default to SLAM Mapping

    assert page.btn_start_slam is not None
    assert page.btn_stop_slam is not None
    assert page.btn_save_map is not None
    assert page.edt_map_name is not None
    assert page.btn_apply_map is not None


def test_slam_tab_switching(app):
    missing = Path("/nonexistent-asset")
    icons = {"home": missing, "back": missing, "emergency": missing}
    page = SlamMapPage(icons)

    # Switch to AMCL mode
    page.btn_tab_amcl.click()
    assert page.stack.currentIndex() == 1
    assert page.btn_tab_amcl.isChecked()
    assert not page.btn_tab_slam.isChecked()

    # Switch back to SLAM mode
    page.btn_tab_slam.click()
    assert page.stack.currentIndex() == 0
    assert page.btn_tab_slam.isChecked()
    assert not page.btn_tab_amcl.isChecked()


def test_slam_start_stop_signals(app):
    missing = Path("/nonexistent-asset")
    icons = {"home": missing, "back": missing, "emergency": missing}
    page = SlamMapPage(icons)

    start_seen, stop_seen = [], []
    page.start_slam_requested.connect(lambda: start_seen.append(True))
    page.stop_slam_requested.connect(lambda: stop_seen.append(True))

    page.btn_start_slam.click()
    assert start_seen == [True]
    assert not page.btn_start_slam.isEnabled()
    assert page.btn_stop_slam.isEnabled()

    page.btn_stop_slam.click()
    assert stop_seen == [True]
    assert page.btn_start_slam.isEnabled()
    assert not page.btn_stop_slam.isEnabled()


def test_slam_save_map_signal(app):
    missing = Path("/nonexistent-asset")
    icons = {"home": missing, "back": missing, "emergency": missing}
    page = SlamMapPage(icons)

    saved_names = []
    page.save_map_requested.connect(saved_names.append)

    page.edt_map_name.setText("test_map_lab")
    page.btn_save_map.click()
    assert saved_names == ["test_map_lab"]


def test_amcl_apply_map_signal(app, tmp_path):
    missing = Path("/nonexistent-asset")
    icons = {"home": missing, "back": missing, "emergency": missing}
    page = SlamMapPage(icons)

    # Create dummy map yaml in temp folder
    dummy_yaml = tmp_path / "test_demo.yaml"
    dummy_yaml.write_text("image: test_demo.pgm\nresolution: 0.05\n")
    dummy_pgm = tmp_path / "test_demo.pgm"
    dummy_pgm.write_bytes(b"P5\n10 10\n255\n" + b"\x80" * 100)

    page._current_map_dir = tmp_path
    page.refresh_map_files()

    applied_requests = []
    page.apply_map_requested.connect(lambda path, backend: applied_requests.append((path, backend)))

    page.btn_tab_amcl.click()
    assert page.map_list_widget.count() >= 1

    page.btn_apply_map.click()
    assert len(applied_requests) == 1
    assert applied_requests[0][0] == str(dummy_yaml)
    assert applied_requests[0][1] == "amcl"


def test_static_map_preview_widget(app, tmp_path):
    widget = StaticMapPreviewWidget()
    dummy_yaml = tmp_path / "preview_sample.yaml"
    dummy_pgm = tmp_path / "preview_sample.pgm"

    dummy_yaml.write_text(f"image: {dummy_pgm.name}\nresolution: 0.05\n")
    dummy_pgm.write_bytes(b"P5\n20 20\n255\n" + b"\x80" * 400)

    success = widget.load_map_yaml(dummy_yaml)
    assert success is True
    assert widget._image_size == (20, 20)
    assert widget._resolution == 0.05

    # Test clear preview
    widget.clear_preview()
    assert widget._map_image.isNull()


def test_admin_slam_button_emits_slam_requested(app):
    missing = Path("/nonexistent-asset")
    from robot_ui.kiosk.pages import AdminPage

    admin = AdminPage(
        nav_icon=missing,
        slam_icon=missing,
        detection_icon=missing,
        start_nav_icon=missing,
        stop_nav_icon=missing,
        status_icon=missing,
    )
    seen = []
    admin.slam_requested.connect(lambda: seen.append(True))
    admin.btn_slam.click()
    assert seen == [True]


def test_touch_map_name_dialog(app):
    dialog = TouchMapNameDialog("floor1")
    assert dialog.entry.text() == "floor1"

    # Test appending tag
    dialog._append_tag("lab")
    assert dialog.get_name() == "floor1_lab"

    # Test backspace
    dialog.entry.backspace()
    assert dialog.get_name() == "floor1_la"

    # Test clear
    dialog.entry.clear()
    dialog._append_tag("tang2")
    assert dialog.get_name() == "tang2"


def test_check_map_name_exists_helper(tmp_path):
    assert check_map_name_exists("", tmp_path) is False

    # Create dummy map yaml
    (tmp_path / "tang1.yaml").write_text("image: tang1.pgm\nresolution: 0.05\n")
    assert check_map_name_exists("tang1", tmp_path) is True
    assert check_map_name_exists("TANG1", tmp_path) is True
    assert check_map_name_exists("tang2", tmp_path) is False

    # Create dual-saved amcl / localization files
    (tmp_path / "lab_amcl.yaml").write_text("image: lab_amcl.pgm\n")
    assert check_map_name_exists("lab", tmp_path) is True
    assert check_map_name_exists("lab_amcl", tmp_path) is True

    (tmp_path / "sanh_localization.posegraph").write_bytes(b"dummy_graph")
    assert check_map_name_exists("sanh", tmp_path) is True


def test_slam_page_duplicate_warning_and_styling(app, tmp_path):
    missing = Path("/nonexistent-asset")
    icons = {"home": missing, "back": missing, "emergency": missing}
    page = SlamMapPage(icons)
    page._current_map_dir = tmp_path

    # Create an existing map in tmp_path
    (tmp_path / "floor_a.yaml").write_text("image: floor_a.pgm\n")
    page.refresh_map_files()

    # 1. Test duplicate detection and red border
    page.edt_map_name.setText("floor_a")
    assert "đã tồn tại" in page.lbl_name_warning.text()
    assert "#ef4444" in page.edt_map_name.styleSheet()
    assert "GHI ĐÈ" in page.btn_save_map.text()

    # 2. Test unique name and green border
    page.edt_map_name.setText("brand_new_zone")
    assert "hợp lệ" in page.lbl_name_warning.text()
    assert "#10b981" in page.edt_map_name.styleSheet()
    assert "LƯU MAP (DUAL-SAVE)" in page.btn_save_map.text()

    # 3. Test empty name warning
    page.edt_map_name.clear()
    assert "Hãy nhập" in page.lbl_name_warning.text()
    assert "#f59e0b" in page.edt_map_name.styleSheet()


def test_touch_dialog_duplicate_warning(app, tmp_path):
    (tmp_path / "room101.yaml").write_text("image: room101.pgm\n")

    # Initialized with existing name
    dialog = TouchMapNameDialog("room101", map_dirs=tmp_path)
    assert "đã tồn tại" in dialog.lbl_warning.text()
    assert "#ef4444" in dialog.entry.styleSheet()
    assert "GHI ĐÈ" in dialog.btn_confirm.text()

    # Changed to unique name
    dialog.entry.setText("room202")
    assert "hợp lệ" in dialog.lbl_warning.text()
    assert "#22c55e" in dialog.entry.styleSheet()
    assert "LƯU TÊN" in dialog.btn_confirm.text()

    # Cleared to empty
    dialog.entry.clear()
    assert "Hãy nhập" in dialog.lbl_warning.text()

