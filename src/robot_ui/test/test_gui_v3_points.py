"""Unit tests for gui_v3 start pose (khoihanh) and standby pose (tram_cho) integration."""
from __future__ import annotations

import math
import os
import tempfile
import pytest
import yaml
from PyQt5.QtWidgets import QApplication

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
import gui_v3


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance()
    if app is None:
        app = QApplication([])
    return app


def test_recompute_cyclic_waypoint_yaws_preserves_special():
    wps = [
        {"name": "khoihanh", "x": 0.0, "y": 0.0, "yaw": 1.57},
        {"name": "tram_cho", "x": 1.0, "y": 0.0, "yaw": -0.78},
        {"name": "diem1", "x": 5.0, "y": 0.0, "yaw": 0.0},
        {"name": "diem2", "x": 5.0, "y": 5.0, "yaw": 0.0},
    ]
    gui_v3.recompute_cyclic_waypoint_yaws(wps)
    # khoihanh and tram_cho custom yaw should be preserved
    assert abs(wps[0]["yaw"] - 1.57) < 1e-4
    assert abs(wps[1]["yaw"] - (-0.78)) < 1e-4
    # diem1 and diem2 should have cyclic calculated yaw
    assert abs(wps[2]["yaw"] - math.pi / 2.0) < 1e-4  # facing (5, 5) from (5, 0)


def test_waypoint_naming_starts_from_diem1():
    class DummyApp:
        pass
    dummy = DummyApp()
    dummy.waypoint_name_from_index = gui_v3.MainWindow.waypoint_name_from_index.__get__(dummy)
    assert dummy.waypoint_name_from_index(0) == "diem1"
    assert dummy.waypoint_name_from_index(1) == "diem2"
    assert dummy.waypoint_name_from_index(5) == "diem6"


def test_save_and_get_special_locations(tmp_path, monkeypatch):
    monkeypatch.setattr(gui_v3, "CONFIG_DIR", str(tmp_path))
    class DummyApp:
        def append_log(self, msg): pass
    dummy = DummyApp()
    dummy._save_special_location = gui_v3.MainWindow._save_special_location.__get__(dummy)
    dummy._get_special_location = gui_v3.MainWindow._get_special_location.__get__(dummy)

    # Save khoihanh
    ok1 = dummy._save_special_location("khoihanh", 1.234, -5.678, 1.570)
    assert ok1 is True
    # Save tram_cho
    ok2 = dummy._save_special_location("tram_cho", 3.456, 7.890, -2.164)
    assert ok2 is True

    # Get back
    loc_start = dummy._get_special_location("khoihanh")
    assert loc_start is not None
    assert loc_start["x"] == 1.234
    assert loc_start["y"] == -5.678
    assert abs(loc_start["yaw"] - 1.570) < 1e-4

    loc_standby = dummy._get_special_location("tram_cho")
    assert loc_standby is not None
    assert loc_standby["x"] == 3.456
    assert loc_standby["y"] == 7.890
    assert abs(loc_standby["yaw"] - (-2.164)) < 1e-4


def test_save_patrol_config_excludes_special_from_route(tmp_path, monkeypatch):
    monkeypatch.setattr(gui_v3, "CONFIG_DIR", str(tmp_path))
    class DummyApp:
        def __init__(self):
            self.batch_waypoints = [
                {"name": "khoihanh", "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"name": "tram_cho", "x": 1.0, "y": 1.0, "yaw": 0.0},
                {"name": "diem1", "x": 2.0, "y": 2.0, "yaw": 0.0},
                {"name": "diem2", "x": 3.0, "y": 3.0, "yaw": 0.0},
            ]
            self.ros_node = None
        def _sync_waypoints_from_table(self): pass
        def refresh_batch_waypoint_views(self): pass
        def append_log(self, msg): pass

    dummy = DummyApp()
    dummy.save_patrol_config_to_yaml = gui_v3.MainWindow.save_patrol_config_to_yaml.__get__(dummy)

    ok = dummy.save_patrol_config_to_yaml(show_dialog=False)
    assert ok is True

    # Check routes.yaml
    with open(tmp_path / "routes.yaml", "r", encoding="utf-8") as f:
        routes_data = yaml.safe_load(f)
    assert routes_data["patrol_route"] == ["diem1", "diem2"]

    # Check locations.yaml
    with open(tmp_path / "locations.yaml", "r", encoding="utf-8") as f:
        locs_data = yaml.safe_load(f)
    assert "khoihanh" in locs_data
    assert "tram_cho" in locs_data
    assert "diem1" in locs_data
    assert "diem2" in locs_data


def test_mainwindow_has_start_and_standby_buttons(qapp):
    win = gui_v3.MainWindow()
    assert hasattr(win, "btn_capture_start_pose")
    assert hasattr(win, "btn_relocalize_start_pose")
    assert hasattr(win, "btn_capture_standby_pose")
    assert hasattr(win, "btn_navigate_standby_pose")
    assert hasattr(win, "btn_tab1_relocalize")
    assert hasattr(win, "btn_tab1_standby")
    win.close()

