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
    assert hasattr(win, "btn_reset_emergency")
    assert hasattr(win, "btn_fsm_reset_emergency")

    # Test on_reset_emergency method execution
    win.on_reset_emergency()
    assert "GIẢI PHÓNG" in win.lbl_fsm_state.text()
    win.close()


def test_resolve_map_waypoints_file(tmp_path, monkeypatch):
    monkeypatch.setattr(gui_v3, "CONFIG_DIR", str(tmp_path / "config"))
    class DummyApp:
        pass
    dummy = DummyApp()
    dummy._resolve_map_waypoints_file = gui_v3.MainWindow._resolve_map_waypoints_file.__get__(dummy)

    # 1. Fallback to locations.yaml when no map
    dummy.selected_map_file = ""
    assert dummy._resolve_map_waypoints_file() == str(tmp_path / "config" / "locations.yaml")

    # 2. Existing map_waypoints in same directory
    map_file = tmp_path / "maps" / "room_a.yaml"
    map_file.parent.mkdir(parents=True, exist_ok=True)
    map_file.write_text("image: room_a.pgm\n", encoding="utf-8")
    wp_file = tmp_path / "maps" / "room_a_waypoints.yaml"
    wp_file.write_text("map_name: room_a.yaml\n", encoding="utf-8")

    dummy.selected_map_file = str(map_file)
    assert dummy._resolve_map_waypoints_file() == str(wp_file)

    # 3. amcl stem fallback (room_b_amcl.yaml -> room_b_waypoints.yaml)
    map_amcl = tmp_path / "maps" / "room_b_amcl.yaml"
    map_amcl.write_text("image: room_b.pgm\n", encoding="utf-8")
    wp_base = tmp_path / "maps" / "room_b_waypoints.yaml"
    wp_base.write_text("map_name: room_b.yaml\n", encoding="utf-8")

    assert dummy._resolve_map_waypoints_file(str(map_amcl)) == str(wp_base)


def test_map_specific_waypoint_load_and_save(tmp_path, monkeypatch):
    monkeypatch.setattr(gui_v3, "CONFIG_DIR", str(tmp_path / "config"))
    (tmp_path / "config").mkdir(parents=True, exist_ok=True)

    map_path = tmp_path / "maps" / "khoaa1_amcl.yaml"
    map_path.parent.mkdir(parents=True, exist_ok=True)
    map_path.write_text("image: khoaa1.pgm\n", encoding="utf-8")

    # Write khoaa1_amcl_waypoints.yaml
    wp_path = tmp_path / "maps" / "khoaa1_amcl_waypoints.yaml"
    data = {
        "map_name": "khoaa1_amcl.yaml",
        "locations": {
            "khoihanh": {"x": 1.0, "y": 2.0, "yaw": 0.5, "note": "Goc"},
            "p1": {"x": 10.0, "y": 20.0, "yaw": 0.0, "note": "Point 1"},
            "p2": {"x": 30.0, "y": 40.0, "yaw": 1.0, "note": "Point 2"},
        },
        "routes": {
            "patrol_route": ["p1", "p2"]
        }
    }
    with open(wp_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f)

    class MockApp:
        def __init__(self):
            self.selected_map_file = str(map_path)
            self.batch_waypoints = []
            self.ros_node = None
        def _sync_waypoints_from_table(self): pass
        def refresh_batch_waypoint_views(self): pass
        def append_log(self, msg): pass

    app = MockApp()
    app._resolve_map_waypoints_file = gui_v3.MainWindow._resolve_map_waypoints_file.__get__(app)
    app._sync_active_config_files = gui_v3.MainWindow._sync_active_config_files.__get__(app)
    app.load_patrol_config_from_yaml = gui_v3.MainWindow.load_patrol_config_from_yaml.__get__(app)
    app.save_patrol_config_to_yaml = gui_v3.MainWindow.save_patrol_config_to_yaml.__get__(app)
    app._save_special_location = gui_v3.MainWindow._save_special_location.__get__(app)
    app._get_special_location = gui_v3.MainWindow._get_special_location.__get__(app)

    # Test load
    loaded_ok = app.load_patrol_config_from_yaml(silent=True)
    assert loaded_ok is True
    assert len(app.batch_waypoints) == 2
    assert app.batch_waypoints[0]["name"] == "p1"
    assert app.batch_waypoints[0]["x"] == 10.0
    assert app.batch_waypoints[1]["name"] == "p2"
    assert app.batch_waypoints[1]["x"] == 30.0

    # Verify global config was synced
    loc_file = tmp_path / "config" / "locations.yaml"
    assert loc_file.exists()
    with open(loc_file, "r", encoding="utf-8") as f:
        synced_locs = yaml.safe_load(f)
    assert "p1" in synced_locs
    assert "khoihanh" in synced_locs

    # Test _get_special_location prioritizing map waypoints
    loc_start = app._get_special_location("khoihanh")
    assert loc_start is not None
    assert loc_start["x"] == 1.0
    assert loc_start["y"] == 2.0

    # Test _save_special_location updates both map waypoints file and global config
    app._save_special_location("tram_cho", 5.5, 6.6, -1.2)
    with open(wp_path, "r", encoding="utf-8") as f:
        re_read_wp = yaml.safe_load(f)
    assert "tram_cho" in re_read_wp["locations"]
    assert re_read_wp["locations"]["tram_cho"]["x"] == 5.5

    with open(loc_file, "r", encoding="utf-8") as f:
        re_read_global = yaml.safe_load(f)
    assert "tram_cho" in re_read_global
    assert re_read_global["tram_cho"]["x"] == 5.5


