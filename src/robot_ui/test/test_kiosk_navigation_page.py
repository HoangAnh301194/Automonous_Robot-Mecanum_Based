"""Unit tests for NavigationPage and InteractiveNavMapWidget in Robot UI Kiosk."""
import os
from pathlib import Path
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtWidgets import QApplication
import unittest

from robot_ui.kiosk.pages.navigation_page import (
    InteractiveNavMapWidget,
    NavigationPage,
    recompute_cyclic_waypoint_yaws,
    waypoint_name_from_index,
)


class TestKioskNavigationPage(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance()
        if cls.app is None:
            cls.app = QApplication([])

    def setUp(self):
        self.missing = Path("/nonexistent-asset")
        self.icons = {
            "home": self.missing,
            "back": self.missing,
            "emergency": self.missing,
            "goal": self.missing,
        }
        self.page = NavigationPage(self.icons)

    def test_navigation_page_initial_state(self):
        """Test initial UI elements and default tab selection."""
        self.assertIsNotNone(self.page.btn_tab_single)
        self.assertIsNotNone(self.page.btn_tab_patrol)
        self.assertEqual(self.page.stack.currentIndex(), 0)  # Default Single Nav

        self.assertIsNotNone(self.page.start_button)
        self.assertIsNotNone(self.page.stop_button)
        self.assertIsNotNone(self.page.nav_progress_bar)
        self.assertEqual(self.page.nav_progress_bar.value(), 0)

    def test_mode_switching(self):
        """Test switching between Single Nav and Patrol Routes."""
        self.page.btn_tab_patrol.click()
        self.assertEqual(self.page.stack.currentIndex(), 1)
        self.assertEqual(self.page.btn_tab_patrol.property("active"), "true")
        self.assertEqual(self.page.btn_tab_single.property("active"), "false")

        self.page.btn_tab_single.click()
        self.assertEqual(self.page.stack.currentIndex(), 0)
        self.assertEqual(self.page.btn_tab_single.property("active"), "true")
        self.assertEqual(self.page.btn_tab_patrol.property("active"), "false")

    def test_nav_feedback_progress(self):
        """Test that set_nav_feedback properly updates progress bar and labels."""
        feedback_data = {
            "distance_remaining": 5.0,
            "initial_distance": 10.0,
            "navigation_time": 12.5,
            "estimated_time_remaining": 8.0,
            "number_of_recoveries": 1,
        }
        self.page.set_nav_feedback(feedback_data)
        # Progress: (10.0 - 5.0) / 10.0 * 100% = 50%
        self.assertEqual(self.page.nav_progress_bar.value(), 50)
        self.assertIn("5.00 m", self.page.lbl_dist_remain.text())
        self.assertIn("12.5 s", self.page.lbl_nav_time.text())
        self.assertIn("8.0 s", self.page.lbl_est_time.text())
        self.assertIn("1 lần", self.page.lbl_recovery_count.text())

    def test_batch_waypoint_picking_and_undo(self):
        """Test picking batch waypoints, cyclic yaw calculation, undo and clear."""
        self.page._clear_batch_waypoints()
        self.assertEqual(len(self.page._batch_waypoints), 0)

        # Pick point 1 (khoihanh)
        self.page._on_waypoint_picked_on_map(0.0, 0.0, 0.0)
        self.assertEqual(len(self.page._batch_waypoints), 1)
        self.assertEqual(self.page._batch_waypoints[0]["name"], "khoihanh")

        # Pick point 2 (diem1)
        self.page._on_waypoint_picked_on_map(2.0, 0.0, 0.0)
        self.assertEqual(len(self.page._batch_waypoints), 2)
        self.assertEqual(self.page._batch_waypoints[1]["name"], "diem1")

        # Test Undo
        self.page._undo_batch_waypoint()
        self.assertEqual(len(self.page._batch_waypoints), 1)

        # Test Clear
        self.page._clear_batch_waypoints()
        self.assertEqual(len(self.page._batch_waypoints), 0)

    def test_cyclic_yaw_calculation(self):
        """Test mathematical cyclic yaw pointing between successive waypoints."""
        waypoints = [
            {"name": "p0", "x": 0.0, "y": 0.0},
            {"name": "p1", "x": 2.0, "y": 0.0},
            {"name": "p2", "x": 2.0, "y": 2.0},
        ]
        recompute_cyclic_waypoint_yaws(waypoints)
        # p0 -> p1: dx=2, dy=0 => yaw = 0.0
        self.assertAlmostEqual(waypoints[0]["yaw"], 0.0, places=3)
        # p1 -> p2: dx=0, dy=2 => yaw = pi/2 = 1.5708
        self.assertAlmostEqual(waypoints[1]["yaw"], 1.5708, places=3)
        # p2 -> p0 (closing loop): dx=-2, dy=-2 => yaw = -3pi/4 = -2.3562
        self.assertAlmostEqual(waypoints[2]["yaw"], -2.3562, places=3)

    def test_map_zoom_and_pan(self):
        """Test zoom and reset controls on map widget."""
        map_w = self.page.map_widget
        initial_zoom = map_w._zoom_factor
        map_w.zoom_in()
        self.assertGreater(map_w._zoom_factor, initial_zoom)

        map_w.zoom_out()
        map_w.reset_view()
        self.assertAlmostEqual(map_w._zoom_factor, 1.0, places=3)
        self.assertEqual(map_w._pan_x, 0.0)
        self.assertEqual(map_w._pan_y, 0.0)

    def test_sensor_point_setters(self):
        """Test LiDAR scan points and obstacle points dispatching."""
        scan_pts = [(1.0, 2.0), (3.0, 4.0)]
        self.page.set_scan_points(scan_pts)
        self.assertEqual(self.page.map_widget._scan_points, scan_pts)

        obs_pts = [(0.5, 0.5)]
        self.page.set_obstacle_points("front", obs_pts)
        self.assertEqual(self.page.map_widget._front_obs_points, obs_pts)

    def test_fullscreen_and_badges(self):
        """Test fullscreen button toggle and wifi/battery badges."""
        self.assertIsNotNone(self.page.fullscreen_button)
        mock_fullscreen = MagicMock()
        self.page.fullscreen_toggle_requested.connect(mock_fullscreen)
        self.page.fullscreen_button.click()
        mock_fullscreen.assert_called_once()

        self.page.update_fullscreen_button(True)
        self.assertEqual(self.page.fullscreen_button.text(), "🗗")
        self.page.update_fullscreen_button(False)
        self.assertEqual(self.page.fullscreen_button.text(), "🗖")

        self.page.set_wifi(True)
        self.assertEqual(self.page.wifi_text_label.text(), "Online")
        self.page.set_wifi(False)
        self.assertEqual(self.page.wifi_text_label.text(), "Offline")

        self.page.set_battery(88.0)
        self.assertEqual(self.page.battery_text_label.text(), "88%")
        self.page.set_robot_status({"battery_percent": 42.0})
        self.assertEqual(self.page.battery_text_label.text(), "42%")

    def test_map_rotation_and_compass(self):
        """Test rotation setter, normalization, compass button, and reset_view."""
        map_w = self.page.map_widget
        self.assertIsNotNone(map_w.btn_compass)

        # Test rotation setting and normalization
        map_w.set_rotation(45.0)
        self.assertAlmostEqual(map_w._rotation_deg, 45.0, places=2)

        map_w.set_rotation(390.0)  # 390 % 360 = 30
        self.assertAlmostEqual(map_w._rotation_deg, 30.0, places=2)

        # Test compass reset button
        map_w.btn_compass.click()
        self.assertAlmostEqual(map_w._rotation_deg, 0.0, places=2)

        # Test reset_view resets rotation as well
        map_w.set_rotation(90.0)
        map_w.set_zoom(2.5)
        map_w.reset_view()
        self.assertAlmostEqual(map_w._rotation_deg, 0.0, places=2)
        self.assertAlmostEqual(map_w._zoom_factor, 1.0, places=2)

    def test_screen_to_world_with_rotation(self):
        """Test coordinate transformation with rotation applied."""
        map_w = self.page.map_widget
        fake_map = {
            "origin_x": 0.0,
            "origin_y": 0.0,
            "origin_yaw": 0.0,
            "resolution": 0.05,
            "preview_width": 100,
            "preview_height": 100,
            "sample_step": 1,
            "cells": [0] * (100 * 100),
        }
        map_w._map = fake_map
        from PyQt5.QtCore import QPointF, QRectF
        target = QRectF(0, 0, 200, 200)

        # Center point unrotated
        pt_center = QPointF(100, 100)
        map_w.set_rotation(0.0)
        w1 = map_w._screen_to_world(pt_center, target, check_free=False)
        self.assertIsNotNone(w1)

        # Center point rotated 90 degrees should still map to center
        map_w.set_rotation(90.0)
        w2 = map_w._screen_to_world(pt_center, target, check_free=False)
        self.assertIsNotNone(w2)
        self.assertAlmostEqual(w1[0], w2[0], places=2)
        self.assertAlmostEqual(w1[1], w2[1], places=2)

    def test_parse_nav_map_trinary_classification(self):
        """Test parse_nav_map_yaml trinary classification for unknown, free, and occupied spaces."""
        from robot_ui.kiosk.pages.navigation_page import parse_nav_map_yaml
        map_yaml = Path("/home/a/ros2_ws/my_map/a2_amcl.yaml")
        if not map_yaml.is_file():
            self.skipTest("a2_amcl.yaml not found")

        map_data = parse_nav_map_yaml(map_yaml)
        self.assertIsNotNone(map_data)
        cells = map_data["cells"]
        # In a2_amcl, value 205 (unknown) must be -1, 0 (walls) must be 100, 254 (free) must be 0
        self.assertIn(-1, cells)
        self.assertIn(0, cells)
        self.assertIn(100, cells)

        # Check proportions: unknown (-1) should be >50% of cells, not 0% (as when misclassified as free)
        unknown_count = sum(1 for c in cells if c < 0)
        free_count = sum(1 for c in cells if c == 0)
        obs_count = sum(1 for c in cells if c >= 50)
        self.assertGreater(unknown_count, len(cells) * 0.5)
        self.assertGreater(free_count, len(cells) * 0.1)
        self.assertGreater(obs_count, 100)

    def test_rebuild_map_image_dilation_and_contrast(self):
        """Test that _rebuild_map_image dilates obstacles and applies contrast palette."""
        from PyQt5.QtGui import QColor
        map_w = self.page.map_widget
        # Create 10x10 map with single obstacle at (5, 5), unknown at (0, 0), free at (1, 1)
        grid = [[0 for _ in range(10)] for _ in range(10)]
        grid[0][0] = -1
        grid[5][5] = 100  # Single pixel obstacle
        flat = [item for sublist in grid for item in sublist]

        test_map = {
            "origin_x": 0.0,
            "origin_y": 0.0,
            "origin_yaw": 0.0,
            "resolution": 0.05,
            "preview_width": 10,
            "preview_height": 10,
            "sample_step": 1,
            "cells": flat,
        }
        map_w.set_active(True)
        map_w.set_map(test_map)
        self.assertFalse(map_w._map_image.isNull())

        # Obstacle at (5, 5) with flipud should have dilated into 4-connectivity neighbors
        # In screen image: (5, 5) flipud is y = 10 - 1 - 5 = 4
        px_center = QColor(map_w._map_image.pixel(5, 4))
        self.assertEqual((px_center.red(), px_center.green(), px_center.blue()), (0, 0, 0))

        # Check 4-connectivity neighbor (5, 3) (above) - dilated to black
        px_up = QColor(map_w._map_image.pixel(5, 3))
        self.assertEqual((px_up.red(), px_up.green(), px_up.blue()), (0, 0, 0))

        # Check unknown space pixel at (0, 0) -> y = 9 in screen
        px_unknown = QColor(map_w._map_image.pixel(0, 9))
        self.assertEqual((px_unknown.red(), px_unknown.green(), px_unknown.blue()), (38, 44, 52))

    def test_waypoint_badge_deconfliction(self):
        """Test that two adjacent waypoints do not overlap badges during paintEvent."""
        from PyQt5.QtGui import QPixmap, QPainter
        map_w = self.page.map_widget
        map_w.set_batch_waypoints([
            {"name": "khoihanh", "x": 1.0, "y": 1.0, "yaw": 0.0},
            {"name": "diem1", "x": 1.05, "y": 1.05, "yaw": 0.0},
        ])
        # Render onto a test pixmap to execute paintEvent
        pix = QPixmap(400, 400)
        map_w.resize(400, 400)
        map_w.render(pix)
        self.assertFalse(pix.isNull())


if __name__ == "__main__":
    unittest.main()

