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

    def test_2d_pose_estimate_mode_and_signals(self):
        """Test toggling 2D pose estimate button and signal forwarding."""
        self.assertIsNotNone(self.page.btn_2d_estimate)
        self.assertFalse(self.page.btn_2d_estimate.isChecked())
        self.assertFalse(self.page.map_widget._estimate_mode)

        # Toggle ON
        self.page.btn_2d_estimate.setChecked(True)
        self.page._toggle_2d_estimate()
        self.assertTrue(self.page.map_widget._estimate_mode)
        self.assertIn("2D Pose Estimate", self.page.status_label.text())

        # Mock initial_pose_requested listener
        mock_handler = MagicMock()
        self.page.initial_pose_requested.connect(mock_handler)

        # Simulate initial pose selected
        self.page.map_widget.initial_pose_selected.emit(1.25, -0.75, 1.57)
        self.assertFalse(self.page.btn_2d_estimate.isChecked())
        mock_handler.assert_called_once_with(1.25, -0.75, 1.57)
        self.assertIn("1.25m", self.page.status_label.text())


if __name__ == "__main__":
    unittest.main()

