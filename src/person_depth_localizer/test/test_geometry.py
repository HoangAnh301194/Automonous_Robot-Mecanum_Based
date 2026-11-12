"""Offline geometry regression tests. No ROS 2, camera or calibration data needed."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from person_depth_localizer.geometry import (
    backproject_depth_pixel, planar_xy, transform_xyz,
)


def tf(tx=0.0, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=tx, y=ty, z=tz),
        rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    ))


class CameraRobotGeometryTests(unittest.TestCase):
    def test_optical_backprojection_principal_point(self):
        xyz = backproject_depth_pixel(320, 240, 2.0, 500, 500, 320, 240)
        np.testing.assert_allclose(xyz, [0.0, 0.0, 2.0])

    def test_optical_backprojection_noncentral(self):
        xyz = backproject_depth_pixel(370, 220, 2.0, 500, 400, 320, 240)
        np.testing.assert_allclose(xyz, [0.2, -0.1, 2.0])

    def test_tf_translation_camera_height_applied_once(self):
        camera_point = backproject_depth_pixel(320, 240, 2.0, 500, 500, 320, 240)
        robot_point = transform_xyz(camera_point, tf(tx=0.25, tz=1.20))
        np.testing.assert_allclose(robot_point, [0.25, 0.0, 3.2])
        x, y, distance = planar_xy(robot_point)
        self.assertAlmostEqual(x, 0.25)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(distance, 0.25)

    def test_optical_frame_forward_ray_points_along_robot_x(self):
        # URDF optical rotation: roll=-90 deg, pitch=0, yaw=-90 deg.
        # Optical +Z (depth forward) becomes robot +X; sensor Z is 1.20m.
        camera_xyz = backproject_depth_pixel(320, 240, 2.0, 500, 500, 320, 240)
        robot_xyz = transform_xyz(
            camera_xyz, tf(tx=0.25, tz=1.20, qx=-0.5, qy=0.5, qz=-0.5, qw=0.5))
        np.testing.assert_allclose(robot_xyz, [2.25, 0.0, 1.20], atol=1e-10)
        self.assertAlmostEqual(planar_xy(robot_xyz)[2], 2.25)

    def test_rotation_then_translation(self):
        # 90-degree rotation about robot Z followed by a 0.5m X offset.
        q = math.sqrt(0.5)
        result = transform_xyz([1.0, 0.0, 0.3], tf(tx=0.5, tz=1.2, qz=q, qw=q))
        np.testing.assert_allclose(result, [0.5, 1.0, 1.5], atol=1e-10)
        self.assertAlmostEqual(planar_xy(result)[2], math.hypot(0.5, 1.0))

    def test_invalid_measurements_and_tf(self):
        with self.assertRaises(ValueError):
            backproject_depth_pixel(0, 0, 0.0, 500, 500, 0, 0)
        with self.assertRaises(ValueError):
            backproject_depth_pixel(0, 0, 1.0, 0, 500, 0, 0)
        with self.assertRaises(ValueError):
            transform_xyz([0, 0, 1], tf(qw=0))
        with self.assertRaises(ValueError):
            planar_xy([float("nan"), 0, 0])


if __name__ == '__main__':
    unittest.main()
