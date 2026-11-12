"""Geometry helpers for uncalibrated camera-to-robot 2D position previews.

No extrinsic heights or offsets are hard-coded here: ROS TF2 is the only
source of camera-to-robot transforms. Projecting XY does not estimate a
person's ground contact point or constitute camera calibration.
"""
import math

import numpy as np


def backproject_depth_pixel(u, v, depth_m, fx, fy, cx, cy):
    """Back-project a depth-image pixel into its optical-frame 3D point (m)."""
    values = np.asarray([u, v, depth_m, fx, fy, cx, cy], dtype=np.float64)
    if not np.isfinite(values).all() or depth_m <= 0 or fx <= 0 or fy <= 0:
        raise ValueError("invalid pixel, depth or depth-camera intrinsics")
    return np.asarray([
        (u - cx) * depth_m / fx,
        (v - cy) * depth_m / fy,
        depth_m,
    ], dtype=np.float64)


def transform_xyz(point, transform_stamped):
    """Apply the TF2 source->target rigid transform to a 3D point, in meters."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    xyz = np.asarray(point, dtype=np.float64)
    if xyz.shape != (3,) or not np.isfinite(xyz).all():
        raise ValueError("expected finite three-dimensional point")

    qxyz = np.asarray([q.x, q.y, q.z], dtype=np.float64)
    qw = float(q.w)
    norm = math.sqrt(float(np.dot(qxyz, qxyz) + qw * qw))
    if not np.isfinite(norm) or norm <= 1e-9:
        raise ValueError("invalid TF quaternion")
    qxyz /= norm
    qw /= norm
    txyz = np.asarray([t.x, t.y, t.z], dtype=np.float64)
    if not np.isfinite(txyz).all():
        raise ValueError("invalid TF translation")

    uv = np.cross(qxyz, xyz)
    uuv = np.cross(qxyz, uv)
    return xyz + 2.0 * (qw * uv + uuv) + txyz


def planar_xy(point_robot):
    """Orthogonally project robot-frame 3D point to its XY plane.

    Returned XY is the horizontal projection of the sampled body point,
    NOT a calibrated person's foot position. Translation Z is NOT added twice.
    """
    point = np.asarray(point_robot, dtype=np.float64)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError("expected finite three-dimensional robot point")
    return float(point[0]), float(point[1]), math.hypot(float(point[0]), float(point[1]))
