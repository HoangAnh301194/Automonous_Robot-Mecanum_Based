"""Verify geometric observability of the rear safety corridor without camera hardware."""

from pathlib import Path

import numpy as np
import yaml

from depth_obstacle_detector.obstacle_detector import (
    build_observability_lut,
    evaluate_corridor_observability,
)


def _rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _make_urdf_ground_roi(width=640, height=480, fx=381.36, roi_y=0):
    cx, cy = width / 2.0, height / 2.0
    R = _rpy_to_R(0.0, 0.698132, 0.0) @ _rpy_to_R(-np.pi / 2.0, 0.0, -np.pi / 2.0)
    t = np.array([0.0, 0.0, 0.25], dtype=np.float32)

    u, v = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
    rays_t = np.stack([(u - cx) / fx, (v - cy) / fx, np.ones_like(u)], axis=-1) @ R.T
    rz = rays_t[:, :, 2]
    d = np.where(rz < -0.05, (-0.280526) / rz, 0.0)
    d = np.where((d >= 0.2) & (d <= 2.5), d * 1000.0, 0.0).astype(np.uint16)

    rh = height - roi_y
    roi = d[roi_y:roi_y + rh, 0:width]
    lut = build_observability_lut(
        roi, 0, roi_y, width, rh, fx, fx, cx, cy, t, R, 0.2, 2.5
    )
    return roi, lut, (t, R, cx, cy)


def test_full_clear_corridor_is_observed():
    """Full visible floor behind robot produces corridor_observed=True."""
    ground_roi, lut, _ = _make_urdf_ground_roi(width=640, height=480, fx=381.36, roi_y=0)
    obs = evaluate_corridor_observability(ground_roi, lut, 0.2, 2.5)
    assert obs['corridor_observed'] is True
    assert obs['corridor_observed_ratio'] >= 0.95
    assert obs['near_zone_observed_ratio'] >= 0.95
    assert obs['center_observed_ratio'] >= 0.95
    assert obs['sector_coverage_ratio'] >= 0.95
    assert obs['observed_angle_min'] <= -0.32
    assert obs['observed_angle_max'] >= 0.32
    assert obs['observed_x_min'] <= 0.50
    assert obs['observed_x_max'] >= 1.0


def test_bundled_cam_sau_ground_is_observed():
    """Bundled my_map/cam_sau.npy ground reference produces valid corridor observability."""
    workspace = Path(__file__).resolve().parents[3]
    cam_sau_path = workspace / 'my_map' / 'cam_sau.npy'
    cam_sau_cfg_path = workspace / 'my_map' / 'cam_sau.yaml'
    ground = np.load(str(cam_sau_path))
    h, w = ground.shape
    with open(cam_sau_cfg_path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}

    rx = max(0, min(int(cfg.get('roi_x', 0)), w - 1))
    ry = max(0, min(int(cfg.get('roi_y', 0)), h - 1))
    rw = max(1, min(int(cfg.get('roi_width', w)), w - rx))
    rh = max(1, min(int(cfg.get('roi_height', h)), h - ry))
    min_range = float(cfg.get('min_range', 0.2))
    max_range = float(cfg.get('max_range', 2.5))

    scale = float(w) / 640.0
    fx = 381.36 * scale
    cx, cy = w / 2.0, h / 2.0
    R = _rpy_to_R(0.0, 0.698132, 0.0) @ _rpy_to_R(-np.pi / 2.0, 0.0, -np.pi / 2.0)
    t = np.array([0.0, 0.0, 0.25], dtype=np.float32)
    roi = ground[ry:ry + rh, rx:rx + rw]
    lut = build_observability_lut(
        roi, rx, ry, rw, rh, fx, fx, cx, cy, t, R, min_range, max_range
    )
    obs = evaluate_corridor_observability(
        roi, lut, min_range, max_range, min_observed_x_max=0.50
    )
    assert obs['corridor_observed'] is True
    assert obs['observed_x_max'] >= 0.90


def test_all_zero_close_occlusion_is_rejected():
    """Camera covered closely returns all zeros -> LaserScan all-inf -> must NOT be safe."""
    _, lut, _ = _make_urdf_ground_roi()
    occluded = np.zeros((480, 640), dtype=np.uint16)
    obs = evaluate_corridor_observability(occluded, lut, 0.2, 2.5)
    assert obs['corridor_observed'] is False
    assert obs['corridor_observed_ratio'] == 0.0
    assert obs['sector_coverage_ratio'] == 0.0


def test_half_occluded_lens_is_rejected():
    """Covering half of the lens must fail symmetry and angular sector coverage."""
    ground_roi, lut, _ = _make_urdf_ground_roi()
    half_covered = ground_roi.copy()
    half_covered[:, :330] = 0
    obs = evaluate_corridor_observability(half_covered, lut, 0.2, 2.5)
    assert obs['corridor_observed'] is False
    assert obs['sector_coverage_ratio'] < 0.70


def test_center_blind_corridor_with_high_outer_validity_is_rejected():
    """High overall valid_ratio with center blind spot behind chassis must be rejected."""
    ground_roi, lut, _ = _make_urdf_ground_roi()
    center_blind = ground_roi.copy()
    # Blank out central vertical band directly behind rear bumper while outer FOV stays valid.
    center_blind[:, 220:420] = 0
    obs = evaluate_corridor_observability(center_blind, lut, 0.2, 2.5)
    assert obs['valid_ratio'] > 0.65
    assert obs['corridor_observed'] is False
    assert obs['center_observed_ratio'] < 0.60


def test_tiny_shifted_roi_with_high_valid_ratio_is_rejected():
    """A shifted bottom-only strip has valid_ratio > 0.90 but lacks longitudinal reach."""
    # Only rows [350:480] look very close to bumper (x_max < 0.45 m).
    ground_roi, lut, _ = _make_urdf_ground_roi(width=640, height=480, fx=381.36, roi_y=350)
    obs = evaluate_corridor_observability(ground_roi, lut, 0.2, 2.5, min_observed_x_max=0.50)
    assert obs['valid_ratio'] > 0.90
    assert obs['observed_x_max'] < 0.50
    assert obs['corridor_observed'] is False
