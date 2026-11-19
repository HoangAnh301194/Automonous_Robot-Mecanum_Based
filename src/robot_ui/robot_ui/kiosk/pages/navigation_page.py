"""Robot navigation and patrol management page for Kiosk UI.

Features:
1. Single Goal Navigation:
   - Interactive 2D map with touch zoom (+, -, reset) and pan.
   - Quick location chips for 1-touch goal selection.
   - Real-time progress bar (%), remaining distance, elapsed time, estimated time to arrival.
   - Nav2 recovery count and status prompt.
   - Start / Cancel goal controls.
2. Patrol Route Management (Mission Center):
   - Batch waypoint picking mode on map.
   - Auto cyclic yaw calculation facing next waypoint.
   - Undo last point and Clear all points.
   - Touch-editable Waypoint Table (X, Y, Name, Note).
   - 2-way sync with locations.yaml and routes.yaml.
   - Start Patrol and Pause Patrol mission controls.
3. Enhanced Sensor Overlays:
   - LiDAR scan points (/scan) in Cyan.
   - Camera depth obstacles (/scan_obstacles) in Amber (front) and Purple (rear).
   - Global / local path line and multi-waypoint patrol route lines.
"""
from __future__ import annotations

import html
import math
import time
from datetime import datetime
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PyQt5.QtCore import QEvent, QPoint, QPointF, QRectF, QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (
    QColor,
    QFont,
    QIcon,
    QImage,
    QImageReader,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
    QTextCursor,
)
from PyQt5.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QFrame,
    QGraphicsOpacityEffect,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QSlider,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


def _find_config_dir() -> Path:
    """Find the directory containing locations.yaml and routes.yaml."""
    candidates = [
        Path.cwd() / "config",
        Path(__file__).resolve().parents[5] / "config",
        Path.home() / "ros2_ws" / "config",
        Path("/media/hoang_anh/5A1479B014798FAD/PTIT/NCKH/Automonous_Robot-Mecanum_Based/config"),
    ]
    for c in candidates:
        try:
            res = c.resolve()
            if res.is_dir():
                return res
        except Exception:
            pass
    fallback = Path.cwd() / "config"
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def recompute_cyclic_waypoint_yaws(waypoints: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Set each patrol waypoint's yaw (rad) to face toward the next waypoint in the cycle.
    
    Fixed anchor/docking waypoints such as 'khoihanh' and 'tram_cho' have custom orientation angles
    saved by the operator and must NEVER be overwritten by the path tangent.
    """
    n = len(waypoints)
    if n < 2:
        if n == 1:
            name_lower = str(waypoints[0].get("name", "")).strip().lower()
            if name_lower not in ("khoihanh", "tram_cho") and "yaw" not in waypoints[0]:
                waypoints[0]["yaw"] = 0.0
        return waypoints
    for i, wp in enumerate(waypoints):
        name_lower = str(wp.get("name", "")).strip().lower()
        if name_lower in ("khoihanh", "tram_cho") or wp.get("lock_yaw", False):
            # Bảo toàn nguyên vẹn góc quay ban đầu của điểm gốc và trạm chờ đã lưu
            continue
        nxt = waypoints[(i + 1) % n]
        dx = float(nxt.get("x", 0.0)) - float(wp.get("x", 0.0))
        dy = float(nxt.get("y", 0.0)) - float(wp.get("y", 0.0))
        wp["yaw"] = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else 0.0
    return waypoints


def waypoint_name_from_index(index: int) -> str:
    """Generate default waypoint name: khoihanh for 0, diem1..N for subsequent."""
    if index == 0:
        return "khoihanh"
    return f"diem{index}"


def parse_nav_map_yaml(yaml_path: Path | str | None) -> dict[str, Any] | None:
    """Parse Nav2/AMCL map YAML and raster image into an OccupancyGrid dictionary."""
    if not yaml_path:
        return None
    path = Path(yaml_path)
    if not path.is_file():
        return None

    try:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        img_rel = str(data.get("image", "")).strip()
        if not img_rel:
            return None

        img_path = (path.parent / img_rel).resolve()
        if not img_path.is_file():
            for ext in (".pgm", ".png", ".jpg", ".jpeg"):
                candidate = path.with_suffix(ext)
                if candidate.is_file():
                    img_path = candidate
                    break

        if not img_path.is_file():
            return None

        reader = QImageReader(str(img_path))
        reader.setAutoTransform(True)
        img = reader.read()
        if img.isNull():
            return None

        gray = img.convertToFormat(QImage.Format_Grayscale8)
        w, h = gray.width(), gray.height()
        if w <= 0 or h <= 0:
            return None

        origin = data.get("origin", [0.0, 0.0, 0.0])
        res = float(data.get("resolution", 0.05))
        negate = int(data.get("negate", 0))
        free_thresh = float(data.get("free_thresh", 0.25))
        occupied_thresh = float(data.get("occupied_thresh", 0.65))

        ptr = gray.bits()
        ptr.setsize(h * gray.bytesPerLine())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape((h, gray.bytesPerLine()))[:, :w]
        # In ROS OccupancyGrid, row 0 is bottom row (y=0 in map frame is bottom)
        arr_flipped = np.flipud(arr)

        cells = np.full((h, w), -1, dtype=np.int16)
        if negate:
            occ = arr_flipped.astype(np.float32) / 255.0
            cells[occ < free_thresh] = 0
            cells[occ > occupied_thresh] = 100
        else:
            # Chuẩn ROS trinary map:
            # 0: Vật cản (Occupied)
            # 254: Vùng đi được (Free space)
            # 205: Vùng xám chưa quét (Unknown)
            occ = (255.0 - arr_flipped.astype(np.float32)) / 255.0
            free_limit = min(float(free_thresh), 0.15)
            cells[occ < free_limit] = 0
            cells[occ > occupied_thresh] = 100
            # Bảo đảm vùng xám 205 và lân cận (180..230) luôn là Unknown (-1)
            cells[(arr_flipped >= 180) & (arr_flipped <= 230)] = -1

        # Bảo toàn độ phân giải gốc của bản đồ (không downsampling bỏ pixel tường 1-pixel)
        max_dim = 4096
        step = max(1, math.ceil(max(w, h) / max_dim))
        preview_width = math.ceil(w / step)
        preview_height = math.ceil(h / step)

        if step > 1:
            cells = cells[::step, ::step]

        flat_cells = cells.flatten().tolist()
        return {
            "frame_id": "map",
            "width": w,
            "height": h,
            "preview_width": preview_width,
            "preview_height": preview_height,
            "sample_step": step,
            "resolution": res,
            "origin_x": float(origin[0]) if len(origin) > 0 else 0.0,
            "origin_y": float(origin[1]) if len(origin) > 1 else 0.0,
            "origin_yaw": float(origin[2]) if len(origin) > 2 else 0.0,
            "cells": flat_cells,
            "yaml_path": str(path),
        }
    except Exception:
        return None


class InteractiveNavMapWidget(QWidget):
    """Touch-friendly interactive 2D map widget.

    Supports:
    - Pan & Zoom (Touch buttons, Mouse wheel, Right-click drag)
    - Single Goal Selection (Click / drag)
    - Batch Waypoint Picking Mode (Click to append patrol points)
    - Real-time Sensor Overlays (LiDAR points /scan, Depth Camera Obstacles)
    - Patrol Route Lines & Waypoint Badges
    - Global/Local Path & Robot Footprint with heading
    """

    goal_selected = pyqtSignal(float, float, float)     # (x, y, yaw)
    waypoint_picked = pyqtSignal(float, float, float)   # (x, y, yaw)
    initial_pose_picked = pyqtSignal(float, float, float) # (x, y, yaw)
    invalid_goal = pyqtSignal(str)

    def __init__(
        self,
        goal_icon: Path | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("interactiveNavMapWidget")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setAttribute(Qt.WA_AcceptTouchEvents, True)
        self.setMouseTracking(True)

        self._map: dict[str, Any] | None = None
        self._map_image = QImage()
        self._pose: dict[str, Any] | None = None
        self._path: list[list[float]] = []
        self._locations: list[dict[str, Any]] = []
        self._goal: dict[str, float] | None = None

        # Patrol & Batch Waypoints
        self._batch_waypoints: list[dict[str, Any]] = []
        self._pick_mode = False

        # Initial Pose Picking Mode (RViz-style 2D Pose Estimate: Press -> Drag -> Release)
        self._initial_pose_mode = False
        self._is_dragging_pose = False
        self._estimate_start_world: tuple[float, float] | None = None
        self._estimate_start_screen: QPointF | None = None
        self._estimate_current_screen: QPointF | None = None
        self._estimate_yaw: float = 0.0
        self._candidate_initial_pose: dict[str, float] | None = None

        # Sensor points in world frame (map coordinates)
        self._scan_points: list[tuple[float, float]] = []
        self._front_obs_points: list[tuple[float, float]] = []
        self._rear_obs_points: list[tuple[float, float]] = []

        # Zoom, Pan & Rotation State (Touch Pinch-to-zoom & Two-finger Rotation)
        self._zoom_factor = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._rotation_deg = 0.0
        self._is_panning = False
        self._last_pan_pos: QPoint | None = None
        self._is_touch_gesturing = False
        self._last_gesture_time: float = 0.0
        self._selection_enabled = True
        self._is_active = True
        self._map_dirty = False
        self._last_pose_paint_time: float = 0.0
        self._last_scan_paint_time: float = 0.0
        self._last_obs_paint_time: float = 0.0

        self._goal_icon = QPixmap(str(goal_icon)) if goal_icon is not None else QPixmap()
        if not self._goal_icon.isNull():
            self._goal_icon = self._goal_icon.scaled(
                32, 32, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )

        self.setMinimumSize(280, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # On-map touch zoom buttons
        self._init_touch_zoom_controls()

    def _init_touch_zoom_controls(self) -> None:
        """Create on-map touch zoom buttons (+, -, reset, compass) in the top-right corner."""
        btn_layout = QVBoxLayout()
        btn_layout.setContentsMargins(0, 8, 8, 0)
        btn_layout.setSpacing(6)
        btn_layout.setAlignment(Qt.AlignTop | Qt.AlignRight)

        btn_style = """
            QPushButton {
                background: rgba(17, 20, 24, 220);
                color: #f8fafc;
                border: 1px solid #242c36;
                border-radius: 8px;
                font-size: 16px;
                font-weight: 800;
            }
            QPushButton:hover {
                background: #242c36;
                border-color: #f6c16a;
                color: #ffffff;
            }
            QPushButton:pressed {
                background: #171c22;
            }
        """

        self.btn_compass = QPushButton("🧭", self)
        self.btn_compass.setFixedSize(36, 36)
        self.btn_compass.setToolTip("Đặt lại hướng Bắc (0°)")
        self.btn_compass.setStyleSheet(btn_style + " QPushButton { font-size: 15px; }")
        self.btn_compass.clicked.connect(self.reset_rotation)

        self.btn_zoom_in = QPushButton("+", self)
        self.btn_zoom_in.setFixedSize(36, 36)
        self.btn_zoom_in.setToolTip("Phóng to (+25%)")
        self.btn_zoom_in.setStyleSheet(btn_style)
        self.btn_zoom_in.clicked.connect(self.zoom_in)

        self.btn_zoom_out = QPushButton("-", self)
        self.btn_zoom_out.setFixedSize(36, 36)
        self.btn_zoom_out.setToolTip("Thu nhỏ (-25%)")
        self.btn_zoom_out.setStyleSheet(btn_style)
        self.btn_zoom_out.clicked.connect(self.zoom_out)

        self.btn_zoom_reset = QPushButton("1:1", self)
        self.btn_zoom_reset.setFixedSize(36, 36)
        self.btn_zoom_reset.setToolTip("Đặt lại góc nhìn vừa khung")
        self.btn_zoom_reset.setStyleSheet(btn_style + " QPushButton { font-size: 11px; font-weight: 700; }")
        self.btn_zoom_reset.clicked.connect(self.reset_view)

        btn_layout.addWidget(self.btn_compass)
        btn_layout.addWidget(self.btn_zoom_in)
        btn_layout.addWidget(self.btn_zoom_out)
        btn_layout.addWidget(self.btn_zoom_reset)
        self.setLayout(btn_layout)

    # -------------------------------------------------------------------------
    # Zoom, Pan & Rotation Control
    # -------------------------------------------------------------------------
    def zoom_in(self) -> None:
        self.set_zoom(self._zoom_factor * 1.25)

    def zoom_out(self) -> None:
        self.set_zoom(self._zoom_factor / 1.25)

    def reset_view(self) -> None:
        self._zoom_factor = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._rotation_deg = 0.0
        self.update()

    def reset_rotation(self) -> None:
        self.set_rotation(0.0)

    def set_rotation(self, deg: float) -> None:
        norm_deg = float(deg) % 360.0
        if norm_deg > 180.0:
            norm_deg -= 360.0
        if abs(norm_deg - self._rotation_deg) > 1e-3:
            self._rotation_deg = norm_deg
            self.update()

    def set_zoom(self, zoom: float) -> None:
        new_zoom = max(0.5, min(12.0, zoom))
        if abs(new_zoom - self._zoom_factor) > 1e-4:
            self._zoom_factor = new_zoom
            self.update()

    # -------------------------------------------------------------------------
    # Data Setters
    # -------------------------------------------------------------------------
    def load_map_yaml(self, yaml_path: Path | str | None) -> bool:
        """Load and render map directly from a Nav2/AMCL map YAML file."""
        map_data = parse_nav_map_yaml(yaml_path)
        if map_data is not None:
            self.set_map(map_data)
            return True
        return False

    def set_active(self, active: bool) -> None:
        """Kích hoạt hoặc hủy kích hoạt chế độ hiển thị thực tế của MapWidget."""
        was_active = self._is_active
        self._is_active = active
        if active and not was_active:
            if self._map_dirty and self._map:
                self._rebuild_map_image()
            self.update()

    def _rebuild_map_image(self) -> None:
        """Tái tạo ảnh bản đồ QImage từ dữ liệu cells đã lưu trong cache."""
        self._map_dirty = False
        map_data = self._map
        if not map_data:
            self._map_image = QImage()
            return
        width = int(map_data.get("preview_width", 0))
        height = int(map_data.get("preview_height", 0))
        cells = np.asarray(map_data.get("cells", []), dtype=np.int16)
        if width <= 0 or height <= 0 or cells.size < width * height:
            self._map_image = QImage()
            return
        cells = cells[: width * height].reshape((height, width))
        obs = (cells >= 50)
        # Giãn nhẹ 1 pixel (4-connectivity) để các nét tường 1-pixel không bị đứt đoạn hay biến thành chấm khi thu phóng
        dilated_obs = obs.copy()
        dilated_obs[1:, :] |= obs[:-1, :]
        dilated_obs[:-1, :] |= obs[1:, :]
        dilated_obs[:, 1:] |= obs[:, :-1]
        dilated_obs[:, :-1] |= obs[:, 1:]

        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[cells < 0] = (38, 44, 52)                      # Unknown: Xám đá phiến sẫm (Dark Slate), phân định rõ ranh giới hành lang
        rgb[(cells >= 0) & (cells < 50)] = (245, 247, 250) # Free space: Trắng sáng rõ ràng
        rgb[dilated_obs] = (0, 0, 0)                        # Obstacle: Đen tuyền 100%, nét tường liền mạch và sắc nét
        rgb = np.ascontiguousarray(np.flipud(rgb))
        self._map_image = QImage(
            rgb.data, width, height, width * 3, QImage.Format_RGB888
        ).copy()

    def set_map(self, map_data: dict[str, Any]) -> None:
        if not map_data:
            return
        # Bảo vệ độ phân giải: Nếu bản đồ hiện tại có số pixel nhiều hơn đáng kể bản đồ mới (downsampled),
        # giữ nguyên bản đồ chi tiết hơn để tránh vỡ nét khi zoom.
        if self._map:
            cur_cells_len = len(self._map.get("cells", []))
            new_cells_len = len(map_data.get("cells", []))
            cur_step = int(self._map.get("sample_step", 1))
            new_step = int(map_data.get("sample_step", 1))
            if cur_step < new_step and new_cells_len < cur_cells_len * 0.5:
                return

        self._map = map_data
        if not self._is_active:
            self._map_dirty = True
            return
        self._rebuild_map_image()
        self.update()

    def set_pose(self, pose: dict[str, Any]) -> None:
        self._pose = pose
        if self._is_active:
            now = time.monotonic()
            if now - self._last_pose_paint_time >= 0.066:  # Giới hạn tối đa 15 Hz repaint
                self._last_pose_paint_time = now
                self.update()

    def set_path(self, path: list[list[float]]) -> None:
        self._path = path
        if self._is_active:
            self.update()

    def set_locations(self, locations: list[dict[str, Any]]) -> None:
        self._locations = [item for item in locations if isinstance(item, dict)]
        if self._is_active:
            self.update()

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal = {"x": x, "y": y, "yaw": yaw}
        if self._is_active:
            self.update()

    def clear_goal(self) -> None:
        self._goal = None
        if self._is_active:
            self.update()

    def set_batch_waypoints(self, waypoints: list[dict[str, Any]]) -> None:
        self._batch_waypoints = list(waypoints)
        if self._is_active:
            self.update()

    def set_pick_mode(self, enabled: bool) -> None:
        self._pick_mode = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.PointingHandCursor)
        if self._is_active:
            self.update()

    def set_initial_pose_mode(self, enabled: bool) -> None:
        """Kích hoạt chế độ 2D Pose Estimate (Press -> Drag -> Release)."""
        self._initial_pose_mode = enabled
        self._is_dragging_pose = False
        self._estimate_start_world = None
        self._estimate_start_screen = None
        self._estimate_current_screen = None
        self.setCursor(Qt.CrossCursor if enabled else Qt.PointingHandCursor)
        if not enabled:
            self._candidate_initial_pose = None
        if self._is_active:
            self.update()

    def set_estimate_mode(self, enabled: bool) -> None:
        """Alias cho set_initial_pose_mode tương thích reference simulator."""
        self.set_initial_pose_mode(enabled)

    def set_candidate_initial_pose(self, x: float | None, y: float | None, yaw: float | None = 0.0) -> None:
        """Lưu và hiển thị preview điểm định vị ứng viên trên bản đồ."""
        if x is None or y is None:
            self._candidate_initial_pose = None
        else:
            self._candidate_initial_pose = {
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw or 0.0),
            }
        if self._is_active:
            self.update()

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = enabled
        if not self._pick_mode:
            self.setCursor(Qt.PointingHandCursor if enabled else Qt.ArrowCursor)

    def set_scan_points(self, points: list[tuple[float, float]]) -> None:
        self._scan_points = points
        if self._is_active:
            now = time.monotonic()
            if now - self._last_scan_paint_time >= 0.10:  # Giới hạn tối đa 10 Hz repaint
                self._last_scan_paint_time = now
                self.update()

    def set_obstacle_points(self, side: str, points: list[tuple[float, float]]) -> None:
        if side == "front":
            self._front_obs_points = points
        elif side == "rear":
            self._rear_obs_points = points
        if self._is_active:
            now = time.monotonic()
            if now - self._last_obs_paint_time >= 0.125:  # Giới hạn tối đa 8 Hz repaint
                self._last_obs_paint_time = now
                self.update()

    # -------------------------------------------------------------------------
    # Coordinate Transforms with Zoom & Pan
    # -------------------------------------------------------------------------
    def _base_map_rect(self) -> QRectF:
        if self._map_image.isNull():
            return QRectF()
        available = QRectF(self.rect()).adjusted(4, 4, -4, -4)
        ratio = min(
            available.width() / max(1, self._map_image.width()),
            available.height() / max(1, self._map_image.height()),
        )
        w = self._map_image.width() * ratio
        h = self._map_image.height() * ratio
        return QRectF(
            available.center().x() - w / 2.0,
            available.center().y() - h / 2.0,
            w,
            h,
        )

    def _transformed_map_rect(self) -> QRectF:
        base = self._base_map_rect()
        if base.isEmpty():
            return QRectF()
        w = base.width() * self._zoom_factor
        h = base.height() * self._zoom_factor
        cx = base.center().x() + self._pan_x
        cy = base.center().y() + self._pan_y
        return QRectF(cx - w / 2.0, cy - h / 2.0, w, h)

    def _world_to_screen(self, x: float, y: float, target: QRectF) -> QPointF | None:
        if not self._map:
            return None
        res = float(self._map.get("resolution", 0.05))
        if res <= 0:
            return None
        ox = float(self._map.get("origin_x", 0.0))
        oy = float(self._map.get("origin_y", 0.0))
        oyaw = float(self._map.get("origin_yaw", 0.0))
        step = int(self._map.get("sample_step", 1))

        dx = x - ox
        dy = y - oy
        cos_y = math.cos(oyaw)
        sin_y = math.sin(oyaw)
        cell_x = (cos_y * dx + sin_y * dy) / res / step
        cell_y = (-sin_y * dx + cos_y * dy) / res / step

        pw = float(self._map.get("preview_width", 1))
        ph = float(self._map.get("preview_height", 1))
        sx = target.left() + (cell_x / pw) * target.width()
        sy = target.bottom() - (cell_y / ph) * target.height()
        return QPointF(sx, sy)

    def _screen_to_world(self, point: QPointF, target: QRectF, check_free: bool = True) -> tuple[float, float] | None:
        if not self._map:
            return None

        # Apply inverse rotation around target center if map is rotated
        if abs(self._rotation_deg) > 1e-4:
            rad = math.radians(-self._rotation_deg)
            cx = target.center().x()
            cy = target.center().y()
            px = point.x() - cx
            py = point.y() - cy
            cos_r = math.cos(rad)
            sin_r = math.sin(rad)
            point = QPointF(cx + px * cos_r - py * sin_r, cy + px * sin_r + py * cos_r)

        if not target.contains(point):
            return None
        pw = int(self._map.get("preview_width", 0))
        ph = int(self._map.get("preview_height", 0))
        res = float(self._map.get("resolution", 0.05))
        step = int(self._map.get("sample_step", 1))
        if pw <= 0 or ph <= 0 or res <= 0:
            return None

        cell_x = (point.x() - target.left()) / target.width() * pw
        cell_y = (target.bottom() - point.y()) / target.height() * ph
        ix = min(pw - 1, max(0, int(cell_x)))
        iy = min(ph - 1, max(0, int(cell_y)))

        # Free space validation check
        if check_free:
            cells = self._map.get("cells", [])
            idx = iy * pw + ix
            if idx >= len(cells) or int(cells[idx]) < 0 or int(cells[idx]) >= 50:
                self.invalid_goal.emit("Vùng chọn rơi vào vật cản hoặc chưa rõ bản đồ")
                return None

        dist_x = cell_x * res * step
        dist_y = cell_y * res * step
        oyaw = float(self._map.get("origin_yaw", 0.0))
        cos_y = math.cos(oyaw)
        sin_y = math.sin(oyaw)
        wx = float(self._map.get("origin_x", 0.0)) + cos_y * dist_x - sin_y * dist_y
        wy = float(self._map.get("origin_y", 0.0)) + sin_y * dist_x + cos_y * dist_y
        return wx, wy

    # -------------------------------------------------------------------------
    # Mouse & Touch Events
    # -------------------------------------------------------------------------
    def event(self, event: QEvent) -> bool:
        if event.type() in (QEvent.TouchBegin, QEvent.TouchUpdate, QEvent.TouchEnd, QEvent.TouchCancel):
            if self._handle_touch_event(event):
                return True
        return super().event(event)

    def _handle_touch_event(self, event: Any) -> bool:
        touch_points = event.touchPoints()
        num_points = len(touch_points)

        if event.type() in (QEvent.TouchEnd, QEvent.TouchCancel) or num_points < 2:
            if self._is_touch_gesturing:
                self._is_touch_gesturing = False
                self._last_gesture_time = time.monotonic()
            return False

        if num_points >= 2:
            p1_curr = touch_points[0].pos()
            p2_curr = touch_points[1].pos()
            p1_prev = touch_points[0].lastPos()
            p2_prev = touch_points[1].lastPos()

            # 1. Pinch Zoom
            d_curr = math.hypot(p2_curr.x() - p1_curr.x(), p2_curr.y() - p1_curr.y())
            d_prev = math.hypot(p2_prev.x() - p1_prev.x(), p2_prev.y() - p1_prev.y())

            if d_prev > 10.0 and d_curr > 10.0:
                scale = d_curr / d_prev
                scale = max(0.80, min(1.25, scale))
                if abs(scale - 1.0) > 0.002:
                    self.set_zoom(self._zoom_factor * scale)

            # 2. Two-finger rotation
            angle_curr = math.atan2(p2_curr.y() - p1_curr.y(), p2_curr.x() - p1_curr.x())
            angle_prev = math.atan2(p2_prev.y() - p1_prev.y(), p2_prev.x() - p1_prev.x())
            delta_angle = angle_curr - angle_prev
            delta_angle = (delta_angle + math.pi) % (2.0 * math.pi) - math.pi

            delta_deg = math.degrees(delta_angle)
            if abs(delta_deg) < 30.0:
                new_rot = (self._rotation_deg + delta_deg) % 360.0
                if new_rot > 180.0:
                    new_rot -= 360.0
                self._rotation_deg = new_rot

            # 3. Two-finger pan
            mid_curr = (p1_curr + p2_curr) / 2.0
            mid_prev = (p1_prev + p2_prev) / 2.0
            dx_pan = mid_curr.x() - mid_prev.x()
            dy_pan = mid_curr.y() - mid_prev.y()
            if math.hypot(dx_pan, dy_pan) < 80.0:
                self._pan_x += dx_pan
                self._pan_y += dy_pan

            self._is_touch_gesturing = True
            self._last_gesture_time = time.monotonic()
            self.update()
            event.accept()
            return True

        return False

    def wheelEvent(self, event: Any) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.15 if delta > 0 else (1.0 / 1.15)
        self.set_zoom(self._zoom_factor * factor)
        event.accept()

    def mousePressEvent(self, event: Any) -> None:
        if time.monotonic() - getattr(self, "_last_gesture_time", 0.0) < 0.35:
            event.accept()
            return

        if event.button() in (Qt.RightButton, Qt.MiddleButton) or (event.modifiers() & Qt.ShiftModifier):
            self._is_panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        if event.button() == Qt.LeftButton:
            target = self._transformed_map_rect()
            if target.isEmpty() or not target.contains(QPointF(event.pos())):
                return

            if self._initial_pose_mode:
                # Mode: RViz 2D Pose Estimate (Bắt đầu Press -> Drag)
                world = self._screen_to_world(QPointF(event.pos()), target, check_free=True)
                if world is None:
                    return
                wx, wy = world
                fallback_yaw = float(self._pose.get("yaw", 0.0)) if self._pose else 0.0
                self._is_dragging_pose = True
                self._estimate_start_world = (wx, wy)
                self._estimate_start_screen = QPointF(event.pos())
                self._estimate_current_screen = QPointF(event.pos())
                self._estimate_yaw = fallback_yaw
                self.set_candidate_initial_pose(wx, wy, fallback_yaw)
                self.update()
                event.accept()
                return

            if self._selection_enabled:
                world = self._screen_to_world(QPointF(event.pos()), target, check_free=True)
                if world is None:
                    return
                wx, wy = world

                if self._pick_mode:
                    # Mode: Batch Waypoint Picking for Patrol Route
                    yaw = 0.0
                    if self._batch_waypoints:
                        last_wp = self._batch_waypoints[-1]
                        dx = wx - float(last_wp.get("x", 0.0))
                        dy = wy - float(last_wp.get("y", 0.0))
                        yaw = math.atan2(dy, dx)
                    self.waypoint_picked.emit(wx, wy, yaw)
                else:
                    # Mode: Single Goal Selection
                    if self._pose:
                        rx = float(self._pose.get("x", wx))
                        ry = float(self._pose.get("y", wy))
                        yaw = math.atan2(wy - ry, wx - rx)
                    else:
                        yaw = 0.0
                    self.set_goal(wx, wy, yaw)
                    self.goal_selected.emit(wx, wy, yaw)
                event.accept()

    def mouseMoveEvent(self, event: Any) -> None:
        if self._is_panning and self._last_pan_pos is not None:
            delta = event.pos() - self._last_pan_pos
            if event.modifiers() & Qt.ShiftModifier:
                # Shift + Drag: Rotate map (for desktop / mouse testing)
                new_rot = (self._rotation_deg + delta.x() * 0.5) % 360.0
                if new_rot > 180.0:
                    new_rot -= 360.0
                self._rotation_deg = new_rot
            else:
                self._pan_x += delta.x()
                self._pan_y += delta.y()
            self._last_pan_pos = event.pos()
            self.update()
            event.accept()
            return

        if self._initial_pose_mode and self._is_dragging_pose and self._estimate_start_screen is not None:
            curr_pos = QPointF(event.pos())
            self._estimate_current_screen = curr_pos
            dx_scr = curr_pos.x() - self._estimate_start_screen.x()
            dy_scr = curr_pos.y() - self._estimate_start_screen.y()
            drag_dist_scr = math.hypot(dx_scr, dy_scr)

            if drag_dist_scr >= 8.0:
                target = self._transformed_map_rect()
                if not target.isEmpty():
                    pw = float(self._map.get("preview_width", 1)) if self._map else 1.0
                    ph = float(self._map.get("preview_height", 1)) if self._map else 1.0
                    res = float(self._map.get("resolution", 0.05)) if self._map else 0.05
                    step = int(self._map.get("sample_step", 1)) if self._map else 1
                    oyaw = float(self._map.get("origin_yaw", 0.0)) if self._map else 0.0

                    # Unrotate screen drag vector if map is rotated
                    if abs(self._rotation_deg) > 1e-4:
                        rot_rad = math.radians(-self._rotation_deg)
                        cos_rot = math.cos(rot_rad)
                        sin_rot = math.sin(rot_rad)
                        d_unrot_x = dx_scr * cos_rot - dy_scr * sin_rot
                        d_unrot_y = dx_scr * sin_rot + dy_scr * cos_rot
                    else:
                        d_unrot_x = dx_scr
                        d_unrot_y = dy_scr

                    d_cell_x = (d_unrot_x / target.width()) * pw
                    d_cell_y = (-d_unrot_y / target.height()) * ph
                    d_dist_x = d_cell_x * res * step
                    d_dist_y = d_cell_y * res * step

                    cos_y = math.cos(oyaw)
                    sin_y = math.sin(oyaw)
                    dwx = cos_y * d_dist_x - sin_y * d_dist_y
                    dwy = sin_y * d_dist_x + cos_y * d_dist_y

                    if math.hypot(dwx, dwy) > 0.02:
                        self._estimate_yaw = math.atan2(dwy, dwx)

            if self._estimate_start_world:
                self.set_candidate_initial_pose(
                    self._estimate_start_world[0],
                    self._estimate_start_world[1],
                    self._estimate_yaw,
                )
            self.update()
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: Any) -> None:
        if time.monotonic() - getattr(self, "_last_gesture_time", 0.0) < 0.35:
            event.accept()
            return

        if event.button() in (Qt.RightButton, Qt.MiddleButton) and self._is_panning:
            self._is_panning = False
            self._last_pan_pos = None
            self.setCursor(Qt.CrossCursor if (self._pick_mode or self._initial_pose_mode) else Qt.PointingHandCursor)
            event.accept()
            return

        if event.button() == Qt.LeftButton and self._initial_pose_mode and self._is_dragging_pose:
            self._is_dragging_pose = False
            start_world = self._estimate_start_world
            yaw = self._estimate_yaw

            self._estimate_start_world = None
            self._estimate_start_screen = None
            self._estimate_current_screen = None

            if start_world is not None:
                wx, wy = start_world
                self.set_candidate_initial_pose(wx, wy, yaw)
                self.initial_pose_picked.emit(wx, wy, yaw)

            self.set_initial_pose_mode(False)
            event.accept()
            return

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event: Any) -> None:
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self.reset_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    # -------------------------------------------------------------------------
    # Rendering
    # -------------------------------------------------------------------------
    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)

        # Background
        painter.fillRect(self.rect(), QColor("#080808"))

        if self._map_image.isNull():
            painter.setPen(QColor("#64748b"))
            painter.setFont(QFont("Sans Serif", 12, QFont.DemiBold))
            painter.drawText(self.rect(), Qt.AlignCenter, "Đang chờ dữ liệu bản đồ (/map)...")
            return

        target = self._transformed_map_rect()
        if target.isEmpty():
            return

        # Clip map to widget boundaries with rounded corners
        widget_clip = QPainterPath()
        widget_clip.addRoundedRect(QRectF(self.rect()).adjusted(2, 2, -2, -2), 10, 10)
        painter.save()
        painter.setClipPath(widget_clip)

        # Apply map rotation around center of transformed map
        if abs(self._rotation_deg) > 1e-4:
            c = target.center()
            painter.translate(c)
            painter.rotate(self._rotation_deg)
            painter.translate(-c)

        # 1. Map Image: Vẽ sắc nét từng ô pixel (tắt SmoothPixmapTransform chống nhòe mờ mép tường)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        painter.drawImage(target, self._map_image)

        # 2. Bật Antialiasing cho các nét vẽ vector tiếp theo (LiDAR, Goal, Waypoints, Path, Robot)
        painter.setRenderHint(QPainter.Antialiasing, True)

        # 2. LiDAR Points (/scan) in Cyan
        if self._scan_points:
            painter.setPen(QPen(QColor(0, 255, 255, 200), 1.5))
            for px, py in self._scan_points:
                pt = self._world_to_screen(px, py, target)
                if pt is not None and self.rect().contains(pt.toPoint()):
                    painter.drawPoint(pt)

        # 3. Depth Obstacle Points (Front in Amber, Rear in Purple)
        if self._front_obs_points:
            painter.setPen(QPen(QColor(245, 158, 11, 230), 3))
            for px, py in self._front_obs_points:
                pt = self._world_to_screen(px, py, target)
                if pt is not None and self.rect().contains(pt.toPoint()):
                    painter.drawPoint(pt)

        if self._rear_obs_points:
            painter.setPen(QPen(QColor(232, 121, 249, 230), 3))
            for px, py in self._rear_obs_points:
                pt = self._world_to_screen(px, py, target)
                if pt is not None and self.rect().contains(pt.toPoint()):
                    painter.drawPoint(pt)

        # 4. Nav2 Plan Path (blue/amber dashed line)
        if len(self._path) > 1:
            points = [
                self._world_to_screen(float(p[0]), float(p[1]), target)
                for p in self._path
            ]
            valid_pts = [pt for pt in points if pt is not None]
            if len(valid_pts) > 1:
                painter.setPen(QPen(QColor("#38bdf8"), 3, Qt.DashLine, Qt.RoundCap, Qt.RoundJoin))
                painter.drawPolyline(QPolygonF(valid_pts))

        # 5. Multi-Waypoint Patrol Route Lines & Badges
        if self._batch_waypoints:
            wp_screen_pts = []
            for wp in self._batch_waypoints:
                pt = self._world_to_screen(float(wp.get("x", 0.0)), float(wp.get("y", 0.0)), target)
                wp_screen_pts.append(pt)

            # Draw connecting lines between waypoints
            n_wp = len(self._batch_waypoints)
            if n_wp > 1:
                painter.setPen(QPen(QColor("#f59e0b"), 2, Qt.SolidLine, Qt.RoundCap))
                for i in range(n_wp):
                    p1 = wp_screen_pts[i]
                    p2 = wp_screen_pts[(i + 1) % n_wp]
                    if p1 is not None and p2 is not None:
                        painter.drawLine(p1, p2)

            # Draw waypoint nodes and badges
            painter.setFont(QFont("Sans Serif", 9, QFont.Bold))
            placed_badge_rects: list[QRectF] = []
            for i, (wp, pt) in enumerate(zip(self._batch_waypoints, wp_screen_pts)):
                if pt is None or not self.rect().contains(pt.toPoint()):
                    continue
                is_home = (i == 0)
                node_color = QColor("#10b981") if is_home else QColor("#f59e0b")

                # Node Circle
                painter.setPen(QPen(QColor("#ffffff"), 2))
                painter.setBrush(node_color)
                painter.drawEllipse(pt, 6, 6)

                # Heading Arrow
                yaw = float(wp.get("yaw", 0.0))
                ax = pt.x() + 16 * math.cos(yaw)
                ay = pt.y() - 16 * math.sin(yaw)
                painter.setPen(QPen(node_color, 2))
                painter.drawLine(pt, QPointF(ax, ay))

                # Name badge with collision avoidance (tránh đè nhãn khi các điểm xuất phát/tuần tra ở gần nhau)
                name = str(wp.get("name", f"P{i}"))
                badge_w = painter.fontMetrics().horizontalAdvance(name) + 12
                badge_h = 18.0
                candidate_offsets = [-26.0, 12.0, -44.0, 30.0]
                best_rect = QRectF(pt.x() - badge_w / 2.0, pt.y() - 26.0, badge_w, badge_h)
                for off_y in candidate_offsets:
                    cand = QRectF(pt.x() - badge_w / 2.0, pt.y() + off_y, badge_w, badge_h)
                    if not any(cand.intersects(r) for r in placed_badge_rects):
                        best_rect = cand
                        break
                placed_badge_rects.append(best_rect)

                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(17, 20, 24, 220))
                painter.drawRoundedRect(best_rect, 4, 4)
                painter.setPen(node_color)
                painter.drawText(best_rect, Qt.AlignCenter, name)

        # 6. Predefined Locations (if not already covered by waypoints)
        if not self._batch_waypoints and self._locations:
            painter.setFont(QFont("Sans Serif", 8, QFont.DemiBold))
            for loc in self._locations:
                pt = self._world_to_screen(float(loc.get("x", 0.0)), float(loc.get("y", 0.0)), target)
                if pt is None or not self.rect().contains(pt.toPoint()):
                    continue
                name = str(loc.get("name", ""))
                tw = painter.fontMetrics().horizontalAdvance(name) + 10
                rect_b = QRectF(pt.x() - tw / 2.0, pt.y() - 22, tw, 16)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(23, 28, 34, 210))
                painter.drawRoundedRect(rect_b, 4, 4)
                painter.setPen(QColor("#cbd5e1"))
                painter.drawText(rect_b, Qt.AlignCenter, name)

        # 7. Robot Footprint & Heading
        if self._pose:
            robot_pt = self._world_to_screen(
                float(self._pose.get("x", 0.0)), float(self._pose.get("y", 0.0)), target
            )
            if robot_pt is not None and self.rect().contains(robot_pt.toPoint()):
                painter.save()
                painter.translate(robot_pt)
                painter.rotate(-math.degrees(float(self._pose.get("yaw", 0.0))))
                # Robot Arrow
                painter.setPen(QPen(QColor("#ffffff"), 2))
                painter.setBrush(QColor("#ef4444"))
                poly = QPolygonF([
                    QPointF(16, 0),
                    QPointF(-10, -10),
                    QPointF(-6, 0),
                    QPointF(-10, 10),
                ])
                painter.drawPolygon(poly)
                painter.restore()

        # 8. Single Goal Pin
        if self._goal:
            gpt = self._world_to_screen(self._goal["x"], self._goal["y"], target)
            if gpt is not None and self.rect().contains(gpt.toPoint()):
                if not self._goal_icon.isNull():
                    painter.drawPixmap(
                        round(gpt.x() - self._goal_icon.width() / 2),
                        round(gpt.y() - self._goal_icon.height()),
                        self._goal_icon,
                    )
                else:
                    painter.setPen(QPen(QColor("#ffffff"), 2))
                    painter.setBrush(QColor("#f43f5e"))
                    painter.drawEllipse(gpt, 8, 8)

        # 9. Candidate Initial Pose Preview (RViz-style Green Arrow & Badge)
        if self._candidate_initial_pose:
            cpt = self._world_to_screen(
                float(self._candidate_initial_pose.get("x", 0.0)),
                float(self._candidate_initial_pose.get("y", 0.0)),
                target,
            )
            if cpt is not None and self.rect().contains(cpt.toPoint()):
                yaw = float(self._candidate_initial_pose.get("yaw", 0.0))
                deg = math.degrees(yaw)

                # Chiều dài hiển thị mũi tên trên màn hình
                arrow_len = 45.0
                if self._is_dragging_pose and self._estimate_start_screen and self._estimate_current_screen:
                    drag_len = math.hypot(
                        self._estimate_current_screen.x() - self._estimate_start_screen.x(),
                        self._estimate_current_screen.y() - self._estimate_start_screen.y(),
                    )
                    arrow_len = max(35.0, min(140.0, drag_len))

                # Hướng vector trên màn hình từ hệ tọa độ thế giới (đảm bảo đúng xoay map)
                tip_world_x = float(self._candidate_initial_pose["x"]) + 0.5 * math.cos(yaw)
                tip_world_y = float(self._candidate_initial_pose["y"]) + 0.5 * math.sin(yaw)
                tip_scr = self._world_to_screen(tip_world_x, tip_world_y, target)
                if tip_scr is not None:
                    v_dx = tip_scr.x() - cpt.x()
                    v_dy = tip_scr.y() - cpt.y()
                    v_norm = math.hypot(v_dx, v_dy)
                    if v_norm > 1e-4:
                        dir_x = v_dx / v_norm
                        dir_y = v_dy / v_norm
                    else:
                        dir_x = math.cos(yaw)
                        dir_y = -math.sin(yaw)
                else:
                    dir_x = math.cos(yaw)
                    dir_y = -math.sin(yaw)

                arrow_tip = QPointF(cpt.x() + arrow_len * dir_x, cpt.y() + arrow_len * dir_y)

                # Vòng tròn đế (Base Circle)
                painter.setPen(QPen(QColor("#10b981"), 2, Qt.DashLine if self._is_dragging_pose else Qt.SolidLine))
                painter.setBrush(QColor(16, 185, 129, 65 if self._is_dragging_pose else 40))
                painter.drawEllipse(cpt, 16, 16)

                painter.setPen(QPen(QColor("#ffffff"), 2))
                painter.setBrush(QColor("#10b981"))
                painter.drawEllipse(cpt, 5, 5)

                # Thân mũi tên (Arrow Shaft)
                painter.setPen(QPen(QColor("#10b981"), 3, Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(cpt, arrow_tip)

                # Đầu mũi tên (Arrow Head)
                perp_x = -dir_y
                perp_y = dir_x
                head_len = 12.0
                head_w = 6.0
                p_head_base = QPointF(arrow_tip.x() - head_len * dir_x, arrow_tip.y() - head_len * dir_y)
                wing1 = QPointF(p_head_base.x() + head_w * perp_x, p_head_base.y() + head_w * perp_y)
                wing2 = QPointF(p_head_base.x() - head_w * perp_x, p_head_base.y() - head_w * perp_y)
                painter.setPen(QPen(QColor("#ffffff"), 1))
                painter.setBrush(QColor("#10b981"))
                poly_head = QPolygonF([arrow_tip, wing1, wing2])
                painter.drawPolygon(poly_head)

                # Nhãn tọa độ HUD
                b_text = f"[2D POSE] ({self._candidate_initial_pose['x']:.2f}, {self._candidate_initial_pose['y']:.2f}, {deg:.0f}°)"
                painter.setFont(QFont("Sans Serif", 9, QFont.Bold))
                tw = painter.fontMetrics().horizontalAdvance(b_text) + 12
                b_rect = QRectF(cpt.x() - tw / 2.0, cpt.y() - 32, tw, 20)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(15, 23, 42, 230))
                painter.drawRoundedRect(b_rect, 4, 4)
                painter.setPen(QColor("#38bdf8"))
                painter.drawText(b_rect, Qt.AlignCenter, b_text)

        painter.restore()

        # Border
        painter.setPen(QPen(QColor("#242c36"), 2))
        painter.drawRoundedRect(QRectF(self.rect()).adjusted(1, 1, -1, -1), 10, 10)

        # Zoom & Status HUD in bottom-left corner
        hud_text = f"Zoom: {self._zoom_factor:.1f}x"
        if abs(self._rotation_deg) > 0.5:
            hud_text += f" | {self._rotation_deg:.0f}°"
        if self._initial_pose_mode:
            hud_text += " | [2D POSE] (Chạm & kéo)"
        elif self._pick_mode:
            hud_text += f" | Đang chấm điểm ({len(self._batch_waypoints)} điểm)"
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(17, 20, 24, 190))
        painter.drawRoundedRect(QRectF(8, self.height() - 28, 200, 22), 5, 5)
        painter.setPen(QColor("#94a3b8"))
        painter.setFont(QFont("Sans Serif", 8, QFont.Bold))
        painter.drawText(QRectF(14, self.height() - 28, 190, 22), Qt.AlignVCenter, hud_text)

        painter.end()


# Backwards compatibility alias for existing code referencing MapWidget
MapWidget = InteractiveNavMapWidget


class InitialPoseDialog(QDialog):
    """Hộp thoại thiết lập vị trí ban đầu cho AMCL định vị robot (2D Pose Estimate)."""

    pose_confirmed = pyqtSignal(float, float, float, str)  # (x, y, yaw, source)
    pick_on_map_requested = pyqtSignal()

    def __init__(
        self,
        current_map_path: Path | None,
        loc_state: str,
        loc_msg: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Định vị Robot - Initial Pose")
        self.setModal(True)
        self.resize(520, 500)
        self.setStyleSheet("""
            QDialog {
                background-color: #0f1318;
                color: #f8fafc;
                border: 1px solid #242c36;
                border-radius: 12px;
            }
            QLabel {
                color: #e2e8f0;
            }
            QGroupBox {
                border: 1px solid #242c36;
                border-radius: 10px;
                margin-top: 14px;
                padding: 12px 10px 10px 10px;
                font-size: 13px;
                font-weight: 700;
                color: #f6c16a;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 4px;
            }
            QPushButton {
                background: #171c22;
                color: #f8fafc;
                border: 1px solid #242c36;
                border-radius: 8px;
                padding: 8px 16px;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #242c36;
                border-color: #f6c16a;
            }
            QPushButton#primaryActionBtn {
                background: #10b981;
                color: #ffffff;
                font-weight: 700;
                border: none;
            }
            QPushButton#primaryActionBtn:hover {
                background: #059669;
            }
            QPushButton#closeBtn {
                background: #334155;
                color: #ffffff;
            }
            QDoubleSpinBox {
                background: #171c22;
                color: #ffffff;
                border: 1px solid #334155;
                border-radius: 6px;
                padding: 4px 8px;
                font-size: 13px;
            }
            QSlider::groove:horizontal {
                height: 6px;
                background: #242c36;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #38bdf8;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #f6c16a;
                width: 18px;
                margin-top: -6px;
                margin-bottom: -6px;
                border-radius: 9px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(12)
        layout.setContentsMargins(20, 20, 20, 20)

        # Header
        lbl_title = QLabel("THIẾT LẬP VỊ TRÍ BAN ĐẦU (INITIAL POSE)")
        lbl_title.setStyleSheet("font-size: 16px; font-weight: 800; color: #f8fafc;")
        layout.addWidget(lbl_title)

        lbl_desc = QLabel("AMCL cần tọa độ xuất phát chính xác để thiết lập liên kết TF map -> odom.")
        lbl_desc.setStyleSheet("font-size: 12px; color: #94a3b8;")
        lbl_desc.setWordWrap(True)
        layout.addWidget(lbl_desc)

        # Status row
        status_box = QFrame()
        status_box.setStyleSheet("background: #171c22; border-radius: 8px; padding: 6px 10px;")
        s_layout = QHBoxLayout(status_box)
        s_layout.setContentsMargins(8, 4, 8, 4)
        lbl_st_title = QLabel("Trạng thái hiện tại:")
        lbl_st_title.setStyleSheet("font-size: 12px; font-weight: 600; color: #cbd5e1;")
        self.lbl_curr_status = QLabel(f"{loc_state}: {loc_msg}")
        self.lbl_curr_status.setStyleSheet("font-size: 12px; font-weight: 700; color: #38bdf8;")
        s_layout.addWidget(lbl_st_title)
        s_layout.addWidget(self.lbl_curr_status, 1)
        layout.addWidget(status_box)

        self._start_wp: dict[str, Any] | None = None
        self._current_map_path = current_map_path

        # Option 1: Waypoint 'khoihanh'
        grp_start = QGroupBox("Cách 1: Đặt tại Điểm Xuất Phát (khoihanh)")
        g1_layout = QVBoxLayout(grp_start)
        g1_layout.setSpacing(8)

        self.lbl_wp_info = QLabel("Đang tìm cấu hình waypoint xuất phát...")
        self.lbl_wp_info.setStyleSheet("color: #cbd5e1; font-size: 12px;")
        self.lbl_wp_info.setWordWrap(True)
        g1_layout.addWidget(self.lbl_wp_info)

        self.btn_apply_start = QPushButton("XÁC NHẬN ĐẶT TẠI KHOIHÀNH")
        self.btn_apply_start.setObjectName("primaryActionBtn")
        self.btn_apply_start.clicked.connect(self._on_apply_start_clicked)
        g1_layout.addWidget(self.btn_apply_start)
        layout.addWidget(grp_start)

        # Option 2: Manual Coordinates / Pick on Map
        grp_manual = QGroupBox("Cách 2: Chọn Trên Bản Đồ Hoặc Chỉnh Tọa Độ Thủ Công")
        g2_layout = QVBoxLayout(grp_manual)
        g2_layout.setSpacing(8)

        coord_row = QHBoxLayout()
        coord_row.addWidget(QLabel("X (m):"))
        self.spn_x = QDoubleSpinBox()
        self.spn_x.setRange(-100.0, 100.0)
        self.spn_x.setSingleStep(0.05)
        self.spn_x.setDecimals(3)
        coord_row.addWidget(self.spn_x)

        coord_row.addWidget(QLabel("Y (m):"))
        self.spn_y = QDoubleSpinBox()
        self.spn_y.setRange(-100.0, 100.0)
        self.spn_y.setSingleStep(0.05)
        self.spn_y.setDecimals(3)
        coord_row.addWidget(self.spn_y)
        g2_layout.addLayout(coord_row)

        yaw_title_row = QHBoxLayout()
        yaw_title_row.addWidget(QLabel("Hướng Robot (Yaw):"))
        self.lbl_yaw_val = QLabel("0.0° (0.00 rad)")
        self.lbl_yaw_val.setStyleSheet("color: #38bdf8; font-weight: 700;")
        yaw_title_row.addWidget(self.lbl_yaw_val, 1)
        g2_layout.addLayout(yaw_title_row)

        self.slider_yaw = QSlider(Qt.Horizontal)
        self.slider_yaw.setRange(-180, 180)
        self.slider_yaw.setValue(0)
        self.slider_yaw.valueChanged.connect(self._on_yaw_slider_changed)
        g2_layout.addWidget(self.slider_yaw)

        # Quick Yaw Buttons: 0, 90, 180, -90
        q_row = QHBoxLayout()
        q_row.setSpacing(6)
        for deg, txt in [(0, "0° (Đông)"), (90, "90° (Bắc)"), (180, "180° (Tây)"), (-90, "-90° (Nam)")]:
            btn_q = QPushButton(txt)
            btn_q.setStyleSheet("font-size: 11px; padding: 4px 6px;")
            btn_q.clicked.connect(partial(self.slider_yaw.setValue, deg))
            q_row.addWidget(btn_q)
        g2_layout.addLayout(q_row)

        btns_row = QHBoxLayout()
        self.btn_pick_map = QPushButton("CHỌN TRỰC TIẾP TRÊN MAP")
        self.btn_pick_map.clicked.connect(self._on_pick_map_clicked)
        btns_row.addWidget(self.btn_pick_map)

        self.btn_apply_manual = QPushButton("XÁC NHẬN VỊ TRÍ NÀY")
        self.btn_apply_manual.setObjectName("primaryActionBtn")
        self.btn_apply_manual.clicked.connect(self._on_apply_manual_clicked)
        btns_row.addWidget(self.btn_apply_manual)
        g2_layout.addLayout(btns_row)

        layout.addWidget(grp_manual)

        # Bottom Close Button
        btn_close = QPushButton("ĐÓNG / HỦY")
        btn_close.setObjectName("closeBtn")
        btn_close.clicked.connect(self.reject)
        layout.addWidget(btn_close)

        self._load_khoihanh_waypoint()

    def _load_khoihanh_waypoint(self) -> None:
        if not self._current_map_path:
            self.lbl_wp_info.setText("Chưa có bản đồ nào được nạp vào hệ thống!")
            self.btn_apply_start.setEnabled(False)
            return

        stem = self._current_map_path.stem
        candidates = [
            self._current_map_path.parent / f"{stem}_waypoints.yaml",
            _find_config_dir().parent / "my_map" / f"{stem}_waypoints.yaml",
            _find_config_dir() / "locations.yaml",
        ]
        found = None
        for c in candidates:
            if c.is_file():
                try:
                    with c.open("r", encoding="utf-8") as f:
                        data = yaml.safe_load(f) or {}
                    locs = data.get("locations", {}) if "locations" in data else data
                    if isinstance(locs, dict) and "khoihanh" in locs:
                        kh = locs["khoihanh"]
                        if isinstance(kh, dict) and "x" in kh and "y" in kh:
                            found = {
                                "x": float(kh.get("x", 0.0)),
                                "y": float(kh.get("y", 0.0)),
                                "yaw": float(kh.get("yaw", 0.0)),
                                "note": str(kh.get("note", "Điểm xuất phát")),
                                "file": c.name,
                            }
                            break
                except Exception:
                    pass

        if found:
            self._start_wp = found
            deg = math.degrees(found["yaw"])
            self.lbl_wp_info.setText(
                f"Tọa độ: X={found['x']:.2f} m, Y={found['y']:.2f} m, Yaw={found['yaw']:.2f} rad ({deg:.0f}°)\n"
                f"Ghi chú: {found['note']} (nguồn: {found['file']})"
            )
            self.btn_apply_start.setEnabled(True)
            self.spn_x.setValue(found["x"])
            self.spn_y.setValue(found["y"])
            self.slider_yaw.setValue(int(deg))
        else:
            self.lbl_wp_info.setText(f"Bản đồ '{stem}' chưa cấu hình điểm 'khoihanh'. Vui lòng chọn thủ công.")
            self.btn_apply_start.setEnabled(False)

    def _on_yaw_slider_changed(self, deg: int) -> None:
        rad = math.radians(deg)
        self.lbl_yaw_val.setText(f"{deg:.1f}° ({rad:.2f} rad)")

    def _on_apply_start_clicked(self) -> None:
        if self._start_wp:
            self.pose_confirmed.emit(
                self._start_wp["x"],
                self._start_wp["y"],
                self._start_wp["yaw"],
                "start_waypoint",
            )
            self.accept()

    def _on_apply_manual_clicked(self) -> None:
        x = self.spn_x.value()
        y = self.spn_y.value()
        yaw = math.radians(self.slider_yaw.value())
        self.pose_confirmed.emit(x, y, yaw, "manual_dialog")
        self.accept()

    def _on_pick_map_clicked(self) -> None:
        self.pick_on_map_requested.emit()
        self.accept()


class NavigationPage(QWidget):
    """Full-featured Kiosk Navigation & Patrol Dashboard."""

    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    patrol_toggle_requested = pyqtSignal()
    fullscreen_toggle_requested = pyqtSignal()
    start_requested = pyqtSignal(float, float, float)     # (x, y, yaw)
    cancel_requested = pyqtSignal()
    start_patrol_requested = pyqtSignal()
    pause_patrol_requested = pyqtSignal()
    reload_route_requested = pyqtSignal()
    initial_pose_requested = pyqtSignal(dict)  # {"x": float, "y": float, "yaw": float, "source": str, "map_name": str}
    patrol_route_updated = pyqtSignal(list)

    def __init__(self, icons: dict[str, Path | None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("navigationPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.icons = icons

        self._config_dir = _find_config_dir()
        self._goal: tuple[float, float, float] | None = None
        self._navigation_active = False
        self._batch_waypoints: list[dict[str, Any]] = []
        self._current_map_path: Path | None = None
        self._is_active = True
        self._localization_state = "NOT_INITIALIZED"
        self._localization_message = "Chưa định vị"
        self._last_log_feedback_time = 0.0
        self.patrol_log_text: QTextEdit | None = None
        self._current_robot_pose: dict[str, Any] | None = None
        self._start_wp: dict[str, Any] | None = None
        self._standby_wp: dict[str, Any] | None = None

        self._init_ui()
        default_map = Path("my_map/a2_amcl.yaml")
        if default_map.is_file():
            self.load_map_yaml(default_map)
        else:
            self._load_patrol_from_yaml(silent=True)
            self._update_start_pose_state()

        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._update_clock)
        self._clock_timer.start(1000)
        self._update_clock()

    def _init_ui(self) -> None:
        self.setStyleSheet("""
            QWidget#navigationPage {
                background-color: #000000;
            }
            QFrame#navHeaderBar {
                background: #0f1318;
                border: 1px solid #242c36;
                border-radius: 12px;
            }
            QLabel#navTitle {
                color: #f6c16a;
                font-size: 16px;
                font-weight: 800;
                letter-spacing: 0.5px;
            }
            QLabel#navSubtitle {
                color: #94a3b8;
                font-size: 11px;
                font-weight: 500;
            }
            QLabel#navClock {
                color: #cbd5e1;
                font-size: 12px;
                font-weight: 700;
            }
            QWidget#navHeaderBadge {
                background: #171c22;
                border: 1px solid #242c36;
                border-radius: 8px;
            }
            QLabel#navBadgeText {
                color: #e2e8f0;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton#navFullscreenButton {
                background: #171c22;
                color: #cbd5e1;
                border: 1px solid #242c36;
                border-radius: 8px;
                font-size: 16px;
                font-weight: 700;
            }
            QPushButton#navFullscreenButton:hover {
                background: #242c36;
                border-color: #f6c16a;
                color: #ffffff;
            }
            /* Segmented Mode Switcher */
            QPushButton#navModeTab {
                background: #171c22;
                color: #94a3b8;
                border: 1px solid #242c36;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 700;
                padding: 6px 16px;
                min-height: 28px;
            }
            QPushButton#navModeTab:hover {
                color: #f8fafc;
                background: #242c36;
            }
            QPushButton#navModeTab[active="true"] {
                background: #2563eb;
                color: #ffffff;
                border-color: #60a5fa;
            }
            /* Action Buttons */
            QPushButton#navStartBtn {
                background: #16a34a;
                color: #ffffff;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 800;
                padding: 8px 16px;
            }
            QPushButton#navStartBtn:hover {
                background: #15803d;
            }
            QPushButton#navStopBtn {
                background: #dc2626;
                color: #ffffff;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 800;
                padding: 8px 16px;
            }
            QPushButton#navStopBtn:hover {
                background: #b91c1c;
            }
            QPushButton#navPauseBtn {
                background: #d97706;
                color: #ffffff;
                border-radius: 8px;
                font-size: 13px;
                font-weight: 800;
                padding: 8px 16px;
            }
            QPushButton#navPauseBtn:hover {
                background: #b45309;
            }
            QPushButton#navToolBtn {
                background: #171c22;
                color: #e2e8f0;
                border: 1px solid #242c36;
                border-radius: 6px;
                font-size: 12px;
                font-weight: 700;
                padding: 6px 12px;
            }
            QPushButton#navToolBtn:hover {
                background: #242c36;
                border-color: #f6c16a;
            }
            QPushButton#navToolBtn[toggled_on="true"] {
                background: #2563eb;
                color: #ffffff;
                border-color: #60a5fa;
            }
            QFrame#navDashboardCard {
                background: #0f1318;
                border: 1px solid #242c36;
                border-radius: 10px;
                padding: 6px;
            }
            QTableWidget {
                background: #080808;
                color: #f1f5f9;
                gridline-color: #242c36;
                border: 1px solid #242c36;
                border-radius: 8px;
                font-size: 11px;
            }
            QTableWidget::item:selected {
                background: #2563eb;
                color: #ffffff;
            }
            QHeaderView::section {
                background: #171c22;
                color: #cbd5e1;
                font-size: 11px;
                font-weight: 700;
                border: none;
                padding: 4px;
            }
        """)

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(8)

        # 1. Header Bar
        root.addLayout(self._create_header())

        # 2. Segmented Mode Switcher (2 Tabs)
        root.addLayout(self._create_mode_switcher())

        # 3. Stacked Pages
        self.stack = QStackedWidget()
        self.tab_single_nav = self._create_single_nav_page()
        self.tab_patrol_route = self._create_patrol_route_page()
        self.stack.addWidget(self.tab_single_nav)
        self.stack.addWidget(self.tab_patrol_route)
        root.addWidget(self.stack, 1)

    def _create_header(self) -> QHBoxLayout:
        header_bar = QFrame()
        header_bar.setObjectName("navHeaderBar")
        bar_layout = QHBoxLayout(header_bar)
        bar_layout.setContentsMargins(10, 6, 12, 6)
        bar_layout.setSpacing(10)

        # Home Button
        self.home_button = QPushButton()
        self.home_button.setObjectName("navHomeButton")
        self.home_button.setCursor(Qt.PointingHandCursor)
        self.home_button.setFixedSize(44, 44)
        self.home_button.setToolTip("Về màn hình chính")
        home_icon = self.icons.get("home")
        if home_icon and Path(home_icon).is_file():
            self.home_button.setIcon(QIcon(str(home_icon)))
            self.home_button.setIconSize(QSize(26, 26))
        else:
            self.home_button.setText("Home")
            self.home_button.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700;")
        self.home_button.clicked.connect(self.home_requested.emit)
        bar_layout.addWidget(self.home_button)

        # Emergency Stop Button
        self.emergency_button = QPushButton()
        self.emergency_button.setObjectName("navEmergencyButton")
        self.emergency_button.setCursor(Qt.PointingHandCursor)
        self.emergency_button.setFixedSize(44, 44)
        self.emergency_button.setToolTip("DỪNG KHẨN CẤP")
        emg_icon = self.icons.get("emergency")
        if emg_icon and Path(emg_icon).is_file():
            self.emergency_button.setIcon(QIcon(str(emg_icon)))
            self.emergency_button.setIconSize(QSize(34, 34))
        else:
            self.emergency_button.setText("DỪNG")
            self.emergency_button.setStyleSheet("color: #ef4444; font-size: 11px; font-weight: 700;")
        self.emergency_button.clicked.connect(self.emergency_stop_requested.emit)
        bar_layout.addWidget(self.emergency_button)

        # Patrol Toggle Button (Quick Patrol Control)
        self.patrol_button = QPushButton()
        self.patrol_button.setObjectName("navPatrolButton")
        self.patrol_button.setCursor(Qt.PointingHandCursor)
        self.patrol_button.setFixedSize(44, 44)
        self.patrol_button.setToolTip("Bắt đầu tuần tra")
        start_icon = self.icons.get("start")
        if start_icon and Path(start_icon).is_file():
            self.patrol_button.setIcon(QIcon(str(start_icon)))
            self.patrol_button.setIconSize(QSize(34, 34))
        else:
            self.patrol_button.setText("▶")
            self.patrol_button.setStyleSheet("color: #10b981; font-size: 16px; font-weight: 700;")
        self.patrol_button.clicked.connect(self.patrol_toggle_requested.emit)
        bar_layout.addWidget(self.patrol_button)

        # Back Button
        self.back_button = QPushButton()
        self.back_button.setObjectName("navBackButton")
        self.back_button.setCursor(Qt.PointingHandCursor)
        self.back_button.setFixedSize(44, 44)
        self.back_button.setToolTip("Quay lại")
        back_icon = self.icons.get("back")
        if back_icon and Path(back_icon).is_file():
            self.back_button.setIcon(QIcon(str(back_icon)))
            self.back_button.setIconSize(QSize(28, 28))
        else:
            self.back_button.setText("‹")
            self.back_button.setStyleSheet("color: #f6c16a; font-size: 20px; font-weight: 700;")
        self.back_button.clicked.connect(self.back_requested.emit)
        bar_layout.addWidget(self.back_button)

        # Title Block
        title_box = QVBoxLayout()
        title_box.setSpacing(2)
        lbl_title = QLabel("TRUNG TÂM ĐIỀU HƯỚNG & TUẦN TRA")
        lbl_title.setObjectName("navTitle")
        lbl_sub = QLabel("Giám sát tiến trình thời gian thực, thiết lập lộ trình và điều khiển Nav2")
        lbl_sub.setObjectName("navSubtitle")
        title_box.addWidget(lbl_title)
        title_box.addWidget(lbl_sub)
        bar_layout.addLayout(title_box)

        bar_layout.addStretch(1)

        # Status Bar on Top Right (Wifi, Battery, Fullscreen, Clock)
        status_bar = QHBoxLayout()
        status_bar.setSpacing(10)

        # Wifi Badge
        self.wifi_widget = QWidget()
        self.wifi_widget.setObjectName("navHeaderBadge")
        wifi_layout = QHBoxLayout(self.wifi_widget)
        wifi_layout.setContentsMargins(8, 4, 10, 4)
        wifi_layout.setSpacing(6)
        self.wifi_icon_label = QLabel()
        self.wifi_icon_label.setFixedSize(22, 22)
        self.wifi_icon_label.setAlignment(Qt.AlignCenter)
        self.wifi_text_label = QLabel("Wifi")
        self.wifi_text_label.setObjectName("navBadgeText")
        wifi_layout.addWidget(self.wifi_icon_label)
        wifi_layout.addWidget(self.wifi_text_label)
        status_bar.addWidget(self.wifi_widget)

        # Battery Badge
        self.battery_widget = QWidget()
        self.battery_widget.setObjectName("navHeaderBadge")
        battery_layout = QHBoxLayout(self.battery_widget)
        battery_layout.setContentsMargins(8, 4, 10, 4)
        battery_layout.setSpacing(6)
        self.battery_icon_label = QLabel()
        self.battery_icon_label.setFixedSize(22, 22)
        self.battery_icon_label.setAlignment(Qt.AlignCenter)
        self.battery_text_label = QLabel("--%")
        self.battery_text_label.setObjectName("navBadgeText")
        battery_layout.addWidget(self.battery_icon_label)
        battery_layout.addWidget(self.battery_text_label)
        status_bar.addWidget(self.battery_widget)

        # Fullscreen Toggle Button
        self.fullscreen_button = QPushButton()
        self.fullscreen_button.setObjectName("navFullscreenButton")
        self.fullscreen_button.setCursor(Qt.PointingHandCursor)
        self.fullscreen_button.setFixedSize(40, 40)
        self.fullscreen_button.clicked.connect(self.fullscreen_toggle_requested.emit)
        self.update_fullscreen_button(is_fullscreen=True)
        status_bar.addWidget(self.fullscreen_button)

        # Clock
        self.time_label = QLabel()
        self.time_label.setObjectName("navClock")
        self.time_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        status_bar.addWidget(self.time_label)

        bar_layout.addLayout(status_bar)

        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(header_bar)
        return layout

    def _create_mode_switcher(self) -> QHBoxLayout:
        bar = QHBoxLayout()
        bar.setSpacing(8)

        self.btn_tab_single = QPushButton("ĐIỀU HƯỚNG ĐIỂM ĐƠN")
        self.btn_tab_single.setObjectName("navModeTab")
        self.btn_tab_single.setProperty("active", "true")
        self.btn_tab_single.clicked.connect(lambda: self._set_active_mode(0))

        self.btn_tab_patrol = QPushButton("LỘ TRÌNH & TUẦN TRA")
        self.btn_tab_patrol.setObjectName("navModeTab")
        self.btn_tab_patrol.setProperty("active", "false")
        self.btn_tab_patrol.clicked.connect(lambda: self._set_active_mode(1))

        bar.addWidget(self.btn_tab_single)
        bar.addWidget(self.btn_tab_patrol)
        bar.addStretch(1)
        return bar

    def _set_active_mode(self, index: int) -> None:
        self.stack.setCurrentIndex(index)
        self.btn_tab_single.setProperty("active", "true" if index == 0 else "false")
        self.btn_tab_patrol.setProperty("active", "true" if index == 1 else "false")
        self.btn_tab_single.style().unpolish(self.btn_tab_single)
        self.btn_tab_single.style().polish(self.btn_tab_single)
        self.btn_tab_patrol.style().unpolish(self.btn_tab_patrol)
        self.btn_tab_patrol.style().polish(self.btn_tab_patrol)

        # Kích hoạt chỉ widget bản đồ của tab hiển thị để tối ưu hóa render
        page_active = getattr(self, "_is_active", True)
        self.map_widget.set_active(page_active and index == 0)
        self.patrol_map_widget.set_active(page_active and index == 1)

        # Sync batch waypoints to map
        if index == 0:
            self.map_widget.set_pick_mode(False)
            self.map_widget.set_batch_waypoints(self._batch_waypoints)
        else:
            self.patrol_map_widget.set_batch_waypoints(self._batch_waypoints)

    # -------------------------------------------------------------------------
    # TAB 1: Single Nav Page
    # -------------------------------------------------------------------------
    def _create_single_nav_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(6)

        # Quick Location Chips Toolbar
        chips_bar = QHBoxLayout()
        chips_bar.setSpacing(6)
        lbl_chips = QLabel("Điểm nhanh:")
        lbl_chips.setStyleSheet("color: #94a3b8; font-size: 11px; font-weight: 700;")
        chips_bar.addWidget(lbl_chips)

        self.chips_layout = QHBoxLayout()
        self.chips_layout.setSpacing(6)
        chips_bar.addLayout(self.chips_layout)
        chips_bar.addStretch(1)

        # Nút Chụp & Khôi phục vị trí gốc (khoihanh) cho bản đồ
        self.btn_capture_start_pose = QPushButton("📸 LƯU ĐIỂM GỐC")
        self.btn_capture_start_pose.setObjectName("navToolBtn")
        self.btn_capture_start_pose.setCursor(Qt.PointingHandCursor)
        self.btn_capture_start_pose.setStyleSheet("""
            QPushButton {
                background: #1e293b;
                color: #38bdf8;
                border: 1px solid #0284c7;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #0284c7;
                color: #ffffff;
            }
        """)
        self.btn_capture_start_pose.setToolTip(
            "Lưu vị trí cất máy hiện tại của robot làm Điểm Xuất Phát (khoihanh) cho bản đồ này"
        )
        self.btn_capture_start_pose.clicked.connect(self._capture_current_pose_as_start)
        chips_bar.addWidget(self.btn_capture_start_pose)

        self.btn_relocalize_start_pose = QPushButton("🎯 VỀ ĐIỂM GỐC")
        self.btn_relocalize_start_pose.setObjectName("navToolBtn")
        self.btn_relocalize_start_pose.setCursor(Qt.PointingHandCursor)
        self.btn_relocalize_start_pose.setStyleSheet("""
            QPushButton {
                background: #064e3b;
                color: #34d399;
                border: 1px solid #059669;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #059669;
                color: #ffffff;
            }
        """)
        self.btn_relocalize_start_pose.setToolTip(
            "Phát /initialpose khôi phục tức thì định vị AMCL về điểm cất máy đã lưu (không lái xe)"
        )
        self.btn_relocalize_start_pose.clicked.connect(self._relocalize_to_start_pose)
        chips_bar.addWidget(self.btn_relocalize_start_pose)

        # Nút Trạm Chờ (tram_cho) thông thoáng an toàn cho Nav2
        self.btn_capture_standby_pose = QPushButton("📸 LƯU TRẠM CHỜ")
        self.btn_capture_standby_pose.setObjectName("navToolBtn")
        self.btn_capture_standby_pose.setCursor(Qt.PointingHandCursor)
        self.btn_capture_standby_pose.setStyleSheet("""
            QPushButton {
                background: #312e81;
                color: #a5b4fc;
                border: 1px solid #4f46e5;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #4f46e5;
                color: #ffffff;
            }
        """)
        self.btn_capture_standby_pose.setToolTip(
            "Lưu vị trí thông thoáng hiện tại của robot làm Trạm Chờ (tram_cho) cho bản đồ này"
        )
        self.btn_capture_standby_pose.clicked.connect(self._capture_current_pose_as_standby)
        chips_bar.addWidget(self.btn_capture_standby_pose)

        self.btn_navigate_standby_pose = QPushButton("🚏 VỀ TRẠM CHỜ")
        self.btn_navigate_standby_pose.setObjectName("navToolBtn")
        self.btn_navigate_standby_pose.setCursor(Qt.PointingHandCursor)
        self.btn_navigate_standby_pose.setStyleSheet("""
            QPushButton {
                background: #065f46;
                color: #6ee7b7;
                border: 1px solid #059669;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #059669;
                color: #ffffff;
            }
        """)
        self.btn_navigate_standby_pose.setToolTip(
            "Nav2 tự động lái robot về Trạm Chờ an toàn ngoài tường"
        )
        self.btn_navigate_standby_pose.clicked.connect(self._navigate_to_standby_pose)
        chips_bar.addWidget(self.btn_navigate_standby_pose)

        layout.addLayout(chips_bar)

        # Live Map
        self.map_widget = InteractiveNavMapWidget(self.icons.get("goal"))
        self.map_widget.goal_selected.connect(self._on_single_goal_selected)
        self.map_widget.invalid_goal.connect(self._show_warning)
        self.map_widget.initial_pose_picked.connect(self._on_map_initial_pose_picked)
        layout.addWidget(self.map_widget, 1)

        # Bottom Progress & Control Card
        dashboard = QFrame()
        dashboard.setObjectName("navDashboardCard")
        dash_layout = QVBoxLayout(dashboard)
        dash_layout.setContentsMargins(10, 6, 10, 6)
        dash_layout.setSpacing(5)

        # Row 1: Progress Bar
        pbar_row = QHBoxLayout()
        pbar_row.setSpacing(8)
        lbl_pbar_title = QLabel("Tiến độ hành trình:")
        lbl_pbar_title.setStyleSheet("color: #cbd5e1; font-size: 12px; font-weight: 700;")

        self.nav_progress_bar = QProgressBar()
        self.nav_progress_bar.setRange(0, 100)
        self.nav_progress_bar.setValue(0)
        self.nav_progress_bar.setTextVisible(True)
        self.nav_progress_bar.setFormat("Tiến độ: %p%")
        self.nav_progress_bar.setFixedHeight(20)
        self.nav_progress_bar.setStyleSheet("""
            QProgressBar {
                border: 1px solid #242c36;
                border-radius: 6px;
                text-align: center;
                background-color: #080808;
                color: #ffffff;
                font-weight: 800;
                font-size: 11px;
            }
            QProgressBar::chunk {
                background-color: #22c55e;
                border-radius: 5px;
            }
        """)
        pbar_row.addWidget(lbl_pbar_title)
        pbar_row.addWidget(self.nav_progress_bar, 1)
        dash_layout.addLayout(pbar_row)

        # Row 2: Metrics chips
        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(10)
        self.lbl_dist_remain = QLabel("Cự ly còn lại: -- m")
        self.lbl_dist_remain.setStyleSheet("color: #38bdf8; font-size: 12px; font-weight: 700;")

        self.lbl_nav_time = QLabel("Thời gian chạy: 0.0 s")
        self.lbl_nav_time.setStyleSheet("color: #cbd5e1; font-size: 12px; font-weight: 600;")

        self.lbl_est_time = QLabel("Ước tính: -- s")
        self.lbl_est_time.setStyleSheet("color: #cbd5e1; font-size: 12px; font-weight: 600;")

        self.lbl_recovery_count = QLabel("Tự cứu: 0 lần")
        self.lbl_recovery_count.setStyleSheet("color: #f59e0b; font-size: 12px; font-weight: 700;")

        metrics_row.addWidget(self.lbl_dist_remain)
        metrics_row.addWidget(self.lbl_nav_time)
        metrics_row.addWidget(self.lbl_est_time)
        metrics_row.addWidget(self.lbl_recovery_count)
        metrics_row.addStretch(1)
        dash_layout.addLayout(metrics_row)

        # Row 3: Action Buttons & Goal Prompt
        action_row = QHBoxLayout()
        action_row.setSpacing(10)
        self.status_label = QLabel("Chạm bản đồ hoặc chọn điểm nhanh để bắt đầu di chuyển")
        self.status_label.setStyleSheet("color: #f1f5f9; font-size: 12px; font-weight: 600;")
        self.status_label.setWordWrap(True)

        self.start_button = QPushButton("BẮT ĐẦU ĐI")
        self.start_button.setObjectName("navStartBtn")
        start_icon = self.icons.get("start")
        if start_icon and Path(start_icon).is_file():
            self.start_button.setIcon(QIcon(str(start_icon)))
            self.start_button.setIconSize(QSize(24, 24))
        self.start_button.clicked.connect(self._start_single_navigation)

        self.stop_button = QPushButton("HỦY / DỪNG")
        self.stop_button.setObjectName("navStopBtn")
        stop_icon = self.icons.get("stop")
        if stop_icon and Path(stop_icon).is_file():
            self.stop_button.setIcon(QIcon(str(stop_icon)))
            self.stop_button.setIconSize(QSize(24, 24))
        self.stop_button.clicked.connect(self._stop_navigation)

        self.lbl_localization_badge = QLabel("● CHƯA ĐỊNH VỊ")
        self.lbl_localization_badge.setObjectName("navLocBadge")
        self.lbl_localization_badge.setStyleSheet(
            "background: rgba(239, 68, 68, 0.2); color: #ef4444; "
            "border: 1px solid #ef4444; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
        )

        self.btn_initial_pose = QPushButton("2D POSE ESTIMATE")
        self.btn_initial_pose.setObjectName("navToolBtn")
        self.btn_initial_pose.setToolTip("Kích hoạt chọn vị trí và hướng robot (2D Pose Estimate) trực tiếp trên bản đồ")
        self.btn_initial_pose.clicked.connect(self._toggle_2d_estimate)
        self.btn_2d_estimate = self.btn_initial_pose

        self.btn_quick_relocalize = QPushButton("🎯 VỀ ĐIỂM GỐC")
        self.btn_quick_relocalize.setObjectName("navToolBtn")
        self.btn_quick_relocalize.setCursor(Qt.PointingHandCursor)
        self.btn_quick_relocalize.setStyleSheet("""
            QPushButton {
                background: #064e3b;
                color: #34d399;
                border: 1px solid #059669;
                border-radius: 6px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 700;
            }
            QPushButton:hover {
                background: #059669;
                color: #ffffff;
            }
        """)
        self.btn_quick_relocalize.setToolTip("Khôi phục định vị AMCL về điểm xuất phát đã lưu của bản đồ này")
        self.btn_quick_relocalize.clicked.connect(self._relocalize_to_start_pose)

        action_row.addWidget(self.status_label, 1)
        action_row.addWidget(self.lbl_localization_badge)
        action_row.addWidget(self.btn_initial_pose)
        action_row.addWidget(self.btn_quick_relocalize)
        action_row.addWidget(self.start_button)
        action_row.addWidget(self.stop_button)
        dash_layout.addLayout(action_row)

        layout.addWidget(dashboard)
        self._set_button_strength(self.stop_button, False)
        return page

    # -------------------------------------------------------------------------
    # TAB 2: Patrol Route Page
    # -------------------------------------------------------------------------
    def _create_patrol_route_page(self) -> QWidget:
        page = QWidget()
        root_layout = QHBoxLayout(page)
        root_layout.setContentsMargins(0, 4, 0, 0)
        root_layout.setSpacing(10)

        # Left Column: Map & Batch Picking Controls
        left_box = QVBoxLayout()
        left_box.setSpacing(6)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(6)

        self.btn_pick_mode = QPushButton("Chấm điểm trên Map: OFF")
        self.btn_pick_mode.setObjectName("navToolBtn")
        self.btn_pick_mode.clicked.connect(self._toggle_batch_pick_mode)

        self.btn_undo_wp = QPushButton("Hoàn tác")
        self.btn_undo_wp.setObjectName("navToolBtn")
        self.btn_undo_wp.clicked.connect(self._undo_batch_waypoint)

        self.btn_clear_wp = QPushButton("Xóa hết")
        self.btn_clear_wp.setObjectName("navToolBtn")
        self.btn_clear_wp.clicked.connect(self._clear_batch_waypoints)

        self.btn_patrol_capture_start = QPushButton("📸 Điểm gốc")
        self.btn_patrol_capture_start.setObjectName("navToolBtn")
        self.btn_patrol_capture_start.setToolTip("Lưu vị trí cất máy hiện tại của robot làm điểm xuất phát (khoihanh)")
        self.btn_patrol_capture_start.clicked.connect(self._capture_current_pose_as_start)

        self.btn_patrol_relocalize = QPushButton("🎯 Định vị gốc")
        self.btn_patrol_relocalize.setObjectName("navToolBtn")
        self.btn_patrol_relocalize.setToolTip("Phát /initialpose khôi phục định vị AMCL về điểm xuất phát đã lưu (không lái xe)")
        self.btn_patrol_relocalize.clicked.connect(self._relocalize_to_start_pose)

        self.btn_patrol_capture_standby = QPushButton("📸 Lưu trạm chờ")
        self.btn_patrol_capture_standby.setObjectName("navToolBtn")
        self.btn_patrol_capture_standby.setToolTip("Lưu vị trí thông thoáng hiện tại của robot làm Trạm Chờ (tram_cho)")
        self.btn_patrol_capture_standby.clicked.connect(self._capture_current_pose_as_standby)

        self.btn_patrol_navigate_standby = QPushButton("🚏 Về trạm chờ")
        self.btn_patrol_navigate_standby.setObjectName("navToolBtn")
        self.btn_patrol_navigate_standby.setToolTip("Nav2 tự động lái robot về Trạm Chờ thông thoáng ngoài tường")
        self.btn_patrol_navigate_standby.clicked.connect(self._navigate_to_standby_pose)

        toolbar.addWidget(self.btn_pick_mode)
        toolbar.addWidget(self.btn_undo_wp)
        toolbar.addWidget(self.btn_clear_wp)
        toolbar.addStretch(1)
        toolbar.addWidget(self.btn_patrol_capture_start)
        toolbar.addWidget(self.btn_patrol_relocalize)
        toolbar.addWidget(self.btn_patrol_capture_standby)
        toolbar.addWidget(self.btn_patrol_navigate_standby)
        left_box.addLayout(toolbar)

        self.patrol_map_widget = InteractiveNavMapWidget(self.icons.get("goal"))
        self.patrol_map_widget.waypoint_picked.connect(self._on_waypoint_picked_on_map)
        left_box.addWidget(self.patrol_map_widget, 1)

        lbl_hint = QLabel("Bật 'Chấm điểm trên Map: ON' rồi chạm lên bản đồ để thêm lần lượt khoihanh -> diem1 -> diem2...")
        lbl_hint.setStyleSheet("color: #64748b; font-size: 11px;")
        left_box.addWidget(lbl_hint)

        root_layout.addLayout(left_box, 3)

        # Right Column: Mission Control & Waypoint Table
        right_box = QVBoxLayout()
        right_box.setSpacing(8)

        # Mission Controls Card
        mission_card = QFrame()
        mission_card.setObjectName("navDashboardCard")
        m_layout = QVBoxLayout(mission_card)
        m_layout.setContentsMargins(10, 8, 10, 8)
        m_layout.setSpacing(6)

        lbl_m_title = QLabel("ĐIỀU KHIỂN NHIỆM VỤ TUẦN TRA")
        lbl_m_title.setStyleSheet("color: #f6c16a; font-size: 12px; font-weight: 800;")
        m_layout.addWidget(lbl_m_title)

        m_btn_row = QHBoxLayout()
        m_btn_row.setSpacing(8)

        self.btn_start_patrol = QPushButton("BẮT ĐẦU TUẦN TRA")
        self.btn_start_patrol.setObjectName("navStartBtn")
        start_icon = self.icons.get("start")
        if start_icon and Path(start_icon).is_file():
            self.btn_start_patrol.setIcon(QIcon(str(start_icon)))
            self.btn_start_patrol.setIconSize(QSize(22, 22))
        self.btn_start_patrol.clicked.connect(self._start_patrol)

        self.btn_pause_patrol = QPushButton("TẠM DỪNG TUẦN TRA")
        self.btn_pause_patrol.setObjectName("navPauseBtn")
        stop_icon = self.icons.get("stop")
        if stop_icon and Path(stop_icon).is_file():
            self.btn_pause_patrol.setIcon(QIcon(str(stop_icon)))
            self.btn_pause_patrol.setIconSize(QSize(22, 22))
        self.btn_pause_patrol.clicked.connect(self._pause_patrol)

        m_btn_row.addWidget(self.btn_start_patrol)
        m_btn_row.addWidget(self.btn_pause_patrol)
        m_layout.addLayout(m_btn_row)

        self.lbl_route_summary = QLabel("Lộ trình: Chưa có điểm nào")
        self.lbl_route_summary.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: 700;")
        self.lbl_route_summary.setWordWrap(True)
        m_layout.addWidget(self.lbl_route_summary)

        right_box.addWidget(mission_card)

        # Waypoint Table Card
        table_card = QFrame()
        table_card.setObjectName("navDashboardCard")
        t_layout = QVBoxLayout(table_card)
        t_layout.setContentsMargins(10, 8, 10, 8)
        t_layout.setSpacing(6)

        t_head = QHBoxLayout()
        lbl_t_title = QLabel("Danh sách điểm tuần tra (sửa trực tiếp):")
        lbl_t_title.setStyleSheet("color: #cbd5e1; font-size: 11px; font-weight: 700;")
        t_head.addWidget(lbl_t_title)
        t_head.addStretch(1)

        self.btn_load_yaml = QPushButton("Nạp Config")
        self.btn_load_yaml.setObjectName("navToolBtn")
        self.btn_load_yaml.clicked.connect(lambda: self._load_patrol_from_yaml(silent=False))

        self.btn_save_yaml = QPushButton("LƯU & ÁP DỤNG")
        self.btn_save_yaml.setStyleSheet("""
            QPushButton {
                background: #2563eb;
                color: #ffffff;
                border-radius: 6px;
                font-size: 12px;
                font-weight: 800;
                padding: 6px 12px;
            }
            QPushButton:hover { background: #1d4ed8; }
        """)
        self.btn_save_yaml.clicked.connect(self._save_patrol_to_yaml)

        t_head.addWidget(self.btn_load_yaml)
        t_head.addWidget(self.btn_save_yaml)
        t_layout.addLayout(t_head)

        self.table_waypoints = QTableWidget(0, 4)
        self.table_waypoints.setHorizontalHeaderLabels(["Tên điểm", "X (m)", "Y (m)", "Ghi chú"])
        self.table_waypoints.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_waypoints.verticalHeader().setVisible(False)
        self.table_waypoints.cellClicked.connect(self._on_waypoint_cell_clicked)
        t_layout.addWidget(self.table_waypoints, 3)

        # Log Console under waypoint table
        log_head = QHBoxLayout()
        lbl_log_title = QLabel("[NHẬT KÝ ĐIỀU HƯỚNG & TUẦN TRA]")
        lbl_log_title.setStyleSheet("color: #38bdf8; font-size: 11px; font-weight: 800;")
        log_head.addWidget(lbl_log_title)
        log_head.addStretch(1)

        self.btn_clear_patrol_log = QPushButton("Xóa Log")
        self.btn_clear_patrol_log.setStyleSheet("""
            QPushButton {
                background: #171c22;
                color: #94a3b8;
                border: 1px solid #242c36;
                border-radius: 4px;
                font-size: 10px;
                font-weight: 700;
                padding: 2px 8px;
            }
            QPushButton:hover {
                background: #242c36;
                color: #ffffff;
            }
        """)
        self.btn_clear_patrol_log.clicked.connect(self._clear_patrol_log)
        log_head.addWidget(self.btn_clear_patrol_log)
        t_layout.addLayout(log_head)

        self.patrol_log_text = QTextEdit()
        self.patrol_log_text.setReadOnly(True)
        self.patrol_log_text.setStyleSheet("""
            QTextEdit {
                background-color: #0b0f14;
                color: #e2e8f0;
                border: 1px solid #242c36;
                border-radius: 6px;
                font-family: 'DejaVu Sans Mono', 'Ubuntu Mono', 'Monospace', monospace;
                font-size: 11px;
                line-height: 1.4;
                padding: 6px;
            }
        """)
        t_layout.addWidget(self.patrol_log_text, 2)

        right_box.addWidget(table_card, 1)
        root_layout.addLayout(right_box, 2)
        return page

    # -------------------------------------------------------------------------
    # Quick Location Chips
    # -------------------------------------------------------------------------
    def _refresh_quick_location_chips(self) -> None:
        """Clear and repopulate quick location buttons from current waypoints."""
        while self.chips_layout.count():
            item = self.chips_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

        for wp in self._batch_waypoints:
            name = str(wp.get("name", ""))
            x = float(wp.get("x", 0.0))
            y = float(wp.get("y", 0.0))
            yaw = float(wp.get("yaw", 0.0))

            btn = QPushButton(name)
            btn.setStyleSheet("""
                QPushButton {
                    background: #171c22;
                    color: #e2e8f0;
                    border: 1px solid #242c36;
                    border-radius: 6px;
                    font-size: 11px;
                    font-weight: 700;
                    padding: 4px 10px;
                }
                QPushButton:hover {
                    background: #242c36;
                    border-color: #f6c16a;
                    color: #ffffff;
                }
            """)
            btn.clicked.connect(
                lambda checked=False, gx=x, gy=y, gyaw=yaw, gname=name:
                self._on_quick_chip_clicked(gx, gy, gyaw, gname)
            )
            self.chips_layout.addWidget(btn)

    def _on_quick_chip_clicked(self, x: float, y: float, yaw: float, name: str) -> None:
        self._goal = (x, y, yaw)
        self.map_widget.set_goal(x, y, yaw)
        self.status_label.setText(f"Điểm chọn: {name} (X={x:.2f}, Y={y:.2f} m)")
        self._set_button_strength(self.start_button, True)

    # -------------------------------------------------------------------------
    # Batch Waypoint Picking Actions
    # -------------------------------------------------------------------------
    def _toggle_batch_pick_mode(self) -> None:
        enabled = not self.patrol_map_widget._pick_mode
        self.patrol_map_widget.set_pick_mode(enabled)
        self.btn_pick_mode.setProperty("toggled_on", "true" if enabled else "false")
        self.btn_pick_mode.setText(f"Chấm điểm trên Map: {'ON' if enabled else 'OFF'}")
        self.btn_pick_mode.style().unpolish(self.btn_pick_mode)
        self.btn_pick_mode.style().polish(self.btn_pick_mode)
        if enabled:
            self.append_patrol_log("[WAYPOINT][INFO] Đã bật chế độ chấm điểm tuần tra (Chạm lên map để thêm điểm).", "progress")
        else:
            self.append_patrol_log("[WAYPOINT][INFO] Đã tắt chế độ chấm điểm trên bản đồ.", "info")

    def _on_waypoint_picked_on_map(self, x: float, y: float, yaw: float) -> None:
        idx = len(self._batch_waypoints)
        name = waypoint_name_from_index(idx)
        note = "Khởi hành / Trạm nước" if idx == 0 else f"Điểm tuần tra {idx}"
        new_wp = {
            "name": name,
            "x": round(x, 3),
            "y": round(y, 3),
            "yaw": round(yaw, 3),
            "note": note,
        }
        self._batch_waypoints.append(new_wp)
        recompute_cyclic_waypoint_yaws(self._batch_waypoints)
        self._sync_views_from_waypoints()
        self.append_patrol_log(f"[WAYPOINT][INFO] Đã thêm điểm '{name}': X={x:.3f}, Y={y:.3f} m", "success")

    def _undo_batch_waypoint(self) -> None:
        if self._batch_waypoints:
            popped = self._batch_waypoints.pop()
            recompute_cyclic_waypoint_yaws(self._batch_waypoints)
            self._sync_views_from_waypoints()
            p_name = popped.get("name", "gần nhất")
            self.append_patrol_log(f"[WAYPOINT][WARN] Đã xóa điểm '{p_name}'.", "warning")

    def _clear_batch_waypoints(self) -> None:
        self._batch_waypoints.clear()
        self._sync_views_from_waypoints()
        self.append_patrol_log("[WAYPOINT][WARN] Đã xóa toàn bộ danh sách điểm tuần tra.", "warning")

    def _on_waypoint_cell_clicked(self, row: int, column: int) -> None:
        """Xử lý khi người dùng chạm vào một hàng waypoint trong bảng."""
        if 0 <= row < len(self._batch_waypoints):
            wp = self._batch_waypoints[row]
            name = str(wp.get("name", f"point_{row}"))
            x = float(wp.get("x", 0.0))
            y = float(wp.get("y", 0.0))
            yaw = float(wp.get("yaw", 0.0))
            self._goal = (x, y, yaw)
            self.map_widget.set_goal(x, y, yaw)
            self._set_button_strength(self.start_button, True)
            self.append_patrol_log(
                f"[WAYPOINT][INFO] Đã chọn waypoint '{name}' trên bảng: X={x:.3f}, Y={y:.3f}",
                "info"
            )

    def append_patrol_log(self, text: str, tone: str = "info") -> None:
        """Ghi dòng nhật ký mới vào khung log tuần tra với timestamp và mã màu."""
        if not hasattr(self, "patrol_log_text") or self.patrol_log_text is None:
            return
        t_str = datetime.now().strftime("%H:%M:%S")
        color_map = {
            "success": "#10b981",  # Green
            "error": "#ef4444",    # Red
            "warning": "#f59e0b",  # Amber/Yellow
            "progress": "#38bdf8", # Sky blue
            "info": "#94a3b8",     # Muted slate
        }
        color = color_map.get(tone, "#94a3b8")
        escaped_text = html.escape(text)
        html_line = f"<span style='color: #64748b;'>[{t_str}]</span> <span style='color: {color}; font-weight: 600;'>{escaped_text}</span>"
        self.patrol_log_text.append(html_line)
        self.patrol_log_text.moveCursor(QTextCursor.End)

    def _clear_patrol_log(self) -> None:
        """Xóa trắng nội dung khung log tuần tra."""
        if hasattr(self, "patrol_log_text") and self.patrol_log_text is not None:
            self.patrol_log_text.clear()
            self.append_patrol_log("[INFO] Đã xóa nhật ký điều hướng.", "info")

    def _sync_views_from_waypoints(self) -> None:
        """Update both maps, table, and summary label from current waypoints."""
        self.map_widget.set_batch_waypoints(self._batch_waypoints)
        self.patrol_map_widget.set_batch_waypoints(self._batch_waypoints)

        # Update table
        self.table_waypoints.setRowCount(len(self._batch_waypoints))
        for r, wp in enumerate(self._batch_waypoints):
            self.table_waypoints.setItem(r, 0, QTableWidgetItem(str(wp.get("name", ""))))
            self.table_waypoints.setItem(r, 1, QTableWidgetItem(f"{float(wp.get('x', 0.0)):.3f}"))
            self.table_waypoints.setItem(r, 2, QTableWidgetItem(f"{float(wp.get('y', 0.0)):.3f}"))
            self.table_waypoints.setItem(r, 3, QTableWidgetItem(str(wp.get("note", ""))))

        # Update summary label
        names = [str(w.get("name", "")) for w in self._batch_waypoints]
        if names:
            cycle = " -> ".join(names + [names[0]])
            self.lbl_route_summary.setText(f"Lộ trình ({len(names)} điểm): {cycle}")
        else:
            self.lbl_route_summary.setText("Lộ trình: Chưa có điểm nào")

        self.patrol_route_updated.emit(list(self._batch_waypoints))

        self._refresh_quick_location_chips()

    def _sync_waypoints_from_table(self) -> None:
        """Extract user edits from table into _batch_waypoints."""
        rows = self.table_waypoints.rowCount()
        if rows == 0 and self._batch_waypoints:
            return
        updated: list[dict[str, Any]] = []
        for r in range(rows):
            item_name = self.table_waypoints.item(r, 0)
            item_x = self.table_waypoints.item(r, 1)
            item_y = self.table_waypoints.item(r, 2)
            item_note = self.table_waypoints.item(r, 3)

            name = (item_name.text().strip() if item_name else "") or waypoint_name_from_index(r)
            prev_wp = self._batch_waypoints[r] if r < len(self._batch_waypoints) else {}
            try:
                x = float(item_x.text().strip()) if item_x else float(prev_wp.get("x", 0.0))
                y = float(item_y.text().strip()) if item_y else float(prev_wp.get("y", 0.0))
            except ValueError:
                x = float(prev_wp.get("x", 0.0))
                y = float(prev_wp.get("y", 0.0))
            note = item_note.text().strip() if item_note else prev_wp.get("note", "")

            updated.append({
                "name": name,
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw": float(prev_wp.get("yaw", 0.0)),
                "note": note,
            })
        self._batch_waypoints = updated
        recompute_cyclic_waypoint_yaws(self._batch_waypoints)

    # -------------------------------------------------------------------------
    # Per-Map YAML Waypoint & Configuration Sync
    # -------------------------------------------------------------------------
    def _get_map_waypoints_path(self, map_path: Path | str | None) -> Path | None:
        """Find or determine the dedicated waypoints YAML file associated with a map."""
        if not map_path:
            return None
        p = Path(map_path)
        stem = p.stem

        # Priority 1: In the same directory as the map, e.g. <map_dir>/<map_stem>_waypoints.yaml
        direct = p.parent / f"{stem}_waypoints.yaml"
        if direct.is_file():
            return direct

        # If stem ends with _amcl or _localization (e.g. a2_amcl), also check base stem
        if stem.endswith("_amcl"):
            base_cand = p.parent / f"{stem[:-5]}_waypoints.yaml"
            if base_cand.is_file():
                return base_cand
        elif stem.endswith("_localization"):
            base_cand = p.parent / f"{stem[:-13]}_waypoints.yaml"
            if base_cand.is_file():
                return base_cand

        # Priority 2: In config/waypoints/<map_stem>_waypoints.yaml
        cfg_cand = self._config_dir / "waypoints" / f"{stem}_waypoints.yaml"
        if cfg_cand.is_file():
            return cfg_cand

        return direct

    def _sync_active_config_files(self) -> None:
        """Mirror current batch waypoints to global config/locations.yaml & routes.yaml for background nodes."""
        try:
            loc_file = self._config_dir / "locations.yaml"
            route_file = self._config_dir / "routes.yaml"

            locations: dict[str, Any] = {}
            route_names: list[str] = []
            for wp in self._batch_waypoints:
                name = str(wp["name"])
                route_names.append(name)
                locations[name] = {
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                }

            with loc_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(locations, f, allow_unicode=True, sort_keys=False)
            with route_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump({"patrol_route": route_names}, f, allow_unicode=True, sort_keys=False)

            self.reload_route_requested.emit()
        except Exception:
            pass

    def _load_patrol_from_file(self, wp_file: Path, silent: bool = True) -> bool:
        """Load waypoints from a dedicated map waypoints file and sync to views."""
        try:
            with wp_file.open("r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}

            locations = data.get("locations", {})
            routes = data.get("routes", {})
            patrol_names = routes.get("patrol_route", []) if isinstance(routes, dict) else []

            loaded: list[dict[str, Any]] = []

            if isinstance(locations, dict) and locations:
                if not patrol_names:
                    patrol_names = list(locations.keys())
                for name in patrol_names:
                    coords = locations.get(name)
                    if not isinstance(coords, dict):
                        continue
                    note = coords.get("note", "Trạm nước / Khởi hành" if name == "khoihanh" else f"Điểm tuần tra {name}")
                    loaded.append({
                        "name": str(name),
                        "x": float(coords.get("x", 0.0)),
                        "y": float(coords.get("y", 0.0)),
                        "yaw": float(coords.get("yaw", 0.0)),
                        "note": note,
                    })
            elif "waypoints" in data and isinstance(data["waypoints"], list):
                for wp in data["waypoints"]:
                    if isinstance(wp, dict) and "x" in wp and "y" in wp:
                        name = str(wp.get("name", "diem"))
                        loaded.append({
                            "name": name,
                            "x": float(wp["x"]),
                            "y": float(wp["y"]),
                            "yaw": float(wp.get("yaw", 0.0)),
                            "note": str(wp.get("note", f"Điểm {name}")),
                        })

            self._batch_waypoints = loaded
            recompute_cyclic_waypoint_yaws(self._batch_waypoints)
            self._sync_views_from_waypoints()
            self._sync_active_config_files()

            if not silent:
                QMessageBox.information(
                    self,
                    "Nạp cấu hình thành công",
                    f"Đã nạp {len(loaded)} điểm tuần tra từ:\n{wp_file.name}",
                )
            self.append_patrol_log(f"[WAYPOINT][INFO] Đã nạp {len(loaded)} điểm tuần tra từ file {wp_file.name}.", "info")
            return True
        except Exception as exc:
            if not silent:
                QMessageBox.warning(self, "Lỗi đọc YAML", f"Không thể đọc file {wp_file.name}:\n{exc}")
            return False

    def _load_patrol_from_yaml(self, silent: bool = False) -> bool:
        """Load waypoints: prioritized from current map's waypoint file, falling back to global config."""
        wp_file = self._get_map_waypoints_path(self._current_map_path)
        if wp_file and wp_file.is_file():
            return self._load_patrol_from_file(wp_file, silent=silent)

        loc_file = self._config_dir / "locations.yaml"
        if loc_file.is_file():
            return self._load_patrol_from_file(loc_file, silent=silent)

        if not silent:
            QMessageBox.warning(self, "Chưa có file cấu hình", "Không tìm thấy file lộ trình nào.")
        return False

    def _save_patrol_to_yaml(self) -> bool:
        """Save waypoints into dedicated file for current map, overwriting existing, and sync to global config."""
        self._sync_waypoints_from_table()
        if not self._batch_waypoints:
            QMessageBox.warning(self, "Chưa có điểm", "Hãy chấm ít nhất 1 điểm trên bản đồ trước khi lưu lộ trình.")
            return False

        target_wp_file = self._get_map_waypoints_path(self._current_map_path)
        if target_wp_file is None:
            target_wp_file = self._config_dir / "locations.yaml"

        try:
            target_wp_file.parent.mkdir(parents=True, exist_ok=True)

            locations: dict[str, Any] = {}
            route_names: list[str] = []
            wp_list: list[dict[str, Any]] = []

            for wp in self._batch_waypoints:
                name = str(wp["name"])
                route_names.append(name)
                locations[name] = {
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                    "note": str(wp.get("note", "")),
                }
                wp_list.append({
                    "name": name,
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                    "note": str(wp.get("note", "")),
                })

            map_name_str = self._current_map_path.name if self._current_map_path else "default_map"
            payload = {
                "map_name": map_name_str,
                "locations": locations,
                "routes": {
                    "patrol_route": route_names,
                },
                "waypoints": wp_list,
            }

            with target_wp_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)

            self._sync_active_config_files()
            self._sync_views_from_waypoints()

            self.append_patrol_log(
                f"[WAYPOINT][INFO] Đã lưu {len(self._batch_waypoints)} điểm vào {target_wp_file.name}.",
                "success"
            )
            QMessageBox.information(
                self,
                "Lưu lộ trình thành công",
                f"Đã lưu {len(self._batch_waypoints)} điểm vào {target_wp_file.name} và đồng bộ tới hệ thống:\n{' -> '.join(route_names)}",
            )
            return True
        except Exception as exc:
            QMessageBox.critical(self, "Lỗi lưu YAML", f"Không thể ghi cấu hình:\n{exc}")
            return False

    # -------------------------------------------------------------------------
    # Single Navigation Controls
    # -------------------------------------------------------------------------
    def _set_button_strength(self, button: QPushButton, strong: bool) -> None:
        effect = button.graphicsEffect()
        if not isinstance(effect, QGraphicsOpacityEffect):
            effect = QGraphicsOpacityEffect(button)
            button.setGraphicsEffect(effect)
        effect.setOpacity(1.0 if strong else 0.35)
        button.setEnabled(strong)

    def set_navigation_active(self, active: bool) -> None:
        self._navigation_active = active
        self._set_button_strength(self.start_button, not active)
        self._set_button_strength(self.stop_button, active)
        self.map_widget.set_selection_enabled(not active)

    def _on_single_goal_selected(self, x: float, y: float, yaw: float) -> None:
        self._goal = (x, y, yaw)
        self.status_label.setText(f"Điểm đến: X={x:.2f}, Y={y:.2f} m")
        self._set_button_strength(self.start_button, True)

    def _show_warning(self, message: str) -> None:
        self.set_command_status("warning", f"[WARN] {message}")

    def _start_single_navigation(self) -> None:
        self.append_patrol_log("[NAV][INFO] Đã bấm nút: BẮT ĐẦU ĐI.", "progress")
        if self._localization_state != "LOCALIZED":
            self.set_command_status(
                "error",
                "[ERROR] Robot chưa hoàn tất định vị ban đầu (chưa đạt LOCALIZED). Vui lòng bấm 2D POSE ESTIMATE."
            )
            QMessageBox.warning(
                self,
                "Chưa Định Vị Robot",
                "Robot chưa hoàn tất định vị ban đầu (chưa đạt trạng thái LOCALIZED tin cậy).\n"
                f"Trạng thái hiện tại: {self._localization_state}.\n"
                "Vui lòng bấm '2D POSE ESTIMATE' để xác nhận vị trí xuất phát trước khi bắt đầu điều hướng!",
            )
            return
        if self._goal is None:
            self._show_warning("Hãy chọn điểm đến trên bản đồ trước khi bắt đầu điều hướng")
            return
        self.set_navigation_active(True)
        self.set_command_status(
            "progress",
            f"[INFO] Bắt đầu điều hướng tới tọa độ: X={self._goal[0]:.2f}, Y={self._goal[1]:.2f}..."
        )
        self.start_requested.emit(*self._goal)

    def _stop_navigation(self) -> None:
        self.append_patrol_log("[NAV][WARN] Đã bấm nút: DỪNG HÀNH TRÌNH.", "warning")
        self.set_navigation_active(False)
        self.set_command_status("warning", "[WARN] Đã yêu cầu dừng hành trình điều hướng.")
        self.cancel_requested.emit()
        self.nav_progress_bar.setValue(0)
        self.lbl_dist_remain.setText("Cự ly còn lại: -- m")

    def _start_patrol(self) -> None:
        self.append_patrol_log("[PATROL][INFO] Đã bấm nút: BẮT ĐẦU TUẦN TRA.", "progress")
        if self._localization_state != "LOCALIZED":
            self.set_command_status(
                "error",
                "[ERROR] Robot chưa hoàn tất định vị ban đầu. Vui lòng bấm 2D POSE ESTIMATE trước khi tuần tra."
            )
            QMessageBox.warning(
                self,
                "Chưa Định Vị Robot",
                "Robot chưa hoàn tất định vị ban đầu (chưa đạt trạng thái LOCALIZED tin cậy).\n"
                f"Trạng thái hiện tại: {self._localization_state}.\n"
                "Vui lòng bấm '2D POSE ESTIMATE' để xác nhận vị trí xuất phát trước khi bắt đầu tuần tra!",
            )
            return
        self.set_command_status("progress", "[INFO] Đang gửi yêu cầu bắt đầu tuần tra...")
        self.start_patrol_requested.emit()

    def _pause_patrol(self) -> None:
        self.append_patrol_log("[PATROL][WARN] Đã bấm nút: TẠM DỪNG TUẦN TRA.", "warning")
        self.pause_patrol_requested.emit()

    def _toggle_2d_estimate(self) -> None:
        """Bật/tắt chế độ chọn vị trí và hướng robot (2D Pose Estimate) trực tiếp trên bản đồ."""
        if self.map_widget._initial_pose_mode:
            self.map_widget.set_initial_pose_mode(False)
            self.btn_initial_pose.setText("2D POSE ESTIMATE")
            self.btn_initial_pose.setStyleSheet("")
            self.append_patrol_log("[ESTIMATE][INFO] Đã hủy chế độ 2D Pose Estimate.", "info")
            self.set_command_status("warning", "[INFO] Đã hủy chế độ 2D Pose Estimate.")
            return

        self.map_widget.set_initial_pose_mode(True)
        self.btn_initial_pose.setText("HỦY 2D POSE")
        self.btn_initial_pose.setStyleSheet(
            "background: #059669; color: #ffffff; font-weight: 700; border: 1px solid #10b981;"
        )
        self.append_patrol_log("[ESTIMATE][INFO] Đã bật chế độ 2D Pose Estimate (Chạm và kéo trên bản đồ).", "progress")
        self.set_command_status("progress", "[INFO] Chạm và kéo trên bản đồ để chọn vị trí và hướng robot.")

    def _on_initial_pose_button_clicked(self) -> None:
        """Alias tương thích luồng cũ."""
        self._toggle_2d_estimate()

    def _enable_map_pose_pick_mode(self) -> None:
        """Kích hoạt mode nếu được gọi từ dialog cũ."""
        self.map_widget.set_initial_pose_mode(True)
        self.btn_initial_pose.setText("HỦY 2D POSE")
        self.btn_initial_pose.setStyleSheet(
            "background: #059669; color: #ffffff; font-weight: 700; border: 1px solid #10b981;"
        )
        self.set_command_status("progress", "[INFO] Chạm và kéo trên bản đồ để chọn vị trí và hướng robot.")

    def _on_map_initial_pose_picked(self, x: float, y: float, yaw: float) -> None:
        """Xử lý khi người dùng hoàn tất thao tác Press -> Drag -> Release trên bản đồ."""
        self._send_initial_pose(x, y, yaw, "2d_pose_estimate")

    def _on_initial_pose_selected(self, x: float, y: float, yaw: float) -> None:
        """Alias tương thích reference simulator."""
        self._on_map_initial_pose_picked(x, y, yaw)

    def _send_initial_pose(self, x: float, y: float, yaw: float, source: str) -> None:
        map_name = self._current_map_path.name if self._current_map_path else ""
        deg = math.degrees(yaw)
        self.set_command_status(
            "progress",
            f"[INFO] Đang gửi 2D Pose: X={x:.2f} m, Y={y:.2f} m, Yaw={yaw:.2f} rad ({deg:.0f}°)..."
        )
        self.initial_pose_requested.emit({
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "source": source,
            "map_name": map_name,
        })
        self.map_widget.set_initial_pose_mode(False)
        self.map_widget.set_candidate_initial_pose(x, y, yaw)
        self.btn_initial_pose.setText("2D POSE ESTIMATE")
        self.btn_initial_pose.setStyleSheet("")

    def set_localization_status(self, state: str, message: str) -> None:
        """Cập nhật huy hiệu trạng thái Localization và kiểm soát điều kiện điều hướng."""
        self._localization_state = str(state).upper()
        self._localization_message = message

        if self._localization_state == "LOCALIZED":
            self.lbl_localization_badge.setText("● ĐÃ ĐỊNH VỊ (TF OK)")
            self.lbl_localization_badge.setStyleSheet(
                "background: rgba(16, 185, 129, 0.2); color: #10b981; "
                "border: 1px solid #10b981; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
            )
            self.status_label.setText(f"Đã định vị thành công: {message}")
        elif self._localization_state == "TF_AVAILABLE":
            self.lbl_localization_badge.setText("● ĐANG HỘI TỤ...")
            self.lbl_localization_badge.setStyleSheet(
                "background: rgba(245, 158, 11, 0.2); color: #f59e0b; "
                "border: 1px solid #f59e0b; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
            )
            self.status_label.setText(f"Có TF map->odom, chờ AMCL hội tụ vị trí: {message}")
        elif self._localization_state == "WAITING_FOR_LOCALIZATION":
            self.lbl_localization_badge.setText("● CHỜ AMCL...")
            self.lbl_localization_badge.setStyleSheet(
                "background: rgba(245, 158, 11, 0.2); color: #f59e0b; "
                "border: 1px solid #f59e0b; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
            )
            self.status_label.setText(f"Đang chờ AMCL định vị: {message}")
        elif self._localization_state == "PUBLISHING":
            self.lbl_localization_badge.setText("● ĐANG GỬI POSE...")
            self.lbl_localization_badge.setStyleSheet(
                "background: rgba(56, 189, 248, 0.2); color: #38bdf8; "
                "border: 1px solid #38bdf8; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
            )
        else:
            self.lbl_localization_badge.setText("● CHƯA ĐỊNH VỊ")
            self.lbl_localization_badge.setStyleSheet(
                "background: rgba(239, 68, 68, 0.2); color: #ef4444; "
                "border: 1px solid #ef4444; border-radius: 6px; padding: 4px 8px; font-weight: 700; font-size: 11px;"
            )

    def on_initial_pose_confirmed(self, data: dict[str, Any]) -> None:
        """Xác nhận định vị từ AMCL kèm tọa độ thực tế."""
        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        cov = float(data.get("covariance_norm", 0.0))
        self.set_localization_status("LOCALIZED", f"Vị trí: ({x:.2f}, {y:.2f}) | Sai số ước lượng: {cov:.2f}m")

    # -------------------------------------------------------------------------
    # Start Pose (khoihanh) Management: Capture & Relocalize
    # -------------------------------------------------------------------------
    def _update_start_pose_state(self) -> None:
        """Tìm và cập nhật trạng thái điểm xuất phát (khoihanh) cho bản đồ hiện tại."""
        start_wp = self._get_saved_start_pose(force_reload=True)
        if start_wp:
            self._start_wp = start_wp
            deg = math.degrees(float(start_wp.get("yaw", 0.0)))
            tip = f"Khôi phục định vị về điểm xuất phát đã lưu (X={float(start_wp['x']):.2f}m, Y={float(start_wp['y']):.2f}m, Yaw={deg:.0f}°)"
            if hasattr(self, "btn_relocalize_start_pose"):
                self.btn_relocalize_start_pose.setToolTip(tip)
            if hasattr(self, "btn_quick_relocalize"):
                self.btn_quick_relocalize.setToolTip(tip)
            if hasattr(self, "btn_patrol_relocalize"):
                self.btn_patrol_relocalize.setToolTip(tip)
        else:
            self._start_wp = None
            tip = "Chưa có điểm xuất phát được lưu cho bản đồ này. Hãy dùng 'LƯU ĐIỂM GỐC' trước."
            if hasattr(self, "btn_relocalize_start_pose"):
                self.btn_relocalize_start_pose.setToolTip(tip)
            if hasattr(self, "btn_quick_relocalize"):
                self.btn_quick_relocalize.setToolTip(tip)
            if hasattr(self, "btn_patrol_relocalize"):
                self.btn_patrol_relocalize.setToolTip(tip)

    def _get_saved_start_pose(self, force_reload: bool = False) -> dict[str, Any] | None:
        """Đọc waypoint 'khoihanh' từ bộ nhớ hoặc từ file <map_stem>_waypoints.yaml."""
        if not force_reload and getattr(self, "_start_wp", None):
            return self._start_wp

        # Kiểm tra trong _batch_waypoints
        for wp in getattr(self, "_batch_waypoints", []):
            if str(wp.get("name", "")).lower() == "khoihanh":
                self._start_wp = {
                    "name": "khoihanh",
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                    "note": str(wp.get("note", "Điểm xuất phát")),
                }
                return self._start_wp

        # Kiểm tra đọc từ file YAML của map hiện tại
        if self._current_map_path:
            stem = self._current_map_path.stem
            candidates = [
                self._get_map_waypoints_path(self._current_map_path),
                self._current_map_path.parent / f"{stem}_waypoints.yaml",
                self._config_dir / "waypoints" / f"{stem}_waypoints.yaml",
            ]
            for c in candidates:
                if c and c.is_file():
                    try:
                        with c.open("r", encoding="utf-8") as f:
                            data = yaml.safe_load(f) or {}
                        locs = data.get("locations", {}) if isinstance(data, dict) and "locations" in data else data
                        if isinstance(locs, dict) and "khoihanh" in locs:
                            kh = locs["khoihanh"]
                            if isinstance(kh, dict) and "x" in kh and "y" in kh:
                                self._start_wp = {
                                    "name": "khoihanh",
                                    "x": float(kh.get("x", 0.0)),
                                    "y": float(kh.get("y", 0.0)),
                                    "yaw": float(kh.get("yaw", 0.0)),
                                    "note": str(kh.get("note", "Điểm xuất phát")),
                                    "source_file": str(c),
                                }
                                return self._start_wp
                    except Exception:
                        pass
        return None

    def _save_start_pose_to_yaml(self, x: float, y: float, yaw: float) -> bool:
        """Ghi điểm xuất phát 'khoihanh' vào file YAML của bản đồ hiện tại và đồng bộ hệ thống."""
        target_wp_file = self._get_map_waypoints_path(self._current_map_path)
        if target_wp_file is None:
            target_wp_file = self._config_dir / "locations.yaml"

        try:
            target_wp_file.parent.mkdir(parents=True, exist_ok=True)
            existing_data: dict[str, Any] = {}
            if target_wp_file.is_file():
                try:
                    with target_wp_file.open("r", encoding="utf-8") as f:
                        existing_data = yaml.safe_load(f) or {}
                except Exception:
                    existing_data = {}

            locations = existing_data.get("locations", {}) if isinstance(existing_data.get("locations"), dict) else {}
            routes = existing_data.get("routes", {}) if isinstance(existing_data.get("routes"), dict) else {}
            wp_list = existing_data.get("waypoints", []) if isinstance(existing_data.get("waypoints"), list) else []

            # Cập nhật locations["khoihanh"]
            locations["khoihanh"] = {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "yaw": round(float(yaw), 3),
                "note": "Điểm xuất phát",
            }

            # Cập nhật wp_list
            found_in_list = False
            for wp in wp_list:
                if isinstance(wp, dict) and str(wp.get("name", "")).lower() == "khoihanh":
                    wp["x"] = round(float(x), 3)
                    wp["y"] = round(float(y), 3)
                    wp["yaw"] = round(float(yaw), 3)
                    wp["note"] = "Điểm xuất phát"
                    found_in_list = True
                    break
            if not found_in_list:
                wp_list.insert(0, {
                    "name": "khoihanh",
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                    "yaw": round(float(yaw), 3),
                    "note": "Điểm xuất phát",
                })

            # Giữ nguyên patrol_route hiện tại, không ép buộc điểm cất máy sát tường vào lộ trình tự động
            patrol_route = routes.get("patrol_route", [])
            if not isinstance(patrol_route, list):
                patrol_route = []
            routes["patrol_route"] = patrol_route

            map_name_str = self._current_map_path.name if self._current_map_path else "default_map"
            payload = {
                "map_name": map_name_str,
                "locations": locations,
                "routes": routes,
                "waypoints": wp_list,
            }

            with target_wp_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)

            # Cập nhật bộ nhớ nội bộ _start_wp
            self._start_wp = {
                "name": "khoihanh",
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "yaw": round(float(yaw), 3),
                "note": "Điểm xuất phát",
                "source_file": str(target_wp_file),
            }

            # Cập nhật _batch_waypoints trong giao diện
            found_in_batch = False
            for wp in self._batch_waypoints:
                if str(wp.get("name", "")).lower() == "khoihanh":
                    wp["x"] = round(float(x), 3)
                    wp["y"] = round(float(y), 3)
                    wp["yaw"] = round(float(yaw), 3)
                    wp["note"] = "Điểm xuất phát"
                    found_in_batch = True
                    break
            if not found_in_batch:
                self._batch_waypoints.insert(0, {
                    "name": "khoihanh",
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                    "yaw": round(float(yaw), 3),
                    "note": "Điểm xuất phát",
                })

            self._sync_active_config_files()
            self._sync_views_from_waypoints()
            self._refresh_quick_location_chips()
            self._update_start_pose_state()
            self.map_widget.set_batch_waypoints(self._batch_waypoints)
            self.patrol_map_widget.set_batch_waypoints(self._batch_waypoints)
            self.map_widget.update()
            self.patrol_map_widget.update()
            return True
        except Exception as exc:
            self.append_patrol_log(f"[WAYPOINT][ERROR] Lỗi khi lưu điểm xuất phát: {exc}", "error")
            return False

    def _capture_current_pose_as_start(self, silent: bool = False) -> None:
        """Chụp tọa độ hiện tại của robot hoặc ứng viên 2D pose làm điểm xuất phát (khoihanh)."""
        pose = None
        if getattr(self, "_current_robot_pose", None) and "x" in self._current_robot_pose:
            pose = self._current_robot_pose
        elif self.map_widget._pose and "x" in self.map_widget._pose:
            pose = self.map_widget._pose
        elif self.map_widget._candidate_initial_pose and "x" in self.map_widget._candidate_initial_pose:
            pose = self.map_widget._candidate_initial_pose

        if not pose or "x" not in pose or "y" not in pose:
            self.set_command_status(
                "warn",
                "[WARN] Chưa có tọa độ robot! Hãy dùng '2D POSE ESTIMATE' kéo vị trí robot trên bản đồ trước."
            )
            self.append_patrol_log(
                "[WAYPOINT][WARN] Không thể chụp điểm xuất phát vì robot chưa có tọa độ hợp lệ.",
                "warn"
            )
            if not silent:
                QMessageBox.warning(
                    self,
                    "Chưa có tọa độ robot",
                    "Hiện tại robot chưa có tọa độ hợp lệ trên bản đồ!\n\n"
                    "Cách khắc phục:\n"
                    "1. Bấm nút '2D POSE ESTIMATE' rồi chạm kéo mũi tên trên bản đồ để xác định vị trí và hướng của robot.\n"
                    "2. Hoặc khởi động Layer 2 và đợi robot nhận vị trí từ hệ thống cảm biến.\n"
                    "Sau đó bấm lại '📸 LƯU ĐIỂM GỐC'."
                )
            return

        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))

        success = self._save_start_pose_to_yaml(x, y, yaw)
        if success:
            deg = math.degrees(yaw)
            map_name = self._current_map_path.name if self._current_map_path else "bản đồ"
            self.set_command_status(
                "success",
                f"[INFO] Đã lưu điểm xuất phát: X={x:.2f} m, Y={y:.2f} m, Yaw={yaw:.2f} rad ({deg:.0f}°)"
            )
            self.append_patrol_log(
                f"[WAYPOINT][INFO] Đã chụp và lưu điểm xuất phát 'khoihanh': X={x:.2f} m, Y={y:.2f} m, Yaw={deg:.0f}° cho {map_name}.",
                "success"
            )
            self.status_label.setText(f"Đã lưu điểm xuất phát: X={x:.2f}, Y={y:.2f} m (khoihanh)")
            if not silent:
                QMessageBox.information(
                    self,
                    "Lưu điểm xuất phát thành công",
                    f"Đã lưu vị trí robot hiện tại làm ĐIỂM XUẤT PHÁT (khoihanh) cho {map_name}:\n\n"
                    f"• X: {x:.3f} m\n"
                    f"• Y: {y:.3f} m\n"
                    f"• Hướng (Yaw): {yaw:.3f} rad ({deg:.1f}°)\n\n"
                    f"Từ bây giờ, nếu robot bị mất định vị, chỉ cần điều khiển robot về lại vị trí này "
                    f"và ấn nút '🎯 VỀ ĐIỂM GỐC' để khôi phục định vị tức thì!"
                )

    def _relocalize_to_start_pose(self, silent: bool = False) -> None:
        """Khôi phục định vị AMCL về điểm xuất phát đã lưu của bản đồ hiện tại."""
        start_wp = self._get_saved_start_pose()
        if not start_wp:
            self.set_command_status(
                "warn",
                "[WARN] Bản đồ này chưa có điểm xuất phát đã lưu! Hãy lái robot về vạch mốc và ấn '📸 LƯU ĐIỂM GỐC' trước."
            )
            self.append_patrol_log(
                "[ESTIMATE][WARN] Không tìm thấy điểm xuất phát 'khoihanh' để khôi phục định vị.",
                "warn"
            )
            if not silent:
                QMessageBox.warning(
                    self,
                    "Chưa có điểm xuất phát",
                    "Bản đồ này chưa có điểm xuất phát (khoihanh) nào được lưu!\n\n"
                    "Vui lòng:\n"
                    "1. Định vị chuẩn cho robot (bằng '2D POSE ESTIMATE') hoặc di chuyển robot đến vạch xuất phát.\n"
                    "2. Ấn nút '📸 LƯU ĐIỂM GỐC' để lưu lại vị trí này.\n"
                    "Sau đó bạn có thể ấn nút này bất cứ lúc nào để khôi phục định vị tức thì."
                )
            return

        x = float(start_wp["x"])
        y = float(start_wp["y"])
        yaw = float(start_wp.get("yaw", 0.0))
        deg = math.degrees(yaw)

        self.append_patrol_log(
            f"[ESTIMATE][INFO] Đang khôi phục định vị về điểm xuất phát: X={x:.2f} m, Y={y:.2f} m, Yaw={yaw:.2f} rad ({deg:.0f}°)...",
            "progress"
        )
        self.set_command_status(
            "progress",
            f"[INFO] Đang đặt lại vị trí robot về điểm gốc: X={x:.2f} m, Y={y:.2f} m, Yaw={deg:.0f}°..."
        )
        self._send_initial_pose(x, y, yaw, "restore_start_pose")

    # -------------------------------------------------------------------------
    # Standby Staging Point (tram_cho) Management
    # -------------------------------------------------------------------------
    def _get_saved_standby_pose(self) -> dict[str, Any] | None:
        """Đọc waypoint 'tram_cho' từ bộ nhớ hoặc từ file <map_stem>_waypoints.yaml."""
        if hasattr(self, "_standby_wp") and self._standby_wp:
            return self._standby_wp

        for wp in self._batch_waypoints:
            if str(wp.get("name", "")).lower() == "tram_cho":
                self._standby_wp = {
                    "name": "tram_cho",
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                }
                return self._standby_wp

        wp_file = self._get_map_waypoints_path(self._current_map_path)
        if wp_file and wp_file.is_file():
            try:
                with wp_file.open("r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                locs = data.get("locations", {})
                if isinstance(locs, dict) and "tram_cho" in locs:
                    tc = locs["tram_cho"]
                    if isinstance(tc, dict) and "x" in tc and "y" in tc:
                        self._standby_wp = {
                            "name": "tram_cho",
                            "x": float(tc["x"]),
                            "y": float(tc["y"]),
                            "yaw": float(tc.get("yaw", 0.0)),
                            "source_file": str(wp_file),
                        }
                        return self._standby_wp
            except Exception:
                pass
        return None

    def _capture_current_pose_as_standby(self, silent: bool = False) -> None:
        """Chụp tọa độ thực tế hiện tại của robot từ TF/Odometry làm Trạm Chờ thông thoáng (tram_cho)."""
        pose = None
        if getattr(self, "_current_robot_pose", None) and "x" in self._current_robot_pose:
            pose = self._current_robot_pose
        elif hasattr(self, "map_widget") and getattr(self.map_widget, "_pose", None) and "x" in self.map_widget._pose:
            pose = self.map_widget._pose
        elif (
            hasattr(self, "map_widget")
            and getattr(self.map_widget, "_candidate_initial_pose", None)
            and "x" in self.map_widget._candidate_initial_pose
        ):
            pose = self.map_widget._candidate_initial_pose

        if not pose or "x" not in pose or "y" not in pose:
            self.set_command_status(
                "warn",
                "[WARN] Chưa có tọa độ robot! Vui lòng định vị robot bằng '🎯 VỀ ĐIỂM GỐC' hoặc '2D POSE ESTIMATE' trước."
            )
            self.append_patrol_log(
                "[WAYPOINT][WARN] Không thể chụp Trạm Chờ vì robot chưa có tọa độ vị trí thực tế.",
                "warn"
            )
            if not silent:
                QMessageBox.warning(
                    self,
                    "Chưa có tọa độ",
                    "Không tìm thấy tọa độ vị trí thực tế của robot trên bản đồ!\n\n"
                    "Vui lòng định vị robot bằng '🎯 VỀ ĐIỂM GỐC' hoặc '2D POSE ESTIMATE' trước khi lưu Trạm Chờ."
                )
            return

        x = float(pose["x"])
        y = float(pose["y"])
        yaw = float(pose.get("yaw", 0.0))

        map_name = self._current_map_path.name if self._current_map_path else "default"
        deg = math.degrees(yaw)
        success = self._save_standby_pose_to_yaml(x, y, yaw)
        if success:
            self.append_patrol_log(
                f"[WAYPOINT][INFO] Đã lưu Trạm Chờ 'tram_cho': X={x:.2f} m, Y={y:.2f} m, Yaw={deg:.0f}° cho {map_name}.",
                "success"
            )
            self.set_command_status("success", f"Đã lưu Trạm Chờ: X={x:.2f}, Y={y:.2f} m (tram_cho)")
            if not silent:
                QMessageBox.information(
                    self,
                    "Lưu Trạm Chờ Thành Công",
                    f"Đã lưu vị trí robot làm TRẠM CHỜ (tram_cho) cho {map_name}:\n\n"
                    f"• X: {x:.3f} m\n"
                    f"• Y: {y:.3f} m\n"
                    f"• Hướng (Yaw): {yaw:.3f} rad ({deg:.1f}°)\n\n"
                    f"Từ bây giờ, khi kết thúc ca làm việc hoặc cần nghỉ ngơi, chỉ cần ấn nút '🚏 VỀ TRẠM CHỜ' "
                    f"để Nav2 tự động lái robot về vị trí an toàn này!"
                )

    def _save_standby_pose_to_yaml(self, x: float, y: float, yaw: float) -> bool:
        """Ghi Trạm Chờ 'tram_cho' vào file YAML của bản đồ hiện tại và đồng bộ hệ thống."""
        target_wp_file = self._get_map_waypoints_path(self._current_map_path)
        if target_wp_file is None:
            target_wp_file = self._config_dir / "locations.yaml"

        try:
            target_wp_file.parent.mkdir(parents=True, exist_ok=True)
            existing_data: dict[str, Any] = {}
            if target_wp_file.is_file():
                with target_wp_file.open("r", encoding="utf-8") as f:
                    existing_data = yaml.safe_load(f) or {}

            locations = existing_data.get("locations", {})
            if not isinstance(locations, dict):
                locations = {}
            routes = existing_data.get("routes", {})
            if not isinstance(routes, dict):
                routes = {}
            wp_list = existing_data.get("waypoints", [])
            if not isinstance(wp_list, list):
                wp_list = []

            # Cập nhật locations["tram_cho"]
            locations["tram_cho"] = {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "yaw": round(float(yaw), 3),
                "note": "Trạm chờ an toàn",
            }

            # Cập nhật wp_list
            found_in_list = False
            for wp in wp_list:
                if isinstance(wp, dict) and str(wp.get("name", "")).lower() == "tram_cho":
                    wp["x"] = round(float(x), 3)
                    wp["y"] = round(float(y), 3)
                    wp["yaw"] = round(float(yaw), 3)
                    wp["note"] = "Trạm chờ an toàn"
                    found_in_list = True
                    break
            if not found_in_list:
                wp_list.append({
                    "name": "tram_cho",
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                    "yaw": round(float(yaw), 3),
                    "note": "Trạm chờ an toàn",
                })

            map_name_str = self._current_map_path.name if self._current_map_path else "default_map"
            payload = {
                "map_name": map_name_str,
                "locations": locations,
                "routes": routes,
                "waypoints": wp_list,
            }

            with target_wp_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)

            self._standby_wp = {
                "name": "tram_cho",
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "yaw": round(float(yaw), 3),
                "note": "Trạm chờ an toàn",
                "source_file": str(target_wp_file),
            }

            # Cập nhật _batch_waypoints trong giao diện
            found_in_batch = False
            for wp in self._batch_waypoints:
                if str(wp.get("name", "")).lower() == "tram_cho":
                    wp["x"] = round(float(x), 3)
                    wp["y"] = round(float(y), 3)
                    wp["yaw"] = round(float(yaw), 3)
                    wp["note"] = "Trạm chờ an toàn"
                    found_in_batch = True
                    break
            if not found_in_batch:
                self._batch_waypoints.append({
                    "name": "tram_cho",
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                    "yaw": round(float(yaw), 3),
                    "note": "Trạm chờ an toàn",
                })

            recompute_cyclic_waypoint_yaws(self._batch_waypoints)
            self._sync_views_from_waypoints()
            self._sync_active_config_files()
            self._refresh_quick_location_chips()
            self.map_widget.set_batch_waypoints(self._batch_waypoints)
            self.patrol_map_widget.set_batch_waypoints(self._batch_waypoints)
            return True
        except Exception as exc:
            self.get_logger().error(f"[WAYPOINT][ERROR] Không thể lưu Trạm Chờ: {exc}")
            return False

    def _navigate_to_standby_pose(self, silent: bool = False) -> None:
        """Nav2 tự động lập lộ trình và lái robot về Trạm Chờ an toàn (tram_cho)."""
        standby_wp = self._get_saved_standby_pose()
        if not standby_wp:
            self.set_command_status(
                "warn",
                "[WARN] Bản đồ này chưa có Trạm Chờ đã lưu! Hãy chọn điểm hoặc lái robot đến vị trí an toàn và ấn '📸 LƯU TRẠM CHỜ'."
            )
            self.append_patrol_log(
                "[NAV][WARN] Không tìm thấy điểm 'tram_cho' để điều hướng về trạm chờ.",
                "warn"
            )
            if not silent:
                QMessageBox.warning(
                    self,
                    "Chưa có Trạm Chờ",
                    "Bản đồ này chưa có Trạm Chờ (tram_cho) nào được lưu!\n\n"
                    "Vui lòng:\n"
                    "1. Di chuyển robot (hoặc chọn 1 điểm thoáng cách tường ~1m trên bản đồ).\n"
                    "2. Ấn nút '📸 LƯU TRẠM CHỜ' để lưu lại vị trí này.\n\n"
                    "Sau đó bạn có thể ấn nút này bất cứ lúc nào để robot tự động lái về!"
                )
            return

        if self._localization_state != "LOCALIZED":
            self.set_command_status(
                "error",
                "[ERROR] Robot chưa hoàn tất định vị ban đầu (chưa đạt LOCALIZED). Vui lòng định vị trước."
            )
            if not silent:
                QMessageBox.warning(
                    self,
                    "Chưa Định Vị Robot",
                    "Robot chưa hoàn tất định vị ban đầu (chưa đạt trạng thái LOCALIZED tin cậy).\n"
                    f"Trạng thái hiện tại: {self._localization_state}.\n"
                    "Vui lòng bấm '🎯 VỀ ĐIỂM GỐC' hoặc '2D POSE ESTIMATE' trước khi bắt đầu điều hướng!",
                )
            return

        x = float(standby_wp["x"])
        y = float(standby_wp["y"])
        yaw = float(standby_wp.get("yaw", 0.0))
        deg = math.degrees(yaw)

        self._goal = (x, y, yaw)
        self.map_widget.set_goal(x, y, yaw)
        self.set_navigation_active(True)
        self.append_patrol_log(
            f"[NAV][INFO] Robot bắt đầu tự động điều hướng về Trạm Chờ: X={x:.2f} m, Y={y:.2f} m, Yaw={deg:.0f}°...",
            "progress"
        )
        self.set_command_status(
            "progress",
            f"[INFO] Đang điều hướng về Trạm Chờ: X={x:.2f}, Y={y:.2f} m..."
        )
        self.start_requested.emit(x, y, yaw)

    # -------------------------------------------------------------------------
    # Header Badges & Fullscreen
    # -------------------------------------------------------------------------
    def update_fullscreen_button(self, is_fullscreen: bool) -> None:
        """Update single button icon and tooltip alternating between fullscreen and windowed mode."""
        if is_fullscreen:
            self.fullscreen_button.setText("🗗")
            self.fullscreen_button.setToolTip("Thoát toàn màn hình (Esc hoặc F11)")
        else:
            self.fullscreen_button.setText("🗖")
            self.fullscreen_button.setToolTip("Toàn màn hình (F11)")

    def set_wifi(self, connected: bool) -> None:
        icon = self.icons.get("wifi_connected") if connected else self.icons.get("wifi_disconnected")
        if icon and Path(icon).is_file():
            pm = QPixmap(str(icon))
            if not pm.isNull():
                self.wifi_icon_label.setPixmap(
                    pm.scaled(20, 20, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
        self.wifi_text_label.setText("Online" if connected else "Offline")
        self.wifi_text_label.setStyleSheet(
            "color: #4cd964; font-size: 11px; font-weight: 700;"
            if connected else "color: #ff5252; font-size: 11px; font-weight: 700;"
        )

    def set_battery(self, percent: float | None) -> None:
        if percent is None or percent < 0:
            key = "0"
            text = "--%"
        else:
            text = f"{percent:.0f}%"
            if percent >= 85:
                key = "100"
            elif percent >= 60:
                key = "75"
            elif percent >= 35:
                key = "50"
            elif percent >= 15:
                key = "25"
            else:
                key = "0"
        battery_icons = self.icons.get("battery_icons", {})
        if isinstance(battery_icons, dict):
            icon = battery_icons.get(key)
            if icon and Path(icon).is_file():
                pm = QPixmap(str(icon))
                if not pm.isNull():
                    self.battery_icon_label.setPixmap(
                        pm.scaled(20, 20, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    )
        self.battery_text_label.setText(text)

    # -------------------------------------------------------------------------
    # Signals Dispatchers
    # -------------------------------------------------------------------------
    def set_nav_feedback(self, data: dict[str, Any]) -> None:
        """Update live progress bar and metrics from Nav2 Action feedback."""
        dist = float(data.get("distance_remaining", 0.0))
        init_dist = float(data.get("initial_distance", dist))
        nav_sec = float(data.get("navigation_time", 0.0))
        est_sec = float(data.get("estimated_time_remaining", 0.0))
        recov = int(data.get("number_of_recoveries", 0))

        # Progress %
        if init_dist > 0.05:
            pct = max(0, min(100, int((init_dist - dist) / init_dist * 100)))
            self.nav_progress_bar.setValue(pct)

        self.lbl_dist_remain.setText(f"Cự ly còn lại: {dist:.2f} m")
        self.lbl_nav_time.setText(f"Thời gian chạy: {nav_sec:.1f} s")
        self.lbl_est_time.setText(f"Ước tính: {est_sec:.1f} s")
        self.lbl_recovery_count.setText(f"Tự cứu: {recov} lần")

        now = time.monotonic()
        if now - self._last_log_feedback_time >= 3.0:
            self._last_log_feedback_time = now
            self.append_patrol_log(
                f"[NAV][DEBUG] Tiến độ: Còn lại {dist:.2f} m | Thời gian chạy: {nav_sec:.1f} s | Ước tính: {est_sec:.1f} s | Tự cứu: {recov}",
                "progress"
            )

    def load_map_yaml(self, yaml_path: Path | str | None) -> bool:
        """Load and render map from YAML file, and automatically load corresponding map waypoints."""
        map_data = parse_nav_map_yaml(yaml_path)
        if map_data is not None:
            self.set_map(map_data)
            self._current_map_path = Path(yaml_path).resolve() if yaml_path else None
            stem = self._current_map_path.stem if self._current_map_path else "bản đồ"

            wp_file = self._get_map_waypoints_path(self._current_map_path)
            if wp_file and wp_file.is_file():
                self._load_patrol_from_file(wp_file, silent=True)
                n = len(self._batch_waypoints)
                self.status_label.setText(f"Đã nạp bản đồ: {stem} ({n} điểm waypoint)")
            else:
                # Clear waypoints on UI for maps that don't have saved waypoints yet
                # Protect global locations.yaml & routes.yaml from being overwritten
                self._clear_batch_waypoints()
                self.status_label.setText(f"Đã nạp bản đồ: {stem} (Chưa có điểm, hãy chấm điểm mới)")
            self._update_start_pose_state()
            return True
        return False

    def set_active(self, active: bool) -> None:
        """Thông báo trạng thái hiển thị của trang Navigation để tối ưu hóa render map."""
        self._is_active = active
        current_tab = self.stack.currentIndex() if hasattr(self, "stack") else 0
        self.map_widget.set_active(active and current_tab == 0)
        self.patrol_map_widget.set_active(active and current_tab == 1)

    def set_map(self, map_data: dict[str, Any]) -> None:
        self.map_widget.set_map(map_data)
        self.patrol_map_widget.set_map(map_data)

    def set_pose(self, pose: dict[str, Any]) -> None:
        self._current_robot_pose = pose
        self.map_widget.set_pose(pose)
        self.patrol_map_widget.set_pose(pose)

    def set_path(self, path: list[list[float]]) -> None:
        self.map_widget.set_path(path)
        self.patrol_map_widget.set_path(path)

    def set_locations(self, locations: list[Any]) -> None:
        self.map_widget.set_locations(locations)
        self.patrol_map_widget.set_locations(locations)

    def set_scan_points(self, points: list[tuple[float, float]]) -> None:
        self.map_widget.set_scan_points(points)
        self.patrol_map_widget.set_scan_points(points)

    def set_obstacle_points(self, side: str, points: list[tuple[float, float]]) -> None:
        self.map_widget.set_obstacle_points(side, points)
        self.patrol_map_widget.set_obstacle_points(side, points)

    def set_navigation_status(self, status: dict[str, Any]) -> None:
        state = str(status.get("status", "IDLE")).upper()
        if state in {"MOVING", "FETCHING_LOCATION", "WAITING_FOR_NAV2"}:
            self.set_navigation_active(True)
        elif state in {"REACHED_GOAL", "CANCELLED", "FAILED"}:
            self.set_navigation_active(False)

    def set_command_status(self, tone: str, message: str) -> None:
        color_map = {
            "success": "#10b981",  # Green
            "error": "#ef4444",    # Red
            "warning": "#f59e0b",  # Amber/Yellow
            "progress": "#38bdf8", # Sky blue
        }
        color = color_map.get(tone, "#94a3b8")
        self.status_label.setStyleSheet(f"color: {color}; font-weight: 600; font-size: 13px;")
        self.status_label.setText(message)
        self.append_patrol_log(message, tone)
        if tone in {"success", "error", "warning"}:
            self.set_navigation_active(False)
            if tone == "success":
                self.nav_progress_bar.setValue(100)

    def set_robot_status(self, status: dict[str, Any]) -> None:
        battery = status.get("battery_percent")
        if battery is None:
            battery = status.get("battery")
        if isinstance(battery, (int, float)):
            self.set_battery(float(battery))
        else:
            self.set_battery(None)

    def _update_clock(self) -> None:
        now = datetime.now()
        self.time_label.setText(now.strftime("%H:%M\n%d/%m"))

    def update_patrol_state(self, is_patrolling: bool) -> None:
        if not hasattr(self, "patrol_button"):
            return
        icon_path = self.icons.get("stop") if is_patrolling else self.icons.get("start")
        tooltip = "Tạm dừng tuần tra" if is_patrolling else "Bắt đầu tuần tra"
        self.patrol_button.setToolTip(tooltip)
        if icon_path and Path(icon_path).is_file():
            self.patrol_button.setIcon(QIcon(str(icon_path)))
            self.patrol_button.setText("")
        else:
            self.patrol_button.setText("⏸" if is_patrolling else "▶")
            self.patrol_button.setStyleSheet(
                "color: #f59e0b; font-size: 16px; font-weight: 700;" if is_patrolling
                else "color: #10b981; font-size: 16px; font-weight: 700;"
            )
