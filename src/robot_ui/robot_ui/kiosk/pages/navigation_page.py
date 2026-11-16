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

import math
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from PyQt5.QtCore import QPoint, QPointF, QRectF, QSize, Qt, QTimer, pyqtSignal
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
)
from PyQt5.QtWidgets import (
    QFrame,
    QGraphicsOpacityEffect,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSizePolicy,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
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
    """Set each patrol waypoint's yaw (rad) to face toward the next waypoint in the cycle."""
    n = len(waypoints)
    if n < 2:
        if n == 1:
            waypoints[0]["yaw"] = 0.0
        return waypoints
    for i, wp in enumerate(waypoints):
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

        if negate:
            occ = arr_flipped.astype(np.float32) / 255.0
        else:
            occ = (255.0 - arr_flipped.astype(np.float32)) / 255.0

        cells = np.full((h, w), -1, dtype=np.int16)
        cells[occ < free_thresh] = 0
        cells[occ > occupied_thresh] = 100

        max_dim = 1024
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
    invalid_goal = pyqtSignal(str)

    def __init__(
        self,
        goal_icon: Path | None = None,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setObjectName("interactiveNavMapWidget")
        self.setAttribute(Qt.WA_StyledBackground, True)
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

        # Sensor points in world frame (map coordinates)
        self._scan_points: list[tuple[float, float]] = []
        self._front_obs_points: list[tuple[float, float]] = []
        self._rear_obs_points: list[tuple[float, float]] = []

        # Zoom & Pan State
        self._zoom_factor = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._is_panning = False
        self._last_pan_pos: QPoint | None = None
        self._selection_enabled = True

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
        """Create on-map touch zoom buttons (+, -, reset) in the top-right corner."""
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

        btn_layout.addWidget(self.btn_zoom_in)
        btn_layout.addWidget(self.btn_zoom_out)
        btn_layout.addWidget(self.btn_zoom_reset)
        self.setLayout(btn_layout)

    # -------------------------------------------------------------------------
    # Zoom & Pan Control
    # -------------------------------------------------------------------------
    def zoom_in(self) -> None:
        self.set_zoom(self._zoom_factor * 1.25)

    def zoom_out(self) -> None:
        self.set_zoom(self._zoom_factor / 1.25)

    def reset_view(self) -> None:
        self._zoom_factor = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
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

    def set_map(self, map_data: dict[str, Any]) -> None:
        self._map = map_data
        width = int(map_data.get("preview_width", 0))
        height = int(map_data.get("preview_height", 0))
        cells = np.asarray(map_data.get("cells", []), dtype=np.int16)
        if width <= 0 or height <= 0 or cells.size < width * height:
            self._map_image = QImage()
            self.update()
            return
        cells = cells[: width * height].reshape((height, width))
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[cells < 0] = (30, 36, 44)                    # Unknown: Dark slate
        rgb[(cells >= 0) & (cells < 50)] = (235, 238, 242) # Free space: Off-white
        rgb[cells >= 50] = (15, 23, 42)                  # Obstacle: Black/Navy
        rgb = np.ascontiguousarray(np.flipud(rgb))
        self._map_image = QImage(
            rgb.data, width, height, width * 3, QImage.Format_RGB888
        ).copy()
        self.update()

    def set_pose(self, pose: dict[str, Any]) -> None:
        self._pose = pose
        self.update()

    def set_path(self, path: list[list[float]]) -> None:
        self._path = path
        self.update()

    def set_locations(self, locations: list[dict[str, Any]]) -> None:
        self._locations = [item for item in locations if isinstance(item, dict)]
        self.update()

    def set_goal(self, x: float, y: float, yaw: float) -> None:
        self._goal = {"x": x, "y": y, "yaw": yaw}
        self.update()

    def clear_goal(self) -> None:
        self._goal = None
        self.update()

    def set_batch_waypoints(self, waypoints: list[dict[str, Any]]) -> None:
        self._batch_waypoints = list(waypoints)
        self.update()

    def set_pick_mode(self, enabled: bool) -> None:
        self._pick_mode = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.PointingHandCursor)
        self.update()

    def set_selection_enabled(self, enabled: bool) -> None:
        self._selection_enabled = enabled
        if not self._pick_mode:
            self.setCursor(Qt.PointingHandCursor if enabled else Qt.ArrowCursor)

    def set_scan_points(self, points: list[tuple[float, float]]) -> None:
        self._scan_points = points
        self.update()

    def set_obstacle_points(self, side: str, points: list[tuple[float, float]]) -> None:
        if side == "front":
            self._front_obs_points = points
        elif side == "rear":
            self._rear_obs_points = points
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

    def _screen_to_world(self, point: QPointF, target: QRectF) -> tuple[float, float] | None:
        if not self._map or not target.contains(point):
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
    def wheelEvent(self, event: Any) -> None:
        delta = event.angleDelta().y()
        if delta == 0:
            return
        factor = 1.15 if delta > 0 else (1.0 / 1.15)
        self.set_zoom(self._zoom_factor * factor)
        event.accept()

    def mousePressEvent(self, event: Any) -> None:
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self._is_panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        if event.button() == Qt.LeftButton and self._selection_enabled:
            target = self._transformed_map_rect()
            world = self._screen_to_world(QPointF(event.pos()), target)
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
            self._pan_x += delta.x()
            self._pan_y += delta.y()
            self._last_pan_pos = event.pos()
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event: Any) -> None:
        if event.button() in (Qt.RightButton, Qt.MiddleButton) and self._is_panning:
            self._is_panning = False
            self._last_pan_pos = None
            self.setCursor(Qt.CrossCursor if self._pick_mode else Qt.PointingHandCursor)
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
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

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

        # 1. Map Image
        painter.drawImage(target, self._map_image)

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

                # Name badge
                name = str(wp.get("name", f"P{i}"))
                badge_w = painter.fontMetrics().horizontalAdvance(name) + 12
                badge_rect = QRectF(pt.x() - badge_w / 2.0, pt.y() - 26, badge_w, 18)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(17, 20, 24, 220))
                painter.drawRoundedRect(badge_rect, 4, 4)
                painter.setPen(node_color)
                painter.drawText(badge_rect, Qt.AlignCenter, name)

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

        painter.restore()

        # Border
        painter.setPen(QPen(QColor("#242c36"), 2))
        painter.drawRoundedRect(QRectF(self.rect()).adjusted(1, 1, -1, -1), 10, 10)

        # Zoom & Status HUD in bottom-left corner
        hud_text = f"Zoom: {self._zoom_factor:.1f}x"
        if self._pick_mode:
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


class NavigationPage(QWidget):
    """Full-featured Kiosk Navigation & Patrol Dashboard."""

    home_requested = pyqtSignal()
    back_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    fullscreen_toggle_requested = pyqtSignal()
    start_requested = pyqtSignal(float, float, float)     # (x, y, yaw)
    cancel_requested = pyqtSignal()
    start_patrol_requested = pyqtSignal()
    pause_patrol_requested = pyqtSignal()
    reload_route_requested = pyqtSignal()
    initial_pose_requested = pyqtSignal()

    def __init__(self, icons: dict[str, Path | None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("navigationPage")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.icons = icons

        self._config_dir = _find_config_dir()
        self._goal: tuple[float, float, float] | None = None
        self._navigation_active = False
        self._batch_waypoints: list[dict[str, Any]] = []

        self._init_ui()
        self._load_patrol_from_yaml(silent=True)

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
        layout.addLayout(chips_bar)

        # Live Map
        self.map_widget = InteractiveNavMapWidget(self.icons.get("goal"))
        self.map_widget.goal_selected.connect(self._on_single_goal_selected)
        self.map_widget.invalid_goal.connect(self._show_warning)
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

        self.btn_initial_pose = QPushButton("ĐỊNH VỊ GỐC")
        self.btn_initial_pose.setObjectName("navToolBtn")
        self.btn_initial_pose.setToolTip("Gửi tọa độ xuất phát /initialpose để AMCL định vị robot tức thì")
        self.btn_initial_pose.clicked.connect(self._on_initial_pose_clicked)

        action_row.addWidget(self.status_label, 1)
        action_row.addWidget(self.btn_initial_pose)
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

        toolbar.addWidget(self.btn_pick_mode)
        toolbar.addWidget(self.btn_undo_wp)
        toolbar.addWidget(self.btn_clear_wp)
        toolbar.addStretch(1)
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
        t_layout.addWidget(self.table_waypoints, 1)

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

    def _undo_batch_waypoint(self) -> None:
        if self._batch_waypoints:
            self._batch_waypoints.pop()
            recompute_cyclic_waypoint_yaws(self._batch_waypoints)
            self._sync_views_from_waypoints()

    def _clear_batch_waypoints(self) -> None:
        if not self._batch_waypoints:
            return
        self._batch_waypoints.clear()
        self._sync_views_from_waypoints()

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

        self._refresh_quick_location_chips()

    def _sync_waypoints_from_table(self) -> None:
        """Extract user edits from table into _batch_waypoints."""
        rows = self.table_waypoints.rowCount()
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
    # YAML Configuration Sync
    # -------------------------------------------------------------------------
    def _load_patrol_from_yaml(self, silent: bool = False) -> bool:
        loc_file = self._config_dir / "locations.yaml"
        route_file = self._config_dir / "routes.yaml"

        locations: dict[str, Any] = {}
        routes: dict[str, Any] = {}
        try:
            if loc_file.is_file():
                with loc_file.open("r", encoding="utf-8") as f:
                    locations = yaml.safe_load(f) or {}
            if route_file.is_file():
                with route_file.open("r", encoding="utf-8") as f:
                    routes = yaml.safe_load(f) or {}
        except Exception as exc:
            if not silent:
                QMessageBox.warning(self, "Lỗi đọc YAML", f"Không thể đọc file cấu hình:\n{exc}")
            return False

        patrol_names = routes.get("patrol_route")
        if not isinstance(patrol_names, list) or not patrol_names:
            patrol_names = list(locations.keys())

        loaded: list[dict[str, Any]] = []
        for name in patrol_names:
            coords = locations.get(name)
            if not isinstance(coords, dict):
                continue
            note = "Trạm nước / Khởi hành" if name == "khoihanh" else f"Điểm tuần tra {name}"
            loaded.append({
                "name": str(name),
                "x": float(coords.get("x", 0.0)),
                "y": float(coords.get("y", 0.0)),
                "yaw": float(coords.get("yaw", 0.0)),
                "note": note,
            })

        self._batch_waypoints = loaded
        recompute_cyclic_waypoint_yaws(self._batch_waypoints)
        self._sync_views_from_waypoints()
        if not silent:
            QMessageBox.information(
                self, "Nạp cấu hình thành công",
                f"Đã nạp {len(loaded)} điểm tuần tra từ config:\n- {loc_file.name}\n- {route_file.name}",
            )
        return True

    def _save_patrol_to_yaml(self) -> bool:
        self._sync_waypoints_from_table()
        if not self._batch_waypoints:
            QMessageBox.warning(self, "Chưa có điểm", "Hãy chấm ít nhất 1 điểm trên bản đồ trước khi lưu lộ trình.")
            return False

        loc_file = self._config_dir / "locations.yaml"
        route_file = self._config_dir / "routes.yaml"

        try:
            locations: dict[str, Any] = {}
            if loc_file.is_file():
                with loc_file.open("r", encoding="utf-8") as f:
                    locations = yaml.safe_load(f) or {}

            routes: dict[str, Any] = {}
            if route_file.is_file():
                with route_file.open("r", encoding="utf-8") as f:
                    routes = yaml.safe_load(f) or {}

            route_names: list[str] = []
            for wp in self._batch_waypoints:
                name = str(wp["name"])
                route_names.append(name)
                locations[name] = {
                    "x": float(wp["x"]),
                    "y": float(wp["y"]),
                    "yaw": float(wp.get("yaw", 0.0)),
                }
            routes["patrol_route"] = route_names

            with loc_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(locations, f, allow_unicode=True, sort_keys=False)
            with route_file.open("w", encoding="utf-8") as f:
                yaml.safe_dump(routes, f, allow_unicode=True, sort_keys=False)

            self.reload_route_requested.emit()
            self._sync_views_from_waypoints()
            QMessageBox.information(
                self, "Lưu lộ trình thành công",
                f"Đã lưu {len(self._batch_waypoints)} điểm và đồng bộ tới hệ thống:\n{' -> '.join(route_names)}",
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
        self.status_label.setText(f"Cảnh báo: {message}")

    def _start_single_navigation(self) -> None:
        if self._goal is None:
            self._show_warning("Hãy chọn điểm đến trên bản đồ")
            return
        self.set_navigation_active(True)
        self.start_requested.emit(*self._goal)

    def _stop_navigation(self) -> None:
        self.set_navigation_active(False)
        self.cancel_requested.emit()
        self.nav_progress_bar.setValue(0)
        self.lbl_dist_remain.setText("Cự ly còn lại: -- m")

    def _start_patrol(self) -> None:
        self.start_patrol_requested.emit()

    def _pause_patrol(self) -> None:
        self.pause_patrol_requested.emit()

    def _on_initial_pose_clicked(self) -> None:
        self.status_label.setText("📍 Đang gửi tọa độ xuất phát ban đầu để định vị robot...")
        self.initial_pose_requested.emit()

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

    def load_map_yaml(self, yaml_path: Path | str | None) -> bool:
        """Load and render map from YAML file into navigation and patrol map views."""
        map_data = parse_nav_map_yaml(yaml_path)
        if map_data is not None:
            self.set_map(map_data)
            stem = Path(yaml_path).stem if yaml_path else "bản đồ"
            self.status_label.setText(f"Đã nạp bản đồ: {stem}")
            return True
        return False

    def set_map(self, map_data: dict[str, Any]) -> None:
        self.map_widget.set_map(map_data)
        self.patrol_map_widget.set_map(map_data)

    def set_pose(self, pose: dict[str, Any]) -> None:
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
        self.status_label.setText(message)
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
