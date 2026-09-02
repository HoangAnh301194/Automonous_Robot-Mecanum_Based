import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGraphicsEllipseItem,
    QGraphicsItem,
    QGraphicsLineItem,
    QGraphicsPixmapItem,
    QGraphicsPolygonItem,
    QGraphicsScene,
    QGraphicsSimpleTextItem,
    QGraphicsView,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
    QAbstractItemView,
    QHeaderView,
)

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


# ============================================================
# Utilities
# ============================================================


def yaw_to_quat(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q



def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    status: str = "pending"  # pending / active / done


# ============================================================
# ROS bridge inside GUI process
# ============================================================


class GuiRosNode(Node):
    def __init__(self):
        super().__init__("route_gui_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("goal_reach_tolerance", 0.35)

        self.map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        self.goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.goal_reach_tolerance = (
            self.get_parameter("goal_reach_tolerance").get_parameter_value().double_value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.robot_pose_xy_yaw: Optional[Tuple[float, float, float]] = None

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.on_map, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def publish_goal(self, wp: Waypoint):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(wp.x)
        msg.pose.position.y = float(wp.y)
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quat(wp.yaw)
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Published goal {wp.name}: x={wp.x:.2f} y={wp.y:.2f} yaw={wp.yaw:.2f}"
        )

    def update_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            x = float(tf.transform.translation.x)
            y = float(tf.transform.translation.y)
            yaw = quat_to_yaw(tf.transform.rotation)
            self.robot_pose_xy_yaw = (x, y, yaw)
        except TransformException:
            self.robot_pose_xy_yaw = None


# ============================================================
# Map widget
# ============================================================


class MapView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QColor(30, 30, 30))

        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)

        self.map_item: Optional[QGraphicsPixmapItem] = None
        self.map_meta = None
        self.map_image_height_px = 0

        self.waypoint_items: List[QGraphicsItem] = []
        self.route_items: List[QGraphicsItem] = []
        self.robot_items: List[QGraphicsItem] = []

        self.add_waypoint_mode = False
        self.click_callback = None

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if self.add_waypoint_mode and event.button() == Qt.LeftButton and self.map_meta is not None:
            scene_pt = self.mapToScene(event.pos())
            map_xy = self.scene_to_map(scene_pt.x(), scene_pt.y())
            if map_xy is not None and self.click_callback is not None:
                self.click_callback(map_xy[0], map_xy[1])
            return
        super().mousePressEvent(event)

    def set_add_waypoint_mode(self, enabled: bool):
        self.add_waypoint_mode = enabled
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)

    def set_click_callback(self, cb):
        self.click_callback = cb

    def set_map(self, msg: OccupancyGrid):
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return

        data = np.array(msg.data, dtype=np.int16).reshape((h, w))
        img = np.zeros((h, w, 3), dtype=np.uint8)

        unknown = data < 0
        free = data == 0
        occ = data > 50
        mid = (~unknown) & (~free) & (~occ)

        img[unknown] = [127, 127, 127]
        img[free] = [255, 255, 255]
        img[occ] = [0, 0, 0]
        img[mid] = [180, 180, 180]

        img = np.flipud(img)
        self.map_image_height_px = h

        data_bytes = img.data.tobytes() 


        qimg = QImage(data_bytes, w, h, 3 * w, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(qimg)

        if self.map_item is not None:
            self.scene.removeItem(self.map_item)

        self.map_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(self.map_item)
        self.scene.setSceneRect(QRectF(0, 0, w, h))
        self.map_meta = msg.info

    def clear_graphics(self, items: List[QGraphicsItem]):
        for item in items:
            self.scene.removeItem(item)
        items.clear()

    def redraw_waypoints(self, waypoints: List[Waypoint], active_idx: int):
        self.clear_graphics(self.waypoint_items)
        self.clear_graphics(self.route_items)

        if self.map_meta is None:
            return

        points_scene = []
        for i, wp in enumerate(waypoints):
            sx, sy = self.map_to_scene(wp.x, wp.y)
            points_scene.append((sx, sy))

            if i == active_idx:
                color = QColor(255, 140, 0)
            elif wp.status == "done":
                color = QColor(0, 170, 0)
            else:
                color = QColor(30, 144, 255)

            radius = 4.0
            ellipse = self.scene.addEllipse(
                sx - radius,
                sy - radius,
                radius * 2,
                radius * 2,
                QPen(color, 1.5),
                color,
            )
            self.waypoint_items.append(ellipse)

            label = self.scene.addSimpleText(wp.name)
            label.setBrush(color)
            label.setPos(sx + 5, sy - 14)
            self.waypoint_items.append(label)

            dx = math.cos(wp.yaw) * 12.0
            dy = -math.sin(wp.yaw) * 12.0
            line = self.scene.addLine(sx, sy, sx + dx, sy + dy, QPen(color, 1.2))
            self.waypoint_items.append(line)

        for i in range(len(points_scene) - 1):
            x1, y1 = points_scene[i]
            x2, y2 = points_scene[i + 1]
            line = self.scene.addLine(x1, y1, x2, y2, QPen(QColor(255, 215, 0), 1.0, Qt.DashLine))
            self.route_items.append(line)

    def redraw_robot(self, robot_pose: Optional[Tuple[float, float, float]]):
        self.clear_graphics(self.robot_items)
        if self.map_meta is None or robot_pose is None:
            return

        x, y, yaw = robot_pose
        sx, sy = self.map_to_scene(x, y)

        size = 10.0
        pts = [
            QPointF(size, 0),
            QPointF(-size * 0.6, size * 0.6),
            QPointF(-size * 0.4, 0),
            QPointF(-size * 0.6, -size * 0.6),
        ]

        rotated = []
        for p in pts:
            rx = p.x() * math.cos(-yaw) - p.y() * math.sin(-yaw)
            ry = p.x() * math.sin(-yaw) + p.y() * math.cos(-yaw)
            rotated.append(QPointF(sx + rx, sy + ry))

        poly = QPolygonF(rotated)
        item = QGraphicsPolygonItem(poly)
        item.setPen(QPen(QColor(220, 20, 60), 1.5))
        item.setBrush(QColor(220, 20, 60))
        self.scene.addItem(item)
        self.robot_items.append(item)

    def map_to_scene(self, mx: float, my: float) -> Tuple[float, float]:
        info = self.map_meta
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        px = (mx - ox) / res
        py = (my - oy) / res
        sy = self.map_image_height_px - py
        return px, sy

    def scene_to_map(self, sx: float, sy: float) -> Optional[Tuple[float, float]]:
        if self.map_meta is None:
            return None
        info = self.map_meta
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        py = self.map_image_height_px - sy
        mx = ox + sx * res
        my = oy + py * res
        return mx, my


# ============================================================
# Main window
# ============================================================


class MainWindow(QMainWindow):
    def __init__(self, ros_node: GuiRosNode):
        super().__init__()
        self.ros = ros_node

        self.setWindowTitle("ROS2 Route GUI - Single File (PyQt5)")
        self.resize(1400, 850)

        self.waypoints: List[Waypoint] = []
        self.current_index: int = -1
        self.route_running = False
        self.route_paused = False
        self.last_map_seq = None

        self._build_ui()

        self.map_view.set_click_callback(self.on_map_clicked)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.on_ros_tick)
        self.ros_timer.start(50)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.refresh_ui)
        self.ui_timer.start(150)

    # ---------------- UI ----------------
    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        # Left panel
        left = QWidget()
        left_layout = QVBoxLayout(left)

        ctrl_box = QGroupBox("Route Control")
        ctrl_layout = QVBoxLayout(ctrl_box)
        self.btn_add_mode = QPushButton("Add Waypoint Mode")
        self.btn_start = QPushButton("Start Route")
        self.btn_pause = QPushButton("Pause")
        self.btn_resume = QPushButton("Resume")
        self.btn_cancel = QPushButton("Cancel")
        self.btn_clear = QPushButton("Clear Route")
        self.chk_auto_zoom = QCheckBox("Fit map on first load")
        self.chk_auto_zoom.setChecked(True)

        ctrl_layout.addWidget(self.btn_add_mode)
        ctrl_layout.addWidget(self.btn_start)
        ctrl_layout.addWidget(self.btn_pause)
        ctrl_layout.addWidget(self.btn_resume)
        ctrl_layout.addWidget(self.btn_cancel)
        ctrl_layout.addWidget(self.btn_clear)
        ctrl_layout.addWidget(self.chk_auto_zoom)
        left_layout.addWidget(ctrl_box)

        param_box = QGroupBox("Waypoint Editor")
        param_layout = QFormLayout(param_box)
        self.spin_x = QDoubleSpinBox()
        self.spin_y = QDoubleSpinBox()
        self.spin_yaw = QDoubleSpinBox()
        for s in (self.spin_x, self.spin_y, self.spin_yaw):
            s.setRange(-9999.0, 9999.0)
            s.setDecimals(3)
            s.setSingleStep(0.1)
        self.spin_yaw.setRange(-math.pi, math.pi)

        self.btn_apply_edit = QPushButton("Apply Edit")
        self.btn_delete = QPushButton("Delete Selected")
        self.btn_up = QPushButton("Move Up")
        self.btn_down = QPushButton("Move Down")

        param_layout.addRow("X", self.spin_x)
        param_layout.addRow("Y", self.spin_y)
        param_layout.addRow("Yaw", self.spin_yaw)
        param_layout.addRow(self.btn_apply_edit)
        param_layout.addRow(self.btn_delete)
        param_layout.addRow(self.btn_up)
        param_layout.addRow(self.btn_down)
        left_layout.addWidget(param_box)

        status_box = QGroupBox("Status")
        status_layout = QFormLayout(status_box)
        self.lbl_route = QLabel("idle")
        self.lbl_robot = QLabel("-")
        self.lbl_goal = QLabel("-")
        self.lbl_map = QLabel("no map")
        status_layout.addRow("Route", self.lbl_route)
        status_layout.addRow("Robot", self.lbl_robot)
        status_layout.addRow("Current Goal", self.lbl_goal)
        status_layout.addRow("Map", self.lbl_map)
        left_layout.addWidget(status_box)
        left_layout.addStretch(1)

        # Center map
        center = QWidget()
        center_layout = QVBoxLayout(center)
        self.map_view = MapView()
        center_layout.addWidget(self.map_view)

        # Right panel
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.addWidget(QLabel("Waypoints"))
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Name", "X", "Y", "Yaw", "Status"])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        right_layout.addWidget(self.table)

        splitter.addWidget(left)
        splitter.addWidget(center)
        splitter.addWidget(right)
        splitter.setSizes([260, 820, 320])

        self.btn_add_mode.clicked.connect(self.toggle_add_mode)
        self.btn_start.clicked.connect(self.start_route)
        self.btn_pause.clicked.connect(self.pause_route)
        self.btn_resume.clicked.connect(self.resume_route)
        self.btn_cancel.clicked.connect(self.cancel_route)
        self.btn_clear.clicked.connect(self.clear_route)
        self.btn_apply_edit.clicked.connect(self.apply_selected_edit)
        self.btn_delete.clicked.connect(self.delete_selected)
        self.btn_up.clicked.connect(self.move_selected_up)
        self.btn_down.clicked.connect(self.move_selected_down)
        self.table.itemSelectionChanged.connect(self.on_table_selection_changed)

    # ---------------- ROS / UI tick ----------------
    def on_ros_tick(self):
        rclpy.spin_once(self.ros, timeout_sec=0.0)
        self.ros.update_robot_pose()

        map_msg = self.ros.map_msg
        if map_msg is not None:
            seq = (map_msg.header.stamp.sec, map_msg.header.stamp.nanosec)
            if seq != self.last_map_seq:
                self.last_map_seq = seq
                self.map_view.set_map(map_msg)
                self.lbl_map.setText(
                    f"{map_msg.info.width}x{map_msg.info.height}, res={map_msg.info.resolution:.3f}"
                )
                if self.chk_auto_zoom.isChecked():
                    self.map_view.fitInView(self.map_view.scene.sceneRect(), Qt.KeepAspectRatio)
                    self.chk_auto_zoom.setChecked(False)

        if self.route_running and not self.route_paused:
            self.check_goal_progress()

    def refresh_ui(self):
        self.update_waypoint_table()
        self.map_view.redraw_waypoints(self.waypoints, self.current_index)
        self.map_view.redraw_robot(self.ros.robot_pose_xy_yaw)

        if self.route_running:
            route_text = "paused" if self.route_paused else "running"
        else:
            route_text = "idle"
        self.lbl_route.setText(route_text)

        rp = self.ros.robot_pose_xy_yaw
        if rp is None:
            self.lbl_robot.setText("TF unavailable")
        else:
            self.lbl_robot.setText(f"x={rp[0]:.2f}, y={rp[1]:.2f}, yaw={rp[2]:.2f}")

        if 0 <= self.current_index < len(self.waypoints):
            wp = self.waypoints[self.current_index]
            self.lbl_goal.setText(f"{wp.name}: ({wp.x:.2f}, {wp.y:.2f}, {wp.yaw:.2f})")
        else:
            self.lbl_goal.setText("-")

    # ---------------- Route logic (Direction 1) ----------------
    def start_route(self):
        if not self.waypoints:
            QMessageBox.warning(self, "No waypoints", "Hãy thêm ít nhất 1 waypoint.")
            return

        for wp in self.waypoints:
            wp.status = "pending"

        self.current_index = 0
        self.route_running = True
        self.route_paused = False
        self.send_current_goal()

    def pause_route(self):
        if not self.route_running:
            return
        self.route_paused = True

    def resume_route(self):
        if not self.route_running:
            return
        self.route_paused = False

    def cancel_route(self):
        self.route_running = False
        self.route_paused = False
        self.current_index = -1
        for wp in self.waypoints:
            if wp.status != "done":
                wp.status = "pending"

    def clear_route(self):
        self.cancel_route()
        self.waypoints.clear()
        self.update_waypoint_table()

    def send_current_goal(self):
        if not (0 <= self.current_index < len(self.waypoints)):
            self.route_running = False
            self.current_index = -1
            return

        for i, wp in enumerate(self.waypoints):
            if i < self.current_index:
                wp.status = "done"
            elif i == self.current_index:
                wp.status = "active"
            else:
                wp.status = "pending"

        self.ros.publish_goal(self.waypoints[self.current_index])

    def check_goal_progress(self):
        if not (0 <= self.current_index < len(self.waypoints)):
            return
        robot = self.ros.robot_pose_xy_yaw
        if robot is None:
            return

        wp = self.waypoints[self.current_index]
        dist = math.hypot(robot[0] - wp.x, robot[1] - wp.y)
        if dist <= self.ros.goal_reach_tolerance:
            wp.status = "done"
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.route_running = False
                self.current_index = -1
                return
            self.send_current_goal()

    # ---------------- Waypoint management ----------------
    def next_name(self) -> str:
        idx = len(self.waypoints)
        alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        if idx < len(alphabet):
            return alphabet[idx]
        return f"P{idx + 1}"

    def toggle_add_mode(self):
        enabled = not self.map_view.add_waypoint_mode
        self.map_view.set_add_waypoint_mode(enabled)
        self.btn_add_mode.setText("Adding... click map" if enabled else "Add Waypoint Mode")

    def on_map_clicked(self, x: float, y: float):
        wp = Waypoint(name=self.next_name(), x=x, y=y, yaw=0.0)
        self.waypoints.append(wp)
        self.update_waypoint_table()
        self.select_row(len(self.waypoints) - 1)

    def update_waypoint_table(self):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.waypoints))
        for row, wp in enumerate(self.waypoints):
            vals = [
                wp.name,
                f"{wp.x:.3f}",
                f"{wp.y:.3f}",
                f"{wp.yaw:.3f}",
                wp.status,
            ]
            for col, val in enumerate(vals):
                item = self.table.item(row, col)
                if item is None:
                    item = QTableWidgetItem()
                    self.table.setItem(row, col, item)
                item.setText(val)
                item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        self.table.blockSignals(False)

    def current_row(self) -> int:
        rows = self.table.selectionModel().selectedRows()
        if not rows:
            return -1
        return rows[0].row()

    def select_row(self, row: int):
        if 0 <= row < self.table.rowCount():
            self.table.selectRow(row)

    def on_table_selection_changed(self):
        row = self.current_row()
        if not (0 <= row < len(self.waypoints)):
            return
        wp = self.waypoints[row]
        self.spin_x.setValue(wp.x)
        self.spin_y.setValue(wp.y)
        self.spin_yaw.setValue(wp.yaw)

    def apply_selected_edit(self):
        row = self.current_row()
        if not (0 <= row < len(self.waypoints)):
            return
        wp = self.waypoints[row]
        wp.x = self.spin_x.value()
        wp.y = self.spin_y.value()
        wp.yaw = self.spin_yaw.value()
        self.update_waypoint_table()

    def delete_selected(self):
        row = self.current_row()
        if not (0 <= row < len(self.waypoints)):
            return
        del self.waypoints[row]
        if self.current_index >= len(self.waypoints):
            self.current_index = len(self.waypoints) - 1
        self.renumber_names()
        self.update_waypoint_table()

    def move_selected_up(self):
        row = self.current_row()
        if row <= 0:
            return
        self.waypoints[row - 1], self.waypoints[row] = self.waypoints[row], self.waypoints[row - 1]
        self.renumber_names()
        self.update_waypoint_table()
        self.select_row(row - 1)

    def move_selected_down(self):
        row = self.current_row()
        if row < 0 or row >= len(self.waypoints) - 1:
            return
        self.waypoints[row + 1], self.waypoints[row] = self.waypoints[row], self.waypoints[row + 1]
        self.renumber_names()
        self.update_waypoint_table()
        self.select_row(row + 1)

    def renumber_names(self):
        alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        for i, wp in enumerate(self.waypoints):
            wp.name = alphabet[i] if i < len(alphabet) else f"P{i + 1}"

    def closeEvent(self, event):
        self.ros.destroy_node()
        rclpy.shutdown()
        super().closeEvent(event)


# ============================================================
# Main entry
# ============================================================


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    ros_node = GuiRosNode()
    win = MainWindow(ros_node)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

