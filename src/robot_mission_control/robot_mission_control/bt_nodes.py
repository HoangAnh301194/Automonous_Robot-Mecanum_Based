import math
import time
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from robot_interfaces.action import DirectGo
from robot_interfaces.srv import GetRoute

from .bt_core import NodeStatus, Condition, Action, Blackboard

# ==========================================
# CONDITIONS
# ==========================================

class IsEmergency(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        lidar_ok = blackboard.get('SYSTEM_lidar_ok', True)
        odom_ok = blackboard.get('SYSTEM_odom_ok', True)
        localization_ok = blackboard.get('SYSTEM_localization_ok', True)

        if not lidar_ok or not odom_ok or not localization_ok:
            return NodeStatus.SUCCESS
        return NodeStatus.FAILURE


class IsBatteryLow(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        battery_low = blackboard.get('SYSTEM_battery_low', False)
        return NodeStatus.SUCCESS if battery_low else NodeStatus.FAILURE


class IsWaterLow(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        water_low = blackboard.get('SYSTEM_water_low', False)
        return NodeStatus.SUCCESS if water_low else NodeStatus.FAILURE


class IsServingCustomer(Condition):
    """Đang trong chế độ phục vụ nước (khóa xe đứng yên)."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        is_serving = blackboard.get('IS_SERVING', False)
        return NodeStatus.SUCCESS if is_serving else NodeStatus.FAILURE


class IsHandWavingDetected(Condition):
    """Ưu tiên cao nhất khi chạy: Có người giơ tay vẫy gọi xe hoặc đang tiếp cận người vẫy."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if blackboard.get('IS_SERVING', False):
            return NodeStatus.FAILURE

        if blackboard.get('APPROACH_ACTIVE', False):
            return NodeStatus.SUCCESS

        if time.time() < blackboard.get('WAVE_COOLDOWN_UNTIL', 0.0):
            return NodeStatus.FAILURE

        is_waving = blackboard.get('HAND_WAVE_ACTIVE', False)
        return NodeStatus.SUCCESS if is_waving else NodeStatus.FAILURE


class IsCustomerEncountered(Condition):
    """Gặp người đứng chắn đường (cự ly < 1.8m, không vẫy) hoặc đang giữ đếm ngược 5s."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if (
            blackboard.get('IS_SERVING', False)
            or blackboard.get('HAND_WAVE_ACTIVE', False)
            or blackboard.get('APPROACH_ACTIVE', False)
        ):
            return NodeStatus.FAILURE

        # Giữ khóa đủ 5 giây đếm ngược mời nước ngay cả khi người hơi nhích khỏi khung hình.
        if blackboard.get('ENCOUNTER_ACTIVE', False):
            return NodeStatus.SUCCESS

        cooldown_until = blackboard.get('ENCOUNTER_COOLDOWN_UNTIL', 0.0)
        if time.time() < cooldown_until:
            return NodeStatus.FAILURE

        person_in_path = blackboard.get('PERSON_IN_PATH', False)
        return NodeStatus.SUCCESS if person_in_path else NodeStatus.FAILURE


class IsPatrolRequested(Condition):
    """Chế độ tuần tra mặc định."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        patrol_requested = blackboard.get('HMI_start_patrol', True)
        return NodeStatus.SUCCESS if patrol_requested else NodeStatus.FAILURE


# ==========================================
# ACTIONS
# ==========================================

class StopRobot(Action):
    def __init__(self, name="StopRobot", node=None):
        super().__init__(name)
        self.node = node
        self._last_log_time = 0.0

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        now = time.monotonic()
        if self.status != NodeStatus.RUNNING:
            if self.node and hasattr(self.node, 'stop_robot_motion'):
                self.node.stop_robot_motion(cancel_nav2=True)
            self.status = NodeStatus.RUNNING
        elif self.node and hasattr(self.node, 'publish_zero_vel'):
            self.node.publish_zero_vel()

        if self.node and (now - self._last_log_time >= 5.0):
            self._last_log_time = now
            self.node.get_logger().error("EMERGENCY STOP! Canceling all navigation.")
            self.node.publish_log("[KHẨN CẤP] Cảm biến gặp sự cố! Dừng xe ngay lập tức.")
        return NodeStatus.RUNNING

    def halt(self, blackboard: Blackboard = None) -> None:
        self._last_log_time = 0.0
        self.status = NodeStatus.FAILURE


class ActionGoCharging(Action):
    def __init__(self, name="ActionGoCharging", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            msg = "CẢNH BÁO: Pin yếu! Đang về trạm sạc..."
            if self.node:
                if hasattr(self.node, 'stop_robot_motion'):
                    self.node.stop_robot_motion(cancel_nav2=True)
                self.node.get_logger().warn(msg)
                self.node.publish_log(msg)
            self.start_time = time.time()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if time.time() - self.start_time > 5.0:
            msg = "Đã đến trạm sạc. Đang sạc..."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def halt(self, blackboard: Blackboard = None) -> None:
        self.start_time = None
        self.status = NodeStatus.FAILURE


class ActionGoRefill(Action):
    def __init__(self, name="ActionGoRefill", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            msg = "CẢNH BÁO: Gần hết nước! Đang về trạm cấp nước..."
            if self.node:
                if hasattr(self.node, 'stop_robot_motion'):
                    self.node.stop_robot_motion(cancel_nav2=True)
                self.node.get_logger().warn(msg)
                self.node.publish_log(msg)
            self.start_time = time.time()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if time.time() - self.start_time > 5.0:
            msg = "Đã bơm nước xong. Tiếp tục nhiệm vụ."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
            blackboard.set('SYSTEM_water_low', False)
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def halt(self, blackboard: Blackboard = None) -> None:
        self.start_time = None
        self.status = NodeStatus.FAILURE


# ==========================================
# SMART AUTONOMOUS ACTIONS
# ==========================================

class ActionApproachWavingPerson(Action):
    """
    Ưu tiên cao khi phát hiện người giơ tay vẫy:
    1. Nhánh Fallback tự động halt ActionPatrol để hủy goal tuần tra cũ.
    2. Lấy tọa độ 3D người vẫy tay từ AI (chỉ dùng khi position_valid hoặc có preview map).
    3. Tính điểm đỗ an toàn cách người 0.9m.
    4. Điều khiển Nav2 chạy lại trước mặt người đó.
    5. Chỉ khi Nav2 báo STATUS_SUCCEEDED -> Chuyển sang CHẾ ĐỘ PHỤC VỤ (IS_SERVING = True).
    """
    def __init__(self, name="ActionApproachWavingPerson", node=None):
        super().__init__(name)
        self.node = node
        self.nav2_client = None
        self.goal_handle = None
        self.is_navigating = False
        self.standoff_distance = 0.90  # Đỗ cách người 90cm an toàn
        self._goal_seq = 0

        if self.node:
            self.nav2_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def _cancel_active_goal(self):
        self._goal_seq += 1
        if self.goal_handle is not None and self.is_navigating:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.is_navigating = False

    def _abort_approach(self, blackboard: Blackboard, reason: str):
        self._cancel_active_goal()
        blackboard.set('HAND_WAVE_ACTIVE', False)
        blackboard.set('APPROACH_ACTIVE', False)
        blackboard.set('WAVE_COOLDOWN_UNTIL', time.time() + 3.0)
        if self.node and reason:
            self.node.get_logger().warn(reason)
        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE

    def _resolve_target_pose_map(self, blackboard: Blackboard):
        waving_person = blackboard.get('WAVING_PERSON')
        if waving_person is not None and getattr(waving_person, 'position_valid', False):
            x_base = float(waving_person.position.x)
            y_base = float(waving_person.position.y)
            if math.isfinite(x_base) and math.isfinite(y_base):
                dist = math.hypot(x_base, y_base)
                angle = math.atan2(y_base, x_base)
                approach_dist = max(0.0, dist - self.standoff_distance)

                pose_base = PoseStamped()
                pose_base.header.stamp = self.node.get_clock().now().to_msg()
                pose_base.header.frame_id = 'base_footprint'
                pose_base.pose.position.x = approach_dist * math.cos(angle)
                pose_base.pose.position.y = approach_dist * math.sin(angle)
                pose_base.pose.position.z = 0.0
                pose_base.pose.orientation.z = math.sin(angle / 2.0)
                pose_base.pose.orientation.w = math.cos(angle / 2.0)

                try:
                    if self.node and hasattr(self.node, 'tf_buffer'):
                        transform = self.node.tf_buffer.lookup_transform(
                            'map', 'base_footprint', rclpy.time.Time(),
                            timeout=Duration(seconds=0.05)
                        )
                        import tf2_geometry_msgs
                        return tf2_geometry_msgs.do_transform_pose_stamped(pose_base, transform), dist
                except Exception as e:
                    if self.node:
                        self.node.get_logger().warn(f"TF transform lỗi: {e}")

        preview_pose = blackboard.get('PERSON_PREVIEW_MAP_POSE', None)
        if preview_pose is not None:
            return preview_pose, 0.0
        return None, 0.0

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            pose_map, dist = self._resolve_target_pose_map(blackboard)
            if pose_map is None:
                return self._abort_approach(blackboard, "Không thể xác định tọa độ map hợp lệ của người vẫy tay!")

            if not self.nav2_client or not self.nav2_client.server_is_ready():
                return self._abort_approach(blackboard, "Nav2 Action Server chưa sẵn sàng để tiếp cận người vẫy tay!")

            blackboard.set('APPROACH_ACTIVE', True)
            msg = f"👋 PHÁT HIỆN NGƯỜI VẪY TAY! Đang tiếp cận người (cách {dist:.2f}m)..."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            self.send_nav2_goal(pose_map)
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if self.is_navigating:
            return NodeStatus.RUNNING

        return self.status

    def send_nav2_goal(self, pose_map: PoseStamped):
        self._cancel_active_goal()
        seq = self._goal_seq

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_map
        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut, s=seq: self.goal_response_callback(fut, s))
        self.is_navigating = True

    def goal_response_callback(self, future, seq=None):
        if seq is not None and seq != self._goal_seq:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.is_navigating = False
            if self.node:
                self.node.get_logger().error(f"Lỗi gửi goal tiếp cận người: {exc}")
                self.node.blackboard.set('HAND_WAVE_ACTIVE', False)
                self.node.blackboard.set('APPROACH_ACTIVE', False)
                self.node.blackboard.set('WAVE_COOLDOWN_UNTIL', time.time() + 3.0)
            self.status = NodeStatus.FAILURE
            return

        if not goal_handle.accepted:
            self.is_navigating = False
            if self.node:
                self.node.get_logger().error("Goal tiếp cận người bị Nav2 từ chối!")
                self.node.blackboard.set('HAND_WAVE_ACTIVE', False)
                self.node.blackboard.set('APPROACH_ACTIVE', False)
                self.node.blackboard.set('WAVE_COOLDOWN_UNTIL', time.time() + 3.0)
            self.status = NodeStatus.FAILURE
            return

        self.goal_handle = goal_handle
        get_res_future = goal_handle.get_result_async()
        get_res_future.add_done_callback(lambda fut, s=seq: self.result_callback(fut, s))

    def result_callback(self, future, seq=None):
        if seq is not None and seq != self._goal_seq:
            return
        self.is_navigating = False
        self.goal_handle = None

        status = GoalStatus.STATUS_ABORTED
        try:
            res_wrapper = future.result()
            status = res_wrapper.status
        except Exception as exc:
            if self.node:
                self.node.get_logger().warn(f"Lỗi nhận kết quả tiếp cận người: {exc}")

        if self.node and hasattr(self.node, 'blackboard'):
            self.node.blackboard.set('HAND_WAVE_ACTIVE', False)
            self.node.blackboard.set('APPROACH_ACTIVE', False)

        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = "🎯 ĐÃ ĐẾN TRƯỚC MẶT NGƯỜI VẪY TAY! Chuyển sang CHẾ ĐỘ PHỤC VỤ."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.blackboard.set('IS_SERVING', True)
            self.status = NodeStatus.SUCCESS
        else:
            msg = f"⚠️ Không thể tiếp cận người vẫy tay (Nav2 status={status}), quay lại tuần tra."
            if self.node:
                self.node.get_logger().warn(msg)
                self.node.publish_log(msg)
                self.node.blackboard.set('WAVE_COOLDOWN_UNTIL', time.time() + 3.0)
            self.status = NodeStatus.FAILURE

    def halt(self, blackboard: Blackboard = None) -> None:
        self._cancel_active_goal()
        if blackboard is not None:
            blackboard.set('APPROACH_ACTIVE', False)
            blackboard.set('HAND_WAVE_ACTIVE', False)
        self.status = NodeStatus.FAILURE


class ActionCustomerEncounter(Action):
    """
    Khi gặp người đứng cản trước mặt (cự ly < 1.8m):
    1. Hủy goal Nav2/Patrol hiện tại và ghìm /cmd_vel = 0 đứng tĩnh đủ 5 giây.
    2. Bật Popup mời nước lên HMI Kiosk (SHOW_5S_INVITE).
    3. Nếu bấm "Phục vụ" -> IS_SERVING = True.
    4. Nếu hết 5s không bấm -> Chào nhẹ, đi tiếp và Cooldown 10s.
    """
    def __init__(self, name="ActionCustomerEncounter", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            self.start_time = time.time()
            blackboard.set('ENCOUNTER_ACTIVE', True)
            if self.node and hasattr(self.node, 'stop_robot_motion'):
                self.node.stop_robot_motion(cancel_nav2=True)

            msg = "👤 GẶP NGƯỜI TRƯỚC MẶT! Dừng tĩnh 5 giây mời nước..."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("SHOW_5S_INVITE")

            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if self.node and hasattr(self.node, 'publish_zero_vel'):
            self.node.publish_zero_vel()

        if blackboard.get('SERVE_REQUESTED', False):
            blackboard.set('SERVE_REQUESTED', False)
            blackboard.set('ENCOUNTER_ACTIVE', False)
            blackboard.set('IS_SERVING', True)
            self.start_time = None
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        if time.time() - self.start_time >= 5.0:
            msg = "⏱ Hết 5s không yêu cầu nước. Chúc Thầy/Cô một ngày tốt lành và tiếp tục tuần tra!"
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            blackboard.set('ENCOUNTER_ACTIVE', False)
            blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', time.time() + 10.0)
            self.start_time = None
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def halt(self, blackboard: Blackboard = None) -> None:
        if blackboard is not None:
            blackboard.set('ENCOUNTER_ACTIVE', False)
        self.start_time = None
        self.status = NodeStatus.FAILURE


class ActionServeCustomer(Action):
    """
    Chế độ Phục Vụ Nước (IS_SERVING == True):
    1. Hủy toàn bộ goal Nav2/Patrol đang chạy và khóa /cmd_vel = 0 đứng yên.
    2. Bật Popup phục vụ (SHOW_SERVING).
    3. Đợi bấm "Đã hài lòng" HOẶC hết 25 giây Timeout.
    4. Cảm ơn, tắt popup, mở khóa và chuyển sang điểm tuần tra tiếp theo.
    """
    def __init__(self, name="ActionServeCustomer", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None
        self.timeout_sec = 25.0

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            self.start_time = time.time()
            blackboard.set('ENCOUNTER_ACTIVE', False)
            blackboard.set('APPROACH_ACTIVE', False)
            blackboard.set('HAND_WAVE_ACTIVE', False)
            blackboard.set('SERVE_REQUESTED', False)
            if self.node and hasattr(self.node, 'stop_robot_motion'):
                self.node.stop_robot_motion(cancel_nav2=True)

            msg = "☕ BẮT ĐẦU PHỤC VỤ NƯỚC: Kính mời Thầy/Cô nhận nước trên khay!"
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("SHOW_SERVING")

            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        # Giữ /cmd_vel = 0 trong suốt thời gian phục vụ nước để xe đứng yên tuyệt đối.
        if self.node and hasattr(self.node, 'publish_zero_vel'):
            self.node.publish_zero_vel()

        finish_by_button = blackboard.get('FINISH_SERVING_REQUESTED', False)
        finish_by_timeout = (time.time() - self.start_time >= self.timeout_sec)

        if finish_by_button or finish_by_timeout:
            reason = "Thầy/Cô đã bấm hài lòng" if finish_by_button else "hết thời gian chờ 25s"
            msg = f"✅ HOÀN TẤT PHỤC VỤ ({reason})! Cảm ơn Thầy/Cô. Tiếp tục tuần tra."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            blackboard.set('IS_SERVING', False)
            blackboard.set('FINISH_SERVING_REQUESTED', False)
            blackboard.set('HAND_WAVE_ACTIVE', False)
            blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', time.time() + 10.0)
            blackboard.set('PATROL_ADVANCE_NEXT', True)

            self.start_time = None
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING

    def halt(self, blackboard: Blackboard = None) -> None:
        if blackboard is not None:
            blackboard.set('IS_SERVING', False)
            blackboard.set('FINISH_SERVING_REQUESTED', False)
        self.start_time = None
        self.status = NodeStatus.FAILURE


class ActionPatrol(Action):
    """
    Hành vi tuần tra chu kỳ qua các điểm [Trạm nước -> Điểm A -> Điểm B -> Điểm C -> Trạm]:
    - Lấy lộ trình từ /config/get_route (nếu có).
    - Hủy goal sạch sẽ qua halt() khi bị ngắt bởi sự kiện ưu tiên cao hơn hoặc Pause.
    - Dùng sequence token để bỏ qua callback muộn từ goal cũ đã hủy.
    """
    def __init__(self, name="ActionPatrol", node=None):
        super().__init__(name)
        self.node = node
        self.default_points = ["khoihanh", "diem1", "diem2", "diem3"]
        self.patrol_points = []
        self.current_idx = 0
        self.action_client = None
        self.get_route_client = None
        self.goal_handle = None
        self.is_waiting_for_result = False
        self.wait_start_time = 0.0
        self.is_waiting_at_waypoint = False
        self.is_fetching_route = False
        self._route_loaded = False
        self._goal_seq = 0
        self._route_seq = 0
        self._advance_after_wait = True

        if self.node:
            self.action_client = ActionClient(self.node, DirectGo, '/navigation/direct_go')
            self.get_route_client = self.node.create_client(GetRoute, '/config/get_route')

    def _schedule_wait(self, advance: bool):
        self.is_waiting_for_result = False
        self.wait_start_time = time.time()
        self._advance_after_wait = advance
        self.is_waiting_at_waypoint = True

    def _cancel_active_goal(self):
        self._goal_seq += 1
        if self.goal_handle is not None and self.is_waiting_for_result:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.goal_handle = None
        self.is_waiting_for_result = False

    def halt(self, blackboard: Blackboard = None) -> None:
        self._cancel_active_goal()
        self._route_seq += 1
        self.is_fetching_route = False
        self.is_waiting_at_waypoint = False
        self.status = NodeStatus.FAILURE

    def fetch_route(self):
        self._route_seq += 1
        seq = self._route_seq

        if not self.get_route_client or not self.get_route_client.service_is_ready():
            if self.node:
                self.node.get_logger().info("Chưa tìm thấy /config/get_route, dùng lộ trình mặc định: " + str(self.default_points))
            self.patrol_points = list(self.default_points)
            self._route_loaded = True
            self.current_idx = 0
            self.is_fetching_route = False
            self.send_next_goal()
            return

        self.is_fetching_route = True
        req = GetRoute.Request()
        req.route_name = "patrol_route"
        future = self.get_route_client.call_async(req)
        future.add_done_callback(lambda fut, s=seq: self.route_response_callback(fut, s))

    def route_response_callback(self, future, seq=None):
        if seq is not None and seq != self._route_seq:
            return
        try:
            response = future.result()
            if response.success and len(response.waypoints) > 0:
                self.patrol_points = list(response.waypoints)
            else:
                self.patrol_points = list(self.default_points)
        except Exception:
            self.patrol_points = list(self.default_points)

        msg = f"Đã sẵn sàng lộ trình tuần tra: {self.patrol_points}"
        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)
        self._route_loaded = True
        self.current_idx = 0
        self.is_fetching_route = False
        self.send_next_goal()

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if blackboard.get('PATROL_ROUTE_CHANGED', False):
            blackboard.set('PATROL_ROUTE_CHANGED', False)
            blackboard.set('PATROL_ADVANCE_NEXT', False)
            self._cancel_active_goal()
            self._route_loaded = False
            self.is_waiting_at_waypoint = False
            self.fetch_route()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if blackboard.get('PATROL_ADVANCE_NEXT', False):
            blackboard.set('PATROL_ADVANCE_NEXT', False)
            if self.patrol_points:
                self.current_idx = (self.current_idx + 1) % len(self.patrol_points)

        if self.status != NodeStatus.RUNNING:
            self.is_waiting_at_waypoint = False
            self.is_waiting_for_result = False
            self.status = NodeStatus.RUNNING
            if not self._route_loaded:
                self.fetch_route()
            else:
                self.send_next_goal()
            return NodeStatus.RUNNING

        if self.is_fetching_route or self.is_waiting_for_result:
            return NodeStatus.RUNNING

        if not self.patrol_points:
            self.patrol_points = list(self.default_points)

        if self.is_waiting_at_waypoint:
            if time.time() - self.wait_start_time > 2.0:
                self.is_waiting_at_waypoint = False
                if self._advance_after_wait:
                    self.current_idx = (self.current_idx + 1) % len(self.patrol_points)
                self.send_next_goal()
            return NodeStatus.RUNNING

        return NodeStatus.RUNNING

    def send_next_goal(self):
        if not self.action_client or not self.patrol_points:
            return
        if self.is_waiting_for_result or self.is_fetching_route:
            return
        if not self.action_client.server_is_ready():
            self._schedule_wait(advance=False)
            return

        self._goal_seq += 1
        seq = self._goal_seq
        target = self.patrol_points[self.current_idx]
        msg = f"🚀 [TUẦN TRA] Đang di chuyển đến điểm: {target}"
        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)

        goal_msg = DirectGo.Goal()
        goal_msg.target_location = target

        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda fut, s=seq: self.goal_response_callback(fut, s))
        self.is_waiting_for_result = True

    def goal_response_callback(self, future, seq=None):
        if seq is not None and seq != self._goal_seq:
            return
        try:
            goal_handle = future.result()
        except Exception:
            self._schedule_wait(advance=False)
            return

        if not goal_handle.accepted:
            if self.node:
                self.node.get_logger().warn(f"Goal tới {self.patrol_points[self.current_idx]} chưa được nhận, sẽ thử lại.")
            self._schedule_wait(advance=True)
            return

        self.goal_handle = goal_handle
        get_res_future = goal_handle.get_result_async()
        get_res_future.add_done_callback(lambda fut, s=seq: self.result_callback(fut, s))

    def result_callback(self, future, seq=None):
        if seq is not None and seq != self._goal_seq:
            return
        self.goal_handle = None
        target = self.patrol_points[self.current_idx]
        success = False
        try:
            res_wrapper = future.result()
            success = bool(res_wrapper.result.success) and (res_wrapper.status == GoalStatus.STATUS_SUCCEEDED)
        except Exception:
            success = False

        if success:
            msg = f"🏁 [TUẦN TRA] Đã đến điểm: {target}"
        else:
            msg = f"⚠️ [TUẦN TRA] Không thể đến được {target}, chuyển điểm tiếp theo."

        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)

        self._schedule_wait(advance=True)
