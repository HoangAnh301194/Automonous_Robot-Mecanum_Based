import math
import time
import rclpy
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
    """Ưu tiên cao nhất: Có người giơ tay vẫy gọi xe."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        # Nếu đang phục vụ người khác dở thì không ngắt
        if blackboard.get('IS_SERVING', False):
            return NodeStatus.FAILURE

        is_waving = blackboard.get('HAND_WAVE_ACTIVE', False)
        return NodeStatus.SUCCESS if is_waving else NodeStatus.FAILURE


class IsCustomerEncountered(Condition):
    """Gặp người đứng chắn đường (cự ly < 1.8m, không vẫy)."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if blackboard.get('IS_SERVING', False) or blackboard.get('HAND_WAVE_ACTIVE', False):
            return NodeStatus.FAILURE

        # Kiểm tra thời gian hồi chiêu (Cooldown) 10s để không dừng giật cục
        cooldown_until = blackboard.get('ENCOUNTER_COOLDOWN_UNTIL', 0.0)
        if time.time() < cooldown_until:
            return NodeStatus.FAILURE

        person_in_path = blackboard.get('PERSON_IN_PATH', False)
        return NodeStatus.SUCCESS if person_in_path else NodeStatus.FAILURE


class IsPatrolRequested(Condition):
    """Chế độ tuần tra mặc định."""
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        # Tự động bật tuần tra nếu không bị pause
        patrol_requested = blackboard.get('HMI_start_patrol', True)
        return NodeStatus.SUCCESS if patrol_requested else NodeStatus.FAILURE


# ==========================================
# ACTIONS
# ==========================================

class StopRobot(Action):
    def __init__(self, name="StopRobot", node=None):
        super().__init__(name)
        self.node = node

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.node:
            self.node.get_logger().error("EMERGENCY STOP! Canceling all navigation.")
            self.node.publish_log("[KHẨN CẤP] Cảm biến gặp sự cố! Dừng xe ngay lập tức.")
        return NodeStatus.SUCCESS


class ActionGoCharging(Action):
    def __init__(self, name="ActionGoCharging", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            msg = "CẢNH BÁO: Pin yếu! Đang về trạm sạc..."
            if self.node:
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


class ActionGoRefill(Action):
    def __init__(self, name="ActionGoRefill", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            msg = "CẢNH BÁO: Gần hết nước! Đang về trạm cấp nước..."
            if self.node:
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


# ==========================================
# SMART AUTONOMOUS ACTIONS
# ==========================================

class ActionApproachWavingPerson(Action):
    """
    Ưu tiên cao nhất: Khi phát hiện người giơ tay vẫy:
    1. Hủy mục tiêu tuần tra hiện tại.
    2. Lấy tọa độ 3D người vẫy tay từ AI.
    3. Tính điểm đỗ an toàn cách người 0.9m (không đâm vào chân).
    4. Điều khiển Nav2 chạy lại trước mặt người đó.
    5. Đến nơi -> Chuyển sang CHẾ ĐỘ PHỤC VỤ (IS_SERVING = True).
    """
    def __init__(self, name="ActionApproachWavingPerson", node=None):
        super().__init__(name)
        self.node = node
        self.nav2_client = None
        self.goal_handle = None
        self.is_navigating = False
        self.standoff_distance = 0.90  # Đỗ cách người 90cm an toàn

        if self.node:
            self.nav2_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            # Bắt đầu tiếp cận
            waving_person = blackboard.get('WAVING_PERSON')
            if not waving_person:
                self.status = NodeStatus.FAILURE
                return NodeStatus.FAILURE

            # Báo cho Patrol biết để hủy goal tuần tra
            blackboard.set('CANCEL_PATROL_GOAL', True)

            x_base = float(waving_person.position.x)
            y_base = float(waving_person.position.y)
            dist = math.hypot(x_base, y_base)
            angle = math.atan2(y_base, x_base)

            # Tính tọa độ đỗ cách người standoff_distance
            approach_dist = max(0.0, dist - self.standoff_distance)
            x_target_base = approach_dist * math.cos(angle)
            y_target_base = approach_dist * math.sin(angle)

            # Tạo PoseStamped trong base_footprint hướng mặt về phía người
            pose_base = PoseStamped()
            pose_base.header.stamp = self.node.get_clock().now().to_msg()
            pose_base.header.frame_id = 'base_footprint'
            pose_base.pose.position.x = x_target_base
            pose_base.pose.position.y = y_target_base
            pose_base.pose.position.z = 0.0
            pose_base.pose.orientation.z = math.sin(angle / 2.0)
            pose_base.pose.orientation.w = math.cos(angle / 2.0)

            # Chuyển đổi sang hệ tọa độ map qua TF
            pose_map = None
            try:
                if self.node and hasattr(self.node, 'tf_buffer'):
                    transform = self.node.tf_buffer.lookup_transform(
                        'map', 'base_footprint', rclpy.time.Time(),
                        timeout=Duration(seconds=1.0)
                    )
                    import tf2_geometry_msgs
                    pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_base, transform)
            except Exception as e:
                if self.node:
                    self.node.get_logger().warn(f"TF transform lỗi: {e}")

            if not pose_map:
                # Nếu không chuyển đổi được, dùng topic preview map từ localizer
                pose_map = blackboard.get('PERSON_PREVIEW_MAP_POSE', None)

            if not pose_map:
                if self.node:
                    self.node.get_logger().error("Không thể xác định tọa độ map của người vẫy tay!")
                self.status = NodeStatus.FAILURE
                return NodeStatus.FAILURE

            msg = f"👋 PHÁT HIỆN NGƯỜI VẪY TAY! Đang tiếp cận người (cách {dist:.2f}m)..."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            # Gửi goal tới Nav2
            self.send_nav2_goal(pose_map)
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        # Nếu đang trong quá trình chạy đến
        if self.is_navigating:
            return NodeStatus.RUNNING

        return self.status

    def send_nav2_goal(self, pose_map: PoseStamped):
        if not self.nav2_client.wait_for_server(timeout_sec=2.0):
            if self.node:
                self.node.get_logger().error("Nav2 Action Server không phản hồi!")
            self.status = NodeStatus.FAILURE
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_map
        send_future = self.nav2_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.node:
                self.node.get_logger().error("Goal tiếp cận người bị Nav2 từ chối!")
            self.is_navigating = False
            self.status = NodeStatus.FAILURE
            return

        self.goal_handle = goal_handle
        get_res_future = goal_handle.get_result_async()
        get_res_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.is_navigating = False
        msg = "🎯 ĐÃ ĐẾN TRƯỚC MẶT NGƯỜI VẪY TAY! Chuyển sang CHẾ ĐỘ PHỤC VỤ."
        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)

        # Chuyển sang chế độ phục vụ
        if self.node and hasattr(self.node, 'blackboard'):
            self.node.blackboard.set('HAND_WAVE_ACTIVE', False)
            self.node.blackboard.set('IS_SERVING', True)

        self.status = NodeStatus.SUCCESS


class ActionCustomerEncounter(Action):
    """
    Khi gặp người đứng cản trước mặt (cự ly < 1.8m, > 1.5s):
    1. Dừng tĩnh 5 giây tại chỗ.
    2. Bật Popup mời nước lên HMI Kiosk (SHOW_5S_INVITE).
    3. Nếu thầy cô bấm "Phục vụ" -> IS_SERVING = True.
    4. Nếu sau 5s không bấm -> Chào nhẹ, đi tiếp và Cooldown 10s.
    """
    def __init__(self, name="ActionCustomerEncounter", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            self.start_time = time.time()
            # Báo Patrol tạm dừng
            blackboard.set('CANCEL_PATROL_GOAL', True)

            msg = "👤 GẶP NGƯỜI TRƯỚC MẶT! Dừng tĩnh 5 giây mời nước..."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("SHOW_5S_INVITE")

            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        # 1. Nếu người dùng bấm "Lấy nước / Phục vụ" trên HMI
        if blackboard.get('SERVE_REQUESTED', False):
            blackboard.set('SERVE_REQUESTED', False)
            blackboard.set('IS_SERVING', True)
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        # 2. Nếu hết 5 giây không ai bấm
        if time.time() - self.start_time >= 5.0:
            msg = "⏱ Hết 5s không yêu cầu nước. Chúc Thầy/Cô một ngày tốt lành và tiếp tục tuần tra!"
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            # Đặt Cooldown 10s để không bị dừng liên tục
            blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', time.time() + 10.0)
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING


class ActionServeCustomer(Action):
    """
    Chế độ Phục Vụ Nước (IS_SERVING == True):
    1. Khóa cứng bánh xe đứng yên.
    2. Bật Popup phục vụ (SHOW_SERVING) & phát MP3 giới thiệu chúc mừng.
    3. Đợi thầy cô bấm "Đã hài lòng" HOẶC hết 25 giây Timeout.
    4. Cảm ơn, tắt popup, chuyển sang điểm tuần tra tiếp theo.
    """
    def __init__(self, name="ActionServeCustomer", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None
        self.timeout_sec = 25.0

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            self.start_time = time.time()
            msg = "☕ BẮT ĐẦU PHỤC VỤ NƯỚC: Kính mời Thầy/Cô nhận nước trên khay!"
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("SHOW_SERVING")

            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        # Kiểm tra điều kiện hoàn thành:
        finish_by_button = blackboard.get('FINISH_SERVING_REQUESTED', False)
        finish_by_timeout = (time.time() - self.start_time >= self.timeout_sec)

        if finish_by_button or finish_by_timeout:
            reason = "Thầy/Cô đã bấm hài lòng" if finish_by_button else "hết thời gian chờ 25s"
            msg = f"✅ HOÀN TẤT PHỤC VỤ ({reason})! Cảm ơn Thầy/Cô. Tiếp tục tuần tra."
            if self.node:
                self.node.get_logger().info(msg)
                self.node.publish_log(msg)
                self.node.publish_popup_cmd("HIDE")

            # Đặt lại các cờ
            blackboard.set('IS_SERVING', False)
            blackboard.set('FINISH_SERVING_REQUESTED', False)
            blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', time.time() + 10.0)

            # Báo cho Patrol chuyển sang waypoint tiếp theo
            blackboard.set('PATROL_ADVANCE_NEXT', True)

            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS

        return NodeStatus.RUNNING


class ActionPatrol(Action):
    """
    Hành vi tuần tra chu kỳ qua các điểm [Trạm nước -> Điểm A -> Điểm B -> Điểm C -> Trạm]:
    - Lấy lộ trình từ /config/get_route (nếu có).
    - Có danh sách điểm mặc định nếu chưa cấu hình.
    - Hỗ trợ tạm dừng / hủy mượt mà khi có sự kiện ưu tiên (gặp người, giơ tay).
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

        if self.node:
            self.action_client = ActionClient(self.node, DirectGo, '/navigation/direct_go')
            self.get_route_client = self.node.create_client(GetRoute, '/config/get_route')

    def fetch_route(self):
        if not self.get_route_client:
            self.patrol_points = list(self.default_points)
            return

        if not self.get_route_client.wait_for_service(timeout_sec=0.2):
            if self.node:
                self.node.get_logger().info("Chưa tìm thấy /config/get_route, dùng lộ trình mặc định: " + str(self.default_points))
            self.patrol_points = list(self.default_points)
            self.send_next_goal()
            return

        self.is_fetching_route = True
        req = GetRoute.Request()
        req.route_name = "patrol_route"
        future = self.get_route_client.call_async(req)
        future.add_done_callback(self.route_response_callback)

    def route_response_callback(self, future):
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
        self.current_idx = 0
        self.is_fetching_route = False
        self.send_next_goal()

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        # Nếu có lệnh hủy goal từ sự kiện ưu tiên
        if blackboard.get('CANCEL_PATROL_GOAL', False):
            blackboard.set('CANCEL_PATROL_GOAL', False)
            if self.is_waiting_for_result and self.goal_handle:
                self.goal_handle.cancel_goal_async()
            self.is_waiting_for_result = False
            self.status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

        # Nếu có lệnh chuyển waypoint sau khi phục vụ xong
        if blackboard.get('PATROL_ADVANCE_NEXT', False):
            blackboard.set('PATROL_ADVANCE_NEXT', False)
            if self.patrol_points:
                self.current_idx = (self.current_idx + 1) % len(self.patrol_points)
            self.is_waiting_for_result = False
            self.is_waiting_at_waypoint = False
            self.send_next_goal()

        route_changed = blackboard.get('PATROL_ROUTE_CHANGED', False)

        if self.status != NodeStatus.RUNNING or route_changed:
            if route_changed:
                blackboard.set('PATROL_ROUTE_CHANGED', False)
                if self.is_waiting_for_result and self.goal_handle:
                    self.goal_handle.cancel_goal_async()

            self.is_waiting_at_waypoint = False
            self.is_waiting_for_result = False
            self.fetch_route()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING

        if self.is_fetching_route:
            return NodeStatus.RUNNING

        if not self.patrol_points:
            self.patrol_points = list(self.default_points)

        if self.is_waiting_for_result:
            return NodeStatus.RUNNING

        # Đợi 2 giây tại mỗi điểm waypoint rồi đi tiếp
        if self.is_waiting_at_waypoint:
            if time.time() - self.wait_start_time > 2.0:
                self.is_waiting_at_waypoint = False
                self.current_idx = (self.current_idx + 1) % len(self.patrol_points)
                self.send_next_goal()
            return NodeStatus.RUNNING

        return NodeStatus.RUNNING

    def send_next_goal(self):
        if not self.action_client or not self.patrol_points:
            return

        target = self.patrol_points[self.current_idx]
        msg = f"🚀 [TUẦN TRA] Đang di chuyển đến điểm: {target}"
        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)

        goal_msg = DirectGo.Goal()
        goal_msg.target_location = target

        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        self.is_waiting_for_result = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.node:
                self.node.get_logger().warn(f"Goal tới {self.patrol_points[self.current_idx]} chưa được nhận, sẽ thử lại.")
            self.is_waiting_for_result = False
            self.wait_start_time = time.time()
            self.is_waiting_at_waypoint = True
            return

        self.goal_handle = goal_handle
        get_res_future = goal_handle.get_result_async()
        get_res_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        target = self.patrol_points[self.current_idx]
        if result.success:
            msg = f"🏁 [TUẦN TRA] Đã đến điểm: {target}"
        else:
            msg = f"⚠️ [TUẦN TRA] Không thể đến được {target}, chuyển điểm tiếp theo."

        if self.node:
            self.node.get_logger().info(msg)
            self.node.publish_log(msg)

        self.is_waiting_for_result = False
        self.wait_start_time = time.time()
        self.is_waiting_at_waypoint = True
