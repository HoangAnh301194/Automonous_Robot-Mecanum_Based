import math
import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

from robot_interfaces.msg import RobotStatus, CustomerStatus, TrackedPersonArray
from robot_interfaces.action import DirectGo

from .bt_core import Blackboard, Fallback, Sequence, NodeStatus
from .bt_nodes import (
    IsEmergency, StopRobot,
    IsBatteryLow, ActionGoCharging,
    IsWaterLow, ActionGoRefill,
    IsServingCustomer, ActionServeCustomer,
    IsHandWavingDetected, ActionApproachWavingPerson,
    IsCustomerEncountered, ActionCustomerEncounter,
    IsPatrolRequested, ActionPatrol
)


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')

        # 1. Blackboard chia sẻ dữ liệu giữa các node BT
        self.blackboard = Blackboard()
        self.blackboard.set('SYSTEM_lidar_ok', True)
        self.blackboard.set('SYSTEM_odom_ok', True)
        self.blackboard.set('SYSTEM_localization_ok', True)
        self.blackboard.set('SYSTEM_battery_low', False)
        self.blackboard.set('SYSTEM_water_low', False)

        self.blackboard.set('HMI_start_patrol', False)
        self.blackboard.set('HAND_WAVE_ACTIVE', False)
        self.blackboard.set('APPROACH_ACTIVE', False)
        self.blackboard.set('WAVING_PERSON', None)
        self.blackboard.set('TARGET_TRACK_ID', None)
        self.blackboard.set('PERSON_IN_PATH', False)
        self.blackboard.set('ENCOUNTER_ACTIVE', False)
        self.blackboard.set('IS_SERVING', False)
        self.blackboard.set('SERVE_REQUESTED', False)
        self.blackboard.set('FINISH_SERVING_REQUESTED', False)
        self.blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', 0.0)
        self.blackboard.set('WAVE_COOLDOWN_UNTIL', 0.0)

        self._person_in_front_since = None
        self._last_wave_seen_at = 0.0
        self._wave_timeout_sec = 2.0

        # 2. TF2 Buffer để chuyển đổi tọa độ sang map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Action Client & Cancel Client tới Nav2 + khóa /cmd_vel
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cancel_nav2_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        # 4. Publishers
        self.pub_log = self.create_publisher(String, '/mission/status_log', 10)
        self.pub_popup = self.create_publisher(String, '/mission/service_popup_cmd', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 5. Subscriptions từ tầng AI mới
        self.sub_tracked_people = self.create_subscription(
            TrackedPersonArray,
            '/perception/tracked_people',
            self.cb_tracked_people,
            10
        )
        self.sub_preview_map = self.create_subscription(
            PointStamped,
            '/mission/person_preview_map',
            self.cb_person_preview_map,
            10
        )

        # Subscriptions hệ thống & HMI
        self.sub_robot_status = self.create_subscription(
            RobotStatus, '/robot_status', self.cb_robot_status, 10
        )
        self.sub_customer = self.create_subscription(
            CustomerStatus, '/hmi/customer_status', self.cb_customer_status, 10
        )

        # Topic điều khiển phục vụ (cho GUI Kiosk hoặc test từ CLI)
        self.sub_serve_trigger = self.create_subscription(
            Bool, '/mission/serve_trigger', self.cb_serve_trigger, 10
        )
        self.sub_finish_serving = self.create_subscription(
            Bool, '/mission/finish_serving', self.cb_finish_serving, 10
        )

        # Services cho HMI Command
        self.srv_start_patrol = self.create_service(
            Trigger, '/mission/start_patrol', self.cb_start_patrol
        )
        self.srv_pause_patrol = self.create_service(
            Trigger, '/mission/pause_patrol', self.cb_pause_patrol
        )
        self.srv_trigger_serve = self.create_service(
            Trigger, '/mission/trigger_serve', self.cb_srv_trigger_serve
        )
        self.srv_trigger_finish = self.create_service(
            Trigger, '/mission/trigger_finish', self.cb_srv_trigger_finish
        )
        self.srv_reload_route = self.create_service(
            Trigger, '/mission/reload_route', self.cb_reload_route
        )

        # 6. Dựng cây Behavior Tree thông minh
        self.tree = self.build_behavior_tree()

        # Timer chạy cây BT ở chu kỳ 10 Hz
        self.timer = self.create_timer(0.1, self.tick_tree)

        self.get_logger().info('🚀 Mission Controller Node (Autonomous FSM / BT) đã khởi động thành công!')

    def publish_zero_vel(self):
        """Publish zero velocity to keep the robot stationary during hold/serving states."""
        self.pub_cmd_vel.publish(Twist())

    def stop_robot_motion(self, cancel_nav2: bool = True):
        """Immediately zero /cmd_vel and cancel any active Nav2 NavigateToPose goal."""
        self.publish_zero_vel()
        if cancel_nav2 and self.cancel_nav2_client.service_is_ready():
            try:
                self.cancel_nav2_client.call_async(CancelGoal.Request())
            except Exception as exc:
                self.get_logger().warn(f"Lỗi gọi cancel Nav2: {exc}")

    def _activate_serving(self):
        self.blackboard.set('SERVE_REQUESTED', False)
        self.blackboard.set('ENCOUNTER_ACTIVE', False)
        self.blackboard.set('APPROACH_ACTIVE', False)
        self.blackboard.set('HAND_WAVE_ACTIVE', False)
        self.blackboard.set('TARGET_TRACK_ID', None)
        self.blackboard.set('IS_SERVING', True)

    def cb_tracked_people(self, msg: TrackedPersonArray):
        """Xử lý dữ liệu AI đa người: phát hiện người vẫy tay hoặc người chắn đường."""
        has_waving = False
        waving_person = None
        has_blocking = False
        has_preview = self.blackboard.get('PERSON_PREVIEW_MAP_POSE', None) is not None

        is_approaching = self.blackboard.get('APPROACH_ACTIVE', False)
        target_track_id = self.blackboard.get('TARGET_TRACK_ID', None)
        target_person_found = False

        for p in msg.people:
            pos_valid = (
                bool(getattr(p, 'position_valid', True))
                and math.isfinite(p.position.x)
                and math.isfinite(p.position.y)
            )

            # 1. Khi đang tiếp cận: ưu tiên bám theo target_track_id (kể cả khi người đã hạ tay)
            if is_approaching and target_track_id is not None and str(getattr(p, 'track_id', '')) == str(target_track_id) and pos_valid:
                target_person_found = True
                waving_person = p
                has_waving = True

            # 2. Khi chưa tiếp cận: phát hiện người vẫy tay mới
            if not is_approaching and p.is_waving and p.wave_valid and (pos_valid or has_preview):
                if waving_person is None or (pos_valid and not getattr(waving_person, 'position_valid', False)):
                    has_waving = True
                    waving_person = p

            if pos_valid:
                dist = math.hypot(p.position.x, p.position.y)
                # Chỉ coi là chắn lối mời nước khi người đứng trực diện trước đầu xe:
                # Cự ly 0.4m - 1.2m, lệch ngang trong luồng thân xe <= 0.35m
                if 0.40 <= dist <= 1.20 and abs(p.position.y) <= 0.35:
                    has_blocking = True

        now_mono = time.monotonic()
        if is_approaching:
            if target_person_found:
                self._last_wave_seen_at = now_mono
                self.blackboard.set('HAND_WAVE_ACTIVE', True)
                self.blackboard.set('WAVING_PERSON', waving_person)
            # Nếu người tạm thời khuất tầm nhìn, giữ nguyên WAVING_PERSON và HAND_WAVE_ACTIVE cũ
            # để xe không bị hủy đột ngột mà tiếp tục chạy theo vị trí đã biết
        else:
            if has_waving:
                self._last_wave_seen_at = now_mono
                self.blackboard.set('HAND_WAVE_ACTIVE', True)
                self.blackboard.set('WAVING_PERSON', waving_person)
                if waving_person is not None and hasattr(waving_person, 'track_id'):
                    self.blackboard.set('TARGET_TRACK_ID', str(waving_person.track_id))
            elif (
                self.blackboard.get('HAND_WAVE_ACTIVE', False)
                and (now_mono - self._last_wave_seen_at) >= self._wave_timeout_sec
            ):
                self.blackboard.set('HAND_WAVE_ACTIVE', False)
                self.blackboard.set('WAVING_PERSON', None)
                self.blackboard.set('TARGET_TRACK_ID', None)

        now = time.time()
        if has_blocking:
            if self._person_in_front_since is None:
                self._person_in_front_since = now
            elif (now - self._person_in_front_since) >= 1.5:
                self.blackboard.set('PERSON_IN_PATH', True)
        else:
            self._person_in_front_since = None
            self.blackboard.set('PERSON_IN_PATH', False)

    def cb_person_preview_map(self, msg: PointStamped):
        """Lưu tọa độ map dự phòng của người được chọn."""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.blackboard.set('PERSON_PREVIEW_MAP_POSE', pose)

    def cb_robot_status(self, msg: RobotStatus):
        self.blackboard.set('SYSTEM_lidar_ok', msg.lidar_ok)
        self.blackboard.set('SYSTEM_odom_ok', msg.odom_ok)
        self.blackboard.set('SYSTEM_localization_ok', msg.localization_ok)
        self.blackboard.set('SYSTEM_battery_low', msg.battery_low)
        self.blackboard.set('SYSTEM_water_low', msg.water_low)

    def cb_customer_status(self, msg: CustomerStatus):
        if msg.customer_detected:
            self.blackboard.set('PERSON_IN_PATH', True)

    def cb_serve_trigger(self, msg: Bool):
        if msg.data:
            self._activate_serving()

    def cb_finish_serving(self, msg: Bool):
        if msg.data:
            self.blackboard.set('FINISH_SERVING_REQUESTED', True)

    def cb_srv_trigger_serve(self, request, response):
        self._activate_serving()
        response.success = True
        response.message = "Đã kích hoạt yêu cầu phục vụ nước."
        return response

    def cb_srv_trigger_finish(self, request, response):
        self.blackboard.set('FINISH_SERVING_REQUESTED', True)
        response.success = True
        response.message = "Đã gửi xác nhận hoàn thành phục vụ."
        return response

    def cb_start_patrol(self, request, response):
        self.blackboard.set('HMI_start_patrol', True)
        self.blackboard.set('PATROL_ROUTE_CHANGED', True)
        response.success = True
        response.message = "Bắt đầu tuần tra"
        self.publish_log("Lệnh HMI: Bắt đầu tuần tra.")
        return response

    def cb_pause_patrol(self, request, response):
        self.blackboard.set('HMI_start_patrol', False)
        self.stop_robot_motion(cancel_nav2=True)
        response.success = True
        response.message = "Tạm dừng tuần tra"
        self.publish_log("Lệnh HMI: Tạm dừng tuần tra.")
        return response

    def cb_reload_route(self, request, response):
        self.blackboard.set('PATROL_ROUTE_CHANGED', True)
        response.success = True
        response.message = "Đã nạp lại lộ trình tuần tra mới."
        self.publish_log("🔄 Đã nạp lại lộ trình tuần tra mới từ cấu hình.")
        return response

    def publish_log(self, text: str):
        msg = String()
        msg.data = text
        self.pub_log.publish(msg)

    def publish_popup_cmd(self, cmd: str):
        """Bắn lệnh mở/tắt Popup sang HMI Kiosk (SHOW_5S_INVITE, SHOW_SERVING, HIDE)."""
        msg = String()
        msg.data = cmd
        self.pub_popup.publish(msg)

    def build_behavior_tree(self):
        """
        Xây dựng Cây Behavior Tree Phân Cấp Ưu Tiên:
        P0: Sự cố khẩn cấp (Emergency)
        P1: Pin yếu (Battery Low)
        P2: Đang phục vụ khách (Serving Customer - Khóa cứng xe)
        P3: Có người vẫy tay gọi (Waving Person - Ưu tiên cao nhất khi chạy)
        P4: Gặp người chắn đường (Customer Encounter - Dừng 5s)
        P5: Cạn nước (Water Low)
        P6: Tuần tra mặc định (Patrol Trạm -> A -> B -> C -> Trạm)
        """
        # P0: Khẩn cấp
        seq_emergency = Sequence("Seq_Emergency", [
            IsEmergency("Cond_IsEmergency"),
            StopRobot("Act_StopRobot", self)
        ])

        # P1: Pin yếu
        seq_battery = Sequence("Seq_BatteryLow", [
            IsBatteryLow("Cond_IsBatteryLow"),
            ActionGoCharging("Act_GoCharging", self)
        ])

        # P2: Đang trong trạng thái phục vụ (khóa xe không cho di chuyển)
        seq_serving = Sequence("Seq_ServingCustomer", [
            IsServingCustomer("Cond_IsServing"),
            ActionServeCustomer("Act_ServeCustomer", self)
        ])

        # P3: Người giơ tay vẫy gọi xe (Ưu tiên tiếp cận)
        seq_wave = Sequence("Seq_WavingPerson", [
            IsHandWavingDetected("Cond_IsHandWaving"),
            ActionApproachWavingPerson("Act_ApproachWavingPerson", self)
        ])

        # P4: Gặp người cản đường (Dừng tĩnh 5s)
        seq_encounter = Sequence("Seq_Encounter", [
            IsCustomerEncountered("Cond_IsEncountered"),
            ActionCustomerEncounter("Act_CustomerEncounter", self)
        ])

        # P5: Hết nước
        seq_water = Sequence("Seq_WaterLow", [
            IsWaterLow("Cond_IsWaterLow"),
            ActionGoRefill("Act_GoRefill", self)
        ])

        # P6: Tuần tra tự động
        seq_patrol = Sequence("Seq_Patrol", [
            IsPatrolRequested("Cond_IsPatrolRequested"),
            ActionPatrol("Act_Patrol", self)
        ])

        # Cây Fallback ưu tiên tổng thể
        root = Fallback("Autonomous_Mission_Manager", [
            seq_emergency,
            seq_battery,
            seq_serving,
            seq_wave,
            seq_encounter,
            seq_water,
            seq_patrol
        ])

        return root

    def tick_tree(self):
        if (
            self.blackboard.get('HAND_WAVE_ACTIVE', False)
            and not self.blackboard.get('APPROACH_ACTIVE', False)
            and (time.monotonic() - self._last_wave_seen_at) >= self._wave_timeout_sec
        ):
            self.blackboard.set('HAND_WAVE_ACTIVE', False)
            self.blackboard.set('WAVING_PERSON', None)
            self.blackboard.set('TARGET_TRACK_ID', None)
        self.tree.tick(self.blackboard)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
