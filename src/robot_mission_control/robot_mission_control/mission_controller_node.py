import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
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

        self.blackboard.set('HMI_start_patrol', True)
        self.blackboard.set('HAND_WAVE_ACTIVE', False)
        self.blackboard.set('PERSON_IN_PATH', False)
        self.blackboard.set('IS_SERVING', False)
        self.blackboard.set('SERVE_REQUESTED', False)
        self.blackboard.set('FINISH_SERVING_REQUESTED', False)
        self.blackboard.set('ENCOUNTER_COOLDOWN_UNTIL', 0.0)

        # Bộ đếm thời gian người đứng chắn trước mặt để chống nhiễu
        self._person_in_front_since = None

        # 2. TF2 Buffer để chuyển đổi tọa độ sang map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Action Client tới Nav2
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 4. Publishers
        self.pub_log = self.create_publisher(String, '/mission/status_log', 10)
        self.pub_popup = self.create_publisher(String, '/mission/service_popup_cmd', 10)

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

        # 6. Dựng cây Behavior Tree thông minh
        self.tree = self.build_behavior_tree()

        # Timer chạy cây BT ở chu kỳ 10 Hz
        self.timer = self.create_timer(0.1, self.tick_tree)

        self.get_logger().info('🚀 Mission Controller Node (Autonomous FSM / BT) đã khởi động thành công!')

    def cb_tracked_people(self, msg: TrackedPersonArray):
        """Xử lý dữ liệu AI đa người: phát hiện người vẫy tay hoặc người chắn đường."""
        has_waving = False
        waving_person = None
        has_blocking = False

        for p in msg.people:
            # Kiểm tra giơ tay vẫy
            if p.is_waving and p.wave_valid:
                has_waving = True
                waving_person = p
                break  # Ưu tiên người vẫy tay đầu tiên

            # Kiểm tra người đứng chắn chính diện mũi xe (< 1.8m, lệch ngang < 0.65m)
            x = p.position.x
            y = p.position.y
            dist = math.hypot(x, y)
            if 0.30 <= dist <= 1.80 and abs(y) <= 0.65:
                has_blocking = True

        # 1. Cập nhật trạng thái giơ tay
        if has_waving:
            self.blackboard.set('HAND_WAVE_ACTIVE', True)
            self.blackboard.set('WAVING_PERSON', waving_person)
        else:
            # Nếu không còn thấy vẫy nữa thì reset cờ sau 2s
            pass

        # 2. Cập nhật trạng thái người chắn đường (cần đứng yên > 0.8s để chống nhiễu)
        now = time.time()
        if has_blocking:
            if self._person_in_front_since is None:
                self._person_in_front_since = now
            elif (now - self._person_in_front_since) >= 0.8:
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
            self.blackboard.set('SERVE_REQUESTED', True)

    def cb_finish_serving(self, msg: Bool):
        if msg.data:
            self.blackboard.set('FINISH_SERVING_REQUESTED', True)

    def cb_srv_trigger_serve(self, request, response):
        self.blackboard.set('SERVE_REQUESTED', True)
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
        self.blackboard.set('CANCEL_PATROL_GOAL', True)
        response.success = True
        response.message = "Tạm dừng tuần tra"
        self.publish_log("Lệnh HMI: Tạm dừng tuần tra.")
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
