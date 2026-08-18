import time
from .bt_core import NodeStatus, Condition, Action, Blackboard

# ==========================================
# CONDITIONS
# ==========================================

class IsEmergency(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        # Check if there is any critical sensor fault or emergency stop
        lidar_ok = blackboard.get('SYSTEM_lidar_ok', True)
        odom_ok = blackboard.get('SYSTEM_odom_ok', True)
        localization_ok = blackboard.get('SYSTEM_localization_ok', True)
        
        if not lidar_ok or not odom_ok or not localization_ok:
            return NodeStatus.SUCCESS # Yes, it is an emergency
        return NodeStatus.FAILURE

class IsBatteryLow(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        battery_low = blackboard.get('SYSTEM_battery_low', False)
        return NodeStatus.SUCCESS if battery_low else NodeStatus.FAILURE

class IsWaterLow(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        water_low = blackboard.get('SYSTEM_water_low', False)
        return NodeStatus.SUCCESS if water_low else NodeStatus.FAILURE

class IsCustomerDetected(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        customer_detected = blackboard.get('CUSTOMER_detected', False)
        return NodeStatus.SUCCESS if customer_detected else NodeStatus.FAILURE

class IsPatrolRequested(Condition):
    def tick(self, blackboard: Blackboard) -> NodeStatus:
        patrol_requested = blackboard.get('HMI_start_patrol', False)
        return NodeStatus.SUCCESS if patrol_requested else NodeStatus.FAILURE

# ==========================================
# ACTIONS
# ==========================================

class StopRobot(Action):
    def __init__(self, name="StopRobot", node=None):
        super().__init__(name)
        self.node = node # ROS 2 node reference for logging/publishing

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.node:
            self.node.get_logger().error("EMERGENCY STOP! Canceling all navigation.")
        # In a real system, publish to /cmd_vel with 0 to stop
        return NodeStatus.SUCCESS

class ActionGoCharging(Action):
    def __init__(self, name="ActionGoCharging", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            if self.node:
                self.node.get_logger().warn("Battery Low! Going to charging pile...")
            self.start_time = time.time()
            # Cancel current patrol and customer tasks
            blackboard.set('HMI_start_patrol', False)
            blackboard.set('CUSTOMER_detected', False)
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
        
        # Simulate navigation to charging pile (5 seconds)
        if time.time() - self.start_time > 5.0:
            if self.node:
                self.node.get_logger().info("Arrived at charging pile. Docking.")
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
            
        self.status = NodeStatus.RUNNING
        return NodeStatus.RUNNING

class ActionGoRefill(Action):
    def __init__(self, name="ActionGoRefill", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            if self.node:
                self.node.get_logger().warn("Water Low! Interrupting mission to refill...")
            self.start_time = time.time()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
        
        # Simulate navigation to water refill (5 seconds)
        if time.time() - self.start_time > 5.0:
            if self.node:
                self.node.get_logger().info("Refilled water successfully. Returning to mission.")
            # Clear the flag (in reality, sensor will update this)
            blackboard.set('SYSTEM_water_low', False)
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
            
        self.status = NodeStatus.RUNNING
        return NodeStatus.RUNNING

class ActionServeCustomer(Action):
    def __init__(self, name="ActionServeCustomer", node=None):
        super().__init__(name)
        self.node = node
        self.start_time = None

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        if self.status != NodeStatus.RUNNING:
            if self.node:
                self.node.get_logger().info("Customer detected! Moving to serve customer...")
            self.start_time = time.time()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
            
        # Simulate serving time (10 seconds)
        if time.time() - self.start_time > 10.0:
            if self.node:
                self.node.get_logger().info("Finished serving customer.")
            blackboard.set('CUSTOMER_detected', False) # Reset state
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
            
        self.status = NodeStatus.RUNNING
        return NodeStatus.RUNNING

from rclpy.action import ActionClient
from robot_interfaces.action import DirectGo

from robot_interfaces.srv import GetRoute
import time

class ActionPatrol(Action):
    def __init__(self, name="ActionPatrol", node=None):
        super().__init__(name)
        self.node = node
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
            return
            
        if not self.get_route_client.wait_for_service(timeout_sec=0.1):
            if self.node:
                self.node.get_logger().error("Không tìm thấy Service /config/get_route! Hãy kiểm tra config_manager_node.")
            self.status = NodeStatus.FAILURE
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
                if self.node:
                    self.node.get_logger().info(f"Đã tải Lộ trình tuần tra: {self.patrol_points}")
                self.current_idx = 0
                self.send_next_goal()
            else:
                if self.node:
                    self.node.get_logger().error("Lộ trình tuần tra trống hoặc không tìm thấy!")
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Lỗi lấy lộ trình: {e}")
        self.is_fetching_route = False

    def tick(self, blackboard: Blackboard) -> NodeStatus:
        # Nếu vừa chuyển từ trạng thái khác sang RUNNING
        if self.status != NodeStatus.RUNNING:
            if self.node:
                self.node.get_logger().info("Bắt đầu khởi tạo Tuần tra...")
            self.is_waiting_at_waypoint = False
            self.is_waiting_for_result = False
            self.fetch_route()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
            
        if self.is_fetching_route:
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
            
        if not self.patrol_points:
            self.status = NodeStatus.FAILURE
            return NodeStatus.FAILURE
            
        # Nếu đang đợi xe chạy tới điểm
        if self.is_waiting_for_result:
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
            
        # Nếu xe đã tới điểm, cho xe đợi 2 giây rồi mới đi điểm tiếp theo
        if self.is_waiting_at_waypoint:
            if time.time() - self.wait_start_time > 2.0:
                self.is_waiting_at_waypoint = False
                self.current_idx = (self.current_idx + 1) % len(self.patrol_points)
                self.send_next_goal()
            self.status = NodeStatus.RUNNING
            return NodeStatus.RUNNING
            
        self.status = NodeStatus.RUNNING
        return NodeStatus.RUNNING
        
    def send_next_goal(self):
        if not self.action_client or not self.patrol_points:
            return
            
        target = self.patrol_points[self.current_idx]
        if self.node:
            self.node.get_logger().info(f"Patrol: Gửi lệnh đến điểm '{target}'...")
            
        goal_msg = DirectGo.Goal()
        goal_msg.target_location = target
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_waiting_for_result = True
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            if self.node:
                self.node.get_logger().error(f"Patrol Goal tới {self.patrol_points[self.current_idx]} bị từ chối!")
            self.is_waiting_for_result = False
            self.wait_start_time = time.time()
            self.is_waiting_at_waypoint = True
            return
            
        self.goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        if self.node:
            self.node.get_logger().info(f"Patrol Goal hoàn tất. Thành công: {result.success}")
        self.is_waiting_for_result = False
        
        self.wait_start_time = time.time()
        self.is_waiting_at_waypoint = True
