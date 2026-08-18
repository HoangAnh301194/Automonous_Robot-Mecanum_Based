import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger

from robot_interfaces.msg import RobotStatus, CustomerStatus
from robot_interfaces.action import Patrol, DirectGo
from robot_interfaces.srv import GetLocation

from .bt_core import Blackboard, Fallback, Sequence, NodeStatus
from .bt_nodes import (
    IsEmergency, StopRobot, IsBatteryLow, ActionGoCharging, 
    IsWaterLow, ActionGoRefill, IsCustomerDetected, ActionServeCustomer,
    IsPatrolRequested, ActionPatrol
)

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        # Initialize Blackboard
        self.blackboard = Blackboard()
        
        # Initialize default Blackboard states
        self.blackboard.set('SYSTEM_lidar_ok', True)
        self.blackboard.set('SYSTEM_odom_ok', True)
        self.blackboard.set('SYSTEM_localization_ok', True)
        self.blackboard.set('SYSTEM_battery_low', False)
        self.blackboard.set('SYSTEM_water_low', False)
        
        self.blackboard.set('CUSTOMER_detected', False)
        self.blackboard.set('HMI_start_patrol', False)
        
        # Subscriptions
        self.sub_robot_status = self.create_subscription(
            RobotStatus, '/robot_status', self.cb_robot_status, 10)
        self.sub_customer = self.create_subscription(
            CustomerStatus, '/hmi/customer_status', self.cb_customer_status, 10)
            
        # Services & Actions for HMI Commands
        self.srv_start_patrol = self.create_service(
            Trigger, '/mission/start_patrol', self.cb_start_patrol)
        self.srv_pause_patrol = self.create_service(
            Trigger, '/mission/pause_patrol', self.cb_pause_patrol)
            
        # Build Behavior Tree
        self.tree = self.build_behavior_tree()
        
        # Timer for ticking the BT (10 Hz)
        self.timer = self.create_timer(0.1, self.tick_tree)
        
        self.get_logger().info('Mission Controller Node (BT based) started.')

    def cb_robot_status(self, msg: RobotStatus):
        # Update Blackboard from RobotStatus
        self.blackboard.set('SYSTEM_lidar_ok', msg.lidar_ok)
        self.blackboard.set('SYSTEM_odom_ok', msg.odom_ok)
        self.blackboard.set('SYSTEM_localization_ok', msg.localization_ok)
        self.blackboard.set('SYSTEM_battery_low', msg.battery_low)
        self.blackboard.set('SYSTEM_water_low', msg.water_low)

    def cb_customer_status(self, msg: CustomerStatus):
        self.blackboard.set('CUSTOMER_detected', msg.customer_detected)

    def cb_start_patrol(self, request, response):
        self.blackboard.set('HMI_start_patrol', True)
        response.success = True
        response.message = "Patrol requested"
        self.get_logger().info("HMI Command: Start Patrol")
        return response

    def cb_pause_patrol(self, request, response):
        self.blackboard.set('HMI_start_patrol', False)
        response.success = True
        response.message = "Patrol paused"
        self.get_logger().info("HMI Command: Pause Patrol")
        return response

    def build_behavior_tree(self):
        """
        Builds the Reactive Fallback Behavior Tree.
        Priority: Emergency -> Battery -> Water -> Customer -> Patrol
        """
        # 1. Emergency Branch
        seq_emergency = Sequence("Seq_Emergency", [
            IsEmergency("Cond_IsEmergency"),
            StopRobot("Act_StopRobot", self)
        ])
        
        # 2. Battery Low Branch
        seq_battery = Sequence("Seq_BatteryLow", [
            IsBatteryLow("Cond_IsBatteryLow"),
            ActionGoCharging("Act_GoCharging", self)
        ])
        
        # 3. Water Low Branch
        seq_water = Sequence("Seq_WaterLow", [
            IsWaterLow("Cond_IsWaterLow"),
            ActionGoRefill("Act_GoRefill", self)
        ])
        
        # 4. Customer Service Branch
        seq_customer = Sequence("Seq_CustomerService", [
            IsCustomerDetected("Cond_IsCustomerDetected"),
            ActionServeCustomer("Act_ServeCustomer", self)
        ])
        
        # 5. Patrol Branch
        seq_patrol = Sequence("Seq_Patrol", [
            IsPatrolRequested("Cond_IsPatrolRequested"),
            ActionPatrol("Act_Patrol", self)
        ])
        
        # Root Reactive Fallback
        root = Fallback("Priority_Manager", [
            seq_emergency,
            seq_battery,
            seq_water,
            seq_customer,
            seq_patrol
        ])
        
        return root

    def tick_tree(self):
        # Tick the Behavior Tree
        status = self.tree.tick(self.blackboard)
        # We can log the status if needed, but we keep it quiet to avoid log spam
        # print(f"BT Status: {status}")

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
