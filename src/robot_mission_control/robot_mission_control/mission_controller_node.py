import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_srvs.srv import Trigger
import time

from robot_interfaces.msg import RobotStatus
from robot_interfaces.action import Patrol, DirectGo
from robot_interfaces.srv import GetLocations

class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        
        # State
        self.current_state = "READY"
        self.water_level = 5.0
        self.water_threshold = 1.0 # Liters
        self.is_refilling = False
        
        # Subscriptions
        self.status_sub = self.create_subscription(RobotStatus, '/hmi/robot_status', self.status_cb, 10)
        
        # Service Clients
        self.client_get_locs = self.create_client(GetLocations, '/config/get_locations')
        
        # Actions
        self._action_server_patrol = ActionServer(
            self, Patrol, '/mission/start_patrol', self.execute_patrol_cb)
        self._action_server_direct_go = ActionServer(
            self, DirectGo, '/mission/direct_go', self.execute_direct_go_cb)
            
        # Services
        self.srv_return_home = self.create_service(Trigger, '/mission/return_home', self.cb_return_home)
        self.srv_pause = self.create_service(Trigger, '/mission/pause_patrol', self.cb_pause_patrol)
        self.srv_resume = self.create_service(Trigger, '/mission/resume_patrol', self.cb_resume_patrol)
        
        self.get_logger().info('Mission Controller Node started.')

    def status_cb(self, msg):
        self.water_level = msg.water_level
        # Auto-refill logic
        if self.current_state in ["READY", "PATROLLING"] and not self.is_refilling:
            if self.water_level < self.water_threshold:
                self.get_logger().warn(f"Water level low ({self.water_level:.2f}L). Interrupting task to auto-refill!")
                self.trigger_auto_refill()

    def trigger_auto_refill(self):
        self.is_refilling = True
        self.current_state = "AUTO_REFILL"
        self.get_logger().info("Navigating to Water Refill Point...")

    def execute_patrol_cb(self, goal_handle):
        self.get_logger().info(f"Starting Patrol: {goal_handle.request.route_name}")
        self.current_state = "PATROLLING"
        
        for i in range(1, 101):
            if self.current_state not in ["PATROLLING", "PAUSED"]:
                # Interrupted by something like Return Home or Auto Refill
                goal_handle.abort()
                return Patrol.Result(success=False, finish_message="Interrupted")
                
            while self.current_state == "PAUSED":
                time.sleep(0.5) # Wait while paused
                
            time.sleep(0.1)
            feedback_msg = Patrol.Feedback()
            feedback_msg.current_point = f"P{i%4 + 1}"
            feedback_msg.percent_complete = float(i)
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        self.current_state = "READY"
        return Patrol.Result(success=True, finish_message="Patrol complete")

    def execute_direct_go_cb(self, goal_handle):
        self.get_logger().info(f"Direct Go to: {goal_handle.request.target_location}")
        self.current_state = "DIRECT_GO"
        
        for i in range(10, 0, -1):
            if self.current_state != "DIRECT_GO":
                goal_handle.abort()
                return DirectGo.Result(success=False, finish_message="Interrupted")
                
            time.sleep(0.5)
            feedback_msg = DirectGo.Feedback()
            feedback_msg.distance_remaining = float(i)
            goal_handle.publish_feedback(feedback_msg)
            
        goal_handle.succeed()
        self.current_state = "READY"
        return DirectGo.Result(success=True, finish_message="Arrived at destination")

    def cb_return_home(self, request, response):
        self.current_state = "RETURNING_HOME"
        self.get_logger().info("Returning home...")
        response.success = True
        response.message = "Returning home"
        return response

    def cb_pause_patrol(self, request, response):
        if self.current_state == "PATROLLING":
            self.current_state = "PAUSED"
            response.success = True
            response.message = "Patrol paused"
        else:
            response.success = False
            response.message = "Not currently patrolling"
        return response

    def cb_resume_patrol(self, request, response):
        if self.current_state == "PAUSED":
            self.current_state = "PATROLLING"
            response.success = True
            response.message = "Patrol resumed"
        else:
            response.success = False
            response.message = "No patrol to resume"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
