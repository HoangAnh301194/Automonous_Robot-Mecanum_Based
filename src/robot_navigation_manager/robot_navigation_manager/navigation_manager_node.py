import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robot_interfaces.action import DirectGo
from robot_interfaces.srv import GetLocation
from robot_interfaces.msg import NavigationStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

import math
import time

class NavigationManagerNode(Node):
    def __init__(self):
        super().__init__('navigation_manager_node')

        # We use ReentrantCallbackGroup to allow concurrent callbacks 
        # (e.g. calling service while in action server callback)
        self.cb_group = ReentrantCallbackGroup()

        # Action Server: DirectGo (from Mission Control or UI)
        self._action_server = ActionServer(
            self,
            DirectGo,
            '/navigation/direct_go',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # Service Client: Get Location (from Config Manager)
        self.get_loc_client = self.create_client(
            GetLocation, 
            '/config/get_location',
            callback_group=self.cb_group
        )

        # Action Client: NavigateToPose (to Nav2)
        self.nav2_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        # Publisher: NavigationStatus
        self.status_pub = self.create_publisher(NavigationStatus, '/navigation/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status, callback_group=self.cb_group)

        # State tracking
        self.current_goal_name = "None"
        self.status_string = "IDLE"
        self.distance_remaining = -1.0
        
        self.nav2_goal_handle = None

        self.get_logger().info('Navigation Manager Node started.')

    def publish_status(self):
        msg = NavigationStatus()
        msg.status = self.status_string
        msg.current_goal_name = self.current_goal_name
        msg.distance_remaining = float(self.distance_remaining)
        self.status_pub.publish(msg)

    async def get_location_pose(self, location_name):
        while not self.get_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /config/get_location service...')
            
        req = GetLocation.Request()
        req.name = location_name
        
        future = self.get_loc_client.call_async(req)
        try:
            # Need to await the future properly
            response = await future
            if response.success:
                return response
            else:
                self.get_logger().error(f"Location not found: {response.message}")
                return None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for DirectGo.')
        # If we have an active nav2 goal, cancel it
        if self.nav2_goal_handle is not None:
            self.get_logger().info('Canceling Nav2 NavigateToPose goal...')
            self.nav2_goal_handle.cancel_goal_async()
            
        self.status_string = "CANCELLED"
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        target = goal_handle.request.target_location
        self.get_logger().info(f'Received DirectGo request to: {target}')
        
        self.current_goal_name = target
        self.status_string = "FETCHING_LOCATION"
        
        # 1. Get coordinates from Config Manager
        loc_response = await self.get_location_pose(target)
        
        result = DirectGo.Result()
        
        if loc_response is None:
            self.status_string = "FAILED"
            self.current_goal_name = "None"
            result.success = False
            result.finish_message = "Failed to get location coordinates"
            goal_handle.abort()
            return result
            
        self.get_logger().info(f'Location found: x={loc_response.x}, y={loc_response.y}, yaw={loc_response.yaw}')
        
        # 2. Wait for Nav2 Action Server
        self.status_string = "WAITING_FOR_NAV2"
        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            self.status_string = "FAILED"
            result.success = False
            result.finish_message = "Nav2 Action Server not available"
            goal_handle.abort()
            return result

        # 3. Construct Nav2 Goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose.header.frame_id = 'map'
        nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_goal.pose.pose.position.x = loc_response.x
        nav2_goal.pose.pose.position.y = loc_response.y
        nav2_goal.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simple conversion for z-axis rotation)
        nav2_goal.pose.pose.orientation.z = math.sin(loc_response.yaw / 2.0)
        nav2_goal.pose.pose.orientation.w = math.cos(loc_response.yaw / 2.0)
        
        self.status_string = "MOVING"
        self.get_logger().info('Sending goal to Nav2 NavigateToPose...')
        
        # 4. Send Goal to Nav2
        send_goal_future = self.nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=self.nav2_feedback_callback
        )
        
        self.nav2_goal_handle = await send_goal_future
        
        if not self.nav2_goal_handle.accepted:
            self.status_string = "FAILED"
            result.success = False
            result.finish_message = "Nav2 rejected the goal"
            goal_handle.abort()
            return result

        self.get_logger().info('Nav2 accepted the goal. Waiting for result...')
        
        # 5. Wait for Result
        get_result_future = self.nav2_goal_handle.get_result_async()
        
        # Periodically check if our goal_handle was canceled while waiting
        while not get_result_future.done():
            if goal_handle.is_cancel_requested:
                # Cancel Nav2 goal and wait for its cancellation
                if self.nav2_goal_handle is not None:
                    await self.nav2_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                self.status_string = "CANCELLED"
                self.current_goal_name = "None"
                result.success = False
                result.finish_message = "DirectGo cancelled by user"
                return result
            time.sleep(0.1) # Sleep briefly to avoid busy loop
            
        nav2_result = get_result_future.result()
        
        # Check Nav2 final status
        # In rclpy, result status are in nav2_result.status, 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        if nav2_result.status == 4: # SUCCEEDED
            self.status_string = "REACHED_GOAL"
            result.success = True
            result.finish_message = "Successfully reached target location"
            goal_handle.succeed()
        elif nav2_result.status == 5: # CANCELED
            self.status_string = "CANCELLED"
            result.success = False
            result.finish_message = "Navigation was cancelled"
            goal_handle.canceled()
        else:
            self.status_string = "FAILED"
            result.success = False
            result.finish_message = "Nav2 failed to reach goal"
            goal_handle.abort()

        self.current_goal_name = "None"
        self.distance_remaining = -1.0
        self.nav2_goal_handle = None
        
        return result

    def nav2_feedback_callback(self, feedback_msg):
        # Update distance remaining based on Nav2 feedback
        self.distance_remaining = feedback_msg.feedback.distance_remaining
        # Also publish distance to DirectGo action feedback
        # Wait, DirectGo is an action server, we need to access its goal_handle?
        # Since execute_callback is managing the action, maybe we shouldn't publish DirectGo feedback here directly
        # but just update the state, or we need to pass goal_handle.
        # For simplicity, we just update the internal state, and let NavigationStatus broadcast it.

def main(args=None):
    rclpy.init(args=args)
    
    node = NavigationManagerNode()
    
    # Use MultiThreadedExecutor so callbacks can run concurrently
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
