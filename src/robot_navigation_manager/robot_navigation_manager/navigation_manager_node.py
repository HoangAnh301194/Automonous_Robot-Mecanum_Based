import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robot_interfaces.action import DirectGo
from robot_interfaces.srv import GetLocation
from robot_interfaces.msg import NavigationStatus
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped


class NavigationManagerNode(Node):
    def __init__(self):
        super().__init__('navigation_manager_node')

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DirectGo,
            '/navigation/direct_go',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        self.get_loc_client = self.create_client(
            GetLocation,
            '/config/get_location',
            callback_group=self.cb_group
        )

        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.nav2_through_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
            callback_group=self.cb_group
        )

        self.status_pub = self.create_publisher(NavigationStatus, '/navigation/status', 10)
        self.timer = self.create_timer(1.0, self.publish_status, callback_group=self.cb_group)

        self._goal_lock = threading.Lock()
        self._exec_seq = 0
        self._active_direct_goal = None
        self.nav2_goal_handle = None

        self.current_goal_name = "None"
        self.status_string = "IDLE"
        self.distance_remaining = -1.0

        self.get_logger().info('Navigation Manager Node started.')

    def publish_status(self):
        msg = NavigationStatus()
        msg.status = self.status_string
        msg.current_goal_name = self.current_goal_name
        msg.distance_remaining = float(self.distance_remaining)
        self.status_pub.publish(msg)

    async def get_location_pose(self, location_name):
        if not self.get_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /config/get_location service...')
            return None

        req = GetLocation.Request()
        req.name = location_name

        future = self.get_loc_client.call_async(req)
        try:
            response = await future
            if response.success:
                return response
            self.get_logger().error(f"Location not found: {response.message}")
            return None
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request for DirectGo.')
        with self._goal_lock:
            is_current = (goal_handle is self._active_direct_goal)
            nav2_handle = self.nav2_goal_handle if is_current else None
        if nav2_handle is not None:
            self.get_logger().info('Canceling active Nav2 goal...')
            try:
                nav2_handle.cancel_goal_async()
            except Exception:
                pass
        if is_current:
            self.status_string = "CANCELING"
        return CancelResponse.ACCEPT

    def _finish_canceled(self, goal_handle, my_seq, result, msg="DirectGo cancelled"):
        goal_handle.canceled()
        with self._goal_lock:
            if my_seq == self._exec_seq:
                self.status_string = "CANCELLED"
                self.current_goal_name = "None"
                self.distance_remaining = -1.0
                self.nav2_goal_handle = None
                self._active_direct_goal = None
        result.success = False
        result.finish_message = msg
        return result

    async def execute_callback(self, goal_handle):
        target = goal_handle.request.target_location
        self.get_logger().info(f'Received DirectGo request to: {target}')

        loc_names = [s.strip() for s in target.split(',') if s.strip()]
        result = DirectGo.Result()

        if not loc_names:
            goal_handle.abort()
            result.success = False
            result.finish_message = "Target location is empty"
            return result

        with self._goal_lock:
            self._exec_seq += 1
            my_seq = self._exec_seq
            prev_nav2 = self.nav2_goal_handle
            self.nav2_goal_handle = None
            self._active_direct_goal = goal_handle
            self.current_goal_name = loc_names[0]
            self.status_string = "FETCHING_LOCATION"

        if prev_nav2 is not None:
            try:
                prev_nav2.cancel_goal_async()
            except Exception:
                pass

        if len(loc_names) == 1:
            return await self._execute_single_pose(goal_handle, loc_names[0], my_seq, result)
        else:
            return await self._execute_through_poses(goal_handle, loc_names, my_seq, result)

    async def _execute_single_pose(self, goal_handle, target, my_seq, result):
        loc_response = await self.get_location_pose(target)

        if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
            return self._finish_canceled(goal_handle, my_seq, result)

        if loc_response is None:
            with self._goal_lock:
                if my_seq == self._exec_seq:
                    self.status_string = "FAILED"
                    self.current_goal_name = "None"
                    self._active_direct_goal = None
            result.success = False
            result.finish_message = f"Failed to get location coordinates for {target}"
            goal_handle.abort()
            return result

        self.get_logger().info(f'Location found: x={loc_response.x}, y={loc_response.y}, yaw={loc_response.yaw}')

        with self._goal_lock:
            if my_seq == self._exec_seq:
                self.status_string = "WAITING_FOR_NAV2"

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result)
            with self._goal_lock:
                if my_seq == self._exec_seq:
                    self.status_string = "FAILED"
                    self.current_goal_name = "None"
                    self._active_direct_goal = None
            result.success = False
            result.finish_message = "Nav2 Action Server not available"
            goal_handle.abort()
            return result

        if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
            return self._finish_canceled(goal_handle, my_seq, result)

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose.header.frame_id = 'map'
        nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_goal.pose.pose.position.x = loc_response.x
        nav2_goal.pose.pose.position.y = loc_response.y
        nav2_goal.pose.pose.position.z = 0.0
        nav2_goal.pose.pose.orientation.z = math.sin(loc_response.yaw / 2.0)
        nav2_goal.pose.pose.orientation.w = math.cos(loc_response.yaw / 2.0)

        with self._goal_lock:
            if my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result, "DirectGo preempted")
            self.status_string = "MOVING"

        self.get_logger().info(f'Sending goal to Nav2 NavigateToPose: {target}')
        send_goal_future = self.nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda fb, s=my_seq, gh=goal_handle: self.nav2_feedback_callback(fb, s, gh)
        )
        local_nav2_handle = await send_goal_future

        if not local_nav2_handle.accepted:
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result)
            with self._goal_lock:
                if my_seq == self._exec_seq:
                    self.status_string = "FAILED"
                    self.current_goal_name = "None"
                    self._active_direct_goal = None
            result.success = False
            result.finish_message = "Nav2 rejected the goal"
            goal_handle.abort()
            return result

        with self._goal_lock:
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                preempted = True
            else:
                self.nav2_goal_handle = local_nav2_handle
                preempted = False

        if preempted:
            try:
                local_nav2_handle.cancel_goal_async()
            except Exception:
                pass
            return self._finish_canceled(goal_handle, my_seq, result)

        self.get_logger().info('Nav2 accepted the goal. Waiting for result...')
        get_result_future = local_nav2_handle.get_result_async()
        done_event = threading.Event()
        get_result_future.add_done_callback(lambda _fut: done_event.set())

        while not done_event.wait(timeout=0.05):
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                try:
                    await local_nav2_handle.cancel_goal_async()
                except Exception:
                    pass
                return self._finish_canceled(goal_handle, my_seq, result, "DirectGo cancelled by caller")

        nav2_result = get_result_future.result()

        if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
            return self._finish_canceled(goal_handle, my_seq, result, "DirectGo cancelled")

        if nav2_result.status == 4:
            final_status = "REACHED_GOAL"
            result.success = True
            result.finish_message = f"Successfully reached target location: {target}"
            goal_handle.succeed()
        elif nav2_result.status == 5:
            final_status = "CANCELLED"
            result.success = False
            result.finish_message = "Navigation was cancelled"
            goal_handle.canceled()
        else:
            final_status = "FAILED"
            result.success = False
            result.finish_message = "Nav2 failed to reach goal"
            goal_handle.abort()

        with self._goal_lock:
            if my_seq == self._exec_seq:
                self.status_string = final_status
                self.current_goal_name = "None"
                self.distance_remaining = -1.0
                self.nav2_goal_handle = None
                self._active_direct_goal = None

        return result

    async def _execute_through_poses(self, goal_handle, loc_names, my_seq, result):
        self.get_logger().info(f'Processing multi-waypoint route: {loc_names}')
        loc_resps = []
        for name in loc_names:
            resp = await self.get_location_pose(name)
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result)
            if resp is None:
                with self._goal_lock:
                    if my_seq == self._exec_seq:
                        self.status_string = "FAILED"
                        self.current_goal_name = "None"
                        self._active_direct_goal = None
                result.success = False
                result.finish_message = f"Failed to get location coordinates for {name}"
                goal_handle.abort()
                return result
            loc_resps.append(resp)

        with self._goal_lock:
            if my_seq == self._exec_seq:
                self.status_string = "WAITING_FOR_NAV2"

        if not self.nav2_through_client.wait_for_server(timeout_sec=5.0):
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result)
            with self._goal_lock:
                if my_seq == self._exec_seq:
                    self.status_string = "FAILED"
                    self.current_goal_name = "None"
                    self._active_direct_goal = None
            result.success = False
            result.finish_message = "Nav2 NavigateThroughPoses Action Server not available"
            goal_handle.abort()
            return result

        if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
            return self._finish_canceled(goal_handle, my_seq, result)

        poses = []
        n_points = len(loc_resps)
        for i in range(n_points):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x = loc_resps[i].x
            p.pose.position.y = loc_resps[i].y
            p.pose.position.z = 0.0

            if i < n_points - 1:
                # Intermediate waypoint: point tangent towards the next point
                # Robot travels through continuously without in-place rotation
                dx = loc_resps[i + 1].x - loc_resps[i].x
                dy = loc_resps[i + 1].y - loc_resps[i].y
                if math.hypot(dx, dy) > 0.01:
                    heading = math.atan2(dy, dx)
                else:
                    heading = loc_resps[i].yaw
                p.pose.orientation.z = math.sin(heading / 2.0)
                p.pose.orientation.w = math.cos(heading / 2.0)
            else:
                # Final destination: exact configured yaw orientation
                p.pose.orientation.z = math.sin(loc_resps[i].yaw / 2.0)
                p.pose.orientation.w = math.cos(loc_resps[i].yaw / 2.0)

            poses.append(p)

        nav2_goal = NavigateThroughPoses.Goal()
        nav2_goal.poses = poses

        with self._goal_lock:
            if my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result, "DirectGo preempted")
            self.status_string = "MOVING"
            self.current_goal_name = loc_names[0]

        self.get_logger().info(f'Sending {len(poses)} waypoints to Nav2 NavigateThroughPoses...')
        send_goal_future = self.nav2_through_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda fb, s=my_seq, gh=goal_handle, names=loc_names: self.nav2_through_feedback_callback(fb, s, gh, names)
        )
        local_nav2_handle = await send_goal_future

        if not local_nav2_handle.accepted:
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                return self._finish_canceled(goal_handle, my_seq, result)
            with self._goal_lock:
                if my_seq == self._exec_seq:
                    self.status_string = "FAILED"
                    self.current_goal_name = "None"
                    self._active_direct_goal = None
            result.success = False
            result.finish_message = "Nav2 rejected the NavigateThroughPoses goal"
            goal_handle.abort()
            return result

        with self._goal_lock:
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                preempted = True
            else:
                self.nav2_goal_handle = local_nav2_handle
                preempted = False

        if preempted:
            try:
                local_nav2_handle.cancel_goal_async()
            except Exception:
                pass
            return self._finish_canceled(goal_handle, my_seq, result)

        self.get_logger().info('Nav2 accepted NavigateThroughPoses goal. Waiting for result...')
        get_result_future = local_nav2_handle.get_result_async()
        done_event = threading.Event()
        get_result_future.add_done_callback(lambda _fut: done_event.set())

        while not done_event.wait(timeout=0.05):
            if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
                try:
                    await local_nav2_handle.cancel_goal_async()
                except Exception:
                    pass
                return self._finish_canceled(goal_handle, my_seq, result, "DirectGo cancelled by caller")

        nav2_result = get_result_future.result()

        if goal_handle.is_cancel_requested or my_seq != self._exec_seq:
            return self._finish_canceled(goal_handle, my_seq, result, "DirectGo cancelled")

        if nav2_result.status == 4:
            final_status = "REACHED_GOAL"
            result.success = True
            result.finish_message = f"Successfully reached all waypoints up to {loc_names[-1]}"
            goal_handle.succeed()
        elif nav2_result.status == 5:
            final_status = "CANCELLED"
            result.success = False
            result.finish_message = "NavigateThroughPoses was cancelled"
            goal_handle.canceled()
        else:
            final_status = "FAILED"
            result.success = False
            result.finish_message = "Nav2 failed to navigate through poses"
            goal_handle.abort()

        with self._goal_lock:
            if my_seq == self._exec_seq:
                self.status_string = final_status
                self.current_goal_name = "None"
                self.distance_remaining = -1.0
                self.nav2_goal_handle = None
                self._active_direct_goal = None

        return result

    def nav2_feedback_callback(self, feedback_msg, seq=None, goal_handle=None):
        if seq is not None and seq != self._exec_seq:
            return
        dist = feedback_msg.feedback.distance_remaining
        self.distance_remaining = dist
        if goal_handle is not None:
            action_fb = DirectGo.Feedback()
            action_fb.distance_remaining = float(dist)
            try:
                goal_handle.publish_feedback(action_fb)
            except Exception:
                pass

    def nav2_through_feedback_callback(self, feedback_msg, seq, goal_handle, loc_names):
        if seq != self._exec_seq:
            return
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        self.distance_remaining = dist

        num_remaining = fb.number_of_poses_remaining
        current_idx = len(loc_names) - num_remaining
        current_idx = max(0, min(current_idx, len(loc_names) - 1))
        active_name = loc_names[current_idx]

        with self._goal_lock:
            if self.current_goal_name != active_name and seq == self._exec_seq:
                self.current_goal_name = active_name
                self.publish_status()

        if goal_handle is not None:
            action_fb = DirectGo.Feedback()
            action_fb.distance_remaining = float(dist)
            try:
                goal_handle.publish_feedback(action_fb)
            except Exception:
                pass

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
