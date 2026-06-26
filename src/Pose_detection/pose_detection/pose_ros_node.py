#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from yolo_msgs.msg import DetectionArray

# Import from existing project
from gesture.hand_raise_detector import is_hand_raised
from gesture.state_machine import WaveStateMachine

class PoseRosNode(Node):
    def __init__(self):
        super().__init__('pose_ros_node')
        
        # ROS Parameters
        self.declare_parameter('margin', 20)
        self.declare_parameter('conf_threshold', 0.4)
        self.margin = self.get_parameter('margin').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        
        # Subscribe to YOLO Detections
        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.publisher = self.create_publisher(String, '/pose/wave_status', 10)
        self.bool_pub = self.create_publisher(Bool, '/pose/wave_detected', 10)
        self.marker_pub = self.create_publisher(Marker, '/pose/wave_markers', 10)
        
        # State machines for tracking each person
        self.machines = {}
        
        self.get_logger().info('Pose ROS Node started. Subscribed to /yolo/detections')

    def detection_callback(self, msg):
        current_ids = set()
        
        for i, detection in enumerate(msg.detections):
            if detection.class_name != "person":
                continue
                
            # Use YOLO tracker ID if available, otherwise index
            person_id = detection.id if detection.id else f"person_{i}"
            current_ids.add(person_id)
            
            if person_id not in self.machines:
                self.machines[person_id] = WaveStateMachine()
                
            # Extract Keypoints
            kp_dict = {}
            for kp in detection.keypoints.data:
                # Map YOLOv8 COCO keypoints (1-indexed based on yolo_ros implementation)
                # 1 = Nose, 10 = Left Wrist, 11 = Right Wrist
                if kp.id == 1:
                    kp_dict["nose"] = (kp.point.x, kp.point.y, kp.score)
                elif kp.id == 10:
                    kp_dict["left_wrist"] = (kp.point.x, kp.point.y, kp.score)
                elif kp.id == 11:
                    kp_dict["right_wrist"] = (kp.point.x, kp.point.y, kp.score)
                    
            # Check logic
            left_r, right_r = is_hand_raised(
                kp_dict, 
                margin=self.margin, 
                conf_threshold=self.conf_threshold
            )
            
            # Update state machine
            self.machines[person_id].update(left_r, right_r)
            
            # 1. Publish Bool status (Any waving?)
            bool_msg = Bool()
            bool_msg.data = self.machines[person_id].is_waving
            self.bool_pub.publish(bool_msg)

            # 2. If waving, publish Text and Marker
            if self.machines[person_id].is_waving:
                hand = self.machines[person_id].active_hand
                event_str = f"[{person_id}] Waving with {hand} hand!"
                self.get_logger().info(event_str)
                
                # String message
                string_msg = String()
                string_msg.data = event_str
                self.publisher.publish(string_msg)

                # RViz Marker (Text above head)
                if "nose" in kp_dict:
                    marker = Marker()
                    marker.header.frame_id = "camera_color_optical_frame"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "wave_detection"
                    marker.id = i
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.action = Marker.ADD
                    # Approximate 3D position if we don't have depth info here
                    # For a real 3D marker, we'd need the depth, but we can use a dummy Z for now
                    marker.pose.position.x = 0.0 
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 1.0 
                    marker.scale.z = 0.2 # Text size
                    marker.color.a = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.text = f"WAVING! ({hand})"
                    marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
                    self.marker_pub.publish(marker)
                
        # Clean up lost tracks
        keys_to_remove = [k for k in self.machines.keys() if k not in current_ids]
        for k in keys_to_remove:
            del self.machines[k]

def main(args=None):
    rclpy.init(args=args)
    node = PoseRosNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
