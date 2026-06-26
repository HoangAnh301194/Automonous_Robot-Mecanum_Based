# Copyright (C) 2026 Gemini CLI

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from yolo_msgs.msg import DetectionArray, Detection, PersonTracking
import math

class PersonTrackerNode(Node):
    """
    ROS 2 Node that extracts person tracking information (image coordinates and distance).
    Publishes simplified data to be consumed by Behavior Trees or other control nodes.
    """

    def __init__(self):
        super().__init__('person_tracker_node')

        # Parameters
        self.declare_parameter('target_id', '') # Empty means track any person
        
        # State
        self.current_target_id = self.get_parameter('target_id').value

        # Pubs & Subs
        self.tracking_pub = self.create_publisher(PersonTracking, 'person_tracking', 10)
        
        self.sub_3d = self.create_subscription(
            DetectionArray,
            'yolo/detections_3d', # Listening to the 3D detections from the yolo pipeline
            self.detections_cb,
            10
        )

        self.get_logger().info('Person Tracker Node Initialized')

    def detections_cb(self, msg: DetectionArray):
        best_person = None
        min_dist = float('inf')

        for det in msg.detections:
            # Check if it's a person (class_id 0 in COCO)
            if det.class_id != 0:
                continue

            # In base_link: x is forward, y is left
            x = det.bbox3d.center.position.x
            y = det.bbox3d.center.position.y
            dist = math.sqrt(x**2 + y**2)

            # Selection Logic
            if self.current_target_id:
                if det.id == self.current_target_id:
                    best_person = det
                    break
            else:
                # If no target ID, track the closest person
                if dist < min_dist:
                    min_dist = dist
                    best_person = det

        if best_person:
            self.publish_tracking_info(best_person, msg.header)

    def publish_tracking_info(self, det: Detection, header):
        track_msg = PersonTracking()
        track_msg.header = header
        track_msg.id = det.id
        
        # 1. Image Coordinates (u, v)
        # BBox center in pixels (from Detection.msg -> BoundingBox2D.msg)
        track_msg.u = det.bbox.center.position.x
        track_msg.v = det.bbox.center.position.y
        
        # 2. Distance and 3D position
        # Position in base_link (from Detection.msg -> BoundingBox3D.msg)
        x = det.bbox3d.center.position.x # Forward
        y = det.bbox3d.center.position.y # Left
        z = det.bbox3d.center.position.z # Up
        
        track_msg.distance = math.sqrt(x**2 + y**2 + z**2)
        
        # Detailed 3D position
        track_msg.position_3d.x = x
        track_msg.position_3d.y = y
        track_msg.position_3d.z = z

        # Publish
        self.tracking_pub.publish(track_msg)
        
        # Log occasionally for debugging
        # self.get_logger().info(f'Tracking ID: {det.id} | Dist: {track_msg.distance:.2f}m | Img: ({track_msg.u:.1f}, {track_msg.v:.1f})')

def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
