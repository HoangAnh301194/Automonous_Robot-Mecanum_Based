import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import sys

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.create_subscription(Image, '/front_camera/depth/image_raw', self.front_cb, 10)
        self.create_subscription(Image, '/rear_camera/depth/image_raw', self.rear_cb, 10)
        self.front_done = False
        self.rear_done = False

    def process(self, msg, name):
        arr = np.frombuffer(msg.data, dtype=np.float32)
        valid = arr[np.isfinite(arr)]
        if len(valid) == 0:
            print(f"{name}: All values are inf/nan (Total: {len(arr)})")
        else:
            print(f"{name}: Valid pixels: {len(valid)}/{len(arr)}, Min: {valid.min():.2f}, Max: {valid.max():.2f}, Mean: {valid.mean():.2f}")
        
    def front_cb(self, msg):
        if not self.front_done:
            self.process(msg, "FRONT")
            self.front_done = True
            self.check_exit()
            
    def rear_cb(self, msg):
        if not self.rear_done:
            self.process(msg, "REAR")
            self.rear_done = True
            self.check_exit()
            
    def check_exit(self):
        if self.front_done and self.rear_done:
            sys.exit(0)

rclpy.init()
node = DepthChecker()
rclpy.spin(node)
