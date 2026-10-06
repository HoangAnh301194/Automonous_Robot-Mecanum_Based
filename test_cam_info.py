import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CamInfoListener(Node):
    def __init__(self):
        super().__init__('cam_info_listener')
        self.sub = self.create_subscription(CameraInfo, '/rear_camera/depth/camera_info', self.cb, 1)
    def cb(self, msg):
        self.get_logger().info(f"K: {msg.k}")
        rclpy.shutdown()

rclpy.init()
node = CamInfoListener()
rclpy.spin(node)
