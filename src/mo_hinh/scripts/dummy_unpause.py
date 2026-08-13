#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class DummyUnpauseNode(Node):
    def __init__(self):
        super().__init__('dummy_unpause_service_node')
        self.srv = self.create_service(Empty, '/unpause_physics', self.unpause_callback)
        self.get_logger().info('Dummy /unpause_physics service is ready.')

    def unpause_callback(self, request, response):
        self.get_logger().info('Received unpause_physics call -> returning OK')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DummyUnpauseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
