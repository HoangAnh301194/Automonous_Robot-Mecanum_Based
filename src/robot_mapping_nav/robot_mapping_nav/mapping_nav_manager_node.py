import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MappingNavManagerNode(Node):
    def __init__(self):
        super().__init__('mapping_nav_manager_node')
        
        self.srv_start_slam = self.create_service(Trigger, '/mapping/start_slam', self.cb_start_slam)
        self.srv_save_map = self.create_service(Trigger, '/mapping/save_map', self.cb_save_map)
        self.srv_load_map = self.create_service(Trigger, '/mapping/load_map', self.cb_load_map)
        
        self.get_logger().info('Mapping & Nav Manager Node started.')

    def cb_start_slam(self, request, response):
        self.get_logger().info('Starting SLAM...')
        # Logic to launch slam_toolbox goes here
        response.success = True
        response.message = "SLAM started"
        return response

    def cb_save_map(self, request, response):
        self.get_logger().info('Saving Map...')
        # Logic to save map (e.g., using nav2_map_server)
        response.success = True
        response.message = "Map saved successfully"
        return response

    def cb_load_map(self, request, response):
        self.get_logger().info('Loading Map...')
        # Logic to load map
        response.success = True
        response.message = "Map loaded successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MappingNavManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
