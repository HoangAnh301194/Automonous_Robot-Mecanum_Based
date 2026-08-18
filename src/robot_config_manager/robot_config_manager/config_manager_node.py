import rclpy
from rclpy.node import Node
import yaml
import os

from robot_interfaces.srv import (
    SaveLocation, GetLocation, DeleteLocation, 
    SetLanguage, FinishSetup, SaveRoute, GetRoute
)

class ConfigManagerNode(Node):
    def __init__(self):
        super().__init__('config_manager_node')
        
        # Determine workspace config path
        self.config_dir = os.path.join(os.path.expanduser('~'), 'robot_ws', 'config')
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)
            
        self.locations_file = os.path.join(self.config_dir, 'locations.yaml')
        self.robot_config_file = os.path.join(self.config_dir, 'robot_config.yaml')
        self.routes_file = os.path.join(self.config_dir, 'routes.yaml')
        
        self.locations = self.load_yaml(self.locations_file)
        self.robot_config = self.load_yaml(self.robot_config_file)
        self.routes = self.load_yaml(self.routes_file)
        
        # Services
        self.srv_save_loc = self.create_service(SaveLocation, '/config/save_location', self.cb_save_location)
        self.srv_get_loc = self.create_service(GetLocation, '/config/get_location', self.cb_get_location)
        self.srv_del_loc = self.create_service(DeleteLocation, '/config/delete_location', self.cb_delete_location)
        self.srv_set_lang = self.create_service(SetLanguage, '/config/set_language', self.cb_set_language)
        self.srv_finish_setup = self.create_service(FinishSetup, '/config/finish_setup', self.cb_finish_setup)
        self.srv_save_route = self.create_service(SaveRoute, '/config/save_route', self.cb_save_route)
        self.srv_get_route = self.create_service(GetRoute, '/config/get_route', self.cb_get_route)
        
        self.get_logger().info('Config Manager Node started. YAML persistence enabled.')

    def load_yaml(self, path):
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        return {}

    def save_yaml(self, data, path):
        with open(path, 'w') as f:
            yaml.dump(data, f)

    def cb_save_location(self, request, response):
        name = request.name
        self.locations[name] = {
            'x': request.x,
            'y': request.y,
            'yaw': request.yaw
        }
        self.save_yaml(self.locations, self.locations_file)
        
        response.success = True
        response.message = f"Location {name} saved."
        self.get_logger().info(response.message)
        return response

    def cb_get_location(self, request, response):
        name = request.name
        if name in self.locations:
            coords = self.locations[name]
            response.x = float(coords['x'])
            response.y = float(coords['y'])
            response.yaw = float(coords['yaw'])
            response.success = True
            response.message = f"Location {name} loaded."
        else:
            response.success = False
            response.message = f"Location {name} not found."
            
        return response

    def cb_delete_location(self, request, response):
        name = request.name
        if name in self.locations:
            del self.locations[name]
            self.save_yaml(self.locations, self.locations_file)
            response.success = True
            response.message = f"Location '{name}' deleted."
        else:
            response.success = False
            response.message = f"Location '{name}' not found."
        self.get_logger().info(response.message)
        return response

    def cb_set_language(self, request, response):
        self.robot_config['language'] = request.language
        self.save_yaml(self.robot_config, self.robot_config_file)
        response.success = True
        response.message = f"Language set to {request.language}"
        self.get_logger().info(response.message)
        return response
        
    def cb_finish_setup(self, request, response):
        self.robot_config['setup_completed'] = request.is_finished
        self.save_yaml(self.robot_config, self.robot_config_file)
        response.success = True
        response.message = f"Setup completed set to {request.is_finished}."
        self.get_logger().info(response.message)
        return response

    def cb_save_route(self, request, response):
        route_name = request.route_name
        self.routes[route_name] = list(request.waypoints)
        self.save_yaml(self.routes, self.routes_file)
        response.success = True
        response.message = f"Route {route_name} saved with {len(request.waypoints)} waypoints."
        self.get_logger().info(response.message)
        return response

    def cb_get_route(self, request, response):
        route_name = request.route_name
        if route_name in self.routes:
            response.waypoints = self.routes[route_name]
            response.success = True
            self.get_logger().info(f"Route '{route_name}' loaded with {len(response.waypoints)} waypoints.")
        else:
            response.waypoints = []
            response.success = False
            self.get_logger().info(f"Route '{route_name}' not found.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConfigManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
