import rclpy
from rclpy.node import Node
import yaml
import os

from robot_interfaces.srv import SaveLocation, GetLocations, SetLanguage
from std_srvs.srv import Trigger

class ConfigManagerNode(Node):
    def __init__(self):
        super().__init__('config_manager_node')
        
        self.config_dir = os.path.expanduser('~/.robot_config')
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)
            
        self.locations_file = os.path.join(self.config_dir, 'locations.yaml')
        self.settings_file = os.path.join(self.config_dir, 'settings.yaml')
        
        self.locations = self.load_yaml(self.locations_file)
        self.settings = self.load_yaml(self.settings_file)
        
        # Services
        self.srv_save_loc = self.create_service(SaveLocation, '/config/save_location', self.cb_save_location)
        self.srv_get_locs = self.create_service(GetLocations, '/config/get_locations', self.cb_get_locations)
        self.srv_set_lang = self.create_service(SetLanguage, '/config/set_language', self.cb_set_language)
        self.srv_finish_setup = self.create_service(Trigger, '/config/finish_setup', self.cb_finish_setup)
        
        self.get_logger().info('Config Manager Node started.')

    def load_yaml(self, path):
        if os.path.exists(path):
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        return {}

    def save_yaml(self, data, path):
        with open(path, 'w') as f:
            yaml.dump(data, f)

    def cb_save_location(self, request, response):
        loc_type = request.location_type
        if loc_type not in self.locations:
            self.locations[loc_type] = {}
            
        self.locations[loc_type][request.location_name] = {
            'x': request.x,
            'y': request.y,
            'theta': request.theta
        }
        self.save_yaml(self.locations, self.locations_file)
        
        response.success = True
        response.message = f"Location {request.location_name} saved as {loc_type}."
        self.get_logger().info(response.message)
        return response

    def cb_get_locations(self, request, response):
        loc_type = request.location_type
        response.names = []
        response.x = []
        response.y = []
        response.theta = []
        
        if loc_type in self.locations:
            for name, coords in self.locations[loc_type].items():
                response.names.append(name)
                response.x.append(float(coords['x']))
                response.y.append(float(coords['y']))
                response.theta.append(float(coords['theta']))
            response.success = True
            response.message = f"Loaded {len(response.names)} locations."
        else:
            response.success = False
            response.message = "Location type not found."
            
        return response

    def cb_set_language(self, request, response):
        self.settings['language'] = request.language
        self.save_yaml(self.settings, self.settings_file)
        response.success = True
        response.message = f"Language set to {request.language}"
        self.get_logger().info(response.message)
        return response
        
    def cb_finish_setup(self, request, response):
        self.settings['setup_completed'] = True
        self.save_yaml(self.settings, self.settings_file)
        response.success = True
        response.message = "Setup completed. System is READY."
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ConfigManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
