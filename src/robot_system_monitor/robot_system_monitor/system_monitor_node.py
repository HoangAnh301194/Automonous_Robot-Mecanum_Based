import rclpy
from rclpy.node import Node
from robot_interfaces.msg import RobotStatus
from std_srvs.srv import Trigger
import random

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')
        
        self.status_pub = self.create_publisher(RobotStatus, '/hmi/robot_status', 10)
        self.srv_check_ready = self.create_service(Trigger, '/monitor/check_setup_ready', self.cb_check_ready)
        
        # Timer for publishing status (~1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Mock values for demonstration
        self.battery_level = 85.0
        self.water_level = 5.0 # Liters
        self.kettle_weight = 1.0 # kg
        self.water_density = 1.0 # kg/L
        
        self.get_logger().info('System Monitor Node started. Monitoring Battery & Water System via Serial.')
        
    def cb_check_ready(self, request, response):
        if self.battery_level > 35.0:
            response.success = True
            response.message = f"Battery OK ({self.battery_level}%)"
        else:
            response.success = False
            response.message = f"Low Battery ({self.battery_level}%). Please charge."
        return response
        
    def read_load_cell_serial(self):
        # In a real scenario, this would read from a Serial port connected to Arduino/HX711
        # Example:
        # line = self.serial_port.readline().decode('utf-8').strip()
        # raw_weight = float(line)
        
        # Simulating serial load cell reading:
        raw_weight = self.kettle_weight + self.water_level + random.uniform(-0.05, 0.05)
        
        # Calculate water volume in liters
        calculated_water_weight = raw_weight - self.kettle_weight
        calculated_water_liters = calculated_water_weight / self.water_density
        
        return max(0.0, float(calculated_water_liters))

    def timer_callback(self):
        msg = RobotStatus()
        msg.state = "READY"
        msg.battery_level = self.battery_level
        msg.is_charging = False
        msg.wifi_signal = "Excellent"
        msg.current_point = "Departure"
        msg.localization_ok = True
        
        # Update water level from serial load cell
        msg.water_level = self.read_load_cell_serial()
        msg.water_system_ready = True
        
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
