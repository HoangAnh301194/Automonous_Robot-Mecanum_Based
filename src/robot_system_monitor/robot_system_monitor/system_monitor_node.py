import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from robot_interfaces.msg import WaterStatus, RobotStatus


# Đường cong xả 3 bình ắc quy chì Yamato 12V 30Ah ghép nối tiếp (36V)
_BATTERY_36V_CURVE = (
    (33.60, 0.0),    # 11.20V/bình (ngưỡng ngắt an toàn)
    (34.80, 10.0),   # 11.60V/bình (cảnh báo nguy cấp)
    (35.70, 25.0),   # 11.90V/bình (ngưỡng pin yếu)
    (36.45, 50.0),   # 12.15V/bình (mức trung bình)
    (37.20, 75.0),   # 12.40V/bình (dung lượng tốt)
    (37.80, 90.0),   # 12.60V/bình
    (38.40, 100.0),  # 12.80V/bình (đầy)
)


def _voltage_to_percentage_36v(v: float) -> float:
    """Nội suy % dung lượng 3 bình ắc quy chì Yamato 12V 30Ah nối tiếp (36V) từ điện áp."""
    if v <= _BATTERY_36V_CURVE[0][0]:
        return 0.0
    if v >= _BATTERY_36V_CURVE[-1][0]:
        return 100.0
    for (v0, p0), (v1, p1) in zip(_BATTERY_36V_CURVE, _BATTERY_36V_CURVE[1:]):
        if v <= v1:
            return p0 + (v - v0) / (v1 - v0) * (p1 - p0)
    return 100.0


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')

        # Subscriptions
        self.sub_battery = self.create_subscription(BatteryState, '/battery_state', self.cb_battery, 10)
        self.sub_battery_alt = self.create_subscription(BatteryState, '/battery', self.cb_battery, 10)
        self.sub_hoverboard_voltage = self.create_subscription(Float64, '/hoverboard/battery_voltage', self.cb_hoverboard_voltage, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.cb_scan, 10)
        self.sub_water = self.create_subscription(WaterStatus, '/water/status', self.cb_water, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.cb_odom, 10)

        # Publisher
        self.pub_status = self.create_publisher(RobotStatus, '/robot_status', 10)

        # Timers
        self.timer = self.create_timer(0.5, self.publish_status) # 2 Hz

        # State variables (khởi tạo 0.0 thay vì 100.0 để tránh phát pin ảo khi chưa nhận dữ liệu)
        self.battery_percent = 0.0
        self.battery_low = False
        
        self.water_liter = 0.0
        self.water_low = False

        self.last_scan_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.lidar_ok = False
        self.odom_ok = False

        self.get_logger().info('System Monitor Node started.')

    def cb_battery(self, msg):
        # Handle ROS standard (0.0 - 1.0) or (0 - 100)
        if msg.percentage <= 1.0:
            self.battery_percent = msg.percentage * 100.0
        else:
            self.battery_percent = msg.percentage
            
        self.battery_low = (self.battery_percent < 20.0)

    def cb_hoverboard_voltage(self, msg: Float64):
        raw_v = float(msg.data)
        if 20.0 <= raw_v <= 50.0:
            self.battery_percent = _voltage_to_percentage_36v(raw_v)
            self.battery_low = (self.battery_percent < 20.0)

    def cb_scan(self, msg):
        self.last_scan_time = self.get_clock().now()

    def cb_water(self, msg):
        self.water_liter = msg.water_liter
        self.water_low = msg.water_low

    def cb_odom(self, msg):
        self.last_odom_time = self.get_clock().now()

    def publish_status(self):
        now = self.get_clock().now()
        
        # Check timeouts (2.0 seconds without data means disconnected/failed)
        scan_timeout = (now - self.last_scan_time).nanoseconds / 1e9 > 2.0
        self.lidar_ok = not scan_timeout

        odom_timeout = (now - self.last_odom_time).nanoseconds / 1e9 > 2.0
        self.odom_ok = not odom_timeout

        # Construct status message
        msg = RobotStatus()
        msg.battery_percent = float(self.battery_percent)
        msg.battery_low = self.battery_low
        
        msg.lidar_ok = self.lidar_ok
        msg.odom_ok = self.odom_ok
        msg.localization_ok = True # Mocking localization for now
        
        msg.water_liter = float(self.water_liter)
        msg.water_low = self.water_low
        
        msg.robot_state = 'IDLE' 
        msg.current_location = 'unknown'
        
        self.pub_status.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
