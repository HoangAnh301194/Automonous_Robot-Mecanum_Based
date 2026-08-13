import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import board
import busio
import adafruit_bno055


class BNO055IMUPublisher(Node):
    def __init__(self):
        super().__init__('bno055_publisher_node')

        # Parameters
        self.declare_parameter('topic_name', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)          # Hz
        self.declare_parameter('reconnect_interval', 2.0)     # seconds
        self.declare_parameter('orientation_covariance', [0.01, 0.0, 0.0,
                                                          0.0, 0.01, 0.0,
                                                          0.0, 0.0, 0.01])
        self.declare_parameter('angular_velocity_covariance', [0.005, 0.0, 0.0,
                                                               0.0, 0.005, 0.0,
                                                               0.0, 0.0, 0.005])
        self.declare_parameter('linear_acceleration_covariance', [0.1, 0.0, 0.0,
                                                                  0.0, 0.1, 0.0,
                                                                  0.0, 0.0, 0.1])

        self.topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.reconnect_interval = float(self.get_parameter('reconnect_interval').value)

        self.orientation_covariance = list(self.get_parameter('orientation_covariance').value)
        self.angular_velocity_covariance = list(self.get_parameter('angular_velocity_covariance').value)
        self.linear_acceleration_covariance = list(self.get_parameter('linear_acceleration_covariance').value)

        self.publisher = self.create_publisher(Imu, self.topic_name, 10)

        self.i2c = None
        self.sensor = None
        self.connected = False
        self.last_reconnect_attempt = 0.0

        publish_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.02
        self.timer = self.create_timer(publish_period, self.timer_callback)

        self.get_logger().info(
            f'Starting BNO055 publisher | topic={self.topic_name} frame_id={self.frame_id} rate={self.publish_rate:.1f}Hz'
        )

        self.connect_sensor(initial=True)

    def connect_sensor(self, initial=False):
        now = time.monotonic()
        if not initial and (now - self.last_reconnect_attempt) < self.reconnect_interval:
            return

        self.last_reconnect_attempt = now

        try:
            self.get_logger().info('Attempting to connect to BNO055...')
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

            # Touch sensor once to verify it is really alive
            _ = self.sensor.temperature

            self.connected = True
            self.get_logger().info('Connected to BNO055 successfully.')

        except Exception as e:
            self.sensor = None
            self.i2c = None
            self.connected = False
            self.get_logger().error(f'Failed to connect to BNO055: {e}')

    def disconnect_sensor(self, reason='Unknown error'):
        if self.connected:
            self.get_logger().warn(f'BNO055 disconnected: {reason}')

        self.sensor = None
        self.i2c = None
        self.connected = False

    @staticmethod
    def is_valid_number(value):
        return value is not None and isinstance(value, (int, float)) and math.isfinite(value)

    def timer_callback(self):
        if not self.connected or self.sensor is None:
            self.connect_sensor()
            return

        try:
            quat = self.sensor.quaternion
            gyro = self.sensor.gyro
            accel = self.sensor.acceleration

            # N?u d? li?u chua s?n sàng thì b? qua chu k? này
            if quat is None or gyro is None or accel is None:
                self.get_logger().warn(
                    'BNO055 data not ready yet (quat/gyro/accel is None).',
                    throttle_duration_sec=2.0
                )
                return

            # Validate d? li?u d? tránh NaN/inf làm h?ng downstream node
            values = [quat[0], quat[1], quat[2], quat[3],
                      gyro[0], gyro[1], gyro[2],
                      accel[0], accel[1], accel[2]]

            if not all(self.is_valid_number(v) for v in values):
                self.get_logger().warn(
                    'Invalid IMU data detected (NaN/Inf/None). Skipping publish.',
                    throttle_duration_sec=2.0
                )
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # BNO055 Adafruit tr? quaternion theo th? t? (w, x, y, z)
            imu_msg.orientation.w = float(quat[0])
            imu_msg.orientation.x = float(quat[1])
            imu_msg.orientation.y = float(quat[2])
            imu_msg.orientation.z = float(quat[3])
            imu_msg.orientation_covariance = self.orientation_covariance

            imu_msg.angular_velocity.x = float(gyro[0])
            imu_msg.angular_velocity.y = float(gyro[1])
            imu_msg.angular_velocity.z = float(gyro[2])
            imu_msg.angular_velocity_covariance = self.angular_velocity_covariance

            imu_msg.linear_acceleration.x = float(accel[0])
            imu_msg.linear_acceleration.y = float(accel[1])
            imu_msg.linear_acceleration.z = float(accel[2])
            imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

            self.publisher.publish(imu_msg)

        except OSError as e:
            # I2C hay b? OSError khi bus l?i ho?c sensor r?t
            self.disconnect_sensor(reason=f'I2C/OSError: {e}')

        except Exception as e:
            # Các l?i b?t thu?ng khác cung xem nhu m?t k?t n?i d? t? recovery
            self.disconnect_sensor(reason=f'Unexpected read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BNO055IMUPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down BNO055 publisher node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
