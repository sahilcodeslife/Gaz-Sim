import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

import serial
import pynmea2  # Make sure to install with: pip install pynmea2

class MiniGPSNode(Node):
    def __init__(self):
        super().__init__('mini_gps_node')

        self.port = '/dev/ttyACM0' # Port for GPS
        self.baud = 9600

        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.get_logger().info('Mini GPS Node started. Listening on /gps/fix')

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            return

        self.timer = self.create_timer(1.0, self.read_gps)

    def read_gps(self):
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)

                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'

                gps_msg.status.status = NavSatStatus.STATUS_FIX if int(msg.gps_qual) > 0 else NavSatStatus.STATUS_NO_FIX
                gps_msg.status.service = NavSatStatus.SERVICE_GPS

                gps_msg.latitude = self._convert_to_decimal(msg.lat, msg.lat_dir)
                gps_msg.longitude = self._convert_to_decimal(msg.lon, msg.lon_dir)
                gps_msg.altitude = float(msg.altitude) if msg.altitude else 0.0

                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.publisher_.publish(gps_msg)
                self.get_logger().info(f'Published GPS: lat={gps_msg.latitude}, lon={gps_msg.longitude}, alt={gps_msg.altitude}')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse GPS: {e}')

    def _convert_to_decimal(self, raw_val, direction):
        if not raw_val:
            return 0.0
        d, m = divmod(float(raw_val), 100)
        decimal = d + (m / 60)
        if direction in ['S', 'W']:
            decimal = -decimal
        return decimal

def main(args=None):
    rclpy.init(args=args)
    node = MiniGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()