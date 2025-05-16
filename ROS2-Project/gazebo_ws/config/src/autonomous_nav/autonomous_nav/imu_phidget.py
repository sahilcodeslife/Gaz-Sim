import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion

class PhidgetIMU(Node):
    def __init__(self):
        super().__init__('phidget_imu')
        # publisher of just the yaw heading (radians)
        self.pub = self.create_publisher(Float32, '/imu/data', 10)
        # subscribe to the raw Phidget Imu
        self.sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)

    def imu_callback(self, msg):
        # 1) extract quaternion → yaw (in radians)
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 2) to degrees + your 12° offset
        deg = math.degrees(yaw)
        deg += 81.0
        # 3) normalize into [0,360)
        deg = (deg + 360.0) % 360.0


        # 4) publish
        heading = Float32()
        heading.data = deg
        self.get_logger().info(f"Published heading: {deg:.1f}°")
        self.pub.publish(heading)

def main(args=None):
    rclpy.init(args=args)
    node = PhidgetIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()