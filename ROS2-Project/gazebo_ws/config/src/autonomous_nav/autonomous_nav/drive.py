import rclpy
import random
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RandomDriver(Node):
    def __init__(self):
        super().__init__('random_driver')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.move_robot)  # Run every 2 seconds

    def move_robot(self):
        msg = Twist()
        msg.linear.x = random.uniform(0.1, 0.5)  # Move forward at random speed
        msg.angular.z = random.uniform(-1.0, 1.0)  # Random left/right turn

        self.publisher_.publish(msg)
        self.get_logger().info(f"Moving - Linear: {msg.linear.x}, Angular: {msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()