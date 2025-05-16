import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from geometry_msgs.msg import Twist

class LidarNavigationNode(Node):
    def __init__(self):
        super().__init__('lidar_navigation')
        self.lidar_subscriber = self.create_subscription(LaserScan,'/scan', self.lidar_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.goal_subscriber = self.create_subscription(NavSatFix, '/gps/fix/goal', self.goal_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_position = NavSatFix()
        self.goal_position = NavSatFix()

        self.obstacle_threshold = 1 #0.0000002 # 2m in earth coordinate
        self.is_obstacle_detected = False

    def gps_callback(self, msg: NavSatFix):
        self.current_position = msg

    def goal_callback(self, msg: NavSatFix):
        self.goal_position = msg

    def lidar_callback(self, msg: LaserScan):
        self.is_obstacle_detected = self.detect_obstacles(msg.ranges)
        self.navigate()

    def detect_obstacles(self, ranges):
        for distance in ranges: # [-45..225]
            if distance < self.obstacle_threshold and distance > 0:
                return True
        return False

    def navigate(self):
        twist = Twist() # velocity command

        if self.is_obstacle_detected:
            self.get_logger().info("obstacle detected")
        else: 
            self.get_logger().info("goal visible")
            #self.get_logger().info(f"Moving towards goal: {self.goal_position.latitude}, {self.goal_position.longitude}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarNavigationNode()
    rclpy.spin(node)
    # lidar_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
