import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class GPSWaypointDriver(Node):
    def __init__(self):
        super().__init__('gps_waypoint_driver')

        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_position = None
        self.current_index = 0
        self.goal_reached = False

        self.waypoints = [
            (31.2296, 121.4768),
            (31.2298, 121.4769),
            (31.2300, 121.4770),
        ]

        self.timer = self.create_timer(1.0, self.control_loop)
        self.get_logger().info("Waypoint driver node started.")

    def gps_callback(self, msg):
        self.current_position = (msg.latitude, msg.longitude)

    def control_loop(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached. Stopping.")
            self.stop()
            return

        if not self.current_position:
            return

        goal = self.waypoints[self.current_index]
        dist = self.haversine(self.current_position, goal)

        self.get_logger().info(
            f"[Waypoint {self.current_index + 1}] Distance: {dist:.2f} m")

        if dist < 1.5:  # Threshold in meters
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}")
            self.current_index += 1
            self.stop()
        else:
            self.drive_forward()

    def drive_forward(self):
        twist = Twist()
        twist.linear.x = 0.3
        twist.angular.z = 0.0  # Basic forward movement
        self.cmd_pub.publish(twist)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def haversine(self, pos1, pos2):
        lat1, lon1 = pos1
        lat2, lon2 = pos2
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2)**2 + math.cos(math.radians(lat1)) * \
            math.cos(math.radians(lat2)) * math.sin(dlon / 2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()