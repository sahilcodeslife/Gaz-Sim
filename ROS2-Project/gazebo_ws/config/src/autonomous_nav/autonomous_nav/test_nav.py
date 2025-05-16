import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import math

# note: code here has yet to implement AriaNode

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')

        # Subscription to GPS and IMU data
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10) # see other node
        #self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10) # see other node
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)  # see other node
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) # see other node

        # latitude, longitude waypoint from .kml file (earth coordinates - decimal)
        self.waypoints = [
            (-31.98057, 115.81716),  # Point 1
            (-31.98038, 115.81717),  # Point 2
            (-31.98017, 115.81717),  # Point 3
            (-31.98082, 115.81713),  # Point 4
            (-31.98081, 115.81746),  # Point 5
            (-31.98041, 115.81756)   # Point 6
        ]

        # Current state of navigation
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.lidar_ranges = []
        self.current_target_index = 0  # Start with the first waypoint

    def gps_callback(self, msg): # get current location
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.get_logger().info(f"Received GPS data: lat={self.current_lat}, lon={self.current_lon}")
        self.navigate()
    
    def lidar_callback(self, msg: LaserScan): # get lidar reading
        self.lidar_ranges = msg.ranges
        self.get_logger().info(f"Received LIDAR data with {len(self.lidar_ranges)} points")

    def imu_callback(self, msg): # get yaw (see below for notes)
        self.current_heading = self.quaternion_to_yaw(msg.orientation)
        self.get_logger().info(f"Received IMU data: heading={self.current_heading}")

    def quaternion_to_yaw(self, q): 
        # convert quaterion q.x, q.y and q.z into euler angles roll, pitch roll
        # note: yaw = left/right, roll = "wing tilt left/right", pitch = "nose up/down"
        # only need yaw for steering 

        qw = q.w # scalar = "real" part of quarternion
        qx = q.x # vector = "imaginary" part of quarternion
        qy = q.y # vector = "imaginary" part
        qz = q.z # vector = "imaginary" part

        # Calculate yaw using the quaternion formula
        yaw = math.atan2(2*(qw*qz + qx*qy), (1 - 2*(qy * qy + qz * qz)))

        # Convert to degrees
        return math.degrees(yaw)

        # _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w]) # euler_from_quaternion is a function apparently
        # return math.degrees(yaw)

    def angle_calculate(self, latitude_cur, longitude_cur, latitude_targ, longitude_targ):
        longitude_cur = math.radians(longitude_cur)
        longitude_targ = math.radians(longitude_targ)
        latitude_cur = math.radians(latitude_cur)
        latitude_targ = math.radians(latitude_targ)

        ang_calc = math.atan2(longitude_targ - longitude_cur, latitude_targ - latitude_cur)
        local_angle = math.degrees(ang_calc)
        return local_angle

    def navigate(self):
        # Ensure we have current GPS and IMU data
        if None in (self.current_lat, self.current_lon, self.current_heading):
            self.get_logger().warn("data missing, skipping navigation step")
            return

        # target latitude, longitude
        target_lat, target_lon = self.waypoints[self.current_target_index]

        # find angle using current latitude, longitude and target latitude, longitude
        bearing = self.angle_calculate(self.current_lat, self.current_lon, target_lat, target_lon)
        angle_diff = bearing - self.current_heading
        twist = Twist()

        # change target once reached unless lap complete
        if self.is_at_target(target_lat, target_lon):
            self.get_logger().info(f"no. of target reach: {self.current_target_index + 1}")
            self.current_target_index += 1
            if self.current_target_index >= len(self.waypoints):
                self.get_logger().info("all waypoints reached")
                return

        # Check for obstacles using LIDAR before moving
        if self.obstacle_detected():
            self.get_logger().info("obstacle detected")
            self.follow_wall(twist)
        else:
            # Steering control
            if angle_diff < 10 or angle_diff > 350:
                twist.linear.x = 0.3  # Move forward
            elif angle_diff < 180:
                twist.angular.z = 0.3  # Turn left
            else:
                twist.angular.z = -0.3  # Turn right

        self.cmd_pub.publish(twist)

    def is_at_target(self, target_lat, target_lon, threshold=0.0001):
        # Simple check if we are within a threshold distance of the target
        lat_diff = abs(self.current_lat - target_lat)
        lon_diff = abs(self.current_lon - target_lon)
        return lat_diff < threshold and lon_diff < threshold

    def obstacle_detected(self, distance_threshold=1.0):
        # Check LIDAR readings in front of robot for obstacles
        if not self.lidar_ranges:
            return False
        front_angles = range(-45, 225) # see RobLab.org 
        mid_index = len(self.lidar_ranges) // 2
        for offset in front_angles:
            index = mid_index + offset
            if 0 <= index < len(self.lidar_ranges):
                if 0.01 < self.lidar_ranges[index] < distance_threshold:
                    return True
        return False
    
    def follow_wall(self, twist: Twist): # follow left wall
        boundary_distance = 0.1  # dist from wall in m (ask team!)
        speed = 0.3 

        self.get_logger().info("obstacle, follow wall")

        if self.lidar_ranges:  # valid lidar or not
            left_distances = self.lidar_ranges[90:225]  # sensor for left side only (see RobLab.org)

            # calc average distance to left cone
            left_distance = sum(left_distances) / len(left_distances)
            # too far from wall
            if left_distance > (boundary_distance + 0.1):
                twist.angular.z = 0.3  # left
                twist.linear.x = speed  
            # too close from wall
            elif left_distance < (boundary_distance + 0.1):
                twist.angular.z = -0.3  # right
                twist.linear.x = speed  
            else:
                twist.angular.z = 0.0  # Stop turning
                twist.linear.x = speed  # Move forward

        else:
            # return to nav() function above
            self.get_logger().info("no obstacles")
            self.navigate() # go back to navigation

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
