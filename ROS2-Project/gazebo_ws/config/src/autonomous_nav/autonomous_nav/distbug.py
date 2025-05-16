# changes:
# deleted subscription to /cone_visible and visible_cd method
# modified can_leave_boundary to rely solely on m-line condition (d_now < di_hit and m_line_dist < 0.2) for exiting BOUNDARY_FOLLOW
# preserved all other functionality of the original distbug code

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WallFollowerNode(Node):
    def __init__(self):
        super().__init__('wall_follower_node')

        # Parameters
        self.declare_parameter('wall_dist_desired', 0.5)  # Desired distance to wall
        self.declare_parameter('max_forward_speed', 0.5)  # Max linear speed
        self.declare_parameter('max_angular_speed', 1.0)  # Max angular speed
        self.declare_parameter('goal_x', 5.0)  # Goal x-coordinate
        self.declare_parameter('goal_y', 5.0)  # Goal y-coordinate
        self.declare_parameter('follow_direction', 'right')  # 'left' or 'right'
        self.wall_dist_desired = self.get_parameter('wall_dist_desired').value
        self.max_forward_speed = self.get_parameter('max_forward_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_pose = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)  # (x, y)
        self.follow_direction = self.get_parameter('follow_direction').value

        # State variables
        self.start_pose = None  # (x,y,theta) at start
        self.current_pose = None  # (x,y,theta) from odom
        self.latest_scan = None
        self.hit_point = None  # (x,y) where wall-following started
        self.position_buffer = []  # For stuck detection
        self.buffer_size = 100
        self.stuck_threshold = 0.5  # Meters
        self.is_wall_following = True  # Control wall-following state

        # Publishers & subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.cmd = Twist()  # Initialize cmd

    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        self.current_pose = (
            pose.position.x,
            pose.position.y,
            2 * math.atan2(pose.orientation.z, pose.orientation.w)  # yaw
        )
        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.hit_point = (self.current_pose[0], self.current_pose[1])  # Set hit point at start

    def control_loop(self):
        if self.current_pose is None or self.latest_scan is None or not self.is_wall_following:
            return  # Wait for data or stop if not wall-following

        # Check for stuck condition
        self.check_stuck()

        # Check m-line condition to stop wall-following
        if self.can_leave_boundary():
            self.get_logger().info('M-line condition satisfied, stopping wall-following')
            self.is_wall_following = False
            self.cmd = Twist()  # Stop publishing commands
            self.cmd_pub.publish(self.cmd)
            return

        # Follow wall
        self.follow_wall()
        self.cmd_pub.publish(self.cmd)

    def follow_wall(self):
        """Maintain desired distance from obstacle on chosen side."""
        scan = self.latest_scan
        idx_center = int((-math.pi/2 - scan.angle_min) / scan.angle_increment) if self.follow_direction == 'right' else \
                     int((math.pi/2 - scan.angle_min) / scan.angle_increment)
        # Average 5 points for robustness
        indices = range(idx_center - 2, idx_center + 3)
        valid_ranges = [scan.ranges[i] for i in indices if 0.0 < scan.ranges[i] < 10.0]
        if not valid_ranges:
            self.cmd = Twist()  # Stop if no valid readings
            self.get_logger().warn('No valid wall distance, stopping')
            return
        side_dist = sum(valid_ranges) / len(valid_ranges)
        if side_dist < 0.2:  # Critical distance
            self.cmd = Twist()
            self.cmd.linear.x = -0.1  # Back up
            self.cmd.angular.z = 0.0
            self.get_logger().warn('Too close to obstacle, backing up')
            return
        error = (self.wall_dist_desired - side_dist) * (1 if self.follow_direction == 'right' else -1)
        ang_z = max(-self.max_angular_speed, min(self.max_angular_speed, 1.0 * error))
        self.cmd = Twist()
        self.cmd.linear.x = 0.2
        self.cmd.angular.z = ang_z

    def distance_to_m_line(self, x, y):
        """Calculate perpendicular distance to m-line (start to goal)."""
        gx, gy = self.goal_pose
        sx, sy = self.start_pose[0], self.start_pose[1]
        a = gy - sy
        b = sx - gx
        c = gx * sy - gy * sx
        return abs(a * x + b * y + c) / math.sqrt(a**2 + b**2) if a**2 + b**2 > 0 else float('inf')

    def can_leave_boundary(self) -> bool:
        """Check if m-line condition is met to stop wall-following."""
        x, y, _ = self.current_pose
        gx, gy = self.goal_pose
        d_now = math.hypot(gx - x, gy - y)
        hx, hy = self.hit_point
        d_hit = math.hypot(gx - hx, gy - hy)
        m_line_dist = self.distance_to_m_line(x, y)
        # Stop wall-following if closer to goal than hit point and near m-line
        return m_line_dist < 0.2 and abs(d_now - d_hit) < 0.3

    def check_stuck(self):
        """Detect if robot is stuck by checking position history."""
        if not self.current_pose:
            return
        x, y, _ = self.current_pose
        self.position_buffer.append((x, y))
        if len(self.position_buffer) > self.buffer_size:
            self.position_buffer.pop(0)
            for px, py in self.position_buffer[:-1]:
                if math.hypot(x - px, y - py) < self.stuck_threshold:
                    self.get_logger().warn('Stuck detected, stopping wall-following')
                    self.is_wall_following = False
                    self.cmd = Twist()
                    self.cmd_pub.publish(self.cmd)
                    break

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()