import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        
        # Random coordinates
        self.waypoints = [
            (-31.980, 115.818),  
            (-31.981, 115.819), 
            (-31.979, 115.817),  
            (-31.980, 115.818)   
        ]

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_waypoint_idx = 0
        
        # Ros publishing?
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # random timer for now
        self.timer = self.create_timer(5.0, self.navigate)
        
        self.get_logger().info('Navigator started. Visiting random set of coordinates...')


    def send_drive_command(self):
        # Create a Twist message
        twist = Twist()
        # Example: drive forward at 0.2 m/s, no rotation
        twist.linear.x = 0.2
        twist.angular.z = 0.0

        # Publish the message
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Publishing drive command: linear.x=0.2, angular.z=0.0")

    def navigate(self):
        if self.current_waypoint_idx < len(self.waypoints):
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = float(self.waypoints[self.current_waypoint_idx][0])  # Latitude
            goal_msg.pose.position.y = float(self.waypoints[self.current_waypoint_idx][1])  # Longitude
            goal_msg.pose.position.z = 0.0  
            goal_msg.pose.orientation.w = 1.0  

            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_idx}: '
                f'({self.waypoints[self.current_waypoint_idx][0]}, '
                f'{self.waypoints[self.current_waypoint_idx][1]})'
            )
            
            self.current_waypoint_idx += 1
        else:
            self.get_logger().info('All waypoints visited. ')
            self.timer.cancel() 

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()