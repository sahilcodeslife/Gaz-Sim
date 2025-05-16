import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class MainControl(Node):
    def __init__(self):
        super().__init__('main_control')
        self.joy_sub = self.create_subscription(
            Twist, '/joy_cmd_vel', self.joy_callback, 10)
        self.auto_sub = self.create_subscription(
            Twist, '/auto_cmd_v4el', self.auto_callback, 10)  # For any other nodes that want to write to this node for velocity
        self.mode_sub = self.create_subscription(
            String, '/robot_mode', self.mode_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_mode = "manual"  
        self.joy_twist = Twist()      
        self.auto_twist = Twist()     
        self.timer = self.create_timer(0.1, self.forward_velocity)  # 10 Hz

        self.get_logger().info("MainControl node started")

    def joy_callback(self, msg):
        """Store the latest velocity command from the joystick."""
        self.joy_twist = msg

    def auto_callback(self, msg):
        """Store the latest velocity command from the autonomous node."""
        self.auto_twist = msg

    def mode_callback(self, msg):
        """Update the current mode based on input from JoyControl."""
        self.current_mode = msg.data
        self.get_logger().info(f"Mode switched to: {self.current_mode}")

    def forward_velocity(self):
        """Forward the appropriate velocity command to /cmd_vel based on mode."""
        output_twist = Twist()

        if self.current_mode == "manual":
            output_twist = self.joy_twist
            self.get_logger().debug("Manual commands")
        elif self.current_mode == "automated": # Perform automated sequeunce (that we code)
            output_twist = self.auto_twist
            self.get_logger().debug("Forwarding autonomous command")
        else:
            output_twist.linear.x = 0.0
            output_twist.angular.z = 0.0
            self.get_logger().warn(f"Unknown mode: {self.current_mode}, stopping robot")

        self.cmd_vel_pub.publish(output_twist)

def main():
    rclpy.init()
    node = MainControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.get_logger().info("Stopped robot on shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()