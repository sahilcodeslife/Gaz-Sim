import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        # Subscription to joystick input
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        # Publisher for velocity commands (manual mode)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher for mode changes (to sync with gps_driving.py)
        self.mode_publisher = self.create_publisher(String, '/robot_mode', 10)
        self.dead_publisher = self.create_publisher(Bool, '/dead_switch', 10)
        # Mode and switch states
        self.automated_mode = False
        self.dead_man_switch = False
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz
        # Current velocity command
        self.twist = Twist()
        self.get_logger().info("JoyControl node started")

    def joy_callback(self, msg):
        # Button 0: Switch to automated mode
        if msg.buttons[0] == 1:
            self.automated_mode = True
            mode_msg = String()
            mode_msg.data = 'automated'
            self.mode_publisher.publish(mode_msg)
            self.get_logger().info("Switched to automated mode")
        # Button 1: Switch to manual mode
        elif msg.buttons[1] == 1:
            self.automated_mode = False
            mode_msg = String()
            mode_msg.data = 'manual'
            self.mode_publisher.publish(mode_msg)
            self.get_logger().info("Switched to manual mode")

        # Dead-man switch (L2 or R2)
        self.dead_man_switch = msg.axes[2] > 0 or msg.axes[5] > 0  # L2 or R2 pressed
        self.dead_publisher.publish(self.dead_man_switch)

        # In manual mode, update velocity from joystick axes
        if not self.automated_mode:
            self.twist.linear.x = msg.axes[1]  # Forward/backward
            self.twist.angular.z = msg.axes[0]  # Left/right rotation

    def publish_velocity(self):
        if self.automated_mode:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            if self.dead_man_switch:
                self.get_logger().debug("Automated mode")
            else:
                self.get_logger().debug("Automated mode")
        else:
            if not self.dead_man_switch:
                # Stop if dead-man switch is not pressed
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

        self.vel_publisher.publish(self.twist)
        self.get_logger().debug(f"Published: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}")

def main():
    rclpy.init()
    node = JoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_twist = Twist()
        node.vel_publisher.publish(stop_twist)
        node.get_logger().info("Stopped robot on shutdown")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()