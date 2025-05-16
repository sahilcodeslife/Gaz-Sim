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
        self.publisher = self.create_publisher(Twist, '/joy_cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        # Mode and switch states
        self.automated_mode = False
        self.dead_man_switch = False
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz
        # Current velocity command
        self.twist = Twist()

    def joy_callback(self, msg):
        # Button 0 → automated, Button 1 → manual
        if msg.buttons[0] == 1 and not self.automated_mode:
            self.automated_mode = True
            self.get_logger().info("Switched to AUTOMATED mode")
            self._publish_mode()
        elif msg.buttons[1] == 1 and self.automated_mode:
            self.automated_mode = False
            self.get_logger().info("Switched to MANUAL mode")
            self._publish_mode()

        if self.automated_mode: # if automated
            self.dead_man_switch = msg.axes[2] > 0 or msg.axes[5] > 0  # L2 or R2 pressed

        # if manual
        if not self.automated_mode:
            self.twist.linear.x = msg.axes[1] * 4.0
            self.twist.angular.z = msg.axes[0] * 1.0

    def _publish_mode(self):
        m = String()
        m.data = 'automated' if self.automated_mode else 'manual'
        self.mode_pub.publish(m)

    def publish_velocity(self):
        if self.automated_mode:
            if self.dead_man_switch:
                # if automated mode, just go forward ? Probably can input custom calls here, or no call in automated mode (give up control)
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            # else:
                # # Stop
                # self.twist.linear.x = 0.0
                # self.twist.angular.z = 0.0 # TODO

        # Publish
        self.publisher.publish(self.twist)
        self.get_logger().debug(f"Published: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}")

def main():
    rclpy.init()
    node = JoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: # Graceful shutdown
        stop_twist = Twist()
        node.publisher.publish(stop_twist)
        node.get_logger().info("Stopped robot on shutdown")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()