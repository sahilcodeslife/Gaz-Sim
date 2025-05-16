#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SummaryNode(Node):
    def __init__(self):
        super().__init__('summary')
        self.summary_sub = self.create_subscription(
            String,
            '/journey_summary',
            self.summary_cb,
            10
        )
        self.summary_sub  # prevent “unused” warning
        self.get_logger().info("👀 Summary node started, waiting for journey_summary…")
        self.history = []

    def summary_cb(self, msg: String):
        # store and print the whole thing
        self.history.append(msg.data)
        self.get_logger().info("🗒️ Received journey summary:\n" + msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = SummaryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()