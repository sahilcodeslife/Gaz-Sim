#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('threshold_distance', 1.0)
        self.declare_parameter('scan_topic', '/scan')

        self.threshold_distance = self.get_parameter('threshold_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value

        self.fovs = {
            'left': {'min': 30.0, 'max': 90.0, 'topic': '/psd/left'},    
            'front': {'min': -30.0, 'max': 30.0, 'topic': '/psd/front'}, 
            'right': {'min': -90.0, 'max': -30.0, 'topic': '/psd/right'}  
        }

        for region in self.fovs:
            self.fovs[region]['min_rad'] = self.fovs[region]['min'] * math.pi / 180.0
            self.fovs[region]['max_rad'] = self.fovs[region]['max'] * math.pi / 180.0

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.publisher_left = self.create_publisher(Float64, '/psd/left', 10)
        self.publisher_front = self.create_publisher(Float64, '/psd/front', 10)
        self.publisher_right = self.create_publisher(Float64, '/psd/right', 10)

        self.get_logger().info(
            f'Obstacle Detector Node started. Threshold distance: {self.threshold_distance:.2f} m'
        )

    def scan_callback(self, msg):
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min  # Typically -π/2 for SICK LiDAR (-90°)
        angle_max = msg.angle_max  # Typically +π/2 for SICK LiDAR (+90°)

        for region, config in self.fovs.items():
            start_index = max(0, int((config['min_rad'] - angle_min) / angle_increment))
            end_index = min(len(msg.ranges) - 1, int((config['max_rad'] - angle_min) / angle_increment))

            valid_ranges = [
                r for r in msg.ranges[start_index:end_index + 1]
                if math.isfinite(r)
            ]

            avg_distance = sum(valid_ranges) / len(valid_ranges) if valid_ranges else float('inf')

            distance_msg = Float64()
            distance_msg.data = avg_distance

            if region == 'left':
                self.publisher_left.publish(distance_msg)
            elif region == 'front':
                self.publisher_front.publish(distance_msg)
            elif region == 'right':
                self.publisher_right.publish(distance_msg)

            self.get_logger().info(
                f'Average distance in {region} FOV: {avg_distance:.2f}',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()