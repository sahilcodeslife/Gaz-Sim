#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import math
import tf_transformations

class ImuHeadingNode(Node):
    def __init__(self):
        super().__init__('imu_heading_node')
        # Subscribers for IMU and magnetometer data
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/imu/mag', self.mag_callback, 10)
        # Publisher for processed IMU data with heading
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        # Storage for latest sensor data
        self.latest_imu = None
        self.latest_mag = None
        # Magnetic declination (radians) for true North; adjust for your location
        self.declination = math.radians(0.0)  # Example: 10° East = 0.1745 radians
        self.get_logger().info('IMU Heading Node started, publishing to /imu/data')

    def imu_callback(self, msg):
        self.latest_imu = msg
        self.compute_and_publish_heading()

    def mag_callback(self, msg):
        self.latest_mag = msg
        self.compute_and_publish_heading()

    def compute_and_publish_heading(self):
        # Ensure we have both IMU and magnetometer data
        if self.latest_imu is None or self.latest_mag is None:
            return

        # Extract accelerometer and magnetometer data
        ax = self.latest_imu.linear_acceleration.x
        ay = self.latest_imu.linear_acceleration.y
        az = self.latest_imu.linear_acceleration.z
        mx = self.latest_mag.magnetic_field.x
        my = self.latest_mag.magnetic_field.y
        mz = self.latest_mag.magnetic_field.z

        # Compute roll and pitch from accelerometer for tilt compensation
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Tilt-compensated magnetometer readings
        mx_h = mx * math.cos(pitch) + mz * math.sin(pitch)
        my_h = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)

        # Compute heading (yaw) relative to magnetic North
        yaw = math.atan2(-my_h, mx_h) + self.declination
        yaw = (yaw + 2 * math.pi) % (2 * math.pi)  # Normalize to [0, 2π]

        # Create Imu message
        imu_msg = Imu()
        imu_msg.header = self.latest_imu.header
        imu_msg.header.frame_id = 'imu_link'  # Match Phidgets frame_id
        # Convert yaw to quaternion (roll and pitch set to 0 for simplicity)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]
        imu_msg.orientation_covariance = [0.0] * 9  # Unknown covariance
        # Copy angular velocity and linear acceleration from raw data
        imu_msg.angular_velocity = self.latest_imu.angular_velocity
        imu_msg.angular_velocity_covariance = self.latest_imu.angular_velocity_covariance
        imu_msg.linear_acceleration = self.latest_imu.linear_acceleration
        imu_msg.linear_acceleration_covariance = self.latest_imu.linear_acceleration_covariance

        # Publish the message
        self.imu_pub.publish(imu_msg)
        # Log heading in degrees for debugging
        yaw_deg = math.degrees(yaw)
        self.get_logger().info(f'Heading (relative to North): {yaw_deg:.2f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = ImuHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()