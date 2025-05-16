import rclpy
from rclpy.node import Node
import pandas as pd
from geometry_msgs.msg import Point

class CSVWayPointPublisher(Node):
    def __init__(self):
        super().__init__('excel_waypoint_publisher')

        # Declare parameters with default values (change parameters here)
        # change name of the file from goal_coordinates.csv to <insert kieran's csv file here>
        # self.declare_parameter('file_path', '/home/team5/ROS2-Project/src/autonomous_nav/autonomous_nav/goal_coordinates.csv')
        self.declare_parameter('file_path', '/root/ROS2-Project/src/autonomous_nav/autonomous_nav/goal_coordinates.csv')
        self.declare_parameter('topic_name', '/goal_point')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('loop_waypoints', False)

        # Get parameter values
        file_path = self.get_parameter('file_path').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.loop_waypoints = self.get_parameter('loop_waypoints').get_parameter_value().bool_value 

        # Read the CSV file into a list of (x, y) tuples + error handling
        try:
            df = pd.read_csv(file_path, usecols=[0, 1], header=None)
            if df.empty:
                self.get_logger().error("CSV file is empty")
                raise ValueError("CSV file is empty")
            if not df.iloc[:, [0, 1]].apply(pd.to_numeric, errors='coerce').notnull().all().all():
                self.get_logger().error("CSV contains non-numeric or missing values")
                raise ValueError("Invalid data in CSV")
            self.waypoints = list(zip(df.iloc[:, 0].astype(float), df.iloc[:, 1].astype(float)))
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
            if self.waypoints:
                self.get_logger().info(f"First waypoint: {self.waypoints[0]}")
        except FileNotFoundError:
            self.get_logger().warn(f"CSV file not found: {file_path}. Using default waypoint (0, 0)")
            self.waypoints = [(0.0, 0.0)]
        except Exception as e:
            self.get_logger().warn(f"Failed to read CSV file: {str(e)}. Using default waypoint (0, 0)")
            self.waypoints = [(0.0, 0.0)]

        # Create publisher for geometry_msgs/Point
        self.pub = self.create_publisher(Point, topic_name, 10) 
        self._idx = 0

        # Create timer to publish waypoints at the specified rate
        self.create_timer(publish_rate, self._publish_next_waypoint)

    def _publish_next_waypoint(self):
        if self._idx < len(self.waypoints):
            point = Point()
            point.x, point.y = self.waypoints[self._idx]
            point.z = 0.0
            self.pub.publish(point)
            self.get_logger().info(f"Published waypoint {self._idx}: ({point.x}, {point.y})")
            self._idx += 1
        elif self.loop_waypoints:
            self._idx = 0
        else:
            self.get_logger().info("All waypoints published. Stopping publisher.")

def main(args=None):
    rclpy.init(args=args)
    node = CSVWayPointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
