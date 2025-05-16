#!/usr/bin/env python
import math
import csv
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from collections import deque
from std_msgs.msg import Bool, String, Float32, Float64
from sensor_msgs.msg import NavSatFix, LaserScan, Imu
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
import time



class GPSWaypointDriver(Node):
    def __init__(self):
        super().__init__('gps_waypoint_driver')

        # —— parameters & state ——
        self.start_position = None
        self.last_arrival = None
        self.autonomous_enabled = False
        self.fake_gps = False
        self.current_position = None      # raw lat/lon
        self.heading = None               # radians
        self.current_pose = None          # (x, y, heading)
        self.current_index = 0
        self.switch = None
        self.front_measurements = deque(maxlen=100)
        self.journey_log = [] # List for dicts, one dict for one point.
        self.wall_dist_desired = 1
        self.call_camera_state = False  # current state (True if camera is on)

        self.fake_position = (-31.980640, 115.817212)
        self.waypoints = []
        self._read_waypoints_from_csv()

        self.left = self.right = self.front = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.call_camera_pub = self.create_publisher(Bool, '/call_camera', 10)
        self.summary_pub = self.create_publisher(String, '/journey_summary', 10)

        # DistBug parameters
        self.follow_direction = 'right'
        self.wall_dist_desired = 0.5
        self.max_angular_speed = 1.0
        self.current_pose = None  # (x, y, theta) in meters and radians
        self.start_pose = None  # (x, y) in meters
        self.goal_pose = None   # (x, y) in meters
        self.hit_point = None   # (lat, lon) on first obstacle contact
        self.position_buffer = []
        self.buffer_size = 20
        self.stuck_threshold = 0.05
        self.ref_latlon = self.fake_position  # Reference for lat/lon to x,y conversion

        # for m-line / wall-following (unused in this snippet)
        self.is_wall_following = False
        self.threshold = 1.0

        # ref for flat-earth conversion
        self.ref_latlon = self.fake_position

        # subscriptions
        self.create_subscription(String,    '/robot_mode',   self.mode_callback,    10)
        self.create_subscription(NavSatFix, '/gps/fix',      self.gps_callback,     10)
        self.create_subscription(Float32,   '/imu/data',     self.heading_callback, 10)
        self.create_subscription(LaserScan, '/scan',         self.lidar_callback,   10)
        self.create_subscription(Imu,       '/phidgets_spatial/spatial',
                                                       self.spatial_callback, 10)
        self.create_subscription(Float64,   '/psd/right',    self.right_callback,   10)
        self.create_subscription(Float64,   '/psd/left',     self.left_callback,    10)
        self.create_subscription(Float64,   '/psd/front',    self.front_callback,   10)
        self.create_subscription(Bool, '/dead_switch', self.dead_switch_callback, 10)
        # timer
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ Waypoint driver node started")

    def dead_switch_callback(self, msg: Bool):
        self.switch = msg.data

    def _read_waypoints_from_csv(self):
        # Read waypoints from CSV file and update self.waypoints.
        file_path = '/root/ROS2-Project/src/autonomous_nav/autonomous_nav/goal_coordinates.csv'
        try:
            waypoints = []
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                for row in reader:
                    if len(row) != 2:
                        self.get_logger().error("CSV row does not contain exactly two values")
                        raise ValueError("Invalid CSV format")
                    try:
                        lat = float(row[0])
                        lon = float(row[1])
                        waypoints.append((lat, lon))
                    except ValueError:
                        self.get_logger().error("CSV contains non-numeric values")
                        raise ValueError("Non-numeric data in CSV")
                if not waypoints:
                    self.get_logger().error("CSV file is empty")
                    raise ValueError("CSV file is empty")
            self.waypoints = waypoints
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from CSV")
            if self.waypoints:
                self.get_logger().info(f"First waypoint: {self.waypoints[0]}")
        except FileNotFoundError:
            self.get_logger().warn(f"CSV file not found: {file_path}. Keeping default waypoints")
        except Exception as e:
            self.get_logger().warn(f"Failed to read CSV file: {str(e)}. Keeping default waypoints")


    def mode_callback(self, msg: String):
        self.autonomous_enabled = (msg.data == 'automated')
        state = "ENABLED" if self.autonomous_enabled else "DISABLED"
        self.get_logger().info(f"🧭 Autonomous driving {state}")
        if self.autonomous_enabled and self.current_position is not None and self.current_index < len(self.waypoints):
            # Inline lat/lon to x,y conversion for start_pose and goal_pose
            R = 6371000.0  # Earth's radius in meters
            lat_ref, lon_ref = math.radians(self.ref_latlon[0]), math.radians(self.ref_latlon[1])
            # Start pose from current_position
            lat, lon = math.radians(self.current_position[0]), math.radians(self.current_position[1])
            x = R * (lon - lon_ref) * math.cos(lat_ref)
            y = R * (lat - lat_ref)
            self.start_pose = (x, y)
            # Goal pose from waypoints
            goal_lat, goal_lon = math.radians(self.waypoints[self.current_index][0]), math.radians(self.waypoints[self.current_index][1])
            x = R * (goal_lon - lon_ref) * math.cos(lat_ref)
            y = R * (goal_lat - lat_ref)
            self.goal_pose = (x, y)
            self.get_logger().info(f"Initialized start_pose: {self.start_pose}, goal_pose: {self.goal_pose}")

    def spatial_callback(self, msg: Imu):
        q = msg.orientation
        _, _, raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # convert to compass heading (0 = North, CW positive)
        self.heading = (math.pi/2 - raw_yaw + math.pi) % (2 * math.pi)

    def heading_callback(self, msg: Float32):
        # if you prefer the simpler Float32 yaw topic
        rad = math.radians(msg.data) % (2 * math.pi)
        self.heading = rad

    def gps_callback(self, msg: NavSatFix):
        # 1) store raw lat/lon
        self.current_position = (msg.latitude, msg.longitude)

        if self.start_position is None:
            self.start_position = (msg.latitude, msg.longitude)
            self.waypoints.append(self.start_position)
            self.get_logger().info(f"+++++++++++++++++++++++Starting at {self.start_position}+++++++++++++++++++++++++")

        # 2) flat-earth conversion → (x east, y north)
        R = 6371000.0
        lat_ref = math.radians(self.ref_latlon[0])
        lon_ref = math.radians(self.ref_latlon[1])
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        dy = (lat - lat_ref) * R
        dx = (lon - lon_ref) * R * math.cos(lat_ref)

        # 3) only set pose if we already have heading
        if self.heading is not None:
            self.current_pose = (dx, dy, self.heading)

        # 4) log right away on each new fix
        idx = self.current_index
        glat, glon = self.waypoints[idx]
        self._log_local_error(msg.latitude, msg.longitude, glat, glon, idx)

    def lidar_callback(self, msg: LaserScan):
        self.lidar = msg

    def left_callback(self, msg: Float64):
        self.left = msg.data

    def right_callback(self, msg: Float64):
        self.right = msg.data

    def front_callback(self, msg: Float64):
        self.front = msg.data
        current_time = time.time()
        self.front_measurements.append((current_time, self.front))

    def _haversine(self, p1, p2):
        lat1, lon1 = p1
        lat2, lon2 = p2
        R = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat/2)**2 +
             math.cos(math.radians(lat1)) *
             math.cos(math.radians(lat2)) *
             math.sin(dlon/2)**2)
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

    def _log_local_error(self, lat, lon, glat, glon, idx):
        """East/North offsets and range to waypoint."""
        lat0, lat1 = math.radians(lat), math.radians(glat)
        lon0, lon1 = math.radians(lon), math.radians(glon)
        R = 6371000.0
        dy = (lat1 - lat0) * R
        dx = (lon1 - lon0) * R * math.cos((lat0+lat1)/2)
        dist = math.hypot(dx, dy)
        self.get_logger().info(
            f"🔢 Waypoint {idx} local error: dx={dx:.1f} m, "
            f"dy={dy:.1f} m (r={dist:.1f} m)"
        )

    def _spin_to_align(self, delta, idx):
        self.get_logger().info(
            f"⏩ Waypoint {idx}, turning Δ={math.degrees(delta):.1f}°"
        )
        t = Twist()
        t.angular.z = -max(-1.0, min(1.0, delta))
        self.cmd_pub.publish(t)

    def _drive_forward(self):
        t = Twist()
        if self.obstacle_detect():
            t.linear.x = 0.0
        else:
            t.linear.x = 1.0
        idx = self.current_index
        wp  = self.waypoints[idx]
        dist = self._haversine(self.current_position, wp)
        self.get_logger().info(
            f"🚗 Driving forward to waypoint {idx}: "
            f"{wp}, dist={dist:.2f} m"
        )
        self.cmd_pub.publish(t)


    def obstacle_detect(self):
        # use ROS time so it works under simulation / bag playback
        now = self.get_clock().now().nanoseconds * 1e-9
        one_second_ago = now - 1.0

        # pull any front‐PSD readings from the last second
        recent = [d for t, d in self.front_measurements if t >= one_second_ago]

        if not recent:
            self.get_logger().warn("No front PSD measurements available; assuming obstacle present")
            return True

        avg_dist = sum(recent) / len(recent)
        self.get_logger().info(f"Average front distance (1s window): {avg_dist:.2f} m")

        if avg_dist < self.threshold:  # obstacle very close
            self.get_logger().warn(f"🚧 Obstacle detected! front avg distance = {avg_dist:.2f} m")
            return True
        else:
            return False

    def distance_to_m_line(self, x, y):
        """Calculate perpendicular distance to m-line (start to goal)."""
        if self.start_pose is None or self.goal_pose is None:
            self.get_logger().warn('start_pose or goal_pose not set')
            # return float('inf')
        gx, gy = self.goal_pose
        sx, sy = self.start_pose
        a = gy - sy
        b = sx - gx
        c = gx * sy - gy * sx
        return abs(a * x + b * y + c) / math.sqrt(a ** 2 + b ** 2)  # if a**2 + b**2 > 0 else float('inf')

    def can_leave_boundary(self) -> bool:
        """Check if m-line condition is met to stop wall-following."""
        if self.current_pose is None or self.goal_pose is None or self.hit_point is None:
            self.get_logger().warn('current_pose, goal_pose, or hit_point not set')
            return False
        x, y, _ = self.current_pose
        gx, gy = self.goal_pose
        d_now = math.hypot(gx - x, gy - y)
        # Convert hit_point (lat, lon) to x, y
        R = 6371000.0
        lat_ref, lon_ref = math.radians(self.ref_latlon[0]), math.radians(self.ref_latlon[1])
        lat, lon = math.radians(self.hit_point[0]), math.radians(self.hit_point[1])
        hx = R * (lon - lon_ref) * math.cos(lat_ref)
        hy = R * (lat - lat_ref)
        d_hit = math.hypot(gx - hx, gy - hy)
        m_line_dist = self.distance_to_m_line(x, y)
        return m_line_dist < 0.2 and abs(d_now - d_hit) < 0.3

    def check_stuck(self):
        """Detect if robot is stuck by checking position history."""
        if self.current_pose is None:
            self.get_logger().warn('current_pose not set')
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


    def control_loop(self):
        if not self.autonomous_enabled:
            return
        elif not self.switch:
            self.get_logger().info('111111111111')
            return

        # optional fake‐GPS swap
        if self.fake_gps:
            self.current_position = self.fake_position

        # wait until we have both lat/lon *and* heading
        if self.current_position is None or self.heading is None:
            self.get_logger().warn('No IMU or GPS data.')
            return

        # all done?
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("All waypoints reached!!!!!!!!!!!!!!!!!!!!!!!!")
            total_dist, elapsed_s = self._print_summary()
            # build one big multi‐line string…
            summary = [
                "Journey complete!",
                f" • total distance: {total_dist:.1f} m",
                f" • total time    : {elapsed_s:.1f} s",
            ]
            for leg in self.journey_log:
                summary.append(
                    f"   – leg {leg['wp']}: {leg['leg_dist']:.1f} m @ {leg['time'].sec}.{leg['time'].nanosec:09d}")
            msg = String()
            msg.data = "\n".join(summary)
            self.summary_pub.publish(msg)
            return

        lat, lon = self.current_position
        glat, glon = self.waypoints[self.current_index]

        # 1) distance & local‐error log
        dist = self._haversine((lat, lon), (glat, glon))

        # 2) bearing (0=N, CW positive)
        dlon = math.radians(glon - lon)
        y = math.sin(dlon) * math.cos(math.radians(glat))
        x = (math.cos(math.radians(lat)) * math.sin(math.radians(glat))
             - math.sin(math.radians(lat)) * math.cos(math.radians(glat)) * math.cos(dlon))
        bearing = math.atan2(y, x) % (2*math.pi)

        # 3) signed error ∈ [–π,π]
        delta = math.atan2(math.sin(bearing - self.heading),
                            math.cos(bearing - self.heading))

        self.call_camera_state = False

        # 4) reached?
        if dist < 1.0:
            self.get_logger().info(
                f"✅ Reached waypoint {self.current_index}: {self.waypoints[self.current_index]}"
            )
            arrival_t = self.get_clock().now()
            prev_wp = self.waypoints[self.current_index - 1] if self.current_index > 0 else self.start_position
            curr_wp = self.waypoints[self.current_index]

            leg_dist = self._haversine(prev_wp, curr_wp)

            if self.last_arrival is None:
                leg_dt = 0.0
            else:

                leg_dt = (arrival_t.nanosec + arrival_t.sec * 1e9
                          - (self.last_arrival.nanosec + self.last_arrival.sec * 1e9)) * 1e-9

            self.journey_log.append({
                'wp': self.current_index - 1,
                'leg_dist': leg_dist,
                'leg_time': leg_dt,
            })
            self.last_arrival = arrival_t

            if not self.call_camera_state:
                self.call_camera_pub.publish(Bool(data=True))
                self.call_camera_state = True
                self.get_logger().info("📷 Camera enabled after reaching waypoint.")

            self.current_index += 1
            return

        # 5) need to spin?
        if abs(delta) > math.radians(5.0):
            self.get_logger().info("Turning towards goal!")
            return self._spin_to_align(delta, self.current_index)

        self._drive_forward()
        # if self.is_wall_following:
        #     # Follow wall function
        #     self.follow_wall()
        #     if self.can_leave_boundary():
        #         self.is_wall_following = False
        # elif self.obstacle_detect():
        #     self.is_wall_following = True
        #     # Turn left then
        #     self.follow_wall()
        # else:
        #     self._drive_forward()


        # 6) otherwise we’re roughly aligned & still out of range → proceed with  _drive_forward / wall-follow
        # if not self.is_wall_following and self.front and self.front.data > self.threshold:
        #     self._drive_forward()
        # elif not self.is_wall_following:
        #     self.is_wall_following = True
        #     self.hit_point = self.current_position
        #     self.get_logger().info("🚧 Starting wall-following")
        # else:
        #     self.follow_wall()
        #     if self.can_leave_boundary():
        #         self.get_logger().info("🏁 Back to m-line; resuming goal-seeking")
        #         self.is_wall_following = False
        #         self.hit_point = None
        #     self.check_stuck()

    def _print_summary(self):
        total_dist = sum(entry['leg_dist'] for entry in self.journey_log)
        start = self.journey_log[0]['time']
        end = self.journey_log[-1]['time']
        elapsed = (end.nanosec + end.sec * 1e9) - (start.nanosec + start.sec * 1e9) # TODO
        elapsed_s = elapsed * 1e-9

        self.get_logger().info(" Journey complete!")
        self.get_logger().info(f" • total time  : {elapsed_s:.1f}s")
        self.get_logger().info(f" • total distance: {total_dist:.1f}m")
        for e in self.journey_log:
            t = e['time']
            self.get_logger().info(f"   – leg {e['wp']} : {e['leg_dist']:.1f}m @ {t.sec}.{t.nanosec:09d}")
        return total_dist, elapsed_s



def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# !/usr/bin/env python
# import math
# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from std_msgs.msg import Bool, String, Float32, Float64
# from sensor_msgs.msg import NavSatFix, LaserScan, Imu
# from geometry_msgs.msg import Twist
# from tf_transformations import euler_from_quaternion
#
# class GPSWaypointDriver(Node):
#     def __init__(self):
#         super().__init__('gps_waypoint_driver')
#
#         # —— parameters & state ——
#         self.start_position = None
#         self.autonomous_enabled = False
#         self.fake_gps = False
#         self.current_position = None      # raw lat/lon
#         self.heading = None               # radians
#         self.current_pose = None          # (x, y, heading)
#         self.current_index = 0
#         self.switch = None
#         self.journey_log = [] # List for dicts, one dict for one point.
#
#         self.fake_position = (-31.980640, 115.817212)
#         self.waypoints = [
#             (-31.980074, 115.817477),  # Point 1
#             (-31.980051, 115.817661),  # Point 2
#             (-31.980157, 115.817718),  # Point 3
#             (-31.980188, 115.817531)  # Point 4
#         ]
#
#         self.left = self.right = self.front = None
#         self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
#
#         # for m-line / wall-following (unused in this snippet)
#         self.is_wall_following = False
#         self.threshold = 1.0
#
#         # ref for flat-earth conversion
#         self.ref_latlon = self.fake_position
#
#         # subscriptions
#         self.create_subscription(String,    '/robot_mode',   self.mode_callback,    10)
#         self.create_subscription(NavSatFix, '/gps/fix',      self.gps_callback,     10)
#         self.create_subscription(Float32,   '/imu/data',     self.heading_callback, 10)
#         self.create_subscription(LaserScan, '/scan',         self.lidar_callback,   10)
#         self.create_subscription(Imu,       '/phidgets_spatial/spatial',
#                                                        self.spatial_callback, 10)
#         self.create_subscription(Float64,   '/psd/right',    self.right_callback,   10)
#         self.create_subscription(Float64,   '/psd/left',     self.left_callback,    10)
#         self.create_subscription(Float64,   '/psd/front',    self.front_callback,   10)
#         self.create_subscription(Bool, '/dead_switch', self.dead_switch_callback, 10)
#         # timer
#         self.create_timer(0.1, self.control_loop)
#         self.get_logger().info("✅ Waypoint driver node started")
#
#     def dead_switch_callback(self, msg: Bool):
#         self.switch = msg.data
#
#     def mode_callback(self, msg: String):
#         self.autonomous_enabled = (msg.data == 'automated')
#         state = "ENABLED" if self.autonomous_enabled else "DISABLED"
#         self.get_logger().info(f"🧭 Autonomous driving {state}")
#
#     def spatial_callback(self, msg: Imu):
#         q = msg.orientation
#         _, _, raw_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
#         # convert to compass heading (0 = North, CW positive)
#         self.heading = (math.pi/2 - raw_yaw + math.pi) % (2 * math.pi)
#         # self.get_logger().info(
#         #     f"🔧 IMU raw yaw: {math.degrees(raw_yaw):.1f}°, "
#         #     f"compass heading: {math.degrees(self.heading):.1f}°"
#         # )
#
#     def heading_callback(self, msg: Float32):
#         # if you prefer the simpler Float32 yaw topic
#         rad = math.radians(msg.data) % (2 * math.pi)
#         self.heading = rad
#
#     def gps_callback(self, msg: NavSatFix):
#         # 1) store raw lat/lon
#         self.current_position = (msg.latitude, msg.longitude)
#
#         if self.start_position is None:
#             self.start_position = (msg.latitude, msg.longitude)
#             self.waypoints.append(self.start_position)
#             self.get_logger().info(f"+++++++++++++++++++++++Starting at {self.start_position}+++++++++++++++++++++++++")
#
#         # 2) flat-earth conversion → (x east, y north)
#         R = 6371000.0
#         lat_ref = math.radians(self.ref_latlon[0])
#         lon_ref = math.radians(self.ref_latlon[1])
#         lat = math.radians(msg.latitude)
#         lon = math.radians(msg.longitude)
#         dy = (lat - lat_ref) * R
#         dx = (lon - lon_ref) * R * math.cos(lat_ref)
#
#         # 3) only set pose if we already have heading
#         if self.heading is not None:
#             self.current_pose = (dx, dy, self.heading)
#
#         # 4) log right away on each new fix
#         idx = self.current_index
#         glat, glon = self.waypoints[idx]
#         self._log_local_error(msg.latitude, msg.longitude, glat, glon, idx)
#
#     def lidar_callback(self, msg: LaserScan):
#         self.lidar = msg
#
#     def left_callback(self, msg: Float64):
#         self.left = msg.data
#
#     def right_callback(self, msg: Float64):
#         self.right = msg.data
#
#     def front_callback(self, msg: Float64):
#         self.front = msg.data
#
#     def _haversine(self, p1, p2):
#         lat1, lon1 = p1
#         lat2, lon2 = p2
#         R = 6371000.0
#         dlat = math.radians(lat2 - lat1)
#         dlon = math.radians(lon2 - lon1)
#         a = (math.sin(dlat/2)**2 +
#              math.cos(math.radians(lat1)) *
#              math.cos(math.radians(lat2)) *
#              math.sin(dlon/2)**2)
#         return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
#
#     def _log_local_error(self, lat, lon, glat, glon, idx):
#         """East/North offsets and range to waypoint."""
#         lat0, lat1 = math.radians(lat), math.radians(glat)
#         lon0, lon1 = math.radians(lon), math.radians(glon)
#         R = 6371000.0
#         dy = (lat1 - lat0) * R
#         dx = (lon1 - lon0) * R * math.cos((lat0+lat1)/2)
#         dist = math.hypot(dx, dy)
#         self.get_logger().info(
#             f"🔢 Waypoint {idx} local error: dx={dx:.1f} m, "
#             f"dy={dy:.1f} m (r={dist:.1f} m)"
#         )
#
#     def _spin_to_align(self, delta, idx):
#         self.get_logger().info(
#             f"⏩ Waypoint {idx}, turning Δ={math.degrees(delta):.1f}°"
#         )
#         t = Twist()
#         t.angular.z = -max(-1.0, min(1.0, delta))
#         self.cmd_pub.publish(t)
#
#     def _drive_forward(self):
#         t = Twist()
#         t.linear.x = 1.0
#         idx = self.current_index
#         wp  = self.waypoints[idx]
#         dist = self._haversine(self.current_position, wp)
#         self.get_logger().info(
#             f"🚗 Driving forward to waypoint {idx}: "
#             f"{wp}, dist={dist:.2f} m"
#         )
#         self.cmd_pub.publish(t)
#
#     def can_leave_boundary(self) -> bool:
#         if (self.current_pose is None
#             or self.goal_pose  is None
#             or self.hit_point  is None):
#             self.get_logger().warn(
#                 "can_leave_boundary: missing current_pose/goal_pose/hit_point"
#             )
#             return False
#         # … rest unchanged …
#
#     def control_loop(self):
#         if not self.autonomous_enabled:
#             return
#         elif not self.switch:
#             return
#
#         # optional fake‐GPS swap
#         if self.fake_gps:
#             self.current_position = self.fake_position
#
#         # wait until we have both lat/lon *and* heading
#         if self.current_position is None or self.heading is None:
#             return
#
#         # all done?
#         if self.current_index >= len(self.waypoints):
#             self.get_logger().info("🎉 All waypoints reached.")
#             return
#
#         lat, lon = self.current_position
#         glat, glon = self.waypoints[self.current_index]
#
#         # 1) distance & local‐error log
#         dist = self._haversine((lat, lon), (glat, glon))
#
#         # 2) bearing (0=N, CW positive)
#         dlon = math.radians(glon - lon)
#         y = math.sin(dlon) * math.cos(math.radians(glat))
#         x = (math.cos(math.radians(lat)) * math.sin(math.radians(glat))
#              - math.sin(math.radians(lat)) * math.cos(math.radians(glat)) * math.cos(dlon))
#         bearing = math.atan2(y, x) % (2*math.pi)
#
#         # 3) signed error ∈ [–π,π]
#         delta = math.atan2(math.sin(bearing - self.heading),
#                             math.cos(bearing - self.heading))
#
#         # 4) reached?
#         if dist < 1.5:
#             self.get_logger().info(
#                 f"✅ Reached waypoint {self.current_index}: {self.waypoints[self.current_index]}"
#             )
#             self.current_index += 1
#             arrival_t = self.get_clock().now()
#             prev_wp = self.waypoints[self.current_index - 1]
#             curr_wp = self.start_position if self.current_index == len(self.waypoints) else self.waypoints[self.current_index]
#             leg_dist = self._haversine(prev_wp, curr_wp)
#             self.journey_log.append({
#                 'wp': self.current_index - 1,
#                 'time': arrival_t,
#                 'leg_dist': leg_dist,
#             })
#             return
#
#         # 5) need to spin?
#         if abs(delta) > math.radians(5.0):
#             return self._spin_to_align(delta, self.current_index)
#
#         self._drive_forward()
#         # 6) otherwise we’re roughly aligned & still out of range → proceed with  _drive_forward / wall-follow
#         # if not self.is_wall_following and self.front and self.front.data > self.threshold:
#         #     self._drive_forward()
#         # elif not self.is_wall_following:
#         #     self.is_wall_following = True
#         #     self.hit_point = self.current_position
#         #     self.get_logger().info("🚧 Starting wall-following")
#         # else:
#         #     self.follow_wall()
#         #     if self.can_leave_boundary():
#         #         self.get_logger().info("🏁 Back to m-line; resuming goal-seeking")
#         #         self.is_wall_following = False
#         #         self.hit_point = None
#         #     self.check_stuck()
#     # def control_loop(self):
#     #     if not self.autonomous_enabled:
#     #         return
#     #
#     #     # optional fake-GPS override
#     #     if self.fake_gps:
#     #         self.current_position = self.fake_position
#     #
#     #     # wait until we have both lat/lon and a valid heading
#     #     if self.current_position is None or self.heading is None:
#     #         return
#     #
#     #     # if we’ve run out of waypoints, we’re done
#     #     if self.current_index >= len(self.waypoints):
#     #         self.get_logger().info("🎉 All waypoints reached.")
#     #         return
#     #
#     #     lat, lon   = self.current_position
#     #     glat, glon = self.waypoints[self.current_index]
#     #
#     #     # compute straight-line distance
#     #     dist = self._haversine((lat, lon), (glat, glon))
#     #
#     #     # if within threshold, notify and advance index
#     #     if dist < 1.5:
#     #         self.get_logger().info(
#     #             f"✅ Reached waypoint {self.current_index}: {self.waypoints[self.current_index]}"
#     #         )
#     #         self.current_index += 1
#     #         return
#     #
#     #     # otherwise—no driving or spinning, just log how far you are
#     #     self.get_logger().info(
#     #         f"⏳ Carrying robot… {dist:.1f} m to waypoint {self.current_index}"
#     #     )
#     #
#     #     # ↓ any previous motion commands are now disabled:
#     #     # if abs(delta) > math.radians(5.0):
#     #     #     return self._spin_to_align(delta, self.current_index)
#     #     #
#     #     # self._drive_forward()
#
#     # def _print_summary(self):
#
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = GPSWaypointDriver()
#     rclpy.spin(node)
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()