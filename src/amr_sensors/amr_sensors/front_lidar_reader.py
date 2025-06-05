import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class FrontLidarReader(Node):
    def __init__(self):
        super().__init__('front_lidar_reader')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.stop_sent = False

    def lidar_callback(self, msg):
        obstacle_detected = False
        points = []

        # Define the front sector in degrees (e.g., -90° to +90°)
        front_min_angle = 0  # left limit (degrees)
        front_max_angle = 360.0   # right limit (degrees)

        current_angle = msg.angle_min  # in radians

        for r in msg.ranges:
            if not (msg.range_min < r < msg.range_max):
                current_angle += msg.angle_increment
                continue

            angle_deg = math.degrees(current_angle)

            # Only consider points in the front field of view
            if front_min_angle <= angle_deg <= front_max_angle:
                # (0, 0) is the origin at the center of the LIDAR
                x = r * math.cos(current_angle)
                y = r * math.sin(current_angle)
                points.append((x, y))

                # Check for obstacles within 500 meters
                if r <= 0.5:
                    self.get_logger().warn(
                        f"Obstacle detected at {r:.2f} m, angle {angle_deg:.2f}°, X: {x:.2f}, Y: {y:.2f}"
                    )
                    obstacle_detected = True
                    break

            current_angle += msg.angle_increment

        if points:
            self.get_logger().info("Front-sector obstacle points (in meters):")
            for (x, y) in points:
                self.get_logger().info(f"X: {x:.2f}, Y: {y:.2f}")

        # Stop robot if obstacle is detected
        if obstacle_detected and not self.stop_sent:
            self.stop_robot()
            self.stop_sent = True
        elif not obstacle_detected and self.stop_sent:
            self.stop_sent = False

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("STOP command sent to /cmd_vel")

def main(args=None):
    rclpy.init(args=args)
    node = FrontLidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
