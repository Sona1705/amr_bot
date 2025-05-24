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
        front_min_angle = -180.0  # degrees
        front_max_angle = 180.0   # degrees

        current_angle = msg.angle_min

        for i, r in enumerate(msg.ranges):
            # Skip invalid range values
            if not (msg.range_min < r < msg.range_max):
                current_angle += msg.angle_increment
                continue

            angle_deg = math.degrees(current_angle)

            # Process only front 180° range
            if front_min_angle <= angle_deg <= front_max_angle:
                # Print range and angle
                print(f"Radius: {r:.2f} m, Theta: {angle_deg:.2f}°")

                # Obstacle detection threshold
                if r <= 0.5:
                    self.get_logger().warn(
                        f"Obstacle detected at {r:.2f} m, angle {angle_deg:.2f}°"
                    )
                    obstacle_detected = True
                    break

            current_angle += msg.angle_increment

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
