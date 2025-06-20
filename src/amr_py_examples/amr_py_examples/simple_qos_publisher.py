import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String

class SimpleQoSPublisher(Node):

    def __init__(self):
        super().__init__('simple_qos_publisher')

        self.qos_profile_pub = QoSProfile(depth=10)

        # Fixed spelling
        self.declare_parameter("reliability", "system_default")
        self.declare_parameter("durability", "system_default")

        reliability = self.get_parameter("reliability").get_parameter_value().string_value
        durability = self.get_parameter("durability").get_parameter_value().string_value

        # Reliability Policy
        if reliability == "best_effort":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.get_logger().info("[Reliability] : Best Effort")
        elif reliability == "reliable":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
            self.get_logger().info("[Reliability] : Reliable")
        elif reliability == "system_default":
            self.qos_profile_pub.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Reliability] : System Default")
        else:
            self.get_logger().error("Selected Reliability QoS: %s doesn't exist" % reliability)
            return

        # Durability Policy
        if durability == "volatile":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE
            self.get_logger().info("[Durability]: Volatile")
        elif durability == "transient_local":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.get_logger().info("[Durability]: Transient Local")
        elif durability == "system_default":
            self.qos_profile_pub.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
            self.get_logger().info("[Durability] : System Default")
        else:
            self.get_logger().error("Selected Durability QoS: %s doesn't exist" % durability)
            return

        self.publisher = self.create_publisher(String, 'chatter', self.qos_profile_pub)
        self.counter_ = 0
        self.frequency_ = 1.0  # seconds
        self.get_logger().info("Publishing at %.1f Hz" % (1.0 / self.frequency_))

        self.timer = self.create_timer(self.frequency_, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS2 - COUNTER: %d' % self.counter_
        self.publisher.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_qos_publisher = SimpleQoSPublisher()
    rclpy.spin(simple_qos_publisher)
    simple_qos_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
