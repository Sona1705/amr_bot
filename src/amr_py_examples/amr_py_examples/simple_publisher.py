import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'chatter', 10)

        self.counter_ = 0
        self.frequency_ = 1.0

        self.get_logger().info("Publishing at %d Hz" % self.frequency_)

        self.timer = self.create_timer(self.frequency_, self.timer_callback)
       
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS2 - COUNTER: %d' % self.counter_

        self.publisher.publish(msg)  # Corrected line: use self.publisher
        
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
