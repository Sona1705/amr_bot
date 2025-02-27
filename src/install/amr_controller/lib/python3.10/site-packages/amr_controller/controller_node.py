import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def __init__(self):
        super().__init__('amr_controller_node')
        self.declare_parameter('linear_scale', 1.5)   # 1.5 m/s linear max
        self.declare_parameter('angular_scale', 3.0)  # 3 rad/s angular max
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info('Joystick Controller Node Started 🚀')

    def joy_callback(self, msg):
        twist = Twist()
        deadzone = 0.1
        twist.linear.x = 0.0 if abs(msg.axes[1]) < deadzone else msg.axes[1] * self.linear_scale
        twist.angular.z = 0.0 if abs(msg.axes[0]) < deadzone else msg.axes[0] * self.angular_scale
        self.publisher_.publish(twist)
        self.get_logger().info(f'Published cmd_vel: {twist}')

def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






















