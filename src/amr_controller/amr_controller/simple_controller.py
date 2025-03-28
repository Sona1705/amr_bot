#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
import math
from rclpy.time import Time
from rclpy.constants import S_TO_NS

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Declare and get parameters 
        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_separation", 0.322)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value  

        self.get_logger().info(f"Using wheel_radius: {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel_separation: {self.wheel_separation_}")

        # Initialize wheel positions
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Initialize robot state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0 

        # Publishers
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "/simple_velocity_controller/commands", 10)

        # Subscribers
        self.vel_sub_ = self.create_subscription(TwistStamped, "/amr_controller/cmd_vel", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, "/joint_states", self.jointCallback, 10)

        # Speed conversion matrix
        self.speed_conversion_ = np.array([
            [self.wheel_radius_/2, self.wheel_radius_/2],
            [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]
        ])
        self.get_logger().info(f"Speed conversion matrix:\n{self.speed_conversion_}")

    def velCallback(self, msg):
        """ Callback function to handle velocity commands """
        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z
        self.get_logger().info(f"Received cmd_vel: linear={linear_x}, angular={angular_z}")

        # Ignore small velocity errors
        if abs(linear_x) < 1e-3 and abs(angular_z) < 1e-3:
            self.get_logger().warn("Ignoring near-zero velocity command.")
            return

        # Compute wheel speeds 
        robot_speed = np.array([[linear_x], [angular_z]])
        wheel_speed = np.matmul(np.linalg.pinv(self.speed_conversion_), robot_speed)  # Use pseudo-inverse

        left_speed = wheel_speed[1, 0]
        right_speed = wheel_speed[0, 0]

        self.get_logger().info(f"Computed wheel speeds: Left={left_speed}, Right={right_speed}")

        # Publish wheel commands
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [right_speed, left_speed]  # Ensure correct order
        self.wheel_cmd_pub_.publish(wheel_speed_msg)
        self.get_logger().info(f"Published wheel speeds: {wheel_speed_msg.data}")

    def jointCallback(self, msg):
        """ Callback function to handle joint states """
        if not msg.position or len(msg.position) < 2:
            self.get_logger().warn("Invalid joint_states received, skipping...")
            return

        # Extract joint positions
        right_wheel_pos = msg.position[0]  # Ensure these indices match your robot's joint order
        left_wheel_pos = msg.position[1]

        # Compute change in wheel positions
        dp_left = left_wheel_pos - self.left_wheel_prev_pos_
        dp_right = right_wheel_pos - self.right_wheel_prev_pos_

        # Ignore insignificant changes (floating-point noise)
        if abs(dp_left) < 1e-6 and abs(dp_right) < 1e-6:
            self.get_logger().warn("Ignoring negligible joint state changes.")
            return

        # Compute time difference
        current_time = Time.from_msg(msg.header.stamp)
        dt_ns = (current_time - self.prev_time_).nanoseconds
        dt_s = dt_ns / S_TO_NS  # Convert nanoseconds to seconds

        if dt_s <= 0:
            self.get_logger().warn("Time delta is zero or negative, skipping computation to avoid division error.")
            return

        # Update previous positions and time
        self.left_wheel_prev_pos_ = left_wheel_pos
        self.right_wheel_prev_pos_ = right_wheel_pos
        self.prev_time_ = current_time

        # Compute wheel velocities
        fi_left = dp_left / dt_s
        fi_right = dp_right / dt_s

        # Compute robot linear and angular velocity
        linear = (self.wheel_radius_ * (fi_right + fi_left)) / 2
        angular = (self.wheel_radius_ * (fi_right - fi_left)) / self.wheel_separation_

        # Update robot position
        d_s = (self.wheel_radius_ * (dp_right + dp_left)) / 2
        d_theta = (self.wheel_radius_ * (dp_right - dp_left)) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Debugging logs
        self.get_logger().info(f"Wheel Positions: Left={left_wheel_pos}, Right={right_wheel_pos}")
        self.get_logger().info(f"Wheel Velocities: Left={fi_left}, Right={fi_right}")
        self.get_logger().info(f"Robot Pose: x={self.x_}, y={self.y_}, theta={self.theta_}")

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

