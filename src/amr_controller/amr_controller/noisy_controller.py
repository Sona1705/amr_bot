#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")

        self.declare_parameter("wheel_radius", 0.08)
        self.declare_parameter("wheel_separation", 0.322)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"✅ Using wheel radius: {self.wheel_radius_}")
        self.get_logger().info(f"✅ Using wheel separation: {self.wheel_separation_}")

        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.prev_time_ = None  # ✅ Fix: Initialize previous time

        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "amr_controller/odom_noisy", 10)

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

    def jointCallback(self, msg):
        current_time = Time.from_msg(msg.header.stamp)

        if self.prev_time_ is None:
            self.prev_time_ = current_time
            return  # ✅ Skip the first callback to avoid dt error

        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS
        if dt == 0:
            return

        # Simulate noise
        wheel_encoder_left = msg.position[1] + np.random.normal(0, 0.005)
        wheel_encoder_right = msg.position[0] + np.random.normal(0, 0.005)

        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.prev_time_ = current_time

        # Angular velocities
        fi_left = dp_left / dt
        fi_right = dp_right / dt

        # Linear & Angular velocity
        linear = (self.wheel_radius_ * (fi_right + fi_left)) / 2.0
        angular = (self.wheel_radius_ * (fi_right - fi_left)) / self.wheel_separation_

        # Update pose
        d_s = (self.wheel_radius_ * (dp_right + dp_left)) / 2.0
        d_theta = (self.wheel_radius_ * (dp_right - dp_left)) / self.wheel_separation_
        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Create quaternion
        q = quaternion_from_euler(0, 0, self.theta_)

        # Fill odometry message
        now_msg_time = self.get_clock().now().to_msg()
        self.odom_msg_.header.stamp = now_msg_time
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)

        # Publish TF
        self.transform_stamped_.header.stamp = now_msg_time
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    node = NoisyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
