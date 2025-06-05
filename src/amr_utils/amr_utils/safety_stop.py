#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        self.declare_parameter("danger_distance", 0.2)
        self.declare_parameter("warning_distance", 0.6)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.state = State.FREE
        self.prev_state = State.FREE

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, "zones", 10)

    def create_zone_marker(self, marker_id, radius, color_rgba, frame_id, stamp, z_height=0.0):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = "zones"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = z_height
        marker.pose.orientation.w = 1.0
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.01
        marker.color.r = color_rgba[0]
        marker.color.g = color_rgba[1]
        marker.color.b = color_rgba[2]
        marker.color.a = color_rgba[3]
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # forever
        return marker

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE
        self.state = State.WARNING

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                break

        # Publish safety stop only if state changed
        if self.state != self.prev_state:
            is_safety_stop = Bool()
            is_safety_stop.data = self.state == State.DANGER
            self.safety_stop_pub.publish(is_safety_stop)

        self.prev_state = self.state

        # Define colors based on current state
        if self.state == State.FREE:
            warning_alpha = 0.5
            danger_alpha = 0.0
        elif self.state == State.WARNING:
            warning_alpha = 1.0
            danger_alpha = 0.5
        else:  # DANGER
            warning_alpha = 1.0
            danger_alpha = 1.0

        # Use a fixed frame_id for markers
        fixed_frame = "laser_link"
        now = self.get_clock().now().to_msg()

        warning_zone = self.create_zone_marker(
            marker_id=0,
            radius=self.warning_distance,
            color_rgba=(1.0, 0.984, 0.0, warning_alpha),  # Yellow
            frame_id=fixed_frame,
            stamp=now,
            z_height=0.005
        )
        danger_zone = self.create_zone_marker(
            marker_id=1,
            radius=self.danger_distance,
            color_rgba=(1.0, 0.0, 0.0, danger_alpha),  # Red
            frame_id=fixed_frame,
            stamp=now,
            z_height=0.01
        )

        marker_array = MarkerArray()
        marker_array.markers.append(warning_zone)
        marker_array.markers.append(danger_zone)
        self.zones_pub.publish(marker_array)


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
