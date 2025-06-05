#!/usr/bin/env python3
import math
import rclpy 
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion

class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return 0 <= pose.x < map_info.width and 0 <= pose.y < map_info.height

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("width", 50.0)
        self.declare_parameter("height", 50.0)
        self.declare_parameter("resolution", 0.1)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scan_callback(self, scan: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, scan.header.frame_id, rclpy.time.Time())
        except LookupException:
            self.get_logger().error("Unable to transform between /odom and /base_footprint")
            return
        
        # Convert robot translation to map pose
        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return
        
        # Get robot yaw from quaternion rotation (correct source)
        quat = t.transform.rotation
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Process each laser scan beam
        for i, range_val in enumerate(scan.ranges):
            if not math.isfinite(range_val):
                continue

            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            px = range_val * math.cos(angle)
            py = range_val * math.sin(angle)
            px += t.transform.translation.x
            py += t.transform.translation.y

            beam_p = coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            cell = poseToCell(beam_p, self.map_.info)
            self.map_.data[cell] = 100

    def timer_callback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main():
    rclpy.init()
    node = MappingWithKnownPoses("mapping_with_known_poses")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
