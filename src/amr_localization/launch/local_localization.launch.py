from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False"
    )

    use_python = LaunchConfiguration("use_python")

    # Transform: base_footprint_ekf -> imu_link_ekf (your existing static transform)
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf", "--child-frame-id", "imu_link_ekf"]
    )

    # NEW: Transform base_footprint -> base_footprint_ekf
    static_tf_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1",
                   "base_footprint", "base_footprint_ekf"]
    )

    # NEW: Transform imu_link -> imu_link_ekf
    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1",
                   "imu_link", "imu_link_ekf"]
    )

    # EKF node
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("amr_localization"), "config", "ekf.yaml")]
    )

    # Optional IMU republisher
    imu_republisher_py = Node(
        package="amr_localization",
        executable="imu_republisher",
        condition=IfCondition(use_python)
    )

    return LaunchDescription([
        use_python_arg,
        static_transform_publisher,
        static_tf_base,
        static_tf_imu,
        robot_localization,
        imu_republisher_py,
    ])
