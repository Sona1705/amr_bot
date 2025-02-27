from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.08",  # 80mm radius
        description="Radius of the middle motorized wheels"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.5",  # Adjust as per AMR specs
        description="Distance between left and right wheels"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    # Package paths
    amr_description_path = FindPackageShare("amr_description")
    amr_controller_path = FindPackageShare("amr_controller")

    # Robot State Publisher node with processed xacro
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": Command([
                "xacro ",
                PathJoinSubstitution([
                    amr_description_path, "urdf", "amr.urdf.xacro"
                ])
            ])
        }],
        output="screen"
    )

    # Controller configuration file
    config_file = PathJoinSubstitution([
        amr_controller_path, "config", "amr_controllers.yaml"
    ])

    # ROS 2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},  # Ensure simulation time is considered
            config_file  # This should be properly formatted in your YAML file
        ],
        output="screen"
    )

    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        output="screen",
    )

    # Spawner for diff_drive_controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "controller_manager"],
        output="screen",
    )

    # Timer action to wait before spawning controllers
    wait_for_ros2_control = TimerAction(
        period=2.0,  # Wait for 2 seconds before spawning controllers
        actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner]
    )

    # Return LaunchDescription with delay before spawning controllers
    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_state_publisher_node,
        ros2_control_node,
        wait_for_ros2_control  # Adding the delay action here
    ])



   