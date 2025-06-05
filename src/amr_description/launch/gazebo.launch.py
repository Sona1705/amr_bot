import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration , PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package share directory for amr_description
    amr_description_dir = get_package_share_directory("amr_description")
    ros_distro = os.environ.get("ROS_DISTRO", "humble")  
    is_ignition = LaunchConfiguration("is_ignition", default="true")  

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(amr_description_dir, "urdf", "amr.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
        amr_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"),"'", " + '.world'"])
    ])

    model_path = str(Path(amr_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(amr_description_dir, "models")

    is_ignition_arg = DeclareLaunchArgument(  
        name="is_ignition",
        default_value="true",
        description="Use Ignition Gazebo (true) or Gazebo Classic (false)"
    )

    # Set the robot description using the xacro file
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
        ]),
        value_type=str
    )

    # Launch robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time":True}]
    )

    # Set Gazebo environment variables for resources
    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )
    
    ignition_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=str(Path(amr_description_dir).parent.resolve())
    )

    # Include Gazebo simulation launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    # Spawn the robot entity into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "amr_bot"]
    )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        output="screen",
        arguments=["joint_state_broadcaster","amr_controller"]
    )
    
    return LaunchDescription([
        model_arg,
        world_name_arg,
        is_ignition_arg,  
        robot_state_publisher_node,
        gazebo_resource_path,
        ignition_resource_path,
        gazebo,
        gz_spawn_entity,
        controller_spawner,
        gz_ros2_bridge
    ])















































