import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amr_description = get_package_share_directory("amr_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(amr_description, "urdf", "amr.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ]),
        launch_arguments={"verbose": "true"}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "amr_bot"],
        output="screen"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        },
        os.path.join(amr_description, "config", "amr_controllers.yaml")],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        controller_manager
    ])


























































