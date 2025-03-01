from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the ros2_control_node for controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",  # Correct executable name
        name="controller_manager",
        output="screen",
        parameters=[{'param_file': '/home/npd/Desktop/amr_bot/install/amr_controller/share/amr_controller/config/amr_controllers.yaml'}]
    )
   
    # Spawning the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", 
            "/controller_manager"
        ],
        output="screen"
    )
   
    # Spawning the simple velocity controller
    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller", 
            "--controller-manager", 
            "/controller_manager"
        ],
        output="screen"
    )
   
    # Return the launch description with all nodes
    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner
    ])
