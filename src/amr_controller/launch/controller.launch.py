import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="True"
        )
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller", 
        default_value="True"
        )  
    use_python_arg = DeclareLaunchArgument(
        "use_python", 
        default_value="False"
        )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", 
        default_value="0.08"
        )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation", 
        default_value="0.322"
        )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    controller_manager_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[os.path.join(
        get_package_share_directory("amr_controller"),
        "config",
        "amr_controllers.yaml"
    )],
    output="screen",
)
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager", 
                   "/controller_manager"],
    )

    # Main wheel controller
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amr_controller", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_simple_controller),
    )

    # Simple controller
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", "--controller-manager", "/controller_manager"]
            ),
            Node(
                package="amr_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius, "wheel_separation": wheel_separation, "use_sim_time": use_sim_time}
                ],
                condition=IfCondition(use_python),
            ),
            Node(
                package="amr_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius, "wheel_separation": wheel_separation, "use_sim_time": use_sim_time}
                ],
                condition=UnlessCondition(use_python),
            ),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_simple_controller_arg,
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        controller_manager_node, 
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
    ])














































































