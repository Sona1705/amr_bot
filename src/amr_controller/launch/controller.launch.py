import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command 
from launch_ros.actions import Node 
from launch.conditions import IfCondition
import xacro

def launch_setup(context, *args, **kwargs):

    use_python = LaunchConfiguration("use_python").perform(context)
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))

    
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
        description="Use Python-based simple controller if set to True"
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.08"
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.594"
    )

    controller_yaml = os.path.join(
        get_package_share_directory("amr_controller"),
        "config",
        "amr_controllers.yaml"
    )

    robot_description = os.path.join(
        get_package_share_directory("amr_description"),
        "urdf",
        "amr_ros2_control.xacro"
    )

    robot_description_node = Node(
        package="xacro",
        executable="xacro",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"--inorder", robot_description}]
    )
    

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml],
        output="screen"
    )

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
    
    simple_controller_spawner = Node(
        package="controller_manager",
        executable="spawner", 
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Timer to delay launching joint state broadcaster
    joint_state_timer = TimerAction(
        period=3.0, # Delay by 3 seconds
        actions=[joint_state_broadcaster_spawner]
    )

    # Timer to delay launching velocity controller
    simple_controller_timer = TimerAction(
        period=5.0, # Delay by 5 seconds
        actions=[simple_controller_spawner]
    )

    simple_controller_py = Node(
        package="amr_controller",
        executable="simple_controller.py",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation
        }],
        output="screen",
        
        condition=IfCondition(use_python)
        
    )

    return ([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        robot_description_node,
        controller_manager,
        simple_controller_py,
        joint_state_timer, 
        simple_controller_timer 
    ])

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_python", default_value="False", description="Use Python-based simple controller if set to True"),
        DeclareLaunchArgument("wheel_radius", default_value="0.08"),
        DeclareLaunchArgument("wheel_separation", default_value="0.594"),
        OpaqueFunction(function=launch_setup)
    ])
    
    

    
