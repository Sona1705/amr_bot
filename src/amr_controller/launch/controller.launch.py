import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    controllers_yaml = os.path.join(
        get_package_share_directory('amr_controller'), 
        'config',
        'amr_controllers.yaml'
    )

    # Get the robot description URDF path
    robot_description = os.path.join(
        get_package_share_directory('amr_description'),
        'urdf',
        'amr_ros2_control.xacro'
    )

    # Start the robot description node
    robot_description_node = Node(
        package='xacro',
        executable='xacro',
        name='robot_description',
        output='screen',
        arguments=['--inorder', robot_description]
    )

    # Start controller manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        output="screen"
    )

    # Spawner for joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",  
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Spawner for simple velocity controller
    simple_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",  
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # Timer to delay launching joint state broadcaster
    joint_state_timer = TimerAction(
        period=3.0,  # Delay by 3 seconds
        actions=[joint_state_broadcaster_spawner]
    )

    # Timer to delay launching velocity controller
    simple_controller_timer = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[simple_controller_spawner]
    )

    return LaunchDescription([
        robot_description_node,  
        controller_manager,      
        joint_state_timer,       
        simple_controller_timer  
    ])
