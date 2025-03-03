from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
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

    joint_state_timer = TimerAction(
        period=3.0,  # Delay joint state broadcaster by 3 seconds
        actions=[joint_state_broadcaster_spawner]
    )

    simple_controller_timer = TimerAction(
        period=5.0,  # Delay velocity controller by 5 seconds
        actions=[simple_controller_spawner]
    )

    return LaunchDescription([
        joint_state_timer,
        simple_controller_timer
    ])
