from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to controller_config.yaml
    controller_parameters = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'controller_config.yaml'
    )

    # Path to joy_config.yaml
    joy_parameters = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'joy_config.yaml'
    )

    return LaunchDescription([
        # Joystick node (joy_node)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_parameters]  #  Absolute path now
        ),
        
        # Controller node (custom node to convert joystick inputs to cmd_vel)
        Node(
            package='amr_controller',
            executable='controller_node',
            name='amr_controller_node',
            output='screen',
            parameters=[controller_parameters]
        )
    ])







