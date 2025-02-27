from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy_config = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'joy_config.yaml'
    )

    return LaunchDescription([
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Teleop Twist Joy node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_config]
        )
    ])

