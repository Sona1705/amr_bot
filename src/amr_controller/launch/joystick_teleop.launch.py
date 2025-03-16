from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(
            get_package_share_directory("amr_controller"), "config", "joy_config.yaml"
        )]
    )
    
    joy_teleop = Node(
        package="teleop_joy",  
        executable="teleop_joy", 
        name="joy_teleop",
        parameters=[os.path.join(
            get_package_share_directory("amr_controller"), "config", "joy_teleop.yaml"
        )],
        remappings=[  
            ("/amr_controller/cmd_vel", "/cmd_vel_stamped")
        ]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])
