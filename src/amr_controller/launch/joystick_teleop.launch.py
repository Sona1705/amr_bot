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
        package="teleop_twist_joy",
        executable="teleop_node",
        name="joy_teleop",
        parameters=[os.path.join(
            get_package_share_directory("amr_controller"), "config", "joy_teleop.yaml"
        )],
        
        remappings=[  
             ("/cmd_vel_stamped", "/amr_controller/cmd_vel")  
       ]
    )


    return LaunchDescription([
        joy_node,
        joy_teleop
    ])
