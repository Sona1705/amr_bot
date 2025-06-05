from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_config = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'slam.yaml'
    )

    return LaunchDescription([
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_config,
                        {"use_sim_time": True}
                    ],
                    remappings=[
                        ('/scan', '/scan'),
                        ('/odom', '/amr_controller/odom')  # Update if needed
                    ]
                )
            ]
        )
    ])
