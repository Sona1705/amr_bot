from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('amr_controller'),
        'config',
        'slam.yaml'
    )

    return LaunchDescription([
        # Optional: static transform publisher from map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # SLAM Toolbox node with slight delay to allow lidar + TF to initialize
        TimerAction(
            period=3.0,  # seconds
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='sync_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        config_path,
                        {"use_sim_time": True}
                    ],
                    remappings=[
                        ('/scan', '/scan'),
                        ('/odom', '/amr_controller/odom')
                    ]
                )
            ]
        )
    ])
