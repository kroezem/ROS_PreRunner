from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='runner_imu',
            executable='bno085_node',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
