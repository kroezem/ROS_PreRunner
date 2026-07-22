# drive.launch.py, map.launch.py, full.launch.py, and localize.launch.py are
# mutually exclusive. Running two at once double-instantiates the sensor drivers
# and corrupts both serial ports.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('runner_bringup'),
        'launch',
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'sensors.launch.py'))),
        Node(
            package='runner_encoder',
            executable='encoder_node',
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rf2o.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'ekf.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_localize.launch.py'))),
    ])
