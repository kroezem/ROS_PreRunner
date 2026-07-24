# map.launch.py, localize.launch.py, and teleop.launch.py are mutually exclusive.
# Each is a complete runnable entry point; run exactly one.
# Running more than one may duplicate UART sensor or PWM motor ownership.
# Internal tiers under launch/include are not standalone production entry points.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    launch_dir = os.path.join(package_share, 'launch')
    include_dir = os.path.join(launch_dir, 'include')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'sensors.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'estimation.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'slam_map.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop.launch.py'))),
    ])
