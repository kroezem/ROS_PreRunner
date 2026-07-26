import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap


def generate_launch_description():
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py',
    )
    slam_config = os.path.join(
        get_package_share_directory('runner_bringup'),
        'config',
        'mapper_params_online_async.yaml',
    )

    return LaunchDescription([
        GroupAction([
            SetRemap(src='/scan', dst='/scan_slam'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'slam_params_file': slam_config,
                    'use_sim_time': 'false',
                }.items(),
            ),
        ]),
    ])
