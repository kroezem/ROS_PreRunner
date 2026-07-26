import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    map_file_name = LaunchConfiguration('map_file_name')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'localization_launch.py',
    )
    slam_config = os.path.join(
        get_package_share_directory('runner_bringup'),
        'config',
        'localizer_params_online_async.yaml',
    )
    configured_slam_params = RewrittenYaml(
        source_file=slam_config,
        param_rewrites={
            'map_file_name': map_file_name,
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file_name',
            description='Resolved slam_toolbox localization map path',
        ),
        GroupAction([
            SetRemap(src='/scan', dst='/scan_slam'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    'slam_params_file': configured_slam_params,
                    'use_sim_time': 'false',
                }.items(),
            ),
        ]),
    ])
