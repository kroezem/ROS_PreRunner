# map.launch.py, localize.launch.py, and teleop.launch.py are mutually exclusive.
# Each is a complete runnable entry point; run exactly one.
# Running more than one may duplicate UART sensor or PWM motor ownership.
# Internal tiers under launch/include are not standalone production entry points.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


MAP_DIRECTORY = '/home/matti/runner_ws/maps'
DEFAULT_MAP_NAME = 'house_good_v1'


def _configure_map(context):
    map_name = LaunchConfiguration('map_name').perform(context)

    if (
        not map_name
        or '..' in map_name
        or '/' in map_name
        or '\\' in map_name
    ):
        raise RuntimeError(
            f"Invalid map_name '{map_name}': expected a map basename without "
            'path separators or ".."'
        )

    map_file_name = os.path.join(MAP_DIRECTORY, map_name)
    missing_files = [
        path
        for path in (
            f'{map_file_name}.posegraph',
            f'{map_file_name}.data',
        )
        if not os.path.isfile(path)
    ]
    if missing_files:
        raise RuntimeError(
            f"Cannot launch localization map '{map_name}': missing required "
            f"file(s): {', '.join(missing_files)}"
        )

    return [SetLaunchConfiguration('map_file_name', map_file_name)]


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    launch_dir = os.path.join(package_share, 'launch')
    include_dir = os.path.join(launch_dir, 'include')
    map_file_name = LaunchConfiguration('map_file_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=DEFAULT_MAP_NAME,
            description='Basename of the slam_toolbox map in '
            f'{MAP_DIRECTORY}',
        ),
        OpaqueFunction(function=_configure_map),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'sensors.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'estimation.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(include_dir, 'slam_localize.launch.py'),
            ),
            launch_arguments={
                'map_file_name': map_file_name,
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'teleop.launch.py'))),
    ])
