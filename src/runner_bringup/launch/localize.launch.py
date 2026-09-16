# AUTONOMY localization application tier. Local control is persistent.

import os
from pathlib import Path

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
from runner_paddock.map_session import map_directory as resolve_map_directory
from runner_paddock.map_session import MapError
from runner_paddock.map_session import POSEGRAPH_STEM
from runner_paddock.map_session import validate_bundle


MAP_DIRECTORY = '/home/matti/runner_ws/maps'


def _configure_map(context):
    map_id = LaunchConfiguration('map_name').perform(context)

    if not map_id:
        raise RuntimeError(
            'map_name is required; pass map_name:=<map id>'
        )
    try:
        validate_bundle(Path(MAP_DIRECTORY), map_id)
    except MapError as error:
        raise RuntimeError(
            f"Cannot launch localization map '{map_id}': {error}"
        ) from error

    map_dir = resolve_map_directory(Path(MAP_DIRECTORY), map_id)
    return [SetLaunchConfiguration('map_file_name', str(map_dir / POSEGRAPH_STEM))]


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    include_dir = os.path.join(package_share, 'launch', 'include')
    map_file_name = LaunchConfiguration('map_file_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            description='Required map id (directory name) under '
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
    ])
