# map.launch.py, localize.launch.py, and teleop.launch.py are mutually exclusive.
# Each is a complete runnable entry point; run exactly one.
# Running more than one may duplicate UART sensor or PWM motor ownership.
# Internal tiers under launch/include are not standalone production entry points.

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


MAP_DIRECTORY = '/home/matti/runner_ws/maps'


def _configure_map(context):
    map_name = LaunchConfiguration('map_name').perform(context)

    if not map_name:
        raise RuntimeError(
            'map_name is required; pass map_name:=<basename>'
        )
    if '..' in map_name or '/' in map_name or '\\' in map_name:
        raise RuntimeError(
            f"Invalid map_name '{map_name}': expected a map basename without "
            'path separators or ".."'
        )

    map_file_name = os.path.join(MAP_DIRECTORY, map_name)
    static_yaml = f'{map_file_name}.yaml'
    missing_files = [path for path in (
        f'{map_file_name}.posegraph',
        f'{map_file_name}.data',
        static_yaml,
    ) if not os.path.isfile(path)]

    if not missing_files:
        image_name = None
        for line in Path(static_yaml).read_text().splitlines():
            if line.strip().startswith('image:'):
                image_name = line.split(':', 1)[1].strip().strip('"\'')
                break
        if not image_name:
            raise RuntimeError(
                f"Cannot launch localization map '{map_name}': "
                f'{static_yaml} has no image entry'
            )
        static_image = image_name
        if not os.path.isabs(static_image):
            static_image = os.path.join(
                os.path.dirname(static_yaml),
                static_image,
            )
        if not os.path.isfile(static_image):
            missing_files.append(static_image)

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
            description='Required basename of the complete map bundle in '
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
