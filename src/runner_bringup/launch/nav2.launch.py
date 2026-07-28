# nav2.launch.py is the complete forward-only Nav2 entry point.

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


MAP_DIRECTORY = '/home/matti/runner_ws/maps'
DEFAULT_MAP_NAME = 'collingwood'


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
    static_yaml = f'{map_file_name}.yaml'
    required_files = (
        f'{map_file_name}.posegraph',
        f'{map_file_name}.data',
        static_yaml,
    )
    missing_files = [path for path in required_files if not os.path.isfile(path)]

    image_name = None
    if not missing_files:
        for line in Path(static_yaml).read_text().splitlines():
            if line.strip().startswith('image:'):
                image_name = line.split(':', 1)[1].strip().strip('"\'')
                break
        if not image_name:
            raise RuntimeError(
                f"Cannot launch map '{map_name}': {static_yaml} has no image "
                'entry'
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
            f"Cannot launch map '{map_name}': missing required file(s): "
            f"{', '.join(missing_files)}"
        )

    return [
        SetLaunchConfiguration('map_file_name', map_file_name),
        SetLaunchConfiguration('static_yaml', static_yaml),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    launch_dir = os.path.join(package_share, 'launch')
    include_dir = os.path.join(launch_dir, 'include')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    navigation_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_to_pose_forward_only.xml',
    )
    route_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_through_poses_forward_only.xml',
    )
    map_file_name = LaunchConfiguration('map_file_name')
    static_yaml = LaunchConfiguration('static_yaml')
    route_file = LaunchConfiguration('route_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=DEFAULT_MAP_NAME,
            description='Basename of both map artifact sets in '
            f'{MAP_DIRECTORY}',
        ),
        DeclareLaunchArgument(
            'route_file',
            default_value='~/.ros/runner_route.json',
            description='Persistent Foxglove route file',
        ),
        OpaqueFunction(function=_configure_map),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'sensors.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'estimation.launch.py'))),
        GroupAction([
            SetRemap(src='/map', dst='/slam_map'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(include_dir, 'slam_localize.launch.py'),
                ),
                launch_arguments={
                    'map_file_name': map_file_name,
                }.items(),
            ),
        ]),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'yaml_filename': static_yaml},
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params,
                {
                    'default_nav_to_pose_bt_xml': navigation_bt,
                    'default_nav_through_poses_bt_xml': route_bt,
                },
            ],
        ),
        Node(
            package='runner_bringup',
            executable='foxglove_goal_bridge',
            name='foxglove_goal_bridge',
            output='screen',
            parameters=[{'route_file': route_file}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': True,
                'bond_timeout': 4.0,
                'node_names': [
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                ],
                'use_sim_time': False,
            }],
        ),
    ])
