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
from runner_paddock.map_session import MAP_YAML_NAME
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
        raise RuntimeError(f"Cannot launch map '{map_id}': {error}") from error

    map_dir = resolve_map_directory(Path(MAP_DIRECTORY), map_id)
    return [
        SetLaunchConfiguration('map_file_name', str(map_dir / POSEGRAPH_STEM)),
        SetLaunchConfiguration('static_yaml', str(map_dir / MAP_YAML_NAME)),
    ]


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    launch_dir = os.path.join(package_share, 'launch')
    include_dir = os.path.join(launch_dir, 'include')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    runtime_override = os.environ.get(
        'PADDOCK_SPEED_POLICY_OVERRIDE',
        '/home/matti/.config/runner/speed_profile_overrides.yaml',
    )
    runtime_parameters = [runtime_override] if os.path.isfile(runtime_override) else []
    speed_envelope = os.path.join(
        get_package_share_directory('runner_drive_adapter'),
        'config',
        'speed_envelope.yaml',
    )
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
            parameters=[nav2_params, *runtime_parameters],
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, speed_envelope],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                nav2_params,
                *runtime_parameters,
                {
                    'default_nav_to_pose_bt_xml': navigation_bt,
                    'default_nav_through_poses_bt_xml': route_bt,
                },
            ],
        ),
        Node(
            package='runner_bringup',
            executable='navigation_runtime',
            name='runner_navigation_runtime',
            output='screen',
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
