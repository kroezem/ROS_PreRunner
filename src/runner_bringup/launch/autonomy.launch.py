# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Launch the complete autonomous-driving stack exactly once."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_MAP_NAME = 'house_good_v1'


def generate_launch_description():
    """Combine Nav2 and the Stage 2 command chain without duplicate sensors."""
    bringup_share = get_package_share_directory('runner_bringup')
    adapter_share = get_package_share_directory('runner_drive_adapter')
    nav2_launch = os.path.join(
        bringup_share,
        'launch',
        'nav2.launch.py',
    )
    mux_parameters = os.path.join(
        bringup_share,
        'config',
        'twist_mux.yaml',
    )
    adapter_parameters = os.path.join(
        adapter_share,
        'config',
        'drive_adapter.yaml',
    )
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            default_value=DEFAULT_MAP_NAME,
            description='Map basename passed to the Nav2 composite',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'map_name': map_name}.items(),
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
        ),
        Node(
            package='runner_teleop',
            executable='keyboard_bridge',
            name='keyboard_bridge',
            output='screen',
            parameters=[{
                'bind_address': '0.0.0.0',
                'port': 49321,
                'allowed_source_ip': '',
                'input_timeout': 0.15,
                'speed_cap': 0.50,
                'publication_rate': 20.0,
                'autonomy_latch_timeout': 600.0,
            }],
        ),
        Node(
            package='runner_teleop',
            executable='teleop_node',
            name='runner_teleop',
            output='screen',
            parameters=[{
                'axis_steer': 0,
                'axis_brake': 2,
                'axis_throttle': 5,
                'deadman_button': 0,
                'fixed_throttle_initial_setpoint': 0.30,
                'fixed_throttle_step': 0.01,
                'fixed_throttle_max_setpoint': 0.50,
                'fixed_throttle_min_setpoint': 0.00,
                'controller_timeout': 0.15,
                'keyboard_state_timeout': 0.15,
            }],
        ),
        Node(
            package='runner_drive_adapter',
            executable='drive_adapter',
            name='drive_adapter',
            output='screen',
            parameters=[adapter_parameters],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_parameters],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),
        Node(
            package='runner_motor',
            executable='motor_node',
            name='motor_driver',
            output='screen',
            parameters=[{'esc_mode': 'race'}],
        ),
    ])
