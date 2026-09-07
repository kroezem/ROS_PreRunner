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


def generate_launch_description():
    """Launch only the AUTONOMY application tier."""
    bringup_share = get_package_share_directory('runner_bringup')
    adapter_share = get_package_share_directory('runner_drive_adapter')
    nav2_launch = os.path.join(
        bringup_share,
        'launch',
        'nav2.launch.py',
    )
    adapter_parameters = os.path.join(
        adapter_share,
        'config',
        'drive_adapter.yaml',
    )
    speed_envelope = os.path.join(
        adapter_share,
        'config',
        'speed_envelope.yaml',
    )
    map_name = LaunchConfiguration('map_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_name',
            description='Required map basename passed to the Nav2 composite',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={'map_name': map_name}.items(),
        ),
        Node(
            package='runner_drive_adapter',
            executable='drive_adapter',
            name='drive_adapter',
            output='screen',
            parameters=[adapter_parameters, speed_envelope],
        ),
        Node(
            package='runner_bringup',
            executable='speed_envelope_observer',
            name='speed_envelope_observer',
            output='screen',
            parameters=[{
                'origin_file': speed_envelope,
                'request_timeout_sec': 0.25,
            }],
        ),
    ])
