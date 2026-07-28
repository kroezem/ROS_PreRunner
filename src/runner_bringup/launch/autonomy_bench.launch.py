# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch the wheels-off-ground Stage 2 command chain.

Topic semantics are explicit: ``/cmd_vel_nav`` uses m/s and rad/s, while
``/cmd_vel_auto``, ``/cmd_vel_teleop``, and ``/cmd_vel`` carry normalized
throttle/brake and steering commands.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start one owner of every Stage 2 command-chain responsibility."""
    bringup_share = get_package_share_directory('runner_bringup')
    adapter_share = get_package_share_directory('runner_drive_adapter')
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

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
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
        Node(
            package='runner_encoder',
            executable='encoder_node',
            name='encoder_node',
            output='screen',
        ),
    ])
