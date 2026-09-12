# Persistent local-control tier: one joy_node, runner_teleop, and the
# existing twist_mux. Owned by runner-local-control.service and kept alive
# across IDLE/MAPPING/AUTONOMY. map.launch.py and localize.launch.py are
# application tiers that run alongside this launch and no longer construct
# joy/teleop/mux nodes. Engineering direct runs must not start a second copy.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mux_parameters = os.path.join(
        get_package_share_directory('runner_bringup'),
        'config',
        'twist_mux.yaml',
    )
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 20.0, 'deadzone': 0.05}],
        ),
        Node(
            package='runner_teleop',
            executable='teleop_node',
            parameters=[{
                'axis_steer': 0,
                'axis_brake': 2,
                'axis_throttle': 5,
                # DualSense X is buttons[0] on the standard hid-playstation map;
                # confirm the index against /joy for the connected controller.
                'deadman_button': 0,
                'manual_trigger_expo': 0.50,
                'fixed_throttle_initial_setpoint': 0.00,
                'fixed_throttle_step': 0.01,
                'fixed_throttle_max_setpoint': 0.50,
                'fixed_throttle_min_setpoint': 0.00,
                'controller_timeout': 0.15,
                'keyboard_state_timeout': 0.15,
            }],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_parameters],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),
    ])
