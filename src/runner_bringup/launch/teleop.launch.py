# map.launch.py, localize.launch.py, and teleop.launch.py are mutually exclusive.
# Each is a complete runnable entry point; run exactly one.
# Running more than one may duplicate application and sensor resources.
# Internal tiers under launch/include are not standalone production entry points.

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
            parameters=[{
                'axis_steer': 0,
                'axis_brake': 2,
                'axis_throttle': 5,
                # DualSense X is buttons[0] on the standard hid-playstation map;
                # confirm the index against /joy for the connected controller.
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
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[mux_parameters],
            remappings=[('/cmd_vel_out', '/cmd_vel')],
        ),
    ])
