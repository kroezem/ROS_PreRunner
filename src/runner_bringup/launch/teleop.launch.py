# map.launch.py, localize.launch.py, and teleop.launch.py are mutually exclusive.
# Each is a complete runnable entry point; run exactly one.
# Running more than one may duplicate UART sensor or PWM motor ownership.
# Internal tiers under launch/include are not standalone production entry points.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
            }],
        ),
        Node(package='runner_motor', executable='motor_node'),
    ])
