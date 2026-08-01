from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Laser z is measured to the scan window, not the sensor body.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '--x', '0.0733',
                '--y', '0.0',
                '--z', '0.1135',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '3.141592653589793',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=[
                '--x', '0.1233',
                '--y', '-0.0025',
                '--z', '0.1060',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),
    ])
