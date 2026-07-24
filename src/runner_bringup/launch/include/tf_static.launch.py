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
                '--x', '0.132',
                '--y', '0.0',
                '--z', '0.1135',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser',
            ],
        ),
        # IMU yaw is pi because the board +X points toward the vehicle rear.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=[
                '--x', '0.082',
                '--y', '0.0025',
                '--z', '0.1060',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '3.14159',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),
    ])
