import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('runner_bringup')
    include_dir = os.path.join(package_share, 'launch', 'include')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'tf_static.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'lidar.launch.py'))),
        Node(
            package='runner_bringup',
            executable='scan_rebinner',
            name='scan_rebinner',
            output='screen',
            parameters=[{
                'bins': 503,
                'input_topic': '/scan',
                'output_topic': '/scan_slam',
            }],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include_dir, 'imu.launch.py'))),
    ])
