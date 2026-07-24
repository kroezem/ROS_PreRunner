from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='runner_bringup',
            executable='rf2o_scan_canonicalizer',
            name='rf2o_scan_canonicalizer',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_rf2o',
            }],
        ),
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            remappings=[('/tf', '/tf_disabled')],
            parameters=[{
                'laser_scan_topic': '/scan_rf2o',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),
    ])
