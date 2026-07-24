from launch import LaunchDescription
from launch_ros.actions import Node


def rf2o_test_node(name, scan_topic, odom_topic, tf_topic):
    """Create an isolated RF2O instance for the angular-origin A/B test."""
    return Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name=name,
        output='screen',
        remappings=[('/tf', tf_topic)],
        parameters=[{
            'laser_scan_topic': scan_topic,
            'odom_topic': odom_topic,
            'publish_tf': False,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='runner_bringup',
            executable='rf2o_scan_canonicalizer',
            name='rf2o_scan_canonicalizer',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_rf2o_test',
            }],
        ),
        rf2o_test_node(
            'rf2o_raw_test',
            '/scan',
            '/odom_rf2o_raw_test',
            '/tf_disabled_raw_test',
        ),
        rf2o_test_node(
            'rf2o_shifted_test',
            '/scan_rf2o_test',
            '/odom_rf2o_shifted_test',
            '/tf_disabled_shifted_test',
        ),
    ])
