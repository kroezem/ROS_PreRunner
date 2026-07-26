"""Publish a fixed-cardinality LaserScan for slam_toolbox."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


FULL_CIRCLE = 2.0 * math.pi


def _nearest_integer(value):
    """Round a non-negative value to its nearest integer, halves upward."""
    return math.floor(value + 0.5)


def rebin_scan(scan, bins):
    """Return a fixed-cardinality scan, or None for an invalid source scan."""
    if bins < 2:
        raise ValueError('bins must be at least 2')

    source_count = len(scan.ranges)
    if source_count < 2:
        return None

    output = LaserScan()
    output.header.stamp.sec = scan.header.stamp.sec
    output.header.stamp.nanosec = scan.header.stamp.nanosec
    output.header.frame_id = scan.header.frame_id
    output.angle_min = 0.0
    output.angle_max = FULL_CIRCLE
    output.angle_increment = FULL_CIRCLE / (bins - 1)
    output.scan_time = scan.scan_time
    output.time_increment = scan.scan_time / (bins - 1)
    output.range_min = scan.range_min
    output.range_max = scan.range_max

    if source_count == bins:
        output.ranges = list(scan.ranges)
        output.intensities = [
            scan.intensities[index]
            if index < len(scan.intensities)
            else 0.0
            for index in range(bins)
        ]
        return output

    selected_indices = [None] * bins
    selected_distances = [math.inf] * bins
    source_denominator = source_count - 1
    target_denominator = bins - 1

    for source_index in range(source_count):
        scaled_index = (
            source_index * target_denominator / source_denominator
        )
        target_index = _nearest_integer(scaled_index)
        source_angle = (
            scan.angle_min + source_index * scan.angle_increment
        )
        target_angle = target_index * output.angle_increment
        distance = abs(source_angle - target_angle)

        distance_is_tied = math.isclose(
            distance,
            selected_distances[target_index],
            rel_tol=1e-12,
            abs_tol=1e-12,
        )
        if (
            distance < selected_distances[target_index]
            and not distance_is_tied
        ):
            selected_indices[target_index] = source_index
            selected_distances[target_index] = distance

    ranges = [math.inf] * bins
    intensities = [0.0] * bins
    for target_index, source_index in enumerate(selected_indices):
        if source_index is None:
            continue
        ranges[target_index] = scan.ranges[source_index]
        if source_index < len(scan.intensities):
            intensities[target_index] = scan.intensities[source_index]

    output.ranges = ranges
    output.intensities = intensities
    return output


class ScanRebinner(Node):
    """Convert variable-cardinality LD19 scans to fixed angular bins."""

    def __init__(self):
        super().__init__('scan_rebinner')
        self.declare_parameter('bins', 503)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_slam')

        self.bins = self.get_parameter('bins').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        if self.bins < 2:
            self.get_logger().fatal(
                f'Invalid bins={self.bins}; bins must be at least 2')
            raise ValueError('bins must be at least 2')

        self.publisher = self.create_publisher(
            LaserScan, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self.scan_callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            f'Configured bins={self.bins}, input_topic={input_topic}, '
            f'output_topic={output_topic}')

    def scan_callback(self, scan):
        """Rebin and publish one scan when its source cardinality is valid."""
        output = rebin_scan(scan, self.bins)
        if output is None:
            self.get_logger().warning(
                'Rejecting scan with fewer than two ranges',
                throttle_duration_sec=5.0,
            )
            return
        self.publisher.publish(output)


def main(args=None):
    """Run the scan rebinner node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ScanRebinner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
