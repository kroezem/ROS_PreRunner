#!/usr/bin/env python3
"""Report localization update metrics from one or more ROS 2 MCAP bags."""

import argparse
import math
import statistics
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRANSLATION_EPSILON_M = 1e-5
YAW_EPSILON_RAD = 1e-5

HEALTH_TOPICS = (
    '/scan',
    '/scan_rf2o',
    '/odom_rf2o',
    '/imu/data',
    '/tf',
    '/initialpose',
)


def normalize_frame(frame_id):
    return frame_id.lstrip('/')


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, percentile_value):
    if not values:
        return None

    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    rank = (len(ordered) - 1) * percentile_value / 100.0
    lower_index = math.floor(rank)
    upper_index = math.ceil(rank)
    if lower_index == upper_index:
        return ordered[lower_index]

    fraction = rank - lower_index
    return (
        ordered[lower_index] * (1.0 - fraction)
        + ordered[upper_index] * fraction
    )


def distribution(values):
    return {
        'median': statistics.median(values) if values else None,
        'p90': percentile(values, 90),
        'p95': percentile(values, 95),
        'maximum': max(values) if values else None,
    }


def format_value(value, digits=3, suffix=''):
    if value is None:
        return 'n/a'
    return f'{value:.{digits}f}{suffix}'


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id='mcap',
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in ('/tf', '/initialpose')
        if topic in topic_types
    }

    counts = {topic: 0 for topic in HEALTH_TOPICS}
    bag_start_ns = None
    bag_end_ns = None
    publications = []
    initial_poses = []

    while reader.has_next():
        topic, serialized_data, receive_ns = reader.read_next()
        if bag_start_ns is None or receive_ns < bag_start_ns:
            bag_start_ns = receive_ns
        if bag_end_ns is None or receive_ns > bag_end_ns:
            bag_end_ns = receive_ns

        if topic in counts:
            counts[topic] += 1

        if topic == '/tf':
            message = deserialize_message(serialized_data, message_types[topic])
            for transform in message.transforms:
                if (
                    normalize_frame(transform.header.frame_id) == 'map'
                    and normalize_frame(transform.child_frame_id) == 'odom'
                ):
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    publications.append({
                        'receive_ns': receive_ns,
                        'header_stamp': transform.header.stamp,
                        'x': translation.x,
                        'y': translation.y,
                        'yaw': quaternion_to_yaw(rotation),
                    })

        elif topic == '/initialpose':
            message = deserialize_message(serialized_data, message_types[topic])
            pose = message.pose.pose
            initial_poses.append({
                'receive_ns': receive_ns,
                'header_stamp': message.header.stamp,
                'frame_id': message.header.frame_id,
                'x': pose.position.x,
                'y': pose.position.y,
                'yaw': quaternion_to_yaw(pose.orientation),
            })

    if bag_start_ns is None:
        raise RuntimeError('bag contains no messages')

    duration_s = (bag_end_ns - bag_start_ns) / 1e9
    corrections = []
    previous_publication = None

    for publication in publications:
        if previous_publication is not None:
            delta_x = publication['x'] - previous_publication['x']
            delta_y = publication['y'] - previous_publication['y']
            translation_delta = math.hypot(delta_x, delta_y)
            yaw_delta = wrap_angle(
                publication['yaw'] - previous_publication['yaw']
            )
            if (
                translation_delta > TRANSLATION_EPSILON_M
                or abs(yaw_delta) > YAW_EPSILON_RAD
            ):
                corrections.append({
                    'receive_ns': publication['receive_ns'],
                    'translation_delta': translation_delta,
                    'yaw_delta': yaw_delta,
                })
        previous_publication = publication

    correction_times_s = [
        (correction['receive_ns'] - bag_start_ns) / 1e9
        for correction in corrections
    ]
    correction_gaps_s = [
        current - previous
        for previous, current in zip(
            correction_times_s,
            correction_times_s[1:],
        )
    ]
    translation_deltas_m = [
        correction['translation_delta'] for correction in corrections
    ]
    absolute_yaw_deltas_rad = [
        abs(correction['yaw_delta']) for correction in corrections
    ]

    for initial_pose in initial_poses:
        receive_ns = initial_pose['receive_ns']
        previous_correction = next(
            (
                correction
                for correction in reversed(corrections)
                if correction['receive_ns'] <= receive_ns
            ),
            None,
        )
        next_correction = next(
            (
                correction
                for correction in corrections
                if correction['receive_ns'] > receive_ns
            ),
            None,
        )
        initial_pose['relative_receive_s'] = (
            receive_ns - bag_start_ns
        ) / 1e9
        initial_pose['since_previous_correction_s'] = (
            (receive_ns - previous_correction['receive_ns']) / 1e9
            if previous_correction is not None
            else None
        )
        initial_pose['until_next_correction_s'] = (
            (next_correction['receive_ns'] - receive_ns) / 1e9
            if next_correction is not None
            else None
        )
        initial_pose['next_translation_delta'] = (
            next_correction['translation_delta']
            if next_correction is not None
            else None
        )
        initial_pose['next_yaw_delta'] = (
            next_correction['yaw_delta']
            if next_correction is not None
            else None
        )

    return {
        'path': bag_path,
        'duration_s': duration_s,
        'counts': counts,
        'publications': publications,
        'corrections': corrections,
        'correction_times_s': correction_times_s,
        'gap_distribution': distribution(correction_gaps_s),
        'translation_distribution': distribution(translation_deltas_m),
        'yaw_distribution': distribution(absolute_yaw_deltas_rad),
        'initial_poses': initial_poses,
        'bag_start_ns': bag_start_ns,
    }


def print_distribution(label, values, unit):
    print(
        f'  {label}: '
        f'median={format_value(values["median"], suffix=unit)}, '
        f'p90={format_value(values["p90"], suffix=unit)}, '
        f'p95={format_value(values["p95"], suffix=unit)}, '
        f'max={format_value(values["maximum"], suffix=unit)}'
    )


def print_bag_report(result):
    duration_s = result['duration_s']
    publications = result['publications']
    corrections = result['corrections']

    print(f'\n=== {result["path"]} ===')
    print('Time basis: bag receive time for all rates, gaps, and associations')
    print(f'Bag duration: {duration_s:.3f} s')
    print('\nTopic health:')
    print(f'  {"topic":<16} {"messages":>10} {"average rate":>15}')
    for topic in HEALTH_TOPICS:
        count = result['counts'][topic]
        rate = count / duration_s if duration_s > 0.0 else 0.0
        print(f'  {topic:<16} {count:>10d} {rate:>12.3f} Hz')

    publication_rate = (
        len(publications) / duration_s if duration_s > 0.0 else 0.0
    )
    print('\nmap -> odom publications:')
    print(f'  count: {len(publications)}')
    print(f'  rate: {publication_rate:.3f} Hz')
    if publications:
        first_s = (
            publications[0]['receive_ns'] - result['bag_start_ns']
        ) / 1e9
        last_s = (
            publications[-1]['receive_ns'] - result['bag_start_ns']
        ) / 1e9
        print(f'  first bag-relative receive time: {first_s:.3f} s')
        print(f'  last bag-relative receive time: {last_s:.3f} s')
    else:
        print('  first bag-relative receive time: n/a')
        print('  last bag-relative receive time: n/a')

    correction_rate = (
        len(corrections) / duration_s if duration_s > 0.0 else 0.0
    )
    print('\nDistinct corrections:')
    print(
        '  definition: change from the previous published transform with '
        f'translation > {TRANSLATION_EPSILON_M:g} m or '
        f'|yaw| > {YAW_EPSILON_RAD:g} rad'
    )
    print('  initial map -> odom publication counted as correction: no')
    print(f'  count: {len(corrections)}')
    print(f'  rate over bag duration: {correction_rate:.3f} Hz')
    print_distribution(
        'inter-correction gap',
        result['gap_distribution'],
        ' s',
    )
    print_distribution(
        'translation magnitude',
        result['translation_distribution'],
        ' m',
    )
    print_distribution(
        'absolute yaw magnitude',
        result['yaw_distribution'],
        ' rad',
    )

    print('\nInitial poses:')
    if not result['initial_poses']:
        print('  none')
    for index, initial_pose in enumerate(result['initial_poses'], start=1):
        stamp = initial_pose['header_stamp']
        print(f'  #{index}')
        print(
            '    bag-relative receive time: '
            f'{initial_pose["relative_receive_s"]:.3f} s'
        )
        print(f'    header stamp: {stamp.sec}.{stamp.nanosec:09d}')
        print(f'    header.frame_id: {initial_pose["frame_id"]!r}')
        print(
            f'    pose: x={initial_pose["x"]:.6f} m, '
            f'y={initial_pose["y"]:.6f} m, '
            f'yaw={initial_pose["yaw"]:.6f} rad'
        )
        print(
            '    time since previous distinct correction: '
            f'{format_value(initial_pose["since_previous_correction_s"], suffix=" s")}'
        )
        print(
            '    time until next distinct correction: '
            f'{format_value(initial_pose["until_next_correction_s"], suffix=" s")}'
        )
        print(
            '    next correction change: '
            f'translation={format_value(initial_pose["next_translation_delta"], 6, " m")}, '
            f'yaw={format_value(initial_pose["next_yaw_delta"], 6, " rad")}'
        )


def print_comparison(results):
    print('\n=== Bag comparison ===')
    headers = (
        'bag',
        'dur_s',
        'corr',
        'corr_hz',
        'gap_med',
        'gap_max',
        'trans_med',
        'trans_p95',
        'trans_max',
        'yaw_abs_max',
    )
    rows = []
    for result in results:
        rows.append((
            result['path'].name,
            f'{result["duration_s"]:.1f}',
            str(len(result['corrections'])),
            f'{len(result["corrections"]) / result["duration_s"]:.3f}',
            format_value(result['gap_distribution']['median']),
            format_value(result['gap_distribution']['maximum']),
            format_value(result['translation_distribution']['median']),
            format_value(result['translation_distribution']['p95']),
            format_value(result['translation_distribution']['maximum']),
            format_value(result['yaw_distribution']['maximum']),
        ))

    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    print(
        '  '.join(
            header.ljust(widths[index])
            for index, header in enumerate(headers)
        )
    )
    print(
        '  '.join('-' * width for width in widths)
    )
    for row in rows:
        print(
            '  '.join(
                value.ljust(widths[index])
                for index, value in enumerate(row)
            )
        )
    print(
        'Units: gaps in s, translation in m, yaw_abs_max in rad; '
        'initial transform excluded from corr.'
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Analyze slam_toolbox map -> odom corrections in MCAP bags.',
    )
    parser.add_argument(
        'bags',
        nargs='+',
        type=Path,
        metavar='BAG',
        help='ROS 2 MCAP bag directory',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    results = []

    for bag_path in args.bags:
        if not bag_path.is_dir():
            raise SystemExit(f'error: bag directory not found: {bag_path}')
        try:
            result = read_bag(bag_path)
        except Exception as error:
            raise SystemExit(f'error: failed to read {bag_path}: {error}') from error
        print_bag_report(result)
        results.append(result)

    if len(results) > 1:
        print_comparison(results)


if __name__ == '__main__':
    main()
