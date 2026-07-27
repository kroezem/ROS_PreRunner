#!/usr/bin/env python3
"""Cross-validate integrated IMU, RF2O, EKF, and SLAM yaw from MCAP bags."""

import argparse
import bisect
import json
import math
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from analyze_localization_bag import (
    load_bag_messages,
    mcap_files_for_path,
    normalize_frame,
    quaternion_to_yaw,
    stamp_to_ns,
)


TOPICS = {
    'imu': '/imu/data',
    'rf2o': '/odom_rf2o',
    'ekf': '/odometry/filtered',
    'tf': '/tf',
}
RATE_SOURCES = ('imu', 'rf2o', 'ekf')
BIN_EDGES = (0.0, 0.3, 1.0, 2.0, math.inf)
DEFAULT_ALIGNMENT_S = 0.02
DEFAULT_MAX_GAP_S = 0.25
MIN_EVIDENCE_SAMPLES = 100
MIN_EVIDENCE_DURATION_S = 5.0
MIN_EVIDENCE_ABS_SLAM_YAW_RAD = 0.25
NEAR_ZERO_YAW_RAD = 1e-6


def wrap_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def unwrap_angles(angles):
    """Unwrap a sequence while preserving its first value."""
    if not angles:
        return []
    unwrapped = [angles[0]]
    for angle in angles[1:]:
        unwrapped.append(unwrapped[-1] + wrap_angle(angle - unwrapped[-1]))
    return unwrapped


def compose_transform(first, second):
    """Compose planar transforms represented by x, y, and yaw."""
    cosine = math.cos(first['yaw'])
    sine = math.sin(first['yaw'])
    return {
        'x': (
            first['x']
            + cosine * second['x']
            - sine * second['y']
        ),
        'y': (
            first['y']
            + sine * second['x']
            + cosine * second['y']
        ),
        'yaw': first['yaw'] + second['yaw'],
    }


def deduplicate_samples(samples):
    """Sort samples and retain the last received value at each timestamp."""
    ordered = sorted(
        samples,
        key=lambda item: (item['time_ns'], item['order']),
    )
    result = []
    for sample in ordered:
        if result and sample['time_ns'] == result[-1]['time_ns']:
            result[-1] = sample
        else:
            result.append(sample)
    return result


def interpolate_sample(samples, time_ns, max_gap_ns, times=None):
    """Linearly interpolate a scalar without spanning an excessive gap."""
    if not samples:
        return None
    if times is None:
        times = [sample['time_ns'] for sample in samples]
    index = bisect.bisect_left(times, time_ns)
    if index < len(samples) and times[index] == time_ns:
        return samples[index]['value']
    if index == 0 or index == len(samples):
        return None
    previous = samples[index - 1]
    current = samples[index]
    interval_ns = current['time_ns'] - previous['time_ns']
    if interval_ns <= 0 or interval_ns > max_gap_ns:
        return None
    fraction = (time_ns - previous['time_ns']) / interval_ns
    return previous['value'] + fraction * (
        current['value'] - previous['value']
    )


def interpolate_transform(samples, time_ns, max_gap_ns, times=None):
    """Interpolate planar translation and already-unwrapped yaw."""
    if not samples:
        return None
    if times is None:
        times = [sample['time_ns'] for sample in samples]
    index = bisect.bisect_left(times, time_ns)
    if index < len(samples) and times[index] == time_ns:
        return {
            'x': samples[index]['x'],
            'y': samples[index]['y'],
            'yaw': samples[index]['yaw'],
        }
    if index == 0 or index == len(samples):
        return None
    previous = samples[index - 1]
    current = samples[index]
    interval_ns = current['time_ns'] - previous['time_ns']
    if interval_ns <= 0 or interval_ns > max_gap_ns:
        return None
    fraction = (time_ns - previous['time_ns']) / interval_ns
    return {
        field: previous[field] + fraction * (
            current[field] - previous[field]
        )
        for field in ('x', 'y', 'yaw')
    }


def prepare_transform_samples(samples):
    """Deduplicate transforms and unwrap their yaw values."""
    samples = deduplicate_samples(samples)
    yaws = unwrap_angles([sample['yaw'] for sample in samples])
    return [
        {**sample, 'yaw': yaw}
        for sample, yaw in zip(samples, yaws)
    ]


def integrate_series(samples, start_ns, end_ns, max_gap_ns):
    """Trapezoidally integrate a scalar series over valid clipped intervals."""
    samples = deduplicate_samples(samples)
    times = [sample['time_ns'] for sample in samples]
    total = 0.0
    valid_ns = 0
    gap_count = 0
    for previous, current in zip(samples, samples[1:]):
        interval_ns = current['time_ns'] - previous['time_ns']
        left_ns = max(start_ns, previous['time_ns'])
        right_ns = min(end_ns, current['time_ns'])
        if right_ns <= left_ns:
            continue
        if interval_ns <= 0 or interval_ns > max_gap_ns:
            gap_count += 1
            continue
        left = interpolate_sample(samples, left_ns, max_gap_ns, times)
        right = interpolate_sample(samples, right_ns, max_gap_ns, times)
        duration_s = (right_ns - left_ns) / 1e9
        total += 0.5 * (left + right) * duration_s
        valid_ns += right_ns - left_ns
    requested_ns = max(0, end_ns - start_ns)
    return {
        'yaw_rad': total,
        'valid_duration_s': valid_ns / 1e9,
        'excluded_duration_s': (requested_ns - valid_ns) / 1e9,
        'gap_count': gap_count,
    }


def ratio(numerator, denominator):
    """Return no ratio for a near-zero reference yaw."""
    if abs(denominator) < NEAR_ZERO_YAW_RAD:
        return None
    return numerator / denominator


def bin_index(abs_rate):
    """Assign an absolute rate to the approved half-open bins."""
    for index, (lower, upper) in enumerate(zip(BIN_EDGES, BIN_EDGES[1:])):
        if lower <= abs_rate < upper:
            return index
    raise ValueError(f'cannot bin yaw rate {abs_rate!r}')


def empty_bin(index):
    lower = BIN_EDGES[index]
    upper = BIN_EDGES[index + 1]
    return {
        'lower_rad_s': lower,
        'upper_rad_s': None if math.isinf(upper) else upper,
        'aligned_sample_count': 0,
        'effective_duration_s': 0.0,
        'excluded_duration_s': 0.0,
        'data_gap_count': 0,
        'slam_yaw_rad': 0.0,
        'slam_abs_yaw_rad': 0.0,
        'imu_yaw_rad': 0.0,
        'rf2o_yaw_rad': 0.0,
        'ekf_yaw_rad': 0.0,
    }


def finalize_bin(result):
    for source in RATE_SOURCES:
        result[f'{source}_to_slam_ratio'] = ratio(
            result[f'{source}_yaw_rad'],
            result['slam_yaw_rad'],
        )
        result[f'{source}_to_slam_abs_ratio'] = ratio(
            abs(result[f'{source}_yaw_rad']),
            abs(result['slam_yaw_rad']),
        )
    checks = {
        'sample_count': (
            result['aligned_sample_count'] >= MIN_EVIDENCE_SAMPLES
        ),
        'duration': (
            result['effective_duration_s'] >= MIN_EVIDENCE_DURATION_S
        ),
        'absolute_slam_yaw': (
            result['slam_abs_yaw_rad']
            >= MIN_EVIDENCE_ABS_SLAM_YAW_RAD
        ),
    }
    result['evidence_checks'] = checks
    result['evidence'] = (
        'sufficient' if all(checks.values()) else 'insufficient'
    )
    return result


def build_grid(start_ns, end_ns, interval_ns):
    times = []
    current = start_ns
    while current < end_ns:
        times.append(current)
        current += interval_ns
    if not times or times[-1] != end_ns:
        times.append(end_ns)
    return times


def aligned_bins(
    rate_samples,
    map_odom,
    odom_base,
    start_ns,
    end_ns,
    alignment_ns,
    max_gap_ns,
):
    """Accumulate all comparisons over identical grid intervals."""
    bins = [empty_bin(index) for index in range(len(BIN_EDGES) - 1)]
    grid = build_grid(start_ns, end_ns, alignment_ns)
    rate_times = {
        source: [
            sample['time_ns'] for sample in rate_samples[source]
        ]
        for source in RATE_SOURCES
    }
    map_odom_times = [sample['time_ns'] for sample in map_odom]
    odom_base_times = [sample['time_ns'] for sample in odom_base]
    values = []
    for time_ns in grid:
        rates = {
            source: interpolate_sample(
                rate_samples[source],
                time_ns,
                max_gap_ns,
                rate_times[source],
            )
            for source in RATE_SOURCES
        }
        first = interpolate_transform(
            map_odom,
            time_ns,
            max_gap_ns,
            map_odom_times,
        )
        second = interpolate_transform(
            odom_base,
            time_ns,
            max_gap_ns,
            odom_base_times,
        )
        slam = compose_transform(first, second) if first and second else None
        values.append({'time_ns': time_ns, 'rates': rates, 'slam': slam})

    previous_slam_yaw = None
    for item in values:
        if item['slam'] is not None:
            if previous_slam_yaw is None:
                item['slam_yaw'] = item['slam']['yaw']
            else:
                item['slam_yaw'] = previous_slam_yaw + wrap_angle(
                    item['slam']['yaw'] - previous_slam_yaw
                )
            previous_slam_yaw = item['slam_yaw']
        else:
            item['slam_yaw'] = None

    for previous, current in zip(values, values[1:]):
        duration_s = (current['time_ns'] - previous['time_ns']) / 1e9
        ekf_values = (
            previous['rates']['ekf'],
            current['rates']['ekf'],
        )
        if any(value is None for value in ekf_values):
            continue
        assignment_rate = 0.5 * sum(abs(value) for value in ekf_values)
        target = bins[bin_index(assignment_rate)]
        complete = (
            previous['slam_yaw'] is not None
            and current['slam_yaw'] is not None
            and all(
                previous['rates'][source] is not None
                and current['rates'][source] is not None
                for source in RATE_SOURCES
            )
        )
        if not complete:
            target['excluded_duration_s'] += duration_s
            target['data_gap_count'] += 1
            continue
        target['aligned_sample_count'] += 1
        target['effective_duration_s'] += duration_s
        slam_delta = current['slam_yaw'] - previous['slam_yaw']
        target['slam_yaw_rad'] += slam_delta
        target['slam_abs_yaw_rad'] += abs(slam_delta)
        for source in RATE_SOURCES:
            target[f'{source}_yaw_rad'] += (
                0.5
                * (
                    previous['rates'][source]
                    + current['rates'][source]
                )
                * duration_s
            )
    return [finalize_bin(result) for result in bins]


def select_message_time(header_stamp_ns, receive_ns):
    if header_stamp_ns > 0:
        return header_stamp_ns, False
    return receive_ns, True


def read_samples(bag_path):
    loaded = load_bag_messages(bag_path)
    topic_types = loaded['topic_types']
    missing = [
        topic for topic in TOPICS.values()
        if topic not in topic_types
    ]
    if missing:
        raise RuntimeError(f'missing required topics: {", ".join(missing)}')
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in TOPICS.values()
    }
    rate_samples = {source: [] for source in RATE_SOURCES}
    map_odom = []
    odom_base = []
    fallback_counts = {source: 0 for source in (*RATE_SOURCES, 'odom_base')}
    order = 0

    for topic, serialized_data, receive_ns in loaded['messages']:
        order += 1
        if topic == TOPICS['imu']:
            message = deserialize_message(
                serialized_data,
                message_types[topic],
            )
            header_ns = stamp_to_ns(message.header.stamp)
            time_ns, fallback = select_message_time(header_ns, receive_ns)
            fallback_counts['imu'] += fallback
            rate_samples['imu'].append({
                'time_ns': time_ns,
                'value': message.angular_velocity.z,
                'order': order,
            })
        elif topic in (TOPICS['rf2o'], TOPICS['ekf']):
            source = 'rf2o' if topic == TOPICS['rf2o'] else 'ekf'
            message = deserialize_message(
                serialized_data,
                message_types[topic],
            )
            header_ns = stamp_to_ns(message.header.stamp)
            time_ns, fallback = select_message_time(header_ns, receive_ns)
            fallback_counts[source] += fallback
            rate_samples[source].append({
                'time_ns': time_ns,
                'value': message.twist.twist.angular.z,
                'order': order,
            })
        elif topic == TOPICS['tf']:
            message = deserialize_message(
                serialized_data,
                message_types[topic],
            )
            for transform in message.transforms:
                parent = normalize_frame(transform.header.frame_id)
                child = normalize_frame(transform.child_frame_id)
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                sample = {
                    'x': translation.x,
                    'y': translation.y,
                    'yaw': quaternion_to_yaw(rotation),
                    'order': order,
                }
                if (parent, child) == ('map', 'odom'):
                    map_odom.append({**sample, 'time_ns': receive_ns})
                elif (parent, child) == ('odom', 'base_link'):
                    header_ns = stamp_to_ns(transform.header.stamp)
                    time_ns, fallback = select_message_time(
                        header_ns,
                        receive_ns,
                    )
                    fallback_counts['odom_base'] += fallback
                    odom_base.append({**sample, 'time_ns': time_ns})

    if not map_odom or not odom_base:
        raise RuntimeError(
            'required map -> odom -> base_link TF chain is absent'
        )
    return {
        'loaded': loaded,
        'rate_samples': {
            source: deduplicate_samples(samples)
            for source, samples in rate_samples.items()
        },
        'map_odom': prepare_transform_samples(map_odom),
        'odom_base': prepare_transform_samples(odom_base),
        'fallback_counts': fallback_counts,
    }


def analyze_bag(
    bag_path,
    start_sec,
    end_sec,
    alignment_sec,
    max_gap_sec,
):
    bag_path = bag_path.expanduser().resolve()
    data = read_samples(bag_path)
    loaded = data['loaded']
    bag_start_ns = loaded['bag_start_ns']
    bag_end_ns = loaded['bag_end_ns']
    if bag_start_ns is None or bag_end_ns is None:
        raise RuntimeError('bag contains no recoverable messages')
    bag_duration_s = (bag_end_ns - bag_start_ns) / 1e9
    requested_end_sec = bag_duration_s if end_sec is None else end_sec
    if start_sec >= bag_duration_s:
        raise RuntimeError(
            f'start time {start_sec:g} s is outside the '
            f'{bag_duration_s:.3f} s bag'
        )
    effective_end_sec = min(requested_end_sec, bag_duration_s)
    if effective_end_sec <= start_sec:
        raise RuntimeError('effective analysis window is empty')
    start_ns = bag_start_ns + round(start_sec * 1e9)
    end_ns = bag_start_ns + round(effective_end_sec * 1e9)
    max_gap_ns = round(max_gap_sec * 1e9)
    alignment_ns = round(alignment_sec * 1e9)

    aggregate = {}
    for source in RATE_SOURCES:
        aggregate[source] = integrate_series(
            data['rate_samples'][source],
            start_ns,
            end_ns,
            max_gap_ns,
        )

    bins = aligned_bins(
        data['rate_samples'],
        data['map_odom'],
        data['odom_base'],
        start_ns,
        end_ns,
        alignment_ns,
        max_gap_ns,
    )
    aggregate['slam'] = {
        'yaw_rad': sum(result['slam_yaw_rad'] for result in bins),
        'valid_duration_s': sum(
            result['effective_duration_s'] for result in bins
        ),
        'excluded_duration_s': 0.0,
        'gap_count': sum(result['data_gap_count'] for result in bins),
    }
    aggregate['slam']['excluded_duration_s'] = (
        (end_ns - start_ns) / 1e9
        - aggregate['slam']['valid_duration_s']
    )
    for source in RATE_SOURCES:
        aggregate[source]['to_slam_ratio'] = ratio(
            aggregate[source]['yaw_rad'],
            aggregate['slam']['yaw_rad'],
        )
        aggregate[source]['to_slam_abs_ratio'] = ratio(
            abs(aggregate[source]['yaw_rad']),
            abs(aggregate['slam']['yaw_rad']),
        )

    counts = {
        source: sum(
            start_ns <= sample['time_ns'] <= end_ns
            for sample in data['rate_samples'][source]
        )
        for source in RATE_SOURCES
    }
    counts.update({
        'map_odom': sum(
            start_ns <= sample['time_ns'] <= end_ns
            for sample in data['map_odom']
        ),
        'odom_base': sum(
            start_ns <= sample['time_ns'] <= end_ns
            for sample in data['odom_base']
        ),
    })
    duration_s = effective_end_sec - start_sec
    return {
        'bag_path': str(bag_path),
        'mcap_paths': [
            str(path.resolve()) for path in mcap_files_for_path(bag_path)
        ],
        'recovery_used': loaded['recovery_used'],
        'truncated': loaded['truncated'],
        'recovery_notes': loaded['recovery_notes'],
        'open_error': loaded['open_error'],
        'bag_duration_s': bag_duration_s,
        'requested_window': {
            'start_sec': start_sec,
            'end_sec': end_sec,
        },
        'effective_window': {
            'start_sec': start_sec,
            'end_sec': effective_end_sec,
            'duration_s': duration_s,
        },
        'topics': TOPICS,
        'timestamp_sources': {
            'imu': 'message header; bag receive fallback if zero',
            'rf2o': 'message header; bag receive fallback if zero',
            'ekf': 'message header; bag receive fallback if zero',
            'map_odom': (
                'bag receive (duplicate header stamps carry differing '
                'transforms)'
            ),
            'odom_base': 'transform header; bag receive fallback if zero',
        },
        'timestamp_fallback_counts': data['fallback_counts'],
        'alignment': {
            'method': (
                'linear interpolation on a fixed grid; planar TF composition'
            ),
            'interval_sec': alignment_sec,
            'maximum_gap_sec': max_gap_sec,
        },
        'integration_method': 'trapezoidal over valid consecutive samples',
        'binning_source': '/odometry/filtered twist.twist.angular.z',
        'bin_edges_rad_s': [0.0, 0.3, 1.0, 2.0, None],
        'minimum_evidence': {
            'aligned_sample_count': MIN_EVIDENCE_SAMPLES,
            'effective_duration_s': MIN_EVIDENCE_DURATION_S,
            'absolute_slam_yaw_rad': MIN_EVIDENCE_ABS_SLAM_YAW_RAD,
        },
        'sample_counts': counts,
        'sample_rates_hz': {
            source: count / duration_s
            for source, count in counts.items()
        },
        'aggregate': aggregate,
        'bins': bins,
    }


def combine_results(results):
    combined = []
    for index in range(len(BIN_EDGES) - 1):
        target = empty_bin(index)
        for result in results:
            source = result['bins'][index]
            for field in (
                'aligned_sample_count',
                'effective_duration_s',
                'excluded_duration_s',
                'data_gap_count',
                'slam_yaw_rad',
                'slam_abs_yaw_rad',
                'imu_yaw_rad',
                'rf2o_yaw_rad',
                'ekf_yaw_rad',
            ):
                target[field] += source[field]
        combined.append(finalize_bin(target))
    return combined


def degrees(radians):
    return math.degrees(radians)


def format_number(value, digits=4):
    return 'n/a' if value is None else f'{value:.{digits}f}'


def bin_label(result):
    upper = result['upper_rad_s']
    if upper is None:
        return f'>={result["lower_rad_s"]:.1f}'
    return f'{result["lower_rad_s"]:.1f}-{upper:.1f}'


def print_bin_table(bins):
    headers = (
        'bin rad/s', 'n', 'valid s', 'excl s', 'gaps',
        'SLAM rad', 'IMU rad', 'RF2O rad', 'EKF rad',
        'I/S', 'R/S', 'E/S', 'evidence',
    )
    rows = []
    for result in bins:
        rows.append((
            bin_label(result),
            str(result['aligned_sample_count']),
            f'{result["effective_duration_s"]:.3f}',
            f'{result["excluded_duration_s"]:.3f}',
            str(result['data_gap_count']),
            f'{result["slam_yaw_rad"]:.4f}',
            f'{result["imu_yaw_rad"]:.4f}',
            f'{result["rf2o_yaw_rad"]:.4f}',
            f'{result["ekf_yaw_rad"]:.4f}',
            format_number(result['imu_to_slam_ratio'], 3),
            format_number(result['rf2o_to_slam_ratio'], 3),
            format_number(result['ekf_to_slam_ratio'], 3),
            result['evidence'],
        ))
    widths = [
        max(len(headers[index]), *(len(row[index]) for row in rows))
        for index in range(len(headers))
    ]
    print('  '.join(
        header.ljust(widths[index])
        for index, header in enumerate(headers)
    ))
    print('  '.join('-' * width for width in widths))
    for row in rows:
        print('  '.join(
            value.ljust(widths[index])
            for index, value in enumerate(row)
        ))


def print_report(result):
    print(f'\n=== {result["bag_path"]} ===')
    print(f'MCAP: {", ".join(result["mcap_paths"])}')
    print(
        f'Recovery used: {"yes" if result["recovery_used"] else "no"}; '
        f'truncated: {"yes" if result["truncated"] else "no"}'
    )
    for note in result['recovery_notes']:
        print(f'Recovery note: {note}')
    requested = result['requested_window']
    effective = result['effective_window']
    requested_end = (
        'bag end'
        if requested['end_sec'] is None
        else f'{requested["end_sec"]:.3f} s'
    )
    print(
        f'Requested window: {requested["start_sec"]:.3f} s to '
        f'{requested_end}; effective: {effective["start_sec"]:.3f} s to '
        f'{effective["end_sec"]:.3f} s '
        f'({effective["duration_s"]:.3f} s)'
    )
    print('Topics:')
    for source, topic in result['topics'].items():
        print(f'  {source}: {topic}')
    print('Timestamp sources:')
    for source, description in result['timestamp_sources'].items():
        fallback = result['timestamp_fallback_counts'].get(source)
        suffix = '' if fallback is None else f'; fallbacks={fallback}'
        print(f'  {source}: {description}{suffix}')
    alignment = result['alignment']
    print(
        'Alignment: '
        f'{alignment["method"]}; interval={alignment["interval_sec"]:.3f} s; '
        f'maximum gap={alignment["maximum_gap_sec"]:.3f} s'
    )
    print(f'Integration: {result["integration_method"]}')
    print(f'Binning source: {result["binning_source"]}')
    print('Bins: [0.0, 0.3), [0.3, 1.0), [1.0, 2.0), [2.0, infinity)')
    minimum = result['minimum_evidence']
    print(
        'Minimum sufficient evidence: '
        f'{minimum["aligned_sample_count"]} aligned intervals, '
        f'{minimum["effective_duration_s"]:.1f} s, and '
        f'{minimum["absolute_slam_yaw_rad"]:.2f} rad accumulated '
        'absolute SLAM yaw'
    )
    print(
        'TF input coverage: '
        f'map -> odom={result["sample_counts"]["map_odom"]} '
        f'({result["sample_rates_hz"]["map_odom"]:.3f} Hz), '
        f'odom -> base_link={result["sample_counts"]["odom_base"]} '
        f'({result["sample_rates_hz"]["odom_base"]:.3f} Hz)'
    )
    print('\nSource coverage and aggregate yaw:')
    print(
        f'{"source":<10} {"count":>8} {"rate Hz":>9} {"valid s":>10} '
        f'{"excluded s":>11} {"yaw rad":>11} {"yaw deg":>11} '
        f'{"signed /SLAM":>13} {"abs /SLAM":>10}'
    )
    for source in (*RATE_SOURCES, 'slam'):
        aggregate = result['aggregate'][source]
        count = result['sample_counts'].get(source)
        rate = result['sample_rates_hz'].get(source)
        print(
            f'{source:<10} '
            f'{str(count) if count is not None else "grid":>8} '
            f'{format_number(rate, 3):>9} '
            f'{aggregate["valid_duration_s"]:>10.3f} '
            f'{aggregate["excluded_duration_s"]:>11.3f} '
            f'{aggregate["yaw_rad"]:>11.4f} '
            f'{degrees(aggregate["yaw_rad"]):>11.3f} '
            f'{format_number(aggregate.get("to_slam_ratio"), 4):>13} '
            f'{format_number(aggregate.get("to_slam_abs_ratio"), 4):>10}'
        )
    print('\nAligned yaw-rate bins:')
    print_bin_table(result['bins'])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Cross-validate IMU, RF2O, and EKF yaw against map-referenced '
            'SLAM yaw from ROS 2 MCAP bags.'
        ),
    )
    parser.add_argument(
        'bags',
        nargs='+',
        type=Path,
        metavar='BAG',
        help='ROS 2 MCAP bag directory or bare .mcap file',
    )
    parser.add_argument(
        '--start-sec',
        type=float,
        default=0.0,
        metavar='SECONDS',
        help='bag-relative analysis start (default: 0)',
    )
    parser.add_argument(
        '--end-sec',
        type=float,
        default=None,
        metavar='SECONDS',
        help='bag-relative analysis end (default: bag end)',
    )
    parser.add_argument(
        '--alignment-sec',
        type=float,
        default=DEFAULT_ALIGNMENT_S,
        metavar='SECONDS',
        help='fixed comparison-grid interval (default: 0.02)',
    )
    parser.add_argument(
        '--max-gap-sec',
        type=float,
        default=DEFAULT_MAX_GAP_S,
        metavar='SECONDS',
        help='maximum interpolation/integration gap (default: 0.25)',
    )
    parser.add_argument(
        '--format',
        choices=('text', 'json'),
        default='text',
        help='output format (default: text)',
    )
    args = parser.parse_args(argv)
    for name in ('start_sec', 'alignment_sec', 'max_gap_sec'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            option = name.replace('_', '-')
            parser.error(
                f'--{option} must be finite and non-negative'
            )
    if args.alignment_sec == 0.0 or args.max_gap_sec == 0.0:
        parser.error(
            '--alignment-sec and --max-gap-sec must be greater than zero'
        )
    if args.end_sec is not None:
        if not math.isfinite(args.end_sec) or args.end_sec < 0.0:
            parser.error('--end-sec must be finite and non-negative')
        if args.end_sec <= args.start_sec:
            parser.error('--end-sec must be greater than --start-sec')
    return args


def main(argv=None):
    args = parse_args(argv)
    results = []
    for bag_path in args.bags:
        try:
            results.append(analyze_bag(
                bag_path,
                args.start_sec,
                args.end_sec,
                args.alignment_sec,
                args.max_gap_sec,
            ))
        except Exception as error:
            raise SystemExit(
                f'error: failed to analyze {bag_path}: {error}'
            ) from error
    combined = combine_results(results) if len(results) > 1 else None
    if args.format == 'json':
        print(json.dumps(
            {'bags': results, 'combined_bins': combined},
            indent=2,
        ))
        return
    for result in results:
        print_report(result)
    if combined is not None:
        print(
            '\n=== Combined bins '
            '(summed contributions; ratios computed after sum) ==='
        )
        print_bin_table(combined)


if __name__ == '__main__':
    main()
