#!/usr/bin/env python3
"""Analyze constant-throttle ground runs recorded in a ROS 2 MCAP bag."""

import argparse
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import analyze_localization_bag as localization


REQUIRED_TOPICS = (
    '/cmd_vel',
    '/wheel/encoder_state',
    '/wheel/odom',
    '/odometry/filtered',
    '/tf',
    '/scan',
)
METRES_PER_EDGE = 0.010282
COMMAND_GAP_LIMIT_S = 0.5
MINIMUM_NEUTRAL_DURATION_S = 2.0
RELATIVE_EPSILON_MPS = 0.03
ABSOLUTE_DISAGREEMENT_MPS = 0.10
RELATIVE_DISAGREEMENT = 0.20
MONOTONIC_DROP_MPS = 0.03
TARGET_SPEED_MPS = 0.3


def mean(values):
    return statistics.fmean(values) if values else None


def sample_std(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0 if values else None


def sign(value, zero_tolerance):
    if value > zero_tolerance:
        return 1
    if value < -zero_tolerance:
        return -1
    return 0


def timestamp_ns(message, receive_ns, topic):
    """Use a valid message stamp where available, otherwise receive time."""
    if topic == '/wheel/encoder_state':
        stamp = message.stamp
    elif topic in ('/wheel/odom', '/odometry/filtered', '/scan'):
        stamp = message.header.stamp
    else:
        return receive_ns, 'bag_receive'
    value = localization.stamp_to_ns(stamp)
    return (value, 'header') if value > 0 else (receive_ns, 'bag_receive_fallback')


def timestamp_diagnostics(samples):
    arrival_times = [item['time_ns'] for item in samples]
    non_monotonic = sum(
        current < previous
        for previous, current in zip(arrival_times, arrival_times[1:])
    )
    duplicates = len(arrival_times) - len(set(arrival_times))
    ordered = sorted(set(arrival_times))
    gaps = [
        (current - previous) / 1e9
        for previous, current in zip(ordered, ordered[1:])
    ]
    median_gap = statistics.median(gaps) if gaps else None
    nominal_gap = median_gap
    gap_limit = max(0.5, 3.0 * nominal_gap) if nominal_gap else 0.5
    large_gaps = sum(gap > gap_limit for gap in gaps)
    duration = (ordered[-1] - ordered[0]) / 1e9 if len(ordered) > 1 else 0.0
    return {
        'count': len(samples),
        'rate_hz': (len(ordered) - 1) / duration if duration > 0 else None,
        'first_timestamp_ns': ordered[0] if ordered else None,
        'last_timestamp_ns': ordered[-1] if ordered else None,
        'non_monotonic_samples': non_monotonic,
        'duplicate_timestamps': duplicates,
        'large_gap_count': large_gaps,
        'large_gap_threshold_s': gap_limit,
        'maximum_gap_s': max(gaps) if gaps else None,
    }


def deduplicate_sorted(samples):
    """Sort by chosen timestamp and retain the last received duplicate."""
    by_time = {}
    for item in samples:
        by_time[item['time_ns']] = item
    return [by_time[key] for key in sorted(by_time)]


def segment_commands(
    command_samples,
    command_tolerance=0.005,
    minimum_duration=6.0,
    settling_duration=2.0,
    zero_tolerance=0.005,
    gap_limit_s=COMMAND_GAP_LIMIT_S,
    minimum_neutral_duration_s=MINIMUM_NEUTRAL_DURATION_S,
):
    """Return nonzero, constant-command segments separated by zero intervals."""
    samples = deduplicate_sorted(command_samples)
    candidates = []
    current = []
    current_preceded_by_zero = False
    neutral_start_ns = None
    neutral_end_ns = None
    current_neutral_start_ns = None
    current_neutral_end_ns = None

    def close():
        if not current:
            return
        start_ns = current[0]['time_ns']
        end_ns = current[-1]['time_ns']
        duration = (end_ns - start_ns) / 1e9
        representative = statistics.median(item['value'] for item in current)
        if duration >= minimum_duration:
            analyzed_start_ns = start_ns + round(settling_duration * 1e9)
            if analyzed_start_ns < end_ns:
                candidates.append({
                    'start_ns': start_ns,
                    'end_ns': end_ns,
                    'analyzed_start_ns': analyzed_start_ns,
                    'analyzed_end_ns': end_ns,
                    'raw_duration_s': duration,
                    'analyzed_duration_s': (end_ns - analyzed_start_ns) / 1e9,
                    'command_samples': list(current),
                    'representative_command': representative,
                    'preceded_by_zero': current_preceded_by_zero,
                    'neutral_start_ns': current_neutral_start_ns,
                    'neutral_end_ns': current_neutral_end_ns,
                })

    for item in samples:
        item_sign = sign(item['value'], zero_tolerance)
        if item_sign == 0:
            close()
            current = []
            if (
                neutral_start_ns is None
                or neutral_end_ns is None
                or (
                    item['time_ns'] - neutral_end_ns
                ) / 1e9 > gap_limit_s
            ):
                neutral_start_ns = item['time_ns']
            neutral_end_ns = item['time_ns']
            continue
        if not current:
            current = [item]
            current_preceded_by_zero = (
                neutral_start_ns is not None
                and neutral_end_ns is not None
                and (
                    neutral_end_ns - neutral_start_ns
                ) / 1e9 >= minimum_neutral_duration_s
            )
            current_neutral_start_ns = neutral_start_ns
            current_neutral_end_ns = neutral_end_ns
            neutral_start_ns = None
            neutral_end_ns = None
            continue
        gap_s = (item['time_ns'] - current[-1]['time_ns']) / 1e9
        representative = statistics.median(sample['value'] for sample in current)
        prospective = current + [item]
        prospective_values = [sample['value'] for sample in prospective]
        if (
            gap_s > gap_limit_s
            or item_sign != sign(representative, zero_tolerance)
            or max(prospective_values) - min(prospective_values)
            > 2.0 * command_tolerance
        ):
            close()
            current = [item]
            current_preceded_by_zero = False
            current_neutral_start_ns = None
            current_neutral_end_ns = None
        else:
            current.append(item)
    close()
    return candidates


def samples_between(samples, start_ns, end_ns):
    return [
        item for item in samples
        if start_ns <= item['time_ns'] <= end_ns
    ]


def segment_metrics(
    candidate,
    encoder_samples,
    state_samples,
    ekf_samples,
    motion_threshold=0.03,
    minimum_samples=2,
    metres_per_edge=METRES_PER_EDGE,
):
    command = candidate['command_samples']
    encoder = samples_between(
        encoder_samples,
        candidate['analyzed_start_ns'],
        candidate['analyzed_end_ns'],
    )
    states = samples_between(
        state_samples,
        candidate['analyzed_start_ns'],
        candidate['analyzed_end_ns'],
    )
    ekf = samples_between(
        ekf_samples,
        candidate['analyzed_start_ns'],
        candidate['analyzed_end_ns'],
    )
    if len(encoder) < minimum_samples or len(states) < minimum_samples or len(ekf) < minimum_samples:
        return None

    command_values = [item['value'] for item in command]
    command_median = statistics.median(command_values)
    encoder_values = [item['value'] for item in encoder]
    ekf_values = [item['value'] for item in ekf]
    stationary_fraction = mean([float(item['stationary']) for item in states])
    encoder_mean = mean(encoder_values)
    ekf_mean = mean(ekf_values)
    difference = encoder_mean - ekf_mean
    absolute_difference = abs(difference)
    relative_difference = (
        absolute_difference / max(abs(encoder_mean), RELATIVE_EPSILON_MPS)
    )
    encoder_abs_mean = mean([abs(value) for value in encoder_values])
    expected_sign = 1 if command_median > 0 else -1
    signed_directional_speed = expected_sign * encoder_mean
    direction_ambiguous = (
        encoder_abs_mean >= motion_threshold
        and abs(encoder_mean) < motion_threshold
    )
    direction_sign_matches = signed_directional_speed >= 0.0
    sustained = (
        encoder_abs_mean >= motion_threshold
        and stationary_fraction <= 0.20
        and direction_sign_matches
        and not direction_ambiguous
        and signed_directional_speed >= motion_threshold
    )
    warnings = []
    if direction_ambiguous:
        warnings.append(
            'signed encoder velocity is direction-ambiguous; run excluded from sustained-motion fit'
        )
    elif not direction_sign_matches:
        warnings.append(
            'encoder motion sign opposes command; run excluded from sustained-motion fit'
        )
    if (
        absolute_difference > ABSOLUTE_DISAGREEMENT_MPS
        or relative_difference > RELATIVE_DISAGREEMENT
    ):
        warnings.append('encoder/EKF disagreement exceeds diagnostic threshold')

    edge_rates = [item['edge_rate'] for item in states]
    edge_speed = mean(edge_rates) * metres_per_edge
    edge_rate_difference = encoder_abs_mean - edge_speed
    if abs(edge_rate_difference) > 0.02:
        warnings.append('wheel odometry disagrees with edge-rate cross-check')

    return {
        'index': None,
        'direction': 'forward' if command_median > 0 else 'reverse',
        'command_start_timestamp_ns': candidate['start_ns'],
        'command_end_timestamp_ns': candidate['end_ns'],
        'analyzed_start_timestamp_ns': candidate['analyzed_start_ns'],
        'analyzed_end_timestamp_ns': candidate['analyzed_end_ns'],
        'command_median': command_median,
        'command_mean': mean(command_values),
        'command_standard_deviation': sample_std(command_values),
        'raw_segment_duration_s': candidate['raw_duration_s'],
        'analyzed_duration_s': candidate['analyzed_duration_s'],
        'command_sample_count': len(command),
        'encoder_sample_count': len(encoder),
        'encoder_state_sample_count': len(states),
        'ekf_sample_count': len(ekf),
        'encoder_mean_signed_mps': encoder_mean,
        'encoder_mean_magnitude_mps': encoder_abs_mean,
        'encoder_standard_deviation_mps': sample_std(encoder_values),
        'encoder_min_mps': min(encoder_values),
        'encoder_max_mps': max(encoder_values),
        'stationary_fraction': stationary_fraction,
        'edge_rate_mean_hz': mean(edge_rates),
        'edge_rate_implied_speed_mps': edge_speed,
        'encoder_abs_minus_edge_implied_mps': edge_rate_difference,
        'ekf_mean_signed_mps': ekf_mean,
        'ekf_mean_magnitude_mps': mean([abs(value) for value in ekf_values]),
        'ekf_standard_deviation_mps': sample_std(ekf_values),
        'encoder_minus_ekf_signed_mps': difference,
        'encoder_ekf_absolute_difference_mps': absolute_difference,
        'encoder_ekf_relative_difference_percent': relative_difference * 100.0,
        'encoder_ekf_disagreement': (
            absolute_difference > ABSOLUTE_DISAGREEMENT_MPS
            or relative_difference > RELATIVE_DISAGREEMENT
        ),
        'direction_sign_matches': direction_sign_matches,
        'direction_ambiguous': direction_ambiguous,
        'sustained_motion': sustained,
        'deadband_candidate': (
            not sustained
            and direction_sign_matches
            and not direction_ambiguous
        ),
        'warnings': warnings,
    }


def group_segments(segments, tolerance):
    groups = []
    for segment in sorted(segments, key=lambda item: abs(item['command_median'])):
        magnitude = abs(segment['command_median'])
        group = next(
            (
                item for item in groups
                if abs(magnitude - item['representative_throttle']) <= tolerance
            ),
            None,
        )
        if group is None:
            group = {'representative_throttle': magnitude, 'segments': []}
            groups.append(group)
        group['segments'].append(segment)
        group['representative_throttle'] = statistics.median(
            abs(item['command_median']) for item in group['segments']
        )
    return groups


def aggregate_segments(segments, tolerance):
    points = []
    for repeat_group, group in enumerate(group_segments(segments, tolerance), 1):
        members = group['segments']
        sustained_members = [
            item for item in members if item['sustained_motion']
        ]
        aggregated_members = sustained_members or members
        total_duration = sum(
            item['analyzed_duration_s'] for item in aggregated_members
        )

        def weighted(field):
            return sum(
                item[field] * item['analyzed_duration_s']
                for item in aggregated_members
            ) / total_duration

        encoder_weighted_mean = weighted('encoder_mean_magnitude_mps')
        encoder_pooled_variance = sum(
            item['analyzed_duration_s'] * (
                item['encoder_standard_deviation_mps'] ** 2
                + (
                    item['encoder_mean_magnitude_mps']
                    - encoder_weighted_mean
                ) ** 2
            )
            for item in aggregated_members
        ) / total_duration
        points.append({
            'normalized_throttle': group['representative_throttle'],
            'analyzed_duration_s': total_duration,
            'encoder_mean_speed_magnitude_mps': encoder_weighted_mean,
            'encoder_standard_deviation_mps': math.sqrt(
                encoder_pooled_variance
            ),
            'ekf_mean_speed_magnitude_mps': weighted(
                'ekf_mean_magnitude_mps'
            ),
            'encoder_ekf_absolute_difference_mps': weighted(
                'encoder_ekf_absolute_difference_mps'
            ),
            'sustained_motion': bool(sustained_members),
            'mixed_sustained_motion': (
                bool(sustained_members) and len(sustained_members) != len(members)
            ),
            'sustained_segment_count': len(sustained_members),
            'repeat_group': repeat_group if len(members) > 1 else None,
            'segment_indices': [
                item['index'] for item in aggregated_members
            ],
            'all_segment_indices': [item['index'] for item in members],
            'segment_count': len(members),
            'weighting': 'analyzed_duration',
            'aggregation_cohort': (
                'sustained_motion_only'
                if sustained_members else 'no_sustained_motion'
            ),
        })
    return points


def deadband_summary(points):
    nonmoving = [
        point['normalized_throttle']
        for point in points if not point['sustained_motion']
    ]
    moving = [
        point['normalized_throttle']
        for point in points if point['sustained_motion']
    ]
    highest_no_motion = max(nonmoving) if nonmoving else None
    lowest_motion = min(moving) if moving else None
    return {
        'highest_tested_no_sustained_motion': highest_no_motion,
        'lowest_tested_sustained_motion': lowest_motion,
        'bracket': (
            [highest_no_motion, lowest_motion]
            if highest_no_motion is not None and lowest_motion is not None
            else None
        ),
    }


def minimum_sustainable_speed(segments):
    moving = [item for item in segments if item['sustained_motion']]
    if not moving:
        return None
    item = min(moving, key=lambda value: value['encoder_mean_magnitude_mps'])
    return {
        'segment_index': item['index'],
        'normalized_throttle': abs(item['command_median']),
        'encoder_mean_speed_magnitude_mps': item['encoder_mean_magnitude_mps'],
        'encoder_standard_deviation_mps': item['encoder_standard_deviation_mps'],
        'stationary_fraction': item['stationary_fraction'],
        'analyzed_duration_s': item['analyzed_duration_s'],
        'ekf_mean_speed_magnitude_mps': item['ekf_mean_magnitude_mps'],
    }


def interpolate_inverse(lower, upper, target):
    speed_span = (
        upper['encoder_mean_speed_magnitude_mps']
        - lower['encoder_mean_speed_magnitude_mps']
    )
    if abs(speed_span) < 1e-12:
        return None
    fraction = (
        target - lower['encoder_mean_speed_magnitude_mps']
    ) / speed_span
    return (
        lower['normalized_throttle']
        + fraction * (
            upper['normalized_throttle'] - lower['normalized_throttle']
        )
    )


def fit_and_target(points, minimum_speed, target=TARGET_SPEED_MPS):
    moving = sorted(
        (item for item in points if item['sustained_motion']),
        key=lambda item: item['normalized_throttle'],
    )
    fit = {
        'type': 'monotonic_piecewise_linear',
        'input_point_count': len(moving),
        'throttle_range': (
            [moving[0]['normalized_throttle'], moving[-1]['normalized_throttle']]
            if moving else None
        ),
        'speed_range_mps': (
            [
                min(item['encoder_mean_speed_magnitude_mps'] for item in moving),
                max(item['encoder_mean_speed_magnitude_mps'] for item in moving),
            ]
            if moving else None
        ),
        'residual_rmse_mps': 0.0 if moving else None,
        'maximum_absolute_residual_mps': 0.0 if moving else None,
        'monotonic': True,
        'suspect_segment_indices': [],
        'throttle_at_0_3_mps': None,
    }
    for previous, current in zip(moving, moving[1:]):
        drop = (
            previous['encoder_mean_speed_magnitude_mps']
            - current['encoder_mean_speed_magnitude_mps']
        )
        if drop > MONOTONIC_DROP_MPS:
            fit['monotonic'] = False
            fit['suspect_segment_indices'].extend(
                previous['segment_indices'] + current['segment_indices']
            )
    fit['suspect_segment_indices'] = sorted(
        set(fit['suspect_segment_indices'])
    )
    if not fit['monotonic']:
        fit['type'] = 'piecewise_linear_raw_non_monotonic'

    target_result = {
        'target_speed_mps': target,
        'status': 'unavailable',
        'measured_range_reaches_target': False,
        'estimated_normalized_throttle': None,
        'interpolation_bracket': None,
        'closest_measured_point': None,
        'below_minimum_sustainable_speed': (
            minimum_speed is not None
            and target < minimum_speed['encoder_mean_speed_magnitude_mps']
        ),
        'warning': None,
    }
    if not moving:
        target_result['warning'] = 'no sustained-motion points'
        return fit, target_result

    closest = min(
        moving,
        key=lambda item: abs(
            item['encoder_mean_speed_magnitude_mps'] - target
        ),
    )
    target_result['closest_measured_point'] = {
        'normalized_throttle': closest['normalized_throttle'],
        'encoder_mean_speed_magnitude_mps': (
            closest['encoder_mean_speed_magnitude_mps']
        ),
        'segment_indices': closest['segment_indices'],
    }
    speeds = [item['encoder_mean_speed_magnitude_mps'] for item in moving]
    if target < min(speeds) or target > max(speeds):
        target_result['status'] = 'outside_measured_moving_range'
        target_result['warning'] = 'no extrapolation performed'
        return fit, target_result

    exact_points = [
        item for item in moving
        if abs(item['encoder_mean_speed_magnitude_mps'] - target) < 1e-12
    ]
    if exact_points:
        exact_throttles = [
            item['normalized_throttle'] for item in exact_points
        ]
        estimate = statistics.median(exact_throttles)
        target_result['interpolation_bracket'] = [
            min(exact_throttles), max(exact_throttles)
        ]
    else:
        bracket = None
        for lower, upper in zip(moving, moving[1:]):
            low_speed = lower['encoder_mean_speed_magnitude_mps']
            high_speed = upper['encoder_mean_speed_magnitude_mps']
            if min(low_speed, high_speed) < target < max(low_speed, high_speed):
                bracket = (lower, upper)
                break
        if bracket is None:
            target_result['status'] = 'non_monotonic_unbracketed'
            target_result['warning'] = 'raw points do not provide a usable bracket'
            return fit, target_result
        estimate = interpolate_inverse(bracket[0], bracket[1], target)
        target_result['interpolation_bracket'] = [
            bracket[0]['normalized_throttle'],
            bracket[1]['normalized_throttle'],
        ]
    target_result['status'] = 'interpolated'
    target_result['measured_range_reaches_target'] = True
    target_result['estimated_normalized_throttle'] = estimate
    if fit['monotonic']:
        fit['throttle_at_0_3_mps'] = estimate
    else:
        target_result['warning'] = (
            'interpolation uses raw non-monotonic points; inspect suspect segments'
        )
    return fit, target_result


def pack_droop_pairs(segments, tolerance):
    results = []
    if not segments:
        return results
    session_start_ns = min(
        item['command_start_timestamp_ns'] for item in segments
    )
    session_end_ns = max(
        item['command_end_timestamp_ns'] for item in segments
    )
    session_span_ns = session_end_ns - session_start_ns
    for direction in ('forward', 'reverse'):
        directional = [
            item for item in segments
            if item['direction'] == direction and item['sustained_motion']
        ]
        groups = [
            group for group in group_segments(directional, tolerance)
            if len(group['segments']) >= 2
        ]
        groups.sort(key=lambda group: group['representative_throttle'])
        for group in groups[:3]:
            ordered = sorted(
                group['segments'],
                key=lambda item: item['command_start_timestamp_ns'],
            )
            first, last = ordered[0], ordered[-1]
            if (
                session_span_ns > 0
                and (
                    first['command_start_timestamp_ns']
                    > session_start_ns + 0.40 * session_span_ns
                    or last['command_start_timestamp_ns']
                    < session_start_ns + 0.60 * session_span_ns
                )
            ):
                continue
            initial = first['encoder_mean_magnitude_mps']
            final = last['encoder_mean_magnitude_mps']
            ekf_initial = first['ekf_mean_magnitude_mps']
            ekf_final = last['ekf_mean_magnitude_mps']
            results.append({
                'direction': direction,
                'normalized_throttle': group['representative_throttle'],
                'first_segment_index': first['index'],
                'last_segment_index': last['index'],
                'first_timestamp_ns': first['command_start_timestamp_ns'],
                'last_timestamp_ns': last['command_start_timestamp_ns'],
                'throttle_difference': (
                    abs(last['command_median']) - abs(first['command_median'])
                ),
                'initial_encoder_speed_mps': initial,
                'final_encoder_speed_mps': final,
                'encoder_absolute_speed_change_mps': final - initial,
                'encoder_percentage_change': (
                    (final - initial) / initial * 100.0
                    if initial > RELATIVE_EPSILON_MPS else None
                ),
                'initial_ekf_speed_mps': ekf_initial,
                'final_ekf_speed_mps': ekf_final,
                'ekf_absolute_speed_change_mps': ekf_final - ekf_initial,
                'ekf_percentage_change': (
                    (ekf_final - ekf_initial) / ekf_initial * 100.0
                    if ekf_initial > RELATIVE_EPSILON_MPS else None
                ),
                'elapsed_session_time_s': (
                    last['command_start_timestamp_ns']
                    - first['command_start_timestamp_ns']
                ) / 1e9,
            })
    return results


def direction_summary(segments, tolerance):
    points = aggregate_segments(segments, tolerance)
    minimum = minimum_sustainable_speed(segments)
    fit, target = fit_and_target(points, minimum)
    raw_deadband_points = [
        {
            'normalized_throttle': abs(item['command_median']),
            'sustained_motion': item['sustained_motion'],
        }
        for item in segments
        if item['deadband_candidate'] or item['sustained_motion']
    ]
    return {
        'aggregated_points': points,
        'deadband': deadband_summary(raw_deadband_points),
        'minimum_sustainable_speed': minimum,
        'target_speed_0_3': target,
        'fit': fit,
    }


def analyze_samples(
    command_samples,
    encoder_samples,
    state_samples,
    ekf_samples,
    settings,
):
    candidates = segment_commands(
        command_samples,
        settings['command_tolerance'],
        settings['minimum_segment_duration_s'],
        settings['settling_duration_s'],
        settings['zero_command_tolerance'],
        settings['command_gap_limit_s'],
        settings['minimum_neutral_duration_s'],
    )
    segments = []
    for candidate in candidates:
        if not candidate['preceded_by_zero']:
            continue
        pre_run_states = samples_between(
            state_samples,
            candidate['neutral_start_ns'],
            candidate['neutral_end_ns'],
        )
        latest_pre_run_state = (
            max(pre_run_states, key=lambda item: item['time_ns'])
            if pre_run_states else None
        )
        if (
            latest_pre_run_state is None
            or not latest_pre_run_state['stationary']
            or (
                candidate['start_ns'] - latest_pre_run_state['time_ns']
            ) / 1e9 > settings['command_gap_limit_s']
        ):
            continue
        result = segment_metrics(
            candidate,
            encoder_samples,
            state_samples,
            ekf_samples,
            settings['motion_speed_threshold_mps'],
            settings['minimum_samples'],
            settings['metres_per_edge'],
        )
        if result is not None:
            result['index'] = len(segments) + 1
            segments.append(result)
    forward = [item for item in segments if item['direction'] == 'forward']
    reverse = [item for item in segments if item['direction'] == 'reverse']
    return {
        'segments': segments,
        'forward': direction_summary(forward, settings['command_tolerance']),
        'reverse': direction_summary(reverse, settings['command_tolerance']),
        'pack_droop_proxy': pack_droop_pairs(
            segments, settings['command_tolerance']
        ),
    }


def decode_bag(bag_path, window_start, window_end):
    loaded = localization.load_bag_messages(
        bag_path, retained_topics=REQUIRED_TOPICS
    )
    missing = [
        topic for topic in REQUIRED_TOPICS
        if topic not in loaded['topic_types']
    ]
    if missing:
        raise RuntimeError('missing required topics: ' + ', '.join(missing))
    window = localization.resolve_analysis_window(
        loaded, window_start, window_end
    )
    records = localization.messages_in_receive_window(
        loaded['messages'],
        window['window_start_ns'],
        window['window_end_ns'],
    )
    if not records:
        raise RuntimeError('selected analysis window contains no messages')

    message_types = {
        topic: get_message(loaded['topic_types'][topic])
        for topic in REQUIRED_TOPICS
    }
    decoded = {topic: [] for topic in REQUIRED_TOPICS}
    stamp_sources = {topic: defaultdict(int) for topic in REQUIRED_TOPICS}
    for order, (topic, serialized, receive_ns) in enumerate(records):
        message = deserialize_message(serialized, message_types[topic])
        chosen_ns, source = timestamp_ns(message, receive_ns, topic)
        if chosen_ns < 0:
            raise RuntimeError(f'{topic} contains an invalid negative timestamp')
        stamp_sources[topic][source] += 1
        item = {
            'time_ns': chosen_ns,
            'receive_ns': receive_ns,
            'order': order,
        }
        if topic == '/cmd_vel':
            item['value'] = float(message.linear.x)
        elif topic == '/wheel/encoder_state':
            item.update({
                'stationary': bool(message.stationary),
                'edge_rate': float(message.edge_rate),
            })
        elif topic in ('/wheel/odom', '/odometry/filtered'):
            item['value'] = float(message.twist.twist.linear.x)
        decoded[topic].append(item)

    empty_topics = [topic for topic, items in decoded.items() if not items]
    if empty_topics:
        raise RuntimeError(
            'selected analysis window has no messages for: '
            + ', '.join(empty_topics)
        )
    return loaded, window, decoded, {
        topic: dict(sources) for topic, sources in stamp_sources.items()
    }


def default_settings(args):
    return {
        'command_tolerance': args.command_tolerance,
        'minimum_segment_duration_s': args.minimum_segment_duration,
        'settling_duration_s': args.settling_duration,
        'zero_command_tolerance': args.zero_command_tolerance,
        'motion_speed_threshold_mps': args.motion_speed_threshold,
        'minimum_samples': args.minimum_samples,
        'command_gap_limit_s': COMMAND_GAP_LIMIT_S,
        'minimum_neutral_duration_s': MINIMUM_NEUTRAL_DURATION_S,
        'metres_per_edge': METRES_PER_EDGE,
        'sustained_motion_max_stationary_fraction': 0.20,
        'encoder_ekf_absolute_disagreement_mps': ABSOLUTE_DISAGREEMENT_MPS,
        'encoder_ekf_relative_disagreement_percent': (
            RELATIVE_DISAGREEMENT * 100.0
        ),
        'relative_difference_epsilon_mps': RELATIVE_EPSILON_MPS,
        'aggregation_weighting': 'analyzed_duration',
    }


def analyze_bag(args):
    loaded, window, decoded, stamp_sources = decode_bag(
        args.bag, args.window_start, args.window_end
    )
    settings = default_settings(args)
    analysis = analyze_samples(
        decoded['/cmd_vel'],
        decoded['/wheel/odom'],
        decoded['/wheel/encoder_state'],
        decoded['/odometry/filtered'],
        settings,
    )
    if not analysis['segments']:
        raise RuntimeError('no usable constant-throttle segments exist')
    topic_summary = {
        topic: {
            **timestamp_diagnostics(decoded[topic]),
            'timestamp_sources': stamp_sources[topic],
        }
        for topic in REQUIRED_TOPICS
    }
    warnings = []
    for topic, summary in topic_summary.items():
        if summary['non_monotonic_samples']:
            warnings.append(f'{topic}: non-monotonic timestamps detected')
        if summary['duplicate_timestamps']:
            warnings.append(f'{topic}: duplicate timestamps detected')
        if summary['large_gap_count']:
            warnings.append(f'{topic}: large data gaps detected')
        if 'bag_receive_fallback' in summary['timestamp_sources']:
            warnings.append(f'{topic}: invalid/zero headers used receive-time fallback')
    for direction in ('forward', 'reverse'):
        if not analysis[direction]['fit']['monotonic']:
            warnings.append(f'{direction}: materially non-monotonic response')
        if not [
            item for item in analysis['segments']
            if item['direction'] == direction
        ]:
            warnings.append(f'{direction}: no usable segments')
    if not analysis['pack_droop_proxy']:
        warnings.append(
            'pack-droop proxy unavailable: no repeated moving low-throttle levels'
        )
    if loaded['truncated']:
        warnings.append('bag was truncated; MCAP recovery was used')
    warnings.extend(
        f'segment {segment["index"]}: {warning}'
        for segment in analysis['segments']
        for warning in segment['warnings']
    )
    return {
        'metadata': {
            'bag': str(args.bag),
            'bag_start_timestamp_ns': window['bag_start_ns'],
            'bag_end_timestamp_ns': window['bag_end_ns'],
            'bag_duration_s': window['bag_duration_s'],
            'window_start_s': window['window_start_s'],
            'window_end_s': window['window_end_s'],
            'window_start_timestamp_ns': window['window_start_ns'],
            'window_end_timestamp_ns': window['window_end_ns'],
            'timestamp_policy': {
                '/cmd_vel': 'bag receive timestamp (Twist has no header)',
                '/wheel/encoder_state': 'message stamp; receive fallback if zero',
                '/wheel/odom': 'header stamp; receive fallback if zero',
                '/odometry/filtered': 'header stamp; receive fallback if zero',
                '/tf': 'bag receive timestamp for coverage/rate diagnostics',
                '/scan': 'header stamp; receive fallback if zero',
            },
            'metres_per_edge_source': (
                'runner_encoder default project metadata; diagnostic only'
            ),
            'recovery_used': loaded['recovery_used'],
        },
        'settings': settings,
        'topic_summary': topic_summary,
        **analysis,
        'warnings': warnings,
    }


def finite_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_json(item) for item in value]
    return value


def fmt(value, digits=3):
    return 'n/a' if value is None else f'{value:.{digits}f}'


def print_direction_table(label, summary):
    print(f'\n{label} throttle-to-speed table')
    print(' throttle  duration  enc mean±sd  EKF mean  |difference|  motion  repeat')
    for point in summary['aggregated_points']:
        repeat = point['repeat_group'] or '-'
        print(
            f' {point["normalized_throttle"]:8.3f}'
            f' {point["analyzed_duration_s"]:8.2f}'
            f' {point["encoder_mean_speed_magnitude_mps"]:7.3f}'
            f'±{point["encoder_standard_deviation_mps"]:5.3f}'
            f' {point["ekf_mean_speed_magnitude_mps"]:8.3f}'
            f' {point["encoder_ekf_absolute_difference_mps"]:12.3f}'
            f' {str(point["sustained_motion"]):>7}'
            f' {str(repeat):>7}'
        )


def print_text(result):
    print('Throttle response analysis')
    print('\nBag and window')
    metadata = result['metadata']
    print(f'Bag: {metadata["bag"]}')
    print(
        f'Window: {metadata["window_start_s"]:.3f} to '
        f'{metadata["window_end_s"]:.3f} s bag-relative'
    )
    print('\nTopic coverage and rates')
    for topic, summary in result['topic_summary'].items():
        print(
            f'{topic}: {summary["count"]} messages, '
            f'{fmt(summary["rate_hz"])} Hz, '
            f'duplicates={summary["duplicate_timestamps"]}, '
            f'non-monotonic={summary["non_monotonic_samples"]}, '
            f'large-gaps={summary["large_gap_count"]}'
        )
    print('\nSegmentation settings')
    settings = result['settings']
    print(
        f'command tolerance={settings["command_tolerance"]:.3f}, '
        f'minimum duration={settings["minimum_segment_duration_s"]:.1f} s, '
        f'settling discard={settings["settling_duration_s"]:.1f} s, '
        f'motion threshold={settings["motion_speed_threshold_mps"]:.3f} m/s'
    )
    print(
        'Rules: encoder-confirmed stop and >=2.0 s neutral before each run; '
        'same nonzero sign; command band width <= 2*tolerance; '
        f'maximum command gap {settings["command_gap_limit_s"]:.1f} s; '
        'zero always separates runs.'
    )
    print('\nPer-segment results')
    print(' idx dir      cmd    raw/tail  enc mean±sd    EKF mean  stationary motion')
    for item in result['segments']:
        print(
            f' {item["index"]:3d} {item["direction"]:7s}'
            f' {item["command_median"]:+.3f}'
            f' {item["raw_segment_duration_s"]:5.1f}/'
            f'{item["analyzed_duration_s"]:4.1f}'
            f' {item["encoder_mean_signed_mps"]:+7.3f}'
            f'±{item["encoder_standard_deviation_mps"]:5.3f}'
            f' {item["ekf_mean_signed_mps"]:+8.3f}'
            f' {item["stationary_fraction"]:10.2%}'
            f' {str(item["sustained_motion"]):>6}'
        )
    print_direction_table('Forward', result['forward'])
    print_direction_table('Reverse', result['reverse'])
    print('\nDeadband and minimum sustainable speed')
    for direction in ('forward', 'reverse'):
        summary = result[direction]
        deadband = summary['deadband']
        minimum = summary['minimum_sustainable_speed']
        print(
            f'{direction}: no-motion max='
            f'{fmt(deadband["highest_tested_no_sustained_motion"])}; '
            f'motion min={fmt(deadband["lowest_tested_sustained_motion"])}; '
            f'minimum sustainable speed='
            f'{fmt(minimum["encoder_mean_speed_magnitude_mps"] if minimum else None)} m/s'
        )
    print('\n0.3 m/s feasibility')
    for direction in ('forward', 'reverse'):
        target = result[direction]['target_speed_0_3']
        print(
            f'{direction}: {target["status"]}; throttle='
            f'{fmt(target["estimated_normalized_throttle"])}; '
            f'below sustainable floor={target["below_minimum_sustainable_speed"]}'
        )
    print('\nCurve fits')
    for direction in ('forward', 'reverse'):
        fit = result[direction]['fit']
        print(
            f'{direction}: {fit["type"]}, points={fit["input_point_count"]}, '
            f'monotonic={fit["monotonic"]}, RMSE='
            f'{fmt(fit["residual_rmse_mps"])} m/s'
        )
    print('\nEncoder versus EKF agreement')
    for direction in ('forward', 'reverse'):
        items = [
            item for item in result['segments']
            if item['direction'] == direction
        ]
        bias = mean([
            item['encoder_minus_ekf_signed_mps'] for item in items
        ])
        flagged = [item['index'] for item in items if item['encoder_ekf_disagreement']]
        print(f'{direction}: signed mean bias={fmt(bias)} m/s; flagged={flagged}')
    print('\nPack-droop proxy')
    if not result['pack_droop_proxy']:
        print('Unavailable: repeated low moving levels were not identified.')
    for item in result['pack_droop_proxy']:
        print(
            f'{item["direction"]} throttle '
            f'{item["normalized_throttle"]:.3f}: segments '
            f'{item["first_segment_index"]}->{item["last_segment_index"]}, '
            f'encoder change={fmt(item["encoder_percentage_change"], 1)}%'
        )
    print('\nWarnings and limitations')
    for warning in result['warnings']:
        print(f'- {warning}')
    print(
        '- Pack voltage was not measured; repeat differences are session drift, '
        'not a direct voltage measurement.'
    )


def positive_finite(parser, name, value, allow_zero=False):
    if not math.isfinite(value) or value < 0 or (not allow_zero and value == 0):
        qualifier = 'non-negative' if allow_zero else 'positive'
        parser.error(f'{name} must be a finite, {qualifier} value')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Segment constant normalized /cmd_vel.linear.x runs in a ROS 2 '
            'MCAP bag and compare encoder-derived and EKF ground speed.'
        )
    )
    parser.add_argument('bag', type=Path, metavar='BAG', help='MCAP file or bag directory')
    parser.add_argument('--window-start', type=float, default=0.0, metavar='SECONDS',
                        help='analysis-window start, seconds from bag start (default: 0)')
    parser.add_argument('--window-end', type=float, metavar='SECONDS',
                        help='analysis-window end, seconds from bag start (default: bag end)')
    parser.add_argument('--format', choices=('text', 'json'), default='text',
                        help='output format (default: text)')
    parser.add_argument('--command-tolerance', type=float, default=0.005,
                        metavar='NORMALIZED',
                        help='maximum normalized command deviation within a segment (default: 0.005)')
    parser.add_argument('--minimum-segment-duration', type=float, default=6.0,
                        metavar='SECONDS',
                        help='minimum raw held-command duration (default: 6.0 s)')
    parser.add_argument('--settling-duration', type=float, default=2.0,
                        metavar='SECONDS',
                        help='discarded seconds at each segment head (default: 2.0)')
    parser.add_argument('--zero-command-tolerance', type=float, default=0.005,
                        metavar='NORMALIZED',
                        help='absolute normalized command treated as neutral (default: 0.005)')
    parser.add_argument('--motion-speed-threshold', type=float, default=0.03,
                        metavar='M/S',
                        help='minimum retained encoder mean magnitude for sustained motion (default: 0.03 m/s)')
    parser.add_argument('--minimum-samples', type=int, default=2, metavar='COUNT',
                        help='minimum retained samples from encoder, state, and EKF (default: 2)')
    args = parser.parse_args(argv)
    positive_finite(parser, '--window-start', args.window_start, allow_zero=True)
    if args.window_end is not None:
        positive_finite(parser, '--window-end', args.window_end)
        if args.window_end <= args.window_start:
            parser.error('--window-end must be greater than --window-start')
    positive_finite(parser, '--command-tolerance', args.command_tolerance)
    positive_finite(
        parser, '--minimum-segment-duration', args.minimum_segment_duration
    )
    positive_finite(parser, '--settling-duration', args.settling_duration, allow_zero=True)
    if args.settling_duration >= args.minimum_segment_duration:
        parser.error('--settling-duration must be shorter than --minimum-segment-duration')
    positive_finite(parser, '--zero-command-tolerance', args.zero_command_tolerance, allow_zero=True)
    positive_finite(parser, '--motion-speed-threshold', args.motion_speed_threshold)
    if args.minimum_samples < 1:
        parser.error('--minimum-samples must be at least 1')
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        result = analyze_bag(args)
    except Exception as error:
        if args.format == 'json':
            print(f'error: {error}', file=sys.stderr)
        else:
            print(f'error: {error}', file=sys.stderr)
        return 1
    if args.format == 'json':
        json.dump(finite_json(result), sys.stdout, indent=2, allow_nan=False)
        print()
    else:
        print_text(result)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
