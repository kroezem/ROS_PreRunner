#!/usr/bin/env python3
"""Analyze Runner fixed-throttle longitudinal profiles with RF2O reference."""

import argparse
import bisect
import csv
import json
import math
import statistics
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import analyze_localization_bag as localization


TOPICS = ('/cmd_vel', '/wheel/odom', '/odom_rf2o')
COMMAND_TOLERANCE = 0.003
COMMAND_GAP_S = 0.30
ZERO_TOLERANCE = 0.005
MIN_SEGMENT_S = 1.0
MIN_STEADY_TAIL_S = 1.0
MIN_SETTLING_S = 0.5
MAX_STEADY_SLOPE_MPS2 = 0.15
MAX_RF2O_INTERPOLATION_GAP_S = 0.25
STOP_SPEED_MPS = 0.03


def mean(values):
    return statistics.fmean(values) if values else None


def median(values):
    return statistics.median(values) if values else None


def sample_std(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0 if values else None


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (index - lower) * (ordered[upper] - ordered[lower])


def linear_fit_xy(points):
    """Return ordinary least-squares y = slope*x + intercept metrics."""
    if len(points) < 2:
        return None
    x_mean = mean([point[0] for point in points])
    y_mean = mean([point[1] for point in points])
    denominator = sum((x - x_mean) ** 2 for x, _ in points)
    if denominator <= 0.0:
        return None
    slope = sum((x - x_mean) * (y - y_mean) for x, y in points) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [y - (slope * x + intercept) for x, y in points]
    ss_res = sum(value * value for value in residuals)
    ss_tot = sum((y - y_mean) ** 2 for _, y in points)
    return {
        'slope': slope,
        'intercept': intercept,
        'rmse': math.sqrt(ss_res / len(points)),
        'max_abs_residual': max(abs(value) for value in residuals),
        'r_squared': 1.0 - ss_res / ss_tot if ss_tot > 0.0 else None,
        'residuals': residuals,
    }


def interpolate(samples, timestamp_ns, max_gap_s=None):
    """Linearly interpolate sorted ``(timestamp_ns, value, receive_ns)`` samples."""
    times = [item[0] for item in samples]
    right = bisect.bisect_left(times, timestamp_ns)
    if right == 0 or right == len(samples):
        return None, None
    before, after = samples[right - 1], samples[right]
    gap_s = (after[0] - before[0]) / 1e9
    if max_gap_s is not None and gap_s > max_gap_s:
        return None, gap_s
    if after[0] == before[0]:
        return after[1], gap_s
    fraction = (timestamp_ns - before[0]) / (after[0] - before[0])
    return before[1] + fraction * (after[1] - before[1]), gap_s


def previous_value(samples, timestamp_ns):
    times = [item[0] for item in samples]
    index = bisect.bisect_right(times, timestamp_ns) - 1
    return samples[index][1] if index >= 0 else None


def decode_bag(bag_path):
    loaded = localization.load_bag_messages(bag_path, retained_topics=TOPICS)
    missing = [topic for topic in TOPICS if topic not in loaded['topic_types']]
    if missing:
        raise RuntimeError('missing required topics: ' + ', '.join(missing))
    types = {topic: get_message(loaded['topic_types'][topic]) for topic in TOPICS}
    samples = {topic: [] for topic in TOPICS}
    frames = {topic: set() for topic in TOPICS if topic != '/cmd_vel'}
    for topic, serialized, receive_ns in loaded['messages']:
        message = deserialize_message(serialized, types[topic])
        if topic == '/cmd_vel':
            timestamp_ns = receive_ns
            value = float(message.linear.x)
        else:
            timestamp_ns = localization.stamp_to_ns(message.header.stamp) or receive_ns
            value = float(message.twist.twist.linear.x)
            frames[topic].add((message.header.frame_id, message.child_frame_id))
        samples[topic].append((timestamp_ns, value, receive_ns))
    for values in samples.values():
        values.sort(key=lambda item: item[0])
    return loaded, samples, frames


def command_segments(command_samples):
    segments = []
    current = []
    for sample in command_samples:
        representative = median([item[1] for item in current]) if current else sample[1]
        if current and (
            (sample[0] - current[-1][0]) / 1e9 > COMMAND_GAP_S
            or abs(sample[1] - representative) > COMMAND_TOLERANCE
        ):
            segments.append(current)
            current = []
        current.append(sample)
    if current:
        segments.append(current)
    results = []
    for samples in segments:
        start_ns, end_ns = samples[0][0], samples[-1][0]
        command = median([item[1] for item in samples])
        duration_s = (end_ns - start_ns) / 1e9
        if abs(command) > ZERO_TOLERANCE and duration_s >= MIN_SEGMENT_S:
            results.append({
                'start_ns': start_ns,
                'end_ns': end_ns,
                'duration_s': duration_s,
                'command': command,
                'command_count': len(samples),
            })
    return results


def values_between(samples, start_ns, end_ns):
    return [item for item in samples if start_ns <= item[0] <= end_ns]


def trend_metrics(samples):
    if len(samples) < 2:
        return None
    origin = samples[0][0]
    fit = linear_fit_xy([((item[0] - origin) / 1e9, item[1]) for item in samples])
    midpoint = (samples[0][0] + samples[-1][0]) // 2
    first = [item[1] for item in samples if item[0] < midpoint]
    second = [item[1] for item in samples if item[0] >= midpoint]
    return {
        'slope_mps2': fit['slope'],
        'half_median_delta_mps': median(second) - median(first),
        'median_mps': median([item[1] for item in samples]),
    }


def find_steady_tail(segment, encoder_samples):
    """Find earliest tail satisfying a documented, direction-aware trend test."""
    start_ns, end_ns = segment['start_ns'], segment['end_ns']
    earliest_ns = start_ns + round(MIN_SETTLING_S * 1e9)
    latest_ns = end_ns - round(MIN_STEADY_TAIL_S * 1e9)
    candidate_ns = earliest_ns
    while candidate_ns <= latest_ns:
        values = values_between(encoder_samples, candidate_ns, end_ns)
        metrics = trend_metrics(values)
        if metrics is not None and len(values) >= 15:
            directional_median = math.copysign(1.0, segment['command']) * metrics['median_mps']
            allowed_delta = max(0.05, 0.05 * abs(metrics['median_mps']))
            if (
                directional_median > STOP_SPEED_MPS
                and abs(metrics['slope_mps2']) <= MAX_STEADY_SLOPE_MPS2
                and abs(metrics['half_median_delta_mps']) <= allowed_delta
            ):
                return candidate_ns, metrics
        candidate_ns += 50_000_000
    return None, None


def summarize_plateaus(segments, samples, bag_start_ns):
    results = []
    for index, segment in enumerate(segments, 1):
        steady_start_ns, settling = find_steady_tail(segment, samples['/wheel/odom'])
        classification = 'steady' if steady_start_ns is not None else 'transient'
        analysis_start_ns = steady_start_ns or segment['start_ns']
        encoder = values_between(samples['/wheel/odom'], analysis_start_ns, segment['end_ns'])
        rf2o = values_between(samples['/odom_rf2o'], analysis_start_ns, segment['end_ns'])
        sign = 1.0 if segment['command'] > 0 else -1.0
        result = {
            'index': index,
            'direction': 'forward' if sign > 0 else 'reverse',
            'command': segment['command'],
            'start_s': (segment['start_ns'] - bag_start_ns) / 1e9,
            'raw_duration_s': segment['duration_s'],
            'classification': classification,
            'settling_time_s': (
                (analysis_start_ns - segment['start_ns']) / 1e9
                if steady_start_ns is not None else None
            ),
            'analyzed_duration_s': (segment['end_ns'] - analysis_start_ns) / 1e9,
            'command_count': segment['command_count'],
            'encoder_count': len(encoder),
            'encoder_mean_mps': mean([item[1] for item in encoder]),
            'encoder_median_mps': median([item[1] for item in encoder]),
            'encoder_std_mps': sample_std([item[1] for item in encoder]),
            'rf2o_count': len(rf2o),
            'rf2o_mean_mps': mean([item[1] for item in rf2o]),
            'rf2o_median_mps': median([item[1] for item in rf2o]),
            'rf2o_std_mps': sample_std([item[1] for item in rf2o]),
            'tail_encoder_slope_mps2': settling['slope_mps2'] if settling else None,
            'tail_half_median_delta_mps': (
                settling['half_median_delta_mps'] if settling else None
            ),
            '_start_ns': segment['start_ns'],
            '_end_ns': segment['end_ns'],
            '_analysis_start_ns': analysis_start_ns,
            '_sign': sign,
        }
        aligned_differences = []
        for timestamp_ns, rf2o_value, _ in rf2o:
            encoder_value, _ = interpolate(samples['/wheel/odom'], timestamp_ns)
            if encoder_value is not None:
                aligned_differences.append(sign * (encoder_value - rf2o_value))
        result.update({
            'encoder_minus_rf2o_mean_mps': mean(aligned_differences),
            'encoder_minus_rf2o_median_mps': median(aligned_differences),
            'encoder_rf2o_rmse_mps': (
                math.sqrt(mean([value * value for value in aligned_differences]))
                if aligned_differences else None
            ),
            'encoder_rf2o_max_abs_mps': (
                max(abs(value) for value in aligned_differences)
                if aligned_differences else None
            ),
            'encoder_rf2o_p95_abs_mps': percentile(
                [abs(value) for value in aligned_differences], 0.95
            ),
            'aligned_count': len(aligned_differences),
        })
        results.append(result)
    return results


def fit_models(plateaus):
    steady = [item for item in plateaus if item['classification'] == 'steady']
    cohorts = {
        'symmetric': steady,
        'forward': [item for item in steady if item['direction'] == 'forward'],
        'reverse': [item for item in steady if item['direction'] == 'reverse'],
    }
    results = []
    for name, cohort in cohorts.items():
        points = [(abs(item['encoder_median_mps']), abs(item['command'])) for item in cohort]
        fit = linear_fit_xy(points)
        if fit is None:
            continue
        speed_residuals = [
            speed - (command - fit['intercept']) / fit['slope']
            for speed, command in points
        ]
        results.append({
            'model': name,
            'point_count': len(points),
            'speed_min_mps': min(point[0] for point in points),
            'speed_max_mps': max(point[0] for point in points),
            'effort_per_speed': fit['slope'],
            'effort_intercept': fit['intercept'],
            'command_rmse': fit['rmse'],
            'command_max_abs_residual': fit['max_abs_residual'],
            'speed_rmse_mps': math.sqrt(mean([value * value for value in speed_residuals])),
            'speed_max_abs_residual_mps': max(abs(value) for value in speed_residuals),
            'r_squared': fit['r_squared'],
        })
    for effort_per_speed, intercept, name in (
        (0.1188, 0.0174, 'existing_feedforward'),
    ):
        points = [
            (abs(item['encoder_median_mps']), abs(item['command'])) for item in steady
        ]
        command_residuals = [
            command - (effort_per_speed * speed + intercept) for speed, command in points
        ]
        speed_residuals = [
            speed - (command - intercept) / effort_per_speed for speed, command in points
        ]
        results.append({
            'model': name,
            'point_count': len(points),
            'speed_min_mps': min(point[0] for point in points),
            'speed_max_mps': max(point[0] for point in points),
            'effort_per_speed': effort_per_speed,
            'effort_intercept': intercept,
            'command_rmse': math.sqrt(mean([value * value for value in command_residuals])),
            'command_max_abs_residual': max(abs(value) for value in command_residuals),
            'speed_rmse_mps': math.sqrt(mean([value * value for value in speed_residuals])),
            'speed_max_abs_residual_mps': max(abs(value) for value in speed_residuals),
            'r_squared': None,
        })
    return results


def find_stop(samples, zero_ns, end_ns, sign):
    values = [item for item in samples if zero_ns <= item[0] <= end_ns]
    for item in values:
        if sign * item[1] <= STOP_SPEED_MPS:
            return item[0]
    return None


def integrate_directional(samples, start_ns, end_ns, sign, initial_speed):
    points = [(item[0], max(0.0, sign * item[1])) for item in samples if start_ns <= item[0] <= end_ns]
    if initial_speed is not None and (not points or points[0][0] > start_ns):
        points.insert(0, (start_ns, max(0.0, initial_speed)))
    return sum(
        0.5 * (left[1] + right[1]) * (right[0] - left[0]) / 1e9
        for left, right in zip(points, points[1:])
    )


def braking_events(segments, samples, bag_start_ns):
    commands = samples['/cmd_vel']
    command_times = [item[0] for item in commands]
    events = []
    detail_rows = []
    for index, segment in enumerate(segments, 1):
        after = bisect.bisect_right(command_times, segment['end_ns'])
        zero_sample = next(
            (item for item in commands[after:] if abs(item[1]) <= ZERO_TOLERANCE), None
        )
        if zero_sample is None or (zero_sample[0] - segment['end_ns']) / 1e9 > 0.35:
            continue
        zero_ns = zero_sample[0]
        next_nonzero = next(
            (item for item in commands[after:] if item[0] > zero_ns and abs(item[1]) > ZERO_TOLERANCE),
            None,
        )
        event_end_ns = min(
            zero_ns + 4_000_000_000,
            samples['/wheel/odom'][-1][0],
            samples['/odom_rf2o'][-1][0],
        )
        sign = 1.0 if segment['command'] > 0 else -1.0
        row = {
            'index': index,
            'direction': 'forward' if sign > 0 else 'reverse',
            'command': segment['command'],
            'zero_command_s': (zero_ns - bag_start_ns) / 1e9,
            'zero_command_duration_s': (
                (next_nonzero[0] - zero_ns) / 1e9 if next_nonzero else None
            ),
        }
        for label, topic in (('encoder', '/wheel/odom'), ('rf2o', '/odom_rf2o')):
            before = [
                sign * item[1] for item in samples[topic]
                if zero_ns - 250_000_000 <= item[0] <= zero_ns
            ]
            initial = median(before)
            stop_ns = find_stop(samples[topic], zero_ns, event_end_ns, sign)
            fit_end_ns = stop_ns or event_end_ns
            decay = [
                ((item[0] - zero_ns) / 1e9, sign * item[1])
                for item in samples[topic]
                if zero_ns <= item[0] <= fit_end_ns
                and initial is not None and 0.10 * initial <= sign * item[1] <= 1.10 * initial
            ]
            fit = linear_fit_xy(decay)
            row.update({
                f'{label}_initial_mps': initial,
                f'{label}_stop_time_s': (
                    (stop_ns - zero_ns) / 1e9 if stop_ns is not None else None
                ),
                f'{label}_distance_m': (
                    integrate_directional(
                        samples[topic], zero_ns, stop_ns, sign, initial
                    )
                    if stop_ns is not None else None
                ),
                f'{label}_linear_decel_mps2': -fit['slope'] if fit else None,
                f'{label}_average_decel_mps2': (
                    initial / ((stop_ns - zero_ns) / 1e9)
                    if stop_ns is not None and initial is not None and stop_ns > zero_ns else None
                ),
            })
        stop_candidates = [
            row.get('encoder_stop_time_s'), row.get('rf2o_stop_time_s')
        ]
        comparison_duration_s = max(
            [value for value in stop_candidates if value is not None] or [1.5]
        )
        comparison_end_ns = zero_ns + round(comparison_duration_s * 1e9)
        rf_event = values_between(samples['/odom_rf2o'], zero_ns, comparison_end_ns)
        rf_gaps = [
            (right[0] - left[0]) / 1e9 for left, right in zip(rf_event, rf_event[1:])
        ]
        rf_receive_ages = [
            (item[2] - item[0]) / 1e9 for item in rf_event
        ]
        row['rf2o_max_gap_s'] = max(rf_gaps) if rf_gaps else None
        row['rf2o_max_receive_age_s'] = (
            max(rf_receive_ages) if rf_receive_ages else None
        )
        row['rf2o_stale_affected'] = bool(
            (rf_gaps and max(rf_gaps) > 0.20)
            or (rf_receive_ages and max(rf_receive_ages) > 0.25)
        )
        divergences = []
        moving_divergences = []
        for timestamp_ns, rf_value, receive_ns in rf_event:
            encoder_value, _ = interpolate(samples['/wheel/odom'], timestamp_ns)
            if encoder_value is None:
                continue
            encoder_mag, rf_mag = sign * encoder_value, sign * rf_value
            difference = encoder_mag - rf_mag
            divergences.append(difference)
            if rf_mag > 0.10:
                moving_divergences.append(difference)
            detail_rows.append({
                'event_index': index,
                'time_from_brake_s': (timestamp_ns - zero_ns) / 1e9,
                'encoder_directional_mps': encoder_mag,
                'rf2o_directional_mps': rf_mag,
                'encoder_minus_rf2o_mps': difference,
                'rf2o_receive_age_s': (receive_ns - timestamp_ns) / 1e9,
            })
        row.update({
            'encoder_minus_rf2o_mean_mps': mean(divergences),
            'encoder_minus_rf2o_max_abs_mps': (
                max(abs(value) for value in divergences) if divergences else None
            ),
            'possible_lock_fraction': (
                mean([float(value < -0.10) for value in moving_divergences])
                if moving_divergences else None
            ),
            'possible_wheel_overspeed_fraction': (
                mean([float(value > 0.10) for value in moving_divergences])
                if moving_divergences else None
            ),
        })
        rf_stop = row['rf2o_stop_time_s']
        row['pure_zero_through_rf2o_stop'] = bool(
            rf_stop is not None
            and (next_nonzero is None or next_nonzero[0] >= zero_ns + rf_stop * 1e9)
        )
        events.append(row)
    return events, detail_rows


def cadence_rows(samples):
    rows = []
    for topic in TOPICS:
        values = samples[topic]
        gaps = [(right[0] - left[0]) / 1e9 for left, right in zip(values, values[1:])]
        duration_s = (values[-1][0] - values[0][0]) / 1e9
        receive_ages = [(item[2] - item[0]) / 1e9 for item in values]
        rows.append({
            'topic': topic,
            'count': len(values),
            'rate_hz': (len(values) - 1) / duration_s,
            'median_gap_s': median(gaps),
            'p95_gap_s': percentile(gaps, 0.95),
            'p99_gap_s': percentile(gaps, 0.99),
            'max_gap_s': max(gaps),
            'gaps_over_0_2_s': sum(gap > 0.20 for gap in gaps),
            'median_receive_age_s': median(receive_ages),
            'p95_receive_age_s': percentile(receive_ages, 0.95),
            'max_receive_age_s': max(receive_ages),
        })
    return rows


def aligned_rows(samples, bag_start_ns):
    rows = []
    for timestamp_ns, encoder, _ in samples['/wheel/odom']:
        rf2o, gap_s = interpolate(
            samples['/odom_rf2o'], timestamp_ns, MAX_RF2O_INTERPOLATION_GAP_S
        )
        command = previous_value(samples['/cmd_vel'], timestamp_ns)
        rows.append({
            'time_s': (timestamp_ns - bag_start_ns) / 1e9,
            'command': command,
            'encoder_mps': encoder,
            'rf2o_mps': rf2o,
            'encoder_minus_rf2o_mps': encoder - rf2o if rf2o is not None else None,
            'rf2o_bracket_gap_s': gap_s,
        })
    return rows


def public_rows(rows):
    return [{key: value for key, value in row.items() if not key.startswith('_')} for row in rows]


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def svg_plot(path, series, title, x_label, y_label, width=900, height=520):
    """Write a dependency-free SVG line/scatter plot."""
    points = [point for item in series for point in item['points'] if None not in point]
    if not points:
        return
    xs, ys = [point[0] for point in points], [point[1] for point in points]
    x_min, x_max, y_min, y_max = min(xs), max(xs), min(ys), max(ys)
    x_pad = max((x_max - x_min) * 0.06, 0.01)
    y_pad = max((y_max - y_min) * 0.08, 0.02)
    x_min, x_max, y_min, y_max = x_min - x_pad, x_max + x_pad, y_min - y_pad, y_max + y_pad
    left, top, right, bottom = 80, 48, width - 28, height - 68
    sx = lambda x: left + (x - x_min) / (x_max - x_min) * (right - left)
    sy = lambda y: bottom - (y - y_min) / (y_max - y_min) * (bottom - top)
    colors = ('#2563eb', '#dc2626', '#16a34a', '#9333ea', '#ea580c', '#0891b2')
    chunks = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="25" text-anchor="middle" font-family="sans-serif" font-size="18">{title}</text>',
        f'<line x1="{left}" y1="{bottom}" x2="{right}" y2="{bottom}" stroke="#111"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{bottom}" stroke="#111"/>',
    ]
    for index in range(6):
        x = x_min + index * (x_max - x_min) / 5
        y = y_min + index * (y_max - y_min) / 5
        chunks.extend([
            f'<text x="{sx(x):.1f}" y="{bottom+20}" text-anchor="middle" font-family="sans-serif" font-size="11">{x:.2f}</text>',
            f'<text x="{left-8}" y="{sy(y)+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="11">{y:.2f}</text>',
        ])
    chunks.extend([
        f'<text x="{(left+right)/2}" y="{height-16}" text-anchor="middle" font-family="sans-serif" font-size="13">{x_label}</text>',
        f'<text transform="translate(18 {(top+bottom)/2}) rotate(-90)" text-anchor="middle" font-family="sans-serif" font-size="13">{y_label}</text>',
    ])
    for index, item in enumerate(series):
        color = colors[index % len(colors)]
        valid = [point for point in item['points'] if None not in point]
        if item.get('line', True) and len(valid) > 1:
            coordinates = ' '.join(f'{sx(x):.1f},{sy(y):.1f}' for x, y in valid)
            chunks.append(f'<polyline points="{coordinates}" fill="none" stroke="{color}" stroke-width="1.5"/>')
        for x, y in valid:
            chunks.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="2.5" fill="{color}"/>')
        chunks.extend([
            f'<rect x="{right-170}" y="{top+index*20}" width="12" height="3" fill="{color}"/>',
            f'<text x="{right-152}" y="{top+5+index*20}" font-family="sans-serif" font-size="12">{item["label"]}</text>',
        ])
    chunks.append('</svg>')
    path.write_text('\n'.join(chunks), encoding='utf-8')


def make_plots(output, plateaus, aligned, braking_detail):
    steady = [item for item in plateaus if item['classification'] == 'steady']
    transient = [item for item in plateaus if item['classification'] != 'steady']
    svg_plot(output / 'command_vs_steady_speed.svg', [
        {'label': 'steady encoder', 'points': [(item['command'], item['encoder_median_mps']) for item in steady], 'line': False},
        {'label': 'steady RF2O', 'points': [(item['command'], item['rf2o_median_mps']) for item in steady], 'line': False},
        {'label': 'transient segment median', 'points': [(item['command'], item['encoder_median_mps']) for item in transient], 'line': False},
    ], 'Command versus measured speed', 'normalized command', 'speed (m/s)')
    scatter = [row for row in aligned if row['rf2o_mps'] is not None and abs(row['command'] or 0.0) > ZERO_TOLERANCE]
    svg_plot(output / 'rf2o_vs_encoder.svg', [
        {'label': 'aligned samples', 'points': [(row['encoder_mps'], row['rf2o_mps']) for row in scatter], 'line': False},
        {'label': 'identity', 'points': [(min(row['encoder_mps'] for row in scatter), min(row['encoder_mps'] for row in scatter)), (max(row['encoder_mps'] for row in scatter), max(row['encoder_mps'] for row in scatter))]},
    ], 'RF2O versus encoder', 'encoder speed (m/s)', 'RF2O speed (m/s)')
    chosen = sorted(set(row['event_index'] for row in braking_detail))[-4:]
    series = []
    for event_index in chosen:
        rows = [row for row in braking_detail if row['event_index'] == event_index]
        series.extend([
            {'label': f'E{event_index} encoder', 'points': [(row['time_from_brake_s'], row['encoder_directional_mps']) for row in rows]},
            {'label': f'E{event_index} RF2O', 'points': [(row['time_from_brake_s'], row['rf2o_directional_mps']) for row in rows]},
        ])
    svg_plot(output / 'representative_braking.svg', series, 'Representative high-speed braking', 'time after zero command (s)', 'directional speed (m/s)')
    high_speed = [
        row for row in aligned
        if any(
            item['start_s'] <= row['time_s']
            <= item['start_s'] + item['raw_duration_s']
            for item in transient if abs(item['command']) >= 0.30
        )
    ]
    svg_plot(output / 'high_speed_transients.svg', [
        {
            'label': 'encoder',
            'points': [(row['time_s'], row['encoder_mps']) for row in high_speed],
        },
        {
            'label': 'RF2O',
            'points': [(row['time_s'], row['rf2o_mps']) for row in high_speed],
        },
    ], 'Unsettled 0.30-0.40 transients', 'bag-relative time (s)', 'speed (m/s)')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    loaded, samples, frames = decode_bag(args.bag)
    args.output.mkdir(parents=True, exist_ok=True)
    segments = command_segments(samples['/cmd_vel'])
    plateaus = summarize_plateaus(segments, samples, loaded['bag_start_ns'])
    models = fit_models(plateaus)
    braking, braking_detail = braking_events(segments, samples, loaded['bag_start_ns'])
    cadence = cadence_rows(samples)
    aligned = aligned_rows(samples, loaded['bag_start_ns'])
    write_csv(args.output / 'aligned_timeseries.csv', aligned)
    write_csv(args.output / 'plateaus.csv', public_rows(plateaus))
    write_csv(args.output / 'model_fits.csv', models)
    write_csv(args.output / 'braking_events.csv', braking)
    write_csv(args.output / 'braking_samples.csv', braking_detail)
    write_csv(args.output / 'topic_cadence.csv', cadence)
    make_plots(args.output, plateaus, aligned, braking_detail)
    report = {
        'bag': str(args.bag),
        'duration_s': (loaded['bag_end_ns'] - loaded['bag_start_ns']) / 1e9,
        'frames': {topic: sorted(list(values)) for topic, values in frames.items()},
        'steady_test': {
            'minimum_segment_s': MIN_SEGMENT_S,
            'minimum_settling_s': MIN_SETTLING_S,
            'minimum_steady_tail_s': MIN_STEADY_TAIL_S,
            'maximum_abs_tail_slope_mps2': MAX_STEADY_SLOPE_MPS2,
            'maximum_half_median_delta': 'max(0.05 m/s, 5% of median speed)',
        },
        'plateaus': public_rows(plateaus),
        'model_fits': models,
        'braking_events': braking,
        'topic_cadence': cadence,
    }
    (args.output / 'analysis.json').write_text(
        json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8'
    )
    print(json.dumps({
        'output': str(args.output),
        'steady_plateaus': sum(item['classification'] == 'steady' for item in plateaus),
        'transient_plateaus': sum(item['classification'] != 'steady' for item in plateaus),
        'braking_events': len(braking),
    }))


if __name__ == '__main__':
    main()
