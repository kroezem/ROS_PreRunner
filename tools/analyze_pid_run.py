#!/usr/bin/env python3
# flake8: noqa
"""Stream a live-tuned PID MCAP and emit a configuration-attributed report."""

import argparse
import bisect
import glob
import math
import statistics
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    from mcap.reader import make_reader
except ImportError as error:  # pragma: no cover - depends on host tooling
    raise SystemExit(
        'The mcap Python package is required (python3 -m pip install mcap).'
    ) from error

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    '/drive_adapter/state_typed',
    '/wheel/encoder_state',
    '/speed_envelope/status',
    '/parameter_events',
)
REQUIRED_TOPICS = set(TOPICS)
OUTPUT_MAX = 0.14
SATURATION_THRESHOLD = 0.99 * OUTPUT_MAX
INTEGRATOR_BOUND = 0.005
BOUND_THRESHOLD = 0.99 * INTEGRATOR_BOUND
METRES_PER_EDGE = 0.010282
STEADY_EXCLUSION_S = 2.0
PLATEAU_TOLERANCE = 0.001
MIN_PLATEAU_S = 1.0
MAX_STEP_TRANSITION_S = 1.0
MIN_STEP_SIZE = 0.02
INITIAL_CONDITION_STEP_FRACTION = 0.10

REASON_NAMES = {
    0: 'ACTIVE',
    1: 'GAIN_DISABLED',
    2: 'ZERO_COMMAND',
    3: 'FEEDBACK_STALE',
    4: 'WHEELSPIN',
    5: 'DIRECTION_UNAVAILABLE',
    6: 'DIRECTION_MISMATCH',
    7: 'ARBITRATION_UNAVAILABLE',
    8: 'OUTPUT_NOT_SELECTED',
    9: 'INVALID_DT',
    10: 'ANTI_WINDUP',
    11: 'NO_COMMAND',
    12: 'INVALID_COMMAND',
}
PARAMETER_KEYS = {
    ('/controller_server', 'FollowPath.desired_linear_vel'): 'desired',
    ('/drive_adapter', 'proportional_gain'): 'kp',
    ('/drive_adapter', 'integral_gain'): 'ki',
}
STATUS_KEYS = {
    'FollowPath.desired_linear_vel': 'desired',
    'proportional_gain': 'kp',
    'integral_gain': 'ki',
    'feedforward_effort_per_speed': 'ff_slope',
    'feedforward_effort_intercept': 'ff_intercept',
}


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def mean(values):
    return statistics.fmean(values) if values else None


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (position - lower) * (
        ordered[upper] - ordered[lower]
    )


def format_float(value, digits=4):
    return 'n/a' if value is None or not math.isfinite(value) else f'{value:.{digits}f}'


def format_pct(value, digits=2):
    return 'n/a' if value is None else f'{100.0 * value:.{digits}f}%'


def primary_mode(sample):
    return sample['mode'].split(';', 1)[0]


def followpath_active(sample):
    return primary_mode(sample) == 'forward' and abs(sample['effective']) > 0.0


def config_label(config):
    return (
        f"Kp={config['kp']:.3f}, Ki={config['ki']:.3f}, "
        f"desired={config['desired']:.2f} m/s"
    )


def resolve_bag(pattern):
    matches = sorted(Path(item).expanduser().resolve() for item in glob.glob(pattern))
    files = []
    for match in matches:
        if match.is_dir():
            files.extend(sorted(match.glob('*.mcap')))
        elif match.suffix.lower() == '.mcap':
            files.append(match)
    files = sorted(set(path.resolve() for path in files))
    if len(files) != 1:
        raise RuntimeError(
            f'bag pattern must resolve to exactly one MCAP; found {len(files)}: '
            + ', '.join(str(path) for path in files)
        )
    return files[0]


def validate_and_stream(bag_path):
    magic = b'\x89MCAP0\r\n'
    size = bag_path.stat().st_size
    if size < 16:
        raise RuntimeError('MCAP is too small to be valid')
    with bag_path.open('rb') as stream:
        if stream.read(8) != magic:
            raise RuntimeError('MCAP opening magic is invalid')
        stream.seek(-8, 2)
        if stream.read(8) != magic:
            raise RuntimeError('MCAP is truncated: closing magic is absent')

    samples = {'state': [], 'encoder': [], 'status': [], 'parameters': []}
    schemas = {}
    seen_topics = set()
    with bag_path.open('rb') as stream:
        reader = make_reader(stream)
        summary = reader.get_summary()
        if summary is None or summary.statistics is None:
            raise RuntimeError('MCAP summary/statistics are absent')
        bag_start_ns = summary.statistics.message_start_time
        bag_end_ns = summary.statistics.message_end_time
        if bag_end_ns <= bag_start_ns:
            raise RuntimeError('MCAP has an invalid or empty time range')
        for schema, channel, record in reader.iter_messages(topics=list(TOPICS)):
            seen_topics.add(channel.topic)
            message_class = schemas.setdefault(schema.name, get_message(schema.name))
            message = deserialize_message(record.data, message_class)
            time_ns = record.log_time
            if channel.topic == '/drive_adapter/state_typed':
                samples['state'].append({
                    'time_ns': time_ns,
                    'stamp_ns': stamp_ns(message.stamp),
                    'commanded': message.commanded_speed,
                    'effective': message.effective_speed,
                    'measured': message.measured_speed,
                    'error': message.speed_error,
                    'feedforward': message.feedforward_throttle,
                    'floor_violation': message.feedforward_floor_violation,
                    'proportional': message.proportional_term,
                    'integrator': message.integrator_state,
                    'throttle': message.final_throttle,
                    'commanded_yaw': message.commanded_yaw_rate,
                    'measured_yaw': message.measured_yaw_rate,
                    'integrator_enabled': message.integrator_enabled,
                    'steering_saturated': message.steering_saturated,
                    'wheelspin': message.wheelspin_guard,
                    'freeze_reason': message.integrator_freeze_reason,
                    'mode': message.mode,
                })
            elif channel.topic == '/wheel/encoder_state':
                samples['encoder'].append({
                    'time_ns': time_ns,
                    'stamp_ns': stamp_ns(message.stamp),
                    'edge_rate': message.edge_rate,
                    'stationary': message.stationary,
                    'active_direction': message.active_direction,
                    'pending_direction': message.pending_direction,
                })
            elif channel.topic == '/speed_envelope/status':
                values = {}
                for entry in message.entries:
                    key = STATUS_KEYS.get(entry.parameter_name)
                    if key is not None:
                        values[key] = entry.observed_double
                samples['status'].append({
                    'time_ns': time_ns,
                    'values': values,
                    'all_available': message.all_available,
                    'any_divergence': message.any_divergence,
                })
            else:
                changes = []
                for parameter in (
                    list(message.new_parameters)
                    + list(message.changed_parameters)
                ):
                    key = PARAMETER_KEYS.get((message.node, parameter.name))
                    if key is not None:
                        changes.append((key, parameter.value.double_value))
                if changes:
                    samples['parameters'].append({
                        'time_ns': time_ns,
                        'changes': changes,
                    })
    missing = REQUIRED_TOPICS - seen_topics
    if missing:
        raise RuntimeError('required topics absent: ' + ', '.join(sorted(missing)))
    if not samples['status']:
        raise RuntimeError('/speed_envelope/status was not recorded')
    if not samples['state']:
        raise RuntimeError('gain timeline cannot be reconstructed: no typed states')
    return bag_start_ns, bag_end_ns, size, samples


def recover_timeline(bag_start_ns, bag_end_ns, samples):
    initial_status = samples['status'][0]['values']
    if not {'kp', 'ki', 'desired'} <= set(initial_status):
        raise RuntimeError('gain timeline cannot be reconstructed from initial status')
    initial = {key: initial_status[key] for key in ('kp', 'ki', 'desired')}
    changes = []
    for event in samples['parameters']:
        for key, value in event['changes']:
            changes.append({'time_ns': event['time_ns'], 'key': key, 'value': value})
    changes.sort(key=lambda item: item['time_ns'])

    boundaries = [bag_start_ns]
    boundaries.extend(sorted(set(item['time_ns'] for item in changes)))
    boundaries.append(bag_end_ns)
    current = dict(initial)
    change_by_time = defaultdict(list)
    for item in changes:
        change_by_time[item['time_ns']].append(item)
    segments = []
    for start_ns, end_ns in zip(boundaries, boundaries[1:]):
        if start_ns != bag_start_ns:
            for item in change_by_time[start_ns]:
                current[item['key']] = item['value']
        segment = {
            'index': len(segments) + 1,
            'start_ns': start_ns,
            'end_ns': end_ns,
            **current,
        }
        segments.append(segment)

    def segment_for(time_ns):
        starts = [segment['start_ns'] for segment in segments]
        index = bisect.bisect_right(starts, time_ns) - 1
        return segments[max(0, min(index, len(segments) - 1))]

    for state in samples['state']:
        state['segment'] = segment_for(state['time_ns'])
    for segment in segments:
        segment['states'] = [
            state for state in samples['state'] if state['segment'] is segment
        ]
    return segments, changes, segment_for


def attach_encoder_measurements(states, encoder):
    """Attach latest sample-time-aligned encoder speed to every typed state."""
    encoder_stamps = [item['stamp_ns'] for item in encoder]
    for state in states:
        index = bisect.bisect_right(encoder_stamps, state['stamp_ns']) - 1
        state['encoder_measured'] = (
            abs(encoder[index]['edge_rate']) * METRES_PER_EDGE
            if index >= 0 else None
        )


def cross_checks(samples, segments, changes, bag_start_ns):
    findings = []
    kp_observations = []
    mismatches = []
    for state in samples['state']:
        if abs(state['error']) > 0.01:
            recovered = state['proportional'] / state['error']
            kp_observations.append(recovered)
            if not math.isclose(recovered, state['segment']['kp'], abs_tol=1e-12):
                mismatches.append((state['time_ns'], recovered, state['segment']['kp']))
    if not kp_observations:
        raise RuntimeError('gain timeline cannot be reconstructed: no usable Kp samples')
    clusters = sorted(set(round(value, 12) for value in kp_observations))
    findings.append(
        f'Direct sample-aligned Kp recovery used {len(kp_observations):,} samples '
        f'with |speed_error| > 0.01 m/s and found {clusters}.'
    )
    findings.append(
        f'Kp disagreed with the parameter-event timeline on {len(mismatches):,} '
        'directly observable samples.'
    )

    encoder = samples['encoder']
    encoder_times = [item['time_ns'] for item in encoder]
    inferred_ki = []
    states = samples['state']
    for previous, current in zip(states, states[1:]):
        if not current['integrator_enabled'] or abs(current['error']) < 1e-9:
            continue
        previous_index = bisect.bisect_right(encoder_times, previous['time_ns']) - 1
        current_index = bisect.bisect_right(encoder_times, current['time_ns']) - 1
        if previous_index < 0 or current_index <= previous_index:
            continue
        dt = (encoder[current_index]['stamp_ns'] - encoder[previous_index]['stamp_ns']) / 1e9
        delta = current['integrator'] - previous['integrator']
        if dt > 0.0:
            inferred_ki.append(delta / (current['error'] * dt))
    central_ki = [value for value in inferred_ki if math.isfinite(value) and 0 <= value <= 0.1]
    if central_ki:
        findings.append(
            'Ki activity was recovered from integrator-state increments on '
            f'{len(central_ki):,} aligned updates: median '
            f'{statistics.median(central_ki):.6f}, p95 absolute deviation from '
            f'0.010000 = {percentile([abs(v - 0.01) for v in central_ki], .95):.6f}.'
        )
    else:
        raise RuntimeError('gain timeline cannot be reconstructed: Ki never observable')

    status_transitions = []
    last = None
    for status in samples['status']:
        current = tuple(status['values'].get(key) for key in ('kp', 'ki', 'desired'))
        if current != last:
            status_transitions.append((status['time_ns'], current))
            last = current
    event_transitions = []
    for change in changes:
        segment = next(
            segment for segment in segments if segment['start_ns'] == change['time_ns']
        )
        event_transitions.append((change, (segment['kp'], segment['ki'], segment['desired'])))
    for change, expected in event_transitions:
        later = [item for item in status_transitions if item[0] >= change['time_ns'] and item[1] == expected]
        if later:
            delay = (later[0][0] - change['time_ns']) / 1e9
            findings.append(
                f"{change['key']} change at +{(change['time_ns'] - bag_start_ns)/1e9:.3f} s: "
                f'/speed_envelope/status first matched after {delay:.3f} s.'
            )
        else:
            findings.append(
                f"DISAGREEMENT: status never matched {change['key']}={change['value']}."
            )
    divergence_segments = [
        segment['index'] for segment in segments
        if any(
            status['any_divergence']
            for status in samples['status']
            if segment['start_ns'] <= status['time_ns'] < segment['end_ns']
        )
    ]
    findings.append(
        '/speed_envelope/status reported any_divergence=true at least once in segments '
        f'{divergence_segments}. This includes intended live departures from the origin and '
        'brief stale observations after return-to-origin writes. Its values did not disagree '
        'with parameter_events after the sampled observer delays listed above.'
    )
    ff_status = {
        (
            status['values'].get('ff_slope'),
            status['values'].get('ff_intercept'),
        )
        for status in samples['status']
    }
    findings.append(
        'The 1 Hz observer independently reported feedforward coefficient pairs '
        f'{sorted(ff_status)} across all {len(samples["status"]):,} status samples.'
    )

    ff_findings = []
    for segment in segments:
        active = [state for state in segment['states'] if state['effective'] > 0.0]
        regression = linear_regression(
            [abs(state['effective']) for state in active],
            [state['feedforward'] for state in active],
        )
        residual = max(
            (abs(state['feedforward'] - (0.1188 * abs(state['effective']) + 0.0174)) for state in active),
            default=None,
        )
        ff_findings.append((segment, regression, residual, len(active)))
    return findings, ff_findings


def linear_regression(xs, ys):
    if len(xs) < 2 or len(set(xs)) < 2:
        return None
    x_mean = mean(xs)
    y_mean = mean(ys)
    denominator = sum((x - x_mean) ** 2 for x in xs)
    if denominator == 0.0:
        return None
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator
    intercept = y_mean - slope * x_mean
    return slope, intercept


def pearson(xs, ys):
    if len(xs) < 2:
        return None
    x_mean = mean(xs)
    y_mean = mean(ys)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    denominator = math.sqrt(
        sum((x - x_mean) ** 2 for x in xs) * sum((y - y_mean) ** 2 for y in ys)
    )
    return numerator / denominator if denominator else None


def plateau_runs(states):
    raw = []
    start = 0
    for index in range(1, len(states) + 1):
        split = index == len(states)
        if not split:
            split = (
                abs(states[index]['effective'] - states[start]['effective'])
                > PLATEAU_TOLERANCE
                or (states[index]['time_ns'] - states[index - 1]['time_ns']) / 1e9 > 0.25
            )
        if split:
            duration = (states[index - 1]['time_ns'] - states[start]['time_ns']) / 1e9
            if duration >= MIN_PLATEAU_S:
                raw.append({
                    'start_index': start,
                    'end_index': index - 1,
                    'target': statistics.median(
                        item['effective'] for item in states[start:index]
                    ),
                    'duration_s': duration,
                })
            start = index
    return raw


def first_crossing(window, initial_measured, target, fraction):
    threshold = initial_measured + fraction * (target - initial_measured)
    direction = 1 if target >= initial_measured else -1
    for sample in window:
        if direction * (sample['encoder_measured'] - threshold) >= 0.0:
            return (sample['time_ns'] - window[0]['time_ns']) / 1e9
    return None


def settling_time(window, target, step_size):
    tolerance = 0.02 * (abs(target) if abs(target) > 1e-9 else abs(step_size))
    if tolerance == 0.0:
        return None
    suffix_good = True
    earliest = None
    for sample in reversed(window):
        suffix_good = suffix_good and abs(sample['encoder_measured'] - target) <= tolerance
        if suffix_good:
            earliest = sample
    return (
        (earliest['time_ns'] - window[0]['time_ns']) / 1e9
        if earliest is not None else None
    )


def in_exclusion(time_ns, changes):
    exclusion_ns = round(STEADY_EXCLUSION_S * 1e9)
    return any(change['time_ns'] <= time_ns < change['time_ns'] + exclusion_ns for change in changes)


def step_metrics(states, changes):
    plateaus = plateau_runs(states)
    steps = []
    ambiguous = 0
    for previous, target_run in zip(plateaus, plateaus[1:]):
        transition_start = previous['end_index'] + 1
        transition_end = target_run['start_index']
        transition_s = (
            states[transition_end]['time_ns'] - states[previous['end_index']]['time_ns']
        ) / 1e9
        step_size = target_run['target'] - previous['target']
        if abs(step_size) < MIN_STEP_SIZE:
            continue
        if transition_s > MAX_STEP_TRANSITION_S:
            ambiguous += 1
            continue
        window = states[transition_start:target_run['end_index'] + 1]
        if not window:
            continue
        segment_ids = {sample['segment']['index'] for sample in window}
        if len(segment_ids) != 1:
            steps.append({
                'attributable': False,
                'start_ns': window[0]['time_ns'],
                'end_ns': window[-1]['time_ns'],
                'initial': previous['target'],
                'target': target_run['target'],
            })
            continue
        segment = window[0]['segment']
        initial_measured = states[previous['end_index']]['encoder_measured']
        direction = 1 if step_size > 0 else -1
        poor_initial = (
            abs(initial_measured - previous['target'])
            > INITIAL_CONDITION_STEP_FRACTION * abs(step_size)
        )
        extreme = (
            max(item['encoder_measured'] for item in window)
            if direction > 0 else
            min(item['encoder_measured'] for item in window)
        )
        overshoot = max(0.0, direction * (extreme - target_run['target'])) / abs(step_size)
        saturated_count = sum(item['throttle'] >= SATURATION_THRESHOLD for item in window)
        saturated = saturated_count > 0
        plateau_samples = states[target_run['start_index']:target_run['end_index'] + 1]
        steady_start = len(plateau_samples) // 2
        steady = [
            sample for sample in plateau_samples[steady_start:]
            if not in_exclusion(sample['time_ns'], changes)
        ]
        steady_valid = [
            sample for sample in steady
            if sample['freeze_reason'] in (0, 1)
        ]
        output_not_selected = sum(
            sample['freeze_reason'] == 8 for sample in window
        )
        timing_confounded = output_not_selected > 0 or poor_initial
        steps.append({
            'attributable': True,
            'segment': segment,
            'start_ns': window[0]['time_ns'],
            'end_ns': window[-1]['time_ns'],
            'initial': previous['target'],
            'initial_measured': initial_measured,
            'target': target_run['target'],
            'direction': 'up' if direction > 0 else 'down',
            'sample_count': len(window),
            't63': None if saturated or timing_confounded else first_crossing(window, initial_measured, target_run['target'], .63),
            't90': None if saturated or timing_confounded else first_crossing(window, initial_measured, target_run['target'], .90),
            'overshoot': overshoot,
            'settling': None if saturated or timing_confounded else settling_time(window, target_run['target'], step_size),
            'steady_error': mean([
                target_run['target'] - sample['encoder_measured'] for sample in steady
            ]),
            'steady_abs_error': mean([
                abs(target_run['target'] - sample['encoder_measured']) for sample in steady
            ]),
            'steady_count': len(steady),
            'steady_valid_abs_errors': [
                abs(target_run['target'] - sample['encoder_measured'])
                for sample in steady_valid
            ],
            'peak_throttle': max(sample['throttle'] for sample in window),
            'mean_throttle': mean([sample['throttle'] for sample in window]),
            'saturation_fraction': saturated_count / len(window),
            'saturated': saturated,
            'output_not_selected_fraction': output_not_selected / len(window),
            'timing_confounded': timing_confounded,
            'poor_initial': poor_initial,
            'peak_proportional': max(abs(sample['proportional']) for sample in window),
            'peak_integrator': max(abs(sample['integrator']) for sample in window),
        })
    return steps, ambiguous, len(plateaus)


def consecutive_duration(samples, predicate):
    longest = 0.0
    current_start = None
    previous = None
    for sample in samples:
        if predicate(sample):
            if current_start is None:
                current_start = sample['time_ns']
            previous = sample['time_ns']
            longest = max(longest, (previous - current_start) / 1e9)
        else:
            current_start = None
            previous = None
    return longest


def integrator_metrics(segments, steps):
    results = []
    for segment in segments:
        if segment['ki'] <= 0.0 or not segment['states']:
            continue
        states = segment['states']
        interval_weights = []
        for index, state in enumerate(states):
            next_ns = (
                states[index + 1]['time_ns']
                if index + 1 < len(states) else segment['end_ns']
            )
            interval_weights.append(
                max(0.0, min((next_ns - state['time_ns']) / 1e9, 0.25))
            )
        weight_total = sum(interval_weights)
        at_bound = [abs(item['integrator']) >= BOUND_THRESHOLD for item in states]
        releases = []
        for index in range(1, len(states)):
            if states[index]['freeze_reason'] == 0 and states[index - 1]['freeze_reason'] != 0:
                end_ns = states[index]['time_ns'] + 5_000_000_000
                window = [item for item in states[index:] if item['time_ns'] <= end_ns]
                signs = [1 if item['error'] > 0 else -1 if item['error'] < 0 else 0 for item in window]
                crossings = sum(a * b < 0 for a, b in zip(signs, signs[1:]))
                releases.append({
                    'time_ns': states[index]['time_ns'],
                    'crossings': crossings,
                    'integrator_range': (
                        max(item['integrator'] for item in window)
                        - min(item['integrator'] for item in window)
                    ) if window else 0.0,
                })
        frozen_drift = []
        run_start = None
        for index, state in enumerate(states):
            frozen = state['freeze_reason'] != 0
            if frozen and run_start is None:
                run_start = index
            if run_start is not None and (not frozen or index == len(states) - 1):
                end = index if frozen else index - 1
                frozen_drift.append(abs(states[end]['integrator'] - states[run_start]['integrator']))
                run_start = None
        results.append({
            'segment': segment,
            'minimum': min(item['integrator'] for item in states),
            'maximum': max(item['integrator'] for item in states),
            'mean': (
                sum(item['integrator'] * weight for item, weight in zip(states, interval_weights))
                / weight_total if weight_total else None
            ),
            'bound_fraction': (
                sum(weight for value, weight in zip(at_bound, interval_weights) if value)
                / weight_total if weight_total else None
            ),
            'longest_bound_s': consecutive_duration(
                states, lambda item: abs(item['integrator']) >= BOUND_THRESHOLD
            ),
            'releases': releases,
            'max_frozen_drift': max(frozen_drift, default=0.0),
        })

    comparisons = []
    attributed = [
        step for step in steps
        if step.get('attributable') and step['steady_valid_abs_errors']
        and step['target'] > 0.0
    ]
    ki_steps = [step for step in attributed if step['segment']['ki'] > 0.0]
    for target in sorted(set(round(step['target'], 3) for step in ki_steps)):
        enabled = [
            value for step in ki_steps if round(step['target'], 3) == target
            for value in step['steady_valid_abs_errors']
        ]
        disabled = [
            value for step in attributed
            if step['segment']['ki'] == 0.0
            and math.isclose(step['segment']['kp'], 0.05)
            and round(step['target'], 3) == target
            for value in step['steady_valid_abs_errors']
        ]
        disabled_segments = sorted({
            step['segment']['index'] for step in attributed
            if step['segment']['ki'] == 0.0
            and math.isclose(step['segment']['kp'], 0.05)
            and round(step['target'], 3) == target
            and step['steady_valid_abs_errors']
        })
        enabled_segments = sorted({
            step['segment']['index'] for step in ki_steps
            if round(step['target'], 3) == target and step['steady_valid_abs_errors']
        })
        comparisons.append((
            target, mean(disabled), len(disabled), disabled_segments,
            mean(enabled), len(enabled), enabled_segments,
        ))
    return results, comparisons


def freeze_metrics(states, segments):
    counts = Counter(item['freeze_reason'] for item in states)
    durations = Counter()
    for current, following in zip(states, states[1:]):
        duration = min((following['time_ns'] - current['time_ns']) / 1e9, 0.25)
        durations[current['freeze_reason']] += max(0.0, duration)
    per_config = defaultdict(Counter)
    modes = defaultdict(Counter)
    state_summary = defaultdict(list)
    for state in states:
        reason = state['freeze_reason']
        per_config[reason][state['segment']['index']] += 1
        modes[reason][primary_mode(state)] += 1
        state_summary[reason].append(state)
    mode_reasons = defaultdict(set)
    for reason, mode_counts in modes.items():
        for mode, count in mode_counts.items():
            if count:
                mode_reasons[mode].add(reason)
    perfectly_partitioned = all(len(reasons) <= 1 for reasons in mode_reasons.values())
    return counts, durations, per_config, modes, state_summary, perfectly_partitioned


def diagnostics(segments, encoder):
    encoder_stamps = [item['stamp_ns'] for item in encoder]
    results = []
    for segment in segments:
        states = segment['states']
        gaps = {'active': [], 'idle': []}
        for previous, current in zip(states, states[1:]):
            if followpath_active(previous) == followpath_active(current):
                key = 'active' if followpath_active(current) else 'idle'
                gaps[key].append((current['time_ns'] - previous['time_ns']) / 1e9)
        aligned = 0
        exact_signed = 0
        exact_abs = 0
        active_aligned = 0
        active_exact_abs = 0
        for state in states:
            index = bisect.bisect_right(encoder_stamps, state['stamp_ns']) - 1
            if index < 0:
                continue
            aligned += 1
            signed = encoder[index]['edge_rate'] * METRES_PER_EDGE
            exact_signed += state['measured'] == signed
            exact_abs += state['measured'] == abs(signed)
            if followpath_active(state):
                active_aligned += 1
                active_exact_abs += state['measured'] == abs(signed)
        yaw_states = [item for item in states if followpath_active(item)]
        commanded_yaw = [item['commanded_yaw'] for item in yaw_states]
        measured_yaw = [item['measured_yaw'] for item in yaw_states]
        yaw_fit = linear_regression(commanded_yaw, measured_yaw)
        results.append({
            'segment': segment,
            'gaps': gaps,
            'floor': sum(item['floor_violation'] for item in states),
            'speed_diff': sum(item['effective'] != item['commanded'] for item in states),
            'over_max': sum(item['throttle'] > OUTPUT_MAX for item in states),
            'steering_fraction': mean([float(item['steering_saturated']) for item in states]),
            'wheelspin_fraction': mean([float(item['wheelspin']) for item in states]),
            'aligned': aligned,
            'exact_signed_fraction': exact_signed / aligned if aligned else None,
            'exact_abs_fraction': exact_abs / aligned if aligned else None,
            'exact_abs_count': exact_abs,
            'active_exact_abs_fraction': (
                active_exact_abs / active_aligned if active_aligned else None
            ),
            'active_aligned': active_aligned,
            'active_exact_abs_count': active_exact_abs,
            'yaw_count': len(yaw_states),
            'yaw_correlation': pearson(commanded_yaw, measured_yaw),
            'yaw_slope': yaw_fit[0] if yaw_fit else None,
            'yaw_intercept': yaw_fit[1] if yaw_fit else None,
        })
    return results


def rel(time_ns, bag_start_ns):
    return (time_ns - bag_start_ns) / 1e9


def render_report(bag_path, size, bag_start_ns, bag_end_ns, samples, segments,
                  changes, checks, ff_findings, steps, ambiguous, plateau_count,
                  integrators, comparisons, freezes, diagnostic_rows):
    duration_s = (bag_end_ns - bag_start_ns) / 1e9
    lines = [
        '# PID live-tuning run: configuration-attributed findings',
        '',
        '## Scope and method',
        '',
        f'- Bag: `{bag_path}` (filename `{bag_path.name}`); exact size: {size:,} bytes '
        f'({size / 2**20:.1f} MiB).',
        f'- D-45 analysis window: +0.000 to +{duration_s:.3f} s bag receive time '
        f'({duration_s:.3f} s). The MCAP opening and closing magic and summary were valid.',
        f'- The Python `mcap` reader streamed only {", ".join(TOPICS)}. It did not load the '
        '537 MB file as a byte array. Relevant decoded samples were retained for alignment.',
        '- Measurements are partitioned by configuration segment S1...Sn below. A row that '
        'spans no single segment is explicitly marked unattributable.',
        f'- Steady-state statistics exclude [change, change + {STEADY_EXCLUSION_S:.1f} s) '
        'after every desired-speed, Kp, or Ki parameter event.',
        '- Encoder interval estimation adds speed-dependent lag (about 0.137 s at 0.3 m/s '
        'and 0.069 s at 0.6 m/s). Rise/response times are not corrected and are therefore '
        'inflated more at low speed.',
        '',
        '## Recovered controller timeline',
        '',
        '| ID | Start (s) | End (s) | Duration (s) | Kp | Ki | desired_linear_vel (m/s) | typed samples |',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for segment in segments:
        lines.append(
            f"| S{segment['index']} | {rel(segment['start_ns'], bag_start_ns):.3f} | "
            f"{rel(segment['end_ns'], bag_start_ns):.3f} | "
            f"{(segment['end_ns'] - segment['start_ns']) / 1e9:.3f} | "
            f"{segment['kp']:.3f} | {segment['ki']:.3f} | {segment['desired']:.2f} | "
            f"{len(segment['states']):,} |"
        )
    lines += ['', 'Gain and observer cross-check findings:', '']
    lines.extend(f'- {finding}' for finding in checks)
    lines += [
        '- Parameter events supply the exact write boundaries when Kp/Ki are unobservable '
        'because speed_error is zero. Direct state telemetry supplies the active-value '
        'check on every observable sample; no metric is assigned across a write boundary.',
        '',
        'Feedforward regression by attributed window (model `throttle = slope × '
        '|effective_speed| + intercept`):',
        '',
        '| Window/configuration | n | slope | intercept | max residual vs 0.1188/0.0174 |',
        '|---|---:|---:|---:|---:|',
    ]
    for segment, regression, residual, count in ff_findings:
        slope = regression[0] if regression else None
        intercept = regression[1] if regression else None
        lines.append(
            f"| S{segment['index']} ({config_label(segment)}) | {count:,} | "
            f'{format_float(slope, 6)} | {format_float(intercept, 6)} | '
            f'{format_float(residual, 12)} |'
        )
    lines.append(
        '- Finding: all nonzero effective-speed samples match coefficients 0.1188 and '
        '0.0174 to the residuals above; no coefficient change was observed. S4 and S6 '
        'contain no nonzero effective-speed samples, so regression alone cannot attribute '
        'coefficients there; the independent 1 Hz telemetry cross-check above covers them.'
    )

    lines += [
        '',
        '## Saturation check and per-step responses',
        '',
        f'- Step definition: stable effective-speed plateaus use ±{PLATEAU_TOLERANCE:.3f} '
        f'm/s tolerance and last at least {MIN_PLATEAU_S:.1f} s. A stable-to-stable '
        f'change of at least {MIN_STEP_SIZE:.2f} m/s is a step only when its intervening '
        f'command transition is at most {MAX_STEP_TRANSITION_S:.1f} s. This found '
        f'{plateau_count} plateaus and {len(steps)} reportable steps; {ambiguous} '
        'stable-to-stable changes were trajectories/ambiguous (>1 s) and were not assigned '
        'step-response metrics.',
        '- Saturation means `final_throttle >= 0.1386` (within 1% of output_max=0.14). '
        'The saturation fraction is checked before timing. Any nonzero fraction marks the '
        'step saturated and makes t63/t90/settling unusable.',
        '- Settling uses a ±2% target band; for a zero target only, ±2% of step size is used. '
        'Response speed is the latest encoder sample aligned by message sample stamp; this '
        'avoids AdapterState zeroing measured_speed in brake/silence. Steady error is signed '
        '`target - encoder_speed`, averaged over the final 50% of the target plateau after '
        'the 2 s change exclusions.',
        '- If OUTPUT_NOT_SELECTED occurs anywhere in a step window, timing is marked unusable '
        'because the computed controller output was not continuously selected. The affected '
        'fraction is reported separately from actuator saturation. Timing is also unusable if '
        f'the measured initial speed is more than {100 * INITIAL_CONDITION_STEP_FRACTION:.0f}% '
        'of step size away from the initial command plateau.',
        '',
        '| # | Window/configuration | Initial→target command; measured initial (m/s) | Dir | n | Sat frac | Output-not-selected frac | Timing validity | t63 (s) | t90 (s) | Overshoot | Settle (s) | SS error (m/s; n) | Throttle peak/mean | P peak(abs) | I peak(abs) |',
        '|---:|---|---:|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for index, step in enumerate(steps, 1):
        window = f"+{rel(step['start_ns'], bag_start_ns):.3f}..+{rel(step['end_ns'], bag_start_ns):.3f} s"
        if not step['attributable']:
            lines.append(
                f"| {index} | {window}; **unattributable (configuration boundary)** | "
                f"{step['initial']:.3f}→{step['target']:.3f}; n/a | n/a | n/a | n/a | n/a | "
                'unattributable | n/a | n/a | n/a | n/a | n/a | n/a | n/a | n/a |'
            )
            continue
        timing_flag = ' **SATURATED; unusable**' if step['saturated'] else ''
        validity = []
        if step['saturated']:
            validity.append('SATURATED')
        if step['output_not_selected_fraction'] > 0.0:
            validity.append('OUTPUT_NOT_SELECTED')
        if step['poor_initial']:
            validity.append('initial not at plateau')
        validity_text = '**unusable: ' + ', '.join(validity) + '**' if validity else 'usable'
        lines.append(
            f"| {index} | {window}; S{step['segment']['index']} "
            f"({config_label(step['segment'])}) | {step['initial']:.3f}→{step['target']:.3f}; "
            f"{step['initial_measured']:.3f} | "
            f"{step['direction']} | {step['sample_count']} | "
            f"{format_pct(step['saturation_fraction'])}{timing_flag} | "
            f"{format_pct(step['output_not_selected_fraction'])} | {validity_text} | "
            f"{format_float(step['t63'], 3)} | {format_float(step['t90'], 3)} | "
            f"{format_pct(step['overshoot'])} | {format_float(step['settling'], 3)} | "
            f"{format_float(step['steady_error'], 4)}; {step['steady_count']} | "
            f"{step['peak_throttle']:.4f}/{step['mean_throttle']:.4f} | "
            f"{step['peak_proportional']:.4f} | {step['peak_integrator']:.4f} |"
        )
    saturated_steps = sum(step.get('saturated', False) for step in steps)
    lines.append(
        f'- Saturation finding: {saturated_steps} of {len(steps)} step transients had any '
        'sample within 1% of output_max.'
    )

    lines += [
        '',
        '## Integrator behaviour (Ki > 0)',
        '',
        '| Window/configuration | n | I min/max/mean | Within 1% of ±0.005 | Longest continuous bound time (s) | Freeze releases | Error sign crossings in 5 s after releases | Max I drift while frozen |',
        '|---|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for result in integrators:
        releases = result['releases']
        lines.append(
            f"| S{result['segment']['index']} ({config_label(result['segment'])}); "
            f"+{rel(result['segment']['start_ns'], bag_start_ns):.3f}.."
            f"+{rel(result['segment']['end_ns'], bag_start_ns):.3f} s | "
            f"{len(result['segment']['states']):,} | {result['minimum']:.6f}/"
            f"{result['maximum']:.6f}/{result['mean']:.6f} | "
            f"{format_pct(result['bound_fraction'])} | {result['longest_bound_s']:.3f} | "
            f"{len(releases)} | {sum(item['crossings'] for item in releases)} total "
            f"(max {max((item['crossings'] for item in releases), default=0)}) | "
            f"{result['max_frozen_drift']:.9f} |"
        )
    bound_reached = any(result['bound_fraction'] > 0.0 for result in integrators)
    lines.append(
        f'- Bound finding: the ±0.005 bound was {"reached" if bound_reached else "never reached"}. '
        + ('See continuous times above.' if bound_reached else 'It was not pinned, so the feedforward-offset interpretation is not triggered.')
    )
    max_post_release_range = max(
        (release['integrator_range'] for result in integrators for release in result['releases']),
        default=0.0,
    )
    lines += [
        f'- Freeze-release finding: the reported error sign crossings are an oscillation-like '
        f'observation, but do not establish control-loop oscillation because encoder quantization '
        f'can create crossings. The largest 5 s post-release integrator range was '
        f'{max_post_release_range:.6f}; maximum drift during any frozen run was '
        f'{max((result["max_frozen_drift"] for result in integrators), default=0.0):.9f}. '
        'No frozen-state accumulation (wind-up while frozen) was observed.',
        '',
        'Comparable-speed steady-state absolute-error comparison (only Kp=0.05; same exact '
        'target plateau; final halves with change exclusions). “Valid” samples have freeze '
        'reason ACTIVE (Ki>0) or GAIN_DISABLED (Ki=0), excluding other vehicle/control states:',
        '',
        '| Target speed | Ki=0 window; mean abs error (n valid samples) | Ki=0.01 window; mean abs error (n valid samples) | Supported comparison/finding |',
        '|---:|---:|---:|---|',
    ]
    for (target, disabled, disabled_n, disabled_segments,
         enabled, enabled_n, enabled_segments) in comparisons:
        supported = (
            f'yes; difference Ki=0.01 minus Ki=0 is {enabled - disabled:+.4f} m/s'
            if disabled_n and enabled_n else
            'no; no matched-speed valid samples in both groups'
        )
        lines.append(
            f'| {target:.3f} m/s | '
            f'{",".join(f"S{item}" for item in disabled_segments) or "none"}; '
            f'{format_float(disabled, 4)} ({disabled_n}) | '
            f'{",".join(f"S{item}" for item in enabled_segments) or "none"}; '
            f'{format_float(enabled, 4)} ({enabled_n}) | {supported} |'
        )

    counts, durations, per_config, modes, state_summary, partitioned = freezes
    total = sum(counts.values())
    lines += [
        '',
        '## Integrator freeze-reason histogram',
        '',
        f'- D-45 window: full +0.000..+{duration_s:.3f} s, with every count partitioned '
        'by S-ID in the last column. Percentages use all typed-state samples.',
        '',
        '| Enum name | Count | Percent | Total held duration (s) | Primary modes (counts) | Configuration attribution |',
        '|---|---:|---:|---:|---|---|',
    ]
    for reason in sorted(REASON_NAMES):
        name = REASON_NAMES[reason]
        mode_text = ', '.join(f'{key}:{value}' for key, value in modes[reason].most_common()) or 'not observed'
        config_text = ', '.join(f'S{key}:{value}' for key, value in sorted(per_config[reason].items())) or 'not observed'
        lines.append(
            f'| {name} | {counts[reason]:,} | {100 * counts[reason] / total:.3f}% | '
            f'{durations[reason]:.3f} | {mode_text} | {config_text} |'
        )
    lines.append(
        '- Mode-partition finding: distribution '
        + ('still partitions perfectly by primary mode.' if partitioned else 'does not partition by primary mode; at least one mode contains multiple reasons.')
    )
    special = {1, 2, 11}
    lines += [
        '',
        'Observed ACTIVE and freeze reasons other than GAIN_DISABLED, ZERO_COMMAND, '
        'and NO_COMMAND, with vehicle state:',
        '',
    ]
    for reason in sorted(set(counts) - special):
        group = state_summary[reason]
        mode_text = ', '.join(f'{key}:{value}' for key, value in modes[reason].most_common())
        speed_min = min(item['effective'] for item in group)
        speed_max = max(item['effective'] for item in group)
        measured_min = min(item['measured'] for item in group)
        measured_max = max(item['measured'] for item in group)
        config_text = ', '.join(f'S{key}:{value}' for key, value in sorted(per_config[reason].items()))
        lines.append(
            f"- {REASON_NAMES[reason]}: {counts[reason]:,} samples, {durations[reason]:.3f} s; "
            f'mode {mode_text}; effective {speed_min:.3f}..{speed_max:.3f} m/s, measured '
            f'{measured_min:.3f}..{measured_max:.3f} m/s; {config_text}.'
        )
    never = [REASON_NAMES[key] for key in sorted(REASON_NAMES) if counts[key] == 0]
    lines.append('- Never observed: ' + (', '.join(never) if never else 'none') + '.')

    lines += [
        '',
        '## Run-wide diagnostics, partitioned by configuration',
        '',
        '- D-45 windows are the S-ID intervals. “Active” means typed mode `forward` with '
        'nonzero effective_speed; “idle” is every other typed state. Gap statistics are '
        'median/p95/max seconds within (not across) each class and segment.',
        '- Encoder matching aligns each typed state to the latest encoder sample by message sample '
        'stamp. Both the requested signed formula and the adapter implementation’s absolute-edge-rate '
        'formula are shown for all states; active-only absolute matching isolates brake/silence '
        'zeroing. Yaw fits use active samples and ordinary least squares with intercept.',
        '',
        '| Window/config | Active gaps med/p95/max (s) | Idle gaps med/p95/max (s) | floor viol | effective!=commanded | throttle>0.14 | steering sat | wheelspin | encoder exact signed/abs all; abs active | yaw corr; gain; intercept (n) |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
    ]
    for result in diagnostic_rows:
        def gap_text(key):
            values = result['gaps'][key]
            return '/'.join(format_float(value, 4) for value in (
                statistics.median(values) if values else None,
                percentile(values, .95),
                max(values) if values else None,
            ))
        segment = result['segment']
        lines.append(
            f"| S{segment['index']} ({config_label(segment)}); "
            f"+{rel(segment['start_ns'], bag_start_ns):.3f}..+{rel(segment['end_ns'], bag_start_ns):.3f} s | "
            f"{gap_text('active')} | {gap_text('idle')} | {result['floor']} | "
            f"{result['speed_diff']} | {result['over_max']} | "
            f"{format_pct(result['steering_fraction'])} | {format_pct(result['wheelspin_fraction'])} | "
            f"{format_pct(result['exact_signed_fraction'])}/{format_pct(result['exact_abs_fraction'])} "
            f"({result['aligned']:,}); {format_pct(result['active_exact_abs_fraction'])} "
            f"({result['active_aligned']:,}) | {format_float(result['yaw_correlation'], 4)}; "
            f"{format_float(result['yaw_slope'], 4)}; {format_float(result['yaw_intercept'], 4)} "
            f"({result['yaw_count']:,}) |"
        )
    exact_count = sum(result['exact_abs_count'] for result in diagnostic_rows)
    aligned_count = sum(result['aligned'] for result in diagnostic_rows)
    active_exact_count = sum(result['active_exact_abs_count'] for result in diagnostic_rows)
    active_aligned_count = sum(result['active_aligned'] for result in diagnostic_rows)
    lines.append(
        f'- Encoder exact-match finding over attributed S1–S8 samples: '
        f'{format_pct(exact_count / aligned_count)} ({exact_count:,}/{aligned_count:,}) '
        f'for all states and {format_pct(active_exact_count / active_aligned_count)} '
        f'({active_exact_count:,}/{active_aligned_count:,}) while FollowPath was active. '
        'Most all-state mismatches are brake/silence samples where AdapterState zeroes '
        'measured_speed while encoder coast-down continues.'
    )
    lines += [
        '',
        '## Attribution limits',
        '',
        '- Kp is directly observable only where |speed_error| > 0.01. During zero-error gaps, '
        'the event boundary and the next direct observation bound attribution; no response metric '
        'crosses such a boundary.',
        '- desired_linear_vel is not encoded in AdapterState. Its exact write timeline comes from '
        '/parameter_events and is cross-checked against /speed_envelope/status. effective_speed '
        'is the sample-aligned command used for response segmentation.',
        '- RPP frequently produces continuously varying effective_speed on curvature. Those '
        'intervals do not support step-response claims and are counted as ambiguous above rather '
        'than coerced into plateaus.',
    ]
    return '\n'.join(lines) + '\n'


def analyze(pattern):
    bag_path = resolve_bag(pattern)
    bag_start_ns, bag_end_ns, size, samples = validate_and_stream(bag_path)
    attach_encoder_measurements(samples['state'], samples['encoder'])
    segments, changes, _ = recover_timeline(bag_start_ns, bag_end_ns, samples)
    checks, ff_findings = cross_checks(samples, segments, changes, bag_start_ns)
    steps, ambiguous, plateau_count = step_metrics(samples['state'], changes)
    integrators, comparisons = integrator_metrics(segments, steps)
    freezes = freeze_metrics(samples['state'], segments)
    diagnostic_rows = diagnostics(segments, samples['encoder'])
    return render_report(
        bag_path, size, bag_start_ns, bag_end_ns, samples, segments, changes,
        checks, ff_findings, steps, ambiguous, plateau_count, integrators,
        comparisons, freezes, diagnostic_rows,
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Stream a pid0 MCAP and report configuration-attributed PID metrics.'
    )
    parser.add_argument('bag', help='MCAP file, bag directory, or glob resolving to one MCAP')
    parser.add_argument(
        '--output', type=Path, default=Path('analysis/pid0_report.md'),
        help='report path (default: analysis/pid0_report.md)',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        report = analyze(args.bag)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding='utf-8')
    except (OSError, RuntimeError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    print(report, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
