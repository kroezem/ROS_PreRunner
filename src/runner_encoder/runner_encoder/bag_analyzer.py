# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Analyze Task 1 wheel-encoder MCAP recordings."""

import argparse
from bisect import bisect_left
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
from statistics import mean, median

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message


WHEEL_TOPIC = '/wheel/odom'
EKF_TOPIC = '/odometry/filtered'
DIRECTION_TOPIC = '/motor/direction'
STATE_TOPIC = '/wheel/encoder_state'
SPEED_BINS = (
    ('0.00 <= |vx| < 0.10', 0.00, 0.10),
    ('0.10 <= |vx| < 0.25', 0.10, 0.25),
    ('0.25 <= |vx| < 0.50', 0.25, 0.50),
    ('0.50 <= |vx| < 1.00', 0.50, 1.00),
    ('1.00 <= |vx|', 1.00, math.inf),
)


@dataclass(frozen=True)
class OdomSample:
    """One deserialized odometry sample."""

    receive_ns: int
    stamp_ns: int
    velocity: float


@dataclass(frozen=True)
class DirectionSample:
    """One deserialized motor-direction sample."""

    receive_ns: int
    direction: int


@dataclass(frozen=True)
class StateSample:
    """One deserialized encoder-state sample."""

    receive_ns: int
    stamp_ns: int
    edge_rate: float
    stationary: bool
    active_direction: int
    pending_direction: int


def _stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _topic_rate(rows) -> float:
    if len(rows) < 2:
        return math.nan
    duration = (rows[-1].receive_ns - rows[0].receive_ns) / 1e9
    return (len(rows) - 1) / duration if duration > 0.0 else math.nan


def _nearest_index(timestamps, target: int) -> int:
    index = bisect_left(timestamps, target)
    candidates = [
        candidate
        for candidate in (index - 1, index)
        if 0 <= candidate < len(timestamps)
    ]
    return min(
        candidates,
        key=lambda candidate: abs(timestamps[candidate] - target),
    )


def _read_bag(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    type_map = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }
    wanted_topics = {
        WHEEL_TOPIC,
        EKF_TOPIC,
        DIRECTION_TOPIC,
        STATE_TOPIC,
    }
    message_types = {
        topic: get_message(type_map[topic])
        for topic in wanted_topics
        if topic in type_map
    }
    samples = {topic: [] for topic in wanted_topics}
    bag_first_ns = None
    bag_last_ns = None

    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        bag_first_ns = (
            receive_ns
            if bag_first_ns is None
            else min(bag_first_ns, receive_ns)
        )
        bag_last_ns = (
            receive_ns
            if bag_last_ns is None
            else max(bag_last_ns, receive_ns)
        )
        if topic not in message_types:
            continue

        message = deserialize_message(serialized, message_types[topic])
        if topic in (WHEEL_TOPIC, EKF_TOPIC):
            samples[topic].append(
                OdomSample(
                    receive_ns=receive_ns,
                    stamp_ns=_stamp_ns(message.header.stamp),
                    velocity=float(message.twist.twist.linear.x),
                )
            )
        elif topic == DIRECTION_TOPIC:
            samples[topic].append(
                DirectionSample(
                    receive_ns=receive_ns,
                    direction=int(message.data),
                )
            )
        else:
            samples[topic].append(
                StateSample(
                    receive_ns=receive_ns,
                    stamp_ns=_stamp_ns(message.stamp),
                    edge_rate=float(message.edge_rate),
                    stationary=bool(message.stationary),
                    active_direction=int(message.active_direction),
                    pending_direction=int(message.pending_direction),
                )
            )

    if bag_first_ns is None or bag_last_ns is None:
        raise RuntimeError(f'bag contains no messages: {bag_path}')
    return bag_first_ns, bag_last_ns, samples


def _print_bag_range(first_ns: int, last_ns: int) -> None:
    print('Bag receive-time range')
    for label, value in (('first', first_ns), ('last', last_ns)):
        timestamp = datetime.fromtimestamp(value / 1e9, timezone.utc)
        print(f'  {label}: {value} ({timestamp.isoformat()})')
    print(f'  duration: {(last_ns - first_ns) / 1e9:.9f} s')


def _print_topic_statistics(samples) -> None:
    print('\nTopic counts and rates')
    for topic in (
        WHEEL_TOPIC,
        EKF_TOPIC,
        DIRECTION_TOPIC,
        STATE_TOPIC,
    ):
        rows = samples[topic]
        rate = _topic_rate(rows)
        rate_text = 'n/a' if math.isnan(rate) else f'{rate:.6f} Hz'
        print(f'  {topic}: {len(rows)} samples, {rate_text}')


def _aligned_wheel_samples(wheel, ekf, tolerance_ns: int):
    ekf_timestamps = [sample.receive_ns for sample in ekf]
    aligned = []
    for wheel_sample in wheel:
        index = _nearest_index(ekf_timestamps, wheel_sample.receive_ns)
        ekf_sample = ekf[index]
        delta_ns = abs(ekf_sample.receive_ns - wheel_sample.receive_ns)
        if delta_ns <= tolerance_ns:
            aligned.append((wheel_sample, ekf_sample, delta_ns))
    return aligned


def _print_wheel_metrics(
    wheel,
    ekf,
    tolerance_ms: float,
    moving_threshold: float,
) -> None:
    zero_count = sum(sample.velocity == 0.0 for sample in wheel)
    zero_fraction = zero_count / len(wheel)
    aligned = _aligned_wheel_samples(
        wheel,
        ekf,
        int(tolerance_ms * 1e6),
    )
    moving = [
        pair
        for pair in aligned
        if abs(pair[1].velocity) > moving_threshold
    ]
    moving_zero_count = sum(
        wheel_sample.velocity == 0.0
        for wheel_sample, _, _ in moving
    )
    moving_zero_fraction = moving_zero_count / len(moving)

    print('\nWheel-zero metrics')
    print(f'  total wheel samples: {len(wheel)}')
    print(
        f'  exact-zero wheel samples: {zero_count} / {len(wheel)} '
        f'= {100.0 * zero_fraction:.4f}%'
    )
    print(
        f'  aligned samples: {len(aligned)} '
        f'(nearest receive time, tolerance {tolerance_ms:g} ms)'
    )
    print(
        f'  aligned moving samples (EKF |vx| > '
        f'{moving_threshold:g} m/s): {len(moving)}'
    )
    print(
        f'  moving samples with exactly-zero wheel speed: '
        f'{moving_zero_count} / {len(moving)} '
        f'= {100.0 * moving_zero_fraction:.4f}%'
    )

    print('\nResidual wheel zeros by aligned EKF absolute speed')
    print('  bin                          aligned  zeros  zero %')
    for label, lower, upper in SPEED_BINS:
        binned = [
            pair
            for pair in aligned
            if lower <= abs(pair[1].velocity) < upper
        ]
        binned_zeros = sum(
            wheel_sample.velocity == 0.0
            for wheel_sample, _, _ in binned
        )
        percentage = (
            100.0 * binned_zeros / len(binned)
            if binned
            else math.nan
        )
        percentage_text = (
            'n/a' if math.isnan(percentage) else f'{percentage:6.2f}%'
        )
        print(
            f'  {label:<28} {len(binned):>7} '
            f'{binned_zeros:>6}  {percentage_text}'
        )


def _print_distribution(label: str, values) -> None:
    counts = Counter(values)
    print(f'  {label}:')
    for value in sorted(counts):
        print(
            f'    {value:+d}: {counts[value]} '
            f'({100.0 * counts[value] / len(values):.4f}%)'
        )


def _print_encoder_state_statistics(states, wheel) -> None:
    print('\nEncoder-state statistics')
    if not states:
        print(f'  {STATE_TOPIC} is not present')
        return

    edge_rates = [sample.edge_rate for sample in states]
    stationary_count = sum(sample.stationary for sample in states)
    print(f'  samples: {len(states)}')
    print(
        f'  edge rate: min={min(edge_rates):.6f}, '
        f'median={median(edge_rates):.6f}, '
        f'mean={mean(edge_rates):.6f}, '
        f'max={max(edge_rates):.6f} edges/s'
    )
    print(
        f'  zero edge-rate samples: '
        f'{sum(rate == 0.0 for rate in edge_rates)} / {len(edge_rates)}'
    )
    print(
        f'  stationary: {stationary_count} / {len(states)} '
        f'= {100.0 * stationary_count / len(states):.4f}%'
    )
    _print_distribution(
        'active-direction distribution',
        [sample.active_direction for sample in states],
    )
    _print_distribution(
        'pending-direction distribution',
        [sample.pending_direction for sample in states],
    )

    transition_count = 0
    supported_count = 0
    unsupported = []
    previous = states[0]
    for sample in states[1:]:
        if sample.active_direction != previous.active_direction:
            transition_count += 1
            if previous.stationary:
                supported_count += 1
            else:
                unsupported.append(
                    (
                        sample.receive_ns,
                        previous.active_direction,
                        sample.active_direction,
                    )
                )
        previous = sample

    print(f'  active-direction transitions: {transition_count}')
    print(
        '  transitions immediately following a stationary sample: '
        f'{supported_count} / {transition_count}'
    )
    if unsupported:
        print('  transitions without recorded stationary evidence:')
        for receive_ns, old_direction, new_direction in unsupported:
            print(
                f'    {receive_ns}: '
                f'{old_direction:+d} -> {new_direction:+d}'
            )

    wheel_stamps = {sample.stamp_ns for sample in wheel}
    matching_stamps = sum(
        sample.stamp_ns in wheel_stamps
        for sample in states
    )
    print(
        f'  state stamps exactly matching wheel stamps: '
        f'{matching_stamps} / {len(states)}'
    )


def _parse_args():
    parser = argparse.ArgumentParser(
        description=(
            'Compute Runner Task 1 wheel-zero and encoder-state metrics '
            'directly from an MCAP bag.'
        )
    )
    parser.add_argument('bag', type=Path, help='MCAP rosbag2 directory')
    parser.add_argument(
        '--tolerance-ms',
        type=float,
        default=40.0,
        help='maximum receive-time nearest-neighbour difference (default: 40)',
    )
    parser.add_argument(
        '--moving-threshold',
        type=float,
        default=0.25,
        help='EKF absolute-vx moving threshold in m/s (default: 0.25)',
    )
    return parser.parse_args()


def main():
    """Run the command-line bag analyzer."""
    args = _parse_args()
    first_ns, last_ns, samples = _read_bag(args.bag)
    wheel = samples[WHEEL_TOPIC]
    ekf = samples[EKF_TOPIC]
    if not wheel or not ekf:
        raise RuntimeError(
            f'bag must contain {WHEEL_TOPIC} and {EKF_TOPIC}'
        )

    _print_bag_range(first_ns, last_ns)
    _print_topic_statistics(samples)
    _print_wheel_metrics(
        wheel,
        ekf,
        args.tolerance_ms,
        args.moving_threshold,
    )
    _print_encoder_state_statistics(samples[STATE_TOPIC], wheel)


if __name__ == '__main__':
    main()
