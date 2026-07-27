#!/usr/bin/env python3
"""Deterministic tests for yaw cross-validation analysis."""

import importlib.util
import math
import struct
import sys
from pathlib import Path

import pytest


TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
SPEC = importlib.util.spec_from_file_location(
    'yaw_analysis',
    TOOLS / 'analyze_yaw_cross_validation.py',
)
YAW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(YAW)

LOCALIZATION_SPEC = importlib.util.spec_from_file_location(
    'localization_analysis',
    TOOLS / 'analyze_localization_bag.py',
)
LOCALIZATION = importlib.util.module_from_spec(LOCALIZATION_SPEC)
LOCALIZATION_SPEC.loader.exec_module(LOCALIZATION)


def scalar_samples(values):
    return [
        {'time_ns': round(time_s * 1e9), 'value': value, 'order': index}
        for index, (time_s, value) in enumerate(values)
    ]


def transform_samples(values):
    return YAW.prepare_transform_samples([
        {
            'time_ns': round(time_s * 1e9),
            'x': x,
            'y': y,
            'yaw': yaw,
            'order': index,
        }
        for index, (time_s, x, y, yaw) in enumerate(values)
    ])


def test_trapezoidal_constant_positive_and_negative_yaw():
    positive = YAW.integrate_series(
        scalar_samples([(0, 2), (1, 2), (2, 2)]),
        0,
        2_000_000_000,
        2_000_000_000,
    )
    negative = YAW.integrate_series(
        scalar_samples([(0, -1.5), (1, -1.5), (2, -1.5)]),
        0,
        2_000_000_000,
        2_000_000_000,
    )
    assert positive['yaw_rad'] == pytest.approx(4.0)
    assert negative['yaw_rad'] == pytest.approx(-3.0)


def test_angle_unwrap_across_positive_pi_to_negative_pi():
    values = YAW.unwrap_angles([3.0, -3.0, -2.8])
    assert values == pytest.approx([3.0, 2 * math.pi - 3.0, 2 * math.pi - 2.8])


def test_tf_composition_map_odom_to_base_link():
    composed = YAW.compose_transform(
        {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2},
        {'x': 3.0, 'y': 0.0, 'yaw': math.pi / 4},
    )
    assert composed['x'] == pytest.approx(1.0)
    assert composed['y'] == pytest.approx(5.0)
    assert composed['yaw'] == pytest.approx(3 * math.pi / 4)


def test_signed_slam_yaw_change_is_preserved():
    rates = {
        source: scalar_samples([(0, -1), (1, -1)])
        for source in YAW.RATE_SOURCES
    }
    map_odom = transform_samples([(0, 0, 0, 0), (1, 0, 0, -1)])
    odom_base = transform_samples([(0, 0, 0, 0), (1, 0, 0, 0)])
    result = YAW.aligned_bins(
        rates, map_odom, odom_base, 0, 1_000_000_000,
        1_000_000_000, 2_000_000_000,
    )[2]
    assert result['slam_yaw_rad'] == pytest.approx(-1.0)
    assert result['imu_yaw_rad'] == pytest.approx(-1.0)


@pytest.mark.parametrize(
    ('value', 'expected'),
    [
        (0.0, 0),
        (0.299999, 0),
        (0.3, 1),
        (0.999999, 1),
        (1.0, 2),
        (1.999999, 2),
        (2.0, 3),
        (10.0, 3),
    ],
)
def test_bin_assignment_boundaries(value, expected):
    assert YAW.bin_index(value) == expected


def test_same_interval_comparison_for_all_sources():
    rates = {
        'imu': scalar_samples([(0, 2), (1, 2)]),
        'rf2o': scalar_samples([(0, 3), (1, 3)]),
        'ekf': scalar_samples([(0, 1), (1, 1)]),
    }
    map_odom = transform_samples([(0, 0, 0, 0), (1, 0, 0, 0)])
    odom_base = transform_samples([(0, 0, 0, 0), (1, 0, 0, 1)])
    result = YAW.aligned_bins(
        rates, map_odom, odom_base, 0, 1_000_000_000,
        1_000_000_000, 2_000_000_000,
    )[2]
    assert result['effective_duration_s'] == pytest.approx(1.0)
    assert result['slam_yaw_rad'] == pytest.approx(1.0)
    assert result['imu_yaw_rad'] == pytest.approx(2.0)
    assert result['rf2o_yaw_rad'] == pytest.approx(3.0)
    assert result['ekf_yaw_rad'] == pytest.approx(1.0)


def test_missing_sample_and_maximum_gap_exclude_interval():
    result = YAW.integrate_series(
        scalar_samples([(0, 1), (1, 1), (4, 1)]),
        0,
        4_000_000_000,
        1_500_000_000,
    )
    assert result['yaw_rad'] == pytest.approx(1.0)
    assert result['valid_duration_s'] == pytest.approx(1.0)
    assert result['excluded_duration_s'] == pytest.approx(3.0)
    assert result['gap_count'] == 1


def test_near_zero_reference_ratio_is_none():
    assert YAW.ratio(1.0, 0.0) is None
    assert YAW.ratio(1.0, 0.5e-6) is None
    assert YAW.ratio(1.0, 2.0) == pytest.approx(0.5)


def test_combined_bags_sum_integrals_before_ratio():
    first = {'bins': [YAW.empty_bin(i) for i in range(4)]}
    second = {'bins': [YAW.empty_bin(i) for i in range(4)]}
    first['bins'][0].update({
        'aligned_sample_count': 50,
        'effective_duration_s': 3,
        'slam_yaw_rad': 1,
        'slam_abs_yaw_rad': 1,
        'imu_yaw_rad': 2,
    })
    second['bins'][0].update({
        'aligned_sample_count': 60,
        'effective_duration_s': 3,
        'slam_yaw_rad': 3,
        'slam_abs_yaw_rad': 3,
        'imu_yaw_rad': 3,
    })
    combined = YAW.combine_results([first, second])[0]
    assert combined['slam_yaw_rad'] == pytest.approx(4)
    assert combined['imu_yaw_rad'] == pytest.approx(5)
    assert combined['imu_to_slam_ratio'] == pytest.approx(1.25)
    assert combined['evidence'] == 'sufficient'


def mcap_record(opcode, content):
    return struct.pack('<BQ', opcode, len(content)) + content


def mcap_string(value):
    encoded = value.encode()
    return struct.pack('<I', len(encoded)) + encoded


def test_truncated_mcap_recovery_retains_complete_message(tmp_path):
    schema = struct.pack('<H', 1) + mcap_string('std_msgs/msg/String')
    channel = (
        struct.pack('<HH', 1, 1)
        + mcap_string('/unused')
        + mcap_string('cdr')
    )
    message = (
        struct.pack('<HIQQ', 1, 1, 123, 123)
        + b'payload'
    )
    path = tmp_path / 'truncated.mcap'
    path.write_bytes(
        LOCALIZATION.MCAP_MAGIC
        + mcap_record(LOCALIZATION.OP_SCHEMA, schema)
        + mcap_record(LOCALIZATION.OP_CHANNEL, channel)
        + mcap_record(LOCALIZATION.OP_MESSAGE, message)
        + b'\x01\x02'
    )
    recovered = LOCALIZATION.recover_mcap_file(path, {'/unused'})
    assert recovered['truncated'] is True
    assert recovered['message_count'] == 1
    assert recovered['messages'] == [('/unused', b'payload', 123)]
    assert 'truncated record header' in recovered['parse_error']


def test_duplicate_timestamp_retains_last_received_sample():
    samples = scalar_samples([(0, 1), (0, 2), (1, 3)])
    values = [
        item['value'] for item in YAW.deduplicate_samples(samples)
    ]
    assert values == [2, 3]


def test_localization_analyzer_invariant_helpers_are_unchanged():
    assert LOCALIZATION.wrap_angle(3 * math.pi) == pytest.approx(math.pi)
    assert LOCALIZATION.format_value(None) == 'n/a'
    assert LOCALIZATION.format_value(1.23456) == '1.235'
