#!/usr/bin/env python3
"""Focused deterministic tests for throttle-response analysis."""

import json
import math
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import analyze_localization_bag as localization  # noqa: E402
import analyze_throttle_response as throttle  # noqa: E402


def scalar(start, end, step, value, noise=0.0):
    samples = []
    index = 0
    time_s = start
    while time_s <= end + 1e-9:
        offset = noise if index % 2 else -noise
        samples.append({
            'time_ns': round(time_s * 1e9),
            'value': value + offset,
            'order': index,
        })
        time_s += step
        index += 1
    return samples


def states(start, end, step, stationary, edge_rate):
    return [
        {
            'time_ns': item['time_ns'],
            'stationary': stationary,
            'edge_rate': edge_rate,
            'order': item['order'],
        }
        for item in scalar(start, end, step, 0.0)
    ]


def settings(**overrides):
    result = {
        'command_tolerance': 0.005,
        'minimum_segment_duration_s': 6.0,
        'settling_duration_s': 2.0,
        'zero_command_tolerance': 0.005,
        'motion_speed_threshold_mps': 0.03,
        'minimum_samples': 2,
        'command_gap_limit_s': 0.5,
        'minimum_neutral_duration_s': 2.0,
        'metres_per_edge': throttle.METRES_PER_EDGE,
    }
    result.update(overrides)
    return result


def candidate(command=0.1, duration=6.0):
    return throttle.segment_commands(
        scalar(0, duration, 0.05, command),
        minimum_duration=6.0,
        settling_duration=2.0,
    )[0]


def metric(command=0.1, speed=0.2, ekf=0.19, stationary=False):
    return throttle.segment_metrics(
        candidate(command),
        scalar(0, 6, 0.05, speed),
        states(
            0, 6, 0.05, stationary,
            abs(speed) / throttle.METRES_PER_EDGE,
        ),
        scalar(0, 6, 0.02, ekf),
    )


def indexed_metric(index, command, speed, ekf=None, start_ns=0):
    item = metric(
        command,
        speed if command > 0 else -abs(speed),
        (speed if command > 0 else -abs(speed)) if ekf is None else ekf,
        stationary=abs(speed) < 0.03,
    )
    item['index'] = index
    item['command_start_timestamp_ns'] = start_ns
    item['command_end_timestamp_ns'] = start_ns + 6_000_000_000
    item['analyzed_start_timestamp_ns'] = start_ns + 2_000_000_000
    item['analyzed_end_timestamp_ns'] = start_ns + 6_000_000_000
    return item


def test_localization_mcap_and_window_helpers_are_reused(monkeypatch):
    loaded = {
        'bag_start_ns': 1_000_000_000,
        'bag_end_ns': 11_000_000_000,
        'messages': [('topic', b'x', 4_000_000_000)],
    }
    window = localization.resolve_analysis_window(loaded, 2.0, 5.0)
    assert window['window_start_ns'] == 3_000_000_000
    assert localization.messages_in_receive_window(
        loaded['messages'], window['window_start_ns'], window['window_end_ns']
    ) == loaded['messages']


def test_localization_loader_default_retained_topics_unchanged(monkeypatch, tmp_path):
    captured = {}

    def fake_recovery(paths, retained, open_error=None):
        captured['topics'] = retained
        return {}

    path = tmp_path / 'bag.mcap'
    path.write_bytes(b'bad')
    monkeypatch.setattr(localization, 'recover_mcap_bag', fake_recovery)
    localization.load_bag_messages(path)
    assert captured['topics'] == set(localization.HEALTH_TOPICS)


def fake_loaded(topic_types=None, messages=None):
    return {
        'topic_types': topic_types or {
            topic: 'unused' for topic in throttle.REQUIRED_TOPICS
        },
        'messages': messages or [],
        'message_count': len(messages or []),
        'bag_start_ns': 0,
        'bag_end_ns': 10_000_000_000,
        'recovery_used': False,
        'truncated': False,
        'recovery_notes': [],
        'open_error': None,
    }


def test_missing_required_topic(monkeypatch):
    topics = {topic: 'unused' for topic in throttle.REQUIRED_TOPICS[:-1]}
    monkeypatch.setattr(
        localization, 'load_bag_messages',
        lambda *args, **kwargs: fake_loaded(topics),
    )
    with pytest.raises(RuntimeError, match='/scan'):
        throttle.decode_bag(Path('bag'), 0.0, None)


def test_empty_analysis_window(monkeypatch):
    loaded = fake_loaded(messages=[
        ('/cmd_vel', b'x', 0),
        ('/cmd_vel', b'x', 10_000_000_000),
    ])
    monkeypatch.setattr(
        localization, 'load_bag_messages', lambda *args, **kwargs: loaded
    )
    with pytest.raises(RuntimeError, match='contains no messages'):
        throttle.decode_bag(Path('bag'), 4.0, 5.0)


def test_constant_command_segmentation_and_settling_discard():
    segments = throttle.segment_commands(scalar(0, 6, 0.05, 0.1))
    assert len(segments) == 1
    assert segments[0]['raw_duration_s'] == pytest.approx(6.0)
    assert segments[0]['analyzed_duration_s'] == pytest.approx(4.0)
    assert segments[0]['analyzed_start_ns'] == 2_000_000_000


def test_joystick_noise_within_tolerance_does_not_split():
    assert len(throttle.segment_commands(
        scalar(0, 6, 0.05, 0.1, noise=0.004)
    )) == 1


def test_command_change_outside_tolerance_splits():
    commands = (
        scalar(0, 6, 0.05, 0.1)
        + scalar(6.05, 12.05, 0.05, 0.12)
    )
    assert len(throttle.segment_commands(commands)) == 2


def test_nonzero_command_change_is_not_accepted_as_fresh_stopped_run():
    commands = (
        scalar(0, 2, 0.05, 0.0)
        + scalar(2.05, 8.05, 0.05, 0.1)
        + scalar(8.10, 14.10, 0.05, 0.2)
    )
    state_samples = states(
        0, 15, 0.05, False, 0.2 / throttle.METRES_PER_EDGE
    )
    state_samples[40]['stationary'] = True
    result = throttle.analyze_samples(
        commands,
        scalar(0, 15, 0.05, 0.2),
        state_samples,
        scalar(0, 15, 0.02, 0.2),
        settings(),
    )
    assert len(result['segments']) == 1
    assert result['segments'][0]['command_median'] == pytest.approx(0.1)


def test_short_neutral_or_missing_stationary_evidence_rejects_run():
    short_neutral = (
        scalar(0, 0.05, 0.05, 0.0)
        + scalar(0.10, 6.10, 0.05, 0.1)
    )
    full_neutral = (
        scalar(0, 2, 0.05, 0.0)
        + scalar(2.05, 8.05, 0.05, 0.1)
    )
    common = (
        scalar(0, 9, 0.05, 0.2),
        states(0, 9, 0.05, False, 0.2 / throttle.METRES_PER_EDGE),
        scalar(0, 9, 0.02, 0.2),
    )
    for commands in (short_neutral, full_neutral):
        result = throttle.analyze_samples(
            commands, common[0], common[1], common[2], settings()
        )
        assert result['segments'] == []


def test_neutral_gap_resets_stop_interval():
    commands = [
        {'time_ns': 0, 'value': 0.0, 'order': 0},
        {'time_ns': 3_000_000_000, 'value': 0.0, 'order': 1},
        *scalar(3.05, 9.05, 0.05, 0.1),
    ]
    candidates = throttle.segment_commands(commands)
    assert len(candidates) == 1
    assert not candidates[0]['preceded_by_zero']


def test_latest_pre_run_encoder_state_must_be_stationary():
    commands = (
        scalar(0, 2, 0.05, 0.0)
        + scalar(2.05, 8.05, 0.05, 0.1)
    )
    state_samples = states(
        0, 9, 0.05, False, 0.2 / throttle.METRES_PER_EDGE
    )
    state_samples[20]['stationary'] = True
    result = throttle.analyze_samples(
        commands,
        scalar(0, 9, 0.05, 0.2),
        state_samples,
        scalar(0, 9, 0.02, 0.2),
        settings(),
    )
    assert result['segments'] == []


def test_sign_change_splits_forward_and_reverse():
    commands = (
        scalar(0, 6, 0.05, 0.1)
        + scalar(6.05, 12.05, 0.05, -0.1)
    )
    segments = throttle.segment_commands(commands)
    assert [math.copysign(1, item['representative_command']) for item in segments] == [1, -1]


def test_zero_stop_separates_repeated_runs():
    commands = (
        scalar(0, 6, 0.05, 0.1)
        + scalar(6.05, 8.0, 0.05, 0.0)
        + scalar(8.05, 14.05, 0.05, 0.1)
    )
    assert len(throttle.segment_commands(commands)) == 2


def test_short_segment_rejected():
    assert throttle.segment_commands(scalar(0, 5.95, 0.05, 0.1)) == []


def test_insufficient_retained_duration_rejected():
    assert throttle.segment_commands(
        scalar(0, 6, 0.05, 0.1),
        minimum_duration=6.0,
        settling_duration=6.0,
    ) == []


def test_insufficient_retained_samples_rejected():
    assert throttle.segment_metrics(
        candidate(), scalar(0, 1, 0.1, 0.2),
        states(0, 1, 0.1, False, 10), scalar(0, 1, 0.1, 0.2)
    ) is None


def test_low_nonzero_no_motion_is_deadband_candidate():
    item = metric(0.04, 0.0, 0.0, stationary=True)
    assert item['deadband_candidate']
    assert not item['sustained_motion']


def test_sustained_motion_classification_and_statistics():
    item = metric(0.1, 0.2, 0.19)
    assert item['sustained_motion']
    assert item['encoder_mean_signed_mps'] == pytest.approx(0.2)
    assert item['encoder_standard_deviation_mps'] == pytest.approx(0.0)
    assert item['ekf_mean_signed_mps'] == pytest.approx(0.19)
    assert item['stationary_fraction'] == 0


def test_stationary_fraction_prevents_twitch_classification():
    item = metric(0.1, 0.04, 0.04, stationary=True)
    assert not item['sustained_motion']


def test_direction_ambiguous_velocity_is_not_sustained_or_deadband():
    speeds = scalar(0, 6, 0.05, 0.1)
    for index, item in enumerate(speeds):
        item['value'] = 0.1 if index % 2 else -0.1
    item = throttle.segment_metrics(
        candidate(0.1),
        speeds,
        states(0, 6, 0.05, False, 0.1 / throttle.METRES_PER_EDGE),
        scalar(0, 6, 0.02, 0.0),
    )
    assert item['direction_ambiguous']
    assert not item['sustained_motion']
    assert not item['deadband_candidate']


def test_encoder_ekf_disagreement_flags_absolute_or_relative():
    absolute = metric(0.1, 0.3, 0.15)
    relative = metric(0.1, 0.04, 0.02)
    assert absolute['encoder_ekf_disagreement']
    assert relative['encoder_ekf_disagreement']


def test_forward_reverse_separation():
    segments = [
        indexed_metric(1, 0.1, 0.2),
        indexed_metric(2, -0.1, 0.15),
    ]
    assert len(throttle.direction_summary(
        [segments[0]], 0.005
    )['aggregated_points']) == 1
    analysis = {
        direction: [item for item in segments if item['direction'] == direction]
        for direction in ('forward', 'reverse')
    }
    assert [len(analysis[key]) for key in ('forward', 'reverse')] == [1, 1]


def test_duplicate_throttle_aggregation_is_duration_weighted():
    first = indexed_metric(1, 0.1, 0.2)
    second = indexed_metric(2, 0.102, 0.4)
    second['analyzed_duration_s'] = 8
    points = throttle.aggregate_segments([first, second], 0.005)
    assert len(points) == 1
    assert points[0]['encoder_mean_speed_magnitude_mps'] == pytest.approx(
        (0.2 * 4 + 0.4 * 8) / 12
    )
    assert points[0]['encoder_standard_deviation_mps'] == pytest.approx(
        math.sqrt(
            (
                4 * (0.2 - (0.2 * 4 + 0.4 * 8) / 12) ** 2
                + 8 * (0.4 - (0.2 * 4 + 0.4 * 8) / 12) ** 2
            ) / 12
        )
    )
    assert points[0]['segment_indices'] == [1, 2]


def test_mixed_repeat_aggregate_uses_sustained_members_only():
    stopped = indexed_metric(1, 0.1, 0.0)
    moving = indexed_metric(2, 0.102, 0.2)
    point = throttle.aggregate_segments([stopped, moving], 0.005)[0]
    assert point['mixed_sustained_motion']
    assert point['encoder_mean_speed_magnitude_mps'] == pytest.approx(0.2)
    assert point['segment_indices'] == [2]
    assert point['all_segment_indices'] == [1, 2]


def test_deadband_bracketing_and_minimum_sustainable_speed():
    segments = [
        indexed_metric(1, 0.04, 0.0),
        indexed_metric(2, 0.06, 0.08),
        indexed_metric(3, 0.08, 0.12),
    ]
    points = throttle.aggregate_segments(segments, 0.005)
    assert throttle.deadband_summary(points)['bracket'] == pytest.approx([0.04, 0.06])
    minimum = throttle.minimum_sustainable_speed(segments)
    assert minimum['encoder_mean_speed_magnitude_mps'] == pytest.approx(0.08)
    assert minimum['normalized_throttle'] == pytest.approx(0.06)


def test_target_0_3_interpolation_and_monotonic_fit():
    segments = [
        indexed_metric(1, 0.1, 0.2),
        indexed_metric(2, 0.2, 0.4),
    ]
    summary = throttle.direction_summary(segments, 0.005)
    target = summary['target_speed_0_3']
    assert target['status'] == 'interpolated'
    assert target['estimated_normalized_throttle'] == pytest.approx(0.15)
    assert summary['fit']['monotonic']
    assert summary['fit']['type'] == 'monotonic_piecewise_linear'


def test_exact_target_flat_bracket_reports_throttle_interval():
    segments = [
        indexed_metric(1, 0.1, 0.3),
        indexed_metric(2, 0.2, 0.3),
    ]
    target = throttle.direction_summary(
        segments, 0.005
    )['target_speed_0_3']
    assert target['status'] == 'interpolated'
    assert target['estimated_normalized_throttle'] == pytest.approx(0.15)
    assert target['interpolation_bracket'] == pytest.approx([0.1, 0.2])


def test_target_below_sustainable_floor_has_no_extrapolation():
    segments = [
        indexed_metric(1, 0.1, 0.4),
        indexed_metric(2, 0.2, 0.6),
    ]
    target = throttle.direction_summary(
        segments, 0.005
    )['target_speed_0_3']
    assert target['below_minimum_sustainable_speed']
    assert target['estimated_normalized_throttle'] is None
    assert target['warning'] == 'no extrapolation performed'


def test_target_above_range_has_no_extrapolation():
    segments = [
        indexed_metric(1, 0.1, 0.1),
        indexed_metric(2, 0.2, 0.2),
    ]
    target = throttle.direction_summary(
        segments, 0.005
    )['target_speed_0_3']
    assert target['status'] == 'outside_measured_moving_range'
    assert target['estimated_normalized_throttle'] is None


def test_non_monotonic_data_warns_without_altering_raw_points():
    segments = [
        indexed_metric(1, 0.1, 0.4),
        indexed_metric(2, 0.2, 0.3),
    ]
    summary = throttle.direction_summary(segments, 0.005)
    assert not summary['fit']['monotonic']
    assert summary['fit']['suspect_segment_indices'] == [1, 2]
    assert [
        item['encoder_mean_speed_magnitude_mps']
        for item in summary['aggregated_points']
    ] == pytest.approx([0.4, 0.3])


def test_repeated_low_throttle_pair_and_pack_droop_percentage():
    segments = [
        indexed_metric(1, 0.1, 0.2, start_ns=0),
        indexed_metric(2, 0.2, 0.4, start_ns=10_000_000_000),
        indexed_metric(3, 0.1, 0.18, start_ns=100_000_000_000),
    ]
    pairs = throttle.pack_droop_pairs(segments, 0.005)
    assert len(pairs) == 1
    assert pairs[0]['first_segment_index'] == 1
    assert pairs[0]['last_segment_index'] == 3
    assert pairs[0]['encoder_percentage_change'] == pytest.approx(-10)
    assert pairs[0]['elapsed_session_time_s'] == pytest.approx(100)


def minimal_result():
    segments = [
        indexed_metric(1, 0.04, 0.0),
        indexed_metric(2, 0.1, 0.2),
        indexed_metric(3, 0.2, 0.4),
        indexed_metric(4, -0.1, 0.2),
        indexed_metric(5, -0.2, 0.4),
    ]
    return {
        'metadata': {
            'bag': 'synthetic',
            'window_start_s': 0.0,
            'window_end_s': 30.0,
        },
        'settings': {
            'command_tolerance': 0.005,
            'minimum_segment_duration_s': 6.0,
            'settling_duration_s': 2.0,
            'motion_speed_threshold_mps': 0.03,
            'command_gap_limit_s': 0.5,
        },
        'topic_summary': {
            topic: {
                'count': 10, 'rate_hz': 20.0,
                'duplicate_timestamps': 0,
                'non_monotonic_samples': 0,
                'large_gap_count': 0,
            }
            for topic in throttle.REQUIRED_TOPICS
        },
        'segments': segments,
        'forward': throttle.direction_summary(
            [item for item in segments if item['direction'] == 'forward'],
            0.005,
        ),
        'reverse': throttle.direction_summary(
            [item for item in segments if item['direction'] == 'reverse'],
            0.005,
        ),
        'pack_droop_proxy': [],
        'warnings': [],
    }


def test_json_contains_no_nan_or_infinity():
    result = minimal_result()
    result['metadata']['bad'] = float('nan')
    encoded = json.dumps(throttle.finite_json(result), allow_nan=False)
    assert 'NaN' not in encoded
    assert 'Infinity' not in encoded
    assert json.loads(encoded)['metadata']['bad'] is None


def test_text_output_smoke(capsys):
    throttle.print_text(minimal_result())
    output = capsys.readouterr().out
    for heading in (
        'Throttle response analysis',
        'Per-segment results',
        'Forward throttle-to-speed table',
        'Reverse throttle-to-speed table',
        'Pack-droop proxy',
        'Warnings and limitations',
    ):
        assert heading in output


def test_cli_help():
    completed = subprocess.run(
        [sys.executable, str(TOOLS / 'analyze_throttle_response.py'), '--help'],
        check=True,
        capture_output=True,
        text=True,
    )
    assert '--window-start' in completed.stdout
    assert '--motion-speed-threshold' in completed.stdout


def test_timestamp_diagnostics_detect_anomalies():
    samples = [
        {'time_ns': 0},
        {'time_ns': 1_000_000_000},
        {'time_ns': 1_000_000_000},
        {'time_ns': 500_000_000},
        {'time_ns': 3_000_000_000},
    ]
    result = throttle.timestamp_diagnostics(samples)
    assert result['duplicate_timestamps'] == 1
    assert result['non_monotonic_samples'] == 1
    assert result['large_gap_count'] == 1


def test_timestamp_source_header_and_receive_fallback():
    header = SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=3))
    message = SimpleNamespace(header=header)
    assert throttle.timestamp_ns(message, 9, '/wheel/odom') == (
        2_000_000_003, 'header'
    )
    message.header.stamp = SimpleNamespace(sec=0, nanosec=0)
    assert throttle.timestamp_ns(message, 9, '/wheel/odom') == (
        9, 'bag_receive_fallback'
    )
