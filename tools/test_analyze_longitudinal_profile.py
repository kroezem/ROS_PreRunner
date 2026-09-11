"""Focused tests for longitudinal profile analysis primitives."""

import pytest

import analyze_longitudinal_profile as profile


def sample_series(values, step_ns=50_000_000):
    return [
        (index * step_ns, value, index * step_ns)
        for index, value in enumerate(values)
    ]


def test_linear_fit_recovers_command_as_function_of_speed():
    fit = profile.linear_fit_xy([(0.3, 0.05), (0.7, 0.10), (1.1, 0.15)])
    assert fit['slope'] == pytest.approx(0.125)
    assert fit['intercept'] == pytest.approx(0.0125)
    assert fit['rmse'] == pytest.approx(0.0)


def test_interpolation_rejects_stale_bracket():
    samples = [(0, 0.0, 0), (100_000_000, 1.0, 100_000_000)]
    value, gap = profile.interpolate(samples, 50_000_000, max_gap_s=0.2)
    assert value == pytest.approx(0.5)
    assert gap == pytest.approx(0.1)
    value, gap = profile.interpolate(samples, 50_000_000, max_gap_s=0.05)
    assert value is None
    assert gap == pytest.approx(0.1)


def test_steady_tail_accepts_flat_tail_and_rejects_ramp():
    segment = {'start_ns': 0, 'end_ns': 2_000_000_000, 'command': 0.1}
    flat = sample_series([min(index * 0.08, 0.7) for index in range(41)])
    ramp = sample_series([index * 0.04 for index in range(41)])
    start_ns, metrics = profile.find_steady_tail(segment, flat)
    assert start_ns is not None
    assert abs(metrics['slope_mps2']) <= profile.MAX_STEADY_SLOPE_MPS2
    assert profile.find_steady_tail(segment, ramp) == (None, None)


def test_command_segmentation_preserves_direction():
    commands = sample_series([0.05] * 25 + [0.0] * 5 + [-0.10] * 25)
    segments = profile.command_segments(commands)
    assert [item['command'] for item in segments] == [0.05, -0.10]
