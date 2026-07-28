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

"""Focused unit tests for conversion and breakaway behavior."""

from dataclasses import replace
import math

import pytest

from runner_drive_adapter.drive_adapter import (
    AdapterConfig,
    CURVATURE_ABS_TOLERANCE,
    CURVATURE_REL_TOLERANCE,
    DriveAdapter,
    lookup_throttle,
    validate_table,
)


THROTTLES = (0.340, 0.350, 0.360, 0.380)
SPEEDS = (0.126, 0.188, 0.233, 0.290)


def test_valid_monotonic_table_is_accepted():
    validate_table(THROTTLES, SPEEDS)


@pytest.mark.parametrize(
    ('throttles', 'speeds'),
    [
        ((0.3, 0.4), (0.1,)),
        ((0.3,), (0.1,)),
        ((0.3, 0.4), (0.1, 0.1)),
        ((0.3, 0.4), (0.2, 0.1)),
        ((0.3, 0.3), (0.1, 0.2)),
        ((0.4, 0.3), (0.1, 0.2)),
        ((-0.1, 0.3), (0.1, 0.2)),
        ((0.3, 1.1), (0.1, 0.2)),
        ((0.3, math.nan), (0.1, 0.2)),
        ((0.3, 0.4), (0.1, math.inf)),
        ((0.3, 0.4), (0.0, 0.2)),
    ],
)
def test_invalid_tables_are_rejected(throttles, speeds):
    with pytest.raises(ValueError):
        validate_table(throttles, speeds)


@pytest.mark.parametrize(
    ('speed', 'expected'),
    list(zip(SPEEDS, THROTTLES)),
)
def test_lookup_exact_breakpoints(speed, expected):
    actual, clamped = lookup_throttle(speed, THROTTLES, SPEEDS)
    assert actual == pytest.approx(expected)
    assert not clamped


@pytest.mark.parametrize('lower_index', range(3))
def test_lookup_interpolates_each_segment(lower_index):
    speed = (SPEEDS[lower_index] + SPEEDS[lower_index + 1]) / 2.0
    expected = (
        THROTTLES[lower_index] + THROTTLES[lower_index + 1]
    ) / 2.0
    actual, clamped = lookup_throttle(speed, THROTTLES, SPEEDS)
    assert actual == pytest.approx(expected)
    assert not clamped


def test_lookup_floor_maximum_clamp_and_determinism():
    assert lookup_throttle(SPEEDS[0], THROTTLES, SPEEDS) == (
        THROTTLES[0],
        False,
    )
    assert lookup_throttle(SPEEDS[-1], THROTTLES, SPEEDS) == (
        THROTTLES[-1],
        False,
    )
    first = lookup_throttle(0.35, THROTTLES, SPEEDS)
    second = lookup_throttle(0.35, THROTTLES, SPEEDS)
    assert first == (0.380, True)
    assert second == first


def _decision(speed, yaw=0.0, now=1.0, adapter=None):
    adapter = adapter or DriveAdapter(AdapterConfig())
    adapter.update_command(speed, yaw, now)
    return adapter.step(now)


@pytest.mark.parametrize(
    ('speed', 'reason'),
    [
        (0.000, 'explicit_stop'),
        (0.030, 'below_steering_min_speed'),
        (0.049, 'below_steering_min_speed'),
        (0.050, 'below_promotion_threshold'),
        (0.062, 'below_promotion_threshold'),
    ],
)
def test_floor_brake_boundaries(speed, reason):
    decision = _decision(speed)
    assert decision.mode == 'brake'
    assert decision.reason == reason
    assert decision.final_throttle == -1.0
    assert decision.normalized_steering == 0.0


@pytest.mark.parametrize('speed', [0.063, 0.100, 0.125])
def test_floor_promotion_is_inclusive_at_lower_boundary(speed):
    decision = _decision(speed)
    assert decision.mode == 'forward'
    assert decision.effective_speed == pytest.approx(0.126)
    assert decision.feedforward_throttle == pytest.approx(0.340)


def test_exact_floor_uses_normal_table_floor():
    decision = _decision(0.126)
    assert decision.effective_speed == pytest.approx(0.126)
    assert decision.feedforward_throttle == pytest.approx(0.340)


def test_negative_speed_brakes_without_reverse_or_steering():
    decision = _decision(-0.10, 0.3)
    assert decision.reason == 'negative_speed'
    assert decision.final_throttle == -1.0
    assert decision.normalized_steering == 0.0


@pytest.mark.parametrize(
    ('speed', 'yaw'),
    [
        (math.nan, 0.0),
        (math.inf, 0.0),
        (0.2, math.nan),
        (0.2, math.inf),
    ],
)
def test_nonfinite_inputs_brake(speed, yaw):
    decision = _decision(speed, yaw)
    assert decision.reason == 'nonfinite_input'
    assert decision.final_throttle == -1.0
    assert decision.normalized_steering == 0.0


def test_straight_positive_and_negative_steering_are_symmetric():
    straight = _decision(0.20, 0.0)
    left = _decision(0.20, 0.20)
    right = _decision(0.20, -0.20)
    assert straight.normalized_steering == 0.0
    assert left.normalized_steering > 0.0
    assert right.normalized_steering < 0.0
    assert left.normalized_steering == pytest.approx(
        -right.normalized_steering
    )


def test_zero_speed_singularity_and_threshold_do_not_calculate_steering():
    assert _decision(0.0, math.inf).reason == 'nonfinite_input'
    threshold = _decision(0.05, 100.0)
    assert threshold.reason == 'below_promotion_threshold'
    assert threshold.normalized_steering == 0.0


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_curvature_boundary_and_adjacent_values(sign):
    config = AdapterConfig()
    speed = 0.20
    omega_limit = speed * config.maximum_curvature
    below = _decision(speed, sign * (omega_limit - 1e-9))
    exact = _decision(speed, sign * omega_limit)
    above = _decision(speed, sign * (omega_limit + 1e-9))

    assert below.mode == 'forward'
    assert exact.mode == 'forward'
    assert exact.normalized_steering == pytest.approx(sign)
    assert above.mode == 'brake'
    assert above.reason == 'steering_infeasible'
    assert above.final_throttle == -1.0
    assert above.normalized_steering == 0.0


def test_curvature_tolerance_is_explicit_and_boundary_is_accepted():
    config = AdapterConfig()
    tolerance = max(
        CURVATURE_ABS_TOLERANCE,
        config.maximum_curvature * CURVATURE_REL_TOLERANCE,
    )
    accepted = _decision(
        0.20,
        0.20 * (config.maximum_curvature + tolerance),
    )
    rejected = _decision(
        0.20,
        0.20 * (config.maximum_curvature + 2.0 * tolerance),
    )
    assert accepted.mode == 'forward'
    assert rejected.reason == 'steering_infeasible'


def test_rejection_diagnostic_and_later_feasible_command():
    adapter = DriveAdapter(AdapterConfig())
    rejected = _decision(0.20, 0.50, adapter=adapter)
    assert 'mode=brake;reason=steering_infeasible;' in (
        rejected.diagnostic_text()
    )
    accepted = _decision(0.20, 0.20, now=1.01, adapter=adapter)
    assert accepted.mode == 'forward'


def test_no_kick_before_first_command_or_for_stop():
    adapter = DriveAdapter(AdapterConfig())
    assert not adapter.step(0.0).publish_command
    stop = _decision(0.0, adapter=adapter)
    assert stop.mode == 'brake'
    assert not stop.kick_active


def test_kick_starts_and_ends_on_motion_then_lookup_resumes():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    kicked = adapter.step(0.0)
    assert kicked.kick_active
    assert kicked.final_throttle >= 0.380
    assert adapter.take_events() == ['started']

    adapter.update_encoder(False, 0.0, 0.05)
    resumed = adapter.step(0.05)
    assert not resumed.kick_active
    assert resumed.final_throttle == pytest.approx(
        resumed.feedforward_throttle
    )
    assert adapter.take_events() == ['ended:motion_confirmed']


def test_timeout_never_restarts_while_continuously_stationary():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    assert adapter.step(0.0).kick_active
    adapter.update_encoder(True, 0.0, 0.24)
    adapter.update_command(0.20, 0.0, 0.24)
    assert adapter.step(0.24).kick_active
    adapter.update_encoder(True, 0.0, 0.49)
    adapter.update_command(0.20, 0.0, 0.49)
    assert adapter.step(0.49).kick_active
    adapter.update_encoder(True, 0.0, 0.74)
    adapter.update_command(0.20, 0.0, 0.74)
    assert adapter.step(0.74).kick_active
    adapter.update_encoder(True, 0.0, 0.75)
    adapter.update_command(0.20, 0.0, 0.75)
    assert not adapter.step(0.75).kick_active
    assert 'ended:timeout' in adapter.take_events()
    adapter.update_encoder(True, 0.0, 1.0)
    adapter.update_command(0.20, 0.0, 1.0)
    assert not adapter.step(1.0).kick_active


def test_motion_rearms_a_later_stall_kick():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    assert adapter.step(0.0).kick_active
    adapter.update_encoder(False, 2.0, 0.05)
    assert not adapter.step(0.05).kick_active
    adapter.update_encoder(True, 0.0, 0.10)
    assert adapter.step(0.10).kick_active


def test_stop_rearms_next_start():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    adapter.step(0.0)
    adapter.update_encoder(True, 0.0, 0.75)
    adapter.step(0.75)
    adapter.update_command(0.0, 0.0, 0.80)
    assert adapter.step(0.80).mode == 'brake'
    adapter.update_encoder(True, 0.0, 0.81)
    adapter.update_command(0.20, 0.0, 0.81)
    assert adapter.step(0.81).kick_active


def test_stale_command_ends_kick_and_is_silent():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    assert adapter.step(0.0).publish_command
    stale = adapter.step(0.251)
    assert not stale.publish_command
    assert stale.reason == 'stale_command'
    assert adapter.take_events() == [
        'started',
        'ended:stale_command',
    ]


def test_fresh_command_after_stale_resumes_without_false_kick_restart():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    adapter.step(0.0)
    adapter.step(0.251)
    adapter.update_encoder(True, 0.0, 0.30)
    adapter.update_command(0.20, 0.0, 0.30)
    resumed = adapter.step(0.30)
    assert resumed.publish_command
    assert not resumed.kick_active


def test_stale_encoder_suppresses_and_bounds_kick_conservatively():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_encoder(True, 0.0, 0.0)
    adapter.update_command(0.20, 0.0, 0.0)
    assert adapter.step(0.0).kick_active
    adapter.update_command(0.20, 0.0, 0.251)
    decision = adapter.step(0.251)
    assert decision.publish_command
    assert not decision.kick_active
    assert decision.final_throttle == decision.feedforward_throttle
    assert 'ended:encoder_stale' in adapter.take_events()


@pytest.mark.parametrize(
    'changes',
    [
        {'wheelbase': 0.0},
        {'wheelbase': math.nan},
        {'max_steering_angle': 0.0},
        {'steering_min_speed': 0.0},
        {'minimum_moving_speed': 0.0},
        {'minimum_moving_speed': 0.10},
        {'floor_promotion_min_ratio': 0.0},
        {'floor_promotion_min_ratio': 1.1},
        {'breakaway_throttle': -0.1},
        {'breakaway_throttle': 1.1},
        {'breakaway_timeout': 0.0},
        {'motion_confirm_edge_rate': 0.0},
        {'cmd_vel_nav_timeout': 0.0},
        {'encoder_state_timeout': 0.0},
        {'publication_rate': 0.0},
    ],
)
def test_invalid_parameters_are_rejected(changes):
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **changes)
