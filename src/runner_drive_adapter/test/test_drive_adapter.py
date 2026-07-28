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

"""Deterministic tests for feedforward-plus-PI speed control."""

from dataclasses import replace
import math

import pytest

from runner_drive_adapter.drive_adapter import (
    AdapterConfig,
    DriveAdapter,
    lookup_throttle,
    validate_table,
)


THROTTLES = (0.340, 0.350, 0.360, 0.380)
SPEEDS = (0.126, 0.188, 0.233, 0.290)


def _update(
    adapter,
    now,
    speed=0.29,
    yaw=0.0,
    measured=0.0,
    ekf=0.0,
    stationary=False,
    encoder=True,
    motion=True,
):
    adapter.update_command(speed, yaw, now)
    if encoder:
        adapter.update_encoder(
            stationary,
            measured / adapter.config.encoder_metres_per_edge,
            1,
            now,
        )
    if motion:
        adapter.update_motion(ekf, now)
    return adapter.step(now)


def test_valid_monotonic_table_and_interpolation_are_retained():
    validate_table(THROTTLES, SPEEDS)
    assert lookup_throttle(0.126, THROTTLES, SPEEDS)[0] == 0.340
    assert lookup_throttle(0.2105, THROTTLES, SPEEDS)[0] == pytest.approx(
        0.355
    )
    assert lookup_throttle(0.60, THROTTLES, SPEEDS) == (0.380, True)


@pytest.mark.parametrize(
    ('throttles', 'speeds'),
    [
        ((0.3,), (0.1,)),
        ((0.3, 0.4), (0.1,)),
        ((0.3, 0.3), (0.1, 0.2)),
        ((0.4, 0.3), (0.1, 0.2)),
        ((-0.1, 0.3), (0.1, 0.2)),
        ((0.3, math.nan), (0.1, 0.2)),
        ((0.3, 0.4), (0.2, 0.1)),
    ],
)
def test_invalid_tables_are_rejected(throttles, speeds):
    with pytest.raises(ValueError):
        validate_table(throttles, speeds)


def test_feedforward_passes_through_at_zero_error():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.233, measured=0.233)

    assert decision.feedforward_throttle == pytest.approx(0.360)
    assert decision.pi_term == pytest.approx(0.0)
    assert decision.final_throttle == pytest.approx(0.360)


def test_positive_output_saturates_and_integrator_freezes():
    adapter = DriveAdapter(AdapterConfig())
    first = _update(adapter, 0.0, speed=0.60, measured=0.0)
    before = adapter.integrator_state
    second = _update(adapter, 0.10, speed=0.60, measured=0.0)

    assert first.final_throttle == 0.50
    assert second.final_throttle == 0.50
    assert second.saturation_state == 'upper'
    assert adapter.integrator_state == before


def test_negative_output_saturates_and_integrator_freezes():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, measured=3.0, ekf=3.0)
    before = adapter.integrator_state
    decision = _update(
        adapter, 0.10, speed=0.29, measured=3.0, ekf=3.0
    )

    assert decision.final_throttle == -0.20
    assert decision.saturation_state == 'lower'
    assert adapter.integrator_state == before


def test_integrator_accumulates_when_unsaturated():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, measured=0.20, ekf=0.20)
    decision = _update(
        adapter, 0.10, speed=0.29, measured=0.20, ekf=0.20
    )

    assert decision.integrator_state == pytest.approx(0.00108)
    assert decision.final_throttle == pytest.approx(0.40808)


def test_integrator_freezes_after_qualified_wheelspin():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.40, measured=0.30, ekf=0.10)
    _update(adapter, 0.10, speed=0.40, measured=0.30, ekf=0.10)
    before = adapter.integrator_state
    decision = _update(
        adapter, 0.30, speed=0.40, measured=0.30, ekf=0.10
    )

    assert decision.wheelspin_guard
    assert adapter.integrator_state == before


def test_wheelspin_requires_ratio_excess_and_duration():
    adapter = DriveAdapter(AdapterConfig())
    first = _update(
        adapter, 0.0, speed=0.40, measured=0.30, ekf=0.10
    )
    too_soon = _update(
        adapter, 0.19, speed=0.40, measured=0.30, ekf=0.10
    )
    qualified = _update(
        adapter, 0.20, speed=0.40, measured=0.30, ekf=0.10
    )

    assert not first.wheelspin_guard
    assert not too_soon.wheelspin_guard
    assert qualified.wheelspin_guard


def test_integrator_is_disabled_below_sustainable_floor():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(
        adapter, 0.0, speed=0.10, measured=0.0, stationary=True
    )

    assert decision.reason == 'floor_promoted_feedforward'
    assert decision.effective_speed == 0.126
    assert decision.feedforward_throttle == 0.340
    assert decision.integrator_state == 0.0
    assert not decision.integrator_enabled
    assert decision.final_throttle == 0.340


def test_stationary_transition_preloads_integrator_for_breakaway():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(
        adapter, 0.0, speed=0.29, measured=0.0, stationary=True
    )

    assert decision.integrator_state == 0.04
    assert decision.final_throttle == 0.50
    assert decision.saturation_state == 'upper'


def test_breakaway_preload_is_not_reapplied_while_stationary():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, stationary=True)
    first = adapter.integrator_state
    _update(adapter, 0.05, speed=0.29, measured=0.29, stationary=True)

    assert adapter.integrator_state == first


def test_command_speed_clamp_keeps_table_saturated_and_pi_active():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.80, measured=0.60)

    assert decision.reason == 'maximum_speed_clamped'
    assert decision.commanded_speed == 0.80
    assert decision.effective_speed == 0.60
    assert decision.feedforward_throttle == 0.380
    assert decision.integrator_enabled


def test_encoder_speed_uses_measured_metres_per_edge():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.29, 0.0, 0.0)
    adapter.update_motion(0.29, 0.0)
    adapter.update_encoder(False, 10.0, 1, 0.0)
    decision = adapter.step(0.0)

    assert decision.measured_speed == pytest.approx(0.10282)


def test_stale_encoder_freezes_pi_and_uses_feedforward():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.233, 0.0, 0.0)
    decision = adapter.step(0.0)

    assert decision.reason == 'encoder_stale_feedforward'
    assert not decision.integrator_enabled
    assert decision.final_throttle == 0.360


def test_stale_nav_command_is_silent_and_clears_integrator():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, stationary=True)
    decision = adapter.step(0.26)

    assert not decision.publish_command
    assert decision.reason == 'stale_command'
    assert adapter.integrator_state == 0.0


@pytest.mark.parametrize(
    ('speed', 'reason'),
    [
        (0.0, 'explicit_stop'),
        (-0.1, 'negative_speed'),
        (0.05, 'below_promotion_threshold'),
    ],
)
def test_stop_and_invalid_forward_commands_full_brake(speed, reason):
    decision = _update(DriveAdapter(AdapterConfig()), 0.0, speed=speed)

    assert decision.mode == 'brake'
    assert decision.reason == reason
    assert decision.final_throttle == -1.0


@pytest.mark.parametrize(
    ('speed', 'yaw'),
    [(math.nan, 0.0), (math.inf, 0.0), (0.2, math.nan)],
)
def test_nonfinite_input_is_safely_braked(speed, yaw):
    decision = _update(
        DriveAdapter(AdapterConfig()), 0.0, speed=speed, yaw=yaw
    )

    assert decision.reason == 'nonfinite_input'
    assert decision.final_throttle == -1.0


def test_steering_infeasible_full_brake_is_retained():
    config = AdapterConfig()
    decision = _update(
        DriveAdapter(config),
        0.0,
        speed=0.20,
        yaw=0.20 * config.maximum_curvature + 1e-6,
    )

    assert decision.reason == 'steering_infeasible'
    assert decision.final_throttle == -1.0


def test_diagnostics_contain_all_tuning_fields():
    decision = _update(
        DriveAdapter(AdapterConfig()),
        0.0,
        speed=0.29,
        measured=0.20,
    )
    text = decision.diagnostic_text()
    required = (
        'measured_speed=',
        'speed_error=',
        'integrator_state=',
        'feedforward_throttle=',
        'pi_term=',
        'saturation_state=',
        'wheelspin_guard=',
        'final_throttle=',
    )

    assert all(field in text for field in required)


@pytest.mark.parametrize(
    'changes',
    [
        {'maximum_commanded_speed': 0.10},
        {'proportional_gain': 0.0},
        {'integral_gain': 0.0},
        {'integrator_min': 0.20},
        {'integrator_max': -0.30},
        {'output_min': 0.0},
        {'output_max': 0.37},
        {'output_max': 1.01},
        {'breakaway_integrator_preload': 0.20},
        {'encoder_metres_per_edge': 0.0},
        {'wheelspin_speed_ratio': 1.0},
        {'wheelspin_min_speed_excess': -0.1},
        {'wheelspin_qualification_sec': 0.0},
        {'motion_signal_timeout_sec': 0.0},
        {'encoder_state_timeout_sec': 0.0},
    ],
)
def test_parameter_validation_rejects_invalid_combinations(changes):
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **changes)
