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
    active_mode=None,
):
    if active_mode is not None:
        adapter.update_active_mode(active_mode, now)
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
    config = AdapterConfig(proportional_gain=0.60)
    adapter = DriveAdapter(config)
    first = _update(adapter, 0.0, speed=0.60, measured=0.0)
    before = adapter.integrator_state
    second = _update(adapter, 0.10, speed=0.60, measured=0.0)

    assert config.output_max == 0.70
    assert first.final_throttle == 0.70
    assert second.final_throttle == 0.70
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

    assert decision.integrator_state == pytest.approx(0.00054)
    assert decision.final_throttle == pytest.approx(0.40754)


def test_stall_integral_gain_activates_below_ratio_and_uses_high_gain():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, measured=0.115, ekf=0.115)
    decision = _update(
        adapter, 0.10, speed=0.29, measured=0.115, ekf=0.115
    )

    assert decision.stall_integral_gain_active
    assert decision.integral_gain == 0.30
    assert decision.integrator_state == pytest.approx(
        0.30 * (0.29 - 0.115) * 0.10
    )


def test_stall_integral_gain_switch_has_hysteresis():
    adapter = DriveAdapter(AdapterConfig())
    entered = _update(
        adapter, 0.0, speed=0.29, measured=0.115, ekf=0.115
    )
    held = _update(
        adapter, 0.10, speed=0.29, measured=0.13, ekf=0.13
    )
    released = _update(
        adapter, 0.20, speed=0.29, measured=0.145, ekf=0.145
    )

    assert entered.stall_integral_gain_active
    assert held.stall_integral_gain_active
    assert not released.stall_integral_gain_active
    assert released.integral_gain == 0.06


def test_high_integral_gain_activates_above_overspeed_ratio():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, measured=0.465, ekf=0.465)
    decision = _update(
        adapter, 0.10, speed=0.29, measured=0.465, ekf=0.465
    )

    assert decision.stall_integral_gain_active
    assert decision.integral_gain == 0.30
    assert decision.integrator_state == pytest.approx(
        0.30 * (0.29 - 0.465) * 0.10
    )


def test_overspeed_integral_gain_switch_has_hysteresis():
    adapter = DriveAdapter(AdapterConfig())
    entered = _update(
        adapter, 0.0, speed=0.29, measured=0.465, ekf=0.465
    )
    held = _update(
        adapter, 0.10, speed=0.29, measured=0.44, ekf=0.44
    )
    released = _update(
        adapter, 0.20, speed=0.29, measured=0.43, ekf=0.43
    )

    assert entered.stall_integral_gain_active
    assert held.stall_integral_gain_active
    assert not released.stall_integral_gain_active
    assert released.integral_gain == 0.06


def test_integral_gain_activation_boundaries_are_strict():
    config = AdapterConfig()
    adapter = DriveAdapter(config)

    low = _update(
        adapter,
        0.0,
        speed=0.29,
        measured=0.40 * 0.29,
        ekf=0.40 * 0.29,
    )
    high = _update(
        adapter,
        0.10,
        speed=0.29,
        measured=1.60 * 0.29,
        ekf=1.60 * 0.29,
    )

    assert not low.stall_integral_gain_active
    assert not high.stall_integral_gain_active


def test_stall_integral_gain_crosses_full_span_in_three_to_five_seconds():
    config = AdapterConfig()
    span = config.integrator_max - config.integrator_min

    assert span / (config.stall_integral_gain * 0.29) == pytest.approx(
        4.71, abs=0.01
    )
    assert span / (config.stall_integral_gain * 0.45) == pytest.approx(
        3.04, abs=0.01
    )


def test_overspeed_correction_reaches_negative_bound_in_2_25_seconds():
    config = AdapterConfig()
    overspeed_error = 0.37

    correction_time = (
        abs(config.integrator_min)
        / (config.stall_integral_gain * overspeed_error)
    )

    assert config.integrator_min == -0.25
    assert config.output_min == -0.20
    assert correction_time == pytest.approx(2.25, abs=0.01)


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
    assert decision.final_throttle == pytest.approx(0.507)
    assert decision.saturation_state == 'none'


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


def test_absent_and_fresh_suppress_mode_allow_normal_integration():
    absent = DriveAdapter(AdapterConfig())
    _update(absent, 0.0, speed=0.29, measured=0.20, ekf=0.20)
    absent_decision = _update(
        absent, 0.10, speed=0.29, measured=0.20, ekf=0.20
    )
    suppress = DriveAdapter(AdapterConfig())
    _update(
        suppress,
        0.0,
        speed=0.29,
        measured=0.20,
        ekf=0.20,
        active_mode='teleop_suppress',
    )
    suppress_decision = _update(
        suppress,
        0.10,
        speed=0.29,
        measured=0.20,
        ekf=0.20,
        active_mode='teleop_suppress',
    )

    assert not absent_decision.active_mode_received
    assert not absent_decision.preempted
    assert absent_decision.integrator_enabled
    assert not suppress_decision.preempted
    assert suppress_decision.integrator_enabled
    assert suppress_decision.integrator_state == pytest.approx(
        absent_decision.integrator_state
    )


def test_fresh_physical_mode_freezes_and_decays_integral():
    config = AdapterConfig()
    adapter = DriveAdapter(config)
    adapter._integrator = 0.10
    _update(
        adapter,
        0.0,
        measured=0.20,
        ekf=0.20,
        active_mode='manual',
    )
    decision = _update(
        adapter,
        0.40,
        measured=0.20,
        ekf=0.20,
        active_mode='manual',
    )

    assert decision.publish_command
    assert decision.preempted
    assert not decision.integrator_enabled
    assert decision.integral_decay_active
    assert decision.integrator_state == pytest.approx(0.075)
    assert decision.feedforward_throttle == pytest.approx(0.38)


@pytest.mark.parametrize(
    ('initial', 'expected'),
    [(0.01, 0.0), (-0.01, 0.0), (-0.10, -0.075)],
)
def test_preemption_decay_never_crosses_zero(initial, expected):
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = initial
    _update(adapter, 0.0, active_mode='manual')
    decision = _update(adapter, 0.40, active_mode='manual')

    assert decision.integrator_state == pytest.approx(expected)


def test_stale_active_mode_restores_existing_integration_behavior():
    config = AdapterConfig()
    adapter = DriveAdapter(config)
    adapter.update_active_mode('manual', 0.0)
    _update(adapter, 0.0, measured=0.20, ekf=0.20)
    before = adapter.integrator_state
    decision = _update(
        adapter,
        config.active_mode_timeout_sec + 0.01,
        measured=0.20,
        ekf=0.20,
    )

    assert decision.active_mode_received
    assert not decision.active_mode_fresh
    assert not decision.preempted
    assert decision.integrator_enabled
    assert decision.integrator_state > before


def test_preemption_decay_has_precedence_over_wheelspin_freeze():
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = 0.10
    _update(
        adapter,
        0.0,
        speed=0.40,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )
    _update(
        adapter,
        0.10,
        speed=0.40,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )
    decision = _update(
        adapter,
        0.30,
        speed=0.40,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )

    assert decision.preempted
    assert decision.wheelspin_guard
    assert not decision.integrator_enabled
    assert decision.integral_decay_active
    assert decision.integrator_state < 0.10


def test_below_floor_reset_remains_stronger_than_preemption_decay():
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = 0.10
    decision = _update(
        adapter,
        0.0,
        speed=0.10,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )

    assert decision.preempted
    assert not decision.integrator_enabled
    assert not decision.integral_decay_active
    assert decision.integrator_state == 0.0
    assert decision.reason == 'floor_promoted_feedforward'


def test_preemption_wheelspin_and_below_floor_keep_reset_precedence():
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = 0.10
    _update(
        adapter,
        0.0,
        speed=0.10,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )
    decision = _update(
        adapter,
        0.20,
        speed=0.10,
        measured=0.30,
        ekf=0.10,
        active_mode='manual',
    )

    assert decision.preempted
    assert decision.wheelspin_guard
    assert not decision.integrator_enabled
    assert not decision.integral_decay_active
    assert decision.integrator_state == 0.0


def test_preemption_does_not_change_feedforward_or_stall_gain():
    normal = DriveAdapter(AdapterConfig())
    preempted = DriveAdapter(AdapterConfig())
    normal_decision = _update(
        normal, 0.0, speed=0.29, measured=0.0, ekf=0.0
    )
    preempted_decision = _update(
        preempted,
        0.0,
        speed=0.29,
        measured=0.0,
        ekf=0.0,
        active_mode='fixed_throttle',
    )

    assert preempted_decision.publish_command
    assert preempted_decision.feedforward_throttle == (
        normal_decision.feedforward_throttle
    )
    assert preempted_decision.integral_gain == normal_decision.integral_gain
    assert preempted_decision.stall_integral_gain_active
    assert normal_decision.stall_integral_gain_active


def test_preemption_defers_but_does_not_discard_breakaway_preload():
    adapter = DriveAdapter(AdapterConfig())
    preempted = _update(
        adapter,
        0.0,
        speed=0.29,
        measured=0.0,
        ekf=0.0,
        stationary=True,
        active_mode='manual',
    )
    resumed = _update(
        adapter,
        0.05,
        speed=0.29,
        measured=0.0,
        ekf=0.0,
        stationary=True,
        active_mode='teleop_suppress',
    )

    assert preempted.integrator_state == 0.0
    assert resumed.integrator_state >= (
        adapter.config.breakaway_integrator_preload
    )


def test_long_preemption_resume_matches_normal_standing_start():
    config = AdapterConfig()
    normal = DriveAdapter(config)
    normal.step(0.0)
    normal_decision = _update(
        normal,
        0.05,
        speed=0.29,
        measured=0.0,
        ekf=0.0,
        active_mode='teleop_suppress',
    )

    resumed = DriveAdapter(config)
    resumed._integrator = config.integrator_max
    resumed.step(0.0)
    now = 0.05
    while now <= 2.65:
        _update(
            resumed,
            now,
            speed=0.29,
            measured=0.0,
            ekf=0.0,
            active_mode='manual',
        )
        now += 0.05
    assert resumed.integrator_state == pytest.approx(0.0, abs=1e-12)
    resumed_decision = _update(
        resumed,
        now,
        speed=0.29,
        measured=0.0,
        ekf=0.0,
        active_mode='teleop_suppress',
    )

    assert resumed_decision.feedforward_throttle == (
        normal_decision.feedforward_throttle
    )
    assert resumed_decision.proportional_term == (
        normal_decision.proportional_term
    )
    assert resumed_decision.integral_gain == normal_decision.integral_gain
    assert resumed_decision.integrator_state == pytest.approx(
        normal_decision.integrator_state, abs=1e-12
    )
    assert resumed_decision.final_throttle == pytest.approx(
        normal_decision.final_throttle, abs=1e-12
    )
    assert resumed_decision.final_throttle <= (
        normal_decision.final_throttle + 1e-12
    )


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


@pytest.mark.parametrize('direction', [-1.0, 1.0])
def test_infeasible_steering_is_clamped_without_braking(direction):
    config = AdapterConfig()
    decision = _update(
        DriveAdapter(config),
        0.0,
        speed=0.152,
        yaw=direction * 0.3625,
        measured=0.152,
    )

    assert abs(0.3625 / 0.152) > config.maximum_curvature
    assert decision.mode == 'forward'
    assert decision.reason == 'closed_loop'
    assert decision.normalized_steering == direction
    assert decision.steering_saturated
    assert decision.final_throttle > 0.0


def test_feasible_steering_is_not_reported_as_saturated():
    config = AdapterConfig()
    decision = _update(
        DriveAdapter(config),
        0.0,
        speed=0.20,
        yaw=0.20 * config.maximum_curvature * 0.5,
    )

    assert not decision.steering_saturated


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
        'integral_gain=',
        'stall_integral_gain_active=',
        'saturation_state=',
        'wheelspin_guard=',
        'final_throttle=',
        'steering_saturated=',
        'active_mode_received=',
        'active_mode_fresh=',
        'active_mode=',
        'preempted=',
        'integral_decay_active=',
    )

    assert all(field in text for field in required)


@pytest.mark.parametrize(
    'changes',
    [
        {'maximum_commanded_speed': 0.10},
        {'proportional_gain': 0.0},
        {'integral_gain': 0.0},
        {'stall_integral_gain': 0.06},
        {'stall_integral_gain_activation_ratio': 0.0},
        {'stall_integral_gain_activation_ratio': 1.0},
        {'stall_integral_gain_hysteresis': 0.0},
        {
            'stall_integral_gain_activation_ratio': 0.95,
            'stall_integral_gain_hysteresis': 0.10,
        },
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
        {'active_mode_timeout_sec': 0.0},
        {'preemption_integrator_decay_rate': 0.0},
    ],
)
def test_parameter_validation_rejects_invalid_combinations(changes):
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **changes)
