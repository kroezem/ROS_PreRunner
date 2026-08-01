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

"""Deterministic tests for bounded parallel-PI speed control."""

from dataclasses import replace
import math
import struct

import pytest

from runner_drive_adapter.drive_adapter import (
    AdapterConfig,
    DriveAdapter,
    INTEGRATOR_FREEZE_PRECEDENCE,
    IntegratorFreezeReason,
    linear_feedforward,
)
from runner_interfaces.msg import AdapterState


def _feedforward(speed):
    config = AdapterConfig()
    return linear_feedforward(
        speed,
        config.feedforward_effort_per_speed,
        config.feedforward_effort_intercept,
    )


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
    sample_time=None,
    pending_direction=1,
):
    if active_mode is not None:
        adapter.update_active_mode(active_mode, now)
    adapter.update_command(speed, yaw, now)
    if encoder:
        adapter.update_encoder(
            stationary,
            measured / adapter.config.encoder_metres_per_edge,
            pending_direction,
            now,
            now if sample_time is None else sample_time,
        )
    if motion:
        adapter.update_motion(ekf, now)
    return adapter.step(now)


@pytest.mark.parametrize(
    ('speed', 'expected_command'),
    [
        (0.250, 0.0471000),
        (0.290, 0.0518520),
        (0.450, 0.0708600),
        (0.600, 0.0886800),
    ],
)
def test_characterized_md13s_linear_inverse(speed, expected_command):
    decision = _update(
        DriveAdapter(AdapterConfig()),
        0.0,
        speed=speed,
        measured=speed,
        ekf=speed,
    )

    assert decision.feedforward_throttle == pytest.approx(
        expected_command, abs=1e-7
    )
    assert decision.final_throttle == pytest.approx(
        expected_command, abs=1e-7
    )


def test_feedforward_passes_through_at_zero_error():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.30, measured=0.30)

    assert decision.feedforward_throttle == pytest.approx(
        _feedforward(0.30)
    )
    assert decision.pi_term == pytest.approx(0.0)
    assert decision.final_throttle == pytest.approx(_feedforward(0.30))


def test_positive_output_saturates_at_compatibility_ceiling():
    config = AdapterConfig(proportional_gain=0.60)
    adapter = DriveAdapter(config)
    first = _update(adapter, 0.0, speed=0.60, measured=0.0)
    second = _update(adapter, 0.10, speed=0.60, measured=0.0)

    assert config.output_max == 0.14
    assert first.final_throttle == 0.14
    assert second.final_throttle == 0.14
    assert second.saturation_state == 'upper'
    assert adapter.integrator_state == 0.0


def test_overspeed_output_saturates_at_zero():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, measured=3.0, ekf=3.0)
    decision = _update(
        adapter, 0.10, speed=0.29, measured=3.0, ekf=3.0
    )

    assert decision.final_throttle == 0.0
    assert decision.saturation_state == 'lower'
    assert adapter.integrator_state == 0.0


@pytest.mark.parametrize('measured', [0.0, 0.20, 0.29, 0.50])
def test_output_is_feedforward_plus_kp_005_correction_only(measured):
    config = AdapterConfig()
    decision = _update(
        DriveAdapter(config), 0.0, speed=0.29, measured=measured
    )
    expected_correction = 0.05 * (0.29 - measured)
    expected_output = max(
        config.output_min,
        min(config.output_max, _feedforward(0.29) + expected_correction),
    )

    assert config.proportional_gain == 0.05
    assert config.integral_gain == 0.0
    assert decision.proportional_term == pytest.approx(expected_correction)
    assert decision.integrator_state == 0.0
    assert decision.pi_term == pytest.approx(expected_correction)
    assert decision.final_throttle == pytest.approx(expected_output)


def test_disabled_integrator_does_not_affect_output():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.29, measured=0.20)

    expected = _feedforward(0.29) + 0.05 * (0.29 - 0.20)
    assert decision.final_throttle == pytest.approx(expected)
    assert decision.integrator_state == 0.0
    assert decision.integral_gain == 0.0
    assert not decision.integrator_enabled
    assert not decision.stall_integral_gain_active
    assert not decision.integral_decay_active


def test_live_tuning_defaults_remain_committed_values_without_writes():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.29, measured=0.20)

    assert adapter.config.proportional_gain == 0.05
    assert adapter.config.integral_gain == 0.0
    assert decision.proportional_term == pytest.approx(0.05 * 0.09)
    assert decision.integrator_state == 0.0


def _set_selected(adapter, now, linear=0.05, angular=0.0):
    adapter.update_adapter_output(linear, angular, now)
    adapter.update_mux_output(linear, angular, now)


def test_parallel_integrator_uses_encoder_sample_dt_and_magnitude_error():
    config = AdapterConfig(integral_gain=0.10)
    adapter = DriveAdapter(config)
    _set_selected(adapter, 0.0)
    first = _update(
        adapter, 0.0, speed=0.29, measured=-0.19, sample_time=10.0
    )
    _set_selected(adapter, 0.1)
    second = _update(
        adapter, 0.1, speed=0.29, measured=-0.19, sample_time=10.1
    )

    assert first.integrator_freeze_reason == IntegratorFreezeReason.INVALID_DT
    assert first.integrator_state == 0.0
    assert second.speed_error == pytest.approx(0.10)
    assert second.integrator_state == pytest.approx(0.001)
    assert second.pi_term == pytest.approx(0.05 * 0.10 + 0.001)
    assert second.final_throttle == pytest.approx(
        _feedforward(0.29) + 0.05 * 0.10 + 0.001
    )
    assert second.integrator_freeze_reason == IntegratorFreezeReason.NONE


@pytest.mark.parametrize('integral_gain', [0.0, 0.10])
@pytest.mark.parametrize('bad_dt', [0.0, -0.1, 0.500001, math.nan])
def test_invalid_sample_dt_skips_integration(bad_dt, integral_gain):
    adapter = DriveAdapter(AdapterConfig(integral_gain=integral_gain))
    _set_selected(adapter, 0.0)
    _update(adapter, 0.0, sample_time=10.0)
    _set_selected(adapter, 0.1)
    decision = _update(adapter, 0.1, sample_time=10.0 + bad_dt)

    assert decision.integrator_state == 0.0
    assert decision.integrator_freeze_reason == IntegratorFreezeReason.INVALID_DT


def test_integrator_is_bounded_to_configured_normalized_effort():
    adapter = DriveAdapter(AdapterConfig(integral_gain=0.10))
    for index in range(12):
        now = index * 0.1
        _set_selected(adapter, now)
        decision = _update(
            adapter,
            now,
            speed=0.29,
            measured=0.0,
            sample_time=20.0 + now,
        )

    assert adapter.config.integrator_bound == 0.005
    assert decision.integrator_state == 0.005


@pytest.mark.parametrize(
    ('speed', 'measured', 'expected_saturation'),
    [(0.60, 0.0, 'upper'), (0.29, 3.0, 'lower')],
)
@pytest.mark.parametrize('integral_gain', [0.0, 0.10])
def test_conditional_anti_windup_rejects_farther_saturation(
    speed, measured, expected_saturation, integral_gain
):
    adapter = DriveAdapter(AdapterConfig(
        proportional_gain=0.60,
        integral_gain=integral_gain,
    ))
    _set_selected(adapter, 0.0)
    _update(adapter, 0.0, speed=speed, measured=measured, sample_time=30.0)
    _set_selected(adapter, 0.1)
    decision = _update(
        adapter, 0.1, speed=speed, measured=measured, sample_time=30.1
    )

    assert decision.saturation_state == expected_saturation
    assert decision.integrator_state == 0.0
    assert (
        decision.integrator_freeze_reason
        == IntegratorFreezeReason.ANTI_WINDUP
    )


@pytest.mark.parametrize(
    ('condition', 'expected'),
    [
        ('feedback_stale', IntegratorFreezeReason.FEEDBACK_STALE),
        ('wheelspin', IntegratorFreezeReason.WHEELSPIN),
        ('direction_unavailable', IntegratorFreezeReason.DIRECTION_UNAVAILABLE),
        ('direction_mismatch', IntegratorFreezeReason.DIRECTION_MISMATCH),
        ('arbitration_unavailable', IntegratorFreezeReason.ARBITRATION_UNAVAILABLE),
        ('output_not_selected', IntegratorFreezeReason.OUTPUT_NOT_SELECTED),
    ],
)
def test_operational_freeze_conditions_are_reported_with_ki_zero(
    condition, expected
):
    adapter = DriveAdapter(AdapterConfig())
    if condition != 'arbitration_unavailable':
        adapter.update_adapter_output(0.05, 0.0, 0.0)
        adapter.update_mux_output(
            0.04 if condition == 'output_not_selected' else 0.05,
            0.0,
            0.0,
        )
    pending_direction = 0 if condition == 'direction_unavailable' else 1
    if condition == 'direction_mismatch':
        pending_direction = -1
    measured = 0.30 if condition == 'wheelspin' else 0.20
    ekf = 0.10 if condition == 'wheelspin' else 0.20
    _update(
        adapter,
        0.0,
        measured=measured,
        ekf=ekf,
        pending_direction=pending_direction,
        sample_time=40.0,
    )

    now = 0.20 if condition in ('feedback_stale', 'wheelspin') else 0.10
    adapter.update_command(0.29, 0.0, now)
    if condition != 'feedback_stale':
        adapter.update_encoder(
            False,
            measured / adapter.config.encoder_metres_per_edge,
            pending_direction,
            now,
            40.0 + now,
        )
    adapter.update_motion(ekf, now)
    if condition != 'arbitration_unavailable':
        adapter.update_adapter_output(0.05, 0.0, now)
        adapter.update_mux_output(
            0.04 if condition == 'output_not_selected' else 0.05,
            0.0,
            now,
        )
    decision = adapter.step(now)

    assert decision.integrator_state == 0.0
    assert decision.integrator_freeze_reason == expected


def test_command_state_freeze_conditions_are_reported_with_ki_zero():
    no_command = DriveAdapter(AdapterConfig()).step(0.0)
    invalid = _update(
        DriveAdapter(AdapterConfig()), 0.0, speed=math.nan
    )
    stopped = _update(DriveAdapter(AdapterConfig()), 0.0, speed=0.0)

    assert (
        no_command.integrator_freeze_reason
        == IntegratorFreezeReason.NO_COMMAND
    )
    assert (
        invalid.integrator_freeze_reason
        == IntegratorFreezeReason.INVALID_COMMAND
    )
    assert (
        stopped.integrator_freeze_reason
        == IntegratorFreezeReason.ZERO_COMMAND
    )


def test_gain_disabled_is_only_the_last_applicable_reason():
    adapter = DriveAdapter(AdapterConfig())
    _set_selected(adapter, 0.0)
    first = _update(adapter, 0.0, measured=0.20, sample_time=45.0)
    _set_selected(adapter, 0.1)
    second = _update(adapter, 0.1, measured=0.20, sample_time=45.1)

    assert first.integrator_freeze_reason == IntegratorFreezeReason.INVALID_DT
    assert (
        second.integrator_freeze_reason
        == IntegratorFreezeReason.GAIN_DISABLED
    )


def test_simultaneous_conditions_follow_documented_precedence():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.29, 0.0, 0.0)
    adapter.update_encoder(False, math.nan, 0, 0.0, None)
    adapter.update_motion(0.0, 0.0)

    decision = adapter.step(0.20)

    assert (
        decision.integrator_freeze_reason
        == IntegratorFreezeReason.FEEDBACK_STALE
    )


def test_documented_freeze_precedence_is_complete_and_deterministic():
    assert INTEGRATOR_FREEZE_PRECEDENCE == (
        IntegratorFreezeReason.NO_COMMAND,
        IntegratorFreezeReason.INVALID_COMMAND,
        IntegratorFreezeReason.ZERO_COMMAND,
        IntegratorFreezeReason.FEEDBACK_STALE,
        IntegratorFreezeReason.WHEELSPIN,
        IntegratorFreezeReason.DIRECTION_UNAVAILABLE,
        IntegratorFreezeReason.DIRECTION_MISMATCH,
        IntegratorFreezeReason.ARBITRATION_UNAVAILABLE,
        IntegratorFreezeReason.OUTPUT_NOT_SELECTED,
        IntegratorFreezeReason.INVALID_DT,
        IntegratorFreezeReason.ANTI_WINDUP,
        IntegratorFreezeReason.GAIN_DISABLED,
    )
    assert set(INTEGRATOR_FREEZE_PRECEDENCE) == (
        set(IntegratorFreezeReason) - {IntegratorFreezeReason.NONE}
    )


def test_integrator_carries_across_zero_direction_change_and_mismatch():
    adapter = DriveAdapter(AdapterConfig(integral_gain=0.01))
    _set_selected(adapter, 0.0)
    _update(adapter, 0.0, measured=0.20, sample_time=50.0)
    _set_selected(adapter, 0.1)
    moving = _update(adapter, 0.1, measured=0.20, sample_time=50.1)
    held = moving.integrator_state

    stopped = _update(adapter, 0.2, speed=0.0, sample_time=50.2)
    mismatched_reverse = _update(
        adapter,
        0.3,
        speed=-0.29,
        measured=0.20,
        sample_time=50.3,
        pending_direction=1,
    )
    _set_selected(adapter, 0.4, linear=-0.05)
    latched_reverse = _update(
        adapter,
        0.4,
        speed=-0.29,
        measured=0.20,
        sample_time=50.4,
        pending_direction=-1,
    )

    assert held > 0.0
    assert stopped.integrator_state == held
    assert stopped.integrator_freeze_reason == IntegratorFreezeReason.ZERO_COMMAND
    assert mismatched_reverse.final_throttle < 0.0
    assert mismatched_reverse.integrator_state == held
    assert (
        mismatched_reverse.integrator_freeze_reason
        == IntegratorFreezeReason.DIRECTION_MISMATCH
    )
    assert latched_reverse.integrator_state > held
    assert latched_reverse.integrator_freeze_reason == IntegratorFreezeReason.NONE
    assert latched_reverse.final_throttle < 0.0


def test_successful_live_integral_gain_change_is_the_only_state_reset():
    adapter = DriveAdapter(AdapterConfig(integral_gain=0.10))
    adapter._integrator = 0.004

    adapter.set_integral_gain(0.20)

    assert adapter.config.integral_gain == 0.20
    assert adapter.integrator_state == 0.0


def test_live_proportional_gain_takes_effect_on_next_cycle_without_reset():
    adapter = DriveAdapter(AdapterConfig())
    first = _update(adapter, 0.0, speed=0.29, measured=0.20)
    adapter._integrator = 0.004

    adapter.set_proportional_gain(0.03)
    second = _update(adapter, 0.1, speed=0.29, measured=0.20)

    assert first.proportional_term == pytest.approx(0.05 * 0.09)
    assert second.proportional_term == pytest.approx(0.03 * 0.09)
    assert second.integrator_state == 0.004


@pytest.mark.parametrize('value', [True, 0, 0.0, -0.01, math.nan])
def test_rejected_proportional_gain_does_not_change_active_state(value):
    adapter = DriveAdapter(AdapterConfig(integral_gain=0.10))
    adapter._integrator = 0.004

    with pytest.raises((TypeError, ValueError)):
        adapter.set_proportional_gain(value)

    assert adapter.config.proportional_gain == 0.05
    assert adapter.config.integral_gain == 0.10
    assert adapter.integrator_state == 0.004


def test_combined_live_gain_update_is_atomic_on_rejection():
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = 0.004

    with pytest.raises(ValueError):
        adapter.set_controller_gains(
            proportional_gain=0.03,
            integral_gain=-0.01,
        )

    assert adapter.config.proportional_gain == 0.05
    assert adapter.config.integral_gain == 0.0
    assert adapter.integrator_state == 0.004


def test_freeze_enum_values_match_the_typed_adapter_state_interface():
    assert int(IntegratorFreezeReason.NONE) == AdapterState.INTEGRATOR_ACTIVE
    assert (
        int(IntegratorFreezeReason.GAIN_DISABLED)
        == AdapterState.INTEGRATOR_GAIN_DISABLED
    )
    assert (
        int(IntegratorFreezeReason.OUTPUT_NOT_SELECTED)
        == AdapterState.INTEGRATOR_OUTPUT_NOT_SELECTED
    )
    assert (
        int(IntegratorFreezeReason.ANTI_WINDUP)
        == AdapterState.INTEGRATOR_ANTI_WINDUP
    )


def test_mux_selection_comparison_uses_bounded_numeric_tolerance():
    adapter = DriveAdapter(AdapterConfig(integral_gain=0.10))
    adapter.update_adapter_output(0.05, -0.10, 0.0)
    adapter.update_mux_output(0.05 + 0.5e-9, -0.10, 0.0)
    matching = _update(adapter, 0.0, sample_time=60.0)
    adapter.update_adapter_output(0.05, -0.10, 0.1)
    adapter.update_mux_output(0.05 + 2.0e-9, -0.10, 0.1)
    different = _update(adapter, 0.1, sample_time=60.1)

    assert matching.integrator_freeze_reason == IntegratorFreezeReason.INVALID_DT
    assert (
        different.integrator_freeze_reason
        == IntegratorFreezeReason.OUTPUT_NOT_SELECTED
    )


def test_ki_zero_replay_is_bit_identical_to_stage_2b_arithmetic():
    config = AdapterConfig()

    def stage_2b_output(speed, measured, feedback_fresh=True, kp=0.05):
        if speed <= 0.0:
            return 0.0
        effective = min(
            config.maximum_commanded_speed,
            max(speed, config.minimum_moving_speed),
        )
        feedforward = _feedforward(effective)
        proportional = (
            kp * (effective - abs(measured)) if feedback_fresh else 0.0
        )
        raw = feedforward + proportional
        return max(config.output_min, min(config.output_max, raw))

    cases = []
    adapter = DriveAdapter(config)
    cases.append(('normal', _update(
        adapter, 0.0, speed=0.29, measured=0.20
    ), stage_2b_output(0.29, 0.20)))
    cases.append(('direction_mismatch', _update(
        adapter, 0.1, speed=0.29, measured=0.20, pending_direction=-1
    ), stage_2b_output(0.29, 0.20)))
    adapter.update_adapter_output(0.05, 0.0, 0.2)
    adapter.update_mux_output(0.04, 0.0, 0.2)
    cases.append(('arbitration_loss', _update(
        adapter, 0.2, speed=0.29, measured=0.20
    ), stage_2b_output(0.29, 0.20)))
    _update(adapter, 0.3, speed=0.40, measured=0.30, ekf=0.10)
    cases.append(('wheelspin', _update(
        adapter, 0.5, speed=0.40, measured=0.30, ekf=0.10
    ), stage_2b_output(0.40, 0.30)))
    cases.append(('zero', _update(
        adapter, 0.51, speed=0.0
    ), stage_2b_output(0.0, 0.0)))

    stale = DriveAdapter(config)
    stale.update_command(0.29, 0.0, 0.0)
    stale.update_encoder(False, 1.0, 1, 0.0, 70.0)
    stale.update_command(0.29, 0.0, 0.26)
    cases.append((
        'stale_feedback',
        stale.step(0.26),
        stage_2b_output(0.29, 0.0, feedback_fresh=False),
    ))
    saturated = DriveAdapter(replace(config, proportional_gain=0.60))
    cases.append(('saturation', _update(
        saturated, 0.0, speed=0.60, measured=0.0
    ), stage_2b_output(0.60, 0.0, kp=0.60)))

    assert {label for label, _, _ in cases} == {
        'normal',
        'direction_mismatch',
        'arbitration_loss',
        'wheelspin',
        'zero',
        'stale_feedback',
        'saturation',
    }
    for _, decision, expected in cases:
        assert struct.pack('!d', decision.final_throttle) == struct.pack(
            '!d', expected
        )


def test_reverse_floor_preserves_sign_with_magnitude_domain_diagnostics():
    decision = _update(
        DriveAdapter(AdapterConfig()),
        0.0,
        speed=-0.10,
        measured=0.0,
        pending_direction=-1,
    )

    expected_magnitude = _feedforward(0.25) + 0.05 * 0.25
    assert decision.mode == 'reverse'
    assert decision.reason == 'floor_promoted'
    assert decision.commanded_speed == -0.10
    assert decision.effective_speed == -0.25
    assert decision.speed_error == pytest.approx(0.25)
    assert decision.feedforward_throttle == pytest.approx(_feedforward(0.25))
    assert decision.proportional_term == pytest.approx(0.05 * 0.25)
    assert decision.integrator_state == 0.0
    assert decision.pi_term == pytest.approx(0.05 * 0.25)
    assert decision.final_throttle == pytest.approx(-expected_magnitude)


def test_reverse_maximum_speed_clamp_preserves_sign():
    decision = _update(
        DriveAdapter(AdapterConfig()),
        0.0,
        speed=-0.80,
        measured=0.60,
        ekf=-0.60,
        pending_direction=-1,
    )

    assert decision.reason == 'maximum_speed_clamped'
    assert decision.commanded_speed == -0.80
    assert decision.effective_speed == -0.60
    assert decision.speed_error == 0.0
    assert decision.final_throttle == pytest.approx(-_feedforward(0.60))


def test_reverse_stale_feedback_uses_signed_feedforward_only():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(-0.30, 0.0, 0.0)

    decision = adapter.step(0.0)

    assert decision.mode == 'reverse'
    assert decision.reason == 'encoder_stale_feedforward'
    assert decision.effective_speed == -0.30
    assert decision.speed_error == pytest.approx(0.30)
    assert decision.proportional_term == 0.0
    assert decision.feedforward_throttle == pytest.approx(_feedforward(0.30))
    assert decision.final_throttle == pytest.approx(-_feedforward(0.30))
    assert (
        decision.integrator_freeze_reason
        == IntegratorFreezeReason.FEEDBACK_STALE
    )


def test_reverse_output_saturates_with_symmetric_authority():
    adapter = DriveAdapter(AdapterConfig(proportional_gain=0.60))

    decision = _update(
        adapter,
        0.0,
        speed=-0.60,
        measured=0.0,
        pending_direction=-1,
    )

    assert decision.saturation_state == 'upper'
    assert decision.feedforward_throttle > 0.0
    assert decision.proportional_term > 0.0
    assert decision.final_throttle == -0.14


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


def test_nonzero_request_is_promoted_before_feedforward_and_p_error():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(
        adapter, 0.0, speed=0.10, measured=0.0, stationary=True
    )

    assert decision.reason == 'floor_promoted'
    assert decision.commanded_speed == 0.10
    assert decision.effective_speed == 0.25
    assert decision.speed_error == pytest.approx(0.25)
    assert decision.feedforward_throttle == pytest.approx(
        _feedforward(0.25)
    )
    assert decision.proportional_term == pytest.approx(0.05 * 0.25)
    assert decision.integrator_state == 0.0
    assert not decision.integrator_enabled
    assert decision.final_throttle == pytest.approx(
        _feedforward(0.25) + 0.05 * 0.25
    )
    assert not decision.feedforward_floor_violation


def test_zero_request_remains_zero_without_feedforward():
    decision = _update(DriveAdapter(AdapterConfig()), 0.0, speed=0.0)

    assert decision.commanded_speed == 0.0
    assert decision.effective_speed == 0.0
    assert decision.feedforward_throttle == 0.0
    assert decision.final_throttle == 0.0
    assert not decision.feedforward_floor_violation


@pytest.mark.parametrize('speed', [1e-12, 0.01, 0.249999, 0.25, 0.60, 0.80])
def test_valid_nonzero_commands_never_assert_feedforward_floor_diagnostic(speed):
    decision = _update(DriveAdapter(AdapterConfig()), 0.0, speed=speed)

    assert 0.25 <= abs(decision.effective_speed) <= 0.60
    assert decision.feedforward_throttle >= 0.04
    assert not decision.feedforward_floor_violation


def test_feedforward_floor_diagnostic_asserts_if_floor_is_bypassed():
    config = AdapterConfig(minimum_moving_speed=0.01)
    decision = _update(
        DriveAdapter(config), 0.0, speed=0.01, measured=0.01
    )

    assert decision.effective_speed == 0.01
    assert decision.feedforward_throttle == pytest.approx(_feedforward(0.01))
    assert decision.feedforward_throttle < 0.04
    assert decision.final_throttle == pytest.approx(
        decision.feedforward_throttle
    )
    assert decision.feedforward_floor_violation


def test_stationary_transition_does_not_add_integral_or_breakaway_output():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(
        adapter, 0.0, speed=0.29, measured=0.0, stationary=True
    )

    assert decision.integrator_state == 0.0
    assert decision.final_throttle == pytest.approx(
        _feedforward(0.29) + 0.05 * 0.29
    )


def test_command_speed_clamp_keeps_inverse_bounded_and_integral_disabled():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.80, measured=0.60)

    assert decision.reason == 'maximum_speed_clamped'
    assert decision.commanded_speed == 0.80
    assert decision.effective_speed == 0.60
    assert decision.speed_error == 0.0
    assert decision.feedforward_throttle == pytest.approx(_feedforward(0.60))
    assert not decision.integrator_enabled


def test_encoder_speed_uses_measured_metres_per_edge():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.29, 0.0, 0.0)
    adapter.update_motion(0.29, 0.0)
    adapter.update_encoder(False, 10.0, 1, 0.0)
    decision = adapter.step(0.0)

    assert decision.measured_speed == pytest.approx(0.10282)


def test_stale_encoder_disables_p_and_uses_feedforward():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.30, 0.0, 0.0)
    decision = adapter.step(0.0)

    assert decision.reason == 'encoder_stale_feedforward'
    assert not decision.integrator_enabled
    assert decision.effective_speed == 0.30
    assert decision.final_throttle == pytest.approx(_feedforward(0.30))


def test_stale_nav_command_is_silent_and_clears_integrator():
    adapter = DriveAdapter(AdapterConfig())
    _update(adapter, 0.0, speed=0.29, stationary=True)
    decision = adapter.step(0.26)

    assert not decision.publish_command
    assert decision.reason == 'stale_command'
    assert adapter.integrator_state == 0.0


def test_preemption_does_not_change_feedforward_or_proportional_feedback():
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
    assert preempted_decision.proportional_term == (
        normal_decision.proportional_term
    )
    assert preempted_decision.final_throttle == (
        normal_decision.final_throttle
    )
    assert preempted_decision.integral_gain == 0.0
    assert not preempted_decision.stall_integral_gain_active


def test_stop_command_full_brakes():
    decision = _update(DriveAdapter(AdapterConfig()), 0.0, speed=0.0)

    assert decision.mode == 'brake'
    assert decision.reason == 'explicit_stop'
    assert decision.final_throttle == 0.0


@pytest.mark.parametrize(
    ('speed', 'yaw'),
    [(math.nan, 0.0), (math.inf, 0.0), (0.2, math.nan)],
)
def test_nonfinite_input_is_safely_braked(speed, yaw):
    decision = _update(
        DriveAdapter(AdapterConfig()), 0.0, speed=speed, yaw=yaw
    )

    assert decision.reason == 'nonfinite_input'
    assert decision.final_throttle == 0.0


@pytest.mark.parametrize(
    ('speed', 'measured', 'encoder'),
    [
        (0.0, 0.0, True),
        (-0.10, 0.0, True),
        (-0.60, 0.0, True),
        (-0.80, 0.0, True),
        (0.10, 0.0, True),
        (0.25, 0.25, True),
        (0.29, 0.0, False),
        (0.60, 0.0, True),
        (0.80, 0.0, True),
        (0.29, 3.0, True),
    ],
)
def test_every_published_adapter_output_obeys_safety_authority_bound(
    speed, measured, encoder
):
    decision = _update(
        DriveAdapter(AdapterConfig()),
        0.0,
        speed=speed,
        measured=measured,
        encoder=encoder,
    )

    assert decision.publish_command
    assert -0.14 <= decision.final_throttle <= 0.14


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
    assert decision.reason == 'floor_promoted'
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
        'feedforward_floor_violation=',
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
        'integrator_freeze_reason=',
    )

    assert all(field in text for field in required)
    assert 'integrator_state=0.000000000' in text
    assert 'integral_gain=0.000000000' in text
    assert 'integrator_enabled=false' in text
    assert 'stall_integral_gain_active=false' in text
    assert 'integral_decay_active=false' in text


@pytest.mark.parametrize(
    'changes',
    [
        {'maximum_commanded_speed': 0.10},
        {'feedforward_effort_per_speed': 0.0},
        {'feedforward_effort_intercept': 1.0},
        {'proportional_gain': 0.0},
        {'integral_gain': -0.01},
        {'integrator_bound': 0.0},
        {'output_min': -0.01},
        {'output_min': 0.01},
        {'output_max': 0.08},
        {'output_max': 0.15},
        {'encoder_metres_per_edge': 0.0},
        {'wheelspin_speed_ratio': 1.0},
        {'wheelspin_min_speed_excess': -0.1},
        {'wheelspin_qualification_sec': 0.0},
        {'motion_signal_timeout_sec': 0.0},
        {'encoder_state_timeout_sec': 0.0},
        {'active_mode_timeout_sec': 0.0},
    ],
)
def test_parameter_validation_rejects_invalid_combinations(changes):
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **changes)
