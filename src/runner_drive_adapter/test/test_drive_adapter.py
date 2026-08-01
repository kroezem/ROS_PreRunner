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

"""Deterministic tests for feedforward-plus-P speed control."""

from dataclasses import replace
import math

import pytest

from runner_drive_adapter.drive_adapter import (
    AdapterConfig,
    DriveAdapter,
    linear_inverse_feedforward,
)


def _feedforward(speed):
    config = AdapterConfig()
    return linear_inverse_feedforward(
        speed,
        config.feedforward_speed_per_command,
        config.feedforward_speed_intercept,
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


@pytest.mark.parametrize(
    ('speed', 'expected_command'),
    [
        (0.126, 0.0339248),
        (0.290, 0.0545615),
        (0.450, 0.0746949),
        (0.600, 0.0935699),
    ],
)
def test_provisional_md13s_linear_inverse(speed, expected_command):
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
    decision = _update(adapter, 0.0, speed=0.233, measured=0.233)

    assert decision.feedforward_throttle == pytest.approx(
        _feedforward(0.233)
    )
    assert decision.pi_term == pytest.approx(0.0)
    assert decision.final_throttle == pytest.approx(_feedforward(0.233))


def test_positive_output_saturates_at_compatibility_ceiling():
    config = AdapterConfig(proportional_gain=0.60)
    adapter = DriveAdapter(config)
    first = _update(adapter, 0.0, speed=0.60, measured=0.0)
    second = _update(adapter, 0.10, speed=0.60, measured=0.0)

    assert config.output_max == 0.12
    assert first.final_throttle == 0.12
    assert second.final_throttle == 0.12
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


def test_stale_integrator_state_cannot_affect_output():
    adapter = DriveAdapter(AdapterConfig())
    adapter._integrator = 0.16
    decision = _update(adapter, 0.0, speed=0.29, measured=0.20)

    expected = _feedforward(0.29) + 0.05 * (0.29 - 0.20)
    assert decision.final_throttle == pytest.approx(expected)
    assert decision.integrator_state == 0.0
    assert decision.integral_gain == 0.0
    assert not decision.integrator_enabled
    assert not decision.stall_integral_gain_active
    assert not decision.integral_decay_active


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
    assert decision.feedforward_throttle == pytest.approx(
        _feedforward(0.126)
    )
    assert decision.integrator_state == 0.0
    assert not decision.integrator_enabled
    assert decision.final_throttle == pytest.approx(_feedforward(0.126))


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
    adapter.update_command(0.233, 0.0, 0.0)
    decision = adapter.step(0.0)

    assert decision.reason == 'encoder_stale_feedforward'
    assert not decision.integrator_enabled
    assert decision.final_throttle == pytest.approx(_feedforward(0.233))


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
        (-0.1, 0.0, True),
        (0.10, 0.0, True),
        (0.126, 0.126, True),
        (0.29, 0.0, False),
        (0.60, 0.0, True),
        (0.80, 0.0, True),
        (0.29, 3.0, True),
    ],
)
def test_every_published_adapter_output_obeys_compatibility_bounds(
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
    assert 0.0 <= decision.final_throttle <= 0.12


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
    assert 'integrator_state=0.000000000' in text
    assert 'integral_gain=0.000000000' in text
    assert 'integrator_enabled=false' in text
    assert 'stall_integral_gain_active=false' in text
    assert 'integral_decay_active=false' in text


@pytest.mark.parametrize(
    'changes',
    [
        {'maximum_commanded_speed': 0.10},
        {'feedforward_speed_per_command': 0.0},
        {'feedforward_speed_intercept': 1.0},
        {'proportional_gain': 0.0},
        {'integral_gain': 0.01},
        {'output_min': -0.01},
        {'output_min': 0.01},
        {'output_max': 0.09},
        {'output_max': 0.13},
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
