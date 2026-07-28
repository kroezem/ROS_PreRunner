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

"""Deterministic tests for conversion and bounded stall assistance."""

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
    motion=0.0,
    edge_rate=0.0,
    stationary=True,
    direction=1,
    command=True,
    motion_sample=True,
    encoder_sample=True,
):
    if command:
        adapter.update_command(speed, yaw, now)
    if motion_sample:
        adapter.update_motion(motion, now)
    if encoder_sample:
        adapter.update_encoder(
            stationary, edge_rate, direction, now
        )
    return adapter.step(now)


def _start_ramp(adapter=None, speed=0.29):
    adapter = adapter or DriveAdapter(AdapterConfig())
    first = _update(adapter, 0.0, speed=speed)
    assert first.assist_state == 'QUALIFYING'
    ramp = _update(adapter, 0.30, speed=speed)
    assert ramp.assist_state == 'RAMPING'
    return adapter, ramp


def _enter_decay(adapter):
    _update(adapter, 0.40, motion=0.07)
    _update(adapter, 0.50, motion=0.07)
    held = _update(adapter, 0.60, motion=0.07)
    assert held.assist_state == 'HOLDING'
    held = _update(adapter, 0.90, motion=0.07)
    assert held.assist_state == 'DECAYING'
    return held


def test_valid_monotonic_table_and_interpolation_are_unchanged():
    validate_table(THROTTLES, SPEEDS)
    assert lookup_throttle(0.126, THROTTLES, SPEEDS) == (
        0.340, False
    )
    value, clamped = lookup_throttle(0.2105, THROTTLES, SPEEDS)
    assert value == pytest.approx(0.355)
    assert not clamped
    assert lookup_throttle(0.35, THROTTLES, SPEEDS) == (
        0.380, True
    )


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


def test_normal_feedforward_remains_unchanged():
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=0.233, motion=0.20)
    assert decision.assist_state == 'NORMAL'
    assert decision.feedforward_throttle == pytest.approx(0.360)
    assert decision.final_throttle == pytest.approx(0.360)


def test_stationary_forward_qualifies_and_ramp_starts_at_feedforward():
    adapter, ramp = _start_ramp()
    assert ramp.feedforward_throttle == pytest.approx(0.380)
    assert ramp.final_throttle == pytest.approx(0.380)
    assert adapter.event_count == 1
    assert adapter.take_transition_events()[0].startswith('start:')


def test_relative_and_absolute_under_speed_conditions_are_both_required():
    adapter = DriveAdapter(AdapterConfig())
    relative_fails = _update(
        adapter, 0.0, speed=0.126, motion=0.055
    )
    assert relative_fails.assist_state == 'NORMAL'
    absolute_fails = _update(
        adapter, 0.1, speed=0.29, motion=0.11
    )
    assert absolute_fails.assist_state == 'NORMAL'


def test_command_must_remain_stable_through_qualification():
    adapter = DriveAdapter(AdapterConfig())
    assert _update(adapter, 0.0).assist_state == 'QUALIFYING'
    assert _update(
        adapter, 0.29, speed=0.26
    ).assist_state == 'QUALIFYING'
    assert _update(
        adapter, 0.31, speed=0.26
    ).assist_state == 'QUALIFYING'
    assert _update(
        adapter, 0.60, speed=0.26
    ).assist_state == 'RAMPING'


def test_ramp_rate_is_deterministic_and_ceiling_is_enforced():
    adapter, _ = _start_ramp()
    half_second = _update(adapter, 0.80)
    assert half_second.final_throttle == pytest.approx(0.430)
    assert half_second.applied_boost == pytest.approx(0.050)
    at_ceiling = _update(adapter, 1.50)
    assert at_ceiling.final_throttle == pytest.approx(0.380)
    assert at_ceiling.assist_state == 'COOLDOWN'
    summary = adapter.take_event_summaries()[0]
    assert summary.peak_throttle == pytest.approx(0.500)
    assert summary.exit_reason == 'ceiling_reached_without_motion'


def test_motion_confirmation_requires_multiple_samples_and_duration():
    adapter, _ = _start_ramp()
    one = _update(adapter, 0.40, motion=0.07)
    assert one.assist_state == 'RAMPING'
    two_too_soon = _update(adapter, 0.50, motion=0.07)
    assert two_too_soon.assist_state == 'RAMPING'
    confirmed = _update(adapter, 0.60, motion=0.07)
    assert confirmed.assist_state == 'HOLDING'
    assert confirmed.final_throttle == pytest.approx(0.400)


def test_hold_transitions_to_decay_then_normal_feedforward():
    adapter, _ = _start_ramp()
    _enter_decay(adapter)
    decision = None
    now = 0.90
    for _ in range(20):
        now += 0.05
        decision = _update(adapter, now, motion=0.07)
        if decision.assist_state == 'COOLDOWN':
            break
    assert decision.assist_state == 'COOLDOWN'
    assert decision.final_throttle == pytest.approx(0.380)
    summary = adapter.take_event_summaries()[0]
    assert summary.exit_reason == 'motion_confirmed_and_decayed'
    assert summary.hold_duration >= 0.30
    assert summary.decay_duration > 0.0


def test_overspeed_skips_hold_and_fast_decays():
    adapter, _ = _start_ramp()
    boosted = _update(adapter, 0.50)
    assert boosted.final_throttle == pytest.approx(0.400)
    fast = _update(adapter, 0.55, motion=0.32)
    assert fast.assist_state == 'DECAYING'
    assert fast.final_throttle < boosted.final_throttle
    now = 0.55
    while adapter.state != 'COOLDOWN':
        now += 0.05
        _update(adapter, now, motion=0.32)
    assert (
        adapter.take_event_summaries()[0].exit_reason
        == 'overspeed_fast_decay'
    )


def test_restall_during_decay_does_not_rearm():
    adapter, _ = _start_ramp()
    _enter_decay(adapter)
    count = adapter.event_count
    decision = _update(adapter, 0.95, motion=0.0)
    assert decision.assist_state == 'DECAYING'
    assert adapter.event_count == count


def test_maximum_duration_terminates_elevated_throttle():
    config = replace(
        AdapterConfig(),
        boost_throttle_ceiling=0.80,
        maximum_assist_duration_sec=0.40,
    )
    adapter, _ = _start_ramp(DriveAdapter(config))
    assert _update(adapter, 0.69).assist_state == 'RAMPING'
    ended = _update(adapter, 0.70)
    assert ended.assist_state == 'COOLDOWN'
    assert ended.final_throttle == pytest.approx(0.380)
    assert (
        adapter.take_event_summaries()[0].exit_reason
        == 'maximum_duration'
    )


def test_cooldown_blocks_then_eventually_allows_rearming():
    config = replace(
        AdapterConfig(),
        boost_throttle_ceiling=0.80,
        maximum_assist_duration_sec=0.40,
    )
    adapter, _ = _start_ramp(DriveAdapter(config))
    _update(adapter, 0.70)
    count = adapter.event_count
    assert _update(adapter, 1.00).assist_state == 'COOLDOWN'
    assert adapter.event_count == count
    assert _update(adapter, 2.20).assist_state == 'QUALIFYING'
    assert _update(adapter, 2.50).assist_state == 'RAMPING'
    assert adapter.event_count == count + 1


def test_wheelspin_returns_to_feedforward_without_braking():
    adapter, _ = _start_ramp()
    _update(
        adapter, 0.40, edge_rate=5.0, stationary=False
    )
    decision = _update(
        adapter, 0.65, edge_rate=5.0, stationary=False
    )
    assert decision.mode == 'forward'
    assert decision.final_throttle == pytest.approx(0.380)
    assert decision.assist_state == 'COOLDOWN'
    assert adapter.take_event_summaries()[0].exit_reason == 'wheelspin'


def test_encoder_direction_disagreement_is_not_forward_wheelspin():
    adapter, _ = _start_ramp()
    _update(
        adapter,
        0.40,
        edge_rate=5.0,
        stationary=False,
        direction=-1,
    )
    decision = _update(
        adapter,
        0.70,
        edge_rate=5.0,
        stationary=False,
        direction=-1,
    )
    assert decision.assist_state == 'RAMPING'


def test_stale_primary_motion_blocks_assist():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.29, 0.0, 0.0)
    adapter.update_encoder(True, 0.0, 1, 0.0)
    assert adapter.step(0.0).assist_state == 'NORMAL'


def test_stale_encoder_blocks_new_assist():
    adapter = DriveAdapter(AdapterConfig())
    adapter.update_command(0.29, 0.0, 0.0)
    adapter.update_motion(0.0, 0.0)
    assert adapter.step(0.0).assist_state == 'NORMAL'


def test_motion_staleness_during_assist_terminates_boost():
    adapter, _ = _start_ramp()
    adapter.update_command(0.29, 0.0, 0.56)
    adapter.update_encoder(True, 0.0, 1, 0.56)
    decision = adapter.step(0.56)
    assert decision.assist_state == 'COOLDOWN'
    assert (
        adapter.take_event_summaries()[0].exit_reason
        == 'motion_signal_stale'
    )


def test_encoder_staleness_during_assist_terminates_boost():
    adapter, _ = _start_ramp()
    adapter.update_command(0.29, 0.0, 0.56)
    adapter.update_motion(0.0, 0.56)
    decision = adapter.step(0.56)
    assert decision.assist_state == 'COOLDOWN'
    assert (
        adapter.take_event_summaries()[0].exit_reason
        == 'encoder_stale'
    )


def test_stale_nav_command_is_silent_and_summarized():
    adapter, _ = _start_ramp()
    adapter.update_motion(0.0, 0.56)
    adapter.update_encoder(True, 0.0, 1, 0.56)
    decision = adapter.step(0.56)
    assert not decision.publish_command
    assert decision.reason == 'stale_command'
    assert (
        adapter.take_event_summaries()[0].exit_reason
        == 'command_stale'
    )


@pytest.mark.parametrize(
    ('speed', 'reason', 'exit_reason'),
    [
        (0.0, 'explicit_stop', 'controller_command_ended'),
        (-0.1, 'negative_speed', 'brake_or_reverse'),
    ],
)
def test_stop_brake_or_reverse_immediately_cancel_assist(
    speed, reason, exit_reason
):
    adapter, _ = _start_ramp()
    decision = _update(adapter, 0.35, speed=speed)
    assert decision.mode == 'brake'
    assert decision.reason == reason
    assert decision.final_throttle == -1.0
    assert adapter.take_event_summaries()[0].exit_reason == exit_reason


@pytest.mark.parametrize(
    ('speed', 'yaw'),
    [(math.nan, 0.0), (math.inf, 0.0), (0.2, math.nan)],
)
def test_invalid_nonfinite_input_is_safely_braked(speed, yaw):
    adapter = DriveAdapter(AdapterConfig())
    decision = _update(adapter, 0.0, speed=speed, yaw=yaw)
    assert decision.mode == 'brake'
    assert decision.reason == 'nonfinite_input'
    assert decision.final_throttle == -1.0


def test_existing_steering_infeasible_full_brake_is_unchanged():
    config = AdapterConfig()
    decision = _update(
        DriveAdapter(config),
        0.0,
        speed=0.20,
        yaw=0.20 * config.maximum_curvature + 1e-6,
        motion=0.2,
    )
    assert decision.mode == 'brake'
    assert decision.reason == 'steering_infeasible'
    assert decision.final_throttle == -1.0
    assert decision.normalized_steering == 0.0


def test_default_steering_limit_matches_measured_turning_radius():
    config = AdapterConfig()

    assert config.max_steering_angle == 0.3614
    assert 1.0 / config.maximum_curvature == pytest.approx(0.4709, abs=1e-4)


def test_old_breakaway_mechanism_is_absent():
    fields = AdapterConfig.__dataclass_fields__
    assert 'breakaway_throttle' not in fields
    assert 'breakaway_timeout' not in fields
    assert 'motion_confirm_edge_rate' not in fields


@pytest.mark.parametrize(
    'changes',
    [
        {'stall_assist_enabled': 1},
        {'under_speed_ratio': 0.0},
        {'under_speed_ratio': 1.1},
        {'ramp_rate_per_sec': 0.0},
        {'decay_rate_per_sec': 0.0},
        {'boost_throttle_ceiling': 0.37},
        {'boost_throttle_ceiling': 1.01},
        {'motion_confirm_speed': -0.1},
        {'overspeed_margin': math.nan},
        {'motion_signal_timeout_sec': 0.0},
        {'encoder_state_timeout_sec': 0.0},
        {'cooldown_duration_sec': 0.0},
    ],
)
def test_parameter_validation_rejects_invalid_combinations(changes):
    with pytest.raises(ValueError):
        replace(AdapterConfig(), **changes)


def test_structured_summary_has_all_required_fields():
    adapter, _ = _start_ramp()
    _update(
        adapter, 0.40, edge_rate=5.0, stationary=False
    )
    _update(
        adapter, 0.65, edge_rate=5.0, stationary=False
    )
    summary = adapter.take_event_summaries()[0]
    text = summary.diagnostic_text()
    required = (
        'event_id',
        'commanded_speed',
        'feedforward_throttle',
        'peak_throttle',
        'event_duration',
        'ramp_duration',
        'hold_duration',
        'decay_duration',
        'primary_motion_signal',
        'primary_speed_start',
        'primary_speed_peak',
        'ekf_speed_start',
        'ekf_speed_peak',
        'encoder_edge_rate_start',
        'encoder_edge_rate_peak',
        'exit_reason',
    )
    assert all(f'{name}=' in text for name in required)
    assert summary.exit_reason == 'wheelspin'


def test_decision_diagnostics_reflect_state_and_boost():
    adapter, _ = _start_ramp()
    decision = _update(adapter, 0.50)
    text = decision.diagnostic_text()
    assert 'stall_assist_state=RAMPING' in text
    assert 'applied_boost=0.020000000' in text
    assert 'event_count=1' in text


def test_shutdown_while_elevated_records_summary_and_no_command():
    adapter, _ = _start_ramp()
    adapter.shutdown(0.50)
    assert adapter.state == 'IDLE'
    assert (
        adapter.take_event_summaries()[0].exit_reason == 'shutdown'
    )
    adapter.update_motion(0.0, 1.0)
    adapter.update_encoder(True, 0.0, 1, 1.0)
    assert not adapter.step(1.0).publish_command
