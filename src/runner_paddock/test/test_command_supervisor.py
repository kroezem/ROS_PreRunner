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

"""Focused timing and transition tests for command supervision."""

import math

import pytest

from runner_paddock.command_supervisor import CommandSupervisor
from runner_paddock.command_supervisor import ControlEvent
from runner_paddock.command_supervisor import VelocityCommand
from runner_paddock.state_machine import Authority
from runner_paddock.state_machine import Mode


CLIENT = 'phone-a'
LEASE = 'lease-a'
MAP = 'collingwood'
COMMAND = VelocityCommand(linear_x=0.4, angular_z=0.2)


def control(supervisor, event, sequence, now, **kwargs):
    """Send one owner event with concise defaults."""
    return supervisor.handle_control_event(
        event=event,
        client_id=kwargs.get('client_id', CLIENT),
        lease_id=kwargs.get('lease_id', LEASE),
        sequence=sequence,
        now=now,
    )


def autonomy_ready(now=0.0):
    """Build a leased AUTONOMY state with goal intent and RUN held."""
    supervisor = CommandSupervisor(active_autonomy_map=MAP)
    assert control(
        supervisor, ControlEvent.LEASE_ACQUIRED, 1, now
    ).accepted
    assert supervisor.request_mode(
        mode=Mode.AUTONOMY,
        request_id=1,
        lease_id=LEASE,
        now=now,
    ).accepted
    assert control(
        supervisor, ControlEvent.GOAL_SELECTED, 2, now
    ).accepted
    assert control(
        supervisor, ControlEvent.RUN_PRESSED, 3, now
    ).accepted
    return supervisor


def test_run_release_and_rerun_gate_each_raw_sample():
    supervisor = autonomy_ready()

    first = supervisor.receive_raw_autonomy(COMMAND, 0.010)
    released = control(
        supervisor, ControlEvent.RUN_RELEASED, 4, 0.020
    )
    blocked = supervisor.receive_raw_autonomy(COMMAND, 0.021)
    rerun = control(supervisor, ControlEvent.RUN_PRESSED, 5, 0.030)
    second = supervisor.receive_raw_autonomy(COMMAND, 0.031)

    assert first.autonomy_command == COMMAND
    assert not first.snapshot.brake_intent
    assert released.snapshot.brake_intent
    assert blocked.autonomy_command is None
    assert rerun.snapshot.reason == 'AUTONOMY_RUN'
    assert second.autonomy_command == COMMAND


def test_stop_closes_and_disarms_autonomy():
    supervisor = autonomy_ready()
    supervisor.receive_raw_autonomy(COMMAND, 0.010)

    stopped = control(supervisor, ControlEvent.STOP, 4, 0.020)
    blocked = supervisor.receive_raw_autonomy(COMMAND, 0.021)

    assert stopped.snapshot.brake_intent
    assert not stopped.snapshot.state.run_held
    assert stopped.snapshot.state.goal is not None
    assert blocked.autonomy_command is None


def test_every_non_running_mode_state_maintains_brake_intent():
    supervisor = CommandSupervisor(active_autonomy_map=MAP)
    assert supervisor.tick(0.0).snapshot.brake_intent
    control(supervisor, ControlEvent.LEASE_ACQUIRED, 1, 0.0)

    mapping = supervisor.request_mode(
        mode=Mode.MAPPING,
        request_id=1,
        lease_id=LEASE,
        now=0.0,
    )
    autonomy = supervisor.request_mode(
        mode=Mode.AUTONOMY,
        request_id=2,
        lease_id=LEASE,
        now=0.0,
    )

    assert mapping.snapshot.brake_intent
    assert autonomy.snapshot.brake_intent


def test_lease_is_valid_at_150_ms_and_closed_immediately_beyond_it():
    supervisor = autonomy_ready()

    boundary = supervisor.receive_raw_autonomy(COMMAND, 0.150)
    beyond = supervisor.receive_raw_autonomy(COMMAND, 0.150000001)

    assert boundary.autonomy_command == COMMAND
    assert boundary.snapshot.lease_age_sec == pytest.approx(0.150)
    assert beyond.autonomy_command is None
    assert beyond.snapshot.brake_intent
    assert not beyond.snapshot.state.lease_active
    assert beyond.snapshot.lease_age_sec > 0.150


def test_heartbeat_uses_pi_receive_time_and_extends_the_lease():
    supervisor = autonomy_ready()
    heartbeat = control(supervisor, ControlEvent.HEARTBEAT, 4, 0.100)

    still_open = supervisor.receive_raw_autonomy(COMMAND, 0.249)
    expired = supervisor.tick(0.250000001)

    assert heartbeat.snapshot.lease_age_sec == 0.0
    assert still_open.autonomy_command == COMMAND
    assert expired.snapshot.reason == 'LEASE_EXPIRED'
    assert expired.snapshot.brake_intent


def test_stale_raw_command_changes_periodic_output_to_brake_intent():
    supervisor = autonomy_ready()
    supervisor.receive_raw_autonomy(COMMAND, 0.0)
    control(supervisor, ControlEvent.HEARTBEAT, 4, 0.140)

    fresh = supervisor.tick(0.150)
    stale = supervisor.tick(0.150000001)

    assert not fresh.snapshot.brake_intent
    assert fresh.snapshot.raw_autonomy_fresh
    assert stale.snapshot.brake_intent
    assert not stale.snapshot.raw_autonomy_fresh
    assert stale.snapshot.reason == 'RAW_AUTONOMY_STALE'


def test_invalid_raw_command_is_never_forwarded_and_brakes():
    supervisor = autonomy_ready()
    invalid = VelocityCommand(linear_x=math.nan)

    result = supervisor.receive_raw_autonomy(invalid, 0.010)

    assert not result.accepted
    assert result.autonomy_command is None
    assert result.snapshot.brake_intent
    assert result.snapshot.reason == 'INVALID_RAW_AUTONOMY'


def test_dualsense_takeover_closes_autonomy_and_requires_rearm():
    supervisor = autonomy_ready()
    supervisor.receive_raw_autonomy(COMMAND, 0.010)

    takeover = supervisor.set_dualsense_active(True, 0.020)
    blocked = supervisor.receive_raw_autonomy(COMMAND, 0.021)
    released = supervisor.set_dualsense_active(False, 0.030)

    assert takeover.snapshot.state.authority == Authority.DUALSENSE
    assert takeover.snapshot.brake_intent
    assert blocked.autonomy_command is None
    assert released.snapshot.reason == 'RUN_REARM_REQUIRED'
    assert released.snapshot.state.authority == Authority.NONE

    control(supervisor, ControlEvent.RUN_RELEASED, 4, 0.040)
    control(supervisor, ControlEvent.RUN_PRESSED, 5, 0.050)
    forwarded = supervisor.receive_raw_autonomy(COMMAND, 0.051)
    assert forwarded.autonomy_command == COMMAND


def test_restart_is_fail_closed_and_drops_transient_inputs():
    supervisor = autonomy_ready()
    supervisor.receive_raw_autonomy(COMMAND, 0.010)

    restarted = supervisor.restart(0.020)
    blocked = supervisor.receive_raw_autonomy(COMMAND, 0.021)

    assert restarted.snapshot.reason == 'RESTART_BRAKE'
    assert restarted.snapshot.brake_intent
    assert restarted.snapshot.state.mode == Mode.IDLE
    assert not restarted.snapshot.state.lease_active
    assert restarted.snapshot.raw_autonomy_age_sec is None
    assert blocked.autonomy_command is None


def test_unstable_runtime_revokes_mode_and_future_command_grant():
    supervisor = autonomy_ready()

    result = supervisor.set_runtime_mode(
        Mode.AUTONOMY,
        MAP,
        0.020,
        runtime_stable=False,
    )

    assert result.snapshot.state.mode == Mode.IDLE
    assert not result.snapshot.state.run_held
    assert not result.snapshot.state.navigation_active
    assert result.snapshot.state.authority == Authority.NONE
    assert result.snapshot.brake_intent


@pytest.mark.parametrize(
    'wrong_owner',
    [
        {'client_id': 'phone-b'},
        {'lease_id': 'lease-b'},
    ],
)
def test_non_owner_event_is_rejected_and_cannot_refresh_lease(wrong_owner):
    supervisor = autonomy_ready()

    rejected = control(
        supervisor,
        ControlEvent.HEARTBEAT,
        4,
        0.140,
        **wrong_owner,
    )
    expired = supervisor.tick(0.150000001)

    assert not rejected.accepted
    assert rejected.snapshot.reason == 'NON_OWNER_EVENT_REJECTED'
    assert expired.snapshot.reason == 'LEASE_EXPIRED'


def test_stale_event_is_rejected_and_cannot_refresh_lease():
    supervisor = autonomy_ready()

    rejected = control(supervisor, ControlEvent.HEARTBEAT, 3, 0.140)
    expired = supervisor.tick(0.150000001)

    assert not rejected.accepted
    assert rejected.snapshot.reason == 'STALE_EVENT_REJECTED'
    assert expired.snapshot.reason == 'LEASE_EXPIRED'


def test_disconnect_event_closes_autonomy_immediately():
    supervisor = autonomy_ready()
    supervisor.receive_raw_autonomy(COMMAND, 0.010)

    released = control(
        supervisor, ControlEvent.LEASE_RELEASED, 4, 0.020
    )

    assert released.snapshot.brake_intent
    assert not released.snapshot.state.lease_active
    assert released.snapshot.state.authority == Authority.NONE


def test_no_raw_command_is_republished_without_a_new_input_sample():
    """Crash/silence relies on downstream timeout; the node cannot pulse stale raw."""
    supervisor = autonomy_ready()
    forwarded = supervisor.receive_raw_autonomy(COMMAND, 0.010)

    periodic = supervisor.tick(0.020)

    assert forwarded.autonomy_command == COMMAND
    assert periodic.autonomy_command is None
    assert not periodic.snapshot.brake_intent


@pytest.mark.parametrize(
    'kwargs',
    [
        {'lease_timeout_sec': 0.0},
        {'lease_timeout_sec': math.inf},
        {'raw_autonomy_timeout_sec': -1.0},
        {'raw_autonomy_timeout_sec': math.nan},
    ],
)
def test_timeouts_must_be_finite_and_positive(kwargs):
    with pytest.raises(ValueError):
        CommandSupervisor(**kwargs)
