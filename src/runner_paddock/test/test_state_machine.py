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

"""Focused tests for the Paddock Stage 1 reducer."""

import pytest

from runner_paddock.state_machine import Authority
from runner_paddock.state_machine import Effect
from runner_paddock.state_machine import Event
from runner_paddock.state_machine import GoalIntent
from runner_paddock.state_machine import Mode
from runner_paddock.state_machine import PaddockState
from runner_paddock.state_machine import transition


LEASE = 'lease-a'
CLIENT = 'phone-a'
MAP = 'collingwood'
GOAL = GoalIntent(map_name=MAP, x=1.0, y=2.0, yaw=0.5)


def apply(state, event, **kwargs):
    """Apply an event, defaulting control events to the owning lease."""
    kwargs.setdefault('lease_id', LEASE)
    return transition(state, event, **kwargs)


def leased_state(mode=Mode.IDLE):
    """Build a state through public transitions rather than mutation."""
    state = PaddockState(active_autonomy_map=MAP)
    result = transition(
        state,
        Event.ACQUIRE_LEASE,
        client_id=CLIENT,
        lease_id=LEASE,
    )
    assert result.effects == (Effect.LEASE_GRANTED,)
    state = result.state
    if mode != Mode.IDLE:
        state = apply(
            state,
            Event.REQUEST_MODE,
            mode=mode,
            request_id=1,
        ).state
    return state


def running_state():
    """Build autonomy with one dispatched goal."""
    state = leased_state(Mode.AUTONOMY)
    state = apply(state, Event.SELECT_GOAL, goal=GOAL).state
    result = apply(state, Event.RUN_PRESS)
    assert result.effects == (Effect.DISPATCH_GOAL,)
    return result.state


def test_startup_is_idle_disarmed_and_map_is_queryable():
    state = PaddockState(active_autonomy_map=MAP)

    assert state.mode == Mode.IDLE
    assert state.active_autonomy_map == MAP
    assert state.authority == Authority.NONE
    assert not state.autonomy_permitted


def test_exactly_one_control_lease_and_non_owner_events_are_ignored():
    state = leased_state()

    rejected = transition(
        state,
        Event.ACQUIRE_LEASE,
        client_id='phone-b',
        lease_id='lease-b',
    )
    ignored = transition(
        state,
        Event.REQUEST_MODE,
        lease_id='lease-b',
        mode=Mode.MAPPING,
        request_id=1,
    )

    assert rejected.state == state
    assert rejected.effects == (Effect.LEASE_REJECTED,)
    assert ignored.state == state


def test_mode_requests_are_ordered_and_idempotent():
    state = leased_state()
    changed = apply(
        state,
        Event.REQUEST_MODE,
        mode=Mode.MAPPING,
        request_id=4,
    )
    stale = apply(
        changed.state,
        Event.REQUEST_MODE,
        mode=Mode.AUTONOMY,
        request_id=3,
    )
    duplicate_mode = apply(
        changed.state,
        Event.REQUEST_MODE,
        mode=Mode.MAPPING,
        request_id=5,
    )

    assert changed.state.mode == Mode.MAPPING
    assert changed.effects == (Effect.BRAKE,)
    assert stale.state == changed.state
    assert duplicate_mode.state.accepted_mode_request_id == 5
    assert duplicate_mode.effects == ()


def test_invalid_mode_value_is_rejected():
    state = leased_state()

    result = apply(
        state,
        Event.REQUEST_MODE,
        mode=99,
        request_id=1,
    )

    assert result.state == state
    assert result.effects == ()


def test_mapping_manual_authority_and_idle_no_drive():
    state = leased_state(Mode.MAPPING)
    active = apply(state, Event.MANUAL_ACTIVE)
    idle = apply(
        active.state,
        Event.REQUEST_MODE,
        mode=Mode.IDLE,
        request_id=2,
    )

    assert active.state.authority == Authority.PADDOCK_MANUAL
    assert idle.state.authority == Authority.NONE
    assert not idle.state.manual_active
    assert idle.effects == (Effect.BRAKE,)


def test_run_release_cancels_and_retains_goal_then_repress_dispatches_fresh():
    state = running_state()

    released = apply(state, Event.RUN_RELEASE)
    rerun = apply(released.state, Event.RUN_PRESS)

    assert released.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert released.state.goal == GOAL
    assert not released.state.navigation_active
    assert released.state.authority == Authority.NONE
    assert rerun.effects == (Effect.DISPATCH_GOAL,)
    assert rerun.state.goal == GOAL
    assert rerun.state.navigation_active
    assert rerun.state.authority == Authority.PADDOCK_AUTONOMY


def test_run_without_a_goal_never_permits_autonomy():
    result = apply(leased_state(Mode.AUTONOMY), Event.RUN_PRESS)

    assert result.effects == ()
    assert result.state.run_held
    assert not result.state.autonomy_permitted
    assert result.state.authority == Authority.NONE


def test_goal_selected_while_run_is_held_dispatches_once():
    held = apply(leased_state(Mode.AUTONOMY), Event.RUN_PRESS).state
    selected = apply(held, Event.SELECT_GOAL, goal=GOAL)

    assert selected.effects == (Effect.DISPATCH_GOAL,)
    assert selected.state.navigation_active


def test_goal_must_match_explicit_active_autonomy_map():
    state = leased_state(Mode.AUTONOMY)
    wrong_map = GoalIntent(map_name='other', x=1.0, y=2.0, yaw=0.5)

    result = apply(state, Event.SELECT_GOAL, goal=wrong_map)

    assert result.state.goal is None
    assert result.effects == ()


def test_stop_brakes_cancels_disarms_and_retains_goal():
    stopped = apply(running_state(), Event.STOP)

    assert stopped.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert stopped.state.goal == GOAL
    assert not stopped.state.run_held
    assert not stopped.state.navigation_active
    assert stopped.state.authority == Authority.NONE


@pytest.mark.parametrize('event', [Event.RELEASE_LEASE, Event.LOSE_LEASE])
def test_lease_end_brakes_cancels_disarms_and_drops_client_goal(event):
    state = running_state()
    ended = apply(state, event)

    assert ended.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert not ended.state.lease_active
    assert ended.state.lease_generation == state.lease_generation + 1
    assert ended.state.goal is None
    assert ended.state.authority == Authority.NONE


def test_dualsense_takeover_is_immediate_and_requires_run_repress():
    state = running_state()
    takeover = transition(state, Event.DUALSENSE_ACTIVE)
    departed = transition(takeover.state, Event.DUALSENSE_INACTIVE)

    assert takeover.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert takeover.state.authority == Authority.DUALSENSE
    assert takeover.state.goal == GOAL
    assert takeover.state.run_blocked_until_release
    assert departed.state.authority == Authority.NONE
    assert not departed.state.autonomy_permitted

    released = apply(departed.state, Event.RUN_RELEASE)
    rerun = apply(released.state, Event.RUN_PRESS)
    assert rerun.effects == (Effect.DISPATCH_GOAL,)
    assert rerun.state.authority == Authority.PADDOCK_AUTONOMY


def test_dualsense_cannot_drive_in_idle():
    takeover = transition(leased_state(), Event.DUALSENSE_ACTIVE)

    assert takeover.state.dualsense_active
    assert takeover.state.authority == Authority.NONE


def test_run_pressed_during_dualsense_requires_release_after_takeover():
    state = leased_state(Mode.AUTONOMY)
    state = apply(state, Event.SELECT_GOAL, goal=GOAL).state
    state = transition(state, Event.DUALSENSE_ACTIVE).state

    pressed = apply(state, Event.RUN_PRESS)
    departed = transition(pressed.state, Event.DUALSENSE_INACTIVE)

    assert pressed.state.run_blocked_until_release
    assert departed.state.authority == Authority.NONE
    assert not departed.state.autonomy_permitted


def test_new_lease_gets_a_fresh_mode_request_sequence():
    state = leased_state(Mode.MAPPING)
    state = apply(state, Event.RELEASE_LEASE).state
    state = transition(
        state,
        Event.ACQUIRE_LEASE,
        client_id='phone-b',
        lease_id='lease-b',
    ).state

    requested = transition(
        state,
        Event.REQUEST_MODE,
        lease_id='lease-b',
        mode=Mode.AUTONOMY,
        request_id=1,
    )

    assert requested.state.mode == Mode.AUTONOMY


def test_mode_change_brakes_cancels_disarms_and_clears_goal():
    changed = apply(
        running_state(),
        Event.REQUEST_MODE,
        mode=Mode.MAPPING,
        request_id=2,
    )

    assert changed.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert changed.state.mode == Mode.MAPPING
    assert changed.state.goal is None
    assert not changed.state.run_held
    assert changed.state.authority == Authority.NONE


def test_active_map_change_invalidates_goal_and_active_navigation():
    changed = transition(
        running_state(),
        Event.SET_ACTIVE_MAP,
        active_autonomy_map='warehouse',
    )

    assert changed.effects == (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
    assert changed.state.active_autonomy_map == 'warehouse'
    assert changed.state.goal is None
    assert not changed.state.autonomy_permitted


def test_restart_is_idle_disarmed_drops_lease_and_transient_goal():
    restarted = transition(running_state(), Event.RESTART)

    assert restarted.effects == ()
    assert restarted.state == PaddockState(active_autonomy_map=MAP)
    assert restarted.state.authority == Authority.NONE
