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

"""ROS-independent Paddock mode and command-authority reducer."""

from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Optional, Tuple


class Mode(IntEnum):
    """Paddock operating mode; values mirror runner_interfaces/ModeState."""

    IDLE = 0
    MAPPING = 1
    AUTONOMY = 2


class Authority(IntEnum):
    """Selected command source; values mirror CommandAuthorityState."""

    NONE = 0
    DUALSENSE = 1
    PADDOCK_MANUAL = 2
    PADDOCK_AUTONOMY = 3


class Event(IntEnum):
    """Inputs accepted by :func:`transition`."""

    ACQUIRE_LEASE = 0
    RELEASE_LEASE = 1
    LOSE_LEASE = 2
    REQUEST_MODE = 3
    SELECT_GOAL = 4
    RUN_PRESS = 5
    RUN_RELEASE = 6
    STOP = 7
    MANUAL_ACTIVE = 8
    MANUAL_INACTIVE = 9
    DUALSENSE_ACTIVE = 10
    DUALSENSE_INACTIVE = 11
    SET_ACTIVE_MAP = 12
    RESTART = 13


class Effect(IntEnum):
    """Declarative work for a future ROS/runtime adapter."""

    BRAKE = 0
    CANCEL_NAVIGATION = 1
    DISPATCH_GOAL = 2
    LEASE_GRANTED = 3
    LEASE_REJECTED = 4


@dataclass(frozen=True)
class GoalIntent:
    """Map-frame goal pose retained independently of a Nav2 action."""

    map_name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PaddockState:
    """Complete durable state needed to make the next transition."""

    mode: Mode = Mode.IDLE
    accepted_mode_request_id: int = 0
    active_autonomy_map: str = ''
    lease_client_id: str = ''
    lease_id: str = ''
    lease_generation: int = 0
    dualsense_active: bool = False
    run_held: bool = False
    run_blocked_until_release: bool = False
    manual_active: bool = False
    goal: Optional[GoalIntent] = None
    navigation_active: bool = False

    @property
    def lease_active(self) -> bool:
        """Return whether the single Paddock control lease is held."""
        return bool(self.lease_id)

    @property
    def autonomy_permitted(self) -> bool:
        """Return the complete Paddock autonomy interlock decision."""
        return (
            self.mode == Mode.AUTONOMY
            and self.lease_active
            and self.run_held
            and not self.run_blocked_until_release
            and not self.dualsense_active
            and self.goal is not None
        )

    @property
    def authority(self) -> Authority:
        """Derive authority so snapshots cannot contain contradictions."""
        if self.mode == Mode.IDLE:
            return Authority.NONE
        if self.dualsense_active:
            return Authority.DUALSENSE
        if self.mode == Mode.MAPPING and self.lease_active:
            if self.manual_active:
                return Authority.PADDOCK_MANUAL
        if self.autonomy_permitted:
            return Authority.PADDOCK_AUTONOMY
        return Authority.NONE


@dataclass(frozen=True)
class Transition:
    """New state plus ordered, side-effect-free commands for an adapter."""

    state: PaddockState
    effects: Tuple[Effect, ...] = ()


def _owns_lease(state: PaddockState, lease_id: str) -> bool:
    return state.lease_active and lease_id == state.lease_id


def _stop_effects(state: PaddockState) -> Tuple[Effect, ...]:
    effects = [Effect.BRAKE]
    if state.navigation_active:
        effects.append(Effect.CANCEL_NAVIGATION)
    return tuple(effects)


def transition(
    state: PaddockState,
    event: Event,
    *,
    client_id: str = '',
    lease_id: str = '',
    mode: Optional[Mode] = None,
    request_id: int = 0,
    goal: Optional[GoalIntent] = None,
    active_autonomy_map: Optional[str] = None,
) -> Transition:
    """Reduce one input without ROS, clocks, I/O, or hidden side effects."""
    if event == Event.RESTART:
        selected_map = (
            state.active_autonomy_map
            if active_autonomy_map is None
            else active_autonomy_map
        )
        return Transition(PaddockState(active_autonomy_map=selected_map))

    if event == Event.SET_ACTIVE_MAP:
        if active_autonomy_map is None:
            return Transition(state)
        changed = active_autonomy_map != state.active_autonomy_map
        effects = _stop_effects(state) if changed else ()
        return Transition(
            replace(
                state,
                active_autonomy_map=active_autonomy_map,
                run_held=False if changed else state.run_held,
                run_blocked_until_release=(
                    False if changed else state.run_blocked_until_release
                ),
                manual_active=False if changed else state.manual_active,
                goal=None if changed else state.goal,
                navigation_active=False if changed else state.navigation_active,
            ),
            effects,
        )

    if event == Event.ACQUIRE_LEASE:
        if state.lease_active or not client_id or not lease_id:
            return Transition(state, (Effect.LEASE_REJECTED,))
        return Transition(
            replace(
                state,
                lease_client_id=client_id,
                lease_id=lease_id,
                lease_generation=state.lease_generation + 1,
                accepted_mode_request_id=0,
            ),
            (Effect.LEASE_GRANTED,),
        )

    if event in (Event.RELEASE_LEASE, Event.LOSE_LEASE):
        if not _owns_lease(state, lease_id):
            return Transition(state)
        return Transition(
            replace(
                state,
                lease_client_id='',
                lease_id='',
                lease_generation=state.lease_generation + 1,
                run_held=False,
                run_blocked_until_release=False,
                manual_active=False,
                goal=None,
                navigation_active=False,
            ),
            _stop_effects(state),
        )

    if event == Event.DUALSENSE_ACTIVE:
        if state.dualsense_active:
            return Transition(state)
        effects = _stop_effects(state) if state.authority in (
            Authority.PADDOCK_MANUAL,
            Authority.PADDOCK_AUTONOMY,
        ) else ()
        return Transition(
            replace(
                state,
                dualsense_active=True,
                run_blocked_until_release=state.run_held,
                manual_active=False,
                navigation_active=False,
            ),
            effects,
        )

    if event == Event.DUALSENSE_INACTIVE:
        return Transition(replace(state, dualsense_active=False))

    if not _owns_lease(state, lease_id):
        return Transition(state)

    if event == Event.REQUEST_MODE:
        if (
            not isinstance(mode, Mode)
            or request_id <= state.accepted_mode_request_id
        ):
            return Transition(state)
        if mode == state.mode:
            return Transition(
                replace(state, accepted_mode_request_id=request_id)
            )
        return Transition(
            replace(
                state,
                mode=mode,
                accepted_mode_request_id=request_id,
                run_held=False,
                run_blocked_until_release=False,
                manual_active=False,
                goal=None,
                navigation_active=False,
            ),
            _stop_effects(state),
        )

    if event == Event.SELECT_GOAL:
        if (
            state.mode != Mode.AUTONOMY
            or goal is None
            or not state.active_autonomy_map
            or goal.map_name != state.active_autonomy_map
        ):
            return Transition(state)
        effects = ()
        if state.navigation_active:
            effects = (Effect.BRAKE, Effect.CANCEL_NAVIGATION)
        new_state = replace(state, goal=goal, navigation_active=False)
        if new_state.autonomy_permitted:
            new_state = replace(new_state, navigation_active=True)
            effects += (Effect.DISPATCH_GOAL,)
        return Transition(new_state, effects)

    if event == Event.RUN_PRESS:
        if state.run_held:
            return Transition(state)
        new_state = replace(
            state,
            run_held=True,
            run_blocked_until_release=state.dualsense_active,
        )
        if new_state.autonomy_permitted:
            return Transition(
                replace(new_state, navigation_active=True),
                (Effect.DISPATCH_GOAL,),
            )
        return Transition(new_state)

    if event == Event.RUN_RELEASE:
        if not state.run_held and not state.run_blocked_until_release:
            return Transition(state)
        return Transition(
            replace(
                state,
                run_held=False,
                run_blocked_until_release=False,
                navigation_active=False,
            ),
            _stop_effects(state),
        )

    if event == Event.STOP:
        return Transition(
            replace(
                state,
                run_held=False,
                run_blocked_until_release=False,
                manual_active=False,
                navigation_active=False,
            ),
            _stop_effects(state),
        )

    if event == Event.MANUAL_ACTIVE:
        if state.mode != Mode.MAPPING or state.dualsense_active:
            return Transition(state)
        return Transition(replace(state, manual_active=True))

    if event == Event.MANUAL_INACTIVE:
        if not state.manual_active:
            return Transition(state)
        return Transition(
            replace(state, manual_active=False),
            (Effect.BRAKE,),
        )

    return Transition(state)
