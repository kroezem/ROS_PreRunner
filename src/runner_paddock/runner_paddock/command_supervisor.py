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

"""Clock-injected command supervision with no ROS dependencies."""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional, Tuple

from runner_paddock.state_machine import Authority
from runner_paddock.state_machine import Event
from runner_paddock.state_machine import GoalIntent
from runner_paddock.state_machine import Mode
from runner_paddock.state_machine import PaddockState
from runner_paddock.state_machine import transition


DEFAULT_LEASE_TIMEOUT_SEC = 0.150
DEFAULT_RAW_AUTONOMY_TIMEOUT_SEC = 0.150
UINT64_MODULUS = 1 << 64
UINT64_HALF_RANGE = 1 << 63


class ControlEvent(IntEnum):
    """Values mirror runner_interfaces/PaddockControlEvent."""

    RUN_PRESSED = 0
    RUN_RELEASED = 1
    STOP = 2
    MANUAL_ACTIVE = 3
    MANUAL_INACTIVE = 4
    GOAL_SELECTED = 5
    LEASE_ACQUIRED = 6
    LEASE_RELEASED = 7
    LEASE_LOST = 8
    HEARTBEAT = 9


@dataclass(frozen=True)
class VelocityCommand:
    """ROS-independent representation of one Twist sample."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    @property
    def valid(self) -> bool:
        """Return whether all components are finite."""
        return all(math.isfinite(value) for value in self.values)

    @property
    def values(self) -> Tuple[float, ...]:
        """Return components in geometry_msgs/Twist field order."""
        return (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        )


@dataclass(frozen=True)
class SupervisorSnapshot:
    """One queryable authority and timing snapshot."""

    state: PaddockState
    brake_intent: bool
    lease_fresh: bool
    lease_age_sec: Optional[float]
    raw_autonomy_fresh: bool
    raw_autonomy_age_sec: Optional[float]
    last_control_sequence: Optional[int]
    reason: str


@dataclass(frozen=True)
class SupervisorResult:
    """Result of one input, including any supervised autonomy output."""

    snapshot: SupervisorSnapshot
    autonomy_command: Optional[VelocityCommand] = None
    accepted: bool = True


def _is_newer_sequence(candidate: int, previous: int) -> bool:
    delta = (candidate - previous) % UINT64_MODULUS
    return 0 < delta < UINT64_HALF_RANGE


class CommandSupervisor:
    """Own the Paddock lease, RUN gate, and raw autonomy freshness."""

    def __init__(
        self,
        *,
        lease_timeout_sec: float = DEFAULT_LEASE_TIMEOUT_SEC,
        raw_autonomy_timeout_sec: float = DEFAULT_RAW_AUTONOMY_TIMEOUT_SEC,
        active_autonomy_map: str = '',
    ) -> None:
        if not math.isfinite(lease_timeout_sec) or lease_timeout_sec <= 0.0:
            raise ValueError('lease_timeout_sec must be finite and positive')
        if (
            not math.isfinite(raw_autonomy_timeout_sec)
            or raw_autonomy_timeout_sec <= 0.0
        ):
            raise ValueError(
                'raw_autonomy_timeout_sec must be finite and positive'
            )
        self.lease_timeout_sec = lease_timeout_sec
        self.raw_autonomy_timeout_sec = raw_autonomy_timeout_sec
        self.state = PaddockState(
            active_autonomy_map=active_autonomy_map
        )
        self.last_lease_receive_at: Optional[float] = None
        self.last_raw_autonomy_at: Optional[float] = None
        self.last_control_sequence: Optional[int] = None

    def _lease_age(self, now: float) -> Optional[float]:
        if self.last_lease_receive_at is None:
            return None
        return max(0.0, now - self.last_lease_receive_at)

    def _raw_age(self, now: float) -> Optional[float]:
        if self.last_raw_autonomy_at is None:
            return None
        return max(0.0, now - self.last_raw_autonomy_at)

    def _lease_is_fresh(self, now: float) -> bool:
        age = self._lease_age(now)
        return (
            self.state.lease_active
            and age is not None
            and age <= self.lease_timeout_sec
        )

    def _raw_is_fresh(self, now: float) -> bool:
        age = self._raw_age(now)
        return age is not None and age <= self.raw_autonomy_timeout_sec

    def _expire_lease(self, now: float) -> bool:
        if self.state.lease_active and not self._lease_is_fresh(now):
            lease_id = self.state.lease_id
            self.state = transition(
                self.state,
                Event.LOSE_LEASE,
                lease_id=lease_id,
            ).state
            self.last_control_sequence = None
            return True
        return False

    def _snapshot(self, now: float, reason: str) -> SupervisorSnapshot:
        lease_fresh = self._lease_is_fresh(now)
        raw_fresh = self._raw_is_fresh(now)
        autonomy_open = self.state.autonomy_permitted and lease_fresh
        return SupervisorSnapshot(
            state=self.state,
            brake_intent=not (autonomy_open and raw_fresh),
            lease_fresh=lease_fresh,
            lease_age_sec=self._lease_age(now),
            raw_autonomy_fresh=raw_fresh,
            raw_autonomy_age_sec=self._raw_age(now),
            last_control_sequence=self.last_control_sequence,
            reason=reason,
        )

    def tick(self, now: float) -> SupervisorResult:
        """Expire time-bounded state and produce periodic brake intent."""
        expired = self._expire_lease(now)
        reason = 'LEASE_EXPIRED' if expired else self._reason(now)
        return SupervisorResult(self._snapshot(now, reason))

    def _reason(self, now: float) -> str:
        if self.state.mode == Mode.IDLE:
            return 'IDLE_BRAKE'
        if self.state.dualsense_active:
            return 'DUALSENSE_TAKEOVER'
        if not self._lease_is_fresh(now):
            return 'NO_FRESH_LEASE'
        if self.state.run_blocked_until_release:
            return 'RUN_REARM_REQUIRED'
        if self.state.mode != Mode.AUTONOMY:
            return 'MODE_BRAKE'
        if not self.state.run_held:
            return 'RUN_RELEASED'
        if self.state.goal is None:
            return 'NO_GOAL_INTENT'
        if not self._raw_is_fresh(now):
            return 'RAW_AUTONOMY_STALE'
        return 'AUTONOMY_RUN'

    def handle_control_event(
        self,
        *,
        event: ControlEvent,
        client_id: str,
        lease_id: str,
        sequence: int,
        now: float,
    ) -> SupervisorResult:
        """Validate ownership/order, refresh the lease, and reduce an event."""
        self._expire_lease(now)
        if event == ControlEvent.LEASE_ACQUIRED:
            if self.state.lease_active:
                return SupervisorResult(
                    self._snapshot(now, 'LEASE_REJECTED'),
                    accepted=False,
                )
            result = transition(
                self.state,
                Event.ACQUIRE_LEASE,
                client_id=client_id,
                lease_id=lease_id,
            )
            if result.state == self.state:
                return SupervisorResult(
                    self._snapshot(now, 'LEASE_REJECTED'),
                    accepted=False,
                )
            self.state = result.state
            self.last_lease_receive_at = now
            self.last_control_sequence = sequence
            return SupervisorResult(self._snapshot(now, 'LEASE_ACQUIRED'))

        if (
            not self.state.lease_active
            or client_id != self.state.lease_client_id
            or lease_id != self.state.lease_id
        ):
            return SupervisorResult(
                self._snapshot(now, 'NON_OWNER_EVENT_REJECTED'),
                accepted=False,
            )
        if (
            self.last_control_sequence is not None
            and not _is_newer_sequence(
                sequence, self.last_control_sequence
            )
        ):
            return SupervisorResult(
                self._snapshot(now, 'STALE_EVENT_REJECTED'),
                accepted=False,
            )

        event_map = {
            ControlEvent.RUN_PRESSED: Event.RUN_PRESS,
            ControlEvent.RUN_RELEASED: Event.RUN_RELEASE,
            ControlEvent.STOP: Event.STOP,
            ControlEvent.MANUAL_ACTIVE: Event.MANUAL_ACTIVE,
            ControlEvent.MANUAL_INACTIVE: Event.MANUAL_INACTIVE,
            ControlEvent.LEASE_RELEASED: Event.RELEASE_LEASE,
        }
        supported = event == ControlEvent.HEARTBEAT or (
            event == ControlEvent.GOAL_SELECTED or event in event_map
        )
        if not supported:
            return SupervisorResult(
                self._snapshot(now, 'EVENT_REJECTED'),
                accepted=False,
            )

        self.last_lease_receive_at = now
        self.last_control_sequence = sequence
        if event == ControlEvent.HEARTBEAT:
            return SupervisorResult(self._snapshot(now, self._reason(now)))
        if event == ControlEvent.GOAL_SELECTED:
            goal = GoalIntent(
                map_name=self.state.active_autonomy_map,
                x=0.0,
                y=0.0,
                yaw=0.0,
            )
            self.state = transition(
                self.state,
                Event.SELECT_GOAL,
                lease_id=lease_id,
                goal=goal,
            ).state
        elif event in event_map:
            self.state = transition(
                self.state,
                event_map[event],
                lease_id=lease_id,
            ).state
        if event == ControlEvent.LEASE_RELEASED:
            self.last_lease_receive_at = None
            self.last_control_sequence = None
        return SupervisorResult(self._snapshot(now, self._reason(now)))

    def request_mode(
        self,
        *,
        mode: Mode,
        request_id: int,
        lease_id: str,
        now: float,
    ) -> SupervisorResult:
        """Apply a lease-scoped mode request without refreshing its lease."""
        self._expire_lease(now)
        previous = self.state
        self.state = transition(
            self.state,
            Event.REQUEST_MODE,
            lease_id=lease_id,
            mode=mode,
            request_id=request_id,
        ).state
        accepted = self.state != previous
        reason = 'MODE_ACCEPTED' if accepted else 'MODE_REJECTED'
        return SupervisorResult(
            self._snapshot(now, reason),
            accepted=accepted,
        )

    def set_dualsense_active(
        self, active: bool, now: float
    ) -> SupervisorResult:
        """Apply highest-priority DualSense presence without lease refresh."""
        self._expire_lease(now)
        event = (
            Event.DUALSENSE_ACTIVE
            if active
            else Event.DUALSENSE_INACTIVE
        )
        self.state = transition(self.state, event).state
        return SupervisorResult(self._snapshot(now, self._reason(now)))

    def receive_raw_autonomy(
        self, command: VelocityCommand, now: float
    ) -> SupervisorResult:
        """Forward this sample only through every current safety gate."""
        self._expire_lease(now)
        if not command.valid:
            self.last_raw_autonomy_at = None
            return SupervisorResult(
                self._snapshot(now, 'INVALID_RAW_AUTONOMY'),
                accepted=False,
            )
        self.last_raw_autonomy_at = now
        snapshot = self._snapshot(now, self._reason(now))
        if (
            snapshot.state.authority == Authority.PADDOCK_AUTONOMY
            and snapshot.lease_fresh
            and snapshot.raw_autonomy_fresh
            and not snapshot.brake_intent
        ):
            return SupervisorResult(snapshot, autonomy_command=command)
        return SupervisorResult(snapshot)

    def restart(self, now: float) -> SupervisorResult:
        """Model process restart: no lease, RUN, goal, or raw sample survives."""
        self.state = transition(self.state, Event.RESTART).state
        self.last_lease_receive_at = None
        self.last_raw_autonomy_at = None
        self.last_control_sequence = None
        return SupervisorResult(self._snapshot(now, 'RESTART_BRAKE'))
