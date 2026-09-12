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

from dataclasses import dataclass, replace
from enum import IntEnum
import math
from typing import Optional, Tuple

from runner_paddock.state_machine import Authority
from runner_paddock.state_machine import Event
from runner_paddock.state_machine import GoalIntent
from runner_paddock.state_machine import Mode
from runner_paddock.state_machine import PaddockState
from runner_paddock.state_machine import transition


# Two independent freshness bounds on the operator link (see __init__):
#   * lease_timeout_sec       -- RUN / autonomous-motion deadman (tight)
#   * control_liveness_sec    -- lease-ownership liveness backstop (forgiving)
DEFAULT_LEASE_TIMEOUT_SEC = 0.150
DEFAULT_CONTROL_LIVENESS_SEC = 3.0
DEFAULT_RAW_AUTONOMY_TIMEOUT_SEC = 0.150
DEFAULT_MANUAL_TIMEOUT_SEC = 0.250
DEFAULT_MANUAL_MAX_SPEED_MPS = 0.40
MAX_MANUAL_MAX_SPEED_MPS = 0.40
DEFAULT_MINIMUM_MOVING_SPEED_MPS = 0.25
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
    CLEAR_STOP = 10


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
class ManualDemand:
    """One authority-bounded browser manual demand."""

    sequence: int
    signed_speed_mps: float
    steering_normalized: float


@dataclass(frozen=True)
class SupervisorSnapshot:
    """One queryable authority and timing snapshot."""

    state: PaddockState
    brake_intent: bool
    lease_fresh: bool
    lease_age_sec: Optional[float]
    raw_autonomy_fresh: bool
    raw_autonomy_age_sec: Optional[float]
    manual_input_fresh: bool
    manual_input_age_sec: Optional[float]
    manual_applied_speed_mps: float
    manual_applied_steering: float
    last_control_sequence: Optional[int]
    reason: str


@dataclass(frozen=True)
class SupervisorResult:
    """Result of one input, including any supervised autonomy output."""

    snapshot: SupervisorSnapshot
    autonomy_command: Optional[VelocityCommand] = None
    manual_demand: Optional[ManualDemand] = None
    manual_command: Optional[VelocityCommand] = None
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
        control_liveness_sec: Optional[float] = None,
        manual_timeout_sec: float = DEFAULT_MANUAL_TIMEOUT_SEC,
        manual_max_speed_mps: float = DEFAULT_MANUAL_MAX_SPEED_MPS,
        minimum_moving_speed_mps: float = DEFAULT_MINIMUM_MOVING_SPEED_MPS,
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
        # ``lease_timeout_sec`` is the RUN / autonomous-motion deadman: with RUN
        # held, no fresh ordered control event for this long revokes autonomous
        # motion (brake). It stays tight per v1.3 and is unaffected by this
        # split. ``control_liveness_sec`` is a separate, forgiving backstop on
        # bare lease *ownership*: the operator keeps the control lease (and the
        # mutating UI controls stay live) as long as the browser is heartbeating
        # inside this window, so ordinary Wi-Fi jitter no longer drops the lease
        # mid-session. It must be >= the motion deadman; it defaults to the
        # motion deadman, which reproduces the pre-split single-timeout model.
        if control_liveness_sec is None:
            control_liveness_sec = lease_timeout_sec
        if (
            not math.isfinite(control_liveness_sec)
            or control_liveness_sec < lease_timeout_sec
        ):
            raise ValueError(
                'control_liveness_sec must be finite and >= lease_timeout_sec'
            )
        self.lease_timeout_sec = lease_timeout_sec
        self.control_liveness_sec = control_liveness_sec
        self.raw_autonomy_timeout_sec = raw_autonomy_timeout_sec
        if not math.isfinite(manual_timeout_sec) or manual_timeout_sec <= 0.0:
            raise ValueError('manual_timeout_sec must be finite and positive')
        if not self.manual_speed_ceiling_valid(
            manual_max_speed_mps, minimum_moving_speed_mps
        ):
            raise ValueError(
                'manual speed ceiling must be zero or within the moving floor '
                f'and {MAX_MANUAL_MAX_SPEED_MPS:.2f} m/s'
            )
        self.manual_timeout_sec = manual_timeout_sec
        self.manual_max_speed_mps = manual_max_speed_mps
        self.minimum_moving_speed_mps = minimum_moving_speed_mps
        self.state = PaddockState(
            active_autonomy_map=active_autonomy_map
        )
        self.last_lease_receive_at: Optional[float] = None
        self.last_raw_autonomy_at: Optional[float] = None
        self.last_manual_input_at: Optional[float] = None
        self.last_raw_manual_at: Optional[float] = None
        self._manual_demand: Optional[ManualDemand] = None
        self.last_control_sequence: Optional[int] = None
        # (client_id, lease_id) of a lease that lapsed the liveness backstop;
        # a matching heartbeat from the same still-connected browser reinstates
        # it in place instead of forcing the operator to reload the page.
        self._lapsed_lease: Optional[Tuple[str, str]] = None

    @staticmethod
    def manual_speed_ceiling_valid(
        value: float,
        minimum_moving_speed_mps: float = DEFAULT_MINIMUM_MOVING_SPEED_MPS,
    ) -> bool:
        """Return whether a browser-manual ceiling is in the operator range."""
        return (
            math.isfinite(value)
            and (
                value == 0.0
                or minimum_moving_speed_mps
                <= value
                <= MAX_MANUAL_MAX_SPEED_MPS
            )
        )

    def set_manual_max_speed_mps(self, value: float) -> None:
        """Apply one validated operational browser-manual speed ceiling."""
        if not self.manual_speed_ceiling_valid(
            value, self.minimum_moving_speed_mps
        ):
            raise ValueError(
                'manual_max_speed_mps must be 0 or within '
                f'[{self.minimum_moving_speed_mps:.2f}, '
                f'{MAX_MANUAL_MAX_SPEED_MPS:.2f}]'
            )
        self.manual_max_speed_mps = float(value)

    def _lease_age(self, now: float) -> Optional[float]:
        if self.last_lease_receive_at is None:
            return None
        return max(0.0, now - self.last_lease_receive_at)

    def _raw_age(self, now: float) -> Optional[float]:
        if self.last_raw_autonomy_at is None:
            return None
        return max(0.0, now - self.last_raw_autonomy_at)

    def _lease_is_fresh(self, now: float) -> bool:
        """RUN / autonomous-motion deadman: tight bound for permitting motion."""
        age = self._lease_age(now)
        return (
            self.state.lease_active
            and age is not None
            and age <= self.lease_timeout_sec
        )

    def _lease_is_live(self, now: float) -> bool:
        """Lease-ownership liveness: forgiving bound for keeping the lease."""
        age = self._lease_age(now)
        return (
            self.state.lease_active
            and age is not None
            and age <= self.control_liveness_sec
        )

    def _raw_is_fresh(self, now: float) -> bool:
        age = self._raw_age(now)
        return age is not None and age <= self.raw_autonomy_timeout_sec

    def _manual_age(self, now: float) -> Optional[float]:
        if self.last_manual_input_at is None:
            return None
        return max(0.0, now - self.last_manual_input_at)

    def _manual_is_fresh(self, now: float) -> bool:
        age = self._manual_age(now)
        return age is not None and age <= self.manual_timeout_sec

    def _raw_manual_is_fresh(self, now: float) -> bool:
        return (
            self.last_raw_manual_at is not None
            and now - self.last_raw_manual_at <= self.raw_autonomy_timeout_sec
        )

    def _expire_manual(self, now: float) -> bool:
        if self.state.manual_active and not self._manual_is_fresh(now):
            self.state = transition(
                self.state, Event.MANUAL_INACTIVE, lease_id=self.state.lease_id
            ).state
            self._manual_demand = None
            self.last_manual_input_at = None
            return True
        return False

    def _clear_manual_if_inactive(self) -> None:
        if not self.state.manual_active:
            self._manual_demand = None
            self.last_manual_input_at = None

    def _expire_lease(self, now: float) -> bool:
        # Only the forgiving liveness backstop drops the lease. The tight motion
        # deadman (_lease_is_fresh) still gates autonomous motion every tick via
        # _snapshot, but a brief link stall no longer costs the operator the
        # lease itself.
        if self.state.lease_active and not self._lease_is_live(now):
            lease_id = self.state.lease_id
            client_id = self.state.lease_client_id
            self.state = transition(
                self.state,
                Event.LOSE_LEASE,
                lease_id=lease_id,
            ).state
            self.last_control_sequence = None
            self._lapsed_lease = (client_id, lease_id)
            self._clear_manual_if_inactive()
            return True
        return False

    def _snapshot(self, now: float, reason: str) -> SupervisorSnapshot:
        # Motion is gated by the tight deadman; the browser-visible lease_fresh
        # flag tracks the forgiving liveness backstop so the status panel does
        # not flap "STALE" between ordinary heartbeats.
        motion_fresh = self._lease_is_fresh(now)
        lease_live = self._lease_is_live(now)
        raw_fresh = self._raw_is_fresh(now)
        autonomy_open = self.state.autonomy_permitted and motion_fresh
        manual_fresh = self._manual_is_fresh(now)
        manual_open = (
            self.state.authority == Authority.PADDOCK_MANUAL
            and manual_fresh
            and self._raw_manual_is_fresh(now)
        )
        demand = self._manual_demand
        return SupervisorSnapshot(
            state=self.state,
            brake_intent=not ((autonomy_open and raw_fresh) or manual_open),
            lease_fresh=lease_live,
            lease_age_sec=self._lease_age(now),
            raw_autonomy_fresh=raw_fresh,
            raw_autonomy_age_sec=self._raw_age(now),
            manual_input_fresh=manual_fresh,
            manual_input_age_sec=self._manual_age(now),
            manual_applied_speed_mps=(
                demand.signed_speed_mps if demand is not None else 0.0
            ),
            manual_applied_steering=(
                demand.steering_normalized if demand is not None else 0.0
            ),
            last_control_sequence=self.last_control_sequence,
            reason=reason,
        )

    def tick(self, now: float) -> SupervisorResult:
        """Expire time-bounded state and produce periodic brake intent."""
        expired = self._expire_lease(now)
        manual_expired = self._expire_manual(now)
        reason = (
            'LEASE_EXPIRED' if expired else
            'MANUAL_INPUT_STALE' if manual_expired else self._reason(now)
        )
        return SupervisorResult(self._snapshot(now, reason))

    def set_runtime_mode(
        self,
        mode: Mode,
        active_autonomy_map: str,
        now: float,
        *,
        runtime_stable: bool,
    ) -> SupervisorResult:
        """Follow authoritative runtime state and disarm across every change."""
        self._expire_lease(now)
        effective_mode = mode if runtime_stable else Mode.IDLE
        changed = (
            effective_mode != self.state.mode
            or active_autonomy_map != self.state.active_autonomy_map
            or not runtime_stable
        )
        self.state = replace(
            self.state,
            mode=effective_mode,
            active_autonomy_map=active_autonomy_map,
            run_held=False if changed else self.state.run_held,
            run_blocked_until_release=(
                False if changed else self.state.run_blocked_until_release
            ),
            manual_active=False if changed else self.state.manual_active,
            goal=None if changed else self.state.goal,
            navigation_active=False if changed else self.state.navigation_active,
        )
        self._clear_manual_if_inactive()
        reason = 'RUNTIME_MODE' if runtime_stable else 'RUNTIME_NOT_STABLE'
        return SupervisorResult(self._snapshot(now, reason))

    def _reason(self, now: float) -> str:
        if self.state.mode == Mode.IDLE:
            return 'IDLE_BRAKE'
        if self.state.dualsense_active:
            return 'DUALSENSE_TAKEOVER'
        if not self._lease_is_live(now):
            return 'NO_FRESH_LEASE'
        if self.state.manual_active:
            if not self._manual_is_fresh(now):
                return 'MANUAL_INPUT_STALE'
            if not self._raw_manual_is_fresh(now):
                return 'RAW_MANUAL_STALE'
            return 'MANUAL_ACTIVE'
        if self.state.run_blocked_until_release:
            return 'RUN_REARM_REQUIRED'
        if self.state.mode != Mode.AUTONOMY:
            return 'MODE_BRAKE'
        if not self.state.run_held:
            return 'RUN_RELEASED'
        if self.state.goal is None:
            return 'NO_GOAL_INTENT'
        if not self._lease_is_fresh(now):
            # Lease still owned (within the liveness backstop) but the RUN
            # deadman lapsed: autonomous motion is revoked until fresh events
            # resume.
            return 'CONTROL_STALE'
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
        goal_x: float = 0.0,
        goal_y: float = 0.0,
        goal_yaw: float = 0.0,
        manual_speed_mps: float = 0.0,
        manual_steering: float = 0.0,
    ) -> SupervisorResult:
        """Validate ownership/order, refresh the lease, and reduce an event."""
        self._expire_lease(now)

        if (
            event == ControlEvent.HEARTBEAT
            and not self.state.lease_active
            and self._lapsed_lease == (client_id, lease_id)
        ):
            # The lease lapsed the liveness backstop, but the same browser is
            # still connected and heartbeating and no other controller has taken
            # over: reinstate ownership in place so the mutating UI controls
            # recover without a page reload. Autonomous motion does not resume
            # until RUN is deliberately pressed again (LOSE_LEASE cleared it).
            reinstated = transition(
                self.state,
                Event.ACQUIRE_LEASE,
                client_id=client_id,
                lease_id=lease_id,
            )
            if reinstated.state != self.state:
                self.state = reinstated.state
                self.last_lease_receive_at = now
                self.last_control_sequence = sequence
                self._lapsed_lease = None
                return SupervisorResult(
                    self._snapshot(now, 'LEASE_REINSTATED')
                )

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
            self._lapsed_lease = None
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
            ControlEvent.LEASE_LOST: Event.LOSE_LEASE,
        }
        supported = event in (
            ControlEvent.HEARTBEAT,
            ControlEvent.CLEAR_STOP,
        ) or (
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
        if event == ControlEvent.CLEAR_STOP:
            # Executor acknowledgement is the authority for STOP state. This
            # event only validates fresh lease ownership/order and disarms all
            # normal grants before the clear request is sent.
            self.state = transition(
                self.state,
                Event.STOP,
                lease_id=lease_id,
            ).state
            return SupervisorResult(self._snapshot(now, 'CLEAR_STOP_REQUESTED'))
        if event == ControlEvent.GOAL_SELECTED:
            if not all(
                math.isfinite(value)
                for value in (goal_x, goal_y, goal_yaw)
            ):
                return SupervisorResult(
                    self._snapshot(now, 'GOAL_POSE_NOT_FINITE'),
                    accepted=False,
                )
            goal = GoalIntent(
                map_name=self.state.active_autonomy_map,
                x=float(goal_x),
                y=float(goal_y),
                yaw=float(goal_yaw),
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
            if event == ControlEvent.MANUAL_ACTIVE:
                if not all(math.isfinite(value) for value in (
                    manual_speed_mps, manual_steering
                )):
                    self.state = transition(
                        self.state, Event.MANUAL_INACTIVE, lease_id=lease_id
                    ).state
                    return SupervisorResult(
                        self._snapshot(now, 'INVALID_MANUAL_DEMAND'),
                        accepted=False,
                    )
                if self.state.authority != Authority.PADDOCK_MANUAL:
                    return SupervisorResult(
                        self._snapshot(now, 'MANUAL_NOT_ELIGIBLE'),
                        accepted=False,
                    )
                speed = max(
                    -self.manual_max_speed_mps,
                    min(self.manual_max_speed_mps, float(manual_speed_mps)),
                )
                if 0.0 < abs(speed) < self.minimum_moving_speed_mps:
                    speed = math.copysign(self.minimum_moving_speed_mps, speed)
                demand = ManualDemand(
                    sequence=sequence,
                    signed_speed_mps=speed,
                    steering_normalized=max(
                        -1.0, min(1.0, float(manual_steering))
                    ),
                )
                self._manual_demand = demand
                self.last_manual_input_at = now
                return SupervisorResult(
                    self._snapshot(now, 'MANUAL_ACTIVE'),
                    manual_demand=demand,
                )
            if event == ControlEvent.MANUAL_INACTIVE:
                self._manual_demand = None
                self.last_manual_input_at = None
        if event == ControlEvent.LEASE_RELEASED:
            self.last_lease_receive_at = None
            self.last_control_sequence = None
        self._clear_manual_if_inactive()
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
        self._clear_manual_if_inactive()
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
            and self._lease_is_fresh(now)
            and snapshot.raw_autonomy_fresh
            and not snapshot.brake_intent
        ):
            return SupervisorResult(snapshot, autonomy_command=command)
        return SupervisorResult(snapshot)

    def receive_raw_manual(
        self,
        command: VelocityCommand,
        sequence: int,
        now: float,
    ) -> SupervisorResult:
        """Forward a converted manual sample only for its current demand."""
        self._expire_lease(now)
        self._expire_manual(now)
        demand = self._manual_demand
        if not command.valid or demand is None or sequence != demand.sequence:
            return SupervisorResult(
                self._snapshot(now, 'INVALID_RAW_MANUAL'), accepted=False
            )
        self.last_raw_manual_at = now
        snapshot = self._snapshot(now, self._reason(now))
        if (
            snapshot.state.authority == Authority.PADDOCK_MANUAL
            and snapshot.manual_input_fresh
            and not snapshot.brake_intent
        ):
            return SupervisorResult(snapshot, manual_command=command)
        return SupervisorResult(snapshot)

    def restart(self, now: float) -> SupervisorResult:
        """Model process restart: no lease, RUN, goal, or raw sample survives."""
        self.state = transition(self.state, Event.RESTART).state
        self.last_lease_receive_at = None
        self.last_raw_autonomy_at = None
        self.last_manual_input_at = None
        self.last_raw_manual_at = None
        self._manual_demand = None
        self.last_control_sequence = None
        self._lapsed_lease = None
        return SupervisorResult(self._snapshot(now, 'RESTART_BRAKE'))
