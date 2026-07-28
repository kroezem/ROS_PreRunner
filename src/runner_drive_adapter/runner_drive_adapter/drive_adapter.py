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

"""Pure conversion and bounded stall-assist state for Runner."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


CURVATURE_REL_TOLERANCE = 1e-12
CURVATURE_ABS_TOLERANCE = 1e-12
ACTIVE_STATES = {'RAMPING', 'HOLDING', 'DECAYING'}


def validate_table(
    throttle_breakpoints: Sequence[float],
    speed_breakpoints: Sequence[float],
) -> None:
    """Validate a replaceable monotonic speed-to-throttle table."""
    if len(throttle_breakpoints) != len(speed_breakpoints):
        raise ValueError('throttle and speed breakpoint lengths must match')
    if len(throttle_breakpoints) < 2:
        raise ValueError('at least two table points are required')
    values = tuple(throttle_breakpoints) + tuple(speed_breakpoints)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('all table values must be finite')
    if not all(0.0 <= value <= 1.0 for value in throttle_breakpoints):
        raise ValueError('throttle breakpoints must be within [0, 1]')
    if not all(value > 0.0 for value in speed_breakpoints):
        raise ValueError('speed breakpoints must be positive')
    if not all(
        left < right
        for left, right in zip(
            throttle_breakpoints,
            throttle_breakpoints[1:],
        )
    ):
        raise ValueError('throttle breakpoints must be strictly increasing')
    if not all(
        left < right
        for left, right in zip(speed_breakpoints, speed_breakpoints[1:])
    ):
        raise ValueError('speed breakpoints must be strictly increasing')


def lookup_throttle(
    speed: float,
    throttle_breakpoints: Sequence[float],
    speed_breakpoints: Sequence[float],
) -> tuple[float, bool]:
    """Interpolate throttle deterministically and clamp above the table."""
    validate_table(throttle_breakpoints, speed_breakpoints)
    if speed <= speed_breakpoints[0]:
        return float(throttle_breakpoints[0]), False
    if speed >= speed_breakpoints[-1]:
        return float(throttle_breakpoints[-1]), speed > speed_breakpoints[-1]

    for index in range(1, len(speed_breakpoints)):
        upper_speed = speed_breakpoints[index]
        if speed <= upper_speed:
            lower_speed = speed_breakpoints[index - 1]
            fraction = (speed - lower_speed) / (
                upper_speed - lower_speed
            )
            lower_throttle = throttle_breakpoints[index - 1]
            upper_throttle = throttle_breakpoints[index]
            return (
                float(
                    lower_throttle
                    + fraction * (upper_throttle - lower_throttle)
                ),
                False,
            )
    raise AssertionError('validated lookup table did not contain speed')


@dataclass(frozen=True)
class AdapterConfig:
    """Validated drive-adapter configuration."""

    wheelbase: float = 0.178
    max_steering_angle: float = 0.3614
    steering_min_speed: float = 0.05
    throttle_breakpoints: tuple[float, ...] = (
        0.340,
        0.350,
        0.360,
        0.380,
    )
    speed_breakpoints: tuple[float, ...] = (
        0.126,
        0.188,
        0.233,
        0.290,
    )
    minimum_moving_speed: float = 0.126
    floor_promotion_min_ratio: float = 0.50
    stall_assist_enabled: bool = True
    under_speed_ratio: float = 0.40
    under_speed_absolute_ceiling: float = 0.10
    under_speed_qualification_sec: float = 0.30
    command_stability_tolerance: float = 0.02
    ramp_rate_per_sec: float = 0.10
    boost_throttle_ceiling: float = 0.50
    maximum_assist_duration_sec: float = 1.50
    motion_confirm_speed: float = 0.06
    motion_confirm_duration_sec: float = 0.20
    motion_hold_duration_sec: float = 0.30
    decay_rate_per_sec: float = 0.15
    overspeed_margin: float = 0.02
    wheelspin_edge_rate_threshold: float = 1.0
    wheelspin_vehicle_speed_threshold: float = 0.02
    wheelspin_qualification_sec: float = 0.25
    cooldown_duration_sec: float = 0.50
    motion_signal_timeout_sec: float = 0.25
    encoder_state_timeout_sec: float = 0.25
    cmd_vel_nav_timeout: float = 0.25
    publication_rate: float = 20.0

    def __post_init__(self) -> None:
        """Reject unsafe or internally inconsistent parameter sets."""
        validate_table(
            self.throttle_breakpoints,
            self.speed_breakpoints,
        )
        if not isinstance(self.stall_assist_enabled, bool):
            raise ValueError('stall_assist_enabled must be a boolean')
        scalar_values = (
            self.wheelbase,
            self.max_steering_angle,
            self.steering_min_speed,
            self.minimum_moving_speed,
            self.floor_promotion_min_ratio,
            self.under_speed_ratio,
            self.under_speed_absolute_ceiling,
            self.under_speed_qualification_sec,
            self.command_stability_tolerance,
            self.ramp_rate_per_sec,
            self.boost_throttle_ceiling,
            self.maximum_assist_duration_sec,
            self.motion_confirm_speed,
            self.motion_confirm_duration_sec,
            self.motion_hold_duration_sec,
            self.decay_rate_per_sec,
            self.overspeed_margin,
            self.wheelspin_edge_rate_threshold,
            self.wheelspin_vehicle_speed_threshold,
            self.wheelspin_qualification_sec,
            self.cooldown_duration_sec,
            self.motion_signal_timeout_sec,
            self.encoder_state_timeout_sec,
            self.cmd_vel_nav_timeout,
            self.publication_rate,
        )
        if not all(math.isfinite(value) for value in scalar_values):
            raise ValueError('all scalar parameters must be finite')
        if self.wheelbase <= 0.0:
            raise ValueError('wheelbase must be greater than zero')
        if not 0.0 < self.max_steering_angle < math.pi / 2.0:
            raise ValueError(
                'max_steering_angle must be between zero and pi/2'
            )
        if self.steering_min_speed <= 0.0:
            raise ValueError('steering_min_speed must be greater than zero')
        if self.minimum_moving_speed <= 0.0:
            raise ValueError('minimum_moving_speed must be greater than zero')
        if not 0.0 < self.floor_promotion_min_ratio <= 1.0:
            raise ValueError(
                'floor_promotion_min_ratio must be within (0, 1]'
            )
        if not 0.0 < self.under_speed_ratio <= 1.0:
            raise ValueError('under_speed_ratio must be within (0, 1]')
        nonnegative = {
            'under_speed_absolute_ceiling':
                self.under_speed_absolute_ceiling,
            'command_stability_tolerance':
                self.command_stability_tolerance,
            'motion_confirm_speed': self.motion_confirm_speed,
            'overspeed_margin': self.overspeed_margin,
            'wheelspin_edge_rate_threshold':
                self.wheelspin_edge_rate_threshold,
            'wheelspin_vehicle_speed_threshold':
                self.wheelspin_vehicle_speed_threshold,
        }
        for name, value in nonnegative.items():
            if value < 0.0:
                raise ValueError(f'{name} must be nonnegative')
        positive = {
            'under_speed_qualification_sec':
                self.under_speed_qualification_sec,
            'ramp_rate_per_sec': self.ramp_rate_per_sec,
            'maximum_assist_duration_sec':
                self.maximum_assist_duration_sec,
            'motion_confirm_duration_sec':
                self.motion_confirm_duration_sec,
            'motion_hold_duration_sec': self.motion_hold_duration_sec,
            'decay_rate_per_sec': self.decay_rate_per_sec,
            'wheelspin_qualification_sec':
                self.wheelspin_qualification_sec,
            'cooldown_duration_sec': self.cooldown_duration_sec,
            'motion_signal_timeout_sec': self.motion_signal_timeout_sec,
            'encoder_state_timeout_sec': self.encoder_state_timeout_sec,
            'cmd_vel_nav_timeout': self.cmd_vel_nav_timeout,
            'publication_rate': self.publication_rate,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        if self.boost_throttle_ceiling < max(
            self.throttle_breakpoints
        ):
            raise ValueError(
                'boost_throttle_ceiling must be at least the maximum '
                'feedforward throttle'
            )
        if self.boost_throttle_ceiling > 1.0:
            raise ValueError(
                'boost_throttle_ceiling must not exceed motor command '
                'limit 1.0'
            )
        if not (
            self.speed_breakpoints[0]
            <= self.minimum_moving_speed
            <= self.speed_breakpoints[-1]
        ):
            raise ValueError(
                'minimum_moving_speed must fall within the speed table'
            )

    @property
    def promotion_threshold(self) -> float:
        """Return the inclusive lower bound for floor promotion."""
        return (
            self.minimum_moving_speed * self.floor_promotion_min_ratio
        )

    @property
    def maximum_curvature(self) -> float:
        """Return maximum measured curvature in inverse metres."""
        return math.tan(self.max_steering_angle) / self.wheelbase


@dataclass(frozen=True)
class AdapterDecision:
    """One timer-cycle output and its bag-observable diagnostic fields."""

    publish_command: bool
    mode: str
    reason: str
    effective_speed: float = 0.0
    feedforward_throttle: float = -1.0
    final_throttle: float = -1.0
    normalized_steering: float = 0.0
    assist_state: str = 'IDLE'
    applied_boost: float = 0.0
    event_count: int = 0
    last_exit_reason: str = 'none'

    def diagnostic_text(self) -> str:
        """Serialize a stable, compact diagnostic record."""
        return (
            f'mode={self.mode};reason={self.reason};'
            f'stall_assist_state={self.assist_state};'
            f'applied_boost={self.applied_boost:.9f};'
            f'event_count={self.event_count};'
            f'last_exit_reason={self.last_exit_reason};'
            f'effective_speed={self.effective_speed:.9f};'
            f'feedforward_throttle={self.feedforward_throttle:.9f};'
            f'final_throttle={self.final_throttle:.9f};'
            f'normalized_steering={self.normalized_steering:.9f}'
        )


@dataclass
class AssistEventSummary:
    """One structured stall-assist event summary."""

    event_id: int
    commanded_speed: float
    feedforward_throttle: float
    peak_throttle: float
    event_duration: float
    ramp_duration: float
    hold_duration: float
    decay_duration: float
    primary_motion_signal: str
    primary_speed_start: float
    primary_speed_peak: float
    ekf_speed_start: float
    ekf_speed_peak: float
    encoder_edge_rate_start: float
    encoder_edge_rate_peak: float
    exit_reason: str

    def diagnostic_text(self) -> str:
        """Return a stable key-value log and bag record."""
        values = (
            ('event_id', str(self.event_id)),
            ('commanded_speed', f'{self.commanded_speed:.9f}'),
            ('feedforward_throttle',
             f'{self.feedforward_throttle:.9f}'),
            ('peak_throttle', f'{self.peak_throttle:.9f}'),
            ('event_duration', f'{self.event_duration:.9f}'),
            ('ramp_duration', f'{self.ramp_duration:.9f}'),
            ('hold_duration', f'{self.hold_duration:.9f}'),
            ('decay_duration', f'{self.decay_duration:.9f}'),
            ('primary_motion_signal', self.primary_motion_signal),
            ('primary_speed_start', f'{self.primary_speed_start:.9f}'),
            ('primary_speed_peak', f'{self.primary_speed_peak:.9f}'),
            ('ekf_speed_start', f'{self.ekf_speed_start:.9f}'),
            ('ekf_speed_peak', f'{self.ekf_speed_peak:.9f}'),
            ('encoder_edge_rate_start',
             f'{self.encoder_edge_rate_start:.9f}'),
            ('encoder_edge_rate_peak',
             f'{self.encoder_edge_rate_peak:.9f}'),
            ('exit_reason', self.exit_reason),
        )
        return ';'.join(f'{key}={value}' for key, value in values)


@dataclass
class _ActiveEvent:
    event_id: int
    start_time: float
    commanded_speed: float
    feedforward_throttle: float
    peak_throttle: float
    primary_speed_start: float
    primary_speed_peak: float
    encoder_edge_rate_start: float
    encoder_edge_rate_peak: float
    ramp_duration: float = 0.0
    hold_duration: float = 0.0
    decay_duration: float = 0.0
    pending_exit_reason: str = 'motion_confirmed_and_decayed'


class DriveAdapter:
    """Convert fresh Nav2 commands and manage bounded stall assistance."""

    def __init__(self, config: AdapterConfig):
        self.config = config
        self._command: Optional[tuple[float, float]] = None
        self._command_time: Optional[float] = None
        self._motion_speed = 0.0
        self._motion_time: Optional[float] = None
        self._motion_above_since: Optional[float] = None
        self._motion_above_samples = 0
        self._encoder_stationary: Optional[bool] = None
        self._encoder_edge_rate = 0.0
        self._encoder_direction = 0
        self._encoder_time: Optional[float] = None
        self._state = 'IDLE'
        self._state_entered_at = 0.0
        self._qualifying_speed: Optional[float] = None
        self._applied_throttle = 0.0
        self._last_step_time: Optional[float] = None
        self._cooldown_until = 0.0
        self._wheelspin_since: Optional[float] = None
        self._event: Optional[_ActiveEvent] = None
        self._event_count = 0
        self._last_exit_reason = 'none'
        self._transition_events: list[str] = []
        self._summaries: list[AssistEventSummary] = []

    @property
    def latest_command(self) -> Optional[tuple[float, float]]:
        """Return the latest raw command for logging only."""
        return self._command

    @property
    def state(self) -> str:
        """Return the current stall-assist state."""
        return self._state

    @property
    def event_count(self) -> int:
        """Return the number of started assist events."""
        return self._event_count

    @property
    def last_exit_reason(self) -> str:
        """Return the most recent event exit reason."""
        return self._last_exit_reason

    def update_command(
        self,
        linear_speed: float,
        yaw_rate: float,
        now: float,
    ) -> None:
        """Store the latest Nav2 command and monotonic receive time."""
        self._command = (linear_speed, yaw_rate)
        self._command_time = now

    def update_motion(self, forward_speed: float, now: float) -> None:
        """Store EKF body-forward velocity and confirmation history."""
        self._motion_speed = forward_speed
        self._motion_time = now
        if (
            math.isfinite(forward_speed)
            and forward_speed >= self.config.motion_confirm_speed
        ):
            if self._motion_above_since is None:
                self._motion_above_since = now
                self._motion_above_samples = 1
            else:
                self._motion_above_samples += 1
        else:
            self._motion_above_since = None
            self._motion_above_samples = 0

    def update_encoder(
        self,
        stationary: bool,
        edge_rate: float,
        pending_direction: int,
        now: float,
    ) -> None:
        """Store encoder motion, direction, and freshness."""
        self._encoder_stationary = bool(stationary)
        self._encoder_edge_rate = edge_rate
        self._encoder_direction = int(pending_direction)
        self._encoder_time = now

    def motion_is_stale(self, now: float) -> bool:
        """Return whether EKF velocity is unavailable, invalid, or old."""
        return (
            self._motion_time is None
            or not math.isfinite(self._motion_speed)
            or now - self._motion_time
            > self.config.motion_signal_timeout_sec
        )

    def encoder_is_stale(self, now: float) -> bool:
        """Return whether encoder state is unavailable, invalid, or old."""
        return (
            self._encoder_time is None
            or not math.isfinite(self._encoder_edge_rate)
            or now - self._encoder_time
            > self.config.encoder_state_timeout_sec
        )

    def take_transition_events(self) -> list[str]:
        """Return and clear important state-transition records."""
        events = self._transition_events
        self._transition_events = []
        return events

    def take_event_summaries(self) -> list[AssistEventSummary]:
        """Return and clear completed event summaries."""
        summaries = self._summaries
        self._summaries = []
        return summaries

    def shutdown(self, now: float) -> None:
        """Terminate an elevated event without publishing stale output."""
        if self._event is not None:
            self._finish_event('shutdown', now, cooldown=False)
        self._state = 'IDLE'
        self._applied_throttle = 0.0

    def step(self, now: float) -> AdapterDecision:
        """Produce one deterministic timer-cycle decision."""
        dt = 0.0
        if self._last_step_time is not None:
            dt = max(0.0, now - self._last_step_time)
        self._last_step_time = now

        if self._command is None or self._command_time is None:
            return self._silence('no_command')
        if now - self._command_time > self.config.cmd_vel_nav_timeout:
            if self._event is not None:
                self._finish_event('command_stale', now)
            return self._silence('stale_command')

        speed, yaw_rate = self._command
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            if self._event is not None:
                self._finish_event('invalid_command', now)
            return self._brake('nonfinite_input')
        if speed < 0.0:
            if self._event is not None:
                self._finish_event('brake_or_reverse', now)
            return self._brake('negative_speed')
        if speed == 0.0:
            if self._event is not None:
                self._finish_event('controller_command_ended', now)
            return self._brake('explicit_stop')
        if speed < self.config.steering_min_speed:
            if self._event is not None:
                self._finish_event('controller_command_ended', now)
            return self._brake('below_steering_min_speed')
        if speed < self.config.promotion_threshold:
            if self._event is not None:
                self._finish_event('controller_command_ended', now)
            return self._brake('below_promotion_threshold')

        requested_curvature = yaw_rate / speed
        maximum_curvature = self.config.maximum_curvature
        tolerance = max(
            CURVATURE_ABS_TOLERANCE,
            maximum_curvature * CURVATURE_REL_TOLERANCE,
        )
        if abs(requested_curvature) > maximum_curvature + tolerance:
            if self._event is not None:
                self._finish_event('steering_infeasible', now)
            return self._brake('steering_infeasible')

        effective_speed = max(speed, self.config.minimum_moving_speed)
        feedforward, clamped = lookup_throttle(
            effective_speed,
            self.config.throttle_breakpoints,
            self.config.speed_breakpoints,
        )
        if clamped:
            effective_speed = self.config.speed_breakpoints[-1]
        delta = math.atan(self.config.wheelbase * requested_curvature)
        steering = max(
            -1.0,
            min(1.0, delta / self.config.max_steering_angle),
        )

        if (
            self._event is not None
            and abs(speed - self._event.commanded_speed)
            > self.config.command_stability_tolerance
        ):
            self._finish_event('controller_command_ended', now)

        self._advance_assist(speed, feedforward, now, dt)
        final = feedforward
        if self._state in ACTIVE_STATES:
            final = max(feedforward, self._applied_throttle)
        boost = max(0.0, final - feedforward)
        if self._event is not None:
            self._event.peak_throttle = max(
                self._event.peak_throttle,
                final,
            )
            self._update_event_signal_peaks()
        reason = 'above_table_clamped' if clamped else 'feedforward'
        if self._state in ACTIVE_STATES:
            reason = 'stall_assist'
        return AdapterDecision(
            True,
            'forward',
            reason,
            effective_speed,
            feedforward,
            final,
            steering,
            self._state,
            boost,
            self._event_count,
            self._last_exit_reason,
        )

    def _advance_assist(
        self,
        speed: float,
        feedforward: float,
        now: float,
        dt: float,
    ) -> None:
        if self._state == 'COOLDOWN':
            if now + 1e-12 < self._cooldown_until:
                return
            self._set_state('NORMAL', now)
        if not self.config.stall_assist_enabled:
            self._set_state('NORMAL', now)
            return
        if self._state in ACTIVE_STATES:
            if self.motion_is_stale(now):
                self._finish_event('motion_signal_stale', now)
                return
            if self.encoder_is_stale(now):
                self._finish_event('encoder_stale', now)
                return
            assert self._event is not None
            if (
                now - self._event.start_time + 1e-12
                >= self.config.maximum_assist_duration_sec
            ):
                self._finish_event('maximum_duration', now)
                return
            if self._wheelspin(now):
                self._finish_event('wheelspin', now)
                return
            if (
                self._motion_speed
                > speed + self.config.overspeed_margin
                and self._state != 'DECAYING'
            ):
                self._event.pending_exit_reason = 'overspeed_fast_decay'
                self._set_state('DECAYING', now)
        if self._state == 'RAMPING':
            if self._motion_confirmed(now):
                self._event.pending_exit_reason = (
                    'motion_confirmed_and_decayed'
                )
                self._set_state('HOLDING', now)
                return
            elapsed = now - self._event.start_time
            self._applied_throttle = min(
                self._event.feedforward_throttle
                + self.config.ramp_rate_per_sec * elapsed,
                self.config.boost_throttle_ceiling,
            )
            if (
                self._applied_throttle
                >= self.config.boost_throttle_ceiling
            ):
                self._finish_event(
                    'ceiling_reached_without_motion',
                    now,
                )
            return
        if self._state == 'HOLDING':
            if (
                now - self._state_entered_at + 1e-12
                >= self.config.motion_hold_duration_sec
            ):
                self._set_state('DECAYING', now)
            return
        if self._state == 'DECAYING':
            self._applied_throttle = max(
                feedforward,
                self._applied_throttle
                - self.config.decay_rate_per_sec * dt,
            )
            if self._applied_throttle <= feedforward:
                reason = self._event.pending_exit_reason
                self._finish_event(reason, now)
            return

        sensors_fresh = (
            not self.motion_is_stale(now)
            and not self.encoder_is_stale(now)
        )
        measured = max(0.0, self._motion_speed)
        under_speed = (
            measured < self.config.under_speed_ratio * speed
            and measured < self.config.under_speed_absolute_ceiling
        )
        if not sensors_fresh or not under_speed:
            self._qualifying_speed = None
            self._set_state('NORMAL', now)
            return
        if self._state != 'QUALIFYING':
            self._qualifying_speed = speed
            self._set_state('QUALIFYING', now)
            return
        if (
            self._qualifying_speed is None
            or abs(speed - self._qualifying_speed)
            > self.config.command_stability_tolerance
        ):
            self._qualifying_speed = speed
            self._state_entered_at = now
            return
        if (
            now - self._state_entered_at + 1e-12
            >= self.config.under_speed_qualification_sec
        ):
            self._start_event(speed, feedforward, now)

    def _motion_confirmed(self, now: float) -> bool:
        return (
            self._motion_above_since is not None
            and self._motion_above_samples >= 2
            and now - self._motion_above_since + 1e-12
            >= self.config.motion_confirm_duration_sec
        )

    def _wheelspin(self, now: float) -> bool:
        condition = (
            self._encoder_direction > 0
            and not self._encoder_stationary
            and self._encoder_edge_rate
            >= self.config.wheelspin_edge_rate_threshold
            and self._motion_speed
            < self.config.wheelspin_vehicle_speed_threshold
        )
        if not condition:
            self._wheelspin_since = None
            return False
        if self._wheelspin_since is None:
            self._wheelspin_since = now
            return False
        return (
            now - self._wheelspin_since + 1e-12
            >= self.config.wheelspin_qualification_sec
        )

    def _start_event(
        self,
        speed: float,
        feedforward: float,
        now: float,
    ) -> None:
        self._event_count += 1
        self._event = _ActiveEvent(
            self._event_count,
            now,
            speed,
            feedforward,
            feedforward,
            self._motion_speed,
            self._motion_speed,
            self._encoder_edge_rate,
            self._encoder_edge_rate,
        )
        self._applied_throttle = feedforward
        self._wheelspin_since = None
        self._set_state('RAMPING', now)
        self._transition_events.append(
            f'start:event_id={self._event_count};'
            f'commanded_speed={speed:.9f};'
            f'feedforward_throttle={feedforward:.9f}'
        )

    def _finish_event(
        self,
        reason: str,
        now: float,
        cooldown: bool = True,
    ) -> None:
        event = self._event
        if event is None:
            return
        self._accumulate_state_time(now)
        self._update_event_signal_peaks()
        event.peak_throttle = max(
            event.peak_throttle,
            self._applied_throttle,
        )
        summary = AssistEventSummary(
            event.event_id,
            event.commanded_speed,
            event.feedforward_throttle,
            event.peak_throttle,
            max(0.0, now - event.start_time),
            event.ramp_duration,
            event.hold_duration,
            event.decay_duration,
            'ekf_velocity',
            event.primary_speed_start,
            event.primary_speed_peak,
            event.primary_speed_start,
            event.primary_speed_peak,
            event.encoder_edge_rate_start,
            event.encoder_edge_rate_peak,
            reason,
        )
        self._summaries.append(summary)
        self._last_exit_reason = reason
        self._event = None
        self._applied_throttle = 0.0
        self._wheelspin_since = None
        self._motion_above_since = None
        self._motion_above_samples = 0
        if cooldown:
            self._cooldown_until = now + self.config.cooldown_duration_sec
            self._set_state('COOLDOWN', now)
        else:
            self._set_state('IDLE', now)

    def _update_event_signal_peaks(self) -> None:
        if self._event is None:
            return
        self._event.primary_speed_peak = max(
            self._event.primary_speed_peak,
            self._motion_speed,
        )
        self._event.encoder_edge_rate_peak = max(
            self._event.encoder_edge_rate_peak,
            self._encoder_edge_rate,
        )

    def _accumulate_state_time(self, now: float) -> None:
        if self._event is None:
            return
        duration = max(0.0, now - self._state_entered_at)
        if self._state == 'RAMPING':
            self._event.ramp_duration += duration
        elif self._state == 'HOLDING':
            self._event.hold_duration += duration
        elif self._state == 'DECAYING':
            self._event.decay_duration += duration

    def _set_state(self, state: str, now: float) -> None:
        if self._state == state:
            return
        previous = self._state
        if self._event is not None:
            self._accumulate_state_time(now)
        self._state = state
        self._state_entered_at = now
        if (
            self._event is not None
            and state in {'HOLDING', 'DECAYING'}
        ):
            self._transition_events.append(
                f'transition:event_id={self._event.event_id};'
                f'from={previous};to={state}'
            )

    def _brake(self, reason: str) -> AdapterDecision:
        if self._state not in {'COOLDOWN'}:
            self._set_state('IDLE', self._last_step_time or 0.0)
        return AdapterDecision(
            True,
            'brake',
            reason,
            assist_state=self._state,
            event_count=self._event_count,
            last_exit_reason=self._last_exit_reason,
        )

    def _silence(self, reason: str) -> AdapterDecision:
        if self._event is None and self._state != 'COOLDOWN':
            self._set_state('IDLE', self._last_step_time or 0.0)
        return AdapterDecision(
            False,
            'silence',
            reason,
            assist_state=self._state,
            event_count=self._event_count,
            last_exit_reason=self._last_exit_reason,
        )
