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

"""Pure conversion and breakaway state for the Runner drive adapter."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


CURVATURE_REL_TOLERANCE = 1e-12
CURVATURE_ABS_TOLERANCE = 1e-12


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
            throttle = lower_throttle + fraction * (
                upper_throttle - lower_throttle
            )
            return float(throttle), False
    raise AssertionError('validated lookup table did not contain speed')


@dataclass(frozen=True)
class AdapterConfig:
    """Validated drive-adapter configuration."""

    wheelbase: float = 0.178
    max_steering_angle: float = 0.3054
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
    breakaway_throttle: float = 0.380
    breakaway_timeout: float = 0.75
    motion_confirm_edge_rate: float = 1.0
    cmd_vel_nav_timeout: float = 0.25
    encoder_state_timeout: float = 0.25
    publication_rate: float = 20.0

    def __post_init__(self) -> None:
        """Reject unsafe or internally inconsistent parameter sets."""
        scalar_values = (
            self.wheelbase,
            self.max_steering_angle,
            self.steering_min_speed,
            self.minimum_moving_speed,
            self.floor_promotion_min_ratio,
            self.breakaway_throttle,
            self.breakaway_timeout,
            self.motion_confirm_edge_rate,
            self.cmd_vel_nav_timeout,
            self.encoder_state_timeout,
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
        if not 0.0 <= self.breakaway_throttle <= 1.0:
            raise ValueError('breakaway_throttle must be within [0, 1]')
        if self.breakaway_timeout <= 0.0:
            raise ValueError('breakaway_timeout must be greater than zero')
        if self.motion_confirm_edge_rate <= 0.0:
            raise ValueError(
                'motion_confirm_edge_rate must be greater than zero'
            )
        if self.cmd_vel_nav_timeout <= 0.0:
            raise ValueError('cmd_vel_nav_timeout must be greater than zero')
        if self.encoder_state_timeout <= 0.0:
            raise ValueError(
                'encoder_state_timeout must be greater than zero'
            )
        if self.publication_rate <= 0.0:
            raise ValueError('publication_rate must be greater than zero')

        validate_table(
            self.throttle_breakpoints,
            self.speed_breakpoints,
        )
        if not (
            self.speed_breakpoints[0]
            <= self.minimum_moving_speed
            <= self.speed_breakpoints[-1]
        ):
            raise ValueError(
                'minimum_moving_speed must fall within the speed table'
            )
        if self.promotion_threshold < 0.0:
            raise ValueError('promotion threshold must not be below zero')

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
    kick_active: bool = False

    def diagnostic_text(self) -> str:
        """Serialize a stable, compact diagnostic record."""
        active = 'true' if self.kick_active else 'false'
        return (
            f'mode={self.mode};reason={self.reason};'
            f'kick_active={active};'
            f'effective_speed={self.effective_speed:.9f};'
            f'feedforward_throttle={self.feedforward_throttle:.9f};'
            f'final_throttle={self.final_throttle:.9f};'
            f'normalized_steering={self.normalized_steering:.9f}'
        )


class DriveAdapter:
    """Convert fresh Nav2 commands and manage one-shot breakaway kicks."""

    def __init__(self, config: AdapterConfig):
        self.config = config
        self._command: Optional[tuple[float, float]] = None
        self._command_time: Optional[float] = None
        self._encoder_stationary: Optional[bool] = None
        self._encoder_edge_rate = 0.0
        self._encoder_time: Optional[float] = None
        self._kick_armed = True
        self._kick_active = False
        self._kick_started_at: Optional[float] = None
        self._events: list[str] = []

    def update_command(
        self,
        linear_speed: float,
        yaw_rate: float,
        now: float,
    ) -> None:
        """Store the latest Nav2 command and its monotonic receive time."""
        self._command = (linear_speed, yaw_rate)
        self._command_time = now

    def update_encoder(
        self,
        stationary: bool,
        edge_rate: float,
        now: float,
    ) -> None:
        """Update encoder freshness and end a kick on confirmed movement."""
        previous_stationary = self._encoder_stationary
        self._encoder_stationary = bool(stationary)
        self._encoder_edge_rate = edge_rate
        self._encoder_time = now

        transition_to_moving = (
            previous_stationary is True and not stationary
        )
        rate_confirms_motion = (
            not stationary
            and math.isfinite(edge_rate)
            and edge_rate >= self.config.motion_confirm_edge_rate
        )
        if transition_to_moving or rate_confirms_motion:
            if self._kick_active:
                self._end_kick('motion_confirmed')
            self._kick_armed = True

    def take_events(self) -> list[str]:
        """Return and clear kick transition events."""
        events = self._events
        self._events = []
        return events

    @property
    def latest_command(self) -> Optional[tuple[float, float]]:
        """Return the latest raw command for logging only."""
        return self._command

    def encoder_is_stale(self, now: float) -> bool:
        """Return whether encoder state is unavailable or too old."""
        return (
            self._encoder_time is None
            or now - self._encoder_time
            > self.config.encoder_state_timeout
        )

    def step(self, now: float) -> AdapterDecision:
        """Produce one deterministic timer-cycle decision."""
        if self._command is None or self._command_time is None:
            return AdapterDecision(False, 'silence', 'no_command')
        if now - self._command_time > self.config.cmd_vel_nav_timeout:
            if self._kick_active:
                self._end_kick('stale_command')
                self._kick_armed = False
            return AdapterDecision(False, 'silence', 'stale_command')

        speed, yaw_rate = self._command
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            return self._brake('nonfinite_input')
        if speed < 0.0:
            return self._brake('negative_speed')
        if speed == 0.0:
            return self._brake('explicit_stop')
        if speed < self.config.steering_min_speed:
            return self._brake('below_steering_min_speed')
        if speed < self.config.promotion_threshold:
            return self._brake('below_promotion_threshold')

        requested_curvature = yaw_rate / speed
        maximum_curvature = self.config.maximum_curvature
        tolerance = max(
            CURVATURE_ABS_TOLERANCE,
            maximum_curvature * CURVATURE_REL_TOLERANCE,
        )
        if abs(requested_curvature) > maximum_curvature + tolerance:
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
        normalized_steering = delta / self.config.max_steering_angle
        normalized_steering = max(
            -1.0,
            min(1.0, normalized_steering),
        )

        encoder_fresh = not self.encoder_is_stale(now)
        if self._kick_active and not encoder_fresh:
            self._end_kick('encoder_stale')
            self._kick_armed = False
        if (
            not self._kick_active
            and self._kick_armed
            and encoder_fresh
            and self._encoder_stationary
        ):
            self._kick_active = True
            self._kick_armed = False
            self._kick_started_at = now
            self._events.append('started')
        if (
            self._kick_active
            and self._kick_started_at is not None
            and now - self._kick_started_at >= self.config.breakaway_timeout
        ):
            self._end_kick('timeout')

        final_throttle = feedforward
        mode = 'forward'
        reason = 'above_table_clamped' if clamped else 'feedforward'
        if self._kick_active:
            final_throttle = max(
                feedforward,
                self.config.breakaway_throttle,
            )
            mode = 'breakaway_kick'
            reason = 'stationary_start'
        return AdapterDecision(
            True,
            mode,
            reason,
            effective_speed,
            feedforward,
            final_throttle,
            normalized_steering,
            self._kick_active,
        )

    def _brake(self, reason: str) -> AdapterDecision:
        if self._kick_active:
            self._end_kick('stop_command')
        self._kick_armed = True
        return AdapterDecision(True, 'brake', reason)

    def _end_kick(self, reason: str) -> None:
        self._kick_active = False
        self._kick_started_at = None
        self._events.append(f'ended:{reason}')
