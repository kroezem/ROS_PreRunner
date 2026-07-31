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

"""Pure feedforward-plus-PI conversion for Runner."""

from dataclasses import dataclass
import math
from typing import Optional, Sequence


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
    """Interpolate throttle and saturate at either end of the table."""
    validate_table(throttle_breakpoints, speed_breakpoints)
    if speed <= speed_breakpoints[0]:
        return float(throttle_breakpoints[0]), speed < speed_breakpoints[0]
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
    maximum_commanded_speed: float = 0.60
    proportional_gain: float = 0.30
    integral_gain: float = 0.06
    stall_integral_gain: float = 0.30
    stall_integral_gain_activation_ratio: float = 0.40
    stall_integral_gain_hysteresis: float = 0.10
    integrator_min: float = -0.25
    integrator_max: float = 0.16
    output_min: float = 0.0
    output_max: float = 0.70
    breakaway_integrator_preload: float = 0.04
    encoder_metres_per_edge: float = 0.010282
    wheelspin_speed_ratio: float = 1.50
    wheelspin_min_speed_excess: float = 0.10
    wheelspin_qualification_sec: float = 0.20
    motion_signal_timeout_sec: float = 0.25
    encoder_state_timeout_sec: float = 0.25
    cmd_vel_nav_timeout: float = 0.25
    active_mode_timeout_sec: float = 0.20
    preemption_integrator_decay_rate: float = 0.0625
    publication_rate: float = 20.0

    def __post_init__(self) -> None:
        """Reject unsafe or internally inconsistent parameter sets."""
        validate_table(
            self.throttle_breakpoints,
            self.speed_breakpoints,
        )
        values = tuple(
            value
            for name, value in self.__dict__.items()
            if name not in {'throttle_breakpoints', 'speed_breakpoints'}
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('all scalar parameters must be finite')
        positive = {
            'wheelbase': self.wheelbase,
            'max_steering_angle': self.max_steering_angle,
            'steering_min_speed': self.steering_min_speed,
            'minimum_moving_speed': self.minimum_moving_speed,
            'maximum_commanded_speed': self.maximum_commanded_speed,
            'proportional_gain': self.proportional_gain,
            'integral_gain': self.integral_gain,
            'stall_integral_gain': self.stall_integral_gain,
            'encoder_metres_per_edge': self.encoder_metres_per_edge,
            'wheelspin_speed_ratio': self.wheelspin_speed_ratio,
            'wheelspin_qualification_sec':
                self.wheelspin_qualification_sec,
            'motion_signal_timeout_sec': self.motion_signal_timeout_sec,
            'encoder_state_timeout_sec': self.encoder_state_timeout_sec,
            'cmd_vel_nav_timeout': self.cmd_vel_nav_timeout,
            'active_mode_timeout_sec': self.active_mode_timeout_sec,
            'preemption_integrator_decay_rate':
                self.preemption_integrator_decay_rate,
            'publication_rate': self.publication_rate,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        if not 0.0 < self.max_steering_angle < math.pi / 2.0:
            raise ValueError(
                'max_steering_angle must be between zero and pi/2'
            )
        if not 0.0 < self.floor_promotion_min_ratio <= 1.0:
            raise ValueError(
                'floor_promotion_min_ratio must be within (0, 1]'
            )
        if self.stall_integral_gain <= self.integral_gain:
            raise ValueError(
                'stall_integral_gain must exceed integral_gain'
            )
        if not (
            0.0 < self.stall_integral_gain_activation_ratio < 1.0
        ):
            raise ValueError(
                'stall_integral_gain_activation_ratio must be within (0, 1)'
            )
        if not (
            0.0 < self.stall_integral_gain_hysteresis
            and self.stall_integral_gain_activation_ratio
            + self.stall_integral_gain_hysteresis
            <= 1.0
        ):
            raise ValueError(
                'stall_integral_gain_hysteresis must be positive and its '
                'exit ratio must not exceed one'
            )
        if self.maximum_commanded_speed < self.minimum_moving_speed:
            raise ValueError(
                'maximum_commanded_speed must reach minimum_moving_speed'
            )
        if self.integrator_min >= self.integrator_max:
            raise ValueError('integrator_min must be less than integrator_max')
        if self.output_min != 0.0:
            raise ValueError('output_min must be zero for forward-only output')
        if self.output_max > 1.0 or self.output_max <= 0.0:
            raise ValueError('output_max must be within (0, 1]')
        if self.output_max < max(self.throttle_breakpoints):
            raise ValueError(
                'output_max must reach the maximum feedforward throttle'
            )
        if not (
            self.integrator_min
            <= self.breakaway_integrator_preload
            <= self.integrator_max
        ):
            raise ValueError(
                'breakaway_integrator_preload must fit integrator bounds'
            )
        if self.wheelspin_speed_ratio <= 1.0:
            raise ValueError('wheelspin_speed_ratio must exceed one')
        if self.wheelspin_min_speed_excess < 0.0:
            raise ValueError(
                'wheelspin_min_speed_excess must be nonnegative'
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
    commanded_speed: float = 0.0
    effective_speed: float = 0.0
    measured_speed: float = 0.0
    speed_error: float = 0.0
    feedforward_throttle: float = 0.0
    proportional_term: float = 0.0
    integrator_state: float = 0.0
    pi_term: float = 0.0
    integrator_enabled: bool = False
    saturation_state: str = 'none'
    wheelspin_guard: bool = False
    final_throttle: float = 0.0
    normalized_steering: float = 0.0
    steering_saturated: bool = False
    integral_gain: float = 0.0
    stall_integral_gain_active: bool = False
    active_mode_received: bool = False
    active_mode_fresh: bool = False
    active_mode: str = ''
    preempted: bool = False
    integral_decay_active: bool = False

    def diagnostic_text(self) -> str:
        """Serialize a stable, compact diagnostic record."""
        return (
            f'mode={self.mode};reason={self.reason};'
            f'commanded_speed={self.commanded_speed:.9f};'
            f'effective_speed={self.effective_speed:.9f};'
            f'measured_speed={self.measured_speed:.9f};'
            f'speed_error={self.speed_error:.9f};'
            f'feedforward_throttle={self.feedforward_throttle:.9f};'
            f'proportional_term={self.proportional_term:.9f};'
            f'integrator_state={self.integrator_state:.9f};'
            f'pi_term={self.pi_term:.9f};'
            f'integrator_enabled={str(self.integrator_enabled).lower()};'
            f'saturation_state={self.saturation_state};'
            f'wheelspin_guard={str(self.wheelspin_guard).lower()};'
            f'integral_gain={self.integral_gain:.9f};'
            f'stall_integral_gain_active='
            f'{str(self.stall_integral_gain_active).lower()};'
            f'active_mode_received='
            f'{str(self.active_mode_received).lower()};'
            f'active_mode_fresh={str(self.active_mode_fresh).lower()};'
            f'active_mode={self.active_mode};'
            f'preempted={str(self.preempted).lower()};'
            f'integral_decay_active='
            f'{str(self.integral_decay_active).lower()};'
            f'final_throttle={self.final_throttle:.9f};'
            f'normalized_steering={self.normalized_steering:.9f};'
            f'steering_saturated='
            f'{str(self.steering_saturated).lower()}'
        )


class DriveAdapter:
    """Convert fresh Nav2 commands with bounded feedforward-plus-PI control."""

    def __init__(self, config: AdapterConfig):
        self.config = config
        self._command: Optional[tuple[float, float]] = None
        self._command_time: Optional[float] = None
        self._motion_speed = 0.0
        self._motion_time: Optional[float] = None
        self._encoder_stationary: Optional[bool] = None
        self._encoder_edge_rate = 0.0
        self._encoder_time: Optional[float] = None
        self._stationary_transition_pending = False
        self._integrator = 0.0
        self._last_step_time: Optional[float] = None
        self._wheelspin_since: Optional[float] = None
        self._stall_integral_gain_active = False
        self._active_mode: Optional[str] = None
        self._active_mode_time: Optional[float] = None

    @property
    def latest_command(self) -> Optional[tuple[float, float]]:
        """Return the latest raw command for logging."""
        return self._command

    @property
    def integrator_state(self) -> float:
        """Return the current integral contribution."""
        return self._integrator

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
        """Store EKF body-forward velocity for the wheelspin guard."""
        self._motion_speed = forward_speed
        self._motion_time = now

    def update_active_mode(self, mode: str, now: float) -> None:
        """Store the latest teleop arbitration state and receive time."""
        self._active_mode = mode
        self._active_mode_time = now

    def active_mode_is_fresh(self, now: float) -> bool:
        """Return whether teleop state was received within its timeout."""
        return (
            self._active_mode_time is not None
            and now - self._active_mode_time
            <= self.config.active_mode_timeout_sec
        )

    def is_confirmed_preempted(self, now: float) -> bool:
        """Treat only a fresh non-suppression mode as preemption."""
        return (
            self.active_mode_is_fresh(now)
            and self._active_mode != 'teleop_suppress'
        )

    def update_encoder(
        self,
        stationary: bool,
        edge_rate: float,
        pending_direction: int,
        now: float,
    ) -> None:
        """Store encoder motion and freshness."""
        del pending_direction
        stationary = bool(stationary)
        if stationary and self._encoder_stationary is not True:
            self._stationary_transition_pending = True
        self._encoder_stationary = stationary
        self._encoder_edge_rate = edge_rate
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

    def shutdown(self, now: float) -> None:
        """Clear controller state without publishing another command."""
        del now
        self._reset_controller()
        self._command = None
        self._command_time = None

    def step(self, now: float) -> AdapterDecision:
        """Produce one deterministic timer-cycle decision."""
        active_mode_received = self._active_mode_time is not None
        active_mode_fresh = self.active_mode_is_fresh(now)
        active_mode = self._active_mode or ''
        preempted = self.is_confirmed_preempted(now)
        preemption_fields = {
            'active_mode_received': active_mode_received,
            'active_mode_fresh': active_mode_fresh,
            'active_mode': active_mode,
            'preempted': preempted,
        }
        dt = 0.0
        if self._last_step_time is not None:
            dt = max(0.0, now - self._last_step_time)
        self._last_step_time = now

        if self._command is None or self._command_time is None:
            self._reset_controller()
            return self._silence('no_command', **preemption_fields)
        if now - self._command_time > self.config.cmd_vel_nav_timeout:
            self._reset_controller()
            return self._silence('stale_command', **preemption_fields)

        speed, yaw_rate = self._command
        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            return self._brake('nonfinite_input', **preemption_fields)
        if speed < 0.0:
            return self._brake('negative_speed', **preemption_fields)
        if speed == 0.0:
            return self._brake('explicit_stop', **preemption_fields)
        if speed < self.config.steering_min_speed:
            return self._brake(
                'below_steering_min_speed', **preemption_fields
            )
        if speed < self.config.promotion_threshold:
            return self._brake(
                'below_promotion_threshold', **preemption_fields
            )

        requested_curvature = yaw_rate / speed
        steering_saturated = (
            abs(requested_curvature) > self.config.maximum_curvature
        )

        commanded_speed = min(speed, self.config.maximum_commanded_speed)
        effective_speed = max(
            commanded_speed,
            self.config.minimum_moving_speed,
        )
        feedforward, _table_saturated = lookup_throttle(
            effective_speed,
            self.config.throttle_breakpoints,
            self.config.speed_breakpoints,
        )
        delta = math.atan(self.config.wheelbase * requested_curvature)
        steering = max(
            -1.0,
            min(1.0, delta / self.config.max_steering_angle),
        )

        measured_speed = 0.0
        if math.isfinite(self._encoder_edge_rate):
            measured_speed = (
                abs(self._encoder_edge_rate)
                * self.config.encoder_metres_per_edge
            )
        speed_error = commanded_speed - measured_speed
        below_floor = speed < self.config.minimum_moving_speed
        feedback_fresh = not self.encoder_is_stale(now)
        wheelspin_guard = self._wheelspin_guard(now, measured_speed)
        base_integration_enabled = not below_floor and feedback_fresh
        integral_gain = self._select_integral_gain(
            measured_speed,
            commanded_speed,
            base_integration_enabled,
        )
        accumulation_enabled = (
            base_integration_enabled
            and not wheelspin_guard
            and not preempted
        )
        integral_decay_active = False

        if below_floor:
            self._integrator = 0.0
            proportional = 0.0
            final = feedforward
            saturation = 'none'
        else:
            if (
                base_integration_enabled
                and not preempted
                and self._stationary_transition_pending
                and self._encoder_stationary
            ):
                self._integrator = max(
                    self._integrator,
                    self.config.breakaway_integrator_preload,
                )
                self._stationary_transition_pending = False

            proportional = (
                self.config.proportional_gain * speed_error
                if feedback_fresh else 0.0
            )
            candidate = self._integrator
            may_integrate = accumulation_enabled and dt > 0.0
            if preempted and dt > 0.0 and self._integrator != 0.0:
                decayed = self._decay_toward_zero(
                    self._integrator,
                    self.config.preemption_integrator_decay_rate * dt,
                )
                integral_decay_active = decayed != self._integrator
                self._integrator = decayed
                candidate = decayed
            if may_integrate:
                candidate = max(
                    self.config.integrator_min,
                    min(
                        self.config.integrator_max,
                        self._integrator
                        + integral_gain * speed_error * dt,
                    ),
                )
            candidate_raw = feedforward + proportional + candidate
            candidate_saturation = self._saturation(candidate_raw)
            if may_integrate and candidate_saturation == 'none':
                self._integrator = candidate
            raw_output = feedforward + proportional + self._integrator
            saturation = self._saturation(raw_output)
            final = max(
                self.config.output_min,
                min(self.config.output_max, raw_output),
            )

        reason = 'closed_loop'
        if speed > self.config.maximum_commanded_speed:
            reason = 'maximum_speed_clamped'
        elif below_floor:
            reason = 'floor_promoted_feedforward'
        elif not feedback_fresh:
            reason = 'encoder_stale_feedforward'
        pi_term = proportional + self._integrator
        return AdapterDecision(
            True,
            'forward',
            reason,
            speed,
            effective_speed,
            measured_speed,
            speed_error,
            feedforward,
            proportional,
            self._integrator,
            pi_term,
            accumulation_enabled,
            saturation,
            wheelspin_guard,
            final,
            steering,
            steering_saturated,
            integral_gain,
            self._stall_integral_gain_active,
            active_mode_received,
            active_mode_fresh,
            active_mode,
            preempted,
            integral_decay_active,
        )

    @staticmethod
    def _decay_toward_zero(value: float, amount: float) -> float:
        """Reduce magnitude by a bounded time-scaled amount without crossing."""
        if value > 0.0:
            return max(0.0, value - amount)
        if value < 0.0:
            return min(0.0, value + amount)
        return 0.0

    def _select_integral_gain(
        self,
        measured_speed: float,
        commanded_speed: float,
        integrator_enabled: bool,
    ) -> float:
        if not integrator_enabled:
            self._stall_integral_gain_active = False
            return self.config.integral_gain

        speed_ratio = measured_speed / commanded_speed
        low_activation = self.config.stall_integral_gain_activation_ratio
        hysteresis = self.config.stall_integral_gain_hysteresis
        low_release = low_activation + hysteresis
        high_activation = 2.0 - low_activation
        high_release = high_activation - hysteresis
        if self._stall_integral_gain_active:
            if low_release <= speed_ratio <= high_release:
                self._stall_integral_gain_active = False
        elif (
            speed_ratio < low_activation
            or speed_ratio > high_activation
        ):
            self._stall_integral_gain_active = True

        if self._stall_integral_gain_active:
            return self.config.stall_integral_gain
        return self.config.integral_gain

    def _wheelspin_guard(self, now: float, encoder_speed: float) -> bool:
        sensors_fresh = (
            not self.encoder_is_stale(now)
            and not self.motion_is_stale(now)
        )
        ekf_speed = abs(self._motion_speed)
        condition = (
            sensors_fresh
            and encoder_speed
            > self.config.wheelspin_speed_ratio * ekf_speed
            and encoder_speed - ekf_speed
            > self.config.wheelspin_min_speed_excess
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

    def _saturation(self, output: float) -> str:
        if output > self.config.output_max:
            return 'upper'
        if output < self.config.output_min:
            return 'lower'
        return 'none'

    def _reset_controller(self) -> None:
        self._integrator = 0.0
        self._wheelspin_since = None
        self._stall_integral_gain_active = False

    def _brake(self, reason: str, **fields) -> AdapterDecision:
        self._reset_controller()
        return AdapterDecision(True, 'brake', reason, **fields)

    def _silence(self, reason: str, **fields) -> AdapterDecision:
        return AdapterDecision(False, 'silence', reason, **fields)
