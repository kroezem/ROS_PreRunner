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

"""Bounded feedforward-plus-parallel-PI conversion for Runner."""

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional


MAXIMUM_OUTPUT_AUTHORITY = 0.14
MINIMUM_EXPECTED_FEEDFORWARD = 0.04
MAX_INTEGRATION_DT_SEC = 0.5
SELECTION_ABS_TOLERANCE = 1e-9


class IntegratorFreezeReason(IntEnum):
    """Typed reasons why one cycle did not update integral state."""

    NONE = 0
    GAIN_DISABLED = 1
    ZERO_COMMAND = 2
    FEEDBACK_STALE = 3
    WHEELSPIN = 4
    DIRECTION_UNAVAILABLE = 5
    DIRECTION_MISMATCH = 6
    ARBITRATION_UNAVAILABLE = 7
    OUTPUT_NOT_SELECTED = 8
    INVALID_DT = 9
    ANTI_WINDUP = 10
    NO_COMMAND = 11
    INVALID_COMMAND = 12


def linear_feedforward(
    speed: float,
    effort_per_speed: float,
    effort_intercept: float,
) -> float:
    """Return the characterized MD13S inverse in normalized effort."""
    return effort_per_speed * abs(speed) + effort_intercept


@dataclass(frozen=True)
class AdapterConfig:
    """Validated drive-adapter configuration."""

    wheelbase: float = 0.178
    max_steering_angle: float = 0.3614
    feedforward_effort_per_speed: float = 0.1188
    feedforward_effort_intercept: float = 0.0174
    minimum_moving_speed: float = 0.25
    maximum_commanded_speed: float = 0.60
    proportional_gain: float = 0.05
    integral_gain: float = 0.0
    integrator_bound: float = 0.005
    output_min: float = 0.0
    output_max: float = MAXIMUM_OUTPUT_AUTHORITY
    encoder_metres_per_edge: float = 0.010282
    wheelspin_speed_ratio: float = 1.50
    wheelspin_min_speed_excess: float = 0.10
    wheelspin_qualification_sec: float = 0.20
    motion_signal_timeout_sec: float = 0.25
    encoder_state_timeout_sec: float = 0.25
    cmd_vel_nav_timeout: float = 0.25
    active_mode_timeout_sec: float = 0.20
    publication_rate: float = 20.0

    def __post_init__(self) -> None:
        """Reject unsafe or internally inconsistent parameter sets."""
        values = tuple(self.__dict__.values())
        if not all(math.isfinite(value) for value in values):
            raise ValueError('all scalar parameters must be finite')
        positive = {
            'wheelbase': self.wheelbase,
            'max_steering_angle': self.max_steering_angle,
            'feedforward_effort_per_speed':
                self.feedforward_effort_per_speed,
            'minimum_moving_speed': self.minimum_moving_speed,
            'maximum_commanded_speed': self.maximum_commanded_speed,
            'proportional_gain': self.proportional_gain,
            'integrator_bound': self.integrator_bound,
            'encoder_metres_per_edge': self.encoder_metres_per_edge,
            'wheelspin_speed_ratio': self.wheelspin_speed_ratio,
            'wheelspin_qualification_sec':
                self.wheelspin_qualification_sec,
            'motion_signal_timeout_sec': self.motion_signal_timeout_sec,
            'encoder_state_timeout_sec': self.encoder_state_timeout_sec,
            'cmd_vel_nav_timeout': self.cmd_vel_nav_timeout,
            'active_mode_timeout_sec': self.active_mode_timeout_sec,
            'publication_rate': self.publication_rate,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        if not 0.0 < self.max_steering_angle < math.pi / 2.0:
            raise ValueError(
                'max_steering_angle must be between zero and pi/2'
            )
        if self.integral_gain < 0.0:
            raise ValueError('integral_gain must be nonnegative')
        if self.maximum_commanded_speed < self.minimum_moving_speed:
            raise ValueError(
                'maximum_commanded_speed must reach minimum_moving_speed'
            )
        if self.output_min != 0.0:
            raise ValueError('output_min must be zero for forward-only output')
        if (
            self.output_max > MAXIMUM_OUTPUT_AUTHORITY
            or self.output_max <= 0.0
        ):
            raise ValueError(
                'output_max must be within the safety authority range '
                '(0, 0.14]'
            )
        maximum_feedforward = linear_feedforward(
            self.maximum_commanded_speed,
            self.feedforward_effort_per_speed,
            self.feedforward_effort_intercept,
        )
        if maximum_feedforward < 0.0:
            raise ValueError(
                'feedforward must be nonnegative at maximum commanded speed'
            )
        if self.output_max < maximum_feedforward:
            raise ValueError(
                'output_max must reach the maximum linear feedforward'
            )
        if self.wheelspin_speed_ratio <= 1.0:
            raise ValueError('wheelspin_speed_ratio must exceed one')
        if self.wheelspin_min_speed_excess < 0.0:
            raise ValueError(
                'wheelspin_min_speed_excess must be nonnegative'
            )
        minimum_feedforward = linear_feedforward(
            self.minimum_moving_speed,
            self.feedforward_effort_per_speed,
            self.feedforward_effort_intercept,
        )
        if minimum_feedforward < 0.0:
            raise ValueError(
                'feedforward must be nonnegative at minimum moving speed'
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
    feedforward_floor_violation: bool = False
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
    integrator_freeze_reason: IntegratorFreezeReason = (
        IntegratorFreezeReason.GAIN_DISABLED
    )

    def diagnostic_text(self) -> str:
        """Serialize a stable, compact diagnostic record."""
        return (
            f'mode={self.mode};reason={self.reason};'
            f'commanded_speed={self.commanded_speed:.9f};'
            f'effective_speed={self.effective_speed:.9f};'
            f'measured_speed={self.measured_speed:.9f};'
            f'speed_error={self.speed_error:.9f};'
            f'feedforward_throttle={self.feedforward_throttle:.9f};'
            f'feedforward_floor_violation='
            f'{str(self.feedforward_floor_violation).lower()};'
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
            f'integrator_freeze_reason='
            f'{self.integrator_freeze_reason.name.lower()};'
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
        self._encoder_edge_rate = 0.0
        self._encoder_time: Optional[float] = None
        self._encoder_sample_time: Optional[float] = None
        self._previous_encoder_sample_time: Optional[float] = None
        self._pending_direction = 0
        self._wheelspin_since: Optional[float] = None
        self._active_mode: Optional[str] = None
        self._active_mode_time: Optional[float] = None
        self._adapter_output: Optional[tuple[float, float]] = None
        self._adapter_output_time: Optional[float] = None
        self._mux_output: Optional[tuple[float, float]] = None
        self._mux_output_time: Optional[float] = None
        self._integrator = 0.0

    @property
    def latest_command(self) -> Optional[tuple[float, float]]:
        """Return the latest raw command for logging."""
        return self._command

    @property
    def integrator_state(self) -> float:
        """Report the bounded integral contribution in normalized effort."""
        return self._integrator

    def set_integral_gain(self, integral_gain: float) -> None:
        """Apply a validated live Ki change and reset integral state."""
        if isinstance(integral_gain, bool) or not isinstance(
            integral_gain, float
        ):
            raise TypeError('integral_gain must be a double')
        self.config = AdapterConfig(
            **{**self.config.__dict__, 'integral_gain': integral_gain}
        )
        self._integrator = 0.0
        self._previous_encoder_sample_time = None

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
        sample_time: Optional[float] = None,
    ) -> None:
        """Store encoder motion and freshness."""
        del stationary
        self._encoder_edge_rate = edge_rate
        self._encoder_time = now
        self._encoder_sample_time = sample_time
        self._pending_direction = pending_direction

    def update_adapter_output(
        self,
        linear: float,
        angular: float,
        now: float,
    ) -> None:
        """Store the adapter command most recently offered to the mux."""
        self._adapter_output = (linear, angular)
        self._adapter_output_time = now

    def update_mux_output(
        self,
        linear: float,
        angular: float,
        now: float,
    ) -> None:
        """Observe the mux-owned output without affecting command publication."""
        self._mux_output = (linear, angular)
        self._mux_output_time = now

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
        """Stop accepting commands without publishing another command."""
        del now
        self._reset_transient_state()
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
        if self._command is None or self._command_time is None:
            self._reset_transient_state()
            return self._silence(
                'no_command',
                IntegratorFreezeReason.NO_COMMAND,
                **preemption_fields,
            )
        speed, yaw_rate = self._command
        if now - self._command_time > self.config.cmd_vel_nav_timeout:
            self._reset_transient_state()
            return self._silence(
                'stale_command',
                IntegratorFreezeReason.NO_COMMAND,
                commanded_speed=speed,
                **preemption_fields,
            )

        if not math.isfinite(speed) or not math.isfinite(yaw_rate):
            return self._brake(
                'nonfinite_input',
                IntegratorFreezeReason.INVALID_COMMAND,
                commanded_speed=speed,
                **preemption_fields,
            )
        if speed < 0.0:
            return self._brake(
                'negative_speed',
                IntegratorFreezeReason.INVALID_COMMAND,
                commanded_speed=speed,
                **preemption_fields,
            )
        if speed == 0.0:
            return self._brake(
                'explicit_stop',
                IntegratorFreezeReason.ZERO_COMMAND,
                commanded_speed=speed,
                **preemption_fields,
            )
        requested_curvature = yaw_rate / speed
        steering_saturated = (
            abs(requested_curvature) > self.config.maximum_curvature
        )

        effective_speed = min(
            self.config.maximum_commanded_speed,
            max(speed, self.config.minimum_moving_speed),
        )
        feedforward = linear_feedforward(
            effective_speed,
            self.config.feedforward_effort_per_speed,
            self.config.feedforward_effort_intercept,
        )
        feedforward_floor_violation = (
            feedforward < MINIMUM_EXPECTED_FEEDFORWARD
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
        speed_error = abs(effective_speed) - abs(measured_speed)
        below_floor = speed < self.config.minimum_moving_speed
        feedback_fresh = not self.encoder_is_stale(now)
        wheelspin_guard = self._wheelspin_guard(now, measured_speed)

        proportional = (
            self.config.proportional_gain * speed_error
            if feedback_fresh else 0.0
        )

        dt = self._integration_dt()
        freeze_reason = self._integrator_freeze_reason(
            now,
            speed,
            wheelspin_guard,
        )
        if freeze_reason == IntegratorFreezeReason.NONE:
            if dt is None:
                freeze_reason = IntegratorFreezeReason.INVALID_DT
            else:
                candidate = max(
                    -self.config.integrator_bound,
                    min(
                        self.config.integrator_bound,
                        self._integrator
                        + self.config.integral_gain * speed_error * dt,
                    ),
                )
                candidate_raw = feedforward + proportional + candidate
                candidate_saturation = self._saturation(candidate_raw)
                drives_farther = (
                    candidate_saturation == 'upper' and speed_error > 0.0
                ) or (
                    candidate_saturation == 'lower' and speed_error < 0.0
                )
                if drives_farther:
                    freeze_reason = IntegratorFreezeReason.ANTI_WINDUP
                else:
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
            reason = 'floor_promoted'
        elif not feedback_fresh:
            reason = 'encoder_stale_feedforward'
        pi_term = proportional + self._integrator
        return AdapterDecision(
            publish_command=True,
            mode='forward',
            reason=reason,
            commanded_speed=speed,
            effective_speed=effective_speed,
            measured_speed=measured_speed,
            speed_error=speed_error,
            feedforward_throttle=feedforward,
            feedforward_floor_violation=feedforward_floor_violation,
            proportional_term=proportional,
            integrator_state=self._integrator,
            pi_term=pi_term,
            integrator_enabled=(
                self.config.integral_gain != 0.0
                and freeze_reason == IntegratorFreezeReason.NONE
            ),
            saturation_state=saturation,
            wheelspin_guard=wheelspin_guard,
            final_throttle=final,
            normalized_steering=steering,
            steering_saturated=steering_saturated,
            integral_gain=self.config.integral_gain,
            stall_integral_gain_active=False,
            active_mode_received=active_mode_received,
            active_mode_fresh=active_mode_fresh,
            active_mode=active_mode,
            preempted=preempted,
            integral_decay_active=False,
            integrator_freeze_reason=freeze_reason,
        )

    def _integration_dt(self) -> Optional[float]:
        sample_time = self._encoder_sample_time
        previous = self._previous_encoder_sample_time
        self._previous_encoder_sample_time = sample_time
        if sample_time is None or previous is None:
            return None
        dt = sample_time - previous
        if not math.isfinite(dt) or dt <= 0.0 or dt > MAX_INTEGRATION_DT_SEC:
            return None
        return dt

    def _integrator_freeze_reason(
        self,
        now: float,
        commanded_speed: float,
        wheelspin_guard: bool,
    ) -> IntegratorFreezeReason:
        if self.config.integral_gain == 0.0:
            return IntegratorFreezeReason.GAIN_DISABLED
        if commanded_speed == 0.0:
            return IntegratorFreezeReason.ZERO_COMMAND
        integration_timeout = 3.0 / self.config.publication_rate
        if (
            self._encoder_time is None
            or not math.isfinite(self._encoder_edge_rate)
            or now - self._encoder_time > integration_timeout
        ):
            return IntegratorFreezeReason.FEEDBACK_STALE
        if wheelspin_guard:
            return IntegratorFreezeReason.WHEELSPIN
        commanded_direction = 1 if commanded_speed > 0.0 else -1
        if self._pending_direction not in (-1, 1):
            return IntegratorFreezeReason.DIRECTION_UNAVAILABLE
        if self._pending_direction != commanded_direction:
            return IntegratorFreezeReason.DIRECTION_MISMATCH
        if not self._arbitration_is_available(now):
            return IntegratorFreezeReason.ARBITRATION_UNAVAILABLE
        if not self._adapter_output_is_selected():
            return IntegratorFreezeReason.OUTPUT_NOT_SELECTED
        return IntegratorFreezeReason.NONE

    def _arbitration_is_available(self, now: float) -> bool:
        timeout = 3.0 / self.config.publication_rate
        return (
            self._adapter_output is not None
            and self._adapter_output_time is not None
            and self._mux_output is not None
            and self._mux_output_time is not None
            and now - self._adapter_output_time <= timeout
            and now - self._mux_output_time <= timeout
        )

    def _adapter_output_is_selected(self) -> bool:
        return all(
            math.isclose(
                adapter_value,
                mux_value,
                rel_tol=0.0,
                abs_tol=SELECTION_ABS_TOLERANCE,
            )
            for adapter_value, mux_value in zip(
                self._adapter_output,
                self._mux_output,
            )
        )

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

    def _reset_transient_state(self) -> None:
        self._wheelspin_since = None

    def _controller_fields(self, freeze_reason) -> dict:
        return {
            'integrator_state': self._integrator,
            'pi_term': self._integrator,
            'integrator_enabled': False,
            'integral_gain': self.config.integral_gain,
            'integrator_freeze_reason': freeze_reason,
        }

    def _brake(self, reason: str, freeze_reason, **fields) -> AdapterDecision:
        self._reset_transient_state()
        return AdapterDecision(
            True,
            'brake',
            reason,
            **self._controller_fields(freeze_reason),
            **fields,
        )

    def _silence(self, reason: str, freeze_reason, **fields) -> AdapterDecision:
        return AdapterDecision(
            False,
            'silence',
            reason,
            **self._controller_fields(freeze_reason),
            **fields,
        )
