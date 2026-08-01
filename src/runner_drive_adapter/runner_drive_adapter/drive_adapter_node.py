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

"""
ROS interface for Runner closed-loop speed conversion.

Measured yaw rate is sourced from
``/odometry/filtered.twist.twist.angular.z``. Steering control remains
open-loop; this odometry signal is its first measured response diagnostic.
"""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_drive_adapter.drive_adapter import AdapterConfig, DriveAdapter
from runner_interfaces.msg import AdapterState, EncoderState
from std_msgs.msg import String


WARNING_THROTTLE_SECONDS = 5.0


class WarningThrottle:
    """Gate repeated warnings by stable key and monotonic time."""

    def __init__(self, interval: float = WARNING_THROTTLE_SECONDS):
        self._interval = interval
        self._last: dict[str, float] = {}

    def allows(self, key: str, now: float) -> bool:
        """Return true for the first event and once per interval."""
        previous = self._last.get(key)
        if previous is not None and now - previous < self._interval:
            return False
        self._last[key] = now
        return True


class DriveAdapterNode(Node):
    """ROS wrapper around the deterministic drive-adapter core."""

    def __init__(self):
        super().__init__('drive_adapter')
        defaults = AdapterConfig()
        names = (
            'wheelbase',
            'max_steering_angle',
            'steering_min_speed',
            'feedforward_speed_per_command',
            'feedforward_speed_intercept',
            'minimum_moving_speed',
            'floor_promotion_min_ratio',
            'maximum_commanded_speed',
            'proportional_gain',
            'integral_gain',
            'stall_integral_gain',
            'stall_integral_gain_activation_ratio',
            'stall_integral_gain_hysteresis',
            'integrator_min',
            'integrator_max',
            'output_min',
            'output_max',
            'breakaway_integrator_preload',
            'encoder_metres_per_edge',
            'wheelspin_speed_ratio',
            'wheelspin_min_speed_excess',
            'wheelspin_qualification_sec',
            'motion_signal_timeout_sec',
            'encoder_state_timeout_sec',
            'cmd_vel_nav_timeout',
            'active_mode_timeout_sec',
            'preemption_integrator_decay_rate',
            'publication_rate',
        )
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        values = {name: self.get_parameter(name).value for name in names}
        try:
            config = AdapterConfig(**values)
        except (TypeError, ValueError) as error:
            message = f'Invalid drive-adapter parameters: {error}'
            self.get_logger().error(message)
            raise ValueError(message) from None

        self.config = config
        self.adapter = DriveAdapter(config)
        self._warnings = WarningThrottle()
        self._shutdown_recorded = False
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self._state_pub = self.create_publisher(
            String, '/drive_adapter/state', 10
        )
        self._typed_state_pub = self.create_publisher(
            AdapterState, '/drive_adapter/state_typed', 10
        )
        self._measured_yaw_rate = 0.0
        self.create_subscription(
            Twist, '/cmd_vel_nav', self._on_command, 10
        )
        self.create_subscription(
            Odometry, '/odometry/filtered', self._on_motion, 10
        )
        self.create_subscription(
            EncoderState,
            '/wheel/encoder_state',
            self._on_encoder,
            10,
        )
        self.create_subscription(
            String, '/teleop/active_mode', self._on_active_mode, 10
        )
        self.create_timer(1.0 / config.publication_rate, self._publish)
        self._log_startup()

    def _on_command(self, message: Twist) -> None:
        self.adapter.update_command(
            message.linear.x,
            message.angular.z,
            time.monotonic(),
        )

    def _on_motion(self, message: Odometry) -> None:
        self._measured_yaw_rate = message.twist.twist.angular.z
        self.adapter.update_motion(
            message.twist.twist.linear.x,
            time.monotonic(),
        )

    def _on_encoder(self, message: EncoderState) -> None:
        self.adapter.update_encoder(
            message.stationary,
            message.edge_rate,
            message.pending_direction,
            time.monotonic(),
        )

    def _on_active_mode(self, message: String) -> None:
        self.adapter.update_active_mode(message.data, time.monotonic())

    def _publish(self) -> None:
        now = time.monotonic()
        decision = self.adapter.step(now)
        self._state_pub.publish(String(data=decision.diagnostic_text()))
        self._typed_state_pub.publish(self._typed_state(decision))
        self._warn_for_decision(decision, now)
        if not decision.publish_command:
            return
        output = Twist()
        output.linear.x = decision.final_throttle
        output.angular.z = decision.normalized_steering
        self._cmd_pub.publish(output)

    def _typed_state(self, decision) -> AdapterState:
        """Build the plot-friendly view of one adapter decision."""
        command = self.adapter.latest_command
        commanded_yaw_rate = command[1] if command is not None else 0.0
        curvature = 0.0
        if (
            command is not None
            and math.isfinite(command[0])
            and math.isfinite(commanded_yaw_rate)
            and command[0] != 0.0
        ):
            curvature = commanded_yaw_rate / command[0]

        message = AdapterState()
        message.stamp = self.get_clock().now().to_msg()
        message.commanded_speed = decision.commanded_speed
        message.measured_speed = decision.measured_speed
        message.speed_error = decision.speed_error
        message.feedforward_throttle = decision.feedforward_throttle
        message.proportional_term = decision.proportional_term
        message.integrator_state = decision.integrator_state
        message.pi_term = decision.pi_term
        message.final_throttle = decision.final_throttle
        message.commanded_yaw_rate = commanded_yaw_rate
        message.measured_yaw_rate = self._measured_yaw_rate
        message.normalized_steering = decision.normalized_steering
        message.steering_curvature_requested = curvature
        message.steering_curvature_max = self.config.maximum_curvature
        message.integrator_enabled = decision.integrator_enabled
        message.steering_saturated = decision.steering_saturated
        message.wheelspin_guard = decision.wheelspin_guard
        message.mode = (
            f'{decision.mode};'
            f'active_mode_received='
            f'{str(decision.active_mode_received).lower()};'
            f'active_mode_fresh='
            f'{str(decision.active_mode_fresh).lower()};'
            f'active_mode={decision.active_mode};'
            f'preempted={str(decision.preempted).lower()};'
            f'integral_decay_active='
            f'{str(decision.integral_decay_active).lower()}'
        )
        return message

    def _warn_for_decision(self, decision, now: float) -> None:
        command = self.adapter.latest_command
        if command is None:
            return
        speed, yaw_rate = command
        if decision.reason == 'negative_speed':
            self._warning(
                'negative_speed',
                f'Rejecting negative speed {speed:.9f} m/s; zero brake',
                now,
            )
        elif decision.reason == 'nonfinite_input':
            self._error(
                'nonfinite_input',
                'Rejecting non-finite /cmd_vel_nav input; zero brake',
                now,
            )
        elif decision.reason == 'maximum_speed_clamped':
            self._warning(
                'maximum_speed_clamped',
                f'Requested speed {speed:.9f} m/s exceeds configured maximum '
                f'{self.config.maximum_commanded_speed:.9f} m/s; clamping',
                now,
            )
        if decision.steering_saturated:
            curvature = yaw_rate / speed
            self._warning(
                'steering_saturated',
                'Clamping infeasible steering request: '
                f'v={speed:.9f} m/s omega={yaw_rate:.9f} rad/s '
                f'curvature={curvature:.12f} 1/m '
                f'maximum_curvature='
                f'{self.config.maximum_curvature:.12f} 1/m',
                now,
            )
        if decision.reason == 'encoder_stale_feedforward':
            self._warning(
                'encoder_stale',
                'Encoder state is stale; using feedforward with frozen PI',
                now,
            )

    def _warning(self, key: str, message: str, now: float) -> None:
        if self._warnings.allows(key, now):
            self.get_logger().warning(message)

    def _error(self, key: str, message: str, now: float) -> None:
        if self._warnings.allows(key, now):
            self.get_logger().error(message)

    def _log_startup(self) -> None:
        c = self.config
        overspeed_enter_ratio = (
            2.0 - c.stall_integral_gain_activation_ratio
        )
        overspeed_exit_ratio = (
            overspeed_enter_ratio - c.stall_integral_gain_hysteresis
        )
        self.get_logger().info(
            'drive_adapter ready: '
            f'wheelbase={c.wheelbase:.6f} m, '
            f'max_steering_angle={c.max_steering_angle:.6f} rad, '
            f'maximum_curvature={c.maximum_curvature:.12f} 1/m'
        )
        self.get_logger().info(
            'Provisional MD13S linear inverse: '
            f'speed_per_command={c.feedforward_speed_per_command:.6f}, '
            f'speed_intercept={c.feedforward_speed_intercept:.6f} m/s, '
            f'maximum_commanded_speed={c.maximum_commanded_speed:.6f} m/s'
        )
        self.get_logger().info(
            'Speed control: primary_feedback=encoder_edge_rate, '
            f'kp={c.proportional_gain:.6f}, '
            f'ki={c.integral_gain:.6f}, '
            f'stall_ki={c.stall_integral_gain:.6f}, '
            f'stall_enter_ratio='
            f'{c.stall_integral_gain_activation_ratio:.6f}, '
            f'stall_exit_ratio='
            f'{c.stall_integral_gain_activation_ratio + c.stall_integral_gain_hysteresis:.6f}, '
            f'overspeed_enter_ratio='
            f'{overspeed_enter_ratio:.6f}, '
            f'overspeed_exit_ratio='
            f'{overspeed_exit_ratio:.6f}, '
            f'output_bounds=[{c.output_min:.6f}, {c.output_max:.6f}], '
            f'wheelspin_ratio={c.wheelspin_speed_ratio:.6f}, '
            f'wheelspin_qualification='
            f'{c.wheelspin_qualification_sec:.6f} s, '
            f'active_mode_timeout={c.active_mode_timeout_sec:.6f} s, '
            f'preemption_integrator_decay_rate='
            f'{c.preemption_integrator_decay_rate:.6f}/s'
        )

    def record_shutdown(self) -> None:
        """Record a bounded shutdown exit exactly once."""
        if self._shutdown_recorded:
            return
        self._shutdown_recorded = True
        self.adapter.shutdown(time.monotonic())

    def destroy_node(self):
        """End any active event without publishing another motor command."""
        self.record_shutdown()
        return super().destroy_node()


def main():
    """Run the drive adapter."""
    rclpy.init()
    node = None
    try:
        node = DriveAdapterNode()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except ValueError:
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
