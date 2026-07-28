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

"""ROS interface for Runner SI conversion and bounded stall assistance."""

import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_drive_adapter.drive_adapter import AdapterConfig, DriveAdapter
from runner_interfaces.msg import EncoderState
from std_msgs.msg import Float32, String, UInt32


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
            'minimum_moving_speed',
            'floor_promotion_min_ratio',
            'stall_assist_enabled',
            'under_speed_ratio',
            'under_speed_absolute_ceiling',
            'under_speed_qualification_sec',
            'command_stability_tolerance',
            'ramp_rate_per_sec',
            'boost_throttle_ceiling',
            'maximum_assist_duration_sec',
            'motion_confirm_speed',
            'motion_confirm_duration_sec',
            'motion_hold_duration_sec',
            'decay_rate_per_sec',
            'overspeed_margin',
            'wheelspin_edge_rate_threshold',
            'wheelspin_vehicle_speed_threshold',
            'wheelspin_qualification_sec',
            'cooldown_duration_sec',
            'motion_signal_timeout_sec',
            'encoder_state_timeout_sec',
            'cmd_vel_nav_timeout',
            'publication_rate',
        )
        for name in names:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter(
            'throttle_breakpoints',
            list(defaults.throttle_breakpoints),
        )
        self.declare_parameter(
            'speed_breakpoints',
            list(defaults.speed_breakpoints),
        )
        values = {name: self.get_parameter(name).value for name in names}
        values['throttle_breakpoints'] = tuple(
            self.get_parameter('throttle_breakpoints').value
        )
        values['speed_breakpoints'] = tuple(
            self.get_parameter('speed_breakpoints').value
        )
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
        self._assist_state_pub = self.create_publisher(
            String, '/stall_assist/state', 10
        )
        self._boost_pub = self.create_publisher(
            Float32, '/stall_assist/applied_boost', 10
        )
        self._event_count_pub = self.create_publisher(
            UInt32, '/stall_assist/event_count', 10
        )
        self._exit_reason_pub = self.create_publisher(
            String, '/stall_assist/last_exit_reason', 10
        )
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
        self.create_timer(1.0 / config.publication_rate, self._publish)
        self._log_startup()

    def _on_command(self, message: Twist) -> None:
        self.adapter.update_command(
            message.linear.x,
            message.angular.z,
            time.monotonic(),
        )

    def _on_motion(self, message: Odometry) -> None:
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

    def _publish(self) -> None:
        now = time.monotonic()
        decision = self.adapter.step(now)
        self._publish_diagnostics(decision)
        self._log_events()
        self._warn_for_decision(decision, now)
        if not decision.publish_command:
            return
        output = Twist()
        output.linear.x = decision.final_throttle
        output.angular.z = decision.normalized_steering
        self._cmd_pub.publish(output)

    def _publish_diagnostics(self, decision) -> None:
        self._state_pub.publish(String(data=decision.diagnostic_text()))
        self._assist_state_pub.publish(
            String(data=decision.assist_state)
        )
        self._boost_pub.publish(
            Float32(data=float(decision.applied_boost))
        )
        self._event_count_pub.publish(
            UInt32(data=decision.event_count)
        )
        self._exit_reason_pub.publish(
            String(data=decision.last_exit_reason)
        )

    def _log_events(self) -> None:
        for event in self.adapter.take_transition_events():
            if event.startswith('start:'):
                self.get_logger().info(
                    f'Stall assist started: {event.removeprefix("start:")}'
                )
            else:
                self.get_logger().info(
                    'Stall assist state: '
                    f'{event.removeprefix("transition:")}'
                )
        for summary in self.adapter.take_event_summaries():
            self.get_logger().info(
                f'Stall assist summary: {summary.diagnostic_text()}'
            )

    def _warn_for_decision(self, decision, now: float) -> None:
        command = self.adapter.latest_command
        if command is None:
            return
        speed, yaw_rate = command
        if decision.reason == 'negative_speed':
            self._warning(
                'negative_speed',
                f'Rejecting negative speed {speed:.9f} m/s; full brake',
                now,
            )
        elif decision.reason == 'nonfinite_input':
            self._error(
                'nonfinite_input',
                'Rejecting non-finite /cmd_vel_nav input; full brake',
                now,
            )
        elif decision.reason == 'above_table_clamped':
            self._warning(
                'above_table_clamped',
                f'Requested speed {speed:.9f} m/s exceeds table maximum '
                f'{self.config.speed_breakpoints[-1]:.9f} m/s; clamping',
                now,
            )
        elif decision.reason == 'steering_infeasible':
            curvature = yaw_rate / speed
            self._warning(
                'steering_infeasible',
                'Rejecting infeasible steering request: '
                f'v={speed:.9f} m/s omega={yaw_rate:.9f} rad/s '
                f'curvature={curvature:.12f} 1/m '
                f'maximum_curvature='
                f'{self.config.maximum_curvature:.12f} 1/m',
                now,
            )
        if (
            decision.mode == 'forward'
            and decision.assist_state in {'NORMAL', 'QUALIFYING'}
        ):
            if self.adapter.motion_is_stale(now):
                self._warning(
                    'motion_signal_stale',
                    'EKF motion signal is stale; stall assist suppressed',
                    now,
                )
            elif self.adapter.encoder_is_stale(now):
                self._warning(
                    'encoder_stale',
                    'Encoder state is stale; stall assist suppressed',
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
        self.get_logger().info(
            'drive_adapter ready: '
            f'wheelbase={c.wheelbase:.6f} m, '
            f'max_steering_angle={c.max_steering_angle:.6f} rad, '
            f'maximum_curvature={c.maximum_curvature:.12f} 1/m'
        )
        self.get_logger().info(
            'Drive table unchanged: '
            f'speeds={list(c.speed_breakpoints)} m/s, '
            f'throttles={list(c.throttle_breakpoints)}, '
            f'maximum_supported_speed={c.speed_breakpoints[-1]:.6f} m/s'
        )
        self.get_logger().info(
            'Stall assist: primary_motion_signal=ekf_velocity, '
            f'enabled={c.stall_assist_enabled}, '
            f'under_speed_ratio={c.under_speed_ratio:.6f}, '
            f'under_speed_absolute_ceiling='
            f'{c.under_speed_absolute_ceiling:.6f} m/s, '
            f'qualification={c.under_speed_qualification_sec:.6f} s, '
            f'ramp_rate={c.ramp_rate_per_sec:.6f}/s, '
            f'ceiling={c.boost_throttle_ceiling:.6f}, '
            f'maximum_duration={c.maximum_assist_duration_sec:.6f} s'
        )

    def record_shutdown(self) -> None:
        """Record a bounded shutdown exit exactly once."""
        if self._shutdown_recorded:
            return
        self._shutdown_recorded = True
        self.adapter.shutdown(time.monotonic())
        self._log_events()

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
