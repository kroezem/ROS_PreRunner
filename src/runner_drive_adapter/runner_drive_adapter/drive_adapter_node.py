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
Adapt physical Nav2 Twist commands to normalized Runner drive commands.

``/cmd_vel_nav`` is ``geometry_msgs/msg/Twist`` with ``linear.x`` in m/s and
``angular.z`` in rad/s. ``/cmd_vel_auto`` is ``geometry_msgs/msg/Twist`` with
``linear.x`` as normalized throttle/brake and ``angular.z`` as normalized
steering. During Stage 1, this output is not connected to ``motor_node``.
"""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_drive_adapter.drive_adapter import (
    AdapterConfig,
    DriveAdapter,
)
from runner_interfaces.msg import EncoderState
from std_msgs.msg import String


WARNING_THROTTLE_SECONDS = 5.0


class WarningThrottle:
    """Gate repeated warnings by stable key and monotonic time."""

    def __init__(self, interval: float = WARNING_THROTTLE_SECONDS):
        self._interval = interval
        self._last: dict[str, float] = {}

    def allows(self, key: str, now: float) -> bool:
        """Return true for the first event and once per configured interval."""
        previous = self._last.get(key)
        if previous is not None and now - previous < self._interval:
            return False
        self._last[key] = now
        return True


class DriveAdapterNode(Node):
    """ROS interface for the deterministic drive-adapter core."""

    def __init__(self):
        super().__init__('drive_adapter')
        defaults = AdapterConfig()
        self.declare_parameter('wheelbase', defaults.wheelbase)
        self.declare_parameter(
            'max_steering_angle',
            defaults.max_steering_angle,
        )
        self.declare_parameter(
            'steering_min_speed',
            defaults.steering_min_speed,
        )
        self.declare_parameter(
            'throttle_breakpoints',
            list(defaults.throttle_breakpoints),
        )
        self.declare_parameter(
            'speed_breakpoints',
            list(defaults.speed_breakpoints),
        )
        self.declare_parameter(
            'minimum_moving_speed',
            defaults.minimum_moving_speed,
        )
        self.declare_parameter(
            'floor_promotion_min_ratio',
            defaults.floor_promotion_min_ratio,
        )
        self.declare_parameter(
            'breakaway_throttle',
            defaults.breakaway_throttle,
        )
        self.declare_parameter(
            'breakaway_timeout',
            defaults.breakaway_timeout,
        )
        self.declare_parameter(
            'motion_confirm_edge_rate',
            defaults.motion_confirm_edge_rate,
        )
        self.declare_parameter(
            'cmd_vel_nav_timeout',
            defaults.cmd_vel_nav_timeout,
        )
        self.declare_parameter(
            'encoder_state_timeout',
            defaults.encoder_state_timeout,
        )
        self.declare_parameter(
            'publication_rate',
            defaults.publication_rate,
        )

        try:
            config = AdapterConfig(
                wheelbase=self._parameter('wheelbase'),
                max_steering_angle=self._parameter(
                    'max_steering_angle'
                ),
                steering_min_speed=self._parameter(
                    'steering_min_speed'
                ),
                throttle_breakpoints=tuple(
                    self._parameter('throttle_breakpoints')
                ),
                speed_breakpoints=tuple(
                    self._parameter('speed_breakpoints')
                ),
                minimum_moving_speed=self._parameter(
                    'minimum_moving_speed'
                ),
                floor_promotion_min_ratio=self._parameter(
                    'floor_promotion_min_ratio'
                ),
                breakaway_throttle=self._parameter(
                    'breakaway_throttle'
                ),
                breakaway_timeout=self._parameter('breakaway_timeout'),
                motion_confirm_edge_rate=self._parameter(
                    'motion_confirm_edge_rate'
                ),
                cmd_vel_nav_timeout=self._parameter(
                    'cmd_vel_nav_timeout'
                ),
                encoder_state_timeout=self._parameter(
                    'encoder_state_timeout'
                ),
                publication_rate=self._parameter('publication_rate'),
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(
                f'Invalid drive-adapter parameters: {error}'
            )
            raise ValueError(
                f'Invalid drive-adapter parameters: {error}'
            ) from None

        self.config = config
        self.adapter = DriveAdapter(config)
        self._warnings = WarningThrottle()
        self._last_encoder_stale = False
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_auto', 10)
        self._state_pub = self.create_publisher(
            String,
            '/drive_adapter/state',
            10,
        )
        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self._on_command,
            10,
        )
        self.create_subscription(
            EncoderState,
            '/wheel/encoder_state',
            self._on_encoder,
            10,
        )
        self.create_timer(1.0 / config.publication_rate, self._publish)
        self._log_startup()

    def _parameter(self, name: str):
        return self.get_parameter(name).value

    def _on_command(self, message: Twist) -> None:
        self.adapter.update_command(
            message.linear.x,
            message.angular.z,
            time.monotonic(),
        )

    def _on_encoder(self, message: EncoderState) -> None:
        self.adapter.update_encoder(
            message.stationary,
            message.edge_rate,
            time.monotonic(),
        )

    def _publish(self) -> None:
        now = time.monotonic()
        decision = self.adapter.step(now)
        self._state_pub.publish(String(data=decision.diagnostic_text()))
        for event in self.adapter.take_events():
            if event == 'started':
                self.get_logger().info('Breakaway kick started')
            else:
                reason = event.split(':', 1)[1]
                self.get_logger().info(
                    f'Breakaway kick ended: {reason}'
                )

        self._warn_for_decision(decision, now)
        if not decision.publish_command:
            return
        output = Twist()
        output.linear.x = decision.final_throttle
        output.angular.z = decision.normalized_steering
        self._cmd_pub.publish(output)

    def _warn_for_decision(self, decision, now: float) -> None:
        reason = decision.reason
        command = self.adapter.latest_command
        if command is None:
            return
        speed, yaw_rate = command
        if reason == 'negative_speed':
            self._warning(
                reason,
                f'Rejecting negative speed {speed:.9f} m/s; full brake',
                now,
            )
        elif reason == 'nonfinite_input':
            self._error(
                reason,
                'Rejecting non-finite /cmd_vel_nav input; full brake',
                now,
            )
        elif reason == 'above_table_clamped':
            self._warning(
                reason,
                f'Requested speed {speed:.9f} m/s exceeds table maximum '
                f'{self.config.speed_breakpoints[-1]:.9f} m/s; clamping',
                now,
            )
        elif reason == 'steering_infeasible':
            curvature = yaw_rate / speed
            self._warning(
                reason,
                'Rejecting infeasible steering request: '
                f'v={speed:.9f} m/s omega={yaw_rate:.9f} rad/s '
                f'curvature={curvature:.12f} 1/m '
                f'maximum_curvature='
                f'{self.config.maximum_curvature:.12f} 1/m',
                now,
            )
        if (
            decision.publish_command
            and decision.mode == 'forward'
            and self.adapter.encoder_is_stale(now)
        ):
            self._warning(
                'encoder_stale',
                'Encoder state is stale; breakaway kick is suppressed',
                now,
            )

    def _warning(self, key: str, message: str, now: float) -> None:
        if self._warnings.allows(key, now):
            self.get_logger().warning(message)

    def _error(self, key: str, message: str, now: float) -> None:
        if self._warnings.allows(key, now):
            self.get_logger().error(message)

    def _log_startup(self) -> None:
        config = self.config
        self.get_logger().info(
            'drive_adapter ready: '
            f'wheelbase={config.wheelbase:.6f} m, '
            f'max_steering_angle={config.max_steering_angle:.6f} rad, '
            f'maximum_curvature={config.maximum_curvature:.12f} 1/m, '
            f'steering_min_speed={config.steering_min_speed:.6f} m/s'
        )
        self.get_logger().info(
            'Drive table: '
            f'speeds={list(config.speed_breakpoints)} m/s, '
            f'throttles={list(config.throttle_breakpoints)}, '
            f'maximum_supported_speed='
            f'{config.speed_breakpoints[-1]:.6f} m/s'
        )
        self.get_logger().info(
            'Floor and kick: '
            f'minimum_moving_speed={config.minimum_moving_speed:.6f} m/s, '
            f'floor_promotion_min_ratio='
            f'{config.floor_promotion_min_ratio:.6f}, '
            f'promotion_boundary={config.promotion_threshold:.6f} m/s, '
            f'breakaway_throttle={config.breakaway_throttle:.6f}, '
            f'breakaway_timeout={config.breakaway_timeout:.6f} s'
        )
        self.get_logger().info(
            'Timing and encoder: '
            f'publication_rate={config.publication_rate:.6f} Hz, '
            f'cmd_vel_nav_timeout={config.cmd_vel_nav_timeout:.6f} s, '
            f'encoder_state_timeout={config.encoder_state_timeout:.6f} s, '
            f'motion_confirm_edge_rate='
            f'{config.motion_confirm_edge_rate:.6f} edges/s'
        )


def main():
    """Run the Stage 1 drive adapter."""
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
