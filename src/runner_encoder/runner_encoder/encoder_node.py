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

import math
import time

import lgpio
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from runner_encoder.encoder_state import EncoderMeasurement, EncoderState
from runner_interfaces.msg import EncoderState as EncoderStateMsg
from std_msgs.msg import Int8


GLITCH_FILTER_US = 100
LARGE_VARIANCE = 1e6
STATIONARY_TIMEOUT_DESCRIPTION = (
    'Pulse-free interval used to declare the encoder stationary. '
    'Distance per edge is 0.010282 m/edge. At 0.2 seconds, one edge per '
    'timeout corresponds to 0.010282 / 0.2 = 0.05141 m/s. This timeout is '
    'therefore a low-speed threshold in disguise.'
)


def build_publications(
    measurement: EncoderMeasurement,
    stamp,
    metres_per_edge: float,
    window_sec: float,
):
    """Build odometry and diagnostics from one encoder state snapshot."""
    edge_rate = measurement.edge_rate(window_sec)
    signed_speed = (
        edge_rate * metres_per_edge * measurement.active_direction
    )

    odom_msg = Odometry()
    odom_msg.header.stamp = stamp
    odom_msg.header.frame_id = 'odom'
    odom_msg.child_frame_id = 'base_link'
    odom_msg.pose.pose.orientation.w = 1.0
    for index in (0, 7, 14, 21, 28, 35):
        odom_msg.pose.covariance[index] = LARGE_VARIANCE
    odom_msg.twist.twist.linear.x = signed_speed
    odom_msg.twist.covariance[0] = 0.01
    for index in (7, 14, 21, 28, 35):
        odom_msg.twist.covariance[index] = LARGE_VARIANCE

    state_msg = EncoderStateMsg()
    state_msg.stamp = stamp
    state_msg.edge_rate = edge_rate
    state_msg.stationary = measurement.stationary
    state_msg.active_direction = measurement.active_direction
    state_msg.pending_direction = measurement.pending_direction

    return odom_msg, state_msg


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        self.declare_parameter('gpio_pin', 22)
        self.declare_parameter('metres_per_edge', 0.010282)
        self.declare_parameter('window_ms', 50)
        self.declare_parameter(
            'stationary_timeout_sec',
            0.2,
            ParameterDescriptor(
                description=STATIONARY_TIMEOUT_DESCRIPTION,
                read_only=True,
            ),
        )
        self._gpio_pin = self.get_parameter('gpio_pin').value
        self._metres_per_edge = self.get_parameter('metres_per_edge').value
        self._window_ms = self.get_parameter('window_ms').value
        self._stationary_timeout_sec = self.get_parameter(
            'stationary_timeout_sec'
        ).value
        if (
            not math.isfinite(self._stationary_timeout_sec)
            or self._stationary_timeout_sec <= 0.0
        ):
            raise ValueError(
                'stationary_timeout_sec must be finite and greater than zero'
            )

        self._state = EncoderState(self._stationary_timeout_sec)
        self._chip = None
        self._gpio_callback = None

        # RP1 header bank (verify with gpiodetect: pinctrl-rp1); chip 0 is brcmstb.
        self._chip = lgpio.gpiochip_open(4)
        try:
            lgpio.gpio_claim_alert(
                self._chip, self._gpio_pin, lgpio.BOTH_EDGES
            )
            lgpio.gpio_set_debounce_micros(
                self._chip, self._gpio_pin, GLITCH_FILTER_US
            )
            self._gpio_callback = lgpio.callback(
                self._chip,
                self._gpio_pin,
                lgpio.BOTH_EDGES,
                self._on_edge,
            )
        except Exception:
            lgpio.gpiochip_close(self._chip)
            self._chip = None
            raise

        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._state_pub = self.create_publisher(
            EncoderStateMsg, '/wheel/encoder_state', 10
        )
        self.create_subscription(
            Int8, '/motor/direction', self._on_direction, 10
        )
        self.create_timer(self._window_ms / 1000.0, self._publish_window)
        self.get_logger().info(
            f'encoder_node ready on GPIO {self._gpio_pin}'
        )

    def _on_edge(self, chip, gpio, level, tick):
        self._state.record_edge(time.monotonic_ns())

    def _on_direction(self, msg: Int8):
        if not self._state.update_direction(msg.data):
            self.get_logger().warning(
                f'Ignoring invalid /motor/direction value {msg.data}; '
                'expected -1, 0, or +1'
            )

    def _publish_window(self):
        window_s = self._window_ms / 1000.0
        measurement = self._state.take_measurement(time.monotonic_ns())
        stamp = self.get_clock().now().to_msg()
        odom_msg, state_msg = build_publications(
            measurement,
            stamp,
            self._metres_per_edge,
            window_s,
        )
        self._odom_pub.publish(odom_msg)
        self._state_pub.publish(state_msg)

    def close_gpio(self):
        if self._gpio_callback is not None:
            self._gpio_callback.cancel()
            self._gpio_callback = None
        if self._chip is not None:
            lgpio.gpiochip_close(self._chip)
            self._chip = None


def main():
    rclpy.init()
    node = None
    try:
        node = EncoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_gpio()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
