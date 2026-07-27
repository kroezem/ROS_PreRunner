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
Publish command-informed velocity from a single-channel wheel encoder.

``pending_direction`` is the latest valid nonzero commanded direction and
signs ``/wheel/odom``. ``active_direction`` remains a diagnostic view of
stop-delimited movement epochs. They should agree under controlled
stop-delimited transitions, which the future D-34 reverse/ESC handshake is
expected to enforce during autonomous control.

Manual direction changes may occur without a pulse-confirmed stop, leaving
``active_direction`` stale. Conversely, ``pending_direction`` may briefly lead
physical reversal during a command transition. Signed wheel odometry is
therefore command-informed; a single-channel encoder cannot measure direction
independently.
"""

import glob
import math
import threading
import time

import gpiod
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.exceptions import ParameterException
from rclpy.node import Node
from runner_encoder.encoder_state import (
    DEFAULT_HISTORY_DEPTH,
    EncoderMeasurement,
    EncoderState,
    MIN_EDGE_INTERVAL_NS,
    validate_history_depth,
)
from runner_interfaces.msg import EncoderState as EncoderStateMsg
from std_msgs.msg import Int8


GPIO_CHIP_LABEL = 'pinctrl-rp1'
GPIO_CONSUMER = 'runner_encoder'
GPIO_EVENT_WAIT_NS = 100_000_000
LARGE_VARIANCE = 1e6
STATIONARY_TIMEOUT_DESCRIPTION = (
    'Pulse-free interval used to declare the encoder stationary. '
    'Distance per edge is 0.010282 m/edge. At 0.2 seconds, one edge per '
    'timeout corresponds to 0.010282 / 0.2 = 0.05141 m/s. This timeout is '
    'therefore a low-speed threshold in disguise.'
)


def open_gpio_chip_by_label(
    label: str,
    *,
    gpiod_module=gpiod,
    chip_paths=None,
):
    """Open exactly one GPIO chip matching a live kernel label."""
    if chip_paths is None:
        chip_paths = sorted(glob.glob('/dev/gpiochip*'))

    matches = []
    try:
        for path in chip_paths:
            chip = gpiod_module.Chip(
                path,
                gpiod_module.Chip.OPEN_BY_PATH,
            )
            if chip.label() == label:
                matches.append((path, chip))
            else:
                chip.close()
    except Exception:
        for _, chip in matches:
            chip.close()
        raise

    if len(matches) != 1:
        for _, chip in matches:
            chip.close()
        raise RuntimeError(
            f'Expected exactly one GPIO chip labeled {label!r}; '
            f'found {len(matches)}'
        )
    return matches[0]


class GpioEventReader:
    """Own one GPIO line and forward kernel edge timestamps from a worker."""

    def __init__(
        self,
        gpio_pin: int,
        record_edge,
        error_callback,
        *,
        gpiod_module=gpiod,
        chip_paths=None,
    ):
        self._gpiod = gpiod_module
        self._record_edge = record_edge
        self._error_callback = error_callback
        self._stop_event = threading.Event()
        self._chip_path = None
        self._chip = None
        self._line = None
        self._line_requested = False
        self._thread = None
        self._event_count = 0
        self._short_interval_count = 0
        self._last_event_ns = None

        try:
            self._chip_path, self._chip = open_gpio_chip_by_label(
                GPIO_CHIP_LABEL,
                gpiod_module=self._gpiod,
                chip_paths=chip_paths,
            )
            self._line = self._chip.get_line(gpio_pin)
            self._line.request(
                consumer=GPIO_CONSUMER,
                type=self._gpiod.LINE_REQ_EV_BOTH_EDGES,
            )
            self._line_requested = True
            thread = threading.Thread(
                target=self._read_events,
                name='encoder_gpio_events',
                daemon=True,
            )
            thread.start()
            self._thread = thread
        except Exception:
            self.close()
            raise

    @property
    def chip_path(self):
        return self._chip_path

    @property
    def event_count(self):
        return self._event_count

    @property
    def short_interval_count(self):
        return self._short_interval_count

    def _read_events(self):
        try:
            while not self._stop_event.is_set():
                if not self._line.event_wait(
                    sec=0,
                    nsec=GPIO_EVENT_WAIT_NS,
                ):
                    continue
                event = self._line.event_read()
                timestamp_ns = event.sec * 1_000_000_000 + event.nsec
                self._event_count += 1
                if (
                    self._last_event_ns is not None
                    and timestamp_ns - self._last_event_ns
                    < MIN_EDGE_INTERVAL_NS
                ):
                    self._short_interval_count += 1
                self._last_event_ns = timestamp_ns
                self._record_edge(timestamp_ns)
        except Exception as error:
            if not self._stop_event.is_set():
                self._error_callback(error)

    def close(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self._line_requested:
            self._line.release()
            self._line_requested = False
        self._line = None
        if self._chip is not None:
            self._chip.close()
            self._chip = None


def read_history_depth(node) -> int:
    """Declare, validate, and report the retained interval count."""
    try:
        node.declare_parameter('history_depth', DEFAULT_HISTORY_DEPTH)
        history_depth = validate_history_depth(
            node.get_parameter('history_depth').value
        )
    except (ParameterException, ValueError) as error:
        node.get_logger().error(f'Invalid history_depth parameter: {error}')
        raise

    node.get_logger().info(
        f'Encoder interval history depth: {history_depth}'
    )
    return history_depth


def build_publications(
    measurement: EncoderMeasurement,
    stamp,
    metres_per_edge: float,
):
    """Build odometry and diagnostics from one encoder state snapshot."""
    edge_rate = measurement.edge_rate
    signed_speed = (
        edge_rate * metres_per_edge * measurement.pending_direction
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

        self._history_depth = read_history_depth(self)
        self._state = EncoderState(
            self._stationary_timeout_sec,
            self._history_depth,
        )
        self._gpio_reader = None

        self._odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self._state_pub = self.create_publisher(
            EncoderStateMsg, '/wheel/encoder_state', 10
        )
        self.create_subscription(
            Int8, '/motor/direction', self._on_direction, 10
        )
        self.create_timer(self._window_ms / 1000.0, self._publish_window)
        try:
            self._gpio_reader = GpioEventReader(
                self._gpio_pin,
                self._state.record_edge,
                self._on_gpio_error,
            )
        except Exception as error:
            self.get_logger().error(
                f'Failed to initialize encoder GPIO {self._gpio_pin}: {error}'
            )
            raise
        self.get_logger().info(
            f'encoder_node ready on {self._gpio_reader.chip_path} '
            f'GPIO {self._gpio_pin}'
        )

    def _on_gpio_error(self, error):
        self.get_logger().error(f'Encoder GPIO event reader failed: {error}')

    def _on_direction(self, msg: Int8):
        if not self._state.update_direction(msg.data):
            self.get_logger().warning(
                f'Ignoring invalid /motor/direction value {msg.data}; '
                'expected -1, 0, or +1'
            )

    def _publish_window(self):
        measurement = self._state.take_measurement(time.monotonic_ns())
        stamp = self.get_clock().now().to_msg()
        odom_msg, state_msg = build_publications(
            measurement,
            stamp,
            self._metres_per_edge,
        )
        self._odom_pub.publish(odom_msg)
        self._state_pub.publish(state_msg)

    def close_gpio(self):
        if self._gpio_reader is not None:
            reader = self._gpio_reader
            reader.close()
            self.get_logger().info(
                'Encoder GPIO stopped: '
                f'events={reader.event_count}, '
                'intervals_below_100us='
                f'{reader.short_interval_count}'
            )
            self._gpio_reader = None


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
