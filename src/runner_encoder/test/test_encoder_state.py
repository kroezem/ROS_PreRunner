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

"""Tests for encoder interval-rate estimation and direction latching."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time
import pytest
import rclpy

from runner_encoder import encoder_node as encoder_node_module
from runner_encoder.encoder_node import (
    build_publications,
    EncoderNode,
    read_history_depth,
)
from runner_encoder.encoder_state import (
    DEFAULT_HISTORY_DEPTH,
    EncoderMeasurement,
    EncoderState,
    MAX_LOOKBACK_NS,
    MIN_EDGE_INTERVAL_NS,
    validate_history_depth,
)


TIMEOUT = 0.2
TIMEOUT_NS = 200_000_000
METRES_PER_EDGE = 0.010282


class _RecordingLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class _ParameterNode:
    _USE_DEFAULT = object()

    def __init__(self, value=_USE_DEFAULT):
        self._value = value
        self.logger = _RecordingLogger()

    def declare_parameter(self, name, default):
        assert name == 'history_depth'
        if self._value is self._USE_DEFAULT:
            self._value = default

    def get_parameter(self, name):
        assert name == 'history_depth'
        return SimpleNamespace(value=self._value)

    def get_logger(self):
        return self.logger


def _start_forward(state: EncoderState, edge_time_ns: int = 0) -> None:
    assert state.update_direction(1)
    state.record_edge(edge_time_ns)


def test_direction_zero_does_not_clear_active_direction():
    state = EncoderState(TIMEOUT)
    _start_forward(state)

    assert state.update_direction(0)

    measurement = state.take_measurement(50_000_000)
    assert measurement.active_direction == 1


def test_direction_zero_does_not_clear_latest_nonzero_direction():
    state = EncoderState(TIMEOUT)
    assert state.update_direction(-1)
    assert state.update_direction(0)

    measurement = state.take_measurement(0)
    assert measurement.pending_direction == -1


@pytest.mark.parametrize('invalid_direction', [-128, -2, 2, 127])
def test_invalid_direction_does_not_affect_state(invalid_direction):
    state = EncoderState(TIMEOUT)
    _start_forward(state)

    assert not state.update_direction(invalid_direction)

    measurement = state.take_measurement(50_000_000)
    assert measurement.active_direction == 1
    assert measurement.pending_direction == 1


def test_opposing_command_while_moving_does_not_change_active_direction():
    state = EncoderState(TIMEOUT)
    _start_forward(state)

    assert state.update_direction(-1)
    state.record_edge(50_000_000)

    measurement = state.take_measurement(50_000_000)
    assert not measurement.stationary
    assert measurement.active_direction == 1
    assert measurement.pending_direction == -1


def test_confirmed_stop_does_not_change_active_direction():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(-1)

    measurement = state.take_measurement(TIMEOUT_NS)

    assert measurement.stationary
    assert measurement.active_direction == 1
    assert measurement.pending_direction == -1


def test_first_edge_after_stop_commits_latest_nonzero_direction():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(-1)
    assert state.take_measurement(TIMEOUT_NS).stationary

    state.record_edge(TIMEOUT_NS + 10_000_000)

    measurement = state.take_measurement(TIMEOUT_NS + 10_000_000)
    assert not measurement.stationary
    assert measurement.active_direction == -1
    assert measurement.edge_rate == 0.0


def test_edge_gap_can_confirm_stop_before_the_next_timer_snapshot():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(-1)

    state.record_edge(TIMEOUT_NS)

    measurement = state.take_measurement(TIMEOUT_NS)
    assert measurement.active_direction == -1
    assert not measurement.stationary


def test_reverse_blip_then_forward_commits_forward_after_later_stop():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(-1)
    assert state.update_direction(1)
    assert state.take_measurement(TIMEOUT_NS).stationary

    state.record_edge(TIMEOUT_NS + 10_000_000)

    measurement = state.take_measurement(TIMEOUT_NS + 10_000_000)
    assert measurement.active_direction == 1
    assert measurement.pending_direction == 1


def test_false_stop_with_same_direction_preserves_effective_sign():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.take_measurement(TIMEOUT_NS).stationary

    state.record_edge(TIMEOUT_NS + 10_000_000)

    measurement = state.take_measurement(TIMEOUT_NS + 10_000_000)
    assert measurement.active_direction == 1
    assert not measurement.stationary


def test_startup_edges_without_direction_keep_active_direction_unknown():
    state = EncoderState(TIMEOUT)

    state.record_edge(0)

    measurement = state.take_measurement(0)
    assert measurement.edge_rate == 0.0
    assert measurement.active_direction == 0
    assert measurement.pending_direction == 0
    assert not measurement.stationary


@pytest.mark.parametrize('active_direction', [-1, 0, 1])
def test_edge_rate_is_independent_of_commanded_direction(active_direction):
    state = EncoderState(TIMEOUT)
    assert state.update_direction(active_direction)
    state.record_edge(0)
    state.record_edge(10_000_000)
    state.record_edge(20_000_000)

    measurement = state.take_measurement(20_000_000)
    assert measurement.edge_rate == 100.0


def test_stationary_becomes_true_only_at_configured_timeout():
    state = EncoderState(TIMEOUT)
    edge_time_ns = 10_000_000_000
    state.record_edge(edge_time_ns)

    assert not state.take_measurement(
        edge_time_ns + TIMEOUT_NS - 1
    ).stationary
    assert state.take_measurement(
        edge_time_ns + TIMEOUT_NS
    ).stationary


def test_pending_forward_signs_odometry_when_active_direction_is_reverse():
    stamp = Time(sec=123, nanosec=456)
    measurement = EncoderMeasurement(
        edge_rate=60.0,
        stationary=False,
        active_direction=-1,
        pending_direction=1,
    )

    odom, state = build_publications(
        measurement,
        stamp,
        METRES_PER_EDGE,
    )

    assert odom.header.stamp == state.stamp
    assert state.edge_rate == 60.0
    assert odom.twist.twist.linear.x == pytest.approx(0.61692)
    assert not state.stationary
    assert state.active_direction == -1
    assert state.pending_direction == 1
    assert set(state.get_fields_and_field_types()) == {
        'stamp',
        'edge_rate',
        'stationary',
        'active_direction',
        'pending_direction',
    }


def test_pending_reverse_signs_odometry_when_active_direction_is_forward():
    measurement = EncoderMeasurement(
        edge_rate=40.0,
        stationary=False,
        active_direction=1,
        pending_direction=-1,
    )

    odom, state = build_publications(
        measurement,
        Time(sec=1, nanosec=2),
        METRES_PER_EDGE,
    )

    assert state.edge_rate == 40.0
    assert state.active_direction == 1
    assert state.pending_direction == -1
    assert odom.twist.twist.linear.x == pytest.approx(-0.41128)


def test_direction_zero_coast_keeps_latest_nonzero_sign():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(0)
    state.record_edge(25_000_000)
    state.record_edge(50_000_000)

    measurement = state.take_measurement(50_000_000)
    odom, diagnostic = build_publications(
        measurement,
        Time(sec=3, nanosec=4),
        METRES_PER_EDGE,
    )

    assert diagnostic.edge_rate == 40.0
    assert diagnostic.pending_direction == 1
    assert odom.twist.twist.linear.x == pytest.approx(0.41128)


def test_startup_activity_exposes_edge_rate_but_signed_speed_is_zero():
    state = EncoderState(TIMEOUT)
    state.record_edge(0)
    measurement = state.take_measurement(0)

    odom, diagnostic = build_publications(
        measurement,
        Time(sec=5, nanosec=6),
        METRES_PER_EDGE,
    )

    assert diagnostic.edge_rate == 0.0
    assert diagnostic.active_direction == 0
    assert diagnostic.pending_direction == 0
    assert odom.twist.twist.linear.x == 0.0


def test_active_direction_latch_remains_stop_delimited():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    assert state.update_direction(-1)
    state.record_edge(50_000_000)

    moving = state.take_measurement(50_000_000)
    assert moving.active_direction == 1
    assert moving.pending_direction == -1

    assert state.take_measurement(250_000_000).stationary
    state.record_edge(260_000_000)
    restarted = state.take_measurement(260_000_000)
    assert restarted.active_direction == -1
    assert restarted.pending_direction == -1


@pytest.mark.parametrize(
    'invalid_timeout',
    [0.0, -0.1, float('nan'), float('inf'), -float('inf')],
)
def test_invalid_stationary_timeout_is_rejected(invalid_timeout):
    with pytest.raises(ValueError):
        EncoderState(invalid_timeout)


def test_default_history_depth_is_four():
    node = _ParameterNode()

    assert read_history_depth(node) == 4
    assert DEFAULT_HISTORY_DEPTH == 4


@pytest.mark.parametrize('history_depth', [1, 2, 4, 8])
def test_explicit_history_depth_is_accepted_and_logged(history_depth):
    node = _ParameterNode(history_depth)

    assert read_history_depth(node) == history_depth
    assert node.logger.info_messages == [
        f'Encoder interval history depth: {history_depth}'
    ]
    assert node.logger.error_messages == []


@pytest.mark.parametrize('history_depth', [0, -1])
def test_nonpositive_history_depth_is_rejected_and_logged(history_depth):
    node = _ParameterNode(history_depth)

    with pytest.raises(ValueError, match='history_depth must be at least 1'):
        read_history_depth(node)

    assert node.logger.error_messages == [
        'Invalid history_depth parameter: history_depth must be at least 1'
    ]


@pytest.mark.parametrize('history_depth', [True, 1.0, '4'])
def test_noninteger_history_depth_is_rejected_and_logged(history_depth):
    node = _ParameterNode(history_depth)

    with pytest.raises(ValueError, match='history_depth must be an integer'):
        read_history_depth(node)

    assert node.logger.error_messages == [
        'Invalid history_depth parameter: history_depth must be an integer'
    ]


def test_history_depth_that_cannot_size_timestamp_deque_is_rejected():
    import sys

    with pytest.raises(ValueError, match='must be no greater than'):
        validate_history_depth(sys.maxsize)


def test_node_defaults_preserve_publication_rate_and_distance(monkeypatch):
    class FakeReader:
        chip_path = '/dev/gpiochip4'
        event_count = 0
        short_interval_count = 0

        def __init__(self, gpio_pin, record_edge, error_callback):
            assert gpio_pin == 22

        def close(self):
            pass

    monkeypatch.setattr(
        encoder_node_module,
        'GpioEventReader',
        FakeReader,
    )

    rclpy.init(args=[])
    node = None
    try:
        node = EncoderNode()

        assert node.get_parameter('metres_per_edge').value == 0.010282
        assert node.get_parameter('window_ms').value == 50
        assert next(node.timers).timer_period_ns == 50_000_000
        assert node._state._edge_timestamps_ns.maxlen == 5
    finally:
        if node is not None:
            node.close_gpio()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


@pytest.mark.parametrize('history_depth', [1, 2, 4, 8])
def test_history_container_length_matches_selected_depth(history_depth):
    state = EncoderState(TIMEOUT, history_depth)

    assert state._edge_timestamps_ns.maxlen == history_depth + 1


@pytest.mark.parametrize('history_depth', [1, 2, 4, 8])
def test_rate_averages_latest_selected_interval_count(history_depth):
    intervals_ns = [
        8_000_000,
        12_000_000,
        10_000_000,
        14_000_000,
        9_000_000,
        11_000_000,
        13_000_000,
        15_000_000,
    ]
    edge_times_ns = [0]
    for interval_ns in intervals_ns:
        edge_times_ns.append(edge_times_ns[-1] + interval_ns)

    state = EncoderState(TIMEOUT, history_depth)
    for edge_time_ns in edge_times_ns:
        state.record_edge(edge_time_ns)

    measurement = state.take_measurement(edge_times_ns[-1])
    retained_span_ns = sum(intervals_ns[-history_depth:])
    expected_rate = history_depth * 1e9 / retained_span_ns

    assert measurement.edge_rate == pytest.approx(expected_rate)


def test_depth_one_uses_only_latest_interval():
    state = EncoderState(TIMEOUT, 1)
    for edge_time_ns in (0, 10_000_000, 30_000_000):
        state.record_edge(edge_time_ns)

    assert state.take_measurement(30_000_000).edge_rate == 50.0
    assert list(state._edge_timestamps_ns) == [10_000_000, 30_000_000]


def test_history_container_drops_oldest_timestamp_when_full():
    state = EncoderState(TIMEOUT, 2)
    for edge_time_ns in (0, 10_000_000, 20_000_000, 30_000_000):
        state.record_edge(edge_time_ns)

    assert list(state._edge_timestamps_ns) == [
        10_000_000,
        20_000_000,
        30_000_000,
    ]


def test_rate_averages_all_retained_intervals():
    state = EncoderState(TIMEOUT)
    for edge_time_ns in (0, 8_000_000, 20_000_000, 30_000_000):
        state.record_edge(edge_time_ns)

    measurement = state.take_measurement(30_000_000)

    assert measurement.edge_rate == 100.0


def test_timestamp_buffer_is_bounded_to_one_magnet_cycle():
    history_depth = 8
    state = EncoderState(TIMEOUT, history_depth)
    interval_ns = 1_000_000
    edge_count = history_depth + 6
    for index in range(edge_count):
        state.record_edge(index * interval_ns)

    measurement = state.take_measurement((edge_count - 1) * interval_ns)

    assert measurement.edge_rate == 1000.0
    assert len(state._edge_timestamps_ns) == history_depth + 1


def test_timestamp_history_is_bounded_by_lookback():
    state = EncoderState(1.0)
    state.record_edge(0)
    state.record_edge(MAX_LOOKBACK_NS - 10_000_000)
    state.record_edge(MAX_LOOKBACK_NS + 10_000_000)

    measurement = state.take_measurement(MAX_LOOKBACK_NS + 10_000_000)

    assert list(state._edge_timestamps_ns) == [
        MAX_LOOKBACK_NS - 10_000_000,
        MAX_LOOKBACK_NS + 10_000_000,
    ]
    assert measurement.edge_rate == 50.0


def test_rate_requires_at_least_two_timestamps():
    state = EncoderState(TIMEOUT)
    state.record_edge(10_000_000)

    assert state.take_measurement(10_000_000).edge_rate == 0.0


def test_duplicate_and_impossibly_short_timestamps_are_ignored():
    state = EncoderState(TIMEOUT)
    state.record_edge(10_000_000)
    state.record_edge(10_000_000)
    state.record_edge(10_000_000 + MIN_EDGE_INTERVAL_NS - 1)
    state.record_edge(20_000_000)

    measurement = state.take_measurement(20_000_000)

    assert list(state._edge_timestamps_ns) == [10_000_000, 20_000_000]
    assert measurement.edge_rate == 100.0


def test_out_of_order_timestamp_is_ignored():
    state = EncoderState(TIMEOUT)
    state.record_edge(20_000_000)
    state.record_edge(10_000_000)
    state.record_edge(30_000_000)

    measurement = state.take_measurement(30_000_000)

    assert list(state._edge_timestamps_ns) == [20_000_000, 30_000_000]
    assert measurement.edge_rate == 100.0


def test_publication_clock_discontinuity_discards_old_epoch():
    state = EncoderState(TIMEOUT)
    state.record_edge(20_000_000)
    state.record_edge(30_000_000)

    measurement = state.take_measurement(10_000_000)

    assert measurement.stationary
    assert measurement.edge_rate == 0.0
    assert list(state._edge_timestamps_ns) == []


def test_rate_decays_after_one_most_recent_interval_without_an_edge():
    state = EncoderState(TIMEOUT)
    for edge_time_ns in (0, 10_000_000, 20_000_000):
        state.record_edge(edge_time_ns)

    assert state.take_measurement(30_000_000).edge_rate == 100.0
    assert state.take_measurement(40_000_000).edge_rate == 50.0
    assert state.take_measurement(120_000_000).edge_rate == 10.0


def test_stop_decay_caps_a_faster_history_using_edge_age():
    state = EncoderState(TIMEOUT)
    for edge_time_ns in (0, 5_000_000, 15_000_000):
        state.record_edge(edge_time_ns)

    measurement = state.take_measurement(35_000_000)

    assert measurement.edge_rate == 50.0


def test_stationary_timeout_zeros_rate_and_clears_history():
    state = EncoderState(TIMEOUT)
    state.record_edge(0)
    state.record_edge(10_000_000)

    measurement = state.take_measurement(10_000_000 + TIMEOUT_NS)

    assert measurement.stationary
    assert measurement.edge_rate == 0.0
    assert list(state._edge_timestamps_ns) == []


def test_long_edge_gap_starts_fresh_stop_epoch_history():
    state = EncoderState(TIMEOUT)
    _start_forward(state)
    state.record_edge(10_000_000)
    assert state.update_direction(-1)

    state.record_edge(10_000_000 + TIMEOUT_NS)
    measurement = state.take_measurement(10_000_000 + TIMEOUT_NS)

    assert measurement.active_direction == -1
    assert measurement.edge_rate == 0.0
    assert list(state._edge_timestamps_ns) == [
        10_000_000 + TIMEOUT_NS
    ]
