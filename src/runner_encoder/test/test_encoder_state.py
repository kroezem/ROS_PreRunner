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

"""Tests for pulse-confirmed encoder direction latching."""

from builtin_interfaces.msg import Time
import pytest

from runner_encoder.encoder_node import build_publications
from runner_encoder.encoder_state import EncoderMeasurement, EncoderState


TIMEOUT = 0.2
TIMEOUT_NS = 200_000_000
WINDOW_SEC = 0.05
METRES_PER_EDGE = 0.010282


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
    assert measurement.edge_count == 1


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
    assert measurement.edge_count == 1
    assert measurement.edge_rate(WINDOW_SEC) == 20.0
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
    assert measurement.edge_rate(WINDOW_SEC) == 60.0


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


def test_diagnostic_and_odometry_share_one_measurement_snapshot():
    stamp = Time(sec=123, nanosec=456)
    measurement = EncoderMeasurement(
        edge_count=3,
        stationary=False,
        active_direction=-1,
        pending_direction=1,
    )

    odom, state = build_publications(
        measurement,
        stamp,
        METRES_PER_EDGE,
        WINDOW_SEC,
    )

    assert odom.header.stamp == state.stamp
    assert state.edge_rate == 60.0
    assert odom.twist.twist.linear.x == pytest.approx(-0.61692)
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


@pytest.mark.parametrize(
    'invalid_timeout',
    [0.0, -0.1, float('nan'), float('inf'), -float('inf')],
)
def test_invalid_stationary_timeout_is_rejected(invalid_timeout):
    with pytest.raises(ValueError):
        EncoderState(invalid_timeout)
