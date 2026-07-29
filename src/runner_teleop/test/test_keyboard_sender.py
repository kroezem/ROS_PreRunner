"""Pure state tests for the globally capturing laptop sender."""

import importlib.util
from pathlib import Path

import pytest


SENDER_PATH = (
    Path(__file__).parents[3] / 'tools' / 'keyboard_sender.py'
)
SPEC = importlib.util.spec_from_file_location(
    'keyboard_sender_under_test',
    SENDER_PATH,
)
sender = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sender)


def test_default_autonomy_latch_covers_long_routes():
    assert sender.DEFAULT_AUTONOMY_LATCH_TIMEOUT == 600.0
    assert sender.DEFAULT_AUTONOMY_HOLD_TIMEOUT == 600.0


@pytest.fixture
def state():
    return sender.KeyboardInput(0.30, 30.0)


def test_initially_disarmed_and_space_retains_hold_to_run_role(state):
    assert not state.autonomy_armed
    assert state.state(now=0.0)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )

    state.press('w', now=0.0)
    state.press('a', now=0.0)
    assert state.state(now=0.0)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )

    state.press('space', now=0.1)
    assert state.state(now=0.0)[:3] == (
        sender.MODE_DRIVE,
        0.30,
        1.0,
    )

    state.release('space')
    assert state.state(now=0.1)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )


def test_independent_key_events_and_opposing_steering_cancel(state):
    state.press('space', now=0.0)
    state.press('a', now=0.0)
    state.press('d', now=0.0)
    assert state.state(now=0.0)[2] == 0.0

    state.release('a')
    assert state.state(now=0.1)[2] == -1.0


def test_s_brakes_even_while_space_and_w_are_held(state):
    state.press('space', now=0.0)
    state.press('w', now=0.0)
    state.press('s', now=0.0)

    assert state.state(now=0.0)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )


def test_escape_clears_every_held_key_and_zeroes_command(state):
    state.press('space', now=0.0)
    state.press('w', now=0.0)
    state.press('a', now=0.0)
    state.press('route_start', now=0.0)
    state.press('escape', now=0.1)

    assert state.pressed == set()
    assert list(state.route_commands) == []
    assert not state.autonomy_armed
    assert state.state(now=0.1)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )


def test_setpoint_steps_once_per_press_without_hold_repeat(state):
    state.press('=', now=0.0)
    state.press('=', now=0.1)
    assert state.setpoint == 0.31
    state.release('=')
    state.press('=', now=0.2)
    assert state.setpoint == 0.32

    state.press('-', now=0.3)
    state.press('-', now=0.4)
    assert state.setpoint == 0.31


def test_autonomy_toggle_expires_from_arm_and_requires_explicit_rearm(state):
    state.press('`', now=10.0)
    assert state.autonomy_armed
    state.release('`')
    assert state.state(now=10.1)[0] == sender.MODE_SUPPRESS
    assert state.state(now=39.999)[0] == sender.MODE_SUPPRESS
    assert state.state(now=40.0)[0] == sender.MODE_BRAKE

    state.release('`')
    state.press('`', now=41.0)
    assert state.state(now=41.0)[0] == sender.MODE_SUPPRESS
    assert state.autonomy_armed_at == 41.0

    state.release('`')
    state.press('`', now=42.0)
    assert state.state(now=42.0)[0] == sender.MODE_BRAKE


def test_second_backtick_press_disarms_and_brakes(state):
    state.press('`', now=1.0)
    state.release('`')
    state.press('`', now=2.0)

    assert not state.autonomy_armed
    assert state.state(now=2.0)[0] == sender.MODE_BRAKE


def test_space_does_not_clear_armed_autonomy_latch(state):
    state.press('`', now=1.0)
    state.release('`')
    state.press('space', now=2.0)
    state.release('space')

    assert state.autonomy_armed
    assert state.state(now=2.0)[0] == sender.MODE_SUPPRESS


def test_escape_disarms_for_at_least_one_second_and_has_21_opportunities(
    state,
):
    state.press('`', now=1.0)
    state.release('`')
    state.press('escape', now=2.0)

    opportunities = [
        state.state(now=2.0 + index * sender.SEND_PERIOD)[0]
        for index in range(21)
    ]
    assert not state.autonomy_armed
    assert opportunities == [sender.MODE_BRAKE] * 21
    assert state.disarm_until == pytest.approx(3.05)
    assert state.disarm_until - 2.0 >= 1.0


def test_escape_while_disarmed_is_idempotent_and_repress_extends_burst(state):
    state.press('space', now=0.0)
    state.press('w', now=0.0)
    state.press('escape', now=1.0)
    state.press('escape', now=1.5)

    assert not state.autonomy_armed
    assert state.pressed == set()
    assert state.disarm_until == pytest.approx(2.55)
    assert state.state(now=2.5)[0] == sender.MODE_BRAKE


def test_sender_shutdown_closes_without_transmitting_disarm():
    class FakeListener:
        stopped = False

        def stop(self):
            self.stopped = True

    class FakeSocket:
        closed = False
        sent = []

        def close(self):
            self.closed = True

        def sendto(self, *_args):
            self.sent.append(_args)

    listener = FakeListener()
    udp_socket = FakeSocket()
    sender.close_sender(listener, udp_socket)

    assert listener.stopped
    assert udp_socket.closed
    assert udp_socket.sent == []


def test_route_function_keys_queue_discrete_commands_without_repeat(state):
    state.press('route_start', now=0.0)
    state.press('route_start', now=0.1)
    state.press('route_loop_toggle', now=0.1)
    state.release('route_start')
    state.press('route_start', now=0.2)

    assert state.state(now=0.2)[3] == sender.ROUTE_START
    assert state.state(now=0.3)[3] == sender.ROUTE_LOOP_TOGGLE
    assert state.state(now=0.4)[3] == sender.ROUTE_START
    assert state.state(now=0.5)[3] == sender.ROUTE_NONE


@pytest.mark.parametrize(
    ('token', 'expected'),
    [
        (
            'clear_global_obstacles',
            sender.ROUTE_CLEAR_GLOBAL_OBSTACLES,
        ),
        (
            'toggle_global_obstacles',
            sender.ROUTE_TOGGLE_GLOBAL_OBSTACLES,
        ),
    ],
)
def test_costmap_function_keys_are_discrete_without_hold_repeat(
    state,
    token,
    expected,
):
    state.press(token, now=0.0)
    state.press(token, now=0.1)
    assert state.state(now=0.1)[3] == expected
    assert state.state(now=0.2)[3] == sender.ROUTE_NONE

    state.release(token)
    state.press(token, now=0.3)
    assert state.state(now=0.3)[3] == expected


@pytest.mark.parametrize(
    ('token', 'expected'),
    [
        ('route_start', sender.ROUTE_START),
        ('route_stop', sender.ROUTE_STOP),
        ('route_clear', sender.ROUTE_CLEAR),
        ('route_loop_toggle', sender.ROUTE_LOOP_TOGGLE),
        ('route_remove_last', sender.ROUTE_REMOVE_LAST),
        (
            'clear_global_obstacles',
            sender.ROUTE_CLEAR_GLOBAL_OBSTACLES,
        ),
        (
            'toggle_global_obstacles',
            sender.ROUTE_TOGGLE_GLOBAL_OBSTACLES,
        ),
    ],
)
def test_each_route_key_has_the_expected_wire_command(
    state,
    token,
    expected,
):
    state.press(token, now=0.0)
    assert state.state(now=0.0)[3] == expected


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf'])
def test_autonomy_timeout_parameter_requires_positive_finite_value(value):
    with pytest.raises(Exception):
        sender.positive_seconds(value)
