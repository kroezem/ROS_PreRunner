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


def test_default_autonomy_hold_covers_long_routes():
    assert sender.DEFAULT_AUTONOMY_HOLD_TIMEOUT == 600.0


@pytest.fixture
def state():
    return sender.KeyboardInput(0.30, 30.0)


def test_space_arms_wasd_and_release_brakes(state):
    state.press('w', now=0.0)
    state.press('a', now=0.0)
    assert state.state(now=0.0)[:3] == (
        sender.MODE_BRAKE,
        0.0,
        0.0,
    )

    state.press('space', now=0.1)
    assert state.state(now=0.1)[:3] == (
        sender.MODE_DRIVE,
        0.30,
        1.0,
    )

    state.release('space')
    assert state.state(now=0.2)[:3] == (
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
    state.press('escape', now=0.1)

    assert state.pressed == set()
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


def test_autonomy_hold_expires_and_requires_release_repress(state):
    state.press('`', now=10.0)
    assert state.state(now=39.999)[0] == sender.MODE_SUPPRESS
    assert state.state(now=40.0)[0] == sender.MODE_BRAKE

    state.press('`', now=41.0)
    assert state.state(now=41.0)[0] == sender.MODE_BRAKE
    state.release('`')
    state.press('`', now=42.0)
    assert state.state(now=42.0)[0] == sender.MODE_SUPPRESS


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
        ('route_start', sender.ROUTE_START),
        ('route_stop', sender.ROUTE_STOP),
        ('route_clear', sender.ROUTE_CLEAR),
        ('route_loop_toggle', sender.ROUTE_LOOP_TOGGLE),
        ('route_remove_last', sender.ROUTE_REMOVE_LAST),
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
