import math

import pytest
import rclpy

from runner_motor import motor_node


class FakePWM:
    instances = []

    def __init__(self, chip, channel):
        self.chip = chip
        self.channel = channel
        self.enabled = False
        self.duty_cycle_ns = None
        self.period_ns = None
        self.__class__.instances.append(self)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False


@pytest.fixture
def motor_factory(monkeypatch):
    nodes = []
    FakePWM.instances = []
    monkeypatch.setattr(motor_node, 'PWM', FakePWM)
    monkeypatch.setattr(motor_node.time, 'sleep', lambda _duration: None)

    def create(mode=None):
        args = []
        if mode is not None:
            value = "''" if mode == '' else mode
            args = ['--ros-args', '-p', f'esc_mode:={value}']
        rclpy.init(args=args)
        try:
            node = motor_node.MotorNode()
        except BaseException:
            rclpy.shutdown()
            raise
        nodes.append(node)
        return node

    yield create

    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_unset_esc_mode_refuses_startup_before_pwm(motor_factory):
    with pytest.raises(ValueError, match='esc_mode is required'):
        motor_factory()
    assert FakePWM.instances == []


def test_invalid_esc_mode_refuses_startup_before_pwm(motor_factory):
    with pytest.raises(ValueError, match='esc_mode is required'):
        motor_factory('invalid')
    assert FakePWM.instances == []


def test_non_string_esc_mode_refuses_startup_before_pwm(motor_factory):
    with pytest.raises(ValueError, match='esc_mode is required'):
        motor_factory(1)
    assert FakePWM.instances == []


def test_empty_esc_mode_refuses_startup_before_pwm(motor_factory):
    with pytest.raises(ValueError, match='esc_mode is required'):
        motor_factory('')
    assert FakePWM.instances == []


@pytest.mark.parametrize('mode', ['race', 'normal'])
def test_valid_esc_mode_is_accepted(motor_factory, mode):
    node = motor_factory(mode)
    assert node._esc_mode == mode
    assert len(FakePWM.instances) == 2
    assert all(pwm.enabled for pwm in FakePWM.instances)


@pytest.mark.parametrize(
    ('mode', 'command', 'expected'),
    [
        ('race', 1.0, 1750),
        ('race', 0.0, 1500),
        ('race', -0.05, 1500),
        ('race', -0.5, 1349),
        ('race', -1.0, 1000),
        ('normal', -0.5, 1405),
        ('normal', -1.0, 1250),
    ],
)
def test_command_mapping(mode, command, expected):
    pulse_us, _direction_input = motor_node.map_esc_command(command, mode)
    assert pulse_us == expected


def test_normal_mapping_preserves_non_finite_clamping():
    assert motor_node.map_esc_command(math.nan, 'normal')[0] == 1750
    assert motor_node.map_esc_command(math.inf, 'normal')[0] == 1750
    assert motor_node.map_esc_command(-math.inf, 'normal')[0] == 1250


@pytest.mark.parametrize('initial_state', [motor_node.STOP, motor_node.FWD])
def test_race_negative_enters_brake(initial_state):
    assert motor_node.next_direction_state(
        initial_state, 'in_rev', 'race'
    ) == motor_node.BRAKE


def test_race_repeated_negative_remains_brake_and_never_reports_reverse():
    state = motor_node.STOP
    directions = []
    for _ in range(3):
        state = motor_node.next_direction_state(state, 'in_rev', 'race')
        directions.append(
            -1 if state == motor_node.REV else int(state != motor_node.STOP)
        )
    assert state == motor_node.BRAKE
    assert directions == [1, 1, 1]


@pytest.mark.parametrize(
    ('initial_state', 'direction_input', 'expected'),
    [
        (motor_node.STOP, 'in_fwd', motor_node.FWD),
        (motor_node.REV, 'in_fwd', motor_node.FWD),
        (motor_node.FWD, 'in_neu', motor_node.STOP),
        (motor_node.REV, 'in_neu', motor_node.STOP),
        (motor_node.STOP, 'in_rev', motor_node.REV),
        (motor_node.REV, 'in_rev', motor_node.REV),
        (motor_node.FWD, 'in_rev', motor_node.BRAKE),
        (motor_node.BRAKE, 'in_rev', motor_node.BRAKE),
    ],
)
def test_normal_direction_transitions(
    initial_state, direction_input, expected
):
    assert motor_node.next_direction_state(
        initial_state, direction_input, 'normal'
    ) == expected


@pytest.mark.parametrize(
    ('mode', 'expected'),
    [('race', 1000), ('normal', 1500)],
)
def test_watchdog_pulse(mode, expected):
    assert motor_node.watchdog_pulse_us(mode) == expected


@pytest.mark.parametrize('mode', ['race', 'normal'])
def test_watchdog_sets_stop_and_publishes_zero(motor_factory, mode):
    node = motor_factory(mode)
    node._dir_state = motor_node.FWD
    node._last_cmd -= motor_node.CMD_TIMEOUT_S + 1.0

    direction_messages = []
    state_messages = []
    node._direction_pub.publish = direction_messages.append
    node._state_pub.publish = state_messages.append

    node._watchdog()

    assert node._dir_state == motor_node.STOP
    assert direction_messages[-1].data == 0
    assert state_messages[-1].data == motor_node.STOP
    expected = 1000 if mode == 'race' else 1500
    assert node.esc.duty_cycle_ns == motor_node.us_to_ns(expected)
