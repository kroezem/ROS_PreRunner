import math

import pytest

from runner_teleop.teleop_node import _active_mode
from runner_teleop.teleop_node import _validate_fixed_throttle_config
from runner_teleop.teleop_node import BRAKE_MODE
from runner_teleop.teleop_node import DPAD_VERTICAL_AXIS_INDEX
from runner_teleop.teleop_node import expected_race_esc_pulse_us
from runner_teleop.teleop_node import FIXED_THROTTLE_INHIBITED_MODE
from runner_teleop.teleop_node import FIXED_THROTTLE_MODE
from runner_teleop.teleop_node import L1_BUTTON_INDEX
from runner_teleop.teleop_node import MANUAL_MODE
from runner_teleop.teleop_node import R1_BUTTON_INDEX
from runner_teleop.teleop_node import TELEOP_SUPPRESS_MODE
from runner_teleop.teleop_node import TeleopNode
from runner_teleop.teleop_node import X_BUTTON_INDEX
from sensor_msgs.msg import Joy


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def __init__(self):
        self.info_messages = []

    def info(self, message):
        self.info_messages.append(message)


def make_node():
    node = TeleopNode.__new__(TeleopNode)
    node.pub = Publisher()
    node._setpoint_pub = Publisher()
    node._active_mode_pub = Publisher()
    node._logger = Logger()
    node.get_logger = lambda: node._logger
    node._axis_steer = 0
    node._axis_brake = 2
    node._axis_throttle = 5
    node._deadman_button = X_BUTTON_INDEX
    node._steer = 0.0
    node._manual_cmd = 0.0
    node._brake = 0.0
    node._manual_held = False
    node._fixed_throttle_held = False
    node._teleop_suppress_held = False
    node.fixed_throttle_inhibited_until_r1_release = False
    node._fixed_throttle_setpoint = 0.30
    node._fixed_throttle_step = 0.01
    node._fixed_throttle_min_setpoint = 0.00
    node._fixed_throttle_max_setpoint = 0.50
    node._dpad_press_active = False
    return node


def joy(
    *,
    steer=0.0,
    brake=1.0,
    throttle=1.0,
    x=False,
    r1=False,
    l1=False,
    create=False,
    dpad_vertical=0.0,
):
    message = Joy()
    message.axes = [
        steer,
        0.0,
        brake,
        0.0,
        0.0,
        throttle,
        0.0,
        dpad_vertical,
    ]
    message.buttons = [0] * 13
    message.buttons[X_BUTTON_INDEX] = int(x)
    message.buttons[L1_BUTTON_INDEX] = int(l1)
    message.buttons[R1_BUTTON_INDEX] = int(r1)
    message.buttons[8] = int(create)
    return message


def cycle(node, message):
    cmd_count = len(node.pub.messages)
    node.on_joy(message)
    node.publish_cmd()
    command = (
        node.pub.messages[-1]
        if len(node.pub.messages) > cmd_count
        else None
    )
    mode = node._active_mode_pub.messages[-1].data
    setpoint = node._setpoint_pub.messages[-1].data
    return command, mode, setpoint


@pytest.mark.parametrize(
    'x,r1,l1,expected_mode,expected_command',
    [
        (False, False, False, BRAKE_MODE, -1.0),
        (True, False, False, MANUAL_MODE, 0.75),
        (False, True, False, FIXED_THROTTLE_MODE, 0.30),
        (False, False, True, TELEOP_SUPPRESS_MODE, None),
        (True, True, False, MANUAL_MODE, 0.75),
        (True, False, True, MANUAL_MODE, 0.75),
        (False, True, True, FIXED_THROTTLE_MODE, 0.30),
        (True, True, True, MANUAL_MODE, 0.75),
    ],
)
def test_priority_table_and_diagnostics(
    x,
    r1,
    l1,
    expected_mode,
    expected_command,
):
    node = make_node()

    command, mode, setpoint = cycle(
        node,
        joy(steer=0.2, throttle=-0.5, x=x, r1=r1, l1=l1),
    )

    assert mode == expected_mode
    assert setpoint == pytest.approx(0.30)
    if expected_command is None:
        assert command is None
    else:
        assert command.linear.x == pytest.approx(expected_command)
        assert command.angular.z == pytest.approx(0.2)


def test_manual_trigger_brake_steering_and_priority_are_unchanged():
    node = make_node()

    throttle, mode, _ = cycle(
        node,
        joy(steer=-0.6, throttle=0.5, x=True, r1=True, l1=True),
    )
    brake, _, _ = cycle(
        node,
        joy(steer=0.3, brake=0.0, throttle=-1.0, x=True),
    )

    assert mode == MANUAL_MODE
    assert throttle.linear.x == 0.25
    assert throttle.angular.z == pytest.approx(-0.6)
    assert brake.linear.x == -0.5
    assert brake.angular.z == pytest.approx(0.3)


def test_fixed_throttle_ignores_r2_and_keeps_brake_and_steering_live():
    node = make_node()

    released_r2, mode, _ = cycle(
        node,
        joy(steer=-0.4, throttle=1.0, r1=True),
    )
    pressed_r2, _, _ = cycle(
        node,
        joy(steer=0.6, throttle=-1.0, r1=True),
    )
    brake, _, _ = cycle(
        node,
        joy(steer=0.2, brake=0.0, throttle=-1.0, r1=True),
    )

    assert mode == FIXED_THROTTLE_MODE
    assert released_r2.linear.x == pytest.approx(0.30)
    assert pressed_r2.linear.x == pytest.approx(0.30)
    assert pressed_r2.angular.z == pytest.approx(0.6)
    assert brake.linear.x == pytest.approx(-0.5)
    assert brake.angular.z == pytest.approx(0.2)


def test_r1_release_with_l1_held_enters_suppression_next_cycle():
    node = make_node()
    cycle(node, joy(r1=True, l1=True))

    command, mode, _ = cycle(node, joy(l1=True))

    assert mode == TELEOP_SUPPRESS_MODE
    assert command is None


def test_l1_release_resumes_full_brake_next_cycle():
    node = make_node()
    cycle(node, joy(l1=True))

    command, mode, _ = cycle(node, joy(steer=-0.25))

    assert mode == BRAKE_MODE
    assert command.linear.x == -1.0
    assert command.angular.z == pytest.approx(-0.25)


def test_brake_state_republishes_full_brake_and_diagnostics_each_cycle():
    node = make_node()
    node.on_joy(joy(steer=0.1))

    for _ in range(5):
        node.publish_cmd()

    assert len(node.pub.messages) == 5
    assert len(node._setpoint_pub.messages) == 5
    assert len(node._active_mode_pub.messages) == 5
    assert all(message.linear.x == -1.0 for message in node.pub.messages)
    assert all(
        message.angular.z == pytest.approx(0.1)
        for message in node.pub.messages
    )
    assert all(
        message.data == BRAKE_MODE
        for message in node._active_mode_pub.messages
    )


def test_suppression_republishes_diagnostics_but_no_command_each_cycle():
    node = make_node()
    node.on_joy(joy(l1=True))

    for _ in range(5):
        node.publish_cmd()

    assert node.pub.messages == []
    assert len(node._setpoint_pub.messages) == 5
    assert len(node._active_mode_pub.messages) == 5
    assert all(
        message.data == TELEOP_SUPPRESS_MODE
        for message in node._active_mode_pub.messages
    )


def test_x_to_r1_latch_requires_r1_release_and_repress():
    node = make_node()

    fixed, fixed_mode, _ = cycle(node, joy(r1=True))
    manual, manual_mode, _ = cycle(
        node,
        joy(throttle=0.0, x=True, r1=True),
    )
    inhibited, inhibited_mode, _ = cycle(node, joy(r1=True))
    released, released_mode, _ = cycle(node, joy())
    fixed_again, fixed_again_mode, _ = cycle(node, joy(r1=True))

    assert fixed_mode == FIXED_THROTTLE_MODE
    assert fixed.linear.x == pytest.approx(0.30)
    assert manual_mode == MANUAL_MODE
    assert manual.linear.x == pytest.approx(0.50)
    assert inhibited_mode == FIXED_THROTTLE_INHIBITED_MODE
    assert inhibited.linear.x == -1.0
    assert released_mode == BRAKE_MODE
    assert released.linear.x == -1.0
    assert fixed_again_mode == FIXED_THROTTLE_MODE
    assert fixed_again.linear.x == pytest.approx(0.30)


def test_simultaneous_x_and_r1_from_idle_latches_when_x_releases_first():
    node = make_node()

    _, mode, _ = cycle(node, joy(x=True, r1=True))
    command, released_x_mode, _ = cycle(node, joy(r1=True))

    assert mode == MANUAL_MODE
    assert released_x_mode == FIXED_THROTTLE_INHIBITED_MODE
    assert command.linear.x == -1.0


def test_r1_released_first_clears_latch_while_x_stays_manual():
    node = make_node()
    cycle(node, joy(x=True, r1=True))

    command, mode, _ = cycle(node, joy(throttle=0.5, x=True))
    brake, brake_mode, _ = cycle(node, joy())
    fixed, fixed_mode, _ = cycle(node, joy(r1=True))

    assert mode == MANUAL_MODE
    assert command.linear.x == pytest.approx(0.25)
    assert brake_mode == BRAKE_MODE
    assert brake.linear.x == -1.0
    assert fixed_mode == FIXED_THROTTLE_MODE
    assert fixed.linear.x == pytest.approx(0.30)


def test_l1_does_not_bypass_x_to_r1_latch():
    node = make_node()
    cycle(node, joy(r1=True, l1=True))
    cycle(node, joy(x=True, r1=True, l1=True))

    command, mode, _ = cycle(node, joy(r1=True, l1=True))

    assert mode == FIXED_THROTTLE_INHIBITED_MODE
    assert command.linear.x == -1.0


def test_create_share_no_longer_controls_teleop_suppression():
    node = make_node()

    command, mode, _ = cycle(node, joy(create=True))

    assert mode == BRAKE_MODE
    assert command.linear.x == -1.0


def dpad_press(node, direction, **buttons):
    result = cycle(
        node,
        joy(dpad_vertical=float(direction), **buttons),
    )
    cycle(node, joy(dpad_vertical=0.0, **buttons))
    return result


def test_dpad_edges_examples_and_no_repeat_while_held():
    node = make_node()

    cycle(node, joy(dpad_vertical=1.0))
    cycle(node, joy(dpad_vertical=1.0))
    assert node._fixed_throttle_setpoint == 0.31

    cycle(node, joy(dpad_vertical=0.0))
    for _ in range(4):
        dpad_press(node, 1)
    assert node._fixed_throttle_setpoint == 0.35

    dpad_press(node, -1)
    assert node._fixed_throttle_setpoint == 0.34


def test_dpad_clamps_and_simultaneous_opposites_make_no_change():
    node = make_node()

    for _ in range(30):
        dpad_press(node, 1)
    assert node._fixed_throttle_setpoint == 0.50
    dpad_press(node, 1)
    assert node._fixed_throttle_setpoint == 0.50

    for _ in range(60):
        dpad_press(node, -1)
    assert node._fixed_throttle_setpoint == 0.00
    dpad_press(node, -1)
    assert node._fixed_throttle_setpoint == 0.00

    cycle(node, joy(dpad_vertical=0.0))
    assert node._fixed_throttle_setpoint == 0.00


@pytest.mark.parametrize(
    'buttons,expected_mode',
    [
        ({}, BRAKE_MODE),
        ({'x': True}, MANUAL_MODE),
        ({'r1': True}, FIXED_THROTTLE_MODE),
        ({'l1': True}, TELEOP_SUPPRESS_MODE),
    ],
)
def test_dpad_works_in_every_mode_without_directly_commanding_motion(
    buttons,
    expected_mode,
):
    node = make_node()

    command, mode, setpoint = dpad_press(node, 1, **buttons)

    assert mode == expected_mode
    assert setpoint == pytest.approx(0.31)
    if not buttons:
        assert command.linear.x == -1.0
    elif buttons.get('l1'):
        assert command is None


def test_dpad_axis_is_the_recorded_dualsense_vertical_hat_axis():
    assert DPAD_VERTICAL_AXIS_INDEX == 7


def test_r1_rising_edge_logs_setpoint_expected_pulse_and_state():
    node = make_node()

    cycle(node, joy(r1=True))
    cycle(node, joy(r1=True))
    cycle(node, joy())
    cycle(node, joy(x=True, r1=True))

    r1_logs = [
        message for message in node._logger.info_messages
        if message.startswith('R1 fixed-throttle request:')
    ]
    assert len(r1_logs) == 2
    assert 'setpoint=0.300' in r1_logs[0]
    assert 'expected race ESC pulse=1563 us' in r1_logs[0]
    assert 'state=active' in r1_logs[0]
    assert 'state=inhibited_until_r1_release' in r1_logs[1]


def test_setpoint_and_latch_reset_with_new_node_instance():
    first = make_node()
    dpad_press(first, 1)
    cycle(first, joy(x=True, r1=True))
    assert first._fixed_throttle_setpoint == 0.31
    assert first.fixed_throttle_inhibited_until_r1_release

    restarted = make_node()

    assert restarted._fixed_throttle_setpoint == 0.30
    assert not restarted.fixed_throttle_inhibited_until_r1_release


@pytest.mark.parametrize(
    'setpoint,expected',
    [
        (0.00, 1500),
        (0.05, 1550),
        (0.30, 1563),
        (0.32, 1566),
        (0.35, 1569),
        (0.36, 1571),
        (0.40, 1577),
        (0.50, 1594),
    ],
)
def test_expected_race_esc_pulse_matches_motor_mapping(setpoint, expected):
    assert expected_race_esc_pulse_us(setpoint) == expected


@pytest.mark.parametrize(
    'initial,step,minimum,maximum',
    [
        (0.30, 0.0, 0.0, 0.5),
        (0.30, -0.01, 0.0, 0.5),
        (0.30, 0.01, -0.01, 0.5),
        (0.30, 0.01, 0.0, 1.01),
        (0.30, 0.01, 0.5, 0.4),
        (0.60, 0.01, 0.0, 0.5),
        (math.nan, 0.01, 0.0, 0.5),
        (0.30, math.inf, 0.0, 0.5),
    ],
)
def test_invalid_fixed_throttle_parameters_are_rejected(
    initial,
    step,
    minimum,
    maximum,
):
    with pytest.raises(ValueError):
        _validate_fixed_throttle_config(
            initial,
            step,
            minimum,
            maximum,
        )


def test_default_fixed_throttle_configuration_is_valid():
    _validate_fixed_throttle_config(0.30, 0.01, 0.00, 0.50)


def test_active_mode_values_are_explicit_and_stable():
    modes = {
        _active_mode(False, False, False, False),
        _active_mode(True, False, False, False),
        _active_mode(False, True, False, False),
        _active_mode(False, False, True, False),
        _active_mode(False, True, False, True),
    }

    assert modes == {
        'brake',
        'manual',
        'fixed_throttle',
        'teleop_suppress',
        'fixed_throttle_inhibited',
    }
