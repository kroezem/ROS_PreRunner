import math
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from runner_interfaces.msg import KeyboardState
from runner_teleop.teleop_node import _active_mode
from runner_teleop.teleop_node import _shape_manual_command
from runner_teleop.teleop_node import _validate_fixed_throttle_config
from runner_teleop.teleop_node import _validate_manual_trigger_expo
from runner_teleop.teleop_node import BRAKE_MODE
from runner_teleop.teleop_node import DPAD_VERTICAL_AXIS_INDEX
from runner_teleop.teleop_node import expected_race_esc_pulse_us
from runner_teleop.teleop_node import FIXED_THROTTLE_INHIBITED_MODE
from runner_teleop.teleop_node import FIXED_THROTTLE_MODE
from runner_teleop.teleop_node import KEYBOARD_BRAKE_MODE
from runner_teleop.teleop_node import KEYBOARD_DISARMED_MODE
from runner_teleop.teleop_node import KEYBOARD_MOTION_MODE
from runner_teleop.teleop_node import KEYBOARD_SUPPRESS_MODE
from runner_teleop.teleop_node import L1_BUTTON_INDEX
from runner_teleop.teleop_node import MANUAL_MODE
from runner_teleop.teleop_node import R1_BUTTON_INDEX
from runner_teleop.teleop_node import TELEOP_SUPPRESS_MODE
from runner_teleop.teleop_node import TeleopNode
from runner_teleop.teleop_node import X_BUTTON_INDEX
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String


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
    node._axis_reverse = 2
    node._axis_throttle = 5
    node._deadman_button = X_BUTTON_INDEX
    node._manual_trigger_expo = 0.50
    node._steer = 0.0
    node._manual_cmd = 0.0
    node._throttle = 0.0
    node._reverse = 0.0
    node._manual_held = False
    node._fixed_throttle_held = False
    node._teleop_suppress_held = False
    node.fixed_throttle_inhibited_until_r1_release = False
    node._fixed_throttle_setpoint = 0.30
    node._fixed_throttle_step = 0.01
    node._fixed_throttle_min_setpoint = 0.00
    node._fixed_throttle_max_setpoint = 0.50
    node._dpad_press_active = False
    node._controller_timeout = 0.15
    node._keyboard_state_timeout = 0.15
    node._last_joy_at = None
    node._controller_live = False
    node._last_keyboard_state_at = None
    node._keyboard_valid = False
    node._keyboard_mode = KeyboardState.MODE_DRIVE
    node._keyboard_throttle = 0.0
    node._keyboard_steer = 0.0
    node._keyboard_motion_requested = False
    node._keyboard_motion_previous = False
    node._keyboard_motion_ready = False
    node._keyboard_motion_armed = False
    node._keyboard_suppress_requested = False
    node._keyboard_suppress_previous = False
    node._keyboard_suppress_ready = False
    node._keyboard_suppress_armed = False
    node._keyboard_suppress_rearm_ready = True
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


def keyboard_state(
    *,
    valid=True,
    mode=KeyboardState.MODE_DRIVE,
    throttle=0.0,
    steering=0.0,
    session_id=1,
    sequence=1,
):
    message = KeyboardState()
    message.valid = valid
    message.mode = mode
    message.throttle = throttle
    message.steering = steering
    message.session_id = session_id
    message.sequence = sequence
    return message


def keyboard_cycle(node, message):
    command_count = len(node.pub.messages)
    node.on_keyboard(message)
    node.publish_cmd()
    command = (
        node.pub.messages[-1]
        if len(node.pub.messages) > command_count
        else None
    )
    return command, node._active_mode_pub.messages[-1].data


@pytest.mark.parametrize(
    'x,r1,l1,expected_mode,expected_command',
    [
        (False, False, False, BRAKE_MODE, 0.0),
        (True, False, False, MANUAL_MODE, 0.5859375),
        (False, True, False, FIXED_THROTTLE_MODE, 0.30),
        (False, False, True, TELEOP_SUPPRESS_MODE, None),
        (True, True, False, MANUAL_MODE, 0.5859375),
        (True, False, True, MANUAL_MODE, 0.5859375),
        (False, True, True, FIXED_THROTTLE_MODE, 0.30),
        (True, True, True, MANUAL_MODE, 0.5859375),
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


def test_manual_r2_forward_l2_reverse_and_priority_use_expo():
    node = make_node()

    throttle, mode, _ = cycle(
        node,
        joy(steer=-0.6, throttle=0.5, x=True, r1=True, l1=True),
    )
    brake, _, _ = cycle(
        node,
        joy(steer=0.3, brake=0.0, throttle=-1.0, x=True),
    )
    neutral, _, _ = cycle(node, joy(x=True))
    deadman_released, _, _ = cycle(node, joy(throttle=-1.0))

    assert mode == MANUAL_MODE
    assert throttle.linear.x == pytest.approx(0.1328125)
    assert throttle.angular.z == pytest.approx(-0.6)
    assert brake.linear.x == pytest.approx(-0.3125)
    assert brake.angular.z == pytest.approx(0.3)
    assert neutral.linear.x == 0.0
    assert deadman_released.linear.x == 0.0


def test_manual_trigger_expo_endpoints_monotonicity_and_sign():
    inputs = [index / 100.0 for index in range(101)]
    forwards = [
        _shape_manual_command(value, 0.50) for value in inputs
    ]
    reverses = [
        _shape_manual_command(-value, 0.50) for value in inputs
    ]

    assert forwards[0] == 0.0
    assert forwards[-1] == 1.0
    assert reverses[0] == 0.0
    assert reverses[-1] == -1.0
    assert all(left < right for left, right in zip(forwards, forwards[1:]))
    assert all(left > right for left, right in zip(reverses, reverses[1:]))
    assert all(value > 0.0 for value in forwards[1:])
    assert all(value < 0.0 for value in reverses[1:])


@pytest.mark.parametrize(
    'magnitude,expected',
    [
        (0.25, 0.1328125),
        (0.50, 0.3125),
        (0.75, 0.5859375),
    ],
)
def test_default_manual_trigger_expo_examples(magnitude, expected):
    assert _shape_manual_command(magnitude, 0.50) == pytest.approx(expected)


def test_fixed_throttle_and_keyboard_bypass_manual_trigger_expo():
    node = make_node()
    node._manual_trigger_expo = 1.0

    fixed_forward, fixed_mode, _ = cycle(
        node, joy(throttle=0.0, r1=True)
    )
    fixed_reverse, _, _ = cycle(
        node, joy(brake=0.0, r1=True)
    )
    cycle(node, joy())
    keyboard_cycle(node, keyboard_state())
    keyboard, keyboard_mode = keyboard_cycle(
        node, keyboard_state(throttle=0.30, sequence=2)
    )

    assert fixed_mode == FIXED_THROTTLE_MODE
    assert fixed_forward.linear.x == pytest.approx(0.30)
    assert fixed_reverse.linear.x == pytest.approx(-0.30)
    assert keyboard_mode == KEYBOARD_MOTION_MODE
    assert keyboard.linear.x == pytest.approx(0.30)


def test_fixed_throttle_is_symmetric_depth_independent_and_l2_has_priority():
    node = make_node()

    released, mode, _ = cycle(
        node,
        joy(steer=-0.4, throttle=1.0, r1=True),
    )
    shallow_r2, _, _ = cycle(
        node,
        joy(steer=0.6, throttle=0.0, r1=True),
    )
    full_r2, _, _ = cycle(
        node,
        joy(steer=0.6, throttle=-1.0, r1=True),
    )
    shallow_l2, _, _ = cycle(
        node,
        joy(steer=0.2, brake=0.0, r1=True),
    )
    full_l2, _, _ = cycle(
        node,
        joy(steer=0.2, brake=-1.0, r1=True),
    )
    both, _, _ = cycle(
        node,
        joy(brake=0.0, throttle=-1.0, r1=True),
    )

    assert mode == FIXED_THROTTLE_MODE
    assert released.linear.x == 0.0
    assert shallow_r2.linear.x == pytest.approx(0.30)
    assert full_r2.linear.x == pytest.approx(0.30)
    assert shallow_r2.angular.z == pytest.approx(0.6)
    assert shallow_l2.linear.x == pytest.approx(-0.30)
    assert full_l2.linear.x == pytest.approx(-0.30)
    assert shallow_l2.angular.z == pytest.approx(0.2)
    assert both.linear.x == pytest.approx(-0.30)


def test_controller_timeout_releases_deadman_to_zero():
    node = make_node()
    moving, mode, _ = cycle(node, joy(throttle=-1.0, x=True))
    assert mode == MANUAL_MODE
    assert moving.linear.x == 1.0

    node._last_joy_at -= node._controller_timeout + 0.01
    node.publish_cmd()

    assert node._active_mode_pub.messages[-1].data == BRAKE_MODE
    assert node.pub.messages[-1].linear.x == 0.0


def test_r1_release_with_l1_held_enters_suppression_next_cycle():
    node = make_node()
    cycle(node, joy(r1=True, l1=True))

    command, mode, _ = cycle(node, joy(l1=True))

    assert mode == TELEOP_SUPPRESS_MODE
    assert command is None


def test_l1_release_resumes_zero_brake_next_cycle():
    node = make_node()
    cycle(node, joy(l1=True))

    command, mode, _ = cycle(node, joy(steer=-0.25))

    assert mode == BRAKE_MODE
    assert command.linear.x == 0.0
    assert command.angular.z == pytest.approx(-0.25)


def test_brake_state_republishes_zero_and_diagnostics_each_cycle():
    node = make_node()
    node.on_joy(joy(steer=0.1))

    for _ in range(5):
        node.publish_cmd()

    assert len(node.pub.messages) == 5
    assert len(node._setpoint_pub.messages) == 5
    assert len(node._active_mode_pub.messages) == 5
    assert all(message.linear.x == 0.0 for message in node.pub.messages)
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

    fixed, fixed_mode, _ = cycle(node, joy(throttle=-1.0, r1=True))
    manual, manual_mode, _ = cycle(
        node,
        joy(throttle=0.0, x=True, r1=True),
    )
    inhibited, inhibited_mode, _ = cycle(node, joy(r1=True))
    released, released_mode, _ = cycle(node, joy())
    fixed_again, fixed_again_mode, _ = cycle(
        node, joy(throttle=-1.0, r1=True)
    )

    assert fixed_mode == FIXED_THROTTLE_MODE
    assert fixed.linear.x == pytest.approx(0.30)
    assert manual_mode == MANUAL_MODE
    assert manual.linear.x == pytest.approx(0.3125)
    assert inhibited_mode == FIXED_THROTTLE_INHIBITED_MODE
    assert inhibited.linear.x == 0.0
    assert released_mode == BRAKE_MODE
    assert released.linear.x == 0.0
    assert fixed_again_mode == FIXED_THROTTLE_MODE
    assert fixed_again.linear.x == pytest.approx(0.30)


def test_simultaneous_x_and_r1_from_idle_latches_when_x_releases_first():
    node = make_node()

    _, mode, _ = cycle(node, joy(x=True, r1=True))
    command, released_x_mode, _ = cycle(node, joy(r1=True))

    assert mode == MANUAL_MODE
    assert released_x_mode == FIXED_THROTTLE_INHIBITED_MODE
    assert command.linear.x == 0.0


def test_r1_released_first_clears_latch_while_x_stays_manual():
    node = make_node()
    cycle(node, joy(x=True, r1=True))

    command, mode, _ = cycle(node, joy(throttle=0.5, x=True))
    brake, brake_mode, _ = cycle(node, joy())
    fixed, fixed_mode, _ = cycle(node, joy(throttle=-1.0, r1=True))

    assert mode == MANUAL_MODE
    assert command.linear.x == pytest.approx(0.1328125)
    assert brake_mode == BRAKE_MODE
    assert brake.linear.x == 0.0
    assert fixed_mode == FIXED_THROTTLE_MODE
    assert fixed.linear.x == pytest.approx(0.30)


def test_l1_does_not_bypass_x_to_r1_latch():
    node = make_node()
    cycle(node, joy(r1=True, l1=True))
    cycle(node, joy(x=True, r1=True, l1=True))

    command, mode, _ = cycle(node, joy(r1=True, l1=True))

    assert mode == FIXED_THROTTLE_INHIBITED_MODE
    assert command.linear.x == 0.0


def test_create_share_no_longer_controls_teleop_suppression():
    node = make_node()

    command, mode, _ = cycle(node, joy(create=True))

    assert mode == BRAKE_MODE
    assert command.linear.x == 0.0


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


def test_dpad_adjusts_fixed_throttle_symmetrically():
    node = make_node()
    dpad_press(node, 1)

    forward, _, forward_setpoint = cycle(
        node, joy(throttle=-1.0, r1=True)
    )
    reverse, _, reverse_setpoint = cycle(
        node, joy(brake=-1.0, r1=True)
    )

    assert forward_setpoint == pytest.approx(0.31)
    assert reverse_setpoint == pytest.approx(0.31)
    assert forward.linear.x == pytest.approx(0.31)
    assert reverse.linear.x == pytest.approx(-0.31)


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
    if not buttons or buttons.get('r1'):
        assert command.linear.x == 0.0
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


@pytest.mark.parametrize('expo', [-0.01, 1.01, math.nan, math.inf, 1])
def test_invalid_manual_trigger_expo_is_rejected(expo):
    with pytest.raises(ValueError):
        _validate_manual_trigger_expo(expo)


def test_default_manual_trigger_expo_is_valid():
    _validate_manual_trigger_expo(0.50)


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


def test_keyboard_motion_brake_steering_and_suppression_priority():
    node = make_node()
    keyboard_cycle(node, keyboard_state())

    forward, mode = keyboard_cycle(
        node,
        keyboard_state(throttle=0.30, sequence=2),
    )
    brake, brake_mode = keyboard_cycle(
        node,
        keyboard_state(
            mode=KeyboardState.MODE_BRAKE_SUPPRESS,
            throttle=0.30,
            steering=-1.0,
            sequence=3,
        ),
    )
    keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_DRIVE, sequence=4),
    )
    suppressed, suppress_mode = keyboard_cycle(
        node,
        keyboard_state(
            mode=KeyboardState.MODE_SUPPRESS,
            throttle=0.30,
            sequence=5,
        ),
    )

    assert mode == KEYBOARD_MOTION_MODE
    assert forward.linear.x == pytest.approx(0.30)
    assert brake_mode == KEYBOARD_BRAKE_MODE
    assert brake.linear.x == 0.0
    assert brake.angular.z == -1.0
    assert suppress_mode == KEYBOARD_SUPPRESS_MODE
    assert suppressed is None


def test_keyboard_reverse_publishes_negative_cmd_vel_teleop():
    node = make_node()
    keyboard_cycle(node, keyboard_state())

    reverse, mode = keyboard_cycle(
        node,
        keyboard_state(throttle=-0.30, sequence=2),
    )

    assert mode == KEYBOARD_MOTION_MODE
    assert reverse.linear.x == pytest.approx(-0.30)


@pytest.mark.parametrize(
    'steering',
    [-1.0, 0.0, 1.0],
)
def test_keyboard_neutral_steering_keeps_zero_brake(steering):
    node = make_node()

    command, mode = keyboard_cycle(
        node,
        keyboard_state(steering=steering),
    )

    assert mode == BRAKE_MODE
    assert command.linear.x == 0.0
    assert command.angular.z == steering


@pytest.mark.parametrize(
    'buttons',
    [
        {'x': True},
        {'r1': True},
        {'l1': True},
    ],
)
def test_controller_preempts_and_held_w_requires_release_repress(buttons):
    node = make_node()
    keyboard_cycle(node, keyboard_state())
    keyboard_cycle(node, keyboard_state(throttle=0.30, sequence=2))

    preempted, _, _ = cycle(node, joy(throttle=0.0, **buttons))
    if buttons.get('l1'):
        assert preempted is None
    else:
        assert preempted is not None

    command, mode, _ = cycle(node, joy())
    assert mode == KEYBOARD_DISARMED_MODE
    assert command.linear.x == 0.0

    still_held, held_mode = keyboard_cycle(
        node,
        keyboard_state(throttle=0.30, sequence=3),
    )
    assert held_mode == KEYBOARD_DISARMED_MODE
    assert still_held.linear.x == 0.0

    keyboard_cycle(node, keyboard_state(sequence=4))
    resumed, resumed_mode = keyboard_cycle(
        node,
        keyboard_state(throttle=0.30, sequence=5),
    )
    assert resumed_mode == KEYBOARD_MOTION_MODE
    assert resumed.linear.x == pytest.approx(0.30)


@pytest.mark.parametrize(
    'buttons',
    [
        {'x': True},
        {'r1': True},
        {'l1': True},
    ],
)
def test_each_controller_clear_prevents_silent_autonomy_resume(buttons):
    node = make_node()
    keyboard_cycle(node, keyboard_state())
    keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_SUPPRESS, sequence=2),
    )

    cycle(node, joy(**buttons))

    command, mode, _ = cycle(node, joy())
    assert command.linear.x == 0.0
    assert mode == KEYBOARD_DISARMED_MODE

    command, mode = keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_SUPPRESS, sequence=3),
    )
    assert command.linear.x == 0.0
    assert mode == KEYBOARD_DISARMED_MODE

    keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_BRAKE, sequence=4),
    )
    command, mode = keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_SUPPRESS, sequence=5),
    )
    assert command is None
    assert mode == TELEOP_SUPPRESS_MODE


def test_sender_timeout_preserves_armed_autonomy_suppression(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(
        'runner_teleop.teleop_node.time.monotonic',
        lambda: now[0],
    )
    node = make_node()
    keyboard_cycle(node, keyboard_state())
    keyboard_cycle(
        node,
        keyboard_state(
            mode=KeyboardState.MODE_SUPPRESS,
            throttle=0.30,
            sequence=2,
        ),
    )

    command, mode = keyboard_cycle(
        node,
        keyboard_state(
            valid=False,
            mode=KeyboardState.MODE_SUPPRESS,
            throttle=0.30,
            sequence=3,
        ),
    )
    assert command is None
    assert mode == TELEOP_SUPPRESS_MODE
    now[0] += 0.151
    node.publish_cmd()
    assert node._active_mode_pub.messages[-1].data == TELEOP_SUPPRESS_MODE

    brake, brake_mode = keyboard_cycle(
        node,
        keyboard_state(
            valid=False,
            mode=KeyboardState.MODE_BRAKE,
            sequence=4,
        ),
    )
    assert brake_mode == BRAKE_MODE
    assert brake.linear.x == 0.0


def test_first_and_repeated_disarm_messages_publish_brake_immediately():
    node = make_node()
    keyboard_cycle(
        node,
        keyboard_state(mode=KeyboardState.MODE_SUPPRESS, sequence=1),
    )
    before = len(node.pub.messages)

    node.on_keyboard(
        keyboard_state(mode=KeyboardState.MODE_BRAKE, sequence=2)
    )
    assert len(node.pub.messages) == before + 1
    assert node.pub.messages[-1].linear.x == 0.0
    assert not node._keyboard_suppress_armed

    for sequence in (2, 2, 3):
        node.on_keyboard(
            keyboard_state(mode=KeyboardState.MODE_BRAKE, sequence=sequence)
        )
        assert node.pub.messages[-1].linear.x == 0.0
        assert not node._keyboard_suppress_armed


@pytest.mark.parametrize(
    'buttons',
    [
        {'x': True},
        {'r1': True},
        {'l1': True},
    ],
)
def test_stale_controller_state_expires_and_does_not_rearm_w(
    monkeypatch,
    buttons,
):
    now = [20.0]
    monkeypatch.setattr(
        'runner_teleop.teleop_node.time.monotonic',
        lambda: now[0],
    )
    node = make_node()
    keyboard_cycle(node, keyboard_state())
    keyboard_cycle(node, keyboard_state(throttle=0.30, sequence=2))
    cycle(node, joy(throttle=0.0, **buttons))

    now[0] += 0.151
    node.on_keyboard(keyboard_state(throttle=0.30, sequence=3))
    node.publish_cmd()

    assert not node._manual_held
    assert not node._fixed_throttle_held
    assert not node._teleop_suppress_held
    assert node._active_mode_pub.messages[-1].data == KEYBOARD_DISARMED_MODE
    assert node.pub.messages[-1].linear.x == 0.0


@pytest.mark.parametrize(
    'throttle,steering,mode',
    [
        (math.nan, 0.0, KeyboardState.MODE_DRIVE),
        (0.0, math.inf, KeyboardState.MODE_DRIVE),
        (1.01, 0.0, KeyboardState.MODE_DRIVE),
        (0.0, -1.01, KeyboardState.MODE_DRIVE),
        (0.0, 0.0, 4),
    ],
)
def test_teleop_defensively_invalidates_bad_keyboard_state(
    throttle,
    steering,
    mode,
):
    node = make_node()

    command, active_mode = keyboard_cycle(
        node,
        keyboard_state(
            throttle=throttle,
            steering=steering,
            mode=mode,
        ),
    )

    assert active_mode == BRAKE_MODE
    assert command.linear.x == 0.0
    assert not node._keyboard_valid


def _spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def test_graph_topic_contract_rate_suppression_and_diagnostics():
    rclpy.init()
    teleop = TeleopNode()
    probe = Node('teleop_stage2_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(teleop)
    executor.add_node(probe)

    commands = []
    command_times = []
    modes = []
    setpoints = []

    def on_command(message):
        commands.append(message)
        command_times.append(time.monotonic())

    probe.create_subscription(
        Twist, '/cmd_vel_teleop', on_command, 10
    )
    probe.create_subscription(
        String, '/teleop/active_mode', modes.append, 10
    )
    probe.create_subscription(
        Float32,
        '/teleop/fixed_throttle_setpoint',
        setpoints.append,
        10,
    )
    joy_pub = probe.create_publisher(Joy, '/joy', 10)

    try:
        _spin_for(executor, 0.32)
        assert len(commands) >= 5
        assert all(command.linear.x == 0.0 for command in commands)
        intervals = [
            later - earlier
            for earlier, later in zip(command_times, command_times[1:])
        ]
        assert intervals
        assert sum(intervals) / len(intervals) == pytest.approx(
            0.05, abs=0.015
        )
        assert modes
        assert setpoints

        teleop_publishers = probe.get_publishers_info_by_topic(
            '/cmd_vel_teleop'
        )
        assert sum(
            endpoint.node_name == 'runner_teleop'
            for endpoint in teleop_publishers
        ) == 1
        assert not any(
            endpoint.node_name == 'runner_teleop'
            for endpoint in probe.get_publishers_info_by_topic('/cmd_vel')
        )

        joy_pub.publish(joy(l1=True))
        _spin_for(executor, 0.08)
        commands.clear()
        mode_count = len(modes)
        setpoint_count = len(setpoints)
        deadline = time.monotonic() + 0.20
        while time.monotonic() < deadline:
            joy_pub.publish(joy(l1=True))
            _spin_for(executor, 0.04)
        assert commands == []
        assert len(modes) > mode_count
        assert len(setpoints) > setpoint_count
        assert modes[-1].data == TELEOP_SUPPRESS_MODE
    finally:
        executor.remove_node(probe)
        executor.remove_node(teleop)
        probe.destroy_node()
        teleop.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
