import errno
import math

from geometry_msgs.msg import Twist
import pytest
import rclpy
from runner_interfaces.msg import EncoderState

from runner_motor import motor_node


class FakePWM:
    instances = []
    events = []
    initial_states = {}
    period_readbacks = {}

    def __init__(self, chip_path, channel):
        self.chip_path = chip_path
        self.channel = channel
        state = self.initial_states.get(channel, {})
        self.period_ns = state.get('period_ns', 0)
        self.duty_cycle_ns = state.get('duty_cycle_ns', 0)
        self.polarity = state.get('polarity', 'normal')
        self.enabled = state.get('enabled', False)
        self.__class__.instances.append(self)

    def get_period_ns(self):
        return self.period_readbacks.get(self.channel, self.period_ns)

    def get_duty_cycle_ns(self):
        return self.duty_cycle_ns

    def get_polarity(self):
        return self.polarity

    def is_enabled(self):
        return self.enabled

    def set_period_ns(self, value):
        if value <= 0 or self.duty_cycle_ns > value:
            raise OSError(errno.EINVAL, 'Invalid argument')
        self.period_ns = value
        self.events.append((self.channel, 'period', value))

    def set_duty_cycle_ns(self, value):
        if self.period_ns == 0 or not 0 <= value <= self.period_ns:
            raise OSError(errno.EINVAL, 'Invalid argument')
        self.duty_cycle_ns = value
        self.events.append((self.channel, 'duty', value))

    def set_polarity(self, value):
        if self.period_ns == 0 or self.enabled:
            raise OSError(errno.EINVAL, 'Invalid argument')
        self.polarity = value
        self.events.append((self.channel, 'polarity', value))

    def enable(self):
        if self.period_ns == 0 or self.duty_cycle_ns > self.period_ns:
            raise OSError(errno.EINVAL, 'Invalid argument')
        self.enabled = True
        self.events.append((self.channel, 'enable', 1))

    def disable(self):
        if self.period_ns == 0 or self.duty_cycle_ns > self.period_ns:
            raise OSError(errno.EINVAL, 'Invalid argument')
        self.enabled = False
        self.events.append((self.channel, 'enable', 0))


class FakeLine:
    def __init__(self):
        self.values = [motor_node.DIR_FORWARD]
        self.released = False

    def set_value(self, value):
        self.values.append(value)
        FakePWM.events.append(('dir', value))

    def release(self):
        self.released = True


class FakeChip:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


@pytest.fixture
def motor_factory(monkeypatch):
    nodes = []
    FakePWM.instances = []
    FakePWM.events = []
    FakePWM.initial_states = {}
    FakePWM.period_readbacks = {}
    line = FakeLine()
    chip = FakeChip()
    monkeypatch.setattr(motor_node, 'SysfsPWM', FakePWM)

    def request_direction_output():
        FakePWM.events.append(('dir', motor_node.DIR_FORWARD))
        return chip, line

    monkeypatch.setattr(
        motor_node, 'request_direction_output', request_direction_output
    )

    def create(initial_states=None, period_readbacks=None):
        FakePWM.initial_states = initial_states or {}
        FakePWM.period_readbacks = period_readbacks or {}
        rclpy.init()
        node = motor_node.MotorNode()
        nodes.append(node)
        return node

    yield create, line, chip

    for node in nodes:
        node.stop()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.mark.parametrize(
    ('command', 'direction', 'duty_ns'),
    [
        (-2.0, -1, 50_000),
        (-0.5, -1, 25_000),
        (0.0, 0, 0),
        (0.25, 1, 12_500),
        (2.0, 1, 50_000),
        (math.nan, 0, 0),
        (math.inf, 0, 0),
        (-math.inf, 0, 0),
    ],
)
def test_signed_command_mapping(command, direction, duty_ns):
    assert motor_node.map_motor_command(command) == (direction, duty_ns)


def test_sysfs_pwm_writes_attributes_without_unexport(tmp_path):
    channel_path = tmp_path / 'pwm0'
    channel_path.mkdir()
    for attribute in ('period', 'duty_cycle', 'polarity', 'enable'):
        (channel_path / attribute).touch()

    pwm = motor_node.SysfsPWM(tmp_path, 0)
    pwm.set_period_ns(50_000)
    pwm.set_duty_cycle_ns(25_000)
    pwm.set_polarity('normal')
    pwm.enable()

    assert (channel_path / 'period').read_text() == '50000'
    assert (channel_path / 'duty_cycle').read_text() == '25000'
    assert (channel_path / 'polarity').read_text() == 'normal'
    assert (channel_path / 'enable').read_text() == '1'
    assert not (tmp_path / 'unexport').exists()


def test_sysfs_pwm_retries_attributes_becoming_present(tmp_path, monkeypatch):
    pwm = motor_node.SysfsPWM(tmp_path, 0)
    sleeps = []

    def make_attribute(_delay):
        sleeps.append(_delay)
        channel_path = tmp_path / 'pwm0'
        channel_path.mkdir()
        (channel_path / 'period').write_text('50000')

    monkeypatch.setattr(motor_node.time, 'sleep', make_attribute)

    assert pwm.get_period_ns() == 50_000
    assert sleeps == [motor_node.PWM_IO_RETRY_S]


def test_sysfs_pwm_retries_attribute_becoming_writable(
    tmp_path, monkeypatch
):
    channel_path = tmp_path / 'pwm0'
    channel_path.mkdir()
    period_path = channel_path / 'period'
    period_path.touch()
    real_open = type(period_path).open
    attempts = []

    def delayed_open(path, *args, **kwargs):
        if path == period_path and args == ('w',) and not attempts:
            attempts.append(path)
            raise PermissionError(errno.EACCES, 'Permission denied', path)
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr(type(period_path), 'open', delayed_open)
    monkeypatch.setattr(motor_node.time, 'sleep', lambda _delay: None)

    motor_node.SysfsPWM(tmp_path, 0).set_period_ns(50_000)

    assert period_path.read_text() == '50000'
    assert attempts == [period_path]


def test_direction_gpio_is_requested_exclusively_by_chip_label():
    requests = []

    class Line:
        def request(self, **kwargs):
            requests.append(kwargs)

    class Chip:
        OPEN_BY_LABEL = object()

        def __init__(self, identifier, how):
            requests.append(('chip', identifier, how))

        def get_line(self, offset):
            requests.append(('line', offset))
            return Line()

    class Gpiod:
        pass

    Gpiod.Chip = Chip
    Gpiod.LINE_REQ_DIR_OUT = object()

    chip, _line = motor_node.request_direction_output(Gpiod)

    assert isinstance(chip, Chip)
    assert requests == [
        ('chip', motor_node.GPIO_CHIP_LABEL, Chip.OPEN_BY_LABEL),
        ('line', motor_node.MOTOR_DIR_GPIO),
        {
            'consumer': 'runner_motor_dir',
            'type': Gpiod.LINE_REQ_DIR_OUT,
            'default_vals': [motor_node.DIR_FORWARD],
        },
    ]


def test_fresh_pwm_rejects_duty_before_period():
    FakePWM.instances = []
    FakePWM.events = []
    FakePWM.initial_states = {}
    FakePWM.period_readbacks = {}
    pwm = FakePWM(motor_node.PWM_CHIP_PATH, motor_node.MOTOR_PWM_CHANNEL)

    with pytest.raises(OSError) as error:
        pwm.set_duty_cycle_ns(0)

    assert error.value.errno == errno.EINVAL
    assert FakePWM.events == []


def test_fresh_pwm_startup_configuration_and_order(motor_factory):
    create, _line, _chip = motor_factory
    create()
    motor, servo = FakePWM.instances

    assert motor.period_ns == motor_node.MOTOR_PERIOD_NS
    assert motor.duty_cycle_ns == 0
    assert motor.polarity == 'normal'
    assert motor.enabled
    assert servo.period_ns == motor_node.SERVO_PERIOD_NS
    assert servo.duty_cycle_ns == motor_node.us_to_ns(motor_node.STEER_CTR)
    assert servo.enabled

    motor_events = [
        event for event in FakePWM.events
        if event[0] in (motor_node.MOTOR_PWM_CHANNEL, 'dir')
    ]
    assert motor_events == [
        (0, 'period', motor_node.MOTOR_PERIOD_NS),
        (0, 'duty', 0),
        ('dir', motor_node.DIR_FORWARD),
        (0, 'enable', 1),
    ]
    servo_events = [
        event for event in FakePWM.events
        if event[0] == motor_node.SERVO_PWM_CHANNEL
    ]
    assert servo_events == [
        (1, 'period', motor_node.SERVO_PERIOD_NS),
        (1, 'duty', motor_node.us_to_ns(motor_node.STEER_CTR)),
        (1, 'enable', 1),
    ]


def test_configured_disabled_pwm_startup_is_state_safe(motor_factory):
    create, _line, _chip = motor_factory
    create({
        0: {
            'period_ns': 100_000,
            'duty_cycle_ns': 75_000,
            'polarity': 'normal',
            'enabled': False,
        },
        1: {
            'period_ns': 25_000_000,
            'duty_cycle_ns': 21_000_000,
            'polarity': 'inversed',
            'enabled': False,
        },
    })

    assert _motor_events() == [
        (0, 'duty', 0),
        (0, 'period', motor_node.MOTOR_PERIOD_NS),
        (0, 'duty', 0),
        ('dir', motor_node.DIR_FORWARD),
        (0, 'enable', 1),
    ]
    assert [event for event in FakePWM.events if event[0] == 1] == [
        (1, 'polarity', 'normal'),
        (1, 'duty', 0),
        (1, 'period', motor_node.SERVO_PERIOD_NS),
        (1, 'duty', motor_node.us_to_ns(motor_node.STEER_CTR)),
        (1, 'enable', 1),
    ]


def test_hot_enabled_recovery_zeros_duty_first_and_stays_enabled(
    motor_factory,
):
    create, _line, _chip = motor_factory
    create({
        0: {
            'period_ns': 100_000,
            'duty_cycle_ns': 75_000,
            'polarity': 'normal',
            'enabled': True,
        },
        1: {
            'period_ns': 25_000_000,
            'duty_cycle_ns': 21_000_000,
            'polarity': 'normal',
            'enabled': True,
        },
    })

    motor_events = [event for event in FakePWM.events if event[0] == 0]
    servo_events = [event for event in FakePWM.events if event[0] == 1]
    assert motor_events == [
        (0, 'duty', 0),
        (0, 'period', motor_node.MOTOR_PERIOD_NS),
        (0, 'duty', 0),
    ]
    assert servo_events == [
        (1, 'duty', 0),
        (1, 'period', motor_node.SERVO_PERIOD_NS),
        (1, 'duty', motor_node.us_to_ns(motor_node.STEER_CTR)),
    ]
    assert all(pwm.enabled for pwm in FakePWM.instances)


def test_hot_enabled_unexpected_polarity_fails_without_disabling():
    FakePWM.instances = []
    FakePWM.events = []
    FakePWM.initial_states = {
        0: {
            'period_ns': 50_000,
            'duty_cycle_ns': 25_000,
            'polarity': 'inversed',
            'enabled': True,
        },
    }
    FakePWM.period_readbacks = {}
    pwm = FakePWM(motor_node.PWM_CHIP_PATH, 0)

    with pytest.raises(RuntimeError, match='unexpected polarity'):
        motor_node.initialize_pwm(pwm, motor_node.MOTOR_PERIOD_NS, 0)

    assert FakePWM.events == [(0, 'duty', 0)]
    assert pwm.enabled


def test_startup_fails_on_incorrect_period_readback(motor_factory):
    create, _line, _chip = motor_factory

    with pytest.raises(RuntimeError, match='PWM period read-back failed'):
        create(period_readbacks={0: motor_node.MOTOR_PERIOD_NS + 1})


def _command(node, linear_x):
    message = Twist()
    message.linear.x = linear_x
    node.on_cmd(message)


def _stationary(node, value=True):
    node._on_encoder_state(EncoderState(stationary=value))


def _motor_events():
    return [
        event for event in FakePWM.events
        if event[0] in (motor_node.MOTOR_PWM_CHANNEL, 'dir')
    ]


def test_opposite_command_brakes_while_encoder_reports_moving(motor_factory):
    create, line, _chip = motor_factory
    node = create()
    FakePWM.events = []

    _command(node, -0.5)
    _stationary(node, False)

    assert _motor_events() == [(0, 'duty', 0)]
    assert line.values == [motor_node.DIR_FORWARD]
    assert node._pending_reversal == (motor_node.DIR_REVERSE, 25_000)


def test_only_post_request_stationary_sample_authorizes_reversal(motor_factory):
    create, line, _chip = motor_factory
    node = create()

    _stationary(node)
    FakePWM.events = []
    _command(node, -0.5)
    assert line.values == [motor_node.DIR_FORWARD]

    _stationary(node)

    assert _motor_events() == [
        (0, 'duty', 0),
        (0, 'duty', 0),
        ('dir', motor_node.DIR_REVERSE),
        (0, 'duty', 25_000),
    ]
    assert line.values[-1] == motor_node.DIR_REVERSE
    assert node._pending_reversal is None


def test_zero_cancels_pending_reversal(motor_factory):
    create, line, _chip = motor_factory
    node = create()

    _command(node, -0.5)
    _command(node, 0.0)
    _stationary(node)

    assert line.values == [motor_node.DIR_FORWARD]
    assert node._pending_reversal is None
    assert node.motor.duty_cycle_ns == 0
    assert node._direction_latch == -1


def test_repeated_pending_commands_apply_latest_duty(motor_factory):
    create, line, _chip = motor_factory
    node = create()

    _command(node, -0.2)
    _command(node, -0.7)
    assert node._pending_reversal == (motor_node.DIR_REVERSE, 35_000)

    _stationary(node)

    assert line.values[-1] == motor_node.DIR_REVERSE
    assert node.motor.duty_cycle_ns == 35_000


def test_reversal_gate_operates_in_both_directions(motor_factory):
    create, line, _chip = motor_factory
    node = create()

    _command(node, -0.4)
    _stationary(node)
    assert line.values[-1] == motor_node.DIR_REVERSE
    assert node.motor.duty_cycle_ns == 20_000

    _command(node, 0.6)
    _stationary(node, False)
    assert line.values[-1] == motor_node.DIR_REVERSE
    assert node.motor.duty_cycle_ns == 0

    _stationary(node)
    assert line.values[-1] == motor_node.DIR_FORWARD
    assert node.motor.duty_cycle_ns == 30_000


def test_same_hardware_direction_command_applies_immediately(motor_factory):
    create, line, _chip = motor_factory
    node = create()

    _command(node, 0.5)

    assert line.values == [motor_node.DIR_FORWARD]
    assert node.motor.duty_cycle_ns == 25_000
    assert node._pending_reversal is None


@pytest.mark.parametrize(
    ('command', 'expected_direction', 'expected_duty'),
    [(-1.0, -1, 0), (0.0, 0, 0), (0.5, 1, 25_000)],
)
def test_command_publishes_direction(
    motor_factory, command, expected_direction, expected_duty
):
    create, _line, _chip = motor_factory
    node = create()
    direction_messages = []
    node._direction_pub.publish = direction_messages.append

    message = Twist()
    message.linear.x = command
    node.on_cmd(message)

    assert direction_messages[-1].data == expected_direction
    assert node.motor.duty_cycle_ns == expected_duty


def test_direction_telemetry_stays_latched_through_zero_watchdog_and_shutdown(
    motor_factory,
):
    create, _line, _chip = motor_factory
    node = create()
    direction_messages = []
    node._direction_pub.publish = direction_messages.append

    _command(node, -0.5)
    _command(node, 0.0)
    assert [message.data for message in direction_messages] == [-1, -1]

    node._last_cmd -= motor_node.CMD_TIMEOUT_S + 1.0
    warnings = []
    node.get_logger().warn = warnings.append
    node._watchdog()

    assert node.motor.duty_cycle_ns == 0
    assert direction_messages[-1].data == -1
    assert warnings == [
        '/cmd_vel watchdog timeout; motor duty=0 (active brake)'
    ]

    node.stop()

    assert node.motor.duty_cycle_ns == 0
    assert node.motor.enabled
    assert direction_messages[-1].data == -1


def test_shutdown_brakes_without_unexport(motor_factory):
    create, line, chip = motor_factory
    node = create()

    node.stop()

    assert node.motor.duty_cycle_ns == 0
    assert node.motor.enabled
    assert line.released
    assert chip.closed
