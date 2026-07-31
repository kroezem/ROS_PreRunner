import math

from geometry_msgs.msg import Twist
import pytest
import rclpy

from runner_motor import motor_node


class FakePWM:
    instances = []
    events = []

    def __init__(self, chip_path, channel):
        self.chip_path = chip_path
        self.channel = channel
        self.period_ns = None
        self.duty_cycle_ns = None
        self.polarity = None
        self.enabled = None
        self.__class__.instances.append(self)

    def set_period_ns(self, value):
        self.period_ns = value
        self.events.append((self.channel, 'period', value))

    def set_duty_cycle_ns(self, value):
        self.duty_cycle_ns = value
        self.events.append((self.channel, 'duty', value))

    def set_polarity(self, value):
        self.polarity = value
        self.events.append((self.channel, 'polarity', value))

    def enable(self):
        self.enabled = True
        self.events.append((self.channel, 'enable', 1))

    def disable(self):
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
    line = FakeLine()
    chip = FakeChip()
    monkeypatch.setattr(motor_node, 'SysfsPWM', FakePWM)

    def request_direction_output():
        FakePWM.events.append(('dir', motor_node.DIR_FORWARD))
        return chip, line

    monkeypatch.setattr(
        motor_node, 'request_direction_output', request_direction_output
    )

    def create():
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


def test_safe_startup_configuration_and_order(motor_factory):
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

    motor_enable = FakePWM.events.index((0, 'enable', 1))
    direction_defined = FakePWM.events.index(
        ('dir', motor_node.DIR_FORWARD)
    )
    assert (0, 'duty', 0) in FakePWM.events[:direction_defined]
    assert direction_defined < motor_enable


def test_direction_change_zeros_duty_before_dir(motor_factory):
    create, line, _chip = motor_factory
    node = create()
    FakePWM.events = []

    message = Twist()
    message.linear.x = -0.5
    node.on_cmd(message)

    assert FakePWM.events[:3] == [
        (0, 'duty', 0),
        ('dir', motor_node.DIR_REVERSE),
        (0, 'duty', 25_000),
    ]
    assert line.values[-1] == motor_node.DIR_REVERSE


@pytest.mark.parametrize(
    ('command', 'expected_direction', 'expected_duty'),
    [(-1.0, -1, 50_000), (0.0, 0, 0), (0.5, 1, 25_000)],
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


def test_watchdog_sets_zero_duty_and_publishes_zero(motor_factory):
    create, _line, _chip = motor_factory
    node = create()
    node.motor.duty_cycle_ns = motor_node.MOTOR_PERIOD_NS
    node._last_cmd -= motor_node.CMD_TIMEOUT_S + 1.0
    direction_messages = []
    warnings = []
    node._direction_pub.publish = direction_messages.append
    node.get_logger().warn = warnings.append

    node._watchdog()

    assert node.motor.duty_cycle_ns == 0
    assert direction_messages[-1].data == 0
    assert warnings == [
        '/cmd_vel watchdog timeout; motor duty=0 (active brake)'
    ]


def test_shutdown_brakes_publishes_zero_and_does_not_unexport(motor_factory):
    create, line, chip = motor_factory
    node = create()
    direction_messages = []
    node._direction_pub.publish = direction_messages.append
    node.motor.duty_cycle_ns = motor_node.MOTOR_PERIOD_NS

    node.stop()

    assert node.motor.duty_cycle_ns == 0
    assert node.motor.enabled
    assert direction_messages[-1].data == 0
    assert line.released
    assert chip.closed
