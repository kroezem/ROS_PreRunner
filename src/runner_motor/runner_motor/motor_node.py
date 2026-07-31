import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import gpiod
import rclpy
from rclpy.node import Node
from runner_interfaces.msg import EncoderState
from std_msgs.msg import Int8

PWM_CHIP_PATH = Path('/sys/class/pwm/pwmchip0')
MOTOR_PWM_CHANNEL = 0  # GPIO12
SERVO_PWM_CHANNEL = 1  # GPIO13
MOTOR_PERIOD_NS = 50_000  # 20 kHz
SERVO_PERIOD_NS = 20_000_000  # 50 Hz
CMD_TIMEOUT_S = 0.2

GPIO_CHIP_LABEL = 'pinctrl-rp1'
MOTOR_DIR_GPIO = 23
DIR_FORWARD = 0
DIR_REVERSE = 1

STEER_CTR = 1500  # servo centre
STEER_US = 500  # ± range around centre


def us_to_ns(us):
    return int(us * 1000)


def map_motor_command(linear_x):
    """Return (direction, duty_ns) for a normalized signed command."""
    if not math.isfinite(linear_x):
        linear_x = 0.0
    command = max(-1.0, min(1.0, linear_x))
    direction = (command > 0.0) - (command < 0.0)
    return direction, int(abs(command) * MOTOR_PERIOD_NS)


class SysfsPWM:
    """Control an already-exported PWM channel without owning its export."""

    def __init__(self, chip_path, channel):
        self.path = Path(chip_path) / f'pwm{channel}'
        if not self.path.is_dir():
            raise RuntimeError(
                f'PWM channel is not exported: {self.path}; '
                'runner-pwm-setup.service must run first'
            )

    def _write(self, attribute, value):
        with (self.path / attribute).open('w') as output:
            output.write(str(value))

    def set_period_ns(self, period_ns):
        self._write('period', period_ns)

    def set_duty_cycle_ns(self, duty_ns):
        self._write('duty_cycle', duty_ns)

    def set_polarity(self, polarity):
        self._write('polarity', polarity)

    def enable(self):
        self._write('enable', 1)

    def disable(self):
        self._write('enable', 0)


def request_direction_output(gpiod_module=gpiod):
    """Exclusively request the MD13S DIR line by GPIO chip label."""
    chip = gpiod_module.Chip(
        GPIO_CHIP_LABEL, gpiod_module.Chip.OPEN_BY_LABEL
    )
    line = chip.get_line(MOTOR_DIR_GPIO)
    try:
        line.request(
            consumer='runner_motor_dir',
            type=gpiod_module.LINE_REQ_DIR_OUT,
            default_vals=[DIR_FORWARD],
        )
    except Exception:
        chip.close()
        raise
    return chip, line


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.motor = SysfsPWM(PWM_CHIP_PATH, MOTOR_PWM_CHANNEL)
        self.servo = SysfsPWM(PWM_CHIP_PATH, SERVO_PWM_CHANNEL)

        # Configure the motor without writing duty against a fresh period of 0.
        self.motor.disable()
        self.motor.set_polarity('normal')
        self.motor.set_period_ns(MOTOR_PERIOD_NS)
        self.motor.set_duty_cycle_ns(0)

        # Steering retains its existing 50 Hz, centred startup behavior.
        self.servo.disable()
        self.servo.set_polarity('normal')
        self.servo.set_period_ns(SERVO_PERIOD_NS)
        self.servo.set_duty_cycle_ns(us_to_ns(STEER_CTR))
        self.servo.enable()

        # Define DIR while motor duty is zero, then start the 20 kHz output.
        self._gpio_chip, self._dir_line = request_direction_output()
        self._hardware_direction = DIR_FORWARD
        self._direction_latch = 0
        self._pending_reversal = None
        self.motor.enable()

        self._direction_pub = self.create_publisher(
            Int8, '/motor/direction', 10
        )
        self._last_cmd = time.monotonic()
        self._cmd_timed_out = False
        self._stopped = False
        self.create_timer(0.05, self._watchdog)

        # Subscribe only after all motor outputs are in their safe state.
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(
            EncoderState,
            '/wheel/encoder_state',
            self._on_encoder_state,
            10,
        )
        self._direction_pub.publish(Int8(data=0))
        self.get_logger().info('motor_driver ready; MD13S PWM=20 kHz')

    def _set_motor(self, direction, duty_ns):
        if direction == 0:
            self._pending_reversal = None
            self.motor.set_duty_cycle_ns(0)
            return

        self._direction_latch = direction
        hardware_direction = (
            DIR_FORWARD if direction > 0 else DIR_REVERSE
        )
        if hardware_direction == self._hardware_direction:
            self._pending_reversal = None
            self.motor.set_duty_cycle_ns(duty_ns)
            return

        self.motor.set_duty_cycle_ns(0)
        self._pending_reversal = (hardware_direction, duty_ns)

    def _on_encoder_state(self, msg: EncoderState):
        if not msg.stationary or self._pending_reversal is None:
            return

        hardware_direction, duty_ns = self._pending_reversal
        self.motor.set_duty_cycle_ns(0)
        self._dir_line.set_value(hardware_direction)
        self._hardware_direction = hardware_direction
        self._pending_reversal = None
        self.motor.set_duty_cycle_ns(duty_ns)

    def on_cmd(self, msg: Twist):
        if self._cmd_timed_out:
            self.get_logger().info('/cmd_vel recovered')
            self._cmd_timed_out = False
        self._last_cmd = time.monotonic()

        direction, duty_ns = map_motor_command(msg.linear.x)
        self._set_motor(direction, duty_ns)
        self._direction_pub.publish(Int8(data=self._direction_latch))

        steer_us = int(
            STEER_CTR
            + max(-1.0, min(1.0, msg.angular.z)) * STEER_US
        )
        self.servo.set_duty_cycle_ns(us_to_ns(steer_us))

    def _watchdog(self):
        if time.monotonic() - self._last_cmd <= CMD_TIMEOUT_S:
            return
        self._pending_reversal = None
        self.motor.set_duty_cycle_ns(0)
        if not self._cmd_timed_out:
            self.get_logger().warn(
                '/cmd_vel watchdog timeout; motor duty=0 (active brake)'
            )
            self._cmd_timed_out = True
            self._direction_pub.publish(
                Int8(data=self._direction_latch)
            )

    def stop(self):
        if self._stopped:
            return
        self._pending_reversal = None
        self.motor.set_duty_cycle_ns(0)
        self._direction_pub.publish(Int8(data=self._direction_latch))
        self.servo.set_duty_cycle_ns(us_to_ns(STEER_CTR))
        self.servo.disable()
        self._dir_line.release()
        self._gpio_chip.close()
        self._stopped = True


def main():
    rclpy.init()
    node = None
    try:
        node = MotorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
