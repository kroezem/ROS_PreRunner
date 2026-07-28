from decimal import Decimal
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

# Invert if physical response is backwards
INVERT_STEER = False
THROTTLE_DEADZONE = 0.05
X_BUTTON_INDEX = 0
L1_BUTTON_INDEX = 4
R1_BUTTON_INDEX = 5
DPAD_VERTICAL_AXIS_INDEX = 7

BRAKE_MODE = 'brake'
MANUAL_MODE = 'manual'
FIXED_THROTTLE_MODE = 'fixed_throttle'
TELEOP_SUPPRESS_MODE = 'teleop_suppress'
FIXED_THROTTLE_INHIBITED_MODE = 'fixed_throttle_inhibited'

NEUTRAL_US = 1500
FWD_ONSET_US = 1550
FORWARD_CROSSOVER = 0.05
FORWARD_EXPONENT = 2.0
FWD_MAX_US = 1750


def _trigger(raw: float) -> float:
    """Normalise -1 pressed / +1 released to 1.0 pressed / 0.0 released."""
    return max(0.0, min(1.0, (1.0 - raw) / 2.0))


def _button_held(msg: Joy, index: int) -> bool:
    return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1


def _active_mode(
    manual_held: bool,
    fixed_throttle_held: bool,
    teleop_suppress_held: bool,
    fixed_throttle_inhibited: bool,
) -> str:
    if manual_held:
        return MANUAL_MODE
    if fixed_throttle_held:
        if fixed_throttle_inhibited:
            return FIXED_THROTTLE_INHIBITED_MODE
        return FIXED_THROTTLE_MODE
    if teleop_suppress_held:
        return TELEOP_SUPPRESS_MODE
    return BRAKE_MODE


def _adjust_setpoint(
    current: float,
    step: float,
    minimum: float,
    maximum: float,
    direction: int,
) -> float:
    value = Decimal(str(current)) + direction * Decimal(str(step))
    value = max(Decimal(str(minimum)), min(Decimal(str(maximum)), value))
    return float(value)


def _validate_fixed_throttle_config(
    initial: float,
    step: float,
    minimum: float,
    maximum: float,
):
    values = (initial, step, minimum, maximum)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('fixed-throttle parameters must all be finite')
    if step <= 0.0:
        raise ValueError('fixed_throttle_step must be greater than zero')
    if minimum < 0.0:
        raise ValueError(
            'fixed_throttle_min_setpoint must be greater than or equal to zero'
        )
    if maximum > 1.0:
        raise ValueError(
            'fixed_throttle_max_setpoint must be less than or equal to one'
        )
    if minimum > maximum:
        raise ValueError(
            'fixed_throttle_min_setpoint must not exceed the maximum'
        )
    if not minimum <= initial <= maximum:
        raise ValueError(
            'fixed_throttle_initial_setpoint must be within configured bounds'
        )


def expected_race_esc_pulse_us(command: float) -> int:
    """Reproduce motor_node's positive race-mode ESC mapping."""
    magnitude = max(0.0, min(1.0, command))
    if magnitude == 0.0:
        return NEUTRAL_US
    if magnitude <= FORWARD_CROSSOVER:
        return int(
            NEUTRAL_US
            + magnitude / FORWARD_CROSSOVER * (FWD_ONSET_US - NEUTRAL_US)
        )
    normalised = (
        (magnitude - FORWARD_CROSSOVER) / (1.0 - FORWARD_CROSSOVER)
    )
    return int(
        FWD_ONSET_US
        + normalised ** FORWARD_EXPONENT * (FWD_MAX_US - FWD_ONSET_US)
    )


class TeleopNode(Node):
    def __init__(self):
        super().__init__('runner_teleop')
        self.declare_parameter('axis_steer', 0)
        self.declare_parameter('axis_brake', 2)
        self.declare_parameter('axis_throttle', 5)
        self.declare_parameter('deadman_button', X_BUTTON_INDEX)
        self.declare_parameter('fixed_throttle_initial_setpoint', 0.30)
        self.declare_parameter('fixed_throttle_step', 0.01)
        self.declare_parameter('fixed_throttle_max_setpoint', 0.50)
        self.declare_parameter('fixed_throttle_min_setpoint', 0.00)

        self._axis_steer = self.get_parameter('axis_steer').value
        self._axis_brake = self.get_parameter('axis_brake').value
        self._axis_throttle = self.get_parameter('axis_throttle').value
        self._deadman_button = self.get_parameter('deadman_button').value
        initial = self.get_parameter(
            'fixed_throttle_initial_setpoint'
        ).value
        self._fixed_throttle_step = self.get_parameter(
            'fixed_throttle_step'
        ).value
        self._fixed_throttle_max_setpoint = self.get_parameter(
            'fixed_throttle_max_setpoint'
        ).value
        self._fixed_throttle_min_setpoint = self.get_parameter(
            'fixed_throttle_min_setpoint'
        ).value
        try:
            _validate_fixed_throttle_config(
                initial,
                self._fixed_throttle_step,
                self._fixed_throttle_min_setpoint,
                self._fixed_throttle_max_setpoint,
            )
        except (TypeError, ValueError) as error:
            self.get_logger().error(
                f'Invalid fixed-throttle configuration: {error}'
            )
            raise ValueError(
                f'Invalid fixed-throttle configuration: {error}'
            ) from None

        self._fixed_throttle_setpoint = initial
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._setpoint_pub = self.create_publisher(
            Float32, '/teleop/fixed_throttle_setpoint', 10
        )
        self._active_mode_pub = self.create_publisher(
            String, '/teleop/active_mode', 10
        )
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.05, self.publish_cmd)

        self._steer = 0.0
        self._manual_cmd = 0.0
        self._brake = 0.0
        self._manual_held = False
        self._fixed_throttle_held = False
        self._teleop_suppress_held = False
        self.fixed_throttle_inhibited_until_r1_release = False
        self._dpad_press_active = False
        self.get_logger().info(
            'runner_teleop ready  |  X=manual  R1=fixed throttle  '
            'L1=teleop suppress  L-stick=steer  L2=brake'
        )
        self.get_logger().info(
            f'Fixed throttle initialized to '
            f'{self._fixed_throttle_setpoint:.3f}; maximum '
            f'{self._fixed_throttle_max_setpoint:.3f}'
        )

    def _update_dpad(self, msg: Joy):
        direction = 0
        if DPAD_VERTICAL_AXIS_INDEX < len(msg.axes):
            value = msg.axes[DPAD_VERTICAL_AXIS_INDEX]
            direction = int(value > 0.5) - int(value < -0.5)

        if direction == 0:
            self._dpad_press_active = False
            return
        if self._dpad_press_active:
            return

        self._dpad_press_active = True
        updated = _adjust_setpoint(
            self._fixed_throttle_setpoint,
            self._fixed_throttle_step,
            self._fixed_throttle_min_setpoint,
            self._fixed_throttle_max_setpoint,
            direction,
        )
        if updated == self._fixed_throttle_setpoint:
            return
        self._fixed_throttle_setpoint = updated
        expected = expected_race_esc_pulse_us(updated)
        self.get_logger().info(
            f'Fixed throttle setpoint changed to {updated:.3f}; '
            f'expected race ESC pulse {expected} us'
        )

    def on_joy(self, msg: Joy):
        manual_held = _button_held(msg, self._deadman_button)
        fixed_throttle_held = _button_held(msg, R1_BUTTON_INDEX)
        teleop_suppress_held = _button_held(msg, L1_BUTTON_INDEX)
        fixed_throttle_rising = (
            fixed_throttle_held and not self._fixed_throttle_held
        )

        if not fixed_throttle_held:
            self.fixed_throttle_inhibited_until_r1_release = False
        elif manual_held:
            self.fixed_throttle_inhibited_until_r1_release = True

        self._manual_held = manual_held
        self._fixed_throttle_held = fixed_throttle_held
        self._teleop_suppress_held = teleop_suppress_held
        self._update_dpad(msg)

        if fixed_throttle_rising:
            inhibited = self.fixed_throttle_inhibited_until_r1_release
            state = (
                'inhibited_until_r1_release' if inhibited else 'active'
            )
            expected = expected_race_esc_pulse_us(
                self._fixed_throttle_setpoint
            )
            self.get_logger().info(
                'R1 fixed-throttle request: '
                f'setpoint={self._fixed_throttle_setpoint:.3f}, '
                f'expected race ESC pulse={expected} us, state={state}'
            )

        if max(
            self._axis_steer,
            self._axis_brake,
            self._axis_throttle,
        ) >= len(msg.axes):
            self._manual_cmd = 0.0
            self._brake = 1.0
            return

        brake_raw = msg.axes[self._axis_brake]
        throttle_raw = msg.axes[self._axis_throttle]
        self._steer = (
            msg.axes[self._axis_steer]
            * (-1.0 if INVERT_STEER else 1.0)
        )
        throttle = _trigger(throttle_raw)
        self._brake = _trigger(brake_raw)
        self._manual_cmd = (
            -self._brake if self._brake > THROTTLE_DEADZONE
            else throttle
        )

    def publish_cmd(self):
        mode = _active_mode(
            self._manual_held,
            self._fixed_throttle_held,
            self._teleop_suppress_held,
            self.fixed_throttle_inhibited_until_r1_release,
        )
        self._setpoint_pub.publish(
            Float32(data=self._fixed_throttle_setpoint)
        )
        self._active_mode_pub.publish(String(data=mode))

        if mode == TELEOP_SUPPRESS_MODE:
            return
        if mode == MANUAL_MODE:
            command = self._manual_cmd
        elif mode == FIXED_THROTTLE_MODE:
            command = (
                -self._brake if self._brake > THROTTLE_DEADZONE
                else self._fixed_throttle_setpoint
            )
        else:
            command = -1.0

        msg = Twist()
        msg.linear.x = command
        msg.angular.z = self._steer
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = TeleopNode()
    except ValueError:
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
