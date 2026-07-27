from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Invert if physical response is backwards
INVERT_STEER = False
THROTTLE_DEADZONE = 0.05
X_BUTTON_INDEX = 0
CREATE_BUTTON_INDEX = 8


def _trigger(raw: float) -> float:
    """Normalise -1 pressed / +1 released to 1.0 pressed / 0.0 released."""
    return max(0.0, min(1.0, (1.0 - raw) / 2.0))


def _selected_command(
    manual_held: bool,
    autonomy_held: bool,
    manual_command: float,
):
    """Return the command to publish, or None while autonomy is enabled."""
    if manual_held:
        return manual_command
    if autonomy_held:
        return None
    return -1.0


class TeleopNode(Node):
    def __init__(self):
        super().__init__('runner_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(0.05, self.publish_cmd)
        self.declare_parameter('axis_steer', 0)
        self.declare_parameter('axis_brake', 2)
        self.declare_parameter('axis_throttle', 5)
        self.declare_parameter('deadman_button', X_BUTTON_INDEX)
        self._axis_steer = self.get_parameter('axis_steer').value
        self._axis_brake = self.get_parameter('axis_brake').value
        self._axis_throttle = self.get_parameter('axis_throttle').value
        self._deadman_button = self.get_parameter('deadman_button').value
        self._steer = 0.0
        self._manual_cmd = 0.0
        self._manual_held = False
        self._autonomy_held = False
        self.get_logger().info(
            'runner_teleop ready  |  L-stick=steer  R2=throttle  L2=brake  '
            f'dead-man button index={self._deadman_button}  '
            f'autonomy-enable Create button index={CREATE_BUTTON_INDEX}')

    def on_joy(self, msg: Joy):
        self._manual_held = (
            0 <= self._deadman_button < len(msg.buttons)
            and msg.buttons[self._deadman_button] == 1
        )
        self._autonomy_held = (
            CREATE_BUTTON_INDEX < len(msg.buttons)
            and msg.buttons[CREATE_BUTTON_INDEX] == 1
        )

        if max(self._axis_steer, self._axis_brake, self._axis_throttle) >= len(msg.axes):
            self._manual_cmd = 0.0
            return

        brake_raw = msg.axes[self._axis_brake]
        throttle_raw = msg.axes[self._axis_throttle]
        self._steer = msg.axes[self._axis_steer] * (-1.0 if INVERT_STEER else 1.0)
        throttle = _trigger(throttle_raw)
        brake = _trigger(brake_raw)
        self._manual_cmd = (
            -brake if brake > THROTTLE_DEADZONE
            else throttle
        )

    def publish_cmd(self):
        command = _selected_command(
            self._manual_held,
            self._autonomy_held,
            self._manual_cmd,
        )
        if command is None:
            return

        msg = Twist()
        msg.linear.x = command
        msg.angular.z = self._steer
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TeleopNode())
    if rclpy.ok():
        rclpy.shutdown()
