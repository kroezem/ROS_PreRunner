import pytest

from runner_teleop.teleop_node import CREATE_BUTTON_INDEX
from runner_teleop.teleop_node import TeleopNode
from runner_teleop.teleop_node import X_BUTTON_INDEX
from sensor_msgs.msg import Joy


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_node():
    node = TeleopNode.__new__(TeleopNode)
    node.pub = Publisher()
    node._axis_steer = 0
    node._axis_brake = 2
    node._axis_throttle = 5
    node._deadman_button = X_BUTTON_INDEX
    node._steer = 0.0
    node._manual_cmd = 0.0
    node._manual_held = False
    node._autonomy_held = False
    return node


def joy(*, steer=0.0, brake=1.0, throttle=1.0, x=False, create=False):
    message = Joy()
    message.axes = [steer, 0.0, brake, 0.0, 0.0, throttle]
    message.buttons = [0] * 13
    message.buttons[X_BUTTON_INDEX] = int(x)
    message.buttons[CREATE_BUTTON_INDEX] = int(create)
    return message


def publish(node, message):
    node.on_joy(message)
    node.publish_cmd()
    return node.pub.messages[-1] if node.pub.messages else None


def test_neither_button_publishes_full_brake_with_steering():
    node = make_node()

    message = publish(node, joy(steer=0.4))

    assert message.linear.x == -1.0
    assert message.angular.z == pytest.approx(0.4)


def test_manual_trigger_mapping_and_steering_are_unchanged():
    node = make_node()

    throttle = publish(node, joy(steer=-0.6, throttle=0.5, x=True))
    brake = publish(node, joy(steer=0.3, brake=0.0, x=True))

    assert throttle.linear.x == 0.25
    assert throttle.angular.z == pytest.approx(-0.6)
    assert brake.linear.x == -0.5
    assert brake.angular.z == pytest.approx(0.3)


def test_create_without_x_publishes_nothing_including_steering():
    node = make_node()

    for _ in range(4):
        node.on_joy(joy(steer=0.8, create=True))
        node.publish_cmd()

    assert node.pub.messages == []


def test_x_wins_when_x_and_create_are_both_held():
    node = make_node()

    message = publish(
        node,
        joy(steer=0.2, throttle=-0.5, x=True, create=True),
    )

    assert message.linear.x == 0.75
    assert message.angular.z == pytest.approx(0.2)


def test_create_release_publishes_full_brake_on_next_cycle():
    node = make_node()
    node.on_joy(joy(steer=-0.25, create=True))
    node.publish_cmd()

    message = publish(node, joy(steer=-0.25))

    assert len(node.pub.messages) == 1
    assert message.linear.x == -1.0
    assert message.angular.z == pytest.approx(-0.25)


def test_brake_state_republishes_full_brake_each_cycle():
    node = make_node()
    node.on_joy(joy(steer=0.1))

    for _ in range(5):
        node.publish_cmd()

    assert len(node.pub.messages) == 5
    assert all(message.linear.x == -1.0 for message in node.pub.messages)
    assert all(
        message.angular.z == pytest.approx(0.1)
        for message in node.pub.messages
    )
