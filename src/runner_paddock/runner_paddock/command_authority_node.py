# Copyright 2026 matti
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Private transitional authority scaffold; outputs must not feed production mux."""

import math
import time
import uuid

from geometry_msgs.msg import Twist
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from runner_interfaces.msg import CommandAuthorityState
from runner_interfaces.msg import ModeState
from runner_interfaces.msg import PaddockControlEvent
from runner_interfaces.msg import PaddockControlLease
from runner_interfaces.msg import StopRequest
from runner_interfaces.msg import StopState

from runner_paddock.command_supervisor import CommandSupervisor
from runner_paddock.command_supervisor import ControlEvent
from runner_paddock.command_supervisor import SupervisorResult
from runner_paddock.command_supervisor import VelocityCommand
from runner_paddock.state_machine import Mode
from sensor_msgs.msg import Joy


RAW_AUTONOMY_TOPIC = '/cmd_vel_auto_raw'
SUPERVISED_AUTONOMY_TOPIC = '/paddock/private/cmd_vel_auto'
PADDOCK_OUTPUT_TOPIC = '/paddock/private/cmd_vel_paddock'
CONTROL_EVENT_TOPIC = '/paddock/control_event'
AUTHORITY_STATE_TOPIC = '/paddock/command_authority_state'
MODE_STATE_TOPIC = '/paddock/mode_state'
LEASE_STATE_TOPIC = '/paddock/control_lease'
DUALSENSE_TOPIC = '/joy'
DEFAULT_SUPERVISION_PERIOD_SEC = 0.010
DEFAULT_DUALSENSE_TIMEOUT_SEC = 0.200
DEFAULT_STOP_STATE_TIMEOUT_SEC = 0.150
STOP_REQUEST_TOPIC = '/paddock/internal/stop_request'
STOP_STATE_TOPIC = '/paddock/stop_state'
X_BUTTON_INDEX = 0
R1_BUTTON_INDEX = 5


def _button_held(message: Joy, index: int) -> bool:
    return index < len(message.buttons) and bool(message.buttons[index])


def _from_twist(message: Twist) -> VelocityCommand:
    return VelocityCommand(
        linear_x=message.linear.x,
        linear_y=message.linear.y,
        linear_z=message.linear.z,
        angular_x=message.angular.x,
        angular_y=message.angular.y,
        angular_z=message.angular.z,
    )


def _to_twist(command: VelocityCommand) -> Twist:
    message = Twist()
    message.linear.x = command.linear_x
    message.linear.y = command.linear_y
    message.linear.z = command.linear_z
    message.angular.x = command.angular_x
    message.angular.y = command.angular_y
    message.angular.z = command.angular_z
    return message


def _authority_message(
    result: SupervisorResult,
    stamp,
    stop_state: StopState | None = None,
    *,
    stop_fresh: bool = False,
) -> CommandAuthorityState:
    """Convert one pure snapshot to its typed authority interface."""
    snapshot = result.snapshot
    state = snapshot.state
    message = CommandAuthorityState()
    message.stamp = stamp
    message.authority = int(state.authority)
    message.client_id = state.lease_client_id
    message.lease_id = state.lease_id
    message.dualsense_active = state.dualsense_active
    message.run_held = state.run_held
    message.autonomy_permitted = state.autonomy_permitted
    message.autonomy_goal_selected = state.goal is not None
    message.autonomy_action_active = state.navigation_active
    message.brake_intent = snapshot.brake_intent
    message.lease_fresh = snapshot.lease_fresh
    message.lease_age_sec = (
        -1.0 if snapshot.lease_age_sec is None else snapshot.lease_age_sec
    )
    message.raw_autonomy_fresh = snapshot.raw_autonomy_fresh
    message.raw_autonomy_age_sec = (
        -1.0
        if snapshot.raw_autonomy_age_sec is None
        else snapshot.raw_autonomy_age_sec
    )
    message.last_control_sequence = (
        0
        if snapshot.last_control_sequence is None
        else snapshot.last_control_sequence
    )
    message.reason = snapshot.reason
    message.stop_state_fresh = stop_fresh
    message.stop_healthy = bool(stop_fresh and stop_state.healthy)
    message.stop_applied = bool(stop_fresh and stop_state.applied)
    message.stop_clear = bool(
        stop_fresh and stop_state.healthy
        and not stop_state.stopped and not stop_state.locked
    )
    message.stop_boot_id = stop_state.boot_id if stop_fresh else ''
    message.stop_generation = stop_state.generation if stop_fresh else 0
    message.stop_reason = (
        stop_state.reason if stop_fresh else 'STOP_STATE_STALE'
    )
    return message


class CommandAuthorityNode(Node):
    """Expose the pure supervisor without touching the live /cmd_vel topic."""

    def __init__(self) -> None:
        super().__init__('runner_command_authority')
        lease_timeout = float(
            self.declare_parameter('lease_timeout_sec', 0.150).value
        )
        raw_timeout = float(
            self.declare_parameter(
                'raw_autonomy_timeout_sec', 0.150
            ).value
        )
        supervision_period = float(
            self.declare_parameter(
                'supervision_period_sec',
                DEFAULT_SUPERVISION_PERIOD_SEC,
            ).value
        )
        self._dualsense_timeout = float(
            self.declare_parameter(
                'dualsense_timeout_sec',
                DEFAULT_DUALSENSE_TIMEOUT_SEC,
            ).value
        )
        active_map = str(
            self.declare_parameter('active_autonomy_map', '').value
        )
        if not math.isfinite(supervision_period) or supervision_period <= 0:
            raise ValueError(
                'supervision_period_sec must be finite and positive'
            )
        if (
            not math.isfinite(self._dualsense_timeout)
            or self._dualsense_timeout <= 0
        ):
            raise ValueError(
                'dualsense_timeout_sec must be finite and positive'
            )

        self._supervisor = CommandSupervisor(
            lease_timeout_sec=lease_timeout,
            raw_autonomy_timeout_sec=raw_timeout,
            active_autonomy_map=active_map,
        )
        self._last_joy_at = None
        self._dualsense_active = False
        self._requester_id = str(uuid.uuid4())
        self._stop_request_id = 0
        self._pending_stop_request = None
        self._last_stop_request_at = None
        self._stop_state = None
        self._stop_state_at = None
        self._stop_state_timeout = DEFAULT_STOP_STATE_TIMEOUT_SEC
        self._auto_pub = self.create_publisher(
            Twist, SUPERVISED_AUTONOMY_TOPIC, 10
        )
        self._paddock_pub = self.create_publisher(
            Twist, PADDOCK_OUTPUT_TOPIC, 10
        )
        self._authority_pub = self.create_publisher(
            CommandAuthorityState, AUTHORITY_STATE_TOPIC, 10
        )
        self._lease_pub = self.create_publisher(
            PaddockControlLease, LEASE_STATE_TOPIC, 10
        )
        stop_qos = QoSProfile(depth=1)
        stop_qos.reliability = ReliabilityPolicy.RELIABLE
        stop_qos.lifespan = Duration(seconds=0.10)
        self._stop_request_pub = self.create_publisher(
            StopRequest, STOP_REQUEST_TOPIC, stop_qos
        )
        self.create_subscription(
            Twist, RAW_AUTONOMY_TOPIC, self._on_raw_autonomy, 10
        )
        self.create_subscription(
            PaddockControlEvent,
            CONTROL_EVENT_TOPIC,
            self._on_control_event,
            10,
        )
        mode_qos = QoSProfile(depth=1)
        mode_qos.reliability = ReliabilityPolicy.RELIABLE
        mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            ModeState, MODE_STATE_TOPIC, self._on_mode, mode_qos
        )
        self.create_subscription(Joy, DUALSENSE_TOPIC, self._on_joy, 10)
        self.create_subscription(
            StopState,
            STOP_STATE_TOPIC,
            self._on_stop_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_timer(supervision_period, self._on_supervision_timer)

        self._apply(self._supervisor.restart(time.monotonic()))
        self.get_logger().info(
            'Command authority offline contract: '
            f'{RAW_AUTONOMY_TOPIC} -> {SUPERVISED_AUTONOMY_TOPIC}; '
            f'brake intent -> {PADDOCK_OUTPUT_TOPIC}; '
            'never publishes /cmd_vel'
        )

    def _on_raw_autonomy(self, message: Twist) -> None:
        result = self._supervisor.receive_raw_autonomy(
            _from_twist(message), time.monotonic()
        )
        self._apply(result)

    def _on_control_event(self, message: PaddockControlEvent) -> None:
        try:
            event = ControlEvent(message.event)
        except ValueError:
            self.get_logger().warning(
                f'Rejected unknown Paddock control event {message.event}'
            )
            return
        result = self._supervisor.handle_control_event(
            event=event,
            client_id=message.client_id,
            lease_id=message.lease_id,
            sequence=message.sequence,
            now=time.monotonic(),
        )
        if not result.accepted:
            self.get_logger().warning(result.snapshot.reason)
        elif event == ControlEvent.STOP:
            self._request_stop(True)
        elif event == ControlEvent.CLEAR_STOP:
            if not self._stop_is_fresh(time.monotonic()):
                self.get_logger().warning('CLEAR_STOP_STATE_STALE')
            elif not self._stop_state.healthy:
                self.get_logger().warning('CLEAR_STOP_ENFORCER_UNHEALTHY')
            elif not self._stop_state.stopped or not self._stop_state.applied:
                self.get_logger().warning('CLEAR_STOP_NOT_APPLIED')
            else:
                self._request_stop(False)
        self._apply(result)

    def _stop_is_fresh(self, now: float) -> bool:
        return (
            self._stop_state is not None
            and self._stop_state_at is not None
            and now - self._stop_state_at <= self._stop_state_timeout
        )

    def _stop_clear(self, now: float) -> bool:
        return (
            self._stop_is_fresh(now)
            and self._stop_state.healthy
            and not self._stop_state.stopped
            and not self._stop_state.locked
        )

    def _request_stop(self, stopped: bool) -> None:
        self._stop_request_id += 1
        request = StopRequest()
        request.requester_id = self._requester_id
        request.request_id = self._stop_request_id
        request.stopped = stopped
        if not stopped:
            request.expected_boot_id = self._stop_state.boot_id
            request.expected_generation = self._stop_state.generation
        self._pending_stop_request = request
        self._publish_stop_request(time.monotonic())

    def _publish_stop_request(self, now: float) -> None:
        self._stop_request_pub.publish(self._pending_stop_request)
        self._last_stop_request_at = now

    def _on_stop_state(self, message: StopState) -> None:
        self._stop_state = message
        self._stop_state_at = time.monotonic()
        request = self._pending_stop_request
        if (
            request is None
            or message.last_requester_id != request.requester_id
            or message.last_request_id != request.request_id
        ):
            return
        if not message.last_request_accepted:
            self.get_logger().warning(message.last_request_reason)
            self._pending_stop_request = None
        elif request.stopped and message.applied:
            self._pending_stop_request = None
        elif (
            not request.stopped
            and message.healthy
            and not message.stopped
            and not message.locked
        ):
            self._pending_stop_request = None

    def _on_mode(self, message: ModeState) -> None:
        try:
            mode = Mode(message.mode)
        except ValueError:
            self.get_logger().warning(
                f'Rejected unknown Paddock mode {message.mode}'
            )
            return
        result = self._supervisor.set_runtime_mode(
            mode=mode,
            active_autonomy_map=message.active_autonomy_map,
            now=time.monotonic(),
            runtime_stable=(message.status == ModeState.STATUS_STABLE),
        )
        self._apply(result)

    def _on_joy(self, message: Joy) -> None:
        now = time.monotonic()
        self._last_joy_at = now
        active = (
            _button_held(message, X_BUTTON_INDEX)
            or _button_held(message, R1_BUTTON_INDEX)
        )
        if active == self._dualsense_active:
            return
        self._dualsense_active = active
        self._apply(self._supervisor.set_dualsense_active(active, now))

    def _on_supervision_timer(self) -> None:
        now = time.monotonic()
        if (
            self._pending_stop_request is not None
            and (
                self._last_stop_request_at is None
                or now - self._last_stop_request_at >= 0.05
            )
        ):
            self._publish_stop_request(now)
        if (
            self._dualsense_active
            and self._last_joy_at is not None
            and now - self._last_joy_at > self._dualsense_timeout
        ):
            self._dualsense_active = False
            self._apply(
                self._supervisor.set_dualsense_active(False, now)
            )
        self._apply(self._supervisor.tick(now))

    def _apply(self, result: SupervisorResult) -> None:
        if result.autonomy_command is not None and self._stop_clear(time.monotonic()):
            self._auto_pub.publish(_to_twist(result.autonomy_command))
        if result.snapshot.brake_intent:
            brake = Twist()
            self._auto_pub.publish(brake)
            self._paddock_pub.publish(brake)
        self._publish_state(result)

    def _publish_state(self, result: SupervisorResult) -> None:
        snapshot = result.snapshot
        state = snapshot.state
        stamp = self.get_clock().now().to_msg()

        now = time.monotonic()
        self._authority_pub.publish(_authority_message(
            result,
            stamp,
            self._stop_state,
            stop_fresh=self._stop_is_fresh(now),
        ))

        lease = PaddockControlLease()
        lease.stamp = stamp
        lease.active = state.lease_active
        lease.client_id = state.lease_client_id
        lease.lease_id = state.lease_id
        lease.generation = state.lease_generation
        self._lease_pub.publish(lease)


def main(args=None) -> None:
    """Run the offline command-authority node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CommandAuthorityNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
