"""
Receive keyboard UDP state and publish the Pi-owned autonomy latch.

Unlike the previous fail-safe-on-link-loss policy, an armed autonomy latch
survives intermittent sender loss.  Link loss was stopping the vehicle
mid-route. Escape is the primary keyboard emergency stop and sends repeated
brake packets; the unchanged motor watchdog remains an independent stop path
for current low-speed Phase 1 operation. Backtick toggles the latch, Space
retains its manual hold-to-run role, DualSense X/R1/L1 clear the latch, and the
latch expires 600 seconds after an explicit arm. Sender termination does not
itself disarm the Pi latch, so operators must explicitly disarm before leaving
the system unattended and revisit this posture before Phase 2 racing speeds.
"""

from dataclasses import dataclass
import ipaddress
import math
import socket
import time

from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_interfaces.msg import KeyboardState

from runner_teleop.keyboard_protocol import decode_packet
from runner_teleop.keyboard_protocol import is_newer_sequence
from runner_teleop.keyboard_protocol import KeyboardPacket
from runner_teleop.keyboard_protocol import PACKET_SIZE
from runner_teleop.keyboard_protocol import PacketError
from runner_teleop.keyboard_protocol import ROUTE_CLEAR
from runner_teleop.keyboard_protocol import ROUTE_CLEAR_GLOBAL_OBSTACLES
from runner_teleop.keyboard_protocol import ROUTE_LOOP_TOGGLE
from runner_teleop.keyboard_protocol import ROUTE_NONE
from runner_teleop.keyboard_protocol import ROUTE_REMOVE_LAST
from runner_teleop.keyboard_protocol import ROUTE_START
from runner_teleop.keyboard_protocol import ROUTE_STOP
from runner_teleop.keyboard_protocol import ROUTE_TOGGLE_GLOBAL_OBSTACLES
from runner_teleop.keyboard_protocol import sequence_delta
from runner_teleop.keyboard_protocol import SET_ROUTE_MODE
from runner_teleop.keyboard_protocol import SET_WAYPOINT_MODE
from sensor_msgs.msg import Joy
from std_msgs.msg import String


DEFAULT_PORT = 49321
DEFAULT_TIMEOUT = 0.15
DEFAULT_SPEED_CAP = 0.50
DEFAULT_PUBLICATION_RATE = 20.0
DEFAULT_AUTONOMY_LATCH_TIMEOUT = 600.0
WARNING_PERIOD = 2.0
COSTMAP_SERVICE_TIMEOUT = 1.0
COSTMAP_REFRESH_INTERVAL = 30.0
COSTMAP_REFRESH_RETRY_INTERVAL = 2.0
COSTMAP_CLEAR_SERVICE = (
    '/global_costmap/clear_entirely_global_costmap'
)
COSTMAP_GET_PARAMETERS_SERVICE = (
    '/global_costmap/global_costmap/get_parameters'
)
COSTMAP_SET_PARAMETERS_SERVICE = (
    '/global_costmap/global_costmap/set_parameters'
)
GLOBAL_OBSTACLE_PARAMETER = 'obstacle_layer.enabled'
X_BUTTON_INDEX = 0
L1_BUTTON_INDEX = 4
R1_BUTTON_INDEX = 5
ROUTE_COMMAND_NAMES = {
    ROUTE_START: 'start',
    ROUTE_STOP: 'stop',
    ROUTE_CLEAR: 'clear',
    ROUTE_LOOP_TOGGLE: 'loop_toggle',
    ROUTE_REMOVE_LAST: 'remove_last',
    ROUTE_CLEAR_GLOBAL_OBSTACLES: 'clear_global_obstacles',
    ROUTE_TOGGLE_GLOBAL_OBSTACLES: 'toggle_global_obstacles',
    SET_WAYPOINT_MODE: 'set_waypoint_mode',
    SET_ROUTE_MODE: 'set_route_mode',
}


@dataclass(frozen=True)
class AcceptedState:
    """One accepted and Pi-capped keyboard state."""

    packet: KeyboardPacket
    source_ip: str
    accepted_at: float
    throttle: float


class KeyboardAutonomyLatch:
    """Apply stateful arm, disarm, controller-clear, and expiry policy."""

    def __init__(self, timeout: float):
        self.timeout = timeout
        self.armed = False
        self.armed_at = None
        self._suppress_previous = False
        self._rearm_ready = True

    def process_mode(self, mode: int, now: float) -> bool:
        """Apply every valid packet; return whether brake was requested."""
        brake = bool(mode & 1)
        suppress = bool(mode & 2)
        if brake:
            self.disarm(require_release=False)
        elif not suppress:
            self._rearm_ready = True
        elif (
            not self._suppress_previous
            and self._rearm_ready
            and not self.armed
        ):
            self.armed = True
            self.armed_at = now
            self._rearm_ready = False
        self._suppress_previous = suppress
        return brake

    def disarm(self, *, require_release: bool) -> None:
        """Clear idempotently, optionally requiring a low suppress state."""
        self.armed = False
        self.armed_at = None
        if require_release:
            self._rearm_ready = False
        else:
            self._rearm_ready = True
        if not require_release:
            self._suppress_previous = False

    def clear_for_controller(self) -> None:
        """Disarm until the sender reports suppression released."""
        self.disarm(require_release=True)

    def expire(self, now: float) -> bool:
        """Disarm once the original arm event reaches its finite lifetime."""
        if (
            self.armed
            and self.armed_at is not None
            and now - self.armed_at >= self.timeout
        ):
            self.disarm(require_release=True)
            return True
        return False


class KeyboardReceiver:
    """Validate source, session, sequence, liveness, and speed policy."""

    def __init__(
        self,
        speed_cap: float,
        timeout: float,
        allowed_source_ip: str = '',
    ):
        self.speed_cap = speed_cap
        self.timeout = timeout
        self.allowed_source_ip = allowed_source_ip
        self.state = None

    def is_live(self, now: float) -> bool:
        if self.state is None:
            return False
        return now - self.state.accepted_at <= self.timeout

    def accept(
        self,
        data: bytes,
        source_ip: str,
        now: float,
    ) -> tuple[AcceptedState | None, str | None, int | None]:
        """Return state, rejection reason, and optional sequence gap."""
        packet, error = self.inspect(data, source_ip)
        if error is not None:
            return None, error, None
        return self.accept_packet(packet, source_ip, now)

    def inspect(
        self,
        data: bytes,
        source_ip: str,
    ) -> tuple[KeyboardPacket | None, str | None]:
        """Validate source and packet syntax without sequence filtering."""
        if self.allowed_source_ip and source_ip != self.allowed_source_ip:
            return None, f'unauthorized source {source_ip}'
        try:
            packet = decode_packet(data)
        except PacketError as error:
            return None, str(error)
        return packet, None

    def accept_packet(
        self,
        packet: KeyboardPacket,
        source_ip: str,
        now: float,
    ) -> tuple[AcceptedState | None, str | None, int | None]:
        """Apply owner and serial-number ordering to a decoded packet."""
        previous = self.state
        live = self.is_live(now)
        if previous is not None:
            same_source = source_ip == previous.source_ip
            same_session = packet.session_id == previous.packet.session_id
            same_owner = same_source and same_session
            if not same_owner:
                if live:
                    return (
                        None,
                        'source/session change while input is live',
                        None,
                    )
                gap = None
            else:
                if not is_newer_sequence(
                    packet.sequence,
                    previous.packet.sequence,
                ):
                    delta = sequence_delta(
                        packet.sequence,
                        previous.packet.sequence,
                    )
                    kind = 'duplicate' if delta == 0 else 'reordered/old'
                    return None, f'{kind} sequence {packet.sequence}', None
                delta = sequence_delta(
                    packet.sequence,
                    previous.packet.sequence,
                )
                gap = delta - 1 if delta > 1 else None
        else:
            gap = None

        accepted = AcceptedState(
            packet=packet,
            source_ip=source_ip,
            accepted_at=now,
            throttle=min(packet.throttle, self.speed_cap),
        )
        self.state = accepted
        return accepted, None, gap


def _validate_configuration(
    bind_address: str,
    port: int,
    allowed_source_ip: str,
    timeout: float,
    speed_cap: float,
    publication_rate: float,
    autonomy_latch_timeout: float,
) -> None:
    try:
        ipaddress.IPv4Address(bind_address)
    except ipaddress.AddressValueError as error:
        raise ValueError('bind_address must be an IPv4 address') from error
    if not 1 <= port <= 65535:
        raise ValueError('port must be within [1, 65535]')
    if allowed_source_ip:
        try:
            ipaddress.IPv4Address(allowed_source_ip)
        except ipaddress.AddressValueError as error:
            raise ValueError(
                'allowed_source_ip must be empty or an IPv4 address'
            ) from error
    if not 0.0 < timeout:
        raise ValueError('input_timeout must be greater than zero')
    if not 0.0 <= speed_cap <= 1.0:
        raise ValueError('speed_cap must be within [0.0, 1.0]')
    if publication_rate <= 0.0:
        raise ValueError('publication_rate must be greater than zero')
    if not math.isfinite(autonomy_latch_timeout) or (
        autonomy_latch_timeout <= 0.0
    ):
        raise ValueError(
            'autonomy_latch_timeout must be finite and greater than zero'
        )


class KeyboardBridge(Node):
    """ROS wrapper around the UDP receiver and Pi-side safety policy."""

    def __init__(self):
        super().__init__('keyboard_bridge')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('allowed_source_ip', '')
        self.declare_parameter('input_timeout', DEFAULT_TIMEOUT)
        self.declare_parameter('speed_cap', DEFAULT_SPEED_CAP)
        self.declare_parameter(
            'publication_rate',
            DEFAULT_PUBLICATION_RATE,
        )
        self.declare_parameter(
            'autonomy_latch_timeout',
            DEFAULT_AUTONOMY_LATCH_TIMEOUT,
        )
        bind_address = self.get_parameter('bind_address').value
        port = self.get_parameter('port').value
        allowed_source_ip = self.get_parameter(
            'allowed_source_ip'
        ).value
        timeout = self.get_parameter('input_timeout').value
        speed_cap = self.get_parameter('speed_cap').value
        publication_rate = self.get_parameter('publication_rate').value
        autonomy_latch_timeout = self.get_parameter(
            'autonomy_latch_timeout'
        ).value
        _validate_configuration(
            bind_address,
            port,
            allowed_source_ip,
            timeout,
            speed_cap,
            publication_rate,
            autonomy_latch_timeout,
        )

        self._receiver = KeyboardReceiver(
            speed_cap,
            timeout,
            allowed_source_ip,
        )
        self._warnings = {}
        self._latch = KeyboardAutonomyLatch(autonomy_latch_timeout)
        self._global_obstacles_state = (
            KeyboardState.GLOBAL_OBSTACLES_UNKNOWN
        )
        self._costmap_action = None
        self._costmap_phase = None
        self._costmap_future = None
        self._costmap_deadline = None
        self._costmap_previous = None
        self._costmap_requested = None
        self._pending_costmap_action = None
        self._next_obstacle_refresh = time.monotonic()
        self._last_published_valid = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.bind((bind_address, port))
        self._publisher = self.create_publisher(
            KeyboardState,
            '/teleop/keyboard_state',
            10,
        )
        self._route_control_pub = self.create_publisher(
            String,
            '/runner/route_control',
            10,
        )
        self._clear_costmap_client = self.create_client(
            ClearEntireCostmap,
            COSTMAP_CLEAR_SERVICE,
        )
        self._get_parameters_client = self.create_client(
            GetParameters,
            COSTMAP_GET_PARAMETERS_SERVICE,
        )
        self._set_parameters_client = self.create_client(
            SetParameters,
            COSTMAP_SET_PARAMETERS_SERVICE,
        )
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_timer(0.005, self._receive)
        self.create_timer(0.02, self._pump_costmap_controls)
        self.create_timer(0.005, self._publish_timeout_transition)
        self.create_timer(1.0 / publication_rate, self._publish)
        if not allowed_source_ip:
            self.get_logger().warning(
                'allowed_source_ip is empty; UDP input is unrestricted by '
                'source address (active source/session locking is not '
                'authentication)'
            )
        self.get_logger().info(
            f'keyboard_bridge listening on {bind_address}:{port}; '
            f'input_timeout={timeout:.3f}s speed_cap={speed_cap:.3f} '
            f'autonomy_latch_timeout={autonomy_latch_timeout:.3f}s'
        )

    def _warn(self, key: str, message: str, now: float) -> None:
        previous = self._warnings.get(key)
        if previous is not None and now - previous < WARNING_PERIOD:
            return
        self._warnings[key] = now
        self.get_logger().warning(message)

    def _receive(self) -> None:
        while True:
            try:
                data, address = self._socket.recvfrom(PACKET_SIZE + 1)
            except BlockingIOError:
                return
            except OSError as error:
                self._warn(
                    'socket',
                    f'UDP receive failed: {error}',
                    time.monotonic(),
                )
                return
            now = time.monotonic()
            packet, error = self._receiver.inspect(data, address[0])
            if error is not None:
                self._warn(
                    error.split()[0],
                    f'Rejected keyboard packet: {error}',
                    now,
                )
                continue
            brake_requested = self._latch.process_mode(packet.mode, now)
            if brake_requested:
                # Safety mutation precedes sequence filtering: byte-identical
                # Repeated Escape-generated packets are idempotent and brake
                # before ordinary sequence/session accounting can reject them.
                self._publish()
            accepted, error, gap = self._receiver.accept_packet(
                packet, address[0], now
            )
            if error is not None:
                self._warn(
                    error.split()[0],
                    f'Rejected keyboard packet: {error}',
                    now,
                )
                continue
            if gap is not None:
                self._warn(
                    'sequence_gap',
                    f'Keyboard packet sequence gap: {gap} missing packet(s)',
                    now,
                )
            if accepted is not None and (
                accepted.throttle < accepted.packet.throttle
            ):
                self._warn(
                    'speed_cap',
                    'Keyboard throttle capped on Pi: '
                    f'requested={accepted.packet.throttle:.3f} '
                    f'applied={accepted.throttle:.3f}',
                    now,
                )
            if (
                accepted is not None
                and accepted.packet.route_command != ROUTE_NONE
            ):
                self._dispatch_route_command(
                    accepted.packet.route_command
                )

    def _dispatch_route_command(self, command: int) -> None:
        if command == ROUTE_CLEAR_GLOBAL_OBSTACLES:
            self._request_costmap_action('clear')
            return
        if command == ROUTE_TOGGLE_GLOBAL_OBSTACLES:
            self._request_costmap_action('toggle')
            return
        self._route_control_pub.publish(String(
            data=ROUTE_COMMAND_NAMES[command]
        ))

    def _request_costmap_action(self, action: str) -> None:
        """Start or boundedly queue one operator costmap action."""
        now = time.monotonic()
        if self._costmap_action is None:
            self._begin_costmap_action(action, now)
            return
        if self._pending_costmap_action is None:
            self._pending_costmap_action = action
            return
        if action == 'clear' and self._pending_costmap_action == 'clear':
            self.get_logger().info(
                'Global costmap clear already pending; coalescing request'
            )
            return
        self.get_logger().warning(
            f'Costmap command {action!r} dropped: bounded pending slot is full'
        )

    def _begin_costmap_action(self, action: str, now: float) -> None:
        self._costmap_action = action
        self._costmap_phase = 'waiting'
        self._costmap_future = None
        self._costmap_deadline = now + COSTMAP_SERVICE_TIMEOUT
        self._costmap_previous = None
        self._costmap_requested = None

    def _finish_costmap_action(self, success: bool, now: float) -> None:
        action = self._costmap_action
        self._costmap_action = None
        self._costmap_phase = None
        self._costmap_future = None
        self._costmap_deadline = None
        self._costmap_previous = None
        self._costmap_requested = None
        if action == 'refresh':
            interval = (
                COSTMAP_REFRESH_INTERVAL
                if success else COSTMAP_REFRESH_RETRY_INTERVAL
            )
            self._next_obstacle_refresh = now + interval
        elif self._global_obstacles_state == (
            KeyboardState.GLOBAL_OBSTACLES_UNKNOWN
        ):
            self._next_obstacle_refresh = (
                now + COSTMAP_REFRESH_RETRY_INTERVAL
            )
        else:
            self._next_obstacle_refresh = now + COSTMAP_REFRESH_INTERVAL
        pending = self._pending_costmap_action
        self._pending_costmap_action = None
        if pending is not None:
            self._begin_costmap_action(pending, now)

    def _set_global_obstacles_state(self, enabled: bool) -> None:
        self._global_obstacles_state = (
            KeyboardState.GLOBAL_OBSTACLES_ENABLED
            if enabled
            else KeyboardState.GLOBAL_OBSTACLES_DISABLED
        )

    @staticmethod
    def _read_boolean_parameter(response):
        values = getattr(response, 'values', [])
        if len(values) != 1:
            return None, 'parameter response did not contain exactly one value'
        value = values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return None, (
                f'{GLOBAL_OBSTACLE_PARAMETER} is missing or not boolean '
                f'(type={value.type})'
            )
        return value.bool_value, None

    def _start_get_parameters(self) -> None:
        request = GetParameters.Request()
        request.names = [GLOBAL_OBSTACLE_PARAMETER]
        self._costmap_future = self._get_parameters_client.call_async(request)
        self._costmap_phase = 'get'

    def _start_set_parameters(self) -> None:
        request = SetParameters.Request()
        request.parameters = [Parameter(
            name=GLOBAL_OBSTACLE_PARAMETER,
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=self._costmap_requested,
            ),
        )]
        self._costmap_future = self._set_parameters_client.call_async(request)
        self._costmap_phase = 'set'

    def _pump_costmap_controls(self) -> None:
        """Advance costmap services without blocking keyboard reception."""
        now = time.monotonic()
        if self._costmap_action is None:
            if now < self._next_obstacle_refresh:
                return
            self._begin_costmap_action('refresh', now)
        if now >= self._costmap_deadline:
            self.get_logger().warning(
                f'Costmap {self._costmap_action} operation timed out after '
                f'{COSTMAP_SERVICE_TIMEOUT:.1f}s; last confirmed obstacle '
                'state retained'
            )
            self._finish_costmap_action(False, now)
            return
        if self._costmap_phase == 'waiting':
            client = (
                self._clear_costmap_client
                if self._costmap_action == 'clear'
                else self._get_parameters_client
            )
            if not client.service_is_ready():
                return
            try:
                if self._costmap_action == 'clear':
                    self._costmap_future = client.call_async(
                        ClearEntireCostmap.Request()
                    )
                    self._costmap_phase = 'clear'
                else:
                    self._start_get_parameters()
            except Exception as error:  # noqa: B902
                self.get_logger().warning(
                    f'Failed to start costmap {self._costmap_action}: {error}'
                )
                self._finish_costmap_action(False, now)
            return
        if self._costmap_phase == 'waiting_set':
            if not self._set_parameters_client.service_is_ready():
                return
            try:
                self._start_set_parameters()
            except Exception as error:  # noqa: B902
                self.get_logger().warning(
                    f'Failed to request obstacle-layer state '
                    f'{self._costmap_requested}: {error}'
                )
                self._finish_costmap_action(False, now)
            return
        if not self._costmap_future.done():
            return
        try:
            response = self._costmap_future.result()
        except Exception as error:  # noqa: B902
            self.get_logger().warning(
                f'Costmap {self._costmap_action} service failed: {error}; '
                'last confirmed obstacle state retained'
            )
            self._finish_costmap_action(False, now)
            return
        if self._costmap_phase == 'clear':
            if response is None:
                self.get_logger().warning(
                    'Global obstacle clear returned no response'
                )
                self._finish_costmap_action(False, now)
                return
            self.get_logger().info(
                'Global obstacle marks cleared successfully via '
                f'{COSTMAP_CLEAR_SERVICE}'
            )
            self._finish_costmap_action(True, now)
            return
        if self._costmap_phase == 'get':
            enabled, error = self._read_boolean_parameter(response)
            if error is not None:
                self.get_logger().warning(
                    f'Global obstacle-layer read failed: {error}; '
                    'last confirmed state retained'
                )
                self._finish_costmap_action(False, now)
                return
            self._set_global_obstacles_state(enabled)
            if self._costmap_action == 'refresh':
                self.get_logger().info(
                    'Global obstacle-layer state refreshed: '
                    f'enabled={enabled}'
                )
                self._finish_costmap_action(True, now)
                return
            self._costmap_previous = enabled
            self._costmap_requested = not enabled
            self.get_logger().info(
                'Global obstacle-layer toggle read: '
                f'previous={enabled} requested={self._costmap_requested}'
            )
            self._costmap_phase = 'waiting_set'
            self._costmap_future = None
            return
        results = getattr(response, 'results', [])
        if len(results) != 1 or not results[0].successful:
            reason = (
                results[0].reason
                if len(results) == 1 and results[0].reason
                else 'set service did not confirm success'
            )
            self.get_logger().warning(
                'Global obstacle-layer toggle failed: '
                f'previous={self._costmap_previous} '
                f'requested={self._costmap_requested} reason={reason}; '
                'last confirmed state retained'
            )
            self._finish_costmap_action(False, now)
            return
        self._set_global_obstacles_state(self._costmap_requested)
        self.get_logger().info(
            'Global obstacle-layer toggle succeeded: '
            f'previous={self._costmap_previous} '
            f'requested={self._costmap_requested} '
            f'resulting={self._costmap_requested}'
        )
        self._finish_costmap_action(True, now)

    def _on_joy(self, message: Joy) -> None:
        buttons = message.buttons
        physical_control = any(
            index < len(buttons) and buttons[index] == 1
            for index in (X_BUTTON_INDEX, R1_BUTTON_INDEX, L1_BUTTON_INDEX)
        )
        if physical_control:
            self._latch.clear_for_controller()
            self._publish()

    def _publish(self) -> None:
        now = time.monotonic()
        self._latch.expire(now)
        state = self._receiver.state
        message = KeyboardState()
        message.valid = self._receiver.is_live(now)
        if self._latch.armed:
            message.mode = KeyboardState.MODE_SUPPRESS
            if state is not None:
                message.session_id = state.packet.session_id
                message.sequence = state.packet.sequence
        elif state is not None and message.valid:
            message.mode = (
                KeyboardState.MODE_BRAKE
                if state.packet.mode & KeyboardState.MODE_SUPPRESS
                else state.packet.mode
            )
            message.session_id = state.packet.session_id
            message.sequence = state.packet.sequence
            message.throttle = state.throttle
            message.steering = state.packet.steering
        else:
            message.mode = KeyboardState.MODE_BRAKE
        message.global_obstacles_state = getattr(
            self,
            '_global_obstacles_state',
            KeyboardState.GLOBAL_OBSTACLES_UNKNOWN,
        )
        self._publisher.publish(message)
        self._last_published_valid = message.valid

    def _publish_timeout_transition(self) -> None:
        timed_out = not self._receiver.is_live(time.monotonic())
        if self._last_published_valid and timed_out:
            self._publish()

    def destroy_node(self):
        self._socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    try:
        node = KeyboardBridge()
    except (OSError, TypeError, ValueError) as error:
        rclpy.logging.get_logger('keyboard_bridge').error(str(error))
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
