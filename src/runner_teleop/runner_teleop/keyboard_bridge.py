"""Receive supervised keyboard UDP state and publish safe Pi-owned state."""

from dataclasses import dataclass
import ipaddress
import socket
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from runner_interfaces.msg import KeyboardState

from runner_teleop.keyboard_protocol import decode_packet
from runner_teleop.keyboard_protocol import is_newer_sequence
from runner_teleop.keyboard_protocol import KeyboardPacket
from runner_teleop.keyboard_protocol import PACKET_SIZE
from runner_teleop.keyboard_protocol import PacketError
from runner_teleop.keyboard_protocol import sequence_delta


DEFAULT_PORT = 49321
DEFAULT_TIMEOUT = 0.15
DEFAULT_SPEED_CAP = 0.50
DEFAULT_PUBLICATION_RATE = 20.0
WARNING_PERIOD = 2.0


@dataclass(frozen=True)
class AcceptedState:
    """One accepted and Pi-capped keyboard state."""

    packet: KeyboardPacket
    source_ip: str
    accepted_at: float
    throttle: float


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
        if self.allowed_source_ip and source_ip != self.allowed_source_ip:
            return None, f'unauthorized source {source_ip}', None
        try:
            packet = decode_packet(data)
        except PacketError as error:
            return None, str(error), None

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
        bind_address = self.get_parameter('bind_address').value
        port = self.get_parameter('port').value
        allowed_source_ip = self.get_parameter(
            'allowed_source_ip'
        ).value
        timeout = self.get_parameter('input_timeout').value
        speed_cap = self.get_parameter('speed_cap').value
        publication_rate = self.get_parameter('publication_rate').value
        _validate_configuration(
            bind_address,
            port,
            allowed_source_ip,
            timeout,
            speed_cap,
            publication_rate,
        )

        self._receiver = KeyboardReceiver(
            speed_cap,
            timeout,
            allowed_source_ip,
        )
        self._warnings = {}
        self._last_published_valid = False
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.bind((bind_address, port))
        self._publisher = self.create_publisher(
            KeyboardState,
            '/teleop/keyboard_state',
            10,
        )
        self.create_timer(0.005, self._receive)
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
            f'timeout={timeout:.3f}s speed_cap={speed_cap:.3f}'
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
            accepted, error, gap = self._receiver.accept(
                data,
                address[0],
                now,
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

    def _publish(self) -> None:
        now = time.monotonic()
        state = self._receiver.state
        message = KeyboardState()
        message.valid = self._receiver.is_live(now)
        if state is not None:
            message.mode = state.packet.mode
            message.session_id = state.packet.session_id
            message.sequence = state.packet.sequence
            if message.valid:
                message.throttle = state.throttle
                message.steering = state.packet.steering
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
