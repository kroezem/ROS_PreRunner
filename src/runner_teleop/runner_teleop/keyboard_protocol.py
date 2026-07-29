"""Deterministic wire protocol for supervised keyboard teleoperation."""

from dataclasses import dataclass
import math
import struct


MAGIC = b'RKEY'
VERSION = 2
MODE_DRIVE = 0
MODE_BRAKE = 1
MODE_SUPPRESS = 2
MODE_BRAKE_SUPPRESS = MODE_BRAKE | MODE_SUPPRESS
VALID_MODE_MASK = MODE_BRAKE | MODE_SUPPRESS
ROUTE_NONE = 0
ROUTE_START = 1
ROUTE_STOP = 2
ROUTE_CLEAR = 3
ROUTE_LOOP_TOGGLE = 4
ROUTE_REMOVE_LAST = 5
ROUTE_CLEAR_GLOBAL_OBSTACLES = 6
ROUTE_TOGGLE_GLOBAL_OBSTACLES = 7
VALID_ROUTE_COMMANDS = {
    ROUTE_NONE,
    ROUTE_START,
    ROUTE_STOP,
    ROUTE_CLEAR,
    ROUTE_LOOP_TOGGLE,
    ROUTE_REMOVE_LAST,
    ROUTE_CLEAR_GLOBAL_OBSTACLES,
    ROUTE_TOGGLE_GLOBAL_OBSTACLES,
}
PACKET_FORMAT = '!4sBBQIffB'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)
SERIAL_HALF_RANGE = 1 << 31
SERIAL_MASK = (1 << 32) - 1


class PacketError(ValueError):
    """Raised when a datagram violates the keyboard protocol."""


@dataclass(frozen=True)
class KeyboardPacket:
    """Validated sender state carried by one UDP datagram."""

    mode: int
    session_id: int
    sequence: int
    throttle: float
    steering: float
    route_command: int


def decode_packet(data: bytes) -> KeyboardPacket:
    """Decode and validate one exact-length version-one datagram."""
    if len(data) != PACKET_SIZE:
        raise PacketError(
            f'wrong packet length {len(data)}; expected {PACKET_SIZE}'
        )
    (
        magic,
        version,
        mode,
        session_id,
        sequence,
        throttle,
        steering,
        route_command,
    ) = (
        struct.unpack(PACKET_FORMAT, data)
    )
    if magic != MAGIC:
        raise PacketError('wrong packet magic')
    if version != VERSION:
        raise PacketError(f'unsupported protocol version {version}')
    if mode & ~VALID_MODE_MASK:
        raise PacketError(f'unsupported mode flags 0x{mode:02x}')
    if session_id == 0:
        raise PacketError('session ID must be nonzero')
    if not math.isfinite(throttle):
        raise PacketError('throttle must be finite')
    if not math.isfinite(steering):
        raise PacketError('steering must be finite')
    if not 0.0 <= throttle <= 1.0:
        raise PacketError('throttle must be within [0.0, 1.0]')
    if not -1.0 <= steering <= 1.0:
        raise PacketError('steering must be within [-1.0, 1.0]')
    if route_command not in VALID_ROUTE_COMMANDS:
        raise PacketError(f'unsupported route command {route_command}')
    return KeyboardPacket(
        mode=mode,
        session_id=session_id,
        sequence=sequence,
        throttle=throttle,
        steering=steering,
        route_command=route_command,
    )


def sequence_delta(new: int, previous: int) -> int:
    """Return the deliberate uint32 serial-number delta."""
    return (new - previous) & SERIAL_MASK


def is_newer_sequence(new: int, previous: int) -> bool:
    """Apply RFC-1982-style ordering to uint32 sequence values."""
    delta = sequence_delta(new, previous)
    return 0 < delta < SERIAL_HALF_RANGE
