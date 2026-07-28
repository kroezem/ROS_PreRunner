import math
import struct

import pytest

from runner_teleop.keyboard_bridge import _validate_configuration
from runner_teleop.keyboard_bridge import KeyboardReceiver
from runner_teleop.keyboard_protocol import decode_packet
from runner_teleop.keyboard_protocol import MAGIC
from runner_teleop.keyboard_protocol import MODE_BRAKE_SUPPRESS
from runner_teleop.keyboard_protocol import PACKET_FORMAT
from runner_teleop.keyboard_protocol import PACKET_SIZE
from runner_teleop.keyboard_protocol import PacketError
from runner_teleop.keyboard_protocol import VERSION


def packet(
    *,
    magic=MAGIC,
    version=VERSION,
    mode=0,
    session=1,
    sequence=1,
    throttle=0.30,
    steering=0.0,
):
    return struct.pack(
        PACKET_FORMAT,
        magic,
        version,
        mode,
        session,
        sequence,
        throttle,
        steering,
    )


def test_packet_contract_is_exact_and_mode_combinations_are_supported():
    assert PACKET_SIZE == 26

    decoded = decode_packet(
        packet(
            mode=MODE_BRAKE_SUPPRESS,
            sequence=0xffffffff,
            throttle=1.0,
            steering=-1.0,
        )
    )

    assert decoded.mode == MODE_BRAKE_SUPPRESS
    assert decoded.sequence == 0xffffffff
    assert decoded.throttle == 1.0
    assert decoded.steering == -1.0


@pytest.mark.parametrize(
    'data',
    [
        packet()[:-1],
        packet() + b'x',
        packet(magic=b'NOPE'),
        packet(version=2),
        packet(mode=4),
        packet(session=0),
        packet(throttle=math.nan),
        packet(throttle=math.inf),
        packet(throttle=-0.01),
        packet(throttle=1.20),
        packet(steering=math.nan),
        packet(steering=-1.01),
        packet(steering=1.01),
    ],
)
def test_malformed_and_invalid_packets_are_rejected(data):
    with pytest.raises(PacketError):
        decode_packet(data)


def test_valid_above_cap_is_capped_after_protocol_validation():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)

    accepted, error, gap = receiver.accept(
        packet(throttle=0.80),
        '100.64.0.2',
        1.0,
    )

    assert error is None
    assert gap is None
    assert accepted.packet.throttle == pytest.approx(0.80)
    assert accepted.throttle == pytest.approx(0.50)
    assert receiver.is_live(1.15)


def test_invalid_packet_does_not_refresh_liveness():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)
    receiver.accept(packet(sequence=1), '100.64.0.2', 1.0)

    accepted, error, _ = receiver.accept(
        packet(sequence=2, throttle=1.20),
        '100.64.0.2',
        1.14,
    )

    assert accepted is None
    assert 'throttle' in error
    assert not receiver.is_live(1.151)
    assert receiver.state.packet.sequence == 1


def test_duplicate_reorder_gap_and_wraparound_rules():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)
    receiver.accept(
        packet(sequence=0xfffffffe),
        '100.64.0.2',
        1.0,
    )

    duplicate, duplicate_error, _ = receiver.accept(
        packet(sequence=0xfffffffe),
        '100.64.0.2',
        1.01,
    )
    wrapped, wrapped_error, gap = receiver.accept(
        packet(sequence=1),
        '100.64.0.2',
        1.02,
    )
    reordered, reordered_error, _ = receiver.accept(
        packet(sequence=0),
        '100.64.0.2',
        1.03,
    )

    assert duplicate is None
    assert 'duplicate' in duplicate_error
    assert wrapped_error is None
    assert gap == 2
    assert wrapped.packet.sequence == 1
    assert reordered is None
    assert 'reordered/old' in reordered_error
    assert receiver.state.accepted_at == 1.02


def test_source_and_session_locking_and_restart_after_timeout():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)
    receiver.accept(packet(session=10), '100.64.0.2', 1.0)

    changed, error, _ = receiver.accept(
        packet(session=11),
        '100.64.0.2',
        1.10,
    )
    restarted, restart_error, _ = receiver.accept(
        packet(session=11),
        '100.64.0.2',
        1.151,
    )

    assert changed is None
    assert 'while input is live' in error
    assert restart_error is None
    assert restarted.packet.session_id == 11


def test_allowed_source_rejection_does_not_refresh_liveness():
    receiver = KeyboardReceiver(
        speed_cap=0.50,
        timeout=0.15,
        allowed_source_ip='100.64.0.2',
    )
    receiver.accept(packet(sequence=1), '100.64.0.2', 1.0)

    accepted, error, _ = receiver.accept(
        packet(sequence=2),
        '192.168.1.10',
        1.14,
    )

    assert accepted is None
    assert 'unauthorized source' in error
    assert not receiver.is_live(1.151)


@pytest.mark.parametrize(
    'changes',
    [
        {'bind_address': 'localhost'},
        {'port': 0},
        {'port': 65536},
        {'allowed_source_ip': 'not-an-ip'},
        {'timeout': 0.0},
        {'speed_cap': -0.01},
        {'speed_cap': 1.01},
        {'publication_rate': 0.0},
    ],
)
def test_invalid_bridge_configuration_is_rejected(changes):
    values = {
        'bind_address': '0.0.0.0',
        'port': 49321,
        'allowed_source_ip': '',
        'timeout': 0.15,
        'speed_cap': 0.50,
        'publication_rate': 20.0,
    }
    values.update(changes)

    with pytest.raises(ValueError):
        _validate_configuration(**values)
