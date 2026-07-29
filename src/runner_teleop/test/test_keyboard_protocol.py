import math
import struct

import pytest

from runner_interfaces.msg import KeyboardState
from runner_teleop.keyboard_bridge import _validate_configuration
from runner_teleop.keyboard_bridge import KeyboardAutonomyLatch
from runner_teleop.keyboard_bridge import KeyboardBridge
from runner_teleop.keyboard_bridge import KeyboardReceiver
from runner_teleop.keyboard_bridge import ROUTE_COMMAND_NAMES
from runner_teleop.keyboard_protocol import decode_packet
from runner_teleop.keyboard_protocol import MAGIC
from runner_teleop.keyboard_protocol import MODE_BRAKE_SUPPRESS
from runner_teleop.keyboard_protocol import PACKET_FORMAT
from runner_teleop.keyboard_protocol import PACKET_SIZE
from runner_teleop.keyboard_protocol import PacketError
from runner_teleop.keyboard_protocol import ROUTE_LOOP_TOGGLE
from runner_teleop.keyboard_protocol import ROUTE_REMOVE_LAST
from runner_teleop.keyboard_protocol import VERSION
from sensor_msgs.msg import Joy


def packet(
    *,
    magic=MAGIC,
    version=VERSION,
    mode=0,
    session=1,
    sequence=1,
    throttle=0.30,
    steering=0.0,
    route_command=0,
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
        route_command,
    )


def test_packet_contract_is_exact_and_mode_combinations_are_supported():
    assert PACKET_SIZE == 27

    decoded = decode_packet(
        packet(
            mode=MODE_BRAKE_SUPPRESS,
            sequence=0xffffffff,
            throttle=1.0,
            steering=-1.0,
            route_command=ROUTE_LOOP_TOGGLE,
        )
    )

    assert decoded.mode == MODE_BRAKE_SUPPRESS
    assert decoded.sequence == 0xffffffff
    assert decoded.throttle == 1.0
    assert decoded.steering == -1.0
    assert decoded.route_command == ROUTE_LOOP_TOGGLE


def test_route_command_codes_map_to_bridge_commands():
    assert ROUTE_COMMAND_NAMES[ROUTE_LOOP_TOGGLE] == 'loop_toggle'
    assert ROUTE_COMMAND_NAMES[ROUTE_REMOVE_LAST] == 'remove_last'


@pytest.mark.parametrize(
    'data',
    [
        packet()[:-1],
        packet() + b'x',
        packet(magic=b'NOPE'),
        packet(version=1),
        packet(mode=4),
        packet(session=0),
        packet(throttle=math.nan),
        packet(throttle=math.inf),
        packet(throttle=-0.01),
        packet(throttle=1.20),
        packet(steering=math.nan),
        packet(steering=-1.01),
        packet(steering=1.01),
        packet(route_command=6),
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


def test_escape_brake_safety_mutation_precedes_duplicate_sequence_rejection():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)
    latch = KeyboardAutonomyLatch(timeout=600.0)
    latch.process_mode(2, 0.0)
    data = packet(mode=1, sequence=7)
    decoded, error = receiver.inspect(data, '100.64.0.2')
    assert error is None

    assert latch.process_mode(decoded.mode, 1.0)
    assert not latch.armed
    accepted, error, _ = receiver.accept_packet(
        decoded, '100.64.0.2', 1.0
    )
    latch.process_mode(2, 1.005)
    assert latch.armed
    assert latch.process_mode(decoded.mode, 1.01)
    assert not latch.armed
    duplicate, duplicate_error, _ = receiver.accept_packet(
        decoded, '100.64.0.2', 1.01
    )

    assert accepted is not None
    assert duplicate is None
    assert 'duplicate' in duplicate_error
    assert decoded.mode == 1


def test_out_of_order_escape_brake_still_applies_safety_mutation():
    receiver = KeyboardReceiver(speed_cap=0.50, timeout=0.15)
    latch = KeyboardAutonomyLatch(timeout=600.0)
    receiver.accept(packet(mode=2, sequence=10), '100.64.0.2', 1.0)
    latch.process_mode(2, 1.0)
    decoded, error = receiver.inspect(
        packet(mode=1, sequence=9),
        '100.64.0.2',
    )
    assert error is None

    assert latch.process_mode(decoded.mode, 1.01)
    assert not latch.armed
    accepted, rejection, _ = receiver.accept_packet(
        decoded,
        '100.64.0.2',
        1.01,
    )
    assert accepted is None
    assert 'reordered/old' in rejection


def test_latch_arm_disarm_sender_loss_and_nonrefreshing_expiry():
    latch = KeyboardAutonomyLatch(timeout=600.0)
    assert not latch.armed

    assert not latch.process_mode(2, 10.0)
    assert latch.armed
    assert latch.armed_at == 10.0
    latch.process_mode(2, 500.0)
    assert latch.armed_at == 10.0
    assert not latch.expire(609.999)
    assert latch.armed
    assert latch.expire(610.0)
    assert not latch.armed

    # Continuing sender traffic cannot silently re-arm after expiry.
    latch.process_mode(2, 611.0)
    assert not latch.armed
    latch.process_mode(1, 612.0)
    latch.process_mode(2, 613.0)
    assert latch.armed


def test_latch_repeated_disarm_is_idempotent_and_immediate():
    latch = KeyboardAutonomyLatch(timeout=600.0)
    latch.process_mode(2, 0.0)

    assert latch.process_mode(1, 0.1)
    assert not latch.armed
    for now in (0.2, 0.3, 1.1):
        assert latch.process_mode(1, now)
        assert not latch.armed


def test_drop_two_thirds_of_escape_burst_and_first_retained_packet_clears():
    latch = KeyboardAutonomyLatch(timeout=600.0)
    latch.process_mode(2, 0.0)
    retained_indices = []

    for index in range(21):
        if index % 3:
            continue
        retained_indices.append(index)
        latch.process_mode(1, index * 0.05)
        assert not latch.armed

    assert retained_indices == [0, 3, 6, 9, 12, 15, 18]
    assert len(retained_indices) == 7
    assert 21 - len(retained_indices) == 14
    assert not latch.armed


def test_keyboard_diagnostic_preserves_latch_across_sender_timeout(
    monkeypatch,
):
    now = [10.0]
    monkeypatch.setattr(
        'runner_teleop.keyboard_bridge.time.monotonic',
        lambda: now[0],
    )
    bridge = KeyboardBridge.__new__(KeyboardBridge)
    bridge._receiver = KeyboardReceiver(speed_cap=0.5, timeout=0.15)
    bridge._latch = KeyboardAutonomyLatch(timeout=600.0)
    bridge._publisher = type(
        'Publisher',
        (),
        {'messages': [], 'publish': lambda self, msg: self.messages.append(msg)},
    )()
    bridge._last_published_valid = False
    bridge._receiver.accept(
        packet(mode=2, sequence=1), '100.64.0.2', now[0]
    )
    bridge._latch.process_mode(2, now[0])

    now[0] = 10.151
    bridge._publish()

    diagnostic = bridge._publisher.messages[-1]
    assert not diagnostic.valid
    assert diagnostic.mode == KeyboardState.MODE_SUPPRESS
    assert bridge._latch.armed

    now[0] = 610.0
    bridge._publish()
    diagnostic = bridge._publisher.messages[-1]
    assert not diagnostic.valid
    assert diagnostic.mode == KeyboardState.MODE_BRAKE
    assert not bridge._latch.armed


def test_sender_timeout_while_disarmed_remains_disarmed(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(
        'runner_teleop.keyboard_bridge.time.monotonic',
        lambda: now[0],
    )
    bridge = KeyboardBridge.__new__(KeyboardBridge)
    bridge._receiver = KeyboardReceiver(speed_cap=0.5, timeout=0.15)
    bridge._latch = KeyboardAutonomyLatch(timeout=600.0)
    bridge._publisher = type(
        'Publisher',
        (),
        {'messages': [], 'publish': lambda self, msg: self.messages.append(msg)},
    )()
    bridge._last_published_valid = False
    bridge._receiver.accept(packet(mode=0), '100.64.0.2', now[0])

    now[0] = 10.151
    bridge._publish()

    diagnostic = bridge._publisher.messages[-1]
    assert not diagnostic.valid
    assert diagnostic.mode == KeyboardState.MODE_BRAKE
    assert not bridge._latch.armed


@pytest.mark.parametrize('button_index', [0, 5, 4])
def test_each_controller_path_requires_explicit_sender_rearm(button_index):
    bridge = KeyboardBridge.__new__(KeyboardBridge)
    bridge._latch = KeyboardAutonomyLatch(timeout=600.0)
    bridge._publish = lambda: None
    bridge._latch.process_mode(2, 0.0)
    buttons = [0] * 6
    buttons[button_index] = 1

    bridge._on_joy(Joy(buttons=buttons))
    assert not bridge._latch.armed

    bridge._on_joy(Joy(buttons=[0] * 6))
    bridge._latch.process_mode(2, 0.1)
    assert not bridge._latch.armed
    bridge._latch.process_mode(1, 0.2)
    bridge._latch.process_mode(2, 0.3)
    assert bridge._latch.armed


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
        {'autonomy_latch_timeout': 0.0},
        {'autonomy_latch_timeout': math.inf},
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
        'autonomy_latch_timeout': 600.0,
    }
    values.update(changes)

    with pytest.raises(ValueError):
        _validate_configuration(**values)
