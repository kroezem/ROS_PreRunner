#!/usr/bin/env python3
"""Send global keyboard state to Runner over versioned UDP.

W, A, S, D, and Space control the vehicle regardless of which application has
focus. The sender must be killed when not in use.
"""

import argparse
import math
import secrets
import signal
import socket
import struct
import sys
import threading
import time

from pynput import keyboard


MAGIC = b'RKEY'
VERSION = 1
MODE_DRIVE = 0
MODE_BRAKE = 1
MODE_SUPPRESS = 2
PACKET_FORMAT = '!4sBBQIff'
DEFAULT_PORT = 49321
SEND_PERIOD = 0.05
SHUTDOWN_PACKET_COUNT = 5


def port_number(value):
    """Parse a nonzero UDP port."""
    try:
        port = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('port must be an integer') from error
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError('port must be within [1, 65535]')
    return port


def finite_unit(value):
    """Parse a finite normalized value."""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('setpoint must be numeric') from error
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise argparse.ArgumentTypeError('setpoint must be within [0.0, 1.0]')
    return result


def resolve_destination(host, port):
    """Resolve one IPv4 UDP destination or raise a useful error."""
    if not host.strip():
        raise ValueError('destination host must not be empty')
    try:
        results = socket.getaddrinfo(
            host,
            port,
            socket.AF_INET,
            socket.SOCK_DGRAM,
        )
    except socket.gaierror as error:
        raise ValueError(
            f'cannot resolve destination host {host!r}: {error}'
        ) from error
    if not results:
        raise ValueError(f'cannot resolve destination host {host!r}')
    return results[0][4]


class KeyboardInput:
    """Thread-safe pressed-key and discrete setpoint state."""

    def __init__(self, setpoint):
        self.setpoint = setpoint
        self.pressed = set()
        self.lock = threading.Lock()

    @staticmethod
    def token(key):
        if key == keyboard.Key.space:
            return 'space'
        try:
            character = key.char
        except AttributeError:
            return None
        return character.lower() if character else None

    def press(self, key):
        token = self.token(key)
        if token not in {'w', 's', 'a', 'd', '[', ']', 'space'}:
            return
        with self.lock:
            if token in self.pressed:
                return
            self.pressed.add(token)
            if token == '[':
                self.setpoint = round(max(0.0, self.setpoint - 0.01), 2)
                print(
                    f'\rRequested throttle setpoint: {self.setpoint:.2f}  ',
                    end='',
                    flush=True,
                )
            elif token == ']':
                self.setpoint = round(min(1.0, self.setpoint + 0.01), 2)
                print(
                    f'\rRequested throttle setpoint: {self.setpoint:.2f}  ',
                    end='',
                    flush=True,
                )

    def release(self, key):
        token = self.token(key)
        if token is None:
            return
        with self.lock:
            self.pressed.discard(token)

    def state(self):
        with self.lock:
            pressed = set(self.pressed)
            setpoint = self.setpoint
        brake = 's' in pressed
        suppress = 'space' in pressed
        mode = MODE_BRAKE if brake else MODE_DRIVE
        if suppress:
            mode |= MODE_SUPPRESS
        throttle = setpoint if 'w' in pressed and not brake else 0.0
        steering = float('a' in pressed) - float('d' in pressed)
        return mode, throttle, steering


def packet(session_id, sequence, mode, throttle, steering):
    """Pack one deterministic protocol version-one datagram."""
    return struct.pack(
        PACKET_FORMAT,
        MAGIC,
        VERSION,
        mode,
        session_id,
        sequence,
        throttle,
        steering,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Send W/S/A/D supervised keyboard state to Runner. On macOS, '
            'grant the launching terminal Input Monitoring or Accessibility '
            'permission. Window focus loss is not generically detectable by '
            'pynput; Pi timeout is authoritative.'
        )
    )
    parser.add_argument('host', help='Runner IPv4 address or hostname')
    parser.add_argument(
        '--port',
        type=port_number,
        default=DEFAULT_PORT,
        help=f'Runner UDP port (default: {DEFAULT_PORT})',
    )
    parser.add_argument(
        '--initial-throttle',
        type=finite_unit,
        default=0.30,
        help='initial requested W throttle (default: 0.30)',
    )
    args = parser.parse_args(argv)
    try:
        destination = resolve_destination(args.host, args.port)
    except ValueError as error:
        parser.error(str(error))

    input_state = KeyboardInput(args.initial_throttle)
    stop = threading.Event()
    session_id = 0
    while session_id == 0:
        session_id = secrets.randbits(64)
    sequence = 0
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def request_stop(_signum=None, _frame=None):
        stop.set()

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    listener = keyboard.Listener(
        on_press=input_state.press,
        on_release=input_state.release,
    )
    listener.start()
    listener.wait()
    if not getattr(listener, 'IS_TRUSTED', True):
        listener.stop()
        udp_socket.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
        print(
            'Keyboard listener is not trusted by this platform. Grant Input '
            'Monitoring or Accessibility permission and retry.',
            file=sys.stderr,
        )
        return 2
    print(
        f'Sending to {destination[0]}:{destination[1]} at 20 Hz; '
        f'session={session_id}; requested throttle={args.initial_throttle:.2f}'
    )
    print(
        'WARNING: W, A, S, D, and Space control the vehicle regardless of '
        'which application has focus.'
    )
    print('WARNING: The sender must be killed when not in use.')
    print('W=forward S=brake A/D=steer Space=suppress [/] adjust')
    exit_code = 0
    try:
        next_send = time.monotonic()
        while not stop.is_set():
            if not listener.is_alive():
                print(
                    '\nKeyboard listener stopped unexpectedly.',
                    file=sys.stderr,
                )
                exit_code = 1
                break
            mode, throttle, steering = input_state.state()
            udp_socket.sendto(
                packet(
                    session_id,
                    sequence,
                    mode,
                    throttle,
                    steering,
                ),
                destination,
            )
            sequence = (sequence + 1) & 0xffffffff
            next_send += SEND_PERIOD
            stop.wait(max(0.0, next_send - time.monotonic()))
    except KeyboardInterrupt:
        pass
    except OSError as error:
        print(f'\nUDP send failed: {error}', file=sys.stderr)
        exit_code = 1
    finally:
        listener.stop()
        for _ in range(SHUTDOWN_PACKET_COUNT):
            try:
                udp_socket.sendto(
                    packet(
                        session_id,
                        sequence,
                        MODE_BRAKE,
                        0.0,
                        0.0,
                    ),
                    destination,
                )
                sequence = (sequence + 1) & 0xffffffff
                time.sleep(0.01)
            except OSError:
                break
        udp_socket.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
        print('\nSender stopped; shutdown brake packets attempted.')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
