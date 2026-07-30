#!/usr/bin/env python3
"""Send latched autonomy and keyboard state to Runner over UDP."""

import argparse
import math
import secrets
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque

MAGIC = b'RKEY'
VERSION = 2
MODE_DRIVE = 0
MODE_BRAKE = 1
MODE_SUPPRESS = 2
ROUTE_NONE = 0
ROUTE_START = 1
ROUTE_STOP = 2
ROUTE_CLEAR = 3
ROUTE_LOOP_TOGGLE = 4
ROUTE_REMOVE_LAST = 5
ROUTE_CLEAR_GLOBAL_OBSTACLES = 6
ROUTE_TOGGLE_GLOBAL_OBSTACLES = 7
SET_WAYPOINT_MODE = 8
SET_ROUTE_MODE = 9
PACKET_FORMAT = '!4sBBQIffB'
DEFAULT_PORT = 49321
SEND_PERIOD = 0.05
MODE_COMMAND_BURST_COUNT = 3
# One extra cadence period guarantees a full second between transmission
# opportunities even when Escape arrives immediately after a send.
DISARM_BURST_SECONDS = 1.0 + SEND_PERIOD
DEFAULT_AUTONOMY_LATCH_TIMEOUT = 600.0
DEFAULT_AUTONOMY_HOLD_TIMEOUT = DEFAULT_AUTONOMY_LATCH_TIMEOUT
ROUTE_KEYS = {
    'route_start': ROUTE_START,
    'route_stop': ROUTE_STOP,
    'route_clear': ROUTE_CLEAR,
    'route_loop_toggle': ROUTE_LOOP_TOGGLE,
    'route_remove_last': ROUTE_REMOVE_LAST,
    'clear_global_obstacles': ROUTE_CLEAR_GLOBAL_OBSTACLES,
    'toggle_global_obstacles': ROUTE_TOGGLE_GLOBAL_OBSTACLES,
}


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


def positive_seconds(value):
    """Parse a finite positive duration."""
    try:
        result = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('duration must be numeric') from error
    if not math.isfinite(result) or result <= 0.0:
        raise argparse.ArgumentTypeError(
            'duration must be finite and greater than zero'
        )
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

    def __init__(self, setpoint, autonomy_latch_timeout):
        self.setpoint = setpoint
        self.autonomy_latch_timeout = autonomy_latch_timeout
        self.pressed = set()
        self.route_commands = deque()
        self.navigation_mode = 'WAYPOINT'
        self.autonomy_armed_at = None
        self.autonomy_armed = False
        self.disarm_until = None
        self.lock = threading.Lock()

    def press(self, token, now=None):
        """Apply one independent normalized keydown event."""
        if token == 'escape':
            self.clear(now=now)
            return
        valid = {
            'w', 's', 'a', 'd', 'space', '`', '=', '-', 'navigation_mode',
            *ROUTE_KEYS,
        }
        if token not in valid:
            return
        timestamp = time.monotonic() if now is None else now
        with self.lock:
            if token in self.pressed:
                return
            self.pressed.add(token)
            if token == '-':
                self.setpoint = round(max(0.0, self.setpoint - 0.01), 2)
                print(
                    f'\rRequested throttle setpoint: {self.setpoint:.2f}  ',
                    end='',
                    flush=True,
                )
            elif token == '=':
                self.setpoint = round(min(1.0, self.setpoint + 0.01), 2)
                print(
                    f'\rRequested throttle setpoint: {self.setpoint:.2f}  ',
                    end='',
                    flush=True,
                )
            elif token == '`':
                if self.autonomy_armed:
                    self._disarm(timestamp, SEND_PERIOD)
                else:
                    self.autonomy_armed = True
                    self.autonomy_armed_at = timestamp
                    self.disarm_until = None
            elif token in ROUTE_KEYS:
                self.route_commands.append(ROUTE_KEYS[token])
            elif token == 'navigation_mode':
                if self.navigation_mode == 'WAYPOINT':
                    self.navigation_mode = 'ROUTE'
                    command = SET_ROUTE_MODE
                else:
                    self.navigation_mode = 'WAYPOINT'
                    command = SET_WAYPOINT_MODE
                self.route_commands.extend(
                    [command] * MODE_COMMAND_BURST_COUNT
                )
                print(
                    f'\nNavigation mode requested: {self.navigation_mode}',
                    flush=True,
                )

    def release(self, token):
        """Apply one independent normalized keyup event."""
        with self.lock:
            self.pressed.discard(token)

    def clear(self, now=None):
        """Clear held/queued state and begin an emergency brake burst."""
        timestamp = time.monotonic() if now is None else now
        with self.lock:
            self.pressed.clear()
            self.route_commands.clear()
            self._disarm(timestamp, DISARM_BURST_SECONDS)

    def _disarm(self, now, duration):
        self.autonomy_armed = False
        self.autonomy_armed_at = None
        until = now + duration
        self.disarm_until = max(self.disarm_until or until, until)

    def state(self, now=None):
        """Return one packet state and consume at most one route command."""
        timestamp = time.monotonic() if now is None else now
        with self.lock:
            pressed = set(self.pressed)
            setpoint = self.setpoint
            if (
                self.autonomy_armed
                and self.autonomy_armed_at is not None
                and timestamp - self.autonomy_armed_at
                >= self.autonomy_latch_timeout
            ):
                self._disarm(timestamp, SEND_PERIOD)
            suppress = self.autonomy_armed
            disarming = (
                self.disarm_until is not None
                and timestamp <= self.disarm_until
            )
            route_command = (
                self.route_commands.popleft()
                if self.route_commands else ROUTE_NONE
            )
        if disarming:
            return MODE_BRAKE, 0.0, 0.0, route_command
        if suppress:
            return MODE_SUPPRESS, 0.0, 0.0, route_command
        if 'space' not in pressed or 's' in pressed:
            return MODE_BRAKE, 0.0, 0.0, route_command
        throttle = setpoint if 'w' in pressed else 0.0
        steering = float('a' in pressed) - float('d' in pressed)
        return MODE_DRIVE, throttle, steering, route_command


def packet(
    session_id,
    sequence,
    mode,
    throttle,
    steering,
    route_command=ROUTE_NONE,
):
    """Pack one deterministic protocol version-two datagram."""
    return struct.pack(
        PACKET_FORMAT,
        MAGIC,
        VERSION,
        mode,
        session_id,
        sequence,
        throttle,
        steering,
        route_command,
    )


def close_sender(listener, udp_socket):
    """Stop local resources without transmitting an implicit disarm."""
    listener.stop()
    udp_socket.close()


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
    parser.add_argument(
        '--autonomy-latch-timeout',
        '--autonomy-hold-timeout',
        dest='autonomy_latch_timeout',
        type=positive_seconds,
        default=DEFAULT_AUTONOMY_LATCH_TIMEOUT,
        help='maximum autonomy latch lifetime in seconds (default: 600)',
    )
    args = parser.parse_args(argv)
    try:
        destination = resolve_destination(args.host, args.port)
    except ValueError as error:
        parser.error(str(error))

    input_state = KeyboardInput(
        args.initial_throttle,
        args.autonomy_latch_timeout,
    )
    stop = threading.Event()
    session_id = 0
    while session_id == 0:
        session_id = secrets.randbits(64)
    sequence = 0
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def request_stop(_signum=None, _frame=None):
        stop.set()

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    from pynput import keyboard

    special_keys = {
        keyboard.Key.space: 'space',
        keyboard.Key.esc: 'escape',
        keyboard.Key.f5: 'route_start',
        keyboard.Key.f6: 'route_stop',
        keyboard.Key.f7: 'route_clear',
        keyboard.Key.f8: 'route_loop_toggle',
        keyboard.Key.f9: 'route_remove_last',
        keyboard.Key.f10: 'clear_global_obstacles',
        keyboard.Key.f11: 'toggle_global_obstacles',
        keyboard.Key.f12: 'navigation_mode',
    }

    def token(key):
        if key in special_keys:
            return special_keys[key]
        try:
            character = key.char
        except AttributeError:
            return None
        return character.lower() if character else None

    listener = keyboard.Listener(
        on_press=lambda key: input_state.press(token(key)),
        on_release=lambda key: input_state.release(token(key)),
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
        'WARNING: keyboard capture is GLOBAL regardless of window focus.'
    )
    print(
        'Backtick toggles autonomy arm/disarm. Arming persists through sender '
        'or connection loss.'
    )
    print(
        'Escape is the PRIMARY EMERGENCY STOP: it clears held keyboard state '
        'and the latch, then sends repeated brake/disarm packets for at least '
        'one second.'
    )
    print(
        'DualSense X, R1, or L1 clears the latch. '
        f'The latch expires after {args.autonomy_latch_timeout:g} seconds.'
    )
    print(
        'Space=hold-to-run; while held W=throttle S=brake A/D=steer; '
        '=/-=setpoint up/down'
    )
    print(
        'Sender termination does not necessarily disarm the Pi-side latch. '
        'Explicitly disarm before leaving the system unattended.'
    )
    print(
        'Revisit this safety posture before Phase 2 racing speeds.'
    )
    print(
        'Routes: F5=start F6=stop F7=clear F8=loop toggle '
        'F9=undo last waypoint'
    )
    print(
        'Navigation mode: default WAYPOINT; F12 alternates requested '
        'WAYPOINT/ROUTE mode'
    )
    print(
        'Costmap: F10=clear accumulated global obstacle marks; '
        'F11=enable or disable the global costmap obstacle layer'
    )
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
            mode, throttle, steering, route_command = input_state.state()
            udp_socket.sendto(
                packet(
                    session_id,
                    sequence,
                    mode,
                    throttle,
                    steering,
                    route_command,
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
        close_sender(listener, udp_socket)
        signal.signal(signal.SIGTERM, previous_sigterm)
        print(
            '\nSender stopped; no shutdown disarm was sent. Explicitly disarm '
            'the Pi-side latch before leaving the system unattended.'
        )
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
