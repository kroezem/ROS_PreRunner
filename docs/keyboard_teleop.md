# Supervised keyboard teleoperation

Keyboard teleoperation is a low-speed input for supervised operation. It does
not replace the DualSense: X, R1, and L1 always preempt it. The Pi remains the
command and safety authority.

## Start the sender

Install `pynput` on the laptop, then run:

```bash
python3 tools/keyboard_sender.py RUNNER_IP --port 49321
```

Space is the driving hold-to-run key. While Space is held, W requests the
selected throttle, S requests full brake, and A/D request full steering.
Releasing Space disarms WASD and requests brake. The backtick key enables
autonomy suppression, equivalent to controller L1. A continuous backtick hold
expires after 600 seconds by default and requires release and re-press; override
it with `--autonomy-hold-timeout`. `=`/`-` change the throttle setpoint by
0.01 once per physical press. Escape clears all held-key state and requests
brake.

Route controls are F5 start, F6 stop, F7 clear, F8 loop toggle, and F9 undo
last waypoint. These publish the corresponding command through the Pi bridge
to `/runner/route_control`.

On macOS, grant the terminal or Python launcher Input Monitoring or
Accessibility permission. `pynput` capture is global regardless of window
focus and does not provide a portable application-focus-loss event. Kill the
sender when it is not in use. The Pi timeout is intended to handle laptop
sleep and lid closure, but lid-close and laptop-sleep behavior is untested.

## Pi policy

`keyboard_bridge` listens on UDP port 49321 by default, validates the exact
27-byte version-two packet, applies the default 0.50 forward cap, and publishes
`/teleop/keyboard_state` at 20 Hz. It marks the state invalid after 150 ms
without a valid newer datagram. `teleop_node` independently expires a silent
bridge state after 150 ms.

`allowed_source_ip` may be set to one IPv4 address. When empty, any source is
eligible while no source/session is active. Active source/session locking
reduces accidental collisions only: UDP is neither authenticated nor
encrypted. Use a trusted LAN or Tailscale and do not treat source locking as
authentication.

Protocol-valid throttle is in `[0.0, 1.0]`; a valid request above the Pi cap is
capped. Non-finite or out-of-range fields, malformed packets, duplicate or old
sequences, unauthorized sources, and live-session changes are rejected without
refreshing liveness.

After controller preemption, packet timeout, network loss, or laptop sleep, a
held W cannot resume motion until W is released and pressed again. A held
backtick cannot resume autonomy suppression after its 600-second timeout until
it is released and pressed again.
