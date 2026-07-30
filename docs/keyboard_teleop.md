# Phase 1 keyboard control and autonomy latch

Keyboard control is a low-speed input for supervised operation. It does not
replace the DualSense: X, R1, and L1 always preempt it. The Pi remains the
command and safety authority.

## Start the sender

Install `pynput` on the laptop, then run:

```bash
python3 tools/keyboard_sender.py RUNNER_IP --port 49321
```

Backtick toggles autonomy arm/disarm. An armed latch persists through sender or
connection loss so intermittent laptop links do not stop a route. Escape is
the primary emergency stop: it clears all held and queued keyboard state,
clears the latch, and sends repeated brake/disarm packets at 20 Hz for at least
one second, even after a brief tap. Repeated Escape presses harmlessly extend
the burst. Taking DualSense control with X, R1, or L1 disarms the keyboard
latch; releasing the controller cannot silently resume it.

The latch expires 600 seconds after the original arm event. Ordinary packets
and reconnect traffic do not refresh it. After expiry or controller takeover,
toggle backtick off and on to explicitly re-arm. Override the lifetime with
`--autonomy-latch-timeout`; the legacy `--autonomy-hold-timeout` spelling
remains accepted.

Space retains its original manual hold-to-run role. While Space is held, W
requests the selected throttle, S requests full brake, and A/D request full
steering. Releasing Space requests brake. Space does not clear the autonomy
latch. `=`/`-` change the throttle setpoint by 0.01 once per physical press.

The navigation bridge and sender both start in WAYPOINT mode. F12 alternates
the sender's locally requested mode between WAYPOINT and ROUTE, printing
`Navigation mode requested: ...`. This is not Pi acknowledgement. Each press
queues three identical absolute `SET_WAYPOINT_MODE` or `SET_ROUTE_MODE`
commands to reduce UDP loss risk; key autorepeat cannot flip the mode again
until F12 is released.

The same Foxglove `/runner/waypoint` `PoseStamped` tool serves both modes and
preserves its complete orientation. In WAYPOINT mode poses enter a
nonpersistent FIFO, execute individually, and are consumed on Nav2 success.
One failure is retried; a second pauses with the failed front retained. F5
resumes, F6 stops while retaining the queue, and F7 clears it. Route-only F8
and F9 commands are ignored in WAYPOINT mode.

In ROUTE mode, F5 start, F6 stop, F7 clear, F8 loop toggle, and F9 undo the
last waypoint retain their persistent-route behavior. Changing modes is
destructive: the bridge cancels active opposite-mode navigation and clears
that mode's complete collection. Repeated receipt of the already-active
absolute mode command is a no-op.

Global costmap controls are F10 to clear currently accumulated global obstacle
marks and F11 to enable or disable the global costmap obstacle layer. F11
always reads the current `obstacle_layer.enabled` parameter before requesting
its inverse. These operator commands do not change the saved static map.

On macOS, grant the terminal or Python launcher Input Monitoring or
Accessibility permission. `pynput` capture is global regardless of window
focus and does not provide a portable application-focus-loss event. Sender
termination does not necessarily disarm the persistent Pi-side latch.
Explicitly disarm before leaving the system unattended. Revisit this safety
posture before Phase 2 racing speeds.

## Pi policy

`keyboard_bridge` listens on UDP port 49321 by default, validates the exact
27-byte version-two packet, including additive absolute navigation commands
8 and 9, applies the default 0.50 forward cap, and publishes
`/teleop/keyboard_state` at 20 Hz. `valid` still reports 150 ms sender
liveness, while `mode=MODE_SUPPRESS` reports an armed Pi latch even when
`valid=false`. Armed keyboard autonomy publishes `teleop_suppress` continuously
on `/teleop/active_mode`; the keyboard-state mode distinguishes its ownership
without changing the active-mode compatibility contract.
`global_obstacles_state` reports the last parameter-service-confirmed global
obstacle-layer state as unknown, disabled, or enabled; transient refresh
failures preserve the last confirmed value.

`allowed_source_ip` may be set to one IPv4 address. When empty, any source is
eligible while no source/session is active. Active source/session locking
reduces accidental collisions only: UDP is neither authenticated nor
encrypted. Use a trusted LAN or Tailscale and do not treat source locking as
authentication.

Protocol-valid throttle is in `[0.0, 1.0]`; a valid request above the Pi cap is
capped. Non-finite or out-of-range fields, malformed packets, unauthorized
sources, and live-session changes are rejected without refreshing liveness.
Valid brake packets are applied before duplicate/old sequence filtering, so
repeated identical emergency packets remain safe and idempotent. Other
duplicate or old packets remain rejected.

This is a deliberate reversal of the previous fail-safe-on-link-loss behavior:
intermittent laptop-link loss was causing mid-route stops. Escape and the
unchanged motor watchdog are now the intended keyboard and independent stop
paths. This posture is accepted only for current low-speed Phase 1 operation
and must be reconsidered before Phase 2 racing speeds, when line-of-sight
controller supervision may again be required.

Route completion does not clear the latch. It should probably do so before a
later goal can move without a new arm, but coupling keyboard safety state to
Nav2 action status is a separate architecture decision.
