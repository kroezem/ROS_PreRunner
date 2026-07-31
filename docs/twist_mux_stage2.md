# Stage 2 command arbitration

Stage 2 creates one controlled connection to the normalized motor command
topic. It does not start or configure a Nav2 controller:

```text
/joy -> runner_teleop -> /cmd_vel_teleop --+
                                            +-> twist_mux -> /cmd_vel
/cmd_vel_nav -> drive_adapter -> /cmd_vel_auto --+             |
                                                               v
                                                          motor_driver
```

The topic contracts are intentionally different on the two sides of the
adapter:

| Topic | Type | `linear.x` | `angular.z` | Owner |
|---|---|---|---|---|
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | m/s | rad/s | external controlled test publisher in Stage 2 |
| `/cmd_vel_auto` | `geometry_msgs/msg/Twist` | normalized forward-only command, 0.0…0.70 | normalized steering | `drive_adapter` |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | normalized signed motor command; negative is intentional L2 reverse only | normalized steering | `runner_teleop` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | normalized signed motor command | normalized steering | `twist_mux` |

`motor_driver` remains the only `/cmd_vel` consumer requiring normalized
motor commands and the sole PWM owner. Nav2-unit commands must never be
connected directly to `/cmd_vel`.

## Arbitration and timing

The Jazzy `twist_mux` input priorities are teleop `100` and autonomy `50`.
No lock topics are configured. Any fresh teleop publication therefore
preempts autonomy in the mux input callback, without waiting for a timeout.
This includes manual X, fixed-throttle R1, and the normal zero-brake command.

The teleop timeout is `0.15 s`. Teleop normally publishes at 20 Hz, so three
nominal periods fit in the timeout: one missed 50 ms publication does not
cause fallthrough. When L1 selects `teleop_suppress`, teleop publishes no
command. A continuously publishing autonomy input can first pass on its next
callback after the last teleop command is older than 0.15 s. Mux scheduling
therefore adds up to approximately one 50 ms autonomy period. L1 is
`teleop_suppress`; it is not, by itself, an autonomy arming gate.

The autonomy timeout is `0.30 s`. The adapter publishes at 20 Hz and becomes
silent when `/cmd_vel_nav` is older than `0.25 s`, so its last command ages
out of mux eligibility shortly after adapter silence. The mux is
input-event-driven: it forwards accepted fresh callbacks and does not
periodically republish a last command. Consequently, once all upstream
publications stop, `/cmd_vel` is immediately event-silent; expiry prevents a
later lower-priority callback from being masked by an old command.

`motor_driver` retains its independent 0.20 s watchdog and applies MD13S active
brake (duty zero) after `/cmd_vel` loss. If `twist_mux` exits, `/cmd_vel` becomes silent
and the watchdog brakes. If `drive_adapter` exits during L1 suppression,
`/cmd_vel_auto` and then `/cmd_vel` become silent and the watchdog brakes.
If teleop exits with no autonomy traffic, the watchdog brakes after the last
mux output. If teleop exits while autonomy is publishing, the mux may fall
through to autonomy after the 0.15 s teleop timeout; that is the ratified
priority-and-timeout behavior.

The Stage 2 bench launch is:

```bash
ros2 launch runner_bringup autonomy_bench.launch.py
```

It starts only `joy_node`, `runner_teleop`, `drive_adapter`, `twist_mux`, and
`encoder_node`; the persistent `runner-motor.service` owns `motor_driver`.
A controlled publisher may supply
`/cmd_vel_nav`; no planner, controller server, costmap, behavior-tree
navigator, or autonomous goal is part of this launch.

For evidence, record or inspect `/cmd_vel_teleop`, `/cmd_vel_auto`,
`/cmd_vel`, `/teleop/active_mode`, `/drive_adapter/state`, `/motor/direction`,
and `/wheel/encoder_state`. `twist_mux` also publishes standard
`/diagnostics`; source transitions should still be established by correlating
timestamped input and output events, not by command values alone.

## Stage 3 handoff

The adapter clamps infeasible steering at the measured maximum while preserving
forward speed. Stage 2 must validate the provisional feedforward, PI gains,
promotion thresholds, integrator bounds, and the new zero output floor without
turning PI deceleration into a reverse command.
