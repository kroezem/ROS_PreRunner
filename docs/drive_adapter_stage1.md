# Runner Phase 1 Stage 1 drive adapter

This is the historical Stage 1 acceptance marker. The adapter established the
current command boundary:

```
/cmd_vel_nav (m/s, rad/s)
  -> runner_drive_adapter
  -> /cmd_vel_auto (normalized forward command/brake, normalized steering)
  -> twist_mux
  -> /cmd_vel
  -> motor_node
```

The adapter remains the sole SI-to-normalized converter. `twist_mux` remains
the sole `/cmd_vel` publisher and `motor_node` remains the sole PWM owner.
Stale `/cmd_vel_nav` still produces silence, explicit zero produces active
brake, reverse remains rejected to zero, and infeasible steering is clamped.
The published motor-command range is forward-only `[0.0, 0.14]`; proportional
correction may reduce output to zero but cannot request reverse. The 0.14
ceiling is a safety authority bound, not a feedforward target.

Stage 1 used an encoder-triggered 0.380 breakaway kick. Stage 4 demonstrated
that it added no torque at the maximum calibrated command because the
steady-state feedforward was already 0.380. The kick and its parameters have
been removed rather than left as a second overlapping throttle modifier.

The current implementation, parameters, evidence, diagnostics, bench
procedure, and floor-test procedure are documented in
[stall_assist.md](stall_assist.md).

## Stage 2B characterized MD13S feedforward

The stale hobby-ESC lookup and provisional plant fit are no longer used.
Accepted forward targets use the characterized inverse directly in normalized
effort:

```
u_ff = 0.1188 * abs(effective_speed) + 0.0174
```

The adapter does not apply a 0.04 feedforward output floor. Instead, every
accepted nonzero speed is promoted to at least 0.25 m/s before feedforward and
P-error calculation. An explicit zero remains exactly zero and receives no
feedforward. A typed `feedforward_floor_violation` diagnostic reports a
computed nonzero-command feedforward below 0.04 without changing output; with
the configured 0.25 m/s floor it remains false and assertion indicates that
the command-floor path was bypassed.

Bag `rf2o_fix1_20260731_214809` confirms that wheel and RF2O velocity signs
agree for 99.47% of 1139 moving samples. The remaining speed chatter is a
feedback limit cycle: the MD13S plant gain is approximately 7.947 m/s per
command, so the former Kp 0.30 produced a proportional loop gain near 2.38.
The controller uses Kp 0.05 and Ki 0. Integral accumulation, breakaway preload,
preemption decay, and error-dependent stall gain switching are bypassed. Every
fresh-feedback output is exactly:

```
clamp(feedforward + 0.05 * speed_error, 0.0, 0.14)
```

Here `speed_error = abs(effective_speed) - abs(measured_speed)`. Fresh-feedback
floor-promoted commands receive the same P correction as other accepted
commands. Stale encoder feedback remains feedforward-only.
The legacy integral diagnostic fields are retained for bag compatibility and
truthfully remain zero/false. Operator driving is required to validate
the operational floor on hardware. Matti owns that floor-autonomy validation;
this software stage does not validate vehicle behavior on hardware.

### Stage 2B typed acceptance semantics

- `commanded_speed` is the raw longitudinal SI request, unchanged by the floor
  or ceiling.
- `effective_speed` is zero, or has magnitude in `[0.25, 0.60]`; feedforward
  and feedback use this value.
- `speed_error` is `abs(effective_speed) - abs(measured_speed)`.
- Comparing `commanded_speed` with `effective_speed` exposes floor or ceiling
  intervention in independently interpreted bags.
- `feedforward_floor_violation` is diagnostic evidence of a bypassed floor,
  never an output clamp.
- `final_throttle` cannot exceed the 0.14 safety authority bound, and Ki
  remains exactly zero pending hardware validation by Matti.

## Teleop preemption

The adapter subscribes to `/teleop/active_mode` (`std_msgs/String`, default
reliable/volatile depth-10 QoS). A fresh value other than `teleop_suppress`
confirms that the mux is discarding adapter output. The adapter still publishes
`/cmd_vel_auto`; `twist_mux` remains the sole arbiter.

Active mode is fresh for 0.20 seconds, four nominal periods of its 20 Hz
publisher. It remains diagnostic and does not change feedforward or
proportional feedback now that integral action is disabled.
`/drive_adapter/state_typed.mode` carries the drive mode plus active-mode
received/fresh/value and preemption flags. Its legacy integral fields report
disabled/zero and `integral_decay_active=false`.
