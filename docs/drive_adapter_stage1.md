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
The published motor-command range is forward-only `[0.0, 0.12]`; provisional
proportional correction may reduce output to zero but cannot request reverse.

Stage 1 used an encoder-triggered 0.380 breakaway kick. Stage 4 demonstrated
that it added no torque at the maximum calibrated command because the
steady-state feedforward was already 0.380. The kick and its parameters have
been removed rather than left as a second overlapping throttle modifier.

The current implementation, parameters, evidence, diagnostics, bench
procedure, and floor-test procedure are documented in
[stall_assist.md](stall_assist.md).

## Provisional MD13S feedforward

The stale hobby-ESC lookup is no longer used. Forward targets use the inverse
of `v = 7.947 * cmd - 0.1436`:

```
cmd = (speed + 0.1436) / 7.947
```

This fit comes from `teleop_20260731_200955`, has R2 0.9968, and covers input
commands 0.05 through 0.25. It is a provisional compatibility fit pending
controlled MD13S characterization. In particular, the lower requested-speed
examples extrapolate below the fitted command range. Every published adapter
output is nevertheless clamped to `[0.0, 0.12]`, and an explicit zero target
still publishes zero.

Bag `rf2o_fix1_20260731_214809` confirms that wheel and RF2O velocity signs
agree for 99.47% of 1139 moving samples. The remaining speed chatter is a
feedback limit cycle: the MD13S plant gain is approximately 7.947 m/s per
command, so the former Kp 0.30 produced a proportional loop gain near 2.38.
The compatibility controller therefore uses provisional pre-characterization
tuning of Kp 0.05 and Ki 0. Integral accumulation, breakaway preload,
preemption decay, and error-dependent stall gain switching are bypassed. Every
fresh-feedback output is exactly:

```
clamp(feedforward + 0.05 * (commanded_speed - measured_speed), 0.0, 0.12)
```

Stale encoder feedback and promoted-floor commands remain feedforward-only.
The legacy integral diagnostic fields are retained for bag compatibility and
truthfully remain zero/false. Operator driving is required to validate
smoothness before controlled MD13S characterization.

For a later controlled tuning pass:

- Characterize feedforward-only tracking before changing Kp 0.05 or enabling
  any integral action. The inverse plant slope is about 0.126 command per m/s.
- Initially constrain the integrator contribution to approximately
  `[-0.03, +0.02]`; the +0.02 side fits within the roughly 0.026 command
  headroom at a 0.600 m/s target. Recalculate from characterized headroom.
- Use zero breakaway preload until MD13S onset is measured under load. If a
  preload is demonstrated necessary, introduce no more than 0.01 first and
  require repeatable stationary-transition evidence.
- Temporarily disable floor promotion by setting its minimum ratio to 1.0
  during characterization, then promote only to a repeatably sustainable
  measured floor. Do not infer that floor from the extrapolated fit.

The feedforward coefficients, output bounds, and floor-promotion settings are
unchanged by this provisional feedback retune.

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
