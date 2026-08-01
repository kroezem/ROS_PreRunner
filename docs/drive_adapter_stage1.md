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
PI correction may reduce output to zero but cannot request reverse.

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

No tuning changes accompany the inverse. For a later controlled tuning pass,
the recommendations are:

- Start with Kp 0.05-0.10 and Ki 0.01-0.03 after verifying feedforward-only
  tracking. The inverse plant slope is about 0.126 command per m/s; the current
  Kp 0.30 and switched Ki 0.06/0.30 are likely too aggressive under this cap.
- Initially constrain the integrator contribution to approximately
  `[-0.03, +0.02]`; the +0.02 side fits within the roughly 0.026 command
  headroom at a 0.600 m/s target. Recalculate from characterized headroom.
- Use zero breakaway preload until MD13S onset is measured under load. If a
  preload is demonstrated necessary, introduce no more than 0.01 first and
  require repeatable stationary-transition evidence.
- Temporarily disable floor promotion by setting its minimum ratio to 1.0
  during characterization, then promote only to a repeatably sustainable
  measured floor. Do not infer that floor from the extrapolated fit.

These are recommendations only; the configured PI gains, integrator bounds,
breakaway preload, and floor-promotion settings remain unchanged in this
compatibility change.

## Teleop preemption

The adapter subscribes to `/teleop/active_mode` (`std_msgs/String`, default
reliable/volatile depth-10 QoS). A fresh value other than `teleop_suppress`
confirms that the mux is discarding adapter output. The adapter still publishes
`/cmd_vel_auto`; `twist_mux` remains the sole arbiter.

Active mode is fresh for 0.20 seconds, four nominal periods of its 20 Hz
publisher. Absent or stale state means not preempted and preserves the previous
controller behavior. During confirmed preemption, integral accumulation is
disabled and existing integral decays toward zero by 0.0625 normalized throttle
contribution per second without crossing zero. This clears the configured
-0.25 bound in 4.0 seconds and +0.16 bound in 2.56 seconds.

Preemption decay has priority over the wheelspin accumulation guard. The
existing below-floor zero reset remains stronger than bounded decay. Neither
condition changes feedforward, proportional output, stall-gain selection,
breakaway preload policy, clamps, command processing, or publication.
`/drive_adapter/state_typed.mode` carries the drive mode plus active-mode
received/fresh/value, preemption, and decay flags; its existing
`integrator_enabled` and `integrator_state` fields report accumulation and
integral value.
