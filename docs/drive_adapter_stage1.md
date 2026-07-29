# Runner Phase 1 Stage 1 drive adapter

This is the historical Stage 1 acceptance marker. The adapter established the
current command boundary:

```
/cmd_vel_nav (m/s, rad/s)
  -> runner_drive_adapter
  -> /cmd_vel_auto (normalized throttle/brake, normalized steering)
  -> twist_mux
  -> /cmd_vel
  -> motor_node
```

The adapter remains the sole SI-to-normalized converter. `twist_mux` remains
the sole `/cmd_vel` publisher and `motor_node` remains the sole PWM owner.
Stale `/cmd_vel_nav` still produces silence, explicit zero still produces
full brake, reverse remains rejected, and `steering_infeasible` still produces
full brake with zero steering.

Stage 1 used an encoder-triggered 0.380 breakaway kick. Stage 4 demonstrated
that it added no torque at the maximum calibrated command because the
steady-state feedforward was already 0.380. The kick and its parameters have
been removed rather than left as a second overlapping throttle modifier.

The current implementation, parameters, evidence, diagnostics, bench
procedure, and floor-test procedure are documented in
[stall_assist.md](stall_assist.md).

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
