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
