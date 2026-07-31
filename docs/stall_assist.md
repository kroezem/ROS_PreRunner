# Runner bounded stall assist

## Scope and Stage 4 evidence

Stage 4 proved the full forward-only autonomy chain on the physical vehicle,
but a thin rug and a room threshold can stop the car long enough for Nav2's
progress checker to abort. This change is confined to
`runner_drive_adapter`: it does not alter the calibrated steady-state table,
the speed cap, steering policy, reverse behavior, mux, motor node, or Nav2.

The local exact-name recording
`bags/autonomous_init_20260727_205851` spans
`2026-07-27 20:58:52.720899-07:00` through
`20:59:55.882251-07:00`, but contains zero `/cmd_vel_nav`, zero
`/cmd_vel_auto`, and zero paths. It is an initialization attempt, not the
powered Stage 4 stall.

The usable powered evidence is in
`bags/autonomous_init_20260727_211106`. At the confirmed stall:

- `/cmd_vel_nav.linear.x` remained `0.290 m/s`.
- Feedforward throttle, breakaway throttle, `/cmd_vel_auto.linear.x`, and the
  muxed `/cmd_vel.linear.x` were all `0.380`. The old kick therefore added
  exactly zero torque.
- The old kick ran from `21:11:28.029964` to `21:11:28.780084`, exiting by
  timeout.
- The last nonzero encoder sample was at `21:11:27.940814`, with forward
  command direction. After its 0.2 s stationary qualification the edge rate
  was zero. EKF velocity after the transient had mean `0.00049 m/s`, standard
  deviation `0.00599 m/s`, and range `-0.00963` to `0.03296 m/s`.
- During the preceding real motion, EKF velocity remained at or above
  `0.06 m/s` for `0.266 s` and peaked at `0.10187 m/s`. Encoder and EKF
  therefore agree that the later condition was a fully stalled wheel, not
  wheelspin.
- The EKF topic is `nav_msgs/msg/Odometry` on `/odometry/filtered`, with
  `header.frame_id=odom`, `child_frame_id=base_link`, and body-forward
  velocity in `twist.twist.linear.x`. It updated at median `15.002 Hz`; the
  largest bag gap was `0.150 s`. Median record-to-header-stamp lag was
  `0.0139 s`.
- No `steering_infeasible` message occurred in this recording. The shorter
  `21:03:59` recording did produce `steering_infeasible` and full brake, so it
  is not treated as clean terrain-stall evidence.

Exact abort ordering in the powered recording was:

1. `21:11:38.134152`: controller server, `Failed to make progress`.
2. `21:11:38.134517`: FollowPath handle abort.
3. `21:11:38.163266`: NavigateToPose handle abort.
4. `21:11:38.163771`: Nav2 goal failed.
5. `21:11:38.165723`: bridge result `ABORTED`, error code 105.
6. Last forward adapter command: `21:11:38.129834`.
7. Controller zero: `21:11:38.134269`; adapter brake publications ended at
   `21:11:38.380910`, followed by stale-command silence.
8. `21:11:38.530252`: motor watchdog timeout.

The bag also shows operator deadman transitions: the mux alternated autonomy
and teleop brake during the attempt. Consequently it proves the zero-torque
kick and the sensor separation, but does not label which physical obstacle
was under the car. Rug and threshold results must come from the targeted
post-bench bags.

## Signal architecture

EKF body-forward velocity is the primary vehicle-translation signal. The
stall noise is far below the `0.06 m/s` confirmation threshold, while genuine
motion crossed it continuously for more than the `0.20 s` qualification.
Confirmation also requires at least two EKF samples; one sample cannot change
state.

Encoder edge rate is the wheelspin cross-check. Forward encoder direction,
edge rate at or above the configured threshold, and EKF velocity below the
wheelspin vehicle-speed threshold must persist before wheelspin is declared.
Wheelspin returns immediately to feedforward and enters cooldown; it never
commands brake and never asks for more torque.

Both inputs use monotonic callback receive times. EKF and encoder data older
than `0.25 s`, or nonfinite values, cannot start assistance. Either signal
becoming stale during elevated throttle ends the event at feedforward and
starts cooldown.

## State machine

`IDLE` handles no command and explicit stops. `NORMAL` applies only calibrated
feedforward. `QUALIFYING` requires a fresh stable forward command, fresh
sensors, and both:

```
measured_speed < under_speed_ratio * commanded_speed
measured_speed < under_speed_absolute_ceiling
```

After continuous qualification, `RAMPING` starts at feedforward and applies:

```
min(feedforward + ramp_rate * elapsed, boost_throttle_ceiling)
```

Confirmed translation stops the ramp and enters `HOLDING`. Overspeed skips
the hold. `DECAYING` subtracts the configured rate until feedforward is
reached. Any completed or failed event enters `COOLDOWN`, where normal
feedforward remains available but re-arming is blocked. A re-stall during
decay cannot start another event.

The maximum event duration covers ramp, hold, and decay. Reaching the ceiling
without confirmed motion also ends the event. Command staleness preserves the
existing silence rule rather than republishing a stale brake or throttle.
Explicit zero, rejected reverse, invalid input, sensor staleness, and shutdown
all terminate elevated throttle immediately at zero command.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `stall_assist_enabled` | `true` | Enable bounded assist |
| `under_speed_ratio` | `0.40` | Relative under-speed limit |
| `under_speed_absolute_ceiling` | `0.10 m/s` | Absolute under-speed limit |
| `under_speed_qualification_sec` | `0.30 s` | Continuous qualification |
| `command_stability_tolerance` | `0.02 m/s` | Allowed command change |
| `ramp_rate_per_sec` | `0.10/s` | Normalized throttle ramp |
| `boost_throttle_ceiling` | `0.50` | Absolute throttle ceiling |
| `maximum_assist_duration_sec` | `1.50 s` | Ramp + hold + decay limit |
| `motion_confirm_speed` | `0.06 m/s` | EKF translation threshold |
| `motion_confirm_duration_sec` | `0.20 s` | Continuous confirmation |
| `motion_hold_duration_sec` | `0.30 s` | Acquired-throttle hold |
| `decay_rate_per_sec` | `0.15/s` | Normalized throttle decay |
| `overspeed_margin` | `0.02 m/s` | Immediate-decay margin |
| `wheelspin_edge_rate_threshold` | `1.0 edges/s` | Encoder movement limit |
| `wheelspin_vehicle_speed_threshold` | `0.02 m/s` | EKF low-motion limit |
| `wheelspin_qualification_sec` | `0.25 s` | Continuous wheelspin evidence |
| `cooldown_duration_sec` | `0.50 s` | Re-arm inhibition |
| `motion_signal_timeout_sec` | `0.25 s` | EKF receive freshness |
| `encoder_state_timeout_sec` | `0.25 s` | Encoder receive freshness |

All values are startup-validated. Ratios, durations, rates, finite values,
nonnegative thresholds, table bounds, and the relationship between maximum
feedforward, assist ceiling, and motor command limit are rejected with a
precise startup error when invalid. Values are never silently clamped.

## Diagnostics and progress checker

The existing `/drive_adapter/state` record remains and now includes assist
state, boost, event count, and last exit reason. Bag-friendly topics are:

- `/stall_assist/state` (`std_msgs/String`)
- `/stall_assist/applied_boost` (`std_msgs/Float32`)
- `/stall_assist/event_count` (`std_msgs/UInt32`)
- `/stall_assist/last_exit_reason` (`std_msgs/String`)

Each event logs one start, important transitions only, and one structured
summary with command, feedforward and peak throttle, total and per-state
durations, EKF and encoder start/peak values, signal architecture, and exit
reason.

Nav2 remains the sustained-pushing backstop. Its unchanged
`required_movement_radius` is `0.05 m` and
`movement_time_allowance` is `10.0 s`. Extend the allowance only if a targeted
test shows genuine forward obstacle progress that is cut off before crossing;
stationary pushing is not a reason to extend it.

## Bench procedure

Before powered validation, the operator must explicitly confirm the traction
battery state, physical restraint, all driven wheels clear of the ground,
fall/contact prevention, clear drivetrain area, and immediate stop access.
Then exercise one representative case for ramp, motion confirmation,
hold/decay, overspeed, wheelspin, maximum duration/cooldown, L1 brake
preemption, command loss, watchdog, and shutdown. Capture the diagnostics and
structured summaries. Do not run a parameter matrix.

## Targeted floor procedure

With Matti present and holding L1 as the autonomy deadman, record all topics
and use a conservative goal with clear space beyond the thin kitchen rug,
then the kitchen-to-dining threshold. Record command, feedforward, peak,
duration, exit, EKF velocity, encoder rate, wheelspin, crossing, fast decay,
progress abort, watchdog ordering, and steering feasibility. Stop once each
obstacle has enough evidence; do not repeatedly push against it.

## Limitations

This mechanism does not estimate or regulate steady-state speed, compensate
traction voltage, or correct the calibrated table. It is bounded scaffolding
for a possible future feedforward-plus-PI controller after separate
characterization. It must not be described as closed-loop speed control.
