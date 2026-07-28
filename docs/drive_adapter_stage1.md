# Runner Phase 1 Stage 1 drive adapter

## Scope and topic ownership

Stage 1 introduced `runner_drive_adapter/drive_adapter` with deliberate
isolation from the motor. Stage 2 now connects the normalized command sources
only through `twist_mux`; see `twist_mux_stage2.md`.

| Owner | Subscription | Publication | Semantics |
|---|---|---|---|
| `drive_adapter` | `/cmd_vel_nav` | `/cmd_vel_auto` | Physical Nav2 Twist in; normalized command out |
| `drive_adapter` | `/wheel/encoder_state` | `/drive_adapter/state` | Motion state in; compact diagnostics out |
| `runner_teleop` | `/joy` | `/cmd_vel_teleop` | Normalized human command |
| `twist_mux` | `/cmd_vel_teleop`, `/cmd_vel_auto` | `/cmd_vel` | Stage 2 command owner |
| `runner_motor` | `/cmd_vel` | PWM | Existing sole PWM owner |

The adapter-only launch file still does not start `motor_node`. In the Stage 2
bench launch, `/cmd_vel_auto` reaches `/cmd_vel` only through `twist_mux`;
there is no direct relay, remap, or bridge.

`/cmd_vel_nav` is `geometry_msgs/msg/Twist`:

- `linear.x`: forward speed in metres per second
- `angular.z`: yaw rate in radians per second

`/cmd_vel_auto` is also `geometry_msgs/msg/Twist`, but its fields deliberately
have different semantics:

- `linear.x`: normalized throttle in `[0, 1]`, or `-1` for full brake
- `angular.z`: normalized steering in `[-1, 1]`

During Stage 1 the output was not connected to `motor_node`. Stage 2 preserves
the same adapter semantics and routes it through the mux.

## Parameters

The measured defaults are in `config/drive_adapter.yaml`.

| Parameter | Default | Units / meaning |
|---|---:|---|
| `wheelbase` | 0.178 | m |
| `max_steering_angle` | 0.3054 | rad |
| `steering_min_speed` | 0.05 | m/s |
| `throttle_breakpoints` | `[0.340, 0.350, 0.360, 0.380]` | normalized |
| `speed_breakpoints` | `[0.126, 0.188, 0.233, 0.290]` | m/s |
| `minimum_moving_speed` | 0.126 | m/s |
| `floor_promotion_min_ratio` | 0.50 | dimensionless |
| `breakaway_throttle` | 0.380 | normalized |
| `breakaway_timeout` | 0.75 | s |
| `motion_confirm_edge_rate` | 1.0 | edges/s |
| `cmd_vel_nav_timeout` | 0.25 | s |
| `encoder_state_timeout` | 0.25 | s |
| `publication_rate` | 20.0 | Hz |

The table must have equal lengths with at least two points. Both sequences
must be finite and strictly increasing; throttle must be within `[0, 1]` and
speed must be positive. The moving floor must fall inside the speed table.
Invalid configuration prevents node startup.

The measured `0.370 -> 0.209 m/s` point is excluded because it was a single
noisy sample with run standard deviation `0.094 m/s`. The `0.360` point was
measured twice at `0.229` and `0.237 m/s`, so retaining the noisy point would
make the otherwise monotonic table worse.

## Feedforward and low-speed policy

Throttle is deterministic piecewise-linear interpolation with speed as the
independent variable. Values above the measured `0.290 m/s` maximum clamp to
that speed and output `0.380`; the adapter never extrapolates. It logs this
clamp at most once per five seconds.

The promotion boundary is:

```
0.126 * 0.50 = 0.063 m/s
```

Boundary behavior is:

| Requested speed | Result |
|---:|---|
| `v < 0` | full brake, zero steering, throttled warning |
| `v = 0` | full brake, zero steering |
| `0 < v < 0.05` | full brake, zero steering; no steering division |
| `0.05 <= v < 0.063` | full brake, zero steering |
| `0.063 <= v < 0.126` | promote effective speed to `0.126 m/s` |
| `v >= 0.126` | normal lookup, with the measured-table upper clamp |

The vehicle is forward-only in Phase 1. Negative speed never maps to reverse
or proportional braking. NaN or infinity in either relevant Twist field
produces full brake and zero steering for that fresh-command cycle, plus a
throttled error; non-finite values are never propagated.

This is open-loop feedforward. There is no PID, integral correction, encoder
speed feedback, voltage compensation, or adaptive calibration. `/battery` is
the Raspberry Pi UPS gauge, not traction-pack voltage. Race-mode negative
motor commands are proportional brake, and physical reverse remains disabled.

## Steering feasibility

For a valid positive-speed command:

```
requested_curvature = omega / v
maximum_curvature = tan(max_steering_angle) / wheelbase
                  = tan(0.3054) / 0.178
                  = 1.771140041436 m^-1
```

A request is feasible when:

```
abs(requested_curvature) <= maximum_curvature
```

The implementation adds only a numerical boundary tolerance:

```
max(1e-12 m^-1, maximum_curvature * 1e-12)
```

For feasible commands:

```
delta = atan(wheelbase * requested_curvature)
normalized_steering = delta / max_steering_angle
```

Positive yaw maps to positive normalized steering. At the positive or negative
curvature limit, normalized steering is respectively `+1` or `-1`, subject to
the documented floating-point tolerance.

An over-limit command is rejected with full brake and zero steering. Its
diagnostic reason is `steering_infeasible`, and the throttled warning contains
requested speed, requested yaw, requested curvature, and maximum curvature.
The adapter does not clamp and drive, increase speed, or invent a substitute
path.

The earlier proposal to reduce speed while preserving requested yaw was
rejected because:

```
required_curvature = omega / v
```

For fixed `omega`, decreasing `v` increases required curvature and makes the
infeasibility worse. Stage 1 therefore rejects the command instead of silently
changing the requested path. Stage 3 controller configuration must keep normal
commands within the measured `0.565 m` turning-radius boundary; rejection is a
safety boundary, not routine control.

## Breakaway kick and encoder evidence

`EncoderState.edge_rate` is unsigned physical GPIO edges per second. The
encoder converts it to speed as:

```
speed = edge_rate * 0.010282 m/edge
```

Measured speed standard deviation at `0.313 m/s` was `0.0034 m/s`, equivalent
to:

```
0.0034 / 0.010282 = 0.331 edges/s
```

The `1.0 edges/s` motion threshold is approximately three times that measured
standard deviation, converts to `0.010282 m/s`, and remains far below the
lowest moving speed:

```
0.126 / 0.010282 = 12.254 edges/s
```

`EncoderState.stationary` remains the primary state: it is derived from
physical pulse absence, becomes false on an accepted edge, and becomes true
after the configured encoder's `0.2 s` pulse-free interval. A `true -> false`
transition ends a kick immediately even before the interval estimator has two
edges. The edge-rate threshold provides secondary confirmation if the
transition itself was not observed.

When a valid moving command, fresh stationary encoder state, and an armed kick
coincide, output is at least `0.380`. The kick ends immediately on confirmed
motion or at `0.75 s`. Timeout disarms it so continuous stationary input cannot
restart it. Confirmed motion re-arms a later stall; any explicit command that
results in braking re-arms the next start.

Encoder input is stale after `0.25 s`, five missed 20 Hz publications. A stale
or unavailable encoder suppresses or ends the additional kick while normal
bounded feedforward remains available. This is conservative: stale motion
state cannot cause indefinite or repeated breakaway throttle.

## Timing, silence, and diagnostics

The adapter publishes on a 20 Hz timer, matching teleop and encoder
publication. This gives deterministic kick timing and four nominal output
opportunities inside the motor's future 0.2 s watchdog window.

`cmd_vel_nav_timeout` is `0.25 s`. No controller server exists in Stage 1, so
there is no live repository controller rate to measure. The selected timeout
is the recommended starting value and allows five missed cycles at the future
20 Hz command rate while still bounding persistence. It must be revisited
against the configured and measured controller rate in Stage 3.

The adapter publishes no `/cmd_vel_auto` before its first navigation command or
after the latest command is older than `0.25 s`. Staleness produces silence,
not brake and not a zero Twist, so a future downstream chain can expire and
the motor watchdog can act. A fresh explicit zero is different: it continuously
publishes full brake while fresh.

`/drive_adapter/state` is a `std_msgs/msg/String` record with stable
semicolon-separated fields:

```
mode=<mode>;reason=<reason>;kick_active=<true|false>;
effective_speed=<m/s>;feedforward_throttle=<normalized>;
final_throttle=<normalized>;normalized_steering=<normalized>
```

Stable reasons distinguish `no_command`, `stale_command`, `explicit_stop`,
`negative_speed`, `nonfinite_input`, `steering_infeasible`, floor braking,
table clamping, normal feedforward, and breakaway operation. Kick start/end
transitions are logged once, including `motion_confirmed`, `timeout`,
`stop_command`, `stale_command`, or `encoder_stale`.

## Known physical limitation

The steering linkage and servo saver have hysteresis around center: release
from left and right can settle on different sides. This is not treated as trim.
Stage 1 adds no steering bias or tuning compensation.

Stage 1 powered bench acceptance confirmed that normalized `+1` steers
physically left and normalized `-1` steers physically right, with both
directions reaching the established full travel. Release from either direction
settled slightly toward the side that had just been commanded, matching the
known hysteresis. Full-right produced no servo buzz; full-left produced slight
buzzing at the endpoint. Stage 1 does not compensate for that observation or
change the ratified steering range. The left-end buzz remains a possible
mechanical-load issue to inspect before autonomous floor operation.
