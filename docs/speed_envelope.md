# Longitudinal speed envelope

`runner_drive_adapter/config/speed_envelope.yaml` is the single launched
configuration origin for longitudinal parameters shared by `controller_server`
and `drive_adapter`. Stage 2B applies the characterized MD13S inverse, the
0.25 m/s adapter command floor, the confirmed RPP floors, and the 0.14 safety
authority bound through this origin. Floor-autonomy hardware validation
remains pending.

The adapter's bounded parallel-form integrator uses the encoder sample stamps
for `dt` and a symmetric `integrator_bound` in normalized effort. Ki is 0.01
in the committed origin. Successful live `proportional_gain` and
`integral_gain` writes are applied on the next adapter control cycle. A Kp write
does not reset integral state; a successful Ki write retains its existing
reset. No other adapter parameter is live-effective. The observer remains
read-only.

`integrator_freeze_reason` evaluates the existing conditions independently of
the configured Ki. If conditions coincide, its deterministic precedence from highest to
lowest is `NO_COMMAND`, `INVALID_COMMAND`, `ZERO_COMMAND`, `FEEDBACK_STALE`,
`WHEELSPIN`, `DIRECTION_UNAVAILABLE`, `DIRECTION_MISMATCH`,
`ARBITRATION_UNAVAILABLE`, `OUTPUT_NOT_SELECTED`, `INVALID_DT`,
`ANTI_WINDUP`, then `GAIN_DISABLED`. `INTEGRATOR_ACTIVE` is reported only when
none applies. Thus validity and ownership failures already encountered by the
control path outrank anti-windup, and `GAIN_DISABLED` is only the final
telemetry fallback when the runtime gain is zero. This ordering changes neither
the integrator update nor output arithmetic.

The committed longitudinal values are:

- `maximum_commanded_speed: 0.60` m/s;
- `feedforward_effort_per_speed: 0.1188` and
  `feedforward_effort_intercept: 0.0174` for
  `u_ff = 0.1188 * abs(effective_speed) + 0.0174`;
- `minimum_moving_speed: 0.25` m/s, with zero-or-at-least-0.25 acceptance;
- `proportional_gain: 0.05`, `integral_gain: 0.01`, and
  `integrator_bound: 0.005`;
- `output_max: 0.14`, a safety authority bound;
- RPP `min_approach_linear_velocity: 0.25` m/s and
  `regulated_linear_scaling_min_speed: 0.30` m/s.

`max_allowed_time_to_collision_up_to_carrot` remains 0.15 s.

The ordinary Nav2 and adapter parameter files retain unrelated behavior,
geometry, and diagnostics. Launch composes those files with the shared origin:

- `controller_server`: `nav2_params.yaml`, then `speed_envelope.yaml`;
- `drive_adapter`: `drive_adapter.yaml`, then `speed_envelope.yaml`.

No motion topic, topic type, or owner changes. `/cmd_vel_nav` remains SI,
`/cmd_vel_auto` and `/cmd_vel` remain normalized effort, and teleop continues to
bypass the adapter intentionally.

## Reconciliation topic

The application-tier `speed_envelope_observer` reads the committed YAML and
uses asynchronous parameter-service requests to observe `/controller_server`
and `/drive_adapter`. It never writes parameters, participates in lifecycle
transitions, or publishes/subscribes to a motion topic.

At 1 Hz it publishes `runner_interfaces/msg/SpeedEnvelopeStatus` on
`/speed_envelope/status`. Every `entries` element contains:

- the canonical origin key, owning node, and ROS parameter name;
- the typed origin value;
- the typed observed value when available;
- per-key availability, divergence, and a diagnostic detail.

`DIVERGENCE_UNKNOWN` means that key could not be observed on the latest attempt.
A missing service, missing parameter, service error, or 0.25 s request timeout
affects only that key. Other entries are still published normally. The first
snapshot after observer startup is expected to be unknown because no service
read has completed yet.

Reads are issued in bounded per-node bursts so the parameter-service request
queue is not overrun by the complete shared origin. Keys retain independent
availability and divergence state, and all 26 entries reconcile after startup
when both consumers are present.

The committed VS Code MCAP recorder uses `--all-topics`, so
`/speed_envelope/status` is included in its recording allowlist without a
control-path subscription or special QoS override.

The old ESC pulse calculations and messages in DualSense teleop are adjacent
cleanup only. They remain inert with respect to the published normalized effort
and are deliberately outside this consolidation.

## Five-parameter live tuning panel

Create one Foxglove **Parameters** panel named `Speed loop — live subset` and
show only these exact node/parameter pairs:

| Node | Parameter | Committed default |
|---|---|---:|
| `/controller_server` | `FollowPath.desired_linear_vel` | `0.45` |
| `/controller_server` | `FollowPath.min_approach_linear_velocity` | `0.25` |
| `/controller_server` | `FollowPath.regulated_linear_scaling_min_speed` | `0.30` |
| `/drive_adapter` | `proportional_gain` | `0.05` |
| `/drive_adapter` | `integral_gain` | `0.01` |

The three `FollowPath` values are protected by RPP's dynamic-parameter mutex.
The two adapter values use one validated config swap in its parameter callback.
All five are read back independently on `/speed_envelope/status`; an override
sets that entry to `DIVERGENCE_DIFFERENT`, and restoring the value returns it to
`DIVERGENCE_MATCH`. The panel must not include any other speed-envelope value,
because no other adapter parameter is authorized for live application.

## Active-route hardware verification

Start the normal route and recorder exactly as for Stage 2B. Do not restart a
node or perform a lifecycle transition. While the route is active, apply these
temporary conservative values one at a time, allowing at least one controller
cycle after each command and confirming the corresponding Parameters-panel and
`/speed_envelope/status` entry:

```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 0.35
ros2 param set /controller_server FollowPath.min_approach_linear_velocity 0.27
ros2 param set /controller_server FollowPath.regulated_linear_scaling_min_speed 0.27
ros2 param set /drive_adapter proportional_gain 0.03
ros2 param set /drive_adapter integral_gain 0.005
ros2 topic echo /speed_envelope/status --once
```

Before stopping the route or recorder, restore every committed default and wait
for the observer to report `DIVERGENCE_MATCH` for all five entries:

```bash
ros2 param set /controller_server FollowPath.desired_linear_vel 0.45
ros2 param set /controller_server FollowPath.min_approach_linear_velocity 0.25
ros2 param set /controller_server FollowPath.regulated_linear_scaling_min_speed 0.30
ros2 param set /drive_adapter proportional_gain 0.05
ros2 param set /drive_adapter integral_gain 0.01
ros2 topic echo /speed_envelope/status --once
```

The temporary speeds stay within the committed 0.25–0.60 m/s command envelope,
Kp and Ki are reduced, and Ki remains bounded by the unchanged 0.005 integrator bound,
and the unchanged 0.14 output authority remains the final ceiling. The hardware
run must demonstrate a behavioral response within one control cycle for each
write, recorded origin/live divergence, and reconciliation after restoration.
