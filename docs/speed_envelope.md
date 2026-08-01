# Longitudinal speed envelope

`runner_drive_adapter/config/speed_envelope.yaml` is the single launched
configuration origin for longitudinal parameters shared by `controller_server`
and `drive_adapter`. Stage 2B applies the characterized MD13S inverse, the
0.25 m/s adapter command floor, the confirmed RPP floors, and the 0.14 safety
authority bound through this origin. Hardware validation remains pending.

The adapter's bounded parallel-form integrator uses the encoder sample stamps
for `dt` and a symmetric `integrator_bound` in normalized effort. Ki remains
zero in the committed origin. Of the adapter parameters, only a successful
live `integral_gain` write is currently applied; it resets integral state. The
observer remains read-only.

The committed longitudinal values are:

- `maximum_commanded_speed: 0.60` m/s;
- `feedforward_effort_per_speed: 0.1188` and
  `feedforward_effort_intercept: 0.0174` for
  `u_ff = 0.1188 * abs(effective_speed) + 0.0174`;
- `minimum_moving_speed: 0.25` m/s, with zero-or-at-least-0.25 acceptance;
- `proportional_gain: 0.05`, `integral_gain: 0.0`, and
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
