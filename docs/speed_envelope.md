# Longitudinal speed envelope

`runner_drive_adapter/config/speed_envelope.yaml` is the single launched
configuration origin for longitudinal parameters shared by `controller_server`
and `drive_adapter`. The consolidation intentionally preserves every prior
numeric and boolean value; it does not validate the inherited ESC-era or
provisional MD13S settings against hardware.

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

The committed VS Code MCAP recorder uses `--all-topics`, so
`/speed_envelope/status` is included in its recording allowlist without a
control-path subscription or special QoS override.

The old ESC pulse calculations and messages in DualSense teleop are adjacent
cleanup only. They remain inert with respect to the published normalized effort
and are deliberately outside this consolidation.
