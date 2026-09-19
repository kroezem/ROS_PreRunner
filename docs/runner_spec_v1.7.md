# Runner — Architecture & Current-State Specification v1.7

**Current-state specification · updated 20 September 2026**

Baseline: `/home/matti/runner_ws`, `main` at
`7e15808` ("Apply measured PWM endpoints for the new steering servo"),
inspected from tracked source, configuration and tests, plus a read-only
inspection of the working tree and the Pi's live runtime files on
2026-09-19/20. Previous specification: `docs/runner_spec_v1.6.md` (frozen at
`058e56a`, source baseline `02b00c5`), which is kept intact as historical
documentation. This document reconciles v1.6 against the 34 commits
`058e56a..7e15808` and against the working-tree state at authoring time.

v1.7 is a **coherent current-state document, not a patch list**. Sections
that v1.6 carried forward unchanged from v1.5 (process tiers, runtime mode
lifecycle, motion authority, Initial Pose, localization/TF, networking) are
restated compactly here and remain fully specified by v1.5/v1.6 where more
detail is needed. Sections that changed materially — navigation and BT,
committed-path execution, the D2 speed profile, RPP/cusp ownership,
costmaps, map bundles, semantics, Paddock tuning/Speed Profiles,
observability, and steering — are rewritten.

**Reading convention.** Every substantive claim carries one of these labels,
stated in words where it matters:

1. **Ratified / current architecture** — a decision Matti ratified that still
   governs the design (numbered D-decisions, §17).
2. **Source/config-confirmed** — true of tracked HEAD source, config and
   tests. This says nothing about physical quality.
3. **Physical measurement** — measured on the vehicle, with the dataset named
   and its scope stated. Measurements are never generalized beyond it.
4. **Provisional / WIP** — deployed or observed, but under active tuning,
   unmeasured, or explicitly not ratified.
5. **Historical / superseded** — kept because it explains the current design.

Where the audit found a claim only in older documents, this spec does not
promote it. **Tests were not run in the authoring pass** (the Pi was live in
AUTONOMY), so "test-covered" below means a test exists in the tree, not that
it was executed for this document. **No successful smoke check, UI
interaction or unit test in this document is physical validation.**
**Deploy coherence is not assumed**: this document describes the repository,
not necessarily what each systemd unit is executing (§15).

**Decisions.** D-89 and D-90 remain ratified as in v1.6. **This v1.7 pass
ratifies three decisions, at Matti's direction on 2026-09-20: D-91
(committed-path retention), D-93 (identity-bound path speed profile
contract) and D-94 (tracker ownership of execution progress and cusp
transitions, narrowly and tracker-implementation-agnostic).** They reuse the
numbers of the v1.6 planning labels but are ratified in the scoped form
stated in §17.1, not as those labels were originally sketched. D-92 and
D-95 remain unratified labels; nothing else is ratified (steering, D2
tuning, `cost_penalty`, default profile, route simulator, recording set,
chair-leg fix and unstick recovery all remain open).

## Uncommitted / WIP state at authoring time

The following exist in the working tree but are **not** in `main` and are
**not established architecture** anywhere in this document:

- **Route simulator (offline planning tool)**: `src/runner_route_sim/`,
  `src/runner_interfaces/srv/SimulateRoute.srv`,
  `src/runner_paddock/runner_paddock/route_sim_runtime.py`, plus uncommitted
  edits to `gateway.py`, `ros_state_node.py`, `app.js`, `index.html`,
  `style.css`, and `src/runner_interfaces/CMakeLists.txt` (§9.6).
- `maps/studio/semantics.png` (untracked) and a modified
  `maps/studio/manifest.json` that lists it.
- `.vscode/tasks.json` edits adding Foxglove start/stop tasks.
- Fourteen `docs/nav2_*_2026091[3-7].md` investigation documents (untracked).
  They are referenced here as evidence trails, not as committed record.

## 1. Executive architecture summary

Paddock (`runner_paddock`, served by `runner-paddock-web.service` on
`0.0.0.0:8000`) is the primary operator interface: runtime selection, manual
driving, supervised autonomous missions, STOP, mapping sessions, map bundle
lifecycle, Speed Profile tuning, planner tuning, obstacle-layer control,
MCAP recording, initial-pose seeding, and a semantic-layer editor. The
browser holds at most one control lease; a persistent Pi-side command
authority validates every intent against continuously refreshed interlocks.
Neither the browser nor the authority can reach the actuator directly.

Runtime and motion authority remain independent dimensions:

- **Runtime** (`ModeState.mode`): `IDLE / MAPPING / AUTONOMY`, with lifecycle
  `STABLE / TRANSITIONING / FAULT`, a monotonic `runtime_epoch`, a
  `mapping_session_id`, and continuously refreshed `ready`/`readiness_reason`.
- **Motion authority** (`CommandAuthorityState.authority`): `NONE / DUALSENSE /
  PADDOCK_MANUAL / PADDOCK_AUTONOMY`, derived from lease, mode, DualSense
  presence, RUN and goal state.

One `twist_mux` remains the sole `/cmd_vel` writer with priorities STOP
(255/lock 200) > DualSense (100) > Paddock manual (75) > Paddock autonomy
(50).

The autonomy execution chain is now, as source-confirmed:

```
Smac Hybrid-A* geometry (REEDS_SHEPP, one planner, no smoothing)
  → commitment (BT: candidate validated, committed, retained while valid)
  → D2 path speed profile (per-point ceiling attached to the committed path)
  → RPP tracker (identity-matched profile, cusp/segment ownership)
  → drive_adapter (feedforward + PI, output authority)
  → motor_node (reversal gate, watchdog, PWM)
```

What changed since v1.6, in one paragraph: the navigation BT was rebuilt
around committed-path semantics and a bounded recovery ladder (Stages A1, B,
C); forward-first certified planning was attempted and **reverted**; a D2
committed-path speed profile with an identity-matched handoff to a vendored
RPP now owns hazard-based speed; RPP owns segment/cusp progress with a
stationary-encoder advance rule; the four fixed presets were replaced by
named Speed Profiles plus separately-owned persistent override and planner
settings; maps are stored one directory per map with an optional semantic
class raster (storage/UI only); and the steering servo was replaced, which
changes only the final PWM mapping in source while **every steering-geometry
number derived from the old servo is now unmeasured or superseded by new
measurements (§2.3)**.

The main unresolved items are listed in §19; the steering actuator mapping,
D2 tuning, the whole-route collapse to minimum speed, chair-leg forgetting,
the reverse-start deadlock, and the override serialization bug all remain
visibly **open**.

## 2. Platform, geometry, steering and hardware boundaries

### 2.1 Platform

LaTrax Prerunner research platform: Raspberry Pi 5, Ubuntu 24.04, ROS 2
Jazzy, LD19 lidar, BNO085 IMU, hall-effect wheel encoder, Cytron MD13S motor
driver, Ackermann steering. Phase 1 (indoor/outdoor navigation) remains the
platform focus. The Pi's configured core clock is 2.8 GHz
(`arm_freq=2800`; live governor `ondemand`, so an instantaneous reading can
be lower).

### 2.2 Geometry that is source-configured

| Quantity | Value | Where | Status |
|---|---|---|---|
| Footprint front / rear / half-width | 0.230 / 0.060 / 0.0825 m | costmap footprints, D2 config, sim config | source-confirmed; not re-measured in this pass |
| Circumscribed radius | ≈0.2444 m | derived from footprint | derived |
| `wheelbase` | 0.178 m | `drive_adapter.yaml` | **physically unverified** (§2.3) |
| `max_steering_angle` | 0.3614 rad | `drive_adapter.yaml` | **old-servo value, not a measured current physical angle** (§2.3) |
| Adapter curvature clamp | `tan(0.3614)/0.178 ≈ 2.1236` 1/m | `drive_adapter.py` | derived from the two above; **not a measured curvature** |
| Smac `minimum_turning_radius` | 0.60 m | `nav2_params.yaml` | planning parameter; see §2.3 for its relation to the measurements |
| RPP `regulated_linear_scaling_min_radius` | 0.75 m | launched speed-envelope config + Paddock | speed-regulation threshold, not a feasibility limit |

v1.6 stated 0.3614 rad, 2.1236 1/m and a "physical minimum turning radius
0.4709 m" as confirmed physical geometry. **Those three figures are not
carried forward as current measured geometry.** They were characterized on
the previous servo/linkage, and 0.4709 m is `1/2.1236`, an arithmetic
consequence of the two configured numbers, never an independent measurement.
The test `src/runner_bringup/test/test_nav2_stage3.py` still pins
`physical_curvature = 2.1236`; it therefore encodes the old assumption and
should be read as a configuration-consistency check, not as physical truth.

### 2.3 Steering: hardware change and characterization (WORK IN PROGRESS)

**Hardware (physical).** A Savox steering servo replaced the previous one.
It is visibly much faster/snappier. It drives the mechanism in the **opposite
direction** for the same pulse width. It **buzzes under linkage load through
much of its range**; this is recorded as a hardware characteristic for now,
not a software issue and not investigated further.

**Source mapping (source-confirmed, `motor_node.py`).**
`steer_us = STEER_CTR + STEER_SIGN * clamp(angular.z, -1, 1) * STEER_US`
with `STEER_CTR = 1500`, `STEER_US = 450`, `STEER_SIGN = -1`
(`33117c3`, `7e15808`). The sign flip is applied at the single final
pulse-width conversion, so the established convention (`/cmd_vel.angular.z`
positive = physical left, ratified in v0.9) is preserved for every producer
(teleop, Nav2 via drive_adapter) without touching their semantics or the
1500 µs center. The 450 µs half-range is the usable range Matti found by
sweeping the servo while watching `duty_cycle`: no perceptible additional
motion past **1050/1950 µs** (a physical observation of the endpoints, not a
measurement of steering angle).

**What the drive adapter still assumes (source-confirmed).** A symmetric,
linear mapping:
`requested curvature κ → δ = atan(L·κ) → normalized = clamp(δ / max_steering_angle, −1, 1)`,
with `L = 0.178` and `max_steering_angle = 0.3614`. Nothing in source was
changed for the new servo beyond `STEER_SIGN`/`STEER_US`.

**Measured vehicle behavior — `turn_radius_0.mcap` (physical measurement,
values as supplied by Matti; this dataset only).**

| Full-lock command | Curvature | Rear-axle turning radius |
|---|---:|---:|
| Full left | ≈1.68 1/m | ≈0.595 m |
| Full right | ≈2.32 1/m | ≈0.431 m |

- The valid speed regime is **below ≈1.1 m/s**. The ≈38% left/right
  curvature asymmetry is real and stable in this dataset.
- Above ≈1.1 m/s the achieved curvature begins to roll off. A lateral
  acceleration of roughly 3–4 m/s² was observed there; **this is recorded as
  an observation and is not a ratified limit and must not be used as one**
  (one dataset is not enough; D2's `max_lateral_acceleration` is a separate,
  tunable policy parameter).

**Measured vehicle response — `turn_response_0.mcap` (physical measurement,
values as supplied by Matti).** Command-to-yaw fitted dead time ≈105–110 ms;
additional lag/rate-limit behavior ≈40 ms; zero-crossing latency median
≈171 ms; no meaningful steering oscillation or limit cycle. **This is
command-to-yaw *vehicle* response, not servo shaft transit time.** It
includes the command path, actuator, mechanism, tires and yaw measurement.

**Relationship to existing numbers (interpretation, labelled as such).**

- Smac `minimum_turning_radius = 0.60 m` essentially matches the measured
  **limiting (left) side**, 0.595 m. It therefore remains the conservative
  *symmetric* planning radius: it is feasible on both sides of the measured
  data and does not exploit the tighter right side. This is a statement
  about planning conservatism against one dataset, not a new steering model.
- The adapter's assumed clamp (2.1236 1/m) lies between the two measured
  full-lock curvatures. A consequence of the arithmetic — not a
  calibration — is that the adapter's notion of "full lock" curvature does
  not correspond to the measured full-left curvature (1.68) or the measured
  full-right curvature (2.32). What the adapter's *partial* commands
  achieve is **unmeasured**.

**Explicitly NOT established, and not to be inferred from the above.**

- The current physical steering angle at full lock (`max_steering_angle`),
  the wheelbase, and the true minimum radius as a geometric property.
- **The shape of κ(command) at partial steering.** We do not know whether
  the full-lock asymmetry is approximately a left/right gain difference, a
  nonlinear endpoint effect, linkage geometry, or something else. No
  calibration, gain split, or new steering model is introduced by this spec.
- Any dependence of curvature on speed beyond the coarse ≈1.1 m/s roll-off
  observation.
- Servo shaft transit time.

**Planned characterization (pending, not yet performed).** Matti will record
a partial-steering run at roughly 0.7–0.9 m/s at approximately
0, ±0.25, ±0.50, ±0.75, ±1.0 normalized steering so that κ(command) can be
characterized. Until that data exists, the actuator mapping is **WIP /
physical-validation-pending**, and any value derived from
`max_steering_angle`, `wheelbase` or the 2.1236 clamp is provisional.

**Physical observation on autonomy.** Initial autonomy testing with the
faster servo did not show an obvious major tracking improvement. This is an
observation, not proof of cause; it is consistent with the tracker/
execution-contract issues in §11–§12 dominating, but that is not tested.

**Historical baseline.** The old servo's steering-response measurements
remain historical baseline only, not current performance.

### 2.4 Resource and boundary ownership

| Resource / boundary | Current owner and rule |
|---|---|
| Motor effort PWM (GPIO12, 20 kHz), steering PWM (GPIO13, 50 Hz), direction GPIO23 | `runner-motor.service`, sole continuous owner; sysfs PWM, never unexported |
| Encoder GPIO22 | `runner-encoder.service`; `/wheel/encoder_state` feeds the reversal gate and every consumer's stationarity/direction evidence |
| Motor watchdog | 200 ms `/cmd_vel` staleness → zero duty (active brake), 50 ms check |
| Direction/reversal gate | motor-local; negative demand means reverse, never brake; requires a fresh post-request stationary encoder sample before flipping DIR |
| LD19 UART, BNO085 UART/reset | sensor-tier processes |
| `odom → base_link` | `ekf_node` only (RF2O is an EKF input) |
| `map → odom` | exactly one `slam_toolbox`: mapping instance in MAPPING, localization instance in AUTONOMY |
| `/map` | MAPPING: `slam_toolbox`; AUTONOMY: `map_server` (per-topic owner check in the mode supervisor) |
| `/cmd_vel` | `twist_mux`, sole writer |

TF has single ownership **per edge**. Retained hard safety limitations:
the planar LD19 scan cannot see descending edges or low obstacles; STOP is a
software inhibit, not proof of stationarity or power isolation; the motor
SIGKILL/PWM-peripheral hazard and deferred heartbeat-gated-FET decision
(v1.2, D-82 subject) remain open.

## 3. Process tiers and ownership model

Unchanged from v1.5/v1.6 at the tier level and unchanged in `services/`:
hardware tier (`runner-pwm-setup`, `runner-motor`, `runner-encoder`, IMU,
lidar, battery, telemetry), persistent local-control tier
(`runner-local-control`), persistent STOP tier (`runner-stop-enforcer`),
persistent operator tier (`runner-command-authority`,
`runner-drive-adapter`, `runner-mode-supervisor`, `runner-map-executor`,
`runner-recording-executor`, `runner-paddock-web`), and application tier
(`runner-mode-mapping`, `runner-mode-autonomy`, `linked` units started by
the supervisor). `keyboard_bridge` remains removed
(`runner_interfaces/msg/KeyboardState.msg` is a leftover message definition
whose consumer status was not audited).

`runner-foxglove.service` is tracked in `services/` but **not installed** on
the Pi at authoring time; uncommitted `.vscode` tasks start/stop it on
demand. Foxglove is therefore an on-demand operator tool, not a persistent
tier member. (No commit since v1.6 changed the unit; treat any claim about a
"Foxglove service change" as unverified.)

**Simulation is not a physical runtime.** If the route simulator is
committed later, it is a UI/workspace *tool* that launches a private
`/sim`-namespaced subprocess (§9.6); it is not a runtime mode, is not part of
`ModeState`, and does not own any physical resource.

## 4. Paddock operator surfaces and control roles

**Lease model** and roles are unchanged from v1.6 §4/§6: at most one
controller lease; observers; `acquire / release / heartbeat / run /
stop / clear_stop / takeover / manual`; explicit one-click **takeover** by
an observer (no owner-facing warning — an open UX/safety item, §19); STOP
assertion, STOP release and takeover are all single-click actions (the
hold-to-confirm helper was built and removed in v1.6's round and does not
exist in the tree). Per-map viewport memory in `localStorage` is retained
(covered by `test_map_viewport_storage.js`). A mapping **vehicle-follow
camera** (`mapping_camera.js`, `24d63a9`) follows the robot while mapping.
Telemetry pill warnings (`telemetry_warning.js`) and a corrected battery
warning threshold are present (`91b6a29`, `1d80e4b`).

**Gateway action set (tracked HEAD):** `acquire, takeover, release,
heartbeat, run, stop, clear_stop, manual, clear_obstacles,
set_obstacle_processing, set_autonomy_tuning, save_autonomy_tuning,
save_speed_profile, delete_speed_profile, set_config, start_recording,
stop_recording, delete_recording, select_mode, new_map, save_map,
select_map, delete_map, save_semantics, select_goal, set_initial_pose`.
Uncommitted working tree adds `simulate_route` (§9.6). Non-gateway
WebSocket messages: `visualization_demand` (demand-driven large frames).
HTTP reads added: `/maps/{name}/semantics.png`, `/speed_profiles`,
`/speed_profiles/{name}`.

**Console layout.** Primary tabs CONTROL and CONFIGURE. CONFIGURE tabs:
RUNTIME, MAPPING, AUTONOMY, SPEED PROFILE, DISPLAY (including the semantic
paint editor), RECORDING, SYSTEM. Detailed tuning surfaces are §9.

## 5. Runtime modes and lifecycle

Unchanged from v1.5/v1.6 §5: `ModeState` with `IDLE / MAPPING / AUTONOMY`,
`STABLE / TRANSITIONING / FAULT`, epoch, session id, capability-specific
readiness, transition/`reconcile()` and NEW MAP quiescence semantics.
`mode_runtime.py` changed since v1.6 only in that its map-bundle validation
now delegates to the single `map_session.validate_bundle` (§8). Mode names
are not changed. Any future "contextual workspace" UI grouping (for example
SIMULATE / MAPPING / AUTONOMY) is **future UI work only** and is not
implemented.

## 6. Motion authority, STOP, lease, RUN and takeover

Unchanged from v1.6 §6: authority is derived from lease, mode, DualSense
presence, RUN and goal state; STOP is a latched, durably persisted global
inhibit with explicit clear semantics; the motor watchdog and reversal gate
are motor-local. No commit in this round touches `command_supervisor.py`,
`stop_enforcer.py` or the mux configuration.

## 7. Command/data-flow and topic/interface ownership

### 7.1 Core operator/command flow (unchanged from v1.6 §7)

`/paddock/control_event`, `/paddock/control_lease`,
`/paddock/command_authority_state` (0.2 s heartbeat), `/paddock/mode_request`,
`/paddock/mode_state`, `/paddock/map_request` → `/paddock/map_state`
(1.0 s, content-deduplicated), `/paddock/navigation_request` →
`/paddock/navigation_state` (deduplicated with a 0.5 s forced heartbeat),
`/paddock/recording_request` → `/paddock/recording_state`,
`/paddock/config_request` → `/paddock/config_state`,
`/paddock/stop_state`, `/paddock/internal/stop_request`,
`/paddock/manual_demand`, `/cmd_vel_nav` (Nav2 → drive_adapter),
`/cmd_vel_auto_raw`, `/cmd_vel_paddock_manual_raw`, `/cmd_vel_auto` (p50),
`/cmd_vel_paddock` (p75), `/cmd_vel_teleop` (p100), `/cmd_vel_stop` (p255),
`/paddock/stop_lock`, `/cmd_vel` (mux → motor), `/teleop/control_state`,
`/wheel/encoder_state`, `/drive_adapter/state`,
`/drive_adapter/state_typed`, `/initialpose`. Each has one writer.

### 7.2 Navigation execution interfaces (new since v1.6, source-confirmed)

| Interface | Type | Writer → consumer | Notes |
|---|---|---|---|
| `/navigation/path_speed_profile` | `PathSpeedProfile` | `GeneratePathSpeedProfile` (bt_navigator) → observers/recording | transient-local record of the handoff |
| `/controller_server/FollowPath/set_path_speed_profile` | `SetPathSpeedProfile.srv` | BT node → RPP | synchronous handoff **before** the FollowPath goal; rejection or timeout makes the BT node FAIL, so the path is not dispatched |
| `/navigation/path_execution_state` | `PathExecutionState` | RPP → commitment nodes, observers | read-only: committed stamp, `path_hash`, `pose_count`, `current_segment`, `global_path_index`, `global_path_offset_m`, `execution_state` (`TRACKING / WAITING_FOR_STATIONARY / COMPLETE`) |
| `/navigation/path_commitment_state` | `String` | `ReportPathCommitment` → observers | `event=… reason=…` (`path_committed`, `candidate_rejected`) |
| `/lookahead_collision_arc`, `/motor/direction` | — | recorded by `navigation_debug` | present in the recording allowlist |
| `/global_costmap/costmap_raw` | `OccupancyGrid`/costmap | `planner_server` → `GeneratePathSpeedProfile` (`CostmapSubscriber`) | **sampled by D2; not in the `navigation_debug` recording set** (§14) |

`runner_interfaces` also carries `AutonomyTuningPolicy.msg` (shared constant
carrier, §12). `SimulateRoute.srv` is **uncommitted** (§9.6).

## 8. Mapping sessions and map bundles

### 8.1 Bundle layout (source-confirmed, `8facad6`)

```
maps/<map_id>/
  map.yaml            (references occupancy.pgm relatively)
  occupancy.pgm
  posegraph.posegraph
  posegraph.data      (pair shares the stem "posegraph": slam_toolbox
                       SerializePoseGraph behavior)
  semantics.png       (optional)
  manifest.json       (keys are relative filenames; SHA-256 digests;
                       revision hash; session id / created preserved)
```

The **directory name is the map identity** (a safe basename, ≤64 chars).
Saves stage into `.staging` and publish with one atomic rename; deletes move
into a tombstone with rollback. `map_session.validate_bundle` is the single
authoritative validator; `mode_runtime.py`, `nav2.launch.py` and
`localize.launch.py` delegate to it (previously three separate `image:`-line
scanners). Phase machine `NONE→STARTING→READY→SAVING→SAVED/FAILED`, NEW MAP
quiescence, SELECT MAP's live-swap rejection and catalog re-validation are
unchanged.

**Legacy migration.** `scripts/migrate_maps.py` converts flat
`<id>.posegraph/.data/.yaml/(raster)/.manifest.json` bundles to the directory
layout. It is manual only (never run at Paddock startup), idempotent, has
`--dry-run`, stages and publishes atomically, recomputes hashes, and moves
the flat originals into `maps/.legacy-migrated/` without deleting them.

### 8.2 Semantic layer (storage/UI only)

`semantics.png` is an **optional** 8-bit grayscale (non-palette) PNG aligned
1:1 with `occupancy.pgm` (same top-down row order; dimensions re-read from
disk and must match exactly; resolution/origin never duplicated — only
`map.yaml` has them). Pixel *values* are authoritative:

| Value | Class |
|---|---|
| 0 | UNCLASSIFIED |
| 1 | CAUTION |
| 2 | LETHAL |

Every other byte value is rejected today. Storage and validation are in
`semantics.py`/`png_codec.py` (stdlib only, atomic write, manifest refresh).
The Paddock editor (DISPLAY tab → Semantic layer) offers UNCLASSIFIED /
CAUTION / LETHAL / ERASER brushes, UNDO, REVERT and SAVE, and the server
serves the raster read-only.

**Semantics currently have no effect on Nav2, planning, D2, RPP or any
autonomy component.** `semantics.py` states that nothing else reads it and a
source search found no consumer in `runner_bringup`, the BT nodes,
`runner_path_speed_profile` or the RPP fork. That the class names read
"CAUTION"/"LETHAL" describes an editing vocabulary and a future intent, not
current behavior. A camera-based or offline surface-classification pipeline
that paints this same aligned raster is **future design only**. Whether the
editor has been exercised in a real browser session is not evidenced in the
repository; the studio map's `semantics.png` is untracked.

## 9. Paddock tuning surfaces and operator tooling

### 9.1 Named Speed Profiles

`99d1fea` replaced the TIMID / CONFIDENT / INSANE / ABSURD preset
abstraction (and the partial-reset behavior that could desynchronize
`desired_linear_vel` from D2 policy). **The Timid/Confident/Insane/Custom
tables in v1.6 §12 are historical and no longer describe the tuning
architecture.**

A **Speed Profile** is a complete driving-behavior snapshot of exactly 24
fields (`SPEED_PROFILE_FIELDS`, source-confirmed):

- controller (7): `desired_linear_vel`, `regulated_linear_scaling_min_speed`,
  `regulated_linear_scaling_min_radius`, `min_lookahead_dist`,
  `max_lookahead_dist`, `lookahead_time`,
  `max_allowed_time_to_collision_up_to_carrot`;
- adapter (1): `maximum_commanded_speed`;
- D2 (16): `minimum_traversal_speed`, `tight_clearance`, `open_clearance`,
  `clearance_curve_family` (0 linear / 1 power / 2 smoothstep),
  `clearance_curve_shape`, `approach_time_s`, `curvature_window`,
  `max_lateral_acceleration`, `footprint_front`, `footprint_rear`,
  `footprint_half_width`, `braking_linear`, `braking_constant`,
  `reaction_time_s`, `recovery_acceleration_gain`,
  `recovery_acceleration_floor`.

`constrained_speed_scaling` and `scaling_reference_speed` no longer exist
anywhere. Fields outside a profile: the **Engineering** set
(`proportional_gain`, `integral_gain`, `feedforward_effort_per_speed`,
`feedforward_effort_intercept`, `output_max`) and the **planner** field
`cost_penalty`.

Behavior (source-confirmed):

- **LOAD** copies a saved profile into the browser's draft and graph; it does
  not apply it. **APPLY LIVE** writes the draft through one atomic,
  read-back-verified transaction across three owners (`/controller_server`,
  `/drive_adapter`, `/bt_navigator`). **SAVE AS** validates and stores a
  named profile in `~/.config/runner/speed_profiles.json`; it performs no ROS
  write. **DELETE** removes a saved profile. `Default Baseline` is an
  immutable, always-present reference (its values duplicate the launched
  origin and cannot be overwritten or deleted).
- Every write and load goes through the one `validate_values` schema
  (`autonomy_tuning.py`), completing missing engineering/planner fields with
  permissive neutral stand-ins so a loaded profile's validity depends only on
  its own fields.
- The Speed Profile tab has draft/dirty state (`speed_profile.js`) so
  live read-back no longer overwrites in-progress edits. This fixed the
  earlier slider snap-back (`b607b98`, `731fe34`).
- **Field-level error attachment is NOT implemented.** An Apply failure
  appears as a single result line (`profile-tuning-result`); nothing marks
  the responsible knob. Improving this is open UX work.

### 9.2 Persistent startup override (separate from named profiles)

`SAVE OVERRIDE` (`save_autonomy_tuning`) validates a snapshot and writes only
the D2 and planner-owned parameters to
`~/.config/runner/speed_profile_overrides.yaml`. `nav2.launch.py` loads this
file (when present) as an extra parameter file for `planner_server` and
`bt_navigator`, so it takes effect at the next AUTONOMY start. It is **not**
the same as a saved named profile, and it does not carry controller/adapter
fields (those come from the tracked config until a live Apply).

**Open bug (source-confirmed at HEAD, unfixed).** `persist_override` formats
values with `{value:.12g}`, so `50.0` is written as `50`. ROS parameter
overrides are type-checked: an integer written for a double parameter caused
a `planner_server` configuration failure for `GridBased.cost_penalty`. This
is a Paddock **persistence/serialization bug**, not planning-policy
evidence. At authoring time the live override file had `cost_penalty: 50.0`
(evidently corrected by hand) while other double-typed `bt_navigator` entries
were integer-looking (for example `clearance_curve_family: 2`,
`max_lateral_acceleration: 1`); why `bt_navigator` tolerated them is not
established here. This spec does not fix or explain it.

### 9.3 Planner settings

`GridBased.cost_penalty` has its own editor ("Planner settings") and is
never touched by profile load/save/delete. The tracked value is **2.0**
(`nav2_params.yaml`, also the Smac default). Live experiments have
**physically preferred `cost_penalty = 50`** and the live override file holds
it, but **50 is not ratified canonical policy** and is not the tracked value.
The Paddock page itself says no value is implied or ratified.

### 9.4 Runtime-only values on the Pi (not canon)

At authoring time the live override held `minimum_traversal_speed: 0.75`
(tracked 0.25), `open_clearance: 0.4`, `clearance_curve_shape: 0.5`,
`approach_time_s: 0.1`, `max_lateral_acceleration: 1`, and the store held a
saved profile named `fast` with `desired_linear_vel =
maximum_commanded_speed = 2.0`. These are experiments under active tuning.
Nothing in this spec ratifies them, and a documented default profile has
not been designated (§19).

### 9.5 Other Paddock surfaces

- **Engineering / controller** disclosure (AUTONOMY tab): the five adapter
  fields above, applied with `APPLY ENGINEERING`.
- **Obstacle processing**: live-toggleable global/local obstacle layers
  through an atomic set→verify; `Clear transient obstacles` calls Nav2's
  clear-entirely services and does not touch the static map.
- **Recording**: `runner_debug`, `navigation_debug`, `everything` profiles
  (§14).
- **Manual browser speed**: `manual_max_speed_mps` 0 or [0.25, 0.40], default
  0.40.
- **Validation coupling (not a control change).** `validate_values` requires
  `output_max ≥ feedforward(maximum_commanded_speed)` (≈0.255 at 2.0 m/s with
  the current feedforward) and enforces the shared absolute ceilings 2.0 m/s
  and 0.30. Because Engineering fields are outside a profile, a high-speed
  profile is rejected on Apply until Engineering `output_max` is compatible
  (the tracked default is 0.14). This is a Paddock schema/UI interaction; it
  does **not** change motor-control architecture.

### 9.6 Route simulator (UNCOMMITTED — not established architecture)

Present only in the working tree: `src/runner_route_sim/` (a private
`/sim`-namespaced launch with its own `map_server`, `planner_server`,
lifecycle manager and a `route_sim_server` node), `SimulateRoute.srv`,
`route_sim_runtime.py`, and gateway/UI edits (SIM START / SIM GOAL /
SIMULATE / CLEAR). The intended behavior, as implemented in that code:
explicit saved-map start/goal with no localization (`use_start`), real Smac
planning in isolated resources, real `runner_path_speed_profile` math called
directly with the operator's Speed Profile snapshot, no fake live `/map`,
`/plan`, TF or `/cmd_vel`, gated off while AUTONOMY is active as a resource
guard (not a safety interlock), and torn down after a 600 s idle period.

**Status: unverified.** It is not in `main`; there are no tests for it; the
sim launch does not publish the static TF that its own config comment says it
does (a previously root-caused Costmap2DROS activation issue whose fix was
not applied); and it spawns a child `ros2 launch` from `paddock-web` rather
than a systemd unit, which is an ownership-model question not yet decided.
It would reproduce nominal deterministic planning and profile math; it is
structurally incapable of reproducing the live timing failure described in
§11.6. Do not treat any of this as current behavior.

## 10. Localization, estimation and TF ownership

Unchanged from v1.5/v1.6: D-37 fixed-cardinality SLAM scan
(`scan_rebinner → /scan_slam`, fixed 503 bins), RF2O via a canonicalized
scan into the EKF, `ekf_node` owning `odom → base_link`, `slam_toolbox`
owning `map → odom`, static extrinsics from existing publishers. **Initial
Pose** confirmation uses direct `map→base_link` TF within 0.25 m / 15°
(`6a6c452`); that closed the "no fresh slam_toolbox pose" report class at
source level but is **not field-revalidated**. The other v1.5 discrepancy —
costmap content apparently surviving an Initial Pose transaction despite the
clear phase — remains open and uninvestigated.

## 11. Navigation, planning and execution (current stack)

Nothing in this section is a claim of physical navigation quality except
where a physical observation is labelled.

### 11.1 Global planning (source-confirmed)

`planner_server` runs one plugin, `nav2_smac_planner::SmacPlannerHybrid`
(vendored in `src/nav2_smac_planner`), configured: `REEDS_SHEPP`,
`allow_unknown: false`, `smooth_path: false` (Jazzy's smoother tightened
paths beyond the Ackermann limit), `reverse_penalty 8.0`, `change_penalty
20.0`, `angle_quantization_bins 72`, `analytic_expansion_ratio 3.5`,
`analytic_expansion_max_length 1.0` (was 3.0), `max_planning_time 5.0`,
`minimum_turning_radius 0.60`, `cost_penalty 2.0` (tracked; §9.3),
tolerance 0.125. There is **no rotate-to-heading** anywhere
(`use_rotate_to_heading: false` in RPP; Smac plans reversing maneuvers
directly).

**Costmap snapshot (`93adbd4`).** Smac formerly held the live global
costmap's mutex for the whole search, blocking live updates. `createPlan`
now copies the complete grid and geometry under the mutex, releases it, and
plans on the request-owned snapshot. **No timestamp/freshness validation of
that snapshot exists in Smac source**; only the snapshot-then-unlock
mechanism does. Candidate validation in the BT (11.2) checks pose
availability and path validity, not costmap age.

### 11.2 Behavior tree and committed path (source-confirmed; Stages A1/B/C)

`navigate_to_pose_forward_only.xml` (and the through-poses variant) is
structured as follows. The "forward_only" name is historical (it named the
recovery posture, not a reverse restriction; reverse is enabled).

1. `UnsetBlackboard path`, then a one-retry `RecoveryNode`
   (`ReplanAfterControllerPatience`) wrapping a `PipelineSequence`.
2. At 3 Hz, a `RateController` runs: *retain the committed path* if
   `PathExists`, the goal is unchanged (otherwise `replan_reason =
   goal_update`), and `PersistentPathValid` holds (corridor 1.25 m,
   3 required observations, 2.0 m progress-search distance).
3. Otherwise *generate, validate and commit a candidate*:
   `RemovePassedGoals → ComputePath… → CandidatePathValid` (validates
   against the current global costmap; failure reasons
   `candidate_empty / candidate_current_pose_unavailable /
   candidate_validation_unavailable / candidate_invalid`) `→
   GeneratePathSpeedProfile → SetBlackboard path → ReportPathCommitment`.
   A rejected candidate is reported (`candidate_rejected`) and fails the
   sequence, keeping the previously committed path if one exists.
4. `FollowPath` executes the committed path. A local-control failure (for
   example RPP `NO_VALID_CONTROL`, 106) propagates to the `RecoveryNode`;
   `WouldAControllerRecoveryHelp` then unsets the path with
   `replan_reason = recovery_replan`, forcing exactly one bounded replan.
5. `controller_server.failure_tolerance: 2.0` publishes zero velocity and
   retries the retained path for up to 2 s before the failure reaches the BT
   (D-90 interim controller patience).

The stock `IsPathValid`-driven replan loop from v1.6 §11 is **gone**.
`navigation_runtime.py` adds the D-90 anti-redispatch-storm guard (identical
mission and robot pose, within 0.10 m and 10°, after the same failure is
rejected with `ANTI_REDISPATCH_STORM`), the 7-state lifecycle, generation
discipline, and full symbolic Nav2 error decoding (unchanged from v1.6).

### 11.3 Forward-first certified planning: attempted and reverted

`ad62558` implemented forward-first planning (a Dubins-primary plugin plus a
Reeds-Shepp fallback) with a candidate-path certification stage (curvature
certification with margin and reverse-segment checks), and `538c9ac` fixed a
window edge case in it. `cad538f` then **removed all of it**
(`candidate_path_certification.*`, the second planner plugin, related BT and
tests): global planning is back to a single Reeds-Shepp planner. Current
HEAD therefore has **no curvature certification and no minimum-reversal-
segment certification** of planned paths. The reasoning is recorded in the
planning reassessment (`docs/nav2_planning_reassessment_20260914.md`,
uncommitted): Smac has no cusp penalty and its analytic expansion can emit
short cusp segments; whether a vendored cusp penalty or another mechanism
will be adopted is undecided. v1.6's Stage D description and the D-92 label
must not be read as describing current behavior.

### 11.4 RPP tracker and cusp/segment ownership (source-confirmed, `2d95a93`)

The tracker remains the vendored Regulated Pure Pursuit lineage
(`src/nav2_regulated_pure_pursuit_controller`, `allow_reversing: true`,
velocity-scaled lookahead 0.30–0.80 m, `stateful: false`). There is no MPPI
and no Vector Pursuit in any configuration; Vector Pursuit was only ever a
candidate under discussion and is not adopted.

**Speed insertion point.** In `applyConstraints`: curvature-based regulation
(`use_regulated_linear_velocity_scaling: true`), then the
`regulated_linear_scaling_min_speed` floor, then
`linear_vel = min(linear_vel, profile_ceiling)`, where
`profile_ceiling = sampleCeiling(profile, path_handler.getPathOffset())` when
an identity-matched profile exists and otherwise
`path_speed_profile_fallback` (0.25 m/s) for the whole path. RPP's **cost
regulation is disabled** (`use_cost_regulated_linear_velocity_scaling:
false` in the launched config) so D2 owns hazard-based slowdown; the
**collision-imminent check remains enabled** and is the source of
`NO_VALID_CONTROL`.

**Identity contract.** A profile applies to a path only if
`validateProfile` passes and its `path_hash`, `pose_count` and
`committed_path_stamp` match the active path. RPP keeps a small cache, so a
profile handed off just before its path arrives matches; an unmatched or
changed profile drops to the fallback. The BT node hands the profile to RPP
by service *before* dispatching `FollowPath`, and refuses dispatch on
rejection or timeout.

**Cusp/segment ownership.** The path handler owns the segment, the global
progress index/offset and the cusp state. Segment terminals are the
identity-matched profile's `CUSP_STOP` points (plus the path end). Pruning
and lookahead cannot cross the active cusp. Advancement to the next segment
requires the robot to be at the segment terminal **and a fresh post-cusp
`EncoderState.stationary == true` sample** (a sample whose sequence is newer
than when the wait started); RPP publishes `WAITING_FOR_STATIONARY`
meanwhile. D2 still samples `getPathOffset()`. `/navigation/path_execution_state`
is read-only, and the Stage C progress check uses this identity-matched state
instead of its old nearest-path estimator. **`motor_node`, `drive_adapter`
and mux semantics were intentionally unchanged** — the motor still performs
its own reversal stop/confirm/breakaway.

**Physical observation.** Star turns became definitively better after this
change. Occasional failed or awkward tiny reversals remain. **This is not
perfect maneuver execution and is not claimed to be.**

### 11.5 Stage status against the ratified roadmap

| Stage (v1.6 §19.2) | Current state |
|---|---|
| A1 obstacle-evidence coherence | **Implemented** (`57ae77c`): Max composition in both costmaps, aligned marking/clearing, 4 m local window (§11.7), 10 Hz local publish, bag analyzer (`navigation_bag_analyzer.py`) as replay harness. |
| A2 evidence filtering | **Not adopted.** The first post-A1 measurement showed baseline flicker (~0.25–0.27 % per frame) but that bag's 106s traced to persistent unmapped obstacles; A2 was judged not yet justified. |
| B recovery executive | **Interim implemented** (`4db4f27`): controller patience + one bounded replan + redispatch guard. The full `BLOCKED → WAIT → RESUME → REPLAN → ABORT` ladder with operator reason codes is not confirmed. |
| C path commitment | **Implemented** (`87c9a92`, `PersistentPathValid`, `CandidatePathValid`) — behavior now governed by D-91 (§17.1); its parameters remain tuning. |
| D planner/path contract | **Forward-first certification reverted** (§11.3). The *speed/clearance profile and transport* portion was delivered as D2 (§11.6). |
| E Runner tracker | **Partial:** profile consumption and cusp protocol in the vendored RPP. Not built: hard curvature cap, creep instead of the regulated speed floor, executable-trajectory stopping-distance guard, `BLOCKED` status. |
| F retrace | Not started. |
| G CONFIDENT validation at ~1.0 m/s | Not performed; the named-preset targets it referenced no longer exist. |

### 11.6 D2 committed-path speed profile

**Definition.** D2 is **per-commit, path-relative speed policy attached to
the committed path**, computed once when a candidate path is accepted. It is
not low-level actuator control: it produces a per-point speed *ceiling*,
which RPP consumes as a ceiling; the drive adapter still converts whatever
speed RPP commands. Planner geometry, commitment, D2, tracker and actuator
safety remain separate layers, and D2 does not replace any of the others'
safety checks.

**Implementation** (`runner_path_speed_profile`, a ROS-independent library
plus the `GeneratePathSpeedProfile` BT node; source-confirmed):

1. *Clearance per pose.* The global costmap's LETHAL and NO_INFORMATION cells
   seed a squared Euclidean distance transform; the **oriented footprint**
   (8 boundary samples of the 0.29 × 0.165 m rectangle, rotated by the pose
   heading) is sampled and its minimum distance, less a half-cell radius, is
   the pose's clearance. An off-grid sample yields zero clearance. This
   replaced the earlier "center distance minus circumscribed radius"
   approximation.
2. *Clearance → speed.* `x = clamp((clearance − tight) / (open − tight), 0, 1)`,
   shaped by `clearance_curve_family` (linear / power `x^shape` / smoothstep
   of the power) and scaled between `minimum_traversal_speed` and the
   preset ceiling (`desired_linear_vel`). This replaced the original
   hyperbolic law and, transiently, coarse creep/caution/open tiers with
   hysteresis (`6c6e81c` → `8774fa2`).
3. *Curvature cap* `√(max_lateral_acceleration / κ)`, with κ from the heading
   change over `curvature_window` (boundary-aware near the path ends).
4. *Cusp and goal zeros.* Direction-sign changes between adjacent poses mark
   `CUSP_STOP` (speed 0) with `APPROACH`/`DEPARTURE` neighbours; the final
   pose speed is exactly 0.
5. *Passes.* A backward **anticipation** pass (`approach_time_s`), a forward
   **recovery** pass with speed-dependent acceleration
   `a(v) = gain·v + floor`, and a backward **braking** pass using
   `a(v) = braking_linear·v + braking_constant` plus `reaction_time_s` of
   travel. The raw clearance/curvature/cusp/goal ceiling is never exceeded.

Fixed bugs: the `approach_time_s` pass was initially inert due to a
backward-pass bug (fixed in `515cf0a`).

**Tracked D2 defaults** (`nav2_params.yaml` on `bt_navigator`):
`minimum_traversal_speed 0.25`, `tight_clearance 0.05`, `open_clearance 0.70`,
curve family 2 (smoothstep), shape 1.0, `approach_time_s 0.0`,
`curvature_window 0.40`, `max_lateral_acceleration 0.35`, footprint as above,
`braking_linear 1.6`, `braking_constant 0.27`, `reaction_time_s 0.40`,
recovery gain 1.6 / floor 0.60. These are tracked starting points, **not
validated safety limits**, and live overrides differ (§9.4).

**Operator tuning intent (not validated numbers).** Open space may target
2.0 m/s and potentially higher later; constrained spaces should slow
continuously rather than in coarse tiers; 0.25 m/s is generally considered
unnecessarily slow now; around 0.5 m/s can be reasonable for very
constrained, chair-like geometry; hallway/medium constraints may support
roughly 1–1.5 m/s depending on geometry. These are current observations and
intent. **They are not safety limits and are not ratified.** The profile
maker works in the UI and in live use; exact parameter quality remains under
active tuning.

**Unresolved — intermittent whole-route collapse to minimum traversal
speed.** Under rapid re-dispatch, profiles sometimes come out at (or near)
`minimum_traversal_speed` along the whole route. Bag `Runner_20260917_112956`
(as reported by Matti; not re-analysed in this pass) shows real repeated
collapse; the selected preset ceiling stayed 2.0; curvature, cusps and
`approach_time` were ruled out for a strong contrasting pair; profiles 388 ms
apart within one action differed materially. **The root cause is unresolved.**
Source-level facts that bear on it, without being a diagnosis:

- `GeneratePathSpeedProfile::tick` performs several synchronous cross-process
  reads (BT parameters, `FollowPath.desired_linear_vel`, the costmap
  subscription) and, on failure, degrades silently: an unreadable preset
  leaves the ceiling at `minimum_traversal_speed` (a flat profile); a missing
  costmap gives empty clearance; RPP falls back to 0.25 m/s if no
  identity-matched profile exists. The preset-read mechanism is contradicted
  for the reported bag by the stable 2.0 ceiling; that leaves live costmap
  content/timing (raw grid, `worldToMap`, obstacle-layer churn) as leading
  hypotheses, none demonstrated.
- The `CostmapSubscriber` cache has no staleness tracking, and
  `navigation_debug` does not record `/global_costmap/costmap_raw`.
- An offline simulator (§9.6) would not reproduce this timing failure.
Later source-only hypotheses do not replace the bag-grounded result.

### 11.7 Costmaps and world representation (source/config-confirmed)

Both are `Costmap2DROS` instances inside `planner_server`
(global) and `controller_server` (local).

| | Local (`odom`, rolling) | Global (`map`, static) |
|---|---|---|
| Size / resolution | 4 × 4 m (derived lower bound 3.53 m from stopping distance + max lookahead + footprint margin, rounded up to integer metres), 0.025 m | map extent, 0.05 m |
| Update / publish | 10 Hz / 20 Hz threshold (measured 10 Hz live per the yaml comment; not re-measured here) | 3 Hz / 1 Hz |
| Layers | obstacle, inflation | static, obstacle, inflation |
| Obstacle range | min 0.05, **max 5.0**, raytrace max 6.0 (was 1.0 / 1.2 at v1.6) | min 0.05, max 5.0, raytrace 6.0 (**unchanged since the 2026-07-26 planning bringup**) |
| Obstacle `combination_method` | 1 (Max), explicit | **1 (Max)** (was 0/Overwrite at v1.6) |
| Inflation radius / scale | 0.45 / 10.0 | **0.50** (was 0.30) / 10.0 |
| Unknown space | tracked false | tracked **true**; D2 clearance treats NO_INFORMATION as an obstacle |

**Correction of an old claim.** Stage A1 changed the **local** obstacle range
1.0→5.0 (and raytrace 1.2→6.0); it did **not** change the global range,
which was already 5.0 before Stage A1. An earlier claim to the contrary is
disproved by `git show 058e56a:…/nav2_params.yaml`.

**D-89 status.** Max composition gives the static layer clearing authority
over its own cells: live rays can clear live marks but cannot erase static
occupancy. Planner and executor keep separate costmap objects (different
frame/resolution/extent/rate) under the shared-semantics rule. The static
map, the live obstacle layer, the master costmap and Paddock's visualization
frames are four distinct things: Paddock's map/costmap/plan frames are
demand-driven views and never feed control.

**Unresolved world-model quality problem.** Dynamic obstacles such as chair
legs are, by physical and operator observation, forgotten or cleared from
useful planning memory too aggressively. Runner can later plan through where
they were until it reacquires them at close range. This is considered a
serious world-model quality regression. **The cause is unresolved and no fix
is established.** It is in tension with the D-89 clear-promptly rule (which
targets phantom persistence, not this behavior) and should be treated as
needing review; candidate factors (rolling-window reset, 6 m raytrace
clearing, Max composition, window churn) are untested hypotheses. A
side-effect hypothesis, unverified in this pass, is that widening the local
window doubled RPP's default `max_robot_pose_search_dist` (default is
half the local width).

### 11.8 Known navigation weaknesses carried forward

- **Reverse-start nose-in deadlock — unresolved.** Smac never
  collision-checks its own start node; the candidate validator does, and
  correctly rejects genuine nose-in wall contact, so a valid reverse-start
  plan can be refused. There is no unstick/backup recovery behavior in the
  BT or runtime. The fix direction (an unstick recovery, not weakening
  `CandidatePathValid`) is not implemented.
- Three 106 clusters from v1.6 §19.1 (immediate costmap-disagreement 106s,
  cusp-tick curvature spikes, corner-cutting at the speed floor) were
  diagnosed on the pre-A1/pre-D2 stack; which persist today has not been
  re-measured.
- Curvature saturation at the adapter clamp is a known minority amplifier;
  given §2.3 its real meaning on the new servo is now unknown.
- The three "radius" values (0.60 planning, 0.75 speed-regulation, adapter
  clamp/physical) have distinct roles; their mutual margins for the eventual
  execution contract remain open, and the physical side of this comparison is
  now the WIP steering characterization.

## 12. Longitudinal control and speed authority

`drive_adapter` remains the one shared closed-loop conversion for Nav2's SI
`/cmd_vel_nav` and authority-bounded manual demand. Source-confirmed
semantics (unchanged since v1.6):

- **MD13S** sign-magnitude PWM + DIR. Effort is normalized 0..1 with sign.
- **Feedforward** `ff = 0.1188·|v| + 0.0174`; **PI** `P = 0.05`, `I = 0.01`,
  `integrator_bound 0.005`; integrator frozen for zero command, stale
  feedback, wheelspin, direction mismatch, arbitration loss, output not
  selected, or saturation driving farther.
- **`speed == 0` is an explicit brake** (`explicit_stop`, duty 0), not a
  floor promotion. Nonzero magnitude below `minimum_moving_speed` (0.25,
  restart-only) is promoted to that floor (`floor_promoted`); magnitude above
  `maximum_commanded_speed` is clamped.
- **`output_max`** clamps final effort; the launched default is **0.14**.
- **Wheelspin guard** is a boolean diagnostic that freezes the integrator; it
  does not reduce output.
- **Direction changes (D-74/D-75 subject).** The motor node enforces the
  stationary-confirmed reversal gate: a request to flip DIR sets duty 0 and
  waits for a fresh post-request `stationary` encoder sample. No upstream
  component owns this permission.

Five distinct speed concepts must not be conflated:

| Concept | Owner | Meaning |
|---|---|---|
| Desired/profile speed | D2 profile (`desired_linear_vel` is its preset ceiling) | per-point ceiling for the committed path |
| `maximum_commanded_speed` | adapter (live-tunable) | clamp on commanded speed |
| `output_max` | adapter (live-tunable, Engineering) | actuator authority (effort) limit |
| Feedforward requirement | derived | `ff(maximum_commanded_speed)` must fit under `output_max` |
| `minimum_moving_speed` | adapter (restart-only) | floor below which nonzero commands are promoted |

Shared absolute experimental ceilings live in
`AutonomyTuningPolicy.msg`: **`maximum_commanded_speed = 2.0`** and
**`output_max = 0.30`** (one constant source for both validators, the fix
from `4b6d850`). They are validation ceilings only and **not evidence that
2.0 m/s or 0.30 effort is physically validated**. The launched origin
(`speed_envelope.yaml`) is `desired_linear_vel 1.00`,
`maximum_commanded_speed 1.00`, `output_max 0.14`,
`regulated_linear_scaling_min_speed 0.40`, `max_allowed_time_to_collision_up_to_carrot
0.60`, cost regulation off. At v1.6 the same file held 0.45/0.60 values.
Live-tunable adapter parameters remain `maximum_commanded_speed`, the two
feedforward terms, `output_max`, `proportional_gain`, `integral_gain`;
anything else requires a restart.

**D-84 (MD13S plant characterization, controller frozen)** remains the basis
for the feedforward/PI values; its *universal 0.14 effort ceiling* was
already superseded by D-88, and 0.14 is now merely the launched default.
The Insane preset and the fixed preset table no longer exist; the D-88
statement that the absolute ceilings are experimental validation limits
remains.

## 13. Steering and lateral-control stack (summary; details §2.3)

- **Nav2 curvature path.** RPP produces linear/angular velocity → adapter
  converts curvature `κ = ω/v` to normalized steering by
  `atan(L·κ)/max_steering_angle`, clamped to ±1, reporting
  `steering_saturated` when `|κ|` exceeds the configured clamp → `/cmd_vel`
  `angular.z` (normalized, positive left) → `motor_node` maps to ±450 µs
  about 1500 µs with `STEER_SIGN = -1`.
- **DualSense / manual paths** are open-loop normalized steering into the
  mux; they bypass the adapter's curvature conversion.
- **Provisional.** Because the mapping from that normalized value to real
  curvature is **unmeasured except at full lock** (§2.3: ≈1.68 left /
  ≈2.32 right, below ≈1.1 m/s), the adapter's steering saturation flag, the
  RPP curvature-related speed regulation, D2's curvature cap, and any
  "feasible curvature" reasoning all rest on a mapping that is
  physical-validation-pending. Measured command-to-yaw dead time (~105–110 ms
  plus ~40 ms lag) is a real vehicle-response fact that RPP lookahead and
  D2 reaction time have not been re-tuned against.

## 14. Recording and observability

Three MCAP profiles from `RecordingExecutor` (`ros2 bag record --storage
mcap`, safe basenames, clean-then-escalated stop, non-overwriting):
`runner_debug`, `everything`, and `navigation_debug`. `navigation_debug`
includes the scan, `/local_costmap/costmap`, `/global_costmap/costmap`,
`/plan`, odometry/IMU, TF, `/map`, the `/cmd_vel*` chain,
`/drive_adapter/state_typed`, `/speed_envelope/status`, Paddock
navigation/authority/lease/control/stop/mode/map/config state,
`/navigation/path_commitment_state`, `/navigation/path_speed_profile`,
`/behavior_tree_log`, `/lookahead_collision_arc`, `/motor/direction`,
`/wheel/encoder_state`, telemetry, battery and `/rosout`. **It does not
include `/navigation/path_execution_state` or `/global_costmap/costmap_raw`**
— the latter is the grid D2 actually samples, which matters for the
unresolved collapse (§11.6). Whether to add them is an open question (§19).
The catalog rescan excludes the active recording's own sidecar.
`navigation_bag_analyzer.py` in `runner_bringup` is the offline analysis
harness from Stage A1.

## 15. Networking, deployment and CPU/performance

**Deployment.** Field Wi-Fi AP/captive portal, Tailnet path and
`services/install.sh` coherent-deploy tooling are unchanged. Every tracked
config change (costmaps, BT XML, launched speed-envelope values) needs the
same `--check`/`--restart`-or-reboot discipline; Python/C++ changes in the
working tree are not in any service until built and restarted. The motor
service was restarted 2026-09-19 23:49 after the steering commits, which is
consistent with the new mapping running but is not proof of it.

**CPU is an explicit design constraint on the Pi 5.** Established
optimizations (all source-confirmed, in tracked config/code): QoS-event
handler suppression on high-churn Paddock nodes, demand-driven large-frame
visualization, duplicate-suppressed and slower state heartbeats
(`command_authority_state` 0.2 s, `map_state` 1.0 s,
`speed_envelope/status` 2.0 s, `navigation_state` deduplicated), SLAM
`transform_publish_period` 0.10 s, `bond_heartbeat_period 1.0`,
`bt_loop_duration 20`, global costmap update 3 Hz, and active-recording
catalog fingerprint narrowing. The route simulator is designed to be
resident only on demand and idle-shutdown after 600 s.

**Figures, all dated and contextual — none is a current validated
measurement:**

- v1.4/v1.5-era bag `confident_0` (session date not restated here; v1.5 §17): under Confident-preset load `total_cpu` mean
  95.5% with command gaps (before that round's cleanup); Paddock web ≈40% of
  one core and `mode_supervisor` ≈30% of one core, unattributed.
- 2026-09-13 bag `Runner_20260913_184429_0.mcap` (post-CPU-fix, pre-A1): mean
  49% / peak 94% total CPU, no throttling flags.
- 2026-09-20, a single ad-hoc `top` sample on the live Pi, AUTONOMY runtime
  up with no mission: 37% user / 53% idle; largest consumers Paddock web
  ≈27% and several persistent nodes at ≈9–18% each. **One sample: not a
  validated idle figure.**
- **No measurement exists of the CPU effect of Stage A1's local costmap
  change** (160² cells, 5 m/6 m ranges, 10 Hz publish) or of D2.

Rules carried forward: avoid persistent optional polling or high-rate
processing without justification; a silent failure mode (stale data,
dropped commands) needs a quantitative validation, not an absence-of-alarm
argument; MPPI's Pi CPU viability remains unmeasured.

## 16. Supported operator vs Advanced/Engineering configuration

Normal operator surface: runtime selection, manual driving, map lifecycle,
Speed Profile load/apply/save-as, planner `cost_penalty`, obstacle
processing, initial pose, recording, manual-speed ceiling, STOP/CLEAR STOP,
takeover. Engineering surface: adapter feedforward/PI/`output_max` and any
`ros2 param`, systemd or SSH work (outside Paddock's authorization model).
Speed Profile owns operator-facing longitudinal behavior; Engineering owns
actuator calibration; they are deliberately separate stores and editors,
with `SAVE OVERRIDE` a third, reboot-persistent channel (§9).

## 17. Decision-log reconciliation

Dispositions of every D-identifier named in v1.6. D-91, D-93 and D-94 are
ratified by this v1.7 pass in the scoped form of §17.1; no other decision is
newly ratified.

| Decision | Disposition | Evidence / notes |
|---|---|---|
| D-37 fixed SLAM scan cardinality | **STILL CURRENT** | scan path untouched |
| D-82 SIGKILL/PWM hazard, heartbeat-gated FET deferred | **STILL CURRENT (open)** | `runner_motor` change is steering only |
| D-84 MD13S characterization / freeze | **PARTIALLY SUPERSEDED** | feedforward/PI/freeze current; universal 0.14 ceiling superseded by D-88 |
| D-88 experimental 2.0 m/s / 0.30 ceilings | **IMPLEMENTATION CHANGED, INTENT SAME** | ceilings in `AutonomyTuningPolicy.msg`; Insane and the fixed presets are gone (`99d1fea`); ceilings remain validation limits, not validated |
| D-89 obstacle-evidence coherence | **RATIFIED; A1 IMPLEMENTED; NEEDS REVIEW** | Max composition, derived window (`57ae77c`); A2 not adopted; chair-leg forgetting unresolved and in tension with its clear-promptly rule |
| D-90 recoverable local-control failure / anti-storm | **RATIFIED; PARTIALLY IMPLEMENTED** | `4db4f27`; operator-facing reason codes and Retrace not confirmed |
| D-91 committed-path retention | **RATIFIED in v1.7 (principle only); IMPLEMENTED** | `87c9a92`; see §17.1. The BT parameters (1.25 m, 3, 2.0 m) remain tuning, not ratified |
| D-92 forward-first policy and path certification | **UNRATIFIED LABEL — deferred; implemented then reverted** | `ad62558` → `cad538f`; single Reeds-Shepp planner; no certification |
| D-93 identity-bound path speed profile contract | **RATIFIED in v1.7; IMPLEMENTED** | `PathSpeedProfile` + `SetPathSpeedProfile` + identity hash + transient-local topic; see §17.1 |
| D-94 tracker ownership of progress and cusps | **RATIFIED in v1.7 (narrow); IMPLEMENTED in the RPP fork** | `2d95a93`; see §17.1. The broader v1.6 label (shared trajectory-guard/`BLOCKED` helpers, MPPI/VP alternatives) is **not** ratified and not built |
| D-95 retrace | **UNRATIFIED LABEL — not started** | no source |

D-92 and D-95 were provisional planning labels in v1.6 and carried no
authority; they remain unratified. Historical decisions relevant to current design (from v1.2–v1.5,
still in force per the v1.3/v1.5 reconciliation tables): D-55 (adapter path
through the authority), D-73/D-74/D-75 (zero brake, signed reverse,
stationary-confirmed direction switching, persistent encoder), D-83
(deliberate DualSense shaping), D-85 (reverse autonomy), D-86 (single
launched speed origin, now qualified by the Speed Profile editor being
authoritative for live tuning), D-87 (purpose/discipline), D-29, D-71/D-76/D-79/D-80.
Open decision questions are in §19.

### 17.1 Decisions ratified by this v1.7 pass (2026-09-20)

Each is an architectural boundary; the numeric parameters and current
implementation details named as tuning are explicitly **not** ratified.

**D-91 — Committed-path retention.** Runner intentionally retains an accepted
(committed) path for as long as it remains valid, rather than continuously
replacing it with newly planned candidates. Candidate generation and path
execution are separate concerns: a new candidate is generated, validated and
committed only when the retained path is no longer valid (or the goal
changes), and a **failed or rejected candidate does not by itself destroy the
committed path**. Future work must not casually revert this to
continuous-replan semantics. *Not ratified:* the current corridor length
(1.25 m), required observations (3), progress-search distance (2.0 m), the
validity test's specific mechanism, and the recovery-ladder details — these
remain tuning (source: `PersistentPathValid`, `CandidatePathValid`, the BT
XML).

**D-93 — Identity-bound path speed profile contract.** The execution
architecture is: committed path → identity-bound `PathSpeedProfile` →
tracker. A profile belongs to one specific committed path and is valid for
that path only if its identity (`path_hash`, `pose_count`,
`committed_path_stamp`) matches; it is handed to the tracker **before**
execution begins; and failure to establish that contract (rejection or
timeout) **prevents dispatch** of the path. D2 is a path-relative policy
layer producing a per-point ceiling that the tracker consumes; it is not
actuator control and does not replace the tracker's or actuator's own safety
checks. *Not ratified:* the D2 clearance curve, braking/recovery parameters,
lateral-acceleration cap, defaults, and the specific transport (service plus
transient-local topic) beyond the identity-and-handoff requirement — these
are active research/tuning. The tracker's behavior when no matching profile
exists (currently a 0.25 m/s fallback) is implementation, not part of this
decision.

**D-94 — Tracker ownership of execution progress and cusps (narrow).** The
tracker/path handler owns progress along the committed path, the segment and
cusp state, and the transition across a reversal. Crossing a cusp requires
**fresh downstream stationary evidence** (a post-cusp stationary encoder
sample newer than the start of the wait). Other components may **observe**
execution progress (`PathExecutionState`) but must not independently
reconstruct it or compete for ownership of it. The contract is
tracker-implementation-agnostic: it is to survive replacing RPP with another
tracker (Vector Pursuit or otherwise), so ownership and interface are
ratified, not RPP. *Not ratified:* "RPP is the permanent tracker"; the v1.6
D-94 label's broader scope (shared trajectory-guard/`BLOCKED`-status helpers,
MPPI/Vector-Pursuit as alternatives); and any claim that current cusp
execution is good (tiny-reversal failures and the reverse-start deadlock
remain open, §11.4, §11.8). Motor-side reversal gating (D-74/D-75 subject)
remains motor-local and unchanged.

## 18. Physical-validation status

| Item | Status |
|---|---|
| Steering PWM sign (`STEER_SIGN = -1`), 1500 µs center, 1050/1950 µs endpoints (`STEER_US = 450`) | **Physically observed and implemented**; source-confirmed |
| Full-lock curvature L ≈1.68 / R ≈2.32 1/m (≈0.595 / 0.431 m), <1.1 m/s, ≈38% asymmetry | **Physical measurement**, `turn_radius_0.mcap`, one dataset |
| Command-to-yaw dead time ≈105–110 ms, lag ≈40 ms, zero-crossing median ≈171 ms, no limit cycle | **Physical measurement**, `turn_response_0.mcap`; vehicle response, not servo transit |
| Partial-steer κ(command); `max_steering_angle`, wheelbase, min radius as geometry | **Pending** (planned 0.7–0.9 m/s run at 0, ±0.25/0.50/0.75/1.0) |
| Roll-off above ≈1.1 m/s; ≈3–4 m/s² lateral | **Observation only**, not a limit |
| Servo buzzing under load | Hardware characteristic, not investigated |
| Faster servo's effect on autonomy tracking | **Observed: no obvious major improvement** (initial testing); cause not established |
| Smac `minimum_turning_radius 0.60` | Source-confirmed; consistent with the measured limiting side |
| Star turns after `2d95a93` | **Physically observed better**, not perfect; tiny-reversal failures remain |
| Reverse-start nose-in deadlock | **Unresolved** |
| Max-composition costmaps, 4 m window | Source/config-confirmed; 10 Hz local publish per yaml comment; **chair-leg forgetting unresolved** |
| D2 profile law, oriented-footprint clearance | Source-confirmed; works in UI and live use; **parameter quality under active tuning**; collapse-to-minimum bug **unresolved** |
| `cost_penalty = 50`, `fast` 2.0 m/s profile, live override values | **Runtime experiments**; not canon |
| 2.0 m/s / 0.30 output ceilings | Validation limits only; **not physically validated** |
| Override int serialization (`{:.12g}`) | **Open bug at HEAD** |
| Field-level Speed Profile error attachment | **Not implemented** |
| Semantic editor | Implemented and tested in-tree; real-browser validation not evidenced; **no effect on planning** |
| Map directory layout, migration | Source/test-confirmed; migration tool manual |
| Route simulator | **Uncommitted, no tests, unverified** |
| Initial-pose TF confirmation | Source-level only; not field-revalidated |
| Tests | Exist in tree; not executed for this document |
| CPU | Dated figures only (§15); no current validated measurement |

## 19. Known limitations, open questions and items needing confirmation

**Unresolved engineering items (must remain visibly open).**

1. Steering: partial-steer κ(command), true angle/wheelbase/radius geometry,
   L/R asymmetry cause, servo dynamics vs vehicle response, re-tuning of
   RPP/D2 against measured response. Adapter mapping is WIP.
2. D2 collapse-to-minimum-speed root cause (§11.6); D2 parameter tuning;
   whether `/global_costmap/costmap_raw` and `path_execution_state` should be
   recorded.
3. Chair-leg/local-costmap forgetting (§11.7).
4. Reverse-start nose-in deadlock; no unstick recovery (§11.8).
5. Star-turn tiny-reversal failures; Smac emits short cusp segments; no cusp
   penalty; no reversal-segment certification.
6. Override serialization bug; `bt_navigator` tolerance unexplained;
   hand-edited live override file.
7. Missing field-level Speed Profile error attachment.
8. Route simulator: commit decision, TF fix, tests, and ownership model of
   its child process.
9. Initial Pose costmap-survival discrepancy; TF-confirmation not
   field-revalidated.
10. One-click takeover has no owner-facing warning.
11. End-to-end stale-command timing unbudgeted; STOP during active autonomous
    motion and DualSense mid-mission takeover unexercised in any recorded
    session; SIGKILL/PWM hazard open; supervisor
    `RuntimeDirectoryPreserve=restart` smoke validation pending.
12. Sensing limitation: the planar LD19 cannot see obstacles above/below the
    scan plane.
13. `KeyboardState.msg` and `speed_envelope_observer` (still launched by
    `autonomy.launch.py`) have not been audited against the current
    single-origin doctrine.
14. The Foxglove unit is tracked but not installed; its intended status is
    unconfirmed.
15. Stage A2 mechanism, full Stage B ladder/reason codes, Stage E items,
    Retrace, and Stage G validation are not built.

**Decisions and confirmations still needed from Matti (not made in this
document).** Whether to ratify forward-first/reversal policy (D-92, deferred;
implementation was reverted and short reverse segments/cusps are unresolved)
and Retrace (D-95); which named profile (if any) is
the documented default given the launched 1.0 m/s origin versus the live
2.0 m/s `fast` profile; whether `cost_penalty` 50 becomes tracked policy;
whether the untracked nav2 docs and the route simulator are committed;
whether to re-derive the tracked `max_steering_angle` / clamp / test constant
after the partial-steer run.

## 20. Future exploration

Supervised frontier exploration and camera-derived semantic traversability
remain future work. The semantic raster (§8.2) is a storage/editing artifact
today; a later pipeline that paints it, and any consumer that turns classes
into costs, do not exist. A later mapping-time Nav2 composition must reuse
the live mapping raster and mapping `map → odom` without a second localizer
or map_server.

## 21. Historical appendix

Condensed history for continuity; §1–§20 govern where they differ.
Through the v1.3 migration stages, v1.4 cleanup and v1.5's CPU/streaming
investigations, see `docs/runner_spec_v1.5.md` §21. v1.6 (`058e56a`) added
one-click STOP/takeover UX, initial-pose TF confirmation, Nav2 error
decoding, the `navigation_debug` profile, wake-rate reductions, the
preflight-bounds fix, and the forensic investigation that ratified D-89 and
D-90 and the Candidate-B roadmap.

Post-v1.6 (`058e56a..7e15808`), grouped by subsystem with the surviving
mechanism and the superseded intermediates:

- **Navigation** — `57ae77c` A1 (survives: Max composition, 4 m window,
  10 Hz recording); `4db4f27` bounded recovery/anti-redispatch (survives);
  `93adbd4` planner snapshot (survives); `87c9a92` committed-path semantics
  (survives); `ad62558`/`538c9ac` forward-first certification (**reverted**
  by `cad538f`); `65f041d`/`8a6b2b5`/`674c397`/`1315c75` diagnostic tracing,
  a double-`future.get()` crash fix, BT plugin export, and BT-log recording.
- **Execution/cusps** — `2d95a93` (survives: RPP owns segment/cusp progress
  and publishes `path_execution_state`).
- **D2/speed policy** — `47503fb`, `c43f1a0`, `d92554a`, `6c6e81c`,
  `8774fa2`, `a7f3499`, `0e25ef1`, `04f86ac`, `515cf0a`. Surviving: the
  continuous clearance→speed law with oriented-footprint clearance,
  identity-matched handoff, RPP cost regulation off. Superseded: the
  original hyperbolic law, the creep/caution/open tier version with
  hysteresis, `constrained_speed_scaling`/`scaling_reference_speed`.
- **Paddock** — `4bf6f5a`, `ffffd89`, `b607b98`, `731fe34`, `99d1fea`
  (Speed Profile editor, draft/dirty state, named profiles replacing the four
  presets), `24d63a9`, `91b6a29`, `1d80e4b`.
- **Map storage / semantics** — `8facad6` (directory-per-map + migration),
  `0882633` (semantics.png + paint editor).
- **Steering** — `33117c3` (`STEER_SIGN = -1`), `7e15808` (`STEER_US =
  450`).
- **CPU/performance** — nothing new since v1.6's `ab78879`/`02b00c5` besides
  the local costmap change in A1 (no CPU measurement).
- **Other** — investigation documents under `docs/nav2_*` (uncommitted).
