# Runner v1.3 implementation status

## Stage 0 — contract installed

Ratified specification and amendments reconciled with 6a7c961. Documentation only.

## Stage 1 — private scaffold isolation

Unfinished authority outputs use /paddock/private/cmd_vel_auto and
/paddock/private/cmd_vel_paddock. Neither is a production mux input.
This default also applies to direct executable starts, not just systemd remaps.
Adapter remains the transitional sole /cmd_vel_auto writer when launched.
Legacy keyboard/Foxglove paths remain transitional until the gated cutover.
This is not v1.3 autonomous authority or global STOP implementation.

Validation: runner_paddock colcon build passed; existing authority plumbing tests
4/4 passed. Restarted installed authority, started the existing managed AUTONOMY
unit with studio, then stopped it. Live publisher counts:
- /cmd_vel_auto: 1, drive_adapter
- /cmd_vel_teleop: 1, runner_teleop
- /cmd_vel: 1, twist_mux
- /paddock/private/cmd_vel_auto: 1, runner_command_authority; 0 subscribers

Observed final Twist was all zero; traction duty_cycle 0 ns (steering 1500000 ns).
No mission dispatched or traction connected. Application stopped after smoke check.

## Stage 2 — acceptance gate not passed; prototype not deployed

See [the detailed gate report](paddock_v1.3_stage2_gate_report.md).
Temporary STOP tests observed 3.293 ms request-to-final-zero and a worst
142.026 ms stale nonzero interval after executor death. Rapid restart also
produced a service timeout and three discovered STOP publisher endpoints.
The probe's printed PASS did not assert that count and is not acceptance.

The prototype was removed, not installed as a persistent service. A clean
runner_interfaces/runner_paddock rebuild passed and the four plumbing tests
passed again. Both application units are inactive, /cmd_vel has zero publishers,
and traction PWM is 0 ns (steering 1500000 ns). Backend DoD is incomplete;
frontend was not started. No physical traction validation was performed.

## Stage 4 — truthful runtime and map execution

Backend only; validation is software/sensor-side with the traction battery
physically disconnected. No frontend, no mission runtime, no autonomy authority
cutover, no legacy autonomy mux input.

### Runtime identity and readiness

`ModeState` gains `runtime_epoch`, `mapping_session_id`, `ready` and
`readiness_reason`. The mode supervisor increments `runtime_epoch` on every
successful application start and allocates a fresh `mapping_session_id`
(`<epoch>-<token>`) per MAPPING start and per NEW MAP; the id is empty in IDLE
and AUTONOMY. Consumers reject map/session/readiness status that does not match
the current epoch/session.

Readiness is now capability-specific and continuously refreshed (0.5 s timer),
not only checked at transition:

- structural: the mode unit is `active`, every required node is *present*
  (duplicate node *names* — e.g. the known double-listed RF2O — are DDS
  discovery residue, not a duplicate owner), no autonomy-only node is present
  during MAPPING, and every critical topic's publisher-owner set matches
  exactly (endpoints whose node identity has not yet propagated are ignored).
- MAPPING capability: `map->odom` and `odom->base_link` TF usable, `/scan_slam`
  fresh for its rate, a `/map` update seen *after* the current session began
  (a retained old-session raster never satisfies this).
- AUTONOMY capability: TF usable, `/scan` fresh, `/map` present from
  `map_server`. Static map is not required to keep republishing.

`readiness_reason` carries the first actionable inhibit reason. The transition
gate waits on structural + capability readiness and only publishes `STABLE`
after it passes; failure publishes `FAULT` with the phase/reason and tears the
half-started graph down. `reconcile()` now gives a freshly active unit a bounded
grace period to finish graph discovery before concluding it is partial.

The supervisor node runs on a `MultiThreadedExecutor` with the readiness
evidence subscriptions in their own callback group and TF on its own thread, so
a blocking transition wait cannot starve the inputs it is waiting on.

### Mapping session, NEW MAP, SAVE MAP

New node `runner_map_executor` (`runner-map-executor.service`, persistent,
never a process-lifecycle owner) owns `/paddock/map_request` ->
`/paddock/map_state` (`MapRequest`, `MapState`, `MapCatalogEntry`).

- Entering MAPPING creates a fresh unsaved session. A repeated MAPPING request
  for the already-stable runtime is idempotent and does not reset the map.
- NEW MAP (`MapRequest.OP_NEW_MAP`, or `ModeRequest.operation = OP_NEW_MAP`) is
  a distinct operation: the executor forwards it to the supervisor, which stops
  the mapping application, verifies the old owners are gone, starts it again and
  allocates a new session epoch. Saved bundles are never deleted. The executor
  discards all state keyed to the old session id and does not report the new
  session ready until current-session `/map` evidence exists.
- SAVE MAP (`MapRequest.OP_SAVE_MAP`) requires a current valid MAPPING session,
  a matching `session_id`, mapping readiness and STOP asserted/inhibited. It is
  transactional: serialize into `maps/.staging/`, capture a same-session `/map`
  occupancy raster and write its PGM/YAML, validate the complete four-artifact
  bundle (non-empty `.posegraph`/`.data`, YAML parses, referenced raster exists
  with a plausible header, dimensions sane, session association), write a
  `<name>.manifest.json` (session id, creation time, revision, SHA-256 of the
  four artifacts), then atomically move the bundle into `maps/` (manifest last).
  Names are bounded safe basenames; separators, `..`, empty names and collisions
  with an existing bundle are rejected. Retries for the same `request_id` return
  the cached outcome. A half-written bundle stays in `.staging/` and never
  enters the catalog. Reuses the proven checks from `scripts/save_map.sh`.

### Map catalog and selection

`MapState.catalog` lists every discovered bundle with `complete`, `revision`,
`session_id`, raster metadata and, for incomplete bundles, the reason.
Completeness re-validates cheaply on each publish (existence + non-empty +
YAML parse + raster header + manifest hash match); it is not a localization
quality test. SELECT (`MapRequest.OP_SELECT_MAP`) requires a verified complete
bundle, is rejected during a runtime transition and never hot-swaps the map
under a live AUTONOMY runtime; the selection is persisted and reported as
`selected_map_requested` / `selected_map_applied` / `selected_map_reason` (the
minimal Stage 4 operational-config surface — speed/obstacle-layer settings are
deliberately deferred to their later stage).

### Web backend

`ros_state_node` subscribes `/paddock/map_state` and the extended `ModeState`;
`state_cache` exposes `map_state` in the state snapshot for the future UI.

### Validation

- `colcon build` (runner_interfaces, runner_paddock) clean.
- 106 runner_paddock package tests pass, including new
  `test_map_session.py` (safe names, catalog completeness/hash-mismatch,
  transactional publish, empty-artifact rejection, session-id-change discards
  prior state, stale evidence drops readiness, idempotent save cache) and
  `test_mode_runtime.py` additions (epoch increments per start, session id
  lifecycle, NEW MAP restart ordering, capability gating + reason surfacing).
- Live, on the running Pi with traction disconnected, driven through the
  supervisor via a lease + typed requests:
  - MAPPING came up and reached `ready: true` (`runtime_epoch 1`,
    `mapping_session_id 1-…`).
  - NEW MAP produced `2-…` with a real SLAM restart; the previously saved
    bundle was preserved.
  - SAVE MAP produced a complete `smoke1` bundle from live SLAM + `/map`;
    manifest SHA-256s matched `sha256sum`; `.staging/` emptied on the atomic
    move; the catalog exposed it only at phase `SAVED`.
  - SELECT accepted the verified bundle and rejected a bogus name with a
    reason.
  - The persistent local-control / STOP / command-authority PIDs were
    unchanged across every MAPPING/AUTONOMY/NEW MAP transition.
  - Final state: mode units inactive, STOP asserted (stopped+locked+applied),
    `/cmd_vel` zero, traction PWM 0 ns, steering 1500000 ns. Test bundle and
    scratch processes removed.
- Silent-failure checks (cheap, direct): duplicate live ownership — persistent
  tier stayed singular and old owners exited before replacements; stale session
  — a new epoch differs and old-session `/map` cannot satisfy new-session
  readiness; save bundle — required artifacts present and non-empty, YAML
  references the written raster, manifest hashes verified, catalog exposes only
  the completed bundle.

### Hardware/integration validation pending

- AUTONOMY runtime start with a verified selected map, and the isolated
  deserialize/load smoke of a newly saved bundle, were not exercised to
  completion live (the offline test harness could not hold a command-authority
  lease long enough to drive the transition, and this shell cannot restart the
  persistent operator services to redeploy consistently). Covered by source and
  unit tests; needs a hands-on run.
- The persistent `runner-command-authority`, `runner-stop-enforcer`,
  `runner-local-control` and `runner-paddock-web` services are still running
  pre-Stage-4 code with the previous `ModeState` type; a full service restart
  or reboot is required for a consistent runtime. `runner-map-executor.service`
  is added but not installed under `/etc/systemd/system` (needs root).
- Autonomous motion remains intentionally disconnected from the mux; the legacy
  autonomy mux input was not reintroduced.

## Stage 5 — one real navigation/mission runtime

Backend only. No mux change, no hold-to-RUN motion authorization, no frontend.
Traction stays physically disconnected; this stage does not move the vehicle.

### Single Nav2 mission owner

New node `runner_navigation_runtime` (`runner_bringup`, application-tier, started
by `nav2.launch.py`) is the *only* component that owns Nav2 mission action
clients. It holds one `NavigateToPose` and one `NavigateThroughPoses` client —
one mission execution component, not one client. The retired
`foxglove_goal_bridge` node, its `/move_base_simple/goal`, `/runner/waypoint`,
`/runner/route_control` and `/teleop/keyboard_state` ingress, its entry point and
its test are removed in the same change; its generation-tracking,
cancellation-retry and delayed-callback protection are refactored into the new
runtime. `mode_runtime.AUTONOMY_ONLY_NODES` now expects
`/runner_navigation_runtime` instead of `/foxglove_goal_bridge`.
`keyboard_bridge` still publishes `/runner/route_control`; that topic simply has
no production consumer now (its retirement is the keyboard cutover's job).

### Interfaces

- `runner_interfaces/NavigationRequest` — authorized `OP_SELECT` / `OP_DISPATCH`
  / `OP_CANCEL` with a real map-frame `geometry_msgs/PoseStamped[]`, immutable
  `mission_id`, monotonic `mission_revision`, bound `runtime_epoch` and `map_id`.
  Sole writer: the command authority.
- `runner_interfaces/NavigationState` — truthful lifecycle
  `IDLE / DISPATCHING / ACTIVE / CANCELING / SUCCEEDED / FAILED / CANCELED`, plus
  process `boot_id`, `mission_valid`, `action_generation`, `goal_uuid`,
  `nav2_status`, `error_code`. Sole writer: the navigation runtime. `ACTIVE` is
  reported only after Nav2 has actually accepted a goal handle — never on
  dispatch intent.
- `PaddockControlEvent` gains `goal_frame` / `goal_x` / `goal_y` / `goal_yaw`;
  `EVENT_GOAL_SELECTED` now carries the operator's real pose instead of the
  previous fabricated `(0, 0, 0)` `GoalIntent`. Non-finite goal poses are
  rejected.

### Generation / epoch invalidation

Separate identities: process `boot_id`, `runtime_epoch` (from `ModeState`),
logical `mission_revision`, and `action_generation`. Every asynchronous Nav2
callback (`on_goal_response`, feedback, `on_result`, cancel response) is checked
against the live `action_generation` *and* the live goal handle; a stale one is
dropped and can never overwrite current mission state or reopen a grant. A
`mission_revision` that is not strictly newer cannot rebind the mission. On a
`runtime_epoch`/map change or on the runtime leaving AUTONOMY, the logical
mission is invalidated and any in-flight action is canceled. On restart the new
`boot_id` makes any previously reported state stale, and the first dispatch
after boot issues a best-effort `CancelGoal` to both action servers before
sending, so an orphaned server goal is never adopted.

### Authority wiring

The command authority publishes `/paddock/navigation_request`: `OP_SELECT` when
it accepts a new goal intent (fresh `mission_id`, bumped `mission_revision`),
`OP_DISPATCH` on the dispatch-intent edge, `OP_CANCEL` on RUN release, STOP,
DualSense takeover, lease loss, mode/epoch change or goal clearing. It subscribes
`/paddock/navigation_state` and now derives `CommandAuthorityState.autonomy_
action_active` from the runtime's real Nav2 lifecycle instead of the reducer's
optimistic `navigation_active` flag. RUN-release keeps the logical mission as a
deliberate-continuation candidate (Q2); epoch/map change and leaving AUTONOMY do
not.

### Validation

- `colcon build` clean for `runner_interfaces`, `runner_paddock`,
  `runner_bringup`.
- Package tests: `runner_paddock` 106 passed; `runner_bringup` 56 passed, 1
  skipped, including new `test_navigation_runtime.py` (pose validation,
  select/dispatch/cancel lifecycle, stale-generation result/acceptance guard,
  stale `mission_revision` guard, epoch-change and leaving-AUTONOMY
  invalidation, cancel-before-delivery).
- Live software smoke (isolated `ROS_DOMAIN_ID`, no Nav2): the node starts and
  publishes truthful `IDLE`; a `SELECT` binds `mission_valid` with the real
  pose and stays `IDLE`; `DISPATCH` with no Nav2 server present holds at
  `DISPATCHING` (never a fake `ACTIVE`); `CANCEL` reaches `CANCELED` with the
  logical mission retained; a new `runtime_epoch` invalidates the mission.
- Final system left safe: no navigation runtime process left running; live
  `/paddock/mode_state` back to `IDLE/STABLE`, authority `NONE`,
  `brake_intent: true`; traction disconnected throughout.

### Hardware/integration validation pending (Stage 5)

- A real Nav2 goal end-to-end (`SELECT → DISPATCH → NAV2_ACCEPTED → EXECUTING →
  terminal`) and a real cancel against a live `bt_navigator` were not run: it
  needs the managed AUTONOMY unit up with a verified map plus a held
  command-authority lease, and the persistent operator services on this Pi are
  still running pre-Stage-4/5 code. Covered by source and the isolated unit
  tests; needs a hands-on run after a service restart/redeploy.
- The delayed-old-`Twist`-across-cancellation quiescence acceptance (spec §10)
  belongs to the Stage 7 controller-boundary cutover; Stage 5 owns mission
  lifecycle only and the converter/mux seam is untouched.
- Autonomous velocity remains intentionally disconnected from the mux; the
  legacy `/cmd_vel_auto` mux input was not reintroduced.
