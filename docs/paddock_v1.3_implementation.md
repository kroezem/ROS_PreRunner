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
  a distinct operation within an active MAPPING session: the executor forwards
  it to the supervisor, whose TRANSITIONING state revokes browser/manual motion
  through command authority. It waits for that epoch-bound revocation and a
  subsequent `EncoderState.stationary=true` sample before stopping the mapping
  application, verifies the old owners are gone, starts it again and allocates a
  new session epoch. It neither requires nor changes global STOP. Saved bundles
  are never deleted. The executor
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

## Stage 7 — autonomy authority / velocity cutover (Part A)

The final v1.3 autonomous command path is now wired end to end:

```
Nav2 -> /cmd_vel_nav -> drive_adapter -> /cmd_vel_auto_raw
     -> command_authority -> /cmd_vel_auto -> existing twist_mux -> /cmd_vel -> motor
```

### Ownership / topology

- `drive_adapter` is the **sole writer of `/cmd_vel_auto_raw`** (was
  `/cmd_vel_auto`). No retuning: only the publisher topic changed. It stays a
  persistent-process diagnostic writer, active only in the AUTONOMY
  application.
- `command_authority` is the **sole writer of the supervised `/cmd_vel_auto`**
  (was the private `/paddock/private/cmd_vel_auto`). It forwards a raw sample
  only while every current-state interlock holds and otherwise emits a bounded
  0.30 s brake transition on `/cmd_vel_auto`, then goes silent.
- `twist_mux` gains exactly one autonomy input: `topic: /cmd_vel_auto`,
  `timeout: 0.30`, `priority: 50`. Ordering is
  `autonomy 50 < teleop 100 < stop-lock 200 < stop-zero 255`, so local
  DualSense teleop and the global STOP always override autonomy. Still one
  mux, still the sole `/cmd_vel` writer. No second mux, no adapter→mux bypass.

### Autonomy permit conditions (fail closed on stale/unknown)

Supervised output on `/cmd_vel_auto` requires **all** of:

- runtime actually `AUTONOMY`, `STATUS_STABLE` **and** `ModeState.ready`
  (a stable-but-not-ready runtime is treated as ineligible);
- current runtime epoch / active map identity match (goal is bound to the
  active map; a map/epoch change clears the goal and RUN);
- fresh Paddock control lease (`lease_timeout_sec`, deployed at 0.5 s as the
  browser-disconnect backstop for a held RUN — a first-integration Wi-Fi
  value to be measured and tightened before traction);
- STOP state fresh **and** clear (`_stop_clear`; a stale `/paddock/stop_state`
  fails closed);
- local DualSense not active and no unresolved takeover
  (`run_blocked_until_release`);
- autonomous RUN permission current (`run_held`; hold-to-run, see below);
- raw converted input fresh (`raw_autonomy_timeout_sec`, 0.15 s at the 20 Hz
  adapter cadence);
- mission lifecycle compatible with motion: a real, current, accepted /
  executing Nav2 action (`NavigationState.STATE_ACTIVE`, fresh). `DISPATCHING`,
  `CANCELING`, terminal, or a stale navigation state forbid output.

Precedence preserved: **STOP > DualSense > Paddock autonomous.**

### RUN semantics — hold-to-run

- RUN press (`EVENT_RUN_PRESSED`) under a fresh lease + ready AUTONOMY + a
  validated goal + neutral local input establishes RUN and dispatches through
  the current generation/epoch (`OP_DISPATCH`).
- RUN release (`EVENT_RUN_RELEASED`) revokes motion immediately (bounded brake
  then silent) **and** requests a Nav2 cancel via the Stage-5 runtime
  (`OP_CANCEL`). The logical mission is retained as a deliberate-continuation
  candidate (Q2); a **new** deliberate RUN edge is required to redispatch.
- Releasing DualSense never auto-resumes: `run_blocked_until_release` holds
  until a fresh release→press after the local takeover clears.
- RUN is also revoked by: browser disconnect / stale heartbeat (lease expiry),
  lease loss, STOP, runtime/map/epoch change, DualSense takeover, readiness
  loss. Reconnect never resurrects RUN, an old goal dispatch, or a STOP clear —
  the authority restarts with no lease, RUN, goal or raw sample.

No fake Nav2 pause: RUN release issues a real cancel; Nav2 may time out while
gated stationary, which is why the logical mission (not the action) is what
survives.

### Validation

- `colcon build` clean for `runner_paddock`, `runner_drive_adapter`,
  `runner_bringup` (`runner_interfaces` unchanged; its pre-existing stale
  build-symlink is untouched).
- Package tests: `runner_paddock` 106 passed (updated authority topic
  contract); `runner_drive_adapter` 110 passed (raw-topic rename);
  `runner_bringup` 56 passed / 1 skipped, including the live-`twist_mux`
  subprocess test rewritten as
  `test_runtime_autonomy_input_is_below_local_teleop_and_stop` (teleop 100
  overrides autonomy 50; local release lets the supervised autonomy input
  drive; stop-lock 200 and stop-zero 255 both mask it).
- Structural smoke: `command_authority` starts and logs the
  `/cmd_vel_auto_raw -> /cmd_vel_auto -> twist_mux` path; with no raw input
  (IDLE) it stays silent (`brake_intent`, no armed brake window).

### Hardware / integration validation pending

- A real held-RUN autonomous drive, DualSense takeover mid-mission, RUN-release
  cancel, and global STOP during motion are **Matti's integration test** — see
  the checklist. No traction was used.
- End-to-end stale-`/cmd_vel` duration and the delayed-old-`Twist`-across-
  cancellation quiescence acceptance (spec §10/§11) are measured on hardware;
  not attempted here.
- The persistent Pi services were still running pre-Stage-7 binaries at the
  start of this pass; a coherent redeploy is required (Part C).

## Stage 8 (partial) — minimal Paddock operator UI (Part B)

Not the polished final UI: functionality over appearance, enough for Matti to
exercise the Stage 7 backend from a tablet.

### Backend: browser intent gateway

`runner-paddock-web.service` is no longer read-only. The paddock ROS node
(`RosStateNode`, unchanged name) now also owns three operator-intent writers
and is the **sole browser-side writer** of each:

- `/paddock/control_event` — lease acquire/release, RUN pressed/released,
  STOP, CLEAR STOP, goal selected, heartbeat;
- `/paddock/mode_request` — runtime selection (IDLE / MAPPING / AUTONOMY),
  `request_id = time.time_ns()` (globally monotonic against the map
  executor's own NEW MAP `ModeRequest`s);
- `/paddock/map_request` — NEW / SAVE (session id filled from `ModeState`) /
  SELECT map.

New pure module `runner_paddock/gateway.py` (`OperatorGateway`, ROS- and
clock-free): one control lease at a time (one *controller*, N *observers*);
per-gateway monotonic `sequence`; hold-to-run edge tracking; every intent is
produced **only** in direct response to a fresh browser message — the gateway
manufactures no renewals, so a silent browser lets the Pi-side lease expire
and RUN is revoked. `on_disconnect` always releases the lease (publishes
`EVENT_LEASE_RELEASED`); a reconnecting browser starts with no lease, no RUN
latch and no goal.

New read subscriptions feeding the stream: `/paddock/control_lease`,
`/paddock/stop_state`, `/teleop/control_state`, plus the STOP fields already
on `CommandAuthorityState`. `StateCache` gains `control_lease`, `stop_state`,
`local_control` and `gateway` sources.

The `/ws` endpoint is now bidirectional: a state-sender task and an
action-receiver task run concurrently; each browser action gets an `ack`
frame (`accepted` / `reason` / `role`). `ros_runtime.submit` /
`.disconnect` run off the event loop.

### Frontend

One page, three cards (Global / Mapping / Autonomy) plus a raw backend-truth
dump. Global: backend/connection, lease role + freshness, runtime + phase +
readiness reason, authority + selected source, concise inhibit reason, STOP
state, big STOP + deliberate CLEAR STOP. Mapping: enter MAPPING, session
id/phase, NEW MAP, SAVE MAP with basename + result, saved-map catalog with
select + clear "◀ selected" marker. Autonomy: enter AUTONOMY / return IDLE,
active map, mission + Nav2 lifecycle (DISPATCHING / ACTIVE / CANCELING /
SUCCEEDED / FAILED / CANCELED), x/y/yaw goal input (robot pose shown as a
reference — native map-click deferred, not started), big HOLD TO RUN
(pointer-down repeats at 10 Hz, any release path — pointer up/leave, window
blur, tab hide — sends RUN released), CANCEL. All mutating controls are
disabled for an observer. The page renders backend truth only; on reconnect
it re-acquires from scratch and shows a "RUN / goal / STOP-clear are NOT
retained" banner.

### Validation

- `runner_paddock` package tests: 115 passed, including new `test_gateway.py`
  (single-lease/observer, monotonic lease-scoped sequence, hold-to-run edges,
  STOP clears the local run latch, disconnect releases + clean reconnect,
  goal/map/mode intent shapes, bad-action rejection) and rewritten
  `test_web_app.py` (bidirectional `/ws` round-trip, controller vs observer
  acks, malformed-JSON action rejected but non-fatal, both connections
  disconnect-released).
- Live software smoke (isolated `ROS_DOMAIN_ID`, no other nodes): `GET /` 200;
  `acquire` → controller; `heartbeat`, RUN press/hold/release edges, goal
  select and an unknown action all return the expected `ack`; the gateway
  node published `/paddock/control_event` etc. without error.

### Pending

- Real browser-in-hand exercise against the live managed stack (needs the
  Part C redeploy). Native map rendering / click-to-goal is deliberately
  deferred.

## Deployment coherence (Part C)

`services/install.sh` is now the single source of truth for the systemd
layout: `--check` reports drift without changing anything, no-arg applies the
layout (symlink every tracked unit, install `setup-runner-pwm` and the polkit
rule, `daemon-reload`, `enable` the persistent tier but never the two
`runner-mode-*` units), `--restart` cycles the operator/application tier in
dependency order.

Drift found on the Pi at the start of this pass (`install.sh --check`):

- `runner-command-authority.service`, `runner-stop-enforcer.service` and
  `runner-pwm-setup.service` were hand-**copied** into `/etc/systemd/system`,
  not symlinked — so repo edits (including the Stage 7
  `lease_timeout_sec:=0.5`) never reached the running units.
- `runner-map-executor.service` (Stage 4) was **never installed** — the
  Mapping UI's NEW / SAVE / SELECT map path has no backend until it is.
- `runner-pwm-setup.service` and its `/usr/local/sbin/setup-runner-pwm` script
  existed only on disk; both are now tracked in `services/`.
- The persistent services were running pre-Stage-4/5/7 code.

Repo changes: added `services/install.sh`, `services/runner-pwm-setup.service`,
`services/setup-runner-pwm`; `chmod 644` on the two mode-600 unit files so they
symlink cleanly; `services/README.md` install section rewritten around the
script and now lists `runner-stop-enforcer` and `runner-map-executor`.

No systemd changes were applied: this environment has no passwordless sudo and
`/etc/polkit-1/rules.d` is not readable unprivileged. The exact operator
commands are in the report's DEPLOYMENT section.

- `runner_interfaces` had a stale `build/` symlink-vs-directory conflict that
  broke `colcon build`; cleared (`rm -rf build/runner_interfaces` + rebuild).
  A clean 4-package build now passes.

### Post-deploy fixes — stale console shell and stale mode_state

Two concrete defects surfaced once the coherent stack was live; both are
read-only diagnoses with a minimal fix, no Paddock redesign.

- **Operator console showed no structured state, header stuck at
  "Connecting…".** `/ws` was delivering fresh `state` frames (the raw
  backend-truth dump updated) but every card stayed at `—`. Cause: the browser
  was running the pre-Stage-8 `app.js` (which targets a `#connection` node that
  no longer exists — its `open` handler throws, so it only ever writes the raw
  `#debug` dump) against the new `index.html`. The stale asset came from
  heuristic HTTP caching of `/static/app.js` (served with validators but no
  `Cache-Control`), which the in-scope service worker's network-first `fetch`
  also honours. Fix: `web_app` now sets `Cache-Control: no-cache` on `/` and
  `/static/*` (ETag/Last-Modified 304s still apply); `service-worker.js` cache
  bumped to `-v2` with a network (`cache: 'reload'`) precache and
  `skipWaiting` / `clients.claim` so a redeploy self-heals. The current
  `app.js` renders correctly against the current `index.html` (verified
  headless: load → open → `state` frame populates every card and the banner).

- **`/paddock/mode_state` aged without bound in steady state** (~72 s stale
  while authority / STOP / local-control / map-state were fresh). Cause: the
  mode supervisor only published `ModeState` on a lifecycle/readiness *change*;
  its 0.5 s refresh timer called `ModeRuntime.refresh()` but that too only
  publishes on change, so a stable IDLE runtime never re-published. Fix:
  `ModeSupervisorNode._on_refresh_timer` now re-publishes the current runtime
  state every tick, giving `mode_state` the same 0.5 s liveness guarantee as
  `map_session_node` already gives `map_state`. `ModeRuntime` publish-on-change
  semantics (and its tests) are unchanged.

Validation: `runner_paddock` build clean; 115 package tests pass; the
`no-cache` headers verified on `/`, `/static/app.js`, `/static/style.css`,
`/static/service-worker.js` and absent on other routes. Live end-to-end
confirmation (console rendering in a real browser, `mode_state` staying fresh
in IDLE) needs the operator to restart `runner-paddock-web` and
`runner-mode-supervisor` — this shell cannot (polkit only grants `matti`
start/stop of the two `runner-mode-*` units). Not started: broader integration
validation.
