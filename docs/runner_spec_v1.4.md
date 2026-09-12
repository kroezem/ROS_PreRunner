# Runner — Architecture & Current-State Specification v1.4

**Current-state specification · 12 September 2026**

Baseline: `/home/matti/runner_ws`, clean HEAD `044b4fb565eba71a70fdc56bdd790bd9da068986`
("Quiesce active mapping resets without global STOP"), inspected directly from
source, tests, configuration and systemd units. Previous specification:
`docs/runner_spec_v1.3.md` (ratified 7 September 2026 against baseline
`6a7c9611`), reconciled here against a workspace that has since implemented
stages 0–8 of that migration plan and gone materially beyond it (live
tuning, obstacle-layer controls, recording, map deletion, Wi-Fi field AP).

**Reading convention.** This document states what is implemented in source
and covered by tests as **current**, distinguishes what is deployed but not
yet hardware-validated or time-bounded as **provisional/open**, and reserves
**future** for capability that does not exist yet. It does not restate v1.3's
migration narrative except where §19 explicitly reconciles against it. Where
current code or a decision Matti has ratified supersedes a v1.3 numeric limit
or claim, this document states the current value and does not carry the old
one forward as if still binding. Historical D-numbers are always qualified by
source version and subject; no new D-number is allocated here. **Deploy
coherence is not assumed**: this document describes the repository at HEAD,
not necessarily the code the Pi's systemd units are currently executing. A
recent validation found `runner-map-executor` serving an older message
schema on the Pi; see §15 and §17.

## 1. Executive architecture summary

Paddock (`runner_paddock`, served by `runner-paddock-web.service` on
`0.0.0.0:8000`) is the delivered primary operator interface for runtime
selection, manual driving, supervised autonomous missions, STOP, mapping
sessions, complete map bundles (including deletion), live autonomy speed
tuning, obstacle-layer control, MCAP recording, and initial-pose seeding. The
browser holds at most one control lease; a persistent Pi-side command
authority (`runner_command_authority`) validates every intent against
continuously refreshed interlocks and grants bounded motion permission.
Neither the browser nor the authority can reach the actuator directly.

Runtime and motion authority remain independent dimensions:

- **Runtime** (`ModeState.mode`): `IDLE / MAPPING / AUTONOMY`, with a
  separate lifecycle `STATUS_STABLE / STATUS_TRANSITIONING / STATUS_FAULT`,
  a monotonic `runtime_epoch`, a `mapping_session_id`, and continuously
  refreshed `ready` / `readiness_reason`.
- **Motion authority** (`CommandAuthorityState.authority`):
  `AUTHORITY_NONE / AUTHORITY_DUALSENSE / AUTHORITY_PADDOCK_MANUAL /
  AUTHORITY_PADDOCK_AUTONOMY`, derived purely from lease, mode, DualSense
  presence, RUN, and goal state — never asserted independently of them
  (`runner_paddock/state_machine.py:112-124`).

One `twist_mux` remains the final arbiter and sole `/cmd_vel` writer, with
priorities **STOP (255/lock 200) > DualSense teleop (100) > Paddock manual
(75) > Paddock autonomy (50)**
(`src/runner_bringup/config/twist_mux.yaml`). Inactive sources are silent; a
selected zero is a real braking command; STOP is a latched, durably
persisted global inhibit with explicit clear semantics.

Reverse autonomy, obstacle-aware costmaps, a 7-state mission lifecycle, live
speed-policy tuning (Timid/Confident/Custom), complete map bundle lifecycle
(new/save/select/delete), MCAP recording, and a field Wi-Fi AP are all real,
tested, current behavior — not proposals. The known open items are:
end-to-end stale-command timing is still not formally budgeted (§6, §17);
the 0.5 s motion deadman is a deployed but provisional value, not a ratified
final bound; and Pi deploy coherence must be reverified after any change
(services/install.sh --check`, §15).

## 2. Platform, geometry, hardware and safety boundaries

Runner is the LaTrax Prerunner research platform: Raspberry Pi 5, Ubuntu
24.04, ROS 2 Jazzy, LD19 lidar, BNO085 IMU, hall-effect wheel encoder, and a
Cytron MD13S motor driver. Phase 1 (indoor/outdoor navigation) remains the
established platform focus; this document does not change platform purpose.

Geometry is unchanged from v1.2/v1.3: wheelbase 0.178 m, maximum steering
0.3614 rad, physical minimum turning radius 0.470 m. The planner's minimum
turning radius is **0.60 m**
(`src/runner_bringup/config/nav2_params.yaml:28`, `SmacPlannerHybrid.
minimum_turning_radius`) — a planning-time conservatism margin over the
physical minimum, not a claim that the vehicle cannot turn tighter.

| Resource / boundary | Current owner and rule |
|---|---|
| Motor effort PWM (GPIO12, 20 kHz), steering PWM (GPIO13, 50 Hz), direction GPIO23 | `runner-motor.service`, sole continuous owner, `pinctrl-rp1` GPIO chip by label; writes sysfs directly, never unexports either PWM channel |
| Encoder GPIO22 | `runner-encoder.service`, sole continuous owner; `/wheel/encoder_state` feeds the motor reversal gate and every consumer's stationarity/direction evidence |
| Motor watchdog | 200 ms `/cmd_vel` staleness → zero duty (active brake), checked every 50 ms; unchanged, no supervisor or browser component replaces it |
| Direction/reversal gate | Motor-local; a negative demand means reverse, never brake; requires a fresh post-request stationary encoder sample before flipping DIR; no upstream component owns this permission |
| LD19 UART (`/dev/ttyAMA0`) | LD19 driver process, application sensor tier |
| BNO085 UART/reset (`/dev/ttyAMA2`) | IMU process, application sensor tier |
| `odom → base_link` | `ekf_node` only; RF2O (`/odom_rf2o`) is an EKF input, never this TF owner |
| `map → odom` | Exactly one `slam_toolbox` instance: mapping SLAM in MAPPING, localization SLAM (remapped `/slam_map`) in AUTONOMY |
| Static extrinsics | Existing static TF publishers (`base_link_to_base_laser`, `base_link_to_imu_link`) |
| `/map` | MAPPING: `slam_toolbox`. AUTONOMY: `map_server`. Enforced per-topic by `runner_mode_supervisor._ownership_ready` (`mode_supervisor_node.py:225-245`), not by node-name counting |
| `/cmd_vel` | `twist_mux`, sole writer, unchanged |

TF is a multi-publisher transport with **single ownership per edge**, not a
single-writer topic; the mode supervisor checks publisher-owner sets per
critical topic/edge every readiness cycle and ignores endpoints whose node
identity has not yet propagated through DDS discovery
(`mode_supervisor_node.py:211-223`, carrying forward v1.3 §12's ruling that a
duplicate node *name* is not proof of a duplicate owner).

Retained hard safety limitations, unchanged by anything in this document:

- The planar LD19 scan cannot see descending edges or low obstacles;
  supervised autonomous/manual driving is not gated on a negative-obstacle
  sensor.
- Paddock/global STOP is a software motion inhibit, not proof of mechanical
  stationarity or independent power isolation.
- The known motor SIGKILL/PWM-peripheral hazard and deferred
  heartbeat-gated-FET decision (v1.2 subject, historically discussed as
  D-82) remain open; nothing in this architecture claims to close them.
  `runner_stop_enforcer` controls the existing mux/lock only.

## 3. Process tiers and ownership model

**Hardware tier (persistent, unmanaged by Paddock):** `runner-pwm-setup`
(oneshot exporter), `runner-motor`, `runner-encoder`, `runner-battery`,
`runner-telemetry`, `runner-foxglove`. Paddock has no privilege to start,
stop, or restart any of these; it can only *request* work from the tiers
below.

**Persistent local-control tier** (`runner-local-control.service`, launched
via `runner_bringup/launch/teleop.launch.py`): one `joy_node`,
`keyboard_bridge` (legacy, see §17), `runner_teleop` (DualSense local
manual/fixed-throttle), and the single `twist_mux`. Alive across
IDLE/MAPPING/AUTONOMY; application launches construct none of these nodes
(`teleop.launch.py:1-5`).

**Persistent STOP tier** (`runner-stop-enforcer.service`,
`runner_stop_enforcer.py`): the durable global-STOP executor, independent of
every other Paddock process.

**Persistent operator tier:** `runner-command-authority.service`
(`runner_command_authority`, lease/RUN/interlock supervision and the sole
supervised writer of `/cmd_vel_auto` and `/cmd_vel_paddock`),
`runner-drive-adapter.service` (`drive_adapter`, the one shared
Nav2/manual-demand longitudinal+steering conversion), `runner-mode-supervisor
.service` (`runner_mode_supervisor`, sole start/stop owner of the two
application units via systemd D-Bus under a narrow polkit rule),
`runner-map-executor.service` (`runner_map_executor`, map-session and bundle
lifecycle), `runner-recording-executor.service` (`runner_recording_executor`,
sole owner of the `ros2 bag record` process), and `runner-paddock-web.service`
(the browser gateway). None of these owns process lifecycle for hardware or
each other except the mode supervisor's narrowly scoped control of the two
fixed mode units.

**Application tier:** `runner-mode-mapping.service` (`map.launch.py`: common
sensor/estimation tier + mapping `slam_toolbox`) and
`runner-mode-autonomy.service` (`autonomy.launch.py`: the same common tier +
localization `slam_toolbox` + `map_server` + Nav2 + `runner_navigation_runtime`
as the sole Nav2 mission/action owner). The two units carry `Conflicts=`,
`KillMode=control-group`, and no `[Install]` section — they cannot be
enabled at boot and are only started/stopped by the mode supervisor
(`services/README.md`).

Ownership summary: authority owns permission, not calculation; mode
supervisor owns application process lifecycle, not permission; map executor
owns bundle files and session bookkeeping, never process lifecycle (it
forwards NEW MAP to the mode supervisor as a typed `ModeRequest`); drive
adapter owns the one shared PI/feedforward conversion, not arbitration; mux
owns final priority selection; motor owns hard actuator safety. Every gate
decision publishes a reason string; there is no bare boolean "armed" flag.

## 4. Paddock operator surfaces and control roles

**Lease model.** `OperatorGateway` (`gateway.py`) holds exactly one control
lease per browser connection at a time; every other connection is a
read-only observer. An intent is produced only in direct response to a fresh
browser message — the gateway manufactures no renewals — so a silent
browser lets the Pi-side lease expire and RUN is revoked
(`gateway.py:16-29`). Disconnect always releases the lease
(`gateway.py:212-222`); a reconnecting browser starts with no lease, no RUN
latch, and no goal.

**Console layout** (`static/index.html`): a `CONTROL` view (map/costmap/plan
canvas, big STOP, mode-specific manual joystick or autonomy run controls,
compact Timid/Confident preset row) and a `CONFIGURE` view with tabs
`RUNTIME / MAPPING / AUTONOMY / DISPLAY / RECORDING / SYSTEM`. All mutating
controls are disabled for an observer connection.

Operator-facing actions accepted by the gateway (`gateway.py:226-604`, one
`_do_*` handler per action): `acquire` / `release` / `heartbeat`, `run`
(hold-to-run), `stop` / `clear_stop`, `manual` (joystick demand),
`select_mode`, `new_map` / `save_map` / `select_map` / `delete_map`,
`select_goal` / `set_initial_pose`, `set_config` (manual speed ceiling),
`set_autonomy_tuning` (preset or field values), `clear_obstacles` /
`set_obstacle_processing`, `start_recording` / `stop_recording` /
`delete_recording`. Every handler validates ownership and finiteness before
producing a typed intent; malformed or unauthorized actions are rejected
with a reason and no intent is published.

**Other surfaces**, unchanged in role from v1.3:

| Surface | Role |
|---|---|
| Paddock | Primary runtime, mapping, manual, mission, RUN/STOP, settings, tuning, obstacle, recording and health interface |
| DualSense | Independent local Bluetooth manual fallback and takeover; highest normal-source mux priority (100) |
| Foxglove | Diagnostic visualization, TF/topic inspection, deeper engineering plots |
| SSH / ROS parameters / `services/install.sh` | Engineering/deploy configuration, not the operator abstraction |
| Laptop keyboard (`keyboard_bridge`) | Legacy local-manual-only input path; its original autonomy-arming purpose is dead code (§17) |

## 5. Runtime modes and lifecycle

`ModeState` (`runner_mode_supervisor.py`, `mode_runtime.py`) publishes
`mode`, `status` (`STATUS_STABLE / STATUS_TRANSITIONING / STATUS_FAULT`),
`accepted_request_id`, `active_autonomy_map`, `detail`, a monotonic
`runtime_epoch` (increments on every successful MAPPING/AUTONOMY start and
on NEW MAP), a per-session `mapping_session_id` (non-empty only in
MAPPING), and continuously refreshed `ready` / `readiness_reason` — a 0.5 s
timer re-evaluates and republishes even a steady IDLE runtime so the topic
never ages without bound (`mode_supervisor_node.py:290-302`).

Readiness is capability-specific and never a bare "process present" check
(`mode_runtime.py:354-390`):

- **Structural**: the mode unit is `active`/`running`; every required node
  is present (common tier + persistent-local tier, plus
  `AUTONOMY_ONLY_NODES` in AUTONOMY and their absence in MAPPING); every
  critical topic's publisher-owner set matches exactly.
- **MAPPING capability**: `map→odom` and `odom→base_link` TF usable,
  `/scan_slam` fresh within 1.5 s, a `/map` update seen *after* the current
  session began (retained old-session raster never satisfies this),
  `/map` fresh within 15 s (slam_toolbox's 5 s `map_update_interval` with
  generous margin).
- **AUTONOMY capability**: TF usable, `/scan` fresh within 1.5 s, `/map`
  present from `map_server` (a static map need not keep republishing).

A transition (`ModeRuntime.transition`, `mode_runtime.py:567-702`) always
stops both fixed units fully (waiting for empty cgroups and mode-scoped ROS
resources to disappear), validates a requested AUTONOMY map's four-artifact
bundle before starting, starts the target unit, and waits for full readiness
before publishing `STABLE`; any failure tears the half-started graph down
and publishes `IDLE/FAULT` with a concrete blocker string, never a
false-STABLE state. A repeated request for the already-stable runtime with
the same map selection is idempotent and does not restart anything
(`mode_runtime.py:607-619`).

**NEW MAP** (`ModeRequest.OP_NEW_MAP`, only valid with `requested_mode =
MAPPING`) is a distinct, lighter operation: it requires an already-active
stable MAPPING session, and instead of the generic stop/start path it first
arms a quiescence barrier (`begin_quiescence` → `_wait_for_quiescence`,
`mode_runtime.py:463-482`) that only clears once the command authority has
*revoked* browser/manual motion for the current epoch **and** a
*subsequent* `EncoderState.stationary=true` sample has arrived
(`mode_supervisor_node.py:171-209`) — a sample already-fresh before
revocation cannot satisfy it. **This does not require, assert, or clear
global STOP**; STOP is untouched by NEW MAP and stays exactly as it was
(§6). Only after quiescence does it replace the mapping SLAM process and
allocate a new `runtime_epoch`/`mapping_session_id`; saved bundles are never
touched.

## 6. Motion authority, STOP, lease, RUN and takeover

**Authority derivation** (`state_machine.py:112-124`) is a pure function of
`mode`, `lease_active`, `dualsense_active`, `manual_active`, and
`autonomy_permitted` (mode AUTONOMY + lease + `run_held` +
`not run_blocked_until_release` + `not dualsense_active` + a selected
goal) — it cannot be set independently, so the authority snapshot can never
report a contradiction.

**Lease timing is split into two bounds**
(`command_supervisor.py:30-42,138-174`):

- `lease_timeout_sec` — the **RUN / autonomous-motion deadman**: with RUN
  held, no fresh ordered control event for this long revokes autonomous
  motion (a brake). Code default 0.150 s; **deployed value is 0.5 s**
  (`services/runner-command-authority.service`:
  `-p lease_timeout_sec:=0.5`) — this is the "0.5 s motion deadman"
  referenced in this document's provenance notes and remains a **first-
  integration, provisional Wi-Fi value**, not a ratified final safety bound.
- `control_liveness_sec` — a forgiving backstop (default and deployed:
  3.0 s) on bare lease *ownership*: ordinary Wi-Fi jitter no longer drops
  the operator's lease (and the UI's mutating controls) mid-session, while
  the tight motion deadman above still gates actual motion every tick. A
  lease that lapses only the liveness bound but whose same browser keeps
  heartbeating is silently reinstated in place (`LEASE_REINSTATED`,
  `command_supervisor.py:434-457`); RUN does not resume without a fresh
  press.

**Autonomous-motion permit** (`_apply` in `command_authority_node.py:719-780`,
`CommandSupervisor._snapshot`): output on `/cmd_vel_auto` requires **all**
of: runtime actually `AUTONOMY` **and** `STATUS_STABLE` **and**
`ModeState.ready` (a stable-but-not-ready runtime fails closed); current
runtime epoch/map bound to the mission; fresh lease (≤0.5 s deployed); STOP
fresh **and** clear; DualSense not active and no unresolved takeover block;
`run_held` current; raw autonomy input fresh (≤0.15 s at the drive adapter's
20 Hz cadence); and a real, current, `NavigationState.STATE_ACTIVE` Nav2
action, itself fresh (≤1.0 s) — `DISPATCHING`/`CANCELING`/terminal/stale
forbid output. On revoke, a single bounded 0.30 s brake transition is
emitted then the publisher goes silent (`_AutonomyOutputGate`,
`command_authority_node.py:119-156`); this covers one full mux autonomy
timeout (0.30 s) so the deliberate zero is arbitrated before the input ages
out.

**Manual-motion permit**: STOP clear, `authority == AUTHORITY_PADDOCK_MANUAL`,
and manual input fresh (0.25 s) — same bounded-brake-then-silent gate on
`/cmd_vel_paddock`.

**Precedence**, enforced by `twist_mux` priorities (`twist_mux.yaml`):
STOP (255 zero-topic, 200 lock) > DualSense teleop (100) > Paddock manual
(75) > Paddock autonomy (50). Timeouts: autonomy/manual inputs 0.30 s,
teleop 0.15 s, STOP lock 0.15 s, STOP zero-topic 0.10 s.

**Global STOP** (`runner_stop_enforcer.py`) is a boot-qualified,
idempotent, durably persisted (fsync'd JSON + directory fsync) state machine
independent of every other Paddock process:

- Assertion is immediate (lock + zero published) and durability follows
  asynchronously on a worker thread; every new stop request invalidates an
  older pending clear.
- Clear requires: fresh encoder state (≤0.20 s) reporting stationary, fresh
  local-control state (≤0.20 s) reporting released+neutral, and durable
  persistence with no fault. `clear_reason()` surfaces the first blocking
  condition (`ENCODER_STALE` / `NOT_STATIONARY` / `LOCAL_STATUS_STALE` /
  `LOCAL_NOT_NEUTRAL` / persistence pending).
- After a clear is accepted, a 0.40 s drain (`DRAIN_TIME`) keeps the lock
  asserted while old stop-velocity samples age out; if neutrality is lost
  during drain the clear reverts to STOP automatically.
- On restart with a durable, valid `stopped=false` record, the executor
  still re-drains for 0.40 s before reporting clear — a known-clear
  restart never skips the barrier.
- Unknown/corrupt persisted state boots as `STOP_STATE_UNKNOWN`, fails
  closed (stopped=True, unhealthy).
- `StopState.applied` is only true when stopped **and** locked **and**
  durable **and** a genuinely zero final `/cmd_vel` sample has been observed
  within 0.10 s — an "applied" claim is backed by the actual mux output,
  not just internal state.

`EVENT_STOP` and `EVENT_CLEAR_STOP` are ordinary lease-owned control events
in current code — like every other `PaddockControlEvent` except
`LEASE_ACQUIRED`, they are only accepted from the connection currently
holding the lease, with the same ownership and monotonic-sequence checks
(`command_supervisor.py:417-500`); there is no separate "STOP from any
authenticated operator regardless of lease" path implemented today. The
command authority forwards an accepted `EVENT_STOP`/`EVENT_CLEAR_STOP` as a
boot-qualified `StopRequest`, but the enforcer alone decides whether a clear
is actually safe to apply (§6 above).

**Local (DualSense) takeover**: any process/takeover-epoch edge or an
`active=true` sample from `/teleop/control_state` immediately revokes all
remote grants (`command_authority_node.py:609-627`); loss of local-control
status itself (not just an active takeover) also forces `dualsense_active`
closed after 0.10 s of silence (`_on_supervision_timer`,
`command_authority_node.py:638-651`), so a missing local-control publisher
cannot leave stale remote permission live.

## 7. Command/data-flow and exact topic/interface ownership

```mermaid
flowchart LR
  Browser[Paddock browser] -->|WebSocket /ws| Web[runner_paddock web / gateway]
  Web -->|control_event, mode_request, map_request,\nrecording_request, config_request| Authority[command_authority]
  Web -->|map_request OP_NEW_MAP/SAVE/SELECT/DELETE| MapExec[map_executor]
  Web -->|Nav2 param get/set, ClearEntireCostmap| Nav2Params[obstacle-layer + costmap-clear services]
  Web -->|initialpose| SLAM[slam_toolbox]
  Authority -->|mode_request| Supervisor[mode_supervisor]
  Supervisor --> AppUnits[MAPPING or AUTONOMY unit]
  Authority -->|navigation_request select/dispatch/cancel| NavRuntime[navigation_runtime]
  NavRuntime -->|NavigateToPose / NavigateThroughPoses| BT[bt_navigator]
  BT -->|cmd_vel_nav| Adapter[drive_adapter]
  Authority -->|manual_demand| Adapter
  Adapter -->|cmd_vel_auto_raw| Authority
  Adapter -->|cmd_vel_paddock_manual_raw| Authority
  Authority -->|cmd_vel_auto p50| Mux[twist_mux]
  Authority -->|cmd_vel_paddock p75| Mux
  Joy[joy_node] --> Teleop[runner_teleop]
  Teleop -->|cmd_vel_teleop p100| Mux
  Teleop -->|control_state| Authority
  Authority -->|stop_request| Stop[runner_stop_enforcer]
  Stop -->|cmd_vel_stop p255 / stop_lock lock200| Mux
  Mux -->|cmd_vel| Motor[motor_node]
  MapExec -->|mode_request OP_NEW_MAP forward| Supervisor
  RecExec[recording_executor] -->|ros2 bag record| Bags[(bags/*.mcap)]
```

| Topic/interface | Type | Sole writer → consumer(s) | Notes |
|---|---|---|---|
| `/paddock/control_event` | `PaddockControlEvent` | web gateway → authority | RUN/STOP/CLEAR/manual/goal/lease/heartbeat, ordered sequence per lease |
| `/paddock/control_lease` | `PaddockControlLease` | authority → executors/gateway | monotonic `generation` |
| `/paddock/command_authority_state` | `CommandAuthorityState` | authority → mode supervisor, gateway, map executor | full interlock/goal/STOP snapshot |
| `/paddock/mode_request` | `ModeRequest` | web gateway or map executor → mode supervisor | `OP_SELECT_RUNTIME` / `OP_NEW_MAP` |
| `/paddock/mode_state` | `ModeState` | mode supervisor → authority, map executor, gateway, navigation runtime | epoch/session/readiness authoritative |
| `/paddock/map_request` → `/paddock/map_state` | `MapRequest`/`MapState` | web gateway → map executor | `OP_NEW_MAP`/`OP_SAVE_MAP`/`OP_SELECT_MAP`/`OP_DELETE_MAP` |
| `/paddock/navigation_request` → `/paddock/navigation_state` | `NavigationRequest`/`NavigationState` | authority → navigation runtime | `OP_SELECT`/`OP_DISPATCH`/`OP_CANCEL`; 7-state lifecycle |
| `/paddock/recording_request` → `/paddock/recording_state` | `RecordingRequest`/`RecordingState` | web gateway → recording executor | `OP_START`/`OP_STOP`/`OP_DELETE` |
| `/paddock/config_request` → `/paddock/config_state` | `ConfigRequest`/`ConfigState` | web gateway → authority | one field, `manual_max_speed_mps`, revision-checked |
| `/paddock/stop_state`, `/paddock/internal/stop_request` | `StopState`/`StopRequest` | stop enforcer ↔ authority | boot-qualified idempotent requests |
| `/paddock/manual_demand` | `ManualDemand` | authority → drive_adapter | signed m/s + normalized steering, sequenced |
| `/cmd_vel_nav` | `Twist` | `controller_server` → drive_adapter | Nav2's SI output, unchanged |
| `/cmd_vel_auto_raw` | `Twist` | drive_adapter → authority | raw converted autonomy command |
| `/cmd_vel_paddock_manual_raw` | `ConvertedCommand` | drive_adapter → authority | raw converted manual command + provenance |
| `/cmd_vel_auto` (p50), `/cmd_vel_paddock` (p75) | `Twist` | authority → twist_mux | supervised, silent when not permitted |
| `/cmd_vel_teleop` (p100) | `Twist` | runner_teleop → twist_mux | local DualSense/keyboard-fallback command |
| `/cmd_vel_stop` (p255), `/paddock/stop_lock` (lock 200) | `Twist`/`Bool` | stop_enforcer → twist_mux | zero + fail-closed lock heartbeat |
| `/cmd_vel` | `Twist` | twist_mux → motor_node | final command, sole writer |
| `/teleop/control_state` | `LocalControlState` | runner_teleop → authority, stop_enforcer, gateway | process/takeover epoch, neutral/released |
| `/wheel/encoder_state` | `EncoderState` | encoder → motor, adapter, EKF, stop_enforcer, mode supervisor | stationary/direction evidence |
| `/drive_adapter/state`, `/drive_adapter/state_typed` | `String`/`AdapterState` | drive_adapter → gateway/diagnostics | full PI/feedforward/integrator diagnostics |
| `/initialpose` | `PoseWithCovarianceStamped` | Paddock web (controlled) → slam_toolbox | STOP+stationary+matching-map gated |

Every raw/supervised command schema carries finite values and identity
fields sufficient to reject stale or cross-epoch delivery (sequence,
`lease_generation`, `runtime_epoch`, and for navigation, `mission_id` /
`mission_revision` / `action_generation`). Command QoS is volatile depth-10
or depth-1 TRANSIENT_LOCAL for status; nothing here is a general pub/sub
free-for-all — each topic above has exactly one writer at any time.

## 8. Mapping sessions, map bundles, NEW/SAVE/SELECT/DELETE

A **mapping session** (`MappingSession`, `map_session.py:525-550`) is
identified by `mapping_session_id` and tracks phase
`NONE→STARTING→READY→SAVING→SAVED` (or `FAILED`), whether it is `unsaved`,
and current-session `/map` evidence freshness (8 s timeout,
`MappingSession.ready`). A fresh session begins on every MAPPING start and
every NEW MAP; a session id change discards *all* prior state keyed to the
old id (`MapSessionModel.observe_runtime`, `map_session.py:589-606`).

**NEW MAP**: see §5 — quiescence via authority revocation + fresh
post-revocation stationary encoder evidence, no STOP interaction.

**SAVE MAP** (`OP_SAVE_MAP`) requires: a current active session matching the
request's `session_id`, the session not `STARTING`/`NONE`, mapping
readiness, and **STOP asserted or the enforcer's lock engaged**
(`map_session_node.py:436-464`, `_stop_inhibited`). It is transactional
(`MapSaveTransaction`, `map_session.py:434-522`):

1. Stage into `maps/.staging/`, refusing to clobber an existing live bundle.
2. Call `slam_toolbox`'s `SerializePoseGraph` into the staging path; verify
   non-empty `.posegraph`/`.data`.
3. Capture the first `/map` `OccupancyGrid` received *after* the save began
   (or a fresh one within 15 s), render a trinary PGM and matching YAML from
   it (nav2 `map_saver` conventions: `occupied_thresh 0.65`,
   `free_thresh 0.196`).
4. Validate the complete four-artifact bundle: non-empty `.posegraph`/
   `.data`, YAML parses with a positive `resolution` and 3-element `origin`,
   the referenced raster exists with a plausible binary-P5 header (positive
   dimensions, `0 < maxval ≤ 255`).
5. Write `<name>.manifest.json` (version, name, session id, creation time,
   a 12-hex-char revision hash, and per-artifact SHA-256 digests).
6. Atomically move the four core artifacts, then the manifest, into
   `maps/` — a reader that observes the manifest has already observed every
   artifact it names. A half-written bundle never leaves `.staging/`.

Retries for the same `request_id` return the cached outcome
(idempotent). The currently committed bundle at HEAD is `maps/studio.{data,
pgm,posegraph,yaml}` — a **legacy bundle saved before the manifest scheme
existed**: it has no `studio.manifest.json`, so its catalog `revision` field
reads empty; this is expected, not a fault.

**Catalog** (`MapState.catalog`, `MapCatalogEntry[]`): every discovered
bundle under `maps/` (not `.staging/`), re-validated cheaply on each publish
(existence, non-empty, YAML parse, raster header, manifest hash match if a
manifest exists). `complete` gates selectability; incomplete bundles carry a
`reason`.

**SELECT MAP** (`OP_SELECT_MAP`) requires a verified complete bundle, is
rejected mid-transition, and never hot-swaps the map under a live AUTONOMY
runtime (`map_session_node.py:382-407`). Selection persists to
`~/.local/state/runner/selected_map` and survives restart
(`_load_selection`/`_store_selection`).

**DELETE MAP** (`OP_DELETE_MAP`, current — not in v1.3) rejects the
currently selected map and the map used by a live AUTONOMY runtime
(`validate_delete_candidate`, `map_session.py:268-279`). Deletion
(`delete_bundle`, `map_session.py:231-265`) stages every artifact into a
private tombstone directory via atomic renames first; if any rename fails,
already-moved files are rolled back before the rejection is returned, so a
partial failure can never leave a half-deleted bundle visible in the
catalog. `MapState.delete_state`/`delete_detail` report the last outcome;
deletion is synchronous, so there is no in-flight state.

## 9. Initial Pose workflow

Current, browser-controlled (`_do_set_initial_pose` in `gateway.py:572-604`,
executed in `ros_state_node.py:1400+`). Preconditions
(`_initial_pose_rejection`): runtime state fresh, mode `AUTONOMY` and
`STATUS_STABLE` and `ready`, and the selected map both **complete** and
**equal to the active autonomy map**. Safety preconditions
(`_initial_pose_safety_rejection`): STOP state fresh and
`stopped ∧ locked ∧ healthy ∧ applied`.

Sequence: the gateway action first issues an `EVENT_STOP` control event
*and* queues the pose intent together in one accepted result
(`gateway.py:594-604`); the node then moves through internal phases
`waiting_stop → awaiting_pose → clearing` (displayed to the operator as
`stopping → localizing → clearing → applied`, `ros_state_node.py:1425-1467`),
publishing `geometry_msgs/PoseWithCovarianceStamped` on `/initialpose` only
once STOP is confirmed applied and stationary, using slam_toolbox's own RViz
`SetInitialPose` planar covariance defaults (x/y variance 0.25 m²,
yaw variance 0.06853891909122467 rad², matching slam_toolbox 2.8.5's
scan-matching-seed semantics). It is confirmed only from a subsequent
map-frame `/pose` sample from slam_toolbox itself — never from the
publish call succeeding.

**Costmap-clear phase.** Once localization is confirmed, the workflow enters
its costmap-clear phase (`_start_initial_pose_clear`, `ros_state_node.py:
1581-1630`): it calls Nav2's `ClearEntireCostmap` service against **both**
the global and local costmap and only reports the transaction `applied`
once both clears succeed (`_finish_initial_pose_clear`,
`ros_state_node.py:1632-1679`) — the recorded detail string is explicit:
*"pose confirmed; global and local costmaps cleared; STOP remains
asserted."* **Global STOP is never cleared by this workflow**: it is
asserted at the start (`EVENT_STOP`) and stays asserted through pose
publication, localization confirmation, and the costmap-clear phase; a
separate, deliberate `CLEAR STOP` is required afterward, and `CLEAR STOP`
is explicitly blocked while an initial-pose transaction is in flight
(`ros_state_node.py:1351-1357`). Timeouts: 5 s to reach STOP, 5 s to
receive pose confirmation, 2 s to complete the costmap-clear phase.

## 10. Localization, estimation and TF ownership

Unchanged in structure from v1.2/v1.3 (D-37 "fixed-cardinality SLAM scan
stream", still current): the LD19 raw `/scan` branches into
`rf2o_scan_canonicalizer → /scan_rf2o → RF2O` (feeding EKF) and
`scan_rebinner → /scan_slam` (feeding slam_toolbox, fixed 503-bin
geometry). `ekf_node` fuses IMU + encoder + RF2O into `odom → base_link`.
`slam_toolbox` owns `map → odom` — the mapping instance in MAPPING, the
localization instance (remapped `/slam_map` diagnostic) in AUTONOMY. Static
extrinsics are unchanged existing publishers. See §2's ownership table for
the authoritative per-edge/per-topic rule and §9 for the current Initial
Pose write path into this localizer.

## 11. Nav2, mission lifecycle and reverse autonomy

**Mission lifecycle** (`NavigationState.STATE_*`,
`runner_bringup/navigation_runtime.py`) is the canonical **7-state**
machine: `IDLE / DISPATCHING / ACTIVE / CANCELING / SUCCEEDED / FAILED /
CANCELED`. `runner_navigation_runtime` is the sole owner of both
`NavigateToPose` and `NavigateThroughPoses` action clients (one execution
component, not two owners); it never reports `ACTIVE` before Nav2 has
actually accepted a goal handle (`on_goal_response`,
`navigation_runtime.py:285-305`).

Generation discipline: `boot_id` (process identity, invalidates prior state
across restart), `runtime_epoch`/`map_id` (bound at `select`, invalidated on
change), `mission_revision` (monotonic per logical mission, rejects stale
rebind), and `action_generation` (monotonic per dispatch attempt — every
async Nav2 callback is checked against it and the live goal handle before
being applied; a stale one is dropped, `is_current`,
`navigation_runtime.py:357-359`). On boot, the first dispatch issues a
best-effort `CancelGoal` against both action servers before sending, so an
orphaned prior-process goal is never adopted
(`CancelResidualGoals`, `navigation_runtime.py:264-266`).

**Cancellation/quiescence invariant** (carried forward from v1.3, not
strengthened beyond what source/tests establish): `cancel()` keeps the
logical mission bound as a deliberate-continuation candidate while
canceling any in-flight action; cancel requests retry every 0.5 s up to 5
attempts, and exhaustion is reported (motion stays revoked, replacement
stays blocked) rather than silently abandoned
(`navigation_runtime.py:676-700`). A dispatch that never reached the Nav2
server before a cancel is resolved locally as `STATUS_CANCELED`
(`_resolve_undelivered_cancel`, `navigation_runtime.py:544-558`) so a late,
unexpected server acceptance of an already-superseded generation cannot
reopen motion. Result/feedback/cancel callbacks are all generation-gated
the same way.

**Reverse autonomy is current and real**, not merely historically ratified:
`SmacPlannerHybrid` plans with `motion_model_for_search: REEDS_SHEPP`
(`nav2_params.yaml:23`, `reverse_penalty 8.0`), and the
`RegulatedPurePursuitController` runs with `allow_reversing: true` and
`use_rotate_to_heading: false` (`nav2_params.yaml:66-67`) — the controller
can and does command negative `linear.x` on `/cmd_vel_nav` when the plan
calls for it. The behavior-tree files are named
`navigate_to_pose_forward_only.xml` / `..._through_poses_forward_only.xml`;
"forward only" names the *recovery-behavior* posture (no explicit
backup/spin recovery actions in the tree — `smooth_path: false` is set
because Jazzy's path smoother over-tightens curvature for this Ackermann
platform), not a restriction on the controller's own reverse capability.
Downstream, the drive adapter's `EncoderState.pending_direction` feedback
path (populated from the motor's own direction line, not a direct
`/motor/direction` subscription) is unchanged from v1.3.

## 12. Longitudinal control and Timid/Confident/Custom speed policy

`drive_adapter` (`runner_drive_adapter`) remains the one shared,
persistent, closed-loop conversion for both Nav2's SI `/cmd_vel_nav` and
authority-bounded manual demand (`/paddock/manual_demand`), publishing raw
`/cmd_vel_auto_raw` and `/cmd_vel_paddock_manual_raw` respectively for the
authority to supervise. "Frozen controller" language from v1.2/v1.3 is
**obsolete**: the committed calibration values below remain the *default*
origin, but a defined subset is now validated, atomically applied, and
live-effective without a process restart
(`add_on_set_parameters_callback`, `drive_adapter_node.py:103,206-233`).

**Live-tunable parameters** (`LIVE_TUNABLE_PARAMETERS`,
`drive_adapter.py:27-34`): `maximum_commanded_speed`,
`feedforward_effort_per_speed`, `feedforward_effort_intercept`,
`output_max`, `proportional_gain`, `integral_gain`. Any other adapter
parameter (wheelbase, `max_steering_angle`, `minimum_moving_speed`,
`integrator_bound`, `output_min`, encoder/wheelspin/timeout thresholds) is
rejected by the parameter callback as not live-tunable and requires a
process restart — these remain engineering-only, launch-time
configuration.

**Timid / Confident are the two normal operator presets**
(`autonomy_tuning.py:91-116`), applied atomically across both owners
(`controller_server` and `drive_adapter`) via `SetParametersAtomically`,
then confirmed by an independent `GetParameters` read-back before being
reported `applied` (`ros_state_node.py:1240-1341`):

| Field | Timid | Confident |
|---|---:|---:|
| `desired_linear_vel` (m/s) | 0.45 | **1.00** |
| `maximum_commanded_speed` (m/s) | 0.60 | **1.00** |
| `regulated_linear_scaling_min_speed` (m/s) | 0.30 | 0.40 |
| `cost_scaling_dist` (m) | 0.45 | 0.60 |
| `cost_scaling_gain` | 1.0 | 1.0 |
| `regulated_linear_scaling_min_radius` (m) | 0.75 | 0.75 |
| `min_lookahead_dist` / `max_lookahead_dist` (m) | 0.30 / 0.80 | 0.30 / 0.80 |
| `lookahead_time` (s) | 1.0 | 1.0 |
| `max_allowed_time_to_collision_up_to_carrot` (s) | 0.15 | 0.60 |
| `proportional_gain` / `integral_gain` | 0.05 / 0.01 | 0.05 / 0.01 |
| `feedforward_effort_per_speed` / `_intercept` | 0.1188 / 0.0174 | 0.1188 / 0.0174 |
| `output_max` | 0.14 | 0.14 |

**Confident's 1.00 m/s intentionally supersedes v1.3's 0.60 m/s autonomy
ceiling** by explicit operator decision — it is not a bug or an
unintended regression of the frozen-controller policy. The characterized
feedforward at 1.00 m/s (0.1188×1.00+0.0174 ≈ 0.136) stays under the
unchanged `output_max` actuator-effort ceiling of 0.14; Confident does not
raise the normalized-effort safety ceiling itself.

Confident's larger `cost_scaling_dist` (0.60 m vs Timid's 0.45 m) and larger
`max_allowed_time_to_collision_up_to_carrot` (0.60 s vs 0.15 s) are **more
conservative near obstacles, not more permissive** — both widen RPP's
safety margin to compensate for the higher target speed:
`costConstraint()` (`regulation_functions.hpp:133-157`) only reduces speed
when `min_distance_to_obstacle < cost_scaling_dist`, and scales the
reduction by `min_distance_to_obstacle / cost_scaling_dist`; a *larger*
`cost_scaling_dist` starts that slowdown farther from an obstacle and
divides by a larger number, so it reduces speed *more*, not less, at any
given standoff distance. Likewise, `isCollisionImminent`'s forward
simulation (`collision_checker.cpp:72-107`) only projects the vehicle's arc
and checks it against the costmap for up to
`max_allowed_time_to_collision_up_to_carrot` seconds; a *larger* value
projects farther into the future (more of the arc, out to the carrot
distance) before accepting a command, catching a potential collision
earlier rather than later. Confident is faster **and** more cautious about
when it starts slowing for an obstacle — it does not trade away obstacle
margin for speed.

**Custom** is not a third preset a user selects — it is `matching_preset()`
(`autonomy_tuning.py:182-189`)'s truthful classification of the live
read-back whenever the ten controller fields plus five adapter fields do
not exactly match either named preset. The operator reaches a genuinely
custom state only through direct field editing, which the UI places under
collapsed **"Advanced speed policy"** and **"Engineering / controller"**
disclosures in `CONFIGURE → AUTONOMY` (`static/index.html:158-186`) — direct
RPP and longitudinal-controller field tuning is Advanced/Engineering
functionality, not the primary operator surface (§16). `validate_values`
(`autonomy_tuning.py:130-179`) enforces cross-field bounds on any custom
write regardless of entry point: positivity, `cost_scaling_gain ≤ 1.0`,
`regulated_linear_scaling_min_speed ≤ desired_linear_vel ≤
maximum_commanded_speed`, `min_lookahead_dist ≤ max_lookahead_dist`,
non-negative feedforward across the command range, and **`output_max` may
never exceed 0.14 and must reach the maximum feedforward implied by the
requested `maximum_commanded_speed`** — the actuator-effort ceiling cannot
be raised through this surface at all.

Manual browser-speed bounds are unchanged from v1.3: ceiling 0 (disabled) or
`[0.25, 0.40]` m/s, default 0.40, moving floor 0.25 m/s
(`command_supervisor.py:37-39`), configurable at runtime via
`manual_max_speed_mps` (`ConfigRequest`/`ConfigState`, revision-checked,
lease-authorized). This single field governs manual driving in both MAPPING
and AUTONOMY; v1.3 §14's separately proposed `mapping_speed_ceiling_mps` was
never implemented as a distinct field — there is exactly one supported
`ConfigRequest.field` name at HEAD, and any other field name is rejected
(`gateway.py:_do_set_config`).

## 13. Obstacle-aware costmaps

Both Nav2 costmaps run a live-toggleable `nav2_costmap_2d::ObstacleLayer`
plus an `InflationLayer`:

- **Local** (`local_costmap`, odom frame, rolling 2×2 m window at 0.025 m
  resolution, `update_frequency 10 Hz`): obstacle layer marks/clears from
  `/scan` (`obstacle_min_range 0.05 m`, `obstacle_max_range 1.0 m`,
  `raytrace_max_range 1.2 m`, heights `0.0–2.0 m`, `inf_is_valid: true`);
  inflation radius 0.45 m, cost-scaling factor 10.0.
- **Global** (`global_costmap`, map frame, static, `update_frequency 5 Hz`,
  0.05 m resolution): `static_layer` (subscribes `/map`, transient-local,
  live updates) plus the same kind of obstacle layer for dynamic
  replanning.

Both plugins' `obstacle_layer.enabled` boolean is a real, live ROS
parameter — Paddock's `set_obstacle_processing` action drives an atomic
set→verify (`GetParameters`/`SetParameters` against
`/global_costmap/global_costmap` and `/local_costmap/local_costmap`,
`ros_state_node.py:915-1092`) and reports `applied`/`rejected`/`drifted` from
the actual read-back, never from the set call's return alone. `Clear
transient obstacles` calls Nav2's own
`clear_entirely_global_costmap`/`clear_entirely_local_costmap` services; it
does not touch the saved static map.

## 14. Recording/observability

`runner_recording_executor` (`recording_executor_node.py`,
`recording.py`) is the single Pi-side owner of the `ros2 bag record --storage
mcap` process, independent of browser connections — closing or reloading
Paddock does not stop an active bag (ownership is reconciled from a runtime
record file across process restart, `_reconcile_runtime_record`,
`recording.py:455-479`). Two profiles: `runner_debug` (a curated ~40-topic
allowlist covering the full command chain, TF, sensors, mode/authority/STOP
state, and diagnostics — `RUNNER_DEBUG_TOPICS`, `recording.py:31-78`) and
`everything` (`--all-topics`). Names are safe basenames
(`SAFE_NAME`, up to 96 chars); an existing output directory is never
overwritten; the currently active bag cannot be deleted. Stop requests a
clean `SIGINT` finalization, escalating to `SIGTERM` at 10 s and `SIGKILL`
at 15 s if rosbag2 does not exit. The catalog (finalized bags under
`bags/`) is cheaply fingerprinted (mtime/size of `metadata.yaml` and the
Paddock manifest sidecar) so it does not reparse every bag on each publish,
and deliberately excludes the growing active bag from that fingerprint so
recording does not itself invalidate the catalog on every write. Paddock
exposes a same-origin `/recordings/{name}/download` endpoint for a
finalized, single-MCAP-file bag only, rejecting anything not in the
authoritative catalog or still active.

## 15. Networking/deployment

**Field Wi-Fi AP and captive portal — implemented capability.**
`network/install.sh` (run once, as root, on a given Pi) creates a
NetworkManager profile `runner-field-ap`: a WPA2-only 2.4 GHz AP named
**Runner-Paddock** (channel 6) at the fixed address `10.42.0.1/24` with
NetworkManager's `shared` IPv4 method providing DHCP/DNS, reachable at
`http://10.42.0.1:8000/` or `http://makro-runner.local:8000/` where mDNS is
supported. The same script installs and (on non-`--check` runs) enables a
separate captive-portal service on port 80 that uses DHCP option 114 and
local DNS answers to surface OS connectivity-check pages with an **OPEN
PADDOCK** link into a normal browser window at port 8000; it does not proxy
or redirect Paddock's API or `/ws` traffic. Both are real, installable,
current source (`network/99-runner-network-manager.yaml`,
`network/runner-captive-dnsmasq.conf`, `network/captive_portal.py`,
`network/runner-captive-portal.service`) — but whether the captive portal
is actually running on any given deployed Pi is a deploy-coherence question
like any other service in §15, not a fact this document asserts; do not
call it "active" without checking `systemctl status
runner-captive-portal.service` on that Pi.

**Current intended operating policy vs. the installer's committed default.**
`network/install.sh` gives `runner-field-ap` `autoconnect-priority 100`
(`install.sh:91`) against NetworkManager's implicit priority 0 for
untouched client profiles, and the script's own output says "the AP will
be preferred at next boot" (`install.sh:130`) — as committed, the AP is the
higher-priority, default-selected profile whenever both it and a client
Wi-Fi profile are present. The **currently intended field policy is the
reverse of that default**: a known/preferred Wi-Fi network first, with
`runner-field-ap` as the fallback only when no preferred network is in
range. Nothing in the repo automates that preference order today — no
script raises a client profile's priority above the AP's 100, and
NetworkManager has no built-in "try Wi-Fi, fall back to AP" behavior for a
single radio (AP mode does not "fail" the way client association does). The
intended policy is therefore enacted operationally, not by the shipped
default: via manual profile priority/switching
(`nmcli connection up '<client profile>'` / `nmcli connection up
runner-field-ap`, `network/README.md`'s "Switching modes"), not by trusting
`install.sh`'s as-committed AP-always priority. Field AP mode replaces
Wi-Fi client mode outright on this single radio (no verified concurrent
AP+STA capability); Tailscale is normally offline while the AP is active
unless an independent uplink (e.g. Ethernet) exists.

**Tailnet path** (unchanged from v1.3): `tailscale serve --bg --https=443
http://127.0.0.1:8000` proxies the same local port at
`https://makro-runner.taila47bfc.ts.net/`, tailnet-only, no Funnel.

**Coherent deploy**: `services/install.sh` is the single source of truth for
the systemd layout — `--check` reports drift without changing anything,
no-arg symlinks every tracked unit (replacing stale hand-copied files),
installs the PWM setup script and the narrowly scoped
`49-runner-mode-units.rules` polkit rule, and enables the persistent tier
(never the two `runner-mode-*` units); `--restart` cycles the operator tier
in dependency order:
`stop-enforcer → command-authority → local-control → drive-adapter →
mode-supervisor → map-executor → recording-executor → paddock-web`, and
deliberately never touches `runner-motor`/`runner-encoder`. **This
document's currency applies to the repository at HEAD, not necessarily to
what the Pi is currently running** — a recent validation observed
`runner-map-executor` on the Pi serving an older `MapState` schema,
confirming that deploy coherence must be re-verified
(`install.sh --check`, then a coherent `--restart` or reboot) after any
change and is never assumed from a green build alone.

## 16. Supported operator vs Advanced/Engineering configuration

**Normal operator surface** (`CONTROL` view and the non-collapsed parts of
`CONFIGURE`): runtime selection (IDLE/MAPPING/AUTONOMY), manual joystick
driving, NEW MAP / SAVE MAP / SELECT MAP / DELETE MAP (delete is
confirmation-gated), Timid/Confident speed presets, global/local obstacle
processing on/off, transient-obstacle costmap clear, initial pose, numeric
goal fallback, MCAP recording start/stop/delete, browser manual-speed
ceiling, and all STOP/CLEAR STOP controls.

**Advanced/Engineering surface**, reached only through collapsed
disclosures in `CONFIGURE → AUTONOMY` (`static/index.html:158-186`):
per-field RPP tuning ("Advanced speed policy": nominal/ceiling/regulated
speeds, cost-scaling distance/gain, curvature radius, lookahead bounds,
collision horizon) and per-field longitudinal-controller tuning
("Engineering / controller": Kp, Ki, feedforward slope/intercept, output
limit). Both write through the same validated, atomic,
read-back-confirmed path as the presets (§12) — the distinction from
"normal" is deliberately about *surface placement and directness*, not
about bypassing validation. SSH/`ros2 param`/direct systemd management
remain purely engineering-only and outside Paddock's authorization model
entirely: geometry, encoder thresholds, motor watchdog, wheelbase/steering
limits, and the drive adapter's non-live-tunable parameters can only be
changed by editing launch/service configuration and restarting the owning
process.

## 17. Known limitations and open engineering work

- **End-to-end stale-command timing remains unbudgeted.** The deployed 0.5 s
  `lease_timeout_sec` motion deadman, the 0.15 s raw-autonomy timeout, the
  0.30 s mux autonomy/manual timeouts, the 0.15 s teleop timeout, and the
  200 ms/50-ms-checked motor watchdog are real, current, measured-in-source
  values, but no document yet states a measured maximum stale-nonzero-
  `/cmd_vel` duration across all of them in series. Treat the 0.5 s deadman
  as **provisional**, not a ratified final safety bound.
- **`keyboard_bridge` is legacy, pending removal, not re-ratified.**
  `keyboard_bridge.py` is still constructed by `teleop.launch.py` inside the
  persistent local-control tier and still implements a 600 s
  (`DEFAULT_AUTONOMY_LATCH_TIMEOUT`) UDP-armed autonomy latch and publishes
  `/runner/route_control` — but nothing in the current graph consumes
  `/runner/route_control` since `foxglove_goal_bridge` was retired in Stage
  5, so the latch's original autonomy-arming purpose is dead code. Its
  `/teleop/keyboard_state` output *is* still live-consumed by
  `runner_teleop` as a genuine local keyboard-driving fallback
  (brake/motion/suppress modes feeding `/cmd_vel_teleop` at mux priority
  100) — that local-driving path works today and is not itself obsolete.
  The obsolete part is specifically the latch/route-control bypass; it
  should be removed rather than treated as a supported production
  interface.
- **Deploy coherence is never assumed** (§15): a Pi that has not run
  `services/install.sh --check` / `--restart` (or rebooted) after a repo
  change may be running stale message schemas or logic.
- **CPU headroom is a measured, open engineering concern, evidenced by the
  `confident` recording (`bags/confident/confident_0.mcap`, a 222 s live
  Confident-preset held-RUN autonomy session with `runner_debug` recording
  active).** Reading `/system/telemetry` directly from that bag:
  `total_cpu_utilization_percent` across all 221 samples was mean 95.5%,
  median 98.7%, with sustained excursions to 100.0% on all four cores by
  the later part of the session (per-core snapshot at start
  `[90.8, 86.6, 90.9, 87.6]`% vs. at end `[100.0, 100.0, 99.0, 100.0]`%);
  `load_average_1min` rose from a first-third mean of 10.65 to a last-third
  mean of 13.65 (max 15.50) — well above the 4-core count, i.e. a growing
  run-queue backlog, not merely a busy CPU. In the same window,
  `/cmd_vel_auto`, `/cmd_vel_nav` and `/cmd_vel` (all three, synchronized to
  within ~0.3 s of each other) show repeated multi-second gaps in the
  recorded command stream — up to **14.05 s** — while
  `/paddock/command_authority_state`, `/drive_adapter/state_typed`,
  `/system/telemetry`, `/tf` and `/scan` show no gap over 3 s in the same
  bag: this is genuine **command-stream starvation** localized to Nav2's
  own velocity output under CPU saturation, not a bag-writer artifact
  (the writer kept up on every other topic throughout). It correlates with
  a lifecycle of repeated `NavigateToPose` aborts in
  `/paddock/navigation_state` — dozens of dispatch→execute→`FAILED(ABORTED)`
  cycles over the 222 s session, almost all `error_code 102` (`TF_ERROR`)
  or `106` (`NO_VALID_CONTROL`, `FollowPath.action` error codes) — a
  concrete, evidenced **goal-approach reliability** problem under load, not
  a hypothetical one.
  Separately, `runnable_processes` did **not** trend upward over the same
  session (first-third mean 15.0 vs. last-third mean 13.5, both well below
  their own transient startup spikes) and no thermal throttling or
  undervoltage flag was ever set (`current_throttled`/`sticky_throttled`/
  `current_undervoltage` all false throughout, temperature 59.0–63.3°C) —
  so this is **not evidence of a process/resource leak**; it is a
  **persistent, material compute cost from running the full Paddock +
  Nav2 + SLAM-localization + active MCAP-recording stack concurrently on
  the Pi 5**, present and load-bearing from early in the session, not one
  that grows without bound. The optimization conclusion — whether recording
  should be lighter by default, whether TF/costmap work should be
  deprioritized relative to the control loop, or whether headroom simply
  needs a hardware/architecture change — is **explicitly left open** by
  this document; only the measurement above is asserted as current fact.
- **RF2O longitudinal profiling is engineering tooling.** The RF2O longitudinal
  profile analyzer (`tools/analyze_longitudinal_profile.py`) cross-references
  `/cmd_vel`, `/wheel/odom`, and `/odom_rf2o` to characterize step-response
  behavior; it is diagnostic tooling for future characterization work, not
  a ratified retuning program and not itself a change to any committed
  calibration value.
- **Hardware/integration items inherited from v1.3 Stage 7/8 remain open**:
  a real held-RUN autonomous drive **has** now been exercised and recorded
  (`bags/confident/confident_0.mcap`, see above), so that specific item is
  no longer open — but global STOP asserted during active autonomous motion
  and a DualSense takeover mid-mission were **not** exercised in that
  recording (`stop_state.stopped` was false and `command_authority_state.
  dualsense_active` was false for the entire 222 s session) and remain
  open. The end-to-end stale-command timing budget (§17, motion-deadman
  bullet) also remains open — the `confident` recording's up-to-14 s
  command-stream gaps are new evidence for *why* that budget matters, not a
  replacement for measuring it. Physical stopping-distance validation
  remains Matti's responsibility.
- The SIGKILL/PWM-peripheral hazard and heartbeat-gated-FET question (§2)
  remain open and are not addressed by anything in v1.4.

## 18. Future exploration and semantic traversability

Unchanged in scope from v1.3 §15: supervised frontier exploration
(MAPPING + AUTONOMOUS) and camera-derived semantic traversability layers
remain future work, not implemented at HEAD. No exploration velocity topic
or special motor path exists. A later mapping-time Nav2 composition would
need to reuse the live mapping raster and mapping `map → odom` without
reintroducing a second localizer/map_server; this is unchanged design
guidance, not new implementation.

## 19. Decision/spec reconciliation against v1.3

| v1.3 subject | v1.4 disposition |
|---|---|
| §1 "Paddock becomes the normal operator interface" | **Delivered.** Paddock is the primary operator interface today, materially exceeding the v1.3 milestone (adds tuning, obstacle control, recording, map deletion, field AP) |
| §5 "keyboard and Foxglove still own operational bypasses" | Foxglove bypass retired (Stage 5). `keyboard_bridge`'s autonomy-latch bypass is dead code, not re-ratified (§17); its local-driving path is retained and current |
| §9 "Remote manual ceiling 0.40 m/s… autonomy retains current maximum 0.60 m/s" | Manual ceiling unchanged at 0.40 m/s. Autonomy ceiling is now **policy-selected**: Timid retains 0.60 m/s; Confident is ratified at **1.00 m/s**, explicitly superseding the flat v1.3 figure (§12) |
| §9, §14 "The frozen controller is retained" | Superseded: a defined parameter subset is validated, atomically applied, and live-effective (§12). Committed defaults are preserved as the Timid-equivalent baseline, not as an immutable runtime constraint |
| §5 "Global STOP... required invariant" | Retained verbatim as implemented in `runner_stop_enforcer` (§6); NEW MAP is now explicitly confirmed **not** to assert/require/clear STOP (§5, §8) — a v1.3-era open question this document resolves with source evidence |
| §10 mission lifecycle table (9-state, `NO_MISSION…SUPERSEDED`) | Superseded by the canonical **implemented 7-state** `NavigationState` machine (§11); the cancellation/quiescence invariant is preserved, not strengthened beyond what tests establish |
| §13.1–13.2 NEW/SAVE MAP | Delivered as specified, plus **SELECT** and **DELETE** (not in v1.3 scope) with symmetric safety validation (§8) |
| §14 supported settings table | Delivered and expanded: `manual_max_speed_mps` config, full Timid/Confident/Custom tuning surface, obstacle-layer toggles, and recording controls now exist as real, tested, revision-checked/read-back-confirmed operator settings |
| §11 browser lease/RUN freshness, "150 ms" figures | Superseded by the measured, split, and deployed values in §6 (0.5 s motion deadman deployed but provisional, 3.0 s liveness backstop, 0.15 s raw-autonomy timeout) — the underlying "not yet end-to-end budgeted" caveat is carried forward unchanged (§17) |
| §15 future exploration/traversability | Unchanged; still future work (§18) |
| §16 historical D-number table (v1.2/v1.0 subjects) | All dispositions from that table remain in force; nothing in v1.4 reopens or renumbers them. New v1.4-only current-state facts (live tuning, DELETE MAP, recording, field AP) have no historical D-number and are not assigned one here |
| §17 gated migration plan, stages 0–8 | Complete; see §20 for a compressed summary. Stage 9 (supervised exploration) remains not started |
| §18 ratified Q1/Q2/Q3 | Q1 (persistent STOP) and Q2 (RUN release/cancel/continuation) stand as implemented (§6, §11). Q3's numeric bounds are carried forward for manual/Timid and **explicitly superseded for Confident autonomy** per Matti's ratified 1.00 m/s decision (§12) |

## 20. Historical appendix: completed v1.3 migration stages

Condensed from `docs/paddock_v1.3_implementation.md` and `services/README.md`
for continuity; treat §1–§19 above as authoritative over this appendix
wherever they differ.

- **Stage 0–1**: contract ratified; unfinished authority scaffolding isolated
  to private non-mux topics before any production cutover.
- **Stage 2**: STOP/local-control contract prototyped and gate-tested
  privately (3.293 ms request-to-zero observed; a rapid-restart edge case
  and endpoint-count discrepancy were found and are why STOP's durability
  and boot-qualification logic is as defensive as §6 describes).
- **Stage 3**: persistent local-control tier (`runner-local-control.service`)
  cut over — joy/teleop/mux/keyboard-bridge move out of application launches
  permanently.
- **Stage 4**: truthful runtime/readiness (`runtime_epoch`,
  `mapping_session_id`, capability-specific readiness) and the map-session
  executor (NEW/SAVE MAP, catalog, manifest) delivered.
- **Stage 5**: `runner_navigation_runtime` delivered as the sole Nav2 mission
  owner, retiring `foxglove_goal_bridge` and its direct goal/keyboard
  ingress atomically.
- **Stage 6** (Remote manual conversion, per v1.3 §17): the shared
  drive-adapter normal-demand contract and manual-only authority/mux input
  — no dedicated Stage 6 section exists in
  `docs/paddock_v1.3_implementation.md`; it was folded into and delivered
  together with Stage 7 Part A's autonomy authority/velocity cutover
  (`9bdf7a2`) rather than landing as a separately documented stage.
- **Stage 7**: the full supervised autonomy velocity path
  (`drive_adapter → command_authority → twist_mux`) wired end to end with
  hold-to-run semantics.
- **Stage 8 / Part C**: minimal browser operator UI delivered and later
  redesigned into the current tabbed console (§4); coherent
  `services/install.sh` deploy tooling added after discovering hand-copied
  (non-symlinked) unit files and a missing `runner-map-executor` install on
  the Pi — the origin of §15's standing "never assume deploy coherence"
  caution.
- **Post-Stage-8, pre-v1.4**: obstacle-layer controls, live autonomy speed
  tuning (Timid/Confident/Custom), MCAP recording, initial-pose workflow,
  safe map deletion, the field Wi-Fi AP, and RF2O longitudinal profiling
  tooling were all added — this is the delta this document formalizes as
  current architecture rather than migration-in-progress.
