# Runner — Architecture & Current-State Specification v1.4

**Current-state specification · 12 September 2026**

Baseline: `/home/matti/runner_ws`, clean HEAD `044b4fb565eba71a70fdc56bdd790bd9da068986` ("Quiesce
active mapping resets without global STOP"). Previous specification: `docs/runner_spec_v1.3.md`
(ratified 7 September 2026 as a migration target). This document supersedes it: v1.3 described a
staged migration from a transitional graph to a Paddock-first architecture; stages 0–8 of that
migration have landed at HEAD, and this document describes **what is actually built and running**,
not a plan to build it.

**Reading convention.** "Current" means present in HEAD source, config, tests or service units, with
a file reference. "Provisional" or "open" flags something deployed but not yet validated as final
policy — do not read either word as "unimplemented." "Future" flags something intentionally not
built. Historical D-numbers are always qualified by source version and subject; no new D-number is
allocated here. Where this document's numeric claims differ from `docs/runner_spec_v1.3.md`, this
document governs; v1.3 remains a historical record of the migration, not a competing current-state
claim.

**Deploy-coherence caveat.** This specification describes repository HEAD. `docs/paddock_v1.3_implementation.md`
records that the persistent operator services on the physical Pi have, at various points, lagged
several stages behind HEAD (stale `ModeState` schema, hand-copied unit files bypassing repo edits),
and a recent validation pass separately observed `runner-map-executor` serving an older message
schema on the deployed Pi. **HEAD source is not a claim about the currently running Pi process
image.** A coherent deploy requires `colcon build` followed by `services/install.sh` (or `--restart`)
per `services/README.md`; this document does not assert that step has been taken as of this writing.

## 1. Executive architecture summary

Paddock is the delivered primary operator interface for runtime selection, manual driving,
supervised autonomous missions, STOP, mapping sessions, map bundles, recording, live speed/controller
tuning and health feedback. A persistent Pi-side command authority validates browser intent and
grants bounded motion permission; neither the browser nor the authority can bypass actuator safety.

Runtime and motion authority remain independent dimensions:

- **Runtime:** `IDLE` / `MAPPING` / `AUTONOMY` — the active application composition.
- **Motion authority:** `NONE` / `DUALSENSE` / `PADDOCK_MANUAL` / `PADDOCK_AUTONOMY` — who currently
  holds permission to command normal motion (`CommandAuthorityState.authority`).
- **Global STOP** is a separate latched inhibit orthogonal to both: `runner_stop_enforcer` asserts or
  clears it independently of runtime and authority.

One `twist_mux` (`src/runner_bringup/config/twist_mux.yaml`) remains the sole `/cmd_vel` writer.
Precedence by mux priority: **global STOP zero (255) > STOP lock (200) > DualSense teleop (100) >
Paddock manual (75) > supervised autonomy (50)**. Inactive sources are silent; a selected zero is a
real braking command.

The full production command path is:

```
Nav2 controller_server --/cmd_vel_nav (SI Twist)--> drive_adapter --/cmd_vel_auto_raw-->
  command_authority --/cmd_vel_auto (0.30s timeout, prio 50)--> twist_mux --/cmd_vel--> motor

Paddock browser --/paddock/control_event--> command_authority --manual demand-->
  drive_adapter --/cmd_vel_paddock_manual_raw--> command_authority --/cmd_vel_paddock
  (0.30s timeout, prio 75)--> twist_mux

joy_node --/joy--> runner_teleop --/cmd_vel_teleop (0.15s timeout, prio 100)--> twist_mux

command_authority --/paddock/internal/stop_request--> runner_stop_enforcer
  --/cmd_vel_stop (0.10s timeout, prio 255) + /paddock/stop_lock (0.15s lock timeout, prio 200)--> twist_mux
```

`drive_adapter` is the single shared longitudinal PI controller for both autonomy and manual demand;
`command_authority` is the sole supervised writer to `/cmd_vel_auto` and `/cmd_vel_paddock`; the mux
is the sole arbiter and `/cmd_vel` writer; `motor_node` is the sole actuator owner. This is fully
wired and code-reachable at HEAD — it is not a migration target.

A parallel, architecturally legacy local-motion path remains live: `keyboard_bridge`, part of the
same persistent local-control tier, still receives UDP keyboard packets and can command
`/cmd_vel_teleop` motion directly through `runner_teleop` (§4.5). This bypasses the Paddock lease
model entirely (it is a local, not a leased-browser, path) but still terminates in the same
STOP/mux-governed `/cmd_vel_teleop` slot as DualSense. It is retained today, not re-ratified as a
production interface: Paddock is the ratified primary operator surface, and keyboard control is
documented here as legacy pending removal.

## 2. Platform, geometry, hardware and safety boundaries

Runner is the LaTrax Prerunner 1/18-scale research platform: Raspberry Pi 5, Ubuntu 24.04, ROS 2
Jazzy, LD19 2D LiDAR, BNO085 IMU, hall-effect wheel encoder, Cytron MD13S motor driver, steering
servo, X1201 UPS (`README.md`). Phase 1 (indoor navigation) is the established platform; there is no
imposed thesis deadline for Phase 2 racing-speed work.

Geometry is unchanged from v1.2/v1.3 and confirmed in current config: wheelbase 0.178 m
(`src/runner_drive_adapter/config/drive_adapter.yaml:5`, `AdapterConfig.wheelbase` default),
maximum steering angle 0.3614 rad, physical minimum turning radius 0.470 m, and the Nav2 planner's
configured `minimum_turning_radius: 0.60` m (`src/runner_bringup/config/nav2_params.yaml:30`). These
remain distinct quantities; this document changes none of them.

| Resource / boundary | Current owner and rule |
|---|---|
| Motor effort PWM, steering PWM, direction GPIO | `runner-motor.service` / `motor_node`; sole continuous owner of GPIO12 (20 kHz hardware PWM) and GPIO23 DIR via `pinctrl-rp1`; GPIO13 is the 50 Hz steering PWM; never unexports either channel |
| Encoder GPIO | `runner-encoder.service`; sole continuous owner of GPIO22; independent of application launches |
| Motor watchdog | `motor_node`, `CMD_TIMEOUT_S = 0.2` (`src/runner_motor/runner_motor/motor_node.py:18`), checked on a 0.05 s timer (`:212`); on timeout, duty is driven to zero (active brake) and a log is emitted (`:279-286`). Unchanged since v1.2/v1.3. |
| Direction/reversal | Motor-local gate; a negative demand means reverse, never brake; fail-closed at zero duty if encoder evidence of stationarity is absent at a direction-change request |
| LD19 UART | LD19 driver process; sole owner of `/dev/ttyAMA0` |
| BNO085 UART/reset | IMU process; sole owner of `/dev/ttyAMA2` |
| `odom → base_link` | `ekf_node` (`src/runner_bringup/config/ekf.yaml`) fuses **RF2O `/odom_rf2o`** (x-linear velocity, yaw velocity) and **BNO085 `/imu/data`** (yaw velocity) only, at 15 Hz. The wheel encoder is **not** an EKF input at HEAD — it feeds `motor_node`, `drive_adapter` and `runner_stop_enforcer` direction/stationarity gating instead. This refines v1.3's data-flow diagram, which implied encoder→EKF fusion. |
| `map → odom` | Exactly one `slam_toolbox` instance: mapping SLAM in `MODE_MAPPING`, localization SLAM (remapped, publishing `/slam_map`) in `MODE_AUTONOMY` |
| `/map` | Mapping: `slam_toolbox`. Autonomy: `map_server`, sole publisher from the selected saved bundle. |
| Static extrinsics | Static TF publishers, unchanged |

TF is a multi-publisher transport with single ownership **per edge**, not a single-writer topic;
duplicate node *names* (the known double-listed RF2O DDS discovery artifact) are not evidence of a
duplicate hardware owner (`docs/paddock_v1.3_implementation.md` Stage 4 notes; `mode_runtime.py`
structural-readiness comments).

**Preserved hard safety limitations (do not treat as closed by this document):**

- Paddock STOP is a software motion inhibit, not proof of mechanical stationarity or independent
  power isolation.
- The known motor SIGKILL/PWM-peripheral persistence hazard and the deferred heartbeat-gated FET
  decision from v1.2 are unchanged; nothing here claims to close them.
- The LD19 scan plane sits 0.1135 m above the floor (`docs/local_costmap_obstacles.md`,
  `docs/global_costmap_obstacles.md`). Descending edges, thresholds, and objects below ≈0.11 m are
  structurally invisible to the planar scan and cannot be corrected in costmap software. This is a
  standing operating-envelope limit for both mapping and obstacle-aware autonomy, unchanged by
  Section 13's obstacle-costmap coverage.

## 3. Process tiers and ownership model

Four tiers, unchanged in shape from v1.3 but now fully populated (`services/README.md`):

**Persistent hardware tier** (never in a mode composite): `runner-pwm-setup`, `runner-motor`,
`runner-encoder`, `runner-battery`, `runner-telemetry`, `runner-foxglove`.

**Persistent local-control tier** (`runner-local-control.service`, `teleop.launch.py`): one
`joy_node`, `keyboard_bridge`, `runner_teleop`, and the one `twist_mux`. Independent of application
launches and of web/authority/mode-supervisor health. `runner-stop-enforcer.service` is a separate
persistent unit in this tier providing global STOP.

**Persistent operator tier**: `runner-command-authority`, `runner-drive-adapter`,
`runner-mode-supervisor`, `runner-map-executor`, `runner-recording-executor`, `runner-paddock-web`.
These own permission, conversion, mode lifecycle, map bundles, recording and the browser gateway —
never PWM, GPIO, systemd beyond the two narrowly-scoped mode units, or Nav2 internals directly.

**Application tier**: sensors + estimation + `slam_toolbox` (mapping) started by `map.launch.py`
under `runner-mode-mapping.service`; sensors + estimation + localization `slam_toolbox` + `map_server`
+ Nav2 + `runner_navigation_runtime` started by `autonomy.launch.py` under
`runner-mode-autonomy.service`. Both are `Conflicts=`, `KillMode=control-group`, no `[Install]`
section (never boot-enabled); `runner-mode-supervisor` is their sole start/stop owner, via the
systemd D-Bus API under a narrow polkit rule (`services/49-runner-mode-units.rules`) scoped to
`start`/`stop` on exactly those two unit names for user `matti`.

`services/install.sh` is the single source of truth for the systemd layout (`--check` reports drift,
no-arg applies it, `--restart` cycles the operator/application tier in dependency order:
`stop-enforcer → command-authority → local-control → drive-adapter → mode-supervisor →
map-executor → recording-executor → paddock-web`). It never touches `runner-motor`/`runner-encoder`.

## 4. Paddock/operator surfaces and control roles

| Surface | Current role |
|---|---|
| Paddock (browser) | Primary runtime, mapping, manual driving, mission, RUN/STOP, recording, live speed/controller tuning and health interface. Sole browser-side writer of `/paddock/control_event`, `/paddock/mode_request`, `/paddock/map_request`, `/paddock/recording_request`, autonomy-tuning writes, and the controlled `/initialpose` publisher. |
| DualSense (Bluetooth, via `joy_node`/`runner_teleop`) | Independent local manual fallback and takeover; deadman button (`X`, index 0), fixed-throttle mode (`R1`), teleop-suppress (`L1`); local effort shaping unchanged from v1.2/v1.3 (§12) |
| Keyboard (UDP, via `keyboard_bridge`) | **Legacy, pending removal.** Still built, still part of the persistent local-control tier launch, still capable of commanding `/cmd_vel_teleop` motion, still gates the global-obstacle-costmap clear/toggle route commands (§13). Not the ratified operator interface; see §4.5 and §19. |
| Foxglove (`runner-foxglove.service`) | Diagnostic visualization, TF/topic inspection, plots, deeper engineering tools |
| SSH/config files/ROS parameters | Engineering/debug configuration; not the normal operator abstraction |

### 4.1 Lease model

`runner_paddock/gateway.py` (`OperatorGateway`, pure/ROS-free) holds at most one control lease at a
time — one *controller*, any number of *observers*. Every outbound intent is produced only in direct
response to a fresh browser message; the gateway manufactures no renewals, so a silent browser lets
the Pi-side lease expire and revokes RUN. `on_disconnect` always releases the lease
(`EVENT_LEASE_RELEASED`); reconnection starts with no lease, no RUN latch, and no goal. A short lease
lapse from the *same* client/lease id can be reinstated by a fresh heartbeat
(`command_supervisor.py`'s `_lapsed_lease` backstop) without forcing a page reload — this is a UX
convenience for Wi-Fi jitter, not a motion-grant carryover: RUN and manual demand are not restored by
reinstatement.

### 4.2 Global STOP (`runner_stop_enforcer`, `src/runner_paddock/runner_paddock/stop_enforcer.py`)

STOP is a persistent, boot-independent, durably-recorded latch, unchanged in intent from v1.3's Q1
and now fully implemented:

- State persisted at `/home/matti/.local/state/runner/stop.json` with monotonic `generation`,
  atomic write (`os.replace` + directory `fsync`) (`:50-65`). Unknown/corrupt state on startup is
  treated as **STOP asserted** (`fault='STOP_STATE_UNKNOWN'`), never as clear (`:84-89`).
- `/paddock/internal/stop_request` (`StopRequest`) is the sole assert/clear input, idempotent per
  `(requester_id, request_id)`, rejecting stale/replayed ids (`:177-190`).
- Assert is immediate and precedes disk I/O: generation increments, lock and zero publish happen
  synchronously, persistence happens on a background thread (`:191-201`).
- Clear requires: fresh encoder evidence of stationarity (≤0.20 s old), fresh local-control status
  reporting released+neutral (≤0.20 s old), and no pending fault/persistence
  (`clear_reason()`, `:152-164`). A satisfied clear request still enters a **0.40 s drain window**
  (`DRAIN_TIME`) during which the lock stays asserted and any regression in the clear conditions
  re-asserts STOP before the drain completes (`:231-244`). This is the "old stop velocity input must
  age out" barrier from v1.3 §5, now concretely 0.40 s.
- The 50 Hz lock/zero enforcement loop (`tick`, 0.02 s timer) is unconditional; status publication is
  event-driven with a 20 Hz (`STATE_HEARTBEAT_PERIOD = 0.05`) liveness floor (`:266-280`).
- `LOCK_TIMEOUT = 0.15` s: `twist_mux`'s lock treats an expired heartbeat as **locked** (fail-closed),
  matching `locks.global_stop.timeout: 0.15` in `twist_mux.yaml`.

Current `twist_mux.yaml` priorities and timeouts, exactly as deployed
(`src/runner_bringup/config/twist_mux.yaml`):

| Input | Topic | Timeout | Priority |
|---|---|---|---|
| Supervised autonomy | `/cmd_vel_auto` | 0.30 s | 50 |
| Supervised Paddock manual | `/cmd_vel_paddock` | 0.30 s | 75 |
| Local teleop (DualSense + keyboard, same slot) | `/cmd_vel_teleop` | 0.15 s | 100 |
| Global STOP zero | `/cmd_vel_stop` | 0.10 s | 255 |
| Global STOP lock | `/paddock/stop_lock` | 0.15 s | 200 |

This confirms v1.3's Q1 priority ordering is deployed as specified, with the 0.30 s autonomy/manual
timeouts, 0.15 s teleop timeout and 0.10/0.15 s STOP timeouts as the actual (not merely proposed)
values.

### 4.3 `/teleop/control_state` (`LocalControlState`)

Published by `runner_teleop` at its 20 Hz (`0.05 s`) command timer. Fields actually present:
`process_epoch` (per-process UUID), `takeover_epoch` (monotonic counter, incremented on every
DualSense/fixed-throttle takeover rising edge — persists across intermediate status updates so a
short local engagement cannot be missed, per v1.3 §11's requirement), `connected`, `active`
(takeover currently held), `neutral`, `released`, `sample_age_sec`, `mode` (free-text diagnostic
string, e.g. `manual`, `keyboard_motion`, `release_brake`). `runner_stop_enforcer` consumes
`released`/`neutral` directly for its clear gate (§4.2).

### 4.4 Drive adapter (`src/runner_drive_adapter`)

Single shared longitudinal PI controller for both Nav2 autonomy and Paddock manual demand. Current
committed defaults (`AdapterConfig`, `drive_adapter.py:83-105`, mirrored by the `TIMID` autonomy
preset in `autonomy_tuning.py`):

- Feedforward: `0.1188·|v| + 0.0174` (normalized effort)
- Proportional gain 0.05, integral gain 0.01 (config file; dataclass default is 0.0 and is
  overridden by `drive_adapter.yaml`/live tuning), integrator bound ±0.005, output magnitude cap
  `MAXIMUM_OUTPUT_AUTHORITY = 0.14`
- `minimum_moving_speed: 0.25` m/s (the frozen zero-or-≥0.25 contract, unchanged)

These are the same numeric constants v1.3 recorded as "frozen." **"Frozen controller" is now obsolete
language**: `1895caa` ("Make drive adapter tuning live-effective") makes exactly six parameters
runtime-mutable via ROS parameter callback, gated by `LIVE_TUNABLE_PARAMETERS`
(`drive_adapter.py:26-32`): `maximum_commanded_speed`, `feedforward_effort_per_speed`,
`feedforward_effort_intercept`, `output_max`, `proportional_gain`, `integral_gain`. Every other
adapter parameter remains immutable at runtime (`drive_adapter_node.py:211`, rejecting non-live-tunable
parameter changes). The *values above remain the committed default/baseline*; live tuning is an
intentional, bounded, validated capability layered on top (§12), not a repudiation of the calibration.

Sole-writer topics (unchanged ownership from v1.3 Stage 7, confirmed in code):
`drive_adapter` → `/cmd_vel_auto_raw` (`Twist`) and `/cmd_vel_paddock_manual_raw`
(`ConvertedCommand`) (`drive_adapter_node.py:111-113`). `command_authority` is the sole writer of the
supervised `/cmd_vel_auto` and `/cmd_vel_paddock`.

Direction feedback still reaches the adapter through `EncoderState.pending_direction`, not a direct
`/motor/direction` subscription. Autonomy steering is still `yaw_rate / speed` (signed); its
zero-speed branch brakes and does not carry manual steering, so manual demand supplies steering
through its own boundary rather than synthetic yaw rate — unchanged from v1.3 §9.

### 4.5 Keyboard bridge — current status (legacy, not re-ratified)

`keyboard_bridge` (`src/runner_teleop/runner_teleop/keyboard_bridge.py`) is **still built, still
launched** as part of `teleop.launch.py` inside `runner-local-control.service` (persistent tier), at
HEAD. It is not a dead file:

- It receives UDP packets on `0.0.0.0:49321`, decodes throttle/steering/mode, and publishes
  `/teleop/keyboard_state` (`KeyboardState`) at 20 Hz.
- `runner_teleop.teleop_node` subscribes `/teleop/keyboard_state` and, when no DualSense takeover is
  held, can select `KEYBOARD_MOTION_MODE` and publish real motion on `/cmd_vel_teleop`
  (`teleop_node.py:569-577`) — i.e. keyboard input can still drive the vehicle today, gated by the
  same mux/STOP precedence as DualSense (priority 100), but **without going through the Paddock
  lease/authority model** at all.
  operator direction is that this bypass is legacy and pending removal, not that it is inert.
- It retains a **600 s "autonomy latch"** concept (`KeyboardAutonomyLatch`,
  `DEFAULT_AUTONOMY_LATCH_TIMEOUT = 600.0`), which today only gates the keyboard's own
  suppress/motion arming state machine (backtick to arm, Escape/DualSense X/L1/R1 to disarm) — it is
  **not** wired to the AUTONOMY runtime or to Nav2 dispatch; there is no remaining "keyboard arms
  autonomous driving" path. The v1.0-era "keyboard autonomy is a 600 s Pi-side latch" behavior
  (D-71) was already superseded for production by v1.3; what remains at HEAD is a same-named timeout
  constant governing an unrelated, narrower local-suppress state machine.
- It also owns the `/runner/route_control` global-obstacle-layer clear/toggle commands consumed
  historically by the route/waypoint system; this is real, exercised functionality (§13) still routed
  through keyboard input, with no Paddock-native equivalent yet.

**v1.4 disposition:** document this as-is. It is not re-ratified as part of the Paddock-first
architecture, must not be extended, and is a concrete candidate for removal once its
obstacle-layer-toggle and any remaining engineering utility are ported to Paddock or Foxglove-driven
service calls (§17, §19).

## 5. Runtime modes and lifecycle

`ModeState` (`src/runner_interfaces/msg/ModeState.msg`) fields at HEAD: `mode` (`MODE_IDLE` /
`MODE_MAPPING` / `MODE_AUTONOMY`), `status` (`STATUS_STABLE` / `STATUS_TRANSITIONING` /
`STATUS_FAULT`), `accepted_request_id`, `active_autonomy_map`, `detail`, `runtime_epoch`,
`mapping_session_id`, `ready`, `readiness_reason`. `runtime_epoch` increments on every successful
application start and on every NEW MAP; consumers reject status/results carrying a stale epoch.

`runner-mode-supervisor.service` is the sole start/stop owner of the two fixed mode units. It always
publishes `TRANSITIONING` first, stops both units, waits for empty cgroups and a graph with no mode
resources, starts and structurally checks the requested mode, and fails closed to `IDLE/FAULT` on any
partial/conflicting/unmanaged graph — it never guesses a mode from an ambient process tree, and never
adopts an externally-launched `map.launch.py`/`autonomy.launch.py` invocation.

Readiness is capability-specific and continuously refreshed on a 0.5 s timer (not only checked at
transition): structural readiness (unit active, every required node present, no cross-mode node,
exact publisher-owner match per critical topic) plus a MAPPING-specific check (`map→odom` and
`odom→base_link` TF usable, `/scan_slam` fresh, a `/map` update seen *after* the current session
began) or an AUTONOMY-specific check (TF usable, `/scan` fresh, `/map` present from `map_server`).
`readiness_reason` carries the first actionable inhibit reason. A stable-but-not-`ready` runtime is
treated as ineligible for motion (§6).

## 6. Motion authority, STOP, lease, RUN and takeover

`CommandAuthorityState` fields (`src/runner_interfaces/msg/CommandAuthorityState.msg`): `authority`
(`NONE`/`DUALSENSE`/`PADDOCK_MANUAL`/`PADDOCK_AUTONOMY`), `runtime_epoch`, `client_id`, `lease_id`,
`dualsense_active`, `run_held`, `autonomy_permitted`, `autonomy_goal_selected`, `goal_frame`/
`goal_map`/`goal_x`/`goal_y`/`goal_yaw`, `autonomy_action_active`, `brake_intent`, `lease_fresh` +
`lease_age_sec`, `raw_autonomy_fresh` + `raw_autonomy_age_sec`, `manual_input_fresh` +
`manual_input_age_sec`, `manual_applied_speed_mps`, `manual_applied_steering`,
`last_control_sequence`, `reason`, and mirrored STOP fields (`stop_state_fresh`, `stop_healthy`,
`stop_applied`, `stop_clear`, `stop_boot_id`, `stop_generation`, `stop_reason`).

Supervised autonomy output on `/cmd_vel_auto` requires **all** of (unchanged from v1.3 Stage 7,
confirmed live in `command_authority_node.py`):

- runtime actually `AUTONOMY`, `STATUS_STABLE` **and** `ModeState.ready`;
- current runtime epoch / active map identity match (a map/epoch change clears goal and RUN);
- fresh Paddock lease (deployed `lease_timeout_sec: 0.5`, a first-integration Wi-Fi value flagged for
  tightening before higher-speed operation, not yet revisited);
- STOP fresh **and** clear;
- no DualSense takeover and no unresolved `run_blocked_until_release`;
- current RUN held (hold-to-run, not click-to-arm);
- fresh raw converted input (`raw_autonomy_timeout_sec: 0.15` s at the adapter's 20 Hz cadence);
- a real, current, `STATE_ACTIVE` Nav2 mission (§10) — `DISPATCHING`, `CANCELING`, terminal, or stale
  navigation state forbid output.

RUN press dispatches through the current epoch/generation; RUN release revokes motion immediately and
requests an asynchronous Nav2 cancel, retaining the logical mission as a deliberate-continuation
candidate (Q2, unchanged) — a fresh RUN press is required to redispatch. Releasing DualSense never
auto-resumes autonomy or manual: `run_blocked_until_release` requires a fresh release→press after
takeover clears. Reconnect never resurrects RUN, a goal, or a STOP clear; the authority restarts with
none of them held.

## 7. Command/data-flow and exact topic/interface ownership

| Topic/interface | Type | Sole writer → consumer(s) | Lifetime | Notes |
|---|---|---|---|---|
| `/paddock/control_event` | `PaddockControlEvent` | Paddock gateway → command authority | Persistent | Lease, RUN, STOP/CLEAR STOP, goal-selected, heartbeat |
| `/paddock/control_lease` | `PaddockControlLease` | Command authority → executors/gateway | Persistent | Lease id/owner/expiry/generation |
| `/paddock/command_authority_state` | `CommandAuthorityState` | Command authority → gateway/adapter/navigation | Persistent | §6 |
| `/paddock/mode_request` | `ModeRequest` | Command authority (runtime selection) + map executor (NEW MAP) → mode supervisor | Persistent | `request_id = time.time_ns()`, globally monotonic across both writers |
| `/paddock/mode_state` | `ModeState` | Mode supervisor → authority/executors/gateway | Persistent | Re-published every 0.5 s tick even with no change (fixed post-deploy; §17) |
| `/paddock/map_request` → `/paddock/map_state` | `MapRequest` / `MapState` | Gateway → map executor; map executor → authority/gateway/mode supervisor | Persistent | NEW/SAVE/SELECT/DELETE (§8) |
| `/paddock/navigation_request` → `/paddock/navigation_state` | `NavigationRequest` / `NavigationState` | Command authority → navigation runtime; navigation runtime → authority/gateway | Application | §10 |
| `/paddock/recording_request` → `/paddock/recording_state` | `RecordingRequest` / `RecordingState` | Gateway → recording executor; recording executor → gateway | Persistent | §14 |
| `/paddock/stop_state`, `/paddock/stop_lock`, `/cmd_vel_stop` | `StopState` / `Bool` / `Twist` | STOP enforcer → authority/local/gateway/mux | Persistent | §4.2 |
| `/paddock/internal/stop_request` | `StopRequest` | Command authority → STOP enforcer | Persistent | Assert/clear with generation |
| `/cmd_vel_nav` | `Twist` (SI m/s, rad/s) | `controller_server` → drive_adapter | Application | Unchanged |
| `/cmd_vel_auto_raw` | `Twist` | drive_adapter → command authority | Persistent process, active in AUTONOMY only | Raw converted autonomy command |
| `/cmd_vel_paddock_manual_raw` | `ConvertedCommand` | drive_adapter → command authority | Persistent | Raw converted manual command |
| `/cmd_vel_auto` | `Twist` | Command authority → twist_mux | Persistent, silent when ineligible | Priority 50, timeout 0.30 s |
| `/cmd_vel_paddock` | `Twist` | Command authority → twist_mux | Persistent, silent when ineligible | Priority 75, timeout 0.30 s |
| `/cmd_vel_teleop` | `Twist` | runner_teleop → twist_mux | Persistent | Priority 100, timeout 0.15 s; fed by DualSense **and** keyboard (§4.5) |
| `/cmd_vel` | `Twist` | twist_mux → motor | Persistent | Sole final command |
| `/teleop/control_state` | `LocalControlState` | runner_teleop → authority/adapter/STOP enforcer/gateway | Persistent | §4.3 |
| `/teleop/keyboard_state` | `KeyboardState` | keyboard_bridge → runner_teleop | Persistent | Legacy (§4.5) |
| `/wheel/encoder_state` | `EncoderState` | Encoder → motor/adapter/STOP enforcer | Persistent | Not an EKF input (§2) |
| `/drive_adapter/state`, `/drive_adapter/state_typed` | — | drive_adapter → gateway/diagnostics | Persistent | Includes live-tuned parameter identity |
| Nav2 `NavigateToPose`, `NavigateThroughPoses` | Actions | `runner_navigation_runtime` client ↔ `bt_navigator` | Application | Sole client owner |
| `/initialpose` | `PoseWithCovarianceStamped` | Paddock gateway (controlled) → slam_toolbox | Persistent writer, eligible runtime only | §9 |
| `/global_costmap/*` obstacle-layer services | Nav2 dynamic-parameter/service calls | keyboard_bridge (legacy) → Nav2 | Application | §13 |

Legacy interfaces from v1.3's transitional graph (`/move_base_simple/goal`, `/runner/waypoint`,
`/runner/route_control` as a production goal path, `/teleop/active_mode`, the private
`/paddock/private/cmd_vel_auto*` scaffold) are gone: `foxglove_goal_bridge` was removed in Stage 5.
`/runner/route_control` survives only as the keyboard-bridge-driven obstacle-layer toggle channel
(§4.5, §13), not as a goal/waypoint ingress.

## 8. Mapping sessions, map bundles, NEW/SAVE/SELECT/DELETE

`runner_map_executor` (`runner-map-executor.service`) owns `/paddock/map_request` →
`/paddock/map_state`, never a process-lifecycle owner itself.

- **Entering MAPPING** creates a fresh unsaved session; a repeated MAPPING request for an
  already-stable runtime is idempotent.
- **NEW MAP** (HEAD behavior, `044b4fb`, ratified as permanent policy): within an already-stable
  MAPPING session, NEW MAP is handled entirely through **authority revocation + fresh
  post-revocation stationary-encoder evidence** — it does **not** require, assert, or clear global
  STOP. `CommandAuthorityState.runtime_epoch` lets the mode supervisor bind the specific revocation
  acknowledgement to the transition that requested it (`command_authority_node.py:797`+1). The
  supervisor's `_quiescence_ready()` gate (`mode_supervisor_node.py:198-207`) requires, in order: (1)
  authority-confirmed revocation, then (2) an `EncoderState.stationary=true` sample timestamped
  *after* that revocation. A pre-revocation stationary sample cannot satisfy the gate (unit-tested
  directly), and the gate cannot pass while `PADDOCK_MANUAL` authority is held. On timeout the prior
  session is left running unchanged with truthfully recomputed readiness — the old SLAM/session owner
  is never torn down speculatively. `MapState.reset_state` (`RESET_IDLE`/`RUNNING`/`SUCCEEDED`/
  `FAILED`) and `reset_detail` surface this to the operator. An already-asserted global STOP is
  preserved unchanged through the operation. Saved bundles are never touched by NEW MAP.
- **SAVE MAP** (`OP_SAVE_MAP`) is unchanged from v1.3 Stage 4: requires a current valid MAPPING
  session and matching `session_id`; transactional — serialize into `maps/.staging/`, capture a
  same-session `/map` occupancy raster, write PGM/YAML, validate the four-artifact bundle
  (non-empty `.posegraph`/`.data`, YAML parses, referenced raster exists with a plausible header,
  session association), write `<name>.manifest.json` (session id, creation time, revision, SHA-256 of
  the four artifacts), then atomically move into `maps/` (manifest last). Retries for the same
  `request_id` return the cached outcome; a half-written bundle stays in `.staging/` and never enters
  the catalog.
- **Catalog and SELECT**: `MapState.catalog` (`MapCatalogEntry[]`) lists every discovered bundle with
  `complete`, `revision`, `session_id`, raster metadata, and an incompleteness reason where
  applicable; completeness re-validates cheaply on each publish (existence + non-empty + YAML parse +
  raster header + manifest hash match). `SELECT` requires a verified complete bundle, is rejected
  during a runtime transition, and never hot-swaps the map under a live AUTONOMY runtime.
  `selected_map_requested` / `selected_map_applied` / `selected_map_reason` surface the
  request/apply/readback distinction.
- **DELETE** (`OP_DELETE_MAP`, current, not present in v1.3): lease-scoped, executor-validated,
  synchronous (`MapState.delete_state`/`delete_request_id`/`delete_name`/`delete_detail`; no in-flight
  state because deletion does not span a transition). Rejects deleting the currently-selected map or
  the map an active AUTONOMY runtime is using (`map_session_node.py:409`+).

`slam_toolbox` 2.8.5's own `Reset.srv` remains available but unused for session boundaries; a fresh
process restart is still the clean-session boundary, per v1.3's reasoning — unchanged at HEAD.

## 9. Initial Pose workflow

Current (`services/README.md`, `ros_state_node.py`): the Paddock gateway is the controlled production
writer to slam_toolbox's `/initialpose` subscriber. It accepts only a finite map-frame pose while
stable AUTONOMY is using the same complete selected map, healthy STOP is applied, and fresh encoder
state reports stationary. It publishes `PoseWithCovarianceStamped` using slam_toolbox's RViz
`SetInitialPose` planar defaults (x/y variance 0.25 m², yaw variance 0.06853891909122467 rad²), and
reports the intent "applied" only after a subsequent map-frame slam_toolbox `/pose` sample confirms
it (`ros_state_node.py:771`, "confirm a seed only from subsequent slam_toolbox pose truth"). This is a
fully current, deployed capability — not a v1.3 proposal — gated identically to SAVE MAP's
stopped/stationary precondition.

## 10. Localization, estimation and TF ownership

Unchanged tier ownership from §2. Explicitly current:

- `ekf_node` fuses **RF2O linear-x + yaw velocity** and **BNO085 yaw velocity** at 15 Hz
  (`ekf.yaml`); `two_d_mode: true`; publishes `odom → base_link`.
- The LD19 raw `/scan` is preserved and branched: `scan_rebinner` → `/scan_slam` (fixed 503-bin
  cardinality, `docs/decision_D-37_fixed_slam_scan_cardinality.md`, v1.2 D-37, still governing —
  Karto hard-rejects a mismatched range count) feeding `slam_toolbox`; `rf2o_scan_canonicalizer` →
  `/scan_rf2o` feeding RF2O. Both slam_toolbox roles (mapping, localization) consume `/scan_slam`.
- `map → odom` is exclusively slam_toolbox's; `map_server` is the sole `/map` publisher in AUTONOMY.

## 11. Nav2, mission lifecycle and reverse autonomy

`runner_navigation_runtime` (`ros2 run runner_bringup navigation_runtime`, started inside
`nav2.launch.py`, application-tier) is the **only** component holding Nav2 mission action clients:
one `NavigateToPose` client, one `NavigateThroughPoses` client. `foxglove_goal_bridge` and its direct
goal/keyboard/waypoint ingress are removed (Stage 5); `mode_runtime.AUTONOMY_ONLY_NODES` now expects
`/runner_navigation_runtime`.

**Canonical 7-state lifecycle** (`NavigationState.msg`, current and exhaustive — this is the actual
enum, not a proposal):

```
STATE_IDLE=0        # runtime alive; no in-flight action
STATE_DISPATCHING=1 # goal sent to Nav2; not yet accepted (no motion)
STATE_ACTIVE=2       # Nav2 accepted and is executing the current goal
STATE_CANCELING=3    # cancel requested; awaiting terminal action result
STATE_SUCCEEDED=4    # real Nav2 success result
STATE_FAILED=5       # rejection / abort / timeout / error
STATE_CANCELED=6     # confirmed Nav2 cancellation
```

`STATE_ACTIVE` is reported only after Nav2 has actually accepted a goal handle, never on dispatch
intent. Separate identities — process `boot_id`, `runtime_epoch`, `mission_revision`,
`action_generation` — are each checked on every asynchronous Nav2 callback; a stale one is dropped and
can never overwrite current mission state or reopen a grant. On restart, the new `boot_id` makes any
previously reported state stale, and the first dispatch after boot issues a best-effort `CancelGoal`
to both action servers before sending, so an orphaned server goal is never adopted.

`command_authority` publishes `/paddock/navigation_request` (`OP_SELECT` on new goal intent,
`OP_DISPATCH` on the dispatch-intent edge, `OP_CANCEL` on RUN release/STOP/DualSense
takeover/lease loss/mode-or-epoch change/goal clearing). RUN-release keeps the logical mission as a
deliberate-continuation candidate (Q2, unchanged); epoch/map change and leaving AUTONOMY invalidate it
outright. This preserves v1.3's cancellation/quiescence invariant as originally specified — this
document does not claim it is stronger than the source and tests above establish, and the delayed-
old-`Twist`-across-cancellation quiescence acceptance test described in v1.3 §10 was exercised only in
isolated unit tests at Stage 5/7, not against a live `bt_navigator` (§17).

Reverse autonomy (Reeds-Shepp/RPP reverse path and downstream reversal ownership) is retained
unchanged from v1.2 D-85; `nav2_regulated_pure_pursuit_controller` is a Runner-modified vendored
package (`src/nav2_regulated_pure_pursuit_controller`, `README.runner.md`).

## 12. Longitudinal control and Timid/Confident/Custom speed policy

**This is the single largest substantive change from v1.3.** `65f2993` ("Add live Paddock autonomy
tuning") introduces `runner_paddock/autonomy_tuning.py` as the authoritative schema for a live,
Paddock-driven speed/controller preset system, replacing v1.3 §14's static bounds table for the
autonomy-tunable subset.

**Presets are real operator UI concepts, not internal-only constants** (`autonomy_tuning.py:91-118`,
surfaced in `static/app.js`/`index.html`):

| Field | Owner (ROS node.param) | Timid | Confident |
|---|---|---|---|
| `desired_linear_vel` | `/controller_server` `FollowPath.desired_linear_vel` | 0.45 | **1.00** |
| `maximum_commanded_speed` | `/drive_adapter` `maximum_commanded_speed` | 0.60 | **1.00** |
| `regulated_linear_scaling_min_speed` | `/controller_server` | 0.30 | 0.40 |
| `cost_scaling_dist` | `/controller_server` | 0.45 | 0.60 |
| `cost_scaling_gain` | `/controller_server` | 1.0 | 1.0 |
| `regulated_linear_scaling_min_radius` | `/controller_server` | 0.75 | 0.75 |
| `min_lookahead_dist` / `max_lookahead_dist` | `/controller_server` | 0.30 / 0.80 | 0.30 / 0.80 |
| `lookahead_time` | `/controller_server` | 1.0 | 1.0 |
| `max_allowed_time_to_collision_up_to_carrot` | `/controller_server` | 0.15 | 0.60 |
| `proportional_gain` / `integral_gain` | `/drive_adapter` | 0.05 / 0.01 | 0.05 / 0.01 |
| `feedforward_effort_per_speed` / `_intercept` | `/drive_adapter` | 0.1188 / 0.0174 | 0.1188 / 0.0174 |
| `output_max` | `/drive_adapter` | 0.14 | 0.14 |

**Confident's 1.00 m/s desired and maximum commanded speed explicitly supersedes v1.3's 0.60 m/s
autonomy maximum by ratified operator decision.** Timid preserves the exact v1.3/v1.2 "frozen"
values as its own preset, not as a separate immutable ceiling — Timid is simply the conservative
choice within the same live-tunable mechanism.

**Custom is the truthful non-preset state**, not a third design option: `matching_preset()`
(`autonomy_tuning.py:182-189`) classifies a live-readback snapshot as `timid`/`confident` only on an
**exact** field-for-field match against the full parameter set; any other combination — including one
field nudged off a preset — reports `custom`. This is intentionally the honest default rather than a
fuzzy "closest preset" heuristic.

**Validation and application** (`validate_values`, `:130-179`): every write is a complete atomic
snapshot across all twelve fields (rejecting partial writes outright), cross-validated
(`regulated_linear_scaling_min_speed ≤ desired_linear_vel ≤ maximum_commanded_speed`,
`min_lookahead_dist ≤ max_lookahead_dist`, feedforward non-negative across the commanded range,
`output_max ≤ 0.14` and `output_max ≥` the feedforward value at `maximum_commanded_speed` —
i.e. the 0.14 authority ceiling from v1.3 is retained as a hard upper bound even under live tuning,
not loosened). Applied atomically per owner (`values_for_owner`) as one `SetParameters` call to
`/controller_server` and one to `/drive_adapter`; the six drive-adapter fields are exactly
`LIVE_TUNABLE_PARAMETERS` (§4.4) — geometry, integrator bound, `output_min`, wheelspin/timeout
constants and every other adapter parameter remain immutable at runtime.

**Manual (Paddock browser teleop) speed is a separate axis, unaffected by autonomy presets.**
`command_supervisor.py`: `manual_max_speed_mps` remains 0 (disabled) or `[0.25, 0.40]`, default 0.40
(`DEFAULT_MANUAL_MAX_SPEED_MPS = MAX_MANUAL_MAX_SPEED_MPS = 0.40`) — identical bounds to v1.3 §14,
unaffected by Confident's 1.00 m/s. DualSense local effort shaping is likewise a separate,
deliberately non-unified semantic path (v1.2 D-83, unchanged).

Direct RPP and longitudinal-controller tuning (i.e. this whole mechanism) is intentionally
**Advanced/Engineering functionality** layered onto the normal Timid/Confident preset picker, not a
freely-editable general parameter surface (§16).

## 13. Obstacle-aware costmaps

Current and deployed, not proposed (`docs/global_costmap_obstacles.md`,
`docs/local_costmap_obstacles.md`, `d852fb3`, `dd6143e`, `8b81813`):

- **Global costmap**: `obstacle_layer` loaded and enabled by default; live `/scan` observations
  participate in path validity checks and global replanning. `combination_method: 0` (Overwrite) —
  live observations replace static-map costs inside the layer's update bounds, which also means a
  raytrace can clear a transient mark but can equally clear a genuinely-occupied but
  below-scan-plane static feature (the known low-obstacle hazard, §2). Enable/disable is a dynamic
  parameter (`obstacle_layer.enabled`), verified to apply without relaunch on Jazzy
  `nav2_costmap_2d` 1.3.12.
- **Local costmap**: consumes `/scan` directly; marking 0.05–1.0 m, clearing raytrace 0.0–1.2 m;
  infinite returns are valid clearing observations; not persisted; no expected-update-rate timeout
  enforced. Obstacle height range 0.0–2.0 m at both layer and source level (the source-level maximum
  must stay explicit — Nav2's own default of 0.0 m would otherwise reject every point given the
  0.1135 m mount height).
- **Operator control today**: the only live toggle/clear path is `keyboard_bridge`'s
  `ROUTE_CLEAR_GLOBAL_OBSTACLES` / `ROUTE_TOGGLE_GLOBAL_OBSTACLES` UDP commands, which call the
  `/global_costmap/global_costmap` `clear_entirely_global_costmap` service and
  get/set-parameters services directly (`keyboard_bridge.py:456-680`). There is no Paddock-native
  UI for this yet — it is real, exercised functionality currently gated behind the legacy keyboard
  path (§4.5), a concrete gap for Paddock parity (§17).

## 14. Recording/observability

Current and deployed (`runner-recording-executor.service`, `src/runner_paddock/runner_paddock/recording.py`):
sole Pi-side owner of the `ros2 bag record` process, independent of WebSocket connection lifetime (closing
or reloading Paddock does not stop an active bag). `RecordingRequest` (`OP_START`/`OP_STOP`/`OP_DELETE`,
lease-scoped, `name`, `profile`) → `RecordingState` (`STATE_IDLE`/`STARTING`/`RECORDING`/`STOPPING`/`FAILED`,
`elapsed_sec`, `size_bytes`, `output_path`, `recorder_pid`, `process_healthy`, catalog of `RecordingEntry[]`).
Two profiles: `runner_debug` (curated topic set) and `everything` (all visible topics, `PROFILES['everything']
= None`). Names are safe basenames; existing paths are never overwritten; active bags cannot be deleted.
Bags land under `/home/matti/runner_ws/bags`. `d060fb8` extended the debug profile to also record RF2O
odometry for offline analysis (§17).

## 15. Networking/deployment

Current, deployed (`network/README.md`, `aa63bbe`): the default field connection is NetworkManager
profile `runner-field-ap` — WPA2-only 2.4 GHz AP "Runner-Paddock" on channel 6, Runner fixed at
`10.42.0.1/24` via NetworkManager `shared` IPv4 (local DHCP/DNS). Paddock is reachable at
`http://10.42.0.1:8000/` or, where mDNS resolves, `http://makro-runner.local:8000/`. A captive-portal
service on port 80 answers OS connectivity probes with a landing page whose **OPEN PADDOCK** link
opens port 8000 in a normal browser window; nothing proxies or redirects Paddock's API or same-origin
`/ws` traffic. `network/install.sh` layers a NetworkManager renderer onto the existing
netplan/systemd-networkd host without deleting existing client profiles or Tailscale config;
`--check` reports drift, `--activate` switches immediately. Manual mode-switching via `nmcli` is
documented and reversible; AP autoconnect priority restores field mode after reboot regardless.
Tailscale Serve remains a tailnet-only (never Funnel) path for non-field use, proxying the same local
port, independent of and unaffected by `runner-paddock-web.service` restarts.

## 16. Supported operator vs Advanced/Engineering configuration

**Supported operator surface (Paddock UI):**

- Runtime selection (IDLE/MAPPING/AUTONOMY), manual driving (bounded speed/steering), RUN/STOP/CLEAR
  STOP, mapping NEW/SAVE/SELECT/DELETE map, Initial Pose (stopped/stationary, matching selected map),
  goal/mission selection and RUN-to-drive, recording start/stop/delete with named profile, and the
  Timid/Confident speed-preset picker with a truthful Custom readout.
- `manual_max_speed_mps`: 0 or `[0.25, 0.40]`, default 0.40 — session-scoped operational override,
  unchanged from v1.3.

**Advanced/Engineering surface (still Paddock-served, but explicitly gated as such):**

- The full live autonomy-tuning field set (§12): all twelve `desired_linear_vel` /
  `regulated_linear_scaling_*` / `cost_scaling_*` / lookahead / `proportional_gain` / `integral_gain`
  / feedforward / `output_max` fields, individually. Timid/Confident remain the supported presets;
  direct field editing is intentionally the engineering path, bounded by `validate_values()`'s
  cross-field checks and the hard `output_max ≤ 0.14` ceiling.
- Global obstacle-layer enable/disable and clear, currently reachable only via the legacy keyboard
  path (§13), not yet a first-class Paddock control.
- SSH/config files/direct ROS parameter access for anything outside the above remains
  engineering-only, as in v1.3.

## 17. Known limitations/open engineering work

- **Motion deadman timing is provisional, not final safety policy.** The motor watchdog is a fixed,
  unchanged `CMD_TIMEOUT_S = 0.2` s (§2). The Paddock-side chain around it — 0.5 s
  `lease_timeout_sec`, 0.15 s `raw_autonomy_timeout_sec`, 0.30 s mux autonomy/manual timeouts, 0.15 s
  mux teleop timeout, 0.10/0.15 s STOP timeouts — is deployed and functioning, but the end-to-end
  stale-command budget across all these serial stages has still not been measured on hardware, exactly
  as v1.3 §11 required and left open. `lease_timeout_sec: 0.5` s is explicitly called out in-source
  as "a first-integration Wi-Fi value to be measured and tightened before traction"
  (`docs/paddock_v1.3_implementation.md`, Stage 7). **Do not treat 0.5 s, or any of the numbers above,
  as a ratified final safety bound** — they are the current deployed configuration, pending a
  measured end-to-end validation.
- **Keyboard bridge is legacy and unremoved** (§4.5): still capable of local motion and of the only
  live obstacle-layer toggle path; a decommission plan needs to (a) port obstacle-layer control to
  Paddock/Foxglove, (b) confirm no other engineering workflow depends on the UDP keyboard sender
  (`tools/keyboard_sender.py`), then (c) remove `keyboard_bridge` from `teleop.launch.py` and retire
  `KeyboardState`.
- **Hardware/integration validation gaps carried forward from v1.3**, not yet closed as of HEAD per
  `docs/paddock_v1.3_implementation.md`: a real end-to-end held-RUN autonomous drive, a live
  `bt_navigator` cancel exercising the delayed-old-`Twist`-across-cancellation quiescence acceptance,
  DualSense takeover mid-mission, and global STOP during motion are all still Matti's pending
  integration tests, not yet exercised against a live, coherently-deployed stack.
- **Deploy coherence is not self-verifying.** `services/install.sh --check` must be run and
  reconciled after any change; a recent validation pass found `runner-map-executor` on the deployed Pi
  serving an older message schema than HEAD. Treat "HEAD says X" and "the Pi is doing X" as separate
  claims until a coherent redeploy is confirmed.
- **RF2O longitudinal profiling is analysis tooling, not a retuning program.** `tools/analyze_longitudinal_profile.py`
  (`6c4dfb2`, `87f5f8a`) is an offline bag analyzer over `/cmd_vel`, `/wheel/odom`, `/odom_rf2o` that
  segments fixed-throttle command steps and measures settling/steady-state RF2O speed per commanded
  step (`COMMAND_TOLERANCE`, `MIN_STEADY_TAIL_S`, `MAX_STEADY_SLOPE_MPS2` etc.). It informed the
  Confident-preset command range but is not itself a live control-loop and implies no ongoing
  auto-retuning commitment.
- **CPU performance** has surfaced as an observed current engineering limitation during recent
  development sessions (readiness-loop scheduling headroom, structural-check cadence) but is recorded
  here only as an open operational note, not as architectural doctrine dictating any component's
  design.

## 18. Future exploration and semantic traversability

Unchanged from v1.3 §15, still entirely future work, not started at HEAD: supervised frontier
exploration as a future `MAPPING + AUTONOMOUS` capability consuming geometric map/state through the
same single navigation runtime and the same RUN/STOP/takeover gates; a later Nav2-on-mapping-raster
application composition that must not reuse `nav2.launch.py` wholesale; and a separate future
semantic terrain/traversability layer (class, confidence, timestamp, source, map/session identity)
that must not repurpose geometric SLAM occupancy and must not itself authorize motion. No negative-
obstacle sensor exists or is a prerequisite for the first supervised exploration version, per the
existing ratified direction.

## 19. Decision/spec reconciliation against v1.3

| v1.3 clause | v1.4 disposition |
|---|---|
| §1 "transitional graph," adapter/authority both writing `/cmd_vel_auto`, web read-only | Superseded: the full production path (§1, §7) is live; web is bidirectional and is the primary operator interface |
| §3 "first supported browser release must complete..." milestone | Achieved and substantially exceeded: mapping, missions, recording, live tuning, Initial Pose, map delete are all live |
| §5 "Ratified packaging (§18-Q1)... exact priority proposal: autonomy 50, manual 75, DualSense 100, lock 200, stop-zero 255" | Confirmed deployed exactly as proposed (§4.2) |
| §9 "manual maximum ... autonomy maximum remains 0.60 m/s" (Q3) | **Explicitly superseded**: Confident preset's 1.00 m/s desired/maximum ratified and shipped (§12); manual ceiling (0.40 m/s) unchanged |
| §9 "the frozen controller is retained," PI/feedforward/output-cap immutable | Superseded in mechanism, preserved in value: the same numeric constants are now the Timid preset and the committed default under a live-tuning mechanism (§4.4, §12), not an immutable build-time constant |
| §10 real navigation mission lifecycle table (9 states incl. `NO_MISSION`/`SELECTED_VALIDATED`/`NAV2_ACCEPTED`/`SUPERSEDED`) | Superseded by the actual shipped 7-state `NavigationState` enum (§11); the richer v1.3 table described intent, not the interface that was built |
| §13.1 NEW MAP "Require STOP and measured stationarity" | **Explicitly superseded** by `044b4fb`: NEW MAP uses authority revocation + post-revocation stationary evidence, not global STOP (§8) |
| §13.2 SAVE MAP "Require STOP/stationarity" | Unchanged/confirmed current |
| §14 supported-configuration table (static bounds, no live tuning, no DELETE) | Superseded by §12 (live tuning) and §8 (DELETE); manual/mapping-ceiling numbers unchanged |
| §16 "the frozen controller is retained" (table row) | Superseded per above |
| §2/§16 keyboard "Retired production authority" framing | Corrected: not actually retired in source at HEAD; documented here as legacy pending removal (§4.5), not re-ratified as retired |
| §17 gated migration plan (stages 0–9) | Stages 0–8 landed; see §20 for the historical record. Stage 9 (supervised exploration) remains future (§18) |
| §11 end-to-end stale-command budget | Still open (§17), as v1.3 left it |

## 20. Historical appendix — completed v1.3 migration stages

For traceability only; current behavior is defined by §1–§19 above, not by this appendix.

| Stage | Subject | Outcome |
|---|---|---|
| 0 | Ratify contract | `docs/decision_v1.3_paddock_first.md`, 7 Sep 2026 |
| 1 | Isolate unfinished authority scaffold | Private non-mux topics; smoke-checked |
| 2 | Build authority/local/STOP contracts privately | Prototype gate not passed on first attempt (`docs/paddock_v1.3_stage2_gate_report.md`); resolved before Stage 3 |
| 3 | Persistent local-control cutover | `382e509`; one joy/teleop/mux/STOP-enforcer set outside application launches |
| 4 | Truthful runtime and map execution | `3b4f5fc`; runtime epoch, mapping session id, continuous readiness, NEW MAP/SAVE MAP/catalog |
| 5 | Real navigation runtime | `8f0cd42`; `foxglove_goal_bridge` retired, `runner_navigation_runtime` sole Nav2 client owner |
| 7 (Part A) | Autonomy authority/velocity cutover | `9bdf7a2`; full `/cmd_vel_auto_raw → authority → /cmd_vel_auto → mux` path wired |
| 8 (Part B) | Minimal Paddock operator UI | `1265ae0`; bidirectional `/ws`, lease/RUN/STOP/mapping/mission browser control |
| — (Part C) | Coherent systemd deploy tooling | `6aa60af`; `services/install.sh` single source of truth |
| — | Post-deploy fixes | `3a0fb8d`, `98106aa`; stale-asset caching and `mode_state` liveness fixed |
| — | Subsequent hardening (post-v1.3-ratification, pre-v1.4) | Obstacle-aware costmaps, browser drive/recording/tuning polish, RF2O profiling tooling, mapping-reset quiescence (`044b4fb`) — see `git log 6a7c9611..044b4fb` |

Stage 6 (remote manual conversion) and Stage 9 (supervised exploration) from v1.3's plan: Stage 6's
manual conversion is live (§4.4, §6); Stage 9 remains future work (§18), not renumbered or reused
here.
