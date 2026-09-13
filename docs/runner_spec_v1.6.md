# Runner — Architecture & Current-State Specification v1.6

**Current-state specification · updated 13 September 2026**

Baseline: `/home/matti/runner_ws`, clean HEAD
`02b00c54fccddc4226cd32fcda23ffcc040a8c86` ("Reduce Paddock status wake
traffic"), inspected directly from source, tests, and configuration.
Previous specification: `docs/runner_spec_v1.5.md` (baseline `4366b52`,
itself amended in place by `de8de9c` to add the Insane preset and D-88 —
that amendment is already fully reflected in the v1.5 text this document
carries forward and is not re-described here). This document reconciles
v1.5 against fourteen further commits landed after `de8de9c`: `06eab5d`
(fix Paddock mobile control and STOP hold), `52ad42a` (explicit Paddock
control takeover), `6a6c452` (confirm initial pose from map-frame TF),
`5dfff73` (make RELEASE STOP a click action), `4c5ed3a` (make takeover a
one-click action), `a136eb4` (remember Paddock viewport per map),
`26317d4` (reduce Paddock QoS event overhead), `0d96b3b` (make Paddock
visualization subscriptions demand-driven), `b939cd1` (expose Nav2 errors
and control telemetry), `e8285b3` (add navigation debug recording
profile), `4b6d850` (fix atomic autonomy preset preflight bounds),
`1aa4f64` (avoid active recording catalog rescans), `ab78879` (reduce
low-risk ROS wake rates), and `02b00c5` (reduce Paddock status wake
traffic) — plus one non-code investigation: a deep, read-only forensic
analysis of Runner's global-plan-replacement and `NO_VALID_CONTROL`
failures (bag `Runner_20260913_184429_0.mcap`, captured with the new
`navigation_debug` profile from `e8285b3`), followed by a target-architecture
proposal and a final ratification pass. That investigation changed **zero**
navigation source or configuration; it produced a ratified target
architecture and two newly adopted decisions (§19).

This round is two largely independent tracks: **§18** is a mixed
operator-UX/protocol-efficiency engineering pass (the fourteen commits
above) — control-takeover hardening, Nav2 error visibility, a new
recording profile, and several wake-rate reductions, none of which change
the control or navigation architecture described in §1–§16. **§19** is the
navigation-architecture investigation and ratification — it changes no
current-state fact in §1–§16 (the BT, planner, and controller are byte-for-
byte what v1.5 described) but establishes a ratified *target* architecture
("Candidate B") and adopts two decisions (D-89, D-90) that future
implementation work is expected to follow. §1–§17 restate v1.5 as current,
with in-place updates only where source/config actually changed (marked
inline); §18–§19 are new; former §18–§21 are renumbered §20–§23 with
updates.

**Reading convention** (unchanged from v1.5). This document states what is
implemented in source and covered by tests as **current**, distinguishes
what is deployed but not yet hardware-validated or time-bounded as
**provisional/open**, and reserves **future** for capability that does not
exist yet. This version adds a fourth category where it matters: **ratified
architecture, not yet implemented** — a decision Matti has approved as the
target design, with source unchanged at this baseline. §19 uses this
category throughout for the Candidate-B stages; nowhere in this document is
planned or ratified-but-unbuilt work described as current. Historical
D-numbers are always qualified by source version and subject. **D-89 and
D-90 are ratified as of this document; D-91 through D-95 are anticipated
decision subjects, provisionally labelled for planning convenience only —
those identifiers are not reserved or final until each is actually
ratified** — do not cite them as binding until their own implementation
stage adopts them (§19). Where current code or a decision Matti has
ratified supersedes an older numeric limit or claim, this document states
the current value and does not carry the old one forward as if still
binding. **Deploy coherence is not assumed**: this document describes the
repository at HEAD, not necessarily the code the Pi's systemd units are
currently executing (§15, §20).

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
speed-policy tuning (Timid/Confident/Insane/Custom), complete map bundle
lifecycle (new/save/select/delete), MCAP recording, and a field Wi-Fi AP
remain real, tested, current behavior. **The Nav2 BT, planner, and
controller are unchanged from v1.5 at the source level** — this round's
navigation work is a forensic investigation and architecture ratification
(§19), not an implementation. What changed since v1.5 in implemented state:

- **Paddock control UX was hardened** (§18.1): STOP assertion, STOP
  release, and observer takeover are all plain single-click actions.
  A 2-second hold-to-confirm gesture was built for release and for
  takeover, then removed the same day in favor of a click — hold-to-confirm
  is not part of the current UI. A new **explicit takeover** action lets an
  observer forcibly take the control lease from whoever currently holds it,
  in one click, without the current holder's cooperation. Per-map browser
  viewport (pan/zoom) is now remembered in `localStorage`.
- **Initial Pose confirmation now uses direct map-frame TF**, not a
  slam_toolbox pose topic (§9, §18.2) — this closes one of v1.5 §9's two
  open operator-observed discrepancies (the "no fresh slam_toolbox pose"
  report class), though it has not yet been field-revalidated against a
  live recurrence.
- **Nav2 controller/planner error codes are now fully decoded** to
  symbolic names (previously only `NONE`/`UNKNOWN` were distinguishable)
  and surfaced to the operator (§11, §18.2). A new `navigation_debug`
  recording profile (§14, §18.2) captures the full navigation-relevant
  topic set. This pairing — real error codes plus a complete recording —
  is what made this round's navigation-architecture investigation (§19)
  possible.
- **Further wake-rate and protocol-overhead reductions** (§18.3): explicit
  QoS event-handler suppression on high-churn Paddock nodes, demand-driven
  large-frame (map/costmap/plan) visualization streaming instead of
  unconditional push, several state-heartbeat publish periods reduced and
  duplicate-suppressed, and reduced Nav2/SLAM-toolbox internal publish/bond
  rates. **None of this is claimed to resolve** the still-open Paddock-web
  (~40% of one core) or `mode_supervisor` (~30% of one core) CPU questions
  from v1.5 §17 — these are separate, lower-risk efficiency changes kept on
  their own merits.
- **A real cross-module validation bug was found and fixed the day after
  Insane was introduced** (§12, §18.4): `drive_adapter`'s own preflight
  check still used the pre-Insane 0.14 output ceiling instead of the
  shared 0.30 experimental ceiling, which could fail an atomic Insane-preset
  application inside its own validation step. Both validators now share one
  constant source (`AutonomyTuningPolicy.msg`).
- **A deep, read-only forensic investigation of a `navigation_debug`
  recording** (`Runner_20260913_184429_0.mcap`: 33 Nav2 attempts, 24
  `NO_VALID_CONTROL` failures, 5 successes, 4 cancellations) traced the
  root causes of Runner's repeated global-plan replacement and controller
  aborts against upstream Nav2 Jazzy source and a faithful replay of every
  failure tick (§19). It found **three mechanistically distinct failure
  clusters**, not one bug, and led to a ratified target navigation
  architecture — a "committed-path Ackermann stack" ("Candidate B"). **D-89
  and D-90 are adopted now**; D-91 through D-95 are anticipated decision
  subjects, provisionally labelled for planning convenience and **not
  reserved or final until ratified**; **no navigation source or
  configuration has changed** as a result — Stage A1 (obstacle-evidence
  coherence) is the next implementation task, not yet started.

The known open items, updated from v1.5 in §20, include everything carried
forward from that section plus: Stage A1 of the navigation architecture is
not started; the initial-pose TF-confirmation fix (above) is source-level
only, not yet field-revalidated; and the new one-click takeover gives any
observer the ability to end another operator's control session without
warning them first — a UX/safety-review item, not yet flagged as a defect.

## 2. Platform, geometry, hardware and safety boundaries

Unchanged from v1.5. Runner is the LaTrax Prerunner research platform:
Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy, LD19 lidar, BNO085 IMU,
hall-effect wheel encoder, and a Cytron MD13S motor driver. Phase 1
(indoor/outdoor navigation) remains the established platform focus.

Geometry is unchanged from v1.2–v1.5: wheelbase 0.178 m, maximum steering
0.3614 rad, physical minimum turning radius 0.4709 m
(`maximum_curvature = tan(0.3614)/0.178 ≈ 2.1236` 1/m, re-derived and
confirmed live from `/rosout` in §19's investigation — unchanged from the
prior figure). The planner's minimum turning radius is **0.60 m**
(`src/runner_bringup/config/nav2_params.yaml`, `SmacPlannerHybrid.
minimum_turning_radius`) — a planning-time conservatism margin over the
physical minimum, not a claim that the vehicle cannot turn tighter. §19
notes a third, different-purpose radius (RPP's `regulated_linear_scaling_
min_radius`, 0.75 m, a speed-regulation threshold, not a hard limit).
**These three values' roles are distinct and understood** — physical hard
minimum, planning-feasibility margin, and speed-regulation threshold are
three different purposes, not three competing claims about the same
quantity — but **whether their numerical margins are mutually appropriate
for the eventual Stage-D/E execution contract remains open** (§19.4).

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
| `/map` | MAPPING: `slam_toolbox`. AUTONOMY: `map_server`. Enforced per-topic by `runner_mode_supervisor._ownership_ready`, not by node-name counting |
| `/cmd_vel` | `twist_mux`, sole writer, unchanged |

TF is a multi-publisher transport with **single ownership per edge**, not a
single-writer topic; the mode supervisor checks publisher-owner sets per
critical topic/edge every readiness cycle and ignores endpoints whose node
identity has not yet propagated through DDS discovery — carrying forward
v1.3 §12's ruling that a duplicate node *name* is not proof of a duplicate
owner.

Retained hard safety limitations, unchanged by anything in this document:

- The planar LD19 scan cannot see descending edges or low obstacles;
  supervised autonomous/manual driving is not gated on a negative-obstacle
  sensor. §19.4 records this as a standing sensing limitation the ratified
  navigation architecture does not claim to solve.
- Paddock/global STOP is a software motion inhibit, not proof of mechanical
  stationarity or independent power isolation.
- The known motor SIGKILL/PWM-peripheral hazard and deferred
  heartbeat-gated-FET decision (v1.2 subject, historically discussed as
  **D-82**) remain open; nothing in this architecture claims to close them.
  `runner_stop_enforcer` controls the existing mux/lock only.

The Pi's measured 2.8 GHz core clock (`arm_freq=2800`, above the Pi 5's
2.4 GHz stock clock) and the reasoning for reading v1.5 §17's CPU findings
against that ceiling are unchanged and carried forward without repetition
here; see v1.5 §2 for the full note.

## 3. Process tiers and ownership model

Unchanged from v1.5: the hardware tier, persistent local-control tier
(`keyboard_bridge` remains removed, `a484be4`), persistent STOP tier,
persistent operator tier (command authority, drive adapter, mode
supervisor, map executor, recording executor, Paddock web), and
application tier (`runner-mode-mapping`/`runner-mode-autonomy`) are exactly
as v1.5 §3 describes. No commit in this round adds, removes, or changes the
ownership of any process tier, service, or `RuntimeDirectory`.

## 4. Paddock operator surfaces and control roles

**Lease model** is unchanged in its base semantics (§6 for the new
takeover capability). **Console layout** is unchanged in structure.

Operator-facing actions accepted by the gateway now include one addition:
`acquire` / `release` / `heartbeat`, `run` (hold-to-run), `stop` /
`clear_stop`, **`takeover`** (new, §6, §18.1), `manual` (joystick demand),
`select_mode`, `new_map` / `save_map` / `select_map` / `delete_map`,
`select_goal` / `set_initial_pose`, `set_config` (manual speed ceiling),
`set_autonomy_tuning` (preset or field values), `clear_obstacles` /
`set_obstacle_processing`, `start_recording` / `stop_recording` /
`delete_recording`. `takeover` is available only to an **observer**
connection when the lease is currently held by someone else
(`gw.lease_held === true` and the requester is not the controller); it is
rejected (as a no-op success) if the requester already holds the lease, and
treated as an ordinary `acquire` if no one holds it (`gateway.py:255-278`).
Every handler still validates ownership and finiteness before producing a
typed intent.

Separately, the browser now sends one non-gateway WebSocket message,
`visualization_demand` (§18.3), to tell the server which large frames
(`map`/`global_costmap`/`local_costmap`/`plan`) it currently wants pushed —
this is transport-level stream control, not an `OperatorGateway` action,
and does not go through `_do_*`/ownership checks (any connected client,
controller or observer, may declare its own visualization demand).

**Other surfaces table** unchanged from v1.5.

## 5. Runtime modes and lifecycle

Unchanged from v1.5: `ModeState` publishes `mode`, `status`
(`STATUS_STABLE / STATUS_TRANSITIONING / STATUS_FAULT`),
`accepted_request_id`, `active_autonomy_map`, `detail`, a monotonic
`runtime_epoch`, a per-session `mapping_session_id`, and continuously
refreshed `ready` / `readiness_reason`, capability-specific and never a
bare "process present" check (structural, MAPPING-capability, and
AUTONOMY-capability readiness exactly as v1.5 §5 describes). Transition,
`reconcile()`, and NEW MAP's quiescence-barrier semantics are all
unchanged. No commit in this round touches `mode_runtime.py`'s transition,
reconciliation, or readiness logic — §18.3's QoS-event and wake-rate
changes touch `mode_supervisor_node.py`'s *node base class* and heartbeat
cadence only, not its readiness contract or semantics.

## 6. Motion authority, STOP, lease, RUN and takeover

Unchanged from v1.5 except one addition and one UI simplification:

**Explicit takeover** (`OperatorGateway._do_takeover`, `gateway.py:255-278`,
`52ad42a`): a non-owning connection can now force a lease transfer while a
lease is held, without the current holder releasing it first. Implemented
as one control-supervisor transition through a new `ControlEvent.LEASE_LOST`
(mapped to `Event.LOSE_LEASE`, `command_supervisor.py:506-509`) —
forcibly clearing the previous owner's connection id, client id, lease id,
RUN latch, and manual-active state — immediately followed by an ordinary
`LEASE_ACQUIRED` for the requesting connection. **This is a new capability
beyond v1.5's lease model**: previously only the current owner could
relinquish control (via `release` or by going silent past the liveness
timeout, §6). The previous owner receives no warning before losing
control mid-session; the UI surfaces a "Take over" button to any observer
whenever a lease is held by someone else, and the action completes on a
single click. Whether this should carry an owner-facing warning or a
takeover-cooldown is not addressed by this document (§20).

**STOP assertion, release, and the (now-removed) hold-to-confirm gesture**:
STOP assertion remains an immediate single click. STOP *release*
(`CLEAR STOP`, previously "hold `btn-clear-stop` for 2s") went through two
UI iterations in one day (`06eab5d` introduced a 2-second hold-to-confirm
gesture shared by both STOP-release and takeover via a new
`hold_to_confirm.js` helper; `5dfff73`/`4c5ed3a` removed the hold gesture
from both the same day, leaving a plain click). **Current UI: both STOP
release and takeover are single clicks; `hold_to_confirm.js` and its test
suite were deleted and are not part of the current codebase.** The
backend-level `EVENT_CLEAR_STOP` semantics (§6, ownership/sequence checks,
the enforcer's own clear preconditions) are unchanged by any of this — the
UI change affects only how many client-side interactions precede sending
the same control event.

## 7. Command/data-flow and exact topic/interface ownership

The ROS topic ownership table and diagram are unchanged from v1.5 — no
commit in this round adds, removes, or re-routes a ROS topic. Paddock's own
WebSocket protocol (not a ROS topic) gained the `takeover` gateway action
(§4, §6) and the `visualization_demand` stream-control message (§4, §18.3);
neither affects the ROS-level table below it.

| Topic/interface | Type | Sole writer → consumer(s) | Notes |
|---|---|---|---|
| `/paddock/control_event` | `PaddockControlEvent` | web gateway → authority | RUN/STOP/CLEAR/manual/goal/lease/heartbeat, ordered sequence per lease |
| `/paddock/control_lease` | `PaddockControlLease` | authority → executors/gateway | monotonic `generation` |
| `/paddock/command_authority_state` | `CommandAuthorityState` | authority → mode supervisor, gateway, map executor | full interlock/goal/STOP snapshot; heartbeat period 0.2 s (§18.3, was 0.1 s) |
| `/paddock/mode_request` | `ModeRequest` | web gateway or map executor → mode supervisor | `OP_SELECT_RUNTIME` / `OP_NEW_MAP` |
| `/paddock/mode_state` | `ModeState` | mode supervisor → authority, map executor, gateway, navigation runtime | epoch/session/readiness authoritative |
| `/paddock/map_request` → `/paddock/map_state` | `MapRequest`/`MapState` | web gateway → map executor | `OP_NEW_MAP`/`OP_SAVE_MAP`/`OP_SELECT_MAP`/`OP_DELETE_MAP`; heartbeat period 1.0 s and content-deduplicated (§18.3, was 0.5 s, no dedup) |
| `/paddock/navigation_request` → `/paddock/navigation_state` | `NavigationRequest`/`NavigationState` | authority → navigation runtime | `OP_SELECT`/`OP_DISPATCH`/`OP_CANCEL`; 7-state lifecycle; content-deduplicated between a 0.5 s forced heartbeat (§18.3); `error_meaning` now a full symbolic decode (§11, §18.2) |
| `/paddock/recording_request` → `/paddock/recording_state` | `RecordingRequest`/`RecordingState` | web gateway → recording executor | `OP_START`/`OP_STOP`/`OP_DELETE`; `OP_START` now accepts `navigation_debug` as a profile (§14) |
| `/paddock/config_request` → `/paddock/config_state` | `ConfigRequest`/`ConfigState` | web gateway → authority | one field, `manual_max_speed_mps`, revision-checked |
| `/paddock/stop_state`, `/paddock/internal/stop_request` | `StopState`/`StopRequest` | stop enforcer ↔ authority | boot-qualified idempotent requests |
| `/paddock/manual_demand` | `ManualDemand` | authority → drive_adapter | signed m/s + normalized steering, sequenced |
| `/cmd_vel_nav` | `Twist` | `controller_server` → drive_adapter | Nav2's SI output, unchanged |
| `/cmd_vel_auto_raw` | `Twist` | drive_adapter → authority | raw converted autonomy command |
| `/cmd_vel_paddock_manual_raw` | `ConvertedCommand` | drive_adapter → authority | raw converted manual command + provenance |
| `/cmd_vel_auto` (p50), `/cmd_vel_paddock` (p75) | `Twist` | authority → twist_mux | supervised, silent when not permitted |
| `/cmd_vel_teleop` (p100) | `Twist` | runner_teleop → twist_mux | local DualSense command |
| `/cmd_vel_stop` (p255), `/paddock/stop_lock` (lock 200) | `Twist`/`Bool` | stop_enforcer → twist_mux | zero + fail-closed lock heartbeat |
| `/cmd_vel` | `Twist` | twist_mux → motor_node | final command, sole writer |
| `/teleop/control_state` | `LocalControlState` | runner_teleop → authority, stop_enforcer, gateway | process/takeover epoch, neutral/released |
| `/wheel/encoder_state` | `EncoderState` | encoder → motor, adapter, EKF, stop_enforcer, mode supervisor | stationary/direction evidence |
| `/drive_adapter/state`, `/drive_adapter/state_typed` | `String`/`AdapterState` | drive_adapter → gateway/diagnostics | full PI/feedforward/integrator diagnostics |
| `/initialpose` | `PoseWithCovarianceStamped` | Paddock web (controlled) → slam_toolbox | STOP+stationary+matching-map gated; confirmation source changed, §9 |

Every raw/supervised command schema carries finite values and identity
fields sufficient to reject stale or cross-epoch delivery. Command QoS is
volatile depth-10 or depth-1 TRANSIENT_LOCAL for status; each topic above
has exactly one writer at any time. Rows unchanged from v1.5 except the
three annotated above (heartbeat/dedup and profile additions from §18); no
row is added, removed, or re-routed.

## 8. Mapping sessions, map bundles, NEW/SAVE/SELECT/DELETE

Unchanged from v1.5: a mapping session's phase machine
(`NONE→STARTING→READY→SAVING→SAVED`/`FAILED`), NEW MAP's quiescence path
(§5), SAVE MAP's transactional four-artifact bundle write (posegraph/data,
YAML, raster, manifest with SHA-256 digests and a 12-hex revision hash),
the catalog's cheap re-validation, SELECT MAP's live-swap rejection, and
DELETE MAP's tombstone-then-rollback deletion are all exactly as v1.5 §8
describes. No commit in this round touches `map_session.py`'s
transactional save/delete/catalog logic; `map_session_node.py`'s only
change this round is the heartbeat-dedup/period change described in
§18.3, which does not alter any of §8's semantics.

## 9. Initial Pose workflow

Unchanged through the STOP-then-pose-then-clear sequence and its
preconditions/timeouts (5 s / 5 s / 2 s), with one substantive change to
**how pose confirmation is evidenced**:

**Confirmation now uses direct map-frame TF, not a slam_toolbox pose topic**
(`6a6c452`). Previously, a dedicated subscription to `/pose`
(`LOCALIZER_POSE_TOPIC`, a `PoseWithCovarianceStamped` slam_toolbox
publishes) was the only path that could advance an `awaiting_pose` phase.
That subscription and its handler (`_on_localizer_pose`) are **removed
entirely**. In their place, `_confirm_initial_pose_from_tf`
(`ros_state_node.py`) runs on every `map→base_link` TF sample the node
already receives for other purposes (§10, and for AUTONOMY readiness,
§5): once a pose request is `awaiting_pose`, each fresh TF sample is
compared against the requested pose intent, and the phase advances
(subject to the same context-changed/safety-rejection checks as before)
once the TF's position is within **0.25 m** and yaw within **15°**
(`INITIAL_POSE_POSITION_TOLERANCE_M`, `INITIAL_POSE_YAW_TOLERANCE_RAD`) of
what was requested. The workflow no longer depends on slam_toolbox
publishing any topic beyond the `map→odom` TF it was already required to
publish. The confirmation-timeout detail string changed accordingly, from
*"application unconfirmed: no fresh slam_toolbox pose"* to *"application
unconfirmed: map-frame robot pose did not match the requested initial
pose."*

**This closes one of v1.5 §9's two open operator-observed discrepancies**:
the "no fresh slam_toolbox pose" report class is structurally gone, since
there is no separate slam_toolbox pose message this step can now fail to
receive. **This is a source-level fix, not yet field-revalidated** against
a live recurrence of the original symptom — treat it as closing the
mechanism, pending operational confirmation (§20). **The other v1.5
discrepancy is untouched and remains open**: obstacle/costmap content was
observed to apparently survive an Initial Pose transaction despite the
`ClearEntireCostmap` phase (unchanged, still described accurately above)
reporting both clears succeeded.

The costmap-clear phase, its explicit non-clearing of global STOP, and all
other v1.5 §9 content are otherwise unchanged.

## 10. Localization, estimation and TF ownership

Unchanged from v1.2–v1.5 (**D-37**, "fixed-cardinality SLAM scan stream",
still current): the LD19 raw `/scan` branches into
`rf2o_scan_canonicalizer → /scan_rf2o → RF2O` (feeding EKF) and
`scan_rebinner → /scan_slam` (feeding slam_toolbox, fixed 503-bin
geometry). `ekf_node` fuses IMU + encoder + RF2O into `odom → base_link`.
`slam_toolbox` owns `map → odom` — the mapping instance in MAPPING, the
localization instance (remapped `/slam_map` diagnostic) in AUTONOMY. Static
extrinsics are unchanged existing publishers. See §2's ownership table for
the authoritative per-edge/per-topic rule.

This section is now load-bearing for one more consumer than in v1.5: §9's
initial-pose confirmation reads the same `map→base_link` TF this section
already describes `slam_toolbox` as publishing, rather than a separate
topic — see §9 for the current Initial Pose write/confirm path into this
localizer.

## 11. Nav2, mission lifecycle and reverse autonomy

The 7-state mission lifecycle, generation discipline (`boot_id`,
`runtime_epoch`/`map_id`, `mission_revision`, `action_generation`),
cancellation/quiescence invariant, and reverse-autonomy configuration
(`REEDS_SHEPP`, `allow_reversing: true`, `smooth_path: false`) are all
**unchanged at the source level** from v1.5 — no commit in this round
touches the BT XML, `nav2_params.yaml`'s planner/controller plugin
selection, or `navigation_runtime.py`'s dispatch/cancel/generation logic.

**Nav2 error decoding was substantially improved** (`b939cd1`).
Previously, `NAV2_ERROR_NAMES` mapped only `{0: NONE, 1: UNKNOWN}` — every
actual Nav2 failure code (`TF_ERROR`, `NO_VALID_CONTROL`, `NO_VALID_PATH`,
etc.) surfaced to Paddock as bare `UNKNOWN`. `_nav2_error_names()`
(`navigation_runtime.py`) now builds a full symbolic table from the
generated `FollowPath.Result` and `ComputePathToPose.Result` message
constants, and `nav2_error_detail()` renders a stable `NAME (code)` string,
appending Nav2's own free-text detail when it adds information beyond the
name. This is surfaced to the operator (`app.js`, `index.html`) and was the
direct enabler of precise, code-level failure classification in this
round's navigation-architecture investigation (§19) — before this change,
a `106` (`NO_VALID_CONTROL`) and a `102` (`TF_ERROR`) were indistinguishable
in Paddock's own recorded state.

**The ratified target navigation architecture (§19) supersedes this
section's BT/replanning/recovery description as intended long-term
design — it does not change what is true today.** As implemented at this
document's baseline, the BT (`navigate_to_pose_forward_only.xml`) still
replans purely on `IsPathValid`/`GlobalUpdatedGoal` at ≤3 Hz with no
recovery subtree, and `IsPathValid` is served by `planner_server` against
the global costmap with no window or persistence (§19 traces this from
upstream source) — none of that has changed in this round; this section's
existing description of "forward only" naming the recovery posture, not a
reverse restriction, remains accurate. §19 records why that BT/replanning
design is no longer the intended end state and what a ratified replacement
(coherent obstacle-evidence semantics, committed-path replanning, a
recovery ladder, a physically-certified path contract, and a
controller-agnostic tracker contract) requires. **Treat this section as
the accurate current implementation and §19 as the ratified target, not yet
built** — Stage A1 has not started.

## 12. Longitudinal control and Timid/Confident/Insane/Custom speed policy

`drive_adapter` (`runner_drive_adapter`) remains the one shared,
persistent, closed-loop conversion for both Nav2's SI `/cmd_vel_nav` and
authority-bounded manual demand (`/paddock/manual_demand`), publishing raw
`/cmd_vel_auto_raw` and `/cmd_vel_paddock_manual_raw` respectively for the
authority to supervise. A defined subset of parameters is validated,
atomically applied, and live-effective without a process restart
(`add_on_set_parameters_callback`).

**Live-tunable parameters**: `maximum_commanded_speed`,
`feedforward_effort_per_speed`, `feedforward_effort_intercept`,
`output_max`, `proportional_gain`, `integral_gain`. Any other adapter
parameter (wheelbase, `max_steering_angle`, `minimum_moving_speed`,
`integrator_bound`, `output_min`, encoder/wheelspin/timeout thresholds) is
rejected as not live-tunable and requires a process restart.

**Timid / Confident are the two normal operator presets; Insane is a
third, explicitly experimental preset.** They are applied atomically
across both owners (`controller_server` and `drive_adapter`) via
`SetParametersAtomically`, then confirmed by an independent
`GetParameters` read-back before being reported `applied`. Timid and
Confident are unchanged since v1.4; Insane copies every Confident value
except three explicitly experimental overrides:

| Field | Timid | Confident | Insane (experimental) |
|---|---:|---:|---:|
| `desired_linear_vel` (m/s) | 0.45 | **1.00** | **1.50** |
| `maximum_commanded_speed` (m/s) | 0.60 | **1.00** | **1.50** |
| `regulated_linear_scaling_min_speed` (m/s) | 0.30 | 0.40 | 0.40 |
| `cost_scaling_dist` (m) | 0.45 | 0.60 | 0.60 |
| `cost_scaling_gain` | 1.0 | 1.0 | 1.0 |
| `regulated_linear_scaling_min_radius` (m) | 0.75 | 0.75 | 0.75 |
| `min_lookahead_dist` / `max_lookahead_dist` (m) | 0.30 / 0.80 | 0.30 / 0.80 | 0.30 / 0.80 |
| `lookahead_time` (s) | 1.0 | 1.0 | 1.0 |
| `max_allowed_time_to_collision_up_to_carrot` (s) | 0.15 | 0.60 | 0.60 |
| `proportional_gain` / `integral_gain` | 0.05 / 0.01 | 0.05 / 0.01 | 0.05 / 0.01 |
| `feedforward_effort_per_speed` / `_intercept` | 0.1188 / 0.0174 | 0.1188 / 0.0174 | 0.1188 / 0.0174 |
| `output_max` | 0.14 | 0.14 | **0.22** |

**Confident's 1.00 m/s intentionally supersedes v1.3's 0.60 m/s autonomy
ceiling** by explicit operator decision. The characterized feedforward at
1.00 m/s (0.1188×1.00+0.0174 ≈ 0.136) stays under the unchanged
`output_max` ceiling of 0.14; Confident does not raise the normalized-effort
safety ceiling itself.

**D-88 — experimental speed/effort ceilings.** Insane deliberately
supersedes **D-84**'s and the earlier v1.5 policy's universal
`output_max ≤ 0.14` rule for this experimental profile only. Timid and
Confident retain their established target speeds and `output_max = 0.14`;
they are not retuned. Paddock permits an absolute `maximum_commanded_speed`
of 2.0 m/s and an absolute `output_max` of 0.30 for experimental tuning —
validation ceilings, not defaults, commanded targets, or evidence that
2.0 m/s has been physically validated. `output_max` must still reach the
feedforward effort implied by `maximum_commanded_speed`; at Insane's
1.50 m/s target the unchanged feedforward requires 0.1956, covered by its
explicit 0.22 output limit.

Confident's larger `cost_scaling_dist` (0.60 m vs Timid's 0.45 m) and
larger `max_allowed_time_to_collision_up_to_carrot` (0.60 s vs 0.15 s) are
**more conservative near obstacles, not more permissive** — both widen
RPP's safety margin to compensate for the higher target speed (see v1.5
§12 for the full `costConstraint()`/`isCollisionImminent()` mechanism
walkthrough, unchanged and reconfirmed against source again during §19's
investigation).

**Custom** is `matching_preset()`'s truthful classification of the live
read-back whenever the ten controller fields plus five adapter fields do
not exactly match a named preset, reached only through collapsed
"Advanced speed policy"/"Engineering / controller" disclosures (§16).
`validate_values` enforces cross-field bounds on any custom write:
positivity, `cost_scaling_gain ≤ 1.0`, `regulated_linear_scaling_min_speed
≤ desired_linear_vel ≤ maximum_commanded_speed`, `min_lookahead_dist ≤
max_lookahead_dist`, non-negative feedforward across the command range,
`maximum_commanded_speed ≤ 2.0`, `output_max ≤ 0.30`, and `output_max`
must reach the maximum feedforward implied by the requested
`maximum_commanded_speed`.

Manual browser-speed bounds are unchanged: ceiling 0 (disabled) or
`[0.25, 0.40]` m/s, default 0.40, moving floor 0.25 m/s, configurable via
`manual_max_speed_mps`.

**A preflight-bounds cross-module drift bug was found and fixed the day
after Insane was introduced** (`4b6d850`). `drive_adapter.py`'s own
`AdapterConfig.__post_init__` validation had, until this fix, checked
`output_max` against a *module-local* constant `MAXIMUM_OUTPUT_AUTHORITY =
0.14` — the pre-Insane ceiling — while `autonomy_tuning.py`'s
`validate_values` already checked the same field against `0.30` (the D-88
experimental ceiling). Because both validators run inside the same atomic
set→verify transaction (§12 above), applying Insane's `output_max = 0.22`
could pass Paddock's own preflight check and then be independently
rejected by the adapter's own validation inside that same transaction —
a real defect, not merely a latent inconsistency, since Insane's own
committed value (0.22) sits between the two ceilings. The fix introduces
`runner_interfaces/msg/AutonomyTuningPolicy.msg`, used purely as a shared
constant carrier (`MAXIMUM_COMMANDED_SPEED=2.0`, `MAXIMUM_OUTPUT_
AUTHORITY=0.30`); both `drive_adapter.py` and `autonomy_tuning.py` now
reference these constants instead of each keeping an independent copy.
This closes a cross-module drift defect; it does not change either ceiling
value (both remain exactly as D-88 recorded them).

## 13. Obstacle-aware costmaps

Both Nav2 costmaps run a live-toggleable `nav2_costmap_2d::ObstacleLayer`
plus an `InflationLayer`:

- **Local** (`local_costmap`, odom frame, rolling 2×2 m window at 0.025 m
  resolution, `update_frequency 10 Hz`, `publish_frequency 3.0 Hz` —
  reduced from 5.0 Hz, §18.3): obstacle layer marks/clears from `/scan`
  (`obstacle_min_range 0.05 m`, `obstacle_max_range 1.0 m`,
  `raytrace_max_range 1.2 m`, heights `0.0–2.0 m`, `inf_is_valid: true`);
  inflation radius 0.45 m, cost-scaling factor 10.0.
- **Global** (`global_costmap`, map frame, static, `update_frequency 3.0 Hz`
  — reduced from 5.0 Hz, §18.3, 0.05 m resolution): `static_layer`
  (subscribes `/map`, transient-local, live updates) plus the same kind of
  obstacle layer for dynamic replanning.

**The global costmap's obstacle layer is configured
`combination_method: 0` (Overwrite)** — unchanged by this round's rate-only
edits to `nav2_params.yaml` (§18.3). Per §19's investigation, this is a
confirmed contributor to plan churn (live ray-trace clearing overwrites
static-map-derived lethal cells) and is targeted for change under the
ratified D-89 invariant; **it has not been changed at this document's
baseline** — Stage A1 has not started.

Both plugins' `obstacle_layer.enabled` boolean is a real, live ROS
parameter — Paddock's `set_obstacle_processing` action drives an atomic
set→verify and reports `applied`/`rejected`/`drifted` from the actual
read-back. `Clear transient obstacles` calls Nav2's own
`clear_entirely_global_costmap`/`clear_entirely_local_costmap` services; it
does not touch the saved static map. Otherwise unchanged from v1.4.

## 14. Recording/observability

Unchanged from v1.5's core mechanism (single Pi-side `RecordingExecutor`
owner, `ros2 bag record --storage mcap`, safe basenames, clean-then-escalated
stop, non-overwriting outputs), with two additions:

**A third recording profile, `navigation_debug`** (`e8285b3`), was added
alongside `runner_debug` and `everything`, specifically for navigation
diagnosis: `/scan`, `/local_costmap/costmap`, `/plan`,
`/odometry/filtered`, `/imu/data`, `/imu/read_errors`, `/tf`, `/tf_static`,
`/map`, the full `/cmd_vel*` chain (`/cmd_vel_nav`, `/cmd_vel_auto_raw`,
`/cmd_vel_auto`, `/cmd_vel`), `/drive_adapter/state_typed`,
`/speed_envelope/status`, the Paddock navigation/authority/lease/control/
stop/mode/map/config state topics, `/system/telemetry`, `/battery`, and
`/rosout` (`NAVIGATION_DEBUG_TOPICS`, `recording.py`). This is narrower
than `everything` but far more complete than `runner_debug` for navigation
analysis specifically — it is the profile that captured
`Runner_20260913_184429_0.mcap`, the bag behind §19's investigation.

**Active-recording catalog rescans were further narrowed** (`1aa4f64`):
the catalog's cheap-fingerprint comparison (v1.5's mtime/size check on
`metadata.yaml` and the Paddock manifest sidecar) now excludes the
*currently active* recording's own sidecar (`.paddock-recording.json`,
which is rewritten frequently while recording) from that comparison for
that one entry — only its `metadata.yaml` is checked while it is
starting/recording/stopping — since the sidecar's own churn was itself
making the catalog look "changed" on essentially every publish tick during
an active recording.

## 15. Networking/deployment

Unchanged from v1.5: the field Wi-Fi AP and captive portal
(`network/install.sh`, `runner-field-ap`, `10.42.0.1/24`), the intended
prefer-known-Wi-Fi-then-fall-back-to-AP operating policy (not automated by
the installer's committed AP-priority default), the Tailnet path, and
`services/install.sh`'s coherent-deploy tooling and restart ordering are
all exactly as v1.5 §15 describes. No commit in this round touches
networking or `services/install.sh`. **This round reinforces §15's
deploy-coherence caution further**: every config value touched by §18.3
(bond heartbeat periods, costmap update/publish rates, SLAM
`transform_publish_period`) requires the same `--check`/`--restart`-or-
reboot discipline as any other tracked config change — a Pi that has not
redeployed since `de8de9c` is still running the pre-round rates.

## 16. Supported operator vs Advanced/Engineering configuration

Unchanged from v1.5: the normal operator surface (runtime selection,
manual driving, map lifecycle, Timid/Confident presets, obstacle
processing, initial pose, recording, manual-speed ceiling, STOP/CLEAR
STOP) versus the Advanced/Engineering surface (per-field RPP tuning under
"Advanced speed policy", per-field longitudinal-controller tuning under
"Engineering / controller", both reached only through collapsed
disclosures in `CONFIGURE → AUTONOMY`) is exactly as v1.5 §16 describes.
Both write through the same validated, atomic, read-back-confirmed path as
the presets (§12); SSH/`ros2 param`/direct systemd management remain
engineering-only and outside Paddock's authorization model entirely.

## 17. Post-v1.4 investigations: CPU/performance, Paddock streaming, mode_supervisor, and the AUTONOMY map-pointer lifecycle bug

Unchanged from v1.5 — this section (and its subsections 17.1–17.4) is
carried forward verbatim as the historical record of that investigation
round. Nothing in this document's §18–§19 revisits or reopens any of its
findings except where explicitly noted (§18.3 makes clear its wake-rate
changes are *not* claimed to resolve the still-open Paddock-web/
mode_supervisor CPU questions this section left open).

## 18. Post-v1.5 engineering round: control UX, Nav2 error visibility, and wake-rate reduction

This section is new in v1.6. It covers the substance of the operator-UX and
protocol/wake-rate work in the fourteen commits between `de8de9c` and this
document's baseline, condensed here in one place; §4/§6/§9/§11/§12/§14
above already record the in-place current-state effect of each change.

### 18.1 Paddock control UX: takeover, one-click STOP, viewport memory

**Explicit takeover** (`52ad42a`, §4, §6) is the substantive new capability:
an observer can now force a lease transfer from whoever currently holds it,
in one action, without that holder's cooperation or prior release. This is
a real widening of who can end an active control session — previously the
owner had to `release` (or go silent past the liveness timeout) before
anyone else could `acquire`.

**Hold-to-confirm was built, then removed, for both STOP release and
takeover, in the same day's work** (`06eab5d` → `5dfff73`/`4c5ed3a`):
`hold_to_confirm.js` implemented a generic 2-second pointer/keyboard hold
gesture (with progress rendering, cancel-on-blur/visibility-change, and a
full test suite); it was wired to both actions, then removed from both the
same day in favor of a plain click, and the helper file plus its tests
were deleted outright. **The current UI has no hold-to-confirm gesture
anywhere**; both STOP release and takeover are single clicks, gated only
by the existing controller/availability disabled-state logic. This history
is recorded here because a future reader searching the repository for
"hold to confirm" will find only test/commit history, not live code.

**Per-map viewport memory** (`a136eb4`): the browser's pan/zoom state on
the map canvas is now saved to `localStorage` keyed by the selected map and
restored on return to that map — a pure client-side convenience with no
ROS or backend interface change.

### 18.2 Nav2 error visibility and the `navigation_debug` recording profile

Covered in full in §11 (error decoding) and §14 (recording profile).
Together, these two changes (`b939cd1`, `e8285b3`) are what made §19's
investigation possible: before them, a bag showed only `UNKNOWN` for every
Nav2 failure and lacked several of the topics (`/local_costmap/costmap`,
`/plan`, `/rosout` at debug-relevant scope, `/drive_adapter/state_typed`)
that investigation needed to trace a 106 to its exact mechanism.

### 18.3 Wake-rate and protocol-overhead reduction

Four independent, narrowly-scoped efficiency changes, none of which is
claimed to move v1.5 §17's still-open Paddock-web or mode_supervisor CPU
numbers — they were made on protocol/efficiency and readiness-scope
grounds, in the same spirit as v1.5 §17.2/§17.3's streaming and readiness
changes:

- **Explicit QoS event-handler suppression** (`26317d4`): a new
  `ExplicitQoSEventNode` (`qos_event_node.py`) overrides `create_subscription`/
  `create_publisher` to pass `use_default_callbacks=False` for QoS event
  callbacks unless a caller explicitly asks for them, avoiding rclpy's
  default warning-only event handlers on every subscription/publisher.
  `ModeSupervisorNode` and `RosStateNode` now inherit from it.
- **Demand-driven visualization streaming** (`0d96b3b`, §4): large periodic
  frames (`map`, `global_costmap`, `local_costmap`, `plan`) are no longer
  pushed to every connected browser unconditionally — `ClientHub` now
  tracks each client's `visualization_demand` (a `frozenset` of requested
  kinds, set via the new `visualization_demand` WebSocket message, §4) and
  only sends a kind to clients that have asked for it. The ~10 Hz small
  state-frame stream (pose, mode, authority, etc.) is unaffected — this
  only gates the large, optional visualization frames.
- **State-heartbeat rate/dedup changes** (`02b00c5`): `navigation_state`
  gained a duplicate-suppression key (`_last_state_key`) so unchanged
  publishes are skipped between an explicit 0.5 s forced heartbeat;
  `map_state` similarly gained content-based dedup (`_message_key`,
  ignoring the timestamp field) alongside a heartbeat period increase from
  0.5 s to 1.0 s; `command_authority_state`'s heartbeat period doubled from
  0.1 s to 0.2 s; `/speed_envelope/status`'s publish period doubled from
  1.0 s to 2.0 s.
- **Nav2/SLAM-toolbox internal rate reductions** (`ab78879`): both SLAM
  parameter files' `transform_publish_period` increased from 0.02 s to
  0.10 s; `nav2_params.yaml` gained `bond_heartbeat_period: 1.0` on
  `map_server`, `planner_server`, `controller_server`, and `bt_navigator`,
  plus `bt_loop_duration: 20` on `bt_navigator`; local-costmap
  `publish_frequency` dropped from 5.0 Hz to 3.0 Hz and global-costmap
  `update_frequency` from 5.0 Hz to 3.0 Hz. **None of these Nav2/costmap
  rate changes touch `combination_method`, resolution, inflation, or any
  other value §19's investigation examined** — they are publish/bond-cadence
  changes only, orthogonal to that investigation's findings, and Stage A1
  (§19) has not yet made any of the changes that investigation calls for.

### 18.4 Autonomy-tuning preflight bounds fix

Covered in full in §12.

## 19. Navigation architecture: forensic investigation and ratified target (Candidate B)

This section is new in v1.6. It summarizes a three-part, read-only
investigation conducted outside normal implementation work: (1) a forensic
analysis of one `navigation_debug` recording tracing every observed
`NO_VALID_CONTROL` (106) failure and plan-replacement event to upstream
Nav2 Jazzy source and replayed bag evidence; (2) a target-architecture
proposal ("Candidate B: committed-path Ackermann stack") answering what
architecture would make Runner navigate the way Matti wants, not merely
patch the 24 observed failures; (3) a final ratification pass reconciling
planning-model criticisms into a ratified design. **All three are
documents, not code**: `docs/nav2_106_forensic_20260913.md`,
`docs/nav2_navigation_architecture_20260913.md`, and
`docs/nav2_architecture_ratification_20260913.md`. No navigation source or
configuration changed as a result. This section distinguishes, throughout,
between **CONFIRMED** (source/config truth or measured bag evidence),
**RATIFIED ARCHITECTURE** (approved as the target, not yet built), and
**PLANNED** (staged future work, not yet a ratified decision).

### 19.1 Confirmed findings (source + bag evidence)

- `106` (`NO_VALID_CONTROL`) is, in every one of the 24 observed instances,
  `nav2_core::NoValidControl` thrown by the vendored Regulated Pure Pursuit
  controller's `isCollisionImminent()` — a LETHAL-cost-only footprint check,
  distinct from RPP's separate, non-lethal cost-based speed regulation.
  Zero of the 24 failures were `102` (`TF_ERROR`); exactly one was a genuine
  planner infeasibility.
- **`IsPathValid` is served by `planner_server`, against the global
  costmap** (upstream `planner_server.cpp`), not by `controller_server` as
  a prior, narrower investigation pass had left open. It is a single-frame
  check of every remaining path pose's footprint cost with no window,
  persistence, or hysteresis — a single lethal cell in one frame invalidates
  the whole remaining path.
- The global costmap's obstacle layer is configured `combination_method: 0`
  (Overwrite) (`nav2_params.yaml`, unchanged by §18.3's rate-only edits to
  that file): live ray-trace clearing overwrites static-map-derived lethal
  cells. 21 of 63 recorded plans in the investigated bag passed through
  cells the static map itself marks occupied — direct evidence this erases
  real obstacles from the planning costmap between marks.
- **Three mechanistically distinct 106 clusters** were identified in the
  investigated bag, not one root cause: (1) five "immediate" 106s where a
  global/local costmap disagreement about the first ~10 cm of a freshly
  (re)dispatched path caused the very first control tick to fail, clustered
  inside rapid same-mission cancel/redispatch storms; (2) ten of 24 106s
  firing on the exact control tick RPP's commanded direction flips at a
  Reeds-Shepp cusp, where the lookahead collapses to the cusp distance and
  replayed curvature reaches 2.5–11 1/m — Smac was found to emit 7–10 cm
  mid-route reversal segments in 20 of 63 plans, each executed by the motor
  as a full stop → encoder-confirmed-stationary wait → full-duty breakaway;
  (3) corner-cutting at RPP's regulated speed floor, where the certified
  path itself is collision-free but RPP's own shortcut arc to an off-axis
  carrot point clips an obstacle a tracking tube would have avoided.
- Curvature/steering saturation at the drive adapter's physical clamp
  (2.1236 1/m, re-confirmed live) is pervasive in both successful and
  failed attempts alike and does not by itself discriminate outcome —
  reconfirming a prior investigation's "minority amplifier, not primary
  cause" verdict rather than overturning it.
- Five Insane-preset 106s occurred in local-costmap-recorded-free space and
  could not be reproduced from the bag's 3.3 Hz recorded costmap frames —
  recorded as an open recording-resolution gap, not resolved.
- CPU/scheduling starvation was explicitly checked and **ruled out** as the
  explanation for this bag's failures (this was a post-CPU-fix recording
  with measured headroom: mean 49% / peak 94% total CPU, no throttling
  flags, only 7 minor control-loop rate misses in 375 s).

### 19.2 Ratified architecture ("Candidate B": committed-path Ackermann stack)

Ratified with modifications after a planning-model challenge pass; not yet
implemented at any stage.

**Core direction, ratified:** retain Nav2's lifecycle/action framework;
establish coherent obstacle-evidence semantics between planning and
execution; use forward-preferred Ackermann planning with reverse as an
explicit, physically-certified maneuver; replace stock `IsPathValid`
replanning with committed-path blocking semantics; attach explicit
speed/clearance execution semantics to the path; evolve the vendored RPP
lineage into a Runner-owned Ackermann tracker; redesign the BT around
recoverable `BLOCKED` behavior instead of mission-fatal local-control
failure; add mission-level anti-redispatch-storm behavior; keep MPPI as a
**permitted, not-ruled-out** later controller experiment inside this same
contract — Pi CPU viability for MPPI is an open empirical question, not a
reason the architecture was designed around avoiding it.

**Architectural invariants, ratified** (full statements in
`docs/nav2_architecture_ratification_20260913.md`):

1. **Obstacle-evidence coherence, not literal single-costmap, and not a
   strict lethality-superset requirement.** Separate planner (map-frame,
   coarser-or-equal resolution) and executor (odom-frame, finer-or-equal
   resolution, sized to cover stopping distance) costmap objects remain
   legitimate, and the execution representation may legitimately be *more*
   conservative than the planning representation from newer or more
   detailed local evidence — that is not incoherence. The actual invariant:
   **for equivalent obstacle evidence within common spatial coverage,
   differences in layer composition, rasterization, resolution, or
   inflation must not make the planner treat physically blocked geometry
   as traversable while execution treats the corresponding footprint as
   lethal.** This targets §19.1's Overwrite finding specifically (the
   planner's costmap disagreeing with the executor's about the *same*
   evidence, in the direction of the planner being wrong), not a general
   ban on the executor knowing more than the planner.
   **Ownership of clearing is explicit and asymmetric**: static-map-derived
   occupancy is cleared only by a change to the static map itself — a
   live sensor ray must never erase a wall or other static obstacle (this
   is what `combination_method: 0`/Overwrite currently gets wrong,
   §19.1). Live (non-static) evidence may be **both marked and cleared by
   live sensing** — a live obstacle persists only while supported by
   whatever confirmation/clearing/expiry semantics Stage A2 adopts, and
   must clear promptly once subsequent evidence shows it is gone (e.g. an
   object placed in the robot's path is marked; once it is removed and the
   lidar reports clear space where it was, that mark must clear — this
   architecture explicitly rejects turning moved objects into permanent
   phantom obstacles). Static authority is a rule about *who may erase
   static evidence*, not a rule against clearing live evidence.
2. **Layered, graded collision authority**, not a single binary abort:
   the planner certifies the path (curvature ≤ physical cap minus margin,
   no reverse segment shorter than the reversal cost distance,
   footprint-swept lethal-free); the commitment monitor guards a corridor
   ahead with persistence; the tracker checks the *commanded* trajectory
   every tick over a derived stopping distance, but the response is graded
   — slow, then creep, then `BLOCKED` — never an immediate mission-fatal
   abort for a recoverable condition.
3. **Numbers are parameters, not architecture** — the execution window
   size, corridor persistence count, tight-clearance threshold, wait
   budget, and creep speed are all measurement-gated tunables; only the
   stopping distance, the minimum execution window, and the minimum
   reversal segment length are derived invariants (computed from the
   braking model and reversal-timing measurements, not chosen by hand).
4. **Confirm-to-block, evidence-to-clear, static-authority-for-static-only**
   temporal filtering, applied only to live (non-static) evidence — static
   evidence is never subject to this filter at all, since only a static-map
   change may clear it (invariant 1). For live evidence: a single
   unconfirmed observation may still trigger the tracker's own safety
   response, but only persistent, confirmed evidence may trigger a replan;
   clearing evidence (or bounded unobserved expiry) must remove a live mark
   promptly, so a dynamic obstacle that is subsequently observed as clear
   space does not linger as a phantom. **The specific filtering mechanism
   (scan-level consistency vs. an evidence-counting layer vs. a decaying
   voxel layer) is explicitly deferred — not chosen — pending Stage A1's
   own measurement of how much residual flicker survives fixing the
   Overwrite/static-authority issue**; Stage A2 picks the mechanism only
   after that measurement exists.
5. **Retrace (reversing along the robot's own recently-driven trajectory)
   is a later, separate stage, executed *through* the tracker** — not a
   generic straight `BackUp` behavior, and not bundled into the first
   recovery-executive change, because it depends on curvature-cap, cusp,
   and direction-gate machinery that does not exist until the tracker
   stage is built.

**Staged roadmap, ratified** (A depends on nothing already implemented; G
is validation):

| Stage | Content | Status |
|---|---|---|
| A — Obstacle-evidence coherence | Static authority (Max, not Overwrite), aligned marking/clearing semantics, derived execution window, `navigation_debug`-class recording, replay harness | **Not started — next implementation task (Stage A1)** |
| B — Recovery executive | Interim `BLOCKED → WAIT → RESUME → REPLAN → ABORT` ladder via controller patience + BT `RecoveryNode`, reason codes, anti-redispatch-storm | Planned |
| C — Path commitment | `IsCommittedPathBlocked` (corridor + persistence + LETHAL-only, planning representation) replaces stock `IsPathValid` as the replan trigger | Planned |
| D — Planner/path contract | Forward-first planning (Dubins-primary baseline), path certification, speed/clearance profile, execution-contract transport decision | Planned |
| E — Runner tracker | Fork of vendored RPP: hard curvature cap, profile consumption, creep instead of a speed floor, executable-trajectory stopping-distance guard, `BLOCKED` status, cusp protocol | Planned |
| F — Retrace | Trajectory-history-based reverse-along-driven-path recovery, executed through the Stage-E tracker | Planned |
| G — CONFIDENT validation | Measured room-route success-rate target at ~1.0 m/s using the Stage-A replay harness; Insane remains stress evidence only, not a target | Planned |

### 19.3 Decision log: D-89 and D-90 (ratified); D-91–D-95 (provisional labels, not decisions)

**D-89 — Obstacle-evidence coherence invariant (ratified).** Clearing
ownership is explicit and asymmetric: **static-map-derived occupancy must
never be erased by live obstacle-layer clearing at runtime** — a live
sensor ray must never remove a wall or other static-map obstacle; only a
change to the static map itself may do that (supersedes
`global_costmap.obstacle_layer.combination_method: 0` as an acceptable
steady-state configuration once Stage A implements the change — **the
configuration itself is unchanged at this document's baseline**). **Live
(non-static) evidence may be both marked and cleared by live sensing**: a
live-marked obstacle must be established by confirmation and removed
promptly by subsequent clearing evidence or bounded expiry, never treated
as permanent — a moved object (e.g. something placed in the robot's path
and later picked back up) must clear from the live layer once the lidar
reports the space free again; this decision explicitly rejects any
reading of "static authority" as a general argument for obstacle
persistence. For the same sensor evidence within common spatial coverage,
differences in planner/executor layer composition, rasterization,
resolution, or inflation must not let the planner treat physically blocked
geometry as traversable while the executor treats the corresponding
footprint as lethal — the executor may legitimately be *more* conservative
from newer or more local evidence than the planner, which is not itself
an incoherence. The execution representation's spatial window must cover
derived stopping distance plus lookahead, as a computed bound, not a hand-
picked constant. Planner and executor may continue to use separate costmap
objects with different resolution, frame, extent, and update rate — this
decision governs shared *semantics*, not object identity, and does not
prescribe the Stage-A2 confirmation/clearing/expiry mechanism, which is
deferred to Stage A1's own measurements. Rationale: §19.1's investigation
found this bag's "immediate 106" and plan-churn failures traced directly
to the planner and executor disagreeing about identical evidence, and
traced the specific mechanism to `Overwrite` erasing static obstacles — a
config choice with no compensating benefit once corrected. This decision
does not itself change `nav2_params.yaml`; implementation is Stage A1.

**D-90 — Recoverable local-control failure and anti-redispatch-storm
(ratified).** A `FollowPath` (tracker) failure must be a recoverable
mission state, not an automatic mission-fatal abort: the executive owns an
explicit wait/resume/replan/abort ladder (Retrace added at Stage F) with a
concrete reason code surfaced to the operator, and repeated identical
redispatch of the same goal from the same pose after the same failure
reason must be detected and blocked by the mission layer
(`navigation_runtime`) rather than repeated automatically. Rationale:
§19.1 found five "immediate" 106s occurring inside rapid, automatic
same-mission cancel→redispatch storms that reproduced the identical static
failure three times in under two seconds with the robot never moving —
today's executive has no notion of "this exact failure already happened
from this exact pose." This decision does not itself change
`navigate_to_pose_forward_only.xml` or `navigation_runtime.py`;
implementation is Stage B.

**D-91 through D-95 are NOT decisions.** They are anticipated decision
*subjects* — topics this investigation expects will need a ratified
decision at some later stage — given provisional numeric labels purely for
planning convenience and cross-referencing within this document. **These
identifiers are not reserved and carry no authority**: they are not
binding, must not be cited as if ratified, and may end up renumbered,
split, merged, or dropped entirely when their stage actually reaches them
and a real decision is drafted and ratified against the numbering scheme
current at that time:

- **D-91 (anticipated subject, unratified)** — committed-path replanning
  semantics (`IsCommittedPathBlocked` parameters and the keep-old-path
  rule), expected at Stage C.
- **D-92 (anticipated subject, unratified)** — forward-first reversing
  policy and path certification (minimum reversal segment, curvature
  certification), expected at Stage D.
- **D-93 (anticipated subject, unratified)** — execution-contract transport
  (path + keyed profile message design), expected before Stage D
  implementation begins.
- **D-94 (anticipated subject, unratified)** — Runner tracker contract in
  controller-agnostic form (shared profile-lookup, trajectory-guard, and
  `BLOCKED`-status helpers; vendored RPP fork as the reference
  implementation, MPPI as a permitted future alternative implementation),
  expected at Stage E.
- **D-95 (anticipated subject, unratified)** — Retrace recovery semantics
  (trajectory-history ownership and lifetime, execution through the
  Stage-E tracker), expected at Stage F.

### 19.4 Preserved negative findings and open questions

- **These are contract failures, not evidence that RPP or Nav2's tracking
  quality is inadequate and needs an MPPI/controller replacement to fix
  today's problem.** The investigation explicitly evaluated and rejected
  "replace the controller first" as the fix for the observed failures —
  the failures trace to costmap disagreement, cusp/reversal handling, and
  validity-vs-commitment semantics, all of which sit above the tracker.
- **MPPI has NOT been ruled out by measured Pi performance** — it was not
  measured on this hardware at all in this investigation. It remains a
  permitted later experiment inside the ratified contract (§19.2); the
  contract's shared-helper requirement (D-94) exists specifically so that
  experiment stays cheap when someone runs it.
- **The LD19's single scan-plane sensing limitation is unchanged and
  unaddressed by this architecture** (§2): no amount of costmap or BT
  redesign gives Runner information about obstacles above or below the
  scan plane; this is recorded as a standing sensing limitation, not a
  problem Candidate B claims to solve.
- **CPU headroom is currently acceptable but remains an explicit
  constraint on later stages**, not a closed question: this investigation's
  own bag showed a 49%-mean/94%-peak load with no starvation evidence for
  *today's* stack, but Stage D/E's certification and profile-generation
  work, and any future MPPI experiment, must each be measured on this
  Pi 5 rather than assumed affordable.
- **Insane remains experimental stress evidence only** — it is not, and
  this architecture is explicitly not designed around, a target operating
  regime; Confident (~1.0 m/s) remains the target for Stage G validation.
- **Open, unresolved by this investigation**: Smac's 0.60 m planning
  radius, RPP's 0.75 m speed-regulation threshold, and the 0.4709 m
  physical hard minimum radius have **distinct, understood roles** — a
  feasibility margin, a speed-regulation threshold (not a feasibility
  limit), and the physical limit itself — so this is not three
  disagreeing claims about one quantity; **whether their numerical
  margins are mutually appropriate for the eventual Stage-D/E execution
  contract remains open**, with no established causal link to a specific
  observed failure beyond the general saturation background (§19.1).
  `IsPathValid`'s behavior could be
  fully traced from upstream source in this pass (superseding an earlier,
  narrower investigation's claim that it was unavailable), closing that
  specific prior open question; the mechanism behind five Insane-preset
  106s that the recorded costmap resolution cannot explain.

## 20. Known limitations and open engineering work

Carried forward from v1.5 §18 where still accurate, with this round's
changes reflected and new items added:

- **End-to-end stale-command timing remains unbudgeted**, unchanged.
- **`keyboard_bridge` removal remains closed**, unchanged.
- **Sustained high-speed AUTONOMY + recording performance still needs a
  fresh validation recording** predating both v1.5's cleanup and this
  round's wake-rate work — unchanged as open; this round's wake-rate
  reductions (§18.3) are not claimed to be that validation.
- **Paddock web (~40% of one core) and mode_supervisor (~30% of one core)
  CPU costs remain unattributed**, unchanged; this round's QoS-event and
  demand-driven-streaming changes (§18.3) are efficiency/scope
  improvements, explicitly not claimed to move either number.
- **Supervisor-only `RuntimeDirectoryPreserve=restart` smoke validation is
  pending**, unchanged.
- **Deploy coherence is never assumed**, unchanged, and now additionally
  applies to every config value touched in §18.3 (bond heartbeat periods,
  costmap rates, SLAM transform-publish period) — a Pi that has not
  redeployed since `de8de9c` is running the pre-round values.
- **RF2O longitudinal profiling remains engineering tooling**, unchanged.
- **Hardware/integration items from v1.3 Stage 7/8**: unchanged; STOP
  during active autonomous motion and DualSense mid-mission takeover
  remain unexercised in any recorded session.
- **The SIGKILL/PWM-peripheral hazard remains open**, unchanged.
- **Initial Pose UX discrepancies**: the "no fresh slam_toolbox pose"
  class is **closed at the source level** by §9/§18.2's TF-confirmation
  change but **not yet field-revalidated**; the apparent costmap-survival-
  past-clear discrepancy **remains open and uninvestigated**, unchanged
  from v1.5.
- **New: one-click explicit takeover has no owner-facing warning.**
  §6/§18.1's takeover action ends another operator's control session
  immediately and without notice to them; whether this needs a warning,
  cooldown, or confirmation step has not been evaluated and is recorded
  here as an open UX/safety-review item, not a defect.
- **New: navigation architecture Stage A1 has not started.** §19 ratifies
  a target architecture and two decisions (D-89, D-90); no navigation
  source or configuration has changed. Until Stage A is implemented, every
  failure mechanism §19.1 documents (costmap-evidence disagreement, cusp/
  reversal mishandling, corner-cutting at the speed floor, redispatch
  storms) remains live in the deployed system exactly as recorded.
- **New: three "turning radius" values have distinct, understood roles,
  but their mutual margins are not yet validated for Stage D/E** (§2,
  §19.4): Smac's 0.60 m planning-feasibility radius, RPP's 0.75 m
  speed-regulation threshold (not a feasibility limit), and the 0.4709 m
  physical hard minimum. No specific failure has been traced to a margin
  mismatch between them beyond the general saturation background; flagged
  for attention no later than Stage D/E.

## 21. Future exploration and semantic traversability

Unchanged in scope from v1.3–v1.5: supervised frontier exploration
(MAPPING + AUTONOMOUS) and camera-derived semantic traversability layers
remain future work, not implemented at HEAD, and are a **different kind of
future work from §19's Candidate-B staging** — §19's Stages A–G are a
ratified target for the *existing* navigation problem (feasible-route
execution in a known/mapped space), not new capability like exploration or
semantic layers. A later mapping-time Nav2 composition would still need to
reuse the live mapping raster and mapping `map → odom` without
reintroducing a second localizer/map_server; this guidance is unchanged.

## 22. Decision/spec reconciliation against v1.5

| v1.5 subject | v1.6 disposition |
|---|---|
| §9 Initial Pose "no fresh slam_toolbox pose" discrepancy | **Closed at the source level** by the map-frame TF confirmation change (`6a6c452`, §9, §18.2) — not yet field-revalidated. The costmap-survival-past-clear discrepancy is untouched and remains open. |
| §17 CPU/Paddock-web/mode_supervisor open cost questions | **Unchanged and still open.** This round's QoS-event, demand-driven-streaming, and heartbeat/rate reductions (§18.3) are protocol-efficiency changes explicitly not claimed to move any of §17's CPU-attribution numbers. |
| §12 Confident/Timid/Insane preset policy and D-88 | **Unchanged.** A cross-module preflight validation bug affecting Insane's atomic application was found and fixed (`4b6d850`, §12, §18.4); no ceiling value changed. |
| Lease/takeover model (§4/§6) | **Extended, not superseded.** Explicit one-click takeover (`52ad42a`) is new capability beyond v1.5's owner-only-release model; a hold-to-confirm mechanic was tried and removed the same day. Flagged as an open UX/safety-review item (§20), not a defect. |
| §11 BT/replanning/recovery description | **Superseded as intended design, not as current fact.** §11's description of today's `IsPathValid`/no-recovery BT remains an accurate description of what is implemented; §19 ratifies a replacement architecture (Candidate B) that has not been built. D-89 and D-90 are adopted; D-91–D-95 are provisionally labelled anticipated subjects, not decisions. |
| Prior investigation's "IsPathValid source unavailable" open question | **Resolved.** §19.1 traced `IsPathValid` to `planner_server.cpp` against the global costmap directly from upstream Jazzy source, superseding an earlier investigation pass's claim that only a compiled binary was available. |
| Recording profiles (§14) | **Extended.** `navigation_debug` (`e8285b3`) added alongside `runner_debug`/`everything`; active-recording catalog fingerprinting further narrowed (`1aa4f64`). |

## 23. Historical appendix: completed v1.3 migration stages and post-v1.5 cleanup

Condensed from `docs/paddock_v1.3_implementation.md`, `services/README.md`,
v1.4 §20, and v1.5 §21 for continuity; treat §1–§22 above as authoritative
over this appendix wherever they differ.

*(Stage 0 through the "Post-v1.4 cleanup round" entries are unchanged from
v1.5 §21 — see that document for the full condensed history through
`4366b52`.)*

- **Post-v1.5 round** (this document's subject, `de8de9c..02b00c5` plus one
  non-code investigation): hardened Paddock's control UX (one-click STOP
  release and explicit lease takeover, after a same-day hold-to-confirm
  experiment was tried and removed, §18.1); fixed initial-pose confirmation
  to use map-frame TF instead of a slam_toolbox topic, closing one of two
  v1.5-open discrepancies (§9, §18.2); fully decoded Nav2 controller/planner
  error codes and added a `navigation_debug` recording profile (§11, §14,
  §18.2); reduced several wake rates and QoS/streaming overheads across
  Paddock and Nav2/SLAM-toolbox without moving §17's still-open CPU-cost
  questions (§18.3); fixed a real cross-module preflight-validation bug in
  the Insane preset's atomic application (§12, §18.4); and completed a
  three-part read-only navigation-architecture investigation — forensic
  analysis, target-architecture proposal, and ratification — that adopted
  two decisions (D-89, D-90) and identified five more anticipated decision
  subjects (provisionally labelled D-91–D-95, not reserved or ratified)
  toward a committed-path Ackermann stack, with **zero navigation
  source or configuration changes** (§19). Stage A1 of that architecture is
  the next implementation task.
