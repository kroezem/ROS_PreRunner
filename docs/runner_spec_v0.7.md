# Runner — Architecture & Current-State Specification

**Version 0.7** · supersedes v0.6 · 2026-07-24
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. The research contribution is **infrastructure-free iterative racing**: characterizing how localization quality bounds lap-time convergence on a fully self-contained, commodity-sensor vehicle.

All compute and sensing is onboard — no external motion capture, no fixed anchors required for baseline operation. The novel self-improving controller (ILC / Learning-MPC) is DTU thesis work, deliberately deferred. This platform exists to make that work measurable.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed — not preemptively because the hardware is owned.
- **Prototype-first.** Localization quality is the contribution, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from data before changing anything. Author against measured reality.
- **A present sensor can still be silently failing.** RF2O published for months with zero covariance (= infinite confidence), and separately stalled for 8.5 s while scans kept flowing. "The topic exists" is not "the topic is honest or alive."
- **A present *estimator* can also be silently degraded.** Localization ran at 0.11 Hz correction rate while every topic looked healthy (§4.10). Rates prove plumbing, not quality.
- **Velocity-only estimators coast blind through sensor silence.** The EKF fuses only velocities; any gap in its lone translation source integrates into unbounded position drift.
- **One owner per resource.** Every TF edge, serial port, PWM channel, and export lifecycle has exactly one writer.
- **A runnable launch must be complete on its own** (D-29). A launch file that starves its own dependencies is a defect, not a composition primitive.

---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** — tagged `phase-0-complete`. |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **In progress** — relocalization achieved; tracking stability in flight. |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control. | Not started |

**Phase 1 — plan and live status** (§7 for the full Nav2 architecture)

1. **Relocalize on the saved map.** — **Done.** Posegraph serialized, `localization_slam_toolbox_node` loads it, `/initialpose` seeding produces a correct lock (§4.10). The v0.6 blocker ("saved map is `.pgm/.yaml`, not a posegraph") is **resolved**: both artifact families now exist for `house_good_v1`.
2. **Localization tracking stability.** — **In flight.** Seeding works; tracking degrades while driving. Baseline measured and root-cause hypothesis identified (§4.10). *This is the current gate.*
3. **Point-to-point in an open area** — click-goal, forward-only, low speed, obstacle avoidance active from the first powered test.
4. **Route-around, then curved goals, then reverse/recovery** — staged (§7.5).
5. **Frontier exploration** — much later.

**Prerequisite ordering note.** Nav2 work does not start until step 2 closes. A path follower fed a pose that jumps 92 cm at unpredictable intervals will produce control behaviour that is uninterpretable and unsafe, and every Nav2 tuning conclusion drawn on top of it would be worthless.

---

## 3 · Hardware

Unchanged from v0.5/v0.6. Summary: Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset via `pinctrl-rp1` by-label, D-12); US1881 hall encoder (GPIO 22, 0.010282 m/edge); X1201 UPS (compute only, I2C 0x36); DualSense teleop. Separate power domains (UPS = compute, NiMH = traction). Mount finalized; LiDAR+IMU co-mounted, LiDAR the highest point.

**Measured vehicle geometry (new — Phase 1 inputs).**

| Quantity | Value | Confidence |
|---|---|---|
| Wheelbase `L` | **0.178 m** | Measured, trusted. |
| Minimum turning radius `R` | **≈0.565 m** | Derived from a 44.5 in wall-parallel U-turn (that width = **2R**, not R). Use as a conservative planner value. |
| Implied max steer `δ_max` | ≈17.5° (`tan δ = L/R`) | **Suspect — low.** Most RC cars reach 25–30°. Likely a software steering clamp analogous to `THR_MAX_US`, i.e. the servo may not be reaching both mechanical end stops. **Must be verified before the drive adapter is trusted.** |

A conservative (too large) `minimum_turning_radius` only makes the planner cautious, never infeasible — so 0.565 m is safe to start with, but the adapter's steering-angle→servo mapping needs the *true* `δ_max`.

---

## 4 · Software architecture

### 4.1 Packages

Unchanged from v0.6 except as noted.

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | Launch files, config, calibration; the RF2O scan-origin canonicalizer. |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz. Full SHTP over UART. Only gyro Z is fused. Hardware reset opens `pinctrl-rp1` **by label** (D-12). |
| `runner_motor` | `motor_node` | `/cmd_vel` → ESC + steering PWM; `/motor/direction` (Int8), `/motor/state` (String). Sole PWM owner. Does not unexport PWM on shutdown (D-23). |
| `runner_encoder` | `encoder_node` | Hall edges → signed `/wheel/odom`, unfused. Sole GPIO 22 owner. |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel`, dead-man gated. Steering carried in `/cmd_vel.angular.z` as a **normalized** command (see D-34). |
| `runner_battery` | `battery_node` | Fuel gauge → `/battery` (systemd). |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan`. Launched via `include/lidar.launch.py`, never the vendor launch (D-22). |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. Vendored fork: `/tf` remap, TF-init retry (D-21), constant covariance (D-24), stall instrumentation (D-25). |

### 4.2 Launch tree — restructured (D-29)

**Superseded:** the v0.6 tree. The old `localization.launch.py` (an incomplete tier) sat one keystroke from `localize.launch.py` (the complete composite) and was exposed in VS Code under the label `Runner: Localization`. Running it started estimation and SLAM with **no sensors** — RF2O logged "Waiting for laser_scans…" indefinitely. `full.launch.py` and `drive.launch.py` were additionally ambiguous ("full" silently meant *mapping*).

**Current structure.** Three runnable composites at the launch root; every included tier quarantined under `launch/include/`.

```
launch/
├── map.launch.py        = sensors + estimation + slam_map      + teleop
├── localize.launch.py   = sensors + estimation + slam_localize + teleop
├── teleop.launch.py     = joy + teleop_node + motor_node
└── include/
    ├── sensors.launch.py       = tf_static + lidar + imu
    ├── estimation.launch.py    = encoder + rf2o + ekf
    ├── slam_map.launch.py      = slam_toolbox, mapping mode
    ├── slam_localize.launch.py = slam_toolbox, localization mode
    ├── lidar / imu / tf_static / rf2o / ekf   (atomic leaves)
    └── ekf_minimal / rf2o_origin_ab_test      (diagnostic only)
```

**Rules (D-29).**
- Each runnable composite starts **every node needed to function standalone**, including teleop — a mapping launch that can't drive can't map.
- The three composites are **mutually exclusive** because each owns single-owner resources (the two sensor UARTs and/or the PWM motor). This is what D-05 always meant; it is now stated in those terms rather than as an unexplained rule.
- **VS Code task parity:** exactly one production task per runnable composite — `Runner: Map`, `Runner: Localize`, `Runner: Teleop` — each with `dependsOn: Stop all launches`. Tiers and leaves get **no** production task. Diagnostics may carry a task only if prefixed `[diag]`.
- `Stop all launches` SIGINTs launch groups and `motor_node`, then restarts `runner-pwm-setup.service` so PWM exports return clean (D-23). Never SIGKILL the motor node.
- `setup.py` must install **both** `launch/*.py` and `launch/include/*.py`. Omitting the second glob leaves the source tree looking correct while installed launches fail.

Audited and closed at commit `b740040` (8/8 PASS: structure, composition, obsolete-file removal, install globs, `package.xml` `exec_depend` on `runner_encoder`, task list, clean rebuild, all three composites parse).

**Outstanding hygiene:** `Runner: Encoder (standalone)` lacks `dependsOn: Stop all launches` despite contending for GPIO 22 with the composites (which now launch `encoder_node`); the ad-hoc Foxglove bridge task collides on `:8765` with the persistent systemd service. Both are one-owner footguns in an otherwise guarded task list.

### 4.3 Transform tree

`map→odom` = slam_toolbox (mapping **or** localization mode — exactly one runs); `odom→base_link` = EKF (sole owner); `base_link→base_laser`, `base_link→imu_link` = static. RF2O TF suppressed (`publish_tf:false` + `/tf→/tf_disabled`). Never launch the vendor `ld19.launch.py` (D-22).

### 4.4 Static extrinsics

Unchanged. `base_link` = rear axle, ground-projected (D-01). `base_link→base_laser`: x 0.132, y 0, z 0.1135, yaw 0. `base_link→imu_link`: x 0.082, y 0.0025, z 0.106, yaw π. Scan handedness correct with `laser_scan_dir: True` (D-19).

### 4.5 Sensor fusion (EKF)

Unchanged from v0.6. `robot_localization` `ekf_node`, 2D, 15 Hz, `world_frame: odom`, `publish_tf: true`, `sensor_timeout: 0.2`. `odom0: /odom_rf2o` → vx + vyaw; `imu0: /imu/data` → vyaw only. No absolute pose, no wheel odom, no `vy`.

The startup warning `parameter 'imu1' is not initialized` is **benign** — `robot_localization` probing for a second IMU that does not exist. Logged here so it stops being re-diagnosed.

### 4.6 Wheel encoder & direction

Unchanged (GPIO 22, 0.010282 m/edge, sign from `/motor/direction`, D-13/D-14). Launched publish-only inside `include/estimation.launch.py`. **Still unfused** — §4.9, D-26.

### 4.7 RF2O covariance (D-24)

Unchanged from v0.6. Constant calibrated twist covariance: `vx 0.02`, `vyaw 0.25` (deliberately loose, deferring short-term yaw to the 50 Hz gyro). Raw adaptive `cov_odo` not shipped; `/rf2o/diag` retained for analysis.

### 4.8 RF2O stall / blackout (D-25)

Unchanged from v0.6. Root-caused to high-rate INFO logging blocking the single-threaded loop; fixed in `ece93d1`. Post-fix 466 s run: max `/odom_rf2o` gap 0.206 s, no blackout. Keep the instrumentation and keep watching `/rf2o/diag` timing fields.

### 4.9 Wheel odometry — reassessed (D-26)

Unchanged from v0.6. Not the smear fix (wheel lies during spin); *is* the natural blackout/rest anchor; cannot provide `vy`. **Blocking caveat:** one bag showed `/wheel/odom` `twist.linear.x` all-zero while moving. Do not fuse until that is explained. Remains unfused.

### 4.10 Fixed-map localization (D-30, D-31) — **new**

**Mode choice.** Fixed-map localization uses **slam_toolbox localization mode against the serialized posegraph** — not AMCL, and not resume-mapping (D-30).

**Artifacts.** `house_good_v1` exists in both families, and they are not interchangeable:
- `.posegraph` + `.data` → consumed by slam_toolbox localization mode. **This is what relocalization needs.**
- `.pgm` + `.yaml` → occupancy grid, for the Nav2 static costmap layer later.

Serialized via `/slam_toolbox/serialize_map` (`result=0`).

**Startup requirements (both were needed; either alone fails).**
1. **Executable:** `localization_slam_toolbox_node` via slam_toolbox's `localization_launch.py`. The async mapping node can deserialize a posegraph but does not provide the dedicated localization behaviour and **does not subscribe `/initialpose`**.
2. **Bootstrap pose:** `map_start_pose: [0.0, 0.0, 0.0]` must be set. The loader rejects the posegraph *before reading the files* with `Map starting pose not specified` if neither `map_start_pose` nor `map_start_at_dock` is given. This bootstrap is only a seed to permit load; the real pose arrives via `/initialpose`.

Fixed in `6e71634`. Verified: map loads 295×270 @ 0.05 m, origin (−11.342, −6.599); lifecycle active; slam_toolbox sole `/map` publisher and sole `map→odom` owner and sole `/initialpose` subscriber.

**Seeding procedure.** Place the car anywhere with recognizable structure visible, then publish a 2D Pose Estimate from Foxglove to `/initialpose`. slam_toolbox does a **local** scan-match refinement around the seed — it does **not** perform global relocalization from an unknown pose, so a roughly-correct human seed is required. This is accepted as an operator step, not a defect.

> **Frame caveat — works by accident.** Foxglove stamps the outgoing pose with the panel's *Display* frame (observed `base_laser`, later `base_link`) while computing the coordinates in the *Fixed* frame (`map`). slam_toolbox appears to ignore `header.frame_id` and read the numbers as map coordinates, which is why seeding succeeds despite a wrong label. Do not rely on this silently: if anything downstream ever respects that frame, it breaks. Setting Display frame to `map` makes the message self-consistent.

**Baseline measurement — the tracking defect.** Bag `localization_recovery_20260724_163355_0.mcap`, 135 s teleop, max 1.02 m/s, moving 28% of the time:

| Metric | Value |
|---|---|
| `/scan` · `/scan_rf2o` · `/odom_rf2o` · `/imu/data` | 9.7 · 9.9 · 9.9 · 49.4 Hz — all healthy |
| **`map→odom` correction events** | **15 in 135 s = 0.11 Hz** |
| Inter-correction gap | median 2.0 s · mean 8.8 s · **max 55.8 s** |
| Correction magnitude | up to **92 cm** (also 59, 45, 36 cm) |
| `/initialpose` messages | 2 (t=22.5 s, t=32.5 s); **only the second produced a correction** |

Between corrections the map-frame pose is **pure RF2O dead-reckoning**. This is the "snaps in on seed, then drifts, then lurches" behaviour: not a tracking failure so much as a localizer being denied ~99% of its opportunities to correct.

**Leading hypothesis (in flight, not yet confirmed).** `localizer_params_online_async.yaml` was created as a verbatim copy of the mapping params and inherited `minimum_travel_distance: 0.5`, `minimum_travel_heading: 0.5` (≈29°), `minimum_time_interval: 0.5`. In *mapping* these throttle pose-graph growth — correct. In *localization* the same knobs throttle **pose correction itself** — the car travels half a metre, or turns 29°, before slam is permitted to check where it is. Expected fix: reduce toward scan rate (~0.05–0.1 m, ~0.05–0.1 rad, ~0.1 s), then re-measure the same three metrics. Watch Pi CPU: localization mode does not grow the graph, so 10 Hz matching should be affordable — confirm, don't assume.

**Residual-drift caveat.** Correction *magnitude* conflates two causes: overdue correction, and genuine RF2O/EKF odometry drift. They cannot be separated until the correction rate is fixed. Characterize odometry drift only *after* the cadence fix, against the new baseline.

### 4.11 Localization-quality scalar (D-32) — **new**

The thesis's independent variable is now defined concretely: **`map→odom` correction cadence and magnitude**.

- Frequent, small corrections → healthy localization.
- Rare, large corrections → degraded localization.

It requires no new sensor, is already present in `/tf`, and is directly comparable across runs. The §4.10 baseline is its first datum. The comparison script (correction rate, gap distribution, magnitude distribution from an MCAP bag) is a durable instrument and should be kept in-repo, not rewritten per experiment.

This supersedes the earlier sketch of using slam_toolbox's internal scan-match response or AMCL covariance. Those remain available as secondary signals; the TF-correction metric is the primary because it is estimator-agnostic and survives a future localizer swap.

---

## 5 · Teleop & motor control

Unchanged from v0.5/v0.6. Dead-man on X gates throttle+brake (D-06); watchdog is primary safety (D-09); ESC curve deadband+expo with `THR_MAX_US=1750` Phase-0 ceiling (D-08); PWM export owned by `runner-pwm-setup.service` (D-23). SIGINT the motor node, never SIGKILL. Residual hazard (SIGKILL leaves ESC hot) still logged, unsolved.

**Phase-1 additions ratified, not yet implemented:** the drive-command adapter and ESC handshake split (D-34) and the autonomy override gating (D-35).

---

## 6 · Failure taxonomy: wheelspin vs fishtail (D-27)

Unchanged from v0.6. Straight-line wheelspin is tolerated (pure-X error, usually snaps back). Fishtail is the one repeatable break: real lateral `vy` the vx-only EKF cannot represent, and it does not auto-recover. No tight `vy = 0`; no prioritized RF2O `vy` fusion. Treated as a fixed-map localization + recovery problem — a bad transient pose cannot corrupt a read-only saved map.

**Now measurable.** With D-32 in place, "does not auto-recover" stops being a qualitative observation: a fishtail should appear as a step change in correction magnitude that does not decay. Recovery behaviour (detect loss → stop → settle → re-seed) can be specified against a threshold instead of a feeling.

---

## 7 · Phase 1 plan: Nav2 point-to-point

Sections 7.1–7.5 unchanged from v0.6 in substance (Smac Hybrid-A\* Dubins reverse-disabled → Regulated Pure Pursuit → local costmap from `/scan` → drive adapter; obstacle avoidance active from the first powered test; impossible routes safely fail the goal). Amendments:

**7.6 Ordering — bench before floor.** The drive adapter is the highest-risk component (D-34) and must be validated **wheels-off-the-ground** before any floor test. Sequence: (0) relocalization stable → (1) Nav2 planning-only, car powered down, visualize Smac paths → (2) adapter on the bench, verify steering sweep, ESC command, and override preemption → (3) first floor goal, 0.5–1 m, forward-only, ~0.3 m/s.

**7.7 Speed ceiling is scan-rate-bound.** The LD19's 10 Hz refresh, not Pi 5 CPU, sets how fast the local costmap can respond. Early autonomous speeds stay low for that reason, not out of general caution.

**7.8 Collision Monitor deferred.** The primary safety layer is already the motor watchdog (D-09) plus the hand override (D-35). Nav2's Collision Monitor is an additional node and config surface; it earns its place at obstacle-handling Level 2+, once the basic chain is proven. Consistent with phase-gated scope.

**7.9 Reeds-Shepp matters more here than usual.** The platform is 4WD with a large turning circle (§3). A forward-only Dubins planner will fail a meaningful fraction of indoor goals. That is acceptable and intended for the first cut — it fails safely — but it is the concrete reason three-point recovery is a genuine requirement for this vehicle rather than a refinement. *Open question for that stage: whether Regulated Pure Pursuit follows cusped (reverse) segments cleanly on Jazzy, or whether reverse segments need a different controller.*

---

## 8 · Open items & known issues

**Current gate**
- **Localization tracking cadence (§4.10).** Hypothesis identified, fix in flight, before/after metrics required. Nav2 does not start until this closes.

**Phase 1 prerequisites, not yet done**
- **Steering end-stop verification.** `δ_max ≈ 17.5°` is suspiciously low; check for a software clamp and confirm the servo reaches both mechanical stops. Blocks the drive adapter (§3, D-34).
- **`/cmd_vel.angular.z` semantics confirmation.** Verify empirically (teleop running, echo while steering) that it is normalized, before the adapter is written (D-34).
- **twist_mux / override implementation** (D-35).

**Carried forward**
- **RF2O duplicate node identity (D-33).** Two `rclcpp::Node` subclasses in one process both take the launch-level `name=` override → two graph nodes named `/rf2o_laser_odometry`. Pre-existing, cosmetic, non-blocking. Fix at the launch layer only, in its own commit — **verify param keying first**: if the params file is keyed by node name, deleting `name=` silently reverts RF2O to defaults (wrong topics, TF re-enabled), which is worse than the bug.
- **`/wheel/odom` all-zero while moving** — blocks all wheel fusion (§4.9).
- **RF2O blackout** not proven impossible — keep watching `/rf2o/diag`.
- **Fishtail has no recovery behaviour** — now specifiable against D-32.
- **Vendored RF2O fork** carries substantial source changes with `VENDORED.md` but no `.gitmodules` mapping — formalize. *Note: this divergence is now an active constraint on decisions, not just debt (D-33).*
- **Task-list hygiene:** encoder-standalone task missing `dependsOn`; ad-hoc Foxglove task collides with the systemd bridge (§4.2).
- **Repo is 3 commits ahead of `origin/main`, unpushed.** `docs/runner_spec_v0.6.md` untracked.
- Halo/crash protection not built. flake8 exclude absent. UPS discharge unmodeled. GPIO 6 AC-loss / `24e5ad4` un-revert pending. `calibrate_hall_edges.py` commit state unconfirmed.

---

## 9 · Decision log

Append-only. D-01…D-28 unchanged (see v0.5, v0.6). New entries:

| ID | Decision | Reasoning |
|---|---|---|
| D-29 | **Launch architecture.** Runnable composites (`map`, `localize`, `teleop`) live at the launch root and each start *every* node needed to run standalone, including teleop. All included tiers move to `launch/include/`. Exactly one VS Code production task per runnable composite; none for tiers or leaves. D-05's mutual exclusion is restated as following from single-owner resources (sensor UARTs, PWM motor). | The `localization` (tier) vs `localize` (composite) near-collision caused a live failure: the tier ran without sensors, RF2O starved on `/scan`, and the whole stack looked broken. `full`/`drive` were additionally uninformative (`full` silently meant *mapping*). An overlay model (`map` + separately-launched `teleop`) was rejected because it recreates the same defect — a mapping launch that cannot drive cannot map. Naming was the proximate cause; incompleteness was the real one. |
| D-30 | **Fixed-map localization = slam_toolbox localization mode on the serialized posegraph.** AMCL and resume-mapping both rejected for now. | Resume-mapping lets a drift *permanently corrupt the saved map*; localization mode corrupts only the current pose estimate, which is recoverable by re-seeding — this is the thesis premise (D-27) applied directly. Localization mode is also the minimum delta from the already-validated stack: same scan matcher, same `map→odom` owner, so a relocalization failure isn't confounded with a new localizer's tuning. AMCL remains a candidate *thesis instrument* later for its explicit pose covariance, but that is a Phase-2 measurement decision, not a bring-up one. |
| D-31 | **Localization params are tuned independently of mapping params.** Copying the mapping config to create the localization config is a defect, not a shortcut. | The mapping config's `minimum_travel_distance/heading/time_interval` exist to throttle pose-graph growth. Inherited into localization mode they throttle *pose correction*, producing a 0.11 Hz correction rate against a 10 Hz scan source and correction jumps up to 92 cm (§4.10). The two modes have opposite objectives for the same knobs: mapping wants fewer nodes, localization wants maximum correction opportunity. |
| D-32 | **Localization-quality scalar = `map→odom` correction cadence + magnitude.** Supersedes the earlier sketch (scan-match response / AMCL covariance) as the *primary* metric. | It is estimator-agnostic (survives a localizer swap), needs no new sensor, is already in `/tf`, and is directly comparable across bags. It fell out of the measured data rather than being invented, and it makes previously-qualitative claims (§6 "fishtail does not auto-recover") quantitative and threshold-able. |
| D-33 | **RF2O duplicate node identity is fixed at the launch layer only** — no vendored source refactor. Deferred to its own commit; param keying must be verified first. | The clean fix (removing `rclcpp::Node` inheritance from the inner class) is architecturally correct but deepens the vendored fork's already-flagged divergence from upstream for a *cosmetic* defect that does not affect the data path. Wrong trade. A launch-file change lives in `runner_bringup` and adds zero divergence. The verification gate exists because a naive `name=` deletion can silently revert RF2O to default topics and re-enable its TF — worse than the bug being fixed. |
| D-34 | **Drive-command semantics split across two layers.** An adapter *upstream* of the mux converts Nav2's `(v, ω)` to a steering angle via the bicycle model (`δ = atan(L·ω / v)`), normalizing to the same convention teleop already uses. The **ESC reverse handshake lives inside `motor_node`** as sole PWM owner. This handshake sequencer is a *separate concern* from the odometry direction-label FSM (D-13). | Teleop treats `/cmd_vel.angular.z` as a normalized steering command; Regulated Pure Pursuit emits it as a physical yaw rate in rad/s. Same field, same message type, incompatible meaning — feeding Nav2 straight into `motor_node` produces steering that is wrong but *plausible-looking*, the signature of the sign/origin bugs already paid for. The handshake cannot live in a new upstream node because that would create a second PWM writer. The two direction machines differ in kind: one *reports* commanded direction to sign encoder counts; the other *commands* a hardware sequence and must gate on a measured stop. |
| D-35 | **Autonomy override uses two independent hold-to-run gates:** X = teleop dead-man (preempts via mux priority), a separate button = autonomy enable. Nothing held → nobody drives. | The intuitive scheme (autonomy runs when no button is held) is fail-*deadly*: the de-energized state — controller asleep, disconnected, out of battery — becomes "drive autonomously." The de-energized state must be *stopped*. Two gates keep a hand on the kill at all times and compose with the existing watchdog (D-09) rather than fighting it. |
| D-36 | **Multi-agent workflow is explicit and role-split** (see `runner_collab_protocol.md`): planning/architecture and decision-log coherence in one place; self-contained evidence-heavy execution delegated; Codex authors on the Pi. Executors escalate before acting on anything touching one-owner resources, phase scope, or the decision log. | The split is state-dependent vs self-contained work, not "planning vs thinking". Delegation without an escalation rule produces two competing architectures that only the human can see. Findings — not conclusions — are relayed back, because summaries of summaries are how confident-but-wrong causal claims enter the record. The spec, not any model's context window, is the shared durable state. |

---

## Appendix · Workflow conventions

Spec changes that diverge require a new decision-log entry; artifact lineage follows `runner_*`.

**Codex:** read-only investigation first, then a separate action prompt against confirmed reality — never author against assumptions. Codex commits; Matti pushes.

**Executor chats:** paste `runner_collab_protocol.md`, then platform context, then the task brief. New chat per task — a chat that just concluded the stack is healthy is the wrong prior for a chat asked to find what's broken.

**Measurement:** before/after comparisons use MCAP bags over a comparable route and speed profile, analyzed by the same in-repo script (D-32). "Feels better" is not a result.

**Standing rules:** always `source install/setup.bash` after a build. A runnable launch must be complete on its own, and ships with its VS Code task in the same change (D-29).

CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.