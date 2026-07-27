# Runner — Architecture & Current-State Specification

**Version 0.8** · supersedes v0.7 · 2026-07-25
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
- **A present *estimator* can also be silently degraded.** Localization ran at 0.10 Hz correction rate while every topic looked healthy (§4.10). Rates prove plumbing, not quality.
- **A consumer can silently reject data that a publisher is correctly producing.** `/scan` was healthy at 9.7 Hz while Karto discarded ~79% of it on a validation check that logged only to stdout (§4.12). Publisher health does not imply consumer acceptance.
- **A signal's meaning can quietly widen past what it measures** (D-38). Two instances so far: `/cmd_vel.angular.z` (normalized steering vs. rad/s yaw rate) and `/motor/direction` (commanded vs. actual travel direction). Both produced plausible-looking wrong behaviour rather than obvious failure.
- **Velocity-only estimators coast blind through sensor silence.** The EKF fuses only velocities; any gap in its lone translation source integrates into unbounded position drift.
- **One owner per resource.** Every TF edge, serial port, PWM channel, and export lifecycle has exactly one writer.
- **A runnable launch must be complete on its own** (D-29). A launch file that starves its own dependencies is a defect, not a composition primitive.

---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** — tagged `phase-0-complete`. |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **In progress** — relocalization reliable; quantitative tracking measurement outstanding. |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control. | Not started |

**Phase 1 — plan and live status** (§7 for the full Nav2 architecture)

1. **Relocalize on the saved map.** — **Done.**
2. **Reliable seeding.** — **Done.** Root cause was scan-cardinality rejection (§4.12), not seeding technique. Post-fix, seeding is repeatable across launches and sometimes converges with no seed at all.
3. **Localization tracking stability.** — **Qualitatively validated, quantitatively unmeasured.** Visually stable under reasonably aggressive driving on the bedroom map. *The driven-bag comparison against baseline is the current gate.*
4. **Point-to-point in an open area** — click-goal, forward-only, low speed, obstacle avoidance active from the first powered test.
5. **Route-around, then curved goals, then reverse/recovery** — staged (§7.5).
6. **Frontier exploration** — much later.

**Prerequisite ordering note.** Nav2 work does not start until step 3 closes *with numbers*. A path follower fed a pose that jumps unpredictably produces control behaviour that is uninterpretable and unsafe, and every Nav2 tuning conclusion drawn on top of it would be worthless. "Looks good in Foxglove" is not the gate; the D-32 metrics are.

---

## 3 · Hardware

Unchanged from v0.5/v0.6. Summary: Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset via `pinctrl-rp1` by-label, D-12); US1881 hall encoder (GPIO 22, 0.010282 m/edge); X1201 UPS (compute only, I2C 0x36); DualSense teleop. Separate power domains (UPS = compute, NiMH = traction). Mount finalized; LiDAR+IMU co-mounted, LiDAR the highest point.

**Measured vehicle geometry (Phase 1 inputs).**

| Quantity | Value | Confidence |
|---|---|---|
| Wheelbase `L` | **0.178 m** | Measured, trusted. |
| Minimum turning radius `R` | **≈0.565 m** | Derived from a 44.5 in wall-parallel U-turn (that width = **2R**, not R). Use as a conservative planner value. |
| Implied max steer `δ_max` | ≈17.5° (`tan δ = L/R`) | **Suspect — low.** Most RC cars reach 25–30°. Likely a software steering clamp analogous to `THR_MAX_US`. **Must be verified before the drive adapter is trusted.** |

**LD19 scan characteristics (measured, 4760 scans).** Fixed angular extent, **varying** resolution:

- `angle_min = 0.0`, `angle_max = 2π` — 100% of scans
- `len(ranges)` varies **495–509**; mode 504 (42.0%), 503 (21.2%), 505 (24.7%)
- `angle_increment = 2π/(N−1)` on every scan, max numerical error 7.4e-10 rad
- `scan_time` median 100.0 ms; ~1.07% of scans exceed 150 ms (skipped publication interval)

This is the root cause of §4.12. Cardinality varies with motor speed; the driver recomputes increments to keep the 0…2π endpoint-inclusive extent fixed.

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | RF2O scan-origin canonicalizer. **Unchanged and untouched by the rebinner work** (D-37). |
| `runner_bringup` | `scan_rebinner` | **New.** `/scan` → `/scan_slam`, fixed 503-bin angular rebinning (D-37). |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz. Only gyro Z fused. Hardware reset opens `pinctrl-rp1` **by label** (D-12). |
| `runner_motor` | `motor_node` | `/cmd_vel` → ESC + steering PWM; `/motor/direction` (Int8), `/motor/state` (String). Sole PWM owner. Does not unexport PWM on shutdown (D-23). |
| `runner_encoder` | `encoder_node` | Hall edges → signed `/wheel/odom`, unfused. Sole GPIO 22 owner. **Signing defect — see §4.6, D-39.** |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel`, dead-man gated. Steering in `/cmd_vel.angular.z` as a **normalized** command — *empirically confirmed*, §4.6. |
| `runner_battery` | `battery_node` | Fuel gauge → `/battery` (systemd). |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan`. Launched via `include/lidar.launch.py`, never the vendor launch (D-22). |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. Vendored fork: `/tf` remap, TF-init retry (D-21), constant covariance (D-24), stall instrumentation (D-25). |

### 4.2 Launch tree (D-29)

Unchanged in structure from v0.7. Three runnable composites at the launch root; included tiers under `launch/include/`.

```
launch/
├── map.launch.py        = sensors + estimation + slam_map      + teleop
├── localize.launch.py   = sensors + estimation + slam_localize + teleop
├── teleop.launch.py     = joy + teleop_node + motor_node
└── include/
    ├── sensors / estimation / slam_map / slam_localize
    ├── lidar / imu / tf_static / rf2o / ekf   (atomic leaves)
    └── ekf_minimal / rf2o_origin_ab_test      (diagnostic only)
```

Rules unchanged (self-contained composites; mutual exclusion from single-owner resources; one VS Code production task per composite; `setup.py` installs both launch globs). Audited at `b740040`, 8/8 PASS.

**New:** runtime map selection added in `5b0be51` — the localization composite takes the posegraph path as a launch argument rather than a hardcoded config value.

**Outstanding hygiene:** `Runner: Encoder (standalone)` lacks `dependsOn: Stop all launches` despite contending for GPIO 22; the ad-hoc Foxglove task collides on `:8765` with the systemd bridge.

### 4.3 Scan topology (D-37) — **new**

```
LD19 /scan  (raw, variable 495–509 bins, single owner)
   ├─→ rf2o_scan_canonicalizer → /scan_rf2o (origin canonicalization) → RF2O
   ├─→ scan_rebinner           → /scan_slam (fixed 503 bins)          → slam_toolbox
   └─→ raw /scan → Nav2 costmaps (later; cardinality-agnostic)
```

Raw `/scan` retains exactly one writer. The two canonicalizers do **different jobs** and are deliberately separate nodes (D-37).

**Both `map` and `localize` consume `/scan_slam`.** This is not optional: a mapping session on raw `/scan` registers whatever cardinality happens to arrive first, producing a posegraph that is incompatible with everything else (§4.12).

### 4.4 Transform tree

`map→odom` = slam_toolbox (mapping **or** localization mode — exactly one runs); `odom→base_link` = EKF (sole owner); `base_link→base_laser`, `base_link→imu_link` = static. RF2O TF suppressed (`publish_tf:false` + `/tf→/tf_disabled`). Never launch the vendor `ld19.launch.py` (D-22).

### 4.5 Static extrinsics

Unchanged. `base_link` = rear axle, ground-projected (D-01). `base_link→base_laser`: x 0.132, y 0, z 0.1135, yaw 0. `base_link→imu_link`: x 0.082, y 0.0025, z 0.106, yaw π. Scan handedness correct with `laser_scan_dir: True` (D-19).

### 4.6 Sensor fusion, encoder, and command semantics

**EKF** unchanged: `robot_localization` `ekf_node`, 2D, 15 Hz, `world_frame: odom`, `publish_tf: true`, `sensor_timeout: 0.2`. `odom0: /odom_rf2o` → vx + vyaw; `imu0: /imu/data` → vyaw only. No absolute pose, no wheel odom, no `vy`. The `parameter 'imu1' is not initialized` warning is **benign** (probing for a nonexistent second IMU).

**`/cmd_vel.angular.z` semantics — CONFIRMED.** Measured across a full teleop bag: max exactly **+1.000**, min exactly **−1.000**, never exceeded. It is a **normalized** steering command, not a physical rate. This closes a v0.7 open item and fixes the denominator problem in D-34.

**Encoder signing defect (D-39) — new.** `/wheel/odom` velocity is signed by `/motor/direction`, which reports **commanded** direction and emits `0` whenever throttle is zero. Measured on a driven bag: of 461 samples with EKF |vx| > 0.25 m/s, **64 (14%) had zero throttle** — coasting — and in **100% of those, `/motor/direction` was 0**, so the encoder reported 0 m/s while the car was genuinely rolling at speed.

This **falsifies D-26's premise** that the encoder is a clean rest anchor: it reads 0.000 at rest *and* while coasting, and the two are indistinguishable. Fusing it as a zero-velocity anchor would inject false zeros during deceleration — precisely after a fast run.

**Ratified fix (not yet implemented):** publish **unsigned edge rate** alongside signed velocity; latch `direction` persistently (change only on confirmed evidence of the opposite direction, never on absence of command); derive **stationary from pulse absence**, not from commanded throttle. Note that "no pulse for T" is a speed threshold in disguise — at 0.010282 m/edge, a 200 ms timeout declares "stopped" below ~0.05 m/s. Choose it deliberately.

**Bonus:** that stationary detector is exactly the "confirmed stop" the ESC reverse handshake requires (D-34). One fix serves both, and it makes the direction latch self-consistent — direction only ever changes *through* a stop.

### 4.7 RF2O covariance (D-24) / 4.8 RF2O stall (D-25)

Unchanged from v0.6/v0.7. Constant twist covariance `vx 0.02`, `vyaw 0.25`. Stall root-caused to high-rate INFO logging; fixed in `ece93d1`; post-fix max `/odom_rf2o` gap 0.206 s over 466 s. Keep watching `/rf2o/diag`.

### 4.9 Wheel odometry (D-26, amended by D-39)

Not the smear fix (wheel lies during spin); cannot provide `vy`. The **rest-anchor premise is now falsified** (§4.6) — it must be repaired per D-39 before any fusion is reconsidered. Remains unfused.

The v0.7 "all-zero while moving" blocker is **root-caused and closed**: those bags had zero commanded throttle (car carried or coasting), so `/motor/direction` was 0 and the encoder correctly signed to zero. The hardware and node were never broken.

### 4.10 Fixed-map localization (D-30, D-31)

**Mode choice.** slam_toolbox localization mode against the serialized posegraph — not AMCL, not resume-mapping (D-30).

**Artifacts.** `.posegraph` + `.data` → slam_toolbox localization. `.pgm` + `.yaml` → Nav2 static costmap later. Not interchangeable.

**Startup requirements (both needed).**
1. `localization_slam_toolbox_node` via slam_toolbox's `localization_launch.py`. The async mapping node deserializes a posegraph but lacks the localization behaviour and **does not subscribe `/initialpose`**.
2. `map_start_pose: [0.0, 0.0, 0.0]` must be set — the loader rejects the posegraph *before reading the files* otherwise. Bootstrap only; the real pose arrives via `/initialpose`.

Fixed in `6e71634`.

**Seeding procedure — corrected.**
1. **Foxglove Display frame must be `map`** when seeding. The 2D Pose Estimate tool computes **and** stamps in the *Display* frame, not the Fixed frame. With Display = `base_link` it emits car-relative coordinates, which slam_toolbox reads as map coordinates — 4 of 8 seeds in one session landed 2.0–2.4 m outside the map. Use **two 3D panels**: Display = `map` for seeding, Display = `base_link` for driving.
2. slam_toolbox **ignores `header.frame_id`** and reads x/y/yaw directly as map coordinates. Confirmed in 2.8.5 source.
3. Verify with `ros2 topic echo /initialpose --qos-durability volatile`. Without the volatile override, transient-local durability replays a cached message and looks like a fresh publish.
4. Seeding is **local** refinement, not global relocalization. A roughly-correct human seed is expected. Post-D-37 it sometimes converges unaided, but that is a bonus, not a supported capability.

**Superseded:** the v0.7 "mandatory post-seed nudge" claim is **disproved**. Measured seed→`/pose` delays: 0.095 s and 0.134 s while effectively stationary (0.000–0.001 m of motion). The apparent need to drive was scan rejection (§4.12), not a motion gate.

**Baseline measurement (pre-fix).** Bag `localization_recovery_20260724_163355_0.mcap`, 135.7 s teleop, max 1.02 m/s, 25% of `/cmd_vel` nonzero:

| Metric | Value |
|---|---|
| `/scan` · `/scan_rf2o` · `/odom_rf2o` · `/imu/data` | 9.72 · 9.89 · 9.87 · 49.43 Hz — all healthy |
| **Distinct `map→odom` corrections** | **14 in 135.7 s = 0.103 Hz** |
| Gap median / p90 / p95 / max | 2.280 / 27.368 / 41.240 / **55.760 s** |
| Translation median / p90 / p95 / max | 0.194 / 0.548 / 0.706 / **0.925 m** |
| Yaw median / p95 / max | 0.058 / 0.201 / 0.231 rad |

Counting convention: **distinct transform changes**, excluding the initial state. The raw `map→odom` publication rate (~49.5 Hz) is meaningless — it republishes the same value.

**Threshold change (D-31), applied in `0c22402`:** `minimum_time_interval` 0.5→0.1, `minimum_travel_distance` 0.5→0.05, `minimum_travel_heading` 0.5→0.05. Verified live. **Effect still unmeasured** — every post-change run was confounded, first by seeding frame errors, then by scan rejection. Do not revert; do not assume validated.

### 4.11 Localization-quality scalar (D-32)

**`map→odom` correction cadence and magnitude.** Frequent + small = healthy; rare + large = degraded. Estimator-agnostic, already in `/tf`, comparable across bags. Instrumented by `tools/analyze_localization_bag.py` (in-repo, reusable — added in `0c22402`).

Secondary signals (scan-match response, AMCL covariance) remain available but are not primary.

### 4.12 Scan cardinality — root cause and fix (D-37) — **new**

**The defect.** Karto registers a `LaserRangeFinder` **once**, with a fixed beam count, and computes every beam's bearing from that **registered** geometry — not from per-message metadata. `LaserRangeFinder::Validate()` hard-rejects any scan whose length differs: no truncation, no padding, no resampling. `LocalizationSlamToolbox::addScan()` then deletes the rejected scan. The only symptom is a stdout line:

```
LaserRangeScan contains 504 range readings, expected 503
```

**Impact measured.** In the diagnostic session, only **1007 of 4760 scans (21.2%)** matched the registered count. ~79% of the LiDAR stream was silently discarded while every topic rate looked perfectly healthy.

**Seed loss.** A `/initialpose` seed is consumed by the *next* scan-processing attempt. If that scan fails validation, **the seed is destroyed, not retained.** This exactly explains the observed seed outcomes: seeds followed by no warning locked in 0.095 / 0.134 s while stationary; seeds followed immediately by a beam-count warning took 7.4 s and 14.6 s and metres of driving. The apparent randomness was never operator technique or room geometry.

**Registration is non-deterministic.** In mapping mode there is no posegraph, so Karto registers from whichever scan first clears the message filter and motion gates. Two consecutive `map.launch.py` runs registered **501** and then **504**. At 501, only **1.2%** of scans pass — roughly one usable scan every 8 seconds, which is why mapping intermittently appeared "broken" with no code change. Localization registers from the deserialized posegraph (503 for `house_good_v1`), which is stable within a map but arbitrary across maps.

**This retroactively explains months of intermittency.** Sessions that mapped well drew 503/504/505; sessions that failed drew 501/502. Same hardware, same code, different first scan.

**Why it surfaced only now.** With `minimum_travel_distance: 0.5`, throttling was the binding constraint — a 21% pass rate still delivered ~2 valid scans/second, which is plenty when you only want one every half metre. Tightening the thresholds (D-31) removed the throttle and exposed validation as the new bottleneck. The bug was always present, merely masked.

**Fix — `scan_rebinner`, committed `2a85135`.** Angular rebinning onto a fixed grid, **not** padding or truncation:

| Field | Value |
|---|---|
| `angle_min` / `angle_max` | `0.0` / `2π` |
| `angle_increment` | `2π/(bins−1)` |
| `len(ranges)` = `len(intensities)` | `bins` (ROS parameter, **default 503**) |
| `header.stamp` | preserved exactly — never restamped |
| `time_increment` | recomputed as `scan_time/(bins−1)` |

Assignment is nearest-in-angle (`round(j·(bins−1)/(N−1))`); collisions keep the nearest beam; empty bins are `inf`.

**Rejected approaches, with reasons — do not revisit:**
- **Pad/truncate to a fixed count.** Karto reads *registered* angles, so a 505-beam scan truncated to 503 carries 0.7143°/beam while Karto interprets 0.7171°/beam — ~1.4° of accumulated bearing error at the final beam, **silently accepted by validation**. Loud rejection is strictly preferable to silent geometric corruption.
- **Range interpolation.** Interpolating across a depth discontinuity (2 m wall beside an 8 m doorway → phantom 5 m return) fabricates obstacles in free space. Nearest-neighbour only.
- **Driver fork.** Would change raw `/scan` for every consumer including RF2O (measured healthy), and create a second maintained vendored fork. Same reasoning as D-33.
- **Generalizing `rf2o_scan_canonicalizer`.** It performs origin canonicalization, a different job. Merging creates one node with two unrelated modes where any future change touches a working path.
- **Patching Karto/slam_toolbox.** Out of scope; upstream fork.

**Validation (stationary).** 805/805 `/scan_slam` messages at exactly 503 ranges and 503 intensities; fixed angle metadata throughout; no non-increasing timestamps; **zero** `LaserRangeScan contains` warnings. `/scan_rf2o` 9.896 Hz, `/odom_rf2o` 9.860 Hz — **no RF2O regression**. Raw `/scan` still varies 501–507, as intended.

**Validation (driven, qualitative).** Against a fresh deterministic 503-bin bedroom posegraph: seeding repeatable across launches; good seeds snap directly onto map geometry; occasional unaided convergence; visually stable under reasonably aggressive driving; **the random seed-loss behaviour is gone.** Confirmed subsequently on the full apartment map.

**Standing rule:** every new map must be built through `/scan_slam`. A map built on raw `/scan` bakes in a random cardinality and will be incompatible with the pinned stream.

### 4.13 Phantom-geometry trap (D-40) — **new**

**Observed.** If localization converges to a *wrong* pose, the live occupancy output forms plausible room-like geometry around that wrong location, and the system then **stops recovering toward the real map**. Reseeding sometimes fails; a full localization restart is sometimes required.

**Mechanism.** slam_toolbox localization mode retains recent successful scans in the in-memory graph and occupancy grid (`scan_buffer_size: 10`), ageing them out over time. Scans captured while mislocalized depict the *real* surroundings placed at the *wrong* map location. The scan matcher then compares new scans against a graph that contains this phantom geometry — which matches excellently, because it is the same physical room. The estimate becomes **locally self-consistent and globally wrong**, and every subsequent scan reinforces it.

**Why this matters beyond operations.** Bad localization here is **self-reinforcing, not self-correcting**. Failure is a **cliff, not a gradient**: inside the convergence basin, tracking is stable; outside it, the system locks onto a false attractor and does not return. This is the concrete mechanism behind the long-standing "fishtail does not auto-recover" observation (§6), and it sharpens the thesis claim from "localization quality bounds lap-time convergence" to a statement about a hard failure threshold.

**Not a safety feature.** The temporary geometry is an artifact of the localization buffer, not a deliberate remapping mode. The saved posegraph is **not** modified — `house_good_v1.posegraph` / `.data` mtimes are unchanged, and localization mode disables the serialization paths.

**Also explains `/map` dimension variability.** `updateMap()` regenerates the occupancy grid from currently-retained scans, so published `/map` bounds shift as buffer contents change (observed: width 254–428, origin x −14.097 to −11.342 across one session). Published bounds are **visualization output**, not a constraint on where Karto can search.

**Operational procedure (current).** Allow a brief automatic recovery attempt; otherwise supply a better seed; if phantom geometry has formed and recovery stalls, restart localization and reseed.

**Open — not yet tested:** whether a shorter `scan_buffer_size` shortens the trap; whether a fresh `/initialpose` clears the buffer or merely seeds into polluted geometry (this would explain why restart is sometimes required); whether D-32's scalar can detect entry into the trap early enough to act (expected signature: sustained large corrections that do not decay, or corrections ceasing while scans continue).

---

## 5 · Teleop & motor control

Unchanged. Dead-man on X gates throttle+brake (D-06); watchdog is primary safety (D-09); ESC curve deadband+expo with `THR_MAX_US=1750` Phase-0 ceiling (D-08); PWM export owned by `runner-pwm-setup.service` (D-23). SIGINT the motor node, never SIGKILL. Residual hazard (SIGKILL leaves ESC hot) still logged, unsolved.

**Phase-1 additions ratified, not yet implemented:** drive adapter and ESC handshake split (D-34), autonomy override gating (D-35), encoder direction/stationary rework (D-39).

---

## 6 · Failure taxonomy: wheelspin vs fishtail (D-27)

Unchanged in substance. Straight-line wheelspin is tolerated (pure-X error, usually snaps back). Fishtail is the repeatable break: real lateral `vy` the vx-only EKF cannot represent, and it does not auto-recover.

**Now mechanistic.** §4.13 supplies the reason it does not recover: a fishtail can push the estimate outside the convergence basin, after which phantom geometry forms and holds it there. Recovery is therefore not a matter of waiting — it requires detection (D-32) plus an explicit reset. Specifying that behaviour is Phase-1 work.

---

## 7 · Phase 1 plan: Nav2 point-to-point

7.1–7.5 unchanged (Smac Hybrid-A\* Dubins reverse-disabled → Regulated Pure Pursuit → local costmap from `/scan` → drive adapter; obstacle avoidance active from the first powered test; impossible routes safely fail the goal).

**7.6 Ordering — bench before floor.** The drive adapter is the highest-risk component (D-34) and must be validated **wheels-off-the-ground** first. Sequence: (0) tracking measured → (1) Nav2 planning-only, car powered down, visualize Smac paths → (2) adapter on the bench: steering sweep, ESC command, override preemption → (3) first floor goal, 0.5–1 m, forward-only, ~0.3 m/s.

**7.7 Speed ceiling is scan-rate-bound.** The LD19's 10 Hz refresh, not Pi 5 CPU, sets local-costmap responsiveness.

**7.8 Collision Monitor deferred.** Primary safety is the motor watchdog (D-09) plus hand override (D-35). Collision Monitor earns its place at obstacle-handling Level 2+.

**7.9 Reeds-Shepp matters more here than usual.** 4WD with a large turning circle (§3); forward-only Dubins will fail a meaningful fraction of indoor goals. Acceptable for the first cut (fails safely), but it is why three-point recovery is a genuine requirement. *Open: whether RPP follows cusped segments cleanly on Jazzy.*

**7.10 Remove rotate-in-place recovery.** Nav2's default behaviour tree includes spin / rotate-in-place, which an Ackermann vehicle physically cannot execute. It must be removed from the BT, or the first failed goal produces a car sitting still, steering uselessly, believing it is recovering.

**7.11 Speed is open-loop.** Nav2 commands m/s; the ESC receives PWM and delivers whatever battery voltage, friction, and incline allow. Nothing measures or corrects it. A speed PID using the encoder's unsigned edge rate (D-39) is the legitimate fix — deferred until the open-loop error is measured. **Steering does *not* get a PID:** there is no servo feedback sensor, and Nav2 is already the outer loop; nesting a second controller on the same objective produces oscillation that looks like bad tuning in both.

---

## 8 · Open items & known issues

**Current gate**
- **Quantitative tracking measurement.** Driven apartment bag, one seed at start, no reseeding, ~135 s, throttle/speed profile comparable to baseline, launch logs captured. Compare via `tools/analyze_localization_bag.py` against §4.10. This is what closes Phase 1 step 3 — qualitative success does not.

**Ratified, not implemented**
- **Encoder direction/stationary rework** (D-39).
- **Steering end-stop verification** — `δ_max ≈ 17.5°` suspect; blocks the drive adapter.
- **Drive adapter** (D-34) — δ = atan(L·ω/v); handle v→0; saturation must reduce *speed* as well as clamp δ; verify sign on the bench.
- **twist_mux + two-gate override** (D-35).
- **Pi telemetry node** — temp + `vcgencmd get_throttled` including the **sticky** since-boot bits, 1 Hz, systemd like `battery_node`. Newly justified: D-31 raised scan-match load ~10×, and thermal throttling would masquerade as localization degradation.
- **Remove Nav2 rotate-in-place** (§7.10).
- **Two-panel Foxglove setup** as the standing convention (§4.10).

**Carried forward**
- **RF2O duplicate node identity (D-33).** Launch-layer fix only, own commit — **verify param keying first**; a naive `name=` deletion can silently revert RF2O to default topics and re-enable its TF.
- **Phantom-trap open questions** (§4.13): `scan_buffer_size` sensitivity; whether reseeding clears the buffer.
- **Magnetometer heading** — `/imu/data.orientation` already carries mag-fused yaw (9-axis rotation vector). Measured residual vs. slam yaw: 8.2° std, 17.5° max, ~19° systematic wander. **Confounded** — slam's own yaw was degraded at measurement time. Re-run against post-D-37 localization before judging. Provisionally: too dirty to fuse, possibly good enough to seed initial heading.
- **UWB deferred.** Trigger is *observed perceptual aliasing*, which has not occurred. Single-anchor UWB is a rank-1 constraint — a weak corrector but a strong **independent integrity check**, which is the actual open gap (§4.13 detection). Revisit only after cheaper options: full-rate correction (done), wheel-odom rest anchor (D-39), mag heading.
- **Docking station** — Phase 2/3. Value is three roles in one object: charger, UWB anchor, and a **physically repeatable session origin** (important for session-to-session lap comparability). Keep the two power domains; feed one DC bus to the X1201 input plus a separate NiMH charger. Requires reverse + cusped planning first.
- **Vendored RF2O fork** — substantial source changes, `VENDORED.md`, no `.gitmodules` mapping. Divergence is an active constraint on decisions (D-33, D-37).
- **Task-list hygiene** (§4.2). Halo/crash protection. flake8 exclude. UPS discharge model. GPIO 6 AC-loss / `24e5ad4` un-revert. `calibrate_hall_edges.py` commit state.

---

## 9 · Decision log

Append-only. D-01…D-36 unchanged (v0.5–v0.7). New entries:

| ID | Decision | Reasoning |
|---|---|---|
| D-37 | **Scan path forks per consumer.** Raw `/scan` stays untouched and single-owner. A new dedicated `scan_rebinner` publishes `/scan_slam` at a fixed 503 bins (ROS parameter) by **angular rebinning**, nearest-in-angle, `inf` for empty bins. Both `map` and `localize` consume it. `rf2o_scan_canonicalizer` is untouched. | Karto registers a fixed beam count once and computes bearings from *registered* geometry, hard-rejecting mismatched scans — 79% of the stream was being discarded silently. Count-only pad/truncate is **unsafe**: it satisfies validation while leaving every beam at a wrong bearing (~1.4° accumulated error), converting a loud failure into silent geometric corruption. Interpolation is rejected because it fabricates obstacles at depth discontinuities. A driver fork was rejected because it changes `/scan` for every consumer including a measured-healthy RF2O and creates a second vendored fork (same reasoning as D-33). Generalizing the RF2O canonicalizer was rejected because it does a different job and merging risks a working path. 503 is chosen over the more common 504 solely for compatibility with `house_good_v1`; post-rebinning, pass-rate arguments are moot. |
| D-38 | **Semantic-drift is a recognized bug class.** When a signal's consumed meaning widens beyond what it measures, treat it as a defect and name the measured quantity explicitly. | Two instances, both producing plausible-but-wrong behaviour rather than clean failure: `/cmd_vel.angular.z` (normalized steering to teleop, rad/s yaw rate from Nav2 — and the conversion error is *speed-dependent*, so it looks nearly right in some regimes); `/motor/direction` (commanded direction to the motor node, actual travel direction to the encoder). Naming the class is the only reliable defence against a third instance. |
| D-39 | **Encoder publishes unsigned edge rate; `direction` is latched persistently; stationary is derived from pulse absence.** Supersedes the D-26 assumption that the encoder is a clean rest anchor. | `/motor/direction` emits 0 on zero throttle, which zeroes the signed velocity — but a car cannot spontaneously stop or reverse. Measured: 14% of moving time is coasting, and during 100% of it the encoder reported 0 m/s while rolling >0.25 m/s. Absence of command is not evidence of absence of motion. Edge rate is always truthful about wheel rotation and is the correct rest/stop signal; it is also exactly the "confirmed stop" the ESC reverse handshake needs (D-34), so one fix serves both and the direction latch becomes self-consistent — direction changes only *through* a stop. |
| D-40 | **The phantom-geometry trap is recognized as a failure mode with a cliff, not a gradient.** Recovery requires explicit detection plus reset; it will not self-correct. Buffered localization scans are an artifact, never to be interpreted as intentional remapping. | Localization mode retains recent scans in the in-memory graph. Scans taken while mislocalized depict the real room at the wrong map location, so the matcher finds an excellent match against its own error and reinforces it every cycle — locally self-consistent, globally wrong. This is the mechanism behind "fishtail does not auto-recover" (D-27, §6) and it means localization failure has a hard threshold rather than graceful degradation. It also explains published `/map` bounds varying within a session while the saved posegraph is provably unmodified. |

---

## Appendix · Workflow conventions

Spec changes that diverge require a new decision-log entry; artifact lineage follows `runner_*`.

**Codex:** read-only investigation first, then a separate action prompt against confirmed reality — never author against assumptions. **Codex commits and pushes** (SSH push access restored 2026-07-25; the earlier "commit only, never push" rule is retired).

**Executor chats:** paste `runner_collab_protocol.md`, then platform context, then the task brief. New chat per task — a chat that just concluded the stack is healthy is the wrong prior for a chat asked to find what's broken. Executors escalate before acting on one-owner resources, phase scope, or decision-log changes (D-36).

**Measurement:** before/after comparisons use MCAP bags over a comparable route and speed profile, analyzed by the same in-repo script (D-32). Report the driving profile alongside the metrics — an unmatched profile invalidates the comparison. **"Feels better" is not a result.**

**Standing rules:** always `source install/setup.bash` after a build. A runnable launch must be complete on its own and ships with its VS Code task in the same change (D-29). Every new map is built through `/scan_slam` (D-37).

CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.