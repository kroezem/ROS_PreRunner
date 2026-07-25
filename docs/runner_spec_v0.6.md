# Runner — Architecture & Current-State Specification

**Version 0.6** · supersedes v0.5 · 2026-07-22
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. The research contribution is **infrastructure-free iterative racing**: characterizing how localization quality bounds lap-time convergence on a fully self-contained, commodity-sensor vehicle.

All compute and sensing is onboard — no external motion capture, no fixed anchors required for baseline operation. The novel self-improving controller (ILC / Learning-MPC) is DTU thesis work, deliberately deferred. This platform exists to make that work measurable.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed — not preemptively because the hardware is owned.
- **Prototype-first.** Localization quality is the contribution, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from data before changing anything. Multiple hypotheses this cycle (mis-rooted TF, sign inversion, extrinsic, phantom velocity) were wrong until a bag disproved them. Author against measured reality.
- **A present sensor can still be silently failing.** RF2O published for months with zero covariance (= infinite confidence), and separately *stalled for 8.5 s while scans kept flowing*. "The topic exists" is not "the topic is honest or alive."
- **Velocity-only estimators coast blind through sensor silence.** The EKF fuses only velocities; with no absolute reference and no zero-velocity anchor, any gap in its lone translation source integrates into unbounded position drift.
- **One owner per resource.** Every TF edge, serial port, PWM channel, and export lifecycle has exactly one writer.

---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** — coherent maps of the full apartment under normal & aggressive driving. |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **Starting.** |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control. | Not started |

**Phase 0 — closed.** Mapping is excellent across normal and aggressive traction-limited indoor driving. The two failures that were blocking clean operation are resolved:
- The RF2O forward-sign inversion (v0.5, canonicalizer).
- The RF2O **zero-covariance** bug and the RF2O **stall/blackout** (this version, §4.7–4.8).

The only repeatable remaining break is a deliberate **fishtail** (§6), reframed as a Phase-1 recovery problem, not a mapping blocker.

**Phase 1 — the plan, in order** (§7 for the full Nav2 architecture)

1. **Relocalize on the saved map.** Load the saved map, confirm the pose locks without driving there. *Note: the saved map is a `.pgm/.yaml` occupancy grid, not a slam_toolbox `.posegraph`; slam_toolbox localization mode needs the serialized posegraph — either re-serialize via slam_toolbox's serialize service, or use AMCL on the grid.*
2. **Point-to-point in an open area** — click-goal, forward-only, low speed, with Nav2 obstacle avoidance active from the first powered test.
3. **Route-around, then curved goals, then reverse/recovery** — staged (§7 dev sequence).
4. **Frontier exploration** — much later, after reliable point-to-point.

---

## 3 · Hardware

Unchanged from v0.5. Summary: Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset); US1881 hall encoder (GPIO 22, 0.010282 m/edge); X1201 UPS (compute only, I2C 0x36); DualSense teleop. Separate power domains (UPS = compute, NiMH = traction). Mount finalized; LiDAR+IMU co-mounted, LiDAR the highest point. GPIO map and the `pinctrl-rp1`/`gpiochip4` by-label convention (D-12) unchanged.

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | Launch files, config, calibration; the RF2O scan-origin canonicalizer. Now also launches `encoder_node` in the localization composite (publish-only, unfused). |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz. Full SHTP over UART (not UART-RVC). Publishes rotation-vector quaternion, gyro, linear accel; only gyro Z is fused. |
| `runner_motor` | `motor_node` | `/cmd_vel` → ESC + steering PWM; `/motor/direction` (Int8), `/motor/state` (String). Does not unexport PWM on shutdown (D-23). |
| `runner_encoder` | `encoder_node` | Hall edges → signed `/wheel/odom`, unfused. Quantized to ~0.206 m/s steps (1 edge / 50 ms). |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel`, dead-man gated. Steering carried in `/cmd_vel.angular.z` (no separate steering topic). |
| `runner_battery` | `battery_node` | Fuel gauge → `/battery` (systemd). |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan`. Launched via `runner_bringup/lidar.launch.py`, never the vendor launch (D-22). |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. Vendored fork carrying: `/tf` remap, TF-init retry (D-21), angular-origin note, constant covariance (D-24), and stall instrumentation (D-25). |

### 4.2 Launch tree

Three mutually-exclusive composites (D-05); `rf2o` composite = canonicalizer + RF2O; `localization` = rf2o + ekf + slam + encoder(publish-only). Diagnostic-only standalone: `ekf_minimal.launch.py`, `rf2o_origin_ab_test.launch.py`.

### 4.3 Transform tree

`map→odom` = slam_toolbox; `odom→base_link` = EKF (sole owner); `base_link→base_laser`, `base_link→imu_link` = static. RF2O TF suppressed (`publish_tf:false` + `/tf→/tf_disabled`). Never launch the vendor `ld19.launch.py` (duplicate `base_laser` at z=0.18) — D-22.

### 4.4 Static extrinsics

`base_link` = rear axle, ground-projected (D-01). `base_link→base_laser`: x 0.132, y 0, z 0.1135, yaw 0 (yaw 0 + physical +X-forward both confirmed). `base_link→imu_link`: x 0.082, y 0.0025, z 0.106, yaw π. Scan handedness correct with `laser_scan_dir: True` (D-19).

### 4.5 Sensor fusion (EKF)

`robot_localization` `ekf_node`, 2D mode, 15 Hz, `world_frame: odom`, `publish_tf: true`, `sensor_timeout: 0.2`, `print_diagnostics: true`.

- `odom0: /odom_rf2o` → fuses **vx + vyaw**.
- `imu0: /imu/data` → fuses **vyaw only** (gyro Z). Not orientation, not accel.
- **No** absolute pose/orientation, **no** wheel odom, **no** `vy` — so this is a velocity-integrating dead-reckoner corrected globally only by slam_toolbox `map→odom`.

**Consequence made explicit (see §4.8):** because RF2O is the *only* translation source and there is no zero-velocity anchor, any RF2O silence makes the EKF coast on stale velocity → position float. Wheel odom is the natural anchor but is not yet fused (§4.9).

### 4.6 Wheel encoder & direction

Unchanged mechanics from v0.5 (GPIO 22, 0.010282 m/edge, sign from `/motor/direction`, D-13/D-14). Now launched publish-only in the localization composite. **Still unfused** — see §4.9 for the reassessed fusion case.

### 4.7 RF2O covariance — the zero-confidence bug and its fix (D-24)

RF2O published **all-zero** twist covariance → the EKF treated RF2O as exact, so no finite-covariance secondary source could influence the estimate, and the EKF trusted RF2O even mid-failure.

A diagnostic publisher `/rf2o/diag` was added (never touching production odom) exposing the internal residual covariance `cov_odo`, `dt`, valid-range count, SSE, and the information matrix (`AtA`); later extended with timing fields (§4.8). Bag analysis (`cov_diag`) established:

- `dt` is stable (~0.10 s); the earlier "dt inverts during turns" worry was wrong.
- `cov_odo/dt²` responds to hard scan matching but is **~500–1000× overconfident**, **2–3 frames late**, and **non-specific** (an ordinary hard turn produced residuals as high as the drift). RF2O-vs-IMU yaw disagreement hit **~28σ** during the drift.

**Decision: publish a calibrated *constant*, not adaptive `cov_odo`.**

```cpp
odom.twist.covariance[0]  = 0.02;   // vx   (std 0.14 m/s) — from clean-drive RF2O-vs-wheel disagreement
odom.twist.covariance[35] = 0.25;   // vyaw (std 0.5 rad/s) — deliberately loose; defers yaw to the 50 Hz IMU
```

The loose vyaw hands short-term yaw-rate authority to the BNO085 gyro; RF2O yaw remains a weak secondary *rate* measurement (not an absolute-yaw correction — global heading is corrected by slam_toolbox `map→odom`, not the EKF). **Validated on a before/after bag:** during the drift, integrated EKF yaw 3.1358 rad tracked IMU 3.1334 rad, not RF2O's over-rotated 3.3011 rad; the EKF's phantom-yaw follow dropped from ~70% → ~45% and filtered-vs-IMU yaw error fell ~56%. No clean-driving regression. The raw adaptive `cov_odo` is **not** shipped; `/rf2o/diag` is retained for future analysis.

### 4.8 RF2O stall / blackout — the stationary-float root cause (D-25)

A bag captured RF2O silent for **8.57 s** while `/scan` (85 msgs) and `/scan_rf2o` kept flowing — RF2O was *receiving* scans and not publishing. Because RF2O is the EKF's only translation source, the velocity-only filter coasted on stale velocity and `odom→base_link` drifted ~0.8–1.0 m ("map floating away at constant velocity"), recovering only when RF2O resumed. `map→odom` stayed fixed throughout — confirming the drift was EKF-side, not slam.

**Root-cause candidate & fix:** RF2O runs a single-threaded `spin_some → process → sleep` loop; three scan-rate INFO logs were credible blocking points under terminal/journald backpressure. Those logs were removed / demoted to DEBUG, and timing instrumentation was added (commit `ece93d1`, pushed). `/rf2o/diag` extended with fields 22–33: update wall/CPU time, publication age, range counts, per-stage timings, scan timestamps, effective dt; warnings throttled at wall>200 ms or age>300 ms.

**Post-fix evidence:** a 466 s aggressive endurance run showed max `/odom_rf2o` gap 0.206 s, mean update 16.4 ms, max 24.8 ms — large headroom vs the 100 ms LiDAR period, **no blackout**. The stall has not reproduced across multiple subsequent runs. Strong (not conclusive) evidence the INFO logging was the culprit. Keep the instrumentation.

### 4.9 Wheel odometry — reassessed (D-26)

The fusion case was re-derived from data and reversed twice:

- **Not** the fix for the drift smear: the wheel *lies* during wheelspin (reached 2.88 m/s while the chassis moved ~1.8).
- **Is** the natural fix for the blackout/float: at true rest the wheel reads a clean `0.000`, a rock-solid zero-velocity anchor exactly where RF2O goes silent or noisy.
- **Cannot** provide `vy`: single-channel, forward-only. The only onboard `vy` source is RF2O's internal lateral estimate — least trustworthy during the exact fishtail where `vy` matters.

**Blocking caveat before any fusion:** one instrumented bag showed `/wheel/odom` `twist.linear.x` **all-zero while the car was moving**. Do not fuse a possibly-dead topic. Investigate (edges reaching the node? `/motor/direction` gating to 0? launch/wiring? hardware?) before enabling. Also needs a slip-aware or deliberately-loose covariance so it anchors at rest without over-trusting during spin. **Remains unfused; a future robustness layer, not a current dependency.**

---

## 5 · Teleop & motor control

Unchanged from v0.5. Dead-man on X gates throttle+brake (D-06); watchdog is primary safety (D-09); ESC curve deadband+expo with `THR_MAX_US=1750` Phase-0 ceiling (D-08); PWM export owned by `runner-pwm-setup.service`, motor node never unexports (D-23). SIGINT the motor node, never SIGKILL. Residual hazard (SIGKILL leaves ESC hot) still logged, unsolved — a Phase-1+ microcontroller/VESC trigger.

---

## 6 · Failure taxonomy: wheelspin vs fishtail (D-27)

Repeated testing separated two motions that look similar but fail differently:

- **Straight-line wheelspin — tolerated.** A hard launch that spins tires but stays straight produces a pure-**X** error. The EKF reconstructs world motion as `R(yaw)·[vx, 0]`, so a forward-only error stays consistent and usually snaps back.
- **Fishtail — the one repeatable break.** Real lateral sliding with rapid alternating yaw gives the chassis a genuine `vy` the EKF model (vx-only) cannot represent: `world_vel = R(yaw)·[vx, vy]` with `vy` discarded. Once a severe fishtail mismatches, it stays offset — it neither floats indefinitely nor auto-recovers.

**Deliberate decisions:**
- **Do not add a tight `vy = 0`** — it encodes a false assumption during real slides.
- **Do not prioritize RF2O `vy` fusion** — minimal gain; the only `vy` source is unreliable precisely during the fishtail.
- **Treat fishtail as a fixed-map localization + recovery problem, not a live-mapping one.** A drift corrupts a *live* map permanently but only corrupts the *pose* against a saved read-only map. This matches the thesis premise: build the map conservatively, then study how localization quality bounds speed and maneuver aggressiveness. Mapping should be driven conservatively (moderate accel/yaw, no deliberate spin, one clean saved map).

---

## 7 · Phase 1 plan: fixed-map localization + Nav2 point-to-point

### 7.1 Target architecture (first version)

```
saved map + localization
      → click goal
      → Smac Hybrid-A*  (Ackermann-feasible global path)
      → Regulated Pure Pursuit  (path follower)
      → live /scan → local costmap (obstacle + inflation layers)
      → controller slows/stops if path becomes unsafe
      → drive adapter: /cmd_vel Twist → steering + signed speed
```

Nav2 uses a global costmap (long-range planning) and a local costmap (short-range + collision avoidance). The obstacle layer consumes `/scan` directly; the inflation layer adds a turning-radius-aware safety margin.

### 7.2 First planner config

- **Smac Hybrid-A\***, motion model **Dubins**, **reverse disabled** — kinematically feasible forward-only primitives, and it postpones the entire ESC reverse handshake until forward navigation works.
- **Regulated Pure Pursuit** controller — already reduces speed around high curvature and near inflated obstacle cost; good for a cautious first cut.

### 7.3 First physical test conditions (not software limits)

Fixed saved map, large clear floor, goal ~0.5–1 m ahead, forward-only, very low max speed, no direction changes, no recovery maneuvers, controller in hand for override. **Obstacle avoidance stays active** — the absence of obstacles is a test condition, not a removed feature. Stripping avoidance would build a throwaway intermediate system.

### 7.4 Obstacle-handling levels

1. **Don't hit anything** — LiDAR marks the local costmap, controller stops on unsafe path. Present from the first powered test.
2. **Route around** — Nav2 replans around a chair/box given turning-radius clearance. Immediately after basic click-to-go.
3. **Close-quarters recovery** — reverse + 3-point turn. Deferred: needs reverse-capable planning, the ESC direction-transition state machine, reliable stop detection, forward/reverse segment handling.
4. **Collision Monitor** — independent emergency layer (outer zone slow, inner zone stop), useful while planner/controller params are experimental.

### 7.5 Development sequence

0. Visualize generated paths without moving.
1. Clear forward goal, avoidance active.
2. Object appears → car stops.
3. Wide obstacle → planner routes around.
4. Curved goals requiring substantial steering.
5. Reverse drivetrain state machine.
6. Reeds-Shepp forward/reverse paths.
7. Three-point turns.
8. Point-to-point during live mapping.
9. Autonomous frontier exploration.

An impossible route should **safely stop and fail the goal**, not improvise a 3-point turn, until Level 3 exists.

---

## 8 · Open items & known issues

- **Relocalization prerequisite:** saved map is an occupancy grid (`.pgm/.yaml`), not a slam_toolbox posegraph. Re-serialize the posegraph (slam_toolbox serialize service) or use AMCL for fixed-map localization.
- **Wheel `/wheel/odom` published all-zero in one bag** while moving — investigate before any fusion (§4.9).
- **RF2O blackout** appears fixed by the INFO-log cleanup (D-25) but is not *proven* impossible — keep watching `/rf2o/diag` timing fields for recurrence.
- **Fishtail mismatch does not auto-recover** — needs explicit localization-loss detection + stop/settle/relocalize behavior (a Phase-1 recovery workstream, §6).
- **RF2O covariance is constant, not adaptive.** `cov_odo` is retained in `/rf2o/diag` if a future adaptive `floor + scale·cov_odo/dt² + PSD-clamp` is ever justified.
- **Vendored RF2O fork now carries substantial source changes** (TF-retry, constant covariance, instrumentation) with a `VENDORED.md` but no `.gitmodules` mapping — formalize it.
- **`calibrate_hall_edges.py`** — confirm commit state (was a dangling working-tree edit).
- Halo/crash protection still not built (parallel mechanical track). flake8 exclude still absent. UPS discharge unmodeled. GPIO 6 AC-loss / `24e5ad4` un-revert still pending.

---

## 9 · Decision log

Append-only. D-01…D-23 unchanged (see v0.5). New entries:

| ID | Decision | Reasoning |
|---|---|---|
| D-24 | RF2O publishes a **constant** calibrated twist covariance (vx 0.02, vyaw 0.25), not adaptive `cov_odo`. vyaw deliberately loose to defer yaw to the IMU. | Zero covariance = infinite EKF confidence (the real bug). `cov_odo` is ~500–1000× overconfident, 2–3 frames late, and non-specific — unsafe to ship. Constants are honest and simple; loose vyaw lets the 50 Hz gyro own short-term yaw (validated: EKF followed IMU, not RF2O's phantom rotation, during the drift). |
| D-25 | RF2O stall root-caused to high-rate INFO logging blocking the single-threaded loop; fixed by removing/DEBUG-ing those logs + adding timing instrumentation (`ece93d1`). | RF2O went silent 8.57 s while scans flowed; the velocity-only EKF coasted → ~1 m float. Post-fix 466 s run showed no blackout and large timing headroom. A velocity-only estimator has no defense against its sole translation source going silent. |
| D-26 | Wheel odom stays **unfused**; reassessed as a blackout/rest robustness anchor, not a smear fix, and cannot provide `vy`. Blocked pending the all-zero-output investigation + a slip-aware covariance. | Wheel lies during spin (2.88 m/s) but is truthful (0.000) at rest where RF2O goes silent. Single-channel → no lateral. Fusing a possibly-dead topic is unsafe; fusing tight covariance would over-trust spin. |
| D-27 | Fishtail is treated as a fixed-map localization/recovery problem, not a mapping one. No tight `vy=0`; no prioritized RF2O `vy` fusion. | Straight wheelspin (pure-X) recovers; fishtail creates real `vy` the vx-only EKF can't represent and doesn't auto-recover. The only `vy` source (RF2O) is least trustworthy during the fishtail. A saved read-only map can't be corrupted by a bad transient pose — matches the thesis premise. |
| D-28 | First point-to-point keeps Nav2 obstacle avoidance active (not stripped); Smac Hybrid-A* (Dubins, reverse disabled) + Regulated Pure Pursuit; reverse/Reeds-Shepp/3-point recovery deferred until the ESC reverse handshake exists. | Avoidance is core Nav2 — removing it builds a throwaway intermediate. Forward-only Dubins postpones reverse complexity while proving the planning→following→costmap→drive-adapter chain. An impossible route safely fails the goal rather than improvising. |

---

## Appendix · Workflow conventions

Spec changes that diverge require a new decision-log entry; artifact lineage follows `runner_*`. Codex workflow: read-only investigation first, then a separate action prompt against confirmed reality — never author against assumptions. Git: Codex commits, Matti pushes (SSH now configured). Always `source install/setup.bash` after a build. CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.