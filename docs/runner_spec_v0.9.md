# Runner — Architecture & Current-State Specification

**Version 0.9** · supersedes v0.8 · 2026-07-27
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. The research contribution is **infrastructure-free iterative racing**: characterizing how localization quality bounds lap-time convergence on a fully self-contained, commodity-sensor vehicle.

All compute and sensing is onboard — no external motion capture, no fixed anchors. The self-improving controller (ILC / Learning-MPC) is DTU thesis work, deliberately deferred. This platform exists to make that work measurable.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed.
- **Prototype-first.** Localization quality is the contribution, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from data before changing anything. Author against measured reality.
- **A present sensor can still be silently failing.** RF2O published for months with zero covariance, and separately stalled for 8.5 s while scans kept flowing. The lgpio encoder backend claimed GPIO 22 successfully and delivered no edge callbacks at all (§4.6). "The topic exists" is not "the topic is honest or alive."
- **A present *estimator* can also be silently degraded.** Localization ran at 0.10 Hz correction rate while every topic looked healthy. Rates prove plumbing, not quality.
- **A consumer can silently reject data a publisher is correctly producing.** `/scan` was healthy at 9.7 Hz while Karto discarded ~79% of it (§4.12).
- **A signal's meaning can quietly widen past what it measures** (D-38). Four instances so far: `/cmd_vel.angular.z` (normalized steering vs. rad/s yaw rate); `/motor/direction` (commanded vs. actual travel direction); `map→odom` translation (frame-origin displacement vs. localization correction magnitude, D-41); and "autonomy arming" describing a gate that does not exist (D-48).
- **A metric can be right and still be reported wrong.** Two false findings — a 21% yaw scale error and inflated correction magnitudes — came from an unstated 78 s analysis window on a 174.7 s recording. **Every reported metric states its window** (D-45).
- **Velocity-only estimators coast blind through sensor silence.**
- **One owner per resource.** Every TF edge, serial port, PWM channel, and export lifecycle has exactly one writer.
- **A runnable launch must be complete on its own** (D-29).
- **Validation effort scales with hardware contact, not with perceived importance** (D-57).

---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** — tagged `phase-0-complete`. |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **In progress** — localization closed with numbers; autonomy bring-up at Stage 2 of 4. |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control. | Not started |

**Phase 1 — live status**

1. Relocalize on the saved map — **Done.**
2. Reliable seeding — **Done.** Root cause was scan-cardinality rejection (§4.12).
3. **Localization tracking stability — Done, quantitatively.** Closed by §4.10. Median induced pose jump 0.0188 m at speeds to 2.798 m/s.
4. **Nav2 point-to-point** — in progress, staged (§7.6).
5. Route-around, curved goals, reverse/recovery — reverse deferred to the board revision (D-46).
6. Frontier exploration — much later.

---

## 3 · Hardware

Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset via `pinctrl-rp1` by-label, D-12); US1881 hall encoder (GPIO 22, libgpiod, 0.010282 m/edge); X1201 UPS (compute only, I2C 0x36); DualSense teleop. LiDAR and IMU co-mounted, LiDAR the highest point.

### 3.1 Measured vehicle geometry (ratified — D-43)

| Quantity | Value | Basis |
|---|---|---|
| Wheelbase `L` | **0.178 m** | Measured. |
| Max steer `δ_max` | **17.5° = 0.3054 rad** | Direct wheel-angle measurement, cross-validated by U-turn geometry. |
| Minimum turning radius `R` | **0.565 m** | 44.5 in wall-parallel U-turn width (= 2R), agrees with `tan δ = L/R`. |
| Maximum curvature | **1.771140 m⁻¹** | `tan(δ_max)/L`. |
| Footprint (`base_link` at rear axle) | `[[0.235, 0.100], [0.235, −0.100], [−0.060, −0.100], [−0.060, 0.100]]` | Full-lock width 0.180 m + 10 mm margin/side. |

**Steering is not software-clamped.** Full ±500 µs about a 1500 µs centre, symmetric, confirmed by source trace and oscilloscope. The v0.8 "suspect — must be verified" flag is **closed**. `STEER_CTR = 1500` and `STEER_US = 500` are the conventional full range.

**Known limitation — steering hysteresis.** Releasing from left lock settles left of centre; from right lock, right of centre. This is linkage slop and the servo saver, **not** a trim offset — no software constant fixes it. Expect it to present to a path follower as a steering deadband around straight-line tracking. It must not be tuned against silently.

### 3.2 LD19 scan characteristics (measured, 4760 scans)

Fixed angular extent, **varying** resolution: `angle_min = 0.0`, `angle_max = 2π`; `len(ranges)` varies 495–509 (mode 504); `angle_increment = 2π/(N−1)` on every scan. Median `scan_time` 100.0 ms. This is the root cause of §4.12.

**Median valid scan range in the mapped apartment is 1.09 m.** This is a tight environment for a 0.565 m turning radius and constrains both planner reachability (§7.9) and yaw observability (§4.14).

### 3.3 Power architecture (D-51) — **new**

**Domains are separated by current path, not by energy source.** The objection to a single domain was always traction current flowing through X1201 traces and connectors — tens of amps through a HAT rated for Pi loads. A current-limited buck from traction into the UPS input does not do that.

Ratified architecture, for the post-Phase-1 board revision:

- **UPS is permanent and never removed.** Trickle-fed from a traction-derived buck whenever traction is connected. It reverts to being hold-up and graceful shutdown rather than a managed energy store.
- **Traction is bolted in and not hot-swappable.** At indoor speeds, session length is shorter than pack life; hot-swap solved a problem this vehicle does not have, at the cost of mechanical complexity and a shell conflict.
- **One USB-C PD input** negotiates 12–20 V, bucks to a 2S balanced charger for traction and to 5 V for the UPS input. Ideal-diode OR the two 5 V sources.
- **Pi-controlled high-side FET on ESC power.** This closes the oldest unsolved hazard on the platform: SIGKILL bypasses `stop()` and leaves the last PWM active. Under race mode a SIGKILL during forward throttle leaves the car accelerating. No software fix exists; the FET is the fix.
- **Traction voltage telemetry** via ADS1115 (I2C 0x48, no conflict with the MAX17040 at 0x36) and a ~3:1 divider, published as `/battery/traction`.
- **Contact pads, not wireless charging**, for the eventual dock. Pogo pins carry full current with no alignment problem; Qi tops out near 15 W with alignment and thermal losses.

**Measured constraint:** ~10 mA standing draw on traction. Determine whether this is ESC standby with the switch on (normal) or leakage with the switch off (a defect) before designing around it.

**Open gap:** `/battery` is the X1201 UPS fuel gauge and says **nothing** about the NiMH traction pack. Traction voltage is currently unobservable, which means a drooping pack and a misbehaving controller are indistinguishable. This is the direct justification for the ADS1115 item and it becomes load-bearing at Phase 3, where ILC assumes a repeatable plant.

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | RF2O scan-origin canonicalizer (D-37). |
| `runner_bringup` | `scan_rebinner` | `/scan` → `/scan_slam`, fixed 503-bin angular rebinning (D-37). |
| `runner_bringup` | `foxglove_goal_bridge` | `/move_base_simple/goal` → `NavigateToPose` actions (§7.6). |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz. |
| `runner_motor` | `motor_node` | `/cmd_vel` → ESC + steering PWM. Sole PWM owner. Requires `esc_mode` (D-46). |
| `runner_encoder` | `encoder_node` | Hall edges → `/wheel/odom`, `/wheel/encoder_state`. Sole GPIO 22 owner, via libgpiod (D-54). |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel`, three-state hold-to-run (D-48). |
| `runner_drive_adapter` | `drive_adapter` | `/cmd_vel_nav` (SI) → `/cmd_vel_auto` (normalized). D-55. |
| `runner_interfaces` | — | `EncoderState.msg`. `ament_cmake` + `rosidl_generate_interfaces`. |
| `runner_battery` | `battery_node` | UPS fuel gauge → `/battery` (systemd). |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan` (D-22). |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. Vendored fork. |

### 4.2 Launch tree (D-29)

```
launch/
├── map.launch.py        = sensors + estimation + slam_map      + teleop
├── localize.launch.py   = sensors + estimation + slam_localize + teleop
├── nav2.launch.py       = localize + map_server + planner + bt_navigator + goal bridge
├── teleop.launch.py     = joy + teleop_node + motor_node
└── include/
    ├── sensors / estimation / slam_map / slam_localize
    ├── lidar / imu / encoder / tf_static / rf2o / ekf   (atomic leaves)
    └── ekf_minimal / rf2o_origin_ab_test               (diagnostic only)
```

**Changed in v0.9:** `encoder_node` moved from a manually-started standalone task into the `sensors` tier. `Stop all launches` matches `ros2 launch runner_bringup` and `motor_node` but **not** `ros2 run`, so a standalone encoder survived every stop and held GPIO 22 into the next session with no error. A launch requiring a manual second process to produce its expected topic set is a D-29 defect. The standalone VS Code task and the ad-hoc Foxglove bridge task (port 8765 collision with `runner-foxglove.service`) are both deleted.

`esc_mode: race` is set in `teleop.launch.py`'s `motor_node` declaration and inherited by the composites. The parameter has no default; an unset launch refuses to start.

### 4.3 Scan topology (D-37)

```
LD19 /scan  (raw, variable 495–509 bins, single owner)
   ├─→ rf2o_scan_canonicalizer → /scan_rf2o (origin canonicalization) → RF2O
   ├─→ scan_rebinner           → /scan_slam (fixed 503 bins)          → slam_toolbox
   └─→ raw /scan → Nav2 costmaps (cardinality-agnostic)
```

**Nav2 costmaps consume raw `/scan`, never `/scan_slam`.** Every new map is built through `/scan_slam`.

### 4.4 Transform and topic ownership

| Resource | Sole owner |
|---|---|
| `map→odom` | slam_toolbox (mapping **or** localization mode) |
| `odom→base_link` | EKF |
| `base_link→base_laser`, `base_link→imu_link` | static |
| `/map` | **`map_server`** (D-50) |
| `/slam_map` | slam_toolbox (visualization only) |
| `/cmd_vel` | `twist_mux` (Stage 2 onward); `teleop_node` before that |
| ESC + steering PWM | `motor_node` |
| GPIO 22 | `encoder_node` |

RF2O TF is suppressed (`publish_tf: false` + `/tf→/tf_disabled`).

### 4.5 Static extrinsics

`base_link` = rear axle, ground-projected (D-01). `base_link→base_laser`: x 0.132, y 0, z 0.1135, yaw 0. `base_link→imu_link`: x 0.082, y 0.0025, z 0.106, yaw π. `laser_scan_dir: True` (D-19).

### 4.6 Encoder — velocity, direction, and GPIO

**GPIO backend (D-54).** The lgpio backend **claimed GPIO 22 successfully and delivered no edge callbacks.** A silently dead sensor reporting healthy. Replaced with `python3-libgpiod`, resolving the chip by live label (`pinctrl-rp1` → `/dev/gpiochip4`), requesting the line exclusively with consumer name `runner_encoder` and both-edge detection. This converges on the project convention already used by the IMU (D-12).

Filtering semantics changed with the backend and are **not equivalent**: lgpio required 100 µs of level stability; libgpiod forwards kernel events and the estimator rejects intervals shorter than 100 µs afterward. Zero sub-100 µs intervals have been observed in hand, low-speed, and full-throttle operation.

**Velocity estimator (D-49).** Fixed-window edge counting replaced with bounded interval timing:

```
edge_rate = (n − 1) / (t_last − t_first)
```

using kernel CLOCK_MONOTONIC event timestamps. `history_depth` is a ROS parameter, **default 4** (retaining 5 timestamps). No-edge decay caps the estimate at `1 / dt_since_last`. Stationary timeout 0.2 s unchanged.

The old estimator quantized badly: at 0.3 m/s it produced **only two distinct values**, 0.205640 and 0.411280 m/s, σ = 0.0938 m/s over 173 samples. Measured depth sweep at ~0.31 m/s:

| Depth | Coefficient of variation | σ vs depth 2 | Nominal added lag |
|---|---|---|---|
| 1 | 6.706% | — | 0 |
| 2 | 1.730% | baseline | 16.5 ms |
| **4** | ~1.4% | **−20.1%** | **49.4 ms** |
| 8 | ~1.35% | −21.7% | 115.3 ms |

Depth 4 measured at 0.313 m/s gives σ = 0.0034 m/s. Depth 8 adds ~2% smoothing for more than double the lag.

**Lag is speed-dependent** — the window spans `history_depth / edge_rate`, so effective lag is ~137 ms at 0.3 m/s and ~27 ms at 1.5 m/s at depth 8; proportionally less at depth 4. If a speed PID later tunes inconsistently across the range, a fixed-*time* window with a minimum interval count is the alternative to evaluate.

**Magnet spacing is period-2, not period-8.** July (scope, 207 periods at ~2900 RPM): 2.60 / 2.68 ms, ±1.5%. Current (kernel timestamps): low speed 31.13 / 34.73 ms, ±5.5%; full speed 1.559 / 1.749 ms, ±5.72%. Structure unchanged, magnitude materially larger. The measurement method also changed, so **this must be re-scoped on the signal line before any mechanical inspection.** No repeatable period-4 or period-8 component was found, so there is no evidence a magnet has shifted. Practical impact is nil: depth 2 cancels period-2 exactly and the default is 4.

**Direction signing (D-42).** `/wheel/odom` is signed by `pending_direction` — the latest valid nonzero command, latched through zeros — **not** by stop-gated `active_direction`. Measured over 533 steady-state samples (command held >1 s, EKF |vx| > 0.5):

| | disagreement with `sign(EKF vx)` |
|---|---|
| Commanded direction | 0.0% |
| **`pending_direction`** | **0.0%** |
| `active_direction` | **27.6%** |

`active_direction` sat on a stale sign for up to 5.25 s at speed, because the 200 ms stop gate is too coarse to fire during real driving. `pending_direction` errors are confined to transition windows (24 episodes, median 0.85 s) where the vehicle is decelerating through zero and the magnitude is small.

`active_direction` is retained as a stop-epoch diagnostic. Under race mode, direction is always forward for autonomy, so the sign is not load-bearing.

**Validity contract**, documented in `EncoderState.msg`: a single-channel encoder cannot observe rotation direction. The sign is command-informed and trustworthy across stop-delimited transitions only.

**Magnitude is unreliable under wheelspin.** Measured wheel/EKF speed ratio while moving: p50 1.01, p90 2.04, p99 4.61; 22.0% of moving samples exceed 1.5×, 24.4% fall below 0.7×. `/wheel/odom` is **not** an EKF velocity source.

**Consumers.** `edge_rate` (unsigned) → speed control. `stationary` → drive-adapter breakaway kick (§4.15) and a possible future zero-velocity update. The D-34 ESC handshake consumer was deleted by race mode.

### 4.7–4.9 RF2O, covariance, wheel odometry

Unchanged. Constant twist covariance `vx 0.02`, `vyaw 0.25` (D-24). Stall root-caused to high-rate INFO logging, fixed in `ece93d1`; max `/odom_rf2o` gap 0.213 s over the 174.7 s reference recording.

**RF2O `vy` is hardcoded to zero in the publisher.** There is nothing to fuse. Exposing it means another change to a vendored fork (D-33), and scan-matched lateral velocity degrades under exactly the fast rotation where it would be needed (§4.14). Lateral velocity remains unobserved — see §6 and the parking lot.

### 4.10 Localization quality — measured (D-32, D-41, D-45, D-52)

**Metric definition (D-41).** Correction magnitude is the **induced pose jump at `base_link`** — both consecutive `map→odom` transforms applied to the robot's current odom-frame position, distance taken between results. It is **not** the translation of `map→odom`, which is inflated by the robot's distance from the odom origin and is therefore not comparable across bags. Measured inflation on the reference recording: **7.259×** at a mean lever arm of 5.94 m.

**Cadence is conditioned on motion (D-52).** slam_toolbox is motion-gated (`minimum_travel_distance: 0.05`), so an unconditioned rate partly measures how long the vehicle stood still. All long gaps in the reference recording occur at mean speed < 0.04 m/s.

**Reference measurement.** `runner_test_20260726_115736`, full 174.684 s recording, **analysis window t+20 s → end (154.2 s, 97.7 m)**:

| Metric | Value |
|---|---|
| **Pose jump at `base_link`** — med / p90 / p95 / max | **0.0188 / 0.0658 / 0.0959 / 0.3556 m** |
| Yaw correction — med / p95 / max | 0.0175 / 0.1396 / 0.3560 rad |
| Corrections | 568 |
| Cadence — unconditioned / motion-conditioned | 3.683 / **5.717 Hz** |
| Gap — med / p90 / p95 | 0.180 / 0.260 / 0.311 s |
| Legacy raw `map→odom` translation — med / p95 | 0.1082 / 0.8323 m |
| Total correction per metre travelled | 0.175 m/m |

**Driving profile:** mean |vx| 0.626 m/s, p95 1.717, **max 2.798 m/s**; |ω| p95 1.991, max 3.534 rad/s; 52.4% throttle duty; deliberate drifting and heavy wheelspin throughout.

**Health:** `/scan` 9.85 Hz max gap 0.202 s; `/odom_rf2o` 9.82 Hz max gap 0.213 s. No stalls.

**Against the §4.10 v0.8 baseline** (0.103 Hz, gap p95 41.24 s, max 55.76 s, 135.7 s at max 1.02 m/s and 25% duty): **35.8× cadence improvement at 2.7× the top speed.** The baseline's translation figures are on the legacy metric and a different route; they are **not** comparable in magnitude and must be re-run through the corrected metric before being cited.

**Mild speed dependence** of correction magnitude:

| Speed band | n | jump med | yaw med |
|---|---|---|---|
| 0.5–1.0 m/s | 219 | 0.0143 m | 0.0140 rad |
| 1.0–1.5 m/s | 172 | 0.0198 m | 0.0175 rad |
| 1.5+ m/s | 104 | 0.0231 m | 0.0279 rad |

**Perspective.** Prior work (Liniger, BARC, F1TENTH) uses motion capture at sub-millimetre accuracy. Runner is 20–100× worse. That gap, characterized in numbers, *is* the contribution.

### 4.11 Instrumentation

`tools/analyze_localization_bag.py` — invariant pose-jump metric, lever-arm reporting, `--start-time`/`--end-time` windows, truncated-MCAP sequential recovery. `tools/analyze_yaw_cross_validation.py` — per-source integrated yaw against the slam reference, binned by yaw rate. `tools/analyze_throttle_response.py` — steady-state segmentation, throttle→speed curve, deadband bracket. `docs/throttle_characterization_protocol.md`, `docs/yaw_cross_validation_protocol.md`.

**Known analyzer limitation:** the yaw tool sums *signed* yaw within each rate bin, so left and right rotation cancel and per-bin ratios are numerically meaningless. The aggregate is sound. Fix by accumulating absolute rotation if the binned view is needed again.

### 4.12 Scan cardinality (D-37)

Unchanged from v0.8. Karto registers a fixed beam count once and hard-rejects mismatched scans; 79% of the stream was silently discarded. Fixed by `scan_rebinner` — angular rebinning onto a fixed 503-bin grid, nearest-in-angle, `inf` for empty bins, header stamp preserved. Pad/truncate, interpolation, driver fork, and canonicalizer merging all explicitly rejected with reasons; **do not revisit.**

### 4.13 Phantom-geometry trap (D-40)

Unchanged. Localization mode retains recent scans (`scan_buffer_size: 10`); scans taken while mislocalized depict the real room at the wrong map location, and the matcher reinforces its own error. Failure is a **cliff, not a gradient**. Recovery requires detection plus explicit reset.

**Not observed post-D-37.** The trap could not be induced during the reference recording. Note that the D-32 scalar **cannot by itself distinguish tracking success from the trap** — small, frequent, well-behaved corrections is also the trap's signature. Supporting evidence in the reference recording: net `map→odom` drift 0.013 rad and 0.90 m over 154 s with no jump-then-quiet signature.

### 4.14 Yaw lag from scan motion distortion (D-53) — **new**

**D-44 is amended, not reversed.** There is **no yaw scale error.** Integrated over the full 174.684 s recording against the slam reference: IMU **1.0023×**, EKF **1.0040×**, RF2O 0.8862×. The v0.8-era 21% figure was an artifact of an unstated 78 s window (D-45).

**But there is a yaw-rate-proportional phase lag.** `map→odom` yaw *is* the running slam-versus-EKF heading disagreement. Regressing it against signed yaw rate (n = 3897, |ω| > 0.2):

**slope = −0.0652 s, correlation −0.596, 35% of variance explained**

| Signed yaw rate | n | Mean `map→odom` yaw | Implied lag |
|---|---|---|---|
| −4.0 … −2.0 | 264 | +0.1249 rad | 0.050 s |
| −1.0 … −0.3 | 846 | +0.0002 | ~0 |
| +1.0 … +2.0 | 519 | −0.1417 | 0.100 s |
| +2.0 … +4.0 | 114 | −0.2351 rad | 0.098 s |

Sign flips exactly with turn direction; magnitude grows with yaw rate; near zero when straight.

**The fitted 65 ms lag is approximately half a LiDAR scan period.** The LD19 sweeps over 100 ms and nothing in the scan path deskews, so Karto's best-fit pose corresponds to roughly the middle of the sweep. **The pose is not wrong; it is late.**

This reconciles three previously contradictory observations: integrated yaw agrees to 0.4% (a lag nets to zero over a closed path); corrections oppose the turn direction 75% of the time at high yaw rate; and yaw corrections grow with yaw rate. Integrated comparison is structurally blind to lag.

**Consequence, by speed:**

| Yaw rate | Heading error | Scan shift at 1.09 m |
|---|---|---|
| 0.5 rad/s | 1.9° | 3.6 cm |
| 1.0 rad/s | 3.7° | 7.1 cm |
| 3.0 rad/s | 11.2° | 21 cm |

Negligible at Phase 1 speeds, dominant at racing speeds. **The fix is IMU-based scan deskewing** — un-rotate each beam to a common timestamp using the 50 Hz gyro before matching. Deferred; it becomes load-bearing at Phase 3 and it is a speed-dependent, mechanistic, measured bound on localization quality, which is close to the thesis claim itself.

Two caveats: a single constant lag explains 35% of the variance, not all of it; and left and right turns give different implied lags (0.02–0.05 s vs 0.10–0.135 s), which a pure scan-timing effect should not produce.

### 4.15 Drive adapter (D-55) — **new**

Resolves the `/cmd_vel` unit mismatch. Nav2 publishes a true `Twist` (m/s, rad/s); `motor_node` consumes normalized ±1.0 in both fields. Direct connection produces ~1.7× oversteer at 0.5 rad/s and 1 m/s, saturating to full lock above 1.0 rad/s, with speed-dependent error.

```
Nav2 → /cmd_vel_nav (SI) → drive_adapter → /cmd_vel_auto (normalized) → twist_mux → /cmd_vel → motor_node
```

**Steering:** `δ = atan(L·ω/v)`, normalized by 0.3054, using L = 0.178.

**Throttle:** feedforward lookup, no integrator. Measured table (`throttle_setpoint_20260727_171906`):

| Normalized throttle | Speed |
|---|---|
| 0.340 | 0.126 m/s |
| 0.350 | 0.188 |
| 0.360 | 0.233 |
| 0.380 | 0.290 |

The measured 0.370 → 0.209 m/s point is **excluded** as a single sample with σ = 0.094 m/s; 0.360 was measured twice at 0.229 and 0.237. Requests above 0.290 m/s clamp to 0.380.

**Breakaway is direction-dependent.** In one direction nothing below 0.380 started the car from rest; in the opposite direction 0.340 did. Three confounded causes (floor slope, drivetrain warm-up, sweep order) could not be separated. `breakaway_throttle = 0.380` is the worst case observed and is conservative by construction.

**Parameters:** `steering_min_speed` 0.05 m/s; `minimum_moving_speed` 0.126 m/s; `floor_promotion_min_ratio` 0.50 (promotion band 0.063–0.126 m/s); `breakaway_throttle` 0.380; `breakaway_timeout` 0.75 s; `motion_confirm_edge_rate` 1.0 edges/s (≈3× the 0.331 edges/s speed-noise equivalent); `cmd_vel_nav_timeout` 0.25 s; `encoder_state_timeout` 0.25 s; 20 Hz publication.

**Breakaway kick** is gated on `/wheel/encoder_state.stationary`, ends on motion confirmation or timeout, and **re-arms whenever `stationary` transitions to false** — so a mid-path stall gets a fresh kick, while a vehicle that never moves gets exactly one.

**Stale input produces silence, not brake.** Publishing brake on stale input would keep commands flowing to `/cmd_vel` and prevent the motor watchdog (D-09) from firing, masking a dead Nav2 from the mechanism designated as primary safety.

**Steering infeasibility brakes; it does not reduce speed (D-56).** The v0.9-draft requirement to "reduce speed under saturation" was **wrong**: curvature is ω/v, so at fixed ω, reducing v *increases* the required steer angle. Brake-on-infeasible is correct for bench validation but may be too harsh in practice — RPP can emit transiently infeasible commands when correcting back onto a path, and stopping dead mid-corner would present as a controller fault. **Instrument the `steering_infeasible` rate during Stage 3**; if it is not rare, change the policy to clamping at maximum steer and accepting understeer.

**Measured (bench, wheels off ground):** publication 20.002 Hz, σ 0.00013 s; kick start latency 18.7 ms, motion-confirmed end 23.2 ms, timeout 749.988 ms, zero restarts during a 350 ms post-timeout stationary window; staleness silence confirmed. Steering sign composed from two independent checks: stick-left → `/cmd_vel.angular.z` = +1.0 → physical left; adapter ω = +0.20 → +0.5768. **Positive Nav2 yaw commands physical-left steering.**

---

## 5 · Teleop & motor control

### 5.1 ESC race mode (D-46)

The ESC is physically in race mode: **reverse is disabled and the reverse channel is proportional brake only.** Bench-verified — forward throttle then reverse stick brakes immediately; a second reverse command at rest does nothing; drag brake is felt when pushing by hand.

**This is preliminary, not permanent.** Reverse returns via a brushed H-bridge in the post-Phase-1 board revision, where direction is which FET pair conducts and no arming state machine exists. Phase 1 (§7.4) was always forward-only, so no scope is lost.

What it buys immediately: the D-34 ESC reverse handshake becomes unnecessary; full 1000 µs brake authority instead of a 1250 µs floor sized to bound reverse throttle; dead-man release and watchdog timeout become real braking events; direction sign becomes irrelevant to autonomy; and the hardware now matches what the planner already believed.

What it costs: manual recovery when the vehicle noses into an obstacle or a goal needs a three-point turn. With R = 0.565 m in a 1.09 m-median-range apartment, a meaningful fraction of indoor goals are unreachable. **Phase 1 success is defined as reaching goals in open areas, not all goals.**

**`esc_mode` has no default and the node refuses to start unset.** In `normal` mode the same brake-on-loss behaviour would command full-speed reverse. The `normal` code path is retained, not deleted.

Braking costs essentially no battery: dynamic braking shorts the motor through the FETs, dissipating kinetic energy. At rest there is no back-EMF and no current. It is **not** a parking brake — holding force is proportional to speed, so the vehicle rolls on a slope.

### 5.2 Teleop three-state hold-to-run (D-48)

Priority **X > R1 > L1 > brake**. Releasing all buttons brakes within one publication cycle.

| Input | Behaviour |
|---|---|
| Nothing held | full brake, `linear.x = −1.0` |
| **X** held | manual — R2 throttle, L2 brake, stick steering |
| **R1** held | fixed throttle setpoint (D-pad adjusts), stick steering, L2 brake live |
| **L1** held | `teleop_suppress` — publishes nothing, so autonomy passes through |

**Naming discipline (D-48).** In code and documentation L1 is **`teleop_suppress`**. "Autonomy enable" is operator-facing language only. It is *not* an arming gate — the actual gate is the mux and the motor watchdog. Describing it as arming would be a fifth instance of D-38.

**X→R1 latch:** pressing X while R1 is held suppresses fixed throttle until R1 is released and re-pressed, preventing surprise re-acceleration on X release.

**Fixed-throttle mode** is a first-class operator mode, not a hidden diagnostic. Beyond finding breakaway, it makes throttle characterization and localization runs repeatable — the previous hand-held analog-trigger segments were the reason the low end of the speed curve came back mushy. Parameters: initial setpoint 0.30 (below breakaway), step 0.01 (≈1.4 µs of pulse near breakaway), min 0.00, max 0.50. Diagnostics `/teleop/fixed_throttle_setpoint` and `/teleop/active_mode` publish continuously, including during suppression, so bag analysis can distinguish R1 samples from X samples.

**Measured:** R1 release-to-brake 5.065 ms; L1 release-to-brake 41.350 ms; watchdog fires 171.464 ms after teleop loss, inside the 0.2 s budget. The L1 asymmetry is because release means *resuming publication* rather than changing a value — the `twist_mux` teleop timeout must comfortably exceed it.

### 5.3 Motor control

ESC curve deadband + expo, `THR_MAX_US = 1750`, forward onset 1550 µs, crossover 0.05, exponent 2.0 (D-08). PWM export owned by `runner-pwm-setup.service` (D-23). Watchdog is primary safety (D-09). SIGINT the motor node, never SIGKILL.

**Do not change the throttle expo.** The motor sees pulse width, not normalized commands; breakaway is at ~1574 µs and is a physical ESC/motor/drivetrain threshold, confirmed by the stand test where unloaded wheels would not turn at command 0.3. Changing the exponent moves which normalized value maps to that pulse width without changing what it produces. The drive adapter inverts the curve by lookup and absorbs the exponent entirely, so expo costs human stick resolution only. The 1550 µs onset sits deliberately at the edge of the ESC's own neutral deadband.

**Residual hazard, unsolved in software:** SIGKILL bypasses `stop()` and leaves the last PWM active. Under race mode, SIGKILL during forward throttle leaves the vehicle accelerating. The Pi-controlled high-side FET (§3.3) is the fix.

---

## 6 · Failure taxonomy: wheelspin vs fishtail (D-27)

Straight-line wheelspin is tolerated (pure-X error, usually snaps back). Fishtail is the repeatable break: real lateral `vy` the vx-only EKF cannot represent.

**Lateral velocity is unobserved on this platform.** RF2O's `vy` is hardcoded to zero (§4.7); the wheel encoder cannot provide it (D-26); the EKF has no `vy` state. This is a *sensing* gap, not a data gap — no amount of recorded driving produces slip labels, because every sample would be labelled `vy = 0`.

**The IMU offers an indirect route.** Body-frame kinematics give `v̇y = a_y − vx·ω`, and every term on the right is already measured — `a_y` from the BNO085 at 50 Hz, `vx` and `ω` from the EKF. When the vehicle tracks properly, `a_y` should equal `vx·ω` exactly; the residual is a direct slip signal at five times RF2O's rate, from a sensor that does not degrade under rotation. Accelerometer bias makes it a transient detector rather than an absolute `vy` source — which suits a fishtail, since a fishtail *is* a transient. Body roll projects gravity into the y axis (5° ≈ 0.85 m/s²), partially handled by the BNO085's gravity-compensated output.

**Untested.** The reference recording contains both topics throughout and can validate this offline. Parked.

§4.13 supplies the mechanism for non-recovery: a fishtail can push the estimate outside the convergence basin, after which phantom geometry holds it there.

---

## 7 · Phase 1: Nav2 point-to-point

### 7.1 Architecture

`planner_server` (Smac Hybrid-A\*) owns the **global costmap**, built on the static map. `controller_server` (Regulated Pure Pursuit) owns the **local costmap**, built on live raw `/scan`, and publishes `/cmd_vel_nav`.

**Smac produces a geometric path with no timing.** Speed is entirely RPP's, decided reactively: a base `desired_linear_vel` scaled down for curvature (fires on nearly every turn at R = 0.565 m), obstacle proximity, and goal approach. It is a heuristic reacting to what is directly ahead — it has no knowledge of a corner two metres away and no concept of the fastest way through the one it is in. Replacing this with a time-parameterized trajectory is the Phase 2 change, and it is larger than swapping controllers.

### 7.2 Stage 1 as built — planning-only (committed)

- `map_server` owns `/map`, serving the static `.pgm`/`.yaml`. **slam_toolbox's map output is remapped to `/slam_map`** — its published bounds vary within a session (D-40: width 254–428, origin x −14.097 to −11.342), so it cannot back a global costmap.
- slam_toolbox retains `map→odom`. **No AMCL** — it would contend for that edge.
- Smac Hybrid-A\*, **DUBIN**, `minimum_turning_radius: 0.565`, `allow_unknown: false`. Reverse disabled through the motion model; there is no separate reverse parameter.
- Static map **re-exported at 0.025 m/cell** (was 0.050; at 0.05 the vehicle is only 3.4 cells wide, too coarse for Smac footprint checking). The 0.050 pair is preserved under backup names. `.posegraph`/`.data` unmodified.
- **Inflation radius 0.30 m, cost scaling 5.0.** Nav2's 0.55 m default would flood the interior with elevated cost, flattening the gradient the planner uses to prefer open routes. Validated: a 0.98 m doorway retains a ~0.85 m sub-inscribed channel with a ~0.33 m zero-cost core.
- **Planning-only behaviour tree** containing only `ComputePathToPose` — no `FollowPath`, `Spin`, `BackUp`, `RecoveryNode`, or `ClearEntireCostmap`.
- **`foxglove_goal_bridge`** converts `/move_base_simple/goal` → `NavigateToPose`, latest-goal-wins, generation-guarded, bounded cancellation retry.
- Local costmap **omitted** in this stage — it belongs to `controller_server`, which is excluded. The raw-`/scan` requirement carries forward.

**Results:** valid curvature-constrained paths produced; planning ~0.26 s; path publication latency ~5 ms; impossible goals abort cleanly with `NO_VALID_PATH` (error 208). One failed goal required a large heading reversal under forward-only Dubins — correctly **not** classified as turning-radius-only, though practically that *is* the §7.9 failure.

### 7.3 Remaining stages

| Stage | Content | Status |
|---|---|---|
| 1 | Drive adapter (§4.15), bench only | **Complete**, `bce8c68` |
| 2 | `twist_mux` + two-gate override (D-35), bench | **In progress** |
| 3 | `controller_server` + local costmap, no driving | Blocked |
| 4 | First floor goal | Blocked |

**Stage 2 notes.** Teleop `/cmd_vel_teleop`, adapter `/cmd_vel_auto`, mux → `/cmd_vel`. Teleop strictly higher priority. Because teleop publishes nothing while L1 is held, the mux falls through to autonomy on a timeout — so autonomy *engagement* is delayed while teleop *preemption* is immediate. That asymmetry is correct. Set the teleop input timeout deliberately (~0.2 s, comfortably above the 41 ms L1 release latency). **Verify the watchdog still fires with the mux in the path** — a mux republishing a stale message would defeat it.

**Stage 3 notes.** Set `min_approach_linear_velocity` to the measured floor so RPP never emits sub-floor commands; the adapter's floor policy becomes a safety net rather than a routine path. Cap maximum speed at the top of the measured throttle table. Local costmap rolling-window size must suit this vehicle. Restore `FollowPath` only — not `Spin` or `BackUp`.

**Stage 4 notes.** Open area, forward-only, 0.5–1 m goal, ~0.25 m/s (inside measured table range), obstacle avoidance active from the first powered test. **This is the first measurement of open-loop speed error** (§7.11) and determines whether a speed PID is needed.

### 7.4–7.11 Standing constraints

Obstacle avoidance active from the first powered test. Impossible routes fail the goal safely. **Speed ceiling is scan-rate-bound** — the LD19's 10 Hz refresh, not Pi 5 CPU, sets local-costmap responsiveness. **Collision Monitor deferred** (§7.8) — primary safety is the motor watchdog plus hand override. **Reeds-Shepp matters more than usual** (§7.9) — forward-only Dubins with a 0.565 m radius will fail a meaningful fraction of indoor goals; count those failures separately as evidence. **Rotate-in-place removed from the BT** (§7.10) — an Ackermann vehicle cannot execute it.

**§7.11 — speed is open-loop.** Nav2 commands m/s; the ESC delivers whatever pack voltage, friction, and incline allow. Nothing measures or corrects it, and traction voltage is unobservable (§3.3). A speed PID on `edge_rate` is the legitimate fix, **deferred until Stage 4 measures the error**. **Steering does not get a PID** — no servo feedback sensor exists, and Nav2 is already the outer loop.

---

## 8 · Open items

**Active**
- Nav2 Stage 2 (`twist_mux`), then Stages 3–4.
- Re-run `analyze_localization_bag.py` on the v0.8 §4.10 baseline bag with the corrected metric, so magnitude figures become comparable.

**Ratified, not implemented**
- Pi telemetry node (temp + sticky `vcgencmd get_throttled` bits, 1 Hz, systemd).
- ADS1115 traction voltage → `/battery/traction` (§3.3).
- Board revision: PD sink, 2S balanced charger, Pi-controlled high-side FET, brushed H-bridge, IMU, IO breakout (§3.3).
- Fix `analyze_yaw_cross_validation.py` to accumulate absolute rotation per bin (§4.11).

**Deferred with stated triggers**
- **IMU scan deskewing** — trigger: Phase 2 speeds, where the 65 ms lag becomes dominant (§4.14).
- **Lateral velocity observation** — IMU residual method (§6) is untested and free; optical flow ruled out for this vehicle.
- **Reeds-Shepp / reverse** — trigger: post-board-revision H-bridge (D-46).
- **Quadrature encoder** — trigger: a consumer of signed wheel velocity emerging. None exists under race mode.
- **Scope re-measurement of magnet period-2 growth** before any mechanical inspection (§4.6).
- **UWB** — trigger: observed perceptual aliasing. Not observed.
- **Docking station** — Phase 2/3. Charger, UWB anchor, and repeatable session origin. Contact pads, not wireless.
- **Phantom-trap questions** (§4.13) — `scan_buffer_size` sensitivity; whether reseeding clears the buffer. Cannot currently be induced.
- **Grey-box vehicle model** — bicycle nominal plus learned residual. This *is* LMPC's structure, not an alternative to it.

**Carried forward**
- RF2O duplicate node identity (D-33) — launch-layer only, verify param keying first.
- Magnetometer heading — re-measure against post-D-37 localization.
- Vendored RF2O fork divergence.
- Extend the throttle table above 0.290 m/s.
- Halo/crash protection. `24e5ad4` un-revert. `scripts/deadband_diagnostic.py` flake8.

---

## 9 · Decision log

Append-only. D-01…D-40 unchanged (v0.5–v0.8).

| ID | Decision | Reasoning |
|---|---|---|
| D-41 | **Localization correction magnitude is the induced pose jump at `base_link`**, not `map→odom` translation. Both are reported; only the former is comparable across bags. | `map→odom` is a rigid transform applied at a lever arm, so identical pose corrections produce larger translations the further the robot sits from the odom origin. Measured inflation 7.259× at a 5.94 m mean lever arm. Third instance of D-38. |
| D-42 | **`/wheel/odom` is signed by `pending_direction`** (latest nonzero command, latched through zeros), not stop-gated `active_direction`. `active_direction` retained as a diagnostic. | Steady-state disagreement with `sign(EKF vx)`: `pending` 0.0%, `active` 27.6%. The 200 ms stop gate is too coarse to fire during real driving, so `active` held a stale sign for up to 5.25 s at speed. `pending` errors are confined to transition windows where speed is near zero. |
| D-43 | **Steering is not software-clamped.** `δ_max = 17.5°`, `R = 0.565 m` are ratified planner inputs. | Full ±500 µs about 1500 µs, symmetric, confirmed by source trace and oscilloscope. Two independent geometric methods agree on 17.5°. Retires the v0.8 "suspect" flag. |
| D-44 | **The yaw scale hypothesis is withdrawn.** Over the full recording IMU integrates to 1.0023× and EKF to 1.0040× the slam reference. | The 21% figure was an artifact of an unstated 78 s window on a 174.7 s bag. RF2O under-reports by 11%, consistent with scan matching under-observing rotation, and contributes negligibly to fused yaw (D-24). **Amended by D-53.** |
| D-45 | **Every reported metric states its analysis window.** | Two false findings — a 21% yaw scale error and inflated correction magnitudes — both traced to an unstated window. The window is part of the result, not context. |
| D-46 | **ESC race mode: forward + proportional brake only.** Reverse deferred to the H-bridge board revision. Preliminary, not permanent. | Deletes the D-34 reverse handshake, grants full 1000 µs brake authority, makes dead-man release and watchdog timeout real braking events, and renders direction sign irrelevant to autonomy. Phase 1 (§7.4) was always forward-only. Cost is manual recovery, bounded by defining Phase 1 success as open-area goals. |
| D-47 | **Dead-man release and watchdog timeout command full brake**, gated on `esc_mode: race`. The parameter has no default; the node refuses to start unset. | Brake is strictly safer than coast on command loss. In `normal` mode the identical behaviour would command full-speed reverse, so the interlock is mandatory rather than defensive. |
| D-48 | **Teleop is three-state hold-to-run: X > R1 > L1 > brake.** In code, L1 is `teleop_suppress`, not "arming". | Unconditional brake-on-release would let teleop's mux priority make autonomy unreachable, so a third state that publishes nothing is required. Fixed throttle is a first-class mode because hand-held analog-trigger segments were the reason the low end of the speed curve was unusable. "Arming" describes a gate that does not exist — the real gate is the mux and the watchdog — and naming it so would be a fifth instance of D-38. |
| D-49 | **Encoder velocity from bounded interval timing**, `history_depth` a ROS parameter, default 4. | Fixed-window counting quantized 0.3 m/s to exactly two values (0.206 / 0.411 m/s, σ 0.094). Measured sweep: depth 1→2 removes 74% of variation, confirming the period-2 magnet component; depth 8 adds ~2% over depth 4 for more than double the lag. |
| D-50 | **`map_server` owns `/map`; slam_toolbox's map output is remapped to `/slam_map`.** | slam_toolbox's published bounds vary within a session (D-40) because `updateMap()` regenerates from currently-buffered scans. A global costmap built on a shifting grid would move under the planner. |
| D-51 | **Power domains are separated by current path, not energy source.** Traction may trickle-charge the UPS through a current-limited buck. UPS permanent; traction bolted in, not hot-swappable. | The objection was always traction current through X1201 traces, not shared energy. Hot-swap solved a problem indoor session lengths do not create, at the cost of mechanical complexity and a shell conflict. |
| D-52 | **Correction cadence is conditioned on vehicle motion.** Gaps spanning stationary periods are excluded and reported separately. | slam_toolbox is motion-gated, so an unconditioned rate partly measures how long the vehicle stood still. Two bags with identical estimator performance but different parked time would score differently — the same class of defect as D-41. |
| D-53 | **slam yaw lags true heading by ~65 ms — approximately half a LiDAR scan period — because Karto matches a 100 ms sweep as instantaneous.** Amends D-44: no scale error, but a yaw-rate-proportional phase lag. Fix is IMU-based scan deskewing, deferred. | Regressing `map→odom` yaw against signed yaw rate gives slope −0.0652 s, correlation −0.596, sign flipping exactly with turn direction. Reconciles integrated agreement to 0.4% (a lag nets to zero over a closed path) with corrections opposing the turn 75% of the time at high yaw rate. Integrated comparison is structurally blind to lag. Speed-dependent and mechanistic, which places it close to the thesis claim. |
| D-54 | **Encoder GPIO uses libgpiod, resolving the chip by live label.** Retires lgpio for this input. | lgpio claimed GPIO 22 successfully and delivered **no** edge callbacks — a silently dead sensor reporting healthy, and the second lgpio failure on this pin. libgpiod is already the project convention (D-12). Filtering semantics are not equivalent and are documented in §4.6. |
| D-55 | **Drive adapter owns SI-to-normalized conversion.** `/cmd_vel_nav` (SI) → `/cmd_vel_auto` (normalized). `motor_node`'s contract is unchanged. Throttle is feedforward-only from a measured lookup table. | Nav2 and `motor_node` use different units on the same message type — the live instance of D-38, producing ~1.7× oversteer at 0.5 rad/s and 1 m/s with speed-dependent error. Explicit topic names make the boundary visible. Feedforward avoids integrator wind-up and keeps failure deterministic; a PID, if Stage 4 justifies one, goes inside this node and affects only the autonomy path. |
| D-56 | **Steering infeasibility brakes; it does not reduce speed.** | The reduce-speed instruction was wrong: curvature is ω/v, so at fixed ω reducing v *increases* required curvature. Brake-on-infeasible is correct for bench validation, but RPP can emit transiently infeasible commands when correcting onto a path, so the rate must be instrumented at Stage 3 and the policy changed to clamp-and-understeer if it is not rare. |
| D-57 | **Validation effort scales with hardware contact, not perceived importance.** Three tiers: (1) full bench acceptance for anything that can command the motor autonomously for the first time; (2) smoke check only — starts, claims resources, publishes ten seconds — for nodes that touch hardware but cannot drive it; (3) unit tests only for pure computation. | Heavy acceptance caught three real failures, all hardware-contact: the lgpio dead-callback path, and two deliveries that built green and did not start. It produced no value on deterministic logic already covered by unit tests, where a 13-case acceptance table and microsecond state-transition timing were ceremony. Re-verifying closed work is never justified — check the commit log instead. |

---

## Appendix · Workflow conventions

**Spec discipline.** The spec is a complete standalone reference, not delta-only. Divergent changes require a new decision-log entry.

**Codex:** read-only investigation first, then a separate action prompt against confirmed reality. Codex commits and pushes.

**Executor chats:** paste `runner_collab_protocol.md`, then platform context, then the task brief. New chat per task. Executors escalate before acting on one-owner resources, phase scope, or decision-log changes (D-36).

**Validation tiers (D-57).** Match the tier to hardware contact. Do not re-verify closed work. Do not ask an executor to confirm something checkable with `git log`.

**Measurement:** before/after comparisons use MCAP bags over a comparable route and speed profile, analyzed by the same in-repo script. Report the driving profile alongside the metrics. **State the analysis window** (D-45). "Feels better" is not a result.

**Standing rules:** always `source install/setup.bash` after a build. A runnable launch must be complete on its own and ships with its VS Code task in the same change (D-29). Every new map is built through `/scan_slam` (D-37). A green build is not evidence a node runs.

CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.