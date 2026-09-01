# Runner — Architecture & Current-State Specification

**Version 1.2** · supersedes v1.1 · 2026-08-11
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

> **Version status.** v1.2 records the completed MD13S plant characterization, the frozen longitudinal controller, the signed adapter path, reverse-capable planning and control, and the speed-envelope consolidation. Phase 1 was re-validated against the MD13S. No usable map bundle is currently committed, so localization and autonomy require a newly generated bundle. Entries that are **ratified but not implemented** are marked inline.

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. All compute and sensing is onboard — no external motion capture, no fixed anchors.

**Runner is explicitly not the thesis vehicle.** If a thesis follows from this work it will be built on a new platform from scratch with better hardware. Runner exists to reach interesting autonomy frontiers, to expose its operator to the problem space, and to identify angles a thesis might take. This reframing (2026-08-11) has consequences that propagate through the whole document: lap-time convergence data is no longer a deliverable with a deadline, breadth of capability is worth more than depth in any single subsystem, and operator tooling is worth building because it will be used for years rather than for one experiment.

**What survives the reframing.** The engineering discipline is why this platform works rather than being a pile of half-finished features, and it is retained without exception. Safety is retained without exception — the stairs incident (§3.6) was not a thesis problem.

**Demonstrated capability.** Autonomous multi-waypoint navigation in a cluttered apartment, including tight-clearance maneuvering between furniture legs, U-turns, and — since 2026-08-01 — reverse-capable planning with three-point turns. The old map artifacts have since been removed; there is currently no committed usable map bundle.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed.
- **Prototype-first.** Localization quality is the interesting variable, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from data before changing anything.
- **Measure before building.** Characterize the plant from data before designing controllers around it. Never bake surface-dependent constants into a feedforward.
- **A present sensor can still be silently failing.** RF2O published for months with zero covariance and separately stalled for 8.5 s while scans flowed. The lgpio encoder backend claimed GPIO 22 and delivered no callbacks. The global costmap's obstacle layer marked **zero** cells for weeks because its observation source loaded `max_obstacle_height: 0.0`.
- **A present estimator can be silently degraded.** Localization ran at 0.10 Hz correction rate while every topic looked healthy.
- **A consumer can silently reject data a publisher is correctly producing** (§4.12). Seventh instance, 2026-08-01: `foxglove_bridge` decoded `AdapterState` against a stale schema, reading a string length from inside a float64 — and failed *only while driving*, because parked those fields are all-zero bytes and the misparse silently yielded an empty string (§4.18).
- **A gate that blocks actuation must publish why** (D-75).
- **A signal's meaning can quietly widen past what it measures** (D-38). Eight instances: `/cmd_vel.angular.z`; `/motor/direction` (twice — D-42 and D-74); `map→odom` translation as correction magnitude (D-41); "autonomy arming" describing a gate that does not exist (D-48); `/plan` treated as the path `FollowPath` receives (D-61); the R1/L2 fixed-versus-proportional asymmetry documented in code and tests but invisible in §5.2 (D-83); and **"turning radius" collapsing the outer-wheel swept radius, the base-link physical minimum, and the planner's headroom value into one name** (§3.1).
- **A metric can be right and reported wrong.** Every reported metric states its analysis window (D-45).
- **A fix can create the next failure.** Sequence matters as much as correctness.
- **A constraint enforced by firmware disappears when the firmware does** (D-70).
- **When a tool result contradicts a directly observed physical outcome, the tool is the suspect.** Established 2026-08-01 after an executor declared a bag empty against the operator's direct observation of the wheels turning; the bag contained 4295 messages and a valid summary.
- **One owner per resource.**
- **A runnable launch must be complete on its own** (D-29, amended by D-71).
- **Validation effort scales with hardware contact, not perceived importance** (D-57).
---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **Complete and re-validated against the MD13S.** Reverse autonomy added. |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. | Not started — **no longer deadline-bearing** (§1) |

**Hardware freeze — status changed.** Under the previous framing, Phase 2 lap-time comparison required a frozen platform before the first baseline run. With Runner no longer the thesis vehicle, that constraint is lifted: the platform may continue changing, and a Phase 2 baseline will be re-established whenever one is actually wanted.

**The one property still worth preserving from that framing:** LMPC's premise is that lap *N+1* improves on lap *N* using data from previous laps, so repeated-lap capability remains the prerequisite for the most interesting control work available on this platform. Phase 2 stops being a deliverable and becomes an enabler.

---

## 3 · Hardware

Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset via `pinctrl-rp1` by-label, D-12); US1881 hall encoder (GPIO 22, libgpiod, 0.010282 m/edge); **Cytron MD13S brushed H-bridge motor driver** (D-70); X1201 UPS (compute only, I2C 0x36); DualSense teleop plus laptop-side keyboard teleop (D-68). LiDAR and IMU co-mounted on a single electronics sled.

### 3.1 Measured vehicle geometry (ratified)

| Quantity | Value | Basis |
|---|---|---|
| Wheelbase `L` | **0.178 m** | Measured |
| Max steer `δ_max` | **0.3614 rad (20.7°)** | Derived from fitted turning radius (D-43 amended) |
| **Physical minimum turning radius** | **0.470 m** | Circle fit to a 1034° sustained turn (residual sd 0.017 m) and a 123° U-turn (residual sd 0.0005 m); cross-checked against a 45-inch outside-wheel U-turn measurement |
| Maximum curvature | **2.1236 m⁻¹** | `tan(δ_max)/L` |
| **Planner minimum turning radius** | **0.60 m** | D-59 — deliberate tracking headroom, **not** a physical measurement |
| **Width, wheels straight** | **0.165 m** | Measured, outside-wheel to outside-wheel |
| **Width, full lock** | **0.180 m** | Measured |
| **Length** | **0.290 m** | Front bumper to rear |
| **Costmap footprint** | `[[0.230, 0.0825], [0.230, −0.0825], [−0.060, −0.0825], [−0.060, 0.0825]]` | `footprint_padding: 0.0` |
| **Inscribed radius** | **0.060 m** | Governed by the rear overhang, not the width |
| **Circumscribed radius** | **0.2443 m** | Runtime-reported |

**Three distinct radii share one name. Do not conflate them.**

| Value | What it is | Where it applies |
|---|---|---|
| **0.470 m** | Physical minimum at `base_link` | Vehicle capability; the ceiling on any commanded curvature |
| **0.565 m** | Outer-wheel swept radius | **Superseded and historical.** Never a planner or controller value |
| **0.60 m** | Planner minimum (D-59) | Smac configuration only; exceeds physical capability on purpose |

**This caused a live error on 2026-08-11.** A Stage 2 brief specified `minimum_turning_radius: 0.565` for the planner, conflating the superseded outer-wheel figure with the base-link value. The executor stopped before editing and escalated, correctly citing D-59 and the existing test that asserts 0.60. The brief was wrong; D-59 stands. Recorded here because the same collapse will recur unless the distinction is explicit.

**Historical correction (D-59).** v0.8 recorded `R = 0.565 m` and `δ_max = 17.5°`. That radius was the **outer wheel's swept radius**, mistakenly read from a wall-parallel U-turn width as 2R at `base_link`; δ_max was derived from it and inherited the error. Both are superseded.

**`δ_max` re-verification outstanding.** The stairs incident (§3.6) and the MD13S rewiring are both plausible causes of a shifted servo horn. `max_steering_angle = 0.3614` is load-bearing for the drive adapter and is unverified since both events.

**Accepted unmodelled margin.** At full lock the vehicle is ~0.180 m wide against a 0.165 m footprint, so roughly 7.5 mm per side is unmodelled during hard turns. Accepted deliberately: small against the p95 localization pose jump of 0.096 m already tolerated, and full lock occurs only at low speed.

**Steering hysteresis.** Releasing from left lock settles left of centre; from right lock, right of centre. Linkage slop and the servo saver — not a trim offset, and no software constant fixes it. Presents to the path follower as a steering deadband.

### 3.2 LD19 scan characteristics

Fixed angular extent, varying resolution: `angle_min = 0.0`, `angle_max = 2π`; `len(ranges)` varies 495–509 (mode 504). Median `scan_time` 100.0 ms. Root cause of §4.12.

**Median valid scan range in the mapped apartment: 1.09 m.** Constrains planner reachability, yaw observability, and motion-smear magnitude (§4.14).

**Scan plane at z = 0.1135 m.** Objects below roughly 0.11 m are structurally undetectable, and **descending edges are undetectable at any height** — a planar sensor cannot observe a negative obstacle. Not fixable in software; a documented operating-envelope limit and the direct cause of §3.6.

**A second class of invisible boundary.** A planar LiDAR sees a lamp post and a bench; it does not see the boundary between pavement and gravel, because that boundary has no vertical structure. No sensor reasonably addable to this platform resolves it. This is the strongest argument for semantic map layers (§7.7) — human annotation is not a workaround there, it is the only correct answer.

### 3.3 Actuation (D-70)

**Cytron MD13S brushed H-bridge**, replacing the stock brushed ESC.

| Property | Value |
|---|---|
| Motor supply | 6–30 V |
| Current | 13 A continuous, 30 A peak (10 s), limited at 30 A |
| Logic level | 3.3 V and 5 V — Pi drives it directly, no level shifter |
| Interface | TTL PWM + DIR. **Not RC PWM** |
| PWM frequency | up to 20 kHz; output frequency equals input frequency |
| Modes | locked-antiphase and sign-magnitude |
| Braking | regenerative |
| Protection | overcurrent, **undervoltage lockout**, temperature |

**Sign-magnitude is used. Locked-antiphase is rejected** — it places "stopped" at 50% duty, so a GPIO fault or float would command full speed. Under sign-magnitude, DIR sets direction, duty sets magnitude, and **duty 0 is unambiguously stopped**.

**Duty 0 applies active braking.** Bench-confirmed: with the traction pack connected and the Pi driving nothing, the motor is shorted and the car cannot be pushed by hand. Practical consequence: disconnect the pack to move the vehicle by hand.

**Unpowered and uncommanded equals brake.** Under the ESC, signal loss was undefined behaviour. This is a genuine safety improvement and is the property that makes D-73 coherent.

**Undervoltage lockout is a new failure mode.** The 6 V floor sits close to a 6-cell NiMH pack (7.2 V nominal) sagging under load near end of discharge. A UVLO trip cuts the motor mid-run and will present as a software stall. **Unmeasured.** This promotes ADS1115 traction telemetry from convenience to necessity.

**Reverse is unconditionally available in hardware.** The ESC's "reverse only reachable from a stop" was firmware behaviour and is gone. The equivalent protection is re-established deliberately in software (D-74) and is validated (§5.5).

### 3.4 GPIO map & chip convention

| GPIO | Owner | Function |
|---|---|---|
| 2 / 3 | UPS + battery | I2C1 — MAX17040 fuel gauge (0x36) |
| 4 / 5 | IMU | UART2 → `/dev/ttyAMA2` (BNO085) |
| 6 | UPS | AC-loss detect, active-low |
| **12** | `motor_node` | **MD13S PWM — 20 kHz duty cycle** |
| **13** | `motor_node` | Steering servo PWM — 50 Hz |
| 14 / 15 | LiDAR | UART0 → `/dev/ttyAMA0` (LD19) |
| 16 | UPS | Charge control, UPS-driven |
| 22 | `encoder_node` | Hall latch, both edges |
| **23** | `motor_node` | **MD13S DIR** (D-77) |
| 26 | IMU | BNO085 reset, active-high pulse |

**Chip convention.** The Pi 5 40-pin header is `gpiochip4` / label `pinctrl-rp1` — **not** `gpiochip0`. Open GPIO **by label**. This has caused three silent no-op bugs (IMU reset, hall calibration, PWM chip index, §4.17).

**Hardware pulldowns on PWM and DIR — ratified, not implemented.** These cover boot, pre-driver-init, and Pi power loss, where the pin is high-impedance. They do **not** close the SIGKILL hazard (§3.5).

### 3.5 Power architecture (D-51)

Domains separate by **current path**, not energy source.

**Servo supply relocated (D-70).** The MD13S has no BEC. The steering servo is fed by a dedicated 5 V regulator taken from the traction pack, with a single-point ground tie to the Pi for signal reference. **Powering the servo from the Pi's 5 V rail was explicitly rejected** — a servo stall is a traction-domain event, and routing it onto the compute rail creates exactly the brownout coupling the domain separation exists to prevent.

Ratified for the post-Phase-1 board revision: UPS permanent and trickle-fed from a traction-derived current-limited buck; traction bolted in, not hot-swappable; one USB-C PD input negotiating 12–20 V; **Pi-controlled high-side FET on motor power**; ADS1115 traction telemetry at I2C 0x48; contact pads not wireless for the eventual dock.

**The FET must be heartbeat-gated, not level-driven (D-82).** A FET held on by a GPIO sitting high inherits the identical failure mode it is meant to solve. It requires a retriggerable monostable or watchdog-timer IC so that loss of refresh opens the FET.

**SIGKILL hazard — open, deferred with trigger.** A hard-killed `motor_node` leaves the RP1 PWM peripheral driving autonomously. Mitigations in force: `Restart=always` bounds exposure to restart latency, and the restarted node writes duty 0 before anything else. **The hazard closes with the board-revision FET, and must close before sustained high-speed operation.** A software guardian daemon was considered and rejected (D-82).

**Observed bottleneck is compute runtime, not traction.** UPS endurance is roughly two hours. Interim mitigation: a USB-C power bank connected to the X1201 input during sessions.

**18650 cells flagged suspect.** Replacements needed; verify X1201 cell format (protected/unprotected, button-top/flat-top) before purchasing. Sources: batteribyen.dk, nkon.nl, Copenhagen vape shops for same-day. **For air travel:** remove cells and carry as individual spare cells in carry-on, which avoids power-bank classification under ICAO rules.

Traction voltage remains unobservable — `/battery` is the UPS fuel gauge only.

### 3.6 Stairs incident (2026-07-30)

Under autonomy at low speed, a third party opened a gate the operating envelope depended on. Dynamic replanning selected the newly-opened route and the vehicle descended a carpeted staircase.

**Root cause is sensing, not planning.** A planar LiDAR at 0.1135 m cannot observe a descending edge; the stairwell presented as free space with strong evidence. Nav2 behaved correctly given its input.

**Contributing cause is envelope enforcement.** The operating boundary existed physically (a gate) and had no software representation, so the system could not know the envelope had changed. This is the origin of D-79.

**Note for recovery-behaviour design (§7.8):** Nav2's `BackUp` does perform local-costmap collision checking, but **that check is 2D**. A stairwell reads as free space, so the costmap check is no protection against this hazard. Keepout masks are the mitigation, not collision checking.

**Damage.** The LiDAR carrier plate is the only fastener location without threaded inserts — a deliberate mechanical fuse. It fired as designed: screws pulled from plastic rather than transferring load into the mast. Screw holes stripped; screws pressed back in. **Location is set by pegs on the carrier plate, not by the threads**, so the extrinsic datum is preserved in principle. **Peg condition is unverified** — a deformed peg permits a rotated seat that feels solid by hand.

**Mechanical reset — ratified, not implemented.** Heat-set brass inserts, **nylon M3 screws as the consumable fuse**, thread engagement ≥ 1.5× diameter so the shank shears rather than the threads stripping, and screw length sufficient to protrude so a sheared stub can be punched out from below. Nylon and brass are non-magnetic, which matters next to the co-mounted BNO085 magnetometer. Deferred pending relocation; spare carrier plates printed.

**Mount design principle.** Mounts should be fully intact or completely gone and re-seatable to a repeatable kinematic datum. A bent-but-not-broken mount silently corrupts extrinsics, which is the dangerous failure mode for SLAM data integrity.

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | RF2O scan-origin canonicalization (D-37) |
| `runner_bringup` | `scan_rebinner` | `/scan` → `/scan_slam`, fixed 503-bin angular rebinning (D-37) |
| `runner_bringup` | `foxglove_goal_bridge` | Goal and route management (§4.16) |
| `runner_bringup` | `speed_envelope_observer` | Origin-vs-observed reconciliation → `/speed_envelope/status` (D-86) |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz |
| `runner_motor` | `motor_node` | `/cmd_vel` → MD13S PWM + DIR, steering PWM. **Persistent systemd service** (D-71). Sole owner of GPIO 12, 13, 23 |
| `runner_encoder` | `encoder_node` | Hall edges → `/wheel/odom`, `/wheel/encoder_state`. Sole GPIO 22 owner, libgpiod (D-54). **Persistent systemd service** (D-75) |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel_teleop`, three-state hold-to-run (D-48, amended D-78, D-83); keyboard bridge |
| `runner_drive_adapter` | `drive_adapter` | `/cmd_vel_nav` (SI) → `/cmd_vel_auto` (normalized), magnitude-domain PI with separate sign channel (§4.15) |
| `runner_telemetry` | `telemetry_node` | SoC temperature and throttle bits, 1 Hz, standalone systemd |
| `runner_interfaces` | — | `EncoderState`, `AdapterState`, `KeyboardState`, `SpeedEnvelopeEntry`, `SpeedEnvelopeStatus`, `SystemTelemetry` |
| `runner_battery` | `battery_node` | UPS fuel gauge → `/battery` (systemd) |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan` (D-22) |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. **Vendored fork** |
| `nav2_regulated_pure_pursuit_controller` | — | **Vendored fork.** Nav2 1.3.12 @ `6be3614` plus Runner path-cost regulation and cusp-bounded lookahead |

### 4.2 Service and launch structure (D-71, amends D-29)

**Hardware tier — persistent systemd services, started at boot, never killed:**

```
runner-motor.service      motor_node       GPIO 12, 13, 23      Restart=always
runner-encoder.service    encoder_node     GPIO 22              Restart=on-failure
runner-battery.service    battery_node     I2C 0x36
runner-telemetry.service  telemetry_node   —
```

**Application tier — launch composites, started and stopped freely:**

```
launch/
├── map.launch.py       = sensors + estimation + slam_map      + teleop
├── localize.launch.py  = sensors + estimation + slam_localize + teleop
├── nav2.launch.py      = localize + map_server + planner + controller + bt_navigator + goal bridge
├── autonomy.launch.py  = nav2 + drive_adapter + twist_mux
├── teleop.launch.py    = joy + keyboard_bridge + teleop_node + twist_mux
└── include/
    ├── sensors / estimation / slam_map / slam_localize
    └── lidar / imu / tf_static / rf2o / ekf
```

**D-29 is amended, not repealed.** A runnable launch must start every *application* node needed to function, and may assume the hardware tier is up.

**Why persistence rather than per-launch ownership.** A process that never exits never tears anything down. Three consequences: the sudo permission ritual disappears; one-owner moves from *convention* to *mechanism*, since a second claimant now fails rather than racing silently; and the configured service restart policies supply supervision without a second hardware writer or an IPC boundary.

**The stop path changes fundamentally.** The node survives, `/cmd_vel` goes stale, and the **D-09 watchdog is the primary stop mechanism** (§5.4).

**New risk, accepted and named.** With a persistent motor node, anything publishing `/cmd_vel` can move the vehicle at any time, including with no composite running. The watchdog bounds it.

**Planned tier restructuring (§9.2).** Sensors and estimation do not change between operating modes, so the natural split is four tiers — hardware, estimation, world model, application — with only the last two switching. `slam_toolbox` in mapping mode and localization mode both own `map→odom` and must be declared mutually exclusive with systemd `Conflicts=`, making the one-owner rule structural rather than remembered.

**`systemctl restart` must not require sudo. Ratified, not implemented.**

### 4.3 Scan topology (D-37)

```
LD19 /scan  (raw, variable 495–509 bins, single owner)
   ├─→ rf2o_scan_canonicalizer → /scan_rf2o  → RF2O
   ├─→ scan_rebinner           → /scan_slam  → slam_toolbox
   └─→ raw /scan → local costmap (cardinality-agnostic)
```

Nav2 costmaps consume raw `/scan`. Every new map is built through `/scan_slam`.

### 4.4 Ownership

| Resource | Sole owner |
|---|---|
| `map→odom` | slam_toolbox (localization mode) |
| `odom→base_link` | EKF |
| `base_link→base_laser`, `base_link→imu_link` | static |
| `/map` | `map_server` (D-50) |
| `/slam_map` | slam_toolbox (visualization only) |
| `/cmd_vel` | `twist_mux` |
| **GPIO 12, 13, 23** | `motor_node` (persistent) |
| **GPIO 22** | `encoder_node` (persistent) |
| Direction latch and `/motor/direction` | `motor_node` — **no other node may subscribe for control purposes** (D-85) |

### 4.5 Static extrinsics (D-76)

`base_link` = rear axle, ground-projected (D-01).

| Edge | x (m) | y (m) | z (m) | yaw (rad) |
|---|---|---|---|---|
| `base_link→base_laser` | **0.0733** | 0.000 | 0.1135 | **π** |
| `base_link→imu_link` | **0.1233** | **−0.0025** | 0.1060 | **0** |

**Derivation.** The sled was rotated 180° about the vertical axis and reattached to the same two chassis holes, 120 mm apart — a point reflection through their midpoint `M = 0.10265 m`: `x_new = 2M − x_old`, y sign-flipped where non-zero, z unchanged, every attached frame gains π of yaw.

**Geometric consistency check.** Front hole → LiDAR 30.65 mm, LiDAR → IMU 50.00 mm, IMU → rear hole 39.35 mm, totalling exactly 120.00 mm. **Measured and confirmed:** rear axle to LiDAR centre ≈ 73 mm against a derived 73.3 mm.

**The IMU is now forward of the LiDAR by 50 mm.** LiDAR↔IMU relative geometry is unchanged — the co-mount principle holds and only the `base_link` legs changed.

**Gyro Z is unaffected by the rotation.** A pure yaw rotation leaves the IMU's Z axis pointing the same way. Since gyro Z is the only IMU channel fused, the EKF is indifferent; the tf_static yaw matters for correctness only.

**Posegraph invalidation.** Every map built under the previous extrinsics is incompatible with this geometry. All map artifacts, including the later incomplete bundles, were deliberately removed on 2026-09-01. **There is currently no committed usable map bundle.** Localization and autonomy take an explicit map basename and require its posegraph, data, YAML, and referenced occupancy image.

**`base_link→base_laser` re-verification after any transit or physical disturbance is the highest-stakes pre-experiment check** on the platform.

**Outstanding.** Z is assumed unchanged, which holds only if the rail is level and the sled mounts symmetrically front-to-rear; unverified.

### 4.6 Encoder

**libgpiod backend (D-54)**, chip resolved by live label.

**Interval-timing estimator (D-49):** `edge_rate = (n−1)/(t_last − t_first)` over `history_depth` intervals, default **4**, using kernel CLOCK_MONOTONIC timestamps. Measured σ = 0.0034 m/s at 0.313 m/s.

**Estimator lag is `history_depth / edge_rate` and is speed-dependent** — approximately 0.257 s at 0.16 m/s, 0.137 s at 0.30 m/s, 0.069 s at 0.60 m/s, 0.024 s at 1.7 m/s. This lag sits inside the speed control loop and **inflates measured rise times at low speed**. It also means that at 0.3 m/s the estimator's first valid output after breakaway is already near steady state, producing an artifact where `t63 == t90` on standing starts (§4.15).

**Calibration:** 0.010282 m/edge, 8 magnets on the spur gear. Reference: **97.3 edges/m**.

**Independent validation (2026-07-31).** Across 13 steady-state plateaus spanning 0.16–1.74 m/s, `|v_RF2O| / v_encoder` was **0.997 ± 0.022**, with a fitted relation `|v_RF2O| = 0.968·v_enc + 0.016`. Two physically independent sensors agreeing to sub-3% over a 10× speed range is the strongest confirmation of the 0.010282 m/edge calibration obtained to date.

**Magnet spacing is period-2**, from bipolar-latch switching asymmetry: ±1.5% in July, ±5.5% currently, structure unchanged. Cancelled entirely by depth ≥ 2. **New stressor:** regenerative braking applies deceleration torque through the same spur gear that carries the magnets.

**Direction (D-74, supersedes D-42).** `/wheel/odom` and `/motor/direction` are signed by the **latched** commanded direction, which flips only on measured stationarity. See §5.5.

**Magnitude is unreliable under wheelspin:** p90 wheel/EKF ratio 2.04. Not an EKF velocity source.

**The encoder is safety-critical.** `/wheel/encoder_state.stationary` gates direction reversal in the actuator path.

### 4.7–4.9 RF2O, covariance, wheel odometry

Constant twist covariance `vx 0.02`, `vyaw 0.25` (D-24). Stall root-caused to high-rate INFO logging, fixed in `ece93d1`.

**RF2O frame bug — fixed (2026-07-31).** RF2O published linear velocity in the **laser frame** while declaring `child_frame_id = base_link`. With `yaw = π` from D-76 this inverted `vx` while leaving yaw rate correct. The bug existed historically but was masked because laser yaw had previously been zero. RF2O now computes linear velocity from the transformed base pose increment rather than the laser-frame increment. **Confirmed from data:** sign inverted in all 13 moving plateaus pre-fix, sign correct in all 78 plateaus post-fix, with ~99.5% agreement with wheel odometry while moving. Localization stability improved immediately.

**RF2O `vy` is hardcoded to zero.** Lateral velocity is not merely unfused — it does not exist in the signal. This is the single highest-value change for slip observability and is independently useful for localization (§6).

**RF2O publishes at ~10 Hz**, the slowest velocity signal in the stack, and degrades under fast rotation from scan smear (§4.14).

### 4.10 Localization quality (D-32, D-41, D-45, D-52)

**Metric:** induced pose jump at `base_link` — both consecutive `map→odom` transforms applied to the robot's current odom-frame position, distance between results. **Not** the translation of `map→odom`, which is inflated by distance from the odom origin (measured 7.259× at a 5.94 m mean lever arm).

**Cadence is conditioned on motion** (D-52).

**Reference, `runner_test_20260726_115736`, window t+20 s → end (154.2 s, 97.7 m):**

| Metric | Value |
|---|---|
| **Pose jump at `base_link`** — med / p90 / p95 / max | **0.0188 / 0.0658 / 0.0959 / 0.3556 m** |
| Yaw correction — med / p95 / max | 0.0175 / 0.1396 / 0.3560 rad |
| Cadence — unconditioned / motion-conditioned | 3.683 / **5.717 Hz** |
| Gap — med / p90 / p95 | 0.180 / 0.260 / 0.311 s |
| Correction per metre travelled | 0.175 m/m |

Driving profile: mean 0.626 m/s, max **2.798 m/s**, 52.4% throttle duty.

**Note:** p95 pose jump (0.096 m) **exceeds the inscribed radius** (0.060 m). A pose correction alone can place the footprint in apparent collision. This bounds how tight collision checking can safely be.

**This baseline predates the extrinsics change (D-76), the RF2O frame fix, and the new map. It should be re-run.**

### 4.11 Instrumentation

`tools/analyze_localization_bag.py`, `analyze_yaw_cross_validation.py`, `analyze_throttle_response.py`, `analyze_pid_run.py` (configuration-attributed PID analysis, D-45-compliant). Typed diagnostics on `/drive_adapter/state_typed`, `/speed_envelope/status`, `/system/telemetry`.

**Bag recording discipline (D-81 — ratified, not implemented).** `--all-topics` is rejected. Six global costmap topics accounted for 367 MB of a 567 MB bag, and all `_updates` topics recorded zero messages, indicating `always_send_full_costmap` republishing entire grids at ~0.9 Hz across six debug publishers — roughly 500 kB/s of continuous DDS traffic with no subscribers. An allowlist reduces bags to ~7–8 MB per driving minute.

**Transfer:** `rsync -avP` with `shasum -a 256` verification. VS Code Remote-SSH's downloader silently truncates large binaries. **Signature: a file size that is an exact multiple of 1 MiB.** Observed twice (105.0 MiB and a second instance).

**MCAP reading discipline.** MCAP is self-describing — schemas are embedded, and `metadata.yaml` is required only by `ros2 bag play`, never by a reader. `get_summary()` reads the footer, written when the recorder closes the file; a bag still recording, killed, or copied mid-write reports zero messages while the data is intact. **Use a streaming reader before concluding a bag is empty.** A bag was incorrectly declared empty on 2026-08-01; it contained 4295 messages and a valid summary.

### 4.12 Scan cardinality (D-37)

Karto registers a fixed beam count once and hard-rejects mismatched scans; 79% of the stream was silently discarded. Fixed by `scan_rebinner` — angular rebinning onto a fixed 503-bin grid, nearest-in-angle, `inf` for empty bins, header stamp preserved. Pad/truncate, interpolation, driver fork, and canonicalizer merging all explicitly rejected; do not revisit.

### 4.13 Phantom-geometry trap (D-40)

Localization mode retains recent scans; scans taken while mislocalized depict the real room at the wrong map location and the matcher reinforces its own error. Failure is a cliff, not a gradient. Not observed post-D-37.

### 4.14 Yaw lag from scan motion distortion (D-53)

**No yaw scale error.** Integrated over the 174.684 s reference: IMU **1.0023×**, EKF **1.0040×**, RF2O 0.8862×.

**But a yaw-rate-proportional phase lag exists.** Regressing `map→odom` yaw against signed yaw rate: **slope −0.0652 s, correlation −0.596**.

**≈65 ms ≈ half a LiDAR scan period.** The LD19 sweeps over 100 ms and nothing deskews, so Karto's best-fit pose corresponds to the middle of the sweep. **The pose is not wrong; it is late.**

| Yaw rate | Heading error | Scan displacement at 1.09 m |
|---|---|---|
| 0.5 rad/s | 1.9° | 3.6 cm |
| 1.0 rad/s | 3.7° | 7.1 cm |
| 3.0 rad/s | 11.2° | 21 cm |

**Direct navigation consequence.** At 0.82 m/s through a 0.60 m radius the yaw rate is ~1.37 rad/s — roughly 0.15 m of scan displacement, **more than double the 0.060 m inscribed radius.** Corner speed regulation is partly a smear-mitigation measure.

Fix is IMU-based scan deskewing. **Deferred** — it binds at high speed, not at current operating speeds. The D-76 relocation reduces the translational component only.

### 4.15 Drive adapter (D-55, D-84)

Resolves the `/cmd_vel` unit mismatch: Nav2 publishes a true `Twist` (m/s, rad/s); `motor_node` consumes normalized ±1.0.

```
Nav2 → /cmd_vel_nav (SI) → drive_adapter → /cmd_vel_auto (normalized) → twist_mux → /cmd_vel → motor_node
```

**Steering:** `δ = atan(L·ω/v)`, normalized by `max_steering_angle` 0.3614.

> **Open defect candidate.** Whether `v` in this expression is **signed** is unverified. It must be — to rotate counterclockwise while reversing, the front wheels steer opposite to forward. If `|v|` is used, reverse steering is inverted. This is one of two candidate causes for the observed reverse tracking asymmetry (§8.2).

**Steering infeasibility clamps; it does not brake (D-60).** When requested curvature exceeds 2.1236 m⁻¹, steering saturates at ±1.0 and speed is maintained, accepting understeer. Measured: with the planner at 0.470 m, saturation reached 51.3%; with D-59 margin it fell to 1.5%.

#### Command pipeline

```
raw /cmd_vel_nav.linear.x
   │
   ├── exactly zero          → effective_speed = 0
   ├── |v| < 0.25            → promote to ±0.25   (sign preserved)
   ├── |v| > 0.60            → cap at ±0.60       (sign preserved)
   └── otherwise             → unchanged
   │
   ├── direction = sign(effective_speed)     ─────────────┐
   └── magnitude → feedforward + PI on |speed|            │
                              │                            │
                              └── output = ±(ff + P + I) ──┘  clamped to [−0.14, +0.14]
```

**The PI is magnitude-domain with sign carried on a separate channel (D-84).** Error is `|effective_speed| − |measured_speed|`, so the error term never changes sign at a cusp and the integrator cannot wind up backwards during a direction transition. This is also what makes a single controller serve both directions.

**There is no feedforward output floor.** The commanded-speed clamp guarantees `FF(0.25) = 0.0471`, comfortably above the cmd-0.04 plant floor. A computed feedforward below 0.04 emits a **diagnostic, never a clamp** — a violation means the speed clamp was bypassed.

#### Frozen longitudinal configuration (D-84)

| Parameter | Value | Basis |
|---|---|---|
| **Feedforward** | `\|cmd\| = 0.1188·\|v\| + 0.0174` | Inverse of the measured plant (§4.19) |
| **Kp** | **0.05** | Frozen — validated, not optimized |
| **Ki** | **0.01** | Parallel form, integral time ≈ 0.6 s ≈ 2τ |
| **Integrator bound** | **±0.005** | Derived: 0.03 m/s inter-surface intercept spread ÷ 8.42 gain |
| **`output_max`** | **±0.14** | Derived: `FF(0.60) + Kp·0.60 + I_max`, ×1.1 slack |
| `maximum_commanded_speed` | 0.60 m/s | Operational ceiling |
| **Commanded-speed floor** | **exactly 0 or ≥ 0.25 m/s** | Plant has no usable steady state below 0.20 m/s |

**`output_max` is a safety parameter, not a controller parameter.** It is the last bound on speed if feedback fails: at 0.14 a stuck-open output produces ~1.03 m/s. Both prior values were artifacts — **0.70 was an ESC-era carry-forward that D-73 changed the lower bound of without re-deriving the upper**, and 0.12 was a provisional compatibility clamp. Peak observed `final_throttle` in validated autonomy is 0.0749, or 53.5% of ceiling.

**Integral time is 2τ, not τ.** Pole-cancellation at `Ti = τ` ignores loop delay, and here the delay (50 ms sample period plus 0.07–0.14 s estimator lag) is comparable to τ = 0.30 s. The **bound does most of the work** — with ±0.005, Ki sets only how fast the bound is reached, not how far the integrator goes.

#### Integrator freeze conditions (D-84)

Anti-windup is **conditional integration**: skip the update when output is outside `[−output_max, +output_max]` and the error sign would drive further into saturation. Freeze means **hold**, never reset. **Every freeze publishes its reason.**

| Reason | Condition | Priority |
|---|---|---|
| `ZERO_COMMAND` | commanded speed exactly zero | high |
| `FEEDBACK_STALE` | no fresh encoder sample within 3× nominal period | high |
| `DIRECTION_MISMATCH` | `/motor/direction` latch ≠ `sign(commanded_speed)` — **the D-74 gate hold; this is the cusp wind-up protection** | high |
| `WHEELSPIN` | encoder speed materially exceeds EKF `vx` | high |
| `OUTPUT_NOT_SELECTED` | `/cmd_vel` disagrees with `/cmd_vel_auto` | high |
| `ARBITRATION_UNAVAILABLE` | arbitration state unknown | high |
| `ANTI_WINDUP` | output saturated in the direction of the error | high |
| `INVALID_DT` | `Δt > 0.5 s` — the integration step is skipped and logged | high |
| `GAIN_DISABLED` | `Ki == 0` | **lowest** |

**`GAIN_DISABLED` is deliberately lowest priority.** When it short-circuited evaluation, the freeze reason partitioned *perfectly* by mode across 703 s of driving and five conditions produced **zero observations between them** — they would all have gone live simultaneously and untested the moment `Ki > 0`. Demoting it makes the field answer "what *would* have frozen the integrator."

**Δt comes from message timestamps, not the nominal period.** After a scheduling stall, integrating across the real gap injects a step several times larger than a normal update.

**The integrator carries across direction changes and is not reset.** The direction-dependent feedforward error is roughly ±0.0007 in command units — the forward and reverse slope and intercept differences largely cancel — which is well inside the ±0.005 bound. Reset occurs only on node start and on a live `Ki` change.

**Known limitation.** `/cmd_vel` value comparison cannot determine publisher provenance when teleop emits exactly the same command as the adapter. Accepted because it fails by *allowing* integration rather than blocking motion, bounded at ±0.005. The robust fix is for the arbiter to publish its selected source rather than inferring from values.

**Stale `/cmd_vel_nav` produces silence, not brake.** Publishing brake would keep commands flowing and prevent the D-09 watchdog from firing.

#### Validated performance (`pid0_20260801_112638`, 703.380 s, 8 configuration segments)

| Criterion | Result |
|---|---|
| Saturation | **0 of 18** reportable steps had any sample within 1% of `output_max` |
| Overshoot | 6.73% on the one clean large step |
| Steady-state error | ≤ 0.017 m/s across all usable steps |
| Integrator bound reached | **never** — peak 0.0034 against ±0.005 |
| Feedforward coefficients | `0.1188 / 0.0174` across all segments, max residual 0.000000 |
| Encoder exact match | 98.98% all states; **99.97% while `FollowPath` active** |

**Ki effect, matched-speed comparison:**

| Target | Ki = 0 mean abs error | Ki = 0.01 | Δ |
|---|---|---|---|
| 0.300 m/s | 0.0259 (n=159) | 0.0038 (n=34) | −85% |
| 0.450 m/s | 0.0099 (n=21) | 0.0024 (n=16) | −75% |

**Interpretation caution.** The integrator's own settled value is 0.0011–0.0018 normalized effort, worth only 0.009–0.015 m/s at gain 8.42 — it cannot account for the full improvement. The remainder is likely run-order drift, since the Ki = 0 samples come from t = 0–254 s and the Ki = 0.01 samples from t = 478–592 s. **Ki does real work of roughly 0.01 m/s; the headline percentage is inflated.**

**Standing-start step metrics are artifacts.** Every 0→0.300 step reports `t63 == t90` at 0.098–0.100 s, physically impossible against τ = 0.30 s. Cause is encoder estimator resolution (§4.6). Of 18 reportable steps, 10 were unusable, 3 were decelerations, 3 were rest-starts with this artifact — **two steps carried real timing information.** This is a structural limit of extracting step data from a Nav2 route, since RPP continuously modulates speed on curvature; 31 of 49 transitions were flagged ambiguous.

**Loop rate.** The adapter published at **20.006 Hz with `FollowPath` actively computing**, median inter-sample 50.0 ms. The 4.5–5.6 Hz figure recorded in earlier versions is **superseded** — it was measured under conditions that no longer hold. This matters because a 5 Hz loop against τ = 0.30 s would be delay-dominated and would make `Kp = 0.05` near the stability limit; at 20 Hz there is genuine headroom above it.

### 4.16 Route management

`foxglove_goal_bridge` provides `/move_base_simple/goal`, `/runner/waypoint`, `/runner/route_control` (`start`, `stop`, `clear`, `loop_on`, `loop_off`, `remove_last`), `/runner/route`, `/runner/route_markers`, `/runner/autonomy_state`. Routes persist to `~/.ros/runner_route.json`.

Routes dispatch as **`NavigateThroughPoses`**, not chained single goals — RPP decelerates to zero at every goal.

**Known defect:** a re-dispatched identical route does not replan from the vehicle's new position. Nav2 reuses the parsed tree when the XML filename is unchanged; BT.CPP halts nodes to IDLE but does not clear `GoalUpdatedCondition`'s remembered poses. Recommended fix: clear the `path` blackboard entry on new action execution. **Not implemented.**

### 4.17 PWM sysfs ownership (D-72, supersedes D-23)

`python-periphery` is removed; `motor_node` owns the sysfs interface directly.

**Every PWM sysfs attribute write applies the entire PWM state.** `__pwm_apply()` rejects any state with `period == 0` or `duty_cycle > period` **before** branching on enable. Generic PWM core behaviour, not RP1-specific. Consequences: on a fresh channel, `period` is the only legally writable attribute — including `enable = 0`, which fails with `EINVAL`; on a hot channel, writing a period smaller than the current duty also fails.

**`disable()` is removed from the startup path.** The defined safe output is **PWM enabled at duty 0**.

```
ensure exported            (tolerate EBUSY)
wait for attributes writable  (udev applies permissions asynchronously)
p = read(period)
if p != 0: write(duty_cycle, 0)     ← hot channel: this IS the safety action, first
write(period, P)
write(duty_cycle, 0)
write(polarity, normal)             (only if differing, only while disabled)
write(enable, 1)
```

Verified startup state: motor `period 50000, duty 0, polarity normal, enable 1`; servo `period 20000000, duty 1500000, enable 1`.

**Duty resolution.** A 50000 ns period with nanosecond duty granularity gives ~50000 discrete steps, so normalized command resolution is ~2×10⁻⁵. **Command granularity is never a limiting factor** — the 0.01 steps in teleop setpoint ladders are a D-pad increment choice, not a hardware constraint.

**Outstanding.** The PWM chip is resolved by index (`pwmchip0`), the same class of bug as `gpiochip0` versus `pinctrl-rp1`. Whether pwm0 and pwm1 share a clock divider is unverified. `Type=simple` reports the unit active before hardware initialization completes, and a `StartLimitBurst` lockout leaves the system with **no motor owner at all**.

### 4.18 Interface schema staleness (2026-08-01)

`foxglove_bridge` runs as a systemd service started at boot. It holds the message typesupport from its launch environment, so **any rebuild of `runner_interfaces` afterwards leaves it decoding against a stale schema.**

**Observed signature.** Live Foxglove reported `Invalid typed array length: 3217589678` and `1073411386`, which decode to `0xBFC885AE` and `0x3FFAF53A` — the high halves of IEEE-754 doubles. The decoder was reading a string length from inside a `float64`.

**Why it failed only while driving.** Parked, the numeric fields are all-zero bytes, so a misaligned length read yields 0 and silently produces an empty string. **The live view was wrong the entire time; it was merely quiet when stationary.** Bags were unaffected because MCAP embeds its own schemas.

**Standing requirement: restart `foxglove_bridge` after any `runner_interfaces` rebuild.**

### 4.19 MD13S plant characterization (D-84)

Two teleop sessions, 2026-07-31. `low_speed0` (151 s, forward only, 13 plateaus, keyboard fixed setpoints) and `low_speed3` (234 s, **both directions**, 78 plateaus, R1 fixed-throttle ladders ascending and descending, 0.02–0.19).

#### Steady-state speed versus normalized command

Band cmd 0.04–0.12, steady window = last 60% of each plateau with a minimum 0.7 s settle skip.

| Fit | slope (m/s per cmd) | intercept | zero-speed cmd | rms resid |
|---|---|---|---|---|
| Forward (n=32) | 8.578 | −0.1605 | 0.0187 | 0.018 m/s |
| Reverse (n=31) | 8.229 | −0.1303 | 0.0158 | 0.019 m/s |
| **Combined (n=63)** | **8.420** | **−0.1468** | 0.0174 | 0.019 m/s |

**Forward and reverse differ by 4.2% in slope, and the combined fit's residual is identical to each direction fitted alone.** Splitting the model buys zero measurable accuracy. **One feedforward serves both directions.** Reverse is marginally quicker off the mark, consistent with less drivetrain preload.

**Inverse in use: `|cmd| = 0.1188·|v| + 0.0174`.** FF(0.25) = 0.0471, FF(0.45) = 0.0709, FF(0.60) = 0.0887.

#### Hard low-speed floor

| cmd | forward | reverse |
|---|---|---|
| 0.02 | 0 | — |
| 0.03 | 0.017 ± 0.015 (zero on 2 of 3 trials) | 0.053 ± 0.016 |
| 0.04 | 0.202 ± 0.002 | 0.203 ± 0.005 |

**There is no usable steady state below ~0.20 m/s.** At cmd 0.03 the standard deviation (0.036) exceeds the mean (0.017) — stick-slip, not slow motion. **Confirmed by descending ladders**, so this is minimum *sustainable*, not merely breakaway: entering 0.03 from motion still yields zero or intermittent 0.03–0.07 m/s creep. The 0.07–0.20 m/s band is unreachable in steady state.

This invalidated two Nav2 parameters that sat inside the dead band — `min_approach_linear_velocity` at 0.126 and `regulated_linear_scaling_min_speed` at 0.15 (§7.5).

#### Dynamics

**Plant time constant τ ≈ 0.30 s**, speed-independent above cmd 0.06, both directions, ~20 transients. Below cmd 0.06 the apparent τ inflates to 0.35–0.50 s from static friction plus encoder estimator lag.

**Braking (regenerative, duty 0):** `a ≈ 1.6·v + 0.27 m/s²` — deceleration proportional to speed, as regen should be. Peak observed 2.50 m/s² from 1.86 m/s, which is 0.25 g; **no lockup signature at any tested speed**.

| Speed | Braking distance | + 0.2 s watchdog | Total |
|---|---|---|---|
| 0.30 m/s | 0.060 m | 0.060 m | **0.120 m** |
| 0.45 m/s | 0.102 m | 0.090 m | **0.192 m** |
| 0.60 m/s | 0.146 m | 0.120 m | **0.266 m** |

Two independent sessions agree on the 0.6 m/s figure to three significant figures.

> **Tire state is an uncontrolled variable.** This model was measured with visibly dusty tires and is therefore conservative — clean tires will brake harder. **Braking performance is tire-state × surface, not a fixed vehicle property**, and belongs alongside battery SoC as a session condition to record. Re-measurement on clean tires is a twenty-minute task using the same protocol. The **feedforward is expected to be unaffected**, since steady-state speed-versus-duty is set by rolling resistance and motor characteristics rather than peak friction.

#### The intercept is not a vehicle constant

Between the two sessions, at identical commands:

| cmd | low_speed0 | low_speed3 | Δ |
|---|---|---|---|
| 0.04 | 0.158 | 0.190 | +20.4% |
| 0.06 | 0.323 | 0.358 | +10.9% |
| 0.08 | 0.488 | 0.527 | +7.9% |
| 0.10 | 0.654 | 0.695 | +6.3% |
| 0.12 | 0.819 | 0.864 | +5.4% |

**The delta is roughly constant in absolute terms (+0.032 to +0.045 m/s) while the percentage shrinks — an intercept shift, not a gain change.** Slopes were 8.266 versus 8.420, 1.9% apart. Meanwhile **within-session repeatability is under 2%**: cmd 0.04 recurred four times across 209 s in `low_speed3` at 0.2035 / 0.1993 / 0.2011 / 0.2006.

**Interpretation.** A constant offset at fixed duty is a constant opposing force — surface rolling resistance, floor grade, or drivetrain friction, indistinguishable in this data. **The intercept is a property of where you are driving, not of the vehicle.** Therefore: fit the slope once, and let the integrator own the offset. Never bake a per-surface intercept in as a constant.

**The same signature explains the forward/reverse asymmetry** — intercepts differ by 0.030 m/s, structurally identical to a constant force. A floor grade is a live hypothesis and is untestable from existing bags, because the BNO085 publishes **gravity-compensated** linear acceleration (median |a| = 0.11 and 0.34 m/s² across the two sessions), so there is no pitch signal. The decisive test is to drive out-and-back, rotate the vehicle 180°, and repeat: if the asymmetry follows the direction of travel across the floor it is the room; if it follows commanded forward-versus-reverse it is the drivetrain.

#### Why the earlier "linear plant" claim was over-stated

A provisional fit of `v = 7.947·cmd − 0.1436` with R² = 0.9968 was read as evidence of linearity. **R² is the wrong instrument over a 10× range** — it is dominated by the span. Residuals from that fit form a clean parabola; a quadratic (`v = −6.334·cmd² + 9.406·cmd − 0.218`) halves the RMS residual from 0.028 to 0.012 m/s, and the slope softens 33% across the range, which is physically expected from drag plus IR sag. **The correct response is to fit the operating band rather than the global range** — the entire Phase 1 envelope lives in cmd ∈ [0.035, 0.094], under 6% of actuator range, where the linear fit is genuinely adequate at 0.019 m/s rms.

### 4.20 Speed envelope consolidation (D-86)

`speed_envelope.yaml` is the **single committed origin** for all longitudinal parameters, replacing values previously scattered across Nav2 YAML, drive-adapter YAML, launch overrides, and code defaults. 27 shared parameters.

**Single origin is not single ownership.** RPP still owns its ROS parameters and `drive_adapter` owns its own; **you cannot own Nav2's parameters without forking Nav2.** The achievable property is single origin plus **detected divergence**.

`speed_envelope_observer` reads live values back from running nodes via parameter clients and publishes `/speed_envelope/status`, comparing committed origin against observed runtime value per key with a divergence flag. It detects live overrides, partial YAML load failures, writes accepted without effect, and general drift.

**Design constraints on the observer:** application tier, never hardware tier; **must fail benign**, publishing `unknown` for a key whose parameter client times out rather than blocking; publishes **both** origin and observed value per key. Not `drive_adapter`, because parameter service calls are blocking and would sit inside a 20 Hz control loop.

**It justified itself immediately** by exposing the `output_max` mismatch between the assumed 0.70 and the actual launched 0.12.

**Observer latency is ~1–2.7 s** after a parameter write. Adequate for reconciliation, **not** for attributing measurements to configurations — for that, recover gains directly from telemetry (`Kp = proportional_term / speed_error` wherever `|speed_error| > 0.01`).

**Live-tunable subset.** `desired_linear_vel`, `min_approach_linear_velocity`, `regulated_linear_scaling_min_speed` (RPP, natively dynamic), plus `proportional_gain` and `integral_gain` (adapter callbacks). Everything else is startup-cached, which is correct for safety and calibration values.

**A lifecycle-transition helper is rejected on safety grounds, not deferred.** A mid-route lifecycle transition on `controller_server` deactivates the controller while moving; `/cmd_vel_nav` goes silent, the D-09 watchdog fires after 0.2 s, and duty 0 is a full brake — which at speed risks the D-27 fishtail, the one failure documented as not auto-recovering. A tuning convenience does not justify inducing the platform's worst localization failure.

---

## 5 · Teleop & motor control

### 5.1 Brake semantics (D-73)

**Brake is not a mode. It is a command of zero speed.**

`/cmd_vel` already means "go this fast." Zero means "go zero fast," and on an H-bridge the plant makes that physically true without additional logic.

- **Braking is proportional, not bang-bang.** Duty below the back-EMF-matching value produces regenerative braking scaled to the gap; duty 0 is the maximum.
- **Negative never means brake, anywhere. Negative means reverse.** Negative-at-speed on an H-bridge is **plugging** — applied voltage adds to back-EMF, current spikes, the vehicle decelerates violently then accelerates in reverse with no stop. The D-74 gate prevents this.
- **There is no coast state.** Sign-magnitude offers drive or brake only. Coast is synthesizable by commanding the duty matching current back-EMF — a feedforward lookup, now available since §4.19.
### 5.2 Throttle semantics — unified (D-83)

> **This section exists because §5.2 of v1.1 was a spec defect.** Its table had one column per button and collapsed two orthogonal axes: *which control selects the mode* and *which control selects direction and magnitude*. The R1/L2 fixed-versus-proportional asymmetry was real, deliberate, documented in code and unit tests — and invisible here. That ambiguity propagated into an executor brief and produced a false "R1 is broken" investigation. **Sign and magnitude source are now stated separately for every input path.**

**Sign convention, all paths:** positive `linear.x` = forward, negative = reverse, exactly zero = brake. There is no brake control anywhere, because releasing the throttle already commands zero, which is full brake.

| Path | Mode select | Direction source | Magnitude source |
|---|---|---|---|
| **DualSense X** | hold X | R2 = forward, L2 = reverse | trigger depth, proportional, shaped |
| **DualSense R1** | hold R1 | R2 = forward, L2 = reverse | **fixed setpoint** (D-pad adjusts), trigger depth **ignored** |
| **DualSense L1** | hold L1 | — | `teleop_suppress`: publishes nothing, autonomy passes through |
| **Nothing held** | — | — | **exactly 0.0** |
| **Keyboard** | Space held | W = forward, S = reverse | fixed setpoint (`=` / `-` adjust) |
| **Keyboard W+S** | — | — | **exactly 0.0** |

**Priority: X > R1 > L1.**

**D-48's original asymmetry is superseded.** Under R1, L2 previously produced *proportional* reverse via `_shape_manual_command(−reverse, expo)` while R2 produced the fixed setpoint. That was defensible under the ESC, where negative meant brake. Under the MD13S, negative is true reverse and the asymmetry makes fixed-throttle characterization impossible in one direction.

**The `linear.x = −1.0` on nothing-held hazard is resolved.** Confirmed from data: across 5410 `/cmd_vel` samples in `low_speed0`, `linear.x` is never negative and idle sits at exactly 0.0.

**Keyboard reverse is implemented.** `Space + S` publishes negative `linear.x`; the prior brake-on-S behaviour was ESC-era semantics.

**Throttle shaping belongs in `teleop_node`, never in `motor_node` (D-78).** `motor_node`'s mapping must remain linear because `drive_adapter` and all characterization depend on it. A cubic expo blend (`out = k·x³ + (1−k)·x`) and a max-output scalar are live-tunable teleop parameters.

**X→R1 latch:** pressing X while R1 is held suppresses fixed throttle until R1 is released and re-pressed.

Measured under the ESC: R1 release-to-brake 5.065 ms; L1 41.350 ms; watchdog fires 171.464 ms after teleop loss.

**Open:** steering has been observed to require the X dead-man held. Cause not isolated — either `teleop_node` publishes nothing without the dead-man, or the D-09 watchdog recenters the servo, newly observable because `motor_node` is persistent. **What the watchdog should do to steering is an open decision** — zeroing throttle is unambiguously correct; centering steering mid-corner at speed is arguably worse than holding the last angle.

### 5.3 Keyboard teleop (D-68)

`tools/keyboard_sender.py` runs on the operator laptop, captures keydown/keyup via `pynput`, and sends UDP at 20 Hz to a Pi-side bridge that owns all safety properties: **150 ms liveness timeout measured on the Pi**, speed cap, and arbitration. The physical controller always preempts.

| Key | Behaviour |
|---|---|
| `` ` `` | autonomy enable, 600 s hold expiry |
| `=` / `-` | throttle setpoint up/down |
| Escape | clear all held state, brake |
| F5–F9 | route start / stop / clear / loop toggle / undo last |
| Space + W/A/S/D | manual driving — **requires `--teleop`, disabled by default** |

**Capture is global regardless of window focus.** Accepted deliberately so Foxglove can be used while driving.

**Fixed setpoints bypass any trigger curve, which makes this the correct tool for throttle characterization.**

**Open question — relocation to the Pi.** Running the sender on the Pi over SSH would remove DDS-over-Tailscale from the safety-critical teleop path. DDS over a VPN fails by *silent staleness* rather than clean loss, and a stale teleop command is worse than none because the watchdog only fires on absence; SSH fails as SIGHUP → process death → watchdog. **The blocker to check first: whether the current dead-man uses true key-release events.** A TTY over SSH delivers key presses only — there is no key-up in the stream — so if release detection is genuine, moving it would downgrade the dead-man to timeout inference, which is a safety regression. If it is already timeout-based, relocation is a straight improvement.

### 5.4 Motor control

**Sign-magnitude on GPIO 12 (20 kHz duty) and GPIO 23 (DIR).** All ESC-era logic removed: deadband reclaim, expo curve, arming handshake, `THR_MAX_US`, neutral pulse width, reverse-from-stop gate.

**D-09 watchdog: 0.2 s `/cmd_vel` timeout → duty 0.** Measured firing latency 171.464 ms after teleop loss.

**The watchdog is the primary stop mechanism** and it applies **full braking**, not coast:

- **Stopping distance:** 0.2 s of continued travel before the brake engages, plus braking distance — now measured (§4.19), totalling 0.120 m at 0.3 m/s and 0.266 m at 0.6 m/s.
- **Every gap in command flow is a braking event.** twist_mux source switches, DDS hiccups, and controller stutters were previously harmless coasts. **However:** peak measured deceleration is 2.50 m/s² (0.25 g) with no lockup signature up to 1.86 m/s, which partially de-risks the fishtail concern at current speeds. A graduated watchdog response remains an open design question for higher speeds.
**Demonstrated instance.** During bring-up, a manual `ros2 topic pub` and `teleop_node` published alternately through `twist_mux`, producing `0.3, 0.0, 0.3, 0.0…`; because duty 0 is active braking, the drivetrain repeatedly accelerated and braked. The underlying sensitivity remains.

### 5.5 Direction (D-74, supersedes D-42) — validated

Two rules replace the previous STOP/FWD/BRAKE/REV state machine:

1. **Latch the last nonzero commanded direction.** Hold it through zero commands, so braking while rolling forward continues to report +1.
2. **The latch may only flip when `/wheel/encoder_state.stationary` is true.** A reversal request while moving commands duty 0 until stationary, then flips and applies.
`/motor/direction` publishes the **latched** direction, not the pending request.

**The gate brakes during the hold; it does not hold the old direction at the new duty.** This is the property that makes reverse autonomy possible without controller cooperation — if it held the old direction, a cusp would drive the vehicle forward indefinitely and never reach stationary.

#### Field validation (`low_speed3_20260731_231040`)

| Measurement | Result |
|---|---|
| Direction latch flips | **23** |
| Flips with nonzero wheel speed | **0** |
| Minimum command zero-gap | **0.021 s** (one publication cycle) |
| Reversals initiated above 0.3 m/s | **16** |
| Fastest reversal initiated | **1.44 m/s** |
| Wheel speed at first reverse command | up to 0.41 m/s |

The operator flipped triggers faster than the vehicle could respond, and the latch held every time. **This is the cusp case, tested.** A bench smoke test (`stage1_reverse_smoke_20260801_130128`) additionally confirmed `DIRECTION_MISMATCH` firing across a transition and clearing cleanly, with the integrator untouched.

**Why this reverses D-42.** D-42 measured stop-gated `active_direction` at 27.6% steady-state disagreement with `sign(EKF vx)` and rejected it in favour of `pending_direction`. The defect was specific: the gate was a **200 ms timeout inferring stoppage**, too coarse to fire during real driving, so the sign sat stale for up to 5.25 s. The new gate **measures** stationarity. Same architecture, different reliability.

**DIR is asserted to the latched direction at all times, including during braking.** Electrically irrelevant at duty 0; avoids a glitch on the next drive command.

**Gate state is three-valued (D-75):**

| State | Source | Action |
|---|---|---|
| MOVING | encoder, fresh | block reversal, brake |
| STATIONARY | encoder, fresh | permit flip |
| **UNKNOWN** | absent or stale | **block, and publish why** |

Failing closed on UNKNOWN is correct; failing **silently** is not — that defect made reverse unreachable for an entire session with no diagnostic.

**Known gap — demonstrated, not theoretical.** If the wheels lock and the vehicle slides, the encoder reads stationary while the vehicle is still translating, so the latch could flip mid-slide. An EKF `vx` cross-check is the candidate mitigation; cost unpriced. Partially de-risked by the measured 0.25 g peak braking with no lockup signature.

---

## 6 · Failure taxonomy (D-27) and slip observability

Straight-line wheelspin is tolerated. Fishtail is the repeatable break: real lateral `vy` the vx-only EKF cannot represent.

**Lateral velocity is unobserved.** RF2O's `vy` is hardcoded to zero; the wheel encoder cannot provide it (D-26); the EKF has no `vy` state. This is a *sensing* gap, not a data gap.

### 6.1 What is measurable today

**Longitudinal slip ratio is directly measurable.** The encoder measures wheel surface speed and is contaminated by slip; RF2O measures motion against the environment and is slip-independent. Therefore:

```
κ_longitudinal = (v_encoder − v_RF2O) / v_RF2O
```

This requires no new hardware and can be computed from existing bags — `wheelspin_guard` events (0.5–0.95% of samples in recent runs) are slip events with both sensors recording.

**Lateral slip is not measurable as configured**, because RF2O's `vy` is hardcoded to zero. Fixing that is the single highest-value change for slip observability.

**The IMU offers an indirect route.** Body-frame kinematics give `v̇y = a_y − ω_z·v_x`, with all three terms measured. Integrated over a short window, `β = atan(v_y/v_x)`.

> **The lever arm is not negligible and may dominate.** `imu_link` sits 0.1233 m forward of `base_link`, so a rotating body gives `a_y_imu = a_y_base + ω̇_z·r_x`. At 5 rad/s² of yaw acceleration that is **0.62 m/s² of pure artifact**, while the real signal at 0.5 m/s and 1 rad/s is `ω_z·v_x` = 0.5 m/s². **The artifact is larger than the quantity being measured.** Correcting it requires differentiating the gyro, which is noisy. Additionally, the estimate drifts under integration and depends on `v_x` from the encoder, which is itself wrong under longitudinal slip.

**One favourable consequence of the frame choice:** `base_link` is at the rear axle, so β measured there *is* the rear tire slip angle in the bicycle model — the quantity a slip controller actually wants.

### 6.2 Why slip control would need a different controller

The entire current stack assumes **wheels roll without slipping**: encoder speed *is* vehicle speed (the PI's feedback), `δ = atan(L·ω/v)` is a kinematic identity valid only at low lateral acceleration, and `wheelspin_guard` actively rejects slip.

Three consequences, none of which are platform limitations:

1. **The kinematic steering map breaks and becomes non-monotonic** — past the friction peak, more steering gives less lateral force. Slip is measurable without the steering command being knowable.
2. **The PI would need to close on RF2O rather than the encoder**, or the setpoint would need to become a slip ratio rather than a speed.
3. **Bandwidth is the binding constraint.** RF2O publishes at ~10 Hz — the slowest signal in the stack — and degrades under fast rotation from scan smear, which is precisely the condition slip control would operate in.
**The correct architecture, if pursued, is a maneuver primitive** with defined entry and exit conditions and bounded duration, shaped like `BackUp` — not a mode the whole stack lives in. This is the drone-flip pattern: exit position hold, run a maneuver under different control laws and feedback, re-acquire position hold. **The architectural prerequisite** is an honestly-typed normalized-command interface with its own mux priority and dead-man (§9.3), because such a primitive must reach past the SI seam.

**New pathway to the fishtail failure.** The D-09 watchdog applies full braking on any command gap. §4.13 supplies the mechanism for non-recovery.

---

## 7 · Nav2 autonomy stack

### 7.1 Architecture

```
Foxglove click → /runner/waypoint → foxglove_goal_bridge → NavigateThroughPoses
  → bt_navigator → Smac Hybrid-A* (Reeds-Shepp) → /plan
  → RegulatedPurePursuitController (vendored, allow_reversing) → /cmd_vel_nav
  → drive_adapter → /cmd_vel_auto → twist_mux → /cmd_vel → motor_node
```

**Smac produces a geometric path with no timing.** Speed is entirely RPP's, decided reactively from curvature and obstacle cost. Replacing this with a time-parameterized trajectory is the Phase 2 change.

### 7.2 The two interfaces that matter

Two boundaries in this stack are real interfaces rather than internal calls, and they define where any future controller or learned policy attaches:

**The SI seam — `Twist` in m/s and rad/s.** Anything that can express "go this fast, turn this hard" plugs in here and inherits everything below it: feedforward, the PI, the speed floor, the ±0.14 clamp, the direction gate, the watchdog, twist_mux arbitration, and the teleop dead-man. MPCC, LMPC, and ILC all attach here. **The work of §4.19 and §4.15 is what makes this seam physically correct.**

**The normalized seam — throttle and steering as raw effort.** Below this there is no model of anything. Teleop injects here, which is why the teleop bypass is preserved deliberately (§7.9).

**Where learning attaches:**

| Level | Policy outputs | Examples | Safety |
|---|---|---|---|
| 1 · Goal selection | a target pose | frontier exploration | fully protected; every layer active |
| 2 · Trajectory | reference path + speed profile | **ILC** | protected; sample-efficient, no simulator |
| 3 · Control | `Twist` at the SI seam | **MPCC, LMPC** | protected |
| 4 · Actuation | normalized effort | slip maneuvers | **bypasses the adapter** (§6.2, §9.3) |

### 7.3 Behaviour tree (D-61)

```
PipelineSequence
├─ RateController (3 Hz)
│  └─ Fallback
│     ├─ ReactiveSequence
│     │  ├─ Inverter → GoalUpdated
│     │  └─ IsPathValid
│     └─ ComputePathToPose / ComputePathThroughPoses
└─ FollowPath
```

Every element fixes a specific measured defect. **Do not restructure casually.**

- **`Fallback`, not `ReactiveFallback`.** A reactive fallback re-evaluates `IsPathValid` while the planner is still running, sees the retained stale path as valid, halts the planner mid-execution, and the success callback never writes the new path. Symptom: `/plan` publishes and displays correctly while `FollowPath` receives zero poses and aborts with error 103.
- **`RateController` outside the `Fallback`.** Inside, it returned RUNNING and short-circuited the fallback, so the planner never ticked.
- **`RateController` at 3 Hz.** Unrate-limited replanning ran at ~12 Hz, producing steering rates to 18.6/s, saturation at 21.1%, and yaw oscillating at twice the commanded frequency. At 1 Hz an invalidated path persisted up to a second while `FollowPath` drove at the obstruction.
- **`Inverter(GoalUpdated)`.** Without it, `IsPathValid` returns SUCCESS on an *empty* path — trivially valid — so planning never runs.
- **`Spin`, `DriveOnHeading`, and rotate-in-place remain excluded.** The vehicle cannot rotate in place; if `Spin` fires it sits at full lock going nowhere until timeout. **This is a candidate explanation for the observed full-crank crawl at goal** (§7.5).
- **`BackUp` is now admissible** (§7.8).
### 7.4 Planner

| Parameter | Value |
|---|---|
| Plugin | `SmacPlannerHybrid` |
| **`motion_model_for_search`** | **REEDS_SHEPP** (D-85) |
| **`minimum_turning_radius`** | **0.60 m** (D-59 — see §3.1) |
| `allow_unknown` | false |
| `angle_quantization_bins` | 72 |
| **`smooth_path`** | **false** |
| `cost_penalty` | 2.0 |
| `max_planning_time` | 5.0 s |

**Planner radius exceeds the physical radius deliberately (D-59).** RPP must request tighter curvature than the path to correct cross-track error, so planning at the vehicle's exact limit guarantees saturation. At 0.60 m, planned curvature caps at 1.667 m⁻¹ against a 2.124 m⁻¹ vehicle limit — roughly 27% headroom. Measured effect: steering saturation 51.3% → 1.5%, yaw tracking 0.62 → 0.81.

**The smoother is disabled (D-58).** Smac obeyed `minimum_turning_radius` exactly — `/unsmoothed_plan` max curvature 2.128 m⁻¹ — but the smoother tightened `/plan` to **2.433 m⁻¹ (R = 0.411 m), 15% beyond the vehicle's physical limit.**

**Reeds-Shepp is enabled (D-85).** Reverse and cusp penalties are set conservatively high so cusps appear only where no forward-only solution exists. **Result: no region of the mapped apartment is unreachable.** Reeds-Shepp uses a single `minimum_turning_radius` for forward and reverse arcs alike, so planned geometry is direction-symmetric; any observed forward/reverse difference is an execution property, not a planning one.

**Expected side effect — monitor, do not treat as a reverse defect.** Reeds-Shepp expands the solution set, so goals that previously had one feasible plan may now have several near-equal-cost variants. Nav2 has no bias toward the plan already being executed. **First observation suggests it may have improved rather than worsened** the known plan-flicker case (§8.3): where two forward-only options were both marginal, a three-point turn is now unambiguously better, breaking the tie. Unvalidated.

### 7.5 Controller

**Vendored fork:** Nav2 1.3.12 at upstream `6be3614`, plus Runner path-cost regulation. Cusp distance correctly bounds both the normal and fixed-curvature lookaheads; direction is selected from the cusp-bounded carrot's robot-frame x.

| Parameter | Value |
|---|---|
| Plugin | `RegulatedPurePursuitController` (vendored) |
| `desired_linear_vel` | 0.45 m/s |
| **`allow_reversing`** | **true** (D-85) |
| `use_rotate_to_heading` | **false** — mutually exclusive with reversing, and structurally impossible on Ackermann |
| **`min_approach_linear_velocity`** | **0.25 m/s** (was 0.126 — inside the plant dead band) |
| **`regulated_linear_scaling_min_speed`** | **0.30 m/s** (was 0.15 — inside the plant dead band) |
| `regulated_linear_scaling_min_radius` | 0.75 m |
| **`cost_scaling_dist`** | **0.45 m** |
| `inflation_cost_scaling_factor` | **10.0** (must equal local `cost_scaling_factor`) |
| `use_cost_regulated_linear_velocity_scaling` | true |
| `use_collision_detection` | true |
| **`max_allowed_time_to_collision_up_to_carrot`** | **0.15 s** — **known inadequate, see §7.6** |
| Goal checker | `SimpleGoalChecker`, xy 0.10 m, **yaw 0.5 rad** |
| Progress checker | `SimpleProgressChecker`, 0.05 m / 10 s |

**Cost regulation must have a floor (D-64).** With `cost_scaling_dist` at 0.45 and no lower bound the vehicle crawled everywhere — median scan range is 1.09 m, so it is within 0.45 m of something almost always.

**RPP cost regulation reads cost at the robot's own cell.** Omnidirectional, no lookahead — unlike curvature regulation — so the vehicle approaches obstacles at full speed and only slows on arrival. An RPP limitation, not a misconfiguration.

**Collision detection is a binary abort (D-65).** Jazzy RPP 1.3.12 throws `nav2_core::NoValidControl` immediately rather than slowing.

**RPP structurally cannot** brake before a corner (it tracks a path, not a trajectory, and reacts to curvature at the carrot, at most `L_d` ahead), has no objective function to optimize, and has no constraint model — so it can command curvature the vehicle cannot execute.

**Lookahead versus feasibility — open, unquantified.** Pure pursuit computes `κ = 2·sin(α)/L_d`, so a lookahead below `2 / κ_max` permits requests beyond the vehicle limit. Against the physical limit of 2.1236 m⁻¹ that threshold is **0.942 m**; against the planner's 1.667 m⁻¹ it is **1.20 m**. Current lookahead is below both. Tension: the local costmap is 2.0 × 2.0 m rolling, giving only 1.0 m of radius, so a compliant lookahead sits near its edge. **Cheap first check: whether `steering_saturated` is true during the observed plough-straight events** (§8.3).

**Goal yaw and the full-crank crawl.** An Ackermann vehicle cannot correct heading at a point. If the goal checker demands a yaw the vehicle arrived outside of, the only way to satisfy it is to drive an arc at full lock and `min_approach_linear_velocity` until it succeeds or the progress checker gives up. **Raising the approach floor from 0.126 to 0.25 m/s made this faster and more conspicuous.** Remedies, in order of preference: widen the goal yaw tolerance substantially, or drop yaw from the goal check for point-to-point navigation. Operator has deprioritized.

**Expected side effect of the 0.25 m/s approach floor:** approximately 0.10 m of goal overshoot (0.047 m braking plus 0.05 m watchdog travel), which sits at the `SimpleGoalChecker` xy tolerance. Materially more than that indicates the floor is applied in the wrong place.

### 7.6 Collision horizon — quantified inadequacy

RPP projects the current commanded velocity forward for `max_allowed_time_to_collision_up_to_carrot` and checks the projected footprint against the local costmap. At 0.15 s and 0.60 m/s that is **0.09 m of lookahead** against **0.266 m of stopping travel**.

**Setting projection distance equal to stopping distance:**

```
0.15·v = v² / (2·(1.6v + 0.27))    →    v = 0.156 m/s
```

**Above 0.156 m/s the horizon is short at every speed.** It is not that 0.6 m/s is too fast — 0.15 s is simply below the vehicle's stopping *time*, which the braking model puts at roughly 0.45 s and near-constant across the range. **Slowing down does not fix it.**

| Speed | Travel before stop | Equivalent horizon | Current |
|---|---|---|---|
| 0.45 m/s | 0.192 m | 0.43 s | 0.15 s |
| 0.60 m/s | 0.266 m | 0.44 s | 0.15 s |
| 0.85 m/s | 0.392 m | 0.46 s | 0.15 s |

**Why it has not bitten.** RPP is not the only protection — costmap inflation keeps planned paths away from obstacles, and operation is attended.

**Why it is not a trivial fix.** At 0.45 s the projection reaches 0.27 m, into inflated space near walls in a narrow hallway, and the abort is **binary** — RPP does not slow, it throws and hands control to recovery. Too long a horizon means constant spurious recovery in tight spaces.

**Resolution: recovery behaviours first, then raise to 0.45 s and measure the spurious-abort rate.** Once recovery exists, an abort means clear-costmap-back-up-replan rather than the vehicle giving up, which largely removes the objection. Back off to 0.30 s if aborts prove frequent. **Requires a decision entry with the measured rate.**

### 7.7 Costmaps and semantic layers

| | Global | Local |
|---|---|---|
| Resolution | **0.050 m** | 0.025 m |
| Size | whole map, non-rolling | 2.00 × 2.00 m rolling |
| Layers | **static + obstacle (default-disabled) + inflation** | obstacle + inflation |
| `inflation_radius` | 0.30 m | **0.45 m** |
| `cost_scaling_factor` | 10.0 | 10.0 |

**Global costmap resolution must equal the served map resolution (D-50 amended).** Smac sizes its heuristic lookup table at configure time; if the static layer resizes the costmap afterwards the table is invalidated and valid tight maneuvers are rejected with `NO_VALID_PATH`.

**The global costmap loads an obstacle layer, disabled by default.** It is ordered between the static and inflation layers and uses Overwrite semantics when enabled. Earlier obstacle configuration marked nothing; fixing that caused lethal cells to accumulate **308 → 1078 → 1539 over 112 seconds, monotonically**, because a whole-map non-rolling costmap clears only where a later raytrace passes.

**Default consequence, accepted deliberately:** until the global obstacle layer is explicitly enabled, the global planner cannot route *around* unmapped obstacles. Live obstacles are handled by the local costmap and RPP collision detection.

**Obstacle staleness is a raytrace configuration problem, not a clearing-event problem.** The classic misconfiguration is `raytrace_max_range` shorter than `obstacle_max_range`, which marks cells at range that nothing is ever allowed to clear. Costmap clearing should be a *recovery*, not routine maintenance.

**Keepout filter and semantic layers (D-79, extended).** Nav2's `costmap_filters` consume masks natively — a mask is a separate occupancy grid registered to the `map` frame with identical `resolution` and `origin`, layered into the costmap. `KeepoutFilter` convention: 100 = keepout, 0 = free.

**The concept is broader than "keepout."** What is actually needed is a **semantic layer**: not "where can't I go" (one binary annotation) but "what is this place" — traversable path, hazard, workspace boundary, named goal, start pose. It encodes facts no sensor on this platform can acquire (§3.2).

**Keep layer types separate.** Keepout is binary and a hard constraint; speed limits are graded and soft; they are consumed by different filters. Named poses are not grids at all and belong in a JSON pose list.

> **Critical property: masks affect planning only.** slam_toolbox in localization mode matches live scans against the **posegraph**, not the PGM. The PGM is a rendering used by the costmap. **Painting the occupancy grid therefore does not change localization** — planning will believe the edit and localization will not, and the disagreement is silent. Masks registered to the `map` frame **survive extrinsics changes that invalidate posegraphs**, which would have spared the D-76 map loss.

**Consumption is solved and configuration-level. Authoring is the gap** — there is no good tool in the ROS ecosystem, and painting a PGM in an image editor is what people actually do.

### 7.8 Recovery behaviours — planned, not implemented

`BackUp` is a separate action server in `nav2_behaviors`. It does not use the planner, the controller, or a path — it is an open-loop primitive, deliberately independent because recovery must work when the normal stack has failed.

**Design requirements:**

1. **`Spin` must remain disabled** (§7.3).
2. **Nav2's `BackUp` does perform local-costmap collision checking, but that check is 2D** — a stairwell reads as free space (§3.6). **The costmap check is not protection against the platform's actual hazard.**
3. **Three distinct failures want three responses**, not one:
| Symptom | Cause | Response |
|---|---|---|
| Throttle commanded, encoder reads zero | physically blocked or stalled | back up — genuinely stuck |
| Encoder turning, pose not advancing | wheelspin, or localization failure | backing up may be wrong; if localization is lost, moving worsens it |
| Collision abort from LiDAR | path blocked | clear costmap, replan; back up only if no plan exists |

**The stall trigger is cleanly detectable from existing telemetry:** commanded speed above the floor with `measured_speed` near zero for longer than breakaway takes — roughly 0.5 s. This is far faster and more specific than `SimpleProgressChecker`'s 0.05 m in 10 s.

**Recovery is the prerequisite for raising the collision horizon** (§7.6).

### 7.9 Why teleop bypasses the drive adapter — deliberate

Teleop publishes normalized effort directly and does not pass through the feedforward or PI. This is intentional and must be preserved:

- **Teleop is the characterization instrument.** Every plant number in §4.19 exists because teleop injects a known normalized command with nothing between it and the motor. Routing teleop through the PI would make the plant permanently unmeasurable — you would be measuring the closed loop and calling it the plant.
- **Teleop is the fallback.** A controller bug must not remove manual control.
- **A trigger is an effort command, not a speed command.** Closing a speed loop around a human's finger produces cruise-control feel that fights the operator on grade and during recovery.
**There is one controller and one passthrough, not two control stacks.**

**A worthwhile addition, not yet built:** an SI-speed teleop mode publishing to `/cmd_vel_nav`, so the operator can command "0.6 m/s" through the *same* controller. That is a second command source into one controller — the opposite of duplication — and it enables closed-loop step tests without Nav2 in the loop.

### 7.10 Measured performance (validated, MD13S plant)

| Metric | Value | Source |
|---|---|---|
| Steering saturation | 1.04% | `2B_20260801_013907` |
| Yaw tracking (measured/commanded) | 0.866 | same |
| Adapter loop rate under `FollowPath` | **20.006 Hz** | same |
| Steady-state speed error, median | **+0.0001 m/s** | same, n=988 |
| Steady-state speed error, rms | 0.0145 m/s | same |
| Overshoot on standing starts | **0** on all four | same |
| Peak `final_throttle` | 0.0749 of 0.14 ceiling | same |
| `feedforward_floor_violation` | 0 | same |
| `effective_speed != commanded_speed` | 0 | same |

**Yaw tracking gain varies inversely with steering saturation across configuration segments** (0.43–0.88 gain against 0.15–6.62% saturation in `pid0`), which is consistent with the lookahead-feasibility question in §7.5.

---

## 8 · Open items

### 8.1 In flight — closable by one bag

- **Stage 4 reverse acceptance measurements never reported.** Cusp overshoot distance, `DIRECTION_MISMATCH` duration per cusp, integrator behaviour across the cusp, `steering_saturated` fraction during cusp segments, total time per cusp, and **whether the sign flip occurred with zero intervening zero-command cycles** — the one case never directly observed.
- **Reverse tracking asymmetry.** Fit `measured_yaw_rate` against `commanded_yaw_rate` **during reverse segments only**. Negative gain ⇒ the adapter uses unsigned `v` in `δ = atan(L·ω/v)`, a real sign bug. Positive but well below the forward value (0.81–0.88) ⇒ trailing-axle instability, which is genuine physics: a front-steered vehicle is directionally stable forward and unstable in reverse, so heading errors grow rather than decay. Remedies for the physics case are longer lookahead in reverse and lower reverse speed.
- **Confirm `Ki = 0.01` is applied** via `/speed_envelope/status`.
- **Decision entry recording the frozen longitudinal values** with supporting evidence.
### 8.2 Before extended autonomy

- **Verify `δ_max = 0.3614`** — unverified since the stairs incident and the rewiring; load-bearing for the adapter.
- **Inspect the LiDAR carrier pegs** (§3.6) and check extrinsics repeatability before and after a driving session.
- **Verify Z is unchanged** after the sled rotation (§4.5).
- **Re-measure braking on clean tires** (§4.19) — twenty minutes, same protocol; log tire state as a session condition.
- **Resolve steering-requires-X** and decide watchdog-versus-steering behaviour (§5.2).
- **Recovery behaviours** (§7.8).
- **Keepout mask and workspace geofence** (D-79) — oldest outstanding safety item, mitigation for an incident that actually occurred.
### 8.3 Known behaviours, unresolved

- **Plan flicker at ambiguous branch points.** Where two homotopies have near-equal cost, the argmin flips with costmap noise because nothing biases toward the plan already being executed. Observed: approaching a split with two feasible U-turn options, the vehicle occasionally commits to neither and drives straight past both. **Mechanism:** RPP derives steering from the carrot; if the path alternates between branches, the carrot alternates, steering alternates, and mean heading is straight. **A faster servo would not fix this** — it would sharpen the zigzag. Candidate fixes: gate replanning on path invalidity rather than a timer, and a plan-acceptance hysteresis filter accepting a new global plan only if better by a margin or if the current one is invalid. **May have partially dissolved with Reeds-Shepp** (§7.4) — re-check before building.
- **Steering saturation and lookahead feasibility** (§7.5). Cheap first check available in existing telemetry.
- **Cusp overshoot.** RPP does not slow before a cusp, so the vehicle brakes from tracking speed and stops past the geometric cusp — estimated **0.125 m at 0.45 m/s** (0.102 m braking plus one 50 ms control cycle; the watchdog delay does not apply because RPP keeps publishing through the cusp). **This is a tracking-accuracy question, not a safety one** — the D-74 gate protects the hardware regardless. **Measure before mitigating.** The floor on any mitigation is ~0.06 m, since the plant has no steady state below 0.20 m/s and `regulated_linear_scaling_min_speed` is 0.30 — so the entire available improvement is 0.125 m → 0.06 m.
- **Full-crank crawl at goal** (§7.5). Operator-deprioritized.
### 8.4 Deferred by decision

- **Collision horizon** (§7.6) — needs a decision entry with a measured spurious-abort rate. Gated behind recovery behaviours.
- **RF2O `vy`** — blocks all lateral slip observability; independently useful for localization.
- **`/cmd_vel` type lie** (§9.3).
- **IMU scan deskewing** — binds at high speed only.
- **Validate dynamic global replanning** — the global obstacle layer is loaded with `combination_method: 0` (Overwrite), disabled by default, and the planning tree has one bounded `ClearEntireCostmap` retry. Physical marking, low-obstacle, and accumulation acceptance remain outstanding.
- **Fix identical-goal re-dispatch** (§4.16).
- **Physically validate obstacle marking** with a placed object. Never done on either costmap.
- **Bag topic allowlist and `always_send_full_costmap` audit** (§4.11).
- **Sudo-free service restart** (§4.2).
- **Re-run the localization baseline** (§4.10) against the new extrinsics, RF2O fix, and new map.
- **Lookahead-based cost regulation** — sample maximum cost between robot and lookahead point rather than at the robot's cell.
- **UVLO margin measurement** — pack voltage under hard launch at low SoC against the 6 V floor.
### 8.5 Ratified, not implemented

- Hardware pulldowns on PWM and DIR (§3.4)
- Mechanical reset: brass inserts, nylon screw fuse, spare carrier plates (§3.6)
- ADS1115 traction voltage → `/battery/traction` — **promoted to necessary by the UVLO**
- Board revision: PD sink, 2S balanced charger, **heartbeat-gated high-side FET**, IMU, IO breakout
- Two operating modes (D-79), extended to tiered authority (§9.4)
- Map bundle and offline refinement tooling (D-80)
- Automatic per-run recording (D-81)
- Persist last pose on shutdown as the default localization seed
- Delta-peak NiMH charger for SoC repeatability
- 18650 replacement cells (§3.5)
### 8.6 Deferred with triggers

| Item | Trigger |
|---|---|
| **IMU scan deskewing** | Sustained high-speed operation |
| **SIGKILL hazard closure via FET** | Must close before sustained high-speed operation |
| **Graduated watchdog response** | High-speed operation (§5.4) |
| **EKF `vx` cross-check on the stationary gate** | Demonstrated gap; partially de-risked by measured 0.25 g peak braking |
| **Cliff sensing** (ToF, direct-to-driver reflex) | Observed at §3.6; keepout filter ships first. **Prerequisite for unattended exploration** |
| **AMCL as a second localization arm** | After map refinement tooling exists (D-80) |
| **Autonomous frontier exploration** | Attended, dead-man only. **Keepout cannot mitigate it** — a mask cannot be drawn for a map that does not exist. Worth testing first on an already-mapped room to compare coverage against accuracy, which removes the stairs risk entirely from the first iteration |
| **ILC** | Needs a repeatable start pose and repetition, **not a racetrack**. A 15 m apartment loop at 0.6 m/s is a 25-second lap; forty laps is under twenty minutes. A bare garage is likely *worse* — large flat featureless concrete is the classic failure case for 2D scan matching |
| **MPCC / LMPC** | Requires a path representation with boundaries and arc-length coordinate, 20–50 Hz solves, and a decision on whether the controller runs inside `controller_server` at all |
| **Docking station** | Charger, anchor, repeatable session origin |
| **Slip maneuver primitive** | Requires RF2O `vy`, a dynamic model, and §9.3 |
| **UWB** | Observed perceptual aliasing — not observed |
| **Quadrature encoder** | **Closed — not proceeding (D-77)** |
| **Event cameras** | **Closed — reading interest only** |

---

## 9 · Paddock — operator console (planned)

### 9.1 Scope

A Pi-hosted web operator console, installable as a PWA. **Not a Foxglove replacement.** Foxglove is the pit wall — arbitrary introspection, TF trees, plotting, bag scrubbing, parameter editing. Paddock is where the car is handled — driving, missions, map editing. Attempting to replace Foxglove is how this becomes a six-month project.

v1 targets two modes: **mapping** (joystick plus live map, for walking behind the vehicle while it maps) and **autonomous** (map, hold-to-drive or toggle, speed parameters, stop).

### 9.2 Architecture

One process that is both a ROS node and a web server: FastAPI + uvicorn with `rclpy` in-process, single systemd unit, application tier.

**The threading constraint:** `rclpy` callbacks and asyncio do not share an event loop. Run the ROS executor in a dedicated thread, have callbacks write to a plain state dict (latest wins), and have asyncio senders read it on fixed timers. Never `await` inside a ROS callback. This decouples ROS callback rate from WebSocket send rate.

**Transport: WebSocket, not WebRTC.** WebRTC's advantage would be unordered/unreliable delivery so stale samples drop, but a monotonic sequence number plus timestamp gives the same semantics with no signaling, ICE, or `aiortc`.

| Channel | Direction | Rate | Encoding |
|---|---|---|---|
| Joystick + heartbeat | → Pi | 20 Hz | JSON `{seq, t, v, w}` |
| Pose + adapter state | → browser | 10 Hz | JSON |
| Local costmap | → browser | 2 Hz | **binary frame**, raw int8 |
| Path / plan | → browser | on change | JSON polyline |
| Mode, map save, files | → Pi | on demand | REST |

**Use the local costmap rather than raw `LaserScan`** for the live overlay. It is slightly larger (6.4 KB at 80×80 versus ~2.0 KB) but already in the map frame, already inflated, and requires zero client-side TF. The static occupancy grid never changes during a run and is served once as a PNG over HTTP.

**Mode switching via systemd targets:**

```
runner-hardware.target     always up
runner-estimation.target   always up
runner-mapping.target      Conflicts=runner-localization.target
runner-localization.target
runner-autonomy.target     Requires=runner-localization.target
```

**`Conflicts=` is load-bearing** — both slam_toolbox modes own `map→odom`, and enforcing exclusivity at the unit level makes the one-owner rule structural rather than remembered. **Paddock must never be able to touch the hardware tier**; run it as a non-root user with a sudoers entry scoped to `systemctl start|stop|status runner-*.target`.

### 9.3 Safety — non-negotiable

**Stop is the default state.** A Pi-side node stops the vehicle whenever the heartbeat counter goes stale (>~150 ms), **independent of socket state**. The joystick *suppresses* the stop; stop is never a message that must arrive. Browsers can take tens of seconds to report a dead socket — **never gate on `onclose`**.

**Rationale.** A backgrounded tab, a sleeping phone, or a WiFi roam does not throw an error. It goes quiet, and quiet is indistinguishable from an operator holding steady. This is the same failure class as DDS-over-VPN staleness, but worse, because browsers are *designed* to deprioritise background tabs.

**Speed ceilings are enforced server-side.** The client sends normalized `[-1,1]`; the server scales to the mode's ceiling. Never trust the client.

**Hold-to-drive versus toggle is part of the mode definition**, not a free-floating UI switch — otherwise toggle silently becomes the default and the dead-man becomes decorative.

Phone specifics: `navigator.wakeLock` while driving; `tailscale cert` for HTTPS, which PWA install and wake lock both require; pointer events with `touch-action: none` on the joystick.

### 9.4 Tiered authority (extends D-79)

**The governing property is required response latency, not interface reliability.** Joystick control is *continuous* — every command matters and a stale one keeps driving. Autonomous supervision is *interrupt* — the vehicle is already doing something safe and the human only intervenes. A 300 ms hiccup in a stop button is survivable in a way that the same hiccup in a joystick is not.

| Interface | Ceiling | Failure detection |
|---|---|---|
| Phone joystick | **0.3 m/s** | heartbeat staleness only |
| Autonomous supervision | full | vehicle is already safe; human is a watchdog |
| DualSense | full | Bluetooth reports disconnection as an event |

Note 0.3 m/s sits just above the 0.20 m/s plant floor and the 0.25 m/s command floor — **there is no room to go slower** without retuning the envelope.

**Enforce tiering in one owner.** A mode-authority node that owns the current mode and publishes required-interlock state, with everything else subscribing. Checking "racing mode requires dead-man" in the BT *and* the adapter *and* teleop guarantees they drift apart and one silently stops checking.

**Architectural prerequisite for any level-4 policy (§7.2):** `/cmd_vel` is typed `geometry_msgs/Twist`, which asserts m/s and rad/s, while carrying normalized effort downstream of the adapter. Teleop exploits this; a learned policy must not. A level-4 interface needs an honestly-typed normalized-command topic with its own mux priority and dead-man.

### 9.5 Map editor

**Rendering:** two stacked canvases, absolutely positioned. Background holds the occupancy grid, drawn once and redrawn only on pan/zoom; overlay holds pose, path, and costmap, cleared each frame. Redrawing a 2000×2000 grid every frame will not perform on a phone.

**The transform to write once and unit-test**, since ROS `y` is up and canvas `y` is down:

```
px = (map_x − origin_x) / resolution
py = height − (map_y − origin_y) / resolution
```

**Mask authoring:** offscreen canvas at exact map pixel dimensions; brush as circle stroke, rectangle as `fillRect`, eraser as `destination-out`; brush size specified in metres and converted via resolution; **bounded** undo stack of `ImageData` snapshots (a 2000×2000 snapshot is 16 MB); save via `canvas.toBlob()` to a POST that writes PGM plus YAML alongside the map.

**Poses are not painted.** Click-to-place writes a JSON list `{name, x, y, yaw}`. The canvas is only the input method.

**Initial pose:** `PoseWithCovarianceStamped` on `/initialpose`, click for position and drag for heading. **Set realistic covariance, not zeros** — an overconfident seed makes slam_toolbox slow to correct a bad guess. `map→odom` jumps on acceptance, so **gate the control on the vehicle being stationary.**

**Painted plan → snapped path.** Capture the drag as a map-frame polyline, simplify with Ramer–Douglas–Peucker at ~0.1 m tolerance, sample waypoints every ~0.5 m, then call `ComputePathToPose` between consecutive pairs with `use_start: true` and concatenate. **Do not write a curve fitter** — Smac already enforces the turning radius, Reeds-Shepp motion model, cusp penalties, and costmap. The stroke is a *corridor hint*; the planner does the snapping. Derive waypoint headings from the stroke tangent. Render individual unplannable segments red rather than failing the whole stroke.

**Multi-bag map fusion is multi-session SLAM** and is by far the hardest item here. slam_toolbox has native support for continuing from a serialized posegraph and for merging; check what is available in Jazzy before building anything. Keep it clear of v1.

### 9.6 Status panel

Logs are the wrong abstraction for "is it up yet." Three columns per subsystem:

- **systemd state** — `systemctl show <unit> --property=ActiveState,SubState`, 1 Hz → is the process alive
- **lifecycle state** — Nav2 nodes are managed; subscribe to `/<node>/transition_event` → is it ready to work
- **data freshness** — last-message age on one representative topic → green fresh, amber stale, red never seen
**The third column is the one that matters** — a node can be `active` and publishing nothing, which has happened on this platform repeatedly. Keep the log stream (`journalctl -u <unit> -f -o json` → WebSocket) behind a click for diagnosis.

Use **Groot2** for behaviour-tree inspection; it highlights the currently ticking node live and is far better than reading XML. `rqt_graph` or Foxglove's graph panel for topics.

### 9.7 Build order

1. FastAPI + rclpy skeleton, WebSocket, heartbeat gate with Pi-side stop
2. Joystick, 0.3 m/s ceiling — **this alone replaces carrying the laptop**
3. Static map render + live pose overlay
4. Status panel
5. Initial pose seeding
6. Costmap + path overlay
7. Click-to-goal
8. Painted plan → snapped path (requires 6 and 7)
9. Keepout painting + save
10. Mode switching, missions, logs behind a click
Each step is independently useful.

---

## 10 · Decision log

Append-only. D-01…D-82 unchanged (v0.5–v1.1), except where explicitly superseded or amended below.

| ID | Decision | Reasoning |
|---|---|---|
| D-83 | **Unified throttle semantics. Under R1, L2 commands `−fixed_throttle_setpoint`; trigger depth selects direction only, never magnitude. Sign and magnitude source are stated separately for every input path (§5.2). Supersedes D-48's R1/L2 asymmetry.** | The asymmetry — R2 fixed, L2 proportional under R1 — was real, deliberate, and documented in code, unit tests, and `throttle_characterization_protocol.md`, yet invisible in the spec, whose mode table collapsed "which control selects the mode" and "which control selects direction" into one column. That ambiguity propagated into an executor brief and produced a false "R1 is broken" investigation; the executor correctly escalated rather than editing. Defensible under the ESC where negative meant brake; under the MD13S, negative is true reverse and the asymmetry makes fixed-throttle characterization impossible in one direction. **The defect was in the spec, not the code**, so the remedy is a section stating semantics once across all three input paths, updated together with implementation, tests, and the characterization protocol in one commit — the code/tests/docs split is how the asymmetry survived. |
| D-84 | **MD13S plant characterized; longitudinal controller frozen. Feedforward `\|cmd\| = 0.1188·\|v\| + 0.0174`; Kp 0.05, Ki 0.01, integrator bound ±0.005, `output_max` ±0.14; commanded speed clamped to exactly 0 or ≥ 0.25 m/s; PI is magnitude-domain with sign on a separate channel. Supersedes all ESC-era throttle parameters and the 0…+0.70 bound of D-73.** | 63 steady-state plateaus across two sessions give slope 8.420 with forward and reverse only 4.2% apart and identical residuals split or combined, so **one feedforward serves both directions**. Descending ladders confirm no usable steady state below 0.20 m/s — at cmd 0.03 the standard deviation exceeds the mean, which is stick-slip, not slow motion — which invalidated `min_approach_linear_velocity` 0.126 and `regulated_linear_scaling_min_speed` 0.15 as physically unachievable. `output_max` is derived from `FF(0.60) + Kp·0.60 + I_max` with slack; **both prior values were artifacts** — 0.70 an ESC-era carry-forward whose upper bound D-73 never re-derived, 0.12 a provisional clamp. The integrator bound is derived from the measured ±0.03 m/s inter-surface intercept spread ÷ gain, and **must not be widened during tuning** — a pinned integrator indicates a feedforward defect. Integral time is 2τ rather than τ because loop delay is comparable to τ. Gains are **frozen, not optimized**: validated over 703 s with zero saturation across 18 steps, 6.73% overshoot on the one clean large step, and steady-state error ≤ 0.017 m/s. No deficiency was observed, so no gain sweep is justified — tuning higher would buy rejection of disturbances never seen. Magnitude-domain error is what prevents integrator wind-up at a cusp and what makes a single controller serve both directions. |
| D-85 | **Reverse autonomy: Smac `DUBIN → REEDS_SHEPP` with high cusp penalties; RPP `allow_reversing: true`; adapter output symmetric ±0.14. RPP must not subscribe to `/motor/direction` or `/wheel/encoder_state`.** | The D-74 gate is enforced in `motor_node`, downstream of every command source, and evaluates wheel speed alone — it is indifferent to upstream intent and cannot be bypassed by a caller unaware of it. Validated across 23 reversals with a minimum command zero-gap of 0.021 s, sixteen initiated above 0.3 m/s and one from 1.44 m/s, **every flip at exactly zero wheel speed**. This also proves the gate *brakes* during the hold rather than holding the old direction at the new duty; otherwise a cusp would drive the vehicle forward indefinitely. **RPP therefore does not need to know a stop is coming and must not be made responsible for producing one** — commanding zero and waiting for stationary would duplicate a rule the actuator layer already enforces and place a hardware constraint in a portable controller. A proposal for RPP to own a full cusp state machine with actuator-state subscriptions was **rejected**: Nav2 already passes current velocity into `computeVelocityCommands`, so no subscription is needed; the one-cycle mismatch it would prevent is the harmless three-sample `DIRECTION_MISMATCH` window observed in the bench smoke test; and the coupling would break simulation and portability while creating a second consumer of actuator state that can disagree with the adapter's. Result: no region of the mapped apartment is unreachable. Residual deficiency is **positional, not safety** — cusp overshoot of roughly 0.125 m at 0.45 m/s — and is to be measured before any mitigation, which if needed is **deceleration only**, never a zero-command hold. |
| D-86 | **`speed_envelope.yaml` is the single committed origin for all longitudinal parameters. A dedicated observer node reads live values back and publishes `/speed_envelope/status` with a per-key divergence flag. Live tuning is restricted to a named subset; a lifecycle-transition helper is rejected.** | Values were scattered across Nav2 YAML, adapter YAML, launch overrides, and code defaults, so the same physical quantity could be set in one place and contradicted in another. **Single ownership is not achievable** — you cannot own Nav2's parameters without forking Nav2 — so the goal is single origin plus *detected* divergence. Divergence is not the danger; **silent** divergence is, and a shared file with N independent loaders and no readback produces exactly that the first time someone writes a parameter live. The observer must sit off the control path, because parameter service calls block and `drive_adapter` runs a 20 Hz loop; it must fail benign, publishing `unknown` on timeout. It justified itself immediately by exposing the `output_max` mismatch between the assumed 0.70 and the launched 0.12. The lifecycle helper is **rejected on safety grounds, not deferred**: a mid-route transition on `controller_server` silences `/cmd_vel_nav`, the D-09 watchdog fires, and duty 0 is a full brake — a tuning convenience does not justify inducing the D-27 fishtail. |
| D-87 | **Runner is explicitly not the thesis vehicle. Phase 2 is no longer deadline-bearing and the hardware freeze is lifted.** | A thesis, if one follows, will be built on a new platform from scratch with better hardware. Runner exists to reach interesting autonomy frontiers, to build operator competence, and to surface angles a thesis might take. Consequences: lap-time convergence data loses urgency, breadth of capability outranks depth in any single subsystem, and operator tooling becomes worth building because it will be used for years rather than for one experiment. **What is explicitly retained:** the engineering discipline, which is why this platform works rather than being a pile of half-finished features; and safety, without exception — §3.6 was not a thesis problem. **What is retained with a better reason:** repeated-lap capability, because LMPC's premise is that lap *N+1* improves on lap *N* from prior-lap data, making Phase 2 an enabler for the most interesting available control work rather than a deliverable. |

---

## Appendix · Workflow conventions

**Spec discipline.** The spec is a complete standalone reference, not delta-only. Divergent changes require a new decision-log entry.

**Codex:** read-only investigation first, then a separate action prompt against confirmed reality. **Codex commits and pushes** — SSH push access is configured.

**Codex brief authoring.** Codex has the repo — it reads every file and runs every command. Never paste source it can open; reference by path. Never write the implementation. Context is only what Codex cannot derive: intent, hardware state, prior decisions, physical constraints, what not to touch. **Target 1500–2000 characters; past ~4000, decompose into gated stages issued one at a time.** Every sentence must be doing one of four jobs: DO, DON'T, ACCEPT, or irreducible CONTEXT.

**Do not over-instrument a test whose observable is directly visible.** When the acceptance criterion is "a wheel turns," observing the wheel *is* the test. A bench smoke test was turned into repeated failed orchestration by building monitoring harnesses around it and then aborting on defects in those harnesses. Correspondingly, **over-specified acceptance criteria invite over-built validation** — six acceptance points for a change that was inert until exercised was itself the error.

**Validation is proportional (D-57).** Smoke checks only: it builds, the node comes up, the topic publishes, the value is not absurd. **Exception — silent failure modes.** If a change could fail with no visible symptom — wrong units, wrong frame, dropped data, a no-op write, a gate blocking without diagnostic — a quantitative check is required regardless of cost. Every one of this project's worst bugs was invisible to inspection.

**Do not re-validate unchanged code.** Re-testing `motor_node`'s direction latch after a change that did not touch it is waste. Unit tests covering a clamp across its whole input range are stronger evidence than a single bench observation.

**Executor chats:** paste `runner_collab_protocol.md`, then platform context, then the task brief. New chat per task. Executors escalate before acting on one-owner resources, phase scope, or decision-log changes (D-36). **Escalation working correctly is a success, not friction** — it has twice caught real errors in briefs.

**Measurement:** MCAP bags over comparable routes, analyzed by the same in-repo scripts. **State the analysis window** (D-45). Where gains or configuration changed mid-recording, **segment first and measure second** — whole-run statistics across configurations are meaningless, and gains are recoverable exactly from telemetry rather than from the 1 Hz observer. "Feels better" is not a result.

**Standing rules:** `source install/setup.bash` after every build. **Restart `foxglove_bridge` after any `runner_interfaces` rebuild** (§4.18). A runnable launch is complete in application nodes and ships with its VS Code task (D-29, amended D-71). Every new map is built through `/scan_slam` (D-37). A green build is not evidence a node runs. **Verify saturation and bounds before concluding a limit is the cause.** Report what was executed, not what is believed to follow from it.

CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.
