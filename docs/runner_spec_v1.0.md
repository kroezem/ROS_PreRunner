# Runner — Architecture & Current-State Specification

**Version 1.0** · supersedes v0.9 · 2026-07-29
Mattias Kroeze · MSc Autonomous Systems, DTU
Autonomous 1/18-scale RC research platform

---

## 1 · What Runner is

Runner is a stock LaTrax Prerunner 1/18-scale RC car converted into a self-contained autonomous research platform. The research contribution is **infrastructure-free iterative racing**: characterizing how localization quality bounds lap-time convergence on a fully self-contained, commodity-sensor vehicle.

All compute and sensing is onboard — no external motion capture, no fixed anchors. The self-improving controller (ILC / Learning-MPC) is DTU thesis work, deliberately deferred. This platform exists to make that work measurable.

**As of this version the vehicle reliably navigates a cluttered apartment autonomously** — multi-waypoint routes, U-turns, and tight-clearance maneuvering between furniture legs. Phase 1 is functionally complete.

**Guiding build principles**

- **Phase-gated scope.** A capability is added only once the failure mode that justifies it has been observed.
- **Prototype-first.** Localization quality is the contribution, not mechanical polish.
- **Diagnostic before fix.** Confirm the actual failure from data before changing anything.
- **A present sensor can still be silently failing.** RF2O published for months with zero covariance and separately stalled for 8.5 s while scans flowed. The lgpio encoder backend claimed GPIO 22 and delivered no callbacks. The global costmap's obstacle layer marked **zero** cells for weeks because its observation source loaded `max_obstacle_height: 0.0`.
- **A present estimator can be silently degraded.** Localization ran at 0.10 Hz correction rate while every topic looked healthy.
- **A consumer can silently reject data a publisher is correctly producing** (§4.12).
- **A signal's meaning can quietly widen past what it measures** (D-38). Five instances: `/cmd_vel.angular.z`; `/motor/direction`; `map→odom` translation as correction magnitude (D-41); "autonomy arming" describing a gate that does not exist (D-48); and `/plan` treated as the path `FollowPath` receives, when the blackboard write is a separate asynchronous step (D-61).
- **A metric can be right and reported wrong.** Every reported metric states its analysis window (D-45).
- **A fix can create the next failure.** Fixing global obstacle marking enabled `IsPathValid` to work, which caused 12 Hz replanning, which caused steering oscillation. Fixing that with a rate limiter caused blindness. Sequence matters as much as correctness.
- **One owner per resource.**
- **A runnable application launch must be complete on its own; persistent
  platform hardware services remain outside composites** (D-29, D-75).
- **Validation effort scales with hardware contact, not perceived importance** (D-57).

---

## 2 · Phase structure

| Phase | Goal | Status |
|---|---|---|
| Phase 0 | Bring-up: sensing, teleop, SLAM localization. | **Complete** |
| Phase 1 | Fixed-map localization + Nav2 point-to-point. | **Functionally complete** — reliable multi-waypoint autonomous navigation in a cluttered apartment. Closeout items in §8. |
| Phase 2 | Racing. Pure Pursuit → MPCC → LMPC. DTU thesis control. | Not started |

---

## 3 · Hardware

Raspberry Pi 5 8GB, Ubuntu 24.04, ROS 2 Jazzy; LD19 LiDAR (10 Hz, `/dev/ttyAMA0`); BNO085 IMU (UART `/dev/ttyAMA2`, 3 Mbaud, GPIO 26 reset via `pinctrl-rp1` by-label, D-12); US1881 hall encoder (GPIO 22, libgpiod, 0.010282 m/edge); X1201 UPS (compute only, I2C 0x36); DualSense teleop. LiDAR and IMU co-mounted.

### 3.1 Measured vehicle geometry (ratified)

| Quantity | Value | Basis |
|---|---|---|
| Wheelbase `L` | **0.178 m** | Measured |
| Max steer `δ_max` | **0.3614 rad (20.7°)** | Derived from fitted turning radius (D-43 amended) |
| **Physical minimum turning radius** | **0.470 m** | Circle fit to a 1034° sustained turn (residual sd 0.017 m) and a 123° U-turn (residual sd 0.0005 m); cross-checked against a 45-inch outside-wheel U-turn measurement |
| Maximum curvature | **2.1236 m⁻¹** | `tan(δ_max)/L` |
| **Width, wheels straight** | **0.165 m** | Measured, outside-wheel to outside-wheel |
| **Width, full lock** | **0.180 m** | Measured |
| **Length** | **0.290 m** | Front bumper to rear |
| **Costmap footprint** | `[[0.230, 0.0825], [0.230, −0.0825], [−0.060, −0.0825], [−0.060, 0.0825]]` | `footprint_padding: 0.0` |
| **Inscribed radius** | **0.060 m** | Governed by the rear overhang, not the width |
| **Circumscribed radius** | **0.2443 m** | Runtime-reported |

**Historical correction (D-59).** v0.8 recorded `R = 0.565 m` and `δ_max = 17.5°`. That radius was the **outer wheel's swept radius**, mistakenly read from a wall-parallel U-turn width as 2R at `base_link`; δ_max was derived from it and inherited the error. Both are superseded.

**Accepted unmodelled margin.** At full lock the vehicle is ~0.180 m wide against a 0.165 m footprint, so roughly 7.5 mm per side is unmodelled during hard turns. Accepted deliberately: it is small against the p95 localization pose jump of 0.096 m already tolerated, and full lock occurs only at low speed.

**Steering hysteresis.** Releasing from left lock settles left of centre; from right lock, right of centre. Linkage slop and the servo saver — not a trim offset, and no software constant fixes it. Presents to the path follower as a steering deadband.

### 3.2 LD19 scan characteristics

Fixed angular extent, varying resolution: `angle_min = 0.0`, `angle_max = 2π`; `len(ranges)` varies 495–509 (mode 504). Median `scan_time` 100.0 ms. Root cause of §4.12.

**Median valid scan range in the mapped apartment: 1.09 m.** This constrains planner reachability, yaw observability, and — critically — motion-smear magnitude (§4.14).

**Scan plane at z = 0.1135 m.** Objects below roughly 0.11 m are structurally undetectable. Not fixable in software; a documented operating-envelope limit.

### 3.3 Power architecture (D-51)

Domains separate by **current path**, not energy source. Ratified for the post-Phase-1 board revision: UPS permanent and trickle-fed from a traction-derived current-limited buck; traction bolted in, not hot-swappable; one USB-C PD input negotiating 12–20 V; Pi-controlled high-side FET on ESC power (closes the SIGKILL hazard); ADS1115 traction telemetry at I2C 0x48; contact pads not wireless for the eventual dock.

**Observed bottleneck is compute runtime, not traction.** UPS endurance is roughly two hours, which limits session length before the traction pack does. This inverts the original assumption and strengthens the case for traction-trickle-charging the UPS. Interim mitigation: a USB-C power bank connected to the X1201 input during sessions.

Traction voltage remains unobservable — `/battery` is the UPS fuel gauge only.

---

## 4 · Software architecture

### 4.1 Packages

| Package | Node | Role |
|---|---|---|
| `runner_bringup` | `rf2o_scan_canonicalizer` | RF2O scan-origin canonicalization (D-37) |
| `runner_bringup` | `scan_rebinner` | `/scan` → `/scan_slam`, fixed 503-bin angular rebinning (D-37) |
| `runner_bringup` | `foxglove_goal_bridge` | Goal and route management (§4.16) |
| `runner_imu` | `bno085_node` | BNO085 → `/imu/data` @ 50 Hz |
| `runner_motor` | `motor_node` | `/cmd_vel` → MD13S sign-magnitude + steering PWM. Persistent systemd hardware owner (D-75, D-76) |
| `runner_encoder` | `encoder_node` | Hall edges → `/wheel/odom`, `/wheel/encoder_state`. Sole GPIO 22 owner, libgpiod (D-54) |
| `runner_teleop` | `teleop_node` | `/joy` → `/cmd_vel_teleop`, three-state hold-to-run (D-48); keyboard bridge |
| `runner_drive_adapter` | `drive_adapter` | `/cmd_vel_nav` (SI) → `/cmd_vel_auto` (normalized), with PI speed control (§4.15) |
| `nav2_regulated_pure_pursuit_controller` | controller plugin | Navigation2 1.3.12 overlay with Runner path-cost lookahead regulation (D-73) |
| `runner_telemetry` | `telemetry_node` | SoC temperature and throttle bits, 1 Hz, standalone systemd |
| `runner_interfaces` | — | `EncoderState`, `AdapterState`, `SystemTelemetry`, `KeyboardState` |
| `runner_battery` | `battery_node` | UPS fuel gauge → `/battery` (systemd) |
| `ldlidar_stl_ros2` | `LD19` | LD19 → `/scan` (D-22) |
| `rf2o_laser_odometry` | `rf2o_..._node` | Laser odometry → `/odom_rf2o`. Vendored fork |

### 4.2 Launch tree (D-29)

```
launch/
├── map.launch.py       = sensors + estimation + slam_map      + teleop
├── localize.launch.py  = sensors + estimation + slam_localize + teleop
├── nav2.launch.py      = localize + map_server + planner + controller + bt_navigator + goal bridge
├── autonomy.launch.py  = nav2 + drive_adapter + twist_mux
├── teleop.launch.py    = joy + teleop_node + twist_mux
└── include/
    ├── sensors / estimation / slam_map / slam_localize
    └── lidar / imu / encoder / tf_static / rf2o / ekf
```

`motor_node` is not part of the launch tree. `runner-motor.service` starts it at
boot with no ESC-mode parameter. Composites own the complete application graph
but depend on the persistent platform hardware service (D-75).

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
| MD13S PWM/DIR + steering PWM | persistent `motor_node` systemd service |
| GPIO 22 | `encoder_node` |

### 4.5 Static extrinsics

`base_link` = rear axle, ground-projected (D-01). `base_link→base_laser`: x 0.132, y 0, z 0.1135, yaw 0. `base_link→imu_link`: x 0.082, y 0.0025, z 0.106, yaw π.

### 4.6 Encoder

**libgpiod backend (D-54)**, chip resolved by live label. The prior lgpio backend claimed GPIO 22 successfully and delivered no edge callbacks.

**Interval-timing estimator (D-49):** `edge_rate = (n−1)/(t_last − t_first)` over `history_depth` intervals, default **4**, using kernel CLOCK_MONOTONIC timestamps. Measured σ = 0.0034 m/s at 0.313 m/s. Effective lag ≈ `history_depth / edge_rate` — roughly 69 ms at 0.3 m/s, speed-dependent.

The prior fixed-window estimator quantized 0.3 m/s to exactly two values (0.206 / 0.411 m/s, σ 0.094).

**Magnet spacing is period-2**, from bipolar-latch switching asymmetry: ±1.5% in July, ±5.5% currently, structure unchanged. Cancelled entirely by depth ≥ 2. The magnitude change coincided with a measurement-method change and must be re-scoped on the signal line before any mechanical inspection.

**Direction (D-42):** `/wheel/odom` signed by `pending_direction` — latest nonzero command, latched through zeros. Steady-state disagreement with `sign(EKF vx)`: 0.0% versus 27.6% for stop-gated `active_direction`. The MD13S motor owner publishes the direct signed command.

**Magnitude is unreliable under wheelspin:** p90 wheel/EKF ratio 2.04. Not an EKF velocity source.

### 4.7–4.9 RF2O, covariance, wheel odometry

Constant twist covariance `vx 0.02`, `vyaw 0.25` (D-24). Stall root-caused to high-rate INFO logging, fixed in `ece93d1`. RF2O's `vy` is hardcoded to zero — lateral velocity remains unobserved (§6).

### 4.10 Localization quality (D-32, D-41, D-45, D-52)

**Metric:** induced pose jump at `base_link` — both consecutive `map→odom` transforms applied to the robot's current odom-frame position, distance between results. **Not** the translation of `map→odom`, which is inflated by distance from the odom origin (measured 7.259× at a 5.94 m mean lever arm).

**Cadence is conditioned on motion** (D-52) — slam_toolbox is motion-gated, so an unconditioned rate partly measures standing time.

**Reference, `runner_test_20260726_115736`, window t+20 s → end (154.2 s, 97.7 m):**

| Metric | Value |
|---|---|
| **Pose jump at `base_link`** — med / p90 / p95 / max | **0.0188 / 0.0658 / 0.0959 / 0.3556 m** |
| Yaw correction — med / p95 / max | 0.0175 / 0.1396 / 0.3560 rad |
| Cadence — unconditioned / motion-conditioned | 3.683 / **5.717 Hz** |
| Gap — med / p90 / p95 | 0.180 / 0.260 / 0.311 s |
| Correction per metre travelled | 0.175 m/m |

Driving profile: mean 0.626 m/s, max **2.798 m/s**, 52.4% throttle duty, deliberate drifting.

Against the v0.8 baseline (0.103 Hz): **35.8× cadence improvement at 2.7× the top speed.**

**Note:** p95 pose jump (0.096 m) **exceeds the inscribed radius** (0.060 m). A pose correction alone can therefore place the footprint in apparent collision. This bounds how tight collision checking can safely be.

### 4.11 Instrumentation

`tools/analyze_localization_bag.py` (invariant metric, windows, truncated-MCAP recovery), `analyze_yaw_cross_validation.py`, `analyze_throttle_response.py`. Typed diagnostics on `/drive_adapter/state_typed` and `/system/telemetry`.

### 4.12 Scan cardinality (D-37)

Karto registers a fixed beam count once and hard-rejects mismatched scans; 79% of the stream was silently discarded. Fixed by `scan_rebinner` — angular rebinning onto a fixed 503-bin grid, nearest-in-angle, `inf` for empty bins, header stamp preserved. Pad/truncate, interpolation, driver fork, and canonicalizer merging all explicitly rejected; do not revisit.

### 4.13 Phantom-geometry trap (D-40)

Localization mode retains recent scans; scans taken while mislocalized depict the real room at the wrong map location and the matcher reinforces its own error. Failure is a cliff, not a gradient. Not observed post-D-37 and cannot currently be induced.

### 4.14 Yaw lag from scan motion distortion (D-53)

**No yaw scale error.** Integrated over the full 174.684 s reference recording against the slam reference: IMU **1.0023×**, EKF **1.0040×**, RF2O 0.8862×.

**But a yaw-rate-proportional phase lag exists.** Regressing `map→odom` yaw against signed yaw rate: **slope −0.0652 s, correlation −0.596**. Sign flips with turn direction, magnitude grows with yaw rate, near zero when straight.

**≈65 ms ≈ half a LiDAR scan period.** The LD19 sweeps over 100 ms and nothing deskews, so Karto's best-fit pose corresponds to the middle of the sweep. **The pose is not wrong; it is late.**

| Yaw rate | Heading error | Scan displacement at 1.09 m |
|---|---|---|
| 0.5 rad/s | 1.9° | 3.6 cm |
| 1.0 rad/s | 3.7° | 7.1 cm |
| 3.0 rad/s | 11.2° | 21 cm |

**This has a direct navigation consequence, not just a localization one.** At 0.82 m/s through a 0.60 m radius the yaw rate is ~1.37 rad/s — roughly 0.15 m of scan displacement, **more than double the 0.060 m inscribed radius.** Hard cornering can therefore paint phantom obstacles into drivable space. Corner speed regulation is partly a smear-mitigation measure.

Fix is IMU-based scan deskewing. **Deferred to Phase 2**, where it becomes load-bearing.

### 4.15 Drive adapter (D-55)

Resolves the `/cmd_vel` unit mismatch: Nav2 publishes a true `Twist` (m/s, rad/s); `motor_node` consumes normalized ±1.0 in both fields.

```
Nav2 → /cmd_vel_nav (SI) → drive_adapter → /cmd_vel_auto (normalized) → twist_mux → /cmd_vel → motor_node
```

**Steering:** `δ = atan(L·ω/v)`, normalized by `max_steering_angle` 0.3614.

**Steering infeasibility clamps; it does not brake (D-60, supersedes D-56).** When requested curvature exceeds 2.1236 m⁻¹, steering saturates at ±1.0 and speed is maintained, accepting understeer. Brake-on-infeasible was correct for bench validation and wrong in execution: Smac plans at `minimum_turning_radius`, so RPP must exceed path curvature to correct cross-track error, making infeasible requests structural rather than occasional. Measured: with the planner at its old 0.470 m limit, saturation reached **51.3%** of driving samples with requests up to 83% over the limit; with planner margin (D-59) it fell to **1.5%**.

**Throttle:** feedforward lookup plus bounded PI.

| Parameter | Value |
|---|---|
| Feedforward table | 0.340→0.126, 0.350→0.188, 0.360→0.233, 0.380→0.290 m/s |
| Kp | 0.30 |
| Ki normal / stall | **0.06 / 0.30** |
| Gain switch | ratio < 0.40 or > 1.60, 0.10 hysteresis |
| Integrator bounds | −0.25 … +0.16 |
| Output bounds | **−0.20 … +0.70** |
| `maximum_commanded_speed` | 0.60 m/s |
| Minimum sustainable speed | 0.126 m/s |

**Error-dependent integral gain (D-66).** At Ki = 0.06 with a 0.29 m/s error the integrator needs 18 s to traverse its range, against a 10 s progress-checker abort — too slow to break a stall. Measured during stalls: throttle peaked at 0.530 against a 0.70 ceiling and the integrator at 0.065 against +0.16, so **neither was saturated** and the ceiling was never the constraint. The high gain is symmetric so persistent overspeed unwinds at the same rate.

**Wheelspin guard** freezes the integrator when encoder speed materially exceeds EKF `vx`.

**Teleop preemption freezes and decays the integrator (D-72).** The adapter
subscribes to `/teleop/active_mode`. A fresh value other than
`teleop_suppress` confirms that `twist_mux` is discarding autonomy output, so
PI accumulation freezes and the existing integral decays toward zero at
0.0625 normalized throttle contribution per second. Missing or stale mode
state preserves the previous behavior. The adapter continues publishing
`/cmd_vel_auto`; `twist_mux` remains the sole command arbiter.

**Breakaway** is gated on `/wheel/encoder_state.stationary` and re-arms on transition to moving, so a mid-path stall gets a fresh attempt.

**Stale `/cmd_vel_nav` produces silence, not brake.** Publishing brake would keep commands flowing and prevent the motor watchdog (D-09) from firing.

**Feedforward table is stale and truncated.** Measured on different flooring at a different battery state; commanded 0.290 has produced 0.354 m/s (ratio 1.24). Speeds above 0.290 m/s are extrapolation. Re-measurement is a Phase 1 closeout item.

### 4.16 Waypoint and route management (D-74)

`foxglove_goal_bridge` provides:

- `/move_base_simple/goal` — single-goal `NavigateToPose`, latest-goal-wins, generation-guarded, bounded cancellation retry
- `/runner/waypoint` (`PoseStamped`) — interpreted by the authoritative bridge mode; the complete operator-supplied quaternion is retained unchanged
- `/runner/route_control` — `start`, `stop`, `clear`, `loop_on`, `loop_off`, `remove_last`
- `/runner/route` (`nav_msgs/Path`), `/runner/route_markers` (numbered `MarkerArray`)
- `/runner/waypoint_queue`, `/runner/waypoint_queue_markers` — consumable queue visualization; the active front is distinct and arrows show orientation
- `/runner/autonomy_state` — goal state, mode, error meaning, route/queue progress, loop mode
- Routes persist to `~/.ros/runner_route.json`

The default **WAYPOINT** mode holds at most 20 complete poses in a
nonpersistent FIFO. The front dispatches through `NavigateToPose`; action
success consumes it and automatically starts the next. A navigation failure
retries the identical front once with a fresh action execution. A second
failure pauses and retains it. `stop` cancels and pauses without consuming,
`start` resumes the retained front with a reset retry budget, and `clear`
cancels and empties the queue. Chained single goals deliberately stop and go
at every waypoint; repeated breakaway is the accepted cost of this workflow.

**ROUTE** mode retains the persistent ordered collection and dispatches it as
`NavigateThroughPoses`, not chained single goals. Route start, stop, clear,
undo, looping, visualization, and storage keep their existing semantics.

Mode commands are absolute and idempotent. Entering WAYPOINT cancels route
execution, clears the persistent route using the existing semantics (the
separately persisted loop preference is retained), and starts an empty queue.
Entering ROUTE cancels queue navigation, clears the entire queue and
retry/pause state, and starts an empty route while retaining that same route
metadata. Repeating the already-active mode command changes nothing. One
bridge-owned monotonic generation guards send, acceptance, result,
cancellation, restoration, retry, replacement, stop, clear, and
mode-transition callbacks.

**Fresh identical execution:** Nav2 reuses the parsed tree when the XML
filename is unchanged; halt-to-IDLE does not clear `GoalUpdatedCondition`'s
remembered pose. Previously an identical goal could therefore leave a valid
old `path` on the blackboard and skip planning from the new robot position.
Both active trees now begin with `UnsetBlackboard key="path"` in a normal
`Sequence` before the unchanged `PipelineSequence`. The reset runs once per
action execution, so `IsPathValid` cannot accept a previous execution's path.

---

## 5 · Teleop & motor control

### 5.1 MD13S sign-magnitude drive (D-76, supersedes D-46/D-47)

GPIO12 supplies normal-polarity 20 kHz PWM and GPIO23 supplies DIR. Positive
normalized commands select forward, negative commands select reverse, and
absolute magnitude maps linearly to PWM duty. Zero duty is MD13S active brake.
Same-direction commands apply immediately. An opposite command first forces
duty to zero and remains pending until a subsequently received
`/wheel/encoder_state` sample reports `stationary=true`; only then does the
motor owner switch DIR and apply the latest pending duty. A zero command
cancels a pending reversal. There is no dwell or timeout-based reversal gate.
There is also no ESC arming sequence, pulse mapping, deadband, expo, or mode
parameter.

GPIO23 is exclusively requested from the `pinctrl-rp1` chip by label. GPIO22
remains exclusively reserved for the encoder and is not part of motor control.
`/motor/direction` reports the last nonzero signed command as `Int8` -1 or +1
through zero-duty braking, watchdog braking, and shutdown. It reports 0 only
before the process has received its first nonzero command.

The stationary gate inherits two explicit limits from its single-channel
encoder evidence. Locked wheels can report no edges while the vehicle is still
sliding, so `stationary=true` does not prove zero ground speed. Also, an encoder
GPIO worker failure is logged but currently leaves the publication timer alive;
after the last edge ages out, the encoder can continue publishing
`stationary=true`. Phase 3 is the trigger to revisit this evidence boundary:
before Phase 3 closed-loop performance work begins, either fault or any observed
locked-wheel slide during hardware validation requires independent motion and
encoder-health evidence before reversal authorization.

The existing teleop command labels still call negative commands "brake"; at
the motor boundary they are now reverse commands. Traction remains disconnected
until the separately planned wheels-off-ground validation resolves the complete
operator-control behavior.

### 5.2 Teleop three-state hold-to-run (D-48)

Priority **X > R1 > L1 > brake**. Releasing all buttons brakes within one publication cycle.

| Input | Behaviour |
|---|---|
| Nothing held | legacy "full brake" label, `linear.x = −1.0`; now MD13S reverse |
| **X** | manual — R2 forward, L2 negative/reverse command, stick steering |
| **R1** | fixed forward setpoint (D-pad adjusts), stick steering, L2 negative/reverse command live |
| **L1** | `teleop_suppress` — publishes nothing, autonomy passes through |

In code and docs L1 is **`teleop_suppress`**. "Autonomy enable" is operator-facing only; it is not an arming gate — the real gate is the mux and the watchdog.

**X→R1 latch:** pressing X while R1 is held suppresses fixed throttle until R1 is released and re-pressed.

Measured: R1 release-to-brake 5.065 ms; L1 41.350 ms; watchdog fires 171.464 ms after teleop loss.

### 5.3 Keyboard teleop (D-68)

`tools/keyboard_sender.py` runs on the operator laptop, captures keydown/keyup
via `pynput`, and sends version-two UDP packets at 20 Hz. The Pi-side bridge
owns packet validation, the **150 ms manual-input liveness timeout measured on
the Pi** (browser and laptop clocks are not trusted), the speed cap, and
arbitration. The physical controller always preempts and clears the keyboard
autonomy latch.

| Key | Behaviour |
|---|---|
| `` ` `` | toggle the autonomy latch; 600 s expiry from the original arm |
| `=` / `-` | throttle setpoint up/down |
| Escape | clear held/queued state and send brake/disarm for at least one second |
| F5 / F6 / F7 / F8 / F9 | route start / stop / clear / loop toggle / undo last |
| F10 / F11 | clear global obstacle marks / toggle the global obstacle layer |
| F12 | alternate the sender-requested WAYPOINT / ROUTE mode |
| Space + W/A/S/D | manual hold-to-run driving |

Protocol v2 retains the exact 27-byte layout. Command values 0–7 keep their
meanings: none, start, stop, clear, loop toggle, remove last, clear global
obstacles, and toggle global obstacles. Values 8 and 9 are the additive
`SET_WAYPOINT_MODE` and `SET_ROUTE_MODE` commands. F12 changes the sender's
local requested mode and queues a finite three-packet burst of the
corresponding absolute command. The bridge is authoritative; the printed
event is requested, not acknowledged, state. Both processes independently
start in WAYPOINT and sender startup does not send a destructive mode command.
F11 reads the current runtime parameter before requesting its inverse. Neither
costmap command changes the saved static map.

**The autonomy latch deliberately survives sender or connection loss
(D-71).** This prevents intermittent laptop links from stopping a route.
Escape is therefore the primary keyboard emergency stop and transmits a
repeated disarm burst; sender termination alone does not guarantee disarm.
The unchanged motor watchdog remains an independent stop path for current
low-speed Phase 1 operation. Explicitly disarm before leaving the system
unattended, and revisit this posture before Phase 2 speeds.

**Capture is global regardless of window focus.** Documented in the module
docstring and startup banner. Accepted deliberately so Foxglove can be used
while driving. Escape clears everything and the autonomy latch auto-expires.

### 5.4 Motor control

GPIO12 uses normal-polarity 20 kHz PWM (`period=50000 ns`) with signed command
magnitude mapped linearly to duty. GPIO23 is DIR. Duty zero is active brake.
GPIO13 remains normal-polarity 50 Hz steering PWM (`period=20000000 ns`).
`runner-pwm-setup.service` temporarily prepares PWM export and permissions;
persistent direct-sysfs runtime control belongs to `motor_node`, which never
unexports either channel (D-75/D-76, superseding the ownership portion of
D-23).

`runner-motor.service` starts at boot, requires the PWM setup service, and is
not stopped with an application composite. Composite teardown stops command
publication; D-09 writes duty zero and publishes direction zero after 200 ms
while the hardware owner remains alive. Shutdown also writes duty zero before
releasing GPIO23. The timer and steering command mapping are unchanged.

**Breakaway is direction-dependent** (0.340–0.380 normalized). Floor slope, drivetrain warm-up, and sweep order were confounded and never separated. The adapter uses the worst case observed.

**Residual hazard, unsolved in software:** SIGKILL bypasses `stop()` and leaves
the last sysfs duty active. The Pi-controlled high-side FET (§3.3) is the fix.

---

## 6 · Failure taxonomy (D-27)

Straight-line wheelspin is tolerated. Fishtail is the repeatable break: real lateral `vy` the vx-only EKF cannot represent.

**Lateral velocity is unobserved.** RF2O's `vy` is hardcoded to zero; the wheel encoder cannot provide it (D-26); the EKF has no `vy` state. This is a *sensing* gap, not a data gap — no amount of recorded driving produces slip labels.

**The IMU offers an indirect route:** body-frame kinematics give `v̇y = a_y − vx·ω`, and every term on the right is already measured at 50 Hz. The residual is a direct slip signal from a sensor that does not degrade under rotation. Accelerometer bias makes it a transient detector rather than an absolute `vy` source — which suits a fishtail. **Untested; the reference recording can validate it offline.**

§4.13 supplies the mechanism for non-recovery.

---

## 7 · Nav2 autonomy stack (Phase 1, operational)

### 7.1 Architecture

```
Foxglove click → /runner/waypoint → foxglove_goal_bridge → NavigateThroughPoses
  → bt_navigator → Smac Hybrid-A* (global) → /plan
  → RegulatedPurePursuitController (local) → /cmd_vel_nav
  → drive_adapter → /cmd_vel_auto → twist_mux → /cmd_vel → motor_node
```

**Smac produces a geometric path with no timing.** Speed is entirely RPP's, decided reactively from curvature and obstacle cost. Replacing this with a time-parameterized trajectory is the Phase 2 change.

### 7.2 Behaviour tree (D-61)

```
PipelineSequence
├─ RateController (3 Hz)
│  └─ RecoveryNode (1 attempt)
│     ├─ Fallback
│     │  ├─ ReactiveSequence
│     │  │  ├─ Inverter → GoalUpdated
│     │  │  └─ IsPathValid
│     │  └─ ComputePathToPose / ComputePathThroughPoses
│     └─ ClearEntireCostmap (global)
└─ FollowPath
```

Every element of this structure fixes a specific measured defect. **Do not restructure casually.**

- **`Fallback`, not `ReactiveFallback`.** A reactive fallback re-evaluates `IsPathValid` on every tick while the planner is still running. It would then see the *retained stale* path as valid, halt the planner mid-execution, and the planner's success callback would never write the new path to the blackboard. Symptom: `/plan` publishes and displays correctly in Foxglove while `FollowPath` receives zero poses and aborts with error 103. Diagnostic signature: `ComputePathThroughPoses RUNNING → IDLE` without ever reaching `SUCCESS`. **`/plan` publication and the blackboard write are separate asynchronous paths** — the fifth instance of D-38.
- **`RateController` outside the `Fallback`, not inside.** Inside, it returned RUNNING and short-circuited the fallback, so the planner never ticked and every goal after the first failed with error 103.
- **`RateController` at 3 Hz.** Unrate-limited replanning ran at ~12 Hz (median `/plan` gap 0.08 s), producing steering rates to 18.6/s, saturation at 21.1%, and yaw oscillating at twice the commanded frequency. At 1 Hz an invalidated path persisted up to a full second while `FollowPath` drove at the obstruction. 3 Hz is the measured compromise.
- **`Inverter(GoalUpdated)`.** Without it, `IsPathValid` returns SUCCESS on an *empty* path — trivially valid, no poses to check — so planning never runs. The first goal after launch works only because the blackboard key does not yet exist.
- **Bounded planning recovery.** Planning is wrapped in a one-retry
  `RecoveryNode`. A planning failure clears the global costmap once before
  retrying; controller failure is not wrapped and cannot trigger motion
  recovery.
- **No `Spin`, `BackUp`, `DriveOnHeading`, or rotate-in-place.** The vehicle cannot execute them and reverse is unavailable.

### 7.3 Planner

| Parameter | Value |
|---|---|
| Plugin | `SmacPlannerHybrid` |
| `motion_model_for_search` | DUBIN (reverse disabled) |
| **`minimum_turning_radius`** | **0.60 m** |
| `allow_unknown` | false |
| `angle_quantization_bins` | 72 |
| **`smooth_path`** | **false** |
| `cost_penalty` | 2.0 |
| `max_planning_time` | 5.0 s |

**Planner radius exceeds the physical radius deliberately (D-59).** RPP must request *tighter* curvature than the path to correct cross-track error, so planning at the vehicle's exact limit guarantees saturation. At 0.60 m, planned curvature caps at 1.667 m⁻¹ against a 2.124 m⁻¹ vehicle limit — roughly 27% headroom. Measured effect: steering saturation fell from 51.3% to 1.5%, yaw tracking from 0.62 to 0.81.

**The smoother is disabled (D-58).** Smac obeyed `minimum_turning_radius` exactly — `/unsmoothed_plan` max curvature 2.128 m⁻¹ — but the smoother tightened `/plan` to **2.433 m⁻¹ (R = 0.411 m), 15% beyond the vehicle's physical limit.** Verified robust across curvature stencils from 0.15 m to 0.76 m baseline. Post-fix, `/plan` and `/unsmoothed_plan` are identical.

### 7.4 Costmaps

| | Global | Local |
|---|---|---|
| Resolution | **0.050 m** | 0.025 m |
| Size | whole map, non-rolling | 2.00 × 2.00 m rolling |
| Layers | **static + obstacle (default disabled) + inflation** | obstacle + inflation |
| `inflation_radius` | 0.30 m | **0.45 m** |
| `cost_scaling_factor` | 10.0 | 10.0 |

**Global costmap resolution must equal the served map resolution (D-50 amended).** Smac sizes its heuristic lookup table at configure time; if the static layer resizes the costmap afterwards the table is invalidated and valid tight maneuvers are rejected with `NO_VALID_PATH`. Measured: planner configured at 0.025 m against a 0.050 m map produced table size 800 and NO_VALID_PATH, while a matched control configured after the map arrived produced table 400 and planned the same goal in 2.86 ms.

**The global obstacle layer is restored as a runtime opt-in (D-70,
superseding D-62).** It loads disabled by default and uses
`combination_method: 0` (Overwrite), so later live observations can clear
transient marks inside the layer's update bounds. When disabled, planning
retains the former static-map behavior. When enabled, the planner can route
around observed obstacles and a planning failure triggers one
`ClearEntireCostmap` recovery before retry.

The opt-in default preserves the measured-safe baseline: a whole-house
non-rolling obstacle layer previously accumulated lethal cells
**308 → 1078 → 1539 over 112 seconds, monotonically** under motion smear and
pose jumps. Overwrite also has a known low-obstacle hazard: a ray below the
0.1135 m scan plane can clear a static cost inside the observation bounds.
Runtime toggling and clear controls are implemented; physical obstacle
marking and bounded-accumulation validation remain outstanding.

### 7.5 Controller

| Parameter | Value |
|---|---|
| Plugin | `RegulatedPurePursuitController` |
| `desired_linear_vel` | 0.45 m/s |
| `min_approach_linear_velocity` | 0.126 m/s |
| **`regulated_linear_scaling_min_speed`** | **0.15 m/s** |
| `regulated_linear_scaling_min_radius` | 0.75 m |
| **`cost_scaling_dist`** | **0.45 m** |
| `inflation_cost_scaling_factor` | **10.0** (must equal local `cost_scaling_factor`) |
| `use_cost_regulated_linear_velocity_scaling` | true |
| `use_collision_detection` | true |
| **`max_allowed_time_to_collision_up_to_carrot`** | **0.15 s** |
| Goal checker | `SimpleGoalChecker`, xy 0.10 m, **yaw 0.5 rad** |
| Progress checker | `SimpleProgressChecker`, 0.05 m / 10 s |

**Cost regulation must have a floor (D-64).** With `cost_scaling_dist` at 0.45 and no lower bound the vehicle crawled everywhere — median scan range is 1.09 m, so it is within 0.45 m of something almost always. Narrowing the range fixed the crawling but removed slowdown where it was wanted. The correct combination is **wide range plus `regulated_linear_scaling_min_speed`**: it slows near obstacles and never regulates itself to a stop.

**Runner vendors Navigation2 RPP 1.3.12 for path-cost lookahead (D-73).**
Instead of regulating from only the robot-cell cost, the overlay samples the
maximum known cost along the transformed path through the carrot at no more
than half a costmap cell between samples. Unknown samples do not mask known
inflation costs. This is statically and unit tested, but controlled floor
validation against stock 1.3.12 remains outstanding.

**`inflation_cost_scaling_factor` must equal the local costmap's `cost_scaling_factor`.** RPP inverts the inflation curve to recover obstacle distance from cell cost; a mismatch makes the recovered distance wrong regardless of `cost_scaling_dist`.

**Collision detection is a binary abort (D-65).** Jazzy RPP 1.3.12 has no graceful-degradation path — a positive check throws `nav2_core::NoValidControl` immediately rather than slowing or holding. Verified against installed source. The projection horizon is therefore the only lever: **0.15 s**, roughly 2.25 cm at crawl speed and 10.8 cm at 0.72 m/s.

**`SimpleGoalChecker`, not `PositionGoalChecker` (D-67).** The latter ignores orientation entirely, so Smac's final heading-correcting arc was cut off and a waypoint placed facing the opposite direction was reached facing the original way. Yaw tolerance is deliberately loose at 0.5 rad — the vehicle cannot rotate in place and has no reverse, so a tight tolerance risks unreachable goals with no recovery.

### 7.6 Measured performance

Latest validated run:

| Metric | Value |
|---|---|
| Steering saturation | 1.5% |
| Yaw tracking (measured/commanded) | 0.81 |
| Commanded speed p50 (after regulation) | 0.280 m/s |
| Measured speed p50 / max | 0.320 / 0.772 m/s |
| Speed tracking ratio p50 | 1.04 |
| Braking samples | **0** |
| `NO_VALID_PATH` | **0** |

**Zero braking samples** — minimum throttle +0.314. RPP regulates speed smoothly downward and the vehicle coasts to match; nothing requests deceleration. The −0.20 negative bound has never been approached.

---

## 8 · Open items

**Phase 1 closeout**

- **Re-measure the feedforward table** above 0.290 m/s using R1 fixed-throttle mode. The current table is stale and truncated; measured speed exceeds command by up to 24%.
- **Physically validate global obstacle marking and clearing** with a placed
  object, then verify that enabled-layer lethal-cell counts stabilize. The
  layer, Overwrite semantics, runtime controls, and bounded planning recovery
  are implemented but have not completed this floor acceptance.
- **CPU.** Control loop observed at 4.5–5.6 Hz against a 20 Hz target. Prior profiling: ~67% across four cores with a **runnable queue of 10.88** — scheduling latency, not exhaustion. slam_toolbox ~87% of one core, Foxglove bridge ~43%.
- **Floor-validate path-cost lookahead.** Compare the vendored RPP overlay
  against stock 1.3.12 for first speed reduction before the robot cell enters
  inflation, while confirming unchanged open-floor speed.

**Ratified, not implemented**

- ADS1115 traction voltage → `/battery/traction`
- Board revision: PD sink, 2S balanced charger, Pi-controlled high-side FET, brushed H-bridge, IMU, IO breakout
- Telemetry consolidation (Pi + UPS + traction into one node) — parked pending profiling justification

**Deferred with triggers**

- **Reverse autonomy enablement** — trigger: wheels-off-ground MD13S validation and an explicit safe operator-control contract; unlocks Reeds-Shepp, `BackUp` recovery, and known direction
- **IMU scan deskewing** — trigger: Phase 2 speeds
- **Lateral velocity via `v̇y = a_y − vx·ω`** — free, untested
- **Quadrature encoder** — trigger: a consumer of signed wheel velocity
- **UWB** — trigger: observed perceptual aliasing
- **Docking station** — Phase 2/3; charger, UWB anchor, repeatable session origin
- **Grey-box vehicle model** — bicycle nominal plus learned residual; this *is* LMPC's structure

---

## 9 · Decision log

Append-only. D-01…D-57 unchanged (v0.5–v0.9).

| ID | Decision | Reasoning |
|---|---|---|
| D-58 | **Smac path smoothing disabled.** | Smac obeyed `minimum_turning_radius` exactly (`/unsmoothed_plan` max curvature 2.128 m⁻¹) but the smoother tightened `/plan` to 2.433 m⁻¹ — R = 0.411 m, 15% beyond the vehicle's physical limit — with no curvature constraint in its objective. Robust across stencil widths, so not discretization noise. |
| D-59 | **Planner `minimum_turning_radius` set to 0.60 m, above the measured physical 0.470 m.** Adapter `max_steering_angle` remains at the physical 0.3614 rad. | RPP must request tighter curvature than the path to correct cross-track error, so planning at the vehicle's exact limit guarantees saturation. Measured: saturation 51.3% → 1.5%, yaw tracking 0.62 → 0.81. Cost is that some very tight U-turns become unplannable — preferable to planning them and executing them badly. |
| D-60 | **Steering infeasibility clamps at maximum and accepts understeer.** Supersedes D-56. | Brake-on-infeasible was correct for bench validation and wrong in execution — a U-turn was killed by a request only 12% over the curvature limit, followed by a progress-checker abort 22 s later. Infeasible requests are structural, not occasional. |
| D-61 | **Behaviour tree: `Fallback` (not `ReactiveFallback`), `RateController` at 3 Hz outside it, `Inverter(GoalUpdated)` guarding `IsPathValid`.** | Three separately measured defects. `ReactiveFallback` re-evaluates the stale path mid-plan and halts the planner before its success callback writes the blackboard — `/plan` publishes while `FollowPath` gets zero poses. `RateController` inside the fallback returns RUNNING and short-circuits planning entirely. Without `Inverter(GoalUpdated)`, `IsPathValid` returns SUCCESS on an empty path, which is trivially valid. |
| D-62 | **The global costmap carries no obstacle layer.** Dynamic obstacles are handled exclusively by the local costmap and RPP collision detection. | A whole-house non-rolling obstacle layer clears only where a later raytrace passes, so phantom marks from motion smear and pose jumps accumulate: measured 308 → 1078 → 1539 lethal cells in 112 s, monotonically, producing progressive `NO_VALID_PATH` and session-length degradation. The layer was inert for the platform's entire history (`max_obstacle_height: 0.0`) — the confident planning the vehicle exhibited was against the static map alone. Accepted cost: no global routing around unmapped obstacles. |
| D-63 | **Footprint corrected to measured dimensions, `footprint_padding: 0.0`.** | The configured footprint was 0.200 × 0.295 m against a measured 0.165 × 0.290 m — 35 mm too wide, 17.5 mm per side when passing between obstacles. Inscribed radius becomes 0.060 m (governed by rear overhang, not width). Full-lock width of 0.180 m leaves ~7.5 mm/side unmodelled during hard turns, accepted as small against the 0.096 m p95 pose jump already tolerated. |
| D-64 | **Cost regulation uses a wide range with a minimum-speed floor**, not a narrow range. `cost_scaling_dist` 0.45 with `regulated_linear_scaling_min_speed` 0.15. | Wide regulation without a floor made the vehicle crawl everywhere — median scan range is 1.09 m, so it is within 0.45 m of something almost always. Narrowing the range fixed crawling but removed slowdown where it was wanted. The floor is what makes wide regulation usable: it slows in tight quarters and never regulates itself to a stop. |
| D-65 | **RPP collision detection horizon reduced to 0.15 s.** | Jazzy RPP has no graceful-degradation path — a positive collision check throws `NoValidControl` immediately, verified against installed source — so the horizon is the only lever. At 0.5 s and 0.72 m/s it aborted goals on obstructions projected 0.36 m ahead in physically clearable space. |
| D-66 | **Error-dependent integral gain, symmetric.** Ki 0.06 normal, 0.30 when the measured/commanded ratio is below 0.40 or above 1.60, 0.10 hysteresis. | At Ki 0.06 the integrator needs 18 s to traverse its range against a 10 s progress-checker abort. Measured during stalls: throttle 0.530 of a 0.70 ceiling, integrator 0.065 of +0.16 — **neither saturated**, so the ceiling was never the constraint and a prior diagnosis blaming it was wrong. Symmetry matters because the stale feedforward table also produces persistent overspeed. |
| D-67 | **`SimpleGoalChecker` with a deliberately loose 0.5 rad yaw tolerance.** | `PositionGoalChecker` ignores orientation, terminating before Smac's final heading-correcting arc — a waypoint placed facing the opposite direction was reached facing the original way. Tolerance is loose because the vehicle cannot rotate in place and has no reverse, so a tight tolerance risks unreachable goals with no recovery. |
| D-68 | **Keyboard teleop is a laptop-side sender over UDP to a Pi-side bridge that owns all safety properties.** Manual driving requires an explicit `--teleop` flag; route control and autonomy enable are always active. | An SSH terminal delivers a character stream with no key-release events and a ~500 ms repeat delay, so hold-to-run is impossible there. Browser-based control was rejected because `keyup` can be swallowed by focus changes and background tabs are timer-throttled. The Pi measures inter-arrival time itself rather than trusting any sender timestamp, so every failure mode — network drop, laptop sleep, sender crash — looks identical. `pynput` capture is global regardless of focus; accepted deliberately so Foxglove remains usable while driving, with WASD off by default as the mitigation. |
| D-69 | **Typed diagnostic messages alongside human-readable strings.** `AdapterState` on `/drive_adapter/state_typed`, `SystemTelemetry` on `/system/telemetry`. | Semicolon-delimited key=value strings cannot be plotted by Foxglove, which made controller tuning dependent on offline bag analysis. The string topics are retained for logs. |
| D-70 | **Restore the global obstacle layer as default-disabled, with Overwrite semantics and one clear-on-planning-failure retry. Supersedes D-62.** | Default-disabled preserves the measured static-map baseline. Runtime opt-in permits dynamic routing; Overwrite allows later rays to clear transient marks, while the bounded `RecoveryNode` clears accumulated marks only after planning fails. Physical marking, low-obstacle, and accumulation acceptance remain outstanding. |
| D-71 | **Keyboard autonomy is a 600 s Pi-side latch; manual hold-to-run is always available. Supersedes the `--teleop` portion of D-68.** | Intermittent sender loss was stopping routes. Backtick toggles the latch, controller takeover clears it, and Escape repeatedly transmits brake/disarm for at least one second. Protocol v2 command values 0–7 add global-costmap clear/toggle without adding another control channel. |
| D-72 | **Freeze and decay the PI integrator during confirmed teleop preemption.** | Continuing to integrate while `twist_mux` discards autonomy output stores a stale correction for later resume. Fresh `/teleop/active_mode` state freezes accumulation and decays the integral toward zero; stale or missing state preserves prior behavior. |
| D-73 | **Vendor Navigation2 RPP 1.3.12 and regulate from maximum known path cost through the carrot.** | Robot-cell-only cost reacts after entering inflation. Half-cell-or-denser path sampling gives anticipatory slowdown while preserving upstream behavior outside cost regulation. Unit/build validation passes; controlled floor comparison remains outstanding. |
| D-74 | **Separate default WAYPOINT queue and persistent ROUTE workflows share `/runner/waypoint`; the bridge remains the sole Nav2 action owner.** WAYPOINT preserves the full supplied pose in a 20-item, in-memory-only FIFO executed sequentially with `NavigateToPose`; one failure retry is allowed before pausing with the front retained. ROUTE preserves full pose orientation, persistence, controls, and `NavigateThroughPoses`. Absolute idempotent mode commands clear and cancel the opposite collection on actual transitions only. | One Foxglove pose tool serves both operator workflows without a competing action owner or topic. Sequential queue execution intentionally accepts stop-and-go and repeated breakaway. A single generation arbiter prevents stale callbacks from reviving canceled work, while clearing `path` once at the start of both BTs forces fresh planning for identical queue retries, resumed goals, and route replays. Protocol-v2 values 8/9 carry repeated absolute F12 requests without changing the 27-byte layout. |
| D-75 | **`motor_node` is a boot-started persistent systemd hardware owner and is removed from every application composite.** This amends D-29: runnable composites remain complete application graphs but may depend on persistent platform services. It supersedes the ownership portion of D-23: `runner-pwm-setup.service` temporarily prepares export and permissions, while `motor_node` continuously owns PWM12/13. D-09 remains the primary composite-stop path. | Launch teardown and hardware ownership have independent lifecycles. Keeping the motor owner alive lets command publication cease on composite teardown and permits the 200 ms watchdog to apply the existing stop output. `Restart=always` with a 0.1 s delay bounds recovery attempts after owner failure. This decision preserves current ESC behavior; MD13S behavior is not yet implemented or validated. |
| D-76 | **Cytron MD13S replaces hobby-ESC pulse control.** GPIO12 is normal-polarity 20 kHz PWM, GPIO23 is exclusive DIR by `pinctrl-rp1` label, and normalized sign/magnitude map directly to DIR/duty. Duty zero is active brake. D-09 remains 200 ms. Supersedes D-08, D-34, D-46, and D-47 at the motor boundary. | Direct sign-magnitude control removes ESC arming, neutral/deadband/expo/pulse mappings, the mode interlock, and the reverse FSM. Safe startup writes duty zero, defines DIR, then enables motor PWM before subscribing. Direct sysfs access never implicitly unexports GPIO12/13 PWM. Traction-disconnected smoke check only; wheels-off-ground validation remains pending. |

---

## Appendix · Workflow conventions

**Spec discipline.** The spec is a complete standalone reference, not delta-only. Divergent changes require a new decision-log entry.

**Codex:** read-only investigation first, then a separate action prompt against confirmed reality. Codex commits and pushes.

**Executor chats:** provide the repository constraints, platform context, and
task brief. New chat per task. Executors escalate before acting on one-owner
resources, phase scope, or decision-log changes (D-36).

**Validation tiers (D-57).** Match effort to hardware contact: full bench acceptance for anything changing what reaches the motor; smoke check for configuration; unit tests for pure computation. Do not re-verify closed work.

**Measurement:** MCAP bags over comparable routes, analyzed by the same in-repo scripts. **State the analysis window** (D-45). "Feels better" is not a result.

**Standing rules:** `source install/setup.bash` after every build. A runnable application launch is complete on its own, may depend on persistent platform services, and ships with its VS Code task (D-29, D-75). Every new map is built through `/scan_slam` (D-37). A green build is not evidence a node runs. **Verify saturation and bounds before concluding a limit is the cause.**

CAD in Onshape; remote via Tailscale + VS Code Remote-SSH.
