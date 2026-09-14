# D2 speed-law reassessment & Advanced Speed Policy UI audit (2026-09-14)

Scope: read-only investigation. No code, parameters, or behavior changed.
Context: physical run `Runner_20260914_182612_0.mcap` (100.1 s, recorded
after `d92554a`) is a successful acceptance run — navigation, committed
route visualization, and rainbow-profile display all work. Remaining
complaint is speed-policy *quality*: too much crawl (0.25 m/s), crawl
triggered by hazards that are not on the future path, and slow recovery
back to higher speed. Task: trace the exact D2 law, find why nearby (not
ahead) hazards can suppress speed, separate the clearance/braking/
acceleration/RPP-residual effects, audit every Advanced Speed Policy UI
control, and propose the smallest fix — no implementation.

## Executive summary

D2's own per-point clearance law is source-confirmed and bag-confirmed to
produce broad, jittery crawl instead of the intended four-tier
(ceiling / 0.5 / 0.25 / 0) behavior, for two independent, additive
reasons: an unrealistically narrow 0.20 m clearance band with no
plateau, and a full-conservative-braking-model reuse for the *acceleration*
pass. Separately, and this is the more likely source of "nearby but
irrelevant hazards suppress speed," stock `nav2_regulated_pure_pursuit`
cost-based regulation (`use_cost_regulated_linear_velocity_scaling`) is
still live, unmodified, and evaluates the robot's **current** pose cost
plus a short lookahead-to-carrot segment every control cycle — independent
of, and stacked underneath, D2's path-relative profile. Finally, none of
D2's actual speed-law parameters are exposed anywhere in Paddock; every
control in "Advanced speed policy" is a pre-D2 RPP/adapter knob that now
either duplicates D2's job under different, current-pose semantics, or is
purely a hard safety backstop.

## CONFIRMED (source-grounded)

**C1. The D2 clearance law is a single 0.20 m linear ramp with no
plateau.** `runner_path_speed_profile.cpp::makeProfile` (path_speed_profile.cpp:300-309)
computes, per path pose, a clearance ceiling that ramps *linearly* from
`creep_speed` (0.25 m/s) at `tight_clearance` (0.15 m clearance beyond the
footprint radius) to `preset_ceiling` at `free_clearance` (0.35 m). There
is no intermediate plateau — a 0.5 m/s "clearly passable" band does not
exist in the law; any value between 0.25 and preset is possible depending
on exact clearance, and small clearance fluctuations map directly to
speed fluctuations. The 0.20 m total band, measured beyond an already
conservative circular `footprint_radius` (0.2444 m plus one grid cell of
padding), is narrow enough that ordinary constrained-but-passable corridors
land close to the `tight_clearance` end and get treated as marginal.

**C2. Clearance is a static per-point distance-transform against a single
costmap snapshot, not a swept corridor and not lookahead-windowed.**
`costmapClearance` (path_speed_profile.cpp:184-244) runs one 2D Euclidean
distance transform over the whole costmap at commit time and samples it at
each path pose's centre point. It has no notion of vehicle heading, footprint
shape (only a circle), or motion direction — it is correct in that it only
measures the clearance of poses that are geometrically close to a hazard,
but it is generated **once per committed path**, not continuously.

**C3. The profile is frozen for the entire lifetime of a committed path;
it is only regenerated on a fresh commit.** `GeneratePathSpeedProfile`
only ticks inside `GenerateValidateAndCommitCandidate` in
`navigate_to_pose_forward_only.xml` (and `..._through_poses_...xml`) —
i.e. only when `RetainCommittedPath` fails (goal change or
`PersistentPathValid` failure) and a brand-new candidate is generated and
accepted. While the committed path holds (the common case, since
`PersistentPathValid` is what keeps commitment sticky), the 3 Hz
`RateController` above it still runs, but the `ReactiveSequence` short-
circuits before `GeneratePathSpeedProfile` and the profile is never
recomputed against the live costmap. **This directly explains "obstacles
beside/behind the vehicle can continue suppressing future profile speed":**
a hazard that was near a downstream path pose at commit time keeps
depressing that pose's ceiling for as long as the path stays committed,
even after the live costmap has since cleared it or the vehicle geometry
has changed relative to it. The bag shows commits roughly every 3–20 s
(21 profile messages over 100 s, non-uniformly spaced), so staleness
windows of several seconds to tens of seconds are routine, not rare.

**C4. Braking (backward pass) and the post-constraint acceleration
(forward pass) reuse the identical stopping-potential model
`a(v) = braking_linear·v + braking_constant`.** path_speed_profile.cpp:355-366,
with an explicit comment acknowledging the reuse ("the same bound makes
acceleration and braking feasibility O(n)"). `braking_linear`/`braking_constant`
(1.6, 0.27) are a deceleration-capability model appropriate for *braking
before* a constraint; using the same curve to bound how fast the ceiling
is allowed to *rise after* a constraint forces the recovery ramp to be
exactly as gradual as the braking ramp into it. If the real vehicle's
achievable forward acceleration is slower than modeled braking (typically
true — braking is a safety-conservative capability, not a design target
for how fast the drivetrain can accelerate), the forward pass is not even
being generous; either way there's no reason the two should share one
number, and no path exists in the current law to make recovery faster than
deceleration without also making the emergency-stop margin less
conservative.

**C5. Independently of D2, stock RPP cost-based and curvature-based
regulation are still live and current-pose-based, and are `min`-combined
with the D2 profile ceiling every control cycle.**
`applyConstraints` (regulated_pure_pursuit_controller.cpp:499-530) computes
`cost_vel` from `heuristics::costConstraint`, driven by `pose_cost` =
`collision_checker_->costAtPose(<robot's current x,y>)` (line 289-290) and
`path_cost` = max cost sampled along the short segment from the robot's
current pose to the current lookahead/carrot point
(`heuristics::costAlongPath`, regulation_functions.hpp:63-123 — this walks
only from the robot's live position to the carrot, not the remaining
committed path). This is then `std::min`'d with `curvature_vel` and, only
after that, with the D2 `profile_ceiling` (line 530). **This is the
"nearby but not future-path-relevant hazard suppresses speed" mechanism**:
`costAtPose`/`costAlongPath` react to whatever is within the inflation
radius of the robot's *current* footprint — including a wall the vehicle
is currently driving past (beside it) or has just passed (behind it, if
still within `cost_scaling_dist`/inflation reach) — regardless of whether
that geometry has any bearing on the path ahead. It is upstream Nav2
behavior, unmodified, and it is not visible in the rainbow profile
Paddock displays (that visualization is the D2 profile only), so an
operator sees a healthy-looking profile while the actually-commanded speed
is additionally throttled by this second, invisible, current-pose layer.

**C6. None of D2's actual speed-law parameters are exposed as ROS
parameters or in Paddock at all.** `GeneratePathSpeedProfile::providedPorts()`
(commitment_nodes.cpp:327-344) declares `creep_speed`, `curvature_window`,
`max_lateral_acceleration`, `tight_clearance`, `free_clearance`,
`footprint_radius`, `braking_linear`, `braking_constant` as BT input ports
with hardcoded defaults; the BT XML never overrides them
(`<GeneratePathSpeedProfile path="{candidate_path}"/>` — no attributes).
They are not declared as node parameters anywhere, so they cannot appear
in `autonomy_tuning.py`'s `PARAMETERS` map, and indeed none of them do.
Every field in Paddock's "Advanced speed policy" panel
(`index.html:172-183`) maps, via `autonomy_tuning.py:PARAMETERS`, to a
pre-existing `/controller_server` (`FollowPath.*`) or `/drive_adapter`
parameter that predates D2. Changing D2's actual speed behavior today
requires a firmware/BT-XML/C++ default change — Paddock cannot touch it.

## Advanced Speed Policy UI audit

| Displayed field | Runtime parameter | Consumer | Live-effective? | Current semantic relevance under D2 |
|---|---|---|---|---|
| Nominal speed | `FollowPath.desired_linear_vel` | RPP base `linear_vel`; also read by `GeneratePathSpeedProfile` as D2's `preset_ceiling` (commitment_nodes.cpp:365-378) | Yes | Still the single source of D2's ceiling — correct, but the UI gives no hint this field is doing double duty as "D2's open-corridor speed." |
| Absolute ceiling (`maximum_commanded_speed`) | `/drive_adapter.maximum_commanded_speed` | Adapter output clamp, downstream of everything else | Yes | Legitimate independent hard cap; unrelated to D2, not misleading. |
| Regulated minimum | `FollowPath.regulated_linear_scaling_min_speed` | Floor applied to `min(cost_vel, curvature_vel)` **before** the D2 profile is applied (line 516-518) | Yes | Superseded in intent: D2 can and does command below this floor, even to 0 (comment at line 520-521 acknowledges this explicitly). The field still functions but its old meaning ("never go below this while navigating") no longer holds once D2 is in the loop — misleading label under D2. |
| Cost slowdown distance / gain (`cost_scaling_dist`, `cost_scaling_gain`) | `FollowPath.cost_scaling_*` | `heuristics::costConstraint` on **current-pose** cost (C5) | Yes | This is the live, current-pose regulation that duplicates D2's job on a different (costmap-cost, not distance-transform-clearance) basis and different (current position, not committed-path arclength) reference frame. Not superseded — still actively firing and, per C5, likely a major contributor to the reported behavior — but its relationship to D2 is invisible to the operator and its units/semantics don't match D2's clearance-based tiers at all. |
| Curvature radius (`regulated_linear_scaling_min_radius`) | `FollowPath.regulated_linear_scaling_min_radius` | `heuristics::curvatureConstraint`, current lookahead curvature | Yes | Independent of D2's own curvature-window ceiling (C1's curvature term, ceiling.cpp:274-299, computed once at commit time from the committed path's actual poses). Two separate curvature-based speed limits now exist: one live/lookahead-based (this control) and one committed-path/window-based (D2, not exposed). Redundant, not obviously harmful, but not coordinated. |
| Min/max lookahead, lookahead time | `FollowPath.min/max_lookahead_dist`, `lookahead_time` | Carrot-point selection | Yes | Unrelated to speed shaping directly; unaffected by D2. |
| Collision horizon (`max_allowed_time_to_collision_up_to_carrot`) | `FollowPath.max_allowed_time_to_collision_up_to_carrot` | `collision_checker_->isCollisionImminent` — binary trip, evaluated after linear_vel (incl. D2) is already set (line ~318-321) | Yes | Legitimate final safety backstop on the already-shaped command; not a speed-shaping control and not superseded by D2 — correctly orthogonal. |

**Bottom line of the audit:** every exposed control is still live, but
none of them are D2's controls. Two of them (`cost_scaling_dist`/`gain`
and `regulated_linear_scaling_min_speed`) are actively fighting or
overriding D2's intent from underneath, on a current-pose basis, while
looking to the operator like general "speed policy" knobs equivalent to
what the rainbow profile shows.

## HYPOTHESES (plausible, not directly proven from source alone)

**H1. The sawtooth pattern in committed profiles (bag-observed, see below)
is clearance-transform noise, not real geometry oscillation.** Because
clearance is sampled at discrete path-pose centres with no spatial
smoothing beyond the single curvature window, small changes in nearest-
obstacle-cell along a corridor (grid quantization, corridor not perfectly
straight) plausibly produce a ceiling that oscillates pose-to-pose inside
the 0.15–0.35 m band, rather than tracking the coarser "is this segment
passable" judgement a human would make.

**H2. Extended goal-approach crawl compounds with clearance crawl.** The
long flat 0.25 m/s run immediately before the goal seen in the bag (see
below) is consistent with the vehicle's final approach geometry being
close to walls (parking-style corridor) *and* separately being within
`approachVelocityConstraint`'s scaling distance — the two effects are not
distinguishable from the profile message alone without also correlating
`min_approach_linear_velocity`/`approach_velocity_scaling_dist`.

## Bag correlation (`Runner_20260914_182612_0.mcap`)

21 `/navigation/path_speed_profile` messages over the 100 s run (commits
roughly every 3–20 s, confirming C3's staleness window is routine).
Per-point tier counts (creep ≤0.26 m/s / mid / at-or-near preset) across
those 21 commits show crawl frequently dominating a committed path, not
just bracketing genuine pinch points, e.g.:

- commit at t≈59.8 s: 24 of 27 points at crawl, 0 at mid, 0 at/near preset
  — essentially the entire committed path judged "marginal."
- commit at t≈0 s (5th profile): 40 of 52 points at crawl on a 1.0 m/s
  preset path.

Sampled raw point sequences confirm both quality problems directly:

- **Sawtooth, not plateau** (commit #7): after an initial 1.36 m crawl
  run, the ceiling does *not* settle at a caution speed — it oscillates
  0.33 → 0.70 → 0.33 → 0.47 → 0.33 → 0.47 m/s across a 2 m stretch, never
  spending more than ~0.3 m at any one value. There is no ~0.5 m/s band in
  the data; "mid" values are scattered continuously between 0.28 and 0.70.
- **Long terminal crawl** (commit #17): from s=4.28 m to the s=5.65 m goal
  (1.37 m, over a quarter of that path's length), the ceiling sits flat at
  exactly 0.25 m/s, right after a cusp/stop 0.5 m earlier — i.e. a cusp,
  ~0.5 m of partial recovery attempt (peaks at 0.54 m/s), then a second
  cusp, then crawl the rest of the way in.

This is direct, bag-quantified support for "crawl used too broadly" (C1
mechanism) and for "insufficient path-ahead behavior reading as noisy,
not smooth" (H1). Confirming the specific slow-recovery timing (C4)
against actual commanded `/cmd_vel_nav` on a per-metre basis was not done
in this pass — flagged as an open question below rather than asserted
numerically.

## OPEN QUESTIONS

1. Does the sawtooth pattern (H1) correlate with local costmap noise/decay
   at the moment of the specific commits sampled, or with genuine
   micro-geometry (e.g. door frames, table legs) in this bag's environment?
   Needs overlay of the committed path against `/global_costmap/costmap`
   at each commit's timestamp — not done in this pass.
2. How much of the terminal 1.37 m crawl (commit #17) is clearance-driven
   vs. `approach_velocity_scaling_dist`-driven? Requires reading
   `FollowPath.approach_velocity_scaling_dist`/`min_approach_linear_velocity`
   live values for this run, not captured in the profile message.
3. Quantitatively, how much of total path time in this bag is spent at
   `cost_vel < profile_ceiling` (i.e., C5's live layer actually binding
   below D2's committed intent)? Would require joining `/cmd_vel_nav`
   against reconstructed `profile_ceiling` per timestamp; not done here.
4. Is `footprint_radius` (0.2444 m, circular) a reasonable conservative
   stand-in for the real chassis, or is it already wider than necessary in
   a way that eats into the 0.20 m clearance band from C1?

## Recommended D2 speed-law / UI contract (smallest change)

Do **not** touch planner acceptance, Stage C, or the shared braking model's
role in braking itself. Proposed changes, each independently adoptable:

**D1. Replace the single linear clearance ramp with an explicit four-tier
step law plus hysteresis**, in `makeProfile`'s clearance block
(path_speed_profile.cpp:300-309):
  - `clear ≥ open_clearance` → `preset_ceiling`.
  - `passable_clearance ≤ clear < open_clearance` → fixed `caution_speed`
    (hypothesis default 0.5 m/s, independent of `preset_ceiling`, not
    interpolated).
  - `tight_clearance ≤ clear < passable_clearance` → `creep_speed`
    (0.25 m/s).
  - `clear < tight_clearance` → treat as the existing cusp/marginal path
    (0, or defer to path validity — out of scope here).
  - Apply a wider tier band than today's 0.20 m total (hypothesis: widen
    `tight_clearance`→`passable_clearance`→`open_clearance` to something
    like 0.10 / 0.25 / 0.45 m beyond footprint, to be tuned physically,
    not asserted numerically here) and require a minimum run-length
    (hysteresis, e.g. don't leave a tier for <0.3 m of arclength) to kill
    the sawtooth (H1) directly, independent of whatever is causing the
    underlying clearance noise.

**D2. Split the reachability model (C4) into two configs: a braking model
(unchanged, used only in the backward pass) and a separate, faster
acceleration-recovery model (used only in the forward pass).** The
forward pass should be allowed to ramp up as fast as the vehicle can
actually accelerate, decoupled entirely from the conservative stopping
model — this is the direct fix for "recovery to higher speed feels too
slow" (C4) and requires no change to the braking guarantee.

**D3. Make the profile arclength-window-aware of the vehicle's live
position, not just commit-time-frozen (C3).** Smallest version: re-tick
`GeneratePathSpeedProfile` (or a lighter-weight re-clearance-only step)
on a bounded timer even while the committed path is retained, so a
cleared-behind hazard's suppression on not-yet-reached poses expires
within one or two costmap update cycles instead of persisting for the
full commitment lifetime. This does not touch `PersistentPathValid` or
commitment logic — only the profile values sampled from the still-valid
committed path.

**D4. Decide explicitly whether `cost_scaling_dist`/`cost_scaling_gain`
(C5) should keep running underneath D2, and if so, rename/re-document
them in Paddock as a distinct "live proximity backstop" rather than
grouping them under the same "Advanced speed policy" panel as D2's intent**
— because right now they are the most likely source of "hazards beside/
behind the vehicle suppress speed that shouldn't be suppressed," and nothing
in the UI tells the operator that this layer exists or that it operates on
current position rather than the committed path.

**D5. Expose D2's own parameters (`creep_speed` overlap already covered by
existing UI field of the same name is a coincidence, not a wiring —
`tight_clearance`, `free_clearance`/new tiers from D1, `footprint_radius`,
and the split braking/acceleration constants from D2) as real ROS
parameters on the BT node or a small dedicated node, and add them to
`autonomy_tuning.py`/Paddock**, so future tuning of the actual speed law
does not require a firmware rebuild — this is an audit finding (C6), not
a numeric recommendation, and is the precondition for tuning D1/D2 without
redeploying.

No numeric defaults above should be taken as final — `caution_speed`,
the widened clearance band, and the acceleration-model constants are
named as placeholders for Matti's physical tuning pass, not adopted
values.
