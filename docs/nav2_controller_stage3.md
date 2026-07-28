# Runner Phase 1 Stage 3 Nav2 controller

## Scope and ownership

Stage 3 extends the existing `nav2.launch.py` composite; it does not create a
second Nav2 stack. The launch now contains sensors, estimation, SLAM
localization, the static map server, planner server, controller server,
behavior-tree navigator, Foxglove goal bridge, and one lifecycle manager.

Command ownership remains:

| Owner | Publication | Semantics |
|---|---|---|
| `controller_server` | `/cmd_vel_nav` | `Twist`: m/s and rad/s |
| `drive_adapter` | `/cmd_vel_auto` | normalized throttle/brake and steering |
| `twist_mux` | `/cmd_vel` | normalized throttle/brake and steering |
| `motor_node` | PWM/GPIO only | sole hardware owner |

The Jazzy controller creates relative topic `cmd_vel`. Its launch action
remaps that topic to `/cmd_vel_nav`. Nav2 has no publication or remapping to
`/cmd_vel`.

The lifecycle manager owns `map_server`, `planner_server`,
`controller_server`, and `bt_navigator`, in that order. Runtime activation
confirmed all four bonds and active states. On shutdown, `controller_server`
deactivated its controller and local costmap, cleaned both up, and exited
cleanly.

## Regulated Pure Pursuit

The installed Nav2 version is 1.3.12. The controller class is:

```text
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
```

| Parameter | Value | Meaning |
|---|---:|---|
| `controller_frequency` | 20.0 Hz | Matches the adapter publication rate |
| `desired_linear_vel` | 0.290 m/s | Highest measured sustainable speed |
| `min_approach_linear_velocity` | 0.126 m/s | Lowest measured sustainable speed |
| `lookahead_dist` | 0.40 m | Fixed path-carrot distance |
| `use_velocity_scaled_lookahead_dist` | false | Keeps tuning explicit over the narrow speed range |
| `approach_velocity_scaling_dist` | 0.40 m | Starts goal-approach scaling within one lookahead |
| `regulated_linear_scaling_min_radius` | 0.80 m | Curvature regulation begins above 1.25 m^-1 |
| `regulated_linear_scaling_min_speed` | 0.126 m/s | Does not request a sub-measured-floor turn speed |
| `use_cost_regulated_linear_velocity_scaling` | false | Separates the measured curvature effect from obstacle-cost scaling |
| `use_collision_detection` | true | RPP checks its projected arc against the local costmap |
| `max_allowed_time_to_collision_up_to_carrot` | 0.5 s | Collision projection is capped at the carrot |
| `use_rotate_to_heading` | false | Ackermann vehicle cannot rotate in place |
| `allow_reversing` | false | Phase 1 is forward-only |
| `transform_tolerance` | 0.3 s | Matches the established Nav2 transform tolerance |

RPP collision detection is internal controller safety logic using the local
costmap. It is not the separate Nav2 Collision Monitor feature; Collision
Monitor was not added.

The 0.40 m lookahead is about 2.25 wheelbases and longer than the 0.295 m
vehicle. It remains well below the 1.09 m median valid apartment scan range,
so it does not routinely pursue geometry outside useful local sensor context.

The initial regulated radius was 0.70 m. A no-driving curved goal produced
1.327-1.369 m^-1 without slowing because that radius activates only above
1.429 m^-1. One evidence-based adjustment to 0.80 m makes regulation begin
at 1.25 m^-1. This is 0.235 m above the physical 0.565 m minimum radius and
below the generic 0.90 m plugin default. With 0.80 m configured, observed
1.522-1.556 m^-1 commands slowed from 0.290 m/s to a median 0.234751 m/s.

`nav2_controller::PositionGoalChecker` owns goal completion with a 0.10 m XY
tolerance and ignores final yaw. This avoids a final rotate-in-place request.
The planner still uses the requested goal orientation to form a
kinematically feasible Dubins path.

## Local costmap

The controller-owned costmap is:

| Parameter | Value |
|---|---:|
| global frame | `odom` |
| robot frame | `base_link` |
| rolling window | true |
| width / height | 2 m / 2 m |
| resolution | 0.025 m/cell |
| update / publish frequency | 10 Hz / 5 Hz |
| obstacle source | `/scan` |
| obstacle range | 0.05-1.0 m |
| raytrace range | 0.0-1.2 m |
| inflation radius | 0.30 m |
| inflation cost scaling | 5.0 |

The exact footprint is:

```text
[[0.235, 0.100], [0.235, -0.100], [-0.060, -0.100], [-0.060, 0.100]]
```

At 0.025 m/cell the rolling grid is 80 by 80 cells, which retains context
close to the measured scan median while remaining light on the Raspberry Pi
5. Inflation matches the global costmap deliberately.

Runtime graph inspection showed `/scan` as
`sensor_msgs/msg/LaserScan`, frame `base_laser`, with a direct subscription
from `/local_costmap/local_costmap`. `/scan_slam` had one subscriber,
`slam_toolbox`, and no local-costmap subscription.

## Behavior tree

`behavior_trees/navigate_to_pose_forward_only.xml` contains:

```text
Sequence
  ComputePathToPose
  FollowPath
```

Planning is one-shot for each goal. The tree contains no `Spin`, `BackUp`,
`DriveOnHeading`, recovery fallback, reverse action, or rotate-in-place
motion. Planner and controller failures propagate through their action
status and error-code blackboard entries; they are not converted into an
unsupported recovery maneuver.

The existing bridge remains:

```text
/move_base_simple/goal -> /navigate_to_pose
```

Stage 3 also fixes its bounded shutdown path: if launch SIGINT has already
invalidated the rclpy context, the bridge now skips creation of an executor
on that invalid context and destroys the node directly.

## No-driving validation

The runtime setup started Nav2 and the drive adapter only. The motor node and
mux were not running. The vehicle remained stationary with no `/cmd_vel`
publisher and no controller-to-motor path.

The final post-activation smoke observation, using the permanently installed
Jazzy 1.3.12 packages, lasted 115 seconds:

| Topic | Publishers |
|---|---:|
| `/cmd_vel_nav` | 1, `controller_server` |
| `/cmd_vel_auto` | 1, `drive_adapter` |
| `/cmd_vel` | 0, topic absent in this setup |

There was one controller process and one lifecycle manager. The local costmap
published `/local_costmap/costmap`; because full costmaps are enabled,
`/local_costmap/costmap_updates` had no messages.

The selected `runner_bringup` build passed. The six focused Stage 3
configuration tests and all 18 Foxglove goal-bridge tests passed (24 total);
the remaining affected bringup tests passed with 39 passed, one skipped, and
one deliberately deselected pre-existing lint check. YAML and BT XML parsing,
Python compilation, changed-file Flake8, and `git diff --check` also passed.
The package-wide lint run still reports the pre-existing import-order issue in
`scripts/deadband_diagnostic.py`; Stage 3 does not modify that file.

The command analysis excludes samples with `abs(linear.x) < 0.05 m/s`.
Diagnostic cycle occupancy uses adapter state samples whose preceding
controller command was no more than 0.25 seconds old. A
`steering_infeasible` event is a transition from any other reason into that
reason.

### Goal A: open straight route

Analysis window:
`1785207977.633373-1785207987.671110`, 10.037737 seconds.

- Planner succeeded with a 0.503465 m, 15-pose path.
- Path heading remained approximately -1.925 rad.
- Controller produced 199 commands at 19.83 Hz.
- Linear range: 0.290-0.290 m/s.
- Angular range: -0.001298 to 0.000023 rad/s.
- Curvature range: -0.004475 to 0.000078 m^-1.
- Controller direction matched the nearly straight planned path.
- The stationary test ended with the expected progress-checker abort.

### Goal B: curved route around mapped apartment obstacles

Final regulated analysis window:
`1785208334.406236-1785208344.418544`, 10.012308 seconds.

- Planner succeeded with a 5.948060 m, 138-pose forward-only Dubins route.
- Initial path heading increased from approximately -1.876 rad toward
  -1.103 rad; positive controller yaw followed that direction.
- Controller produced 200 commands at 19.98 Hz.
- Linear range: 0.233000-0.238098 m/s; median 0.234751 m/s.
- Angular range: 0.362500-0.362500 rad/s.
- Curvature range: 1.522481-1.555791 m^-1.
- Maximum curvature remained below 1.771140041436 m^-1.
- The stationary test ended with the expected progress-checker abort.

Using `abs(kappa) < 0.5 m^-1` for straight segments and
`abs(kappa) > 1.0 m^-1` for turns:

| Segment class | Samples | Median speed |
|---|---:|---:|
| low curvature | 199 | 0.290000 m/s |
| high curvature | 200 | 0.234751 m/s |

The observed reduction is 0.055249 m/s, or 19.05 percent.

### Goal C: impossible occupied target

Analysis window:
`1785208124.045375-1785208127.368670`, 3.323295 seconds.

- Target `(0.75, -0.20)` is an occupied map cell.
- Planner timed out and aborted with error code 207.
- No plan or controller command was produced.
- Failure was obstacle/footprint related, not turning-radius-only.

At the physical localization pose, preliminary movement-requiring plans also
failed while a tolerance-only goal succeeded. The vehicle was close to a
boundary with no forward-only Dubins escape compatible with the footprint;
a reversing vehicle could leave that pose, but reverse remains disabled.

### Command feasibility and adapter diagnostics

Across the final reachable-goal windows (20.050045 seconds):

| Measurement | Result |
|---|---:|
| controller samples | 399 |
| maximum linear command | 0.290000 m/s |
| minimum positive command | 0.233000 m/s |
| maximum absolute angular command | 0.362500 rad/s |
| maximum absolute curvature | 1.555791 m^-1 |
| infeasible controller samples | 0 |
| `steering_infeasible` transitions | 0 |
| events per second | 0.000 |
| infeasible diagnostic cycles | 0 / 399, 0.00% |
| adapter floor promotions | 0 / 399, 0.00% |

The adapter reported 16 `stationary_start` breakaway cycles per reachable
goal because the encoder correctly observed a stationary vehicle. These are
not floor promotions.

The Stage 3 evidence does not support changing the adapter's
`steering_infeasible` full-brake policy before Stage 4. Reconsider it only if
future physical path tracking produces non-rare infeasible transitions.

Recorded evidence is in the ignored local bag directories:

```text
bags/stage3_controller_20260727_1957
bags/stage3_controller_goal_b_curvature_20260727_2010
bags/stage3_controller_goal_b_regulated_20260727_2012
```
