# Runner predictive cost regulation fork

This package is vendored from Navigation2 tag `1.3.12`, commit
`6be3614013ec586051b86c97b919b293281490fe`. It intentionally has the same
package and plugin names as the Jazzy binary package so a normal colcon
workspace overlay takes precedence after `install/setup.bash` is sourced.
Build it explicitly as an override:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_regulated_pure_pursuit_controller \
  --allow-overriding nav2_regulated_pure_pursuit_controller
source install/setup.bash
```

The fork changes cost-regulated velocity scaling from the cost at the robot
pose to the maximum known cost sampled along the transformed path from the
robot through the controller's lookahead point. Samples are no farther apart
than half a costmap cell. `NO_INFORMATION` samples are ignored, matching the
upstream cost constraint's behavior for unknown cost while ensuring unknown
does not mask a known inflation cost elsewhere in the window.

The maximum is deliberate. It makes the existing cost-to-speed heuristic react
as soon as an inflation cost enters the lookahead window. A distance-weighted
cost would produce a smoother ramp but attenuate the forward cost and delay
the requested anticipatory slowdown. A path containing only free or low costs
still passes the same cost class into the upstream constraint and therefore
does not reduce open-floor speed.

Files diverging from upstream 1.3.12 are:

- `include/nav2_regulated_pure_pursuit_controller/regulation_functions.hpp`
- `src/regulated_pure_pursuit_controller.cpp`
- `test/CMakeLists.txt`
- `test/test_path_cost.cpp` (new)
- `README.runner.md` (new)

For each Jazzy Navigation2 update, compare the new upstream package with tag
`1.3.12`, rebase or reapply these changes, and rerun the package tests plus
floor validation. In particular, inspect changes to `computeVelocityCommands`,
`getLookAheadPoint`, `applyConstraints`, `costConstraint`, transformed-path
frame semantics, and costmap APIs. Do not assume an updated Debian package
changes this overlay: the sourced workspace package continues to win until
the vendored source is updated and rebuilt.

## Floor validation

Bench tests do not satisfy the physical acceptance criterion. With the motor
path enabled in a controlled test area, record `/cmd_vel_nav`,
`/local_costmap/costmap`, `/plan`, `/tf`, and `/tf_static` while driving a
straight or gently curved path toward a repeatable inflated complex region.
Use an open-floor run at the same `desired_linear_vel` as the control.

For each command timestamp, derive the robot's distance to the first
non-free inflation cell on the followed path. The fork passes when:

1. the open-floor command remains at `desired_linear_vel` outside curvature
   and goal-approach regulation;
2. the approach run has a repeatable, measurable command reduction while that
   first inflation cell is still ahead of the robot; and
3. the first reduction is not deferred until the robot's own costmap cell
   becomes inflated.

Report the chosen inflation-entry cost threshold, first speed reduction,
distance to inflation at that instant, and equivalent values from a stock
1.3.12 run. Repeating each run at least three times separates the controller
effect from costmap timing and localization noise.
