# Navigation Stage A1 — obstacle-evidence coherence

Stage A1 implements Runner specification v1.6 D-89 without changing the
planner, behavior tree, controller, tracker, TF, command ownership, speed, or
reversal behavior. Temporal confirmation, clearing counts, and expiry remain
deferred to A2 until a new CONFIDENT bag measures residual flicker.

## Costmap contract

Both obstacle layers consume `/scan` with the same marking, ray-clearing,
height, minimum/maximum range, update expectation, persistence, and infinity
semantics. Both explicitly use `combination_method: 1` (Max). In the global
costmap this means an obstacle-layer clear cannot reduce occupancy supplied by
the static layer. In both costmaps the live layer is rebuilt from current live
evidence, so ray clearing still removes a live mark after an object moves.

The representations intentionally remain different where their jobs differ:

| Property | Planning | Execution | Reason |
|---|---:|---:|---|
| Frame | `map` | `odom` | Global planning vs stable rolling control |
| Resolution | 0.050 m | 0.025 m | Whole-map cost vs local collision detail |
| Update / publish | 3 / 1 Hz | 10 Hz / 20 Hz threshold | The 20 Hz threshold produces one frame per 10 Hz update; a 10 Hz threshold measured only 6.25 Hz from timer phase/rounding |
| Extent | Static map | 4 m rolling square | Whole route vs derived local window |
| Inflation | 0.30 m | 0.45 m | Both exceed the 0.2444 m circumscribed radius; execution retains its larger existing margin |

The execution window is derived at the highest selectable preset ceiling,
1.50 m/s. The measured braking model is `a(v) = 1.6v + 0.27 m/s²`, command
latency is bounded by the 0.20 s motor watchdog, maximum controller lookahead
is 0.80 m, and the footprint circumscribed radius is 0.2444 m:

```text
a(1.50) = 2.67 m/s²
stopping = 1.50² / (2 × 2.67) + 1.50 × 0.20 = 0.7214 m
required radius = stopping + lookahead + footprint margin
                = 0.7214 + 0.80 + 0.2444 = 1.7658 m
required centred width = 3.5316 m
configured width/height = 4 m (Nav2 declares integer metres; rounded up)
```

## Repeatable evidence

The `navigation_debug` profile now includes both costmap representations,
the local map at its 10 Hz update rate, the RPP collision arc, motor direction,
encoder state, plans, and ROS logs. Analyze any resulting MCAP with:

```bash
source /opt/ros/jazzy/setup.bash
source /home/matti/runner_ws/install/setup.bash
ros2 run runner_bringup analyze_navigation_bag \
  /home/matti/runner_ws/bags/Runner_YYYYMMDD_HHMMSS
```

Add `--json` for a stable machine-readable report. The report includes:

- future-path footprint overlap with static-map lethal cells (the measured
  start pose is deliberately excluded);
- first 0.5 m execution-map lethality for every recorded dispatch;
- action outcomes and 106 classes;
- replans per travelled metre and identical same-pose redispatch storms;
- per-frame lethal mark/clear flicker for each recorded representation; and
- controller-loop rate misses from `/rosout`.

The old reference bag lacks `/global_costmap/costmap`, so planning-map flicker
is truthfully reported unavailable. New A1 bags contain it at 1 Hz. The old
bag's 3 Hz local publication also limits first-tick reconstruction; new A1 bags
record every 10 Hz local update.
