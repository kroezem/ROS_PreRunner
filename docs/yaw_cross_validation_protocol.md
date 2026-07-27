# Yaw estimator cross-validation driving protocol

## Purpose and acceptance criteria

This protocol collects a purpose-built MCAP bag for comparing integrated IMU,
RF2O, and EKF yaw with map-referenced SLAM yaw. It is a measurement procedure,
not an estimator diagnosis. Do not use the resulting ratios to change estimator
settings without a separate review.

Each yaw-rate bin should contain at least 100 aligned samples, 5 seconds of
effective common duration, and 0.25 rad of accumulated absolute SLAM yaw.
More duration is preferred in the low-rate control because reference yaw
accumulates slowly. A bin that cannot meet all three thresholds remains useful
context but is marked `insufficient` by the analyzer.

## Preconditions

- Use the normal mapping composite.
- Drive in a known, static, feature-rich mapped environment.
- Confirm localization is initialized and stable before recording.
- Avoid moving people, changing door positions, and large dynamic obstacles
  where practical.
- Confirm the required topics below are publishing.
- Record all topics with MCAP. Prefer `--all-topics` to avoid omissions.
- Choose a clear area and safe throttle limit. Smooth, sustained rates matter
  more than aggressive steering transitions.

Required topics:

- `/imu/data`
- `/odom_rf2o`
- `/odometry/filtered`
- `/tf`
- `/tf_static`
- `/scan`
- `/scan_rf2o`
- `/map`
- `/motor/direction`
- `/wheel/odom`
- `/wheel/encoder_state`

Also include `/initialpose` and any SLAM diagnostic or correction topics present
in the runtime graph. The copy-paste command below records all topics.

In Foxglove, display
`/odometry/filtered.twist.twist.angular.z` as a numeric value and time-series
plot. Its signed value shows direction; its absolute value determines the
analyzer bin. Use it to settle into each target range before counting the
manoeuvre duration.

## Manoeuvres

Perform left and right manoeuvres in the same area where possible. Repeated
circles or broad figure eights provide controlled rotations in both directions
and reduce direction, floor, and feature-layout asymmetry.

| Manoeuvre | Target absolute yaw rate | Target duration | Repetitions | Direction | Reason |
|---|---:|---:|---:|---|---|
| Stationary baseline | near 0 rad/s | at least 10 s | 1 before driving | none | Establish bias and timestamp stability |
| Straight driving | below 0.1 rad/s | full safe pass | at least 2 | alternate travel direction | Populate the near-zero control region |
| Sustained low-rate turns | 0.1–0.3 rad/s | at least 15 s accumulated per direction | as needed | left and right | Key control where scan motion distortion should be small |
| Medium-rate turns | 0.5–1.0 rad/s | at least 15 s accumulated per direction | as needed | left and right | Populate the middle comparison range |
| High-rate turns | 1.2–2.0 rad/s | at least 10 s accumulated per direction | as needed | left and right | Populate the hard-turn range |
| Very-high-rate turns | at least 2.0 rad/s | 5 s accumulated per direction is desirable | multiple short arcs if needed | left and right | Populate the open-ended bin only when safely achievable |
| Circles or figure eights | use the applicable ranges above | enough to meet bin targets | repeated | controlled left and right | Reduce directional and environmental asymmetry |
| Cool-down stationary | near 0 rad/s | at least 10 s | 1 after driving | none | Check post-run bias and timestamp stability |

Do not require unsafe driving merely to fill the very-high-rate bin. Leave it
under-populated and let the analyzer mark it insufficient if the available area
does not permit safe operation. Avoid counting sharp transitions toward the
sustained-duration targets; hold steering and throttle approximately constant.

## Recording

Start the normal mapping composite and establish stable localization first.
Then open a separate shell and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/runner_ws/install/setup.bash
mkdir -p ~/runner_ws/bags
OUTPUT=~/runner_ws/bags/yaw_cross_validation_$(date +%Y%m%d_%H%M%S)
echo "Recording to: $OUTPUT"
ros2 bag record \
  --storage mcap \
  --all-topics \
  --output "$OUTPUT"
```

Complete the manoeuvres, including the final stationary interval. Press
`Ctrl-C` once in the recording shell and wait for `ros2 bag record` to return to
the prompt. Do not power down or copy the bag while MCAP and `metadata.yaml` are
being finalized. Verify finalization with:

```bash
ros2 bag info "$OUTPUT"
```

## Analysis

Analyze the finalized complete recording:

```bash
source /opt/ros/jazzy/setup.bash
source ~/runner_ws/install/setup.bash
python3 ~/runner_ws/tools/analyze_yaw_cross_validation.py \
  "$OUTPUT"
```

To exclude setup or shutdown time, specify bag-relative bounds:

```bash
python3 ~/runner_ws/tools/analyze_yaw_cross_validation.py \
  "$OUTPUT" \
  --start-sec 10 \
  --end-sec 90
```

Multiple bags may be supplied positionally. Each bag is reported separately,
followed by combined bins whose yaw contributions and durations are summed
before ratios are computed:

```bash
python3 ~/runner_ws/tools/analyze_yaw_cross_validation.py \
  ~/runner_ws/bags/yaw_cross_validation_run1 \
  ~/runner_ws/bags/yaw_cross_validation_run2
```

Use `--alignment-sec` to select the fixed comparison grid (default `0.02`) and
`--max-gap-sec` to select the largest gap that may be interpolated or integrated
(default `0.25`). `--format json` emits machine-readable output.

## Analyzer timing and comparison rules

- Requested windows are relative to the first bag receive timestamp. The end is
  clipped to the recovered bag end.
- IMU, RF2O, EKF, and `odom -> base_link` use nonzero message/transform header
  timestamps, with bag receive timestamps only as a reported fallback for zero
  stamps.
- `map -> odom` uses bag receive timestamps. In the reference bag its repeated
  header stamps can carry differing transform values, so those headers do not
  uniquely timestamp publications.
- All timestamps share the recorded ROS/system epoch. The analyzer reports each
  topic's selected timestamp source and fallback count.
- Equal timestamps retain the last received sample. Values are sorted before
  use; no interval with non-positive duration is integrated.
- Scalar rates and planar TF are linearly interpolated onto a fixed grid.
  Transform yaw is unwrapped before interpolation.
- `T_map_base = T_map_odom * T_odom_base`; its composed yaw is unwrapped again
  across the comparison grid.
- Rate integrals use the trapezoidal rule. No interpolation or integration
  crosses the maximum accepted gap.
- Each grid interval is assigned using the absolute mean EKF angular velocity
  at its endpoints. Boundaries are `[0.0, 0.3)`, `[0.3, 1.0)`, `[1.0, 2.0)`,
  and `[2.0, infinity)` rad/s.
- Binned IMU, RF2O, EKF, and SLAM contributions are accumulated only when all
  four sources cover the same grid interval. Missing intervals and gaps are
  shown rather than silently bridged.
- Signed ratios are omitted when signed SLAM yaw is within `1e-6` rad of zero.
  Absolute-magnitude ratios are also reported, without replacing signed ratios.
