# Runner on-ground throttle characterization protocol

This protocol creates the Stage B measurement bag. It does not actuate the
car automatically and does not define a controller or Nav2 mapping. Matti
drives manually. The analyzer only reads the resulting MCAP bag.

## Known command semantics

`/cmd_vel_teleop.linear.x` and mux output `/cmd_vel.linear.x` are normalized
ESC commands, not metres per second.
`runner_teleop/teleop_node.py` maps R2 to `0.0` through `+1.0` and L2 to
`0.0` through `-1.0` while X is held. With no motion button held, it
continuously publishes the race-mode full-brake command `-1.0`.
`runner_motor/motor_node.py` clamps the field to `[-1.0, +1.0]`: positive is
forward, values below `-0.05` request brake, and values from `-0.05` through
zero produce the 1500 us neutral pulse. Zero is neutral/coast at this layer,
not an active brake.

The ESC conversion is in `motor_node.py`, after `/cmd_vel`: the first 0.05
of positive magnitude ramps from 1500 to the configured 1550 us forward
onset, then an exponent-2 curve reaches the 1750 us forward limit. Reverse
magnitude uses the same crossover and exponent toward the 1250 us
brake/reverse limit. Those software values do **not** establish the physical
ground-motion deadband; this protocol measures it. The published normalized
values in the ladders below are therefore valid, but physical motion remains
unknown until the test.

## Operator controls

- **X:** hold for normal manual drive. R2 supplies positive throttle.
- **R1:** hold for fixed-throttle drive. R2 does not change its positive
  throttle.
- **L1:** `teleop_suppress`: teleop stops publishing `/cmd_vel_teleop` while
  L1 alone is held. L1 by itself is not an autonomy arming gate and does not
  make autonomy safe or active.
- **D-pad up/down:** raise/lower the process-lifetime fixed-throttle setpoint
  by one configured step per press.
- **L2:** proportional brake in both X and R1 modes; it overrides positive
  throttle.
- **Left stick:** steering remains live in X and R1 modes.
- **Release all motion buttons:** full brake within the next 50 ms publication
  cycle.

The priority is:

```text
X > R1 > L1 > brake
```

| X | R1 | L1 | Effective behavior |
|---|---|---|---|
| released | released | released | Full brake |
| held | any | any | Normal manual control |
| released | held | any | Fixed-throttle control, unless inhibited |
| released | released | held | Teleop publishes nothing |

If X takes over while R1 is held, releasing X does not resume fixed throttle.
Teleop reports `fixed_throttle_inhibited` and publishes full brake until R1
is released. R1 must then be pressed again to request fixed throttle.

The production defaults are an initial setpoint of `0.30`, step `0.01`,
minimum `0.00`, and maximum `0.50`. The selected setpoint persists only for
the teleop process lifetime and resets to `0.30` at every process start unless
the production launch explicitly overrides the initial parameter. Every R1
press logs the selected setpoint and expected race-mode ESC pulse. That pulse
is calculated from the configured software mapping; it is not measured
feedback.

`/teleop/fixed_throttle_setpoint` (`std_msgs/msg/Float32`) continuously
publishes the selected setpoint. `/teleop/active_mode`
(`std_msgs/msg/String`) continuously publishes one of `brake`, `manual`,
`fixed_throttle`, `teleop_suppress`, or `fixed_throttle_inhibited`.

The Stage 2 mux falls through to autonomy after its 0.15 s teleop input
timeout while L1 suppression is held. Human teleop preemption is immediate,
while autonomy engagement is delayed by that timeout and the next autonomy
publication. The independent motor watchdog remains the final safety
mechanism.

## Area, vehicle, and safety

- Use a large, clear indoor or outdoor straight-line area. Keep enough clear
  distance to stop at the highest tested speed.
- Keep the wheels on the ground. A free-spinning stand is not valid for
  throttle-to-ground-speed mapping.
- Use one stated surface for the entire bag. Record exactly one of hardwood,
  carpet, asphalt, concrete, or another named surface. Do not mix surfaces.
- Keep tires, tire pressure/setup, and vehicle mass unchanged for the session.
- Begin with a normally charged traction pack.
- Hold steering straight and keep people, animals, and obstacles outside the
  test lane. Reverse runs require extra clearance.
- Abort if steering, motor, encoder, localization, `/cmd_vel`, or scan data is
  unhealthy, or if the vehicle response is unexpected.
- Matti operates the controls and retains the dead-man/stop responsibility.
  Neither the recording command nor analyzer actuates the vehicle.

## Existing launch mode and checks

Use the existing complete mapping entry point:

```bash
source /opt/ros/jazzy/setup.bash
source /home/matti/runner_ws/install/setup.bash
ros2 launch runner_bringup map.launch.py
```

`map.launch.py` is the existing composite that includes sensors, estimation,
SLAM mapping, and the mutually exclusive teleop entry point. Together these
provide teleop, motor, encoder, EKF, TF, and scan without adding another
resource owner. Run exactly this one top-level launch; do not also start
`teleop.launch.py` or `localize.launch.py`.

In another sourced terminal, check:

```bash
ros2 topic hz /wheel/encoder_state
ros2 topic hz /wheel/odom
ros2 topic hz /odometry/filtered
ros2 topic hz /scan
ros2 topic info /cmd_vel --verbose
```

Expect encoder state and wheel odometry near 20 Hz, EKF near its configured
publication rate (typically about 50 Hz), and scan near the sensor rate.
Confirm `/cmd_vel` has the intended single publisher and motor subscriber.
Also observe that encoder `stationary=true` while physically stopped. Abort
instead of recording if a required topic is absent or unhealthy.

## Required recording

The bag must contain all eight topics:

| Topic | Why it is required |
|---|---|
| `/cmd_vel` | Source normalized command and constant-run boundaries. |
| `/teleop/fixed_throttle_setpoint` | Selected repeatable positive command. |
| `/teleop/active_mode` | Command provenance and suppression state. |
| `/wheel/encoder_state` | Stationary status and unsigned edge-rate cross-check. |
| `/wheel/odom` | Primary signed encoder-derived velocity. |
| `/odometry/filtered` | EKF signed velocity comparison. |
| `/tf` | Transform coverage and later forensic checks. |
| `/scan` | Localization sensor coverage and later forensic checks. |

Recording all topics is acceptable if that is operationally simpler, but
verify these exact eight afterward. Use MCAP. Do not enable shell nounset before
sourcing ROS; the Jazzy setup script is not nounset-safe in this environment.

```bash
source /opt/ros/jazzy/setup.bash
source /home/matti/runner_ws/install/setup.bash
mkdir -p /home/matti/runner_ws/bags
OUTPUT="/home/matti/runner_ws/bags/throttle_characterization_$(date +%Y%m%d_%H%M%S)"
ros2 bag record \
  --storage mcap \
  --output "$OUTPUT" \
  /cmd_vel \
  /teleop/fixed_throttle_setpoint \
  /teleop/active_mode \
  /wheel/encoder_state \
  /wheel/odom \
  /odometry/filtered \
  /tf \
  /scan
```

Start recording before the first run and stop it after all end repeats. Then:

```bash
ros2 bag info "$OUTPUT"
```

Explicitly verify that all eight required topics appear and that their message
counts are plausible for the recorded duration.

## Timing and segment execution

Use these analyzer-matched defaults:

- command tolerance: `0.005` normalized command
- minimum held-throttle segment: `6.0 s`
- settling period discarded from the segment head: `2.0 s`
- minimum retained steady-state tail: `4.0 s`
- zero-command tolerance: `0.005` normalized command
- sustained-motion speed threshold: `0.03 m/s`

The 2.0 s discard is a conservative initial value, not a measured vehicle
time constant. It should allow the car to settle before the retained tail.
Four retained seconds yield roughly 80 encoder publications at 20 Hz and
about 200 EKF publications near 50 Hz while remaining practical in a limited
straight test area.

For every level:

1. Begin fully stopped.
2. Ensure the bag is already recording.
3. Hold steering straight.
4. Apply the target normalized throttle without feathering.
5. Hold it for at least 6.0 seconds.
6. Release to neutral (`/cmd_vel.linear.x` near zero).
7. Wait for `/wheel/encoder_state.stationary=true` **and** physical stop.
8. Leave at least 2.0 seconds of zero command between segments.
9. Record the target and actual hold in the run sheet.

If a run is interrupted or the command cannot be held within `0.005`, repeat
it as a new run; do not try to repair it by hand. A joystick display or
`ros2 topic echo /cmd_vel` may be used to verify the exact published value,
but keep attention on safe driving.

## Pass 1: physical deadband discovery

Run forward and reverse separately, stopping fully between every level:

```text
0.00, 0.02, 0.04, 0.06, 0.08, 0.10,
0.12, 0.14, 0.16, 0.18, 0.20
```

Use the listed positive values for forward and their exact negatives for
reverse. Hold every nonzero level for at least 6.0 seconds. The `-0.02` and
`-0.04` commands are expected to remain neutral in the current motor layer;
recording them verifies the command-path boundary and must not be mistaken for
an ESC ground-motion result.

If motion starts, continue through at least three ladder levels above the
first sustained motion, subject to safe speed. If it has not started by
magnitude 0.20, continue in exact 0.02 normalized increments until it does,
stopping if speed or motor behavior becomes unsafe. Do not assume a deadband
before measuring it.

## Pass 2: operating range

From Pass 1, call the lowest tested magnitude with sustained motion `T`.
Before driving Pass 2, calculate and write the exact numeric commands in the
run sheet:

```text
T, T+0.02, T+0.04, T+0.07, T+0.10, T+0.15, T+0.22,
one conservatively selected level approaching 1.5 m/s
```

Clamp every result to `[0.00, 1.00]`; use positives forward and exact
negatives reverse. This calculation must result in about eight explicit
numeric forward levels and eight explicit numeric reverse levels. Do not
reuse forward `T` for reverse: determine each direction's `T` independently.

The final level is adjusted conservatively over stopped, discrete attempts;
the first session need not hit exactly 1.5 m/s. Never increase it merely to
complete the table if the available lane or response is unsafe.

## Reverse behavior

Characterize reverse independently and retain the negative published signs.
The ESC may require its existing brake/re-arm behavior:

- come to a complete physical and encoder-confirmed stop before each reverse
  run;
- follow the existing manual ESC behavior;
- do not automate, bypass, or redesign the handshake;
- discard and repeat a segment if a negative command was held but reverse did
  not actually engage.

The analyzer never combines forward and reverse into one curve.

## End repeats and session-drift proxy

At the end of the full session, repeat the lowest three **moving** throttle
levels in the same direction and on the same unchanged surface. Preferably do
this for both directions. Use run-sheet labels such as:

```text
forward_low_1_start ... forward_low_3_start
forward_low_1_end   ... forward_low_3_end
reverse_low_1_start ... reverse_low_3_start
reverse_low_1_end   ... reverse_low_3_end
```

The analyzer identifies repeats from command magnitude and time, not these
manual labels. Because traction voltage is not recorded, the change in speed
at matched throttle is only a **pack-droop proxy/session-drift measurement**,
not pack voltage drop. Motor/pack temperature, tire heating, surface changes,
and localization changes are confounders.

## Run sheet

Fill one row per attempted segment and mark repeats rather than overwriting
failed attempts.

| Seq. | Direction | Target throttle | Intended label | Actual hold (s) | Surface | Notes | Repeated at end? |
|---:|---|---:|---|---:|---|---|---|
| 1 | forward/reverse |  |  |  |  |  | yes/no |
| 2 | forward/reverse |  |  |  |  |  | yes/no |
| 3 | forward/reverse |  |  |  |  |  | yes/no |
| … |  |  |  |  |  |  |  |

## Offline analysis

After copying the bag to this repository:

```bash
source /opt/ros/jazzy/setup.bash
source /home/matti/runner_ws/install/setup.bash
python3 /home/matti/runner_ws/tools/analyze_throttle_response.py \
  "$OUTPUT" \
  --format text
python3 /home/matti/runner_ws/tools/analyze_throttle_response.py \
  "$OUTPUT" \
  --format json > throttle_response.json
python3 -m json.tool throttle_response.json >/dev/null
```

Use `--window-start` and `--window-end` only to exclude known setup or
post-run time. The defaults in this protocol and analyzer are identical.
Do not interpret a non-protocol bag as actual throttle characterization.
