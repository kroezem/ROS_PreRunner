# PID live-tuning run: configuration-attributed findings

## Scope and method

- Bag: `/home/matti/runner_ws/bags/pid0_20260801_112638/pid0_20260801_112638_0.mcap` (filename `pid0_20260801_112638_0.mcap`); exact size: 537,194,278 bytes (512.3 MiB).
- D-45 analysis window: +0.000 to +703.380 s bag receive time (703.380 s). The MCAP opening and closing magic and summary were valid.
- The Python `mcap` reader streamed only /drive_adapter/state_typed, /wheel/encoder_state, /speed_envelope/status, /parameter_events. It did not load the 537 MB file as a byte array. Relevant decoded samples were retained for alignment.
- Measurements are partitioned by configuration segment S1...Sn below. A row that spans no single segment is explicitly marked unattributable.
- Steady-state statistics exclude [change, change + 2.0 s) after every desired-speed, Kp, or Ki parameter event.
- Encoder interval estimation adds speed-dependent lag (about 0.137 s at 0.3 m/s and 0.069 s at 0.6 m/s). Rise/response times are not corrected and are therefore inflated more at low speed.

## Recovered controller timeline

| ID | Start (s) | End (s) | Duration (s) | Kp | Ki | desired_linear_vel (m/s) | typed samples |
|---|---:|---:|---:|---:|---:|---:|---:|
| S1 | 0.000 | 185.428 | 185.428 | 0.050 | 0.000 | 0.45 | 3,578 |
| S2 | 185.428 | 253.823 | 68.394 | 0.050 | 0.000 | 0.55 | 1,368 |
| S3 | 253.823 | 349.448 | 95.625 | 0.050 | 0.000 | 0.60 | 1,913 |
| S4 | 349.448 | 362.944 | 13.496 | 0.050 | 0.000 | 0.45 | 270 |
| S5 | 362.944 | 467.531 | 104.587 | 0.030 | 0.000 | 0.45 | 2,091 |
| S6 | 467.531 | 477.588 | 10.057 | 0.050 | 0.000 | 0.45 | 202 |
| S7 | 477.588 | 592.008 | 114.420 | 0.050 | 0.010 | 0.45 | 2,288 |
| S8 | 592.008 | 703.380 | 111.372 | 0.050 | 0.010 | 0.60 | 2,227 |

Gain and observer cross-check findings:

- Direct sample-aligned Kp recovery used 7,079 samples with |speed_error| > 0.01 m/s and found [0.03, 0.05].
- Kp disagreed with the parameter-event timeline on 0 directly observable samples.
- Ki activity was recovered from integrator-state increments on 4,138 aligned updates: median 0.010000, p95 absolute deviation from 0.010000 = 0.000000.
- desired change at +185.428 s: /speed_envelope/status first matched after 1.116 s.
- desired change at +253.823 s: /speed_envelope/status first matched after 2.722 s.
- desired change at +349.448 s: /speed_envelope/status first matched after 1.097 s.
- kp change at +362.944 s: /speed_envelope/status first matched after 1.601 s.
- kp change at +467.531 s: /speed_envelope/status first matched after 1.014 s.
- ki change at +477.588 s: /speed_envelope/status first matched after 1.956 s.
- desired change at +592.008 s: /speed_envelope/status first matched after 2.539 s.
- /speed_envelope/status reported any_divergence=true at least once in segments [2, 3, 4, 5, 6, 7, 8]. This includes intended live departures from the origin and brief stale observations after return-to-origin writes. Its values did not disagree with parameter_events after the sampled observer delays listed above.
- The 1 Hz observer independently reported feedforward coefficient pairs [(0.1188, 0.0174)] across all 703 status samples.
- Parameter events supply the exact write boundaries when Kp/Ki are unobservable because speed_error is zero. Direct state telemetry supplies the active-value check on every observable sample; no metric is assigned across a write boundary.

Feedforward regression by attributed window (model `throttle = slope × |effective_speed| + intercept`):

| Window/configuration | n | slope | intercept | max residual vs 0.1188/0.0174 |
|---|---:|---:|---:|---:|
| S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 2,746 | 0.118800 | 0.017400 | 0.000000000000 |
| S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s) | 1,103 | 0.118800 | 0.017400 | 0.000000000000 |
| S3 (Kp=0.050, Ki=0.000, desired=0.60 m/s) | 1,066 | 0.118800 | 0.017400 | 0.000000000000 |
| S4 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0 | n/a | n/a | n/a |
| S5 (Kp=0.030, Ki=0.000, desired=0.45 m/s) | 1,807 | 0.118800 | 0.017400 | 0.000000000000 |
| S6 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0 | n/a | n/a | n/a |
| S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s) | 2,044 | 0.118800 | 0.017400 | 0.000000000000 |
| S8 (Kp=0.050, Ki=0.010, desired=0.60 m/s) | 2,125 | 0.118800 | 0.017400 | 0.000000000000 |
- Finding: all nonzero effective-speed samples match coefficients 0.1188 and 0.0174 to the residuals above; no coefficient change was observed. S4 and S6 contain no nonzero effective-speed samples, so regression alone cannot attribute coefficients there; the independent 1 Hz telemetry cross-check above covers them.

## Saturation check and per-step responses

- Step definition: stable effective-speed plateaus use ±0.001 m/s tolerance and last at least 1.0 s. A stable-to-stable change of at least 0.02 m/s is a step only when its intervening command transition is at most 1.0 s. This found 80 plateaus and 18 reportable steps; 31 stable-to-stable changes were trajectories/ambiguous (>1 s) and were not assigned step-response metrics.
- Saturation means `final_throttle >= 0.1386` (within 1% of output_max=0.14). The saturation fraction is checked before timing. Any nonzero fraction marks the step saturated and makes t63/t90/settling unusable.
- Settling uses a ±2% target band; for a zero target only, ±2% of step size is used. Response speed is the latest encoder sample aligned by message sample stamp; this avoids AdapterState zeroing measured_speed in brake/silence. Steady error is signed `target - encoder_speed`, averaged over the final 50% of the target plateau after the 2 s change exclusions.
- If OUTPUT_NOT_SELECTED occurs anywhere in a step window, timing is marked unusable because the computed controller output was not continuously selected. The affected fraction is reported separately from actuator saturation. Timing is also unusable if the measured initial speed is more than 10% of step size away from the initial command plateau.

| # | Window/configuration | Initial→target command; measured initial (m/s) | Dir | n | Sat frac | Output-not-selected frac | Timing validity | t63 (s) | t90 (s) | Overshoot | Settle (s) | SS error (m/s; n) | Throttle peak/mean | P peak(abs) | I peak(abs) |
|---:|---|---:|---|---:|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | +15.536..+18.386 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.300→0.000; 0.283 | down | 58 | 0.00% | 0.00% | usable | 0.647 | 0.898 | 0.00% | 0.898 | 0.0000; 24 | 0.0558/0.0085 | 0.0027 | 0.0000 |
| 2 | +18.435..+27.234 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.000→0.300; 0.000 | up | 177 | 0.00% | 0.00% | usable | 0.098 | 0.098 | 31.21% | n/a | 0.0072; 80 | 0.0680/0.0537 | 0.0150 | 0.0000 |
| 3 | +37.994..+63.935 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.300→0.000; 0.286 | down | 520 | 0.00% | 0.00% | usable | 0.139 | 0.239 | 0.00% | 0.239 | 0.0000; 260 | 0.0000/0.0000 | 0.0000 | 0.0000 |
| 4 | +63.983..+71.143 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.000→0.300; 0.000 | up | 144 | 0.00% | 7.64% | **unusable: OUTPUT_NOT_SELECTED** | n/a | n/a | 12.49% | n/a | 0.0212; 72 | 0.0680/0.0567 | 0.0150 | 0.0000 |
| 5 | +76.336..+78.685 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.300→0.450; 0.287 | up | 48 | 0.00% | 0.00% | usable | 0.598 | 0.798 | 6.73% | 2.299 | 0.0083; 21 | 0.0761/0.0708 | 0.0052 | 0.0000 |
| 6 | +123.585..+135.733 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.300→0.000; 0.000 | down | 244 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 0.00% | n/a | 0.0000; 122 | 0.0000/0.0000 | 0.0000 | 0.0000 |
| 7 | +146.883..+148.387 s; S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s) | 0.300→0.250; 0.273 | down | 31 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 98.12% | n/a | 0.0400; 13 | 0.0539/0.0492 | 0.0025 | 0.0000 |
| 8 | +204.483..+215.085 s; S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s) | 0.300→0.000; 0.000 | down | 213 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 0.00% | n/a | 0.0000; 107 | 0.0000/0.0000 | 0.0000 | 0.0000 |
| 9 | +215.136..+219.234 s; S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s) | 0.000→0.300; 0.000 | up | 83 | 0.00% | 98.80% | **unusable: OUTPUT_NOT_SELECTED** | n/a | n/a | 0.00% | n/a | 0.3000; 42 | 0.0680/0.0680 | 0.0150 | 0.0000 |
| 10 | +219.284..+221.192 s; S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s) | 0.300→0.000; 0.000 | down | 39 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 0.00% | n/a | 0.0000; 20 | 0.0000/0.0000 | 0.0000 | 0.0000 |
| 11 | +221.233..+223.635 s; S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s) | 0.000→0.300; 0.000 | up | 49 | 0.00% | 71.43% | **unusable: OUTPUT_NOT_SELECTED** | n/a | n/a | 0.00% | n/a | 0.2193; 25 | 0.0680/0.0660 | 0.0150 | 0.0000 |
| 12 | +388.483..+390.595 s; S5 (Kp=0.030, Ki=0.000, desired=0.45 m/s) | 0.000→0.450; 0.000 | up | 43 | 0.00% | 0.00% | usable | 0.101 | 0.754 | 0.00% | n/a | 0.0162; 18 | 0.0765/0.0719 | 0.0116 | 0.0000 |
| 13 | +452.484..+458.185 s; S5 (Kp=0.030, Ki=0.000, desired=0.45 m/s) | 0.450→0.300; 0.424 | down | 115 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 21.86% | n/a | 0.0106; 50 | 0.0694/0.0540 | 0.0019 | 0.0000 |
| 14 | +487.485..+489.835 s; S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s) | 0.434→0.383; 0.441 | down | 48 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 51.82% | n/a | 0.0038; 19 | 0.0722/0.0647 | 0.0034 | 0.0011 |
| 15 | +489.883..+491.788 s; S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s) | 0.383→0.450; 0.378 | up | 39 | 0.00% | 0.00% | usable | 0.453 | 0.651 | 13.27% | 0.651 | -0.0005; 16 | 0.0741/0.0719 | 0.0032 | 0.0010 |
| 16 | +521.633..+524.034 s; S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s) | 0.300→0.000; 0.310 | down | 49 | 0.00% | 0.00% | usable | 0.250 | 0.360 | 0.00% | 0.360 | 0.0000; 25 | 0.0000/0.0000 | 0.0000 | 0.0011 |
| 17 | +524.083..+527.883 s; S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s) | 0.000→0.300; 0.000 | up | 77 | 0.00% | 0.00% | usable | 0.100 | 0.100 | 19.07% | n/a | 0.0004; 34 | 0.0693/0.0552 | 0.0150 | 0.0018 |
| 18 | +661.333..+664.234 s; S8 (Kp=0.050, Ki=0.010, desired=0.60 m/s) | 0.440→0.600; 0.473 | up | 59 | 0.00% | 0.00% | **unusable: initial not at plateau** | n/a | n/a | 12.57% | n/a | -0.0035; 23 | 0.0949/0.0888 | 0.0050 | 0.0025 |
- Saturation finding: 0 of 18 step transients had any sample within 1% of output_max.

## Integrator behaviour (Ki > 0)

| Window/configuration | n | I min/max/mean | Within 1% of ±0.005 | Longest continuous bound time (s) | Freeze releases | Error sign crossings in 5 s after releases | Max I drift while frozen |
|---|---:|---:|---:|---:|---:|---:|---:|
| S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s); +477.588..+592.008 s | 2,288 | -0.000056/0.002740/0.001128 | 0.00% | 0.000 | 8 | 118 total (max 21) | 0.000000000 |
| S8 (Kp=0.050, Ki=0.010, desired=0.60 m/s); +592.008..+703.380 s | 2,227 | 0.000653/0.003413/0.001776 | 0.00% | 0.000 | 4 | 67 total (max 22) | 0.000000000 |
- Bound finding: the ±0.005 bound was never reached. It was not pinned, so the feedforward-offset interpretation is not triggered.
- Freeze-release finding: the reported error sign crossings are an oscillation-like observation, but do not establish control-loop oscillation because encoder quantization can create crossings. The largest 5 s post-release integrator range was 0.001047; maximum drift during any frozen run was 0.000000000. No frozen-state accumulation (wind-up while frozen) was observed.

Comparable-speed steady-state absolute-error comparison (only Kp=0.05; same exact target plateau; final halves with change exclusions). “Valid” samples have freeze reason ACTIVE (Ki>0) or GAIN_DISABLED (Ki=0), excluding other vehicle/control states:

| Target speed | Ki=0 window; mean abs error (n valid samples) | Ki=0.01 window; mean abs error (n valid samples) | Supported comparison/finding |
|---:|---:|---:|---|
| 0.300 m/s | S1,S2; 0.0259 (159) | S7; 0.0038 (34) | yes; difference Ki=0.01 minus Ki=0 is -0.0221 m/s |
| 0.383 m/s | none; n/a (0) | S7; 0.0093 (19) | no; no matched-speed valid samples in both groups |
| 0.450 m/s | S1; 0.0099 (21) | S7; 0.0024 (16) | yes; difference Ki=0.01 minus Ki=0 is -0.0074 m/s |
| 0.600 m/s | none; n/a (0) | S8; 0.0070 (23) | no; no matched-speed valid samples in both groups |

## Integrator freeze-reason histogram

- D-45 window: full +0.000..+703.380 s, with every count partitioned by S-ID in the last column. Percentages use all typed-state samples.

| Enum name | Count | Percent | Total held duration (s) | Primary modes (counts) | Configuration attribution |
|---|---:|---:|---:|---|---|
| ACTIVE | 4,139 | 29.698% | 206.960 | forward:4139 | S7:2029, S8:2110 |
| GAIN_DISABLED | 6,270 | 44.988% | 313.495 | forward:6270 | S1:2696, S2:743, S3:1045, S5:1786 |
| ZERO_COMMAND | 116 | 0.832% | 5.786 | brake:116 | S1:29, S2:17, S3:20, S5:17, S7:18, S8:15 |
| FEEDBACK_STALE | 0 | 0.000% | 0.000 | not observed | not observed |
| WHEELSPIN | 110 | 0.789% | 5.507 | forward:110 | S1:34, S2:13, S3:18, S5:19, S7:12, S8:14 |
| DIRECTION_UNAVAILABLE | 0 | 0.000% | 0.000 | not observed | not observed |
| DIRECTION_MISMATCH | 0 | 0.000% | 0.000 | not observed | not observed |
| ARBITRATION_UNAVAILABLE | 18 | 0.129% | 0.896 | forward:18 | S1:5, S2:4, S3:3, S5:2, S7:3, S8:1 |
| OUTPUT_NOT_SELECTED | 354 | 2.540% | 17.694 | forward:354 | S1:11, S2:343 |
| INVALID_DT | 0 | 0.000% | 0.000 | not observed | not observed |
| ANTI_WINDUP | 0 | 0.000% | 0.000 | not observed | not observed |
| NO_COMMAND | 2,930 | 21.023% | 146.432 | silence:2930 | S1:803, S2:248, S3:827, S4:270, S5:267, S6:202, S7:226, S8:87 |
| INVALID_COMMAND | 0 | 0.000% | 0.000 | not observed | not observed |
- Mode-partition finding: distribution does not partition by primary mode; at least one mode contains multiple reasons.

Observed ACTIVE and freeze reasons other than GAIN_DISABLED, ZERO_COMMAND, and NO_COMMAND, with vehicle state:

- ACTIVE: 4,139 samples, 206.960 s; mode forward:4139; effective 0.250..0.600 m/s, measured 0.000..0.635 m/s; S7:2029, S8:2110.
- WHEELSPIN: 110 samples, 5.507 s; mode forward:110; effective 0.300..0.495 m/s, measured 0.172..0.483 m/s; S1:34, S2:13, S3:18, S5:19, S7:12, S8:14.
- ARBITRATION_UNAVAILABLE: 18 samples, 0.896 s; mode forward:18; effective 0.300..0.468 m/s, measured 0.000..0.000 m/s; S1:5, S2:4, S3:3, S5:2, S7:3, S8:1.
- OUTPUT_NOT_SELECTED: 354 samples, 17.694 s; mode forward:354; effective 0.300..0.300 m/s, measured 0.000..0.262 m/s; S1:11, S2:343.
- Never observed: FEEDBACK_STALE, DIRECTION_UNAVAILABLE, DIRECTION_MISMATCH, INVALID_DT, ANTI_WINDUP, INVALID_COMMAND.

## Run-wide diagnostics, partitioned by configuration

- D-45 windows are the S-ID intervals. “Active” means typed mode `forward` with nonzero effective_speed; “idle” is every other typed state. Gap statistics are median/p95/max seconds within (not across) each class and segment.
- Encoder matching aligns each typed state to the latest encoder sample by message sample stamp. Both the requested signed formula and the adapter implementation’s absolute-edge-rate formula are shown for all states; active-only absolute matching isolates brake/silence zeroing. Yaw fits use active samples and ordinary least squares with intercept.

| Window/config | Active gaps med/p95/max (s) | Idle gaps med/p95/max (s) | floor viol | effective!=commanded | throttle>0.14 | steering sat | wheelspin | encoder exact signed/abs all; abs active | yaw corr; gain; intercept (n) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| S1 (Kp=0.050, Ki=0.000, desired=0.45 m/s); +0.000..+185.428 s | 0.0500/0.0556/0.1086 | 0.0500/0.0514/0.0571 | 0 | 0 | 0 | 6.62% | 0.95% | 98.91%/98.91% (3,578); 99.96% (2,746) | 0.6609; 0.4293; 0.0271 (2,746) |
| S2 (Kp=0.050, Ki=0.000, desired=0.55 m/s); +185.428..+253.823 s | 0.0500/0.0547/0.0621 | 0.0500/0.0525/0.0586 | 0 | 0 | 0 | 0.15% | 0.95% | 99.63%/99.63% (1,368); 100.00% (1,103) | 0.4874; 0.4790; -0.0086 (1,103) |
| S3 (Kp=0.050, Ki=0.000, desired=0.60 m/s); +253.823..+349.448 s | 0.0500/0.0566/0.0720 | 0.0500/0.0519/0.0592 | 0 | 0 | 0 | 6.33% | 0.94% | 98.38%/98.38% (1,913); 99.91% (1,066) | 0.7416; 0.6508; 0.0228 (1,066) |
| S4 (Kp=0.050, Ki=0.000, desired=0.45 m/s); +349.448..+362.944 s | n/a/n/a/n/a | 0.0500/0.0517/0.0538 | 0 | 0 | 0 | 0.00% | 0.00% | 100.00%/100.00% (270); n/a (0) | n/a; n/a; n/a (0) |
| S5 (Kp=0.030, Ki=0.000, desired=0.45 m/s); +362.944..+467.531 s | 0.0501/0.0559/0.0661 | 0.0500/0.0520/0.0540 | 0 | 0 | 0 | 0.57% | 0.91% | 98.85%/98.85% (2,091); 100.00% (1,807) | 0.8074; 0.8550; 0.0170 (1,807) |
| S6 (Kp=0.050, Ki=0.000, desired=0.45 m/s); +467.531..+477.588 s | n/a/n/a/n/a | 0.0500/0.0519/0.0532 | 0 | 0 | 0 | 0.00% | 0.00% | 100.00%/100.00% (202); n/a (0) | n/a; n/a; n/a (0) |
| S7 (Kp=0.050, Ki=0.010, desired=0.45 m/s); +477.588..+592.008 s | 0.0500/0.0558/0.0724 | 0.0500/0.0526/0.0578 | 0 | 0 | 0 | 2.10% | 0.52% | 98.95%/98.95% (2,288); 99.95% (2,044) | 0.8270; 0.8750; 0.0074 (2,044) |
| S8 (Kp=0.050, Ki=0.010, desired=0.60 m/s); +592.008..+703.380 s | 0.0500/0.0563/0.0679 | 0.0499/0.0519/0.0558 | 0 | 0 | 0 | 4.58% | 0.63% | 99.15%/99.15% (2,227); 100.00% (2,125) | 0.8127; 0.8133; 0.0134 (2,125) |
- Encoder exact-match finding over attributed S1–S8 samples: 98.98% (13,795/13,937) for all states and 99.97% (10,888/10,891) while FollowPath was active. Most all-state mismatches are brake/silence samples where AdapterState zeroes measured_speed while encoder coast-down continues.

## Attribution limits

- Kp is directly observable only where |speed_error| > 0.01. During zero-error gaps, the event boundary and the next direct observation bound attribution; no response metric crosses such a boundary.
- desired_linear_vel is not encoded in AdapterState. Its exact write timeline comes from /parameter_events and is cross-checked against /speed_envelope/status. effective_speed is the sample-aligned command used for response segmentation.
- RPP frequently produces continuously varying effective_speed on curvature. Those intervals do not support step-response claims and are counted as ambiguous above rather than coerced into plateaus.
