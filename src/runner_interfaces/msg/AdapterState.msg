# Typed drive-adapter diagnostics for live plotting.
# measured_yaw_rate comes from /odometry/filtered.twist.twist.angular.z;
# steering remains open-loop, so this is its first measured response signal.
uint8 INTEGRATOR_ACTIVE=0
uint8 INTEGRATOR_GAIN_DISABLED=1
uint8 INTEGRATOR_ZERO_COMMAND=2
uint8 INTEGRATOR_FEEDBACK_STALE=3
uint8 INTEGRATOR_WHEELSPIN=4
uint8 INTEGRATOR_DIRECTION_UNAVAILABLE=5
uint8 INTEGRATOR_DIRECTION_MISMATCH=6
uint8 INTEGRATOR_ARBITRATION_UNAVAILABLE=7
uint8 INTEGRATOR_OUTPUT_NOT_SELECTED=8
uint8 INTEGRATOR_INVALID_DT=9
uint8 INTEGRATOR_ANTI_WINDUP=10
uint8 INTEGRATOR_NO_COMMAND=11
uint8 INTEGRATOR_INVALID_COMMAND=12
# When conditions coincide, integrator_freeze_reason reports the first
# applicable reason in this highest-to-lowest precedence:
# NO_COMMAND, INVALID_COMMAND, ZERO_COMMAND, FEEDBACK_STALE, WHEELSPIN,
# DIRECTION_UNAVAILABLE, DIRECTION_MISMATCH, ARBITRATION_UNAVAILABLE,
# OUTPUT_NOT_SELECTED, INVALID_DT, ANTI_WINDUP, GAIN_DISABLED.
# INTEGRATOR_ACTIVE applies only when none is applicable. GAIN_DISABLED is the
# final fallback even when integral_gain is zero; Ki=0 does not mask telemetry.

builtin_interfaces/Time stamp
# Raw longitudinal SI request received from /cmd_vel_nav. This remains
# unchanged when adapter floor or ceiling logic intervenes.
float64 commanded_speed
# Magnitude-domain speed presented to feedforward and feedback control after
# acceptance: zero, or within [minimum_moving_speed, maximum_commanded_speed].
float64 effective_speed
float64 measured_speed
# abs(effective_speed) - abs(measured_speed)
float64 speed_error
float64 feedforward_throttle
# True only when a computed nonzero-command feedforward is below 0.04. This is
# evidence that the configured command-floor path was bypassed; it never clamps.
bool feedforward_floor_violation
float64 proportional_term
float64 integrator_state
float64 pi_term
float64 final_throttle
float64 commanded_yaw_rate
float64 measured_yaw_rate
float64 normalized_steering
float64 steering_curvature_requested
float64 steering_curvature_max
bool integrator_enabled
bool steering_saturated
bool wheelspin_guard
uint8 integrator_freeze_reason
string mode
