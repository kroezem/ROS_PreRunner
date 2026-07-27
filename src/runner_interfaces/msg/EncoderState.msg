# ROS timestamp shared with the corresponding /wheel/odom publication.
builtin_interfaces/Time stamp

# Unsigned physical GPIO edge rate in edges per second.
float64 edge_rate

# True after the configured interval has elapsed with no physical edge.
bool stationary

# Diagnostic direction of the current stop-delimited movement epoch:
# -1, 0 (unknown), or +1. This does not sign /wheel/odom.
int8 active_direction

# Latest valid nonzero /motor/direction command, used to sign /wheel/odom.
# This diagnostic value is not persistent pending transition state and cannot
# independently trigger a later active-direction change. A zero command does
# not erase it; a later nonzero command replaces it.
int8 pending_direction
