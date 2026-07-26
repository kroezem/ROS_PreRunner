# ROS timestamp shared with the corresponding /wheel/odom publication.
builtin_interfaces/Time stamp

# Unsigned physical GPIO edge rate in edges per second.
float64 edge_rate

# True after the configured interval has elapsed with no physical edge.
bool stationary

# Direction used to sign /wheel/odom: -1, 0 (unknown), or +1.
int8 active_direction

# Latest valid nonzero /motor/direction command. This diagnostic value is not
# persistent pending transition state and cannot independently trigger a later
# direction change. A zero command does not erase it; a later nonzero command
# replaces it.
int8 pending_direction
