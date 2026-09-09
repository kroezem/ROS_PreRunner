# Source-specific output from the one shared drive adapter. command.linear.x
# is normalized longitudinal effort and command.angular.z is normalized
# steering. The authority validates provenance before supervising it.
uint8 SOURCE_AUTONOMY=0
uint8 SOURCE_PADDOCK_MANUAL=1

builtin_interfaces/Time stamp
uint8 source
uint64 sequence
uint64 lease_generation
uint64 runtime_epoch
geometry_msgs/Twist command

