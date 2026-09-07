# Authorized navigation-mission operation from the command authority.
#
# Sole writer: the command authority. Sole consumer: the one navigation
# runtime. This is an authorized intent, never a direct Nav2 action bypass;
# the navigation runtime revalidates every field before it touches Nav2.
uint8 OP_SELECT=0    # bind a new validated logical mission; no dispatch
uint8 OP_DISPATCH=1  # dispatch the currently selected mission to Nav2
uint8 OP_CANCEL=2    # cancel any in-flight action; retain the logical mission

uint8 MISSION_SINGLE_GOAL=0
uint8 MISSION_ORDERED_POSES=1

builtin_interfaces/Time stamp
# Control lease that authorized this operation; the runtime does not re-check
# lease ownership (the authority already did) but records it for provenance.
string lease_id
uint8 operation

# Immutable identity of the logical mission this operation refers to. Empty for
# a pure OP_CANCEL that only targets whatever is in flight.
string mission_id
# Monotonic logical-mission revision. A callback or request carrying an older
# revision than the current one is stale and cannot become current.
uint64 mission_revision
# ModeState.runtime_epoch the mission is bound to. A mission from a superseded
# epoch can never be dispatched or resumed.
uint64 runtime_epoch
# Autonomy map-bundle identity the mission belongs to.
string map_id

uint8 mission_type
# Real, finite map-frame poses with valid orientations. One entry for a single
# goal; the ordered list for ordered poses. Never a placeholder (0,0,0) unless
# that is the operator's actual validated goal.
geometry_msgs/PoseStamped[] poses
