# Truthful navigation-mission and Nav2-action lifecycle.
#
# Sole writer: the one navigation runtime. Consumers: the command authority and
# the Paddock gateway. Application-lifetime: absence of this message (or a
# changed boot_id) means any previously reported mission state is stale.
uint8 STATE_IDLE=0        # runtime alive; no in-flight action
uint8 STATE_DISPATCHING=1 # goal sent to Nav2; not yet accepted (no motion)
uint8 STATE_ACTIVE=2      # Nav2 accepted and is executing the current goal
uint8 STATE_CANCELING=3   # cancel requested; awaiting terminal action result
uint8 STATE_SUCCEEDED=4   # real Nav2 success result
uint8 STATE_FAILED=5      # rejection / abort / timeout / error
uint8 STATE_CANCELED=6    # confirmed Nav2 cancellation

builtin_interfaces/Time stamp
# Navigation-runtime process identity. Changes on every restart so a consumer
# cannot carry mission state across a runtime epoch/restart.
string boot_id
uint8 state

# Currently bound logical mission (may outlive a terminal action as a
# deliberate-continuation candidate). Empty mission_id means nothing is bound.
string mission_id
uint64 mission_revision
uint64 runtime_epoch
string map_id
uint8 mission_type
bool mission_valid

# Current Nav2 action attempt. action_generation is a monotonic counter; every
# asynchronous Nav2 callback is checked against it and the live goal handle.
uint64 action_generation
string goal_uuid
# action_msgs/GoalStatus value of the last observed action status; -1 = none.
int8 nav2_status
uint16 error_code
string error_meaning
# First actionable reason when not progressing; last error text otherwise.
string detail
