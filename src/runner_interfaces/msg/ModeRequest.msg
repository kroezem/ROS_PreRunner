# Idempotent request from the current Paddock control lease holder.
uint8 MODE_IDLE=0
uint8 MODE_MAPPING=1
uint8 MODE_AUTONOMY=2

builtin_interfaces/Time stamp
uint64 request_id
string lease_id
uint8 requested_mode
# Required map basename for AUTONOMY; ignored for IDLE and MAPPING.
string autonomy_map
