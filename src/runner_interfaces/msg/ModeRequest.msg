# Idempotent request from the current Paddock control lease holder.
uint8 MODE_IDLE=0
uint8 MODE_MAPPING=1
uint8 MODE_AUTONOMY=2

# Runtime/map operation carried by this request.
uint8 OP_SELECT_RUNTIME=0
uint8 OP_NEW_MAP=1

builtin_interfaces/Time stamp
uint64 request_id
string lease_id
uint8 requested_mode
# Required map basename for AUTONOMY; ignored for IDLE and MAPPING.
string autonomy_map
# OP_SELECT_RUNTIME performs an idempotent runtime selection. OP_NEW_MAP is only
# valid with requested_mode MAPPING and forces a fresh mapping SLAM session even
# when MAPPING is already the stable runtime.
uint8 operation
