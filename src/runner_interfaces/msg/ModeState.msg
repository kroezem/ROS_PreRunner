# Authoritative, queryable Paddock mode snapshot.
uint8 MODE_IDLE=0
uint8 MODE_MAPPING=1
uint8 MODE_AUTONOMY=2

builtin_interfaces/Time stamp
uint8 mode
# Last accepted ModeRequest request_id; zero before any request is accepted.
uint64 accepted_request_id
# Explicit map selected for autonomy. Empty means that none is selected.
string active_autonomy_map
