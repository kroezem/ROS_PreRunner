# Ordered control events accepted from the current Paddock lease holder.
uint8 EVENT_RUN_PRESSED=0
uint8 EVENT_RUN_RELEASED=1
uint8 EVENT_STOP=2
uint8 EVENT_MANUAL_ACTIVE=3
uint8 EVENT_MANUAL_INACTIVE=4
uint8 EVENT_GOAL_SELECTED=5
uint8 EVENT_LEASE_ACQUIRED=6
uint8 EVENT_LEASE_RELEASED=7
uint8 EVENT_LEASE_LOST=8

builtin_interfaces/Time stamp
uint64 sequence
string client_id
string lease_id
uint8 event
