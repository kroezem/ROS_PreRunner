# Atomic state produced by the Pi-side supervised keyboard bridge.
uint8 MODE_DRIVE=0
uint8 MODE_BRAKE=1
uint8 MODE_SUPPRESS=2
uint8 MODE_BRAKE_SUPPRESS=3

int8 GLOBAL_OBSTACLES_UNKNOWN=-1
int8 GLOBAL_OBSTACLES_DISABLED=0
int8 GLOBAL_OBSTACLES_ENABLED=1

bool valid
uint8 mode
float32 throttle
float32 steering
uint64 session_id
uint32 sequence
int8 global_obstacles_state
