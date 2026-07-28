builtin_interfaces/Time stamp
bool pi_valid
float32 soc_temperature_celsius
uint32 throttled_raw
bool current_undervoltage
bool current_frequency_capped
bool current_throttled
bool current_soft_temperature_limit
bool sticky_undervoltage
bool sticky_frequency_capped
bool sticky_throttled
bool sticky_soft_temperature_limit
# CPU utilization is computed from successive /proc/stat samples.
bool cpu_valid
float32 total_cpu_utilization_percent
uint16[] cpu_core_ids
float32[] per_core_cpu_utilization_percent
# Load and runnable_processes come from one /proc/loadavg sample.
bool load_valid
float32 load_average_1min
float32 load_average_5min
float32 load_average_15min
uint32 runnable_processes
