# Lockehaven occupancy-map recovery

Date: 2026-07-30 (America/Vancouver)

The original mapping session had only the serialized SLAM Toolbox artifacts:

- `maps/lockehaven.posegraph`
- `maps/lockehaven.data`

The missing `maps/lockehaven.pgm` and `maps/lockehaven.yaml` were regenerated
from that pose graph without feeding any new scans into the graph.

## Recovery configuration

The installed ROS 2 Jazzy package was
`ros-jazzy-slam-toolbox 2.8.5-1noble.20260614.104642`. A temporary copy of
`runner_bringup/config/mapper_params_online_async.yaml` was used with these
recovery-only values:

```yaml
slam_toolbox:
  ros__parameters:
    mode: mapping
    map_file_name: /home/matti/runner_ws/maps/lockehaven
    map_start_at_dock: true
    scan_topic: /__lockehaven_recovery_no_scan
    use_map_saver: true
    enable_interactive_mode: false
    map_update_interval: 1.0
```

`use_map_saver` must be enabled because it creates the explicit
`/slam_toolbox/save_map` service. It did not write files on startup or
shutdown; the output appeared only after the service call.

The node was started with:

```bash
source /opt/ros/jazzy/setup.bash
source /home/matti/runner_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/tmp/lockehaven_recovery_params.yaml \
  use_sim_time:=false
```

Before saving, `/map` was verified as:

- resolution: `0.05 m/cell`
- dimensions: `346 x 343` cells (`17.30 x 17.15 m`)
- origin: `[-7.015443565252343, -9.805202049144059, 0]`
- cells: 40,260 free, 6,766 occupied, 71,652 unknown

The rendered grid was a coherent multi-room floor plan with connected central
space, perimeter walls, interior partitions and door openings, and mapped
rooms extending left, right, and lower-right. Both output files were confirmed
absent at this point.

The only map-writing command was:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/matti/runner_ws/maps/lockehaven'}}"
```

It returned `result=0`. The saved PGM was pixel-identical to the preview made
from `/map`.

## Result

```text
lockehaven.posegraph  24,872,261 bytes
lockehaven.data        2,390,577 bytes
lockehaven.pgm           118,693 bytes
lockehaven.yaml              132 bytes
```

The original serialized files retained their timestamps and SHA-256 hashes:

```text
0ccf946877c95fd9af5b20ad6d2a23183f0aa95e71c114bde4ea602c23aac6ee  lockehaven.posegraph
13f258ade8c0436c3593f8799086f1dae0d963d203af824218b8a07e704fb688  lockehaven.data
```

Generated artifact hashes:

```text
feba984ec3bbe2ddcc7aa7a770ba74ec15325e6a6bdec8563d947f41879c3b19  lockehaven.pgm
e6febf16febba0b09ebb5ccc9de77a7f15f155f410ca47d41db1d31a13196501  lockehaven.yaml
```
