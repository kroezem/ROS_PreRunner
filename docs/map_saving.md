# Runner map saving

The studio bundle is committed (6a7c961); artifact presence does not certify physical localization quality. Localization and autonomy require
an explicit basename containing nonempty `.posegraph`, `.data`, `.yaml`, and
the occupancy image referenced by that YAML.

Two paths produce a bundle. The Paddock backend (`runner_map_executor`, v1.3
Stage 4) is the normal operator path: it stages into `maps/.staging/`, captures
a same-session occupancy raster, validates the complete four-artifact bundle,
writes `<name>.manifest.json` (session id, creation time, revision, SHA-256 of
the four artifacts) and only then moves the bundle into `maps/`. A half-written
bundle stays in `.staging/` and never becomes selectable. See
`docs/paddock_v1.3_implementation.md`.

The `Runner: Save Map` VS Code task / `scripts/save_map.sh` is the engineering
path and remains valid. It creates two different representations of the same
map:

- `slam_toolbox`'s `SerializePoseGraph` service creates `.posegraph` and
  `.data`.
- Nav2's `map_saver_cli` creates `.pgm` and `.yaml`. The task explicitly uses
  transient-local map QoS and a 10-second subscription timeout.

The task refuses to run if any artifact already exists for the requested
basename. It reports success only after all four files exist and are nonempty.
A missing or incorrectly typed `/slam_toolbox/serialize_map` service fails
before any export. The serialized files are verified before occupancy export,
so a reported serialization success that produced no files cannot degrade into
another PGM/YAML-only basename.
A failed step may leave a partial basename in place; this is intentional so
that potentially valuable map data is never removed automatically.

## Recover occupancy files for an incomplete basename

Use this only when the existing `.posegraph` and `.data` are nonempty and both
occupancy files are absent. The checks below prevent overwriting any of the four
artifacts. Run it while the mapping launch is active and publishing `/map`:

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/runner_ws/install/setup.bash"

NAME="incomplete_map"
MAP="$HOME/runner_ws/maps/$NAME"

[[ "$NAME" =~ ^[A-Za-z0-9._-]+$ ]]
test -s "${MAP}.posegraph"
test -s "${MAP}.data"
test ! -e "${MAP}.yaml"
test ! -e "${MAP}.pgm"

ros2 run nav2_map_server map_saver_cli \
  -f "$MAP" \
  --fmt pgm \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0

test -s "${MAP}.yaml"
test -s "${MAP}.pgm"
```

If either occupancy artifact already exists, stop and choose a new basename or
inspect the existing files. Do not rerun the saver over that basename.

The focused shell test can be run without ROS or real map files:

```bash
scripts/test_save_map.sh
```
