#!/usr/bin/env bash

set -euo pipefail

if (( $# != 1 )); then
  echo "Usage: $0 MAP_ID" >&2
  exit 2
fi

name=$1
if [[ ! "$name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || [[ "$name" == *..* ]]; then
  echo "Invalid map id. Start with a letter or number, use only letters, numbers, dots, underscores, and hyphens, and do not use '..'." >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace_root=$(cd -- "$script_dir/.." && pwd -P)
map_root="$workspace_root/maps"
map_dir="$map_root/$name"
staging_root="$map_root/.staging"
staging_dir="$staging_root/$name"

mkdir -p -- "$map_root" "$staging_root"

if [[ -e "$map_dir" ]]; then
  echo "Refusing to overwrite existing map directory: $map_dir" >&2
  exit 1
fi

rm -rf -- "$staging_dir"
mkdir -p -- "$staging_dir"
cleanup() { rm -rf -- "$staging_dir"; }
trap cleanup EXIT

posegraph_stem="$staging_dir/posegraph"

serialize_service=/slam_toolbox/serialize_map
serialize_type=slam_toolbox/srv/SerializePoseGraph

if ! discovered_type=$(timeout --foreground 5s ros2 service type \
  "$serialize_service" 2>&1)
then
  if [[ -n "$discovered_type" ]]; then
    printf '%s\n' "$discovered_type" >&2
  fi
  echo "Required slam_toolbox service is unavailable: $serialize_service" >&2
  echo "Run this save while Runner: Map is active and slam_toolbox is configured." >&2
  exit 1
fi
if [[ "$discovered_type" != "$serialize_type" ]]; then
  echo "Unexpected type for $serialize_service: '$discovered_type'" >&2
  echo "Expected: $serialize_type" >&2
  exit 1
fi

echo "Saving posegraph to $posegraph_stem"
if ! serialize_output=$(timeout --foreground 120s ros2 service call \
  "$serialize_service" \
  "$serialize_type" \
  "{filename: \"$posegraph_stem\"}" 2>&1)
then
  printf '%s\n' "$serialize_output" >&2
  echo "SerializePoseGraph call failed." >&2
  exit 1
fi
printf '%s\n' "$serialize_output"

serialize_result=$(printf '%s\n' "$serialize_output" |
  sed -nE 's/.*result[=:][[:space:]]*([0-9]+).*/\1/p' |
  tail -n 1)
if [[ "$serialize_result" != "0" ]]; then
  if [[ -z "$serialize_result" ]]; then
    echo "Could not determine SerializePoseGraph result from the service response." >&2
  else
    echo "SerializePoseGraph returned result=$serialize_result; expected result=0." >&2
  fi
  exit 1
fi

for suffix in posegraph data; do
  artifact="${posegraph_stem}.${suffix}"
  if [[ ! -s "$artifact" ]]; then
    echo "SerializePoseGraph reported success but did not create a nonempty $artifact" >&2
    echo "Occupancy-grid export was not attempted; the map bundle is incomplete." >&2
    exit 1
  fi
done

# map_saver_cli ties its YAML and image outputs to one shared stem, so it
# cannot itself write the canonical occupancy.pgm/map.yaml pair. Save to a
# private stem, then rename/rewrite into the canonical filenames below.
saver_stem="$staging_dir/raw_occupancy"

echo "Saving occupancy grid to $saver_stem"
if ! ros2 run nav2_map_server map_saver_cli \
  -f "$saver_stem" \
  --fmt pgm \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
then
  echo "Occupancy-grid save failed." >&2
  exit 1
fi

if [[ ! -s "${saver_stem}.pgm" || ! -s "${saver_stem}.yaml" ]]; then
  echo "Missing or empty map artifact: ${saver_stem}.pgm / ${saver_stem}.yaml" >&2
  exit 1
fi

mv -- "${saver_stem}.pgm" "$staging_dir/occupancy.pgm"
sed -E 's/^image:.*/image: occupancy.pgm/' "${saver_stem}.yaml" > "$staging_dir/map.yaml"
rm -f -- "${saver_stem}.yaml"

missing=0
for artifact in posegraph.posegraph posegraph.data occupancy.pgm map.yaml; do
  if [[ ! -s "$staging_dir/$artifact" ]]; then
    echo "Missing or empty map artifact: $staging_dir/$artifact" >&2
    missing=1
  fi
done
if (( missing != 0 )); then
  exit 1
fi

mv -- "$staging_dir" "$map_dir"
trap - EXIT

echo
echo "Saved and verified all map artifacts under $map_dir:"
ls -lh -- "$map_dir"
