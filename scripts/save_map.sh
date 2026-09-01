#!/usr/bin/env bash

set -euo pipefail

if (( $# != 1 )); then
  echo "Usage: $0 MAP_NAME" >&2
  exit 2
fi

name=$1
if [[ ! "$name" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]] || [[ "$name" == *..* ]]; then
  echo "Invalid map name. Start with a letter or number, use only letters, numbers, dots, underscores, and hyphens, and do not use '..'." >&2
  exit 2
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
workspace_root=$(cd -- "$script_dir/.." && pwd -P)
map_dir="$workspace_root/maps"
map_path="$map_dir/$name"

mkdir -p -- "$map_dir"

if compgen -G "${map_path}.*" >/dev/null; then
  echo "Refusing to overwrite existing map artifacts: ${map_path}.*" >&2
  ls -lh -- "${map_path}."* >&2
  exit 1
fi

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

echo "Saving posegraph to $map_path"
if ! serialize_output=$(timeout --foreground 120s ros2 service call \
  "$serialize_service" \
  "$serialize_type" \
  "{filename: \"$map_path\"}" 2>&1)
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

for extension in posegraph data; do
  artifact="${map_path}.${extension}"
  if [[ ! -s "$artifact" ]]; then
    echo "SerializePoseGraph reported success but did not create a nonempty $artifact" >&2
    echo "Occupancy-grid export was not attempted; the map bundle is incomplete." >&2
    exit 1
  fi
done

echo "Saving occupancy grid to $map_path"
if ! ros2 run nav2_map_server map_saver_cli \
  -f "$map_path" \
  --fmt pgm \
  --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
then
  echo "Occupancy-grid save failed." >&2
  exit 1
fi

missing=0
for extension in posegraph data yaml pgm; do
  artifact="${map_path}.${extension}"
  if [[ ! -s "$artifact" ]]; then
    echo "Missing or empty map artifact: $artifact" >&2
    missing=1
  fi
done
if (( missing != 0 )); then
  exit 1
fi

echo
echo "Saved and verified all map artifacts:"
ls -lh -- \
  "${map_path}.posegraph" \
  "${map_path}.data" \
  "${map_path}.yaml" \
  "${map_path}.pgm"
