#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
test_root=$(mktemp -d)
trap 'rm -rf -- "$test_root"' EXIT

mock_bin="$test_root/bin"
test_workspace="$test_root/workspace"
map_dir="$test_workspace/maps"
mock_log="$test_root/ros2.log"
mkdir -p -- "$mock_bin" "$map_dir" "$test_workspace/scripts"
cp -- "$script_dir/save_map.sh" "$test_workspace/scripts/save_map.sh"
save_script="$test_workspace/scripts/save_map.sh"

cat > "$mock_bin/ros2" <<'MOCK_ROS2'
#!/usr/bin/env bash
set -euo pipefail

printf '%q ' "$@" >> "$MOCK_ROS2_LOG"
printf '\n' >> "$MOCK_ROS2_LOG"

if [[ ${1:-} == service && ${2:-} == call ]]; then
  if [[ ${MOCK_SERIALIZE_CLI_STATUS:-0} != 0 ]]; then
    echo "mock serialize transport failure" >&2
    exit "$MOCK_SERIALIZE_CLI_STATUS"
  fi

  result=${MOCK_SERIALIZE_RESULT:-0}
  if [[ "$result" == 0 && ${MOCK_CREATE_SERIALIZED:-1} == 1 ]]; then
    printf 'posegraph\n' > "${MOCK_MAP_PATH}.posegraph"
    printf 'data\n' > "${MOCK_MAP_PATH}.data"
  fi
  printf 'response:\nslam_toolbox.srv.SerializePoseGraph_Response(result=%s)\n' "$result"
  exit 0
fi

if [[ ${1:-} == run && ${2:-} == nav2_map_server && ${3:-} == map_saver_cli ]]; then
  [[ " $* " == *" --fmt pgm "* ]]
  [[ " $* " == *" -p map_subscribe_transient_local:=true "* ]]
  [[ " $* " == *" -p save_map_timeout:=10.0 "* ]]
  if [[ ${MOCK_OCCUPANCY_STATUS:-0} != 0 ]]; then
    echo "mock occupancy timeout" >&2
    exit "$MOCK_OCCUPANCY_STATUS"
  fi
  if [[ ${MOCK_CREATE_OCCUPANCY:-1} == 1 ]]; then
    printf 'image: %s.pgm\n' "$(basename -- "$MOCK_MAP_PATH")" > "${MOCK_MAP_PATH}.yaml"
    printf 'P5\n1 1\n255\n0' > "${MOCK_MAP_PATH}.pgm"
  fi
  exit 0
fi

echo "unexpected ros2 invocation" >&2
exit 97
MOCK_ROS2
chmod +x "$mock_bin/ros2"

run_save()
{
  local name=$1
  shift
  MOCK_MAP_PATH="$map_dir/$name" \
  MOCK_ROS2_LOG="$mock_log" \
  PATH="$mock_bin:$PATH" \
    env "$@" "$save_script" "$name"
}

assert_fails_without_success()
{
  local output_file=$1
  shift
  if "$@" > "$output_file" 2>&1; then
    echo "Expected command to fail: $*" >&2
    exit 1
  fi
  if rg -q "Saved and verified all map artifacts" "$output_file"; then
    echo "Failure path printed a false success message: $*" >&2
    exit 1
  fi
}

success_output="$test_root/success.out"
run_save success > "$success_output" 2>&1
for extension in posegraph data yaml pgm; do
  test -s "$map_dir/success.$extension"
done
rg -q "Saved and verified all map artifacts" "$success_output"

assert_fails_without_success "$test_root/serialize-result.out" \
  run_save serialize_result_failure MOCK_SERIALIZE_RESULT=255
rg -q "SerializePoseGraph returned result=255" "$test_root/serialize-result.out"

assert_fails_without_success "$test_root/occupancy.out" \
  run_save occupancy_failure MOCK_OCCUPANCY_STATUS=1
rg -q "Occupancy-grid save failed" "$test_root/occupancy.out"

assert_fails_without_success "$test_root/missing.out" \
  run_save missing_occupancy MOCK_CREATE_OCCUPANCY=0
rg -q "Missing or empty map artifact" "$test_root/missing.out"

printf 'existing\n' > "$map_dir/existing.posegraph"
before_calls=$(wc -l < "$mock_log")
assert_fails_without_success "$test_root/overwrite.out" run_save existing
after_calls=$(wc -l < "$mock_log")
test "$before_calls" -eq "$after_calls"
rg -q "Refusing to overwrite existing map artifacts" "$test_root/overwrite.out"

assert_fails_without_success "$test_root/invalid.out" run_save 'bad/name'
rg -q "Invalid map name" "$test_root/invalid.out"

echo "save_map.sh tests passed"
