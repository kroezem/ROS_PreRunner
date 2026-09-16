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

if [[ ${1:-} == service && ${2:-} == type ]]; then
  if [[ ${MOCK_SERVICE_AVAILABLE:-1} != 1 ]]; then
    exit 1
  fi
  printf '%s\n' "${MOCK_SERVICE_TYPE:-slam_toolbox/srv/SerializePoseGraph}"
  exit 0
fi

if [[ ${1:-} == service && ${2:-} == call ]]; then
  filename=$(printf '%s' "${5:-}" | sed -nE 's/.*filename: *"([^"]*)".*/\1/p')

  if [[ ${MOCK_SERIALIZE_CLI_STATUS:-0} != 0 ]]; then
    echo "mock serialize transport failure" >&2
    exit "$MOCK_SERIALIZE_CLI_STATUS"
  fi

  result=${MOCK_SERIALIZE_RESULT:-0}
  if [[ "$result" == 0 && ${MOCK_CREATE_SERIALIZED:-1} == 1 ]]; then
    printf 'posegraph\n' > "${filename}.posegraph"
    printf 'data\n' > "${filename}.data"
  fi
  printf 'response:\nslam_toolbox.srv.SerializePoseGraph_Response(result=%s)\n' "$result"
  exit 0
fi

if [[ ${1:-} == run && ${2:-} == nav2_map_server && ${3:-} == map_saver_cli ]]; then
  [[ " $* " == *" --fmt pgm "* ]]
  [[ " $* " == *" -p map_subscribe_transient_local:=true "* ]]
  [[ " $* " == *" -p save_map_timeout:=10.0 "* ]]
  stem=""
  previous=""
  for arg in "$@"; do
    if [[ "$previous" == "-f" ]]; then
      stem=$arg
    fi
    previous=$arg
  done
  if [[ ${MOCK_OCCUPANCY_STATUS:-0} != 0 ]]; then
    echo "mock occupancy timeout" >&2
    exit "$MOCK_OCCUPANCY_STATUS"
  fi
  if [[ ${MOCK_CREATE_OCCUPANCY:-1} == 1 ]]; then
    printf 'image: %s.pgm\n' "$(basename -- "$stem")" > "${stem}.yaml"
    printf 'P5\n1 1\n255\n0' > "${stem}.pgm"
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

assert_no_partial_directory()
{
  local name=$1
  if [[ -e "$map_dir/$name" ]]; then
    echo "Failed save left a published map directory: $map_dir/$name" >&2
    exit 1
  fi
  if [[ -e "$map_dir/.staging/$name" ]]; then
    echo "Failed save left a staging directory behind: $map_dir/.staging/$name" >&2
    exit 1
  fi
}

success_output="$test_root/success.out"
run_save success > "$success_output" 2>&1
for artifact in posegraph.posegraph posegraph.data occupancy.pgm map.yaml; do
  test -s "$map_dir/success/$artifact"
done
test "$(find "$map_dir/success" -maxdepth 1 -type f | wc -l)" -eq 4
rg -q '^image: occupancy\.pgm$' "$map_dir/success/map.yaml"
rg -q "Saved and verified all map artifacts" "$success_output"
[[ ! -e "$map_dir/.staging/success" ]]

assert_fails_without_success "$test_root/serialize-result.out" \
  run_save serialize_result_failure MOCK_SERIALIZE_RESULT=255
rg -q "SerializePoseGraph returned result=255" "$test_root/serialize-result.out"
assert_no_partial_directory serialize_result_failure

before_calls=$(wc -l < "$mock_log")
assert_fails_without_success "$test_root/service-unavailable.out" \
  run_save service_unavailable MOCK_SERVICE_AVAILABLE=0
after_calls=$(wc -l < "$mock_log")
test "$((after_calls - before_calls))" -eq 1
rg -q "Required slam_toolbox service is unavailable" \
  "$test_root/service-unavailable.out"
assert_no_partial_directory service_unavailable

before_calls=$(wc -l < "$mock_log")
assert_fails_without_success "$test_root/service-type.out" \
  run_save service_type_mismatch MOCK_SERVICE_TYPE=example_interfaces/srv/AddTwoInts
after_calls=$(wc -l < "$mock_log")
test "$((after_calls - before_calls))" -eq 1
rg -q "Unexpected type for /slam_toolbox/serialize_map" \
  "$test_root/service-type.out"
assert_no_partial_directory service_type_mismatch

before_calls=$(wc -l < "$mock_log")
assert_fails_without_success "$test_root/missing-serialized.out" \
  run_save missing_serialized MOCK_CREATE_SERIALIZED=0
after_calls=$(wc -l < "$mock_log")
test "$((after_calls - before_calls))" -eq 2
rg -q "Occupancy-grid export was not attempted" \
  "$test_root/missing-serialized.out"
assert_no_partial_directory missing_serialized

assert_fails_without_success "$test_root/occupancy.out" \
  run_save occupancy_failure MOCK_OCCUPANCY_STATUS=1
rg -q "Occupancy-grid save failed" "$test_root/occupancy.out"
assert_no_partial_directory occupancy_failure

assert_fails_without_success "$test_root/missing.out" \
  run_save missing_occupancy MOCK_CREATE_OCCUPANCY=0
rg -q "Missing or empty map artifact" "$test_root/missing.out"
assert_no_partial_directory missing_occupancy

mkdir -p -- "$map_dir/existing"
printf 'existing\n' > "$map_dir/existing/posegraph.posegraph"
before_calls=$(wc -l < "$mock_log")
assert_fails_without_success "$test_root/overwrite.out" run_save existing
after_calls=$(wc -l < "$mock_log")
test "$before_calls" -eq "$after_calls"
rg -q "Refusing to overwrite existing map directory" "$test_root/overwrite.out"

assert_fails_without_success "$test_root/invalid.out" run_save 'bad/name'
rg -q "Invalid map id" "$test_root/invalid.out"
assert_fails_without_success "$test_root/dotdot.out" run_save 'bad..name'
rg -q "Invalid map id" "$test_root/dotdot.out"
assert_fails_without_success "$test_root/leading-dot.out" run_save '.hidden'
rg -q "Invalid map id" "$test_root/leading-dot.out"

echo "save_map.sh tests passed"
