#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(cd -- "$script_dir/.." && pwd)
runtime_env="$package_dir/runtime.env"

if [[ ! -r "$runtime_env" ]]; then
  printf 'runtime.env 파일을 읽을 수 없습니다: %s\n' "$runtime_env" >&2
  exit 1
fi

set -a
# shellcheck source=/dev/null
source "$runtime_env"
set +a

if [[ -z "${VEHICLE_ROBOT_ID:-}" || -z "${VEHICLE_COMMAND_API_PORT:-}" ]]; then
  printf 'runtime.env에 VEHICLE_ROBOT_ID와 VEHICLE_COMMAND_API_PORT를 설정해야 합니다.\n' >&2
  exit 1
fi

running_processes() {
  ps -eo pid=,args= | awk \
    -v port="$VEHICLE_COMMAND_API_PORT" \
    -v robot_id="$VEHICLE_ROBOT_ID" \
    '
      index($0, "ros2 run vehicle_command_api vehicle_command_api") == 0 { next }
      $0 ~ ("--port[[:space:]]+" port "([[:space:]]|$)") \
        && $0 ~ ("--robot-id[[:space:]]+" robot_id "([[:space:]]|$)") {
          print $1
        }
    '
}

processes=$(running_processes)
if [[ -z "$processes" ]]; then
  printf 'ROS Vehicle Command API가 실행 중이 아닙니다.\n'
  exit 0
fi

while IFS= read -r pid; do
  kill -TERM "$pid"
done <<< "$processes"

printf 'ROS Vehicle Command API 종료를 요청했습니다. PID=%s\n' "${processes//$'\n'/,}"
