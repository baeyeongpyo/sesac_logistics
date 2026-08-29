#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(cd -- "$script_dir/.." && pwd)
runtime_env="$package_dir/runtime.env"
log_dir="$HOME/log"
log_file="$log_dir/vehicle_command_api"

running_processes() {
  ps -eo pid=,args= | awk -v path="$package_dir/vehicle_command_api.py" \
    '
      {
        match_at = index($0, "python3 " path)
        if (!match_at) next

        suffix = substr($0, match_at + length("python3 " path))
        if (suffix == "" || suffix ~ /^[[:space:]]/) print $1
      }
    '
}

if running=$(running_processes) && [[ -n "$running" ]]; then
  printf 'Vehicle Command API가 이미 실행 중입니다. PID=%s\n' "${running//$'\n'/,}"
  exit 0
fi

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

mkdir -p "$log_dir"

(
  cd -- "$package_dir"
  nohup python3 "$package_dir/vehicle_command_api.py" \
    --host 0.0.0.0 \
    --port "$VEHICLE_COMMAND_API_PORT" \
    --robot-id "$VEHICLE_ROBOT_ID" \
    --cmd-vel-topic /cmd_vel \
    --action-name /navigate_to_pose \
    >"$log_file" 2>&1 < /dev/null &
)

printf 'Vehicle Command API를 시작했습니다. 로그: %s\n' "$log_file"
