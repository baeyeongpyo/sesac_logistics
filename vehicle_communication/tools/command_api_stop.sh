#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(cd -- "$script_dir/.." && pwd)

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

processes=$(running_processes)
if [[ -z "$processes" ]]; then
  printf 'Vehicle Command API가 실행 중이 아닙니다.\n'
  exit 0
fi

while IFS= read -r pid; do
  kill -TERM "$pid"
done <<< "$processes"

printf 'Vehicle Command API 종료를 요청했습니다. PID=%s\n' "${processes//$'\n'/,}"
