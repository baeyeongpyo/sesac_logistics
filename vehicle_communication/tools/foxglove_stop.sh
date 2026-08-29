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

if [[ -z "${FOXGLOVE_PORT:-}" || ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]]; then
  printf 'runtime.env에 숫자 FOXGLOVE_PORT를 설정해야 합니다.\n' >&2
  exit 1
fi

running_processes() {
  ps -eo pid=,args= \
    | grep -E "[r]os2 launch foxglove_bridge foxglove_bridge_launch[.]xml.*port:=${FOXGLOVE_PORT}([[:space:]]|$)" \
    | awk '{ print $1 }' \
    || true
}

processes=$(running_processes)
if [[ -z "$processes" ]]; then
  printf 'Foxglove Bridge가 실행 중이 아닙니다.\n'
  exit 0
fi

while IFS= read -r pid; do
  kill -TERM "$pid"
done <<< "$processes"

printf 'Foxglove Bridge 종료를 요청했습니다. PID=%s\n' "${processes//$'\n'/,}"
