#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
package_dir=$(cd -- "$script_dir/.." && pwd)
runtime_env="$package_dir/runtime.env"
log_dir="$HOME/log"
log_file="$log_dir/foxglove_bridge"

running_processes() {
  ps -eo pid=,args= \
    | grep -E "[r]os2 launch foxglove_bridge foxglove_bridge_launch[.]xml.*port:=${FOXGLOVE_PORT}([[:space:]]|$)" \
    | awk '{ print $1 }' \
    || true
}

if [[ ! -r "$runtime_env" ]]; then
  printf 'runtime.env 파일을 읽을 수 없습니다: %s\n' "$runtime_env" >&2
  exit 1
fi

set -a
# shellcheck source=/dev/null
source "$runtime_env"
set +a

if [[ -z "${FOXGLOVE_PORT:-}" ]]; then
  printf 'runtime.env에 FOXGLOVE_PORT를 설정해야 합니다.\n' >&2
  exit 1
fi
if [[ ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]]; then
  printf 'FOXGLOVE_PORT는 숫자여야 합니다: %s\n' "$FOXGLOVE_PORT" >&2
  exit 1
fi
if running=$(running_processes) && [[ -n "$running" ]]; then
  printf 'Foxglove Bridge가 이미 실행 중입니다. PID=%s\n' "${running//$'\n'/,}"
  exit 0
fi

mkdir -p "$log_dir"

nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 \
  port:="$FOXGLOVE_PORT" \
  topic_whitelist:='["^/(map|tf|tf_static|joint_states|amcl_pose|odom|ros_robot_controller/battery)$"]' \
  best_effort_qos_topic_whitelist:='["(?!)"]' \
  client_topic_whitelist:='["(?!)"]' \
  service_whitelist:='["(?!)"]' \
  param_whitelist:='["(?!)"]' \
  asset_uri_allowlist:='["(?!)"]' \
  capabilities:='[none]' \
  include_hidden:=false \
  min_qos_depth:=1 \
  max_qos_depth:=10 \
  use_compression:=false \
  >"$log_file" 2>&1 < /dev/null &

printf 'Foxglove Bridge를 시작했습니다. 로그: %s\n' "$log_file"
