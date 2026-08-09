#!/usr/bin/env bash
set -euo pipefail

readonly SLAM_DATA_ROOT="${SLAM_DATA_ROOT:-/slam-data}"
readonly REQUESTED_SESSION="${NAV_SESSION_ID:-}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

selection="$(python3 "${script_dir}/map_session.py" "$SLAM_DATA_ROOT" "$REQUESTED_SESSION")"
IFS=$'\t' read -r mode session_id map_yaml <<<"$selection"

if [[ "$mode" == 'localization' ]]; then
  printf 'mentorpi navigation_mode=localization-dual session_id=%s map_yaml=%s\n' \
    "$session_id" "$map_yaml"
  exec ros2 launch mentorpi_nav shared_navigation.launch.py \
    map_yaml:="$map_yaml" \
    warehouse_x:="${MAP_TO_WAREHOUSE_X:-0.0}" \
    warehouse_y:="${MAP_TO_WAREHOUSE_Y:-0.0}" \
    warehouse_yaw:="${MAP_TO_WAREHOUSE_YAW:-0.0}"
fi

printf 'mentorpi navigation_mode=mapping session_id=none reason=no_valid_saved_map\n'
exec ros2 launch mentorpi_nav navigation.launch.py mode:=mapping
