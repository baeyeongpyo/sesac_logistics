#!/usr/bin/env bash
set -euo pipefail

rosbag_root="${ROSBAG_ROOT:-/rosbag}"
requested_session_id="${ROSBAG_SESSION_ID:-}"
session_id="$requested_session_id"

mkdir -p "$rosbag_root"
if [[ -z "$session_id" ]]; then
  session_base="$(date -u +%Y%m%dT%H%M%SZ)"
  session_id="$session_base"
  suffix=1
  while [[ -e "${rosbag_root}/${session_id}" ]]; do
    printf -v session_id '%s-%02d' "$session_base" "$suffix"
    suffix=$((suffix + 1))
  done
fi

output_path="${rosbag_root}/${session_id}"
if [[ ! "$session_id" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
  printf 'rosbag-recorder config_error=invalid_session_id session_id=%s\n' "$session_id" >&2
  exit 64
fi
if [[ -n "$requested_session_id" && -e "$output_path" ]]; then
  printf 'rosbag-recorder config_error=session_exists output=%s\n' "$output_path" >&2
  exit 65
fi

printf 'rosbag-recorder session_id=%s output=%s\n' "$session_id" "$output_path"

exec ros2 bag record --output "$output_path" \
  /tf /tf_static /fleet/status \
  /robot_1/odom /robot_1/scan_raw /robot_1/imu/data_raw \
  /robot_2/odom /robot_2/scan_raw /robot_2/imu/data_raw
