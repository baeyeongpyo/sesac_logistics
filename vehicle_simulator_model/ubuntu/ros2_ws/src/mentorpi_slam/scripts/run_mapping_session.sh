#!/usr/bin/env bash
# Record a mapping run into an unpublished staging directory, then atomically publish it.
set -euo pipefail

die() {
  printf 'run_mapping_session: %s\n' "$*" >&2
  exit 1
}

require_environment() {
  local variable_name="$1"
  [[ -n "${!variable_name:-}" ]] || die "${variable_name} must be set and non-empty"
}

validate_timeout() {
  local variable_name="$1"
  local value="${!variable_name}"
  [[ "$value" =~ ^[0-9]+$ ]] || die "${variable_name} must be a non-negative integer"
}

for required_variable in \
  SLAM_DATA_ROOT SESSION_ID IMAGE_VERSION GIT_COMMIT WORLD_VERSION MODEL_VERSION \
  TF_CALIBRATION_VERSION; do
  require_environment "$required_variable"
done

if [[ ! "$SESSION_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
  die 'SESSION_ID may contain only A-Z, a-z, 0-9, period, underscore, and hyphen'
fi
if [[ "$SLAM_DATA_ROOT" != /* || "$SLAM_DATA_ROOT" =~ [[:cntrl:]] || "$SLAM_DATA_ROOT" == *\"* || "$SLAM_DATA_ROOT" == *\'* ]]; then
  die 'SLAM_DATA_ROOT must be a safe absolute path without control characters or quotes'
fi

SLAM_SERVICE_WAIT_SECONDS="${SLAM_SERVICE_WAIT_SECONDS:-30}"
ROS_COMMAND_TIMEOUT_SECONDS="${ROS_COMMAND_TIMEOUT_SECONDS:-10}"
PROCESS_STOP_TIMEOUT_SECONDS="${PROCESS_STOP_TIMEOUT_SECONDS:-10}"
PROCESS_KILL_TIMEOUT_SECONDS="${PROCESS_KILL_TIMEOUT_SECONDS:-3}"
for timeout_variable in \
  SLAM_SERVICE_WAIT_SECONDS ROS_COMMAND_TIMEOUT_SECONDS \
  PROCESS_STOP_TIMEOUT_SECONDS PROCESS_KILL_TIMEOUT_SECONDS; do
  validate_timeout "$timeout_variable"
done

mkdir -p "$SLAM_DATA_ROOT"
slam_data_root="$(cd -- "$SLAM_DATA_ROOT" && pwd -P)"
stage_parent="${slam_data_root}/.inprogress"
stage_dir="${stage_parent}/${SESSION_ID}"
final_dir="${slam_data_root}/${SESSION_ID}"
lock_parent="${slam_data_root}/.session-locks"
lock_dir="${lock_parent}/${SESSION_ID}.lock"

mkdir -p "$stage_parent" "$lock_parent"
[[ ! -e "$stage_dir" && ! -e "$final_dir" ]] || die "refusing to reuse existing session path for ${SESSION_ID}"

rosbag_pid=''
slam_pid=''
published=0
finalization_in_progress=0

release_lock() {
  rmdir "$lock_dir" 2>/dev/null || true
}

process_alive() {
  kill -0 "$1" 2>/dev/null
}

wait_for_exit() {
  local process_pid="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while process_alive "$process_pid"; do
    ((SECONDS < deadline)) || return 1
    sleep 0.1
  done
  return 0
}

run_bounded() {
  local description="$1"
  local timeout_seconds="$2"
  shift 2
  "$@" &
  local command_pid=$!
  if ! wait_for_exit "$command_pid" "$timeout_seconds"; then
    printf 'run_mapping_session: %s exceeded %ss\n' "$description" "$timeout_seconds" >&2
    kill -TERM "$command_pid" 2>/dev/null || true
    if ! wait_for_exit "$command_pid" "$PROCESS_KILL_TIMEOUT_SECONDS"; then
      kill -KILL "$command_pid" 2>/dev/null || true
    fi
  fi
  wait "$command_pid"
}

run_ros2() {
  run_bounded "ros2 $1" "$ROS_COMMAND_TIMEOUT_SECONDS" ros2 "$@"
}

stop_process() {
  local stop_signal="$1"
  local process_pid="$2"
  local requested_stop=0
  [[ -n "$process_pid" ]] || return 0

  if process_alive "$process_pid"; then
    requested_stop=1
    kill "-$stop_signal" "$process_pid" 2>/dev/null || true
    if ! wait_for_exit "$process_pid" "$PROCESS_STOP_TIMEOUT_SECONDS"; then
      kill -TERM "$process_pid" 2>/dev/null || true
      if ! wait_for_exit "$process_pid" "$PROCESS_KILL_TIMEOUT_SECONDS"; then
        kill -KILL "$process_pid" 2>/dev/null || true
      fi
    fi
  fi
  if wait "$process_pid"; then
    return 0
  fi
  # A process we explicitly stopped may report its signal exit status; it is
  # still a successful bounded cleanup once it has been reaped.
  ((requested_stop)) && return 0
  return 1
}

stop_recording_and_slam() {
  local status=0
  if ! stop_process INT "$rosbag_pid"; then status=1; fi
  if ! stop_process TERM "$slam_pid"; then status=1; fi
  rosbag_pid=''
  slam_pid=''
  return "$status"
}

cleanup() {
  local status=$?
  if (( ! published )); then
    stop_recording_and_slam || true
  fi
  release_lock
  return "$status"
}
mkdir "$lock_dir" 2>/dev/null || die "another mapping lifecycle owns session ${SESSION_ID}"
trap cleanup EXIT
[[ ! -e "$stage_dir" && ! -e "$final_dir" ]] || die "session path appeared while acquiring lock for ${SESSION_ID}"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
installed_slam_config="${script_dir}/../../share/mentorpi_slam/config/slam.yaml"
source_slam_config="${script_dir}/../config/slam.yaml"
slam_config="${SLAM_CONFIG_PATH:-$installed_slam_config}"
if [[ ! -f "$slam_config" && -z "${SLAM_CONFIG_PATH:-}" && -f "$source_slam_config" ]]; then
  slam_config="$source_slam_config"
fi
[[ -f "$slam_config" ]] || die "installed slam.yaml was not found: ${slam_config}"
session_artifacts="${script_dir}/session_artifacts.py"
[[ -f "$session_artifacts" ]] || die "session_artifacts.py was not found: ${session_artifacts}"
atomic_publisher="${ATOMIC_PUBLISHER:-${script_dir}/atomic_publish.py}"
[[ -x "$atomic_publisher" ]] || die "atomic_publish.py was not executable: ${atomic_publisher}"

mkdir "$stage_dir"
mkdir -p "$stage_dir/posegraph" "$stage_dir/rosbag2"

wait_for_slam_services() {
  local deadline=$((SECONDS + SLAM_SERVICE_WAIT_SECONDS))
  local services
  while :; do
    if ! services="$(run_ros2 service list 2>/dev/null)"; then
      return 1
    fi
    if grep -Fxq '/slam_toolbox/save_map' <<<"$services" \
      && grep -Fxq '/slam_toolbox/serialize_map' <<<"$services"; then
      return 0
    fi
    ((SECONDS < deadline)) || break
    sleep 1
  done
  printf 'run_mapping_session: timed out waiting for slam_toolbox services\n' >&2
  return 1
}

make_save_map_request() {
  python3 - "$1" <<'PY'
import json
import sys

print(json.dumps({'name': {'data': sys.argv[1]}}))
PY
}

make_posegraph_request() {
  python3 - "$1" <<'PY'
import json
import sys

print(json.dumps({'filename': sys.argv[1]}))
PY
}

rosbag_metadata_is_nonempty() {
  find "$stage_dir/rosbag2" -type f -name metadata.yaml -size +0c -print -quit | grep -q .
}

rosbag_storage_is_nonempty() {
  find "$stage_dir/rosbag2" -type f \( -name '*.db3' -o -name '*.mcap' \) -size +0c -print -quit | grep -q .
}

posegraph_is_nonempty() {
  find "$stage_dir/posegraph" -type f -size +0c -print -quit | grep -q .
}

write_metadata_and_checksums() {
  local slam_params_sha256 created_at
  slam_params_sha256="$(python3 - "$slam_config" <<'PY'
import hashlib
import pathlib
import sys
print(hashlib.sha256(pathlib.Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)"
  created_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  python3 - "$session_artifacts" "$stage_dir" "$SESSION_ID" "$IMAGE_VERSION" \
    "$GIT_COMMIT" "$WORLD_VERSION" "$MODEL_VERSION" "$slam_params_sha256" \
    "$TF_CALIBRATION_VERSION" "$created_at" <<'PY'
import importlib.util
from pathlib import Path
import sys
(_, module_path, session_dir, session_id, image_version, git_commit, world_version,
 model_version, slam_params_sha256, tf_calibration_version, created_at) = sys.argv
spec = importlib.util.spec_from_file_location('session_artifacts', module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.write_manifest(Path(session_dir), {
    'session_id': session_id, 'robot_id': 'robot_1', 'image_version': image_version,
    'git_commit': git_commit, 'world_version': world_version, 'model_version': model_version,
    'slam_params_sha256': slam_params_sha256,
    'tf_calibration_version': tf_calibration_version, 'created_at': created_at,
})
module.write_checksums(Path(session_dir))
PY
}

publish_session() {
  # Contract token for the same-parent rename: mv "$stage_dir" "$final_dir"
  if ! "$atomic_publisher" "$stage_dir" "$final_dir"; then return 1; fi
  [[ ! -e "$stage_dir" && -d "$final_dir" ]] || return 1
  published=1
}

finalize_session() {
  if ((finalization_in_progress)); then
    return 1
  fi
  finalization_in_progress=1
  local save_request posegraph_request
  if ! wait_for_slam_services; then return 1; fi
  save_request="$(make_save_map_request "${stage_dir}/map")"
  posegraph_request="$(make_posegraph_request "${stage_dir}/posegraph/mentorpi")"
  if ! run_ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "$save_request"; then return 1; fi
  if ! run_ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "$posegraph_request"; then return 1; fi
  if ! stop_recording_and_slam; then return 1; fi
  if [[ ! -s "$stage_dir/map.yaml" || ! -s "$stage_dir/map.pgm" ]]; then return 1; fi
  if ! posegraph_is_nonempty || ! rosbag_metadata_is_nonempty || ! rosbag_storage_is_nonempty; then return 1; fi
  if ! write_metadata_and_checksums; then return 1; fi
  publish_session
}

on_signal() {
  local status
  if ((finalization_in_progress)); then
    return 0
  fi
  if finalize_session; then
    exit 0
  else
    status=$?
    exit "$status"
  fi
}
trap on_signal INT TERM

ros2 bag record --output "$stage_dir/rosbag2/mapping" \
  /clock /tf /tf_static /robot_1/scan_raw /robot_1/imu/data_raw /robot_1/odom &
rosbag_pid=$!
ros2 launch mentorpi_slam mapping.launch.py &
slam_pid=$!

if wait "$slam_pid"; then
  slam_pid=''
  finalize_session
else
  exit $?
fi
