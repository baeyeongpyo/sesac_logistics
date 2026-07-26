#!/usr/bin/env bash
# Record a mapping run into an unpublished staging directory, then atomically publish it.
set -euo pipefail

die() {
  printf 'run_mapping_session: %s\n' "$*" >&2
  exit 1
}

require_environment() {
  local variable_name="$1"
  if [[ -z "${!variable_name:-}" ]]; then
    die "${variable_name} must be set and non-empty"
  fi
}

for required_variable in \
  SLAM_DATA_ROOT SESSION_ID IMAGE_VERSION GIT_COMMIT WORLD_VERSION MODEL_VERSION \
  TF_CALIBRATION_VERSION; do
  require_environment "$required_variable"
done

if [[ ! "$SESSION_ID" =~ ^[A-Za-z0-9._-]+$ ]]; then
  die 'SESSION_ID may contain only A-Z, a-z, 0-9, period, underscore, and hyphen'
fi

stage_dir="${SLAM_DATA_ROOT}/.inprogress/${SESSION_ID}"
final_dir="${SLAM_DATA_ROOT}/${SESSION_ID}"
if [[ -e "$stage_dir" || -e "$final_dir" ]]; then
  die "refusing to reuse existing session path for ${SESSION_ID}"
fi

mkdir -p "${SLAM_DATA_ROOT}/.inprogress"
mkdir "$stage_dir"
mkdir -p "$stage_dir/posegraph" "$stage_dir/rosbag2"

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

rosbag_pid=''
slam_pid=''
finalization_started=0

wait_for_slam_services() {
  local wait_seconds="${SLAM_SERVICE_WAIT_SECONDS:-30}"
  local attempt services
  [[ "$wait_seconds" =~ ^[0-9]+$ ]] || die 'SLAM_SERVICE_WAIT_SECONDS must be a non-negative integer'

  for ((attempt = 0; attempt <= wait_seconds; attempt++)); do
    services="$(ros2 service list 2>/dev/null || true)"
    if grep -Fxq '/slam_toolbox/save_map' <<<"$services" \
      && grep -Fxq '/slam_toolbox/serialize_map' <<<"$services"; then
      return 0
    fi
    if ((attempt < wait_seconds)); then
      sleep 1
    fi
  done

  printf 'run_mapping_session: timed out waiting for slam_toolbox services\n' >&2
  return 1
}

stop_process() {
  local stop_signal="$1"
  local process_pid="$2"
  [[ -n "$process_pid" ]] || return 0

  if kill -0 "$process_pid" 2>/dev/null; then
    kill "-$stop_signal" "$process_pid" 2>/dev/null || true
  fi
  wait "$process_pid"
}

stop_recording_and_slam() {
  local status=0
  if ! stop_process INT "$rosbag_pid"; then
    status=1
  fi
  if ! stop_process TERM "$slam_pid"; then
    status=1
  fi
  rosbag_pid=''
  slam_pid=''
  return "$status"
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

(
    _, module_path, session_dir, session_id, image_version, git_commit,
    world_version, model_version, slam_params_sha256, tf_calibration_version,
    created_at,
) = sys.argv
spec = importlib.util.spec_from_file_location('session_artifacts', module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
metadata = {
    'session_id': session_id,
    'robot_id': 'robot_1',
    'image_version': image_version,
    'git_commit': git_commit,
    'world_version': world_version,
    'model_version': model_version,
    'slam_params_sha256': slam_params_sha256,
    'tf_calibration_version': tf_calibration_version,
    'created_at': created_at,
}
session_path = Path(session_dir)
module.write_manifest(session_path, metadata)
module.write_checksums(session_path)
PY
}

rosbag_is_nonempty() {
  find "$stage_dir/rosbag2" -type f -size +0c -print -quit | grep -q .
}

finalize_session() {
  if ((finalization_started)); then
    return 1
  fi
  finalization_started=1

  if ! wait_for_slam_services; then
    stop_recording_and_slam || true
    return 1
  fi
  if ! ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '${stage_dir}/map'}}"; then
    stop_recording_and_slam || true
    return 1
  fi
  if ! ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${stage_dir}/posegraph/mentorpi'}"; then
    stop_recording_and_slam || true
    return 1
  fi
  if ! stop_recording_and_slam; then
    return 1
  fi
  if ! rosbag_is_nonempty; then
    printf 'run_mapping_session: rosbag did not contain data\n' >&2
    return 1
  fi
  if ! write_metadata_and_checksums; then
    return 1
  fi
  if [[ ! -s "$stage_dir/map.yaml" || ! -s "$stage_dir/map.pgm" ]]; then
    printf 'run_mapping_session: save_map did not produce a non-empty map.yaml and map.pgm\n' >&2
    return 1
  fi

  mv "$stage_dir" "$final_dir"
}

on_signal() {
  local status
  trap - INT TERM
  if finalize_session; then
    exit 0
  else
    status=$?
  fi
  stop_recording_and_slam || true
  exit "$status"
}

trap 'on_signal' INT TERM

ros2 bag record --output "$stage_dir/rosbag2/mapping" \
  /clock /tf /tf_static /robot_1/scan_raw /robot_1/imu/data_raw /robot_1/odom &
rosbag_pid=$!
ros2 launch mentorpi_slam mapping.launch.py &
slam_pid=$!

while kill -0 "$slam_pid" 2>/dev/null; do
  sleep 1
done

if wait "$slam_pid"; then
  slam_pid=''
  finalize_session
else
  status=$?
  stop_recording_and_slam || true
  exit "$status"
fi
