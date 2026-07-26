#!/usr/bin/env bash
set -o pipefail

readonly HEALTH_TIMEOUT_SECONDS=8
readonly STATS_TOPIC=/world/mentorpi_warehouse/stats

health_error() {
  printf 'mentorpi health=%s status=fail %s\n' "${1:-unknown}" "${2:-reason=unknown}" >&2
}

check_server() {
  local stats_output
  local -a iterations

  if ! stats_output="$(
    timeout "$HEALTH_TIMEOUT_SECONDS" gz topic -e -t "$STATS_TOPIC" -n 2 2>&1
  )"; then
    health_error server "check=stats_payload reason=no_two_samples"
    return 1
  fi

  mapfile -t iterations < <(
    awk '$1 == "iterations:" {print $2}' <<<"$stats_output"
  )
  if ((${#iterations[@]} < 2)); then
    health_error server "check=stats_progress reason=missing_iterations samples=${#iterations[@]}"
    return 1
  fi
  if [[ ! "${iterations[0]}" =~ ^[0-9]+$ || ! "${iterations[1]}" =~ ^[0-9]+$ ]]; then
    health_error server "check=stats_progress reason=invalid_iterations"
    return 1
  fi
  if ((iterations[1] <= iterations[0])); then
    health_error server \
      "check=stats_progress reason=not_advancing first=${iterations[0]} second=${iterations[1]}"
    return 1
  fi

  printf 'mentorpi health=server status=ok iterations=%s,%s\n' \
    "${iterations[0]}" "${iterations[1]}"
}

check_ros_payload() {
  local robot="$1"
  local kind="$2"
  local topic="/${robot}/${kind}"
  local payload

  if ! payload="$(
    timeout "$HEALTH_TIMEOUT_SECONDS" ros2 topic echo --once "$topic" 2>&1
  )"; then
    health_error adapter "check=payload robot=$robot topic=$topic reason=no_message"
    return 1
  fi
  if ! grep -q '^header:' <<<"$payload"; then
    health_error adapter "check=payload robot=$robot topic=$topic reason=invalid_message"
    return 1
  fi
}

check_robot_tf() {
  local robot="$1"
  local parent="${robot}/odom"
  local child="${robot}/base_footprint"
  local tf_output

  tf_output="$(
    timeout "$HEALTH_TIMEOUT_SECONDS" \
      ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true
  )"
  if ! grep -q 'Translation:' <<<"$tf_output" \
      || ! grep -q 'Rotation:' <<<"$tf_output"; then
    health_error adapter \
      "check=tf robot=$robot parent=$parent child=$child reason=lookup_failed"
    return 1
  fi
}

check_adapter() {
  local status=0
  local pid
  local -a pids=()

  if ! source /opt/ros/humble/setup.bash; then
    health_error adapter "check=ros_setup reason=humble_setup_failed"
    return 1
  fi
  if ! source /opt/mentorpi_ws/install/setup.bash; then
    health_error adapter "check=ros_setup reason=mentorpi_setup_failed"
    return 1
  fi

  check_ros_payload robot_1 scan_raw & pids+=("$!")
  check_ros_payload robot_1 odom & pids+=("$!")
  check_robot_tf robot_1 & pids+=("$!")
  check_ros_payload robot_2 scan_raw & pids+=("$!")
  check_ros_payload robot_2 odom & pids+=("$!")
  check_robot_tf robot_2 & pids+=("$!")

  for pid in "${pids[@]}"; do
    if ! wait "$pid"; then
      status=1
    fi
  done
  if ((status != 0)); then
    return "$status"
  fi

  printf 'mentorpi health=adapter status=ok robots=robot_1,robot_2\n'
}

case "${1:-}" in
  server)
    check_server
    ;;
  adapter)
    check_adapter
    ;;
  *)
    health_error unknown "reason=usage expected=server-or-adapter"
    exit 64
    ;;
esac
