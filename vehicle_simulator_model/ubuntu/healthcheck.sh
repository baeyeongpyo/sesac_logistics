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

check_adapter_manager() {
  if ! pgrep -f 'simulation_manager.py' >/dev/null; then
    health_error adapter 'check=simulation_manager reason=not_running'
    return 1
  fi
  printf 'mentorpi health=adapter status=ok manager=simulation_manager\n'
}

case "${1:-}" in
  server)
    check_server
    ;;
  adapter)
    check_adapter_manager
    ;;
  *)
    health_error unknown "reason=usage expected=server-or-adapter"
    exit 64
    ;;
esac
