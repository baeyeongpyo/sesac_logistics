#!/usr/bin/env bash
# Execute one docker -> pallet 3 mission through the local Vehicle Command API.

set -Eeuo pipefail

VEHICLE_COMMAND_API_URL="${VEHICLE_COMMAND_API_URL:-http://127.0.0.1:8082}"
POLL_INTERVAL_SEC="${PALLET3_POLL_INTERVAL_SEC:-0.5}"
STEP_TIMEOUT_SEC="${PALLET3_STEP_TIMEOUT_SEC:-120}"
EVENT_RETRY_ATTEMPTS=11
HTTP_RESPONSE_FILE=""
HTTP_RESPONSE=""
HTTP_STATUS=""
MISSION_SUCCEEDED=0
ABORTING=0

OPERATION_ID=""
ROBOT_ID=""
PICK_MODE=""
FLEET_MANAGER_URL=""

usage() {
  echo "usage: $0 --operation-id UUID --robot-id ID --pick-mode auto_dock|manual --fleet-manager-url URL" >&2
}

require_value() {
  if [[ $# -lt 2 || -z "$2" ]]; then
    usage
    exit 2
  fi
}

parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --operation-id)
        require_value "$@"
        OPERATION_ID="$2"
        shift 2
        ;;
      --robot-id)
        require_value "$@"
        ROBOT_ID="$2"
        shift 2
        ;;
      --pick-mode)
        require_value "$@"
        PICK_MODE="$2"
        shift 2
        ;;
      --fleet-manager-url)
        require_value "$@"
        FLEET_MANAGER_URL="${2%/}"
        shift 2
        ;;
      *)
        usage
        exit 2
        ;;
    esac
  done

  if [[ -z "$OPERATION_ID" || -z "$ROBOT_ID" || -z "$PICK_MODE" || -z "$FLEET_MANAGER_URL" ]]; then
    usage
    exit 2
  fi
  if [[ "$PICK_MODE" != "auto_dock" && "$PICK_MODE" != "manual" ]]; then
    echo "pick mode must be auto_dock or manual" >&2
    exit 2
  fi
}

request_json() {
  local method="$1"
  local url="$2"
  local payload="$3"
  local -a curl_args

  if [[ -n "$HTTP_RESPONSE_FILE" ]]; then
    rm -f -- "$HTTP_RESPONSE_FILE"
  fi
  HTTP_RESPONSE_FILE="$(mktemp "${TMPDIR:-/tmp}/pallet3-mission-http.XXXXXX")"
  curl_args=(
    -sS
    --connect-timeout 2
    --max-time 5
    -o "$HTTP_RESPONSE_FILE"
    -w '%{http_code}'
    -X "$method"
    -H 'Content-Type: application/json'
  )
  if [[ -n "$payload" ]]; then
    curl_args+=(--data "$payload")
  fi
  if ! HTTP_STATUS="$(curl "${curl_args[@]}" "$url")"; then
    HTTP_RESPONSE=""
    return 1
  fi
  HTTP_RESPONSE="$(<"$HTTP_RESPONSE_FILE")"
  [[ "$HTTP_STATUS" =~ ^2[0-9][0-9]$ ]]
}

wait_vehicle_reply() {
  local expected_detail="$1"
  local association="$2"
  local deadline=$((SECONDS + STEP_TIMEOUT_SEC))
  local detail=""
  local observed_operation=""

  while (( SECONDS < deadline )); do
    if request_json GET "${VEHICLE_COMMAND_API_URL}/v1/operation-status" ""; then
      if detail="$(jq -r '.detail // ""' <<<"$HTTP_RESPONSE" 2>/dev/null)" \
        && observed_operation="$(jq -r '.operation_id // ""' <<<"$HTTP_RESPONSE" 2>/dev/null)"; then
        if [[ "$detail" == "$expected_detail" ]]; then
          if [[ "$association" == "operation" && "$observed_operation" == "$OPERATION_ID" ]]; then
            return 0
          fi
          if [[ "$association" == "released" && "$observed_operation" != "$OPERATION_ID" ]]; then
            return 0
          fi
        fi
      fi
    fi
    sleep "$POLL_INTERVAL_SEC"
  done
  echo "timed out waiting for vehicle reply: ${expected_detail}" >&2
  return 1
}

report_mission_event() {
  local event_type="$1"
  local idempotency_key="${OPERATION_ID}:$(tr '[:upper:]' '[:lower:]' <<<"${event_type%%_*}")"
  local event_url="${FLEET_MANAGER_URL}/api/v1/vehicles/${ROBOT_ID}/missions/pallet3/${OPERATION_ID}/events"
  local payload
  payload="$(printf '{"event_type":"%s","idempotency_key":"%s"}' "$event_type" "$idempotency_key")"

  for ((attempt = 1; attempt <= EVENT_RETRY_ATTEMPTS; attempt++)); do
    if request_json POST "$event_url" "$payload"; then
      return 0
    fi
    if (( attempt < EVENT_RETRY_ATTEMPTS )); then
      sleep "$POLL_INTERVAL_SEC"
    fi
  done
  echo "Fleet Manager event was unavailable for 5 seconds: ${event_type}" >&2
  abort_mission
}

run_auto_dock_pick() {
  local payload
  payload="$(printf '{"operation_id":"%s","operation":"PICK","product_type":"NORMAL","location":"DOCK_1","target":{"type":"NEAREST"}}' "$OPERATION_ID")"
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/auto-dock" "$payload" \
    || abort_mission
  wait_vehicle_reply "AUTO_DOCK_PICK_COMPLETED" operation || abort_mission
}

run_manual_pick() {
  local payload
  payload="$(printf '{"operation_id":"%s"}' "$OPERATION_ID")"
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/fork/up" "$payload" \
    || abort_mission
  wait_vehicle_reply "FORK_UP_COMPLETE" operation || abort_mission
}

complete_pick() {
  report_mission_event "PICK_COMPLETED"
  if [[ "$PICK_MODE" == "manual" ]]; then
    request_json POST "${VEHICLE_COMMAND_API_URL}/v1/missions/pallet3/${OPERATION_ID}/picked" '{}' \
      || abort_mission
  fi
}

drive_to_p3() {
  local payload
  payload="$(printf '{"operation_id":"%s","purpose":"PLACE","waypoints":[{"frame_id":"map","x":-0.440,"y":-0.900,"yaw":0.0},{"frame_id":"map","x":-0.440,"y":-1.690,"yaw":-1.5707963267948966},{"frame_id":"map","x":-0.440,"y":-2.380,"yaw":-1.5707963267948966}]}' "$OPERATION_ID")"
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/navigation/waypoints" "$payload" \
    || abort_mission
  wait_vehicle_reply "NAVIGATION_SUCCEEDED" operation || abort_mission
}

lower_fork() {
  local payload
  payload="$(printf '{"operation_id":"%s"}' "$OPERATION_ID")"
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/fork/down" "$payload" \
    || abort_mission
  wait_vehicle_reply "FORK_DOWN_COMPLETE" operation || abort_mission
}

complete_place() {
  report_mission_event "PLACE_READY"
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/missions/pallet3/${OPERATION_ID}/placed" '{}' \
    || abort_mission
}

reverse_after_place() {
  local payload='{"linear_x":-0.18,"linear_y":0.0,"angular_z":0.0,"hold_ms":1000}'
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/cmd-vel" "$payload" || abort_mission
  wait_vehicle_reply "MANUAL_COMMAND_EXPIRED" released || abort_mission
}

return_to_docker() {
  local payload='{"waypoints":[{"frame_id":"map","x":-0.440,"y":-1.690,"yaw":0.0},{"frame_id":"map","x":-0.440,"y":-0.900,"yaw":0.0},{"frame_id":"map","x":0.085,"y":-0.905,"yaw":0.0}]}'
  request_json POST "${VEHICLE_COMMAND_API_URL}/v1/navigation/waypoints" "$payload" \
    || abort_mission
  wait_vehicle_reply "NAVIGATION_SUCCEEDED" released || abort_mission
  report_mission_event "RETURN_COMPLETED"
}

best_effort_stop() {
  curl -sS --connect-timeout 2 --max-time 5 -o /dev/null \
    -X POST -H 'Content-Type: application/json' --data '{}' \
    "${VEHICLE_COMMAND_API_URL}/v1/stop" >/dev/null 2>&1 || true
}

abort_mission() {
  if (( ABORTING )); then
    exit 1
  fi
  ABORTING=1
  best_effort_stop
  exit 1
}

on_exit() {
  local exit_code="$1"
  if (( exit_code != 0 )) && (( ! ABORTING )) && (( ! MISSION_SUCCEEDED )); then
    ABORTING=1
    best_effort_stop
  fi
  if [[ -n "$HTTP_RESPONSE_FILE" ]]; then
    rm -f -- "$HTTP_RESPONSE_FILE"
  fi
}

main() {
  case "$PICK_MODE" in
    auto_dock)
      run_auto_dock_pick
      ;;
    manual)
      run_manual_pick
      ;;
  esac
  complete_pick
  drive_to_p3
  lower_fork
  complete_place
  reverse_after_place
  return_to_docker
  MISSION_SUCCEEDED=1
}

trap 'on_exit "$?"' EXIT
trap 'abort_mission' INT TERM

parse_args "$@"
command -v curl >/dev/null 2>&1 || { echo 'curl is required' >&2; exit 2; }
command -v jq >/dev/null 2>&1 || { echo 'jq is required' >&2; exit 2; }
main
