#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /opt/mentorpi_ws/install/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

printf 'mentorpi service=%s image_version=%s session_id=%s robot_ids=%s\n' \
  "${SERVICE_NAME:-unknown}" \
  "${IMAGE_VERSION:-development}" \
  "${SESSION_ID:-none}" \
  "${ROBOT_IDS:-robot_1,robot_2}"

valid_ipv4() {
  local candidate="$1"
  local first second third fourth octet

  [[ "$candidate" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || return 1
  IFS='.' read -r first second third fourth <<<"$candidate"
  for octet in "$first" "$second" "$third" "$fourth"; do
    ((10#$octet <= 255)) || return 1
  done
  ((10#$first >= 1 && 10#$first <= 223)) || return 1
}

if [[ -n "${GZ_RELAY_HOST:-}" ]]; then
  readonly GZ_RELAY_RESOLVE_ATTEMPTS=20
  relay_ip=''
  for ((attempt = 1; attempt <= GZ_RELAY_RESOLVE_ATTEMPTS; attempt++)); do
    if relay_candidates="$(
      getent ahostsv4 "$GZ_RELAY_HOST" 2>/dev/null \
        | awk '{print $1}' \
        | sort -u
    )"; then
      while IFS= read -r relay_candidate; do
        if valid_ipv4 "$relay_candidate"; then
          relay_ip="$relay_candidate"
          break
        fi
      done <<<"$relay_candidates"
    fi

    [[ -n "$relay_ip" ]] && break
    ((attempt < GZ_RELAY_RESOLVE_ATTEMPTS)) && sleep 1
  done

  if [[ -z "$relay_ip" ]]; then
    printf 'mentorpi relay_error=resolve_failed relay_host=%s attempts=%s\n' \
      "$GZ_RELAY_HOST" "$GZ_RELAY_RESOLVE_ATTEMPTS" >&2
    exit 70
  fi

  export GZ_RELAY="$relay_ip"
  printf 'mentorpi relay_host=%s relay_target=%s\n' \
    "$GZ_RELAY_HOST" "$GZ_RELAY"
fi

exec "$@"
