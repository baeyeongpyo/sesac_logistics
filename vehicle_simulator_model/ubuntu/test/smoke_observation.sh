#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != lan || "$#" -ne 1 ]]; then
  echo 'Usage: smoke_observation.sh lan' >&2
  exit 2
fi

: "${GZ_SERVER_IP:?set GZ_SERVER_IP}"
: "${GZ_CLIENT_IP:?set GZ_CLIENT_IP}"

bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

check_client() {
  local name="$1"
  local topics
  topics="$(
    GZ_IP="$GZ_CLIENT_IP" \
    GZ_RELAY="$GZ_SERVER_IP" \
    GZ_PARTITION=mentorpi-sim \
    gz topic -l
  )"
  for topic in \
    /world/mentorpi_warehouse/stats \
    /robot_1/scan_raw \
    /robot_2/scan_raw; do
    grep -Fxq "$topic" <<<"$topics" || {
      printf '%s missing %s\n' "$name" "$topic" >&2
      return 1
    }
  done
}

check_client client-a &
pid_a="$!"
check_client client-b &
pid_b="$!"
wait "$pid_a"
wait "$pid_b"

docker compose \
  -f "$bundle_dir/compose.yaml" \
  -f "$bundle_dir/compose.lan.yaml" \
  exec -T gazebo-server /usr/local/bin/mentorpi-healthcheck server
docker compose \
  -f "$bundle_dir/compose.yaml" \
  -f "$bundle_dir/compose.lan.yaml" \
  exec -T sim-adapter /usr/local/bin/mentorpi-healthcheck adapter
