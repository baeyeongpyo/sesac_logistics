#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

if [[ "${1:-}" == fastdds && "${2:-}" == discovery ]]; then
  exec "$@"
fi

validate_domain_id() {
  local name="$1"
  local value="$2"

  if [[ ! "$value" =~ ^[0-9]+$ ]] || ((10#$value > 232)); then
    printf 'dds-domain-bridge config_error=invalid_domain_id name=%s value=%s\n' \
      "$name" "$value" >&2
    exit 64
  fi
}

: "${CENTRAL_DOMAIN_ID:?CENTRAL_DOMAIN_ID is required}"
: "${ROBOT_1_DOMAIN_ID:?ROBOT_1_DOMAIN_ID is required}"
: "${ROBOT_2_DOMAIN_ID:?ROBOT_2_DOMAIN_ID is required}"
: "${DDS_DISCOVERY_HOST:?DDS_DISCOVERY_HOST is required}"

validate_domain_id CENTRAL_DOMAIN_ID "$CENTRAL_DOMAIN_ID"
validate_domain_id ROBOT_1_DOMAIN_ID "$ROBOT_1_DOMAIN_ID"
validate_domain_id ROBOT_2_DOMAIN_ID "$ROBOT_2_DOMAIN_ID"

if [[ "$CENTRAL_DOMAIN_ID" == "$ROBOT_1_DOMAIN_ID" \
  || "$CENTRAL_DOMAIN_ID" == "$ROBOT_2_DOMAIN_ID" \
  || "$ROBOT_1_DOMAIN_ID" == "$ROBOT_2_DOMAIN_ID" ]]; then
  printf 'dds-domain-bridge config_error=duplicate_domain_id\n' >&2
  exit 65
fi

source /usr/local/bin/dds-domain-bridge-env

envsubst '$CENTRAL_DOMAIN_ID $ROBOT_1_DOMAIN_ID $ROBOT_2_DOMAIN_ID' \
  < /etc/dds-domain-bridge/bridge.yaml.template \
  > /tmp/dds-domain-bridge.yaml

printf 'dds-domain-bridge service=%s central_domain=%s robot_1_domain=%s robot_2_domain=%s\n' \
  "${SERVICE_NAME:-dds-domain-bridge}" \
  "$CENTRAL_DOMAIN_ID" "$ROBOT_1_DOMAIN_ID" "$ROBOT_2_DOMAIN_ID"

exec "$@"
