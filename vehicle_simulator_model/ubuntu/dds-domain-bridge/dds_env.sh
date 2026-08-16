#!/usr/bin/env bash
# Resolve a Discovery Server locator and optionally create a temporary Super Client profile.

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

configure_dds_discovery() {
  local discovery_host="${DDS_DISCOVERY_HOST:?DDS_DISCOVERY_HOST is required}"
  local discovery_port="${DDS_DISCOVERY_PORT:-11811}"
  local discovery_ip='' candidate

  if [[ ! "$discovery_port" =~ ^[0-9]+$ ]] \
    || ((10#$discovery_port < 1 || 10#$discovery_port > 65535)); then
    printf 'dds-domain-bridge discovery_error=invalid_port discovery_port=%s\n' \
      "$discovery_port" >&2
    return 71
  fi

  if valid_ipv4 "$discovery_host"; then
    discovery_ip="$discovery_host"
  else
    while IFS= read -r candidate; do
      if valid_ipv4 "$candidate"; then
        discovery_ip="$candidate"
        break
      fi
    done < <(getent ahostsv4 "$discovery_host" 2>/dev/null | awk '{print $1}' | sort -u)
  fi

  if [[ -z "$discovery_ip" ]]; then
    printf 'dds-domain-bridge discovery_error=resolve_failed discovery_host=%s\n' \
      "$discovery_host" >&2
    return 72
  fi

  export ROS_DISCOVERY_SERVER="${discovery_ip}:${discovery_port}"

  if [[ "${DDS_SUPER_CLIENT:-0}" == 1 ]]; then
    configure_dds_super_client "$discovery_ip" "$discovery_port"
  fi
}

configure_dds_super_client() {
  local discovery_ip="$1"
  local discovery_port="$2"
  local profile="${DDS_SUPER_CLIENT_PROFILE:-/tmp/dds-domain-bridge-super-client-${BASHPID:-$$}.xml}"

  umask 077
  printf '%s\n' \
    '<?xml version="1.0" encoding="UTF-8" ?>' \
    '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">' \
    '  <participant profile_name="dds_domain_bridge_super_client" is_default_profile="true">' \
    '    <rtps><builtin><discovery_config>' \
    '      <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>' \
    '      <discoveryServersList>' \
    '        <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">' \
    '          <metatrafficUnicastLocatorList><locator><udpv4>' \
    "            <address>${discovery_ip}</address>" \
    "            <port>${discovery_port}</port>" \
    '          </udpv4></locator></metatrafficUnicastLocatorList>' \
    '        </RemoteServer>' \
    '      </discoveryServersList>' \
    '    </discovery_config></builtin></rtps>' \
    '  </participant>' \
    '</profiles>' > "$profile"

  DDS_SUPER_CLIENT_PROFILE="$profile"
  export DDS_SUPER_CLIENT_PROFILE
  export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
  export FASTDDS_DEFAULT_PROFILES_FILE="$profile"
  unset ROS_DISCOVERY_SERVER
}

configure_dds_discovery
