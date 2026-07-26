#!/usr/bin/env bash
# Resolve the Docker DNS name into the numeric locator required by ROS Humble Fast DDS.

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
  local discovery_host="${DDS_DISCOVERY_HOST:-}"
  local discovery_port="${DDS_DISCOVERY_PORT:-11811}"
  local discovery_ip='' discovery_candidates discovery_candidate attempt
  local resolve_attempts=20

  [[ -n "$discovery_host" ]] || return 0
  if [[ ! "$discovery_port" =~ ^[0-9]+$ ]] \
      || ((10#$discovery_port < 1 || 10#$discovery_port > 65535)); then
    printf 'mentorpi discovery_error=invalid_port discovery_port=%s\n' \
      "$discovery_port" >&2
    return 71
  fi

  for ((attempt = 1; attempt <= resolve_attempts; attempt++)); do
    if discovery_candidates="$(
      getent ahostsv4 "$discovery_host" 2>/dev/null \
        | awk '{print $1}' \
        | sort -u
    )"; then
      while IFS= read -r discovery_candidate; do
        if valid_ipv4 "$discovery_candidate"; then
          discovery_ip="$discovery_candidate"
          break
        fi
      done <<<"$discovery_candidates"
    fi

    [[ -n "$discovery_ip" ]] && break
    ((attempt < resolve_attempts)) && sleep 1
  done

  if [[ -z "$discovery_ip" ]]; then
    printf 'mentorpi discovery_error=resolve_failed discovery_host=%s attempts=%s\n' \
      "$discovery_host" "$resolve_attempts" >&2
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
  local profile="${DDS_SUPER_CLIENT_PROFILE:-/tmp/mentorpi-fastdds-super-client-${BASHPID:-$$}.xml}"

  if [[ "$profile" != /* || "$profile" =~ [[:cntrl:]] ]]; then
    printf 'mentorpi discovery_error=invalid_super_client_profile profile=%s\n' \
      "$profile" >&2
    return 73
  fi

  umask 077
  # Server ID 0 created by `fastdds discovery -i 0` has this stable GUID prefix.
  # The diagnostic participant must be a Super Client to receive the complete
  # Discovery Server v2 graph used by ros2 CLI introspection.
  printf '%s\n' \
    '<?xml version="1.0" encoding="UTF-8" ?>' \
    '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">' \
    '  <participant profile_name="mentorpi_super_client" is_default_profile="true">' \
    '    <rtps>' \
    '      <builtin>' \
    '        <discovery_config>' \
    '          <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>' \
    '          <discoveryServersList>' \
    '            <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">' \
    '              <metatrafficUnicastLocatorList>' \
    '                <locator>' \
    '                  <udpv4>' \
    "                    <address>${discovery_ip}</address>" \
    "                    <port>${discovery_port}</port>" \
    '                  </udpv4>' \
    '                </locator>' \
    '              </metatrafficUnicastLocatorList>' \
    '            </RemoteServer>' \
    '          </discoveryServersList>' \
    '        </discovery_config>' \
    '      </builtin>' \
    '    </rtps>' \
    '  </participant>' \
    '</profiles>' > "$profile"

  DDS_SUPER_CLIENT_PROFILE="$profile"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
  export FASTDDS_DEFAULT_PROFILES_FILE="$profile"
  unset ROS_DISCOVERY_SERVER
}

configure_dds_discovery
