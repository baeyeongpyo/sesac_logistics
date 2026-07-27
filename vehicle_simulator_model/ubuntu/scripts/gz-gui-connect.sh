#!/usr/bin/env bash
set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo 'Usage: gz-gui-connect.sh <server-ip> <client-ip>' >&2
  exit 2
fi

server_ip="$1"
client_ip="$2"
bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export GZ_IP="$client_ip"
export GZ_RELAY="$server_ip"
export GZ_PARTITION="${GZ_PARTITION:-mentorpi-sim}"
export GZ_SIM_RESOURCE_PATH="$bundle_dir/ros2_ws/src${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

if ! gz sim --versions | grep -Fxq '8.14.0'; then
  echo 'Gazebo Sim 8.14.0 is required on the GUI client' >&2
  exit 3
fi

topics="$(gz topic -l)"
if ! grep -Fxq '/world/mentorpi_warehouse/stats' <<<"$topics"; then
  echo "Gazebo server is not reachable at ${server_ip} on partition ${GZ_PARTITION}" >&2
  exit 4
fi

exec gz sim --force-version 8 -g
