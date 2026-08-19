#!/usr/bin/env bash
set -euo pipefail

workspace=/ws
mkdir -p "$workspace/build" "$workspace/install" "$workspace/log"
chown -R ros:ros "$workspace/build" "$workspace/install" "$workspace/log"

exec runuser -u ros --preserve-environment -- \
  /usr/local/bin/mentorpi-map-server
