#!/usr/bin/env bash
set -euo pipefail

rosbag_root="${ROSBAG_ROOT:-/rosbag}"

mkdir -p "$rosbag_root"
chown ros:ros "$rosbag_root"

exec runuser -u ros --preserve-environment -- \
  /usr/local/bin/mentorpi-rosbag-recorder
