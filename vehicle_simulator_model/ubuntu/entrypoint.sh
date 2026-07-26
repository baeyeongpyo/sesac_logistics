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

exec "$@"
