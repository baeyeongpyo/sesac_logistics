#!/usr/bin/env bash
set -euo pipefail

: "${MAP_YAML:?MAP_YAML is required}"
: "${MAP_USE_SIM_TIME:=false}"

# ROS 2 Humble setup scripts reference optional variables under `set -u`.
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u

exec ros2 launch mentorpi_map_server map_server.launch.py \
  "map_yaml:=${MAP_YAML}" \
  "use_sim_time:=${MAP_USE_SIM_TIME}"
