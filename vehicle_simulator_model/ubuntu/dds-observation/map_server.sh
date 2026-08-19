#!/usr/bin/env bash
set -euo pipefail

: "${MAP_YAML:?MAP_YAML is required}"
: "${MAP_USE_SIM_TIME:=false}"

# ROS 2 Humble의 generated setup 파일은 `set -u` 환경에서 설정되지 않은
# AMENT_TRACE_SETUP_FILES를 참조할 수 있다. setup을 불러오는 범위에서만
# nounset을 해제하고, 이후 실행 구간에서는 다시 활성화한다.
set +u
source /opt/ros/humble/setup.bash
set -u
cd /ws
colcon build --symlink-install --packages-select mentorpi_map_server
set +u
source /ws/install/setup.bash
set -u

exec ros2 launch mentorpi_map_server map_server.launch.py \
  "map_yaml:=${MAP_YAML}" \
  "use_sim_time:=${MAP_USE_SIM_TIME}"
