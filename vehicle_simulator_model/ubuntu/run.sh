#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "$BUNDLE_DIR/compose.yaml")

usage() {
  cat <<'EOF'
Usage: ./run.sh <command>

Commands:
  build              Build the immutable linux/amd64 MentorPi image.
  sim-up [gpu]       Start Gazebo and the ROS adapter in the background.
  down               Stop and remove the simulation services.
  logs               Follow Gazebo and adapter service logs.
  test               Run static checks and validate the runtime image.
  fork-up             Publish the robot_1 fork target height of 0.11 m.
EOF
}

case "${1:-}" in
  build)
    "${COMPOSE[@]}" build
    ;;
  sim-up)
    if [[ "${2:-}" == 'gpu' ]]; then
      COMPOSE+=( -f "$BUNDLE_DIR/compose.gpu.yaml" )
    elif [[ -n "${2:-}" ]]; then
      echo 'sim-up accepts only the optional gpu profile' >&2
      exit 2
    fi
    "${COMPOSE[@]}" up -d gazebo-server sim-adapter
    ;;
  down)
    "${COMPOSE[@]}" down
    ;;
  logs)
    "${COMPOSE[@]}" logs -f gazebo-server sim-adapter
    ;;
  test)
    python3 -m unittest \
      "$BUNDLE_DIR/test/test_bundle.py" \
      "$BUNDLE_DIR/ros2_ws/src/mentorpi_description/test/test_original_model.py" \
      "$BUNDLE_DIR/ros2_ws/src/mentorpi_gz_sim/test/test_gz_pose_to_odom.py" \
      "$BUNDLE_DIR/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py" -v
    "${COMPOSE[@]}" run --rm --no-deps gazebo-server bash -lc \
      'gz sim --versions && ros2 pkg prefix mentorpi_description && ros2 pkg prefix mentorpi_gz_sim'
    ;;
  fork-up)
    "${COMPOSE[@]}" run --rm --no-deps sim-adapter \
      ros2 topic pub --once /robot_1/fork/command std_msgs/msg/Float64 '{data: 0.11}'
    ;;
  -h|--help|help|'')
    usage
    ;;
  *)
    echo "Unknown command: $1" >&2
    usage >&2
    exit 2
    ;;
esac
