#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "$BUNDLE_DIR/compose.yaml")

usage() {
  cat <<'EOF'
Usage: ./run.sh <command>

Commands:
  build     Build the Ubuntu MentorPi image.
  shell     Open a ROS-enabled shell in the source-mounted container.
  headless  Start the two-robot simulation without a GUI.
  gui       Render on the Ubuntu GPU and display it through trusted SSH X11.
  test      Build both ROS packages and check their installed prefixes.
  fork-up [headless|gui]
            Publish the robot_1 fork target height of 0.11 m on the matching network.
EOF
}

case "${1:-}" in
  build)
    "${COMPOSE[@]}" build
    ;;
  shell)
    "${COMPOSE[@]}" run --rm mentorpi-sim bash
    ;;
  headless)
    "${COMPOSE[@]}" run --rm mentorpi-sim \
      ros2 launch mentorpi_gz_sim two_robot_sim.launch.py headless:=true
    ;;
  gui)
    if [[ -z "${DISPLAY:-}" ]]; then
      echo 'DISPLAY is empty; reconnect to the Ubuntu server with ssh -Y.' >&2
      exit 2
    fi
    auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
    if [[ ! -r "$auth_file" ]]; then
      echo "Xauthority is not readable: $auth_file" >&2
      exit 3
    fi
    if [[ ! -e /dev/dri/renderD128 ]]; then
      echo 'Ubuntu GPU render node is unavailable: /dev/dri/renderD128' >&2
      exit 4
    fi
    export XAUTHORITY="$auth_file"
    export RENDER_GID="${RENDER_GID:-$(stat -c '%g' /dev/dri/renderD128)}"
    "${COMPOSE[@]}" run --rm --no-deps mentorpi-gui bash -lc \
      '/opt/VirtualGL/bin/vglrun -d egl -c proxy ros2 launch mentorpi_gz_sim two_robot_sim.launch.py headless:=false'
    ;;
  test)
    "${COMPOSE[@]}" run --rm mentorpi-sim bash -lc \
      'colcon build --symlink-install && source install/setup.bash && ros2 pkg prefix mentorpi_description && ros2 pkg prefix mentorpi_gz_sim'
    ;;
  fork-up)
    service=mentorpi-sim
    if [[ "${2:-headless}" == 'gui' ]]; then
      service=mentorpi-host
    elif [[ "${2:-headless}" != 'headless' ]]; then
      echo 'fork-up mode must be headless or gui' >&2
      exit 2
    fi
    "${COMPOSE[@]}" run --rm "$service" \
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
