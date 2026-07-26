#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export MENTORPI_IMAGE="${MENTORPI_IMAGE:-mentorpi-sim:harmonic}"
COMPOSE=(docker compose -f "$BUNDLE_DIR/compose.yaml")
ROS_SETUP='source /opt/ros/humble/setup.bash && source /opt/mentorpi_ws/install/setup.bash'

prepare_gpu() {
  local render_node=''
  local render_gid
  local -a render_nodes

  if [[ "$(uname -s)" != 'Linux' ]]; then
    echo 'GPU mode requires native Linux with a DRI render node.' >&2
    exit 2
  fi

  shopt -s nullglob
  render_nodes=(/dev/dri/renderD*)
  shopt -u nullglob
  if ((${#render_nodes[@]} == 0)); then
    echo 'GPU mode requires /dev/dri/renderD*.' >&2
    exit 3
  fi
  for render_node in "${render_nodes[@]}"; do
    [[ -r "$render_node" ]] && break
    render_node=''
  done
  if [[ -z "$render_node" ]]; then
    echo 'No readable /dev/dri/renderD* node is available.' >&2
    exit 4
  fi

  render_gid="$(stat -c '%g' "$render_node")"
  if [[ ! "$render_gid" =~ ^[0-9]+$ ]]; then
    echo "Render node GID is not numeric: $render_gid" >&2
    exit 5
  fi
  export RENDER_GID="$render_gid"
  printf 'mentorpi gpu_render_node=%s render_gid=%s\n' "$render_node" "$RENDER_GID"
}

validate_session_id() {
  local session_id="${1:-}"
  if [[ ! "$session_id" =~ ^[A-Za-z0-9._-]+$ \
    || "$session_id" == '.' || "$session_id" == '..' ]]; then
    echo 'session ID may contain only A-Z, a-z, 0-9, period, underscore, and hyphen, but not . or ..' >&2
    return 2
  fi
}

validate_mapping_stop_timeout() {
  if [[ ! "$MAPPING_STOP_TIMEOUT_SECONDS" =~ ^[0-9]+$ ]]; then
    echo 'MAPPING_STOP_TIMEOUT_SECONDS must be a non-negative integer' >&2
    return 2
  fi
}

wait_for_mapper_exit() {
  local mapper_id="$1"
  local deadline=$((SECONDS + MAPPING_STOP_TIMEOUT_SECONDS))
  local mapper_state

  while :; do
    mapper_state="$(
      docker inspect --format '{{.State.Running}} {{.State.ExitCode}}' \
        "$mapper_id" 2>/dev/null || true
    )"
    if [[ "$mapper_state" =~ ^false[[:space:]]+([0-9]+)$ ]]; then
      printf '%s\n' "${BASH_REMATCH[1]}"
      return 0
    fi
    if [[ "$mapper_state" != true\ * ]]; then
      echo 'slam-mapper container state is unavailable before finalization completed' >&2
      return 2
    fi
    if ((SECONDS >= deadline)); then
      echo "timed out waiting ${MAPPING_STOP_TIMEOUT_SECONDS}s for slam-mapper finalization" >&2
      return 1
    fi
    sleep 1
  done
}

usage() {
  cat <<'EOF'
Usage: ./run.sh <command>

Commands:
  build              Build the immutable linux/amd64 MentorPi image.
  sim-up [gpu]       Start Gazebo and the ROS adapter in the background.
  down               Stop and remove the simulation services.
  logs               Follow Gazebo and adapter service logs.
  topics             List ROS topics from the running adapter.
  test               Run static checks and validate the runtime image.
  fork-up             Publish the robot_1 fork target height of 0.11 m.
  mapping-up <id>     Start a one-shot SLAM mapping session.
  mapping-stop        Send SIGINT and wait for safe SLAM finalization.
  mapping-status <id> Verify a published mapping session's checksums.
EOF
}

case "${1:-}" in
  build)
    docker build --platform "${TARGET_PLATFORM:-linux/amd64}" \
      --tag "$MENTORPI_IMAGE" "$BUNDLE_DIR"
    ;;
  sim-up)
    if [[ "${2:-}" == 'gpu' ]]; then
      prepare_gpu
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
  topics)
    "${COMPOSE[@]}" exec sim-adapter bash -lc \
      "$ROS_SETUP && ros2 topic list"
    ;;
  test)
    printf 'mentorpi test stage=host-static\n'
    python3 "$BUNDLE_DIR/test/test_bundle.py" -v
    python3 \
      "$BUNDLE_DIR/ros2_ws/src/mentorpi_description/test/test_original_model.py" -v
    python3 -m unittest discover \
      -s "$BUNDLE_DIR/ros2_ws/src/mentorpi_gz_sim/test" \
      -p 'test_harmonic_launch_contract.py' -v

    printf 'mentorpi test stage=compose-config\n'
    "${COMPOSE[@]}" config --quiet
    RENDER_GID="${RENDER_GID:-0}" \
      "${COMPOSE[@]}" -f "$BUNDLE_DIR/compose.gpu.yaml" config --quiet

    printf 'mentorpi test stage=runtime-ctest\n'
    "${COMPOSE[@]}" run --rm --no-deps gazebo-server bash -lc \
      "$ROS_SETUP && cd /opt/mentorpi_ws && \
       gz sim --versions && \
       ros2 pkg prefix mentorpi_description && \
       ros2 pkg prefix mentorpi_gz_sim && \
       colcon test --packages-select mentorpi_gz_sim --event-handlers console_direct+ && \
       colcon test-result --verbose"
    ;;
  fork-up)
    adapter_id="$("${COMPOSE[@]}" ps -q sim-adapter)"
    if [[ -z "$adapter_id" ]]; then
      echo 'sim-adapter is not running; start it with ./run.sh sim-up.' >&2
      exit 3
    fi
    adapter_health="$(
      docker inspect --format '{{.State.Health.Status}}' "$adapter_id" 2>/dev/null || true
    )"
    if [[ "$adapter_health" != 'healthy' ]]; then
      echo "sim-adapter is not healthy: ${adapter_health:-unknown}" >&2
      exit 4
    fi
    "${COMPOSE[@]}" exec -T sim-adapter bash -lc \
      "$ROS_SETUP && timeout 10 ros2 topic pub --once /robot_1/fork/command std_msgs/msg/Float64 '{data: 0.11}'"
    ;;
  mapping-up)
    if [[ "$#" -ne 2 ]]; then
      echo 'mapping-up requires exactly one session ID' >&2
      exit 2
    fi
    validate_session_id "$2"
    export SESSION_ID="$2"
    export IMAGE_VERSION="${IMAGE_VERSION:-mentorpi-sim:harmonic}"
    export GIT_COMMIT="${GIT_COMMIT:-$(git -C "$BUNDLE_DIR" rev-parse HEAD)}"
    export WORLD_VERSION="${WORLD_VERSION:-warehouse-v1}"
    export MODEL_VERSION="${MODEL_VERSION:-mentorpi-m1-v1}"
    export TF_CALIBRATION_VERSION="${TF_CALIBRATION_VERSION:-ground-truth-v1}"
    "${COMPOSE[@]}" --profile mapping up -d \
      gazebo-server sim-adapter slam-mapper
    ;;
  mapping-stop)
    if [[ "$#" -ne 1 ]]; then
      echo 'mapping-stop does not accept arguments' >&2
      exit 2
    fi
    MAPPING_STOP_TIMEOUT_SECONDS="${MAPPING_STOP_TIMEOUT_SECONDS:-90}"
    validate_mapping_stop_timeout
    mapper_id="$("${COMPOSE[@]}" ps -q --all slam-mapper)"
    if [[ -z "$mapper_id" ]]; then
      echo 'slam-mapper is not running; no mapping finalization is in progress' >&2
      exit 3
    fi
    kill_failed=0
    if ! "${COMPOSE[@]}" kill -s SIGINT slam-mapper; then kill_failed=1; fi
    if ! mapper_exit_code="$(wait_for_mapper_exit "$mapper_id")"; then
      if ((kill_failed)); then
        echo 'failed to send SIGINT to slam-mapper and its finalization state is unknown; services remain running' >&2
        exit 4
      fi
      echo 'mapping finalization status is unknown; leaving mapper and simulation services running' >&2
      exit 5
    fi
    if [[ "$mapper_exit_code" == 0 ]]; then
      echo 'mapping finalization succeeded'
      finalization_status=0
    else
      echo "mapping finalization failed (slam-mapper exit code ${mapper_exit_code})" >&2
      finalization_status="$mapper_exit_code"
    fi
    if ! "${COMPOSE[@]}" stop gazebo-server sim-adapter; then
      echo 'mapping finalization completed, but simulation service cleanup failed' >&2
      [[ "$finalization_status" == 0 ]] && exit 6
    fi
    exit "$finalization_status"
    ;;
  mapping-status)
    if [[ "$#" -ne 2 ]]; then
      echo 'mapping-status requires exactly one session ID' >&2
      exit 2
    fi
    validate_session_id "$2"
    SESSION_ID="$2" "${COMPOSE[@]}" --profile mapping run --rm --no-deps slam-inspector \
      bash -lc '
        set -euo pipefail
        session_dir="/slam-data/${SESSION_ID}"
        if [[ ! -d "$session_dir" ]]; then
          echo "published mapping session not found: ${SESSION_ID}" >&2
          exit 3
        fi
        if [[ ! -f "$session_dir/checksums.sha256" ]]; then
          echo "published mapping session has no checksums: ${SESSION_ID}" >&2
          exit 4
        fi
        find "$session_dir" -type f -print | sort
        cd "$session_dir"
        sha256sum -c checksums.sha256
      '
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
