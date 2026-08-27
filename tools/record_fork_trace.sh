#!/usr/bin/env bash
set -u

trace_root="${1:-$PWD/fork_traces}"
trace_dir="$trace_root/fork_trace_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$trace_dir"

stamp_stream() {
    while IFS= read -r trace_line; do
        printf '%s %s\n' "$(date --iso-8601=ns)" "$trace_line"
    done
}

ros_echo() {
    topic_name="$1"
    docker exec -u ubuntu IntelPi bash -lc "
        source /opt/ros/humble/setup.bash
        source /home/ubuntu/ros2_ws/install/setup.bash
        export ROS_DOMAIN_ID=215
        ros2 topic echo '$topic_name' --field data
    "
}

(ros_echo /fork/command | stamp_stream) \
    > "$trace_dir/fork_command.log" &
command_pid=$!

(ros_echo /robot_1/fork/state | stamp_stream) \
    > "$trace_dir/fork_state.log" &
state_pid=$!

(ros_echo /robot_1/auto_dock/status | stamp_stream) \
    > "$trace_dir/auto_dock_status.log" &
status_pid=$!

(
    while true; do
        printf '%s ' "$(date --iso-8601=ns)"
        pinctrl get 17,18,22,27 | tr '\n' ' '
        printf '\n'
        sleep 0.05
    done
) > "$trace_dir/gpio.log" &
gpio_pid=$!

cleanup_trace() {
    trap - EXIT INT TERM
    kill "$command_pid" "$state_pid" "$status_pid" "$gpio_pid" \
        2>/dev/null || true
    wait "$command_pid" "$state_pid" "$status_pid" "$gpio_pid" \
        2>/dev/null || true
    printf '\n저장 위치: %s\n' "$trace_dir"
}
trap cleanup_trace EXIT INT TERM

printf '기록 중입니다. FORK UP을 한 번 누른 뒤 Ctrl+C를 누르세요.\n'
wait
