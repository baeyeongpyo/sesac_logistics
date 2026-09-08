#!/usr/bin/env bash

set -u

publish_stop_commands() {
    if ! command -v ros2 >/dev/null; then
        printf 'Warning: ros2 not found; skipping pre-stop motor command.\n' >&2
        return
    fi

    local zero_twist
    local zero_motors
    zero_twist='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    zero_motors='{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}'

    printf 'Publishing zero velocity and motor commands before shutdown.\n'
    timeout 2 ros2 topic pub -r 20 /controller/cmd_vel \
        geometry_msgs/msg/Twist "$zero_twist" >/dev/null &
    local controller_stop_pid=$!
    timeout 2 ros2 topic pub -r 20 /cmd_vel \
        geometry_msgs/msg/Twist "$zero_twist" >/dev/null &
    local nav_stop_pid=$!
    timeout 2 ros2 topic pub -r 20 /ros_robot_controller/set_motor \
        ros_robot_controller_msgs/msg/MotorsState "$zero_motors" \
        >/dev/null &
    local motor_stop_pid=$!

    wait "$controller_stop_pid" "$nav_stop_pid" "$motor_stop_pid" || true
}

live_pids() {
    local pattern="$1"
    local pid
    local state
    while read -r pid; do
        [[ -z "$pid" || "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
        state="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
        [[ -z "$state" || "$state" == Z* ]] && continue
        printf '%s\n' "$pid"
    done < <(pgrep -f -- "$pattern" || true)
}


# Only signal individual PIDs: a process-group signal would also kill the driver.
signal_patterns() {
    local signal="$1" pattern
    shift
    local -a pids
    for pattern in "$@"; do
        mapfile -t pids < <(live_pids "$pattern")
        ((${#pids[@]})) || continue
        matched=1
        printf 'Stopping (%s): %s (PID %s)\n' "$signal" "$pattern" "${pids[*]}"
        kill -"$signal" "${pids[@]}" 2>/dev/null || true
    done
}

patterns_gone() {
    local pattern failed=0
    local -a pids
    for pattern in "$@"; do
        mapfile -t pids < <(live_pids "$pattern")
        if ((${#pids[@]})); then
            printf 'ERROR: process still alive: %s (PID %s)\n' "$pattern" "${pids[*]}" >&2
            failed=1
        fi
    done
    return "$failed"
}

stop_patterns() {
    local attempt
    signal_patterns INT "$@"
    sleep 3
    signal_patterns TERM "$@"
    for attempt in 1 2 3 4 5; do
        sleep 0.5
        signal_patterns KILL "$@"
    done
    sleep 0.1
    patterns_gone "$@"
}

publish_final_motor_stop() {
    local zero_motors='{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}'
    # Wait for a subscriber, send a finite burst, and retain DDS for delivery.
    # CLI success is NOT a physical wheel-stop acknowledgement.
    timeout 8 ros2 topic pub --times 20 -r 20 \
        --wait-matching-subscriptions 1 --keep-alive 0.5 \
        /ros_robot_controller/set_motor \
        ros_robot_controller_msgs/msg/MotorsState "$zero_motors" >/dev/null
}

shutdown_stack() {
    matched=0
    # A graceful launch shutdown cascades to ALL children, including the motor
    # driver. Kill only the supervisors so they cannot cascade or respawn.
    signal_patterns KILL "${supervisor_patterns[@]}"
    sleep 0.1
    if ! patterns_gone "${supervisor_patterns[@]}"; then
        printf 'ERROR: supervisor shutdown failed; preserving motor communication.\n' >&2
        return 1
    fi
    if ! stop_patterns "${worker_patterns[@]}"; then
        printf 'ERROR: worker shutdown failed; preserving motor communication.\n' >&2
        return 1
    fi
    local -a motors
    mapfile -t motors < <(live_pids "${motor_patterns[0]}")
    if ((${#motors[@]})); then
        printf 'Command producers stopped; sending final direct motor topic zeros.\n'
        if ! publish_final_motor_stop; then
            printf 'WARNING: ROS zero failed; continuing to independent serial stop.\n' >&2
        fi
        stop_patterns "${motor_patterns[@]}" || return 1
    fi
    # Works even if the ROS driver already crashed; no ROS discovery is needed.
    if ! timeout 5 /usr/bin/python3 /home/ubuntu/ros2_ws/tools/motor_direct_stop.py; then
        printf 'ERROR: direct serial stop failed. Use emergency stop / motor power cutoff.\n' >&2
        return 1
    fi
    printf 'Stack stopped; serial zero burst flushed. Verify wheels are stopped.\n'

}

supervisor_patterns=(
    "/opt/ros/humble/bin/ros2 launch mentorpi_scan_filter filtered_navigation.launch.py"
    "/opt/ros/humble/bin/ros2 launch auto_dock auto_dock.launch.py"
)
worker_patterns=(
    "[y]_place_square_topline_trial.py"
    "[y]_slot_topline_insert_trial.py"
    "/opt/ros/humble/bin/ros2 run fork_control fork_controller"
    "/home/ubuntu/ros2_ws/install/mentorpi_scan_filter/lib/mentorpi_scan_filter/scan_filter"
    "/home/ubuntu/third_party_ros2/third_party_ws/install/ascamera/lib/ascamera/ascamera_node"
    "/opt/ros/humble/lib/depthimage_to_laserscan/depthimage_to_laserscan_node"
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*depth_scan_frame"
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher .* depth_cam ascamera_"
    "/opt/ros/humble/lib/joint_state_publisher/joint_state_publisher"
    "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher /home/ubuntu/ros2_ws/src/simulations/mentorpi_description/urdf/mentorpi.xacro"
    "/home/ubuntu/ros2_ws/install/controller/lib/controller/odom_publisher"
    "/opt/ros/humble/lib/robot_localization/ekf_node"
    "/home/ubuntu/third_party_ros2/third_party_ws/install/ldlidar_stl_ros2/lib/ldlidar_stl_ros2/ldlidar_stl_ros2_node"
    "/opt/ros/humble/lib/joy/joy_node"
    "/home/ubuntu/ros2_ws/install/peripherals/lib/peripherals/joystick_control"
    "/home/ubuntu/third_party_ros2/third_party_ws/install/imu_calib/lib/imu_calib/apply_calib"
    "/opt/ros/humble/lib/imu_complementary_filter/complementary_filter_node"
    "/opt/ros/humble/lib/rclcpp_components/component_container_isolated .*__node:=nav2_container"
    "/home/ubuntu/ros2_ws/install/auto_dock/lib/auto_dock/auto_dock_node"
    "/home/ubuntu/ros2_ws/install/fork_control/lib/fork_control/fork_controller"
    "/shared/yolo_symbol_seg_node.py"
)
motor_patterns=(
    "/home/ubuntu/ros2_ws/install/ros_robot_controller/lib/ros_robot_controller/ros_robot_controller"
)

main() {
if ((EUID != 0)); then
    exec sudo -n "$0" "$@"
fi

# sudo drops the calling zsh environment. Establish vehicle 1 DDS explicitly.
set +u
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=215 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_DISCOVERY_SERVER CYCLONEDDS_URI

# Latch before zeros: a live trial must not overwrite the stop with its next
# nonzero command. New test_y invocations snapshot this marker independently.
install -d -o ubuntu -g ubuntu /home/ubuntu/.local/state/y_place_trial
touch /home/ubuntu/.local/state/y_place_trial/stop
for trial_pattern in \
    '[y]_place_square_topline_trial.py' \
    '[y]_slot_topline_insert_trial.py'; do
    mapfile -t trial_pids < <(pgrep -f -- "$trial_pattern" || true)
    if ((${#trial_pids[@]})); then
        printf 'Stopping standalone Y trial: %s\n' "${trial_pids[*]}"
        kill -INT "${trial_pids[@]}" 2>/dev/null || true
    fi
done
publish_stop_commands

shutdown_stack
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    main "$@"
fi
