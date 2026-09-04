#!/usr/bin/env bash

set -u

publish_stop_commands() {
    if ! command -v ros2 >/dev/null 2>&1; then
        printf 'Warning: ros2 not found; skipping pre-stop motor command.\n' >&2
        return
    fi

    local zero_twist
    local zero_motors
    zero_twist='{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
    zero_motors='{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}'

    printf 'Publishing zero velocity and motor commands before shutdown.\n'
    timeout 2 ros2 topic pub -r 20 /controller/cmd_vel \
        geometry_msgs/msg/Twist "$zero_twist" >/dev/null 2>&1 &
    local controller_stop_pid=$!
    timeout 2 ros2 topic pub -r 20 /cmd_vel \
        geometry_msgs/msg/Twist "$zero_twist" >/dev/null 2>&1 &
    local nav_stop_pid=$!
    timeout 2 ros2 topic pub -r 20 /ros_robot_controller/set_motor \
        ros_robot_controller_msgs/msg/MotorsState "$zero_motors" \
        >/dev/null 2>&1 &
    local motor_stop_pid=$!

    wait "$controller_stop_pid" "$nav_stop_pid" "$motor_stop_pid" || true
}

if ((EUID != 0)); then
    publish_stop_commands
    exec sudo -n env STOPPER_PRESTOP_DONE=1 "$0" "$@"
fi

if [[ "${STOPPER_PRESTOP_DONE:-0}" != "1" ]]; then
    publish_stop_commands
fi

patterns=(
    "/opt/ros/humble/bin/ros2 launch mentorpi_scan_filter filtered_navigation.launch.py"
    "/opt/ros/humble/bin/ros2 run fork_control fork_controller"
    "/opt/ros/humble/bin/ros2 launch auto_dock auto_dock.launch.py"
    "/home/ubuntu/ros2_ws/install/mentorpi_scan_filter/lib/mentorpi_scan_filter/scan_filter"
    "/home/ubuntu/third_party_ros2/third_party_ws/install/ascamera/lib/ascamera/ascamera_node"
    "/opt/ros/humble/lib/depthimage_to_laserscan/depthimage_to_laserscan_node"
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher .*depth_scan_frame"
    "/opt/ros/humble/lib/tf2_ros/static_transform_publisher .* depth_cam ascamera_"
    "/opt/ros/humble/lib/joint_state_publisher/joint_state_publisher"
    "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher /home/ubuntu/ros2_ws/src/simulations/mentorpi_description/urdf/mentorpi.xacro"
    "/home/ubuntu/ros2_ws/install/ros_robot_controller/lib/ros_robot_controller/ros_robot_controller"
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

matched=0
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

for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(live_pids "$pattern")
    if ((${#pids[@]} == 0)); then
        continue
    fi
    matched=1
    printf 'Stopping: %s (PID %s)\n' "$pattern" "${pids[*]}"
    kill -INT "${pids[@]}" 2>/dev/null || true
done

sleep 3

for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(live_pids "$pattern")
    if ((${#pids[@]} == 0)); then
        continue
    fi
    printf 'Force stopping remaining process: %s (PID %s)\n' \
        "$pattern" "${pids[*]}"
    kill -TERM "${pids[@]}" 2>/dev/null || true
done

# A launch process can respawn a child in the gap between the two scans.
# Re-scan long enough to cross the auto_dock respawn delay, and use SIGKILL
# only for processes that survived the graceful signals.
for _attempt in 1 2 3 4 5; do
    sleep 0.5
    remaining=0
    for pattern in "${patterns[@]}"; do
        mapfile -t pids < <(live_pids "$pattern")
        if ((${#pids[@]} == 0)); then
            continue
        fi
        remaining=1
        printf 'Killing remaining process: %s (PID %s)\n' \
            "$pattern" "${pids[*]}"
        kill -KILL "${pids[@]}" 2>/dev/null || true
    done
done

remaining=0
for pattern in "${patterns[@]}"; do
    mapfile -t pids < <(live_pids "$pattern")
    if ((${#pids[@]} > 0)); then
        remaining=1
        printf 'ERROR: process still alive: %s (PID %s)\n' \
            "$pattern" "${pids[*]}" >&2
    fi
done

if ((remaining != 0)); then
    exit 1
elif ((matched == 0)); then
    printf 'Navigation stack is not running.\n'
else
    printf 'Navigation stack and orphan child processes stopped.\n'
fi
