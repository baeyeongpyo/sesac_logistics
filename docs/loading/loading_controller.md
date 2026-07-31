---
id: loading.loading_controller
title: Loading Controller
type: component
owner: fork_test
status: draft
source_files:
  - vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_apriltag_control/mentorpi_apriltag_control/loading_controller.py
  - vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_apriltag_control/launch/ascamera_loading.launch.py
---

# Loading Controller

This document describes the loading-zone logic after the robot has already
arrived near the pallet area.

## Goal

Find the target AprilTag, align the robot with the tag, approach until the stop
distance is reached, then command the fork lift-up action.

## Inputs

| Input | ROS topic | Message | Meaning |
|---|---|---|---|
| AprilTag target | `/usb_camera/apriltag/target` | `std_msgs/String` JSON | Detected tag id, image center error, pixel size, and optional pose distance |
| Front scan | `/robot_1/scan_raw` | `sensor_msgs/LaserScan` | Front obstacle distance for safety stop |

## Outputs

| Output | ROS topic | Message | Meaning |
|---|---|---|---|
| Base command | `/robot_1/controller/cmd_vel` | `geometry_msgs/Twist` | Forward/reverse speed and steering-derived yaw rate |
| Fork command | `/robot_1/fork/command` | `std_msgs/String` | Lift command such as `UP` |
| Loading status | `/loading/status` | `std_msgs/String` | Current loading state, including `TAG_SEARCH_FAILED` |

## State Machine

| State | Condition | Action |
|---|---|---|
| `TAG_SEARCH` | Target tag is not visible | Move in short forward steering arcs to scan the loading area |
| `TAG_SEARCH_FAILED` | Target tag is still not visible after the search timeout | Stop and publish failure status |
| `TAG_ALIGN` | Target tag is visible but not centered | Move slowly while steering toward center |
| `APPROACH` | Tag is centered but still too far | Move forward toward the tag |
| `BACK_OFF` | Tag is centered but too close | Reverse until target distance band is reached |
| `STOP_AT_DISTANCE` | Tag is centered and inside stop band | Stop base motion |
| `FORK_INSERT_FORWARD` | Stop position reached | Move forward for the fixed insert duration |
| `LIFT_UP` | Stop position reached and lift command not sent yet | Publish lift height once |
| `SAFETY_STOP` | Front obstacle is closer than safety threshold | Stop base motion |

## Current Assumptions

- The robot is already inside the loading zone.
- Path planning to the loading zone is outside this component.
- Lateral movement is disabled; `linear.y` is not used.
- Steering is limited by `max_steering_angle`, currently 90 degrees.
- In-place rotation is not used. Tag search uses short forward arc motion.
- With the USB webcam path, camera calibration is not trusted yet, so stopping
  can use AprilTag pixel width instead of metric pose distance.
- The printed AprilTag is approximately 47 mm wide.
- The current target id in the launch file is `1`.
- Obstacles are handled as stop conditions first. Predictive avoidance is not
  part of this component yet.

## Key Parameters

| Parameter | Current value | Meaning |
|---|---:|---|
| `target_id` | `1` | AprilTag id to follow |
| `stop_distance` | `0.19` m | Desired final distance when calibrated pose is available |
| `stop_tag_width_px` | `114.0` px | Pixel-width stop target for uncalibrated USB camera |
| `stop_tag_width_tolerance_px` | `8.0` px | Acceptable pixel-width band |
| `safety_stop_distance` | `0.0` in USB launch | Disabled for the current USB-camera + Gazebo hybrid test |
| `wheelbase` | `0.22` m | Wheelbase used for steering-derived yaw rate |
| `max_steering_angle` | `1.5708` rad | 90 degree steering limit |
| `search_linear_speed` | `0.04` m/s | Slow forward speed while searching for a missing tag |
| `search_steering_angle` | `0.65` rad | Steering angle used during local arc search |
| `search_leg_duration` | `2.0` s | Time before alternating the search steering direction |
| `search_timeout` | `12.0` s | Time before publishing `TAG_SEARCH_FAILED` |
| `lift_command` | `UP` | Fork command published after insert motion |
| `insert_duration` | `3.0` s | Fixed forward motion time after stopping at tag distance |
| `insert_speed` | `0.03` m/s | Slow insert speed used for time-based fixed motion |

## Notes For Next Implementation

The next useful improvement is camera calibration. Once `/usb_camera/camera_info`
contains valid intrinsics, the controller can prefer metric `distance` over
pixel width for `STOP_AT_DISTANCE`.
