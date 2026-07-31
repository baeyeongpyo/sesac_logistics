---
id: vehicle.runtime_jazzy
title: Vehicle Runtime Jazzy
type: runtime
owner: fork_test
status: draft
source_files:
  - vehicle_simulator_model/ubuntu/scripts/setup_vehicle_runtime_jazzy.sh
  - vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_apriltag_control/mentorpi_apriltag_control/loading_controller.py
  - vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_apriltag_control/launch/ascamera_loading.launch.py
---

# Vehicle Runtime Jazzy

This document records the runtime changes made on the Raspberry Pi vehicle
container during the AprilTag loading test.

## Existing State

The active vehicle container was based on `ros:jazzy-ros-base-noble`. That base
image did not include the MentorPi drive packages, peripherals launch files,
Oradar MS200 LiDAR driver, rqt tools, Angstrong camera workspace, or the local
fork lift controller package.

The repository Dockerfile under `vehicle_simulator_model/ubuntu/` is a Humble
Gazebo simulation image. It is not the same image as the Raspberry Pi Jazzy
runtime container.

## Added Sources

| Source | Purpose |
|---|---|
| Hiwonder `MentorPi` | `controller`, `ros_robot_controller`, `ros_robot_controller_msgs`, `sdk`, and `peripherals` packages |
| Oradar MS200 setup repository | `oradar_lidar` ROS 2 driver source |
| Local `fork_control` package | Fork lift `UP`/`DOWN`/`STOP` command subscriber |
| Angstrong `ascamera` workspace | RGB/depth/point cloud camera topics for the HP60C camera |
| Local `mentorpi_apriltag_control` package | AprilTag detection and loading-zone final approach logic |

## Reproduction

Inside the Jazzy container, run:

```bash
vehicle_simulator_model/ubuntu/scripts/setup_vehicle_runtime_jazzy.sh
```

Then copy or mount the local packages into `/home/ubuntu/ros2_ws/src` and build:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros2_ws
colcon build --symlink-install --packages-select fork_control mentorpi_apriltag_control
```

The current robot also needs hardware access to `/dev/ttyACM0`, camera USB
devices, and GPIO chips. The manually tested container was run with host
networking and privileged device access.

## Runtime Topics

| Topic | Meaning |
|---|---|
| `/ascamera_hp60c/camera_publisher/rgb0/image` | RGB image used for AprilTag detection |
| `/ascamera_hp60c/camera_publisher/rgb0/camera_info` | Camera intrinsics for metric tag pose |
| `/ascamera_hp60c/camera_publisher/depth0/image_raw` | Depth image, currently available for other teams |
| `/ascamera_hp60c/camera_publisher/depth0/points` | Point cloud, currently available for other teams |
| `/cmd_vel` | Final base command during loading logic |
| `/fork/command` | Fork command; loading controller publishes `UP` |
| `/loading/status` | Loading state, including `TAG_SEARCH_FAILED` |
| `/scan` | LiDAR scan. Software is installed, but a LiDAR device must be detected before this publishes |

## Boundaries

This runtime covers the loading-zone handoff after the vehicle has arrived near
the pallet. Nav2 path planning and obstacle interrupt behavior are separate
team responsibilities. The loading controller may use `/scan` as a safety stop
input when a LiDAR publisher is available, but it does not own global obstacle
avoidance.
