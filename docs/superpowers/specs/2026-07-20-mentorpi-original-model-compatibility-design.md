# MentorPi Original Model Compatibility Design

## Goal

Restore the MentorPi Mecanum robot's original detailed visual geometry and
sensor transforms while preserving the frame names consumed by the current
multi-robot ROS 2, Nav2, and Gazebo packages.

## Source of truth

The preserved upstream workspace archive is:

`promotion-shelf/20260708-163950-mentorpi/raw/mentorpi-ros2-ws-group-control-2026-07-08.tar.gz`

The source model is
`src/simulations/mentorpi_description/urdf/mecanum.xacro`, with its Mecanum
STL files under `meshes/mecanum/`.

## Design

The `mentorpi_description` package will contain a local copy of the original
Mecanum STL assets and a Xacro description based on the original model.

The original chassis, four wheels, lidar, and depth-camera visual meshes will
be preserved.  The package will retain the current public frames instead of
the upstream visual-link names:

| Current public frame | Upstream visual link | Transform relative to `base_link` |
| --- | --- | --- |
| `base_laser` | `lidar_frame` | `xyz=-0.012242 -0.00008533 0.092501`, `rpy=0 0 0` |
| `depth_camera_link` | `depth_cam` | `xyz=0.061376 -0.00013463 0.051154`, `rpy=0 0 0` |

`base_footprint` remains 0.070 m below `base_link`, matching the upstream
Mecanum description. The original wheel mesh assets and joint origins will be
used. The current `robot_name` and `frame_prefix` launch arguments continue to
work through `robot_state_publisher` frame prefixing; they do not alter Xacro
geometry.

The Gazebo SDF sensor frame poses will be made consistent with the restored
URDF transforms expressed from `base_footprint`:

| Frame | Transform relative to `base_footprint` |
| --- | --- |
| `base_laser` | `xyz=-0.012242 -0.00008533 0.162501` |
| `depth_camera_link` | `xyz=0.061376 -0.00013463 0.121154` |

## Compatibility constraints

- The frame names `base_footprint`, `base_link`, `base_laser`,
  `depth_camera_link`, and `imu_link` remain available.
- ROS laser scans continue to identify `base_laser`; depth-camera messages
  continue to identify `depth_camera_link`.
- The model continues to support two namespaced robots via the existing launch
  files.
- No Project or Team wiki artifacts are changed.

## Verification

A source-parity test will extract the upstream Xacro from the preserved archive
and assert the restored mesh filenames and sensor/wheel transforms match its
Mecanum source. A second test will assert that the Gazebo SDF publishes sensor
frames at the same `base_footprint` transforms as the URDF. When `xacro` is
available, the expanded URDF will also be parsed to verify the public frame
tree.
