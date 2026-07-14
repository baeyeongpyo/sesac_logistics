# MentorPi M1 Gazebo Sim Migration Design

**Status:** Approved design pending written-spec review  
**Target:** `bae@ypddns.iptime.org:/home/bae/sesac_logistics`

## Goal

Replace the Gazebo Classic 11 implementation with a ROS 2 Humble and Gazebo
Fortress simulation for two Hiwonder MentorPi M1 Standard robots. Preserve the
existing robot-facing ROS contracts, prove that each robot can be controlled
independently in the Gazebo Sim GUI, and remove all superseded Classic runtime
artifacts only after the replacement passes its verification suite.

## Baseline and Compatibility

- ROS 2 remains Humble on Ubuntu 22.04 in Docker.
- Gazebo Sim uses Fortress, the Gazebo release officially paired with Humble.
- ROS integration uses `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_image` instead
  of `gazebo_ros_pkgs` and embedded ROS plugins.
- The simulated vehicle remains the M1 Standard depth-camera kit with
  `MACHINE_TYPE=MentorPi_Mecanum` and `DEPTH_CAMERA_TYPE=ascamera`.
- The physical M1 is not connected; all direct velocity commands in this design
  are simulation-only.

## Preserved Runtime Contract

| Interface | robot_1 | robot_2 |
|---|---|---|
| Command | `/robot_1/controller/cmd_vel` | `/robot_2/controller/cmd_vel` |
| Odometry | `/robot_1/odom` | `/robot_2/odom` |
| LiDAR | `/robot_1/scan_raw` | `/robot_2/scan_raw` |
| IMU | `/robot_1/imu/data_raw` | `/robot_2/imu/data_raw` |
| Depth image | `/robot_1/depth/image_raw` | `/robot_2/depth/image_raw` |
| Camera info | `/robot_1/depth/camera_info` | `/robot_2/depth/camera_info` |
| Odom frame | `robot_1/odom` | `robot_2/odom` |
| Base frame | `robot_1/base_footprint` | `robot_2/base_footprint` |
| LiDAR frame | `robot_1/base_laser` | `robot_2/base_laser` |

Global `/clock`, `/tf`, `/tf_static`, `/map`, and the two namespaced Nav2 action
servers remain unchanged. Fleet coordination continues to send high-level
`NavigateToPose` goals and never publishes chassis velocity.

## Gazebo Sim Architecture

### Model

Create a Gazebo Sim-native SDFormat M1 model with four continuous wheel joints,
primitive geometry based on the existing published dimensions, 2D GPU LiDAR,
IMU, and depth camera. Use the Fortress `MecanumDrive` system so the model accepts
forward/backward `linear.x`, lateral `linear.y`, and yaw `angular.z` commands.
Keep the ROS URDF/Xacro free of simulator plugins and use it only for
`robot_state_publisher` and ROS-side frame publication.

### World and server

Replace the Classic world with a modern SDFormat world that loads the Physics,
UserCommands, SceneBroadcaster, Sensors, and IMU systems explicitly. The world
contains local ground, walls, shelves, and corridor geometry and does not fetch
Fuel resources at runtime.

`run-sim.sh --headless` starts only the Gazebo server and ROS bridges. It does
not start Xvfb or a Gazebo client. `run-sim.sh --gui` starts the Gazebo Sim client
through the existing VirtualGL/EGL and SSH X11 path.

### ROS bridge

Run explicit per-robot bridges so command traffic flows from ROS to Gazebo and
sensor, odometry, and transform traffic flows from Gazebo to ROS. Use a dedicated
image bridge for depth images. Bridge configuration is checked into the
simulation package; no ad-hoc terminal bridge commands are required during
normal use.

## Independent Robot Control

Provide three operator paths:

1. `control-robot.sh ROBOT MOTION` sends a bounded command followed by an
   explicit stop. Supported motions are `forward`, `backward`, `strafe-left`,
   `strafe-right`, `rotate-left`, `rotate-right`, and `stop`.
2. `teleop-robot.sh ROBOT` starts keyboard teleoperation remapped only to the
   selected robot.
3. `control-demo.sh` moves robot 1 and robot 2 in visibly different sequences so
   namespace isolation can be observed in the GUI.

The command scripts accept only `robot_1` or `robot_2`, use conservative speed
and duration limits, always publish a final zero Twist, and are documented as
standalone-simulation tools. Manual Twist control is not run concurrently with
Nav2 or the fleet dispatcher.

## SLAM, Nav2, and Fleet Preservation

SLAM Toolbox, shared-map approval, two independent Nav2 stacks, Collision
Monitor, and fleet reservation behavior remain in scope. Their simulator
dependency changes from Classic topics to equivalent bridged Gazebo Sim topics.

The existing Classic-generated candidate and approved map directories are
deleted after the new simulator passes sensor, TF, and independent-control
tests. A fresh `candidate-001` and `v001` are then created from Gazebo Sim before
the Nav2 and fleet runtime tests are accepted.

## Verification

### Static tests

- No `gazebo_ros`, `libgazebo_ros_*`, `gazebo`, `gzserver`, or `gzclient`
  dependency remains in runtime files.
- Required `ros_gz_*` dependencies, Gazebo systems, bridge directions, topic
  names, frame names, and Mecanum wheel joints are present.
- Operator scripts reject invalid robot names and unsupported motions.

### Runtime tests

- Both robot entities appear in Gazebo Sim.
- `/clock`, scan, odometry, IMU, depth image, camera info, and required TF chains
  publish for both robots.
- Commanding robot 1 changes robot 1 odometry while robot 2 stays within a small
  stationary tolerance; the reciprocal test then commands robot 2.
- Forward, lateral, and yaw commands produce the expected dominant odometry
  change and each case ends at zero velocity.
- SLAM saves a new map, Nav2 activates for both robots, and fleet goals are
  accepted with corridor reservation ordering intact.
- The X11 Gazebo Sim GUI opens and the independent-control demo is visible.

## Atomic Migration and Legacy Cleanup

Build and test the replacement under a new Docker image tag before cleanup.
Only after static, headless sensor/TF, and independent-control tests pass:

- remove the `mentorpi_gazebo` Classic package and replace it with
  `mentorpi_gz_sim`;
- remove Classic plugin tags, dependencies, launch commands, Xvfb-only paths,
  and obsolete Classic tests;
- remove Classic-generated `maps/candidates` and `maps/approved` contents before
  generating the fresh Gazebo Sim map;
- remove superseded remote design/plan files that describe the Classic runtime;
- remove the old `sesac-logistics/mentorpi-m1-sim:local` Docker image after no
  containers reference it.

Project wiki artifacts, preserved Hiwonder source evidence, and the long-term
shared-map/fleet architecture are not legacy runtime artifacts and are not
deleted.

## Failure Handling

- A missing bridge topic or TF prevents later SLAM/Nav2 tests from running.
- A failed independent-control assertion stops migration before legacy cleanup.
- GUI failure does not invalidate headless server behavior, but migration is not
  declared complete until the X11 launch path is verified.
- If the new image fails before cutover, the current remote tree and Classic
  image remain available for diagnosis. No partial cleanup is performed.

## Acceptance Criteria

Migration is complete when the new image builds, all static and runtime tests
pass, each robot can be controlled independently and observed in Gazebo Sim,
fresh SLAM/Nav2/fleet tests pass on the new `v001`, the README contains exact
operator commands, and no Classic runtime dependency or superseded artifact
remains on the server.
