# MentorPi M1 Shared-Mapping Architecture and SLAM/Nav2 Simulation Design

**Date:** 2026-07-14  
**Status:** Approved for implementation planning  
**Target:** `bae@ypddns.iptime.org:/home/bae/sesac_logistics`

## Goal

Replace the existing TurtleBot3-specific demonstration with a Hiwonder MentorPi M1 Standard simulation that can create and save a 2D map, run two independently namespaced Nav2 stacks on an approved shared map, and demonstrate robot-local collision handling. Preserve the longer-term architecture for server-side shared-map optimization, multi-vehicle coordination, and post-navigation loading/unloading.

## Product Baseline

The simulated vehicle represents the Hiwonder MentorPi M1 Standard configuration:

- Mecanum chassis, 212 x 171 x 147 mm
- Raspberry Pi 5 and RRC Lite control architecture
- Oradar MS200 2D LiDAR contract
- Nuwa-HP60C depth-camera contract using the Hiwonder `ascamera` path
- ROS 2 Humble
- Hiwonder runtime identity `MACHINE_TYPE=MentorPi_Mecanum`
- Hiwonder depth-camera identity `DEPTH_CAMERA_TYPE=ascamera`

The physical robot is not yet network-accessible. Published dimensions and documented topic/TF contracts therefore define the simulator baseline. Hardware-specific signs, calibration, USB identifiers, exact sensor frames, and timing are validated only after the physical robot becomes available.

## Scope

### Included in the first implementation

- A MentorPi-specific URDF/Xacro model built from published dimensions
- A Gazebo Classic 11 warehouse world
- Holonomic planar simulation of the Mecanum base
- Simulated 2D LiDAR, depth camera, odometry, and IMU contracts
- Two robots named `robot_1` and `robot_2`
- SLAM Toolbox mapping with `robot_1`
- Occupancy-map and serialized pose-graph saving
- An approved shared-map directory with explicit map version metadata
- Independent AMCL, local costmap, planner, controller, velocity smoother, collision monitor, and lifecycle manager instances for both robots
- A small fleet-simulation dispatcher that sends `NavigateToPose` goals and prevents simultaneous entry into one configured shared corridor
- Headless automated verification and XQuartz/VirtualGL GUI launch paths

### Preserved architecture, deferred from the first implementation

- Live multi-robot pose-graph exchange and optimization
- Automatic candidate-map merging and promotion
- Production fleet scheduling and traffic optimization
- Vision marker detection and depth-assisted docking
- Fork insertion, lift, placement, load detection, and retreat control
- Dynamic empty/fork/loaded footprints
- Physical M1 image, ARM64 deployment, hardware drivers, and calibration

Deferred components are represented by documented interfaces and package boundaries, not placeholder runtime behavior.

## System Architecture

### Robot-local responsibility

Each physical M1 ultimately owns the functions whose latency or availability affects safety:

```text
local lidar + odometry + IMU + TF
  -> local localization or SLAM tracking
  -> local Nav2 planning and control
  -> local costmap and collision monitor
  -> local command arbitration
  -> chassis command
```

The server never streams `cmd_vel` directly to the chassis. A robot may reject, stop, delay, or replan a fleet goal when sensor, map-version, reservation, or safety conditions are invalid.

### Server responsibility

The Ubuntu server owns asynchronous and fleet-level work:

```text
robot observations + pose-graph/submap artifacts + status
  -> map optimization and candidate generation
  -> validation and approved map version
  -> task assignment and shared-resource reservation
  -> high-level goal delivered to a robot
```

The initial simulator implements approved-map storage, goal assignment, and one shared-corridor reservation. Live pose-graph fusion remains a later server backend.

## Simulation Components

### `mentorpi_description`

- Owns the MentorPi M1 Xacro model and geometry.
- Defines `base_footprint`, `base_link`, `base_laser`, IMU, RGB, depth, and optical frames.
- Accepts a `frame_prefix` so every frame is unambiguous for multiple robots.
- Defines parameterized `fork_root` and `fork_tip` frames, disabled until measured fork dimensions are supplied.
- Uses primitive geometry rather than claiming unavailable official CAD fidelity.

### `mentorpi_gazebo`

- Owns the warehouse world, Gazebo sensor plugins, spawn logic, and two-robot launch.
- Uses planar holonomic motion to reproduce the Mecanum `Twist` contract.
- Publishes namespaced odometry, IMU, LiDAR, and depth-camera topics.
- Spawns robots at known, separated initial poses.

### `mentorpi_slam`

- Owns SLAM Toolbox configuration, mapping launch, map save, pose-graph serialization, and map manifests.
- Uses `robot_1` as the first mapping leader.
- Consumes `/robot_1/scan_raw` and the transform from `robot_1/odom` to `robot_1/base_footprint`.
- Publishes the shared `map` frame and an occupancy grid during mapping.
- Saves both raster maps and serialized pose graphs so later sessions can continue or be merged.

### `mentorpi_navigation`

- Owns common Nav2 defaults and robot-specific substitutions.
- Runs one localization and navigation stack per robot namespace.
- Uses a shared absolute `/map` topic while keeping odometry, scan, costmaps, actions, and lifecycle services namespaced.
- Uses `map -> robot_N/odom -> robot_N/base_footprint` as the localization chain.
- Keeps general navigation differential-like with lateral velocity disabled. Holonomic lateral motion is reserved for the deferred docking controller.
- Uses a conservative polygon footprint based on the bare M1 chassis plus localization margin. No unmeasured fork extent is asserted.

### `mentorpi_safety`

- Owns the command pipeline and collision-monitor configuration.
- Routes Nav2 output through velocity smoothing and collision monitoring before the simulated base receives it.
- Stops or limits motion when the local LiDAR observation violates configured stop/slowdown zones.
- Remains independent per robot and does not depend on the fleet dispatcher.

### `mentorpi_fleet_sim`

- Sends high-level `NavigateToPose` goals only.
- Tracks robot identity, goal ID, map version, state, and reservation freshness.
- Implements one deterministic mutual-exclusion reservation for a narrow corridor in the test world.
- Holds or cancels a goal when map versions differ or the reservation is stale.
- Does not publish velocity commands.

## Namespace, Topic, and TF Contract

| Contract | Robot 1 | Robot 2 |
|---|---|---|
| Namespace | `/robot_1` | `/robot_2` |
| LiDAR | `/robot_1/scan_raw` | `/robot_2/scan_raw` |
| Odometry | `/robot_1/odom` | `/robot_2/odom` |
| Nav2 action | `/robot_1/navigate_to_pose` | `/robot_2/navigate_to_pose` |
| Nav command input | `/robot_1/cmd_vel_nav` | `/robot_2/cmd_vel_nav` |
| Base command | `/robot_1/controller/cmd_vel` | `/robot_2/controller/cmd_vel` |
| Odom frame | `robot_1/odom` | `robot_2/odom` |
| Base frame | `robot_1/base_footprint` | `robot_2/base_footprint` |
| LiDAR frame | `robot_1/base_laser` | `robot_2/base_laser` |
| Shared frame/topic | `map`, `/map` | `map`, `/map` |

The implementation must not mix absolute `/odom` or `/scan_raw` parameter values into robot-specific stacks.

## Map Lifecycle

### Initial mapping

1. Spawn `robot_1` in the warehouse world.
2. Validate scan, odometry, and TF before starting SLAM.
3. Run SLAM Toolbox in mapping mode.
4. Drive a repeatable mapping route and close at least one loop.
5. Save PGM/YAML and serialized pose graph.
6. Create a manifest containing version, checksum, frame contract, resolution, creation time, and source robot.
7. Promote the tested artifact to simulator map version `v001`.

### Navigation

1. Stop mapping publishers so only one source owns `map -> robot_1/odom`.
2. Start the shared map server for `v001`.
3. Start independent AMCL and Nav2 stacks for both robots.
4. Set or validate each initial pose.
5. Accept fleet goals only when the requested map version equals `v001`.

### Future partial updates

Structural observations are recorded as candidate sessions. Candidate maps never replace the active map while robots are moving. A future optimizer may merge pose graphs or update selected regions; promotion requires checksum validation and a stopped-state re-localization test.

## Command Ownership and Safety

Priority is fixed as:

```text
emergency stop
  > collision/safety stop
  > future docking controller
  > Nav2 command
  > manual simulation command
```

Only the final robot-local command gate publishes to `controller/cmd_vel`. Collision monitoring remains active if the dispatcher is stopped or disconnected. Spin recovery is disabled in constrained shared areas because a future fork increases swept volume.

## Error Handling

- Missing or stale scan, odometry, or TF prevents SLAM/Nav2 readiness.
- Duplicate TF publishers fail the verification suite.
- A robot with a mismatched map version rejects fleet goals.
- Stale corridor reservations prevent new corridor entry.
- Collision-monitor activation overrides Nav2 motion.
- Loss of the dispatcher permits a robot to stop safely but not accept new fleet work.
- Candidate-map corruption leaves the current approved map unchanged.
- GUI failure does not invalidate headless simulation; GUI and headless results are reported separately.

## Docker and Operator Workflow

The existing Docker/X11 foundation is infrastructure rather than a robot model and remains in use. TurtleBot3 dependencies, environment variables, launch commands, offline-model contracts, and tests are removed.

Expected operator commands are separated by responsibility:

```text
build and static verification
headless simulation smoke test
two-robot Gazebo/RViz launch
mapping launch
map and pose-graph save
approved-map navigation launch
two-goal fleet demonstration
```

The GUI path continues to use server-side EGL rendering through VirtualGL and trusted SSH X11 forwarding. The headless path requires no X server.

## Verification and Acceptance Criteria

### Static contracts

- No TurtleBot3 package, model, variable, launch file, or test remains.
- The M1 dimensions, runtime identities, namespace contract, topic contract, and frame contract are declared.
- No privileged container or Docker socket mount exists.
- Xauthority remains read-only for GUI use.

### Model and sensor integration

- Two robot entities spawn with unique names.
- Both robots publish non-empty scan, odometry, IMU, depth image, camera-info, joint-state, and robot-description data.
- TF trees contain no duplicate authorities or unprefixed robot-local frames.

### SLAM

- SLAM lifecycle becomes active with `robot_1` data.
- The occupancy map contains known and unknown cells after movement.
- Map YAML/PGM and pose-graph files are saved.
- Manifest checksums validate.

### Nav2

- Shared map server and both AMCL instances become active.
- Both complete Nav2 lifecycle activation.
- Each robot accepts a distinct goal and changes pose toward it.
- Navigation commands pass through each robot's safety pipeline.

### Fleet and safety

- Two robots cannot hold the same corridor reservation simultaneously.
- A waiting robot proceeds after reservation release.
- A simulated local obstacle causes the affected robot to stop or replan without stopping the other robot.
- Stopping the dispatcher does not disable local collision monitoring.

### GUI

- Gazebo and RViz remain running through the bounded X11 test.
- VirtualGL reports a direct context and the server AMD renderer.
- Logs contain no GLX context error or GUI segmentation fault.

## Physical-Robot Migration

When an M1 becomes accessible, simulation plugins are replaced behind the same ROS interfaces. Migration begins with read-only inspection of Hiwonder's installed workspace and validates:

- actual `scan_raw`, odometry, IMU, controller, image, and camera-info topics
- actual TF frame names and signs
- Mecanum lateral and yaw command signs
- MS200 range and timing
- depth-camera frame calibration
- chassis and future fork footprint measurements
- Raspberry Pi 5 architecture, RAM, and Hiwonder Docker image

No physical motion is commanded until the read-only graph, stationary sensor data, TF, and emergency-stop path pass verification.

## Risks and Mitigations

| Risk | Mitigation |
|---|---|
| Published geometry differs from the physical kit | Keep geometry parameterized and require measurements before hardware motion |
| Planar plugin hides real Mecanum slip | Treat simulator odometry as functional, then retune against hardware logs |
| Shared TF collisions | Prefix all robot-local frames and test TF authorities |
| Humble lacks current decentralized multi-robot SLAM features | Keep first release on mapping-leader + approved shared map; evaluate source-built multi-robot support separately |
| Real-time map updates destabilize localization | Use candidate versions and stopped-state promotion |
| Fork dimensions are unknown | Disable fork collision geometry until measured |
| Fleet dispatcher becomes unavailable | Keep local Nav2 safety active and block new shared-resource entry |

## Source Basis

This design follows the project-selected Hiwonder M1 product notes, getting-ready implementation guide, MentorPi SLAM/Nav2 notes, offline-first policy, multi-vehicle architecture, and fork/docking review. The current bundle duplicate-candidate warnings do not alter the selected Project/policy sources and require no design conflict resolution.
