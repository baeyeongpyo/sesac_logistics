# MentorPi M1 Gazebo Sim Migration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the remote Gazebo Classic runtime with a Gazebo Fortress simulation that preserves the two-robot ROS contract and proves isolated Mecanum control.

**Architecture:** Build in `/home/bae/sesac_logistics_gz_migration` while `/home/bae/sesac_logistics` remains usable. A native SDFormat model and world run in Gazebo Fortress; explicit `ros_gz` bridges preserve ROS topics and frames. Cut over and remove Classic artifacts only after static, sensor/TF, control-isolation, SLAM, Nav2, and fleet checks pass.

**Tech Stack:** Ubuntu 22.04, Docker Compose, ROS 2 Humble, Gazebo Fortress / gz-sim 6, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, SDFormat, Nav2, SLAM Toolbox, Bash, pytest, SSH X11, VirtualGL/EGL.

## Global Constraints

- Preserve `/robot_N/controller/cmd_vel`, `/robot_N/odom`, `/robot_N/scan_raw`, `/robot_N/imu/data_raw`, `/robot_N/depth/image_raw`, `/robot_N/depth/camera_info`, `/tf`, `/tf_static`, and `/clock`.
- Preserve `robot_N/odom`, `robot_N/base_footprint`, `robot_N/base_laser`, and all existing Nav2 action names.
- Use `MACHINE_TYPE=MentorPi_Mecanum` and support `linear.x`, `linear.y`, and `angular.z` in simulation.
- Do not run manual Twist control concurrently with Nav2 or fleet control.
- Do not fetch Gazebo Fuel assets at runtime.
- Do not delete the working Classic directory or image until the replacement passes pre-cutover tests.
- Delete Classic-generated maps and create a fresh Gazebo Sim `candidate-001` and `v001` before accepting Nav2/fleet regression tests.
- Do not delete project wiki artifacts or preserved Hiwonder source evidence.

---

### Task 1: Isolated staging tree and migration contract tests

**Files:**
- Create remotely: `/home/bae/sesac_logistics_gz_migration`
- Create: `tests/test_gz_sim_contract.sh`
- Create: `tests/test_control_scripts.sh`
- Modify: `scripts/verify.sh`

**Interfaces:**
- Consumes: the current remote Classic tree.
- Produces: failing static tests that require `mentorpi_gz_sim`, `ros_gz`, Mecanum systems, bridge topics, and bounded operator scripts.

- [ ] **Step 1: Clone the runtime into staging without deleting the source**

```bash
ssh bae@ypddns.iptime.org 'test ! -e ~/sesac_logistics_gz_migration && cp -a ~/sesac_logistics ~/sesac_logistics_gz_migration'
```

- [ ] **Step 2: Add the Gazebo Sim contract test before production changes**

The test must fail unless the Dockerfile contains `ros-humble-ros-gz`, the new package is named `mentorpi_gz_sim`, the SDF contains four Mecanum wheel-joint parameters and `gz_frame_id` sensor fields, and runtime files contain none of `gazebo_ros`, `libgazebo_ros_`, `gzserver`, or `gzclient`.

- [ ] **Step 3: Add the operator-script contract test before scripts exist**

The test invokes `control-robot.sh invalid forward` and `control-robot.sh robot_1 invalid`, expects both to return exit status 2, and checks that every motion path publishes a final zero Twist.

- [ ] **Step 4: Run RED tests**

```bash
bash tests/test_gz_sim_contract.sh
bash tests/test_control_scripts.sh
```

Expected: both fail because the Gazebo Sim package and control scripts do not exist.

### Task 2: Fortress image, native model, world, launch, and bridges

**Files:**
- Modify: `Dockerfile`
- Modify: `compose.yaml`
- Modify: `ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro`
- Delete: `ros2_ws/src/mentorpi_gazebo/`
- Create: `ros2_ws/src/mentorpi_gz_sim/package.xml`
- Create: `ros2_ws/src/mentorpi_gz_sim/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro`
- Create: `ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf`
- Create: `ros2_ws/src/mentorpi_gz_sim/launch/two_robot_sim.launch.py`
- Create: `ros2_ws/src/mentorpi_gz_sim/config/robot_1_bridge.yaml`
- Create: `ros2_ws/src/mentorpi_gz_sim/config/robot_2_bridge.yaml`
- Modify: `scripts/run-sim.sh`
- Modify: every mapping/navigation/fleet runtime and test reference from `mentorpi_gazebo` to `mentorpi_gz_sim`.

**Interfaces:**
- Consumes: the topic/frame table in the design spec.
- Produces: two Gazebo entities, ROS sensor/odometry/TF topics, and ROS-to-Gazebo command bridges.

- [ ] **Step 1: Replace Classic dependency assertions with Gazebo Sim assertions and run RED**

```bash
bash tests/test_mentorpi_contract.sh
bash tests/test_description_contract.sh
bash tests/test_gz_sim_contract.sh
```

Expected: failure reports reference missing `ros_gz` dependencies and the absent new simulation package.

- [ ] **Step 2: Install Fortress integration dependencies**

Use `ros-humble-ros-gz`, `ros-humble-ros-gz-image`, and `ros-humble-teleop-twist-keyboard`; remove `ros-humble-gazebo-ros-pkgs` and `ros-humble-gazebo-plugins`. Rename the image to `sesac-logistics/mentorpi-m1-gz-sim:local`.

- [ ] **Step 3: Implement the native Mecanum SDF and local world**

Use `gz-sim-mecanum-drive-system` with front-left, front-right, back-left, and back-right continuous wheel joints; set wheelbase, separation, radius, odometry rate, command topic, odometry topic, TF topic, and robot-prefixed frames. Add GPU LiDAR, IMU, and depth sensors with robot-prefixed topics and `gz_frame_id` values.

- [ ] **Step 4: Implement launch and directional bridge configuration**

Start `ros_gz_sim/gz_sim.launch.py`, spawn two processed SDF strings at `(-1.5,-0.8)` and `(-1.5,0.8)`, start robot-state publishers, two parameter bridges, two image bridges, and bridge `/clock` once. Headless arguments are `-r -s`; GUI arguments are `-r`.

- [ ] **Step 5: Run GREEN static tests and build**

```bash
./scripts/verify.sh --static
./scripts/build.sh
```

Expected: all contract/unit tests pass and Docker build exits 0.

- [ ] **Step 6: Run sensor and TF integration test**

```bash
bash tests/test_two_robot_sim.sh
```

Expected: both robots publish clock, scan, odometry, IMU, depth image, camera info, and the two required TF chains.

### Task 3: Bounded individual control and isolation verification

**Files:**
- Create: `scripts/control-robot.sh`
- Create: `scripts/teleop-robot.sh`
- Create: `scripts/control-demo.sh`
- Create: `tests/test_independent_control_runtime.sh`
- Modify: `tests/test_operator_workflow.sh`
- Modify: `README.md`

**Interfaces:**
- Consumes: `/robot_1/controller/cmd_vel`, `/robot_2/controller/cmd_vel`, and both odometry topics.
- Produces: bounded per-robot commands, keyboard teleoperation, a visible demo, and quantitative namespace-isolation evidence.

- [ ] **Step 1: Run the existing RED operator test**

```bash
bash tests/test_control_scripts.sh
```

Expected: failure because the three operator scripts are absent.

- [ ] **Step 2: Implement bounded motion scripts**

`control-robot.sh ROBOT MOTION [DURATION]` accepts only `robot_1|robot_2`, the seven documented motions, and duration `0.1..10.0`; it publishes at 10 Hz and traps exit to publish zero. `teleop-robot.sh ROBOT` remaps keyboard `cmd_vel` only to the selected final simulation command topic. `control-demo.sh` invokes forward, lateral, and yaw motions for both robots with explicit pauses.

- [ ] **Step 3: Run GREEN operator tests**

```bash
bash tests/test_control_scripts.sh
bash tests/test_operator_workflow.sh
```

Expected: both tests print PASS.

- [ ] **Step 4: Add and run isolation runtime test**

Start the two-robot simulation, record both initial odometry poses, command only robot 1, and assert robot 1 displacement exceeds 0.05 m while robot 2 displacement stays below 0.02 m. Repeat with robot 2. Then command lateral movement and yaw and assert the expected odometry component changes.

```bash
bash tests/test_independent_control_runtime.sh
```

Expected: `independent Mecanum control: PASS`.

### Task 4: Fresh map, Nav2, and fleet regression

**Files:**
- Modify: `scripts/drive-mapping-route.sh`
- Modify: `tests/test_slam_runtime.sh`
- Modify: `tests/test_two_robot_nav2_runtime.sh`
- Modify: `tests/test_fleet_runtime.sh`
- Recreate: `maps/candidates/candidate-001/`
- Recreate: `maps/approved/v001/`

**Interfaces:**
- Consumes: the bridged robot 1 scan/odom/TF contract.
- Produces: a new Gazebo Sim map and regression evidence for both Nav2 stacks and fleet ordering.

- [ ] **Step 1: Remove only staged Classic-generated map contents**

```bash
rm -rf maps/candidates maps/approved
mkdir -p maps/candidates maps/approved
```

- [ ] **Step 2: Run mapping and save a new candidate**

```bash
bash tests/test_slam_runtime.sh
```

Expected: a non-empty `candidate-001` PGM/YAML, pose graph, manifest, and valid checksums.

- [ ] **Step 3: Promote and validate the fresh map**

```bash
./scripts/promote-map.sh candidate-001 v001
./scripts/validate-map-manifest.sh maps/approved/v001
```

Expected: validation prints PASS.

- [ ] **Step 4: Run Nav2 and fleet regression**

```bash
bash tests/test_two_robot_nav2_runtime.sh
bash tests/test_fleet_runtime.sh
```

Expected: both AMCL/Nav2 stacks activate, both actions exist, Collision Monitor runs for both robots, and fleet logs show robot 1 reservation release before robot 2 acceptance.

### Task 5: Cutover, X11 validation, and legacy cleanup

**Files:**
- Modify: remote directory names under `/home/bae`
- Delete: Classic-only remote plans/specs, image, package, maps, tests, and dependencies.
- Modify: `README.md`

**Interfaces:**
- Consumes: all passing staged verification evidence.
- Produces: `/home/bae/sesac_logistics` containing only the Gazebo Sim runtime and exact operator guidance.

- [ ] **Step 1: Run the full staged headless verification immediately before cutover**

```bash
./scripts/smoke-headless.sh
```

Expected: the full suite exits 0 and prints `full headless MentorPi Gazebo Sim smoke: PASS`.

- [ ] **Step 2: Check the X11 launch path**

```bash
./scripts/smoke-virtualgl-x11.sh
./scripts/run-sim.sh --gui
```

Expected: Gazebo Sim opens through SSH X11. In a second terminal, `./scripts/control-demo.sh` visibly moves the two named robots independently.

- [ ] **Step 3: Atomically replace the runtime directory**

Stop and remove only project simulation containers, rename the current directory to a temporary Classic backup, rename staging to `sesac_logistics`, and rerun `./scripts/verify.sh --static` from the final path.

- [ ] **Step 4: Audit and remove Classic artifacts**

```bash
rg -n 'gazebo_ros|libgazebo_ros_|gzserver|gzclient|Gazebo Classic' Dockerfile compose.yaml README.md scripts tests ros2_ws || true
docker image inspect sesac-logistics/mentorpi-m1-sim:local
```

Expected before removal: source search has no matches; old image inspection succeeds. Remove the temporary Classic backup and old image, then confirm both paths/images are absent.

- [ ] **Step 5: Run final verification from the final directory**

```bash
./scripts/verify.sh --static
./scripts/smoke-headless.sh
```

Expected: both exit 0 with no Classic runtime references and the new `v001` validates.
