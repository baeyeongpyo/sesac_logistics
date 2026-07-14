# MentorPi M1 SLAM/Nav2 Simulation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the remote TurtleBot3 demo with a two-robot Hiwonder MentorPi M1 Standard Gazebo simulation that supports robot-1 SLAM/map saving, shared-map navigation for both robots, local collision handling, and a minimal shared-corridor fleet demonstration.

**Architecture:** The Ubuntu server hosts Gazebo, shared map artifacts, and the fleet simulator. Each simulated robot has a unique namespace and frame prefix and runs independent localization, Nav2, velocity smoothing, and collision monitoring; only high-level goals cross the fleet boundary. SLAM starts with `robot_1` as mapping leader and saves both raster and pose-graph artifacts for later server-side map merging.

**Tech Stack:** Docker Compose, Ubuntu 22.04 container, ROS 2 Humble, Gazebo Classic 11, Xacro/URDF, `gazebo_ros`, SLAM Toolbox, Nav2, AMCL, `nav2_collision_monitor`, Python `rclpy`, pytest, Bash contract/integration tests, VirtualGL/EGL, SSH X11.

## Global Constraints

- Target host and project: `bae@ypddns.iptime.org:/home/bae/sesac_logistics`.
- Vehicle identity: Hiwonder MentorPi M1 Standard, `MACHINE_TYPE=MentorPi_Mecanum`, `DEPTH_CAMERA_TYPE=ascamera`.
- Model dimensions: 212 x 171 x 147 mm; primitive geometry is used because official CAD is unavailable.
- Runtime topics and frames must follow the approved `robot_1`/`robot_2` contract.
- General navigation keeps `linear.y=0`; holonomic lateral motion is reserved for deferred docking.
- The server does not publish chassis `cmd_vel`; the fleet simulator sends `NavigateToPose` goals only.
- No privileged container and no Docker socket mount. Xauthority remains read-only.
- Keep the verified VirtualGL 3.1.4/EGL X11 infrastructure; remove all TurtleBot3-specific dependencies, configuration, scripts, and tests.
- Mapping output is a candidate until its checksums and manifest validate; moving robots never hot-swap the active map.
- The physical M1 remains out of scope until read-only hardware inspection is possible.
- The remote target is not a Git repository; do not initialize one implicitly. Use the local design/plan commits plus explicit remote deployment and verification checkpoints.

---

## File Structure

The remote project is rebuilt with these ownership boundaries:

```text
Dockerfile                         image dependencies and overlay build
compose.yaml                       host networking, render device, map/log volumes
docker/entrypoint.sh               sources ROS and /opt/mentorpi_ws/install
ros2_ws/src/mentorpi_description/  M1 geometry, frames, RViz model
ros2_ws/src/mentorpi_gazebo/       Gazebo world, plugins, spawn launches
ros2_ws/src/mentorpi_slam/         SLAM config and mapping launch
ros2_ws/src/mentorpi_navigation/   shared map, AMCL, Nav2 launch and params
ros2_ws/src/mentorpi_safety/       collision-monitor params and launch
ros2_ws/src/mentorpi_fleet_sim/    high-level goal and corridor reservation node
maps/approved/v001/                approved map image, YAML, pose graph, manifest
maps/candidates/                   mapping-session outputs
scripts/                           build/run/save/smoke/verify operator commands
tests/                             static, unit, and integration contracts
README.md                          supported workflows and real-robot boundary
```

---

### Task 1: Replace the TurtleBot3 Foundation with MentorPi Contracts

**Files:**
- Modify: `Dockerfile`
- Modify: `compose.yaml`
- Create: `docker/entrypoint.sh`
- Replace: `tests/test_environment.sh`
- Create: `tests/test_no_turtlebot3.sh`
- Create: `tests/test_mentorpi_contract.sh`
- Modify: `.dockerignore`

**Interfaces:**
- Consumes: verified VirtualGL package `vendor/virtualgl_3.1.4_amd64.deb`, host `/dev/dri/renderD128`, render GID `${RENDER_GID:-991}`.
- Produces: image `sesac-logistics/mentorpi-m1-sim:local`, sourced overlay `/opt/mentorpi_ws/install`, environment identities `MentorPi_Mecanum` and `ascamera`.

- [ ] **Step 1: Write failing removal and environment contract tests.**

`tests/test_no_turtlebot3.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

if rg -n -i 'turtlebot3|TURTLEBOT3_MODEL|tb3_simulation' \
  Dockerfile compose.yaml README.md scripts tests ros2_ws 2>/dev/null; then
  echo "FAIL: TurtleBot3-specific content remains" >&2
  exit 1
fi
echo "TurtleBot3 removal contract: PASS"
```

`tests/test_mentorpi_contract.sh` asserts:

```bash
grep -Fq 'MACHINE_TYPE=MentorPi_Mecanum' Dockerfile
grep -Fq 'DEPTH_CAMERA_TYPE=ascamera' Dockerfile
grep -Fq 'ros-humble-slam-toolbox' Dockerfile
grep -Fq 'ros-humble-navigation2' Dockerfile
grep -Fq 'ros-humble-nav2-bringup' Dockerfile
grep -Fq 'ros-humble-nav2-collision-monitor' Dockerfile
grep -Fq 'ros-humble-gazebo-ros-pkgs' Dockerfile
grep -Fq 'ros-humble-xacro' Dockerfile
grep -Fq '/dev/dri/renderD128:/dev/dri/renderD128' compose.yaml
grep -Fq '/tmp/.Xauthority:ro' compose.yaml
grep -Fq 'sesac-logistics/mentorpi-m1-sim:local' compose.yaml
```

- [ ] **Step 2: Run tests and verify RED.**

Run on the remote host:

```bash
cd ~/sesac_logistics
bash tests/test_no_turtlebot3.sh
bash tests/test_mentorpi_contract.sh
```

Expected: `test_no_turtlebot3.sh` fails on the existing TurtleBot3 Dockerfile and scripts; `test_mentorpi_contract.sh` fails on missing M1 identities/image.

- [ ] **Step 3: Replace the image and Compose contract.**

The Dockerfile keeps the checksum-verified VirtualGL install and installs these additional packages:

```dockerfile
ros-humble-gazebo-ros-pkgs
ros-humble-gazebo-plugins
ros-humble-joint-state-publisher
ros-humble-navigation2
ros-humble-nav2-bringup
ros-humble-nav2-collision-monitor
ros-humble-robot-localization
ros-humble-slam-toolbox
ros-humble-tf2-tools
ros-humble-xacro
python3-colcon-common-extensions
python3-pytest
```

It copies `ros2_ws/src` to `/opt/mentorpi_ws/src`, runs:

```dockerfile
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/mentorpi_ws \
    && colcon build --symlink-install
```

The entrypoint contains:

```bash
#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source /opt/mentorpi_ws/install/setup.bash
exec "$@"
```

- [ ] **Step 4: Delete obsolete TurtleBot3/Gazebo contract tests and scripts.**

Delete:

```text
tests/test_gazebo_offline_contract.sh
tests/test_gazebo_version_handling.sh
tests/test_nav2_runtime_dependencies.sh
scripts/run-demo.sh
scripts/smoke-headless.sh
```

Keep and later adapt `scripts/shell.sh`, `scripts/smoke-virtualgl-x11.sh`, and `scripts/verify.sh`.

- [ ] **Step 5: Run static tests and Compose parsing.**

```bash
bash tests/test_environment.sh
bash tests/test_no_turtlebot3.sh
bash tests/test_mentorpi_contract.sh
docker compose config -q
```

Expected: all commands exit 0.

---

### Task 2: Build the MentorPi Description and Two-Robot Gazebo World

**Files:**
- Create: `ros2_ws/src/mentorpi_description/package.xml`
- Create: `ros2_ws/src/mentorpi_description/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro`
- Create: `ros2_ws/src/mentorpi_description/launch/description.launch.py`
- Create: `ros2_ws/src/mentorpi_gazebo/package.xml`
- Create: `ros2_ws/src/mentorpi_gazebo/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_gazebo/worlds/warehouse.world`
- Create: `ros2_ws/src/mentorpi_gazebo/launch/sim.launch.py`
- Create: `ros2_ws/src/mentorpi_gazebo/launch/two_robot_sim.launch.py`
- Create: `tests/test_description_contract.sh`
- Create: `tests/test_two_robot_sim.sh`

**Interfaces:**
- Consumes: `robot_name`, `frame_prefix`, initial pose, and `use_sim_time` launch arguments.
- Produces: entities `robot_1`/`robot_2`; global `/tf` and `/tf_static`; namespaced `scan_raw`, `odom`, `imu/data_raw`, depth image, camera info, joint states, and controller command topics.

- [ ] **Step 1: Write the failing M1 geometry/topic contract.**

`tests/test_description_contract.sh` checks exact model facts:

```bash
grep -Fq 'name="chassis_length" value="0.212"' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'name="chassis_width" value="0.171"' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'name="chassis_height" value="0.147"' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'libgazebo_ros_planar_move.so' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'libgazebo_ros_ray_sensor.so' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'scan_raw' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'base_laser' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
grep -Fq 'fork_enabled' ros2_ws/src/mentorpi_description/urdf/mentorpi_m1.urdf.xacro
```

- [ ] **Step 2: Run the contract and verify RED because the packages do not exist.**

```bash
bash tests/test_description_contract.sh
```

- [ ] **Step 3: Implement the M1 Xacro.**

Use prefixed frame names:

```xml
<xacro:arg name="robot_name" default="robot_1"/>
<xacro:arg name="frame_prefix" default="robot_1/"/>
<xacro:arg name="fork_enabled" default="false"/>
<xacro:property name="chassis_length" value="0.212"/>
<xacro:property name="chassis_width" value="0.171"/>
<xacro:property name="chassis_height" value="0.147"/>
```

The planar plugin must remap commands and odometry within each namespace while publishing prefixed frame IDs:

```xml
<plugin name="${robot_name}_planar_move" filename="libgazebo_ros_planar_move.so">
  <ros>
    <namespace>/${robot_name}</namespace>
    <remapping>cmd_vel:=controller/cmd_vel</remapping>
    <remapping>odom:=odom</remapping>
    <remapping>/tf:=/tf</remapping>
  </ros>
  <odometry_frame>${frame_prefix}odom</odometry_frame>
  <robot_base_frame>${frame_prefix}base_footprint</robot_base_frame>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
</plugin>
```

The ray sensor publishes `LaserScan` to `scan_raw` with `${frame_prefix}base_laser`. Depth camera and IMU plugins use the same namespace and prefix contract.

- [ ] **Step 4: Implement the warehouse and two-robot spawn launch.**

The world contains walls, shelves, static obstacles, and a named narrow corridor. `two_robot_sim.launch.py` expands Xacro twice and spawns:

```text
robot_1 at (-1.5, -0.8, 0.05), yaw 0
robot_2 at (-1.5,  0.8, 0.05), yaw 0
```

Both robot-state publishers remap `tf` and `tf_static` to the global topics while using prefixed frame IDs.

- [ ] **Step 5: Build and run the headless two-robot integration test.**

`tests/test_two_robot_sim.sh` launches Gazebo in a named detached container, then verifies:

```bash
ros2 topic echo /robot_1/scan_raw --once
ros2 topic echo /robot_2/scan_raw --once
ros2 topic echo /robot_1/odom --once
ros2 topic echo /robot_2/odom --once
ros2 topic echo /robot_1/imu/data_raw --once
ros2 topic echo /robot_2/imu/data_raw --once
ros2 topic echo /robot_1/depth/image_raw --once
ros2 topic echo /robot_2/depth/camera_info --once
ros2 run tf2_ros tf2_echo robot_1/odom robot_1/base_footprint
ros2 run tf2_ros tf2_echo robot_2/base_footprint robot_2/base_laser
```

Expected: all return data within bounded timeouts and both Gazebo entities exist.

---

### Task 3: Add Mapping, Map Serialization, and Versioned Map Artifacts

**Files:**
- Create: `ros2_ws/src/mentorpi_slam/package.xml`
- Create: `ros2_ws/src/mentorpi_slam/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_slam/config/slam.yaml`
- Create: `ros2_ws/src/mentorpi_slam/launch/mapping.launch.py`
- Create: `scripts/run-mapping.sh`
- Create: `scripts/drive-mapping-route.sh`
- Create: `scripts/save-map.sh`
- Create: `scripts/validate-map-manifest.sh`
- Create: `tests/test_slam_contract.sh`
- Create: `tests/test_slam_runtime.sh`
- Create: `maps/approved/.gitkeep`
- Create: `maps/candidates/.gitkeep`

**Interfaces:**
- Consumes: `/robot_1/scan_raw`, global TF containing `robot_1/odom -> robot_1/base_footprint`, simulation clock.
- Produces: `/map`, `map -> robot_1/odom`, candidate PGM/YAML, serialized pose graph/data, and manifest with version/checksums.

- [ ] **Step 1: Write failing SLAM configuration and map-manifest contracts.**

The test checks:

```bash
grep -Fq 'map_frame: map' ros2_ws/src/mentorpi_slam/config/slam.yaml
grep -Fq 'odom_frame: robot_1/odom' ros2_ws/src/mentorpi_slam/config/slam.yaml
grep -Fq 'base_frame: robot_1/base_footprint' ros2_ws/src/mentorpi_slam/config/slam.yaml
grep -Fq 'scan_topic: /robot_1/scan_raw' ros2_ws/src/mentorpi_slam/config/slam.yaml
grep -Fq 'mode: mapping' ros2_ws/src/mentorpi_slam/config/slam.yaml
grep -Fq 'serialize_map' scripts/save-map.sh
grep -Fq 'sha256sum' scripts/save-map.sh
```

- [ ] **Step 2: Verify RED.**

```bash
bash tests/test_slam_contract.sh
```

- [ ] **Step 3: Implement synchronous mapping and a deterministic route.**

`mapping.launch.py` starts only the SLAM Toolbox node; Gazebo is started separately. `drive-mapping-route.sh` publishes bounded low-speed commands to `/robot_1/controller/cmd_vel` in a rectangular route with explicit stops between segments.

- [ ] **Step 4: Implement atomic candidate saving and manifest validation.**

`save-map.sh <version>` writes into a temporary directory under `maps/candidates`, calls both map saver and `/slam_toolbox/serialize_map`, creates SHA-256 entries, then renames the completed directory. The manifest contains:

```yaml
version: candidate-001
source_robot: robot_1
map_frame: map
odom_frame: robot_1/odom
base_frame: robot_1/base_footprint
scan_topic: /robot_1/scan_raw
resolution: 0.05
```

`validate-map-manifest.sh` rejects missing files, checksum mismatches, and unsupported frames.

- [ ] **Step 5: Run the mapping integration test.**

`tests/test_slam_runtime.sh` starts one-robot Gazebo plus SLAM, drives enough motion to add pose-graph nodes, verifies `/map` has non-zero width/height and both known/unknown cells, saves `candidate-001`, and validates all checksums.

Expected output:

```text
SLAM map publication: PASS
Pose graph serialization: PASS
Candidate manifest validation: PASS
```

---

### Task 4: Add Shared-Map Localization and Two Independent Nav2 Stacks

**Files:**
- Create: `ros2_ws/src/mentorpi_navigation/package.xml`
- Create: `ros2_ws/src/mentorpi_navigation/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_navigation/config/nav2.yaml`
- Create: `ros2_ws/src/mentorpi_navigation/launch/shared_map_navigation.launch.py`
- Create: `scripts/promote-map.sh`
- Create: `scripts/run-navigation.sh`
- Create: `tests/test_nav2_contract.sh`
- Create: `tests/test_two_robot_nav2_runtime.sh`

**Interfaces:**
- Consumes: validated `maps/approved/v001/map.yaml`, global `/map`, each robot's scan/odom/TF.
- Produces: per-robot AMCL transforms, lifecycle-managed Nav2 servers, and `/robot_N/navigate_to_pose` actions.

- [ ] **Step 1: Write failing namespace and safety-chain Nav2 contracts.**

The static test rejects absolute robot-local topics and requires differential-like navigation:

```bash
if grep -Eq '(^|[[:space:]])/(odom|scan_raw)([[:space:]]|$)' ros2_ws/src/mentorpi_navigation/config/nav2.yaml; then
  echo "FAIL: robot-local Nav2 topics must be relative" >&2
  exit 1
fi
grep -Fq 'max_vel_y: 0.0' ros2_ws/src/mentorpi_navigation/config/nav2.yaml
grep -Fq 'min_vel_y: 0.0' ros2_ws/src/mentorpi_navigation/config/nav2.yaml
grep -Fq 'robot_1/odom' ros2_ws/src/mentorpi_navigation/launch/shared_map_navigation.launch.py
grep -Fq 'robot_2/odom' ros2_ws/src/mentorpi_navigation/launch/shared_map_navigation.launch.py
grep -Fq '/map' ros2_ws/src/mentorpi_navigation/launch/shared_map_navigation.launch.py
```

- [ ] **Step 2: Verify RED.**

```bash
bash tests/test_nav2_contract.sh
```

- [ ] **Step 3: Implement validated promotion.**

`scripts/promote-map.sh candidate-001 v001` first runs manifest validation, checks that no simulation robot container is navigating, copies the candidate to a temporary approved directory, rewrites only the version field, recalculates checksums, and atomically renames it to `maps/approved/v001`.

- [ ] **Step 4: Implement the shared map and per-robot Nav2 launch.**

Launch one global map server and lifecycle manager, then for each namespace launch:

```text
AMCL + localization lifecycle manager
controller_server
planner_server
behavior_server (spin disabled)
bt_navigator
waypoint_follower
velocity_smoother
navigation lifecycle manager
```

Per-robot parameter rewrites set:

```text
base_frame_id = robot_N/base_footprint
odom_frame_id = robot_N/odom
global_frame_id = map
scan_topic = scan_raw
odom_topic = odom
```

All robot nodes use global `/tf`, `/tf_static`, and `/map` remaps.

- [ ] **Step 5: Run the two-robot Nav2 lifecycle test.**

The integration test starts the two-robot world and shared navigation, sets deterministic initial poses, then verifies:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /robot_1/amcl
ros2 lifecycle get /robot_2/amcl
ros2 lifecycle get /robot_1/controller_server
ros2 lifecycle get /robot_2/controller_server
ros2 action list | grep -Fx /robot_1/navigate_to_pose
ros2 action list | grep -Fx /robot_2/navigate_to_pose
```

Expected: every lifecycle node is active and both actions exist.

---

### Task 5: Add Robot-Local Collision Monitoring and Fleet Corridor Reservation

**Files:**
- Create: `ros2_ws/src/mentorpi_safety/package.xml`
- Create: `ros2_ws/src/mentorpi_safety/CMakeLists.txt`
- Create: `ros2_ws/src/mentorpi_safety/config/collision_monitor.yaml`
- Create: `ros2_ws/src/mentorpi_safety/launch/safety.launch.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/package.xml`
- Create: `ros2_ws/src/mentorpi_fleet_sim/setup.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/setup.cfg`
- Create: `ros2_ws/src/mentorpi_fleet_sim/resource/mentorpi_fleet_sim`
- Create: `ros2_ws/src/mentorpi_fleet_sim/mentorpi_fleet_sim/__init__.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/mentorpi_fleet_sim/corridor_reservation.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/mentorpi_fleet_sim/dispatcher.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/config/tasks.yaml`
- Create: `ros2_ws/src/mentorpi_fleet_sim/launch/fleet_demo.launch.py`
- Create: `ros2_ws/src/mentorpi_fleet_sim/test/test_corridor_reservation.py`
- Create: `tests/test_command_ownership.sh`
- Create: `tests/test_fleet_runtime.sh`

**Interfaces:**
- Consumes: `cmd_vel_nav`, smoothed velocity, `scan_raw`, robot poses, `NavigateToPose` actions, map version `v001`, monotonic time.
- Produces: collision-filtered `controller/cmd_vel`, goal dispatch, reservation states `free/held/expired`, and fleet status logs.

- [ ] **Step 1: Write failing pure-Python reservation tests.**

```python
def test_second_robot_waits_until_first_releases():
    table = CorridorReservation(ttl_seconds=5.0)
    assert table.acquire("corridor_a", "robot_1", now=10.0)
    assert not table.acquire("corridor_a", "robot_2", now=11.0)
    assert table.release("corridor_a", "robot_1")
    assert table.acquire("corridor_a", "robot_2", now=12.0)

def test_expired_lease_can_be_reassigned():
    table = CorridorReservation(ttl_seconds=5.0)
    assert table.acquire("corridor_a", "robot_1", now=10.0)
    assert table.acquire("corridor_a", "robot_2", now=15.1)

def test_wrong_robot_cannot_release_lease():
    table = CorridorReservation(ttl_seconds=5.0)
    table.acquire("corridor_a", "robot_1", now=10.0)
    assert not table.release("corridor_a", "robot_2")
```

- [ ] **Step 2: Run pytest and verify RED because `CorridorReservation` does not exist.**

```bash
python3 -m pytest -q ros2_ws/src/mentorpi_fleet_sim/test/test_corridor_reservation.py
```

- [ ] **Step 3: Implement the minimal reservation table.**

Use immutable lease records and monotonic timestamps:

```python
@dataclass(frozen=True)
class Lease:
    robot_id: str
    expires_at: float

class CorridorReservation:
    def __init__(self, ttl_seconds: float): ...
    def acquire(self, resource_id: str, robot_id: str, now: float) -> bool: ...
    def renew(self, resource_id: str, robot_id: str, now: float) -> bool: ...
    def release(self, resource_id: str, robot_id: str) -> bool: ...
    def holder(self, resource_id: str, now: float) -> str | None: ...
```

- [ ] **Step 4: Run unit tests and verify GREEN.**

```bash
python3 -m pytest -q ros2_ws/src/mentorpi_fleet_sim/test/test_corridor_reservation.py
```

Expected: `3 passed`.

- [ ] **Step 5: Implement the collision-monitor chain and ownership contract.**

For each robot:

```text
controller_server cmd_vel -> cmd_vel_nav
velocity_smoother output -> cmd_vel_smoothed
collision_monitor output -> controller/cmd_vel
Gazebo planar base subscribes -> controller/cmd_vel
```

`tests/test_command_ownership.sh` fails if dispatcher code imports a Twist publisher or contains `controller/cmd_vel`; it requires `ActionClient(...NavigateToPose...)` and the collision-monitor output remap.

- [ ] **Step 6: Implement the fleet demo and integration test.**

The dispatcher validates `map_version == v001`, acquires `corridor_a`, sends a goal to robot 1, releases on exit/completion, then permits robot 2. `tests/test_fleet_runtime.sh` asserts logs never show two simultaneous holders, both robots accept goals, and stopping the dispatcher leaves both collision-monitor nodes active.

---

### Task 6: Operator Scripts, Full Verification, and Documentation

**Files:**
- Create: `scripts/build.sh`
- Create: `scripts/run-sim.sh`
- Create: `scripts/run-fleet-demo.sh`
- Create: `scripts/smoke-headless.sh`
- Modify: `scripts/shell.sh`
- Modify: `scripts/smoke-virtualgl-x11.sh`
- Replace: `scripts/verify.sh`
- Replace: `README.md`
- Create: `tests/test_operator_workflow.sh`

**Interfaces:**
- Consumes: all packages and tests from Tasks 1-5, SSH `DISPLAY`/Xauthority for GUI mode.
- Produces: documented one-command build, simulation, mapping, map save, navigation, fleet demo, headless verification, and GUI verification workflows.

- [ ] **Step 1: Write the failing operator-workflow contract.**

Require executable strict-mode scripts:

```bash
required=(
  scripts/build.sh scripts/run-sim.sh scripts/run-mapping.sh
  scripts/drive-mapping-route.sh scripts/save-map.sh scripts/promote-map.sh
  scripts/run-navigation.sh scripts/run-fleet-demo.sh
  scripts/smoke-headless.sh scripts/smoke-virtualgl-x11.sh scripts/verify.sh
)
for script in "${required[@]}"; do
  test -x "$script"
  grep -Fq 'set -euo pipefail' "$script"
done
```

The README test requires exact workflows for headless, X11 GUI, SLAM, promotion, Nav2, fleet demo, and the physical-robot safety boundary.

- [ ] **Step 2: Verify RED.**

```bash
bash tests/test_operator_workflow.sh
```

- [ ] **Step 3: Implement scripts and README.**

Commands exposed to the operator:

```bash
./scripts/build.sh
./scripts/smoke-headless.sh
./scripts/run-sim.sh
./scripts/run-mapping.sh
./scripts/drive-mapping-route.sh
./scripts/save-map.sh candidate-001
./scripts/promote-map.sh candidate-001 v001
./scripts/run-navigation.sh v001
./scripts/run-fleet-demo.sh v001
./scripts/smoke-virtualgl-x11.sh
./scripts/verify.sh
```

- [ ] **Step 4: Run static and unit verification.**

```bash
./scripts/verify.sh --static
python3 -m pytest -q ros2_ws/src/mentorpi_fleet_sim/test
```

Expected: no failures, no TurtleBot3 matches, and reservation tests pass.

- [ ] **Step 5: Build and run full headless verification.**

```bash
./scripts/build.sh
./scripts/smoke-headless.sh
```

The smoke test must freshly verify two entities, sensor topics, TF, SLAM/map artifacts, two active Nav2 stacks, fleet reservation ordering, and local collision-monitor independence.

- [ ] **Step 6: Run bounded X11 GUI verification.**

From Mac XQuartz through trusted forwarding:

```bash
ssh -Y -i ~/.ssh/id_ed25519_github_sesac_tracking_vehicle_wiki bae@ypddns.iptime.org
cd ~/sesac_logistics
./scripts/smoke-virtualgl-x11.sh
timeout 90s ./scripts/run-sim.sh
```

Verify direct AMD rendering, live `gzclient` and `rviz2`, two MentorPi entities, and no GLX/segmentation errors.

- [ ] **Step 7: Record final evidence.**

Capture exact image ID, VirtualGL version, Gazebo version, ROS distro, package list, map manifest checksums, test pass counts, renderer, and any hardware-only unverified items in the final handoff.
