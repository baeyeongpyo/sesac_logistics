# MentorPi Harmonic Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fortress/X11 기반 단일 컨테이너를 ROS 2 Humble + Gazebo Harmonic의 Headless 서버와 sim-adapter 서비스로 전환한다.

**Architecture:** `gazebo-server`는 물리·센서만 실행하고 `sim-adapter`는 모델 spawn, ROS bridge, robot_state_publisher, ground-truth odom을 담당한다. 두 서비스는 내부 Compose 네트워크와 동일한 `GZ_PARTITION`만 공유하며 외부 포트를 공개하지 않는다.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic 8.x, `ros-humble-ros-gzharmonic`, Docker Compose, Python `unittest`, ROS 2 launch

## Global Constraints

- 서버 컨테이너는 Ubuntu 22.04와 `linux/amd64`를 사용한다.
- ROS 2 Humble을 유지하고 `ros-humble-ros-gzharmonic`을 사용한다.
- ROS 토픽 이름과 `robot_1`, `robot_2` namespace를 유지한다.
- 서버 실행 경로에서 X11, Xauthority, XQuartz, VirtualGL, `DISPLAY`를 제거한다.
- 초기 odom은 `gz_pose_to_odom.py`의 ground-truth odom을 유지한다.
- Gazebo Transport와 ROS 2 DDS 포트를 외부에 공개하지 않는다.

## File Map

- `vehicle_simulator_model/ubuntu/Dockerfile`: Harmonic/ROS 런타임 이미지와 향후 webbridge용 build stage의 기반
- `vehicle_simulator_model/ubuntu/entrypoint.sh`: 서비스 시작 식별자 로그와 ROS 환경 source
- `vehicle_simulator_model/ubuntu/compose.yaml`: `gazebo-server`, `sim-adapter` 서비스
- `vehicle_simulator_model/ubuntu/compose.gpu.yaml`: `/dev/dri` EGL 가속 override
- `vehicle_simulator_model/ubuntu/run.sh`: build, test, sim-up, down, logs, fork-up 운영 명령
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/gazebo_server.launch.py`: Gazebo Headless 전용 launch
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/sim_adapter.launch.py`: 두 로봇 spawn과 ROS adapter launch
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/two_robot_sim.launch.py`: 로컬 통합 검증용 wrapper
- `vehicle_simulator_model/ubuntu/test/test_bundle.py`: Docker/Compose 배포 계약
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py`: Harmonic SDF 계약
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py`: launch 분리 계약
- `vehicle_simulator_model/ubuntu/README.md`: 개발·서버 실행 문서

---

### Task 1: Harmonic 런타임 이미지 계약

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/Dockerfile`
- Delete: `vehicle_simulator_model/ubuntu/vendor/virtualgl_3.1.4_amd64.deb`

**Interfaces:**
- Produces: `runtime` build target containing `gz sim`, `ros_gz_bridge`, `ros_gz_image`, `ros_gz_sim`

- [ ] **Step 1: Write the failing Dockerfile contract test**

Replace the VirtualGL assertions with:

```python
def test_runtime_image_uses_humble_with_harmonic(self):
    dockerfile = (BUNDLE / 'Dockerfile').read_text()
    self.assertIn('FROM osrf/ros:humble-desktop-full-jammy AS runtime', dockerfile)
    self.assertIn('https://packages.osrfoundation.org/gazebo.gpg', dockerfile)
    self.assertIn('gz-harmonic', dockerfile)
    self.assertIn('ros-humble-ros-gzharmonic', dockerfile)
    for removed in ('ros-humble-ros-gz \\\\', 'VirtualGL', 'x11-apps', 'xauth', 'dbus-x11'):
        self.assertNotIn(removed, dockerfile)
    self.assertFalse((BUNDLE / 'vendor/virtualgl_3.1.4_amd64.deb').exists())
```

- [ ] **Step 2: Run the test and verify Fortress/VirtualGL expectations fail**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: FAIL because the Dockerfile still installs `ros-humble-ros-gz` and VirtualGL.

- [ ] **Step 3: Replace the runtime image installation block**

Use the official OSRF repository and non-default Humble/Harmonic pairing:

```dockerfile
FROM osrf/ros:humble-desktop-full-jammy AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-harmonic \
        libxml2-utils \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-ros-gzharmonic \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*
```

Keep the workspace build, non-root user, entrypoint, and `/ws` working directory. Remove the VirtualGL copy, checksum, PATH, X11 and Qt environment entries.

- [ ] **Step 4: Remove the vendored VirtualGL package and run tests**

Run:

```bash
git rm vehicle_simulator_model/ubuntu/vendor/virtualgl_3.1.4_amd64.deb
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: Dockerfile contract test passes; Compose-related legacy assertions may still fail until Task 4.

- [ ] **Step 5: Commit**

```bash
git add vehicle_simulator_model/ubuntu/Dockerfile vehicle_simulator_model/ubuntu/test/test_bundle.py
git commit -m "build: Humble 런타임을 Gazebo Harmonic으로 전환"
```

### Task 2: World, model and bridge Harmonic migration

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/mentorpi_m1/model.sdf.xacro`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/config/robot_1_bridge.yaml`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/config/robot_2_bridge.yaml`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/package.xml`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py`

**Interfaces:**
- Produces: Harmonic-compatible SDF 1.10 model and `gz.msgs.*` bridge configuration
- Preserves: all existing ROS topic names and robot frames

- [ ] **Step 1: Change the model contract test to require Harmonic names**

Add:

```python
def test_gazebo_assets_use_harmonic_names(self):
    world = (SOURCE / 'mentorpi_gz_sim/worlds/warehouse.sdf').read_text()
    model = SDF.read_text()
    bridges = '\n'.join(path.read_text() for path in BRIDGE_CONFIGS)
    combined = '\n'.join((world, model, bridges))

    self.assertIn('gz-sim-physics-system', world)
    self.assertIn('gz::sim::systems::MecanumDrive', model)
    self.assertIn('gz.msgs.LaserScan', bridges)
    self.assertIn('xmlns:gz="http://gazebosim.org/schema"', model)
    self.assertNotIn('ignition-gazebo', combined)
    self.assertNotIn('ignition::gazebo', combined)
    self.assertNotIn('ignition.msgs', combined)
    self.assertNotIn('ignition:expressed_in', combined)
```

Update the existing drive plugin and Float64 bridge expectations to `gz::sim` and `gz.msgs.Double`.

- [ ] **Step 2: Run the model contract test and verify failure**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py -v
```

Expected: FAIL on Fortress plugin and message names.

- [ ] **Step 3: Apply exact Harmonic replacements**

Use these plugin pairs:

```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
```

Use `gz-sim-mecanum-drive-system`, `gz-sim-joint-position-controller-system`, namespace `gz::sim::systems`, XML namespace `xmlns:gz="http://gazebosim.org/schema"`, and `<fdir1 gz:expressed_in="base_footprint">`.

Replace every bridge type prefix `ignition.msgs.` with `gz.msgs.` and change the package description to “Gazebo Harmonic simulation”.

- [ ] **Step 4: Run model and bundle tests**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py \
  vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: Harmonic model tests pass.

- [ ] **Step 5: Commit**

```bash
git add vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim \
        vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py
git commit -m "feat: MentorPi 월드와 브리지를 Harmonic으로 마이그레이션"
```

### Task 3: Gazebo server and sim-adapter launch split

**Files:**
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/gazebo_server.launch.py`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/sim_adapter.launch.py`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/two_robot_sim.launch.py`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py`

**Interfaces:**
- Produces: `gazebo_server.launch.py verbosity:=2`
- Produces: `sim_adapter.launch.py`
- Preserves: `two_robot_sim.launch.py` as a local combined wrapper

- [ ] **Step 1: Write a launch boundary test**

```python
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch'


class HarmonicLaunchContractTest(unittest.TestCase):
    def test_server_launch_only_starts_gazebo(self):
        text = (LAUNCH / 'gazebo_server.launch.py').read_text()
        self.assertIn(\"'-r -s --headless-rendering\", text)
        self.assertIn(\"get_package_share_directory('ros_gz_sim')\", text)
        self.assertNotIn('robot_state_publisher', text)
        self.assertNotIn(\"executable='create'\", text)

    def test_adapter_launch_owns_spawn_and_bridges(self):
        text = (LAUNCH / 'sim_adapter.launch.py').read_text()
        for token in ('robot_state_publisher', \"executable='create'\", 'parameter_bridge',
                      'image_bridge', 'gz_pose_to_odom.py'):
            self.assertIn(token, text)
        self.assertNotIn('gz_sim.launch.py', text)

    def test_combined_launch_includes_both_boundaries(self):
        text = (LAUNCH / 'two_robot_sim.launch.py').read_text()
        self.assertIn('gazebo_server.launch.py', text)
        self.assertIn('sim_adapter.launch.py', text)
```

- [ ] **Step 2: Run the boundary test and verify missing launch files**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py -v
```

Expected: ERROR because the two new launch files do not exist.

- [ ] **Step 3: Extract the Gazebo server launch**

Implement `gazebo_server.launch.py` with `verbosity` launch argument, the installed warehouse path, and:

```python
gz_args = f'-r -s --headless-rendering -v {verbosity} {world}'
return LaunchDescription([
    DeclareLaunchArgument('verbosity', default_value='2'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('ros_gz_sim')) / 'launch' / 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    ),
])
```

- [ ] **Step 4: Move `_robot_nodes` and both robot groups to `sim_adapter.launch.py`**

Keep the current description, SDF xacro mapping, bridge configuration, image bridge and ground-truth odom node. `generate_launch_description()` returns only both robot groups.

- [ ] **Step 5: Replace `two_robot_sim.launch.py` with a wrapper**

Include both installed launch files using `IncludeLaunchDescription`. Pass `verbosity` only to the server launch.

- [ ] **Step 6: Run launch and existing odom tests**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_gz_pose_to_odom.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch \
        vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test
git commit -m "refactor: Gazebo 서버와 ROS adapter launch 분리"
```

### Task 4: Headless Compose operations

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/compose.yaml`
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Modify: `vehicle_simulator_model/ubuntu/entrypoint.sh`
- Modify: `vehicle_simulator_model/ubuntu/README.md`

**Interfaces:**
- Produces: `./run.sh sim-up`, `./run.sh down`, `./run.sh logs`, `./run.sh test`, `./run.sh fork-up`
- Produces: internal `mentorpi` network with `GZ_PARTITION=mentorpi-sim`

- [ ] **Step 1: Replace legacy Compose assertions**

Assert:

```python
compose = (BUNDLE / 'compose.yaml').read_text()
for service in ('gazebo-server:', 'sim-adapter:'):
    self.assertIn(service, compose)
for required in ('GZ_PARTITION: mentorpi-sim', 'condition: service_healthy',
                 'LIBGL_ALWAYS_SOFTWARE'):
    self.assertIn(required, compose)
for removed in ('mentorpi-gui:', 'DISPLAY:', 'XAUTHORITY:', 'VirtualGL',
                '/dev/dri/renderD128:/dev/dri/renderD128'):
    self.assertNotIn(removed, compose)
for forbidden_port in ('10317:', '10318:', '9002:'):
    self.assertNotIn(forbidden_port, compose)
```

Assert `run.sh` includes `sim-up`, `down`, `logs`, `test`, `fork-up` and excludes `ssh -Y`, `vglrun`, `DISPLAY`.

Assert `entrypoint.sh` contains `SERVICE_NAME`, `IMAGE_VERSION`, `SESSION_ID`, and `ROBOT_IDS`.

- [ ] **Step 2: Run bundle tests and verify legacy Compose fails**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: FAIL on `mentorpi-gui` and X11 configuration.

- [ ] **Step 3: Implement Compose services**

Define a shared runtime anchor with image, platform, `GZ_PARTITION`, `ROS_DOMAIN_ID`, internal network and restart policy. Use:

```yaml
services:
  gazebo-server:
    <<: *runtime
    command: ros2 launch mentorpi_gz_sim gazebo_server.launch.py
    healthcheck:
      test: ['CMD-SHELL', 'gz topic -l | grep -q /world/mentorpi_warehouse/stats']
      interval: 5s
      timeout: 3s
      retries: 20

  sim-adapter:
    <<: *runtime
    command: ros2 launch mentorpi_gz_sim sim_adapter.launch.py
    depends_on:
      gazebo-server:
        condition: service_healthy
    healthcheck:
      test: ['CMD-SHELL', 'ros2 topic list | grep -q /robot_1/scan_raw']
      interval: 5s
      timeout: 3s
      retries: 20
```

Create `compose.gpu.yaml` as the explicit GPU execution profile:

```yaml
services:
  gazebo-server:
    devices:
      - /dev/dri:/dev/dri
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
```

The base service uses `LIBGL_ALWAYS_SOFTWARE: "${LIBGL_ALWAYS_SOFTWARE:-1}"`. `run.sh sim-up gpu` adds `-f compose.gpu.yaml`; normal `sim-up` uses the software-rendered base file. Do not mount source into server services.

- [ ] **Step 4: Add startup identity logging**

Before `exec "$@"`, log one stable line:

```bash
printf 'mentorpi service=%s image_version=%s session_id=%s robot_ids=%s\n' \
  "${SERVICE_NAME:-unknown}" \
  "${IMAGE_VERSION:-development}" \
  "${SESSION_ID:-none}" \
  "${ROBOT_IDS:-robot_1,robot_2}"
```

Set `SERVICE_NAME` per Compose service and pass `IMAGE_VERSION` through the shared runtime environment.

- [ ] **Step 5: Replace run commands**

`sim-up` runs `docker compose up -d gazebo-server sim-adapter`; `down` runs `docker compose down`; `logs` follows both services; `test` runs Python tests then a one-shot runtime container checking `gz sim --versions` and package prefixes.

- [ ] **Step 6: Rewrite README around immutable images and Headless hosting**

Document Mac native GUI development, `linux/amd64` image build, server `sim-up`, logs, stop, fork command, and the separation between browser rendering and server sensor offscreen rendering.

- [ ] **Step 7: Run static verification**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_description/test/test_original_model.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_gz_pose_to_odom.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py -v
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add vehicle_simulator_model/ubuntu/compose.yaml \
        vehicle_simulator_model/ubuntu/compose.gpu.yaml \
        vehicle_simulator_model/ubuntu/run.sh \
        vehicle_simulator_model/ubuntu/entrypoint.sh \
        vehicle_simulator_model/ubuntu/README.md \
        vehicle_simulator_model/ubuntu/test/test_bundle.py
git commit -m "feat: Harmonic Headless 서버 운영 구성 추가"
```

### Task 5: Linux/amd64 runtime integration verification

**Files:**
- No planned file changes; this task is a verification gate

**Interfaces:**
- Verifies: Harmonic 8.x, two robots, scan/IMU/depth, ground-truth odom and TF

- [ ] **Step 1: Build the target image**

Run:

```bash
docker buildx build --load --platform linux/amd64 \
  -t mentorpi-sim:harmonic \
  vehicle_simulator_model/ubuntu
```

Expected: image build completes and workspace packages build.

- [ ] **Step 2: Verify runtime versions**

Run:

```bash
docker run --rm --platform linux/amd64 mentorpi-sim:harmonic \
  bash -lc 'gz sim --versions && ros2 pkg prefix ros_gz_sim && ros2 pkg prefix mentorpi_gz_sim'
```

Expected: `8.x` and both package prefixes.

- [ ] **Step 3: Start the simulation and inspect health**

Run:

```bash
cd vehicle_simulator_model/ubuntu
./run.sh sim-up
docker compose ps
docker compose exec sim-adapter ros2 topic list
```

Expected: both services healthy and required robot topics listed.

- [ ] **Step 4: Verify data updates**

Run:

```bash
docker compose exec sim-adapter ros2 topic echo --once /robot_1/scan_raw
docker compose exec sim-adapter ros2 topic echo --once /robot_1/imu/data_raw
docker compose exec sim-adapter ros2 topic echo --once /robot_1/odom
docker compose exec sim-adapter timeout 5 ros2 run tf2_ros tf2_echo \
  robot_1/odom robot_1/base_footprint
```

Expected: each topic returns one message and TF reports a connected transform.

- [ ] **Step 5: Stop and record verification**

Run:

```bash
./run.sh down
git status --short
```

Expected: services stop and the worktree contains no generated runtime files.

If a command fails, stop this task and return to the task that owns the failing contract. Do not add a runtime workaround in this verification step.
