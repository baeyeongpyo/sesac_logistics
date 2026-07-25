# MentorPi SLAM Session Artifacts Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `robot_1`의 시뮬레이션 센서와 ground-truth odom을 사용해 재현 가능한 SLAM 세션 산출물을 원자적으로 생성한다.

**Architecture:** `slam-mapper` 서비스는 `slam_toolbox`와 rosbag recorder를 세션 단위로 실행한다. 종료 시 지도와 pose graph를 저장하고 manifest/checksum을 생성한 후 `.inprogress` 디렉터리를 최종 세션 디렉터리로 rename한다.

**Tech Stack:** ROS 2 Humble, `slam_toolbox`, rosbag2, Python 3 standard library, Bash, Docker Compose, Python `unittest`

## Global Constraints

- mapping 대상은 우선 `robot_1`이다.
- 입력 계약은 `/clock`, `/tf`, `/tf_static`, `/robot_1/scan_raw`, `/robot_1/imu/data_raw`, `/robot_1/odom`이다.
- 결과는 `map.yaml`, `map.pgm`, pose graph, rosbag2, `manifest.json`, `checksums.sha256`를 포함한다.
- 완성되지 않은 세션은 최종 세션 경로에 나타나지 않는다.
- Nav2, 지도 병합, 차량 업로드, 운영 지도 전환은 포함하지 않는다.

## File Map

- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/`: SLAM 패키지
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/session_artifacts.py`: manifest/checksum 생성
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh`: 세션 수명주기
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/`: 순수 Python/스크립트 계약 테스트
- `vehicle_simulator_model/ubuntu/compose.yaml`: `slam-mapper`와 `slam-data` volume
- `vehicle_simulator_model/ubuntu/run.sh`: mapping-up, mapping-stop, mapping-status

---

### Task 1: Restore and pin the SLAM Toolbox package

**Files:**
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/CMakeLists.txt`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/package.xml`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/config/slam.yaml`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/launch/mapping.launch.py`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py`
- Modify: `vehicle_simulator_model/ubuntu/Dockerfile`

**Interfaces:**
- Produces: `ros2 launch mentorpi_slam mapping.launch.py`
- Consumes: robot_1 scan, odom, TF and `/clock`

- [ ] **Step 1: Write the failing package contract test**

```python
import unittest
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]


class SlamContractTest(unittest.TestCase):
    def test_mapping_frames_and_topic_are_pinned(self):
        text = (PACKAGE / 'config/slam.yaml').read_text()
        for line in (
            'use_sim_time: true',
            'mode: mapping',
            'map_frame: map',
            'odom_frame: robot_1/odom',
            'base_frame: robot_1/base_footprint',
            'scan_topic: /robot_1/scan_raw',
        ):
            self.assertIn(line, text)

    def test_launch_uses_sync_slam_toolbox(self):
        text = (PACKAGE / 'launch/mapping.launch.py').read_text()
        self.assertIn(\"executable='sync_slam_toolbox_node'\", text)
        self.assertIn(\"name='slam_toolbox'\", text)
```

- [ ] **Step 2: Run the test and verify the package is missing**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py -v
```

Expected: ERROR because `mentorpi_slam` assets do not exist.

- [ ] **Step 3: Restore the reviewed package from project artifacts**

Copy the existing `mentorpi_slam` package contract from `artifacts/vehicle/raw/ros2_ws/src/mentorpi_slam`, preserving `sync_slam_toolbox_node` and the reviewed parameter values. Add `ament_cmake_pytest` only if tests are registered through CMake; direct `unittest` remains the baseline.

- [ ] **Step 4: Install runtime dependencies**

Add to the runtime Dockerfile:

```dockerfile
ros-humble-slam-toolbox \
ros-humble-rosbag2 \
```

- [ ] **Step 5: Run the package and bundle tests**

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add vehicle_simulator_model/ubuntu/Dockerfile \
        vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam
git commit -m "feat: MentorPi slam_toolbox mapping 패키지 추가"
```

### Task 2: Deterministic manifest and checksums

**Files:**
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/session_artifacts.py`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/CMakeLists.txt`

**Interfaces:**
- Produces: `write_manifest(session_dir: Path, metadata: Mapping[str, str]) -> Path`
- Produces: `write_checksums(session_dir: Path) -> Path`

- [ ] **Step 1: Write deterministic artifact tests**

```python
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/session_artifacts.py'
SPEC = importlib.util.spec_from_file_location('session_artifacts', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_manifest_and_checksums_are_deterministic(self):
    with TemporaryDirectory() as directory:
        session = Path(directory)
        (session / 'map.yaml').write_text('resolution: 0.05\n')
        (session / 'map.pgm').write_bytes(b'P5\n1 1\n255\n\\x00')
        metadata = {
            'session_id': 'session-001',
            'robot_id': 'robot_1',
            'image_version': 'sha-f471f23',
            'git_commit': 'f471f23',
            'world_version': 'warehouse-v1',
            'model_version': 'mentorpi-m1-v1',
            'slam_params_sha256': 'a' * 64,
            'tf_calibration_version': 'ground-truth-v1',
            'created_at': '2026-07-26T00:00:00Z',
        }

        manifest_path = MODULE.write_manifest(session, metadata)
        checksum_path = MODULE.write_checksums(session)

        manifest = json.loads(manifest_path.read_text())
        self.assertEqual(manifest, metadata)
        lines = checksum_path.read_text().splitlines()
        self.assertEqual([line.split('  ', 1)[1] for line in lines],
                         ['manifest.json', 'map.pgm', 'map.yaml'])
```

- [ ] **Step 2: Run the test and verify the module is missing**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py -v
```

Expected: ERROR importing `session_artifacts.py`.

- [ ] **Step 3: Implement manifest validation**

Define the exact required fields:

```python
REQUIRED_FIELDS = (
    'session_id', 'robot_id', 'image_version', 'git_commit',
    'world_version', 'model_version', 'slam_params_sha256',
    'tf_calibration_version', 'created_at',
)
```

`write_manifest` rejects missing or empty values, writes sorted/indented JSON to `manifest.json.tmp`, then replaces `manifest.json`.

- [ ] **Step 4: Implement sorted SHA-256 output**

`write_checksums` recursively hashes regular files except `checksums.sha256` and temporary files. Sort by POSIX relative path and write `<digest>  <relative-path>`.

- [ ] **Step 5: Install and test the script**

Add:

```cmake
install(PROGRAMS
  scripts/session_artifacts.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py -v
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam
git commit -m "feat: SLAM 세션 manifest와 checksum 생성"
```

### Task 3: Atomic mapping session lifecycle

**Files:**
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh`
- Create: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py`
- Modify: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/CMakeLists.txt`

**Interfaces:**
- Consumes env: `SLAM_DATA_ROOT`, `SESSION_ID`, `IMAGE_VERSION`, `GIT_COMMIT`, `WORLD_VERSION`, `MODEL_VERSION`, `TF_CALIBRATION_VERSION`
- Produces: `${SLAM_DATA_ROOT}/${SESSION_ID}` only after successful finalization

- [ ] **Step 1: Write the script contract test**

```python
def test_script_records_required_topics_and_saves_artifacts(self):
    text = SCRIPT.read_text()
    for topic in ('/clock', '/tf', '/tf_static', '/robot_1/scan_raw',
                  '/robot_1/imu/data_raw', '/robot_1/odom'):
        self.assertIn(topic, text)
    self.assertIn('/slam_toolbox/save_map', text)
    self.assertIn('/slam_toolbox/serialize_map', text)
    self.assertIn('.inprogress', text)
    self.assertIn('session_artifacts.py', text)
    self.assertIn('mv \"$stage_dir\" \"$final_dir\"', text)
```

- [ ] **Step 2: Run and verify the script is missing**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
```

Expected: ERROR because the script does not exist.

- [ ] **Step 3: Implement strict input and staging creation**

Use `set -euo pipefail`. Reject an empty session ID and IDs containing anything outside `[A-Za-z0-9._-]`. Set:

```bash
stage_dir="${SLAM_DATA_ROOT}/.inprogress/${SESSION_ID}"
final_dir="${SLAM_DATA_ROOT}/${SESSION_ID}"
```

Fail if either path already exists. Create `posegraph` and `rosbag2` parent paths.

- [ ] **Step 4: Implement process start and signal trap**

Start `ros2 bag record` for the six pinned topics and `ros2 launch mentorpi_slam mapping.launch.py`. Store both PIDs. On `TERM` or `INT`, wait for SLAM services, call:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '${stage_dir}/map'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${stage_dir}/posegraph/mentorpi'}"
```

Stop rosbag with `SIGINT`, stop SLAM, and wait for both processes.

- [ ] **Step 5: Finalize metadata and atomically publish**

Calculate the SHA-256 of the installed `slam.yaml`. Invoke `session_artifacts.py` with all required metadata, generate checksums, verify `map.yaml` and `map.pgm` are non-empty, then rename staging to final.

- [ ] **Step 6: Install and test**

Install the shell script in `lib/mentorpi_slam`, run:

```bash
bash -n vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh
python3 -m unittest \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam
git commit -m "feat: 원자적 SLAM mapping 세션 실행 추가"
```

### Task 4: Compose mapping profile and operator commands

**Files:**
- Modify: `vehicle_simulator_model/ubuntu/compose.yaml`
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/README.md`

**Interfaces:**
- Produces: `slam-data` named volume with external name `mentorpi-slam-data`
- Produces: `./run.sh mapping-up <session-id>`, `./run.sh mapping-stop`, `./run.sh mapping-status <session-id>`

- [ ] **Step 1: Add failing mapping profile assertions**

Assert Compose contains `slam-mapper`, `profiles: [mapping]`, `slam-data:/slam-data`, dependency on healthy `sim-adapter`, and command `run_mapping_session.sh`. Assert `run.sh` validates the session ID and exports `SESSION_ID`.

- [ ] **Step 2: Run bundle tests and verify failure**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_bundle.py -v
```

Expected: FAIL because mapping service and commands do not exist.

- [ ] **Step 3: Add `slam-mapper`**

Use the runtime image, mapping profile, same ROS domain, `SLAM_DATA_ROOT=/slam-data`, and a named volume declared with `name: mentorpi-slam-data`:

```yaml
command: ros2 run mentorpi_slam run_mapping_session.sh
depends_on:
  sim-adapter:
    condition: service_healthy
```

- [ ] **Step 4: Add operator commands**

`mapping-up` requires exactly one session ID and runs:

```bash
export SESSION_ID="$2"
export IMAGE_VERSION="${IMAGE_VERSION:-mentorpi-sim:harmonic}"
export GIT_COMMIT="${GIT_COMMIT:-$(git -C "$BUNDLE_DIR" rev-parse HEAD)}"
export WORLD_VERSION="${WORLD_VERSION:-warehouse-v1}"
export MODEL_VERSION="${MODEL_VERSION:-mentorpi-m1-v1}"
export TF_CALIBRATION_VERSION="${TF_CALIBRATION_VERSION:-ground-truth-v1}"
"${COMPOSE[@]}" --profile mapping up -d \
  gazebo-server sim-adapter slam-mapper
```

`mapping-stop` sends `SIGINT` to `slam-mapper`, waits for container exit, then stops the remaining services. `mapping-status` lists the final session directory and validates checksums with `sha256sum -c`.

- [ ] **Step 5: Update documentation and tests**

Document start, safe stop, successful artifact layout, interrupted `.inprogress` behavior and volume backup.

Run:

```bash
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_slam_contract.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_session_artifacts.py \
  vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/test/test_mapping_session_script.py -v
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add vehicle_simulator_model/ubuntu
git commit -m "feat: SLAM mapping Compose 프로필 추가"
```

### Task 5: End-to-end mapping verification

**Files:**
- No planned file changes; this task is a verification gate

- [ ] **Step 1: Build and start a named session**

Run:

```bash
cd vehicle_simulator_model/ubuntu
./run.sh build
./run.sh mapping-up smoke-001
```

Expected: Gazebo, adapter and mapper become healthy/running.

- [ ] **Step 2: Drive robot_1 through the warehouse**

Publish bounded velocity commands for a square path, always ending with a zero command:

```bash
docker compose exec sim-adapter ros2 topic pub -r 5 \
  /robot_1/controller/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.12}, angular: {z: 0.0}}'
```

Repeat with rotation commands and stop each publisher with `Ctrl+C`; publish a final zero Twist.

- [ ] **Step 3: Finalize and verify**

Run:

```bash
./run.sh mapping-stop
./run.sh mapping-status smoke-001
```

Expected: all six artifact groups exist, checksums pass, and no final directory was exposed before stop.

- [ ] **Step 4: Inspect map dimensions and manifest**

Run:

```bash
docker run --rm -v mentorpi-slam-data:/data:ro alpine \
  sh -c 'test -s /data/smoke-001/map.yaml && test -s /data/smoke-001/map.pgm && cat /data/smoke-001/manifest.json'
```

Expected: non-empty map files and complete metadata.
