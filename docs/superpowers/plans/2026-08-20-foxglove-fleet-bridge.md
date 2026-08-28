# Foxglove Fleet Bridge Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 차량 ROS Domain 215/216의 설정 기반 telemetry를 Foxglove WebSocket으로 받아 서버 Domain 225에 재발행하고, 서버 Foxglove endpoint 하나로 관제할 수 있는 독립 `fleet_bridge/` 번들을 만든다.

**Architecture:** 차량은 Fast DDS를 host network/IPC 안에서만 사용하고, 설정에 따라 rate/on-change filter를 거친 topic만 pin된 Foxglove Bridge 0.8.5로 노출한다. 서버의 차량별 Python worker는 `foxglove.websocket.v1` CDR frame을 수신해 설정된 QoS로 Domain 225에 발행하며, 서버 Foxglove Bridge가 두 차량 namespace를 관제 endpoint로 노출한다.

**Tech Stack:** ROS 2 Humble, Ubuntu 22.04, Fast DDS (`rmw_fastrtps_cpp`), Foxglove Bridge 0.8.5, Python 3.10, rclpy, PyYAML, python3-websockets, Docker Compose

**Spec:** `docs/superpowers/specs/2026-08-20-foxglove-fleet-bridge-design.md`

## Global Constraints

- 차량은 `robot_1=215`, `robot_2=216`, 서버는 `ROS_DOMAIN_ID=225`를 사용한다.
- 차량과 서버 이미지는 ROS 2 Humble/Ubuntu 22.04를 사용한다.
- Foxglove Bridge는 commit `41f96cc6053632a472d9a821989952771b1117f2`(tag `0.8.5`)로 pin한다.
- 차량 DDS는 `network_mode: host`, `ipc: host`, `FASTDDS_BUILTIN_TRANSPORTS=DEFAULT`를 기본으로 한다.
- 차량 명령은 기존 Fleet Manager REST 경로를 유지하고 Foxglove client publish를 활성화하지 않는다.
- 기존 `vehicle_simulator_model/ubuntu/dds-observation`과 사용자 변경 파일은 수정하지 않는다.
- 설정은 read-only volume으로 주입하고 초기 구현에는 hot reload를 넣지 않는다.

---

### Task 1: 공통 fleet/telemetry 설정 계약

**Files:**
- Create: `fleet_bridge/common/fleet_bridge_config/package.xml`
- Create: `fleet_bridge/common/fleet_bridge_config/setup.py`
- Create: `fleet_bridge/common/fleet_bridge_config/setup.cfg`
- Create: `fleet_bridge/common/fleet_bridge_config/resource/fleet_bridge_config`
- Create: `fleet_bridge/common/fleet_bridge_config/fleet_bridge_config/__init__.py`
- Create: `fleet_bridge/common/fleet_bridge_config/fleet_bridge_config/models.py`
- Create: `fleet_bridge/common/fleet_bridge_config/fleet_bridge_config/loader.py`
- Create: `fleet_bridge/common/fleet_bridge_config/test/test_loader.py`
- Create: `fleet_bridge/config/fleet.yaml`
- Create: `fleet_bridge/config/telemetry.yaml`

**Interfaces:**
- Produces: `load_fleet(path, environ) -> FleetConfig`
- Produces: `load_telemetry(path, robot_id) -> tuple[TopicConfig, ...]`
- Produces: immutable dataclasses `FleetConfig`, `VehicleConfig`, `TopicConfig`, `FilterConfig`, `RateConfig`, `QosConfig`

- [ ] **Step 1: 유효 설정과 오류 계약 테스트 작성**

```python
def test_load_telemetry_expands_robot_and_preserves_filter_and_qos(tmp_path):
    path = write_yaml(tmp_path, TELEMETRY_FIXTURE)
    topics = load_telemetry(path, "robot_1")
    scan = next(topic for topic in topics if topic.id == "scan")
    assert scan.source == "/robot_1/scan"
    assert scan.uplink == "/robot_1/fleet_bridge/scan"
    assert scan.filter.mode == "rate"
    assert scan.qos.reliability == "best_effort"

def test_load_telemetry_rejects_duplicate_uplink(self):
    path = self.write_yaml(DUPLICATE_UPLINK_FIXTURE)
    with self.assertRaisesRegex(ConfigError, "duplicate uplink"):
        load_telemetry(path, "robot_1")
```

- [ ] **Step 2: 설정 테스트가 import 실패로 RED인지 확인**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config python3 -m unittest discover -s fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v`

Expected: FAIL because `fleet_bridge_config.loader` does not exist.

- [ ] **Step 3: 엄격한 dataclass와 YAML loader 구현**

`loader.py`는 `${NAME}` 환경변수만 fleet YAML에서 치환하고, telemetry의 `{robot}`만 robot ID로 치환한다. 허용 key를 집합으로 검사하며 message type은 `^[A-Za-z][A-Za-z0-9_]*/msg/[A-Za-z][A-Za-z0-9_]*$`, topic은 절대 경로로 검증한다. rate는 양수, QoS는 `best_effort|reliable`, `volatile|transient_local`, `keep_last`만 허용한다.

- [ ] **Step 4: 공통 설정 테스트 통과 확인**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config python3 -m unittest discover -s fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v`

Expected: all config tests PASS.

- [ ] **Step 5: 공통 설정과 예제 정책 커밋**

```bash
git add fleet_bridge/common fleet_bridge/config/fleet.yaml fleet_bridge/config/telemetry.yaml
git commit -m "feat: add fleet bridge configuration contract"
```

### Task 2: 차량 telemetry filter와 Foxglove launch 정책

**Files:**
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/package.xml`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/setup.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/setup.cfg`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/resource/fleet_telemetry_filter`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/fleet_telemetry_filter/__init__.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/fleet_telemetry_filter/policy.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/fleet_telemetry_filter/node.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/fleet_telemetry_filter/launch_config.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/launch/vehicle_foxglove.launch.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test/test_policy.py`
- Create: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test/test_launch_config.py`

**Interfaces:**
- Consumes: `TopicConfig` from Task 1
- Produces: `ForwardPolicy.should_forward(message, now_ns) -> bool`
- Produces: `bridge_parameters(topics, mode, port) -> dict[str, object]`
- Produces: executable `telemetry_filter` and launch file `vehicle_foxglove.launch.py`

- [ ] **Step 1: rate/on-change/heartbeat/critical bypass 테스트 작성**

```python
def test_rate_policy_drops_samples_inside_period():
    policy = ForwardPolicy(rate_filter(2.0))
    assert policy.should_forward(object(), 0)
    assert not policy.should_forward(object(), 100_000_000)
    assert policy.should_forward(object(), 500_000_000)

def test_battery_critical_sample_bypasses_rate_limit():
    policy = ForwardPolicy(battery_filter())
    assert policy.should_forward(Battery(0.50, 12.3), 0)
    assert policy.should_forward(Battery(0.19, 12.2), 1_000_000)
```

- [ ] **Step 2: policy 테스트 RED 확인**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter python3 -m unittest discover -s fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test -p 'test_*.py' -v`

Expected: FAIL because `ForwardPolicy` and `bridge_parameters` do not exist.

- [ ] **Step 3: 순수 policy와 Bridge parameter 생성 구현**

`ForwardPolicy`는 monotonic nanoseconds를 사용하고 `rate`, `on_change`, heartbeat, numeric field threshold, critical-lower-than 우회를 구현한다. `bridge_parameters`는 fleet mode에서 enabled topic의 `uplink`, debug mode에서 `debug=true`인 `source`를 exact regex로 만들고, service/param/client topic whitelist는 `(?!)`로 설정한다. Humble이 빈 string array override를 처리하지 못하므로 capabilities는 pin된 Bridge의 어떤 알려진 기능과도 일치하지 않는 `none` sentinel 하나를 사용한다.

- [ ] **Step 4: rclpy 동적 filter node와 launch 구현**

Node는 `rosidl_runtime_py.utilities.get_message()`로 type을 불러오고 source subscriber와 uplink publisher를 생성한다. passthrough 항목은 node에서 재발행하지 않는다. launch는 `ROBOT_ID`, `TELEMETRY_CONFIG`, `FOXGLOVE_MODE`, `FOXGLOVE_PORT`를 읽어 filter와 `foxglove_bridge/foxglove_bridge`를 시작한다.

- [ ] **Step 5: 차량 policy/launch 테스트 통과 확인**

Run: Task 2 Step 2 command

Expected: all vehicle pure-Python tests PASS without a ROS installation.

- [ ] **Step 6: 차량 filter 커밋**

```bash
git add fleet_bridge/vehicle/ros2_ws
git commit -m "feat: add configurable vehicle telemetry filter"
```

### Task 3: Foxglove WebSocket v1 protocol과 worker 상태

**Files:**
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/package.xml`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/setup.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/setup.cfg`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/resource/foxglove_ros_worker`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/__init__.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/protocol.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/state.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_protocol.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_state.py`

**Interfaces:**
- Produces: `parse_server_message(text) -> ServerInfo | Advertise | Unadvertise | IgnoredMessage`
- Produces: `subscribe_message(pairs) -> str`
- Produces: `parse_message_frame(data) -> MessageFrame`
- Produces: `ReconnectBackoff.next_delay() -> float`
- Produces: thread-safe `WorkerState.snapshot(now) -> dict[str, object]`

- [ ] **Step 1: JSON/binary frame/backoff/status 테스트 작성**

```python
def test_parse_message_frame_extracts_subscription_timestamp_and_cdr():
    frame = parse_message_frame(b"\x01" + struct.pack("<IQ", 7, 1234) + b"cdr")
    assert frame.subscription_id == 7
    assert frame.timestamp_ns == 1234
    assert frame.payload == b"cdr"

def test_backoff_caps_at_thirty_seconds():
    backoff = ReconnectBackoff(initial=1.0, maximum=30.0)
    assert [backoff.next_delay() for _ in range(7)] == [1, 2, 4, 8, 16, 30, 30]
```

- [ ] **Step 2: protocol/state 테스트 RED 확인**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest discover -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_protocol.py' -v`

Expected: FAIL because protocol module does not exist.

- [ ] **Step 3: protocol parser와 상태 모델 구현**

Binary Message Data는 opcode 1, little-endian uint32 subscription ID, uint64 timestamp, 나머지 CDR payload로 파싱한다. 13 byte보다 짧거나 opcode가 다르면 `ProtocolError`를 발생시킨다. Advertise channel은 id/topic/encoding/schemaName을 필수로 검증한다.

- [ ] **Step 4: protocol/state 테스트 통과 확인**

Run: `PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest discover -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_*.py' -v`

Expected: protocol and state tests PASS.

- [ ] **Step 5: protocol/state 커밋**

```bash
git add fleet_bridge/server/ros2_ws/src/foxglove_ros_worker
git commit -m "feat: add foxglove websocket worker protocol"
```

### Task 4: ROS CDR republisher와 worker runtime

**Files:**
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/republisher.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/foxglove_ros_worker/main.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_republisher.py`
- Create: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_worker.py`

**Interfaces:**
- Consumes: Task 1 `TopicConfig`, Task 3 protocol/state
- Produces: `qos_kwargs(config) -> dict[str, object]` for ROS policy mapping tests
- Produces: `RateGate.allow(key, now_ns) -> bool`
- Produces: `FoxgloveWorker.run_forever() -> Awaitable[None]`
- Produces: console script `foxglove_ros_worker`

- [ ] **Step 1: channel type 검증, rate gate, QoS mapping 테스트 작성**

```python
def test_channel_is_accepted_only_for_configured_cdr_type():
    selector = ChannelSelector((odom_topic(),))
    assert selector.select(Channel(1, "/robot_1/odom", "cdr", "nav_msgs/msg/Odometry"))
    assert selector.select(Channel(2, "/robot_1/odom", "json", "nav_msgs/msg/Odometry")) is None

def test_qos_mapping_preserves_transient_local():
    assert qos_kwargs(tf_static_topic().qos) == {
        "reliability": "reliable", "durability": "transient_local",
        "history": "keep_last", "depth": 1,
    }
```

- [ ] **Step 2: republisher/worker 테스트 RED 확인**

Run: Task 3 Step 4 command

Expected: FAIL because `republisher` and `main` do not exist.

- [ ] **Step 3: ROS 의존성과 순수 정책을 분리한 republisher 구현**

모듈 import 시 ROS가 없는 host test도 실행되게 rclpy import는 `RosRepublisher` 생성 시점에 수행한다. 실제 container에서는 `deserialize_message`, `get_message`, 설정 QoS publisher를 사용하며 원본 header stamp와 frame ID를 변경하지 않는다.

- [ ] **Step 4: async WebSocket worker 구현**

worker는 `websockets.connect(uri, subprotocols=["foxglove.websocket.v1"])`로 접속한다. advertise마다 설정된 CDR channel만 subscribe하고 subscription ID mapping을 유지한다. malformed frame은 연결을 끊고 1~30초 backoff로 재접속하며, rclpy executor는 별도 thread에서 실행한다.

- [ ] **Step 5: 전체 worker 단위 테스트 통과 확인**

Run: Task 3 Step 4 command

Expected: all worker tests PASS.

- [ ] **Step 6: worker runtime 커밋**

```bash
git add fleet_bridge/server/ros2_ws/src/foxglove_ros_worker
git commit -m "feat: republish foxglove cdr telemetry to ros"
```

### Task 5: Humble 이미지와 Docker Compose 배포 계약

**Files:**
- Create: `fleet_bridge/vehicle/Dockerfile`
- Create: `fleet_bridge/vehicle/entrypoint.sh`
- Create: `fleet_bridge/server/Dockerfile`
- Create: `fleet_bridge/server/entrypoint.sh`
- Create: `fleet_bridge/docker-compose.vehicle.yaml`
- Create: `fleet_bridge/docker-compose.server.yaml`
- Create: `fleet_bridge/config/server_foxglove.yaml`
- Create: `fleet_bridge/.env.example`
- Create: `fleet_bridge/test/test_compose_contract.py`
- Create: `fleet_bridge/test/test_bundle_contract.py`

**Interfaces:**
- Consumes: all ROS packages and config files from Tasks 1-4
- Produces: image targets `mentorpi-fleet-bridge-vehicle:humble` and `mentorpi-fleet-bridge-server:humble`
- Produces: Compose services `foxglove-fleet`, `foxglove-debug`, `worker-robot-1`, `worker-robot-2`, `server-foxglove`

- [ ] **Step 1: Compose와 Dockerfile 정적 계약 테스트 작성**

```python
def test_vehicle_services_share_host_network_and_ipc():
    services = compose_config("docker-compose.vehicle.yaml")["services"]
    assert services["foxglove-fleet"]["network_mode"] == "host"
    assert services["foxglove-fleet"]["ipc"] == "host"
    assert services["foxglove-debug"]["profiles"] == ["debug"]

def test_server_workers_use_domain_225_and_read_only_config_mounts():
    services = compose_config("docker-compose.server.yaml")["services"]
    assert services["worker-robot-1"]["environment"]["ROS_DOMAIN_ID"] == "225"
```

- [ ] **Step 2: 번들 계약 테스트 RED 확인**

Run: `python3 -m unittest discover -s fleet_bridge/test -p 'test_*.py' -v`

Expected: FAIL because Dockerfiles and Compose files do not exist.

- [ ] **Step 3: pin된 Foxglove Bridge source build 이미지 구현**

두 Dockerfile은 build stage에서 commit SHA를 checkout하고 필요한 ROS package와 함께 colcon build한다. 차량 runtime은 filter와 bridge를, 서버 runtime은 worker와 bridge를 포함한다. entrypoint는 ROS 및 workspace setup을 source한 뒤 전달된 command를 exec한다.

- [ ] **Step 4: 차량/서버 Compose와 read-only config mount 구현**

차량 fleet 서비스는 port 8766, debug profile은 8765를 사용한다. 서버 worker 두 개는 각각 robot ID와 URI를 받고 Domain 225를 사용한다. 모든 설정 mount는 `read_only: true`로 선언한다.

- [ ] **Step 5: Compose와 번들 계약 테스트 통과 확인**

Run: Task 5 Step 2 command

Expected: all static/Compose contract tests PASS.

- [ ] **Step 6: 배포 번들 커밋**

```bash
git add fleet_bridge
git commit -m "feat: add foxglove fleet bridge docker bundle"
```

### Task 6: 운영 문서와 전체 검증

**Files:**
- Create: `fleet_bridge/README.md`
- Modify: `docs/superpowers/specs/2026-08-20-foxglove-fleet-bridge-design.md`
- Test: `fleet_bridge/common/fleet_bridge_config/test/test_loader.py`
- Test: `fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test/test_policy.py`
- Test: `fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test/test_protocol.py`
- Test: `fleet_bridge/test/test_compose_contract.py`

**Interfaces:**
- Produces: 차량/서버 build, 시작, debug profile, scan/battery 설정 변경, A/B 네트워크 검증 절차

- [ ] **Step 1: README 필수 운영 절차 계약 테스트 추가**

README 테스트는 `docker compose --env-file`, `ROBOT_ID`, `ROS_DOMAIN_ID`, `scan`, `battery`, `ping`, `docker stats`, `ros2 topic hz` 문구와 기존 Domain Bridge 중지 경고를 요구한다.

- [ ] **Step 2: README 계약 RED 확인**

Run: `python3 -m unittest discover -s fleet_bridge/test -p 'test_*.py' -v`

Expected: FAIL because README does not exist.

- [ ] **Step 3: 한국어 운영 README 작성**

차량별 `.env` 예시, vehicle/server 이미지 build, Compose 시작/중지, debug profile, config 수정 후 재시작, 기존 Domain Bridge 중복 publisher 방지, 실제 LAN A/B 측정 절차를 포함한다.

- [ ] **Step 4: 모든 host 단위/계약 테스트 실행**

```bash
PYTHONPATH=fleet_bridge/common/fleet_bridge_config python3 -m unittest discover -s fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v
PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter python3 -m unittest discover -s fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test -p 'test_*.py' -v
PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker python3 -m unittest discover -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_*.py' -v
python3 -m unittest discover -s fleet_bridge/test -p 'test_*.py' -v
```

Expected: all tests PASS with no warnings or errors.

- [ ] **Step 5: Compose config 검증**

```bash
docker compose --env-file fleet_bridge/.env.example -f fleet_bridge/docker-compose.vehicle.yaml config --quiet
docker compose --env-file fleet_bridge/.env.example -f fleet_bridge/docker-compose.server.yaml config --quiet
```

Expected: both commands exit 0.

- [ ] **Step 6: Docker image build와 ROS package test**

```bash
docker build -f fleet_bridge/vehicle/Dockerfile -t mentorpi-fleet-bridge-vehicle:test fleet_bridge
docker build -f fleet_bridge/server/Dockerfile -t mentorpi-fleet-bridge-server:test fleet_bridge
```

Expected: both native-architecture images build successfully. ARM64 native build and actual vehicle/network A/B checks remain deployment-host verification if the current host is not ARM64/Linux.

- [ ] **Step 7: 문서 및 검증 결과 커밋**

```bash
git add fleet_bridge/README.md docs/superpowers/specs/2026-08-20-foxglove-fleet-bridge-design.md docs/superpowers/plans/2026-08-20-foxglove-fleet-bridge.md
git commit -m "docs: add fleet bridge deployment guide"
```
