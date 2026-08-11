# Fleet registry와 시뮬레이션 adapter 수명주기 분리 구현 계획

> **실행 주의:** 이 계획은 `superpowers:executing-plans` 절차로, 아래 작업을 순서대로 수행한다. 구현 전 각 단계의 테스트를 먼저 추가하고, 단계별 검증을 마친 뒤 다음 단계로 진행한다.

**목표:** 중앙 Domain 215에서 실제 차량과 시뮬레이션 차량을 차량별 worker로 연결하고, `sim-up`은 공용 Gazebo/Foxglove 기반만, `sim-adapter-up`/`sim-adapter-down`은 가상 차량만 관리하도록 분리한다.

**구조:** 중앙 `fleet-manager`는 physical registry 항목의 Domain Bridge worker와 online 상태를 관리한다. 선택적으로 실행되는 `sim-adapter` 서비스는 simulation registry 항목의 Gazebo entity, 차량별 ROS adapter, Nav2, Domain Bridge worker를 관리한다. 두 서비스는 중앙 Domain 215에서 Foxglove Scene 상태를 공유하며, 차량 ROS 프로세스는 각 차량 Domain에서 실행한다.

**기술:** ROS 2 Humble, `domain_bridge`, rclpy, `ros_gz_sim`, Gazebo Harmonic, Nav2, Docker Compose, Foxglove, pytest

**근거 설계:** `docs/superpowers/specs/2026-08-11-fleet-registry-worker-design.md`

---

## 공통 제약과 메시지 계약

- 중앙 PC는 `ROS_DOMAIN_ID=215`를 사용한다. physical 차량은 `robot_1=1`, `robot_2=2`, simulation 차량은 `sim_robot_1=100`, `sim_robot_2=101`을 기본값으로 둔다.
- `fleet_registry.yaml`에는 선언 구성만 저장한다. online 상태, worker PID, 마지막 수신 시각, Gazebo entity 상태는 런타임 상태이며 파일에 쓰지 않는다.
- worker는 중앙 PC에서만 실행한다. 실제 차량에는 bridge 설치를 요구하지 않는다.
- 관제 명령은 공용 `/cmd_vel`을 만들지 않고, 차량별 이름공간으로만 왕복한다.

  ```text
  /{vehicle}/manual/cmd_vel
  /{vehicle}/move_base_simple/goal
  /{vehicle}/navigation/cancel
  /{vehicle}/safety/stop
  ```

- 차량 내부의 최종 제어 토픽은 `/{vehicle}/controller/cmd_vel` 하나이며, `safety/stop > manual > Nav2` 우선순위와 watchdog 정지를 적용한다.
- `sim-up`은 simulation 차량을 시작하지 않는다. `sim-adapter-up`은 registry의 `kind: simulation`, `enabled: true` 항목만 시작한다. `sim-adapter-down`은 그 차량만 정상 삭제한다.
- 기존의 사용자 작업 파일 `vehicle_simulator_model/ubuntu/run-command.md`는 이번 변경 범위에서 수정하거나 stage하지 않는다.

## 예상 파일 구성

```text
vehicle_simulator_model/ubuntu/
├── Dockerfile
├── compose.yaml
├── compose.foxglove.yaml
├── run.sh
├── README.md
└── ros2_ws/src/
    ├── mentorpi_fleet/
    │   ├── config/fleet_registry.yaml
    │   ├── launch/fleet_manager.launch.py
    │   ├── mentorpi_fleet/{registry,bridge_config,worker_manager,fleet_manager,simulation_manager}.py
    │   └── test/test_{registry,bridge_config,worker_manager}.py
    ├── mentorpi_gz_sim/
    │   ├── launch/vehicle_adapter.launch.py
    │   ├── launch/sim_adapter.launch.py
    │   └── config/vehicle_bridge.yaml.in
    ├── mentorpi_nav/
    │   ├── launch/vehicle_navigation.launch.py
    │   └── mentorpi_nav/{goal_bridge,cmd_vel_mux,cmd_vel_relay}.py
    └── mentorpi_foxglove_scene/
        ├── mentorpi_foxglove_scene/{dynamic_scene,sdf_scene_publisher}.py
        └── test/test_{dynamic_scene,scene_contract}.py
```

## 1. fleet registry 패키지와 구성 검증을 만든다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/package.xml`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/setup.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/setup.cfg`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/resource/mentorpi_fleet`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/__init__.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/registry.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/config/fleet_registry.yaml`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/test/test_registry.py`
- 수정: `vehicle_simulator_model/ubuntu/Dockerfile`

**1단계: 실패하는 테스트를 먼저 작성한다.**

`test_registry.py`에서 다음을 검사한다.

1. 기본 registry가 `robot_1`, `robot_2`, `sim_robot_1`, `sim_robot_2`를 올바른 kind, Domain, namespace로 읽는다.
2. 중복 `id`, namespace, Domain ID, 알 수 없는 profile, 허용하지 않은 Domain 범위, simulation 항목의 누락된 pose를 `RegistryValidationError`로 거부한다.
3. `enabled: false` 항목은 보존하되 시작 대상 목록에서 제외된다.
4. 잘못된 새 registry를 읽을 때 현재 유효 registry 객체를 변경하지 않는 호출 계약을 확인한다.

**2단계: 최소 구현을 작성한다.**

`registry.py`에 다음 공개 인터페이스를 만든다.

```python
@dataclass(frozen=True)
class VehicleSpec:
    vehicle_id: str
    kind: Literal['physical', 'simulation']
    domain_id: int
    namespace: str
    profile: str
    enabled: bool
    spawn: SpawnPose | None
    nav_enabled: bool

def load_registry(path: Path) -> FleetRegistry: ...
def enabled_vehicles(registry: FleetRegistry, kind: str | None = None) -> list[VehicleSpec]: ...
```

`fleet_registry.yaml`은 `control_domain: 215`, 허용 profile 목록, `vehicles:` 배열을 가진다. simulation 항목에만 `spawn: {x, y, z, yaw}`와 `nav_enabled`를 허용한다. 기본 항목은 위 네 차량을 사용하되 실제 차량과 simulation 차량을 동시에 등록 가능하게 만든다.

Dockerfile에 `ros-humble-domain-bridge` 및 Python YAML 의존성을 명시적으로 설치하고, 새 ament 패키지가 기존 workspace build 범위에 포함되도록 한다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q test/test_fleet_registry.py ros2_ws/src/mentorpi_fleet/test/test_registry.py
colcon test --packages-select mentorpi_fleet
colcon test-result --verbose
```

기존 테스트 파일명이 다르면 새 registry 테스트를 runtime test suite에 연결하되, 테스트가 Docker daemon 없이 실행되도록 fixture를 순수 YAML 임시 파일로 유지한다.

## 2. 차량별 domain_bridge 설정과 worker 생명주기를 구현한다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/bridge_config.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/worker_manager.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/test/test_bridge_config.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/test/test_worker_manager.py`
- 새 파일: `vehicle_simulator_model/ubuntu/test/test_fleet_bundle.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- `robot_1` worker YAML은 telemtry의 source Domain 1 → destination 215, 명령의 source 215 → destination 1만 포함하는지 확인한다.
- `robot_2` 설정에 `robot_1` 이름공간과 Domain 1이 섞이지 않는지 확인한다.
- worker manager에 세 번째 차량을 추가하거나 한 차량만 갱신할 때 기존 worker의 `Popen` 객체에는 terminate/restart가 호출되지 않는지 mock으로 검증한다.
- 삭제/비활성화 시 해당 차량만 stop하고, invalid registry reload 시 현재 worker 집합을 유지하는지 검증한다.

**2단계: 최소 구현을 작성한다.**

`bridge_config.py`가 `VehicleSpec` 하나마다 독립 YAML을 `/run/mentorpi-fleet/<vehicle>/domain_bridge.yaml`에 원자적으로 작성하게 한다. profile별 다음 토픽 타입/QoS를 명시한다.

| 방향 | 토픽 |
| --- | --- |
| 차량 → 215 | `/{vehicle}/odom`, `/{vehicle}/tf`, `/{vehicle}/tf_static`, `/{vehicle}/scan`, `/{vehicle}/imu`, camera/depth, `/{vehicle}/ground_truth/pose`, Nav2 상태 |
| 215 → 차량 | `/{vehicle}/manual/cmd_vel`, `/{vehicle}/move_base_simple/goal`, `/{vehicle}/navigation/cancel`, `/{vehicle}/safety/stop` |

`worker_manager.py`는 `subprocess.Popen`으로 한 차량당 한 `domain_bridge` process만 소유한다. registry 파일 mtime을 polling하고, 새 구성의 유효성 검증이 끝난 뒤에만 diff를 적용한다. namespace 또는 Domain 변경은 해당 차량만 replace하며, 환경에는 공통 DDS discovery 설정을 전달한다. process 실패는 차량별 `error` 기록으로 격리한다.

**3단계: 정적 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_fleet/test/test_bridge_config.py \
  ros2_ws/src/mentorpi_fleet/test/test_worker_manager.py test/test_fleet_bundle.py
```

## 3. 중앙 fleet-manager와 차량 online 상태를 구현한다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/fleet_manager.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/launch/fleet_manager.launch.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/test/test_fleet_manager.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/setup.py`
- 수정: `vehicle_simulator_model/ubuntu/compose.yaml`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- central Domain의 `/{vehicle}/odom` 입력으로 online 전환, timeout으로 offline 전환을 clock fixture로 검사한다.
- physical manager는 physical 항목만 worker로 만들고, simulation 항목을 만들지 않는지 확인한다.
- registry 변경으로 추가·삭제된 vehicle의 `/fleet/status` JSON payload가 바뀌고, 다른 vehicle의 상태와 worker는 유지되는지 확인한다.

**2단계: 최소 구현을 작성한다.**

`fleet_manager`는 `ROS_DOMAIN_ID=215`에서 실행하고 `--kind physical`로 physical registry만 관리한다. 상태는 `std_msgs/msg/String` JSON으로 `/fleet/status`에 publish한다. payload에는 `id`, `kind`, `domain_id`, `online`, `state` (`online`, `offline`, `error`, `removing`)와 마지막 수신 timestamp를 포함하며, registry 원본을 수정하지 않는다.

manager는 등록된 차량의 `/id/odom` subscriber를 동적으로 만들고, watchdog timeout으로 offline을 계산한다. `on_shutdown`에서 owned worker를 차례대로 stop하고 status를 offline으로 내보낸다. compose의 `fleet-manager` service가 이 node를 상시 실행하고 registry volume을 read-only로 mount하며, Docker healthcheck는 process 생존과 `/fleet/status` publish 가능 상태를 검사한다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_fleet/test/test_fleet_manager.py test/test_fleet_bundle.py
colcon test --packages-select mentorpi_fleet
colcon test-result --verbose
```

## 4. 고정 2대 adapter를 단일 차량 adapter로 치환한다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/vehicle_adapter.launch.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/config/vehicle_bridge.yaml.in`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/sim_adapter.launch.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/setup.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_vehicle_adapter_launch.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- launch description이 `robot_id`, pose, bridge config path를 launch argument로 노출하고, 고정 `robot_1`/`robot_2` 목록이 없는지 확인한다.
- 두 vehicle launch를 서로 다른 `ROS_DOMAIN_ID`로 생성할 때 Node namespace, controller bridge, ground-truth pose 구독이 각각의 vehicle로만 만들어지는지 검사한다.
- `/clock` bridge가 simulation vehicle별 domain에 제공되는지 확인한다.

**2단계: 최소 구현을 작성한다.**

`vehicle_adapter.launch.py`는 한 대만 담당한다. Gazebo entity 생성, model-specific `ros_gz_bridge`, image bridge, `robot_state_publisher`, `gz_pose_to_odom`을 같은 vehicle namespace로 실행한다. bridge YAML은 template에서 vehicle id를 치환하여 각 worker run directory에 생성하고, `GZ_PARTITION=mentorpi-sim` 공유 world를 사용한다.

기존 `sim_adapter.launch.py`는 더 이상 robot 2대를 직접 launch하지 않는다. 선택적 `sim-adapter` service가 실행할 `simulation_manager`의 진입점으로 바꾸거나, 호환 shim으로 단일 launch manager를 호출하게 한다. scene publisher는 vehicle adapter에서 시작하지 않아 중복 static/dynamic scene publish를 만들지 않는다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_gz_sim/test/test_vehicle_adapter_launch.py \
  test/test_sim_adapter_bundle.py
```

## 5. simulation-manager와 정상 제거 절차를 구현한다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/mentorpi_fleet/simulation_manager.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/test/test_simulation_manager.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_fleet/setup.py`
- 수정: `vehicle_simulator_model/ubuntu/compose.yaml`
- 수정: `vehicle_simulator_model/ubuntu/compose.foxglove.yaml`
- 수정: `vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py`
- 수정: `vehicle_simulator_model/ubuntu/test/test_observation_bundle.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- enabled simulation registry만 spawn request, vehicle adapter process, vehicle Nav2 process, bridge worker를 받는지 검사한다.
- 하나의 simulation vehicle 삭제 시 정확히 그 차량에 대해 `block_commands → stop_nav → stop_adapter_bridge → delete_entity → scene_delete → stop_worker` 순서가 호출되는지 검증한다.
- simulation manager 종료가 physical manager를 terminate하지 않는지, `sim-up` compose 대상에 `sim-adapter`가 없는지 확인한다.

**2단계: 최소 구현을 작성한다.**

`simulation_manager`는 별도 Docker service `sim-adapter`에서 central Domain 215로 동작한다. registry 중 simulation만 diff하고, 각각에 대해 다음을 소유한다.

1. `ros2 run ros_gz_sim create`로 shared Gazebo world에 고유 entity spawn
2. `ROS_DOMAIN_ID=<vehicle domain>` 환경의 `vehicle_adapter.launch.py`
3. `ROS_DOMAIN_ID=<vehicle domain>` 환경의 `vehicle_navigation.launch.py` (`nav_enabled: true`일 때)
4. 그 vehicle의 `domain_bridge` worker

서비스 종료 신호 또는 `simulation_manager --shutdown` 제어 요청 시, 각 simulation vehicle에서 새 중앙 명령을 먼저 차단하고 Nav2, adapter/bridge, Gazebo entity, Foxglove Scene 삭제, worker 종료 순으로 처리한다. entity 삭제는 `ros_gz_sim delete` 또는 동등한 Gazebo service 호출을 사용하고 timeout 후에도 scene delete를 보장한다.

compose에서 `sim-adapter`는 기본 `sim-up` 대상이 아닌 독립 service로 정의한다. Foxglove bridge는 Gazebo server와 fleet-manager에만 의존하며 sim-adapter healthy에 의존하지 않는다. `fleet-manager`는 physical만, `sim-adapter`는 simulation만 소유해 같은 registry 항목을 중복 처리하지 않는다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_fleet/test/test_simulation_manager.py \
  test/test_runtime_env_config.py test/test_observation_bundle.py
docker compose -f compose.yaml -f compose.foxglove.yaml config --quiet
```

## 6. 차량별 Nav2·수동 제어·안전 중재 경로를 구현한다

**파일:**

- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/launch/vehicle_navigation.launch.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/mentorpi_nav/cmd_vel_mux.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/mentorpi_nav/goal_bridge.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/mentorpi_nav/cmd_vel_relay.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/setup.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/test/test_cmd_vel_mux.py`
- 새 파일: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/test/test_vehicle_navigation_launch.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- mux가 safety stop, fresh manual, fresh Nav2, stale input 순으로 최종 `controller/cmd_vel`을 결정하는지 검사한다.
- manual command와 Nav2 output은 vehicle namespace 밖에 publish하지 않는지 확인한다.
- `goal_bridge`가 `/{vehicle}/navigation/cancel`을 받고 해당 vehicle의 NavigateToPose goal만 취소하는지 확인한다.
- 두 vehicle launch config를 비교해 서로 상대 vehicle namespace나 Domain에 의존하지 않는지 검사한다.

**2단계: 최소 구현을 작성한다.**

`vehicle_navigation.launch.py`는 한 차량의 map server와 Nav2 stack을 해당 vehicle Domain/namespace에서 launch한다. 동일 PGM/YAML을 읽되 map server와 controller는 vehicle마다 독립 node가 된다.

Nav2 cmd_vel은 `/{vehicle}/navigation/cmd_vel`로 relay하고, manual input은 `/{vehicle}/manual/cmd_vel`, emergency stop은 `/{vehicle}/safety/stop`으로 받는다. 새 mux node는 최종 `/{vehicle}/controller/cmd_vel`만 Gazebo/실차 local controller에 전달한다. 각 command source에는 짧은 timeout을 적용하고 timeout 또는 central bridge 단절 시 zero Twist를 publish한다. `goal_bridge`는 goal/cancel lifecycle과 status를 namespaced action server로 한정한다.

실제 차량 firmware/controller가 이 repository 밖에 있는 경우에도 동일 이름의 local mux 계약과 watchdog 요구사항을 README에 명시한다. 이 저장소에서는 simulation adapter에 mux를 배치해 동작을 검증한다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_nav/test/test_cmd_vel_mux.py \
  ros2_ws/src/mentorpi_nav/test/test_vehicle_navigation_launch.py test/test_navigation_bundle.py
colcon test --packages-select mentorpi_nav
colcon test-result --verbose
```

## 7. Foxglove Scene을 registry/presence 기반 다중 차량으로 일반화한다

**파일:**

- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/mentorpi_foxglove_scene/dynamic_scene.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/mentorpi_foxglove_scene/sdf_scene_publisher.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/setup.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/test/test_dynamic_scene.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/test/test_scene_contract.py`
- 수정: `vehicle_simulator_model/ubuntu/test/test_foxglove_scene_bundle.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

- `sim_robot_3`가 online status와 pose를 받으면 dynamic Scene entity로 생성되는지 검사한다.
- offline, timeout, registry removal status를 받으면 해당 id만 `SceneEntityDeletion`으로 내보내고 다른 vehicle entity는 유지되는지 검사한다.
- static warehouse scene이 vehicle registration과 관계없이 계속 유지되는지 확인한다.

**2단계: 최소 구현을 작성한다.**

scene publisher는 hard-coded `robot_1`, `robot_2` loop를 제거한다. `/fleet/status`로 online vehicle 목록을 수신해 subscriber/entity를 동적으로 만들고, simulation 차량은 bridged ground-truth pose/TF로 model entity를 갱신한다. pose freshness와 status freshness를 함께 검사해 마지막 위치가 남지 않도록 한다.

`dynamic_scene.py`는 생성 시 허용 vehicle set을 고정하지 않고 presence registry를 입력으로 받게 한다. entity id는 vehicle id를 안정적으로 사용하며, removal 시 `SceneEntityDeletion`을 한 번 발행한다. `/warehouse_scene/static`은 계속 공용 warehouse geometry만 내보내며 `/warehouse_scene/dynamic`은 online vehicle model만 내보낸다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q ros2_ws/src/mentorpi_foxglove_scene/test/test_dynamic_scene.py \
  ros2_ws/src/mentorpi_foxglove_scene/test/test_scene_contract.py \
  test/test_foxglove_scene_bundle.py
```

## 8. Compose/run 명령과 운영 문서를 수명주기에 맞춘다

**파일:**

- 수정: `vehicle_simulator_model/ubuntu/run.sh`
- 수정: `vehicle_simulator_model/ubuntu/compose.yaml`
- 수정: `vehicle_simulator_model/ubuntu/compose.foxglove.yaml`
- 수정: `vehicle_simulator_model/ubuntu/README.md`
- 수정: `vehicle_simulator_model/ubuntu/test/test_runtime_env_config.py`
- 수정: `vehicle_simulator_model/ubuntu/test/test_bundle.py`
- 수정: `vehicle_simulator_model/ubuntu/test/test_navigation_bundle.py`

**1단계: 실패하는 테스트를 먼저 작성한다.**

shell/Docker fixture로 다음 command contract를 검사한다.

| 명령 | 반드시 시작 | 시작하면 안 되는 대상 |
| --- | --- | --- |
| `sim-up` | discovery, Gazebo, fleet-manager, Foxglove | sim-adapter, simulation Nav2, sim worker |
| `sim-adapter-up` | enabled simulation adapter/worker/Nav2 | fleet-manager 재시작, physical worker 변경 |
| `sim-adapter-down` | graceful simulation cleanup | Gazebo, Foxglove, fleet-manager, physical worker 종료 |
| `nav-up` / `mapping-up` | 이미 active인 대상에만 기능 시작 | sim-adapter 자동 기동 |

또한 compose config render에서 service profile, volume, `ROS_DOMAIN_ID=215`, `GZ_PARTITION` 공유 값, Foxglove dependency 계약을 검사한다.

**2단계: 최소 구현을 작성한다.**

`run.sh`에 `sim-adapter-up` 및 `sim-adapter-down` command를 추가한다.

```bash
./run.sh sim-up
./run.sh sim-adapter-up
./run.sh sim-adapter-down
./run.sh down
```

- `sim-up`은 `dds-discovery gazebo-server fleet-manager foxglove-bridge`만 `up -d` 한다.
- `sim-adapter-up`은 fleet-manager/Gazebo의 healthy 여부를 확인한 뒤 `sim-adapter` service만 시작한다.
- `sim-adapter-down`은 먼저 adapter container의 graceful shutdown entrypoint를 호출하고 제한 시간까지 기다린 뒤 해당 service만 stop/remove 한다. cleanup 실패 메시지에는 남은 Gazebo entity id를 출력한다.
- `nav-up`, `mapping-up`, `fork-up`은 필요한 simulation adapter가 active가 아니면 `sim-adapter-up`을 안내하고 자동으로 start하지 않는다.
- `topics`, `logs`는 core service와 optional simulation service를 구분해, adapter가 꺼져 있어도 central observability 명령이 실패하지 않게 한다.

README에는 위 명령 표, registry 예시, 실제 차량 추가/삭제 절차, central vs vehicle Domain과 Foxglove 표시 규칙을 추가한다. 사용자 소유의 untracked `run-command.md`는 수정하지 않는다.

**3단계: 검증한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q test/test_runtime_env_config.py test/test_bundle.py test/test_navigation_bundle.py
docker compose -f compose.yaml -f compose.foxglove.yaml config --quiet
```

## 9. 통합 검증과 실제 런타임 검증을 수행한다

**파일:** 구현 파일 전체 및 필요 시 위 테스트 파일

**1단계: 정적·빌드 검증을 수행한다.**

```bash
cd vehicle_simulator_model/ubuntu
pytest -q
./run.sh --env dev build
docker compose -f compose.yaml -f compose.foxglove.yaml config --quiet
```

**2단계: 시뮬레이션 수명주기를 검증한다.**

Docker Desktop가 실행 중인 환경에서 다음을 순서대로 실행한다.

```bash
./run.sh sim-up
./run.sh topics
./run.sh sim-adapter-up
./run.sh nav-up
./run.sh sim-adapter-down
./run.sh down
```

확인 항목:

1. `sim-up` 직후 `/warehouse_scene/static`은 있지만 `/warehouse_scene/dynamic`에 `sim_robot_*` entity가 없다.
2. `sim-adapter-up` 후 registry의 enabled sim vehicle만 Gazebo world와 Foxglove dynamic scene에 나타난다.
3. vehicle별 command topic을 발행했을 때 반대 vehicle의 `/controller/cmd_vel`에는 메시지가 도달하지 않는다.
4. 두 sim vehicle은 하나의 Gazebo world에 있으므로 이동 중 충돌이 Gazebo 물리에서 함께 계산된다.
5. `sim-adapter-down` 후 Gazebo/Foxglove/fleet-manager는 살아 있고 `sim_robot_*` entity와 worker만 사라진다.
6. registry에 simulation 또는 physical 항목을 add/remove/disable할 때 해당 vehicle worker만 변하고 다른 worker의 PID와 telemtry가 유지된다.

**3단계: 최종 변경 검토를 수행한다.**

```bash
git diff --check
git status --short
git diff -- vehicle_simulator_model/ubuntu
```

사용자 소유의 기존 dirty 파일은 stage하지 않고, 이 기능에 해당하는 파일만 의도적으로 추가한다.
