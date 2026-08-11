# 중앙 PC Fleet 레지스트리와 차량별 Bridge Worker 설계

**작성일:** 2026-08-11  
**상태:** 사용자 승인 반영, 구현 계획 검토 대기  
**대체 범위:** `2026-08-10-central-domain-bridge-design.md`의 단일 Bridge 및
실행 모드 분리 부분을 이 문서의 상시 Fleet 관리 구조로 대체한다.

## 목표

중앙 PC의 Gazebo, Foxglove, Fleet manager는 계속 실행한다. 실제 차량과
시뮬레이션 차량은 레지스트리에 등록된 경우에만 중앙 Domain 215로 연결하며,
토픽을 실제로 수신하는 차량만 Foxglove에 표시한다.

차량을 추가하거나 삭제할 때 기존 차량의 관측·수동 조작·Nav2 주행은 중단되지
않아야 한다.

## Domain 및 이름공간 할당

| 차량 종류 | ID 예시 | ROS Domain | namespace |
| --- | --- | ---: | --- |
| 실제 | `robot_1` | 1 | `/robot_1` |
| 실제 | `robot_2` | 2 | `/robot_2` |
| 시뮬레이션 | `sim_robot_1` | 100 | `/sim_robot_1` |
| 시뮬레이션 | `sim_robot_2` | 101 | `/sim_robot_2` |
| 중앙 PC | 해당 없음 | 215 | 공용 관제 namespace |

추가 차량에는 사용하지 않은 안전 Domain ID와 고유한 ID·namespace를 할당한다.
`robot_*`와 `sim_robot_*`는 같은 중앙 PC에 동시에 존재할 수 있으며 서로 토픽을
공유하지 않는다.

## 구성과 런타임 상태의 분리

`fleet_registry.yaml`은 사람이 관리하는 선언형 구성이다. 다음만 기록한다.

- `id`, `kind` (`physical` 또는 `simulation`), `domain_id`, `namespace`
- 공통 메시지 계약을 고르는 `profile`
- 관제 참여를 결정하는 `enabled`
- 시뮬레이션 차량만의 Gazebo 초기 위치와 Nav2 활성화 값

`online`, 마지막 수신 시각, worker PID, Gazebo entity 상태는 구성 파일에 쓰지
않는다. Fleet manager가 `/robot_id/odom` 수신을 기준으로 계산해 `/fleet/status`에
발행한다. 일정 시간 동안 odom을 받지 못하면 `offline`으로 전환한다.

## 상시 Fleet manager

Fleet manager는 중앙 PC Domain 215에서 실행하며 registry 파일 변경을 감시한다.
변경을 원자적으로 읽고 검증한 뒤, 바뀐 차량에 해당하는 worker만 추가·교체·삭제한다.

```text
fleet_registry.yaml 변경
        │
        ▼
fleet manager ── worker 생성/종료 ── domain_bridge worker (차량 Domain ↔ 215)
        │
        ├─ simulation: Gazebo entity + vehicle adapter + 로컬 Nav2 관리
        └─ presence: odom/TF → /fleet/status + Foxglove Scene 표시·삭제
```

worker는 모두 중앙 PC에서 실행한다. 실제 차량에는 domain_bridge 설치가 필요 없고,
고유 Domain, namespace, DDS discovery 연결과 로컬 안전 제어기만 필요하다.

각 worker는 한 차량의 양방향 토픽만 브리지한다. 관측 토픽은 차량 Domain에서
215로, 명령 토픽은 215에서 그 차량 Domain으로 단방향 전송한다. 공용 `/cmd_vel`은
만들지 않으며 차량별 명령은 다음 이름으로 제한한다.

```text
/{robot}/manual/cmd_vel
/{robot}/move_base_simple/goal
/{robot}/navigation/cancel
/{robot}/safety/stop
```

## 차량 추가·삭제 절차

### 실제 차량

등록 추가 시 manager는 해당 차량의 domain_bridge worker를 시작한다. 실제 차량이
이미 실행 중이면 odom이 발견되어 `online`과 Foxglove 표시가 자동으로 시작된다.
꺼져 있으면 worker는 유지하되 `offline`으로 남는다. 기존 worker는 재시작하지 않는다.

등록 삭제 또는 `enabled: false` 시 manager는 해당 worker만 종료하고 Foxglove Scene에
삭제를 발행한다. 차량의 로컬 watchdog은 중앙 명령이 사라졌을 때 정지해야 한다.

### 시뮬레이션 차량

등록 추가 시 manager는 공용 `GZ_PARTITION`의 Gazebo server에 고유 이름의 SDF 모델을
생성한다. 이어서 그 차량 Domain으로 실행되는 adapter와 Nav2를 시작하고 worker를
연결한다. 모든 시뮬레이션 모델은 하나의 Gazebo world에 있으므로 물리·충돌은
공통으로 계산된다.

삭제 시 새 명령을 막고, 해당 Nav2와 adapter를 종료한 뒤 Gazebo entity를 삭제하고,
마지막으로 worker와 Foxglove Scene entity를 제거한다. 다른 시뮬레이션 및 실제 차량은
계속 실행한다.

## Foxglove 표시 규칙

Foxglove Scene publisher는 registry의 `enabled` 차량 중 온라인 차량만 entity로
발행한다. 상태가 offline이 되거나 등록이 삭제되면 `SceneEntityDeletion`으로 차량
모델을 제거한다. 따라서 비실행 차량은 목록이나 3D 장면에 남지 않는다.

시뮬레이션 차량의 자세는 Gazebo에서 브리지한 pose/TF로 표시한다. 실제 차량은
브리지된 TF와 odom으로 표시하며, 실제 frame 계약이 확인되기 전에는 LiDAR·TF 등
수신 가능한 데이터만 표시한다.

## 안전과 오류 처리

- registry 검증은 중복 ID, namespace, Domain ID, 예약 Domain 사용, 알 수 없는 profile,
  잘못된 simulation pose를 거부한다. 이전에 동작하던 registry와 worker는 유지한다.
- `domain_id`나 `namespace`가 변경되면 해당 차량 worker만 교체한다.
- worker 시작 실패는 그 차량을 `error` 상태로 기록하며 다른 worker에 영향을 주지 않는다.
- 모든 실제 차량은 중앙 연결 단절에 대비한 로컬 속도 watchdog을 가진다.
- 수동·Nav2·정지 명령의 최종 속도 중재는 각 차량 로컬에 두며, 우선순위는
  `safety/stop` > 수동 조작 > Nav2다.

## 검증 기준

1. registry schema 단위 테스트가 유효한 실제·시뮬레이션 차량을 수용하고 중복 또는
   안전하지 않은 Domain ID를 거부한다.
2. registry에 세 번째 차량을 추가할 때 기존 두 worker의 PID·연결 상태가 유지되는지
   확인한다.
3. `robot_1`의 수동·Nav2·정지 명령은 Domain 1에만, `robot_2` 명령은 Domain 2에만
   전달되는지 검증한다.
4. 시뮬레이션 차량을 추가·삭제할 때 공용 Gazebo world의 entity와 중앙 Domain 215의
   Foxglove Scene entity가 생성·삭제되는지 검증한다.
5. odom timeout 후 해당 차량만 offline 및 Scene 삭제가 되고 다른 차량의 명령 경로가
   유지되는지 검증한다.

## 근거와 범위

ROS 2 Domain Bridge는 Domain별 ROS context를 만들고 YAML에서 토픽별 출발·목적
Domain, 타입, QoS를 명시한다. Fleet manager는 이 정적 Bridge 구성을 차량별 worker로
생성·관리하는 프로젝트 구현이다.

`ros_gz_sim create`는 ROS에서 Gazebo entity를 생성할 수 있어 registry의 simulation
항목을 공용 world에 반영하는 데 사용한다. Open-RMF의 fleet adapter 설정 파일은
fleet 구성과 연결 설정을 분리하는 참고 사례일 뿐, Open-RMF 패키지는 이 구현 범위에
포함하지 않는다.

포함하지 않는 범위는 다중 차량 충돌 회피, 교통 우선순위, 구역 예약과 원격 LAN의
방화벽·Discovery Server 배포 자동화다.
