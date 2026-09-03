# Logistics Orchestrator

`logistics_orchestrator`는 Fleet Manager의 실제 차량 상태와 Inventory의 최신 재고를
다시 조회해 다음 작업을 결정하는 중앙 제어 서비스입니다. 차량 명령은 직접 차량에
보내지 않고 반드시 Fleet Manager를 통해 전달합니다.

## 소유 경계

- `inventory_db`: zone, 재고, 예약, `operation_id`, PICK/PLACE 완료의 원장
- `fleet_manager`: 차량 최신 상태와 상태 변경 로그, 모든 차량 보고의 event outbox,
  차량 모델/기능 확인과 Fleet Bridge 명령 중계
- `fleet_bridge`: 모델별 차량 API·ROS·Foxglove 명령 변환만 담당
- `logistics_orchestrator`: 우선순위, WAIT 차량 배차, 작업 단계, recovery, 명령 outbox

Inventory 환경은 별도로 먼저 구성해야 합니다. Orchestrator는 zone이나 초기 재고를
생성·수정하지 않습니다. `docker`(24), `p1`~`p3`(각 1), `f1`~`f9`(각 1),
`n1`~`n9`(각 1)가 모두 enabled여야 자동 명령을 보냅니다.

`docker`는 Inventory의 출발 zone ID입니다. PICK Auto Dock 명령에서는 `docker`를
물리 도크 식별자 `DOCK_1`로 명시적으로 매핑합니다.

## 실행

```bash
cd operations/logistics_orchestrator
cp .env.example .env
docker compose --env-file .env up -d --build
curl --fail http://127.0.0.1:8083/healthz
```

Fleet Manager와 Inventory에도 아래 환경 변수를 설정한 뒤 재시작합니다.

```dotenv
# operations/fleet_manager/.env
ORCHESTRATOR_EVENT_URL=http://host.docker.internal:8083/api/v1/events/fleet
EVENT_RETRY_INTERVAL_SEC=2

# operations/inventory_db/.env
ORCHESTRATOR_EVENT_URL=http://host.docker.internal:8083/api/v1/events/inventory
EVENT_RETRY_INTERVAL_SEC=2
```

각 원장은 상태 변경 transaction 안에서 outbox event를 기록합니다. HTTP 전송이
실패해도 원장은 되돌리지 않으며, 성공할 때까지 같은 `event_id`로 지수 backoff
재전송합니다.

## API

```text
GET  /healthz
GET  /api/v1/status
POST /api/v1/reconcile
POST /api/v1/events/fleet
POST /api/v1/events/inventory
POST /api/v1/operations/{operation_id}/pallet-3/bypass-pick
POST /api/v1/poc/pallet-3-missions
GET  /api/v1/poc/pallet-3-missions/{mission_id}
POST /api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation
```

`/healthz`는 Docker readiness용입니다. `/api/v1/status`는 inbox, command outbox,
환경 오류를 확인하는 운영용 API입니다. `/api/v1/reconcile`은 Orchestrator 재시작이나
event 누락 점검 시 Fleet Manager와 Inventory snapshot을 안전하게 다시 읽습니다.
`/bootstrap`은 제공하지 않습니다. Inventory 환경 구성은 Inventory의 책임입니다.

## 작업 규칙과 상태 흐름

1. Docker에 FRESH가 있으면 가장 낮은 빈 `p*`, P가 모두 차면 가장 낮은 빈 `f*`로
   먼저 이송합니다. FRESH가 물리적으로 남아 있으면 NORMAL은 선택하지 않습니다.
2. Docker FRESH가 0일 때만 Docker NORMAL을 빈 `p*`, 그 다음 빈 `n*`으로 이송합니다.
3. Docker가 완전히 비었을 때만 `f1`~`f9`에서 `p1`~`p3`를 보충하고, FRESH source가
   전부 비었을 때 `n1`~`n9`를 사용합니다.

차량은 `WAIT` 상태일 때만 새 명령을 받습니다. 일반 흐름은 다음과 같습니다.

```text
WAIT -> DRIVE -> WAIT(NAVIGATION_SUCCEEDED) -> PICK -> WAIT(AUTO_DOCK_PICK_COMPLETED)
     -> DRIVE -> WAIT(NAVIGATION_SUCCEEDED) -> PLACE -> WAIT(AUTO_DOCK_PLACE_COMPLETED)
```

PICK 완료는 Inventory `pick-completions`를 `{operation_id}:pick` 키로 한 번만
반영한 뒤 PLACE 목적지 Nav2 goal을 보냅니다. PLACE 완료도 `{operation_id}:place`로
한 번만 반영합니다.

`INIT`, `FAIL`, `DRIVE`, `PICK`, `PLACE` 상태에는 신규 명령을 보내지 않습니다.
사고·경로 실패·재부팅 뒤에는 운영자가 차량을 수동 대응한 뒤 해당 Fleet Bridge의
`POST /api/v1/vehicle-command/{robot_id}/operation/idle`을 호출해 차량이 `WAIT`
(`OPERATOR_READY`)을 보고해야 합니다. 이 승인은 작업마다가 아니라 INIT/FAIL에서
정상 운행 상태로 되돌릴 때만 필요합니다. 이후 활성 작업은 남아 있는 PICK 또는 PLACE
단계부터 이어집니다.

## 영속 데이터

`../data/orchestrator.db`(`operations/data/orchestrator.db`)에는 다음 데이터가 보존됩니다.

- `orchestrator_inbox`: `(source, event_id)` 중복 제거
- `orchestrator_steps`: `RESERVED`부터 `PLACE_COMMITTED`까지의 작업 단계
- `command_outbox`: Fleet Manager 명령과 `PENDING`, `SENT`, `DELIVERY_UNKNOWN`, `FAILED` 결과
- `operation_recoveries`: `FAIL` 후 운영자 `OPERATOR_READY`를 기다리는 작업
- `pallet3_operation_workflows`: `docker -> p3` 작업의 waypoint·fork·후진·복귀 단계

차량 명령 HTTP timeout은 수락 여부를 알 수 없으므로 `DELIVERY_UNKNOWN`으로 남기고
즉시 재전송하지 않습니다. 다음 실제 차량 보고나 수동 recovery가 있어야만 진행합니다.

## Pallet 3 일반 작업

Pallet 3은 별도 POC 미션이 아니라 Inventory의 일반 `docker -> p3` 작업으로 실행합니다.
Inventory가 발급한 `operation_id` 하나가 재고 예약, 차량 적재 상태, Fleet 명령 outbox와
Pallet 3 workflow를 연결합니다. 따라서 Control Center에서도 다른 운송 작업과 동일하게
표시됩니다.

### 자동 적재

차량이 비적재·활성 작업 없음 상태일 때 Inventory 작업을 생성하고 차량을 `idle`로 전환합니다.

```bash
curl --fail-with-body -X POST http://127.0.0.1:8081/api/v1/operations \
  -H 'Content-Type: application/json' \
  -d '{
    "robot_id":"robot_1",
    "payload_type":"FRESH",
    "source_zone_id":"docker",
    "destination_zone_id":"p3",
    "priority":0
  }'

curl --fail-with-body -X POST \
  http://127.0.0.1:8080/api/v1/vehicle-command/robot_1/operation/idle \
  -H 'Content-Type: application/json' \
  -d '{"reason":"PALLET_3_OPERATION"}'
```

차량이 `WAIT` / `OPERATOR_READY`를 보고하면 아래 순서로 자동 진행합니다.

```text
docker 접근 Nav2
  -> Auto Dock PICK(location=DOCK_1)
  -> Inventory PICK 완료 (docker -1, 차량 적재, TO_PLACE)
  -> Pallet 3 outbound waypoint
  -> Fork DOWN
  -> FORK_DOWN_COMPLETE
  -> Inventory PLACE 완료 (p3 +1, 작업 COMPLETED, 차량 비적재)
  -> cmd_vel(-0.18 m/s, 1000 ms)
  -> dock_1 복귀 waypoint
```

### 수동 적재

Auto Dock PICK를 쓰지 않을 때는 먼저 같은 Inventory 작업을 생성하고, 작업자가 실제로
물류를 포크에 적재한 뒤 **차량을 `idle`로 전환하기 전에** 아래 API를 호출합니다.

```bash
curl --fail-with-body -X POST \
  http://127.0.0.1:8083/api/v1/operations/{operation_id}/pallet-3/bypass-pick \
  -H 'Content-Type: application/json' \
  -d '{"operator_confirmed":true}'
```

이 요청은 차량이 비적재이고 작업이 `TO_PICK`이며 Docker 접근 Nav2/Auto Dock PICK 명령이
아직 없을 때만 허용됩니다. 성공하면 `operation_id:manual-pick` 멱등 키로 Inventory PICK을
기록하므로 docker 재고가 정확히 한 번 감소하고 작업은 `TO_PLACE`가 됩니다. 이어서 위의
`operation/idle`을 호출하면 Docker 주행과 Auto Dock을 건너뛰고 Pallet 3 outbound waypoint부터
시작합니다.

Pallet 3 도착 뒤에는 별도 하역 확인 API가 없습니다. 일치하는 `operation_id`의
`FORK_DOWN_COMPLETE`가 들어온 경우에만 p3 재고 반영과 후진·복귀가 진행됩니다. outbound
waypoint는 `(-0.440,-0.900) -> (-0.420,-2.000) -> (-0.420,-2.400)`이고, 복귀 waypoint는
`(-0.420,-2.000) -> (-0.440,-0.900) -> (0.085,-0.905)`입니다.

### 복구

Fleet가 `FAIL`이면 새 명령을 보내지 않습니다. 운영자가 `operation/idle`을 호출해 다시
`WAIT` / `OPERATOR_READY`가 된 뒤에만 안전한 단계부터 재개합니다. 아직 PICK 전이면 Docker
접근부터, PICK 완료 뒤 outbound 중이면 Pallet 3 waypoint부터 재시도합니다. Fork DOWN 명령,
후진 또는 복귀 중 실패하면 실제 물리 수행 여부가 불명확하므로 자동 재발행하지 않고 stop과
`FAILED` workflow를 남깁니다. 물류 상태를 현장에서 해소한 뒤에는 기존 작업을 초기화하지
말고 새 Inventory `operation_id`로 다음 반복 작업을 생성합니다.

## Legacy Pallet 3 POC

아래 API는 이미 실행 중인 레거시 시연 미션을 마무리하기 위한 호환 경로입니다. 새 운영
작업에는 사용하지 마십시오. 레거시 POC의 `mission_id`와 하역 확인은 위 일반 작업 흐름과
별개입니다.

이 POC는 `dock_1`의 Auto Dock PICK가 `AUTO_DOCK_PICK_COMPLETED`를 보고한 뒤에만
시작합니다. POC는 그 PICK의 `operation_id`를 그대로 사용하므로 차량의 적재 상태와
Nav2·fork 상태를 같은 작업으로 연결합니다. Pick/Fork up과 Inventory 완료 처리는 자동으로
수행하지 않습니다. 활성 POC 차량은 일반 재고 자동 배차에서 제외됩니다.

하드웨어 없이 PICK 완료를 검증할 때에는, 먼저 실제 Auto Dock PICK 명령이 `PICKING` 상태로
수락된 것을 확인한 뒤 차량 ROS 환경에서 다음 `std_msgs/msg/Empty` 한 건을 발행할 수 있습니다.

```bash
ros2 topic pub --once /robot_1/auto_dock/drive_ready std_msgs/msg/Empty "{}"
```

기본 topic 이름은 `/{robot_id}/auto_dock/drive_ready`입니다. Auto Dock PICK가 활성화되지 않은
상태의 publish는 차량 API가 무시합니다.

Auto Dock을 사용할 수 없는 **주행 POC 전용**으로는 요청 본문에 `bypass_pick: true`를
명시할 수 있습니다. 이 경우에만 `AUTO_DOCK_PICK_COMPLETED`와 PICK `operation_id` 검증을
건너뛰고 `WAIT` 상태의 차량으로 pallet_3 waypoint 주행을 즉시 시작합니다. 실제 적재,
fork up, Inventory PICK 완료를 처리하거나 완료로 기록하지 않습니다. 차량 Command API에
진행 중인 작업이 없어야 하며, 적재물·fork·주행 경로가 안전한 상태에서만 사용해야 합니다.
하역 확인 이후의 fork down, 후진, dock_1 복귀는 기본 POC와 동일하게 계속 실행됩니다.

```bash
curl --fail-with-body -X POST http://127.0.0.1:8083/api/v1/poc/pallet-3-missions \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"robot_1"}'
```

```bash
curl --fail-with-body -X POST http://127.0.0.1:8083/api/v1/poc/pallet-3-missions \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"robot_1","bypass_pick":true}'
```

응답의 `mission_id`로 상태를 조회합니다. 동일 차량에 활성 POC가 있으면 같은 미션을
반환하므로 시작 요청을 재전송해도 Nav2 명령을 중복 발행하지 않습니다.

반복 주행 POC에서 이전 활성 미션을 명시적으로 끝내고 새 `mission_id`로 다시 출발하려면
`bypass_pick: true`와 함께 `new_mission: true`를 사용합니다. Fleet 상태가 `WAIT`일 때만
허용되며, 기존 활성 POC는 `FAILED` / `SUPERSEDED_BY_NEW_POC_REQUEST`로 기록한 뒤 새 outbound
waypoint 명령을 발행합니다. 이동·fork 동작 중에는 사용하지 마십시오. Auto Dock PICK에 연결된
기본 POC에는 새 작업 ID를 안전하게 만들 수 없으므로 `new_mission`을 사용할 수 없습니다.

```bash
curl --fail-with-body -X POST http://127.0.0.1:8083/api/v1/poc/pallet-3-missions \
  -H 'Content-Type: application/json' \
  -d '{"robot_id":"robot_1","bypass_pick":true,"new_mission":true}'
```

```bash
curl --fail-with-body http://127.0.0.1:8083/api/v1/poc/pallet-3-missions/{mission_id}
curl --fail-with-body -X POST \
  http://127.0.0.1:8083/api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation
```

미션은 아래 순서를 강제합니다.

```text
FOLLOW_WAYPOINTS(dock_1 → pallet_3)
  → AWAIT_UNLOAD_CONFIRMATION
  → fork DOWN
  → /fork/state DOWN_COMPLETE
  → cmd_vel(-0.18 m/s, 1000 ms) 및 정지 확인
  → FOLLOW_WAYPOINTS(pallet_3 → dock_1)
```

출발 waypoint는 `(-0.440,-0.900) → (-0.420,-2.000) → (-0.420,-2.400)`이고,
복귀 waypoint는 `(-0.420,-2.000) → (-0.440,-0.900) → (0.085,-0.905)`입니다.
`DOWN_COMPLETE`와 후진 정지 보고는 모두 같은 `mission_id`여야 합니다. Fork 오류·타임아,
Nav2 실패 또는 명령 전달 실패는 미션을 `FAILED`로 기록하고 Fleet Manager 경유 즉시 정지를
요청합니다.
