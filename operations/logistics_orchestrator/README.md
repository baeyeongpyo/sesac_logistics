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

차량 명령 HTTP timeout은 수락 여부를 알 수 없으므로 `DELIVERY_UNKNOWN`으로 남기고
즉시 재전송하지 않습니다. 다음 실제 차량 보고나 수동 recovery가 있어야만 진행합니다.

## Pallet 3 POC

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
