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
