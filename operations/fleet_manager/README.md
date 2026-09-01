# Fleet Manager

Fleet Manager는 차량의 최신 상태와 **상태 변경 로그**를 SQLite에 저장합니다.
물류 작업과 재고의 원장은 `inventory_db`에 있으며, 다음 작업 판단은
`logistics_orchestrator`의 책임입니다.

## 실행

```bash
cd operations/fleet_manager
cp .env.example .env
docker compose --env-file .env up --build
```

기본 API 주소는 `.env`의 `FLEET_MANAGER_API_PORT`를 사용합니다.

## 차량 상태 보고

Fleet Bridge가 아래 API를 호출합니다.

```text
POST /api/v1/vehicles/{robot_id}/state
```

```json
{
  "state": "DRIVE",
  "previous_state": "WAIT",
  "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
  "attempt_id": "2d6d014b-4041-4a22-ac02-917a77e6c709",
  "source": "NAV2",
  "detail": "NAVIGATION_STARTED",
  "observed_at": "2026-08-31T12:00:00Z"
}
```

`state`는 `INIT`, `WAIT`, `DRIVE`, `PICK`, `PLACE`, `FAIL` 중 하나입니다.
같은 상태의 상세 이벤트는 최신 상태만 갱신하고 로그를 추가하지 않습니다. 다만
`NAVIGATION_SUCCEEDED`, `AUTO_DOCK_PICK_COMPLETED`, `AUTO_DOCK_PLACE_COMPLETED`처럼
같은 `WAIT` 상태라도 Orchestrator가 구분해야 하는 실제 차량 보고는 모두 event outbox에
보존됩니다.

## 조회

```text
GET /healthz
GET /api/v1/vehicles
GET /api/v1/vehicles/{robot_id}
GET /api/v1/vehicles/{robot_id}/logs?limit=100&before_id={id}
```

## Orchestrator 이벤트 전달

`ORCHESTRATOR_EVENT_URL`이 설정되면 Fleet Manager는 각 상태 보고를 상태 저장과 같은
SQLite transaction에 outbox로 기록하고, 같은 `event_id`로
`POST /api/v1/events/fleet`를 성공 2xx까지 지수 backoff 재전송합니다. 상태 변경 로그의
기록 여부와 무관하게 모든 report가 전달 대상입니다.

```dotenv
ORCHESTRATOR_EVENT_URL=http://host.docker.internal:8083/api/v1/events/fleet
EVENT_RETRY_INTERVAL_SEC=2
```
