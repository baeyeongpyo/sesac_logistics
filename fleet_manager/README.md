# Fleet Manager

Fleet Manager는 차량의 최신 상태와 **상태 변경 로그만** SQLite에 저장합니다.
물류 작업과 재고의 원장은 `inventory_db`에 있으며, 다음 작업 판단은
`logistics_orchestrator`의 책임입니다.

## 실행

```bash
cp .env.example .env
docker compose --env-file .env up --build
```

기본 API 주소는 `http://127.0.0.1:8082`입니다.

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
같은 상태의 상세 이벤트는 최신 상태만 갱신하고 로그를 추가하지 않습니다.

## 조회

```text
GET /healthz
GET /api/v1/vehicles
GET /api/v1/vehicles/{robot_id}
GET /api/v1/vehicles/{robot_id}/logs?limit=100&before_id={id}
```
