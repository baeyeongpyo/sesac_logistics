# 파렛트 재고 서비스

`inventory_db/`는 차량 제어 API와 분리된 중앙 파렛트 재고 서비스입니다. 개별 pallet ID는 저장하지 않고 `(zone_id, payload_type)` 단위의 수량을 관리합니다.

## 실행

```bash
cd inventory_db
cp .env.example .env
docker compose up --build -d
curl http://127.0.0.1:8081/healthz
```

SQLite 파일은 호스트의 `inventory_db/data/inventory.db`에 생성됩니다. `data/`와 `.env`는 Git에서 제외됩니다. 중지는 `docker compose down`으로 수행하며 bind mount의 DB 파일은 유지됩니다.

## 상태 모델

- `payload_type`: `EMPTY`, `FRESH`, `NORMAL`. `EMPTY`는 공파렛트가 포크에 적재되었거나 zone에 존재함을 뜻합니다.
- `robot_pallet_states.has_pallet`: 차량 포크에 파렛트가 있으면 `true`이고, 이때만 `payload_type`이 있습니다. 포크의 높이 상태는 저장하지 않습니다.
- `pallet_stocks`: zone별 수량과 이미 운송 작업에 배정된 `reserved_quantity`를 저장합니다. `available_quantity = quantity - reserved_quantity`입니다.
- `transport_operations`: 작업 생성 요청의 `robot_id`에 즉시 배정되며, 생성 직후 비적재 차량은 `TO_PICK` 상태로 source zone을 다음 주행 목적지로 받습니다. PICK 뒤 적재 차량은 `TO_PLACE` 상태로 destination zone을 받습니다. 완료·취소되지 않은 작업은 destination의 적치 슬롯 1개도 예약합니다.

작업 생성은 비적재·비작업 차량, source 가용 재고 1개와 destination 가용 슬롯 1개를 단일 SQLite transaction으로 함께 확인·예약합니다. destination의 가용 슬롯은 `capacity - 실제 적치 수량 - 도착 예정 작업 수`입니다. PICK 완료는 source 재고를 1 감소시키고 차량 상태를 적재로 바꿉니다. PLACE 완료는 destination 재고를 1 증가시키고 차량 상태를 비적재로 바꿉니다. 각 전이는 단일 SQLite transaction으로 처리하며 `idempotency_key`가 같은 완료 보고는 같은 이벤트를 반환합니다.

이전 버전의 차량 미배정 `QUEUED` 작업은 서비스 시작 시 삭제되고, 해당 source 재고 예약도 함께 해제됩니다. 새 스키마에는 `QUEUED` 상태가 없습니다.

## HTTP API

- `GET /healthz`
- `PUT`, `GET /api/v1/zones`, `DELETE /api/v1/zones/{zone_id}` — 재고·예약·운송 작업·재고 이벤트가 없는 빈 zone만 삭제
- `PUT /api/v1/stocks/{zone_id}/{payload_type}`, `GET /api/v1/stocks`
- `POST /api/v1/operations`
- `GET /api/v1/operations/active` — 재고 예약 또는 PICK/PLACE가 진행 중인 작업 목록
- `GET /api/v1/robots/{robot_id}/pallet-state`
- `GET /api/v1/robots/{robot_id}/next-instruction`
- `POST /api/v1/operations/{operation_id}/pick-completions`
- `POST /api/v1/operations/{operation_id}/place-completions`

`POST /api/v1/operations` 요청에는 `operation_id`를 넣지 않습니다. inventory가 예약 성공 후 UUID를 생성해 응답의 `operation_id`로 반환합니다.

```json
{
  "robot_id": "mentorpi_m1_01",
  "payload_type": "FRESH",
  "source_zone_id": "docker",
  "destination_zone_id": "fresh_storage",
  "priority": 10
}
```

작업 생성은 `robot_id`가 비어 있거나 다른 활성 작업·적재 상태를 가지면 409으로 거부됩니다. 예를 들어 차량은 `next-instruction`의 `zone_id`까지 주행한 뒤, zone 내부의 비전·피킹 노드가 완료를 확인하면 `pick-completions` 또는 `place-completions`를 호출합니다. 이 서비스는 Nav2, 비전, 포크 제어를 직접 수행하지 않습니다.
