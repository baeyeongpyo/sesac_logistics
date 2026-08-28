# 파렛트 재고 서비스 설계

## 목표

파렛트 ID 없이 zone별 집계 재고, 차량의 현재 파렛트 적재 여부, 운송 작업 상태를
중앙 서버에서 영속적으로 관리한다. 서비스는 별도 `inventory_db/` 디렉터리와
Docker Compose로 배포하며, 기존 `fleet_bridge` 차량 명령 API와 독립적으로
운영한다.

## 범위

포함:

- SQLite WAL 데이터베이스와 재고 HTTP API
- zone, 적재물 종류, zone별 집계 재고, 차량 적재 상태, 운송 작업, 입출고 이력
- source 재고 예약, PICK/PLACE 원자적 갱신, 중복 완료 통지 멱등 처리
- 다음 목적지 결정을 위한 `PICK` 또는 `PLACE` 지시 조회
- `inventory_db/docker-compose.yaml` 단독 실행과 로컬 bind volume 영속화

제외:

- 차량의 Nav2 주행 명령, vision pick/place 실행, Foxglove 통신
- 포크 높이 또는 TF 저장
- 개별 물리 파렛트 ID 추적
- 교통 충돌 회피와 목적지 적치 용량 선예약

## 배포 경계

`inventory_db/`는 다음만 소유한다.

```text
inventory_db/
  app/                 HTTP API와 SQLite 저장소
  tests/               독립 서비스 테스트
  Dockerfile
  docker-compose.yaml
  data/                런타임 SQLite 파일과 WAL/SHM 파일, Git 미추적
```

Compose는 `./data:/data`를 bind mount하고 DB 파일은
`INVENTORY_DB_PATH`(기본 `/data/inventory.db`)에 둔다. SQLite WAL은 동일 호스트의
로컬 파일 시스템에서만 사용한다. 이 서비스는 포트 `8081`을 기본으로
`127.0.0.1`에만 바인딩한다.

FastAPI 애플리케이션 수명 동안 `InventoryStore` 인스턴스 하나를 유지한다.
`InventoryStore`는 세션이나 `sqlite3.Connection`을 공유하지 않는다. 각 public
메서드는 새 connection을 열고, 쓰기에서 `BEGIN IMMEDIATE`로 짧은 트랜잭션을
만든 뒤 commit 또는 rollback 후 close한다. 따라서 FastAPI dependency injection은
저장소 객체만 전달하고 DB connection을 요청 간 재사용하지 않는다.

## 데이터 모델

### `zones`

| 컬럼 | 규칙 |
|---|---|
| `zone_id` | TEXT PK |
| `name` | TEXT NOT NULL |
| `map_name` | TEXT NOT NULL |
| `nav_x`, `nav_y`, `nav_yaw` | REAL NOT NULL |
| `capacity` | NULL 또는 0 이상의 INTEGER |
| `enabled` | 0 또는 1 |
| `created_at`, `updated_at` | UTC ISO-8601 TEXT |

### `payload_types`

초기 행은 `EMPTY`, `FRESH`, `NORMAL`이다. `EMPTY`는 공파렛트가 존재한다는
뜻이며 파렛트 부재를 뜻하지 않는다.

| 컬럼 | 규칙 |
|---|---|
| `code` | TEXT PK |
| `display_name` | TEXT NOT NULL |
| `enabled` | 0 또는 1 |

### `pallet_stocks`

| 컬럼 | 규칙 |
|---|---|
| `zone_id`, `payload_type` | 복합 PK 및 FK |
| `quantity` | 0 이상의 실제 zone 재고 |
| `reserved_quantity` | 0 이상, `quantity` 이하 |
| `version` | 1부터 증가 |
| `updated_at` | UTC ISO-8601 TEXT |

`available_quantity`는 저장하지 않고 `quantity - reserved_quantity`로 계산한다.

### `robot_pallet_states`

| 컬럼 | 규칙 |
|---|---|
| `robot_id` | TEXT PK |
| `has_pallet` | 0 또는 1 |
| `payload_type` | `has_pallet=0`이면 NULL, 1이면 `payload_types` FK |
| `version` | 1부터 증가 |
| `reported_at`, `updated_at` | UTC ISO-8601 TEXT |

포크 위치는 이 테이블에 저장하지 않는다. 차량 로컬 config의 command/operation
복구 정보도 이 서비스가 소유하지 않는다.

### `transport_operations`

| 컬럼 | 규칙 |
|---|---|
| `operation_id` | TEXT PK |
| `robot_id` | 배정 전 NULL |
| `payload_type` | NOT NULL FK |
| `source_zone_id`, `destination_zone_id` | 서로 다른 zone FK |
| `status` | `QUEUED`, `TO_PICK`, `PICKING`, `TO_PLACE`, `PLACING`, `COMPLETED`, `FAILED`, `RECOVERY_REQUIRED`, `CANCELLED` |
| `priority` | INTEGER, 큰 값이 높은 우선순위 |
| `failure_code` | NULL 또는 오류 코드 |
| `version` | 1부터 증가 |
| `created_at`, `updated_at`, `completed_at` | UTC ISO-8601 TEXT |

### `inventory_events`

| 컬럼 | 규칙 |
|---|---|
| `event_id` | UUID TEXT PK |
| `idempotency_key` | TEXT UNIQUE |
| `operation_id` | FK |
| `robot_id` | TEXT NOT NULL |
| `event_type` | `PICK_COMPLETED`, `PLACE_COMPLETED`, `STOCK_ADJUSTED` |
| `zone_id`, `payload_type` | FK |
| `quantity_delta` | PICK는 -1, PLACE는 +1, 수동 조정은 0이 아닌 정수 |
| `occurred_at`, `created_at` | UTC ISO-8601 TEXT |

## 작업 전이

1. 작업 생성은 source 재고의 `reserved_quantity`를 1 증가시키고 작업을
   `QUEUED`로 저장한다. 가용 재고가 없으면 아무 행도 변경하지 않는다.
2. 차량 배정은 `QUEUED` 작업에 `robot_id`를 저장하고 `TO_PICK`으로 변경한다.
3. PICK 완료는 source의 `quantity`와 `reserved_quantity`를 각각 1 줄이고,
   차량 상태를 `has_pallet=1`과 작업의 `payload_type`으로 변경하며, 작업을
   `TO_PLACE`로 변경하고 `PICK_COMPLETED` 이력을 추가한다.
4. PLACE 완료는 destination의 `quantity`를 1 늘리고, 차량 상태를
   `has_pallet=0`, `payload_type=NULL`로 변경하며, 작업을 `COMPLETED`로
   변경하고 `PLACE_COMPLETED` 이력을 추가한다.
5. 3과 4는 하나의 SQLite transaction이다. 같은 `idempotency_key`로 다시
   호출하면 기존 이벤트를 반환하고 재고나 차량 상태를 다시 변경하지 않는다.

`next_instruction(robot_id)`는 저장하지 않는 파생값이다. 차량이 파렛트를
가지지 않으면 배정 작업의 source zone으로 `PICK`, 가지고 있으면 destination
zone으로 `PLACE` 지시를 반환한다.

## HTTP 계약

모든 endpoint는 `/api/v1` 아래에 둔다.

| 메서드와 경로 | 역할 |
|---|---|
| `GET /healthz` | DB 초기화 여부 확인 |
| `PUT /zones/{zone_id}` | zone 생성 또는 갱신 |
| `GET /zones` | 사용 가능한 zone 조회 |
| `PUT /stocks/{zone_id}/{payload_type}` | 초기 재고 또는 수동 재고 수량 설정 |
| `GET /stocks` | zone별 현재·예약·가용 재고 조회 |
| `POST /operations` | source 재고를 예약한 운송 작업 생성 |
| `POST /operations/{operation_id}/assignments` | 차량 배정 |
| `GET /robots/{robot_id}/pallet-state` | 현재 차량 파렛트 상태 조회 |
| `GET /robots/{robot_id}/next-instruction` | 다음 PICK/PLACE 목적지 조회 |
| `POST /operations/{operation_id}/pick-completions` | 멱등 PICK 완료 반영 |
| `POST /operations/{operation_id}/place-completions` | 멱등 PLACE 완료 반영 |

요청 모델은 허용하지 않는 JSON 필드를 거부한다. SQL에는 문자열 보간을 사용하지
않고 sqlite parameter binding만 사용한다. domain 오류는 HTTP 404, 409, 422로
명확히 구분한다.
