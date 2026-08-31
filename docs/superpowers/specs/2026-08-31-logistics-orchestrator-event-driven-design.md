# 이벤트 기반 물류 오케스트레이터 설계

작성일: 2026-08-31

## 목적

새 `logistics_orchestrator` Docker 서비스를 추가한다. 이 서비스는 Fleet
Manager의 차량 상태 이벤트와 Inventory의 물류 상태 이벤트를 수신하고, 최신 두
원장을 다시 조회해 다음 물류 작업을 결정한다. 차량 명령은 Fleet Bridge를 통해서만
전달한다.

원장의 소유권은 바꾸지 않는다.

| 서비스 | 소유 책임 |
| --- | --- |
| `fleet_manager` | 차량 최신 상태, 상태 변경 로그, 차량 상태 이벤트 outbox |
| `inventory_db` | zone, 재고, 예약, 운송 작업, PICK/PLACE 완료 이벤트, Inventory 이벤트 outbox |
| `logistics_orchestrator` | 작업 우선순위, 차량 배차, 명령 단계, 이벤트 inbox, 외부 명령 outbox, 재시작 재조정 |
| `fleet_bridge` | 차량 HTTP API 중계 |
| `vehicle_communication` | Nav2와 Auto Dock의 실제 관측 상태 보고 |

## 물류 zone과 초기 설정

오케스트레이터는 명시적인 bootstrap API로 Inventory에 없는 zone만 생성한다. 기존
zone의 좌표, enabled, capacity, 재고는 수정하거나 초기화하지 않는다. 재고 수량은
관리자가 Inventory API에서 설정·실사 보정한다.

| zone | capacity | 지도 좌표 / 설정 |
| --- | ---: | --- |
| `docker` | 24 | `map_0825`, x=`0.110`, y=`-0.980`, yaw=`0.0` |
| `p1`~`p3` | 각 1 | `warehouse_zones.yaml`의 `P1`~`P3` 중심점 |
| `f1`~`f9` | 각 1 | `warehouse_zones.yaml`의 `F1`~`F9` 중심점 |
| `n1`~`n9` | 각 1 | `warehouse_zones.yaml`의 `N1`~`N9` 중심점 |

`p1`~`p3`에는 FRESH 또는 NORMAL 모두 적치할 수 있다. `f*`는 FRESH만,
`n*`는 NORMAL만 목적지로 선택한다. Nav2 goal 좌표와 Auto Dock `location`은
zone 정책 파일에 분리해 둔다. 기본 Auto Dock location은 대문자 marker ID(`P1`,
`F1`, `N1`)이며 `docker`는 `DOCKER`로 설정한다.

## 작업 선택 규칙

후보 작업은 source 재고의 `available_quantity`, 목적지의 실제 적치 수량과 inbound
예약을 모두 반영해 선택한다. 모든 "첫 번째" slot은 숫자 오름차순이다.

1. Docker에 FRESH가 하나라도 있으면 FRESH를 우선한다.
   - 빈 `p1`~`p3`가 있으면 `docker/FRESH -> p*`
   - P가 모두 차 있고 빈 `f1`~`f9`가 있으면 `docker/FRESH -> f*`
   - 둘 다 없으면 FRESH가 남아 있는 동안 NORMAL 작업을 만들지 않는다.
2. Docker의 FRESH 실제 재고가 0이고 Docker에 NORMAL이 있으면 NORMAL을 선택한다.
   - 빈 `p1`~`p3`가 있으면 `docker/NORMAL -> p*`
   - P가 모두 차 있고 빈 `n1`~`n9`가 있으면 `docker/NORMAL -> n*`
3. Docker의 FRESH와 NORMAL 실제 재고가 모두 0일 때만 내부 보충을 한다.
   - 빈 `p1`~`p3`가 있으면 `f1`~`f9/FRESH -> p*`
   - FRESH source가 모두 비었을 때만 `n1`~`n9/NORMAL -> p*`
4. 적합한 source·destination 또는 `WAIT` 차량이 없으면 작업을 만들지 않는다.

Inventory의 `POST /api/v1/operations`가 source 재고·목적지 슬롯·차량 중복을 원자적으로
검증하므로, 후보 선택 직후 409 응답을 받으면 해당 cycle의 후보를 버리고 최신 상태를
다시 조회한다.

## 이벤트 전달

두 원장 서비스는 상태를 DB transaction으로 확정한 뒤 같은 transaction에 outbox 행을
기록한다. HTTP 요청 실패는 원장 transaction을 되돌리지 않는다. background dispatcher는
성공 응답을 받을 때까지 지수 backoff로 재전송한다.

```text
vehicle -> fleet_bridge -> fleet_manager state 저장
                             -> fleet_event_outbox
                             -> POST orchestrator /api/v1/events/fleet

inventory mutation -> inventory DB 저장
                   -> inventory_event_outbox
                   -> POST orchestrator /api/v1/events/inventory
```

- Fleet outbox는 **모든** 차량 상태 보고를 enqueue한다. 같은 상위 상태 `WAIT`이라도
  `NAVIGATION_SUCCEEDED`, `AUTO_DOCK_PICK_COMPLETED`,
  `AUTO_DOCK_PLACE_COMPLETED`의 detail이 다르므로 상태 변경 로그만으로는 충분하지 않다.
- Inventory outbox는 zone bootstrap, stock 변경, operation 생성, PICK 완료, PLACE 완료를
  enqueue한다.
- 이벤트 envelope은 `event_id`, `event_type`, `occurred_at`, `payload`를 포함한다.
  source별 `event_id`는 불변이며 재전송에도 유지한다.

오케스트레이터는 `orchestrator_inbox`의 `(source, event_id)` UNIQUE 제약으로 중복을
제거한다. HTTP 2xx는 inbox 기록이 완료된 뒤에만 반환한다.

## 오케스트레이터 상태와 명령 흐름

오케스트레이터는 이벤트 수신 후 event payload만으로 명령하지 않는다. Fleet Manager의
`GET /api/v1/vehicles/{robot_id}`와 Inventory의 active operations, stocks, zones를 다시
조회한다. 이를 통해 지연·중복 이벤트가 현재 상태를 되돌리지 못하게 한다.

`orchestrator_steps`는 operation별 진행 단계와 명령 전송 여부를 저장한다.

```text
RESERVED
  -> NAV_TO_PICK_SENT
  -> PICK_SENT
  -> PICK_COMMITTED
  -> NAV_TO_PLACE_SENT
  -> PLACE_SENT
  -> PLACE_COMMITTED
```

1. `WAIT` 차량에 해당 차량의 활성 `TO_PICK` 작업이 있고 Fleet detail이
   `NAVIGATION_SUCCEEDED`이면 Auto Dock PICK을 명령한다.
2. detail이 `AUTO_DOCK_PICK_COMPLETED`이면 idempotency key
   `{operation_id}:pick`으로 Inventory `pick-completions`를 호출한다. 성공 뒤 destination
   좌표로 Nav2 PLACE goal을 보낸다.
3. `TO_PLACE` 작업의 `NAVIGATION_SUCCEEDED`이면 Auto Dock PLACE를 명령한다.
4. detail이 `AUTO_DOCK_PLACE_COMPLETED`이면 `{operation_id}:place` 키로
   `place-completions`를 호출한다. 완료 Inventory 이벤트를 다시 받아 새 작업을 선택한다.
5. 활성 작업이 없는 `WAIT` 차량은 작업 선택 규칙에 따라 operation을 만들고 source로
   Nav2 PICK goal을 보낸다.

Nav2 goal에는 `operation_id`, `purpose=PICK|PLACE`, `frame_id=map`, zone pose를 보낸다.
Auto Dock에는 같은 `operation_id`, `operation=PICK|PLACE`, `product_type`, zone marker
location, `target={"type":"NEAREST"}`를 보낸다.

외부 명령은 `command_outbox`에 먼저 저장한 뒤 Fleet Bridge로 전송한다. HTTP timeout은
명령 수락 여부가 불명확하므로 새 명령을 즉시 재전송하지 않는다. Fleet 상태가 해당
`operation_id`의 예상 `DRIVE`, `PICK`, `PLACE`, 또는 완료 detail로 진행될 때까지
`DELIVERY_UNKNOWN`으로 둔다.

## 복구와 안전

- `INIT`, `FAIL`, `DRIVE`, `PICK`, `PLACE` 상태에는 새 명령을 보내지 않는다.
- `FAIL` 뒤에는 사람이 차량을 대응하고 `/operation/idle`로 `WAIT`를 보고할 때까지
  자동 재개하지 않는다.
- 복구 뒤 `WAIT`가 되면 Inventory의 활성 작업과 저장 step을 다시 대조하여 아직 완료되지
  않은 단계부터 이어간다. `PICK_COMMITTED`이면 PLACE 주행부터, `TO_PICK`이면 PICK 주행
  또는 PICK 도킹부터 재개한다.
- 서비스 시작 시와 `/api/v1/reconcile` 호출 시 Fleet Manager와 Inventory 전체 snapshot을
  읽어 inbox 누락·오케스트레이터 재시작을 보정한다. 이 작업은 새 상태를 만들지 않고
  안전하게 재실행할 수 있다.
- Inventory 완료 API와 오케스트레이터 event inbox 모두 멱등하므로 `drive_ready`와
  Inventory 이벤트가 중복되어도 재고는 한 번만 반영된다.

## Docker와 API

새 `logistics_orchestrator/`는 FastAPI, SQLite, Docker Compose 구조를 사용한다.

```text
GET  /healthz
GET  /api/v1/status
POST /api/v1/bootstrap
POST /api/v1/reconcile
POST /api/v1/events/fleet
POST /api/v1/events/inventory
```

환경 변수는 `ORCHESTRATOR_DB_PATH`, `INVENTORY_URL`, `FLEET_MANAGER_URL`,
`FLEET_BRIDGE_URL`, `ORCHESTRATOR_EVENT_URL`, `EVENT_RETRY_INTERVAL_SEC`를 사용한다.
Fleet Manager와 Inventory는 각각 `ORCHESTRATOR_EVENT_URL`이 설정된 경우에만 outbox
dispatcher를 활성화한다.

## 테스트와 완료 기준

테스트를 먼저 추가한다.

1. zone bootstrap은 없는 zone만 만들고 재고·기존 설정을 덮어쓰지 않는다.
2. Docker FRESH, Docker NORMAL, Docker empty 각각에서 source·destination 우선순위가
   정확히 선택된다.
3. P/F/N의 capacity와 reservation이 있는 slot은 후보에서 제외된다.
4. Fleet `WAIT`의 Nav2 완료, PICK 완료, PLACE 완료 detail이 올바른 Auto Dock,
   Inventory 완료, 다음 Nav2 명령으로 이어진다.
5. `INIT`·`FAIL`에는 명령하지 않으며, `WAIT` 복구 뒤 활성 작업을 이어간다.
6. Fleet 및 Inventory outbox는 HTTP 실패 뒤 동일 event_id로 재전송한다.
7. Orchestrator inbox와 Inventory PICK/PLACE completion은 중복 이벤트에 대해
   한 번만 반영된다.
8. 세 Docker Compose 설정과 기존 Inventory/Fleet/Bridge/Vehicle 회귀 테스트를 통과한다.
