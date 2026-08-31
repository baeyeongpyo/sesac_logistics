# Fleet Manager 차량 상태 수집·저장 설계

작성일: 2026-08-31

## 목적

차량의 실제 Nav2·Auto Dock 이벤트를 Fleet Manager에 저장한다. Fleet Bridge는
상태를 저장하거나 판단하지 않고 전달만 한다. Inventory는 물류·재고 상태의
원장으로 유지하며, 이후 `logistics_orchestrator`가 Fleet Manager와 Inventory를
조회하여 다음 명령을 제어한다.

이 설계는 상태 변경 로그만 저장한다. 좌표 궤적, 속도, ROS bag은 범위에 포함하지
않는다.

## 역할과 데이터 흐름

```text
Nav2 Action / Auto Dock ROS topic / 차량 API 기동
  -> vehicle_communication
  -> Fleet Bridge 상태 전달 API
  -> Fleet Manager 상태 저장 API
  -> SQLite (최신 snapshot + 상태 변경 로그)

logistics_orchestrator
  -> Fleet Manager 최신 차량 상태 조회
  -> Inventory 작업·재고 상태 조회
  -> Fleet Bridge 명령 API 호출
```

| 구성요소 | 책임 |
| --- | --- |
| `vehicle_communication` | 실제 차량 이벤트를 정규화해 상태 payload를 생성·전달한다. |
| `fleet_bridge` | 차량의 상태 payload를 Fleet Manager로 그대로 HTTP 전달한다. 상태를 저장하거나 다음 작업을 판단하지 않는다. |
| `fleet_manager` | 차량별 최신 상태와 상태 변경 로그를 SQLite에 저장·조회한다. |
| `inventory_db` | `operation_id`의 물류 작업, PICK/PLACE 재고 이벤트, 적재 상태를 저장한다. |
| `logistics_orchestrator` | 두 저장소를 조회하고 명령 및 다음 작업 생성을 결정한다. 이번 구현 범위에서는 만들지 않는다. |

## Fleet Manager 외부 상태

Fleet Manager가 저장·제공하는 상위 상태는 아래 여섯 값이다.

| 상태 | 차량의 실제 근거 |
| --- | --- |
| `INIT` | 차량 API 기동 또는 차량 재부팅 |
| `WAIT` | 수동 대응 뒤 `/v1/operation/idle`이 완료됐거나 Nav2/Auto Dock 완료 뒤 대기 중 |
| `DRIVE` | Nav2가 NavigateToPose goal을 실제 수락 |
| `PICK` | Auto Dock이 PICK 작업의 실행 상태를 실제 발행 |
| `PLACE` | Auto Dock이 PLACE 작업의 실행 상태를 실제 발행 |
| `FAIL` | Nav2 실패/취소 결과, Auto Dock `ERROR`, API `stop` 또는 `cancel` |

위 상태는 HTTP 명령을 받은 순간에 바꾸지 않는다. Nav2 action goal response와
result, Auto Dock `/auto_dock/status` 및 `/auto_dock/drive_ready`, 차량 API
기동·정지 처리에서 확인된 이벤트만 근거로 한다.

현재 차량 API의 내부 `IDLE`, `PICK_COMPLETE`, `PLACE_COMPLETE`, `FAILED`,
`CANCELLED` 등은 기존 명령 guard와의 호환을 위해 유지할 수 있다. Fleet Manager로
보내는 payload에서는 각각 `WAIT`, `WAIT`, `WAIT`, `FAIL`, `FAIL`로 정규화한다.

## 상태 전달 포맷

차량은 Fleet Bridge에 `POST /api/v1/vehicle-status/{robot_id}`로 아래 JSON을
보낸다. `robot_id`는 URL 경로가 권위이며 request body에는 넣지 않는다.

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

| 필드 | 규칙 |
| --- | --- |
| `state` | 필수. `INIT`, `WAIT`, `DRIVE`, `PICK`, `PLACE`, `FAIL` 중 하나 |
| `previous_state` | 필수. 첫 상태일 때 `null` |
| `operation_id` | Inventory가 생성한 UUID. 독립 주행·기동 상태는 `null` 가능 |
| `attempt_id` | Nav2 action attempt. Nav2와 관계없는 상태는 `null` 가능 |
| `source` | 필수. `VEHICLE`, `NAV2`, `AUTO_DOCK`, `API` 중 하나 |
| `detail` | 필수. `NAVIGATION_STARTED`, `NAVIGATION_FAILED`, `AUTO_DOCK_ALIGNING`, `API_STOP` 등 원인 코드 |
| `observed_at` | 필수 UTC ISO-8601 시각. 차량에서 실제 이벤트를 관측한 시각 |

승인 ID, 사용자 ID, 인증·권한 필드는 넣지 않는다.

## Fleet Manager 저장 모델

Fleet Manager는 두 SQLite 테이블을 가진다.

### `vehicle_states`

차량별 최신 상태 snapshot이다. `robot_id`를 기본 키로 upsert한다.

```text
robot_id, state, previous_state,
operation_id, attempt_id,
source, detail,
observed_at, updated_at
```

### `vehicle_state_logs`

차량 상태가 바뀔 때만 append하는 이력이다. `id`는 SQLite 자동 증가 키다.

```text
id, robot_id,
state, previous_state,
operation_id, attempt_id,
source, detail,
observed_at, created_at
```

같은 상태에서 detail만 달라진 보고는 `vehicle_states` snapshot만 갱신한다. 상위
상태가 바뀔 때만 `vehicle_state_logs`에 새 행을 넣는다. 예를 들어 Auto Dock의
`SEARCHING -> ALIGNING`은 모두 `PICK`이므로 로그를 추가하지 않고, `PICK -> WAIT`
또는 `PICK -> FAIL`은 로그를 추가한다.

Fleet Manager는 `operation_id`의 존재 여부를 Inventory에 동기 조회하거나 외래 키로
강제하지 않는다. 서로 다른 서비스의 데이터베이스이므로, `operation_id`는 두
원장을 연결하는 참조값이다.

## API

### Fleet Bridge가 호출하는 저장 API

```text
POST /api/v1/vehicles/{robot_id}/state
```

유효한 상태 payload를 받으면 최신 snapshot을 upsert하고, 상태 변경이면 로그를
추가한 뒤 저장된 최신 snapshot을 `200`으로 반환한다.

### 조회 API

```text
GET /healthz
GET /api/v1/vehicles
GET /api/v1/vehicles/{robot_id}
GET /api/v1/vehicles/{robot_id}/logs
```

로그 조회는 `limit`(기본 100, 최대 500)과 `before_id`를 지원해 최신 로그부터
페이지를 나눈다. 좌표·경로 데이터를 반환하지 않는다.

### Fleet Bridge 전달 API

```text
POST /api/v1/vehicle-status/{robot_id}
```

Bridge는 request body를 검증한 뒤 `FLEET_MANAGER_URL`의 저장 API로 전달하고,
응답 상태와 JSON을 그대로 차량에 반환한다. 자체 DB나 메모리 cache를 만들지 않는다.

## 차량 측 변경

`vehicle_communication`에 상태 reporter를 추가한다. reporter URL은 runtime 환경
변수 또는 CLI 인자로 설정하며, Fleet Bridge의 전달 API를 가리킨다.

1. API 시작 뒤 `INIT`을 한 번 전달한다.
2. Nav2 goal 요청은 pending으로만 기록한다. ROS action server가 goal을 수락한
   response를 받은 뒤 `DRIVE`를 전달한다.
3. Nav2 success는 `WAIT`, Nav2 failed/cancelled는 `FAIL`을 전달한다.
4. Auto Dock arrival 요청은 pending으로만 기록한다. Auto Dock status topic에서
   `SEARCHING`, `ALIGNING`, `INSERTING`, `WAIT_*`, `REVERSING`, `TURNING`을 받은
   뒤 operation에 따라 `PICK` 또는 `PLACE`를 전달한다.
5. Auto Dock `drive_ready`는 `WAIT`, `ERROR`·rejected는 `FAIL`을 전달한다.
6. `/v1/stop`과 `/v1/navigation/cancel`은 즉시 기존 정지 처리를 수행하고
   `FAIL`을 전달한다.

상태 전달 HTTP 오류는 차량의 Nav2/Auto Dock 실행 상태를 바꾸지 않는다. 이
사이드 프로젝트에서는 전달 실패를 다음 상태 변경 때 다시 전송하며, 영속 outbox나
재시도 큐는 범위에서 제외한다.

## 배포

`fleet_manager/`는 `inventory_db/`와 같은 FastAPI + SQLite + Docker Compose
구조로 만든다. `FLEET_MANAGER_DB_PATH`의 기본값은 `/data/fleet_manager.db`다.

Fleet Bridge `command-api` 컨테이너에는 `FLEET_MANAGER_URL`을 추가한다. 각 차량의
`vehicle_communication/runtime.env`에는 Fleet Bridge 전달 URL을 추가한다.

## 테스트와 완료 기준

테스트를 먼저 추가한다.

1. Fleet Manager: 새 차량 상태 저장, 최신 snapshot 갱신, 상태 변경 로그 추가,
   같은 상태 detail 갱신 시 로그 미추가, 목록·로그 페이지 조회.
2. Fleet Bridge: 유효 payload를 Fleet Manager로 그대로 전달하고, Fleet Manager
   오류를 그대로 반환.
3. Vehicle: Nav2 명령만으로 `DRIVE`를 보고하지 않음, Nav2 수락 후 `DRIVE`, 성공
   후 `WAIT`, 실패·cancel·stop 후 `FAIL` 보고.
4. Auto Dock: arrival 명령만으로 `PICK`/`PLACE`를 보고하지 않음, 실제 status 뒤
   해당 상태 보고, `drive_ready` 후 `WAIT`, `ERROR` 후 `FAIL` 보고.
5. 회귀: 기존 vehicle command, Fleet Bridge relay, Inventory 테스트를 모두 실행.

완료 시 `logistics_orchestrator`는 Fleet Manager의 최신 상태와 로그를 읽을 수
있고, Fleet Bridge는 어떤 차량 상태도 저장하지 않는다.
