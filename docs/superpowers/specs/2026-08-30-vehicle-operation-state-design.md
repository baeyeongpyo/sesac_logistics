# 차량 작업 상태와 서버 주도 복구 설계

## 목적과 범위

차량별 `vehicle_command_api`는 실제 차량의 현재 상태를 제공하고, Fleet
Manager는 그 응답을 변경하지 않고 중계한다. Inventory와 중앙 서버는 운송 작업
이력과 다음 지시의 기준이며, 차량은 서버가 보낸 독립적인 `DRIVE` 또는 Auto
Dock 명령만 수행한다.

이 문서는 다음을 정의한다.

- 차량 상태 조회 응답과 상태 전이
- 기동, 취소, 실패 뒤의 작업 허가 절차
- Nav2와 Auto Dock 명령의 분리
- 서버가 작업 이력에 따라 재시작·실패 뒤 작업을 다시 지시하는 방식

다음은 범위 밖이다.

- 차량이 재기동 뒤 이전 작업을 스스로 재개하는 기능
- Auto Dock의 내부 탐색·삽입 단계부터의 재개
- Fleet Manager의 차량 상태 캐시

## 책임 경계

```text
Inventory / 중앙 서버
  - operation_id, 재고·적재 상태, 다음 작업 지시의 기준
  - 차량 상태 이벤트를 받아 다음 DRIVE 또는 AUTO DOCK 명령 결정

Fleet Manager
  - robot_id별 vehicle_command_api HTTP 요청·응답 중계

vehicle_command_api
  - Nav2 action, Auto Dock ROS topic을 받아 차량 상태를 단일하게 변경
  - /v1/vehicle-status 제공

vehicle_task_reporter
  - 차량 정규화 이벤트를 중앙 서버에 전송하고 전송 실패를 재시도
  - 차량 상태를 변경하지 않음
```

상태는 정적 설정 파일이나 Fleet Manager 캐시에 저장하지 않는다. `robot_id`,
ROS topic 이름, HTTP 주소만 설정으로 둔다. 동적 상태의 기준은 실행 중인
`vehicle_command_api`다.

Inventory는 operation UUID와 적재 상태를 보존한다. `next-instruction`은
비적재 차량에는 `PICK` 목적지, 적재 차량에는 `PLACE` 목적지를 같은
`operation_id`와 함께 반환한다. 차량이 다시 켜져도 이 서버 이력을 기준으로
다음 명령을 새로 보낸다.

## 상태 조회 계약

Fleet Manager의 다음 경로는 차량 API 응답을 그대로 중계한다.

```text
GET /api/v1/vehicle-command/{robot_id}/vehicle-status
```

`operation` 계약은 다음과 같다.

```json
{
  "robot_id": "robot_2",
  "battery": {
    "raw_value": 8311,
    "received_at": "2026-08-30T09:19:01.221Z",
    "stale": false
  },
  "operation": {
    "operation_id": null,
    "previous_operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
    "state": "IDLE",
    "previous_state": "CANCELLED",
    "detail": "OPERATOR_READY"
  }
}
```

| 필드 | 의미 |
| --- | --- |
| `operation_id` | 현재 또는 종료 처리가 아직 끝나지 않은 Inventory 작업 UUID. `IDLE`과 `INIT`에서는 `null`이다. |
| `previous_operation_id` | 가장 최근에 끝났거나 운영자에 의해 해제된 작업 UUID. 현재 작업이 없을 때도 직전 작업을 추적한다. |
| `state` | 아래 표의 공개 차량 상태다. |
| `previous_state` | 가장 최근 상태 전이 직전의 공개 상태다. 취소·실패 시에는 중단 지점이고, `IDLE`에서는 직전에 끝나거나 해제된 상태다. 없으면 `null`이다. |
| `detail` | 기계가 읽을 수 있는 구체 사유. 예: `NAVIGATION_FAILED`, `AUTO_DOCK_ERROR`, `OPERATOR_READY`. |

`CANCELLED`와 `FAILED`에서는 원래 `operation_id`를 유지한다. 따라서 중앙
서버는 어느 Inventory 작업이 중단됐는지, 어느 차량 단계에서 중단됐는지
`previous_state`와 함께 확인할 수 있다.

## 공개 상태

| 상태 | 의미 | 새 작업 수락 |
| --- | --- | --- |
| `INIT` | 차량 API가 시작됐으며 운영자가 아직 작업을 허가하지 않았다. | 불가 |
| `IDLE` | 적재·진행 명령이 없는 작업 가능 상태다. | DRIVE 또는 PICK 가능 |
| `DRIVE` | Nav2 goal을 수행 중이다. | 불가 |
| `PICKING` | Auto Dock이 Pick 작업을 수행 중이다. | 불가 |
| `PICK_COMPLETE` | Pick이 완료돼 차량이 적재 상태다. | PLACE 목적의 DRIVE만 가능 |
| `PLACE` | Auto Dock이 Place 작업을 수행 중이다. | 불가 |
| `PLACE_COMPLETE` | Place 완료 이벤트가 생성된 순간의 종료 상태다. | 불가 |
| `FAILED` | Nav2, Auto Dock 또는 차량 명령 처리 실패다. | 불가 |
| `CANCELLED` | 운영자 취소 또는 즉시 정지로 작업을 중단했다. | 불가 |

Auto Dock의 `SEARCHING`, `ALIGNING`, `INSERTING`, `WAIT_UP_COMPLETE`,
`WAIT_DOWN_COMPLETE`, `REVERSING`, `TURNING`은 공개 상태로 추가하지 않는다.
작업의 의미는 각각 `PICKING` 또는 `PLACE`로 유지하고 원본 상태·사유는
`detail`에 보존한다.

## 상태 전이

```text
차량 API 기동
  -> INIT

INIT -- POST /v1/operation/idle --> IDLE

IDLE -- DRIVE 명령 --> DRIVE
DRIVE -- Nav2 성공 --> IDLE 또는 PICK_COMPLETE
IDLE -- AUTO DOCK PICK 명령 --> PICKING
PICKING -- drive_ready --> PICK_COMPLETE
PICK_COMPLETE -- DRIVE 명령 --> DRIVE
PICK_COMPLETE -- AUTO DOCK PLACE 명령 --> PLACE
PLACE -- drive_ready --> PLACE_COMPLETE -- 자동 --> IDLE

DRIVE | PICKING | PICK_COMPLETE | PLACE -- 취소 --> CANCELLED
DRIVE | PICKING | PICK_COMPLETE | PLACE -- 오류 --> FAILED
CANCELLED | FAILED -- POST /v1/operation/idle --> IDLE
```

`DRIVE` 성공 뒤의 상태는 명령을 시작하기 전의 적재 맥락으로 정한다.

- 비적재 `IDLE`에서 시작한 DRIVE가 성공하면 Nav2 명령이 끝났으므로 `IDLE`이
  된다. Reporter는 `DRIVE_COMPLETED` 이벤트를 먼저 전송하고, 중앙 서버가
  필요한 경우 별도의 Auto Dock 명령을 보낸다.
- `PICK_COMPLETE`에서 시작한 DRIVE가 성공하면 차량은 계속 적재 상태이므로
  `PICK_COMPLETE`를 유지한다. 중앙 서버가 같은 작업 ID로 Place Auto Dock
  명령을 보낸다.

`PLACE_COMPLETE`는 Reporter가 `PLACE_COMPLETED` 이벤트를 만들기 위한 종료
전이다. 직후 상태 스냅샷은 자동으로 `IDLE`이 되며, `previous_operation_id`와
`previous_state: "PLACE_COMPLETE"`으로 완료 작업을 확인한다. 폴링만으로
완료 이벤트를 보장하지 않으며 Reporter가 정규화 이벤트를 사용한다.

`INIT_POSE`는 `/initialpose`를 발행할 뿐 위치 추정의 정확성을 보증하지
않는다. 따라서 성공 응답 뒤에도 `INIT`을 유지한다. 운영자 또는 상위 서버가
현장 확인 뒤 `POST /v1/operation/idle`을 호출할 때만 `IDLE`이 된다.

`CANCELLED`와 `FAILED`는 자동으로 `IDLE`이 되지 않는다. 차량은 0 속도를
발행하고, 원인을 조치한 뒤 명시적인 `idle` API 호출이 있을 때까지 DRIVE와
Auto Dock 명령을 거부한다.

## 명령 API와 서버 시퀀싱

DRIVE와 Auto Dock은 서로 다른 명령이다. 정상 작업과 재시작 뒤 작업 모두
동일한 두 명령을 중앙 서버가 순서대로 보낸다.

```text
서버 -- POST /v1/navigation/goals(operation_id, goal) --> 차량
차량 -- DRIVE_COMPLETED event --> Reporter --> 서버
서버 -- POST /v1/auto-dock(operation_id, PICK|PLACE spec) --> 차량
차량 -- PICK_COMPLETED 또는 PLACE_COMPLETED event --> Reporter --> 서버
```

`/v1/navigation/goals`는 물류 작업의 경우 Inventory가 만든
`operation_id`를 받아야 한다. Nav2 action handle을 위한 차량 내부 attempt ID는
별도로 생성해 재시도와 같은 Inventory 작업 UUID를 혼동하지 않는다.

`/v1/auto-dock`은 Nav2 goal을 받지 않는다. 서버가 Nav2 성공 이벤트를 확인한
뒤 호출하며, 차량 API는 다음 JSON을 `/{robot_id}/nav2/arrival` topic에
발행한다.

```json
{
  "status": "SUCCEEDED",
  "location": "DOCK_1",
  "operation": "PICK",
  "product_type": "NORMAL",
  "target": {"type": "NEAREST"}
}
```

Auto Dock의 `drive_ready` topic은 실제 fork 작업, 후진, 준비 자세까지 끝난
뒤의 완료 신호다. `READY` 상태 메시지만으로 완료를 판단하지 않는다. Auto
Dock의 `ERROR` 또는 거부 상태는 `FAILED`로 전이한다.

## 기동·취소·실패 뒤의 재개

차량은 `resume` API나 임의 상태 주입 API를 제공하지 않는다. 재기동하면 항상
`INIT`으로 시작하며, 이전 작업 상태를 메모리에서 복원해 움직이지 않는다.

재개 절차는 다음과 같다.

1. 운영자가 `INIT_POSE`를 발행하고 실제 위치를 확인한다.
2. 운영자가 `POST /v1/operation/idle`로 차량을 작업 가능 상태로 전환한다.
3. 중앙 서버는 `previous_operation_id`와 Inventory의 활성 작업,
   `next-instruction`, 차량 적재 상태를 대조한다.
4. 중앙 서버는 안전한 시작점에서 동일한 `operation_id`를 넣어 DRIVE 또는
   Auto Dock 명령을 다시 보낸다.

Auto Dock이 이미 실행 중이던 작업은 탐색·삽입 중간에서 재개하지 않는다.
안전한 위치로 Nav2 재주행한 뒤 Pick 또는 Place를 처음부터 다시 수행한다.

## `idle` API

```http
POST /v1/operation/idle
Content-Type: application/json

{"reason": "OPERATOR_CONFIRMED"}
```

이 API는 `INIT`, `CANCELLED`, `FAILED`에서만 상태를 `IDLE`로 바꾼다. 활성
Nav2 action 또는 Auto Dock 동작이 감지되면 `409 Conflict`로 거부한다.
`PICK_COMPLETE`에서는 적재 상태를 잃지 않도록 호출을 거부한다.

성공 시에는 현재 `operation_id`를 `null`로 비우고 해당 값을
`previous_operation_id`로 옮긴다. `previous_state`는 `INIT`, `CANCELLED`,
또는 `FAILED`를 유지한다.

## 이벤트와 Reporter

Reporter가 중앙 서버에 전송하는 정규화 이벤트는 최소 다음을 포함한다.

```json
{
  "event_id": "차량이 생성한 UUID",
  "robot_id": "robot_2",
  "operation_id": "inventory-operation-uuid",
  "event_type": "DRIVE_COMPLETED",
  "state": "IDLE",
  "previous_state": "DRIVE",
  "detail": "NAVIGATION_SUCCEEDED",
  "occurred_at": "2026-08-30T09:19:01.221Z"
}
```

이벤트 유형은 `DRIVE_COMPLETED`, `PICK_COMPLETED`, `PLACE_COMPLETED`,
`OPERATION_FAILED`, `OPERATION_CANCELLED`다. Reporter는 `event_id`를
idempotency key로 중앙 서버에 전송하고, HTTP 전송 실패 시 로컬 outbox에서
재시도한다. 상태 조회 API의 일시적인 값에 의존해 완료 이벤트를 재구성하지
않는다.

## 검증 기준

- 차량 API 시작 직후 상태가 항상 `INIT`이다.
- `INIT_POSE` 성공만으로는 `INIT`이 바뀌지 않는다.
- `idle` API는 허용 상태에서만 `IDLE`을 만들고, 활성 동작에는 `409`를 낸다.
- 취소·실패는 작업 ID와 직전 상태를 보존하고 새 명령을 거부한다.
- PICK 완료 뒤에는 Place 목적 DRIVE만 수락한다.
- Auto Dock `drive_ready`만 Pick·Place 완료로 판정한다.
- Place 완료 이벤트가 한 번 전송되고, 상태 스냅샷은 `IDLE`과 직전 작업
  정보를 반환한다.
- 차량 재기동 뒤에는 자동 명령이 발생하지 않으며, 서버가 같은 작업 UUID로
  독립 DRIVE·Auto Dock 명령을 다시 보낼 때만 재개된다.
