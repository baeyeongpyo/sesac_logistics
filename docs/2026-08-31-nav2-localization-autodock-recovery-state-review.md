# 차량 주행 상태 설계

작성일: 2026-08-31
범위: `inventory_db`, `fleet_bridge`, `vehicle_communication`, Auto Dock

## 상태

## 구현 상태 보고 규약

Fleet Manager에 저장되는 상태는 차량의 실제 이벤트를 근거로 하며 payload는 아래
필드로 고정한다.

```text
state, previous_state, operation_id, attempt_id,
source(VEHICLE|NAV2|AUTO_DOCK|API), detail, observed_at(UTC)
```

Bridge는 payload를 저장·변환하지 않는다. 같은 상위 상태의 Auto Dock 상세 이벤트는
Fleet Manager 최신 snapshot만 갱신하고 상태 변경 로그를 추가하지 않는다. `INIT`과
`FAIL`에서 현장 대응을 마친 뒤 `/v1/operation/idle`을 호출하면 `WAIT`가 보고되며,
그 이후 정상 `DRIVE -> PICK -> DRIVE -> PLACE` 흐름에는 작업별 승인 없이 진행한다.

차량의 최상위 상태는 아래 여섯 개만 사용한다.

| 상태 | 의미 |
| --- | --- |
| `INIT` | 차량 재부팅 뒤, 아직 수동 점검과 initial pose 설정을 하지 않은 상태 |
| `WAIT` | 수동 점검과 관리자 승인이 끝난 주행 준비 상태 또는 대기 작업이 없는 상태 |
| `DRIVE` | Nav2 주행 중 |
| `PICK` | Auto Dock Pick 작업 중 |
| `PLACE` | Auto Dock Place 작업 중 |
| `FAIL` | Nav2가 실패를 응답했거나 API `stop`·`cancel`이 실행된 상태 |

기존 `IDLE`은 외부 API 호환용 별칭으로만 유지하고, 서버에서는 `WAIT`와 같은
의미로 처리한다.

## 관리자 승인

관리자 승인은 매 작업마다 받지 않는다. `INIT` 또는 `FAIL`에서 수동 대응을
끝낸 뒤 `WAIT`로 복귀할 때만 관리자가 승인한다. 승인 ID, 사용자 식별,
인증·권한 처리는 이 사이드 프로젝트 범위에서 사용하지 않는다.

```text
INIT / FAIL
  -> 수동 대응
  -> 필요 시 initial_pose 설정
  -> 관리자 승인
  -> WAIT
```

`WAIT`가 된 차량은 이후 자동 작업을 계속 수행할 수 있다. 따라서 정상 흐름의
작업 완료, 새 작업 생성, `WAIT -> DRIVE` 전이에는 추가 승인이 없다.

## 정상 작업 흐름

```text
WAIT
  -> DRIVE              # Pick 지점으로 Nav2 주행
  -> PICK               # Auto Dock Pick
  -> DRIVE              # Place 지점으로 Nav2 주행
  -> PLACE              # Auto Dock Place
  -> DRIVE              # 다음 작업이 있으면 즉시 계속

PLACE 완료 후 다음 작업이 없으면 WAIT
```

Auto Dock의 `drive_ready`를 받으면 다음 작업이 있으면 곧바로 `DRIVE`로
전환한다. 다음 작업이 없을 때만 `WAIT`으로 전환한다. 이 정상 흐름에서는
관리자 승인을 다시 요구하지 않는다.

## 실패와 복구

```text
Nav2 FAILED 응답
API stop 또는 cancel
  -> FAIL

FAIL
  -> 수동 대응 및 정지 확인
  -> 필요 시 initial_pose 설정
  -> 관리자 승인
  -> WAIT
  -> DRIVE              # 대기 작업이 있으면 자동 시작
```

- 위치 정보가 일시적으로 좋지 않아도 이미 `DRIVE`인 Nav2 주행을 자동으로
  `FAIL`로 바꾸지 않는다. 경고는 로그와 관제 화면에만 표시한다.
- 차량이 재부팅돼 멈춰 있으면 무조건 `INIT`이다. 수동 대응과 관리자 승인 뒤에
  `WAIT`으로 복귀한다.
- `stop`과 `cancel` 자체는 즉시 처리한다. 이후 재주행만 관리자 승인 뒤에
  가능하다.

## 최소 데이터

```text
vehicle_id, status,
operation_id, task_type(PICK | PLACE),
attempt_id, reason, updated_at
```

`reason`에는 `NAV2_FAILED`, `API_STOP`, `API_CANCEL`처럼 `FAIL` 원인을 기록한다.
Auto Dock 이벤트에는 `operation_id`, `attempt_id`를 포함해 이전 작업의 이벤트가
현재 작업을 바꾸지 못하게 한다.
