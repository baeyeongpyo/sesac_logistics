# 직접 실행 Pallet 3 Runner 설계

## 목표

`192.168.100.27`에서 Logistics Orchestrator(`:8083`)를 호출하지 않고,
`NORMAL` 파렛트 하나를 `docker`에서 `p3`로 이송한다. 차량 상태 보고는 Vehicle
Command API, Fleet Bridge, Fleet Manager 경로를 계속 사용하므로 기존 대시보드에서
확인할 수 있어야 한다.

## 범위

독립 Python runner와 집중 단위 테스트를 추가한다. runner는 기존 Inventory, Fleet
Manager, Fleet Bridge API만 호출한다. ROS 2 action·topic을 직접 발행하거나 차량 ROS
graph를 변경하지 않는다.

서버 기본 endpoint는 다음과 같다.

| 서비스 | URL |
| --- | --- |
| Inventory | `http://192.168.100.27:8081` |
| Fleet Bridge | `http://192.168.100.27:8080` |
| Fleet Manager | `http://192.168.100.27:8090` |

runner는 배포·테스트를 위해 로봇 ID와 endpoint override 인자를 받는다.

## 흐름

1. 차량이 Auto Dock 명령을 수락할 수 있는 상태인지 확인한다.
2. `payload_type=NORMAL`, `source_zone_id=docker`, `destination_zone_id=p3`인
   Inventory 작업을 하나 만들고, 반환된 `operation_id`를 저장한다.
3. Fleet Manager를 통해 `DOCK_1`에서 `NORMAL`을 대상으로 하는 Auto Dock `PICK`을
   전송한다.
4. 일치하는 Fleet Manager 보고 `AUTO_DOCK_PICK_COMPLETED`를 기다린 후,
   idempotency key `<operation_id>:pick`으로 Inventory `pick-completions`를 호출한다.
5. Fleet Manager를 통해 정의된 `dock_1 -> p3` FollowWaypoints 경로를 전송하고,
   일치하는 `NAVIGATION_SUCCEEDED`를 기다린다.
6. Fleet Manager를 통해 fork DOWN을 전송하고, 일치하는 `FORK_DOWN_COMPLETE`를
   기다린다.
7. Fleet Manager를 통해 `linear_x=-0.18`, `linear_y=0`, `angular_z=0`,
   `hold_ms=1000`을 전송하고, 일치하는 `MANUAL_COMMAND_EXPIRED`를 기다린다.
8. `<operation_id>:place` key로 Inventory `place-completions`를 호출한다. 이것이
   물류 작업의 종료 지점이다. P3 재고를 증가시키고, 차량 적재 상태를 해제하며,
   Inventory 작업을 완료한다.
9. 정의된 `p3 -> dock_1` FollowWaypoints 명령을 물류 작업 ID 없이 전송한다. 복귀는
   완료된 물류 작업의 일부가 아니므로 완료를 기다리지 않고, Fleet Manager가 명령을
   수락하면 runner는 성공 종료한다.

## 상태와 보고 규칙

- Auto Dock, Nav2, fork, 후진, stop 명령은 모두 기존 Fleet Manager / Fleet Bridge
  경로를 사용한다. 따라서 Vehicle Command API의 Fleet Manager 보고와 대시보드를
  유지한다.
- 운송 작업 ID는 `place-completions`가 성공할 때까지만 사용한다. dock 복귀 명령에는
  절대로 원래 물류 작업 ID를 넣지 않는다.
- `place-completions` 전에는 0.5초마다 `GET /api/v1/operations/active`를 조회한다.
  작업 ID가 존재하고 기대 상태여야 한다. pick 완료 전에는 `TO_PICK`, 그 뒤에는
  `TO_PLACE`가 기대 상태다.
- active-operation 조회 실패부터 monotonic outage timer를 시작한다. 성공 응답은
  timer를 초기화한다. 성공 응답 없이 5초가 지나야 `COMMUNICATION_LOST`로 보고,
  이후 명령을 전송하지 않고 종료한다.
- HTTP 200인데 작업 ID가 없거나 상태가 다르면 통신 장애가 아니라
  `OPERATION_STATE_MISMATCH`다. 즉시 다음 명령을 차단하고 종료한다.
- 일치하는 Fleet 보고가 `FAIL`이거나, 명령 전달 timeout/503이거나, 로컬
  `SIGINT`/`SIGTERM`을 받으면 place 완료 전 runner를 중단한다. Inventory는 운영자
  복구를 위해 현재 활성 상태를 유지한다. 로컬 signal을 받으면 Fleet Manager stop을
  한 번 best-effort로 요청한다.
- 전달 결과가 불명인 명령은 절대로 자동 재전송하지 않는다.

## 경로

P3로 가는 경로:

```text
(-0.440, -0.900, 0)
(-0.440, -1.690, -pi/2)
(-0.440, -2.340, -pi/2)
```

Dock 1 복귀 경로:

```text
(-0.420, -2.000, -pi/2)
(-0.440, -0.900, 0)
( 0.085, -0.905, 0)
```

## 오류 처리

차량 보고만으로 Inventory 완료를 추론하지 않는다. `pick-completions`는 Auto Dock
PICK 완료 뒤에만 호출하며, `place-completions`는 fork DOWN 완료와 제한 시간 후진
완료 뒤에만 호출한다.

place 완료 전 runner가 종료되면 Inventory 작업을 취소하거나 강제 완료하지 않는다.
runner 내부 deadline이 발생했을 때 Fleet API가 연결되어 있으면 stop을 한 번
요청한다. 실제 네트워크 단절 상황에서는 이 요청이 차량까지 도달하지 않을 수 있다.

## 검증

단위 테스트는 fake HTTP transport와 monotonic clock을 사용해 다음을 검증한다.

- 정확한 API 호출 순서와 payload
- 모든 차량 명령의 상태 보고 gate
- idempotency key와 Inventory 상태 전이
- 복귀 경로에 물류 작업 ID가 없는지
- 5초 통신 장애 유예 및 연결 복구 시 timer 초기화
- 일치하는 Fleet 실패·작업 상태 불일치 시 즉시 중단
- 로컬 signal에서 best-effort stop 요청 한 번
