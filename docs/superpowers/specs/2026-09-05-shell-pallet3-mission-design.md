# 차량 Shell 기반 Pallet 3 Direct Mission 설계

## 목표

Pallet 3 `NORMAL` 물류 이동을 차량에서 실행하는 `bash` mission script로
순차 처리한다. 차량은 기존 `vehicle_command_api`가 실제 ROS 2 Nav2, Auto Dock,
포크 callback을 처리한 결과를 받아 다음 단계를 실행한다. Fleet Manager는 미션을
시작하고 차량 reply를 대시보드에 기록하며, PICK/PLACE 물류 전이만 Inventory에
즉시 반영한다. Logistics Orchestrator는 이 미션의 명령을 소유하지 않는다.

## 책임 경계

| 구성요소 | 책임 | 하지 않는 일 |
| --- | --- | --- |
| `pallet3_mission.sh` (차량) | 순차적인 미션 실행, 로컬 Vehicle Command API 완료 상태 대기, Fleet Manager mission event 응답 대기 | Inventory 직접 호출, ROS 노드 재기동 |
| `vehicle_command_api` (차량) | 기존 ROS command/callback 처리, shell process group 시작·종료, stop 시 ROS 정지와 shell 종료 | Inventory 상태 전이 |
| Fleet Bridge | Fleet Manager와 차량 API 사이의 HTTP 전달 | 미션 순서 판단, 재고 변경 |
| Fleet Manager | Inventory 작업 생성, shell 시작 요청, dashboard 상태 저장, PICK/PLACE reply를 Inventory에 반영 | 개별 Nav2·포크·후진 명령의 순차 전송 |
| Inventory | 재고 예약 및 PICK/PLACE 원장 전이 | 차량 ROS 제어 |

기존 차량 status relay는 그대로 Fleet Manager에 `INIT`, `WAIT`, `DRIVE`,
`PICK`, `PLACE`, `FAIL`과 세부 reply를 보고한다. 따라서 대시보드는 기존과
동일한 Fleet Manager 상태/로그를 사용한다.

## HTTP 계약

### 미션 시작

Fleet Manager에 다음 요청을 추가한다.

```text
POST /api/v1/vehicles/{robot_id}/missions/pallet3
```

Fleet Manager는 다음을 순서대로 수행한다.

1. Fleet snapshot이 `WAIT`인지 확인한다.
2. Inventory `POST /api/v1/operations`으로 `NORMAL`, `docker -> p3`, `robot_id` 작업을 만든다.
3. 자신의 `direct_pallet3_missions` 저장소에 `operation_id`, `robot_id`,
   `phase=STARTING`을 기록한다.
4. Fleet Bridge를 통해 차량의 다음 endpoint를 호출한다.

```text
POST /v1/missions/pallet3
```

요청은 Inventory가 생성한 `operation_id`와 Fleet Manager event base URL을 가진다.
차량 endpoint는 새 process group으로 shell을 시작한 뒤 `202`를 반환한다. Fleet
Manager는 그 응답을 성공으로 받은 경우에만 시작 요청을 `202`로 반환한다.

Inventory 생성 뒤 차량이 shell을 수락하지 못하면 Fleet Manager는 작업을 자동
완료·취소하지 않는다. 차량 이동이 없음을 확인한 운영자가 Inventory 복구 API로
예약을 해제한다. 이는 실제 PICK 가능성이 있는 분산 실패에서 재고를 임의로
되돌리지 않기 위한 안전 경계다.

### 차량 reply와 Inventory 반영

shell은 로컬 ROS 완료를 확인한 뒤 Fleet Manager에 동기 event를 보낸다.

```text
POST /api/v1/vehicles/{robot_id}/missions/pallet3/{operation_id}/events
```

event body는 `event_type`과 `idempotency_key`만 허용한다.

| event_type | shell이 보내는 시점 | Fleet Manager의 동작 | 성공 응답 뒤 shell 다음 단계 |
| --- | --- | --- | --- |
| `PICK_COMPLETED` | Auto Dock `drive_ready`가 PICK 완료를 보고한 뒤 | Inventory `pick-completions`, key=`{operation_id}:pick` | P3 outbound FollowWaypoints |
| `PLACE_READY` | `FORK_DOWN_COMPLETE`가 확인된 뒤 | Inventory `place-completions`, key=`{operation_id}:place` | operation ID 없는 1초 후진 |
| `RETURN_COMPLETED` | Docker 복귀 Nav2 성공과 정지가 확인된 뒤 | 미션 복귀 완료를 기록한다. Inventory는 호출하지 않는다. | shell 정상 종료 |

Fleet Manager는 미션별 event type을 한 번만 수락한다. 동일한 idempotency key의
재전송은 원래 성공 응답을 반환한다. 순서에 맞지 않는 event, 다른 robot ID,
Inventory 상태 불일치는 `409`로 거절하고 shell은 즉시 stop한다.

Fleet Manager는 PICK/PLACE event를 처리할 때 Inventory API의 성공 응답을 받은
뒤에만 shell에 `2xx`를 반환한다. 그러므로 shell은 PICK 원장이 `TO_PLACE`가 되기
전 P3로 출발하지 않으며, PLACE 원장이 완료되기 전 후진하거나 Dock으로 복귀하지
않는다.

## 차량 Shell 순서

`pallet3_mission.sh`는 operation ID와 Fleet Manager mission event URL을 인자로
받는다. 기존 `vehicle_command_api`가 실행 중인 `127.0.0.1:8082`만 호출한다.
ROS 2 CLI를 직접 여러 개 실행하거나 Nav2를 별도 launch하지 않는다.

1. 로컬 `/v1/auto-dock`에 `PICK`, `DOCK_1`, `NORMAL`, operation ID를 요청한다.
2. `/v1/operation-status`가 같은 operation ID의
   `PICK_COMPLETE/AUTO_DOCK_PICK_COMPLETED`가 될 때까지 대기한다.
3. `PICK_COMPLETED` event를 Fleet Manager에 전송하고 성공 응답을 기다린다.
4. 로컬 `/v1/navigation/waypoints`로 `dock_1 -> p3` 경로를 요청하고
   `NAVIGATION_SUCCEEDED`를 기다린다.
5. 로컬 `/v1/fork/down`을 요청하고 `FORK_DOWN_COMPLETE`를 기다린다.
6. `PLACE_READY` event를 Fleet Manager에 전송하고 Inventory place 완료의 성공 응답을
   기다린다.
7. 로컬 `POST /v1/missions/pallet3/{operation_id}/placed`를 호출해 차량 API의
   operation context를 해제한다. 이 호출 뒤의 차량 명령에는 operation ID를 넣지
   않는다.
8. 로컬 `/v1/cmd-vel`에 `linear_x=-0.18`, `linear_y=0`, `angular_z=0`,
   `hold_ms=1000`을 요청하고 `MANUAL_COMMAND_EXPIRED`를 기다린다.
9. operation ID 없이 P3 -> Dock 1 FollowWaypoints를 요청하고 Docker 도착의
   `NAVIGATION_SUCCEEDED`를 기다린다. Nav2 성공으로 속도 0 정지가 확인된 뒤
   `RETURN_COMPLETED` event를 Fleet Manager에 전송하고 shell을 정상 종료한다.

`/v1/missions/pallet3/{operation_id}/placed`는 같은 operation ID의 P3 mission만
해제하며, 활성 Nav2·Auto Dock·fork 명령이 있으면 거절한다. 이 endpoint는
`WAIT/PLACE_COMPLETED`를 Fleet Manager에 보고한 뒤 차량의 현재 operation ID를
비운다. 따라서 place 완료 뒤 후진과 Docker 복귀는 완료된 Inventory operation에
연결되지 않는다.

Outbound 경로는 다음 waypoint를 사용한다.

```text
(-0.440, -0.900,  0.0)
(-0.440, -1.690, -1.5707963267948966)
(-0.440, -2.380, -1.5707963267948966)
```

Return 경로는 다음 waypoint를 사용한다.

```text
(-0.440, -1.690,  0.0)
(-0.440, -0.900,  0.0)
( 0.085, -0.905,  0.0)
```

## Stop·취소·통신 장애

Fleet Manager의 기존 endpoint
`POST /api/v1/vehicles/{robot_id}/commands/stop`는 새 mission을 포함한 stop이다.
Fleet Bridge는 기존 차량 `/v1/stop`으로 전달한다.

차량 `vehicle_command_api`의 `stop()`은 하나의 lock 안에서 다음을 수행한다.

1. active Pallet 3 shell을 취소 상태로 표시하고 process group에 `SIGTERM`을 보낸다.
2. `/cmd_vel` 0을 발행하고 활성 Nav2 goal을 cancel하며, Auto Dock stop topic을 발행한다.
3. 이후 도착하는 shell 대기 성공·ROS callback은 취소 generation과 다르면 무시한다.
4. Fleet Manager에 `FAIL/API_STOP` 상태를 기존 relay로 보고한다.

shell은 `INT`, `TERM`, `EXIT` trap으로 다음 단계 실행을 막는다. Docker 도착과
`RETURN_COMPLETED` 성공 이외의 종료에서는 로컬 `/v1/stop`을 best-effort로 요청한
뒤 종료한다.
Fleet Manager stop이 먼저 호출한 `/v1/stop`과 중복되어도 stop은 idempotent여야 한다.

shell이 Fleet Manager mission event endpoint에서 연결 오류 또는 비-2xx를 받으면
0.5초 간격으로 재시도한다. 연속 5초 동안 성공 응답이 없으면 local `/v1/stop`을
best-effort로 호출하고 실패 종료한다. 서버가 Inventory에 반영하지 못했을 때도 같은
규칙이 적용된다.

PLACE 완료 전 stop·cancel·Nav2/Auto Dock/fork 실패가 발생하면 Inventory operation은
active 상태로 남긴다. PLACE 완료 성공 뒤 후진 또는 Docker 복귀가 실패하거나 stop되면
Inventory operation은 이미 완료이며, 차량 복귀만 별도 복구한다.

## 구성과 배포

- 차량 shell: `vehicle_communication/tools/pallet3_mission.sh`
- 차량 API: mission start endpoint, PID/process-group supervisor, 기존 stop 통합
- Fleet Bridge: `pallet3_mission` capability와 start endpoint proxy
- Fleet Manager: Inventory base URL 설정, mission 저장소, start/event endpoints
- direct 실행 중에는 Logistics Orchestrator를 중지하고 Fleet Manager의
  `ORCHESTRATOR_EVENT_URL`을 비운다. Fleet Manager 상태 저장·대시보드 표시는 계속된다.

Fleet Manager와 Inventory는 중앙 서버에서 각각 `127.0.0.1:8090`,
`127.0.0.1:8081`을 기본으로 사용한다. 차량 shell의 local Vehicle Command API 기본
주소는 `http://127.0.0.1:8082`이다.

## 검증

자동 테스트는 아래를 보장한다.

1. Fleet Manager가 `WAIT`가 아닌 차량에는 Inventory 작업을 만들지 않는다.
2. 시작 요청은 Inventory operation ID를 shell에 전달하며, 중복 미션 시작을 거절한다.
3. `PICK_COMPLETED`와 `PLACE_READY`가 각각 한 번만 올바른 Inventory completion을 호출한다.
4. 순서 위반·다른 robot ID·Inventory 409는 event endpoint가 거절한다.
5. Fleet Manager stop은 Vehicle Command API stop relay와 shell process group `SIGTERM`을
   모두 수행하며, stop 뒤 다음 shell 명령이 실행되지 않는다.
6. shell은 단계별 local API 성공/완료 상태와 5초 Fleet Manager event 응답 유예를 확인한다.
7. PLACE 성공 뒤 후진·return command에는 operation ID가 없고, 후진 또는 return 실패가
   Inventory 완료를 되돌리지 않는다.
8. `RETURN_COMPLETED`는 Docker 도착과 Nav2 정지 뒤에만 기록된다.
