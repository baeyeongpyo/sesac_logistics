# Pallet 3 POC 미션 설계

## 목적과 범위

픽업이 완료된 차량을 `dock_1`에서 `pallet_3`까지 이동시키고, 작업자의
하역 확인 뒤 포크를 내린 후, 포크 하강 완료 신호를 확인하여 후진하고
`dock_1`으로 복귀시키는 단일 차량 POC 미션을 제공한다.

이 기능은 기존 재고 예약, 자동 pick/place 완료 처리, 일반 자동 배차 흐름을
변경하지 않는다. POC 미션은 명시적인 API 호출로만 시작하며 `pallet_3`만
지원한다.

## 실행 경로

모든 차량 명령은 기존 소유권 경계를 통과한다.

```text
Logistics Orchestrator
  -> Fleet Manager
  -> Fleet Bridge
  -> Vehicle Command API
  -> ROS2 (/follow_waypoints, /fork/command, /cmd_vel, /fork/state)
```

Orchestrator가 Fleet Bridge나 차량 Command API를 직접 호출하지 않는다.

## API 계약

### 미션 시작

`POST /api/v1/poc/pallet-3-missions`

요청 본문은 `{"robot_id":"robot_2"}`이며, 차량이 `WAIT` 상태이고
`dock_1`에서 픽업이 끝난 상태일 때만 수락한다. 응답은 생성된 `mission_id`와
초기 phase `NAV_TO_PALLET_3`을 포함한다.

같은 차량에 진행 중인 POC 미션이 있거나 차량이 `WAIT`가 아니면 `409`를
반환한다. 명령 전달 실패나 미지의 전달 결과는 미션을 진행시키지 않고
`FAILED` 또는 `DELIVERY_UNKNOWN`으로 보존한다.

### 하역 확인

`POST /api/v1/poc/pallet-3-missions/{mission_id}/unload-confirmation`

`AWAITING_UNLOAD_CONFIRMATION` 상태의 미션에서만 수락한다. 이 확인은
작업자가 차량 위치, 팔레트 해제 가능 여부, 사람·장애물 부재를 확인했다는
의미다. 수락되면 포크 하강 명령을 한 번만 발행하고 phase를
`AWAITING_FORK_DOWN`으로 바꾼다. 다른 phase의 요청은 `409`를 반환한다.

### 상태 조회

`GET /api/v1/poc/pallet-3-missions/{mission_id}`는 mission ID, robot ID,
phase, 마지막 오류, 생성·갱신 시각을 반환한다. UI나 운영자는 이 endpoint로
하역 확인 가능 시점과 실패 원인을 확인한다.

## 고정 경로와 단계 전이

시작 전제는 차량이 `dock_1`에서 픽업을 끝내고 포크를 든 상태다. pick 또는
fork up은 이 POC의 책임이 아니다.

| Phase | 트리거 | 차량 명령 또는 다음 상태 |
| --- | --- | --- |
| `NAV_TO_PALLET_3` | 미션 생성 | FollowWaypoints로 출발 경로 전송 |
| `AWAITING_UNLOAD_CONFIRMATION` | 출발 Nav2 성공 | 작업자 하역 확인 대기 |
| `AWAITING_FORK_DOWN` | 하역 확인 | `/fork/command`에 `DOWN` 전송 |
| `REVERSING` | 명령 뒤 수신한 `DOWN_COMPLETE` | `linear_x=-0.18`, `linear_y=0`, `angular_z=0`, `hold_ms=1000` 전송 |
| `NAV_TO_DOCK_1` | 수동 후진 만료·0속도 발행 확인 | FollowWaypoints로 복귀 경로 전송 |
| `COMPLETED` | 복귀 Nav2 성공 | 미션 종료 |
| `FAILED` | 어떤 단계의 실패·취소·타임아웃 | `/v1/stop` 전송 후 미션 중단 |

출발 경로는 아래 세 pose를 순서대로 사용한다.

| 순서 | x | y | yaw |
| ---: | ---: | ---: | ---: |
| 1 | -0.440 | -0.900 | 0 |
| 2 | -0.420 | -2.000 | -π/2 |
| 3 | -0.420 | -2.400 | -π/2 |

복귀 경로는 포크 하강 후의 후진이 끝난 뒤에만 전송한다.

| 순서 | x | y | yaw |
| ---: | ---: | ---: | ---: |
| 1 | -0.420 | -2.000 | -π/2 |
| 2 | -0.440 | -0.900 | 0 |
| 3 | 0.085 | -0.905 | 0 |

## 포크 완료 이벤트

차량 Command API는 `/fork/state`의 `std_msgs/msg/String`을 구독한다. `data`는
JSON 객체이며, POC에는 다음 계약을 적용한다.

```json
{"state":"DOWN_COMPLETE","error":""}
```

- `DOWN_COMPLETE`와 빈 `error`는 포크 하강 성공이다.
- 빈 값이 아닌 `error`, JSON 형식 오류, 예상하지 않은 완료 상태, 또는
  설정된 대기 시간 내 응답 부재는 실패다.
- `DOWN`을 발행하기 전에 수신된 메시지는 무시한다. 발행 후에 수신한
  `DOWN_COMPLETE`만 해당 미션에 연결한다.
- 포크 완료/오류는 Fleet 상태 이벤트에 `operation_id=mission_id`와 함께
  전달해 Orchestrator가 다른 차량·다른 미션의 메시지를 사용하지 않게 한다.

`UP_COMPLETE`는 이 POC의 단계 전이에 사용하지 않는다. 다만 같은 구독과
파서는 이후 픽업 POC에서 재사용할 수 있어야 한다.

## 컴포넌트 변경

### Vehicle Command API

- `--fork-state-topic` 인자(기본 `/fork/state`)와 구독을 추가한다.
- 포크 하강 요청에 POC mission ID를 전달할 수 있게 하고, 하강 완료·오류를
  Fleet Bridge 상태 relay로 보고한다.
- 기존 독립 `/v1/fork/up`, `/v1/fork/down` 호출은 호환성을 유지한다.
- 수동 후진의 타이머가 1초 뒤 0속도를 발행한 뒤에만 `MANUAL_COMMAND_EXPIRED`
  상태를 보고한다.

### Fleet Bridge와 Fleet Manager

- Fleet Bridge의 기존 vehicle-native `navigation/waypoints`, `fork/down`,
  `cmd-vel`, `operation-status`, `stop` 중계 API를 사용한다.
- Fleet Manager에는 해당 중계 endpoint와 capability를 추가한다. Orchestrator가
  사용하는 유일한 차량 명령 진입점은 Fleet Manager다.
- Fleet 상태 모델은 POC의 포크 완료 event source와 detail을 보존한다.

### Logistics Orchestrator

- POC mission과 명령 outbox를 영속 저장한다. 프로세스 재시작 뒤에도 같은
  `mission_id` 및 phase로 재개하며, 같은 phase의 명령은 중복 발행하지 않는다.
- Fleet event의 `robot_id`, `operation_id`, `detail`이 현재 mission과 모두
  일치할 때만 다음 phase로 진행한다.
- 시작, 하역 확인, Nav2 완료, 포크 완료, 수동 후진 완료마다 상태를 저장한다.

## 실패 처리와 안전

- Nav2 실패·취소, 포크 오류·타임아웃, 후진 명령 전달 실패, 복귀 Nav2 실패는
  모두 `FAILED`로 종료하고 `/v1/stop`을 요청한다.
- POC 중에는 다른 POC 시작과 하역 확인의 중복 제출을 막는다.
- `DOWN_COMPLETE` 전에는 후진이나 복귀 Nav2 명령을 절대 전송하지 않는다.
- 후진은 기존 차량 Command API를 통해 전송해 1,000 ms 후 자동 0속도가
  발행되도록 한다. 직접 `cmd_vel`을 지속 발행하지 않는다.
- 실제 차량에서 시작하기 전에 `/follow_waypoints`, `/fork/command`,
  `/fork/state`, `/cmd_vel`의 이름·타입과 차량 Command API의 `linear_x` 제한을
  확인한다.

## 검증

단위·계약 테스트는 다음을 포함한다.

1. POC 시작이 팔레트 3 출발 waypoint 세 개를 Fleet Manager로 정확히 전송한다.
2. 출발 Nav2 성공 전에는 하역 확인과 포크 하강을 거부한다.
3. 하역 확인은 정확히 한 번의 `DOWN`만 전송한다.
4. 명령 이후의 `DOWN_COMPLETE`만 `-0.18`·`1000 ms` 후진으로 진행시킨다.
5. 후진 0속도 완료 뒤 정확한 dock_1 복귀 waypoint 세 개를 전송한다.
6. 중복 fleet event, 중복 하역 확인, Orchestrator 재시작이 명령을 재발행하지 않는다.
7. 포크 오류·타임아웃 및 각 Nav2 실패가 stop 요청과 `FAILED`를 남긴다.
8. Fleet Manager가 등록되지 않은 차량 또는 미지원 capability를 거부한다.

통합 POC는 사람과 장애물이 없는 저속 환경에서 수행한다. 먼저 Nav2 경로와
포크 상태 이벤트를 별도로 확인한 뒤 전체 미션을 실행한다.
