# Nav2 토픽 취소와 통합 작업 상태 설계

## 목적

Foxglove Bridge 3.2.2가 `action_msgs/srv/CancelGoal`의 C++ type support symbol을
찾지 못해 Nav2 cancel service client를 만들지 못하는 환경에서, ROS 패키지를
재설치하지 않고 Nav2 취소를 안정적으로 전달한다.

Fleet Bridge는 Foxglove `clientPublish`로 단방향 취소 토픽을 발행하고, 별도의
상태 토픽을 구독해 실제 취소 여부를 확인한다. 차량은 Nav2, 수동 주행, 비전 기반
상차·하차의 현재 작업 상태를 하나의 계약으로 외부에 제공한다.

## 설계 원칙

- Foxglove의 generic service client와 `action_msgs/srv/CancelGoal` 원격 호출을
  사용하지 않는다.
- 외부 명령과 상태는 배포 부담이 적은 `std_msgs/msg/String` JSON으로 전달한다.
- 명령 접수와 실제 작업 종료를 구분한다.
- 취소 성공은 Nav2 또는 비전 작업의 종료와 최종 이동 출력 0이 모두 확인된
  경우에만 선언한다.
- 모든 API 명령의 `command_id`는 서버가 생성한다. 클라이언트는 일반 명령에서
  UUID를 만들거나 전달하지 않는다.
- 취소 명령 재전송은 같은 `command_id`를 사용하며 차량에서 멱등하게 처리한다.
- 특정 이전 명령을 지정할 때만 클라이언트가 서버가 반환했던 UUID를
  `target_command_id`로 전달한다.
- 수동 입력은 자동 작업보다 우선한다. 수동 입력이 들어오면 자동 작업을 중단하고
  수동 입력 종료 후 자동으로 재개하지 않는다.
- 수동 개입 직전의 작업 상태와 작업 식별자를 보존한다. 후속 재주행 판단 로직은
  이 정보를 사용하지만 이번 구현 범위에는 포함하지 않는다.
- 취소, 정지, 수동 개입은 포크 높이를 변경하지 않는다. 포크는 명시적인 포크
  명령이 있을 때만 움직인다.

## 전체 구조

```text
Fleet REST API
  POST /api/v1/robots/{robot_id}/nav2/cancel
  POST /api/v1/robots/{robot_id}/stop
        |
        | Foxglove clientPublish: std_msgs/msg/String
        v
  navigation.cancel_topic
        |
        v
vehicle operation supervisor
  +-- Nav2 goal bridge/action client
  +-- cmd_vel mux 이동 제어권·zero 확인
  +-- vision task controller 상태·취소 계약
  +-- safety stop
        |
        | std_msgs/msg/String
        v
  navigation.status_topic
        |
        | Foxglove subscription
        v
Fleet Bridge 취소 확인·재시도·API 응답
```

Fleet Bridge는 로봇별 Foxglove URI와 토픽을 `fleet.yaml`에서 읽는다. 물리 차량의
ROS graph가 비네임스페이스 토픽을 사용하면 `/navigation/cancel`과
`/operation/status`를 설정하고, 다중 로봇 시뮬레이터는
`/{robot_id}/navigation/cancel`과 `/{robot_id}/operation/status`를 설정할 수 있다.
코드는 토픽 이름을 하드코딩하지 않는다.

command client는 취소 요청 확인을 위해 차량의 원본 status 토픽을 직접 구독한다.
일반 모니터링 경로에서는 기존 telemetry worker가 같은 원본 토픽을 서버 Domain의
`/{robot_id}/operation/status`로 재발행한다. 따라서 API 요청이 없을 때도 중앙
Foxglove와 rosbag에서 로봇별 최신 상태를 관찰할 수 있다.

차량의 operation supervisor는 외부 상태 토픽의 단일 작성자다. Nav2 goal bridge,
cmd_vel mux, 비전 작업 제어기는 내부 이벤트만 supervisor에 제공한다. 여러 노드가
같은 외부 상태 토픽을 직접 발행해 상태 순서가 뒤섞이는 구조는 사용하지 않는다.

## 설정 계약

취소 토픽, 상태 토픽, 재시도 횟수와 시간값은 모두 로봇별 설정으로 관리한다.

```yaml
vehicles:
  - id: robot_2
    foxglove_uri: ${ROBOT_2_FOXGLOVE_URI}
    enabled: true
    command:
      topic: /cmd_vel
      type: geometry_msgs/msg/Twist
      max_linear_x: 0.3
      max_angular_z: 1.0
      max_hold_ms: 1000
      publish_rate_hz: 10
    navigation:
      goal_topic: /navigation/goal
      goal_type: std_msgs/msg/String
      cancel_topic: /navigation/cancel
      cancel_type: std_msgs/msg/String
      status_topic: /operation/status
      status_type: std_msgs/msg/String
      cancel_max_attempts: 3
      cancel_retry_interval_ms: 500
      cancel_confirmation_timeout_ms: 2000
```

설정 의미와 검증 범위는 다음과 같다.

| 설정 | 의미 | 기본값 | 유효 범위 |
|---|---|---:|---:|
| `goal_topic` | command ID와 PoseStamped 데이터를 담은 Nav2 goal command 토픽 | `/navigation/goal` | 유효한 절대 ROS 토픽 |
| `goal_type` | Nav2 goal command envelope 메시지 타입 | `std_msgs/msg/String` | 고정 |
| `cancel_topic` | Fleet Bridge가 취소 요청을 발행하는 토픽 | `/navigation/cancel` | 유효한 절대 ROS 토픽 |
| `cancel_type` | 취소 요청 메시지 타입 | `std_msgs/msg/String` | 고정 |
| `status_topic` | 차량이 통합 상태를 발행하는 토픽 | `/operation/status` | 유효한 절대 ROS 토픽 |
| `status_type` | 통합 상태 메시지 타입 | `std_msgs/msg/String` | 고정 |
| `cancel_max_attempts` | 최초 발행을 포함한 최대 취소 발행 횟수 | `3` | `1..10` |
| `cancel_retry_interval_ms` | 상태 확인 전 다음 취소 발행까지의 간격 | `500` | `100..10000` ms |
| `cancel_confirmation_timeout_ms` | 마지막 발행 후 최종 상태를 기다리는 시간 | `2000` | `100..30000` ms |

값은 코드 상수나 환경별 launch 파일에 중복 정의하지 않는다. Fleet Bridge config
loader가 누락된 값에는 기본값을 적용하고, 범위를 벗어난 값은 시작 시
`ConfigError`로 거부한다. 운영자는 `fleet.yaml`만 수정하고 Fleet Bridge를
재시작해 정책을 바꿀 수 있다.

기존 `geometry_msgs/msg/PoseStamped` goal 발행 계약은 command ID를 전달할 수 있는
`std_msgs/msg/String` goal command envelope로 교체한다. 기존 `cancel_service`와
`cancel_service_type` 설정도 `cancel_topic`과 `cancel_type`으로 교체한다. 모든
차량 설정과 loader 테스트를 같은 변경에서 갱신해 서비스 기반 경로나 식별자 없는
goal 경로가 실수로 다시 선택되지 않게 한다.

차량은 각 취소 메시지에 포함된 `max_attempts`와
`confirmation_timeout_ms`를 사용한다. 이로써 중앙 정책과 차량 측 실패 판정이
서로 다른 값을 사용하는 것을 방지한다.

## UUID 소유권과 명령 지정 규칙

API 서버가 명령 식별자의 단일 생성자다. schema validation을 통과한 모든 명령은
API가 접수할 때 UUID v4 `command_id`를 하나 생성하고 응답에 포함한다. Nav2와
향후 상차·하차처럼 장기 실행되는 명령은 차량으로 보내는 command envelope와
상태에도 같은 값을 사용한다.

| 식별자 | 생성 주체 | 용도 |
|---|---|---|
| `command_id` | API 서버 | 현재 API 명령 자체의 식별자 |
| `target_command_id` | API 서버가 과거에 생성, 클라이언트가 선택적으로 재전달 | 취소·정지 대상인 이전 명령 지정 |
| `operation_id` | 상위 Fleet/RMF 작업 계층 | 여러 하위 명령을 묶는 물류 작업 식별자 |

클라이언트는 goal, cmd_vel, cancel, stop의 `command_id`를 요청 body에 넣지 않는다.
서버가 응답으로 반환한 값을 저장했다가 특정 명령을 취소해야 할 때만
`target_command_id`로 전달한다. cancel에서 target을 생략하면 해당 로봇의 현재
활성 자동 명령을 대상으로 하고, stop에서 생략하면 모든 활성 이동을 대상으로
한다.

취소 명령 자체도 별도의 `command_id`를 가진다. 예를 들어 goal 명령 ID가 `G1`인
상태에서 이를 취소하면 서버는 취소 명령 ID `C1`을 새로 생성하고, payload에는
`command_id=C1`, `target_command_id=G1`을 넣는다. C1의 N회 재시도에서는 새로운
UUID를 만들지 않는다.

차량 supervisor는 현재 활성 Nav2·비전 작업의 `command_id`를 보관한다. 지정된
target이 활성 명령과 다르면 다른 작업을 취소하지 않고
`TARGET_COMMAND_NOT_ACTIVE`를 반환한다. target이 이미 terminal 상태라면
`TARGET_ALREADY_TERMINAL`로 멱등 성공 처리한다.

기존 `geometry_msgs/msg/Twist` 형식을 유지하는 bounded cmd_vel API도 서버가
command ID를 생성해 응답하지만, UUID를 Twist 데이터에 삽입하지 않는다. 서버의
로봇별 command registry가 publish loop와 command ID를 연결하므로 활성 cmd_vel을
target으로 지정하면 해당 loop를 중단하고 zero Twist를 보낸다. 장기 실행 명령은
차량 supervisor까지 ID가 전달되므로 API 서버가 재시작해도 transient-local 최신
상태를 통해 현재 command ID를 다시 확인할 수 있다.

## Nav2 goal 명령 토픽 계약

Nav2 goal API는 서버가 생성한 command ID와 기존 PoseStamped 형태를 하나의
`std_msgs/msg/String.data` JSON envelope로 차량에 전달한다.

```json
{
  "version": 1,
  "command_id": "42dc49df-a591-4298-99e8-e78af80c1089",
  "command": "NAVIGATE_TO_POSE",
  "operation_id": "delivery-20260824-0042",
  "pose": {
    "header": {
      "stamp": {"sec": 1787541418, "nanosec": 880201225},
      "frame_id": "map"
    },
    "pose": {
      "position": {"x": 1.5, "y": 0.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
```

차량 goal bridge는 envelope를 검증하고 `command_id`를 active goal handle과 함께
보관한 뒤 내부적으로 `NavigateToPose.Goal`을 만든다. accepted, result, cancel 상태
모두 같은 command ID와 연결한다. `operation_id`는 상위 물류 작업이 제공한 경우에만
사용하며, 없으면 `null`이다.

## 취소 명령 토픽 계약

취소 명령은 `std_msgs/msg/String.data`에 다음 JSON을 넣는다.

```json
{
  "version": 1,
  "command_id": "f83a3b7b-30d9-4c38-a887-f5f48d281998",
  "target_command_id": "42dc49df-a591-4298-99e8-e78af80c1089",
  "command": "CANCEL_NAVIGATION",
  "attempt": 1,
  "max_attempts": 3,
  "confirmation_timeout_ms": 2000,
  "requested_at_ns": 1787541418880201225
}
```

필드 규칙은 다음과 같다.

| 필드 | 규칙 |
|---|---|
| `version` | 초기 계약은 정수 `1` |
| `command_id` | API 서버가 취소 명령에 부여하고 모든 재시도에서 유지하는 UUID |
| `target_command_id` | 특정 이전 명령 대상이면 해당 명령 UUID, 현재 활성 명령 대상이면 `null` |
| `command` | `CANCEL_NAVIGATION` 또는 `STOP` |
| `attempt` | `1`부터 시작하며 발행할 때마다 1 증가 |
| `max_attempts` | 해당 로봇의 `cancel_max_attempts` 설정값 |
| `confirmation_timeout_ms` | 해당 로봇의 최종 확인 제한시간 설정값 |
| `requested_at_ns` | 최초 API 요청 시각을 nanosecond로 기록하고 재시도에서 유지 |

차량은 같은 `command_id`를 다시 받아도 병렬 Nav2 cancel 요청을 만들지 않는다.
진행 중이면 최신 `attempt`만 기록하고 `CANCELING`을 다시 발행한다. 이미 완료된
요청이면 같은 최종 상태를 다시 발행한다. 최근 완료 요청 ID는 제한된 크기의
메모리 캐시에 보관한다.

잘못된 JSON, 지원하지 않는 `version` 또는 `command`, 범위를 벗어난 attempt는
실행하지 않는다. supervisor는 현재 작업 상태를 유지하면서 diagnostic 상태
메시지의 `detail`에 `INVALID_CANCEL_REQUEST`를 기록한다. 파싱할 수 있는
`command_id`가 있으면 그대로 포함하고, 없으면 `null`을 사용한다.

## 통합 상태 토픽 계약

통합 상태도 `std_msgs/msg/String.data` JSON으로 발행한다.

```json
{
  "version": 1,
  "robot_id": "robot_2",
  "state": "MANUAL",
  "previous_state": "NAVIGATING",
  "source": "MANUAL",
  "command_id": null,
  "previous_command_id": "42dc49df-a591-4298-99e8-e78af80c1089",
  "target_command_id": null,
  "operation_id": "delivery-20260824-0042",
  "attempt": null,
  "interrupted_by": "MANUAL",
  "resume_decision_required": true,
  "detail": "NAV_CANCEL_CONFIRMED",
  "timestamp_ns": 1787541418952019842
}
```

| 필드 | 의미 |
|---|---|
| `version` | 상태 계약 버전. 초기값은 `1` |
| `robot_id` | Fleet 설정의 canonical robot ID |
| `state` | 아래에 정의한 현재 외부 상태 |
| `previous_state` | 현재 상태 직전의 의미 있는 상태. 수동 개입 시 중단된 자동 작업 상태를 보존 |
| `source` | `SYSTEM`, `NAV2`, `MANUAL`, `VISION_PICK`, `VISION_PLACE`, `SAFETY` 중 하나 |
| `command_id` | 차량까지 ID가 전달된 현재 API 명령 ID. 로컬 명령이나 raw Twist이면 `null` |
| `previous_command_id` | 수동 개입 등으로 중단된 직전 API 명령 ID, 없으면 `null` |
| `target_command_id` | 취소·정지 상태이면 대상 명령 ID, 아니면 `null` |
| `operation_id` | Fleet 또는 작업 제어기가 부여한 작업 식별자. 없으면 `null` |
| `attempt` | 취소 요청과 관련된 가장 최근 발행 횟수, 아니면 `null` |
| `interrupted_by` | 수동 개입이면 `MANUAL`, 그 외에는 `null` |
| `resume_decision_required` | 자동 재개 여부를 별도 로직이 판단해야 하면 `true` |
| `detail` | 성공·실패·중단 원인을 나타내는 짧은 machine-readable 코드 |
| `timestamp_ns` | 차량에서 상태를 확정한 시각 |

상태 publisher는 `reliable`, `transient_local`, depth 1 QoS를 사용한다. 새
subscriber는 가장 최근 상태를 받을 수 있지만, Fleet Bridge는 과거의 terminal
상태를 현재 취소 요청의 성공으로 오인하지 않도록 반드시 취소 명령의
`command_id`를 비교한다.

## 상태 정의

### `STOPPED`

- 활성 자동 작업과 유효한 수동 이동 명령이 없다.
- 최종 이동 출력이 0이다.
- 시스템 시작 시 정상 초기 상태다.
- `/stop` 요청은 자동 작업 종료와 zero 출력이 모두 확인된 뒤 이 상태로 끝난다.

### `NAVIGATING`

- Nav2 `NavigateToPose` goal이 실제로 accepted 되었고 결과를 기다리고 있다.
- 목표 토픽을 수신했지만 action server가 아직 수락하지 않은 상태는
  `NAVIGATING`으로 선언하지 않는다.
- Nav2 goal이 성공하면 `COMPLETED`로 전이한다.

### `MANUAL`

- cmd_vel mux가 유효한 수동 명령을 최종 이동 제어권자로 선택했다.
- 자동 작업 중 수동 명령이 들어오면 자동 작업 출력을 즉시 차단하고 이 상태로
  전이한다.
- `previous_state`에는 중단된 `NAVIGATING`, `PICKING` 또는 `PLACING`을 저장한다.
- 수동 입력이 끝나도 자동 작업으로 복귀하지 않는다.

### `PICKING`

- 비전 작업 제어기가 물류 인식, 정렬, 접근, 포크 삽입 또는 들어올리기 작업을
  수락하고 수행 중이다.
- 내부 세부 단계는 외부 상태를 늘리지 않고 `detail`로 표현할 수 있다.

### `PLACING`

- 비전 작업 제어기가 하차 위치 인식, 정렬, 접근, 내려놓기 또는 후퇴 작업을
  수락하고 수행 중이다.
- 내부 세부 단계는 외부 상태를 늘리지 않고 `detail`로 표현할 수 있다.

### `COMPLETED`

- Nav2, 상차 또는 하차 작업이 실제 성공 결과를 반환했다.
- 어떤 작업이 완료됐는지는 `source`, `command_id`, `operation_id`로 구분한다.
- 다음 명령이 들어올 때까지 유지하는 terminal 상태다.

### `CANCELING`

- 명시적인 cancel 또는 stop 요청을 받고 활성 자동 작업 종료와 이동 정지를
  확인하는 중이다.
- 단순히 취소 토픽을 수신한 사실만으로 `CANCELED`를 발행하지 않는다.
- 중복 요청을 받으면 상태와 최신 `attempt`를 다시 발행한다.

### `CANCELED`

- 취소 대상 Nav2·비전 작업이 종료됐고 이동 출력이 0이다.
- `command_id`가 현재 취소 명령과 일치해야 Fleet Bridge가 성공으로 인정한다.
- 특정 명령 취소에서는 `target_command_id`도 요청값과 일치해야 한다.
- 다음 명령이 들어올 때까지 유지하는 terminal 상태다.

### `CANCEL_FAILED`

- 설정된 발행 횟수와 확인 제한시간을 모두 소진했지만 작업 종료 또는 zero 출력을
  확인하지 못했다.
- 실패 후에도 safety stop 또는 자동 출력 차단을 유지한다.
- 다음 명령 또는 명시적인 복구 판단 전까지 유지하는 terminal 상태다.

## 상태 전이 규칙

| 현재 상태 | 이벤트 | 다음 상태 | 필수 처리 |
|---|---|---|---|
| 시작 전 | supervisor 정상 시작 | `STOPPED` | zero 출력 확인 |
| `STOPPED`, `COMPLETED`, `CANCELED` | Nav2 goal accepted | `NAVIGATING` | 새 `command_id`와 선택적 `operation_id` 연결 |
| `NAVIGATING` | Nav2 success | `COMPLETED` | `source=NAV2` |
| `NAVIGATING` | Nav2 rejected 또는 aborted | `STOPPED` | `detail=NAV_REJECTED` 또는 `NAV_ABORTED` |
| `STOPPED`, `COMPLETED` | 비전 상차 작업 accepted | `PICKING` | `source=VISION_PICK` |
| `STOPPED`, `COMPLETED` | 비전 하차 작업 accepted | `PLACING` | `source=VISION_PLACE` |
| `PICKING`, `PLACING` | 비전 작업 success | `COMPLETED` | 작업 source 유지 |
| `PICKING`, `PLACING` | 비전 작업 failed | `STOPPED` | `detail=PICK_FAILED` 또는 `PLACE_FAILED` |
| 자동 작업 상태 | 외부 cancel | `CANCELING` | 작업 취소와 zero 출력 시작 |
| `CANCELING` | 작업 종료와 zero 출력 확인 | `CANCELED` | 취소 command ID, target ID와 attempt 포함 |
| `CANCELING` | 횟수·시간 소진 | `CANCEL_FAILED` | 자동 출력 차단 유지 |
| 자동 또는 수동 작업 상태 | 외부 stop | `CANCELING` | 자동 작업 취소와 safety stop 동시 실행 |
| `CANCELING` | stop 조건 모두 확인 | `STOPPED` | `detail=STOP_CONFIRMED` |
| 활성 작업 없음 | 외부 cancel | `CANCELING` 후 `CANCELED` | zero 출력 확인 후 같은 command ID로 즉시 확인 |
| 모든 상태 | 새 명시적 작업 명령 accepted | 해당 작업 상태 | 이전 terminal 상태 해제 |

`COMPLETED`, `CANCELED`, `CANCEL_FAILED`는 결과 관찰을 위해 다음 명시적 명령이
수락될 때까지 유지한다. 일정 시간 후 자동으로 `STOPPED`로 바꾸지 않는다.

## 수동 개입 규칙

자동 작업 중 수동 cmd_vel이 수신되면 다음 순서로 처리한다.

```text
NAVIGATING / PICKING / PLACING
  -> 중단 상태와 operation_id 저장
  -> mux에서 자동 이동 출력 즉시 차단
  -> 자동 작업 cancel 시작
  -> MANUAL
  -> 수동 deadman timeout 또는 zero 명령
  -> STOPPED
```

수동 개입 시 최상위 상태는 즉시 `MANUAL`이 된다. 내부 자동 취소 진행 여부는
`detail=NAV_CANCEL_PENDING`, `PICK_CANCEL_PENDING` 또는
`PLACE_CANCEL_PENDING`으로 나타낸다. 취소 확인 후에는 detail을
`*_CANCEL_CONFIRMED`로 바꾼다.

자동 작업 취소가 실패하더라도 mux는 해당 자동 출력의 차단을 해제하지 않는다.
수동 입력이 끝나면 `CANCEL_FAILED`로 전이해 운영자 또는 후속 복구 로직의 판단을
요청한다.

정상적으로 자동 작업이 중단된 뒤 수동 입력이 끝나면 다음 값을 유지한 채
`STOPPED`가 된다.

- `previous_state`: 중단된 `NAVIGATING`, `PICKING` 또는 `PLACING`
- `previous_command_id`: 중단된 API 명령 ID
- `operation_id`: 중단된 작업 식별자
- `interrupted_by`: `MANUAL`
- `resume_decision_required`: `true`

이번 범위에서는 이 정보를 보고 자동 재주행하지 않는다. 후속 재개 기능은 현재
위치, 기존 목표, 작업 종류, 장애물과 안전 조건을 다시 평가해 다음 중 하나를
명시적으로 선택해야 한다.

- 기존 목표로 새 Nav2 goal 발행
- 현재 위치 기준으로 재계획
- 상차·하차 비전 정렬을 처음부터 다시 수행
- 기존 작업 종료 또는 운영자 개입 요청

## 포크 유지 규칙

- cancel, stop, 수동 개입 메시지는 포크 제어 토픽에 값을 발행하지 않는다.
- 상차·하차 취소 시 비전 작업 제어기는 현재 포크 위치를 유지한다.
- 자동으로 원점 복귀, 하강 또는 상승하지 않는다.
- 포크 이동은 별도의 명시적 포크 명령이나 승인된 비전 작업 단계에서만 수행한다.
- 후속 재개 로직은 저장된 포크 상태와 물류 감지 결과를 함께 확인해야 한다.

## 취소 재시도와 실패 판정

Fleet Bridge는 다음 알고리즘을 사용한다.

1. API 서버가 취소 명령의 새 `command_id`를 만들고 `attempt=1`을 발행한다.
2. 같은 WebSocket 연결에서 상태 토픽을 구독한다.
3. `CANCEL_NAVIGATION`은 현재 command ID의 `CANCELED`, `STOP`은 현재 command ID의
   `STOPPED`를 받으면 성공으로 종료한다.
4. `cancel_retry_interval_ms` 동안 확인 상태가 없으면 attempt를 증가시켜 같은
   요청을 다시 발행한다.
5. `cancel_max_attempts`까지 발행한 뒤에는
   `cancel_confirmation_timeout_ms` 동안 최종 상태를 기다린다.
6. 제한시간 안에 현재 요청의 성공 상태가 없으면 실패로 종료한다.

`CANCEL_NAVIGATION`의 성공 상태는 `CANCELED`, `STOP`의 성공 상태는
`STOPPED`다. 다른 command ID, 다른 target ID, 오래된 transient-local terminal
상태, 잘못된 JSON은 성공 판정에서 제외한다.

차량은 첫 요청에서 실제 취소를 시작하고 `CANCELING`을 발행한다. 같은 요청의
재전송은 진행 중인 취소 future를 복제하지 않는다. 마지막 attempt를 받은 뒤에도
확인이 끝나지 않으면 설정된 확인 제한시간 후 `CANCEL_FAILED`를 발행한다.

Foxglove 연결 실패, status channel 미광고 또는 CDR 발행 실패는 API에서 503으로
반환한다. 연결은 성공했지만 상태 확인이 실패한 경우에도 성공으로 위장하지 않는다.

## REST API 응답 변경

기존 service response의 `nav2_return_code`와 `nav2_goals_canceling`은 더 이상
신뢰할 수 없으므로 토픽 기반 결과로 교체한다.

```json
{
  "robot_id": "robot_2",
  "command": "nav2_cancel",
  "command_id": "f83a3b7b-30d9-4c38-a887-f5f48d281998",
  "target_command_id": "42dc49df-a591-4298-99e8-e78af80c1089",
  "final_state": "CANCELED",
  "attempts": 2
}
```

일반 명령 요청에는 command UUID를 받지 않는다. 예를 들어 Nav2 goal 요청 body는
기존 PoseStamped JSON과 선택적인 상위 `operation_id`만 받으며, 서버가 생성한
`command_id`를 응답에 추가한다.

```json
{
  "robot_id": "robot_2",
  "command": "nav2_goal_pose",
  "command_id": "42dc49df-a591-4298-99e8-e78af80c1089",
  "delivery_state": "PUBLISHED"
}
```

`PUBLISHED`는 차량 토픽 전달 완료를 뜻할 뿐 Nav2 goal accepted나 목적지 도착을
뜻하지 않는다. 실제 실행 상태는 같은 `command_id`가 포함된 operation status로
확인한다.

cancel과 stop은 body를 생략할 수 있다. 특정 이전 명령만 대상으로 할 때는 서버가
그 명령의 응답으로 반환했던 ID를 전달한다.

```json
{
  "target_command_id": "42dc49df-a591-4298-99e8-e78af80c1089"
}
```

- cancel body가 없거나 `target_command_id=null`이면 현재 활성 자동 명령을
  취소한다.
- stop body가 없거나 `target_command_id=null`이면 해당 로봇의 자동·수동 이동을
  모두 정지한다.
- target이 현재 활성 명령과 일치하면 해당 명령만 취소한다.
- target이 이미 terminal이면 멱등 성공과 `TARGET_ALREADY_TERMINAL`을 반환한다.
- target이 존재하지 않거나 현재 활성 명령과 다르면 다른 명령을 건드리지 않고
  HTTP 409와 `TARGET_COMMAND_NOT_ACTIVE`를 반환한다.
- 클라이언트가 일반 명령 body에 `command_id`를 임의로 넣으면 422로 거부한다.

- 유효 요청을 발행하고 확인 상태를 받으면 기존 API와 동일하게 202를 반환한다.
- 설정 오류나 입력 오류는 4xx로 반환한다.
- 연결·발행·확인 실패와 `CANCEL_FAILED`는 503으로 반환한다.
- target 없는 `/stop`은 cancel 토픽의 `command=STOP`과 global safety stop을 함께
  사용한다. targeted stop은 선택한 명령 source만 중단하며 target 불일치 시 global
  safety stop을 발행하지 않는다.

## Foxglove Bridge 설정

차량 Foxglove Bridge는 cancel/status 토픽만 추가로 허용한다. Nav2 hidden cancel
service를 노출하지 않으므로 현재 type support symbol 오류를 반복하지 않는다.

물리 차량의 비네임스페이스 예시는 다음과 같다.

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
  address:=0.0.0.0 \
  port:=8765 \
  topic_whitelist:='["^/(tf|tf_static|robot_description|joint_states|amcl_pose|odom|ros_robot_controller/battery|operation/status)$"]' \
  client_topic_whitelist:='["^/(navigation/goal|cmd_vel|navigation/cancel|safety/stop)$"]' \
  service_whitelist:='["(?!)"]' \
  param_whitelist:='["(?!)"]' \
  capabilities:='[clientPublish]' \
  include_hidden:=false
```

실제 whitelist는 `fleet.yaml`의 토픽과 일치해야 한다. 다중 로봇 시뮬레이터는
robot namespace를 포함한 정규식을 사용한다.

## 구현 책임 경계

### Fleet Bridge

- 모든 API 명령의 UUID v4 `command_id` 생성과 응답 포함
- 선택적 `target_command_id` request validation과 차량 전달
- 로봇별 active/recent-terminal command registry와 bounded cmd_vel publish loop 관리
- 로봇별 cancel/status 토픽과 재시도 설정 로드·검증
- String JSON CDR 직렬화·역직렬화
- 상태 channel 구독과 command ID·target command ID 기반 확인
- 설정값에 따른 재발행과 최종 timeout
- REST 응답과 실패 코드 변환
- operation status를 telemetry worker 설정에 추가해
  `/{robot_id}/operation/status`로 재발행

### 차량 operation supervisor

- 외부 cancel 요청 검증과 멱등 처리
- 활성·최근 terminal command ID를 제한된 크기로 관리하고 target 일치 여부 확인
- Nav2·비전 취소 요청 조정
- cmd_vel mux의 최종 zero/제어권 확인
- 단일 통합 상태 publisher 역할
- 수동 개입 이전 상태 보존
- 포크 유지 규칙 준수

### Nav2 goal bridge

- String goal command envelope의 command ID와 PoseStamped 데이터를 검증
- 검증한 PoseStamped를 NavigateToPose action으로 전달
- active goal handle과 command ID를 함께 보관
- 자신이 소유한 active goal handle 취소
- cancel future와 action result가 확인되기 전에 성공을 선언하지 않음
- accepted, succeeded, canceled, aborted 내부 이벤트 발행

### cmd_vel mux

- safety stop, manual, 비전 정렬, Nav2 명령의 우선순위 적용
- 수동 개입 시 자동 출력 차단
- 현재 이동 제어권과 zero 출력 내부 이벤트 제공
- 취소 실패 후 자동 출력이 다시 살아나지 않도록 차단 유지

### 비전 작업 제어기

- `PICKING`, `PLACING`, 완료, 실패 내부 이벤트 제공
- 취소 요청을 받으면 자율 접근과 포크 동작 중단
- 별도 포크 명령이 없으면 현재 포크 높이 유지

비전 기반 상차·하차 알고리즘 자체와 중단 후 재개 판단기는 이번 구현 범위 밖이다.
이번 변경은 이들이 나중에 연결할 수 있는 상태와 취소 인터페이스를 정의한다.

## 테스트 전략

### Config 테스트

- 모든 새 설정이 로봇별로 로드된다.
- 누락 시 기본값을 적용한다.
- 타입, topic, 최소·최대 범위를 검증한다.
- robot_1과 robot_2에 서로 다른 retry 정책을 적용할 수 있다.
- 기존 cancel service 설정이 남아 있으면 허용하지 않고 migration 오류를 제공한다.
- goal type이 `std_msgs/msg/String`이 아니면 거부한다.

### Fleet Bridge 단위 테스트

- 일반 API 명령마다 서버가 UUID v4 command ID를 생성해 응답한다.
- 일반 명령 body에 클라이언트 command ID가 있으면 422로 거부한다.
- cancel String CDR payload가 같은 command ID와 증가하는 attempt를 가진다.
- cancel 재시도는 같은 command ID를 사용한다.
- target command ID가 payload와 API 응답에 유지된다.
- target이 active, terminal, unknown인 경우를 각각 검증한다.
- targeted bounded cmd_vel은 해당 publish loop만 중단하고 zero Twist를 보낸다.
- 성공 상태를 받으면 최대 횟수 전에 재시도를 중단한다.
- 다른 command ID·target command ID와 과거 terminal 상태를 무시한다.
- 최대 횟수와 최종 확인시간을 정확히 적용한다.
- 연결, channel 광고, malformed status, `CANCEL_FAILED`를 503으로 변환한다.
- target 없는 `/stop`은 safety stop 경로도 Nav2 취소 실패 여부와 무관하게
  시도한다.
- target 불일치 stop은 global safety stop을 발행하지 않는다.

### 차량 상태 머신 테스트

- goal command ID가 active goal handle과 상태에 연결된다.
- 문서의 모든 상태 전이를 표 기반 단위 테스트로 검증한다.
- 취소 접수만으로 `CANCELED`를 발행하지 않는다.
- 중복 command ID가 병렬 cancel future를 만들지 않는다.
- Nav2 취소 결과와 zero 출력이 모두 있어야 `CANCELED`가 된다.
- timeout 시 `CANCEL_FAILED`가 되고 자동 출력 차단을 유지한다.
- terminal 상태가 다음 명시적 명령 전까지 유지된다.

### 수동 개입 테스트

- `NAVIGATING -> MANUAL`에서 Nav2 출력이 즉시 차단된다.
- `PICKING/PLACING -> MANUAL`에서 비전 작업 취소가 시작된다.
- `previous_state`, `previous_command_id`, `operation_id`,
  `resume_decision_required`가 보존된다.
- 수동 deadman timeout 후 자동 작업이 재개되지 않는다.
- 자동 취소 실패 시 수동 종료 후 `CANCEL_FAILED`가 된다.
- 모든 경로에서 포크 명령을 발행하지 않는다.

### 통합 테스트

- mock Foxglove WebSocket에서 cancel 발행, status 수신, 조기 종료를 검증한다.
- 차량 ROS graph에서 Nav2 주행 중 cancel 토픽을 발행해 실제 action result와 zero
  cmd_vel을 확인한다.
- Nav2 주행 중 수동 cmd_vel을 넣어 자동 출력이 차단되고 상태가 `MANUAL`로
  바뀌는지 확인한다.
- 차량 `/operation/status`가 서버 `/{robot_id}/operation/status`로 재발행되어
  중앙 Foxglove와 rosbag에서 관찰되는지 확인한다.
- 마지막으로 `robot_2`의 REST `/nav2/cancel`과 `/stop`을 실제 차량에서 재시험한다.

## 범위 밖

- `action_msgs` 또는 Foxglove Bridge 재설치·교체
- Foxglove generic service client를 통한 Nav2 cancel
- 비전 인식, 정렬, 포크 상차·하차 알고리즘 구현
- 수동 개입 후 자동 재주행 여부를 판단하는 복구·재계획 로직
- 취소나 정지에 따른 자동 포크 원점 복귀
- Open-RMF task phase와 operation status의 직접 연동

## 완료 기준

1. cancel service를 사용하지 않고 토픽만으로 Nav2 취소를 요청할 수 있다.
2. Fleet Bridge가 설정된 횟수·간격·확인시간에 따라 재시도하고 현재 요청의 상태를
   확인한다.
3. 통합 상태 토픽이 모든 정의 상태와 전이 규칙을 일관되게 제공한다.
4. 수동 개입이 자동 이동 출력을 즉시 차단하며 자동 작업이 스스로 재개되지 않는다.
5. 중단 이전 상태와 작업 ID가 후속 판단을 위해 보존된다.
6. cancel, stop, manual takeover가 포크 높이를 변경하지 않는다.
7. retry 정책은 코드 수정 없이 로봇별 `fleet.yaml` 변경으로 조정할 수 있다.
8. 모든 API 명령 UUID는 서버가 생성하며 일반 명령 요청 body는 UUID를 요구하지
   않는다.
9. 특정 명령 취소는 이전 API 응답의 `target_command_id`가 일치할 때만 실행된다.
