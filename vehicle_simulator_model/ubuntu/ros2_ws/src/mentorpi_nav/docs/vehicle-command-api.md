# Vehicle Command API 운영 가이드

## 목적과 실행 방식

`vehicle_command_api.py`는 이미 차량에서 실행 중인 전역 Nav2 action
`/navigate_to_pose`와 전역 속도 토픽 `/cmd_vel`에 HTTP 요청을 연결한다.

이 프로세스는 Nav2를 새로 실행하지 않는다. 따라서 실차에서 다음처럼 전역
`/bt_navigator`, `/planner_server`, `/controller_server`가 이미 보이는 경우에
사용한다.

```bash
ros2 node list --no-daemon | grep -E 'bt_navigator|planner_server|controller_server'
ros2 action list -t | grep '/navigate_to_pose'
```

`vehicle_navigation.launch.py`는 `robot_id` namespace 안에 별도 Nav2 node를
시작하는 simulator용 launch이므로, 현재 전역 Nav2가 동작 중인 차량에서는 이
API를 검증하기 위해 실행하지 않는다.

## 실행

차량의 `~/ros2_ws`에서 ROS 환경을 로드한 뒤 source script를 직접 실행한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.zsh

python3 src/mentorpi_nav/scripts/vehicle_command_api.py \
  --host 0.0.0.0 \
  --port 8082 \
  --robot-id robot_2 \
  --cmd-vel-topic /cmd_vel \
  --action-name /navigate_to_pose
```

`0.0.0.0`은 모든 차량 네트워크 인터페이스에서 수신하도록 하는 bind 주소다.
클라이언트 요청에는 실제 차량 IP를 사용한다. 예를 들어 차량 IP가
`192.168.100.20`이면 base URL은 `http://192.168.100.20:8082`다.

설정값은 실행 인자로 바꾼다.

| 인자 | 기본값 | 의미 |
|---|---:|---|
| `--robot-id` | 없음(필수) | 이 API 인스턴스가 제어하는 차량 식별자 |
| `--battery-topic` | `/ros_robot_controller/battery` | `std_msgs/msg/UInt16` 배터리 원시값 토픽 |
| `--battery-stale-sec` | `3.0` | 이 시간보다 오래된 배터리값을 stale로 표시 (초) |
| `--initial-pose-topic` | `/initialpose` | AMCL 초기 위치 발행 토픽 |
| `--initial-pose-position-variance` | `0.25` | initial pose X/Y covariance 대각값 (m²) |
| `--initial-pose-yaw-variance` | `0.0685` | initial pose yaw covariance 대각값 (rad²) |
| `--max-linear-x` | `0.10` | 수동 전진/후진 최대 속도 (m/s) |
| `--max-angular-z` | `0.50` | 수동 회전 최대 속도 (rad/s) |
| `--max-hold-ms` | `1000` | 수동 속도 유지 최대 시간 (ms) |
| `--action-server-timeout-sec` | `1.0` | Nav2 action server 탐색 대기 시간 |
| `--goal-response-timeout-sec` | `3.0` | goal 수락 응답 대기 시간 |
| `--cancel-response-timeout-sec` | `3.0` | cancel 수락 응답 대기 시간 |

빌드된 패키지에서 실행하려면 먼저 설치한다.

```bash
cd ~/ros2_ws
colcon build --packages-select mentorpi_nav --symlink-install
source install/setup.zsh
ros2 run mentorpi_nav vehicle_command_api.py \
  --host 0.0.0.0 \
  --port 8082 \
  --robot-id robot_2
```

## API 확인과 테스트

### Health와 OpenAPI

```bash
curl -sS http://192.168.100.20:8082/healthz
curl -sS http://192.168.100.20:8082/openapi.json | python3 -m json.tool
```

`/openapi.json`은 OpenAPI 3.0.3 문서다. HTTP client 생성이나 연동 시 이
문서를 계약으로 사용한다.

### 수동 속도 명령

```bash
curl -i -X POST http://192.168.100.20:8082/v1/cmd-vel \
  -H 'Content-Type: application/json' \
  --data '{"linear_x": 0.05, "angular_z": 0.0, "hold_ms": 500}'
```

성공하면 `202`와 `MANUAL` 상태를 반환한다. hold 시간이 지나면 API는 0속도를
한 번 발행하고 상태를 `IDLE`로 바꾼다. 수동 명령을 받기 전에 활성 Nav2 목표가
있으면 해당 목표의 cancel을 먼저 요청한다.

### Nav2 목표 전송

```bash
curl -i -X POST http://192.168.100.20:8082/v1/navigation/goals \
  -H 'Content-Type: application/json' \
  --data '{"frame_id": "map", "x": 1.50, "y": 0.0, "yaw": 0.0}'
```

성공 시 `202`와 차량이 만든 `operation_id`를 반환한다. `operation_id`는 이후
상태 조회 및 취소 대상을 지정할 때 사용한다. `frame_id`는 생략하면 `map`이며,
다른 frame은 거부한다.

### 작업 상태 조회

```bash
curl -sS http://192.168.100.20:8082/v1/operation-status | python3 -m json.tool
```

응답은 `operation_id`, `state`, `detail`을 가진다.

| state | 의미 |
|---|---|
| `IDLE` | 실행 중인 명령 없음 |
| `MANUAL` | 제한 시간 안의 직접 `/cmd_vel` 명령 |
| `NAVIGATING` | Nav2가 목표를 수락해 주행 중 |
| `CANCELLING` | Nav2 cancel 결과 대기 중 |
| `CANCELLED` | 지정 cancel 후 Nav2가 취소 결과를 보고함 |
| `COMPLETED` | Nav2 goal 성공 |
| `FAILED` | goal 거절, Nav2 미가용, abort 또는 action 오류 |
| `STOPPED` | `/v1/stop`이 즉시 0속도를 발행함 |

### 차량 상태 조회

```bash
curl -sS http://192.168.100.20:8082/v1/vehicle-status | python3 -m json.tool
```

`robot_id`는 실행 시 `--robot-id`로 지정한 값을 그대로 반환한다. `battery`는
`/ros_robot_controller/battery`에서 받은 `UInt16` 원시값이며, 메시지 단위가
확인되기 전까지 전압이나 퍼센트로 변환하지 않는다. 아직 받지 못했거나
`--battery-stale-sec`보다 오래된 값은 `stale: true`로 표시한다.

```json
{
  "robot_id": "robot_2",
  "battery": {
    "raw_value": 8354,
    "received_at": "2026-08-26T06:52:31.420Z",
    "stale": false
  },
  "operation": {
    "operation_id": null,
    "state": "IDLE",
    "detail": "READY"
  }
}
```

### AMCL 초기 위치 설정

```bash
curl -i -X POST http://192.168.100.20:8082/v1/localization/initial-pose \
  -H 'Content-Type: application/json' \
  --data '{"x": 1.50, "y": 0.0, "yaw": 0.0}'
```

성공하면 전역 `/initialpose`에 `geometry_msgs/msg/PoseWithCovarianceStamped`를
한 번 발행한다. `frame_id`는 생략 시 `map`이며, 다른 frame은 거부한다.
응답의 `INITIAL_POSE_PUBLISHED`는 AMCL이 메시지를 받도록 발행했다는 뜻이며,
위치 추정이 수렴했다는 보장은 아니다.

```json
{
  "operation_id": "a5fbf2ae-30d3-482b-bc64-fde849155349",
  "state": "INITIAL_POSE_PUBLISHED",
  "frame_id": "map",
  "x": 1.5,
  "y": 0.0,
  "yaw": 0.0
}
```

`NAVIGATING`, `CANCELLING`, 또는 `MANUAL` 상태에서는 위치 재설정이 위험하므로
발행하지 않고 `409`을 반환한다. 호출자는 먼저 `/v1/stop`을 직접 호출하고,
Nav2의 취소 처리가 끝난 뒤 initial pose 요청을 다시 수행해야 한다.

```json
{
  "error": "VEHICLE_MOTION_ACTIVE"
}
```

### 지정 취소와 즉시 정지

```bash
operation_id='응답에서_받은_UUID'

curl -i -X POST http://192.168.100.20:8082/v1/navigation/cancel \
  -H 'Content-Type: application/json' \
  --data "{\"operation_id\": \"${operation_id}\"}"

curl -i -X POST http://192.168.100.20:8082/v1/stop
```

`cancel`은 지정된 활성 Nav2 작업의 영구 취소를 요청한다. Nav2가 canceled
결과를 전달하면 상태는 `CANCELLED`가 된다. `stop`은 먼저 `/cmd_vel`에 0속도를
발행하고 활성 작업의 cancel을 함께 요청하며, 즉시 `STOPPED`를 반환한다.

## 현재 단계의 안전 경계

이 1차 독립 API는 현재 배선된 `/cmd_vel` 경로에 0속도 메시지를 발행하고 Nav2
cancel을 요청한다. 기존 Nav2 또는 다른 publisher가 이후 새 속도 메시지를
발행하지 못하게 하는 하드웨어 수준 stop latch는 만들지 않는다. stop latch,
자동 재개 금지, Fleet Manager 상태 보고와 재전송 정책은 다음 단계에서 기존
`cmd_vel` mux/중재 경로 및 Fleet Manager API와 함께 적용한다.

## 로컬 검증

ROS를 설치하지 않은 개발 환경에서도 HTTP 계약을 검증할 수 있다.

```bash
cd ~/ros2_ws/src/mentorpi_nav/test
python3 -m unittest test_vehicle_command_api test_cmd_vel_http_test
python3 -m py_compile ../scripts/vehicle_command_api.py
```
