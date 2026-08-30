# Vehicle Communication 운영 가이드

## 목적과 실행 방식

`vehicle_command_api.py`는 이미 차량에서 실행 중인 전역 Nav2 action
`/navigate_to_pose`, 전역 속도 토픽 `/cmd_vel`, Auto Dock ROS topic에 HTTP 요청을
연결한다.

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

차량에는 이 디렉터리 전체를 `/opt/vehicle_communication`으로 배포한다. 차량별 설정은
`runtime.env`에서만 바꾼다.

```dotenv
VEHICLE_ROBOT_ID=robot_2
VEHICLE_COMMAND_API_PORT=8082
FOXGLOVE_PORT=8765
```

### ROS 2 패키지 빌드와 독립 실행

이 디렉터리는 ROS 2 패키지 이름 `vehicle_command_api`로도 설치할 수 있다.
차량 workspace의 `src` 아래에 배치한 뒤, Nav2를 다시 시작하지 않고 HTTP API만
독립 실행한다.

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vehicle_command_api
source install/setup.bash

ros2 run vehicle_command_api vehicle_command_api \
  --robot-id robot_2 \
  --cmd-vel-topic /cmd_vel
```

`ros2 run` 프로세스는 기존 전역 ROS graph의 `/cmd_vel` publisher에만 연결한다.
따라서 이 명령은 Nav2 launch를 다시 실행하지 않는다. 소스를 변경한 경우에는
`colcon build`와 `source install/setup.bash`를 다시 실행한다.

별도 터미널에서 다음으로 cmd_vel HTTP API를 검증한다.

```bash
ros2 topic echo /cmd_vel

curl -i -X POST http://127.0.0.1:8082/v1/cmd-vel \
  -H 'Content-Type: application/json' \
  --data '{"linear_x":0.05,"angular_z":0.0,"hold_ms":500}'
```

성공 기준은 HTTP `202`, 비영(非零) `Twist` 한 건, 500 ms 뒤의 0속도 `Twist` 한 건이다.

실행 스크립트는 ROS 2 setup 파일을 직접 `source`하지 않는다. 실행 전 사용하는
셸에서 `.zshrc`를 통해 ROS 2 환경을 준비해야 한다.

```bash
cd /opt/vehicle_communication
chmod +x tools/*.sh

# 각각 독립 실행 및 정지
./tools/foxglove_start.sh
./tools/foxglove_stop.sh
./tools/command_api_start.sh
./tools/command_api_stop.sh

# 두 서비스를 함께 실행 및 정지
./tools/vehicle_communication_start.sh
./tools/vehicle_communication_stop.sh
```

각 start 스크립트는 `ps` 명령으로 자기 서비스의 실제 명령행을 조회한다. Foxglove는
launch 명령과 `FOXGLOVE_PORT`, Command API는 이 번들의 절대 파일 경로를 함께
비교한다. 이미 실행 중이면 새 프로세스를 만들지 않고 종료한다. stop 스크립트도
같은 기준으로 찾아 `SIGTERM`을 전송한다. 통합 정지 스크립트는 API를 먼저 종료한
뒤 Bridge를 종료한다.

Foxglove Bridge의 허용 topic, QoS, 서비스/파라미터 차단, 압축 설정은
`tools/foxglove_start.sh`에 고정되어 있다. 변경이 필요하면 해당 스크립트를
교체해 배포한다.

로그 파일은 차량 사용자의 `~/log`에 생성된다.

```text
~/log/foxglove_bridge
~/log/vehicle_command_api
```

`0.0.0.0`은 모든 차량 네트워크 인터페이스에서 수신하도록 하는 bind 주소다.
클라이언트 요청에는 실제 차량 IP를 사용한다. 예를 들어 차량 IP가
`192.168.100.20`이면 base URL은 `http://192.168.100.20:8082`다.

`vehicle_command_api.py`를 직접 실행해야 하는 경우에도 아래 인자를 사용할 수 있다.

| 인자 | 기본값 | 의미 |
|---|---:|---|
| `--robot-id` | 없음(필수) | 이 API 인스턴스가 제어하는 차량 식별자 |
| `--battery-topic` | `/ros_robot_controller/battery` | `std_msgs/msg/UInt16` 배터리 원시값 토픽 |
| `--battery-stale-sec` | `3.0` | 이 시간보다 오래된 배터리값을 stale로 표시 (초) |
| `--initial-pose-topic` | `/initialpose` | AMCL 초기 위치 발행 토픽 |
| `--initial-pose-position-variance` | `0.25` | initial pose X/Y covariance 대각값 (m²) |
| `--initial-pose-yaw-variance` | `0.0685` | initial pose yaw covariance 대각값 (rad²) |
| `--auto-dock-arrival-topic` | `/{robot_id}/nav2/arrival` | Auto Dock Pick·Place 시작 JSON 발행 토픽 |
| `--auto-dock-status-topic` | `/{robot_id}/auto_dock/status` | Auto Dock 상태 JSON 구독 토픽 |
| `--auto-dock-stop-topic` | `/{robot_id}/auto_dock/stop` | Auto Dock 중단 `std_msgs/msg/Empty` 발행 토픽 |
| `--auto-dock-drive-ready-topic` | `/{robot_id}/auto_dock/drive_ready` | Auto Dock 실제 완료 `std_msgs/msg/Empty` 구독 토픽 |
| `--max-linear-x` | `0.10` | 수동 전진/후진 최대 속도 (m/s) |
| `--max-angular-z` | `0.50` | 수동 회전 최대 속도 (rad/s) |
| `--max-hold-ms` | `1000` | 수동 속도 유지 최대 시간 (ms) |
| `--action-server-timeout-sec` | `1.0` | Nav2 action server 탐색 대기 시간 |
| `--goal-response-timeout-sec` | `3.0` | goal 수락 응답 대기 시간 |
| `--cancel-response-timeout-sec` | `3.0` | cancel 수락 응답 대기 시간 |

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
한 번 발행하고 명령 전 상태로 복귀한다. `MANUAL`은 물류 작업 상태가 아닌
유지보수 상태이므로 `INIT`에서는 다시 `INIT`으로, `IDLE`에서는 다시 `IDLE`로
돌아간다. 수동 명령을 받기 전에 활성 Nav2 목표가 있으면 해당 목표의 cancel을
먼저 요청한다.

### Nav2 목표 전송

```bash
curl -i -X POST http://192.168.100.20:8082/v1/navigation/goals \
  -H 'Content-Type: application/json' \
  --data '{"operation_id":"73d5b9af-5a12-4f34-a96c-5de116df1e8e","purpose":"PICK","frame_id":"map","x":1.50,"y":0.0,"yaw":0.0}'
```

물류 작업에서는 Inventory/중앙 서버가 만든 `operation_id`를 반드시 보낸다. 응답의
`attempt_id`는 차량 내부 Nav2 action handle 식별자이며 서버 작업 ID와 다르다.
`operation_id`를 생략하면 독립 주행 검증용 UUID를 차량이 만든다. `purpose`는 적재
전 주행에서는 선택 사항이지만 `PICK_COMPLETE`의 적재 상태에서는 반드시 `PLACE`여야
한다. `frame_id`는 생략하면 `map`이며, 다른 frame은 거부한다. 차량은 `INIT`에서
DRIVE를 받지 않으므로 초기 위치 확인 뒤 아래 `operation/idle` 요청을 먼저 수행해야 한다.

### 작업 상태 조회

```bash
curl -sS http://192.168.100.20:8082/v1/operation-status | python3 -m json.tool
```

응답은 현재/직전 서버 작업을 함께 확인할 수 있도록 `operation_id`,
`previous_operation_id`, `state`, `previous_state`, `detail`을 가진다.

| state | 의미 |
|---|---|
| `INIT` | API 기동 직후. 운영자 확인 전에는 물류 작업을 수락하지 않음 |
| `IDLE` | 적재·진행 명령이 없는 작업 가능 상태 |
| `DRIVE` | Nav2 목표 주행 중 |
| `PICKING` | Auto Dock Pick 작업 중 |
| `PICK_COMPLETE` | Pick 완료 뒤 차량이 적재된 상태 |
| `PLACE` | Auto Dock Place 작업 중 |
| `PLACE_COMPLETE` | Place 완료 이벤트 직후의 내부 종료 상태. 스냅샷은 바로 `IDLE`로 전환 |
| `FAILED` | Nav2 또는 Auto Dock 오류. 운영자 해제 전 새 작업 불가 |
| `CANCELLED` | cancel 또는 stop으로 중단됨. 운영자 해제 전 새 작업 불가 |
| `MANUAL` | 제한 시간 직접 `/cmd_vel` 유지보수 명령. 만료 시 명령 전 상태로 복귀 |

### 기동·복구 후 작업 허가

`/v1/localization/initial-pose`는 AMCL에 위치 후보를 발행할 뿐 작업을 허가하지
않는다. API 기동, `CANCELLED`, `FAILED` 뒤에는 현장 위치와 안전 상태를 확인한 후
다음 요청으로만 `IDLE`로 전환한다.

```bash
curl -i -X POST http://192.168.100.20:8082/v1/operation/idle \
  -H 'Content-Type: application/json' \
  --data '{"reason":"OPERATOR_CONFIRMED"}'
```

`PICK_COMPLETE`에서는 적재 상태를 잃지 않도록 이 요청을 거부한다. 재기동이나
실패 뒤에는 차량이 과거 상태를 복원하지 않는다. 중앙 서버가 Inventory 이력과
`previous_operation_id`를 대조한 뒤 동일한 `operation_id`로 DRIVE 또는 Auto Dock
명령을 새로 보낸다.

### Auto Dock Pick·Place 전송

Nav2 목표 도착을 서버가 확인한 다음에만 Auto Dock 명령을 별도로 보낸다.

```bash
curl -i -X POST http://192.168.100.20:8082/v1/auto-dock \
  -H 'Content-Type: application/json' \
  --data '{"operation_id":"73d5b9af-5a12-4f34-a96c-5de116df1e8e","operation":"PICK","product_type":"NORMAL","location":"DOCK_1","target":{"type":"NEAREST"}}'
```

`PICK`은 `IDLE`, `PLACE`는 동일한 작업 ID의 `PICK_COMPLETE`에서만 수락한다. API는
위 JSON에 `status: "SUCCEEDED"`를 추가하여 `/{robot_id}/nav2/arrival`에 발행한다.
Auto Dock의 `READY` status는 완료가 아니다. fork, 후진, 준비 자세까지 끝난
`/{robot_id}/auto_dock/drive_ready`만 `PICK_COMPLETE` 또는 `PLACE_COMPLETE`으로
판정한다. `PLACE_COMPLETE` 뒤에는 자동으로 `IDLE` snapshot으로 전환한다.

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
    "previous_operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
    "state": "IDLE",
    "previous_state": "CANCELLED",
    "detail": "OPERATOR_READY"
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

`DRIVE`, `PICKING`, `PLACE`, 또는 `MANUAL` 상태에서는 위치 재설정이 위험하므로
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

`cancel`은 지정된 활성 Nav2 작업의 영구 취소를 요청하고 상태를 `CANCELLED`로
바꾼다. `stop`은 먼저 `/cmd_vel`에 0속도를 발행하고 활성 Nav2 또는 Auto Dock
작업의 cancel/stop topic을 함께 요청한다. 둘 다 자동으로 `IDLE`로 돌아가지 않으며
원인 조치와 `operation/idle` 운영자 확인 뒤에만 다음 작업을 받을 수 있다.

## 안전 경계와 Reporter 분리

`stop`은 Nav2 cancel과 Auto Dock stop topic을 요청하고 0속도 명령을 발행하지만,
하드웨어 수준 stop latch 또는 다른 publisher의 속도 명령 차단 기능은 제공하지
않는다. 작업 완료·실패 이벤트를 중앙 서버에 재전송하는 기능도 이 패키지의
책임이 아니다. 그 기능은 Nav2와 Auto Dock 양쪽에서 재사용할 별도
`vehicle_task_reporter` 패키지에서 상태 스냅샷을 읽어 구현한다.

## 로컬 검증

ROS를 설치하지 않은 개발 환경에서도 HTTP 계약을 검증할 수 있다.

```bash
cd /opt/vehicle_communication
python3 -m unittest discover -s test -v
python3 -m py_compile vehicle_command_api.py
```
