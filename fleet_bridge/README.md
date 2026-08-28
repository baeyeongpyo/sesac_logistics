# Fleet Foxglove Server Bridge

이 번들은 서버에서만 실행한다. 각 차량의 Foxglove Bridge WebSocket은 telemetry를
ROS 2 Domain 225의 차량별 `/{robot_id}/*` topic으로 재발행하는 데만 사용한다.
명령 API는 차량별 `vehicle_command_api` HTTP endpoint로 `cmd_vel`, Nav2 goal/cancel,
초기 위치, 상태 조회, `stop`을 전달하고 Swagger UI를 제공한다.

```text
robot_1 Foxglove Bridge :8766          server / Humble / Domain 225
  /odom, /tf, /amcl_pose  ────────>  worker-robot-1 -> /robot_1/*
robot_1 vehicle_command_api :8082  <── Command API :8080
  /v1/cmd-vel, /v1/navigation/*,
  /v1/stop

robot_2 Foxglove Bridge :8766          server Foxglove Bridge :8765
  /odom, /tf, /amcl_pose  ────────>  worker-robot-2 -> /robot_2/*
```

차량용 Docker 이미지, Compose 파일, ROS topic filter는 이 저장소에 포함하지 않는다.
차량 주행 컨테이너 또는 차량 운영 환경에서 Foxglove Bridge와
`vehicle_command_api`를 실행해야 한다. 차량 Bridge는 서버가 연결할 `:8766` endpoint로
telemetry 원본 topic을 노출하고, 차량 API는 `:8082`에서 명령을 수신한다.

차량 Foxglove Bridge는 telemetry 관측 전용으로 운용한다. 명령 토픽 publish, service
호출 capability는 Fleet Manager 명령 경로에 필요하지 않다. 정규식 문법과 파라미터
이름은 차량에 설치한 Bridge 버전에서 다시 확인한다.

```yaml
foxglove_bridge:
  ros__parameters:
    capabilities: []
```

## 서버 설정

[`config/fleet.yaml`](config/fleet.yaml)은 차량 ID, telemetry용 Foxglove URI, 차량 명령
API URL, 안전한 속도 명령 상한을 정의한다. `id`가 server ROS topic prefix가 되므로
`robot_1`은 `/robot_1/*`,
`robot_2`는 `/robot_2/*`로 분리된다.

```dotenv
SERVER_ROS_DOMAIN_ID=225
ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766
ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766
ROBOT_1_COMMAND_API_URL=http://192.168.10.215:8082
ROBOT_2_COMMAND_API_URL=http://192.168.10.216:8082
COMMAND_API_HOST=127.0.0.1
COMMAND_API_PORT=8080
```

`.env.example`을 서버용 env 파일로 복사한 뒤 차량 IP를 수정한다.

```bash
cp fleet_bridge/.env.example fleet_bridge/.env.server
```

서버 ROS 2 Domain은 225만 사용한다. 차량의 ROS domain ID, Docker network/IPC,
Fast DDS 설정은 차량 Bridge를 운영하는 쪽에서 관리하며 이 번들의 설정 대상이 아니다.

## 서버 이미지와 실행

저장소 루트에서 서버 이미지를 빌드한다.

```bash
docker build \
  -f fleet_bridge/server/Dockerfile \
  -t mentorpi-fleet-bridge-server:humble \
  fleet_bridge
```

이미지는 ROS 2 Humble/Jammy와 서버 관제용 Foxglove Bridge 0.8.5 commit
`41f96cc6053632a472d9a821989952771b1117f2`를 사용한다. 차량 telemetry worker는
차량 Bridge의 `foxglove.sdk.v1`을 우선 요청하고, 기존
`foxglove.websocket.v1` Bridge는 fallback으로 유지한다. Command API는 Foxglove
WebSocket을 열지 않고 차량 HTTP API만 호출한다.

Docker Compose plugin이 있는 서버에서는 다음을 실행한다.

```bash
docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml \
  up -d
```

Compose plugin이 없는 Linux 서버에서는 명령 API만 직접 실행할 수 있다.

```bash
docker run -d --name fleet-command-api --restart unless-stopped \
  --network host \
  -v "$(pwd)/fleet_bridge/config/fleet.yaml:/config/fleet.yaml:ro" \
  -e ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766 \
  -e ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766 \
  -e ROBOT_1_COMMAND_API_URL=http://192.168.10.215:8082 \
  -e ROBOT_2_COMMAND_API_URL=http://192.168.10.216:8082 \
  -e COMMAND_API_HOST=0.0.0.0 \
  -e COMMAND_API_PORT=8080 \
  mentorpi-fleet-bridge-server:humble \
  ros2 run foxglove_ros_worker fleet_command_api
```

서버 관제에는 Foxglove 앱에서 `ws://<server-ip>:8765` 하나만 연결한다. 이 endpoint는
`/robot_1/*`, `/robot_2/*`, `/map`만 제공하며 observation-only이다. 서버에서
topic publish, service 호출, parameter 변경은 허용하지 않는다.

## telemetry mapping

[`config/telemetry.yaml`](config/telemetry.yaml)은 server worker가 구독할 차량 원본
topic과 서버 재발행 topic의 단일 설정이다.

- `enabled`: 해당 topic을 서버 worker가 구독할지 결정한다.
- `source`: 차량 Foxglove Bridge가 advertise하는 원본 topic이다.
- `target`: 서버 Domain 225에 재발행할 topic이다. `/{robot}` template은 차량 ID로
  확장된다.
- TF와 telemetry 메시지의 `frame_id` 및 `child_frame_id`는 중계 시 차량 ID를 접두사로
  붙인다. 예를 들어 차량의 `base_footprint`는 `robot_1/base_footprint`가 된다. 공용
  지도 좌표계인 `map`은 접두사를 붙이지 않아 두 차량이 같은 지도에서 표현된다.
- `type`: 양쪽에서 확인할 ROS message type이다. type이 다르면 구독하지 않는다.
- `worker_rate.max_rate_hz`: WebSocket 수신 뒤 서버 ROS topic으로 재발행하는 최대
  빈도다. 차량에서 worker로 전송되는 원본 대역폭 자체는 줄이지 않으므로, 차량 측
  프레임레이트·압축 정책은 차량 Bridge 운영 환경에서 별도로 적용한다.
- `qos`: 서버 publisher의 reliability, durability, history, depth다.
- `paired_with`: image와 해당 `CameraInfo`를 한 쌍으로 선언한다. 쌍은 서로를 가리키며
  `enabled` 상태도 같아야 한다.
- `replay_rate_hz`: 수신한 마지막 메시지를 캐시하고 지정 주기로 다시 발행한다. 정적
  metadata가 늦게 연결한 Foxglove에서도 보이도록 RGB/depth `CameraInfo`에 `1.0`을 쓴다.

서버는 각 활성 topic의 `source`를 한 번만 받을 수 있으므로 같은 차량 설정에서 source를
중복해서 등록하면 시작 전에 거부한다. `target`도 중복할 수 없다.

CameraInfo를 worker가 한 번이라도 수신한 뒤에만 캐시 재발행이 가능하다. 차량 Bridge가
worker 연결 이전에 CameraInfo를 한 번만 발행했다면 서버가 과거 메시지를 복원할 수 없다.
이 경우 차량 측 camera driver 또는 차량 Foxglove Bridge에서 해당 CameraInfo를 주기 발행
또는 transient-local로 제공해야 한다.

### 관제 토픽과 중앙 map의 소유권

`telemetry.yaml`의 모든 항목은 차량 Foxglove Bridge에서 worker로 수신하는 차량 원본이다.
[`config/tmp/vehicle_node_topic`](config/tmp/vehicle_node_topic)에 기록한 76개 topic과 type을
권위 목록으로 사용하며 현재는 모두 `enabled: true`다. worker는 경로를 정규화하거나 별칭으로
바꾸지 않고 차량 ID만 접두사로 붙인다. 예를 들어 `/odom`은 `/robot_1/odom`,
`/goal_pose`(`geometry_msgs/msg/PoseStamped`)는 `/robot_1/goal_pose`,
`/ros_robot_controller/battery`(`std_msgs/msg/UInt16`)는
`/robot_1/ros_robot_controller/battery`가 된다. 따라서 서로 다른 차량의 같은 원본 topic은
서버 Domain 225에서 충돌하지 않는다.

RGB와 depth image의 중앙 ROS 재발행은 각각 최대 5 Hz, `scan_filtered`는 최대 2 Hz,
배터리는 최대 0.2 Hz로 제한한다. 이 제한은 worker가 WebSocket으로 메시지를 받은 뒤에
적용되므로 차량에서 서버로 들어오는 원본 네트워크 대역폭은 줄이지 않는다.

반면 [`config/central_topics.yaml`](config/central_topics.yaml)의
`/controller_server/map`은 차량에서 오지 않는다. 중앙 map-server가 자체 발행하는 원본이며,
`central-topic-republisher`가 수신할 때만 `/map`에 재발행한다. `/map` publisher는
`reliable`, `transient_local`, `keep_last(1)` QoS로 마지막 지도 하나를 보존하므로, 반복
발행 없이 늦게 연결한 구독자도 마지막 지도를 받을 수 있다. source와 target을 분리해 자기
자신을 다시 구독하는 loop를 막고, Foxglove는 `/map`을 사용한다.
차량 Nav2의 `/map`은 별도 소유권을 가지며 각각 `/robot_1/map`, `/robot_2/map`으로
중계되므로 중앙 `/map`과 충돌하지 않는다.

중앙 Foxglove(`ws://<server-ip>:8765`)는 `/robot_1/*`, `/robot_2/*`, `/map`을
관측용으로 노출한다. 예를 들어 RGB 영상은
`/robot_1/ascamera/camera_publisher/rgb0/image`, depth 영상은
`/robot_1/ascamera/camera_publisher/depth0/image_raw`이다. 중앙 Bridge의
`clientPublish`, service, parameter 기능은 계속 비활성화되어 있으므로 이 endpoint에서
차량 명령을 발행할 수는 없다.

Nav2의 plan, local plan, costmap, behavior tree log, `/goal_pose`도 권위 목록에 포함되어
차량별 namespace로 중계된다. 현재 스냅샷에 없는 hidden `/_action/*` topic은 이 설정에
추측해서 추가하지 않는다. NavigateToPose action 상태까지 필요하면 차량에서
`ros2 topic list -t --include-hidden-topics`로 다시 수집한 뒤 같은 source/type/target 계약에
추가한다.

이 중계는 관측 경로다. Command API가 차량 `/goal_pose`에 발행한 목표도 다시
`/{robot}/goal_pose`로 관측될 수 있지만, 서버의 namespaced telemetry topic에 발행하는
것만으로 차량 Nav2 명령이 전달되지는 않는다.

```yaml
- id: scan_filtered
  enabled: true
  source: /scan_filtered
  target: /{robot}/scan_filtered
  type: sensor_msgs/msg/LaserScan
  worker_rate:
    max_rate_hz: 2.0
  qos:
    reliability: best_effort
    durability: volatile
    history: keep_last
    depth: 1
```

서버 이미지는 이 목록을 해석하는 Nav2, DWB, lifecycle, map 메시지 패키지와 MentorPi의
`ros_robot_controller_msgs`를 포함한다. 커스텀 인터페이스는 Dockerfile에 고정한 MentorPi
커밋에서 빌드한다. 차량 측 topic rate 또는 message type이 바뀌면 권위 목록과
`telemetry.yaml`을 함께 갱신한다.

## 테스트 Command API와 Swagger

`command-api`는 FastAPI 기반이다. Swagger UI는
`http://<server-ip>:8080/docs`에서 연다. 기본 `COMMAND_API_HOST=127.0.0.1`은
서버 자신의 Fleet Manager만 호출할 수 있게 한다. 다른 장비에서 Swagger를 열려면
`COMMAND_API_HOST=0.0.0.0`으로 명시적으로 변경하고 방화벽을 먼저 설정한다.

Fleet Manager는 차량 HTTP API로 자동 폴백하지 않는다. 차량 API 연결 실패는 HTTP 503,
차량 API가 반환한 409/422/503 등의 상태는 그대로 Fleet Manager 응답으로 전달한다.
따라서 중복 명령 전송 없이 실패 원인을 호출자에게 확인시킬 수 있다.

### 차량 API 전체 중계

차량 `vehicle_command_api`의 모든 공개 경로는 아래 짧은 Fleet Manager 경로로도 사용할 수
있다. `{robot_id}`는 `config/fleet.yaml`에 등록된 차량을 선택하며, Fleet Manager는 해당
차량의 `command_api_url`로만 요청을 전달한다. 이 경로는 차량 API의 요청 본문, HTTP 상태
코드, JSON 응답 본문을 바꾸지 않는다. 따라서 차량 API의 OpenAPI 문서에서 정의한
차량-native 요청 형식을 그대로 사용한다.

| Fleet Manager 경로 | 차량 API 경로 |
| --- | --- |
| `GET /api/v1/vehicle-command/{robot_id}/healthz` | `GET /healthz` |
| `GET /api/v1/vehicle-command/{robot_id}/openapi.json` | `GET /openapi.json` |
| `GET /api/v1/vehicle-command/{robot_id}/operation-status` | `GET /v1/operation-status` |
| `GET /api/v1/vehicle-command/{robot_id}/vehicle-status` | `GET /v1/vehicle-status` |
| `POST /api/v1/vehicle-command/{robot_id}/cmd-vel` | `POST /v1/cmd-vel` |
| `POST /api/v1/vehicle-command/{robot_id}/navigation/goals` | `POST /v1/navigation/goals` |
| `POST /api/v1/vehicle-command/{robot_id}/navigation/cancel` | `POST /v1/navigation/cancel` |
| `POST /api/v1/vehicle-command/{robot_id}/localization/initial-pose` | `POST /v1/localization/initial-pose` |
| `POST /api/v1/vehicle-command/{robot_id}/stop` | `POST /v1/stop` |

예를 들어 차량-native Nav2 목표와 AMCL 초기 위치 요청은 다음과 같다. 주행 중
`initial-pose`가 차량에서 `409 {"error":"VEHICLE_MOTION_ACTIVE"}`로 거부되면 이 응답도
Fleet Manager에서 동일하게 반환된다. 호출자는 먼저 별도의 `stop`을 성공시킨 뒤 초기 위치를
다시 요청해야 한다.

```bash
curl -X POST http://127.0.0.1:8080/api/v1/vehicle-command/robot_1/navigation/goals \
  -H 'content-type: application/json' \
  -d '{"x":1.0,"y":2.0,"yaw":0.0}'

curl -X POST http://127.0.0.1:8080/api/v1/vehicle-command/robot_1/localization/initial-pose \
  -H 'content-type: application/json' \
  -d '{"x":1.0,"y":2.0,"yaw":0.0}'

curl http://127.0.0.1:8080/api/v1/vehicle-command/robot_1/vehicle-status
```

## 상태와 네트워크 확인

worker는 차량별로 독립 재접속하며 1초에서 최대 30초까지 backoff한다. 상태는
`/{robot}/fleet_bridge/status`에 1 Hz JSON `std_msgs/msg/String`으로 발행한다.
정상 수신이 기본 10초를 넘기면 `state`가 `stale`이 된다.

```bash
docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml logs -f worker-robot-1

docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml exec worker-robot-1 \
  bash -lc 'ros2 topic echo /robot_1/fleet_bridge/status'
```

기존 Domain Bridge와 server worker가 같은 target을 동시에 발행하면 중복 data가 생긴다.
전환 시 기존 Domain Bridge를 먼저 중지한다. 네트워크 지연 A/B 비교는 최소 100회 ping,
`docker stats`, 서버의 `ros2 topic hz /robot_1/odom`를 함께 기록한다.

```bash
ping -c 100 <robot_1-ip>
docker stats --no-stream
```

## 정적 검증

```bash
PYTHONPATH=fleet_bridge/common/fleet_bridge_config \
  python3 -m unittest discover \
  -s fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v

PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker \
  python3 -m unittest discover \
  -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_*.py' -v

python3 -m unittest discover -s fleet_bridge/test -p 'test_*.py' -v

docker compose --env-file fleet_bridge/.env.example \
  -f fleet_bridge/docker-compose.server.yaml config --quiet
```
