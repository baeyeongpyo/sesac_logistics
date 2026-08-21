# Fleet Foxglove Server Bridge

이 번들은 서버에서만 실행한다. 각 차량이 제공하는 Foxglove Bridge WebSocket을
수신해 ROS 2 Domain 225의 차량별 `/{robot_id}/*` topic으로 재발행하고, 시험용
`cmd_vel`·`stop` REST API와 Swagger UI를 제공한다.

```text
robot_1 Foxglove Bridge :8766          server / Humble / Domain 225
  /odom, /tf, /amcl_pose  ────────>  worker-robot-1 -> /robot_1/*
  /cmd_vel  <─────────────────────  Command API :8080

robot_2 Foxglove Bridge :8766          server Foxglove Bridge :8765
  /odom, /tf, /amcl_pose  ────────>  worker-robot-2 -> /robot_2/*
```

차량용 Docker 이미지, Compose 파일, ROS topic filter는 이 저장소에 포함하지 않는다.
차량 주행 컨테이너 또는 차량 운영 환경에서 Foxglove Bridge를 직접 실행해야 한다.
차량 Bridge는 서버가 연결할 `:8766` endpoint를 제공하고, telemetry 원본 topic을
노출해야 한다. 원격 명령을 사용할 경우 `clientPublish`는 `/cmd_vel`만 허용해야 한다.

## 서버 설정

[`config/fleet.yaml`](config/fleet.yaml)은 차량 ID, Foxglove URI, 안전한 명령 상한을
정의한다. `id`가 server ROS topic prefix가 되므로 `robot_1`은 `/robot_1/*`,
`robot_2`는 `/robot_2/*`로 분리된다.

```dotenv
SERVER_ROS_DOMAIN_ID=225
ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766
ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766
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
`41f96cc6053632a472d9a821989952771b1117f2`를 사용한다. 차량에 접속하는 server
worker와 Command API는 차량 Bridge의 `foxglove.sdk.v1`을 우선 요청하고, 기존
`foxglove.websocket.v1` Bridge는 fallback으로 유지한다. 따라서 차량 Bridge와
서버 관제용 Bridge의 protocol 버전은 서로 독립적으로 관리한다.

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
  -e COMMAND_API_HOST=0.0.0.0 \
  -e COMMAND_API_PORT=8080 \
  mentorpi-fleet-bridge-server:humble \
  ros2 run foxglove_ros_worker fleet_command_api
```

서버 관제에는 Foxglove 앱에서 `ws://<server-ip>:8765` 하나만 연결한다. 이 endpoint는
`/robot_1/*`, `/robot_2/*`, `/fleet/*`만 제공하며 observation-only이다. 서버에서
topic publish, service 호출, parameter 변경은 허용하지 않는다.

### 차량 Bridge protocol probe

차량 Bridge의 WebSocket protocol을 확인할 때는 컨테이너 내부에 편집기를 설치하거나
긴 Python 코드를 붙여 넣을 필요가 없다. 서버 호스트의 probe 스크립트를 표준입력으로
`command-api` 컨테이너에 전달한다. 아래 명령은 연결 및 `serverInfo`만 확인하며 ROS
명령을 발행하지 않는다.

```bash
docker compose -f docker-compose.server.yaml exec -T command-api \
  python3 - < tools/foxglove_sdk_probe.py
```

`foxglove.sdk.v1`, `clientPublish`, `json`을 지원하는 Bridge에만 정지 명령을 시험할
때는 아래처럼 명시적으로 옵션을 준다. 이 명령은 `/cmd_vel`에 zero `Twist` 하나를
발행하므로 차량이 움직이는 중이라면 정지한다.

```bash
docker compose -f docker-compose.server.yaml exec -T command-api \
  python3 - --send-zero-cmd-vel < tools/foxglove_sdk_probe.py
```

기본 URI는 `command-api` 컨테이너의 `ROBOT_2_FOXGLOVE_URI`다. 다른 endpoint를
확인하려면 다음처럼 `--uri`를 준다.

```bash
docker compose -f docker-compose.server.yaml exec -T command-api \
  python3 - --uri ws://<robot-ip>:<port> < tools/foxglove_sdk_probe.py
```

## telemetry mapping

[`config/telemetry.yaml`](config/telemetry.yaml)은 server worker가 구독할 차량 원본
topic과 서버 재발행 topic의 단일 설정이다.

- `enabled`: 해당 topic을 서버 worker가 구독할지 결정한다.
- `source`: 차량 Foxglove Bridge가 advertise하는 원본 topic이다.
- `target`: 서버 Domain 225에 재발행할 topic이다. `/{robot}` template은 차량 ID로
  확장된다.
- `type`: 양쪽에서 확인할 ROS message type이다. type이 다르면 구독하지 않는다.
- `worker_rate.max_rate_hz`: WebSocket 수신 뒤 서버 ROS topic으로 재발행하는 최대
  빈도다.
- `qos`: 서버 publisher의 reliability, durability, history, depth다.

서버는 각 활성 topic의 `source`를 한 번만 받을 수 있으므로 같은 차량 설정에서 source를
중복해서 등록하면 시작 전에 거부한다. `target`도 중복할 수 없다.

### 관제 토픽과 중앙 map의 소유권

`telemetry.yaml`의 모든 항목은 차량 Foxglove Bridge에서 worker로 수신하는 차량 원본이다.
`enabled: true`이면 worker 중계와 rosbag 기록에 함께 포함되고, `false`이면 둘 다
제외된다. raw depth도 이 설정에서는 활성화되어 있으므로 central ROS Domain과 rosbag에는
들어간다. `battery`는 활성 상태 토픽이며 worker가 최대 0.2 Hz로 재발행한다.

반면 [`config/central_topics.yaml`](config/central_topics.yaml)의
`/controller_server/map`은 차량에서 오지 않는다. 중앙 map-server가 자체 발행하는 topic이며
rosbag recorder가 별도 설정으로 기록한다. 차량 telemetry에 `/map` 또는
`/{robot}/map`을 추가하지 않는다.

중앙 Foxglove(`ws://<server-ip>:8765`)는 관제용 화면이라 `/robot_1/rgb/image_raw`와
`/robot_2/rgb/image_raw`는 표시하지만 raw depth image와 depth camera info는 topic picker에
노출하지 않는다. depth 점검은 차량의 Foxglove Bridge에 직접 연결해서 수행한다.

Nav2의 plan, local plan, costmap, NavigateToPose action status는 현재 `enabled: false`로
선언되어 있다. 필요할 때 해당 항목만 `true`로 바꾸고 worker와 recorder를 재생성한다.

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

새 message type을 활성화하면 서버 이미지에 해당 ROS message package가 설치되어 있어야
한다. 차량 측 topic rate 또는 message type 변경은 차량 Bridge 운영 환경에서 맞춘다.

### rosbag recorder

`rosbag-recorder`는 활성 차량 target, 각 차량의 `/fleet_bridge/status`, 활성 중앙 topic을
새 rosbag2 세션에 기록한다. `ROSBAG_SESSION_ID`가 비어 있으면 UTC 시각 기반 이름을 쓰고,
이미 있는 디렉터리는 덮어쓰지 않는다. Compose를 올리기 전에 호스트 저장 경로를 만든다.

```bash
sudo mkdir -p /srv/fleet-rosbag
docker compose --env-file .env.server -f docker-compose.server.yaml \
  up -d --force-recreate rosbag-recorder
```

특정 세션 이름이 필요하면 `.env.server`에 `ROSBAG_SESSION_ID=inspection-001`처럼 설정한 뒤
같은 명령으로 recorder를 다시 생성한다. 기록 시작 뒤 `/rosbag/<session>/metadata.yaml`이
생성됐는지 확인한다.

## 테스트 Command API와 Swagger

`command-api`는 FastAPI 기반이다. Swagger UI는
`http://<server-ip>:8080/docs`에서 연다. 기본 `COMMAND_API_HOST=127.0.0.1`은
서버 자신의 Fleet Manager만 호출할 수 있게 한다. 다른 장비에서 Swagger를 열려면
`COMMAND_API_HOST=0.0.0.0`으로 명시적으로 변경하고 방화벽을 먼저 설정한다.

- `POST /api/v1/robots/{robot_id}/cmd_vel`

  ```json
  {
    "linear_x": 0.1,
    "angular_z": 0.0,
    "hold_ms": 300
  }
  ```

- `POST /api/v1/robots/{robot_id}/stop`

`cmd_vel`은 `config/fleet.yaml`의 선속도·각속도·유지 시간 상한을 검증한다. 지정한
`hold_ms` 동안만 command를 발행하며 종료와 오류 경로에서 반드시 zero Twist를 보낸다.
`stop`은 즉시 zero Twist를 전송한다. 차량 URI 연결 실패, `clientPublish` 미허용,
CDR 미지원은 HTTP 503으로 반환한다.

```bash
curl -X POST http://127.0.0.1:8080/api/v1/robots/robot_1/cmd_vel \
  -H 'content-type: application/json' \
  -d '{"linear_x":0.1,"angular_z":0.0,"hold_ms":300}'

curl -X POST http://127.0.0.1:8080/api/v1/robots/robot_1/stop
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
