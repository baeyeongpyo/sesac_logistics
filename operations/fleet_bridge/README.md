# Fleet Bridge

`fleet_bridge`는 관제 서버와 차량 사이의 통신 중개 bundle이다. 차량별 Foxglove
WebSocket telemetry를 관제 ROS 2 Domain으로 재발행하고, 차량 Command API로 HTTP
명령을 전달하며, 최신 telemetry를 SQLite에 기록한다.

중앙 Foxglove Bridge, 3D asset, 지도 발행, 창고 zone overlay는 모두
[`../monitoring`](../monitoring/README.md)이 소유한다.

## 차량 인터페이스

각 차량은 다음 endpoint를 제공해야 한다.

```text
Foxglove Bridge WebSocket  ws://<vehicle-ip>:8766
Command API                http://<vehicle-ip>:8082
```

차량 Foxglove Bridge는 telemetry 관측 전용으로 운용한다. 명령은 Fleet Bridge의
Command API가 차량 HTTP endpoint로만 전달한다.

## 설정 및 실행

`config/fleet.yaml`은 차량 ID, Foxglove URI, Command API URL, 속도 상한을 관리한다.
`config/telemetry.yaml`은 차량 원본 topic과 `/{robot_id}/*` 재발행 topic의 type,
`worker_rate`, qos를 정의한다.

```bash
cp operations/fleet_bridge/.env.example operations/fleet_bridge/.env.server
docker compose --env-file operations/fleet_bridge/.env.server \
  -f operations/fleet_bridge/docker-compose.yaml up -d --build
```

Compose는 다음 차량 통신 서비스만 실행한다.

```text
worker-robot-1       차량 1 telemetry WebSocket 중계
worker-robot-2       차량 2 telemetry WebSocket 중계
command-api          차량 Command API HTTP 중계 (:8080)
telemetry-writer     차량 pose·battery 최신 상태 SQLite 기록
```

관제 PC의 ROS Domain은 225이며 worker와 telemetry writer는 host network/IPC에서
실행한다. `telemetry-writer`가 기록하는 DB에는 차량별 최신 pose와 battery raw 값만
유지된다. map frame과 일치하는 차량 TF만 pose로 저장하며, stale TF는 이전 정상 pose를
덮어쓰지 않는다.

## Command API

Swagger UI는 `http://<server-ip>:8080/docs`에서 제공한다. Fleet Bridge는 등록된
차량에 대해 다음 요청을 전달한다.

- `GET /api/v1/vehicle-command/{robot_id}/vehicle-status`
- `POST /api/v1/vehicle-command/{robot_id}/localization/initial-pose`
- `POST /api/v1/vehicle-command/{robot_id}/navigation/goals`
- `POST /api/v1/vehicle-command/{robot_id}/navigation/waypoints`
- `POST /api/v1/vehicle-command/{robot_id}/cmd-vel`
- `POST /api/v1/vehicle-command/{robot_id}/fork/up`
- `POST /api/v1/vehicle-command/{robot_id}/fork/down`
- `POST /api/v1/vehicle-command/{robot_id}/stop`

## 확인 및 테스트

```bash
docker compose --env-file operations/fleet_bridge/.env.server \
  -f operations/fleet_bridge/docker-compose.yaml logs -f worker-robot-1

docker compose --env-file operations/fleet_bridge/.env.server \
  -f operations/fleet_bridge/docker-compose.yaml exec worker-robot-1 \
  bash -lc 'ROS2CLI_NO_DAEMON=1 ros2 topic hz /robot_1/scan_filtered'

PYTHONPATH=operations/fleet_bridge/common/fleet_bridge_config \
  python3 -m unittest discover -s operations/fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v

PYTHONPATH=operations/fleet_bridge/common/fleet_bridge_config:operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker \
  python3 -m unittest discover -s operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/test -p 'test_*.py' -v

python3 -m unittest discover -s operations/fleet_bridge/test -p 'test_*.py' -v
```
