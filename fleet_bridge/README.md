# Fleet Foxglove Bridge

차량의 ROS 2 telemetry를 DDS Domain 사이에서 직접 중계하지 않고 Foxglove
WebSocket으로 서버에 전달하는 배포 번들이다. 차량 명령은 기존 Fleet Manager의
REST API 경로를 그대로 사용한다. 이 번들은 상태 수집과 관제만 담당하며,
Foxglove client publish·service·parameter 기능은 비활성화한다.

```text
robot_1 / Humble / Domain 215                         server / Humble / Domain 225
ROS nodes -> filter -> Foxglove :8766 -> worker-robot-1 -> /robot_1/*

robot_2 / Humble / Domain 216
ROS nodes -> filter -> Foxglove :8766 -> worker-robot-2 -> /robot_2/*
                                                        |
                                                        +-> Foxglove :8765
```

차량과 서버 이미지는 모두 ROS 2 Humble/Jammy를 사용한다. 차량 주행 환경이
Humble이므로 Foxglove 컨테이너만 Jazzy로 올리지 않는다. 차량 내부 DDS는
`rmw_fastrtps_cpp`와 Fast DDS 기본 SHM+UDP를 사용하되 `ROS_LOCALHOST_ONLY=1`,
`network_mode: host`, `ipc: host`로 차량 호스트 안에 한정한다. 차량 간 LAN 구간에는
DDS discovery가 아니라 WebSocket 한 연결만 사용한다.

## 포트와 Domain

| 위치 | ROS_DOMAIN_ID | 포트 | 용도 |
|---|---:|---:|---|
| robot_1 | 215 | 8766 | 서버 worker용 제한 telemetry |
| robot_1 | 215 | 8765 | 필요할 때만 여는 직접 debug |
| robot_2 | 216 | 8766 | 서버 worker용 제한 telemetry |
| robot_2 | 216 | 8765 | 필요할 때만 여는 직접 debug |
| server | 225 | 8765 | 두 차량 통합 Foxglove 관제 |

차량의 기존 주행 컨테이너도 같은 `ROS_DOMAIN_ID`, host network, host IPC를
사용해야 한다. 가능하면 해당 컨테이너에도 `ROS_LOCALHOST_ONLY=1`을 적용한다.

## 이미지 빌드

저장소 루트에서 실행한다.

```bash
docker build \
  -f fleet_bridge/vehicle/Dockerfile \
  -t mentorpi-fleet-bridge-vehicle:humble \
  fleet_bridge

docker build \
  -f fleet_bridge/server/Dockerfile \
  -t mentorpi-fleet-bridge-server:humble \
  fleet_bridge
```

두 이미지는 Foxglove Bridge 0.8.5의 commit
`41f96cc6053632a472d9a821989952771b1117f2`를 source build한다. 이 버전과 서버
worker가 사용하는 `foxglove.websocket.v1` protocol을 함께 고정했으므로 commit을
독립적으로 변경하면 안 된다. 서버 이미지는 Jammy 기본 패키지의 Python 3.10
비호환 문제를 피하기 위해 WebSocket client도 `websockets==10.4`로 고정한다.

## 차량 실행

각 차량에 `fleet_bridge` 디렉터리를 배포한 뒤 차량별 env 파일을 만든다.

```bash
cp fleet_bridge/.env.example fleet_bridge/.env.vehicle
```

robot_1은 다음 값을 사용한다.

```dotenv
ROBOT_ID=robot_1
ROS_DOMAIN_ID=215
```

robot_2에서는 다음처럼 바꾼다.

```dotenv
ROBOT_ID=robot_2
ROS_DOMAIN_ID=216
```

항상 실행하는 제한 telemetry endpoint를 시작한다.

```bash
docker compose --env-file fleet_bridge/.env.vehicle \
  -f fleet_bridge/docker-compose.vehicle.yaml \
  up -d foxglove-fleet
```

상세 진단 중에만 직접 endpoint를 추가한다. `foxglove-debug`는 profile 서비스라
평상시 `up`에는 포함되지 않는다.

```bash
docker compose --env-file fleet_bridge/.env.vehicle \
  -f fleet_bridge/docker-compose.vehicle.yaml \
  --profile debug up -d foxglove-debug
```

진단이 끝나면 raw scan/camera 구독을 먼저 끊고 서비스를 종료한다.

```bash
docker compose --env-file fleet_bridge/.env.vehicle \
  -f fleet_bridge/docker-compose.vehicle.yaml \
  --profile debug stop foxglove-debug
```

## 서버 실행과 Foxglove 연결

서버 env의 두 URI를 실제 차량 IP로 변경한다. 포트는 fleet endpoint인 8766이다.

```dotenv
SERVER_ROS_DOMAIN_ID=225
ROBOT_1_FOXGLOVE_URI=ws://192.168.10.215:8766
ROBOT_2_FOXGLOVE_URI=ws://192.168.10.216:8766
```

```bash
cp fleet_bridge/.env.example fleet_bridge/.env.server
docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml \
  up -d
```

Foxglove 앱에서는 `ws://<server-ip>:8765` 하나에 연결한다. 서버 Bridge가 Domain
225의 `/robot_1/*`, `/robot_2/*`, `/fleet/*`만 노출하므로 두 차량을 한 화면에서
볼 수 있다. 차량 원본을 일시적으로 확인하려면 별도 탭에서
`ws://<robot-ip>:8765`에 연결하고 차량의 debug profile을 종료한 뒤 탭도 닫는다.

Humble은 빈 string-array parameter를 표현하지 못하므로 Bridge `capabilities`에는
`none` sentinel을 사용한다. pin된 Bridge가 인식하는 capability 이름과 일치하지
않아 client publish, service, parameter, asset, connection graph 기능은 활성화되지
않는다. deny whitelist도 별도로 적용된다.

현재 구성은 평문 `ws://`이고 인증 기능을 제공하지 않는다. 운영망 방화벽에서
8765/8766 접근 주체를 제한하고, 외부망을 통과해야 하면 TLS reverse proxy 또는
VPN을 별도로 둔다.

## telemetry.yaml 설정

[`config/telemetry.yaml`](config/telemetry.yaml)이 차량 whitelist, 차량 filter,
서버 worker mapping, 서버 publisher QoS의 단일 설정이다.

- `enabled`: 차량의 8766 fleet endpoint와 서버 worker 구독 여부이다.
- `debug`: 차량의 8765 직접 endpoint 노출 여부이다. `enabled: false`인 raw topic도
  debug만 `true`로 둘 수 있다.
- `source`: 차량 ROS node가 발행하는 원본 topic이다.
- `uplink`: 차량 Foxglove Bridge가 노출하고 서버 worker가 구독하는 topic이다.
- `target`: worker가 서버 Domain 225에 발행하는 topic이다.
- `type`: 양쪽에서 검증할 ROS message type이다. 일치하지 않으면 구독하지 않는다.
- `filter`: 차량에서 WebSocket을 통과하기 전에 적용하는 `passthrough`, `rate`,
  `on_change` 정책이다.
- `worker_rate.max_rate_hz`: WebSocket 수신 후 Domain 225에 발행하는 2차 상한이다.
  LAN 대역폭 절감은 반드시 차량의 `filter`에서 수행한다.
- `qos`: 서버 publisher의 `reliability`, `durability`, `history`, `depth`이다.

설정 파일은 컨테이너에 read-only로 mount된다. 변경 후 같은 파일을 해당 차량과
서버에 배포하고 관련 서비스를 재시작한다. 새로운 message package를 추가했다면
두 Dockerfile에도 ROS package를 추가하고 이미지를 다시 빌드해야 한다.

### scan 추가 또는 제한 변경

기본 `scan` 항목은 `enabled: false`이다. 서버 상시 관제가 필요할 때만 `true`로
바꾼다. 현재 값은 차량과 worker 양쪽에서 최대 2 Hz, Best Effort, Keep Last 1이다.

```yaml
- id: scan
  enabled: true
  source: /{robot}/scan
  uplink: /{robot}/fleet_bridge/scan
  target: /{robot}/scan
  type: sensor_msgs/msg/LaserScan
  filter:
    mode: rate
    max_rate_hz: 2.0
  worker_rate:
    max_rate_hz: 2.0
  qos:
    reliability: best_effort
    durability: volatile
    history: keep_last
    depth: 1
  debug: true
```

대역폭이 여전히 크면 두 `max_rate_hz`를 함께 낮춘다. `scan_raw_debug`는 상시
전달하지 않고 차량에 직접 연결할 때만 사용한다.

### battery 정책 변경

기본 battery 정책은 percentage가 1% 또는 voltage가 0.1 V 이상 변할 때만
전송하고, 변화가 없어도 30초마다 heartbeat를 보낸다. `max_rate_hz: 0.2`는 일반
sample을 최대 5초에 한 번으로 제한한다. percentage가 20% 이하인 critical
sample은 `bypass_rate_limit: true`로 즉시 전달한다.

```yaml
filter:
  mode: on_change
  max_rate_hz: 0.2
  heartbeat_sec: 30
  thresholds:
    percentage: 0.01
    voltage: 0.1
  critical:
    field: percentage
    below: 0.2
    bypass_rate_limit: true
```

## 상태 및 장애 확인

worker는 차량별로 독립 재접속하며 1초에서 최대 30초까지 backoff한다. 각 worker는
`/{robot}/fleet_bridge/status`에 다음 필드를 JSON `std_msgs/msg/String`으로 1 Hz
발행한다: `robot_id`, `connection`, `state`, `last_message_at`, `reconnect_count`,
`error`. 정상 수신이 10초를 넘기면 `state`가 `stale`이 된다.

```bash
docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml logs -f worker-robot-1

docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml exec worker-robot-1 \
  bash -lc 'ros2 topic echo /robot_1/fleet_bridge/status'
```

topic이 보이지 않으면 다음 순서로 확인한다.

1. 차량 주행 컨테이너와 `foxglove-fleet`의 `ROS_DOMAIN_ID`, host network/IPC가
   같은지 확인한다.
2. 차량 8766 포트가 LISTEN 상태인지 확인한다.
3. server worker log에서 WebSocket 연결과 schema/type mismatch를 확인한다.
4. 차량과 서버에 배포된 `telemetry.yaml`이 동일한지 확인한다.
5. TF를 합쳐 볼 경우 차량이 `robot_1/...`, `robot_2/...` frame prefix를 이미
   발행하는지 확인한다. worker는 serialized frame ID를 변경하지 않는다.

## 기존 Domain Bridge 전환과 지연 A/B 검증

같은 target topic을 기존 Domain Bridge와 새 worker가 동시에 Domain 225에
발행하면 중복 데이터가 생긴다. `bridge-robot-1`, `bridge-robot-2` 등 기존
Domain Bridge 컨테이너를 먼저 중지한 뒤 해당 차량 worker를 시작한다. 기존 구성은
삭제하지 말고 A/B 비교와 rollback 용도로 보관한다.

네트워크 지연은 실제 Ubuntu 차량 LAN에서 동일 조건으로 최소 100회 측정한다.

1. 모든 bridge를 끈 baseline을 측정한다.
2. 기존 DDS Domain Bridge만 켜고 같은 측정을 수행한다.
3. 기존 bridge를 끈 뒤 새 `foxglove-fleet`와 server worker만 켜고 측정한다.
4. 각 단계에서 ping RTT/packet loss, 차량 CPU·메모리·network I/O, 서버 topic
   rate를 함께 기록한다.

```bash
ping -c 100 <robot_1-ip>
docker stats --no-stream

docker compose --env-file fleet_bridge/.env.server \
  -f fleet_bridge/docker-compose.server.yaml exec worker-robot-1 \
  bash -lc 'timeout 15 ros2 topic hz /robot_1/odom'
```

scan을 활성화하거나 `foxglove-debug`에서 raw topic을 구독할 때도 같은 측정을
반복한다. WebSocket 서비스 시작만으로 RTT가 상승하는지, 실제 고대역 topic을
구독했을 때만 상승하는지를 분리해 기록해야 원인을 판단할 수 있다.

## 정적 검증

```bash
PYTHONPATH=fleet_bridge/common/fleet_bridge_config \
  python3 -m unittest discover \
  -s fleet_bridge/common/fleet_bridge_config/test -p 'test_*.py' -v

PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter \
  python3 -m unittest discover \
  -s fleet_bridge/vehicle/ros2_ws/src/fleet_telemetry_filter/test -p 'test_*.py' -v

PYTHONPATH=fleet_bridge/common/fleet_bridge_config:fleet_bridge/server/ros2_ws/src/foxglove_ros_worker \
  python3 -m unittest discover \
  -s fleet_bridge/server/ros2_ws/src/foxglove_ros_worker/test -p 'test_*.py' -v

python3 -m unittest discover -s fleet_bridge/test -p 'test_*.py' -v

docker compose --env-file fleet_bridge/.env.example \
  -f fleet_bridge/docker-compose.vehicle.yaml config --quiet
docker compose --env-file fleet_bridge/.env.example \
  -f fleet_bridge/docker-compose.server.yaml config --quiet
```
