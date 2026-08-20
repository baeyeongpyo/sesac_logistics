# Foxglove Fleet Bridge 설계

**작성일:** 2026-08-20
**상태:** 사용자 설계 및 설정 기반 운영 승인, 구현 진행

## 1. 목표

저장소 루트에 독립 실행 번들 `fleet_bridge/`를 추가한다.

- 각 실제 차량은 자신의 ROS Domain에서 차량용 `foxglove_bridge` 컨테이너를
  실행한다.
- 서버는 차량마다 하나의 `foxglove_ros_worker`를 실행해 Foxglove WebSocket의
  ROS 2 CDR 메시지를 수신하고 Domain 225에 재발행한다.
- 서버의 `foxglove_bridge`는 Domain 225에 모인 차량별 토픽을 하나의 관제
  endpoint로 노출한다.
- 운영자가 상세 센서 진단을 해야 할 때는 서버를 거치지 않고 차량의 debug
  Foxglove endpoint에 직접 연결할 수 있다.
- RMF/Fleet Manager의 차량 명령은 이 번들에서 Foxglove publish로 보내지 않고
  기존 REST API 명령 경로를 유지한다.

## 2. 고정 환경

| 대상 | ROS Domain ID | 역할 |
| --- | ---: | --- |
| `robot_1` | 215 | 차량 1 ROS graph와 Foxglove Bridge |
| `robot_2` | 216 | 차량 2 ROS graph와 Foxglove Bridge |
| 서버 | 225 | 차량 상태 재발행, Fleet Manager/RMF, 통합 Foxglove 관제 |

- ROS 배포판은 Humble, 기반 운영체제는 Ubuntu 22.04로 한다.
- 차량용 이미지는 `ros:humble-ros-base-jammy`에서 pin된 Foxglove Bridge source와
  프로젝트 ROS package를 multi-stage build한다.
- Foxglove가 제공하는 사전 빌드 Docker image는 amd64 전용일 수 있으므로 차량
  이미지의 기반으로 사용하지 않는다. 차량 장비의 native `linux/arm64` 빌드를
  지원하고 서버는 `linux/amd64`를 지원한다.
- 모든 차량과 서버는 서로의 고정 IP 또는 DNS 이름과 TCP port에 접근 가능한
  신뢰된 LAN에 있다고 가정한다.

## 3. 전체 아키텍처

```text
robot_1 / Domain 215
  ROS publishers
       |
       +--> telemetry filter --> foxglove-fleet :8766 --+
       |                            |
       +--> foxglove-debug :8765    | Foxglove WebSocket
                                    v
server / Domain 225          foxglove-worker-robot-1
                                    |
                                    +--> /robot_1/* ROS publishers

robot_2 / Domain 216
  ROS publishers
       |
       +--> telemetry filter --> foxglove-fleet :8766 --+
       |                            |
       +--> foxglove-debug :8765    | Foxglove WebSocket
                                    v
server / Domain 225          foxglove-worker-robot-2
                                    |
                                    +--> /robot_2/* ROS publishers

server / Domain 225
  /robot_1/* + /robot_2/*
             |
             v
  server-foxglove-bridge :8765
             |
             v
  운영자 Foxglove 통합 관제

상세 진단:
  운영자 Foxglove --> ws://robot_1:8765 또는 ws://robot_2:8765

명령:
  RMF --> Fleet Manager --> REST API --> 차량 Command Gateway --> Nav2
```

`foxglove_bridge`는 WebSocket 서버이므로 차량 Bridge와 서버 Bridge를 직접
연결하지 않는다. `foxglove_ros_worker`가 차량 Bridge의 클라이언트이면서 서버
Domain 225의 ROS publisher 역할을 한다.

## 4. 디렉터리 구조

```text
fleet_bridge/
  .env.example
  README.md
  docker-compose.server.yaml
  docker-compose.vehicle.yaml
  config/
    fleet.yaml
    telemetry.yaml
    server_foxglove.yaml
  common/
    fleet_bridge_config/
      package.xml
      setup.py
      fleet_bridge_config/
        loader.py
        models.py
  vehicle/
    Dockerfile
    entrypoint.sh
    ros2_ws/
      src/
        fleet_telemetry_filter/
          package.xml
          setup.py
          fleet_telemetry_filter/
            policy.py
            node.py
          launch/
            vehicle_foxglove.launch.py
          test/
            test_policy.py
  server/
    Dockerfile
    entrypoint.sh
    ros2_ws/
      src/
        foxglove_ros_worker/
          package.xml
          setup.py
          setup.cfg
          resource/foxglove_ros_worker
          foxglove_ros_worker/
            __init__.py
            main.py
            config.py
            protocol.py
            republisher.py
          test/
            test_config.py
            test_protocol.py
            test_republisher.py
  test/
    test_compose_contract.py
    test_bundle_contract.py
```

기존 `vehicle_simulator_model/ubuntu/dds-observation` 및 그 안의 사용자 변경은
수정하지 않는다. 초기 구현은 새 번들 안에서 독립적으로 검증한다.

## 5. 차량용 Docker image

### 5.1 이미지 내용

차량용 이미지는 다음 패키지만 포함하는 작은 ROS runtime으로 만든다.

- Foxglove WebSocket v1 호환성을 고정한 `foxglove_bridge` 0.8.5
- `ros-humble-rmw-fastrtps-cpp`
- `fleet_telemetry_filter`
- 상태 토픽 스키마에 필요한 표준 ROS message package
- launch 파일과 entrypoint

`foxglove_ros_worker`가 구현하는 WebSocket client protocol과 차량 Bridge의
protocol drift를 방지하기 위해 차량과 서버 이미지 모두
`foxglove/ros-foxglove-bridge` commit
`41f96cc6053632a472d9a821989952771b1117f2`(tag `0.8.5`)를 source build한다.
이 버전은 `foxglove.websocket.v1`을 사용한다. Foxglove SDK 계열 Bridge로의
업그레이드는 새 protocol fixture와 container integration test가 통과한 뒤 별도
변경으로 수행한다.

Docker Compose는 `network_mode: host`와 `ipc: host`를 사용한다. 컨테이너의
`ROS_DOMAIN_ID`는 차량별 `.env`에서 215 또는 216으로 설정한다. 차량 host의 ROS
graph 내부 통신은 `FASTDDS_BUILTIN_TRANSPORTS=DEFAULT`로 SHM을 우선 사용한다.
SHM 권한 또는 IPC 문제가 확인된 경우에만 `.env`에서 UDPv4 전송으로 전환한다.
차량 ROS 노드가 모두 host network를 공유하면 `ROS_LOCALHOST_ONLY=1`로 DDS가 차량
외부 네트워크로 전파되지 않게 한다.

### 5.2 fleet endpoint

`foxglove-fleet` 서비스는 port 8766에서 항상 실행하며 서버 worker만 접속한다.

- ROS node name은 `foxglove_fleet_bridge`로 고정한다.
- 읽기 전용으로 구성한다.
- `clientPublish`, service call, parameter read/write, asset access를 비활성화한다.
- 운영 allowlist 토픽만 노출한다.
- outgoing backlog는 작은 유한 크기로 제한해 느린 서버 연결이 차량 memory를
  지속적으로 증가시키지 못하게 한다.
- topic whitelist와 Best Effort 강제 목록은 `telemetry.yaml`에서 생성하며 image에
  하드코딩하지 않는다.

### 5.3 debug endpoint

`foxglove-debug` 서비스는 Compose `debug` profile에서만 실행하며 port 8765를
사용한다.

- ROS node name은 `foxglove_debug_bridge`로 고정해 fleet Bridge와 충돌하지 않게
  한다.
- 운영자가 차량에 직접 연결할 때만 시작한다.
- 카메라, scan, TF, costmap 등 진단 토픽을 별도 allowlist로 제공한다.
- raw image 대신 compressed image를 우선하고 PointCloud는 기본 allowlist에서
  제외한다.
- 이 endpoint가 활성화돼도 fleet endpoint의 topic allowlist와 권한은 바뀌지
  않는다.

## 6. 차량 telemetry 계약

서버로 상시 전달하는 기본 토픽은 저대역 운영 상태로 제한한다.

| source topic | uplink topic | target topic | type | QoS/제한 |
| --- | --- | --- | --- | --- |
| `/{robot}/odom` | `/{robot}/odom` | `/{robot}/odom` | `nav_msgs/msg/Odometry` | best effort, keep last 5 |
| `/{robot}/tf` | `/{robot}/tf` | `/{robot}/tf` | `tf2_msgs/msg/TFMessage` | best effort, keep last 20 |
| `/{robot}/tf_static` | `/{robot}/tf_static` | `/{robot}/tf_static` | `tf2_msgs/msg/TFMessage` | reliable, transient local, keep last 1 |
| `/{robot}/navigation/status` | `/{robot}/navigation/status` | `/{robot}/navigation/status` | `std_msgs/msg/String` | reliable, keep last 10 |
| `/{robot}/battery_state` | `/{robot}/fleet_bridge/battery_state` | `/{robot}/battery_state` | `sensor_msgs/msg/BatteryState` | reliable, 변경 감지/최대 0.2Hz, 30초 heartbeat |
| `/{robot}/diagnostics` | `/{robot}/diagnostics` | `/{robot}/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | reliable, keep last 10 |
| `/{robot}/scan` | `/{robot}/fleet_bridge/scan` | `/{robot}/scan` | `sensor_msgs/msg/LaserScan` | best effort, keep last 1, 최대 2Hz, 기본 비활성 |

실차에 아직 존재하지 않는 optional 토픽은 worker 시작 실패의 원인이 되지 않는다.
Foxglove channel이 advertise될 때 동적으로 구독하고, 연결 후 뒤늦게 advertise되는
토픽도 구독한다.

다음 토픽은 상시 서버 재발행에서 제외한다.

- raw/compressed camera image
- `scan_raw`와 원본 고주기 `scan`
- PointCloud
- costmap
- 고주기 디버깅 중간 결과

이 토픽은 차량 debug endpoint로 직접 관찰한다. 추후 서버에서 필요하면 명시적
설정과 대역폭 시험을 거쳐 개별 추가한다.

### 6.1 설정 소유권

`config/telemetry.yaml`을 차량과 서버가 함께 읽는 단일 telemetry 정책으로 둔다.
각 항목은 `enabled`, `source`, `uplink`, `target`, `type`, `filter`, `worker_rate`,
`qos`, `debug`를 선언한다.

- `source`: 차량의 원본 ROS topic
- `uplink`: 차량 fleet Foxglove endpoint가 노출하는 topic
- `target`: worker가 Domain 225에 발행하는 topic
- `filter.mode`: `passthrough`, `rate`, `on_change` 중 하나
- `filter.max_rate_hz`: 차량 측 전송 상한이며 네트워크 사용량을 줄인다.
- `worker_rate.max_rate_hz`: 서버 측 2차 상한이며 Domain 225 부하를 제한한다.
- `qos`: worker publisher의 reliability, durability, history, depth
- `debug`: 차량 debug endpoint가 원본 `source`를 노출할지 여부

설정의 `{robot}`은 실행 시 `robot_id`로 치환한다. 알 수 없는 key, 중복 topic ID,
중복 uplink/target, 잘못된 message type, 0 이하 rate, 허용되지 않은 QoS 값은 시작
전에 거절한다. 설정 변경은 image rebuild 없이 read-only volume 교체 후 해당
컨테이너 재시작으로 적용한다. 초기 구현에는 hot reload를 넣지 않는다.

### 6.2 차량 telemetry filter

`passthrough` topic은 원본 topic을 그대로 Foxglove allowlist에 넣는다. `rate` 또는
`on_change` topic은 차량의 `fleet_telemetry_filter`가 별도 `uplink` topic으로
재발행하고, fleet endpoint는 원본이 아닌 uplink만 노출한다.

Battery `on_change` 기본 정책은 percentage 0.01 또는 voltage 0.1 이상 변화할 때
전송하고, 변화가 없어도 30초마다 heartbeat를 전송한다. percentage가 0.20 이하인
critical sample은 rate limit을 우회한다. Scan은 활성화할 때 차량에서 최대 2Hz로
제한하고 Best Effort/Keep Last 1을 사용한다.

## 7. `foxglove_ros_worker`

### 7.1 프로토콜 범위

worker는 Foxglove WebSocket protocol의 수신에 필요한 최소 client 기능만
구현한다.

- pin된 Bridge와 같은 `foxglove.websocket.v1` subprotocol로 연결한다.
- server info와 channel advertise를 처리한다.
- allowlist와 type이 일치하는 channel만 subscribe한다.
- binary message frame에서 subscription ID, timestamp, CDR payload를 분리한다.
- `encoding == cdr`이고 schema name이 설정 type과 일치하는 메시지만 허용한다.
- client publish, service, parameter, asset 기능은 구현하지 않는다.

공식 Python Foxglove SDK의 WebSocket API는 서버 제공에 집중하므로 worker는
`python3-websockets`로 최소 client protocol을 구현한다. protocol parser는 ROS와
분리해 byte fixture 기반 단위 테스트를 수행한다. source build한 pin 버전
`foxglove_bridge`와 연결하는 container integration test로 protocol 호환성을 최종
검증한다.

### 7.2 ROS 재발행

worker는 Domain 225에서 rclpy node로 실행한다.

1. channel의 schema name을 `rosidl_runtime_py.utilities.get_message()`로 ROS
   Python message type으로 해석한다.
2. CDR payload를 `rclpy.serialization.deserialize_message()`로 복원한다.
3. 설정된 target topic과 QoS로 publisher를 만들고 message를 발행한다.
4. `worker_rate.max_rate_hz`가 있으면 최신 sample 중심으로 2차 제한한다. 이 제한은
   Domain 225 부하를 줄이지만 이미 WebSocket을 지난 데이터이므로 차량 uplink
   대역폭 절감 수단으로 사용하지 않는다.

worker가 처리하는 상시 운영 telemetry는 저대역이므로 초기 구현은 Python을
사용한다. raw image/PointCloud를 서버로 상시 전달하도록 범위가 확대되면 C++
generic publisher worker로 교체하는 별도 성능 작업을 수행한다.

### 7.3 topic과 frame 정책

- source topic이 이미 `/{robot}/...` namespace를 사용하면 같은 이름으로
  Domain 225에 발행한다.
- namespace가 없는 source topic을 지원할 때는 `telemetry.yaml`에서 target을
  반드시 `/{robot}/...`로 지정한다.
- worker는 일반 message의 `header.frame_id`를 임의로 변경하지 않는다.
- 두 차량의 TF frame은 차량에서 이미 `robot_1/...`, `robot_2/...` prefix를
  사용해야 한다.
- frame prefix가 없는 실차에는 server relay를 활성화하기 전에 차량 frame
  설정을 수정한다. serialized TF를 조용히 재작성하지 않는다.
- 차량과 서버는 NTP/chrony로 시간을 동기화한다. worker는 원본 ROS header stamp를
  보존한다.

## 8. 연결과 오류 처리

worker는 차량별로 독립 실행한다. 한 차량의 연결 실패가 다른 worker나 서버
Foxglove Bridge를 종료시키지 않는다.

- 연결 실패 시 1초부터 최대 30초까지 exponential backoff로 재접속한다.
- 재접속 시 channel 목록과 publisher mapping을 다시 검증하고 재구독한다.
- schema/type mismatch는 해당 channel만 차단하고 오류를 기록한다.
- malformed frame은 연결을 닫고 재접속한다.
- 마지막 정상 수신 시각이 freshness timeout을 넘으면 차량을 `stale`로 표시한다.
- worker 상태를 `/{robot}/fleet_bridge/status`의 `std_msgs/msg/String` JSON으로
  1Hz 발행한다.
- 상태 payload에는 `robot_id`, `connection`, `state`, `last_message_at`,
  `reconnect_count`, `error`를 포함한다.
- 상태 큐는 과거 메시지를 무제한 보존하지 않는다. ROS sensor state는 최신값이
  중요하므로 유한 queue와 drop-oldest 정책을 사용한다.

## 9. 서버 관제 Bridge

서버 `foxglove-bridge` 서비스는 ROS Domain 225에서 port 8765를 연다.

- `network_mode: host`, `ipc: host`를 사용한다.
- `/robot_1/*`, `/robot_2/*`, `/fleet/*`만 allowlist에 포함한다.
- 기본은 관측 전용이고 client publish, service, parameter write를 비활성화한다.
- 운영자는 평상시 `ws://<server>:8765`에 연결해 두 차량을 함께 관찰한다.
- 상세 진단 시에는 별도 Foxglove 탭에서 `ws://<robot>:8765`에 직접 연결한다.

서버 Bridge가 차량 Bridge에 접속하는 기능은 없다. 차량 연결과 Domain 225
재발행 책임은 전적으로 worker가 가진다.

## 10. Fleet 설정

`config/fleet.yaml`은 초기 두 차량을 정적으로 선언한다.

```yaml
server:
  domain_id: 225
  foxglove_port: 8765
vehicles:
  - id: robot_1
    foxglove_uri: ${ROBOT_1_FOXGLOVE_URI}
    namespace: /robot_1
    enabled: true
  - id: robot_2
    foxglove_uri: ${ROBOT_2_FOXGLOVE_URI}
    namespace: /robot_2
    enabled: true
```

설정 loader는 `${NAME}` 형식만 환경변수로 치환하며, 값이 없으면 시작 전에
명확하게 실패한다. `.env.example`에는 예시 주소를 제공하되 실제 IP는 배포
환경의 `.env` 또는 Compose override에서 주입한다. credential과 인증 token을
Git에 저장하지 않는다.

초기 Compose는 차량마다 명시적인 worker 서비스를 제공한다. 동적 차량 추가와
worker process supervisor는 이번 범위에서 제외하고, 설정 검증과 운영 안정성이
확인된 후 기존 `mentorpi_fleet` registry와 통합한다.

`server_foxglove.yaml`은 Domain 225 관제 Bridge의 allowlist와 읽기 전용
capabilities를 설정한다. `fleet.yaml`, `telemetry.yaml`, `server_foxglove.yaml`은
모두 컨테이너에 read-only로 mount한다.

## 11. 기존 `mentorpi_fleet`와의 관계

기존 `mentorpi_fleet`의 Domain Bridge worker를 즉시 수정하거나 제거하지 않는다.
새 번들의 PoC와 실제 네트워크 검증이 통과한 뒤 다음 교체가 가능하다.

```text
기존 BridgeWorkerManager
  ros2 run domain_bridge ...

후속 교체
  ros2 run foxglove_ros_worker worker ...
```

Domain Bridge와 Foxglove worker가 동시에 같은 target topic을 Domain 225에
발행하면 중복 메시지가 발생하므로 실제 전환 시험에서는 해당 차량의 기존
Domain Bridge 컨테이너를 먼저 중지한다.

## 12. 명령 경계

이번 구현에는 다음을 포함하지 않는다.

- Foxglove `clientPublish`를 사용한 차량 명령
- `/cmd_vel` 네트워크 streaming
- Foxglove를 통한 Nav2 action 호출
- Fleet Manager REST API 구현 또는 변경

명령은 기존 설계대로 Fleet Manager가 command ID, idempotency, timeout을
관리하며 차량 Command Gateway의 REST API를 호출한다. worker가 받은
`navigation/status` 또는 향후 정규 `rmf_state`는 Fleet Manager의 완료 판정에
사용할 수 있지만, HTTP 수락 응답을 실제 작업 완료로 간주하지 않는다.

## 13. 보안

- 차량 fleet endpoint는 신뢰된 서버 IP에서만 접근하도록 host firewall로
  제한한다.
- debug endpoint는 기본 중지하고 필요할 때만 시작한다.
- Bridge capabilities는 최소 권한으로 설정한다.
- 외부망에 노출할 경우 plain `ws://` 대신 WSS 또는 VPN을 사용한다.
- server endpoint도 운영자 네트워크 CIDR로 제한한다.
- Docker socket과 host filesystem을 컨테이너에 mount하지 않는다.

## 14. 테스트 전략

### 14.1 단위 테스트

- fleet/telemetry YAML schema와 중복 ID·topic 검증
- Foxglove server info/advertise JSON parsing
- subscribe request 생성
- binary data frame parsing과 malformed frame 거절
- channel schema/type mismatch 거절
- ROS QoS mapping
- `{robot}` 치환과 차량/server 설정 일관성
- rate/on-change/heartbeat/critical bypass filter policy
- reconnect backoff 상한
- freshness와 status payload

### 14.2 정적 번들 테스트

- 차량 image가 Humble Foxglove Bridge와 Fast DDS runtime을 설치하는지 확인
- vehicle compose가 host network, host IPC, 차량별 Domain ID를 사용하는지 확인
- fleet endpoint가 읽기 전용 capabilities와 엄격한 topic allowlist를 갖는지 확인
- debug service가 `debug` profile에만 있는지 확인
- server worker가 Domain 225를 사용하고 차량별 URI/namespace를 분리하는지 확인
- server Bridge가 Domain 225와 port 8765를 사용하는지 확인

### 14.3 container integration test

1. 테스트 publisher와 차량 Foxglove Bridge를 임시 ROS Domain에서 실행한다.
2. worker가 `Odometry` CDR channel을 구독한다.
3. worker Domain의 target topic에서 동일 field 값을 수신한다.
4. transient-local `tf_static`이 늦은 subscriber에도 전달되는지 확인한다.
5. 차량 Bridge를 재시작하고 worker가 재접속·재구독하는지 확인한다.
6. allowlist 밖의 image topic이 target Domain에 나타나지 않는지 확인한다.

### 14.4 실제 차량 단계 검증

1. `robot_1/odom` 한 토픽만 활성화한다.
2. worker 미실행/실행 상태에서 차량 ping, packet loss, CPU, network throughput을
   비교한다.
3. `robot_1`이 안정적이면 `navigation/status`, battery, diagnostics를 추가한다.
4. 같은 검증을 `robot_2`에 적용한다.
5. 서버 Foxglove에서 두 차량 namespace가 동시에 보이는지 확인한다.
6. 차량 debug endpoint에서 scan/camera를 구독한 동안 ping과 운영 상태 age를
   측정한다.

## 15. 완료 기준

1. `fleet_bridge/vehicle/Dockerfile`이 차량 native architecture에서 빌드된다.
2. 차량 fleet endpoint가 allowlist 토픽만 CDR channel로 광고한다.
3. 서버 worker가 두 차량에 독립 연결하고 Domain 225에 namespace가 분리된 ROS
   토픽을 발행한다.
4. 서버 Foxglove endpoint 하나로 두 차량의 운영 상태를 관찰할 수 있다.
5. 운영자는 필요할 때 차량 debug endpoint에 직접 연결할 수 있다.
6. server/vehicle 재시작 뒤 자동 재접속하고 stale 상태를 올바르게 표시한다.
7. raw image, `scan_raw`, PointCloud가 평상시 서버 uplink에 포함되지 않는다.
8. 기존 Domain Bridge를 중지한 상태에서 ping이 정상 범위를 유지하는지 실제 LAN
   A/B 시험 결과를 기록한다.
9. 관련 단위·정적·container integration test가 모두 통과한다.
10. whitelist, worker mapping, QoS, scan/battery 제한은 config 수정과 서비스
    재시작만으로 변경할 수 있다.

## 16. 주요 위험과 완화

| 위험 | 완화 |
| --- | --- |
| Foxglove protocol version 차이 | Bridge 0.8.5 commit pin, WebSocket v1 parser fixture, pin 버전 integration test |
| raw 센서로 Wi-Fi 재포화 | 상시 allowlist에서 제외, debug endpoint 별도 profile |
| Python deserialize 부하 | 저대역 telemetry만 처리, raw는 직접 연결, 필요 시 C++ worker로 교체 |
| TF frame 충돌 | 차량별 frame prefix를 배포 전 필수 검증 |
| worker와 Domain Bridge 중복 publisher | 전환 시 차량별 기존 bridge를 먼저 중지 |
| 차량 ARM64 image 호환성 | 공식 amd64 전용 image 대신 ROS apt 기반 native build |
| 차량 하나의 연결 장애가 전체 관제로 전파 | 차량별 worker 프로세스와 독립 reconnect 상태 |
