# Standalone DDS Observation Bundle

동일한 `ROS_DOMAIN_ID=215`를 사용하는 실차 `robot_1`, `robot_2`의 telemetry를 관제 PC에서
한곳으로 모으는 독립 Docker bundle이다. `foxglove-bridge`는 Foxglove Studio에 WebSocket을
제공하고, `map-server`는 관제용 정적 지도를 `/controller_server/map`으로 발행하며,
`rosbag-recorder`는 두 차량의 핵심 주행 telemetry를 누적 저장한다.

`foxglove-bridge`와 `map-server`는 `network_mode: host`와 `ipc: host`를 함께 사용한다.
이는 Fast DDS가 같은 호스트의 두 컨테이너 사이에서 Shared Memory transport로 실제 메시지를
전달할 수 있게 하기 위한 설정이다.

```text
robot_1 (/robot_1/*, Domain 215) ─┐
                                  ├─ 관제 PC: foxglove-bridge → Foxglove Studio
robot_2 (/robot_2/*, Domain 215) ─┘          rosbag-recorder → persistent rosbag
관제 지도 (map.yaml + map.pgm) ──── map-server → /controller_server/map
```

이 bundle은 동일 Domain의 기본 DDS multicast discovery를 사용한다. 따라서
`dds-discovery`, `dds-domain-bridge`, Gazebo에는 의존하지 않는다. Linux Docker host에서만
운영한다. Docker Desktop의 host network는 실차 운영 검증 환경이 아니다.

## 시작

```bash
cd vehicle_simulator_model/ubuntu/dds-observation
cp .env.example .env
docker compose up -d --build
docker compose ps
docker compose logs -f foxglove-bridge rosbag-recorder
```

Foxglove Studio는 관제 서버와 같은 네트워크의 신뢰된 PC에서
`ws://<관제-서버-LAN-IP>:<FOXGLOVE_PORT>`로 연결한다. 기본값은
`ws://<관제-서버-LAN-IP>:8765`이며, `.env`에서 포트를 변경할 수 있다.

```env
FOXGLOVE_BIND_ADDRESS=0.0.0.0
FOXGLOVE_PORT=9234
```

`FOXGLOVE_BIND_ADDRESS`를 `0.0.0.0`으로 두면 모든 네트워크 인터페이스에서 수신한다.
방화벽에서는 신뢰된 출발지에만 설정한 `FOXGLOVE_PORT`의 TCP 접속을 허용한다.

## 관제 지도

`MAP_DIRECTORY`는 `map.yaml`과 이 파일이 참조하는 `map.pgm`이 함께 있는 호스트
디렉터리다. 컨테이너에는 읽기 전용 `/maps`로 mount된다. `map-server`는 이 지도와
호스트의 `mentorpi_map_server` 패키지 소스를 mount하고, 시작할 때마다 `/ws` named
volume에 해당 패키지를 빌드한 뒤 `/controller_server/map`을 발행한다.

```env
MAP_DIRECTORY=/srv/mentorpi/maps/current
MAP_USE_SIM_TIME=false
```

소스 또는 지도 파일을 변경한 뒤에는 map-server만 다시 생성한다.

```bash
docker compose up -d --force-recreate map-server
docker compose logs -f map-server
```

`rosbag-recorder`는 `/tf`, `/tf_static`, `/fleet/status`와 두 차량의 `odom`, `scan_raw`,
`imu/data_raw`를 named volume에 기록한다. Depth camera topic은 포함하지 않는다. 재시작 시
UTC timestamp 이름의 새 bag을 만들며, 같은 초에 재시작되면 `-01`, `-02` suffix로 기존 bag을
보존한다.

```bash
# 특정 세션 이름으로 rosbag recorder만 재생성
ROSBAG_SESSION_ID=live-20260816-01 \
  docker compose up -d --force-recreate rosbag-recorder

# 누적 rosbag 목록 확인
docker run --rm -v mentorpi-rosbag-data:/rosbag alpine ls -lah /rosbag
```

자동 보존 기간·용량 제한은 아직 적용하지 않는다. 관제 PC의 디스크 사용량을 운영 항목으로
확인하고, 필요해지면 별도 retention 정책을 추가한다.
