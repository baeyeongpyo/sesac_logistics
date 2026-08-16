# Standalone DDS Domain Bridge

실차 MentorPi의 telemetry를 중앙 ROS 2 Domain에서 확인하기 위한 독립 Docker
bundle이다. Gazebo, `sim-up`, `sim-adapter`, mapping, Nav2, Foxglove를 시작하거나
의존하지 않는다.

## 구성

```text
실차 Domain 1 / Domain 2
        │
        ├─ Fast DDS Discovery Server (중앙 서버 UDP 11811)
        │
        └─ dds-domain-bridge ──> 중앙 Domain 215 ──> dds-observe
```

`dds-discovery`는 discovery control traffic만 처리한다. 센서 payload는 bridge와
실차 사이에서 Fast DDS UDPv4로 직접 전달된다.

## 시작

중앙 Linux 서버에서 실행한다. Docker Desktop의 host network는 Linux Docker의 host
network와 동작 방식이 다르므로 운영 환경으로 사용하지 않는다.

```bash
cd vehicle_simulator_model/ubuntu/dds-domain-bridge
cp .env.example .env
docker compose up -d --build
docker compose ps
docker compose logs -f dds-discovery dds-domain-bridge
```

실차는 이미 설정된 값이 중앙 서버의 **고정 LAN/VPN IP**를 가리키는지 확인한다.

```bash
echo "$RMW_IMPLEMENTATION"          # rmw_fastrtps_cpp
echo "$ROS_DOMAIN_ID"               # robot_1=1, robot_2=2 기본값
echo "$ROS_DISCOVERY_SERVER"        # <중앙서버-IP>:11811
echo "$FASTDDS_BUILTIN_TRANSPORTS"  # UDPv4
```

`ROS_DISCOVERY_SERVER`에 `127.0.0.1`이나 Docker service 이름을 넣지 않는다. 이 값은
실차가 접근할 수 있는 중앙 서버의 IP여야 한다.

## 데이터 확인

bridge가 실행된 central Domain `215`에서 수신되는 topic을 확인한다.

```bash
docker compose exec dds-domain-bridge dds-observe topics
docker compose exec dds-domain-bridge dds-observe echo /robot_1/odom
docker compose exec dds-domain-bridge dds-observe hz /robot_1/scan_raw
```

`topics`는 Super Client의 **전체 Discovery Server graph**를 출력하므로,
`DDS_OBSERVE_DOMAIN_ID`로 source Domain을 분리하지 않는다. 이 명령은 연결된
차량 namespace가 Discovery Server에 등록됐는지 확인하는 용도다.

실차 source Domain에서 알려진 telemetry를 실제로 수신하는지는 normal DDS
subscription인 `echo` 또는 `hz`로 확인한다. 아래 예시는 `robot_1`이 Domain `1`일 때다.

```bash
docker compose exec \
  -e DDS_OBSERVE_DOMAIN_ID=1 \
  dds-domain-bridge dds-observe echo /robot_1/odom_raw

docker compose exec \
  -e DDS_OBSERVE_DOMAIN_ID=1 \
  dds-domain-bridge dds-observe hz /robot_1/scan_raw
```

실제 topic 이름을 모르는 경우에는 실차에서 직접 normal client로 `ros2 topic list
--no-daemon`을 실행한다. Discovery Server v2의 일반 client는 전체 graph를 노출하지
않으므로, 이 방법은 해당 실차에서 실행해야 의미가 있다. bridge 메인 프로세스와
`echo`/`hz`는 일반 client로 유지된다.

## Domain과 topic allowlist 변경

기본값은 central `215`, `robot_1=1`, `robot_2=2`다. 실제 Domain이 다르면 `.env`를
수정한 뒤 bridge만 재생성한다.

```bash
ROBOT_1_DOMAIN_ID=10
ROBOT_2_DOMAIN_ID=11
CENTRAL_DOMAIN_ID=215

docker compose up -d --force-recreate dds-domain-bridge
```

기본 allowlist는 다음 telemetry만 source Domain에서 central Domain으로 단방향
bridge한다.

- `odom`, `odom_raw`
- `scan`, `scan_raw`
- `imu`, `ros_robot_controller/imu`
- `tf`, `tf_static`

각 topic에는 `/robot_1` 또는 `/robot_2` namespace가 있어야 한다. 실제 topic 이름이
다르면 `bridge.yaml.template`에 해당 topic과 정확한 ROS message type을 추가한 뒤
bridge를 재생성한다. `cmd_vel`, goal, cancel, stop 등 제어 topic은 의도적으로 이
bundle에 포함하지 않는다.

## 네트워크와 종료

실차 서브넷에서 중앙 서버 UDP `11811`과 DDS UDP payload 통신을 허용해야 한다.
외부 인터넷에는 노출하지 않는다.

```bash
docker compose down
```
