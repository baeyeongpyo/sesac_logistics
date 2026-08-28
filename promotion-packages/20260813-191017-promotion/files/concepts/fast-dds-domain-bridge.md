---
title: MentorPi Fast DDS Discovery Server와 DDS Domain Bridge
created: 2026-08-13
updated: 2026-08-13
type: concept
status: review-required
tags:
  - robotics
  - ros2
  - dds
  - fast-dds
  - discovery-server
  - domain-bridge
  - mentorpi
sources:
  - title: Fast DDS Discovery Server settings
    url: https://fast-dds.docs.eprosima.com/en/latest/fastdds/discovery/discovery_server.html
    accessed: 2026-08-13
  - title: Fast DDS ROS 2 middleware guide
    url: https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/ros2.html
    accessed: 2026-08-13
  - path: vehicle_simulator_model/ubuntu/dds-domain-bridge
    accessed: 2026-08-13
  - path: vehicle_simulator_model/ubuntu/dds_env.sh
    accessed: 2026-08-13
  - path: vehicle_simulator_model/ubuntu/compose.yaml
    accessed: 2026-08-13
  - path: artifacts/vehicle/sources/hiwonder-mentorpi-getting-ready-implementation-guide.md
    accessed: 2026-08-13
---

# MentorPi Fast DDS Discovery Server와 DDS Domain Bridge

## 범위와 핵심 결론

Fast DDS는 eProsima의 DDS/RTPS 구현체이며, ROS 2에서는 RMW 구현
`rmw_fastrtps_cpp`를 통해 사용한다. 이 프로젝트의 Fast DDS 구성은 여러
MentorPi 차량을 하나의 ROS graph로 무분별하게 합치는 방식이 아니다. 차량마다
서로 다른 DDS Domain을 유지하고, 중앙의 Discovery Server로 discovery를
unicast화하며, `dds-domain-bridge`가 검증된 telemetry만 중앙 관측 Domain으로
단방향 전달한다.

```text
robot_1 ROS 2 Domain 1 ─┐
                         ├─ Fast DDS Discovery Server (중앙 서버 UDP 11811)
robot_2 ROS 2 Domain 2 ─┘
                         │ discovery control traffic
                         ▼
               dds-domain-bridge
           Domain 1 / 2 → central Domain 215
                         │
                         ▼
      odom·scan·imu·TF 관측 및 중앙 ROS 2 서비스
```

Discovery Server는 센서 payload를 프록시하지 않는다. 연결 가능한 endpoint를
찾는 discovery 정보만 교환하며, 매칭된 Publisher와 Subscriber의 실제 DDS
payload는 차량과 bridge/consumer 사이에서 UDPv4로 직접 흐른다. 따라서
서버 UDP 11811만 여는 것으로 충분하지 않고, 차량과 중앙 consumer 사이의 DDS
UDP payload 경로도 허용해야 한다.

## 1. DDS, RTPS, ROS 2, Fast DDS의 관계

DDS(Data Distribution Service)는 분산 publish/subscribe 통신 표준이다. 참여자
(Participant)가 Topic에 대해 DataWriter(Publisher) 또는 DataReader(Subscriber)를
만들고, 이름·타입·QoS가 호환되는 endpoint끼리 자동 매칭된다. RTPS(Real-Time
Publish-Subscribe)는 이 discovery와 데이터 전달에 사용되는 wire protocol이다.

ROS 2는 DDS를 직접 API로 노출하지 않고 RMW(ROS Middleware) 계층으로 감싼다.
이 프로젝트에서 Fast DDS를 사용하려면 ROS 2 process가 다음 구현을 선택해야 한다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`FASTRTPS_*`와 `FASTDDS_*` 환경 변수는 함께 보일 수 있다. Fast DDS의 이전
이름이 Fast RTPS였기 때문에, 호환성을 위해 XML profile 경로에
`FASTRTPS_DEFAULT_PROFILES_FILE`와 `FASTDDS_DEFAULT_PROFILES_FILE`을 모두
설정한다.

### DDS Domain

`ROS_DOMAIN_ID`는 discovery와 통신을 논리적으로 격리하는 DDS Domain 번호다.
같은 Domain에 있는 ROS 2 participant만 서로를 찾는 것이 기본 동작이며, 서로
다른 Domain의 topic은 자동으로 보이지 않는다. 이 프로젝트는 기본적으로
`robot_1=1`, `robot_2=2`, 중앙 관측 Domain=`215`를 사용한다. bridge entrypoint는
각 ID가 0~232의 정수이고 서로 중복되지 않는지 먼저 검증한다.

Domain은 namespace와 다른 경계다.

| 구분 | 역할 | 이 프로젝트의 예 |
| --- | --- | --- |
| DDS Domain | discovery·통신 graph 격리 | 차량별 1, 2 및 중앙 215 |
| ROS namespace | 같은 Domain 안의 이름 충돌 방지 | `/robot_1`, `/robot_2` |
| Domain bridge | 서로 다른 Domain 사이의 명시적 전달 | telemetry Domain 1/2 → 215 |

## 2. Discovery Server가 필요한 이유

일반적인 Simple Discovery는 multicast를 사용해 participant와 endpoint를 찾는다.
같은 평면 LAN에서는 간단하지만, Wi-Fi AP 정책, VLAN/서브넷 경계, VPN, Docker
network, multicast 차단 환경에서는 discovery가 불안정해질 수 있다.

Fast DDS Discovery Server 모드에서는 client가 알고 있는 서버의 IP:port locator로
unicast discovery 정보를 보낸다. 서버는 client의 모든 데이터를 중계하는 대신,
각 client가 실제로 매칭할 endpoint 정보만 전달한다. 이 때문에 discovery traffic은
중앙화되지만, user traffic은 role과 무관하게 endpoint 간 직접 통신한다.

| 역할 | 동작 | 프로젝트 사용처 |
| --- | --- | --- |
| `SERVER` | client discovery 정보를 받고 필요한 정보로 재배포 | `fastdds discovery -i 0 -l 0.0.0.0 -p 11811` |
| 일반 `CLIENT` | 매칭에 필요한 discovery 정보만 수신 | 실차 ROS 2 node, domain bridge, 실제 observer |
| `SUPER_CLIENT` | 서버가 아는 전체 discovery graph를 수신 | `dds-observe topics` 진단 전용 |
| `BACKUP` | discovery DB를 파일에 유지 | 현재 bundle에서는 사용하지 않음 |

`SUPER_CLIENT`는 서버가 아니다. discovery 정보를 다시 배포하지 않으며, 모든
운영 process에 설정하면 불필요한 graph 정보가 노출되고 진단용 역할과 운영 역할이
섞인다. 이 프로젝트는 `dds-observe topics` 실행 때만 임시 XML profile을 만들고,
명령 종료 시 삭제한다. 반면 `dds-observe echo`와 `dds-observe hz`는 일반 client로
실제 선택 Domain의 payload를 구독한다.

## 3. 프로젝트 구현

### Discovery Server와 환경 설정

`dds-domain-bridge/docker-compose.yaml`은 Linux host network에서 두 service를
실행한다.

```yaml
dds-discovery:
  command: [fastdds, discovery, -i, "0", -l, 0.0.0.0, -p, "11811"]

dds-domain-bridge:
  environment:
    RMW_IMPLEMENTATION: rmw_fastrtps_cpp
    ROS_DOMAIN_ID: 215
    DDS_DISCOVERY_HOST: 127.0.0.1
    DDS_DISCOVERY_PORT: 11811
    FASTDDS_BUILTIN_TRANSPORTS: UDPv4
```

bridge container에는 `DDS_DISCOVERY_HOST=127.0.0.1`이 맞다. 같은 Linux host의
Discovery Server에 접속하기 때문이다. 반면 실차의 `ROS_DISCOVERY_SERVER`에는
`127.0.0.1`이나 Docker service 이름을 넣으면 안 된다. 차량에서 도달 가능한 중앙
서버의 **고정 LAN/VPN IPv4 주소**와 port 11811을 사용해야 한다.

`dds_env.sh`는 host 이름을 IPv4로 해석해 `ROS_DISCOVERY_SERVER=<IPv4>:<port>`를
설정한다. Fast DDS가 요구하는 numeric locator를 만들기 위한 처리이며, port 범위와
IPv4 주소도 검증한다. `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`는 내장 transport를
UDPv4로 고정해 Docker shared-memory transport 의존을 피하고, 서로 다른 host의
직접 payload 전달을 명시한다.

### Domain Bridge와 allowlist

`ros-humble-domain-bridge`는 template을 런타임 환경값으로 렌더링한 뒤 실행한다.
wildcard를 허용하지 않고 topic·ROS message type·source Domain·target Domain을
모두 명시한다.

| Topic 계열 | ROS 타입 | 흐름 |
| --- | --- | --- |
| `odom`, `odom_raw` | `nav_msgs/msg/Odometry` | robot_N Domain → 215 |
| `scan`, `scan_raw` | `sensor_msgs/msg/LaserScan` | robot_N Domain → 215 |
| `imu`, `ros_robot_controller/imu` | `sensor_msgs/msg/Imu` | robot_N Domain → 215 |
| `tf`, `tf_static` | `tf2_msgs/msg/TFMessage` | robot_N Domain → 215 |

모든 topic은 `/robot_1/...` 또는 `/robot_2/...` namespace를 사용한다. 실제 차량의
topic 이름이나 type이 다르면 `bridge.yaml.template`에 정확한 항목을 추가하고
bridge를 재생성해야 한다.

`/cmd_vel`, goal, cancel, stop과 같은 제어 topic은 의도적으로 allowlist에 없다.
central Domain은 관측용 telemetry를 받지만, 이 bundle만으로 실차 제어 명령을
되돌려 보내지 않는다. 이는 다중 차량 통합 시 제어 권한과 관측 경로를 분리하는
안전 경계다.

## 4. 기존 MentorPi 기본 DDS 설정과의 관계

선택된 MentorPi Project wiki의 기존 운영 가이드는 `.typerc`에서
`ROS_DOMAIN_ID=0`, `CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml`을 사용하는
기본 구성을 기록한다. 즉, 원본 MentorPi ROS 2 환경은 CycloneDDS 기준이다.

이 문서의 Discovery Server 및 bridge 설계는 Fast DDS 전환 구성이다. Discovery
Server를 사용하려는 participant는 `rmw_fastrtps_cpp`를 선택하고 중앙 서버 locator를
설정해야 한다. CycloneDDS 기본 환경과 Fast DDS Discovery Server 환경을 같은 shell
설정에 무심코 혼합하면 discovery 문제가 발생할 수 있으므로, 차량별 launch/service
환경에서 RMW와 DDS 관련 변수를 명시적으로 고정한다.

## 5. 기동·관측·장애 점검

중앙 Linux 서버에서 standalone bundle을 시작한다.

```bash
cd vehicle_simulator_model/ubuntu/dds-domain-bridge
cp .env.example .env
docker compose up -d --build
docker compose ps
docker compose logs -f dds-discovery dds-domain-bridge
```

관측은 목적에 따라 구분한다.

```bash
# Discovery Server가 아는 전체 graph: Super Client 진단
docker compose exec dds-domain-bridge dds-observe topics

# Domain 1에서 실제 payload 1건 수신: 일반 DDS subscription
docker compose exec -e DDS_OBSERVE_DOMAIN_ID=1 \
  dds-domain-bridge dds-observe echo /robot_1/odom_raw

# Domain 1 LiDAR 수신 주기 확인
docker compose exec -e DDS_OBSERVE_DOMAIN_ID=1 \
  dds-domain-bridge dds-observe hz /robot_1/scan_raw
```

`topics` 결과는 source Domain filter를 적용하지 않는 Discovery Server 전체 graph다.
따라서 "차량이 Discovery Server에 등록됐는지" 확인할 때 사용한다. telemetry가
정말 도착하는지는 source Domain을 지정한 `echo` 또는 `hz`로 확인해야 한다.

### 점검 순서

1. 각 실차에서 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, 고유한 `ROS_DOMAIN_ID`,
   중앙 서버 IP 기반 `ROS_DISCOVERY_SERVER`, `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`를
   확인한다.
2. 중앙 서버에서 Discovery Server가 UDP 11811을 listen하는지와 bridge 로그의
   Domain ID가 의도한 값인지 확인한다.
3. `dds-observe topics`로 discovery 등록을 확인한다.
4. 각 source Domain에서 `echo`/`hz`로 `odom_raw`, `scan_raw` 등 payload를 실제로
   확인한다.
5. source topic 이름·type이 allowlist와 다르면 template을 수정하고 bridge만
   재생성한다.

## 6. 운영 제약과 보안

- Linux Docker의 host network를 운영 전제로 한다. Docker Desktop의 host networking은
  같은 방식으로 동작하지 않으므로 실차 운영 검증 환경이 아니다.
- 중앙 서버 UDP 11811 및 DDS UDP payload 통신은 차량이 있는 신뢰된 LAN/VPN에만
  허용한다. public Internet에 노출하지 않는다.
- Discovery Server는 단일 장애점이 될 수 있다. 서버가 중단되면 새로운 discovery와
  재매칭에 영향을 줄 수 있으므로 서비스 health와 재시작 정책을 운영 항목으로 둔다.
- Discovery Server가 payload relay라는 가정으로 방화벽을 열거나 용량을 산정하면
  안 된다. 실제 payload의 source/destination은 bridge와 개별 차량 endpoint다.
- central Domain 215는 관측 통합용이다. 제어 topic을 추가하려면 권한, watchdog,
  emergency stop, QoS, 차량별 route를 별도 설계·검증해야 한다.

## 코드 근거와 검증 상태

다음은 정적 분석으로 확인했다.

- standalone compose는 `dds-discovery`와 `dds-domain-bridge`만 실행하고, 둘 다
  host network를 사용한다.
- Discovery Server는 ID 0, UDP 11811에서 시작한다.
- bridge는 central 215, robot_1 1, robot_2 2를 기본값으로 사용하며 중복 Domain을
  거부한다.
- bridge template은 16개 telemetry topic을 명시적으로 단방향 전달하며 제어 topic을
  포함하지 않는다.
- observer는 전체 graph 진단에만 임시 Super Client profile을 사용하고, 종료 시
  profile을 삭제하도록 구현돼 있다.

Docker Engine이 있는 중앙 Linux 서버 및 실제 MentorPi와의 end-to-end payload
전달은 이 문서 작성 시 실행 검증하지 않았다. 방화벽, 차량 설정, DDS QoS 호환성,
실제 topic 이름·type은 배포 전 위 점검 절차로 검증해야 한다.
