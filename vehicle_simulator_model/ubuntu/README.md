# MentorPi Headless Docker Bundle

MentorPi Gazebo Harmonic 시뮬레이션을 Docker Compose로 운영하는 headless 번들이다.
서버에는 이미지와 Compose 파일만 배포하며 실행 중인 서비스에 ROS 소스를 bind mount하지
않는다. 기본 렌더러는 Mesa 소프트웨어 렌더링이고, GPU 장치는 명시적인 `gpu` profile에서만
전달한다.

## 이미지 참조와 개발 빌드

Mac에서는 Docker 컨테이너 GUI 대신 네이티브 Gazebo GUI 개발 환경을 사용한다. 이 번들은
MentorPi 서버에서 센서와 물리 시뮬레이션을 운영하기 위한 `linux/amd64` 이미지다.

Mac Docker Desktop에서 `scripts/gz-gui-connect.sh`의 Gazebo Transport preflight가 `exit 4`로
끝나면 direct Gazebo GUI transport는 **UNSUPPORTED**다. 이 결과는 내부 네트워크와 외부 포트
비공개 계약을 바꾸어 우회하지 않는다. Mac Docker Desktop에서 native `gz sim -g` transport를
사용할 수 없으므로 Task 4 browser viewer 제공 후 이를 사용한다.

모든 launcher 명령은 명시적인 named profile을 요구한다. profile 값은 이미 export된 환경
변수보다 우선하며, launcher는 legacy bare `.env`를 읽지 않는다. 개발 PC에서는
`.env.dev.example`을 `.env.dev`로 복사한다. 모든 명령은 `MENTORPI_IMAGE` 하나를 이미지
reference, Compose의 `IMAGE_VERSION` 로그 값으로 공유한다. 기본값은 Task 5와 호환되는
`mentorpi-sim:harmonic`이다.

```bash
# Development PC
cp .env.dev.example .env.dev
./run.sh --env dev build
./run.sh --env dev test
./run.sh --env dev sim-up

# Linux server
cp .env.server.example .env.server
# Edit GZ_SERVER_IP to this server's LAN or VPN IP.
./run.sh --env server sim-up

# Dedicated browser viewer stack
cp .env.server-viewer.example .env.server-viewer
./run.sh --env server-viewer viewer-up local
```

```bash
cd vehicle_simulator_model/ubuntu
cp .env.dev.example .env.dev
./run.sh --env dev build
./run.sh --env dev test
```

`build`는 위 reference로 `docker build --platform linux/amd64`를 실행하고 이미지 안의
`/opt/mentorpi_ws`에 ROS 패키지를 빌드한다. Compose 파일에는 `build:`가 없으므로 `sim-up`과
`test`는 절대로 암묵적으로 소스를 빌드하지 않으며, 방금 build한 동일한 reference를 사용한다.

추가 환경은 launcher를 수정하지 않고 profile 파일로 만든다. 예를 들어 `.env.dev1`은
`./run.sh --env dev1 <command>`로 선택하며, `.env.*` 규칙에 따라 Git에서 무시된다. 선택한
profile의 값은 상속된 export보다 우선한다.

서버 배포에서는 `.env.server`의 `MENTORPI_IMAGE`를 CI가 만든 명시적 tag 또는 digest로
수정하고, 그 정확한 reference를 `docker pull`로 pull한다. 선택한 profile이 상속된 shell
환경보다 우선하므로 `MENTORPI_IMAGE`를 export해서 배포 이미지를 바꾸지 않는다.

```bash
cp .env.server.example .env.server
# Edit MENTORPI_IMAGE to registry.example.com/mentorpi-sim:2026.07.26 or a digest.
docker pull registry.example.com/mentorpi-sim:2026.07.26
./run.sh --env server sim-up
```

버전 tag는 레지스트리에서 다른 이미지로 이동할 수 있으므로 그 자체로 불변하지 않다. digest
reference만 내용 불변성을 제공한다. 이 번들의 운영 불변성은 Compose가 source bind mount나
build context를 갖지 않아 배포 서버에서 소스를 재빌드하지 않는 범위까지다. digest로 운영하는
서버에서는 `./run.sh --env server build`를 실행하지 말고, 검증된 digest를 pull하여 사용한다.

## 실행 구조와 케이스 선택

### 서비스별 실행 범위

| 서비스 | 실행 위치 | 역할 | 단독 실행 여부 |
| --- | --- | --- | --- |
| `dds-discovery` | Docker 내부 | `sim-adapter`와 `slam-mapper` 사이의 ROS 2 discovery | 일반 운영에서는 `run.sh`가 함께 실행 |
| `gazebo-server` | Docker 내부 | 물리·센서 시뮬레이션과 오프스크린 렌더링 | 가능하지만 ROS sensor topic은 생성하지 않음 |
| `sim-adapter` | Docker 내부 | robot spawn, Gazebo–ROS bridge, odom 변환, TF 발행 | Gazebo와 Discovery Server가 먼저 준비돼야 함 |
| `slam-mapper` | Docker 내부 | scan·odom·TF를 사용한 지도 생성과 rosbag 기록 | 단독 실행 금지, `mapping-up`으로만 시작 |
| `slam-inspector` | Docker 내부 | 저장된 지도 세션과 checksum을 read-only로 검증 | `mapping-status`가 필요할 때만 one-shot 실행 |
| Gazebo GUI·URDF viewer | 개발 PC | world·모델 작성 및 시각 확인 | Docker 서비스와 분리된 개발 도구 |

`sim-adapter만` 실행해도 센서 데이터의 원본인 Gazebo가 없으면 정상 상태가 될 수 없다.
따라서 일반 시뮬레이션은 `sim-up`, 지도 생성은 `mapping-up`을 진입점으로 사용한다.
`slam-mapper`는 실행 중인 adapter와 container ID를 공유하지 않으며 독립적으로 통신하지만,
유효한 입력 데이터가 있어야 지도 세션을 publish할 수 있다.

모든 Docker 명령은 다음 디렉터리에서 실행한다.

```bash
cd vehicle_simulator_model/ubuntu
```

서비스 기준 빠른 명령은 다음과 같다.

| 목적 | 명령 | 비고 |
| --- | --- | --- |
| Gazebo server만 진단 | `docker compose up -d gazebo-server` | world만 실행되며 robot spawn·ROS topic은 없음 |
| Gazebo + sim adapter | `./run.sh --env dev sim-up` | Discovery Server까지 포함하는 권장 진입점 |
| sim adapter 재시작 | `docker compose restart sim-adapter` | Gazebo와 mapper는 유지 |
| SLAM mapper 시작 | `./run.sh --env dev mapping-up <session-id>` | 필요한 시뮬레이션 서비스도 함께 시작 |
| mapper 로그 | `docker compose --profile mapping logs -f slam-mapper` | `Ctrl+C`는 log follow만 종료 |
| mapper 안전 종료 | `./run.sh --env dev mapping-stop` | 지도 저장과 atomic publish 수행 |
| 지도 검증 | `./run.sh --env dev mapping-status <session-id>` | named volume을 read-only로 검사 |
| 자율주행 시작 | `./run.sh --env dev nav-up auto [session-id]` | 저장 지도가 있으면 AMCL, 없으면 SLAM으로 기동 |
| 자율주행 상태 | `./run.sh --env dev nav-status` | 선택 모드와 Nav2 토픽 endpoint 확인 |
| 자율주행 종료 | `./run.sh --env dev nav-down` | Nav2·목표점 bridge·속도 relay만 정지 |

`docker compose up -d sim-adapter`도 Compose dependency에 의해 Gazebo와 Discovery Server를
함께 시작한다. 다만 운영 절차와 GPU profile 처리를 일관되게 유지하려면 `./run.sh --env dev sim-up`을
사용한다. mapper는 `docker compose up`으로 직접 시작하지 않는다.

### Nav2 자율주행: 저장 지도 우선, SLAM 자동 전환

`nav-up auto`는 `/slam-data`의 지도 세션을 먼저 checksum과 manifest로 검증한다. 유효한 세션이 있으면 `map_server + AMCL + Nav2`를 실행하고, 지정한 세션이 없거나 유효하지 않으면 `slam_toolbox + Nav2`로 전환한다. 두 모드가 동시에 `map → robot_1/odom` TF를 publish하지 않도록, 실행 중인 `slam-mapper`는 먼저 `mapping-stop`으로 안전 종료해야 한다.

```bash
# 기본: 가장 최근의 유효 세션으로 localization
./run.sh --env dev nav-up auto

# 특정 세션으로 localization. 세션이 없거나 훼손되면 SLAM 모드로 전환
./run.sh --env dev nav-up auto warehouse-20260727-01
./run.sh --env dev nav-status
```

Foxglove는 `ws://localhost:8765/`에 연결하고 3D panel의 Fixed frame을 `map`으로 선택한다. 저장 지도 모드에서는 먼저 `Publish` panel에서 `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`, frame_id=`map`)로 로봇의 대략적인 현재 위치와 heading을 한 번 지정한다. 그 다음 `Publish` panel에서 `/move_base_simple/goal` (`geometry_msgs/PoseStamped`, frame_id=`map`)을 발행하면 bridge가 Nav2 `/navigate_to_pose` Action으로 전달한다. Nav2의 `/cmd_vel_nav`은 watchdog relay를 거쳐 `/robot_1/controller/cmd_vel`로 전달되며, 명령이 끊기면 0.35초 후 정지 명령을 보낸다.

SLAM fallback은 즉시 탐색·주행을 위한 임시 지도 모드다. 재사용할 지도가 필요하면 `nav-down` 후 `mapping-up <session-id>`와 `mapping-stop`으로 별도 세션을 저장하고, 다시 `nav-up auto <session-id>`를 실행한다.

### 케이스 1 — 개발 PC: 네이티브 Gazebo GUI

목적은 world·로봇 외형·센서 배치 같은 시각 모델을 작성하고 확인하는 것이다. Mac을 포함한
개발 PC에서는 Gazebo Harmonic을 네이티브로 설치하고 Docker GUI를 사용하지 않는다.

macOS에서는 Gazebo server와 GUI를 한 process에서 실행할 수 없다. `.env.dev`에 저장된
`GZ_IP`, `GZ_PARTITION`, `GZ_SIM_RESOURCE_PATH`를 launcher가 읽도록 두 터미널에서 각각
명령을 실행한다.

```bash
# 한 번만: 프로젝트 bundle 디렉터리에서 수행
cd vehicle_simulator_model/ubuntu
cp .env.dev.example .env.dev

# Terminal 1: world를 읽는 Gazebo server
./run.sh --env dev gz-server

# Terminal 2: server에 연결하는 Gazebo GUI
./run.sh --env dev gz-gui
```

`-s`는 server-only, `-r`은 simulation 즉시 시작, `-g`는 GUI-only 옵션이다. 두 터미널 중
server를 먼저 실행하고 실행 중인 상태로 유지해야 한다. GUI가 빈 화면이면 두 터미널의
`GZ_IP`·`GZ_PARTITION` 값이 같은지와 server가 종료되지 않았는지 확인한다.
`.env.dev`의 `GZ_IP=127.0.0.1`은 같은 Mac 안의 두 process만 연결하고 잘못 선택된 network
interface의 multicast 오류를 피하기 위한 설정이다. `GZ_SIM_RESOURCE_PATH`는 새 창고
컨베이어·랙·충전기·표식 모델을 찾기 위한 source 모델 경로다.

이 실행은 world 파일만 열기 때문에 두 MentorPi robot은 자동으로 생성되지 않는다. 현재 robot
spawn은 ROS 2 `sim-adapter` launch가 담당한다. macOS GUI는 일부 plugin이 불안정할 수 있으므로
world·camera 시각 확인 범위로 사용한다. 자세한 제약은
[Gazebo Harmonic macOS 실행 안내](https://gazebosim.org/docs/harmonic/getstarted/#macos)를
참고한다. 로봇 URDF만 확인할 때는 프로젝트에 설치된 `urdf-viz-large-mesh`를 사용한다.

```bash
urdf-viz-large-mesh \
  ros2_ws/src/mentorpi_description/urdf/mecanum_forklift.urdf \
  --axis-scale 0.01 \
  --web-server-port 7778 \
  --package-path mentorpi_description="$PWD/ros2_ws/src/mentorpi_description"
```

ROS 2 Humble과 workspace를 네이티브로 실행할 수 있는 Ubuntu 개발 PC에서는 전체 시뮬레이션을
headless server와 GUI client로 나눌 수 있다.

```bash
# Terminal 1: ROS 2 + Gazebo server + adapter
export GZ_IP=127.0.0.1
export GZ_PARTITION=mentorpi-sim
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch mentorpi_gz_sim two_robot_sim.launch.py

# Terminal 2: 같은 PC의 Gazebo GUI
export GZ_IP=127.0.0.1
export GZ_PARTITION=mentorpi-sim
gz sim -g
```

Mac 개발 PC에서는 ROS 2 Humble 전체 실행을 전제로 하지 않는다. Mac에서는 world와 URDF를
시각적으로 수정하고, ROS bridge·SLAM 통합 검증은 아래 Docker 통합 테스트에서 수행한다.

### 케이스 2 — 개발 PC: Docker 통합 테스트

서버 환경을 아직 준비하지 않았거나 변경사항을 서버로 전달하기 전에 검증할 때 사용한다.
Mac Docker Desktop에서도 실행할 수 있지만 `linux/amd64` emulation과 소프트웨어 렌더링을
사용하므로 서버보다 느릴 수 있다. 컨테이너의 Gazebo GUI 화면은 제공하지 않는다.

```bash
cp .env.dev.example .env.dev
./run.sh --env dev build
./run.sh --env dev test
./run.sh --env dev sim-up
docker compose ps
./run.sh --env dev topics
./run.sh --env dev fork-up
./run.sh --env dev down
```

`sim-up`은 `dds-discovery`, `gazebo-server`, `sim-adapter`를 함께 시작한다. `topics`에서
`/clock`, `/tf`, `/robot_1/scan_raw`, `/robot_1/odom`, `/robot_2/scan_raw`,
`/robot_2/odom` 등이 확인되면 ROS adapter가 실제 payload를 받고 있는 상태다.

개발 PC Docker에서도 지도 생성까지 검증하려면 서버와 동일한 SLAM 절차를 사용한다.

```bash
SESSION_ID=dev-map-20260727-01
./run.sh --env dev mapping-up "$SESSION_ID"
docker compose --profile mapping logs -f slam-mapper
# 주행 또는 시뮬레이션 데이터 수집 후 Ctrl+C로 log follow만 종료
./run.sh --env dev mapping-stop
./run.sh --env dev mapping-status "$SESSION_ID"
```

### 케이스 3 — 서버 PC: 시뮬레이션만 실행

서버에서는 검증된 이미지 tag 또는 digest를 pull한 뒤 headless로 실행한다. 서버에서 source를
bind mount하거나 다시 빌드하지 않는다.

```bash
cd vehicle_simulator_model/ubuntu
cp .env.server.example .env.server
# .env.server의 MENTORPI_IMAGE를 registry.example.com/mentorpi-sim@sha256:<digest>로 편집
docker pull registry.example.com/mentorpi-sim@sha256:<digest>

./run.sh --env server sim-up
docker compose ps
./run.sh --env server topics
./run.sh --env server logs
```

`./run.sh --env server logs`는 `dds-discovery`, `gazebo-server`, `sim-adapter` 로그를 follow한다. 로그 확인을
끝낼 때 누르는 `Ctrl+C`는 log follow만 종료하며 background 서비스는 계속 실행된다.

운영을 종료할 때는 다음 명령을 사용한다.

```bash
./run.sh --env server down
```

서버가 native Ubuntu이고 `/dev/dri/renderD*`를 사용할 수 있을 때만 GPU profile을 선택한다.

```bash
./run.sh --env server sim-up gpu
```

### 케이스 4 — 서버 PC: SLAM 지도 생성

지도 생성은 일반 `sim-up` 대신 고유한 session ID와 함께 `mapping-up`으로 시작한다.
`mapping-up`이 Discovery Server, Gazebo, adapter, mapper의 의존 순서를 처리하므로 먼저
`sim-up`을 실행할 필요가 없다.

```bash
cd vehicle_simulator_model/ubuntu
# .env.server의 MENTORPI_IMAGE를 검증된 tag 또는 digest로 편집

SESSION_ID=warehouse-20260727-01
./run.sh --env server mapping-up "$SESSION_ID"
docker compose --profile mapping ps
docker compose --profile mapping logs -f slam-mapper
```

필요한 데이터 수집이 끝나면 `down`이 아니라 반드시 `mapping-stop`으로 종료하고 결과를
검증한다.

```bash
./run.sh --env server mapping-stop
./run.sh --env server mapping-status "$SESSION_ID"
```

`mapping-stop` 성공 후 지도, posegraph, rosbag, manifest, checksum은
`mentorpi-slam-data` named volume의 `/slam-data/<session-id>/`에 저장된다. 실패한 세션은
`.inprogress/<session-id>`로 남으며 같은 ID로 재시작하지 않는다.

### 케이스 5 — 개발 PC에서 서버 PC 운영

개발 PC에서 서버에 SSH로 접속해 같은 서버 명령을 실행할 수 있다. SSH는 명령 실행과 로그
확인을 위한 경로이며 Gazebo GUI 전달 경로가 아니다.

```bash
ssh <server-user>@<server-host>
cd <deploy-path>/vehicle_simulator_model/ubuntu
# .env.server의 MENTORPI_IMAGE를 검증된 tag 또는 digest로 편집하고 해당 reference를 docker pull
./run.sh --env server sim-up
./run.sh --env server topics
./run.sh --env server logs
```

개발 PC에서 만든 이미지를 서버에 전달하는 권장 흐름은 다음과 같다.

```bash
# 개발 PC 또는 CI: .env.release를 만들고 MENTORPI_IMAGE를 registry.example.com/mentorpi-sim:<release-tag>로 편집
cp .env.dev.example .env.release
./run.sh --env release build
./run.sh --env release test
docker push registry.example.com/mentorpi-sim:<release-tag>

# 서버 PC
# .env.server의 MENTORPI_IMAGE를 같은 release-tag로 편집
docker pull registry.example.com/mentorpi-sim:<release-tag>
./run.sh --env server sim-up
```

현재 Compose는 `mentorpi` network를 `internal: true`로 만들고 Gazebo Transport, ROS DDS,
웹 포트를 host에 publish하지 않는다. 따라서 개발 PC의 `gz sim -g`를 서버 Docker의
Gazebo에 직접 연결하거나 브라우저에서 Gazebo 화면을 보는 기능은 현재 지원하지 않는다.
원격 웹 모니터링이 필요하면 read-only 영상·상태 bridge와 인증·접근 제어를 별도 서비스로
추가해야 한다.

### adapter 재시작과 mapper 복구

일반 adapter process 재시작은 다음처럼 수행한다.

```bash
docker compose restart sim-adapter
docker compose ps
./run.sh --env server topics
```

container 자체를 교체하는 배포·장애 조건을 재현하려면 다음 명령을 사용한다.

```bash
docker compose up -d --no-deps --force-recreate sim-adapter
docker compose ps
./run.sh --env server topics
```

active mapping 중에도 mapper container는 유지되고 새 adapter participant를 다시 발견한다.
지도 저장 전에는 adapter가 `healthy`로 복구됐는지 확인하고 `mapping-stop`을 실행한다.
복구에 30초 이상 필요하면 대기 시간을 명시적으로 늘릴 수 있다.

```bash
MAPPING_RECONNECT_TIMEOUT_SECONDS=60 ./run.sh --env server mapping-stop
```

`dds-discovery`를 단독으로 강제 재생성하면 내부 IP가 바뀔 수 있다. Discovery Server를
교체해야 할 때는 active mapping을 먼저 안전하게 종료한 후 전체 stack을 `down`/`sim-up`
순서로 계획 재시작한다. `slam-mapper`가 비정상 종료된 세션은 같은 session ID로 재시작하지
않고 `.inprogress` 자료를 보존한 채 새 ID로 시작한다.

## 서버 운영

Docker Engine 및 Docker Compose 2.24.4 이상이 설치된 Linux 서버에서 실행한다.
`viewer-up`은 이 최소 버전을 preflight하고, 미지원 버전에서는 서비스를 변경하기 전에
nonzero로 종료한다.

```bash
./run.sh --env server sim-up
./run.sh --env server logs
./run.sh --env server topics
./run.sh --env server fork-up
./run.sh --env server down
```

위 명령은 `MENTORPI_IMAGE`가 가리키는 동일한 이미지를 사용한다. 기본 local reference가 없는
서버에서는 `.env.server`에 설정한 정확한 tag 또는 digest를 먼저 `docker pull`한다.

`--env server` profile의 `./run.sh --env server sim-up`은 LAN profile이다. `dds-discovery`, `gazebo-server`,
`sim-adapter`와 mapping support 서비스는 affected simulation services로서 host networking을
사용하며, `GZ_SERVER_IP`는 Docker host의 LAN 또는 VPN 주소여야 한다. Gazebo 서버
healthcheck가 통과한 뒤 adapter가 시작하며, Gazebo 서비스는
`GZ_PARTITION=mentorpi-sim`을 공유한다.
서버 health는 진행 중인 stats payload 2개를, adapter health는 양 robot의 scan·odom
payload, robot별 odom-to-base TF, 연속 증가하는 `/clock`을 확인한다. topic 이름만 존재하는
상태는 healthy가 아니다.

ROS 2 discovery는 전용 `dds-discovery` 서비스가 담당한다. host-network containers인
`sim-adapter`와 `slam-mapper`는 `DDS_DISCOVERY_HOST=127.0.0.1`로 연결하므로 discovery
control traffic은 host loopback으로만 흐르며, Fast DDS payload transport는
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`로 고정한다. `dds-discovery`는
`restart: unless-stopped`로 운영한다.

`GZ_SERVER_IP`로 구성한 LAN Gazebo Transport 노출은 host firewall에서 신뢰된 개발자 CIDR로
제한한다. 이 LAN/GZ transport를 공용 인터넷에 공개하지 않으며, firewall 정책을 바꾸지 않고
Docker 내부 네트워크라는 전제로 노출을 판단해서는 안 된다.

`./run.sh --env server fork-up`은 실행 중인 `sim-adapter`가 healthy일 때만 10초 제한 안에서 fork
command를 publish한다. 서비스가 없거나 unhealthy면 새 container를 만들지 않고 실패한다.

실행 중인 adapter의 ROS topic을 확인할 때는 `./run.sh --env server topics`를 사용한다. Compose
`exec`는 entrypoint가 source한 shell 환경을 상속하지 않으므로 이 명령과 adapter
healthcheck는 DDS helper와 ROS setup을 각각 source한 shell에서 ROS CLI를 실행한다.
Discovery Server v2의 일반 client는 필요한 endpoint만 전달받기 때문에 전체 graph 조회에는
부족하다. `topics`는 진단 프로세스만 임시 Super Client로 구성하고 daemon을 사용하지 않아
전체 topic graph를 조회한다. 실제 adapter와 mapper는 일반 client 구성을 유지한다.

```bash
docker compose exec sim-adapter bash -lc \
  'export DDS_SUPER_CLIENT=1
   source /usr/local/bin/mentorpi-dds-env
   source /opt/ros/humble/setup.bash
   source /opt/mentorpi_ws/install/setup.bash
   ros2 topic list --no-daemon'
```

서버 GPU를 사용할 때만 다음처럼 명시한다.

```bash
./run.sh --env server sim-up gpu
```

이 profile은 Gazebo 서버에 `/dev/dri`를 전달하고 `LIBGL_ALWAYS_SOFTWARE=0`으로 바꾼다.
`run.sh`는 native Linux에서 readable `/dev/dri/renderD*`를 선택하고 numeric render GID를
Compose `group_add`에 전달한다. Mac과 DRI render node가 없는 Linux에서는 GPU mode가
Docker 실행 전에 실패한다. 기본 profile은 `LIBGL_ALWAYS_SOFTWARE=1`이므로 GPU 장치가
없어도 운영할 수 있다.

native Ubuntu GPU smoke test는 release gate다. Ubuntu release 후보에서 `./run.sh --env server sim-up gpu`
실행 후 양 서비스 health, 양 robot scan payload, Gazebo 렌더 로그를 확인해야 한다. 이 검증은
Mac Docker Desktop에서 대체할 수 없다.

## 공유 관찰 운영

`--env server`는 LAN 모드이며, affected simulation services에 host networking을 적용한다.
`--env dev`와 `--env server-viewer`만 Docker 내부 모드로 시뮬레이션과 ROS adapter를
운영한다. 이 두 profile에서 simulation services는 internal `mentorpi` bridge network를 사용하고,
DDS client는 Docker DNS의 `dds-discovery`를 locator로 해석한다. browser viewer의
`web-gateway`는 별도의 viewer edge exposure 정책을 유지한다. 다음 표는 지원하는 운영 조합과
각각의 개발 PC 접속 방법이다.

| 목적 | 서버 실행 | 개발 PC 접속 |
| --- | --- | --- |
| Headless 통합 검증 | `./run.sh --env dev sim-up` | `topics`, logs, healthcheck |
| 같은 LAN 네이티브 GUI | `.env.server`의 `GZ_SERVER_IP` 설정 후 `./run.sh --env server sim-up` | `scripts/gz-gui-connect.sh <server-lan-ip> <client-lan-ip>` |
| 로컬 브라우저 viewer | `./run.sh --env server-viewer viewer-up local` | `http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1` |
| 외부 팀 viewer | `.env.server-viewer`에 viewer 값을 설정 후 `./run.sh --env server-viewer viewer-up public` | `https://<VIEWER_DOMAIN>/vnc.html?view_only=1&autoconnect=1` |
| 지도 생성 | `./run.sh --env dev mapping-up <session-id>` | logs와 `mapping-status` |

### Linux LAN 네이티브 GUI

이 모드는 신뢰된 LAN에서만 사용한다. Linux 서버 host firewall은 승인된 개발자 CIDR만
허용해야 하며, raw Gazebo Transport를 인터넷이나 신뢰되지 않은 네트워크에 노출해서는 안 된다.

#### 서버 `.env.server` 구성

Linux 시뮬레이션 서버에서는 `.env.server.example`을 복사해 LAN 또는 VPN 설정을 서버에
영속화한다. bare `.env`는 legacy local-only 파일로 Git에는 무시되지만 launcher가 읽지 않는다.

```bash
cd vehicle_simulator_model/ubuntu
cp .env.server.example .env.server
# Edit GZ_SERVER_IP to this server's LAN or VPN IP.
./run.sh --env server sim-up
```

`.env.server`의 각 설정은 `NAME=value` 문법이어야 한다. 빈 줄과 `#`으로 시작하는 주석은 허용되지만,
`export NAME=value` 또는 `NAME = value` 같은 형식은 허용되지 않는다. 값은 shell처럼 해석하지
않고 그대로 읽으므로, 인용부호가 필요 없는 주소와 이미지 태그를 사용한다. profile 값은 이미
export된 환경 변수보다 우선하므로, 설정 변경은 해당 `.env.<profile>` 파일을 편집해 수행한다.

서버와 GUI client가 모두 같은 LAN에 있고 각 client가 해당 LAN 주소를 명시할 때 다음처럼 실행한다.

```bash
# Linux server: .env.server에서 GZ_SERVER_IP=192.168.50.10으로 편집
./run.sh --env server sim-up

# Mac A
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.20

# Mac B
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.21
```

GUI client는 서버의 `.env.server`에 있는 주소를 재사용하지 않는다. 각 client는 `gz-gui-connect.sh`의
두 번째 인수로 그 client 자신의 실제 LAN 또는 VPN 주소를 전달해야 한다.

두 GUI client는 같은 Gazebo world에 동시에 접속한다. 모든 GUI 창을 닫아도 simulation은
서버에서 계속 실행되며, 중지는 서버에서 `./run.sh --env server down`으로만 수행한다. Mac Docker Desktop의
preflight가 `exit 4`이면 이 raw transport 경로는 UNSUPPORTED이므로 아래 local 또는 public
browser viewer로 전환한다.

### Read-only browser viewer

browser viewer는 `.env.server-viewer.example`을 복사한 전용 profile로 운영한다. 이 profile의
`COMPOSE_PROJECT_NAME=mentorpi-server-viewer`와 `SIM_NETWORK_MODE=internal` 설정은 browser viewer가
실행 중인 LAN stack에 붙지 않고 discovery, simulation, adapter, viewer를 포함한 자체 internal stack을
시작하도록 분리한다. LAN 네이티브 GUI는 계속 `.env.server`와 `--env server`를 사용한다.
local viewer는 서버 자신의 브라우저에서만 접속하도록 loopback에 바인드한다.

```bash
cp .env.server-viewer.example .env.server-viewer
./run.sh --env server-viewer viewer-up local
# http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1
```

외부 팀용 public 모드는 application auth나 basic auth를 제공하지 않는다. 허용한 source CIDR와
Linux host firewall만 접근 경계이며, 허용 CIDR 외 요청은 HTTP 403을 받는다. public 모드의 strict
입력 검증을 거치는 유일한 지원 운영 진입점은
`./run.sh --env server-viewer viewer-up public`이다.
`docker compose` 직접 호출로 public viewer를 올리는 것은 지원하지 않는다.

```bash
# Edit .env.server-viewer: VIEWER_DOMAIN=sim.example.com
# Edit .env.server-viewer: VIEWER_ALLOW_CIDRS=203.0.113.10/32 203.0.113.11/32
./run.sh --env server-viewer viewer-up public
```

Router/NAT는 public 80과 443만 Linux 서버로 전달한다. Caddy는 ACME redirect/challenge에 80을
사용하고 viewer는 443에서 제공한다. 동적으로 바뀌는 팀 IP는 allowlist를 갱신하거나, 별도의
인증된 access method를 선택해야 한다. `0.0.0.0/0`와 `::/0`은 거부된다.

noVNC 6080, VNC 5900, Gazebo Transport, ROS DDS는 절대로 port-forward하지 않는다. 특히
Gazebo Transport를 공용 인터넷에 공개하지 않는다. router의 공개 포트는 80/443으로 제한하고
Linux firewall도 같은 노출 정책을 강제한다.

### 종료, 로그, 복구와 서비스 독립성

```bash
./run.sh --env server-viewer viewer-logs
./run.sh --env server-viewer viewer-down
./run.sh --env server-viewer logs
./run.sh --env server-viewer topics
./run.sh --env server-viewer down
```

`viewer-down`은 `gazebo-viewer`, `web-gateway`, `foxglove-bridge`만 중지한다. Task 6 runtime 검증은 이 viewer
lifecycle 변경이 `gazebo-server`와 `sim-adapter`를 중지시키지 않음을 확인했다. 따라서 이 두
서비스의 viewer 장애 복구는 `viewer-down` 뒤 동일한 local/public 명령으로 viewer만 다시 올린다.
반대로 `--env server-viewer down`은 전용 browser simulation stack을 중지하며, LAN stack의
`--env server down`과 서로 다른 Compose project를 대상으로 한다.

mapping session 환경은 mapper와 inspector에만 전달되며 server, adapter, discovery, viewer에는
전달되지 않는다. 따라서 같은 Compose project와 volume 설정을 유지하면 `viewer-up`이 active
mapping의 simulation service 구성을 변경하지 않고, viewer lifecycle은 실행 중 mapper를 재생성하지 않는다.
지도 생성은 독립적인 one-shot mapper이므로 `mapping-up <session-id>`,
`mapping-stop`, `mapping-status <session-id>`를 사용하며, viewer lifecycle과 묶어서 중지하지
않는다.

### 릴리스 상태

다음 항목은 아직 최종 acceptance 전이며 release blocker다.

- Ubuntu native LAN two-client/visual gate
- official Caddy image의 public direct 80/443 port 검증

## SLAM 매핑 세션 운영

매핑은 일회성 `slam-mapper` 서비스로 실행한다. 세션 ID는 영문 대소문자, 숫자, `.`, `_`, `-`만
사용하며, 같은 ID는 publish된 결과 또는 진행 중인 작업과 재사용할 수 없다.

```bash
./run.sh --env dev mapping-up warehouse-20260726-01
./run.sh --env dev mapping-stop
./run.sh --env dev mapping-status warehouse-20260726-01
```

`mapping-up`은 Discovery Server, Gazebo, adapter가 준비된 뒤 mapper를 시작하고, `SESSION_ID`
및 이미지·world·model·TF 버전 metadata를 mapper에 전달한다. `mapping-status`의 inspector도
검사할 세션 ID를 받지만 다른 runtime service에는 session 환경을 전달하지 않는다. 정상 종료는
`mapping-stop`만 사용한다. adapter가 재시작 중이면 이 명령은
`MAPPING_RECONNECT_TIMEOUT_SECONDS` 동안 health
복구를 기다린다. 복구되면 기존 mapper에 `SIGINT`를 보내 map, posegraph, rosbag, manifest,
checksum finalization이 끝날 때까지 `MAPPING_STOP_TIMEOUT_SECONDS`만큼 기다린다. adapter가
제한 시간 안에 복구되지 않으면 mapper나 지원 서비스를 중지하지 않고 nonzero를 반환하므로,
복구 후 `mapping-stop`을 다시 실행할 수 있다.

mapper lifecycle은 finalization 시작 직전과 atomic publish 직전에 각각 adapter input guard를
실행한다. 이 guard는 양 robot의 scan·odom payload, odom-to-base TF, 연속 증가하는 `/clock`을
검증한다. adapter가 finalization 도중 다시 끊기면 최종 session directory를 publish하지 않고
`.inprogress/<session-id>`를 보존한다. 따라서 adapter container ID 변경 자체는 실패 조건이
아니며, 실제 센서 데이터 복구 여부가 finalization 조건이다. 지도와 posegraph 저장은 graph
목록 조회 결과에 의존하지 않고 명시적 service type으로 직접 호출하며,
`ROS_COMMAND_TIMEOUT_SECONDS` 안에 service가 연결되지 않으면 실패 처리한다.

일반 `down`, Compose recreate, orchestration cleanup이 보내는 `SIGTERM`도 lifecycle에서는 abort 신호다.
`SIGTERM`은 map save나 posegraph serialization을 호출하지 않으며, mapper의 30초 stop grace 안에서 recorder와
SLAM child를 bounded cleanup한 후 nonzero로 끝난다. 따라서 abort된 세션을 이어서 publish하거나 같은 ID로
재사용하지 않는다. 보존된 staging 자료가 필요하면 read-only로 별도 조사·백업하고, 복구 운용은 반드시 새
session ID로 `mapping-up`부터 다시 시작한다.

새 Docker volume의 root는 처음에 `root:root`이므로, 첫 mapping-up은 별도 one-shot initializer를 root로
실행해 `/slam-data` root만 image의 `ros:ros`(uid/gid 1000)로 설정한다. mapper는 계속 non-root로 실행된다.
initializer는 recursive chown을 하지 않으므로 기존 세션 내용을 변경하지 않는다.

mapper는 `ros2 run` wrapper 대신 설치된 lifecycle Bash executable을 직접 command로 사용한다. image
entrypoint가 ROS 환경을 source한 뒤 `exec`하므로 이 Bash가 container PID 1이 되고, `mapping-stop`의
`SIGINT`가 lifecycle finalization trap에 직접 전달된다.

성공한 세션은 Docker named volume `mentorpi-slam-data`의 다음 최종 디렉터리에 원자적으로 publish된다.

```text
/slam-data/<session-id>/
├── map.yaml
├── map.pgm
├── posegraph/mentorpi.posegraph
├── rosbag2/mapping/
├── manifest.json
└── checksums.sha256
```

기본 운영 volume 이름은 `mentorpi-slam-data`다. 기존 운영 자료와 분리된 검증이나 migration을
수행할 때만 `SLAM_VOLUME_NAME`을 명시하고, 한 mapping/viewer lifecycle의 모든 명령에 같은
`COMPOSE_PROJECT_NAME`과 `SLAM_VOLUME_NAME`을 유지한다.

`COMPOSE_PROJECT_NAME`과 `SLAM_VOLUME_NAME`을 별도 profile 파일에 설정한 뒤 그 이름을
선택한다. 예를 들어 `.env.mapping-validation`을 만들면 다음처럼 실행한다.

```bash
./run.sh --env mapping-validation mapping-up validation-01
./run.sh --env mapping-validation viewer-up local
./run.sh --env mapping-validation viewer-down
./run.sh --env mapping-validation mapping-stop
./run.sh --env mapping-validation mapping-status validation-01
```

`mapping-status`는 volume을 read-only로 연결한 inspector 컨테이너에서 이 최종 디렉터리만 나열하고,
그 안에서 `sha256sum -c checksums.sha256`를 실행한다. 따라서 `.inprogress/<session-id>`는 성공으로
표시되지 않는다. 저장·posegraph·bag·checksum 중 하나라도 finalization에 실패하면 결과는
`/slam-data/.inprogress/<session-id>/`에 남으며, 이를 복구된 성공 세션으로 취급하면 안 된다.

volume은 host 경로에 의존하지 않는다. 운영 전후에는 다음처럼 tar archive로 백업하고, 복원할 때는 빈
volume에 archive를 풀어 넣는다.

```bash
docker run --rm -v mentorpi-slam-data:/slam-data:ro -v "$PWD":/backup \
  alpine tar czf /backup/mentorpi-slam-data-backup.tgz -C /slam-data .

# 복원은 volume에 써야 하므로 :ro를 붙이지 않는다.
docker volume create mentorpi-slam-data
docker run --rm -v mentorpi-slam-data:/slam-data -v "$PWD":/backup \
  alpine tar xzf /backup/mentorpi-slam-data-backup.tgz -C /slam-data
```

## 렌더링 경계

서버는 카메라·라이다 등 시뮬레이션 센서에 필요한 오프스크린 렌더링만 수행한다. Docker
bundle의 `viewer` profile은 브라우저 Gazebo viewer와 Foxglove Bridge를 제공하지만, X11,
Xauthority, DISPLAY 또는 원격 GUI 전달은 사용하지 않는다.

### Foxglove Studio 연결

`viewer-up local`은 `foxglove-bridge`를 기존 internal `mentorpi` 네트워크의 Fast DDS discovery
client로 실행한다. Docker Desktop의 loopback 포트 전달을 위해 Bridge는 `viewer-edge`에도
연결하지만 ROS/DDS 서비스는 `mentorpi`에서만 발견한다. Foxglove WebSocket만 host loopback에
공개하며, Foxglove Studio는 Docker 서비스가 아니라 개발 PC의 macOS 앱 또는 브라우저에서
실행한다.

```bash
cp .env.server-viewer.example .env.server-viewer
./run.sh --env server-viewer viewer-up local
# Foxglove Studio에서 Foxglove WebSocket 연결: ws://localhost:8765
```

필요하면 `.env.server-viewer`에서 `FOXGLOVE_PORT`를 변경할 수 있다. public viewer 모드는
Foxglove Bridge 포트를 공개하지 않는다. Gazebo Transport, ROS DDS, VNC/noVNC와 Foxglove
WebSocket을 공용 인터넷에 직접 port-forward하지 않는다.

### Warehouse SDF 3D 장면

`sim-adapter`는 warehouse SDF를 Foxglove의 표준 `SceneUpdate` 토픽으로 함께 발행한다.
Foxglove 3D panel에서 `/warehouse_scene/static`과 `/warehouse_scene/dynamic`을 추가하면
warehouse 구조물, `robot_1`·`robot_2`, 그리고 pallet/payload 상태를 볼 수 있다. 정적 장면은
late-joiner도 받도록 durable QoS로 한 번 발행하며, 동적 장면은 10 Hz로 갱신된다.

두 장면은 `robot_1/odom` 기준이다. SLAM을 함께 실행 중이면 3D panel Fixed frame은 `map`으로,
Gazebo만 실행 중이면 `robot_1/odom`으로 선택한다. `/warehouse/entity_poses`는 Gazebo의
`Pose_V`를 bridge한 내부 입력 토픽이므로 3D panel에 직접 추가할 필요는 없다.

```text
/warehouse_scene/static   # ground, walls, conveyor, rack, charger, markings
/warehouse_scene/dynamic  # robot_1, robot_2, pallet_*, pallet_*_payload
/warehouse/entity_poses   # Gazebo → ROS scene publisher input
```
