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

모든 명령은 `MENTORPI_IMAGE` 하나를 이미지 reference, Compose의 `IMAGE_VERSION` 로그 값으로
공유한다. 기본값은 Task 5와 호환되는 `mentorpi-sim:harmonic`이다.

```bash
cd vehicle_simulator_model/ubuntu
export MENTORPI_IMAGE=mentorpi-sim:harmonic
./run.sh build
./run.sh test
```

`build`는 위 reference로 `docker build --platform linux/amd64`를 실행하고 이미지 안의
`/opt/mentorpi_ws`에 ROS 패키지를 빌드한다. Compose 파일에는 `build:`가 없으므로 `sim-up`과
`test`는 절대로 암묵적으로 소스를 빌드하지 않으며, 방금 build한 동일한 reference를 사용한다.

서버 배포에서는 CI가 만든 명시적 tag 또는 digest를 전달한다.

```bash
export MENTORPI_IMAGE=registry.example.com/mentorpi-sim:2026.07.26
./run.sh sim-up

export MENTORPI_IMAGE='registry.example.com/mentorpi-sim@sha256:<digest>'
./run.sh sim-up
```

버전 tag는 레지스트리에서 다른 이미지로 이동할 수 있으므로 그 자체로 불변하지 않다. digest
reference만 내용 불변성을 제공한다. 이 번들의 운영 불변성은 Compose가 source bind mount나
build context를 갖지 않아 배포 서버에서 소스를 재빌드하지 않는 범위까지다. digest로 운영하는
서버에서는 `./run.sh build`를 실행하지 말고, 검증된 digest를 pull하여 사용한다.

## 서버 운영

Docker Engine 및 Docker Compose v2가 설치된 Linux 서버에서 실행한다.

```bash
./run.sh sim-up
./run.sh logs
./run.sh topics
./run.sh fork-up
./run.sh down
```

위 명령은 `MENTORPI_IMAGE`가 가리키는 동일한 이미지를 사용한다. 기본 local reference가 없는
서버에서는 먼저 해당 reference를 pull하거나, registry tag/digest를 export한다.

`sim-up`은 내부 `mentorpi` 네트워크에서 `dds-discovery`, `gazebo-server`, `sim-adapter`를
시작한다. 외부 Gazebo Transport 포트와 ROS DDS 포트는 공개하지 않는다. Gazebo 서버
healthcheck가 통과한 뒤 adapter가 시작하며, Gazebo 서비스는
`GZ_PARTITION=mentorpi-sim`을 공유한다.
서버 health는 진행 중인 stats payload 2개를, adapter health는 양 robot의 scan·odom
payload, robot별 odom-to-base TF, 연속 증가하는 `/clock`을 확인한다. topic 이름만 존재하는
상태는 healthy가 아니다.

ROS 2 discovery는 전용 `dds-discovery` 서비스가 담당한다. `sim-adapter`와 `slam-mapper`는
각자 독립된 network·IPC namespace를 유지하면서 같은 내부 bridge network에 연결된다. 두
서비스의 공통 DDS helper는 Docker DNS의 `dds-discovery`를 숫자 IPv4 locator로 해석해
`ROS_DISCOVERY_SERVER=<IPv4>:11811`을 export하며, Fast DDS payload transport는
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`로 고정한다. shared memory나
`network_mode: service:sim-adapter`에 의존하지 않으므로 adapter container가 재시작·재생성되어도
mapper container는 그대로 유지되고 새 DDS participant를 다시 발견한다.

`dds-discovery`는 외부 포트를 공개하지 않는 내부 제어 서비스이며 `restart: unless-stopped`로
운영한다. adapter 재시작과 달리 discovery server 자체를 강제로 재생성하면 기존 client가
해석한 내부 IP가 바뀔 수 있으므로, discovery service 교체는 전체 시뮬레이션 stack의 계획된
재시작으로 수행한다.

`./run.sh fork-up`은 실행 중인 `sim-adapter`가 healthy일 때만 10초 제한 안에서 fork
command를 publish한다. 서비스가 없거나 unhealthy면 새 container를 만들지 않고 실패한다.

실행 중인 adapter의 ROS topic을 확인할 때는 `./run.sh topics`를 사용한다. Compose
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
./run.sh sim-up gpu
```

이 profile은 Gazebo 서버에 `/dev/dri`를 전달하고 `LIBGL_ALWAYS_SOFTWARE=0`으로 바꾼다.
`run.sh`는 native Linux에서 readable `/dev/dri/renderD*`를 선택하고 numeric render GID를
Compose `group_add`에 전달한다. Mac과 DRI render node가 없는 Linux에서는 GPU mode가
Docker 실행 전에 실패한다. 기본 profile은 `LIBGL_ALWAYS_SOFTWARE=1`이므로 GPU 장치가
없어도 운영할 수 있다.

native Ubuntu GPU smoke test는 release gate다. Ubuntu release 후보에서 `./run.sh sim-up gpu`
실행 후 양 서비스 health, 양 robot scan payload, Gazebo 렌더 로그를 확인해야 한다. 이 검증은
Mac Docker Desktop에서 대체할 수 없다.

## 공유 관찰 운영

기본 `internal` 모드는 Docker 내부에서 시뮬레이션과 ROS adapter를 운영한다. 다음 표는 지원하는
운영 조합과 각각의 개발 PC 접속 방법이다.

| 목적 | 서버 실행 | 개발 PC 접속 |
| --- | --- | --- |
| Headless 통합 검증 | `./run.sh sim-up` | `topics`, logs, healthcheck |
| 같은 LAN 네이티브 GUI | `SIM_NETWORK_MODE=lan GZ_SERVER_IP=<server-lan-ip> ./run.sh sim-up` | `scripts/gz-gui-connect.sh <server-lan-ip> <client-lan-ip>` |
| 로컬 브라우저 viewer | `./run.sh viewer-up local` | `http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1` |
| 외부 팀 viewer | `VIEWER_DOMAIN=... VIEWER_ALLOW_CIDRS='...' ./run.sh viewer-up public` | `https://<VIEWER_DOMAIN>/vnc.html?view_only=1&autoconnect=1` |
| 지도 생성 | `./run.sh mapping-up <session-id>` | logs와 `mapping-status` |

### Linux LAN 네이티브 GUI

이 모드는 신뢰된 LAN에서만 사용한다. Linux 서버 host firewall은 승인된 개발자 CIDR만
허용해야 하며, raw Gazebo Transport를 인터넷이나 신뢰되지 않은 네트워크에 노출해서는 안 된다.
서버와 GUI client가 모두 같은 LAN에 있고 각 client가 해당 LAN 주소를 명시할 때 다음처럼 실행한다.

```bash
# Linux server
export SIM_NETWORK_MODE=lan
export GZ_SERVER_IP=192.168.50.10
./run.sh sim-up

# Mac A
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.20

# Mac B
./scripts/gz-gui-connect.sh 192.168.50.10 192.168.50.21
```

두 GUI client는 같은 Gazebo world에 동시에 접속한다. 모든 GUI 창을 닫아도 simulation은
서버에서 계속 실행되며, 중지는 서버에서 `./run.sh down`으로만 수행한다. Mac Docker Desktop의
preflight가 `exit 4`이면 이 raw transport 경로는 UNSUPPORTED이므로 아래 local 또는 public
browser viewer로 전환한다.

### Read-only browser viewer

viewer는 시뮬레이션과 독립된 read-only 관찰 서비스다. local 모드는 서버 자신의 브라우저에서만
접속하도록 loopback에 바인드한다.

```bash
./run.sh viewer-up local
# http://127.0.0.1:8080/vnc.html?view_only=1&autoconnect=1
```

외부 팀용 public 모드는 application auth나 basic auth를 제공하지 않는다. 허용한 source CIDR와
Linux host firewall만 접근 경계이며, 허용 CIDR 외 요청은 HTTP 403을 받는다. public 모드의 strict
입력 검증을 거치는 유일한 지원 운영 진입점은 `./run.sh viewer-up public`이다. `docker compose`
직접 호출로 public viewer를 올리는 것은 지원하지 않는다.

```bash
export VIEWER_DOMAIN=sim.example.com
export VIEWER_ALLOW_CIDRS='203.0.113.10/32 203.0.113.11/32'
./run.sh viewer-up public
```

Router/NAT는 public 80과 443만 Linux 서버로 전달한다. Caddy는 ACME redirect/challenge에 80을
사용하고 viewer는 443에서 제공한다. 동적으로 바뀌는 팀 IP는 allowlist를 갱신하거나, 별도의
인증된 access method를 선택해야 한다. `0.0.0.0/0`와 `::/0`은 거부된다.

noVNC 6080, VNC 5900, Gazebo Transport, ROS DDS는 절대로 port-forward하지 않는다. 특히
Gazebo Transport를 공용 인터넷에 공개하지 않는다. router의 공개 포트는 80/443으로 제한하고
Linux firewall도 같은 노출 정책을 강제한다.

### 종료, 로그, 복구와 서비스 독립성

```bash
./run.sh viewer-logs
./run.sh viewer-down
./run.sh logs
./run.sh topics
./run.sh down
```

`viewer-down`은 `gazebo-viewer`와 `web-gateway`만 중지한다. Task 6 runtime 검증은 이 viewer
lifecycle 변경이 `gazebo-server`와 `sim-adapter`를 중지시키지 않음을 확인했다. 따라서 이 두
서비스의 viewer 장애 복구는 `viewer-down` 뒤 동일한 local/public 명령으로 viewer만 다시 올린다.
반대로 `down`은 simulation stack을 중지한다.

실행 중 mapper와 viewer lifecycle의 완전 격리는 설계 목표이지만 release acceptance 전이다.
mapper 실행 중 viewer lifecycle 변경을 운영 보장으로 간주하지 말고, 검증된 maintenance window와
절차에서만 실행한다. 지도 생성은 독립적인 one-shot mapper이므로 `mapping-up <session-id>`,
`mapping-stop`, `mapping-status <session-id>`를 사용하며, viewer lifecycle과 묶어서 중지하거나
재시작하지 않는다.

### 릴리스 상태

다음 항목은 아직 최종 acceptance 전이며 release blocker다.

- Ubuntu native LAN two-client/visual gate
- official Caddy image의 public direct 80/443 port 검증
- 실행 중 mapper가 viewer lifecycle과 완전히 격리되는지의 검증

## SLAM 매핑 세션 운영

매핑은 일회성 `slam-mapper` 서비스로 실행한다. 세션 ID는 영문 대소문자, 숫자, `.`, `_`, `-`만
사용하며, 같은 ID는 publish된 결과 또는 진행 중인 작업과 재사용할 수 없다.

```bash
./run.sh mapping-up warehouse-20260726-01
./run.sh mapping-stop
./run.sh mapping-status warehouse-20260726-01
```

`mapping-up`은 Discovery Server, Gazebo, adapter가 준비된 뒤 mapper를 시작하고, `SESSION_ID`
및 이미지·world·model·TF 버전 metadata를 컨테이너에 전달한다. 정상 종료는 `mapping-stop`만
사용한다. adapter가 재시작 중이면 이 명령은 `MAPPING_RECONNECT_TIMEOUT_SECONDS` 동안 health
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

브라우저 렌더링은 사용자의 클라이언트에서 수행한다. 서버는 카메라·라이다 등 시뮬레이션
센서에 필요한 오프스크린 렌더링만 수행한다. 따라서 서버 Compose 구성에는 X11, Xauthority,
DISPLAY 또는 원격 GUI 전달 설정이 없다.
