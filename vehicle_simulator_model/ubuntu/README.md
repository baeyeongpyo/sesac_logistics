# MentorPi Headless Docker Bundle

MentorPi Gazebo Harmonic 시뮬레이션을 Docker Compose로 운영하는 headless 번들이다.
서버에는 이미지와 Compose 파일만 배포하며 실행 중인 서비스에 ROS 소스를 bind mount하지
않는다. 기본 렌더러는 Mesa 소프트웨어 렌더링이고, GPU 장치는 명시적인 `gpu` profile에서만
전달한다.

## 이미지 참조와 개발 빌드

Mac에서는 Docker 컨테이너 GUI 대신 네이티브 Gazebo GUI 개발 환경을 사용한다. 이 번들은
MentorPi 서버에서 센서와 물리 시뮬레이션을 운영하기 위한 `linux/amd64` 이미지다.

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

`sim-up`은 내부 `mentorpi` 네트워크에서 `gazebo-server`와 `sim-adapter`를 차례로 시작한다.
외부 Gazebo Transport 포트와 ROS DDS 포트는 공개하지 않는다. Gazebo 서버 healthcheck가
통과한 뒤 adapter가 시작하며, 두 서비스는 `GZ_PARTITION=mentorpi-sim`을 공유한다.
서버 health는 진행 중인 stats payload 2개를, adapter health는 양 robot의 scan·odom
payload와 robot별 odom-to-base TF를 확인한다. topic 이름만 존재하는 상태는 healthy가 아니다.

ROS 2 컨테이너 경계는 Fast DDS의 discovery와 shared-memory data path를 함께 보존하도록
project 내부에서만 결합한다. `sim-adapter`는 `ipc: shareable`인 namespace owner이고,
`slam-mapper`는 `network_mode: service:sim-adapter`와 `ipc: service:sim-adapter`로 그
network·IPC namespace에 합류한다. 별도 bridge namespace에서는 publisher discovery가,
network만 공유하고 IPC를 분리하면 실제 sensor payload가 끊길 수 있기 때문이다. 이 설정은
host network/host IPC를 사용하거나 포트를 공개하지 않으며, 신뢰된 mapper 한 서비스만 adapter와
결합한다. `gazebo-server`는 계속 별도 컨테이너로 내부 `mentorpi` network에 연결되고 기존
`GZ_RELAY_HOST=gazebo-server` 경로를 유지한다.

따라서 mapper는 adapter보다 먼저 시작할 수 없고, 종료할 때도 mapper finalization 후 adapter를
정지해야 한다. 배포용 Compose를 확장할 때 mapper의 두 `service:sim-adapter` 설정 중 하나만
제거하지 않는다. 별도 network/IPC 경계가 필요하면 Fast DDS transport/discovery 설정을 명시하고
scan·odom·TF·clock 및 `/map` 실제 payload를 다시 검증해야 한다.

`./run.sh fork-up`은 실행 중인 `sim-adapter`가 healthy일 때만 10초 제한 안에서 fork
command를 publish한다. 서비스가 없거나 unhealthy면 새 container를 만들지 않고 실패한다.

실행 중인 adapter의 ROS topic을 확인할 때는 `./run.sh topics`를 사용한다. Compose
`exec`는 entrypoint가 source한 shell 환경을 상속하지 않으므로 이 명령과 adapter
healthcheck는 `/opt/ros/humble/setup.bash`와 `/opt/mentorpi_ws/install/setup.bash`를
각각 source한 shell에서 ROS CLI를 실행한다. 직접 진단해야 할 때도 같은 형태를 사용한다.

```bash
docker compose exec sim-adapter bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/mentorpi_ws/install/setup.bash && ros2 topic list'
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

## SLAM 매핑 세션 운영

매핑은 일회성 `slam-mapper` 서비스로 실행한다. 세션 ID는 영문 대소문자, 숫자, `.`, `_`, `-`만
사용하며, 같은 ID는 publish된 결과 또는 진행 중인 작업과 재사용할 수 없다.

```bash
./run.sh mapping-up warehouse-20260726-01
./run.sh mapping-stop
./run.sh mapping-status warehouse-20260726-01
```

`mapping-up`은 Gazebo와 adapter가 healthy가 된 뒤 mapper를 시작하고, `SESSION_ID` 및 이미지·world·model·TF
버전 metadata를 컨테이너에 전달한다. `mapping-stop`은 mapper에 먼저 `SIGINT`를 보내 map, posegraph, rosbag,
manifest, checksum finalization이 끝날 때까지 제한 시간(기본 90초)만큼 기다린다. 완료 전에 `down`을 호출하지
않으므로 finalization을 중단하지 않는다. mapper가 0으로 종료되면 성공을, 다른 종료 코드면 실패를 명확히
보고한 뒤 Gazebo와 adapter를 정지한다. 제한 시간을 넘으면 mapper와 지원 서비스를 모두 유지한다.

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
