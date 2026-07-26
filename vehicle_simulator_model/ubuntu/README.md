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

## 렌더링 경계

브라우저 렌더링은 사용자의 클라이언트에서 수행한다. 서버는 카메라·라이다 등 시뮬레이션
센서에 필요한 오프스크린 렌더링만 수행한다. 따라서 서버 Compose 구성에는 X11, Xauthority,
DISPLAY 또는 원격 GUI 전달 설정이 없다.
