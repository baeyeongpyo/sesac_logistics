# MentorPi Headless Docker Bundle

MentorPi Gazebo Harmonic 시뮬레이션을 Docker Compose로 운영하는 headless 번들이다.
서버에는 이미지와 Compose 파일만 배포하며 실행 중인 서비스에 ROS 소스를 bind mount하지
않는다. 기본 렌더러는 Mesa 소프트웨어 렌더링이고, GPU 장치는 명시적인 `gpu` profile에서만
전달한다.

## 개발과 이미지 빌드

Mac에서는 Docker 컨테이너 GUI 대신 네이티브 Gazebo GUI 개발 환경을 사용한다. 이 번들은
MentorPi 서버에서 센서와 물리 시뮬레이션을 운영하기 위한 `linux/amd64` 이미지다.

```bash
cd vehicle_simulator_model/ubuntu
./run.sh build
./run.sh test
```

`build`는 이미지 안의 `/opt/mentorpi_ws`에 ROS 패키지를 빌드한다. 서버 운영 중에는 소스를
마운트하지 않으므로 소스 변경을 배포하려면 이미지를 다시 빌드한다.

## 서버 운영

Docker Engine 및 Docker Compose v2가 설치된 Linux 서버에서 실행한다.

```bash
./run.sh sim-up
./run.sh logs
./run.sh fork-up
./run.sh down
```

`sim-up`은 내부 `mentorpi` 네트워크에서 `gazebo-server`와 `sim-adapter`를 차례로 시작한다.
외부 Gazebo Transport 포트와 ROS DDS 포트는 공개하지 않는다. Gazebo 서버 healthcheck가
통과한 뒤 adapter가 시작하며, 두 서비스는 `GZ_PARTITION=mentorpi-sim`을 공유한다.

서버 GPU를 사용할 때만 다음처럼 명시한다.

```bash
./run.sh sim-up gpu
```

이 profile은 Gazebo 서버에 `/dev/dri`를 전달하고 `LIBGL_ALWAYS_SOFTWARE=0`으로 바꾼다.
기본 profile은 `LIBGL_ALWAYS_SOFTWARE=1`이므로 GPU 장치가 없어도 운영할 수 있다.

## 렌더링 경계

브라우저 렌더링은 사용자의 클라이언트에서 수행한다. 서버는 카메라·라이다 등 시뮬레이션
센서에 필요한 오프스크린 렌더링만 수행한다. 따라서 서버 Compose 구성에는 X11, Xauthority,
DISPLAY 또는 원격 GUI 전달 설정이 없다.
