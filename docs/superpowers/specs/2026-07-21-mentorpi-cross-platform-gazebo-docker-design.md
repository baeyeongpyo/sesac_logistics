# MentorPi 크로스 플랫폼 Gazebo Docker 설계

**상태:** 설계 검증 완료, 문서 검토 대기

## 목표

macOS, Linux, Windows 호스트에서 MentorPi 지게차 시뮬레이션을 개발하고
실행할 수 있는 재현 가능한 단일 Linux 컨테이너를 제공한다. 컨테이너는 ROS 2
Humble과 Gazebo Fortress를 사용하고, 각 호스트는 자체 X11 서버를 통해
Gazebo GUI를 표시한다.

## 호환성 기준

- 컨테이너 운영체제: Ubuntu 22.04 (Jammy)
- ROS 배포판: ROS 2 Humble
- 시뮬레이터: 공식 `ros-humble-ros-gz` 패키지로 설치하는 Gazebo Fortress.
  Humble과 Fortress를 권장 조합으로 사용한다.
- 컨테이너 아키텍처: 기본값 `linux/amd64`
- Linux 및 일반적인 Windows/WSL2 환경은 이미지를 네이티브로 실행한다.
  Apple Silicon macOS는 Docker Desktop 에뮬레이션으로 동일한 이미지를
  실행한다. 이는 별도의 best-effort ARM64 이미지보다 검증된 하나의
  Gazebo/플러그인 ABI를 우선하기 위함이다.
- 기존 SDF는 Fortress 호환 상태로 유지한다. 즉,
  `ignition-gazebo-*` 시스템 플러그인 이름과 `ignition::gazebo::*`
  네임스페이스를 Docker 작업에서 마이그레이션하지 않는다.

## 구성 요소

### 이미지

`docker/Dockerfile`은 ROS 2 Humble 데스크톱 이미지를 기반으로 하고,
ROS-Gazebo 통합 메타 패키지, Xacro, colcon, rosdep, XML 검증 도구,
Mesa/X11 진단 도구를 설치한다. 호스트 X 서버를 설치하거나 호스트의
디스플레이 설정을 변경하지 않는다.

`docker/entrypoint.sh`는 `/opt/ros/humble/setup.bash`를 source하고,
워크스페이스가 빌드된 경우에만 `/ws/install/setup.bash`를 추가로 source한다.
전달받은 명령을 그대로 실행하므로 같은 이미지에서 대화형 셸, 테스트, 빌드,
시뮬레이터 launch를 모두 수행할 수 있다.

### Compose 서비스

`compose.yaml`은 대화형 `mentorpi-sim` 서비스 하나를 제공한다. 저장소의
`ros2_ws` 디렉터리를 `/ws`에 마운트하고, 기본
`TARGET_PLATFORM=linux/amd64`를 사용하며, `DISPLAY`를
`host.docker.internal:0`으로 전달한다. Mesa 소프트웨어 렌더링을 켜고,
Linux에서는 `host.docker.internal`을 위한 `host-gateway` 대체 경로를
추가한다.

서비스는 Gazebo를 자동 시작하지 않는다. 셸 접근, 빌드, 테스트, GUI launch를
명시적으로 실행하여 시작 실패를 쉽게 진단할 수 있도록 한다.

### 호스트 X11 연결

컨테이너 내부 명령은 모든 플랫폼에서 동일하다. 달라지는 부분은 호스트 X11
설정뿐이다.

| 호스트 | X11 연결 방식 | 컨테이너의 표시 대상 |
| --- | --- | --- |
| macOS | 네트워크 클라이언트를 허용한 XQuartz | `host.docker.internal:0` |
| Linux | 로컬 Docker 브리지에 TCP 연결을 허용한 호스트 X 서버 | `host.docker.internal:0` |
| Windows | 로컬 Docker 트래픽을 허용한 WSL2 + VcXsrv, X410 또는 다른 X 서버 | `host.docker.internal:0` |

호스트 설정 명령은 Docker 전용 README에 기록한다. 명령은 개발 세션 동안에만
X11 접근을 허용하고, 종료 시 접근을 해제하는 절차를 포함한다. Xauthority
쿠키나 호스트별 IP 주소는 프로젝트에 커밋하지 않는다.

## 실행 절차

1. macOS/Windows에서는 Docker Desktop을, Linux에서는 Docker 데몬을 시작한다.
2. 사용 중인 플랫폼의 README에 따라 호스트 X11 서버를 시작하고 설정한다.
3. 저장소 루트에서 `docker compose build`를 실행한다.
4. `/ws`에서 `docker compose run --rm mentorpi-sim colcon build --symlink-install`을
   실행한다. 이어서 대화형 셸 또는 두 로봇 launch를 실행한다.
5. 화면이 보이는 시뮬레이터는
   `ros2 launch mentorpi_gz_sim two_robot_sim.launch.py headless:=false`로 시작한다.
6. `/robot_1/fork/command` 토픽으로 포크 컨트롤러를 검증한다.

## 검증

### 정적 검증

- `docker compose config`가 서비스, 기본 플랫폼, 워크스페이스 마운트,
  디스플레이 변수, 이미지 빌드 경로를 올바르게 해석한다.
- Dockerfile에 ROS Humble, `ros-humble-ros-gz`, Xacro, colcon,
  X11/Mesa 진단 도구가 포함된다.
- 문서에 macOS, Linux, Windows의 X11 사전 조건과 정확한 빌드, 셸,
  시뮬레이터, 포크 명령이 포함된다.

### 런타임 검증

- Linux/amd64 머신에서 `docker compose build`가 완료된다. Apple Silicon은
  동일한 이미지를 에뮬레이션으로 실행할 수 있다.
- 컨테이너 내부에서 `ros2 pkg prefix ros_gz_sim` 및
  `ign gazebo --versions`가 성공한다.
- `colcon build --symlink-install`이 `mentorpi_description`과
  `mentorpi_gz_sim`을 빌드하고, 기존 Python/XML 회귀 검사가 통과한다.
- Gazebo를 실행하기 전에 `xclock`이 호스트에 열려 X11 전달을 증명한다.
- `headless:=false`로 Gazebo를 실행하면 두 로봇이 표시되고,
  `/robot_1/fork/command`를 `0.0`에서 `0.11`로 변경했을 때 포크가
  눈에 보이게 상승한다.

## 범위에서 제외하는 항목

- Fortress에서 Harmonic 또는 그 이후 Gazebo 배포판으로의 마이그레이션
- 네이티브 macOS 또는 네이티브 Windows Gazebo 설치
- GPU 패스스루 보장. 이식성 기준은 Mesa 소프트웨어 렌더링이며,
  Apple Silicon에서 GUI 성능이 제한될 수 있다.
- 로봇 운동학, SDF 플러그인, ROS 토픽 계약 변경
