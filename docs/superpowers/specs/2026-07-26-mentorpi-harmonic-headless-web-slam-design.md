# MentorPi Harmonic Headless·웹 모니터링·SLAM 데이터 생성 설계

**상태:** 사용자 구조 승인 완료, 구현 계획 작성 전 문서 검토

## 1. 목적

개발 PC에서 MentorPi 모델과 Gazebo 월드를 개발·검증하고, 개발이 완료된
실행물을 Ubuntu 서버에 배포한다. 서버는 Gazebo GUI나 개발 도구를 제공하지
않고 다음 런타임만 호스팅한다.

- Gazebo Harmonic 기반 물리·센서 시뮬레이션
- Gazebo Transport와 ROS 2 사이의 데이터 변환
- SLAM 지도 및 재현 데이터 생성
- 브라우저 기반 Gazebo 3D 모니터링

서버의 Gazebo·ROS 2 네트워크는 외부에 직접 노출하지 않는다. 사용자는
HTTPS/WSS 웹 엔드포인트를 통해 시뮬레이션을 관찰한다.

## 2. 확정된 기술 기준

- 개발 PC: Apple Silicon macOS
- 개발용 Gazebo: 네이티브 Gazebo Harmonic 8.x
- 서버 호스트: Ubuntu 22.04 또는 24.04, `linux/amd64`
- 서버 컨테이너: Ubuntu 22.04
- ROS 배포판: ROS 2 Humble 유지
- ROS/Gazebo 조합: `ros-humble-ros-gzharmonic`
- 서버 Gazebo: GUI 없는 `gz sim -s`
- 모델 포맷: URDF/Xacro 및 SDFormat 14
- 외부 모니터링: 브라우저의 gzweb 호환 클라이언트
- 컨테이너 배포: 불변 이미지 태그와 Compose 설정

ROS 2 Humble을 유지하는 이유는 현재 패키지와 SLAM 자산의 변경 범위를
Gazebo 마이그레이션과 분리하기 위해서다. ROS 2 Jazzy 전환은 별도
마이그레이션으로 다룬다.

## 3. 고려한 실행 방식

### 3.1 선택: 서버 Headless + 웹 모니터링

Gazebo Server와 ROS/SLAM 노드는 서버에서 실행하고, Gazebo WebSocket
브리지가 장면을 브라우저에 전달한다. 브라우저가 3D 장면을 렌더링한다.

장점:

- 서버에 X11, XQuartz, VirtualGL, `DISPLAY` 설정이 필요 없다.
- Mac에 서버용 컨테이너 GUI를 전달할 필요가 없다.
- Gazebo Transport와 ROS 2 DDS를 공용망에 노출하지 않는다.
- 서버는 완성된 이미지의 실행과 데이터 보존에 집중할 수 있다.

제약:

- 웹 화면은 네이티브 Gazebo GUI 플러그인 전체를 제공하지 않는다.
- 모델·충돌·관성·센서 배치 수정은 개발 PC에서 수행해야 한다.

### 3.2 제외: Mac의 네이티브 `gz sim -g` 원격 연결

Gazebo Transport를 라우터 너머로 연결하려면 VPN, relay, 양방향 주소 도달성,
동일 partition 관리가 필요하다. 운영 모니터링이 웹으로 충족되므로 기본
경로에서 제외한다.

### 3.3 제외: 서버 GUI 원격 데스크톱

서버가 GUI를 렌더링하고 화면을 전송하므로 GPU·EGL·원격 데스크톱 운영 부담이
커진다. 모델 개발은 Mac에서 수행하므로 채택하지 않는다.

## 4. 시스템 경계

```text
개발 PC
  모델·월드·설정·ROS 코드 수정
  네이티브 Harmonic GUI 검증
  테스트 및 서버용 이미지 빌드
                 │
                 │ 이미지 Registry 또는 오프라인 이미지 번들
                 ▼
Ubuntu 서버
  gazebo-server ────── gazebo-web-bridge ────── HTTPS/WSS ────── 브라우저
        │
        ▼ Gazebo Transport
  sim-adapter
        │
        ▼ ROS 2 topics / TF
  slam-mapper
        │
        ▼
  maps / posegraph / rosbag / manifest 영구 볼륨
```

브라우저는 Gazebo Transport와 ROS 2 DDS에 직접 접속하지 않는다. VPN은 기본
요구사항이 아니다. 서버 관리자가 SSH 접근을 사설화하려는 경우에만 선택적으로
사용한다.

## 5. 서비스 구성

Compose에는 책임이 분리된 서비스를 둔다. 각 서비스가 별도 이미지일 필요는
없으며, 공통 Harmonic/ROS 기반 이미지를 서로 다른 명령으로 재사용할 수 있다.

### 5.1 `gazebo-server`

- Harmonic 월드와 로봇을 로드한다.
- GUI 없이 서버 모드로 실행한다.
- 물리, UserCommands, SceneBroadcaster, Sensors, IMU 시스템을 제공한다.
- GPU LiDAR와 depth camera를 포함한 센서 데이터를 생성한다.
- 종료 신호를 받으면 정상적으로 시뮬레이션을 종료한다.

시각 센서 생성에는 서버의 offscreen rendering이 필요하다. 기본 배포는 Mesa
소프트웨어 렌더링을 지원하고, 서버에 GPU가 있으면 `/dev/dri`와 EGL을 사용하는
가속 프로필을 선택할 수 있다. 두 프로필은 동일한 월드와 토픽 계약을 사용한다.

### 5.2 `sim-adapter`

- `ros_gz_bridge`로 Gazebo 메시지를 ROS 2 메시지로 변환한다.
- 로봇별 namespace와 토픽 이름을 고정한다.
- Gazebo의 동적 pose에서 초기 단계용 ground-truth odom과 TF를 생성한다.
- SLAM이 사용하는 `/scan`, `/imu`, `/odom`, `/tf`, `/tf_static`, `/clock`
  계약을 제공한다.

wheel odom과 노이즈 모델은 초기 파이프라인 검증 이후 별도 단계에서 추가한다.

### 5.3 `slam-mapper`

- mapping 모드에서만 실행한다.
- 프로젝트 자산의 `mentorpi_slam`을 기준으로 `slam_toolbox`의
  `sync_slam_toolbox_node`를 사용한다.
- `sim-adapter`가 발행한 센서·odom·TF를 소비한다.
- 지도 이미지와 메타데이터, pose graph, 재현용 rosbag을 영구 볼륨에 기록한다.
- 실행마다 고유한 `session_id` 디렉터리를 사용한다.

한 세션의 결과물은 다음을 포함한다.

```text
slam-data/<session_id>/
  map.yaml
  map.pgm
  posegraph/
  rosbag2/
  manifest.json
  checksums.sha256
```

`manifest.json`에는 최소한 이미지 버전, Git commit, world/model 버전, 로봇 ID,
SLAM 파라미터 해시, TF/캘리브레이션 버전, 생성 시각을 기록한다.

### 5.4 `gazebo-web-bridge`

- Gazebo와 같은 내부 네트워크 또는 partition에서 WebSocket 브리지를 실행한다.
- SceneBroadcaster가 제공하는 장면을 gzweb 호환 형식으로 전달한다.
- 외부에는 원본 WebSocket 포트를 직접 노출하지 않는다.
- 웹 모니터링 장애가 Gazebo·SLAM 실행을 중단시키지 않도록 독립 서비스로
  관리한다.

### 5.5 `web-gateway`

- Harmonic WebSocket 형식과 호환되는 버전으로 고정한 gzweb 정적 클라이언트를
  제공한다.
- HTTPS와 WSS를 종단한다.
- WebSocket reverse proxy, 인증, 연결 제한, 접근 로그를 담당한다.
- 외부 공개 포트는 기본적으로 443 하나만 사용한다.

인증 정보와 TLS 개인 키는 이미지에 포함하지 않고 서버 배포 secret으로
주입한다.

## 6. Harmonic 마이그레이션

기존 Fortress 전용 요소를 Harmonic 형식으로 일괄 전환한다.

- `ros-humble-ros-gz`를 Harmonic용 ROS 패키지로 교체
- `ign gazebo` 명령을 `gz sim`으로 교체
- `IGN_*` 환경변수를 `GZ_*`로 교체
- `ignition-gazebo-*` 시스템 파일명을 `gz-sim-*`로 교체
- `ignition::gazebo::*` 네임스페이스를 `gz::sim::*`로 교체
- 브리지 메시지 이름을 `ignition.msgs.*`에서 `gz.msgs.*`로 교체
- Harmonic의 SDF 14와 플러그인 파라미터로 월드·모델을 검증
- X11, Xauthority, XQuartz, VirtualGL 전용 구성을 제거

마이그레이션은 토픽 이름과 로봇 namespace를 변경하지 않는다. 기존 소비자가
보는 ROS 2 인터페이스는 유지한다.

## 7. 개발·배포 흐름

### 7.1 개발 PC

1. 모델, 월드, 센서 설정을 수정한다.
2. 네이티브 Harmonic의 `gz sim -s`와 `gz sim -g`로 시각 검증한다.
3. 정적 검사와 ROS 단위 테스트를 실행한다.
4. 서버 대상 `linux/amd64` 이미지를 빌드한다.
5. 통합 테스트를 통과한 이미지에 Git commit 기반 불변 태그를 부여한다.

Mac에서 서버용 Linux 이미지를 직접 실행하지 않는 경우 CI의 Linux/amd64
runner가 이미지 빌드와 통합 테스트를 담당한다.

### 7.2 서버 PC

1. 승인된 이미지와 Compose 설정을 가져온다.
2. `docker compose pull` 또는 오프라인 이미지 import를 수행한다.
3. 환경변수와 secret을 검증한다.
4. 서비스를 기동하고 health check를 확인한다.
5. 지도·rosbag·manifest를 호스트 영구 볼륨에 보존한다.

서버에서는 소스 수정이나 `colcon build`를 하지 않는다.

## 8. 실행 프로필

### `mapping`

- `gazebo-server`
- `sim-adapter`
- `slam-mapper`
- `gazebo-web-bridge`
- `web-gateway`

SLAM 지도 데이터 생성과 웹 모니터링을 위한 기본 프로필이다.

### `sim`

- `gazebo-server`
- `sim-adapter`
- `gazebo-web-bridge`
- `web-gateway`

SLAM 없이 시뮬레이션과 센서 계약만 검증한다.

### `gpu`

기본 프로필에 서버 GPU/EGL 장치와 런타임 설정을 추가한다. 프로필 이름은
서비스 역할이 아니라 렌더링 가속 방식만 선택한다.

Nav2 실행 프로필은 이번 구현 범위에 포함하지 않는다.

## 9. 오류 처리와 관측성

- 월드, 모델, 플러그인 또는 필수 secret이 없으면 시작 단계에서 실패한다.
- `gazebo-server` health check는 world stats와 clock 갱신 여부를 확인한다.
- `sim-adapter` health check는 필수 ROS 토픽과 TF 연결성을 확인한다.
- `slam-mapper`는 결과물을 임시 경로에 기록하고, 완성된 세션만 최종 경로로
  원자적으로 전환한다.
- WebSocket 연결 실패는 웹 서비스만 degraded 상태로 표시하고 시뮬레이션과
  SLAM을 계속 실행한다.
- 모든 서비스 로그에는 이미지 버전, session ID, robot ID를 포함한다.
- 지도 저장 실패나 영구 볼륨 용량 부족은 성공 세션으로 표시하지 않는다.

## 10. 보안 경계

외부에 공개하는 것은 HTTPS/WSS gateway뿐이다.

다음 인터페이스는 서버 내부에 유지한다.

- Gazebo Transport discovery와 topic/service endpoint
- ROS 2 DDS
- Docker daemon
- 원본 WebSocket 포트
- 지도·rosbag 영구 볼륨

웹 gateway는 TLS, 인증, 연결 수 제한을 적용한다. 서버 관리용 SSH는 키 기반
접근과 방화벽 제한을 사용한다. VPN은 네이티브 Gazebo GUI나 ROS 2 도구를
외부에서 직접 연결해야 할 때만 선택한다.

## 11. 검증 기준

### 정적 검증

- Compose 설정에 X11, `DISPLAY`, Xauthority, VirtualGL 의존성이 없다.
- 실행 경로에 Fortress CLI, 플러그인, 메시지 이름이 남아 있지 않다.
- Dockerfile은 Harmonic과 Humble/Harmonic ROS bridge를 설치한다.
- 동일 토픽 계약이 두 로봇 namespace에서 유지된다.
- Compose의 외부 공개 포트는 web gateway로 제한된다.

### 개발 PC 검증

- Mac M1에서 Harmonic 8.x 서버와 GUI가 실행된다.
- 프로젝트 월드와 두 로봇이 네이티브 GUI에 표시된다.
- 모델, collision, joint, 센서 pose가 기대한 위치에 표시된다.

### 서버 컨테이너 검증

- `linux/amd64` 이미지가 빌드되고 `gz sim --versions`에 Harmonic 8.x가
  표시된다.
- GUI 없이 월드가 실행되고 `/clock`과 world stats가 계속 갱신된다.
- 두 로봇의 scan, IMU, depth image, camera info가 발행된다.
- ground-truth odom과 TF tree가 연결된다.
- SLAM이 `map.yaml`, `map.pgm`, pose graph, rosbag, manifest, checksum을
  생성한다.

### 웹 검증

- 인증된 브라우저가 HTTPS/WSS로 접속한다.
- 창고 월드와 두 로봇이 표시된다.
- 차량 이동이 브라우저 장면에 갱신된다.
- 브라우저 연결을 종료해도 Gazebo와 SLAM은 계속 실행된다.
- Gazebo Transport와 ROS 2 포트가 공용망에서 접근되지 않는다.

## 12. 범위에서 제외하는 항목

- 웹에서 URDF/Xacro/SDF 또는 모델 소스 수정
- 네이티브 Gazebo GUI의 전체 플러그인 기능을 웹에 재현
- 실제 차량 SLAM 데이터의 서버 업로드
- 여러 차량의 지도 병합·최적화·검증
- 운영 지도 배포와 안전한 지도 전환 조건
- 실제 차량의 Nav2 실행과 목적지 명령 처리
- wheel odom과 센서 노이즈 튜닝
- ROS 2 Jazzy 마이그레이션

위 항목은 Gazebo·SLAM 데이터 생성 파이프라인이 안정된 후 독립 설계로
진행한다.
