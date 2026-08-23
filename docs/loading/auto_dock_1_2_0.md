# Auto Dock 1.2.0 변경점

## 릴리스 기준

- Git tag: `1.2.0`
- ROS package version: `1.2.0`
- 기준 브랜치: `fork_test`
- 1.1 코드는 복구 및 비교용 `1.1` 태그로 보존한다.

## 핵심 변경

### 자동주행 상태머신 분리

1.1에서는 탐색·정렬·삽입 로직이 Qt `TeleopWindow` 안에 있었고 headless runner도 GUI 객체를 생성했다. 1.2.0에서는 운영 상태머신을 ROS 패키지의 `/auto_dock` 노드로 분리했다.

따라서 GUI/UI를 실행하지 않아도 다음 과정이 동작한다.

```text
arrival 수신
  -> 원형 탐색
  -> 후보 최초 검출 시 정지
  -> 안정 검출 확인
  -> odom 기준 가상 목표 좌표 생성
  -> 접근 중 카메라 관측으로 목표 좌표 연속 보정
  -> 정렬 및 삽입
  -> lift up
  -> idle
```

### 정지 상태 수정

1.1 runner는 arrival을 받은 뒤 실행 표식인 `started_at`을 계속 유지했다. GUI 내부 제어가 멈추거나 `/controller/cmd_vel`에 0을 한 번 발행해도 다음 tick에서 탐색을 다시 시작했기 때문에 지속 정지가 되지 않았다.

1.2.0은 `/robot_N/auto_dock/stop` 또는 arrival 토픽의 `cancel`/`stop`을 받으면 상태 자체를 `idle`로 바꾼다. 이후 새로운 arrival을 받기 전에는 주행 명령을 다시 시작하지 않는다.

### LiDAR interrupt와 재계획

탐색·정렬·삽입 중 LiDAR가 설정 거리보다 가까운 장애물을 검출하면 주행을 interrupt하고 반대 방향으로 짧게 이격한다. 이격 후 저장된 가상 목표 좌표가 있으면 다시 정렬하고, 없으면 탐색부터 재개한다. 자체 차체 반사는 `lidar_self_filter_distance_m` 값으로 제외한다.

### 차량과 YOLO 자동 선택

```text
ROS_DOMAIN_ID=215 -> vehicle 1 -> /robot_1
ROS_DOMAIN_ID=216 -> vehicle 2 -> /robot_2
```

`auto_dock.launch.py`는 차량 번호를 별도로 입력하지 않아도 DDS domain으로 차량 namespace를 결정한다. `/robot_N/symbol_seg/detections`를 발행하는 `/yolo_tag`가 이미 있으면 재사용하고, 없으면 YOLO를 실행한다.

### 개발용 GUI/UI 클라이언트

GUI와 control_ui는 각각 `DevControlClientNode`를 실행한다. 이 노드는 운영 자동주행 상태머신을 소유하지 않고 기존 토픽을 구독하거나 명령을 발행한다.

```text
subscribe /ascamera/camera_publisher/rgb0/image
subscribe /robot_N/symbol_seg/detections
subscribe /robot_N/auto_dock/status

publish   /robot_N/nav2/arrival
publish   /robot_N/auto_dock/stop
publish   /controller/cmd_vel        # 수동 디버깅
publish   /fork/command              # 수동 디버깅
```

GUI는 별도 YOLO 영상 스트림을 받지 않는다. 원본 카메라 프레임과 detections JSON을 구독하고 박스·태그 중심점을 로컬에서 합성한다. Calibration, ARC 샘플, 수동 조작과 1.1 비교용 로컬 디버깅 화면은 유지하지만 운영 1.2.0 실행은 arrival/stop 인터페이스를 사용한다.

GUI에서는 `1.2 arrival 토픽 발행`과 `1.2 stop 토픽 발행` 버튼을 사용한다. control_ui에서는 각각 `P`, `K` 키를 사용한다.

## 노드와 토픽

```text
/yolo_tag
  subscribe: /ascamera/camera_publisher/rgb0/image
  subscribe: /ascamera/camera_publisher/depth0/image_raw
  publish:   /robot_N/symbol_seg/detections

/auto_dock
  subscribe: /robot_N/nav2/arrival
  subscribe: /robot_N/auto_dock/stop
  subscribe: /robot_N/symbol_seg/detections
  subscribe: /scan_raw
  subscribe: /odom_raw
  publish:   /controller/cmd_vel
  publish:   /fork/command
  publish:   /robot_N/auto_dock/status

/dev_control_client
  subscribe: 원본 카메라, detections, auto_dock status
  publish:   arrival, stop, 수동 cmd_vel, 수동 fork command
```

`/robot_N/auto_dock/status`는 상태가 바뀔 때만 발행한다. 마지막 상태 하나는 `TRANSIENT_LOCAL` QoS로 보존하므로 개발 GUI/UI를 나중에 실행해도 받을 수 있다.

## 설정값

`/shared/vehicle_pose_config.json`에서 다음 주요 값을 읽는다.

- `search_linear_speed_m_s`: 탐색 직선 속도
- `search_circle_diameter_m`: 탐색 원 지름
- `stable_detection_frames`: 안정 검출 프레임 수
- `dock_standoff_m`: 정렬 완료 목표 거리
- `insertion_distance_cm`: 정렬 후 추가 삽입 거리
- `centerline_offset_cm`: 카메라와 차량 중심선 차이
- `lidar_stop_distance_m`: LiDAR interrupt 거리
- `lidar_self_filter_distance_m`: 차체 반사 제외 거리

## 실행

차량 zsh에서 이미 `ROS_DOMAIN_ID`가 설정돼 있으면 다음 명령만 사용한다.

```zsh
ros2 launch auto_dock auto_dock.launch.py
```

임시 목표 발행 예시는 다음과 같다.

```zsh
ros2 topic pub --once /robot_2/nav2/arrival std_msgs/msg/String "data: 'arrived clover heart'"
```

## 남은 Nav2 연동

현재 `filtered_navigation.launch.py`는 `/robot_N/nav2/arrival`을 자동으로 발행하지 않는다. Nav2 도착 결과를 arrival 인터페이스로 연결하는 작업은 아직 남아 있다. 또한 Nav2는 `/cmd_vel`, auto_dock은 `/controller/cmd_vel`을 사용하므로 두 제어기가 동시에 발행하지 않도록 상태 전환 또는 velocity mux가 필요하다.

## 검증

- 2호차 DDS domain `216`에서 `auto_dock` 패키지 빌드 완료
- GUI/UI와 차량 배포 파일 SHA-256 일치 확인
- `idle/ready -> cancelled/emergency_stop` 상태 전환 확인
- 원본 카메라와 detection JSON의 로컬 오버레이 합성 확인
- 실제 arrival 주행 명령은 릴리스 검증 중 발행하지 않음
