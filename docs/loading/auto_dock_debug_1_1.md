# Auto Dock 1.1 디버깅 실행 가이드

## 범위

이 문서는 `fork_test` 브랜치의 headless `auto_dock` 디버깅 버전 기준이다.
`1.1` 태그는 디버깅 변경 직전의 안정 기준점이다.

## 실행 순서

차량의 bringup(기본 제어기, 카메라, LiDAR)이 먼저 실행되어 있어야 한다. 차량 zsh에서 ROS 환경을 읽고, 아래 launch를 실행한다. 이 launch는 YOLO 심볼 검출 노드와 `auto_dock` runner를 함께 실행한다.

```zsh
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
ROS_DOMAIN_ID=215 ros2 launch auto_dock auto_dock.launch.py vehicle:=1 ros_domain_id:=215
```

탐색·정렬·삽입·lift up 시작은 Nav2 도착 어댑터 토픽으로 보낸다.

```zsh
ROS_DOMAIN_ID=215 ros2 topic pub --once /robot_1/nav2/arrival std_msgs/msg/String "data: 'arrived clover heart'"
```

payload 형식은 `arrived <왼쪽 상단 태그> <오른쪽 상단 태그>`다. 가능한 태그는 `star`, `diamond`, `spade`, `clover`, `heart`다.

## 임시 config override

JSON 파일을 바꾸지 않고 이번 launch에만 config 값을 바꾸려면 `config_overrides`를 준다. JSON 안에는 바꾸고 싶은 키만 넣는다.

```zsh
ROS_DOMAIN_ID=215 ros2 launch auto_dock auto_dock.launch.py vehicle:=1 ros_domain_id:=215 config_overrides:='{"search_linear_speed_m_s":0.08,"stable_detection_frames":3,"search_circle_diameter_m":0.5}'
```

간단한 탐색 속도 override도 가능하다.

```zsh
ROS_DOMAIN_ID=215 ros2 launch auto_dock auto_dock.launch.py vehicle:=1 ros_domain_id:=215 search_linear_speed_m_s:=0.08
```

두 인자를 같이 쓸 경우 `search_linear_speed_m_s:=...`가 우선한다.

## 영구 JSON config

차량의 공용 config 경로는 컨테이너 안에서 `/shared/vehicle_pose_config.json`이다. 주요 디버깅 키는 다음과 같다.

| 키 | 의미 |
| --- | --- |
| `search_linear_speed_m_s` | 원형 탐색 전진 속도(m/s). 낮을수록 목표를 지나치기 전 검출할 여유가 늘어난다. |
| `search_circle_diameter_m` | 원형 탐색 지름(m). 이 값과 전진 속도로 회전 속도가 계산된다. |
| `stable_detection_frames` | 정렬로 전환하기 전 필요한 연속 검출 프레임 수. |
| `arc_cycle_pause_sec` | 자동 정렬 사이클 간 정지·재계산 대기 시간(초). |
| `lidar_stop_distance_m` | 자동 상태에서 LiDAR interrupt가 걸리는 거리(m). |
| `lidar_self_filter_distance_m` | 포크/차체 자기 반사로 무시할 LiDAR 최소 거리(m). |

runner는 매 `arrival` 트리거 시 JSON config를 다시 읽는다.

## 탐색부터 lift up까지

1. `arrival`을 받으면 YOLO에 선택 태그 쌍을 전달하고 원형 탐색을 시작한다.
2. 선택 태그 후보가 한 프레임이라도 보이면 차량을 정지한다.
3. 정지 상태에서 연속 검출, PnP 자세, depth 거리 품질을 확인한다. 후보가 사라지면 원형 탐색으로 돌아간다.
4. 안정 검출이 되면 PnP/depth 측정으로 팔레트의 가상 world 좌표를 만들고, odom 변화에 따라 차량 좌표계에서 계속 재계산한다.
5. 이동 중 새 YOLO 측정이 들어오면 가상 목표 좌표를 보정하며 전후·좌우·회전을 동시에 제어한다.
6. 근거리에서 다시 정지·재검출해 최종 정렬한 뒤 삽입하고, 성공으로 판정되면 `/fork/command`에 `UP`을 보낸다.
7. LiDAR가 설정 안전거리 안으로 들어오면 현재 자동 동작을 interrupt하고, 짧게 이격한 뒤 가상 목표를 재계산해 재시도한다.

## 노드·토픽·UDP 통신선

아래 그림의 각 화살표에는 **전송 방식 / 이름 / 데이터 형태 / 발행 조건**을 같이 적었다.

```text
[Nav2 또는 테스트 터미널]
  │  # ROS2 publish: 사람이 테스트를 시작하거나 Nav2가 목표 도착을 알림
  │  # topic: /robot_1/nav2/arrival
  │  # type: std_msgs/String
  │  # data: "arrived clover heart"
  ▼
[auto_dock 노드]
  │  # UDP datagram: 같은 차량 컨테이너의 YOLO 제어 포트로 목표 태그 전달
  │  # destination: 127.0.0.1:8091
  │  # JSON: {"type":"target","top_left":"clover","top_right":"heart"}
  │  # condition: arrival을 받아 태그 쌍이 유효할 때 한 번 전송
  ▼
[yolo_symbol_seg 노드]  ◀── ROS2 RGB/Depth/CameraInfo ── [Depth Camera]
  │                         # camera topic: RGB, depth, 내부파라미터
  │                         # condition: 카메라가 프레임을 계속 제공할 때
  │
  │  # ROS2 publish: YOLO+PnP+depth 결과
  │  # topic: /robot_1/symbol_seg/detections
  │  # type: std_msgs/String(JSON)
  │  # data: target_top, detections, candidate, PnP, depth_yaw 등
  │  # condition: 처리 가능한 RGB/depth 프레임마다 (기본 최대 4Hz)
  ▼
[auto_dock 노드]  ◀── ROS2 /scan_raw ── [LiDAR]
  │                    # LaserScan: 안전거리 interrupt 판단용
  │
  │                 ◀── ROS2 /odom_raw ── [Odom]
  │                    # Odometry: 가상 목표 좌표를 현재 차량 좌표로 변환
  │
  │  # ROS2 publish: 탐색·정렬·삽입 이동 명령
  │  # topic: /controller/cmd_vel
  │  # type: geometry_msgs/Twist
  │  # data: linear.x(전후), linear.y(좌우), angular.z(회전)
  │  # condition: 자동 탐색/정렬/삽입/LiDAR 이격 중 반복 발행; 취소 시 0속도 발행
  ▼
[차량 주행 Controller]

[auto_dock 노드]
  │  # ROS2 publish: 포크 제어 명령
  │  # topic: /fork/command
  │  # type: std_msgs/String
  │  # data: "UP" 또는 "STOP"
  │  # condition: 삽입 완료 판정 시 UP; 취소·종료 시 STOP
  ▼
[fork_controller]

[auto_dock 노드]
  │  # ROS2 publish: 상태 변화 알림
  │  # topic: /robot_1/auto_dock/status
  │  # type: std_msgs/String(JSON)
  │  # data: state, reason, stamp_monotonic 등
  │  # condition: idle/탐색/복구/완료/취소처럼 상태 또는 이유가 변할 때만
  ▼
[Foxglove 또는 상태 확인 프로그램]
```

### 사용하지 않는 예전 UDP 중계선

아래 경로는 YOLO 코드에는 남아 있지만 현재 auto_dock 흐름은 사용하지 않는다.

```text
[GUI] ── UDP 127.0.0.1:8091 ──> [yolo_symbol_seg]
 # JSON {"type":"drive", ...} 또는 {"type":"fork","command":"UP"}
 # YOLO가 ROS2 /controller/cmd_vel 또는 /fork/command로 중계하던 이전 방식
 # 현재는 auto_dock가 두 ROS2 토픽을 직접 발행하므로 이 경로는 비활성
```

## 정지 주의

`/controller/cmd_vel`에 0값을 한 번 발행해도 다른 주행 노드가 연속 발행 중이면 곧바로 덮어써질 수 있다. 디버깅 중에는 auto_dock launch 터미널에서 `Ctrl+C`로 runner를 종료한 뒤, Nav2 등 다른 `/controller/cmd_vel` 발행 노드도 없는지 확인한다.
