# Auto Dock 1.2-dev 상태 정리

## 목적

1.1의 자동주행 로직은 Qt `TeleopWindow` 안에 있었고 headless runner도 GUI 객체를 생성했다. 1.2-dev는 탐색·정렬·삽입·lift 상태머신을 ROS `auto_dock` 패키지 노드로 분리해 GUI/UI 없이 실행한다. `1.1` 태그는 실차 기준 복구용으로 그대로 보관한다.

## 자동 차량 선택

노드와 YOLO는 프로세스 시작 전에 이미 설정된 `ROS_DOMAIN_ID`를 사용한다.

```text
ROS_DOMAIN_ID=215 -> vehicle 1 -> /robot_1
ROS_DOMAIN_ID=216 -> vehicle 2 -> /robot_2
```

따라서 정상 차량 zsh에서는 `vehicle:=...` 또는 `ros_domain_id:=...`를 입력하지 않는다.

## 실행 노드와 토픽

```text
/yolo_tag
  publish: /robot_N/symbol_seg/detections

/auto_dock
  subscribe: /robot_N/nav2/arrival
  subscribe: /robot_N/auto_dock/stop
  subscribe: /robot_N/symbol_seg/detections
  subscribe: /scan_raw
  subscribe: /odom_raw
  publish: /controller/cmd_vel
  publish: /fork/command
  publish: /robot_N/auto_dock/status
```

`auto_dock.launch.py`는 해당 차량의 `/robot_N/symbol_seg/detections`에 `/yolo_tag` 발행자가 있는지 ROS graph로 확인한다. 있으면 재사용하고 없으면 `/shared/yolo_symbol_seg_node.py`를 실행한다.

```zsh
ros2 launch auto_dock auto_dock.launch.py
```

## 1.1 정지 문제

1.1 runner는 `arrival`을 받으면 `started_at`을 유지한다. 내부 탐색/정렬 상태가 정지되어도 `started_at`이 남아 있으면 다음 tick에서 이를 일시적인 검출·제어 실패로 보고 `start_target_search()`를 다시 호출한다.

따라서 다음 방식은 지속 정지가 아니다.

- GUI의 로컬 정지: GUI 상태만 끄므로 runner가 다음 tick에 탐색을 재시작한다.
- `/controller/cmd_vel` 0을 한 번 발행: runner의 다음 속도 명령에 덮인다.
- LiDAR interrupt: 1.1 설계상 이격 후 탐색/정렬을 재개한다.

1.1에서 지속 정지로 설계된 유일한 경로는 runner가 직접 받은 `cancel/stop`으로 `started_at=None`을 만드는 것이다. runner 중복 실행, 다른 domain, subscriber 부재가 있으면 이 명령도 전달되지 않는다.

1.2 노드는 정지 명령을 받으면 상태를 `idle`로 바꾸며 자동 재탐색하지 않는다.

## Nav2 점검 결과

2호차의 `filtered_navigation.launch.py`와 포함된 navigation launch는 다음 사용자 정의 인터페이스를 사용하지 않는다.

- `/yolo_tag`
- `/robot_2/symbol_seg/detections`
- `/robot_2/nav2/arrival`
- `/robot_2/auto_dock/status`
- `/robot_2/auto_dock/stop`
- `/fork/command`

즉 Nav2 도착 결과를 auto_dock `arrival`로 연결하는 코드는 아직 없다. Nav2는 `/cmd_vel`, auto_dock는 `/controller/cmd_vel`을 발행하며 현재 차량의 `odom_publisher`가 두 토픽을 모두 구독하므로, 동시에 속도를 발행하지 않도록 상태 전환 또는 velocity mux가 필요하다.

## GUI/UI 상태

`control_ui.py`와 `vehicle_camera_teleop_gui.py`는 개발·디버깅 기준인 `1.1`의 기능과 화면을 그대로 유지한다. `auto_dock` 1.2 상태머신은 GUI/UI가 없어도 독립 실행된다.

```text
GUI의 `1.2 arrival 토픽 발행` 버튼 또는 control_ui의 `P`
  -> publish String("arrived LEFT RIGHT") to /robot_N/nav2/arrival
```

기존 주행 계산, 원형 탐색, 단일/3회/무제한 자동주행, ARC 샘플, Calibration 기능은 변경하지 않는다. 추가 버튼은 현재 선택된 좌·우 태그를 arrival 문자열에 넣기만 하며 GUI 내부 1.1 상태를 시작하거나 정지하지 않는다. 차량 번호를 고르면 DDS domain은 `1 -> 215`, `2 -> 216`으로 정하고, 공통 차량 내부 토픽은 `/scan_raw`, `/odom_raw`, `/controller/cmd_vel`, `/fork/command`를 사용한다.
