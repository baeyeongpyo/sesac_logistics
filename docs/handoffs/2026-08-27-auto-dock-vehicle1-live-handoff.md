# Auto Dock 차량 1 실차 작업 핸드오프 — 2026-08-27

## 문서 성격

이 문서는 2026-08-27 차량 1 실차 세션에서 발생한 증상, 실패한 접근,
최종 보정 방식, 배포 상태를 다음 세션에 넘기기 위한 작업 핸드오프다.
Project/Team wiki truth가 아니며, 실차 runtime 설정 파일도 canonical source가
아니다.

## 인계 시점 핵심 요약

- 대상 차량: 차량 1
- SSH: `intelions@192.168.100.38`
- Docker container: `IntelPi`
- ROS domain: `215`
- Git branch: `fork_test`
- 현재 HEAD: `655a981`
- 이번 세션 변경은 아직 커밋하지 않았다.
- 저장소에는 이번 작업과 무관한 사용자 변경 및 untracked 파일이 많다.
  전체 stage, `git reset --hard`, `git clean`을 하지 않는다.
- 최신 로컬 파일과 차량 1 배포 파일의 SHA-256은 일치한다.
- 인계 시점에는 `/shared/auto_dock_test_panel.py`만 실행 중이다.
  Auto Dock, YOLO, fork_controller는 실행 중이 아니다.
- 실행 중인 패널 프로세스는 최신 패널 파일 배포 전에 시작된 프로세스이므로
  메모리에는 수동 fork COMPLETE 버튼이 남아 있을 수 있다. 반드시 재시작한다.
- 차량 2에는 이번 세션 최종 변경을 배포하지 않았다.

## 최신 파일과 차량 1 배포 위치

| 기능 | 저장소 파일 | 차량 1 위치 | SHA-256 |
| --- | --- | --- | --- |
| Auto Dock FSM | `ros2_ws_src/auto_dock/auto_dock/auto_dock_node.py` | `/home/ubuntu/ros2_ws/src/auto_dock/auto_dock/auto_dock_node.py` | `db0a84ca668b0f925d1ae9ce2150eb9d62d53d7132ae332a028d3a035f0d23dc` |
| Fork controller | `ros2_ws_src/fork_control/fork_control/fork_controller.py` | `/home/ubuntu/ros2_ws/src/fork_control/fork_control/fork_controller.py` | `c9ec319beb0ad7c228b1ba8b3f166f8f00b4d43ff46394f2a6c891439f26e6a0` |
| 테스트 패널 | `tools/auto_dock_test_panel.py` | `/shared/auto_dock_test_panel.py` | `f5f3c33fe4f4f898b06f0f5882b316c3356dc64ca2445590bb3262d435be3035` |
| YOLO + 개별 태그 depth | `tools/yolo_symbol_seg_node.py` | `/shared/yolo_symbol_seg_node.py` | `fa90693cbdc5c3b2b2242f8c9f4f89bee41a63a817c932c418db45373dbc45aa` |

Fork controller의 simulator mirror도 같은 로직으로 갱신했다.

```text
vehicle_simulator_model/ubuntu/ros2_ws/src/fork_control/fork_control/fork_controller.py
```

## 인계 시점 실행 상태

마지막 확인 결과 실행 중인 관련 프로세스는 패널 하나뿐이었다.

```text
python3 /shared/auto_dock_test_panel.py
```

다음 프로세스는 내려가 있었다.

```text
/auto_dock
/yolo_tag
/fork_controller
```

따라서 다음 실차 시험은 카메라/Nav2 주행 스택을 먼저 올린 뒤 Auto Dock
launch와 패널을 새로 시작해야 한다. 소스와 install build는 차량 1에 배포돼
있지만, 실행 중 프로세스 메모리는 파일 배포로 자동 갱신되지 않는다.

## 검증 결과

- Auto Dock 전체 테스트: `83 passed`
- Fork controller 추가 로직 테스트: `2 passed`
- YOLO 파일: `python3 -m py_compile` 통과
- 차량 1 `colcon build --packages-select auto_dock`: 성공
- 차량 1 `colcon build --packages-select fork_control`: 성공
- setuptools `setup.py install is deprecated` 경고만 있었고 빌드는 성공했다.

## 최종 활성 SEARCHING 동작

초기 회전 스캔 실험은 차량 설정에서 꺼져 있다.

```json
{
  "nav2_scan_approach_enabled": 0,
  "tag_guided_lateral_search_enabled": 1
}
```

Arrival을 받으면 Auto Dock은 `search_heading_yaw`에 시작 odom yaw를 저장한다.
SEARCHING에서는 화면 중심에 가장 가까운 **개별 심볼 태그**를 고른다.
완성된 2×2 pallet entity는 필요하지 않다.

현재 우선순위는 다음과 같다.

1. 시작 yaw 대비 차체 yaw 오차가 3도를 넘으면 횡이동을 멈추고 저속 회전 보정한다.
2. 정면 개별 태그 depth가 20 cm보다 멀고 30 cm 이하이면 횡이동을 멈추고 전진 보정한다.
3. yaw와 거리가 모두 조건 안에 들어오면 순수 횡이동한다.
4. 개별 태그 depth가 30 cm를 넘으면 노이즈로 제외한다.
5. 30 cm 이내의 유효 개별 태그 depth가 없으면 움직이지 않고
   `tag_guided_search_depth_missing`을 발행한다.

차량 1의 주요 활성 runtime 값은 다음과 같다.

```json
{
  "tag_guided_lateral_search_enabled": 1,
  "tag_search_max_distance_cm": 20.0,
  "tag_search_noise_max_distance_cm": 30.0,
  "tag_search_yaw_tolerance_deg": 3.0,
  "tag_search_max_angular_speed_rad_s": 0.06,
  "tag_search_forward_correction_speed_m_s": 0.12,
  "search_lateral_speed_m_s": 0.12,
  "search_lateral_direction": "left"
}
```

`search_forward_compensation_m_s: 0.02`는 설정 파일에 남아 있지만
tag-guided 분기가 활성화된 동안에는 최종 순수 횡이동 명령에 사용하지 않는다.

## 개별 태그 depth 생성

기존 YOLO는 완성된 2×2 entity에만 `depth_yaw`를 넣었다. 실차에서는 개별
태그가 여러 개 검출되어도 `entities: []`가 될 수 있어 Auto Dock이 계속
정지했다. 이를 보정하기 위해 모든 개별 심볼 detection의 중심 5×5 depth
영역에서 유효 depth 중앙값을 계산하고 다음 필드를 붙인다.

```json
{
  "class": "star",
  "box": [0, 0, 0, 0],
  "depth": {
    "camera_depth_m": 0.0,
    "forward_distance_cm": 0.0,
    "bearing_deg": 0.0,
    "distance_reference": "fork tip to tag face"
  }
}
```

거리 기준은 depth camera 원점이 아니라 fork tip에서 태그 면까지다.
`depth_camera_to_fork_tip_offset_cm`를 차감한다.

카메라는 약 4 Hz였고 RGB/depth 프레임 간격이 150 ms를 넘으면서 개별 depth가
간헐적으로 사라졌다. 개별 태그 depth에만 기본 350 ms 시간 허용값을 적용했다.
기존 두 태그 쌍의 face yaw 계산은 150 ms 조건을 유지한다.

```text
individual_tag_depth_max_age_sec default: 0.35
```

이 값은 현재 `/shared/vehicle_pose_config.json`에 명시돼 있지 않고 코드 기본값을
사용한다.

## 최종 ALIGNING 동작

차량 1에서는 이동 우선 정렬이 활성화돼 있다.

```json
{
  "translation_first_alignment_enabled": 1,
  "translation_alignment_max_angular_speed_rad_s": 0.06
}
```

- 전후 및 좌우 이동으로 거리와 중심을 우선 맞춘다.
- 후방 충전독 때문에 정렬 중 음수 전진 명령은 내지 않는다.
- 큰 yaw 오차도 정지하지 않고 최대 `0.06 rad/s`로 계속 회전 보정한다.
- 실제 시험에서 yaw `-33.6°`를 신뢰 불가로 보고 정지시키는 로직은 실패로
  판정해 제거했다.
- yaw 3도 이내, lateral 오차 2.5 cm 이내, standoff 범위에 들어오면 삽입으로
  전환한다.

관련 기본/현재 값은 다음과 같다.

```json
{
  "dock_standoff_m": 0.2,
  "insertion_distance_cm": 12.0,
  "distance_coefficient": 0.8935677101700673,
  "lateral_coefficient": 0.9391809600813833
}
```

## PICK/PLACE 후진 및 우회전

실제 성공 시험에서 PICK, 포크 상승, 후진까지 완료됐다.

```text
state: READY
reason: drive_ready_right_turn_skipped
reversed_cm: 32.3
blocking_range_cm: null
```

후진은 정상 완료됐지만 후진 완료 순간 `scan_updated_at` age가 0.5초를 넘었다는
이유로 우회전을 즉시 영구 생략했다. 직후 `/scan_raw`는 약 9 Hz로 정상이었다.

보정 후에는 후진 완료 시 신선한 scan이 없으면 현재 상태를 유지하고 최대 2초
동안 재검사한다.

```text
right_turn_waiting_for_fresh_scan
```

- 신선한 scan이 들어오면 swept rectangle 공간 판정 후 우회전을 시작한다.
- 실제 장애물이 있으면 기존처럼 우회전을 생략한다.
- 2초 동안 scan이 끝내 들어오지 않으면
  `drive_ready_right_turn_scan_timeout`으로 READY를 발행한다.
- `right_turn_scan_wait_timeout_sec`는 현재 runtime JSON에 없고 코드 기본값 2초다.

이미 `READY`로 끝난 작업에는 보정 코드를 배포해도 우회전이 소급 실행되지 않는다.
다음 작업부터 적용된다.

## Fork controller 자동 완료

ROS 연결 자체는 끊겨 있지 않았다.

```text
Auto Dock publish: /fork/command String (UP/DOWN/STOP)
fork_controller subscribe: /fork/command
fork_controller publish: /robot_1/fork/state String JSON
Auto Dock subscribe: /robot_1/fork/state
```

문제는 fork_controller가 `motor.forward()/backward()`를 먼저 실행하고
`active_command`를 나중에 기록했다는 점이다. 모터 시작 직후 리미트 콜백이
들어오면 콜백이 이동 중이 아니라고 판단해 모터만 멈추고 COMPLETE를 누락할
수 있었다. 사용자는 패널에서 `UP_COMPLETE`를 수동 발행해야 차량 후진이
시작되는 증상을 확인했다.

보정 내용:

1. `active_command`를 먼저 기록한다.
2. 그 다음 모터 출력을 켠다.
3. gpiozero edge callback 외에 50 ms timer에서도 활성 명령과 리미트 입력을
   재확인한다.
4. 리미트 도달 시 fork_controller가 직접 `UP_COMPLETE` 또는
   `DOWN_COMPLETE`를 발행한다.
5. 테스트 패널의 수동 `UP_COMPLETE`, `DOWN_COMPLETE`, `FAILED` publisher와
   버튼은 제거했다.

인계 시점 GPIO 확인값은 다음과 같았다.

```text
GPIO17 LOW  (motor output)
GPIO18 LOW  (motor output)
GPIO22 LOW  (upper limit)
GPIO27 LOW  (lower limit)
```

리미트 배선이 실제 끝 위치에서도 계속 LOW라면 소프트웨어는 물리적 완료를 알 수
없다. 타이머로 완료를 가장하지 않았으며, 그 경우 배선/커넥터/스위치를 고쳐야 한다.

## 테스트 패널 최종 변경

- `/robot_1/fork/state` publisher를 제거했다.
- 패널은 fork state를 구독해 표시만 한다.
- `FORK UP`, `FORK DOWN`, `FORK STOP` 수동 명령은 유지한다.
- 완료 상태는 fork_controller 리미트 스위치에서 자동 발행한다는 문구를 표시한다.
- tag-guided 검색 설정을 패널 상세 설정에 추가했다.
- 실행 중인 패널은 최신 파일보다 먼저 시작됐으므로 재시작해야 이 변경이 보인다.

### 수동 SEARCHING LiDAR 기록

- 수동 주행 영역에 `LiDAR 기록 시작/종료` 버튼을 추가했다.
- 기록 중에는 `/scan_raw`의 유효한 30 cm 이내 포인트와 패널의 수동
  `cmd_vel` 명령을 같은 JSONL 타임라인에 저장한다.
- 각 scan에는 각도, 거리, scan header 시각, 전·후·좌·우 최솟값이 포함된다.
- 기본 저장 위치는 `/shared/lidar_records/vehicle_1_lidar_*.jsonl`이다.
- 2026-08-27 차량 1 `/shared/auto_dock_test_panel.py`에 배포하고 구문 검사를
  통과했다. 확인 시점에는 패널 프로세스가 실행 중이지 않았다.

### SEARCHING 방식 및 LiDAR safety 체크박스

- 패널에서 옵션 1(정면 YOLO bbox/depth)과 옵션 2(후방 LiDAR 30 cm)를
  독립 체크박스로 선택한다.
- 둘 다 선택하면 후방 LiDAR가 30 cm 미만일 때 전진 보정을 우선하고,
  30 cm 이상이면 옵션 1 판단을 이어간다.
- LiDAR safety와 backoff도 각각 체크박스로 저장한다. backoff는 safety가
  켜진 경우에만 동작한다.
- 차체 치수는 실측 지시에 따라 LiDAR 기준 전방 30 cm, 후방 6 cm,
  좌우 각 6 cm로 차량 1 runtime과 코드 기본값을 수정했다.
- 정면 ±20도 범위의 20 cm 이하 LiDAR 값은 차체 자기반사로 무시한다.
- 차량 1에서 Auto Dock 테스트 `87 passed`, 빌드 성공 후 배포했다.
- ALIGNING yaw 오차가 3도를 넘으면 회전 명령을 최소 `0.10 rad/s`, 최대
  `0.12 rad/s`로 제한해 실차 데드밴드를 넘도록 수정했다.

## 이번 세션에서 실패한 접근과 교훈

### 1. Arrival 후 고정 15 cm 전진

후방 충전독과 간격을 만들기 위해 Arrival 직후 무조건 15 cm 전진한 접근은
실패했다. 앞의 엉뚱한 팔레트에 fork를 건 상태로 횡이동해 팔레트를 밀었다.

보정:

- 고정 15 cm 전진 코드를 삭제했다.
- 관련 runtime 키도 차량 1 설정에서 삭제했다.
- 태그 depth를 이용한 조건부 전진만 허용한다.

삭제한 키:

```text
rear_dock_safe_search_enabled
rear_dock_escape_distance_m
rear_dock_escape_speed_m_s
```

### 2. 시간 기반 횡이동/전진 펄스

메카넘 횡이동 시 실제 차체가 뒤로 밀리는 현상을 상쇄하려고
`0.35초 횡이동 + 0.12초 전진`을 반복한 접근도 실차 위치와 무관한 하드코딩이라
폐기했다.

보정:

- 시간 기반 펄스 코드를 삭제했다.
- 정면 개별 태그 depth가 20 cm를 넘을 때만 횡이동을 멈추고 전진한다.

삭제한 키:

```text
search_pulsed_forward_correction_enabled
search_lateral_phase_sec
search_forward_correction_phase_sec
search_forward_correction_speed_m_s
```

### 3. 임의의 25 cm 최소거리

횡이동 최소 태그거리 25 cm를 실차 근거 없이 추가했다가 즉시 제거했다.

보정:

- 25 cm 조건과 `tag_search_min_lateral_clearance_cm` 키를 삭제했다.
- 사용자가 지정한 20 cm 유지 기준과 30 cm 노이즈 상한만 사용한다.

### 4. 가장 왼쪽 태그 기준

초기에는 가장 왼쪽 태그/엔티티를 기준으로 진행 방향과 거리를 잡으려 했다.
사용 목적과 맞지 않았고 인접 팔레트를 기준으로 삼을 위험이 있어 활성 SEARCHING
경로에서 제거했다.

보정:

- 화면 중심에 가장 가까운 개별 심볼 태그를 정면 태그로 선택한다.
- 거리에는 개별 태그 depth를 사용한다.
- 차체 회전 여부에는 태그 bearing이 아니라 시작 odom yaw 대비 현재 odom yaw를
  사용한다.

### 5. 완성 entity depth만 요구

개별 태그는 여러 개 보였지만 `entities: []`인 프레임에서 Auto Dock이
`tag_guided_search_depth_missing`으로 멈췄다.

보정:

- 개별 detection마다 중심 depth를 추가했다.
- 완성 2×2 entity가 없어도 SEARCHING이 가능하다.

### 6. 큰 yaw를 신뢰 불가로 보고 정지

정렬 중 `-33.6°`가 관측됐을 때 회전을 금지하고 정지시킨 접근은 작업을 막았다.

보정:

- 큰 yaw 정지 조건을 제거했다.
- 이동 우선 정렬을 유지하면서 회전속도만 `0.06 rad/s`로 제한한다.

### 7. 카메라가 켜졌는데 publisher 0으로 보인 상황

첫 확인 시 `/ascamera/camera_publisher/rgb0/image` publisher가 실제로 0이었다.
사용자가 카메라를 시작한 뒤 같은 토픽에 publisher 1과 약 3.6~4.4 Hz가
확인됐다. 토픽 이름은 기존 코드와 핸드오프의 값이 맞았다.

교훈:

- 카메라 창 존재 여부가 아니라 ROS publisher count와 `ros2 topic hz`를 확인한다.
- 카메라 시작 직후 DDS discovery 전에 확인하면 0으로 보일 수 있으므로 짧게
  재확인한다.

### 8. 차량이 전혀 움직이지 않았던 첫 원인

초기에는 Nav2/차량 주행 스택이 실행되지 않아 `/odom_raw` publisher가 0이고
`/controller/cmd_vel`의 실제 구동 subscriber도 없었다. Auto Dock은 속도를
발행했지만 받을 컨트롤러가 없었다.

보정:

- Nav2/주행 스택과 odom을 먼저 올린다.
- `/controller/cmd_vel` subscriber에 실제 구동 노드가 있는지 확인한다.

### 9. ARRIVAL 입력을 놓친 진단

사용자가 이미 버튼을 누른 뒤 다른 작업으로 이동했는데, 뒤늦게 15초 토픽
캡처를 걸고 메시지가 없다는 이유로 패널 활성화 문제라고 단정했다. 이 진단은
타이밍을 놓친 것이었다.

교훈:

- 이미 지나간 volatile 토픽 입력은 사후 echo로 증명할 수 없다.
- 다음 재현에서는 상태와 명령을 즉시 캡처하고 장시간 대기로 사용자를 묶지 않는다.

### 10. Fork가 물리적으로 안 움직였던 상황

세션 초반 Auto Dock과 fork_controller는 UP 명령을 정상 전달했고 GPIO17 HIGH,
GPIO18 LOW였지만 포크가 움직이지 않았다. 상단 리미트 GPIO22도 LOW였다.
소프트웨어 이후의 모터 드라이버 전원, Enable, 배선, 커넥터 접촉불량 가능성이
높다고 판단했고 stall 보호를 위해 STOP을 반복 발행해 GPIO17/18을 LOW로 내렸다.

교훈:

- 출력 HIGH인데 물리 구동이 없으면 계속 energize하지 않는다.
- 즉시 STOP 후 드라이버 전원/Enable/모터 커넥터/리미트 배선을 확인한다.

## 안전 관련 현재 상태

차량 1 runtime 설정에서 다음 값은 꺼져 있다.

```json
{
  "lidar_safety_enabled": false,
  "lidar_backoff_enabled": false
}
```

따라서 Auto Dock의 일반 주행 충돌 방지는 현재 LiDAR backoff에 의존하지 않는다.
낮은 충전독처럼 LiDAR 스캔 높이에 걸리지 않는 장애물은 계속 감지할 수 없다.
SEARCHING의 20 cm 태그 거리 유지가 낮은 후방 장애물의 일반 충돌 방지 기능을
대체하지는 않는다. 첫 재시험은 비상정지 준비 상태에서 실시한다.

## 비활성 실험 코드와 정리 필요 항목

`nav2_scan_approach_enabled`는 차량 1에서 0이지만 소스에는 다음 초기 실험 코드가
아직 남아 있다.

```text
scan_sweep
scan_forward_search
scan_approach
remember_leftmost_scan_tag
```

이 코드는 활성 SEARCHING 경로에서는 사용되지 않는다. 사용자가 불필요한 실험
코드를 나중에 정리하길 원했으므로, 실차 동작이 안정된 뒤 별도 정리 커밋에서
삭제할 후보로 남긴다. 지금 즉시 대규모 삭제하면서 검증된 활성 경로를 흔들지 않는다.

runtime JSON에는 비활성 scan 실험 값도 남아 있다.

```text
nav2_scan_angle_deg
nav2_scan_angular_speed_rad_s
nav2_scan_confirmation_sec
nav2_approach_standoff_m
nav2_approach_speed_m_s
nav2_approach_max_angular_speed_rad_s
nav2_forward_search_speed_m_s
nav2_forward_search_max_distance_m
```

`nav2_scan_approach_enabled: 0`이므로 현재는 사용되지 않는다.

## 다음 실차 시험 시작 순서

1. 차량 1에서 중복 프로세스가 없는지 확인한다.

```bash
ros2 node list | sort
ps -ef | grep -E 'auto_dock|yolo_symbol_seg|fork_controller|auto_dock_test_panel'
```

2. Nav2/차량 주행 스택과 ASCamera를 팀의 정상 launch로 먼저 시작한다.

3. 카메라와 odom을 확인한다.

```bash
ros2 topic info /ascamera/camera_publisher/rgb0/image -v
ros2 topic hz /ascamera/camera_publisher/rgb0/image
ros2 topic hz /ascamera/camera_publisher/depth0/image_raw
ros2 topic info /odom_raw -v
```

4. Auto Dock 묶음을 시작한다.

```bash
ros2 launch auto_dock auto_dock.launch.py
```

이 launch는 프로젝트의 현재 구성상 fork_controller와 Auto Dock을 올리고,
YOLO가 이미 발행 중이면 재사용한다. 중복 `/fork_controller`와 `/yolo_tag`를
반드시 확인한다.

5. 패널을 재시작한다.

```bash
python3 /shared/auto_dock_test_panel.py
```

6. 최신 YOLO 메시지에서 개별 태그 `depth`가 연속으로 들어오는지 확인한다.

```bash
ros2 topic echo --once /robot_1/symbol_seg/detections --field data
```

7. fork state publisher가 fork_controller 하나뿐인지 확인한다.

```bash
ros2 topic info /robot_1/fork/state -v
```

패널이 최신 코드라면 `/robot_1/fork/state` publisher로 나타나면 안 된다.

8. 첫 Arrival 전 다음 설정을 확인한다.

```text
tag_guided_lateral_search_enabled = 1
tag_search_max_distance_cm = 20
tag_search_noise_max_distance_cm = 30
tag_search_yaw_tolerance_deg = 3
translation_first_alignment_enabled = 1
nav2_scan_approach_enabled = 0
```

9. 첫 시험은 STOP 버튼과 물리 비상정지를 즉시 사용할 수 있는 상태에서 진행한다.

## 다음 세션에서 우선 확인할 로그/상태

SEARCHING이 멈추면 상태 reason을 먼저 본다.

```bash
ros2 topic echo --once /robot_1/auto_dock/status --field data
```

주요 reason:

```text
tag_guided_search_depth_missing
tag_guided_search_odom_missing
tag_yaw_correction_before_lateral
front_tag_distance_correction
front_tag_pose_held_lateral_search
```

포크 완료가 안 되면 다음을 동시에 확인한다.

```bash
ros2 topic echo /fork/command
ros2 topic echo /robot_1/fork/state
pinctrl get 17,18,22,27
```

후진 후 우회전이 안 되면 다음 reason을 구분한다.

```text
right_turn_waiting_for_fresh_scan
drive_ready_right_turn_scan_timeout
drive_ready_right_turn_skipped
drive_ready_right_turn_aborted
drive_ready_after_right_turn_90
```

## Git 인계 주의

이번 변경 대상은 주로 다음 파일이다.

```text
ros2_ws_src/auto_dock/auto_dock/auto_dock_node.py
ros2_ws_src/auto_dock/test/test_arrival_contract.py
ros2_ws_src/fork_control/fork_control/fork_controller.py
ros2_ws_src/fork_control/test/test_fork_controller_logic.py
tools/auto_dock_test_panel.py
tools/yolo_symbol_seg_node.py
vehicle_simulator_model/ubuntu/ros2_ws/src/fork_control/fork_control/fork_controller.py
docs/handoffs/2026-08-27-auto-dock-vehicle1-live-handoff.md
```

커밋할 때도 위 파일을 명시적으로 stage하고, 저장소 전체 변경을 한꺼번에 stage하지
않는다. 이번 세션 도중 다른 사용자 작업 파일이 다수 수정/추가된 상태다.
