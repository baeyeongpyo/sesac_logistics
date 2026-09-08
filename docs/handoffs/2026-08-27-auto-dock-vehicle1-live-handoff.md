# Auto Dock 차량 1 실차 작업 핸드오프 — 2026-08-27

> 적재 Y PLACE의 9월 5~6일 요구사항·최근 네 세션·반응식과 실차 배포 이력은 [2026-09-06 인계](2026-09-06-loaded-y-place-response-handoff.md)를 먼저 읽는다. 아래 실행 상태는 8월 27일 당시 기록이다.

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
- 현재 HEAD: `2430a2a`
- 후속 실차 세션의 backoff/결합 SEARCHING 변경은 아직 커밋하지 않았다.
- 저장소에는 이번 작업과 무관한 사용자 변경 및 untracked 파일이 많다.
  전체 stage, `git reset --hard`, `git clean`을 하지 않는다.
- 최신 로컬 파일과 차량 1 배포 파일의 SHA-256은 일치한다.
- 인계 직전 `/auto_dock`, `/yolo_tag`, `/fork_controller`, 패널과 주행 스택이
  실행 중이었다. 단, 마지막 Auto Dock 업로드 뒤 `/auto_dock`은 재시작하지 않았다.
- 실행 중 `/auto_dock`은 직전 빌드 코드로
  `ERROR / lidar_backoff_rear_clearance_limit` 상태였다. 최신 후방 전진복구 코드는
  install에만 있으며 프로세스 재시작 뒤 적용된다.
- 차량 2에는 이번 세션 최종 변경을 배포하지 않았다.

## 최신 파일과 차량 1 배포 위치

| 기능 | 저장소 파일 | 차량 1 위치 | SHA-256 |
| --- | --- | --- | --- |
| Auto Dock FSM | `ros2_ws_src/auto_dock/auto_dock/auto_dock_node.py` | `/home/ubuntu/ros2_ws/src/auto_dock/auto_dock/auto_dock_node.py` | `b8c4be65a5198800636fb31416cb73a258e326ceeb29eeaec584ea8ff0dce321` |
| Fork controller | `ros2_ws_src/fork_control/fork_control/fork_controller.py` | `/home/ubuntu/ros2_ws/src/fork_control/fork_control/fork_controller.py` | `c9ec319beb0ad7c228b1ba8b3f166f8f00b4d43ff46394f2a6c891439f26e6a0` |
| 테스트 패널 | `tools/auto_dock_test_panel.py` | `/shared/auto_dock_test_panel.py` | `f5f3c33fe4f4f898b06f0f5882b316c3356dc64ca2445590bb3262d435be3035` |
| YOLO + 개별 태그 depth | `tools/yolo_symbol_seg_node.py` | `/shared/yolo_symbol_seg_node.py` | `fa90693cbdc5c3b2b2242f8c9f4f89bee41a63a817c932c418db45373dbc45aa` |

Fork controller의 simulator mirror도 같은 로직으로 갱신했다.

```text
vehicle_simulator_model/ubuntu/ros2_ws/src/fork_control/fork_control/fork_controller.py
```

## 인계 시점 실행 상태

초기 인계 시점에는 패널만 실행 중이었지만, 후속 실차 세션에서 전체 노드를 다시
올렸다. 마지막 확인 시 다음 노드가 존재했다.

```text
/auto_dock
/auto_dock_test_panel
/fork_controller
/yolo_tag
```

마지막 업로드 뒤에는 Auto Dock을 재시작하지 않았으므로 최신 install 코드가
메모리에 반영되지 않았다. 다음 시험 전에 사용자가 `/auto_dock`을 재시작한다.

## 검증 결과

- Auto Dock 전체 테스트: `106 passed`
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

후속 실차 세션에서 차량 1 runtime 설정을 다시 읽었을 때 다음 값은 켜져 있었다.

```json
{
  "lidar_safety_enabled": 1,
  "lidar_backoff_enabled": 1
}
```

일반 LiDAR safety와 backoff가 모두 활성 상태다. 낮은 충전독처럼 LiDAR 스캔
높이에 걸리지 않는 장애물은 계속 감지할 수 없다. SEARCHING의 20 cm 태그 거리
유지가 일반 충돌 방지를 대체하지는 않는다. 첫 재시험은 비상정지 준비 상태에서
실시한다.

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

## 후속 실차 세션 작업 기록: SEARCHING, backoff, X 격자

### 실차 진단에서 확인한 사실

1. 첫 SEARCHING 정지는 `rear_lidar_search_scan_stale`였다. 당시 `/scan_raw`의
   publisher가 0이었고, LiDAR 노드를 올린 뒤 publisher 1을 확인했다.
2. 기존 backoff는 `-0.12 m/s`를 0.7초만 발행하고 실제 이동량을 확인하지 않은 채
   원래 장애물이 남아 있으면 `lidar_front_blocked_after_backoff` ERROR로 끝났다.
3. 결합 옵션에서 목표 `heart/clover` 중 heart가 없고, 화면 끝 clover depth가
   34.2 cm로 30 cm 노이즈 상한을 넘자 `tag_guided_search_depth_missing`으로
   정지했다. 후방은 66.3 cm, yaw는 0도였으므로 옵션 2 횡탐색은 가능했다.
4. 결합 분기 수정 뒤 status는
   `rear_lidar_clearance_held_lateral_search`, `linear.y=+0.12 m/s`였고 실제
   `/controller/cmd_vel`에서도 같은 명령이 약 20 Hz로 연속 관측됐다. STOP 덮어쓰기는
   관측되지 않았다. 이때 차체가 움직이지 않은 현상은 Auto Dock 출력 이후의 주행
   컨트롤러 상태로 구분했다.
5. 직전 backoff 구현은 후방이 30 cm 미만이면
   `lidar_backoff_rear_clearance_limit` ERROR로 끝났다. 사용자의 요구는 이 경우
   전진해서 후방 여유를 다시 확보하는 것이다.
6. 전진복구를 추가한 뒤에도 SEARCHING 일반 safety가 전·후·좌·우를 모두
   감시하면서 옵션 1/2의 종방향 보정과 충돌했다. 차체는 횡이동하지 못하고
   앞뒤로 반복 이동했으며, 마지막 관측 상태는
   `lidar_backoff_right_clearance_limit`였다.
7. ALIGNING 중 검출이 끊기면 이전 best pose로는 복귀했지만 state가 ALIGNING에
   남아 `alignment_target_lost_holding`과 `alignment_best_pose_restored`를
   반복했다. 화면에 목표가 간헐적으로 다시 보여도 안정 streak가 끊겨 횡탐색으로
   돌아가지 못했다.
8. 첫 SEARCHING 복귀 수정은 짧은 한두 프레임 손실에도 정렬을 중단했다. 진입
   조건은 stable candidate 2프레임, best entity 일치, 유효 PnP였지만 ALIGNING
   중에는 0.5초 손실만으로 복귀가 시작되어 ALIGNING/SEARCHING이 출렁였다.
9. 손실 STOP을 없앤 뒤에도 `alignment_yaw_stabilizing`이 3프레임 재확인 동안
   STOP을 발행했고, 짧은 손실 때 확인 카운트를 0으로 초기화했다. 따라서 검출이
   간헐적이면 잠긴 pose가 있어도 정렬이 반복 정지했다.

Git 이력으로 확인한 원래 회귀 시점은 `3c1089c`
(`fix(auto-dock): stabilize live alignment and fork workflow`,
2026-08-27 15:28:45 KST)이다. 부모 `655a981`의 `tick_docking()`은 candidate가
보일 때만 target을 갱신하고, candidate가 없어도 기존 `target_world`로 계속
정렬했다. `3c1089c`에서 `candidate is None`이면 즉시 STOP하고 0.5초 뒤 best
pose로 복귀하는 분기가 추가됐다. 그 뒤 후속 미커밋 작업에서 best pose 복귀 후
SEARCHING 재진입을 추가하면서 ALIGNING/SEARCHING 반복 현상이 발생했다.

### 최종 구현한 동작

- 원래 LiDAR 장애물이 남아 있으면 0.7초 한 번으로 끝내지 않고 backoff 명령을
  계속 발행한다.
- backoff scan freshness를 확인하며, 비상 상한은 기본 5초와 30 cm 이동이다.
- 전방 장애물 때문에 후진하는 동안 후방 raw LiDAR가 30 cm 미만이면 ERROR 대신
  `+0.12 m/s`로 전진한다.
- 후방이 31 cm까지 복구되면 `lidar_backoff_rear_clearance_restored_search`로
  SEARCHING에 복귀한다. 1 cm는 경계 진동 방지 여유다.
- 옵션 1+2에서 옵션 1 태그/depth가 없더라도 후방 30 cm가 확보되면 시작 yaw를
  보정한 뒤 옵션 2 횡탐색을 계속한다. status extra에
  `tag_depth_missing: true`를 남긴다.
- 옵션 1만 켠 경우에는 기존처럼 유효한 태그 depth가 없으면 정지한다.
- 옵션 1 또는 2가 켜진 guided SEARCHING에서는 일반 safety가 설정된 실제
  횡이동 진행 방향만 감시한다. 앞·뒤 거리는 옵션 1/2가 전담하므로 일반
  backoff가 보정 명령을 뒤집지 않는다. 예를 들어 왼쪽 횡탐색 중에는 왼쪽만
  일반 safety 대상으로 삼고 오른쪽 장애물은 backoff를 유발하지 않는다.
- ALIGNING에서 목표를 잃으면 이전 best pose로 한 번 복귀한다. 복귀 지점에서도
  목표가 없으면 기존 world target/entity 잠금을 해제하고
  `alignment_target_lost_resuming_search`로 SEARCHING을 재개한다.
- 짧은 검출 손실은 상태를 바꾸지 않는다. 잠긴 world target을 이용해 최대 1.5초간
  선속도 0.04 m/s, 각속도 0.08 rad/s 이하로 저속 정렬·접근을 계속하며
  `alignment_target_lost_using_locked_pose`를 발행한다. 1.5초 이상 완전히
  손실된 경우에만 best pose 복귀와 SEARCHING 재개를 수행한다.
- ALIGNING의 3프레임 yaw 재확인 중에도 STOP하지 않고 잠긴 pose로 계속 정렬한다.
  재확인 전 측정값은 world target 갱신에는 쓰지 않으며, 짧은 손실에는 이미 확보한
  확인 프레임 수와 마지막 stamp를 보존한다.

새로 추가된 주요 reason은 다음과 같다.

```text
lidar_backoff_continuing
lidar_backoff_scan_stale
lidar_backoff_forward_for_rear_clearance
lidar_backoff_rear_clearance_restored_search
lidar_backoff_distance_limit
lidar_backoff_time_limit
```

### X 개수 기반 슬롯 격자 시도와 실패 기록

차량 1에서 오늘 촬영된 빈 슬롯 사진 10장을 확인했다.

```text
/home/ubuntu/recordings/vehicle1/captures/capture_20260827_*.jpg
```

사진은 원거리, 좌우 편향, 근거리 잘림 각도를 포함하지만 모두 빈 매트였고 팔레트가
칸을 가린 사례는 없었다. 흰 삼각형 꼭짓점으로 X 허브 후보를 찾고 부분 3×3
homography/PnP로 외곽을 추정하는 실험을 했다. 반복 X 무늬와 외곽 교차점이 서로
다른 3×3 격자로 과적합됐고, 물리 자세 필터와 경계 지지율을 추가해도 10장 중
3장 정도만 통과했으며 일부 자세는 일관되지 않았다. 이 상태는 잘못된 슬롯으로
이동할 위험이 있어 실험 코드를 전부 되돌렸고 실차에는 배포하지 않았다. 현재
Auto Dock 코드에는 X 기반 fallback이 없다. 다음 시도는 점유 팔레트가 포함된
사진을 추가하고, 반복 X만 세지 말고 보이는 외곽선 또는 기준 템플릿으로 절대
행/열을 고정해야 한다.

### 검증과 배포 상태

- 로컬 Auto Dock 테스트: `106 passed`
- `py_compile`, `git diff --check`: 통과
- 차량 1 `colcon build --packages-select auto_dock`: 성공
- 차량 1 source/install SHA-256:
  `b8c4be65a5198800636fb31416cb73a258e326ceeb29eeaec584ea8ff0dce321`
- 컨테이너 ROS 환경은 `/opt/ros/humble/setup.bash`를 먼저 source해야 한다.
- 최신 backoff 후방 전진복구 코드는 배포·빌드됐지만 실행 노드는 재시작하지 않았다.
- 변경 파일은 `auto_dock_node.py`, `test_arrival_contract.py`, 이 핸드오프이며
  아직 커밋하지 않았다.

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

## 2026-08-27 후보 이탈 회전 및 약한 횡정렬 수정

실차 ALIGNING에서 `translation_first_alignment_enabled=1`이어도 기존 코드는
전진·횡이동·회전을 동시에 발행했다. 횡 속도도 최대 0.08 m/s라서 실차의 정지
구간을 넘지 못했고, 화면 가장자리의 정상 후보를 회전으로 먼저 밀어낸 뒤 후보를
잃어버릴 수 있었다. PnP yaw는 부호를 반전하고 depth yaw는 그대로 사용하는 현재
환산식이 두 센서의 기하 부호를 같은 방향으로 맞추므로, 근거 없이 yaw 부호 자체는
변경하지 않았다.

비-slot 심볼 정렬에서는 횡 오차가 2.5 cm 이상이면 전진과 회전을 금지하고 순수
횡이동을 먼저 한다. `translation_alignment_min_lateral_speed_m_s` 기본값 0.12
m/s를 추가해 기존 max 0.08 설정보다 작아지지 않게 했으며, 횡 오차가 허용 범위로
들어온 뒤에만 기존 전진/yaw 정렬을 수행한다. 후보를 잠깐 잃은 1.5초 locked-pose
구간에서도 마지막 yaw를 따라 회전하지 않고, 횡 오차가 남으면 0.12 m/s 순수
횡이동, 횡 정렬이 끝났으면 저속 전진만 허용한다.

검증 결과 Auto Dock 테스트는 106개 전부 통과했고 `git diff --check`와
`py_compile`도 통과했다. 이 변경의 배포 전 소스 SHA256은
`fcbdbd1e4a920607a1bb4649d06b97c3a4665202c229cae5a93c611e62cc5ab1`이다.

### SEARCHING 전방 20cm 유지 기준 수정

옵션 1의 의도는 전방의 임의 심볼 깊이로 20cm 간격을 유지하면서 횡이동해
상자 전면과 평행하게 탐색하는 것이다. 기존에는 화면 중앙에 가까운 태그를 매
프레임 선택했기 때문에 가까운 상자 앞에서도 더 먼 배경 상자로 기준이 바뀌어
0.12m/s 전진 명령이 나갈 수 있었다.

전방 ±45도 내 유효 심볼 중 가장 가까운 깊이를 보수적인 기준으로 선택하도록
변경했다. 목표 간격은 20cm, 데드밴드는 ±1.5cm이며, 멀면 전진하고 가까우면
후진하며 데드밴드 안에서만 횡이동한다. 옵션 2를 함께 쓸 때 전방 20cm와 후방
30cm 조건이 충돌하면 `search_front_rear_clearance_conflict`로 정지하고 어느
상자도 앞으로 밀지 않는다. Auto Dock 테스트 108개, `git diff --check`,
`py_compile`을 통과했으며 배포 소스 SHA256은
`a5d1bdcce2914840fd28d9980e93c7254bc21c61762d6ef2199c93bfd8e3caeb`이다.
