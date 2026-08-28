# Nav2 저장 지도 우선·SLAM 대체 설계

## 목표

Foxglove 3D 패널이 발행하는 `/move_base_simple/goal`을 `robot_1`의 Nav2
`NavigateToPose` 액션으로 전달해, 클릭한 목표점까지 안전하게 주행한다.

정상 운용은 저장 지도 기반의 AMCL 로컬라이제이션을 사용한다. 검증된 지도 세션이
없을 때만 SLAM Toolbox mapping 모드와 Nav2를 함께 시작한다.

## 범위

- 대상 차량은 `robot_1`만이다. `robot_2`는 자율 주행 대상이 아니다.
- Gazebo 구동 입력은 기존 `/robot_1/controller/cmd_vel`을 그대로 사용한다.
- Foxglove는 변경하지 않고 기존 `/move_base_simple/goal` 발행 기능을 사용한다.
- 지도 세션은 기존 `slam-data` 볼륨과 `mapping-stop`의 atomic publish 규약을 재사용한다.

## 실행 모드

### localization

`map.yaml`, `map.pgm`, `manifest.json`, `checksums.sha256`가 모두 존재하고 checksum
검증에 통과한 지도 세션을 선택한다.

1. `map_server`가 해당 `map.yaml`을 제공한다.
2. AMCL이 `/robot_1/scan_raw`와 `robot_1/odom → robot_1/base_footprint`를 사용해
   `map → robot_1/odom`을 발행한다.
3. Nav2 planner, controller, behavior-tree navigator, behavior server와 lifecycle
   manager를 시작한다.

### mapping

검증된 저장 지도 세션이 없을 때만 사용한다.

1. 기존 SLAM Toolbox mapping 설정이 `/map`과 `map → robot_1/odom`을 발행한다.
2. Nav2의 navigation 서버만 시작하고 `map_server`와 AMCL은 시작하지 않는다.
3. 미탐색 또는 점유 영역의 목표는 Nav2가 거부하거나 재계획한다.

두 모드에서 AMCL과 SLAM Toolbox를 동시에 실행하지 않는다. 둘은 같은
`map → robot_1/odom` 변환의 발행자가 될 수 있다.

## 공통 Nav2 인터페이스

| 입력/출력 | 값 |
| --- | --- |
| global frame | `map` |
| odom frame | `robot_1/odom` |
| robot base frame | `robot_1/base_footprint` |
| laser scan | `/robot_1/scan_raw` |
| velocity output | `/robot_1/controller/cmd_vel` |
| goal input | `/move_base_simple/goal` |
| action output | `/navigate_to_pose` |

Global costmap은 `/map`의 static layer와 inflation layer를 사용한다. Local
costmap은 LiDAR obstacle layer와 inflation layer를 사용한다. 차량 footprint와
속도·가속도 제한은 SDF 치수 및 Gazebo 주행 결과에 맞춰 보수적으로 시작하고 통합
검증으로 조정한다.

## 목표 어댑터

`goal_bridge`는 `/move_base_simple/goal`의 `PoseStamped`를 구독한다.

1. frame이 `map`인지 확인하고, 그렇지 않으면 TF로 `map` 변환을 시도한다.
2. 새 목표가 들어오면 진행 중인 `NavigateToPose` goal을 취소한 뒤 새 goal을 보낸다.
3. Nav2 lifecycle이 active가 아니거나 goal이 유효하지 않으면 속도 0을 발행하고
   상태·실패 이유를 진단 토픽에 발행한다.
4. 성공, 취소, 실패의 모든 종료 경로에서 속도 0을 발행한다.

## 실행 진입점

`nav-up auto [session-id]`를 추가한다.

- session-id가 주어지면 해당 세션만 검증한다.
- 생략하면 최신의 검증 통과 세션을 선택한다.
- 선택 가능한 세션이 없으면 `mapping` 모드로 시작한다.
- 시작 로그와 상태 토픽에 선택된 mode와 session id를 명확히 표시한다.

지도 생성이 끝나 `mapping-stop`이 성공한 경우, 현재 Nav2 goal을 취소하고 0 속도를
보낸 뒤 navigation 서비스를 재시작한다. 새로 publish된 세션을 검증하고 성공하면
localization 모드로 전환한다. 주행 중 `map → odom` 발행자를 교체하지 않는다.

## 배포 및 구성

- Docker 이미지에 ROS 2 Humble용 Navigation2와 Nav2 bringup 패키지를 추가한다.
- `mentorpi_nav` 패키지에 Nav2 parameters, launch 파일, goal bridge를 둔다.
- 기존 mapper와 별도의 `nav2` Compose service를 추가한다. 이 서비스는 sim-adapter와
  선택한 localization 또는 mapping provider가 준비된 뒤 시작한다.
- 실행 스크립트와 README에 모드, session 선택, 중지·상태 확인 절차를 추가한다.

## 검증

1. 지도 세션 선택: 파일 누락·checksum 불일치·정상 세션을 각각 검사한다.
2. 모드 상호 배타성: localization은 AMCL만, mapping은 SLAM Toolbox만
   `map → odom`을 발행함을 검사한다.
3. goal bridge: map-frame goal 전달, 새 goal의 이전 goal 취소, 실패 시 정지 명령을
   단위 테스트한다.
4. 통합: Nav2 lifecycle active, `/move_base_simple/goal` 수신, 경로 생성,
   `/robot_1/controller/cmd_vel` 전달, 목표 종료 후 정지를 확인한다.
5. 회귀: 기존 mapping session 저장·검증과 Foxglove bridge가 유지되는지 검사한다.
