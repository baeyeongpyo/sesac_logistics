---
title: MentorPi Nav2 주행 로직 분석
created: 2026-08-05
updated: 2026-08-05
type: concept
status: review-required
tags:
  - robotics
  - ros2
  - nav2
  - mentorpi
  - simulation
sources:
  - path: vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/config/nav2.yaml
    accessed: 2026-08-05
  - path: vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/launch/navigation.launch.py
    accessed: 2026-08-05
  - path: vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/scripts
    accessed: 2026-08-05
  - path: vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam
    accessed: 2026-08-05
  - path: vehicle_simulator_model/ubuntu/README.md
    accessed: 2026-08-05
  - path: artifacts/vehicle/raw/ros2_ws/src/mentorpi_navigation
    accessed: 2026-08-05
---

# MentorPi Nav2 주행 로직 분석

## 범위와 근거 상태

이 문서는 `vehicle_simulator_model/ubuntu`의 MentorPi 시뮬레이터 Nav2 구현을
분석한 결과다. 대상 `mentorpi_nav/` 디렉터리와 navigation bundle 테스트는
분석 시점에 Git 추적 대상이 아닌 작업 트리 파일이므로, 현재 동작 의도와
정적 계약은 확인했지만 컨테이너 내 실제 Nav2 노드 기동까지 검증한 상태는
아니다. 따라서 아래의 `검증 필요` 항목은 정식 운영 결정 전에 실행 검증이
필요하다.

선택된 Project wiki는 MentorPi M1이 Mecanum 기반 ROS2 플랫폼이고 LiDAR
SLAM/경로 계획을 지원한다는 하드웨어·제품 맥락을 제공한다. 본 문서의
구체적인 토픽, 프레임, 파라미터 및 제어 흐름은 현재 시뮬레이터 소스에서
도출했다.

## 전체 주행 경로

```text
유효한 저장 지도 선택                유효한 저장 지도가 없음
map_server + AMCL                    slam_toolbox
          \                          /
           map -> robot_1/odom TF 및 현재 위치
                            |
Foxglove /move_base_simple/goal (frame_id=map)
                            |
                        goal_bridge
                            |
                  /navigate_to_pose action
                            |
BT navigator -> planner_server -> controller_server
       |              |                  |
       |          전역 경로          /cmd_vel_nav
       |                                 |
       +-- behavior_server           cmd_vel_relay
                                        |
                         /robot_1/controller/cmd_vel
                                        |
                       Gazebo Mecanum 차량 및 /robot_1/odom
```

주행은 기본적으로 `NavigateToPose` 하나만 노출한다. waypoint follower나
`NavigateThroughPoses`를 현재 `mentorpi_nav/config/nav2.yaml`에서 명시적으로
구성하지 않았다. 새 목표는 이전 action을 취소하고 새 목표를 전송하는 방식으로
선점한다. `goal_bridge`는 완료·취소·실패 모두에서 0 속도를 직접 발행한다.

## 1. 지도 선택과 위치 추정

`run_navigation.sh`는 `/slam-data`에서 지도 세션을 먼저 찾는다. `map_session.py`
는 `map.yaml`, `map.pgm`, `manifest.json`, `checksums.sha256`가 모두 존재하고
SHA-256 검증을 통과한 세션만 유효하다고 판단한다. 사용자가 세션 ID를 주지
않으면 `created_at` 기준 최신 유효 세션을 선택한다.

- 유효한 지도: `navigation.launch.py mode:=localization`으로 `map_server`와
  AMCL을 시작한다.
- 유효한 지도가 없음: `mode:=mapping`으로 `slam_toolbox`를 시작한다.
- 두 모드는 조건부 launch로 상호 배타적이어서 둘이 동시에 `map -> odom`
  변환을 제공하지 않도록 설계됐다.

AMCL은 `map`을 전역 프레임으로, `robot_1/odom`을 odometry 프레임으로,
`robot_1/base_footprint`를 기준 프레임으로 사용한다. 입력은
`/robot_1/scan_raw` LaserScan과 `/robot_1/odom`이며, Mecanum 차량에 맞게
`nav2_amcl::OmniMotionModel`을 선택했다. 파티클 수는 300~1200이고,
0.05 m 또는 0.05 rad 이상 움직일 때 갱신한다.

SLAM fallback은 5 cm 해상도, 10 m 최대 LiDAR 범위, loop closure 및 scan
matching을 사용한다. README가 설명하듯 이는 임시 탐색·주행용 지도이며,
재사용할 지도는 별도 mapping session에서 저장·checksum 검증해야 한다.

### 위치 추정의 운영상 의미

- localization 모드에서는 `/initialpose`를 `map` 프레임으로 한 번 입력해
  AMCL 수렴을 돕는다.
- LiDAR, odom, TF의 시간·좌표가 맞지 않으면 planner가 정상이어도 경로가
  튀거나 costmap이 비어 보일 수 있다.
- 현재 세션 선택은 checksum과 기본 manifest 필드만 검증한다. `robot_id`,
  world/model/TF calibration version 또는 지도 품질은 선택 조건이 아니므로,
  서로 다른 환경에서 만든 지도 사용은 별도 호환성 정책이 필요하다.

## 2. 목표 수신과 action 관리

`goal_bridge.py`는 `/move_base_simple/goal`의 `PoseStamped`를 구독한다.
목표 프레임이 정확히 `map`이 아니면 상태 topic에 거절 사유를 내보내고 즉시
정지한다. 목표가 허용되면 `/navigate_to_pose` action으로 전송한다.

| 상황 | bridge 동작 |
|---|---|
| `frame_id != map` | 거절, 0 속도 발행 |
| Nav2 action server가 1초 안에 없음 | 거절, 0 속도 발행 |
| 실행 중 새 목표 수신 | 기존 goal 취소 후 새 goal 전송 |
| action 성공/취소/실패 | 상태 발행 후 0 속도 발행 |

이 구현은 과거 goal의 늦은 result callback이 현재 goal 상태를 덮지 않도록
goal handle identity를 비교한다. 해당 선점 보호와 토픽/프레임 계약은 단위
테스트로 확인돼 있다.

## 3. 전역 경로 판단

`planner_server`는 `nav2_navfn_planner/NavfnPlanner`를 사용한다.
`use_astar: false`이므로 이 설정의 탐색은 A*가 아니라 NavFn의 Dijkstra 계열
potential-field 탐색이다. planner 기대 주기는 5 Hz, 목표 허용 오차는 0.20 m다.

전역 costmap은 `map` 프레임의 5 cm 격자이며 다음 세 층을 합친다.

1. `StaticLayer`: 저장된 occupancy map의 벽·고정 장애물.
2. `ObstacleLayer`: `/robot_1/scan_raw`에서 8 m 이내 장애물을 marking하고
   10 m까지 raytrace하여 사라진 장애물을 clearing한다.
3. `InflationLayer`: 로봇 주변 0.30 m에 비용을 퍼뜨리고
   `cost_scaling_factor: 4.0`으로 거리별 비용을 감소시킨다.

`track_unknown_space: true`와 `allow_unknown: false`의 조합으로, 지도에서
unknown인 칸은 경로로 사용할 수 없다. 즉 이 시스템은 최단 거리보다
"통과 가능하고 장애물에서 여유가 있는" 낮은 누적 비용 경로를 택한다.

## 4. 지역 비용 지도와 경로 추종

지역 costmap은 `robot_1/odom` 프레임에서 로봇과 함께 이동하는 4 m × 4 m
rolling window다. 역시 5 cm 해상도이며, LiDAR 장애물은 6 m marking / 8 m
clearing 범위로 반영한다. static map은 지역 지도에 직접 넣지 않고 전역
지도와 planner가 장거리 구조를 담당한다.

차량 충돌 형상은 global/local costmap 모두 다음 polygon이다.

```text
[[0.18, 0.15], [0.18, -0.15], [-0.18, -0.15], [-0.18, 0.15]]
```

따라서 Nav2는 0.36 m × 0.30 m 외접 직사각형을 기준으로 통과 가능 여부를
판단한다. 이 값이 실제 포크, 카메라, 적재물을 포함하는지 실측 검증이
필요하다. footprint보다 실제 전방 돌출물이 길면 Nav2는 충돌을 예측하지
못한다.

`controller_server`는 15 Hz의
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`를
쓴다. 이 controller는 전역 경로 위에서 lookahead(기본 0.35 m, 속도에 따라
0.20~0.55 m)를 잡고, 그 지점을 향하는 선형·각속도를 만든다.

| 제어 항목 | 현재 값 | 주행 영향 |
|---|---:|---|
| 희망 선속도 | 0.18 m/s | 일반 주행 상한 |
| 목표 접근 최소 속도 | 0.05 m/s | 목표 근처 감속 |
| lookahead 시간 | 1.5 s | 속도에 따른 앞보기 거리 조절 |
| 충돌 검사 | 사용 | carrot까지 최대 1.0초 충돌 예상 검사 |
| 방향 맞춤 회전 | 사용 | 경로 방향과 다르면 0.5 rad/s로 회전 |
| 목표 허용 오차 | 0.12 m, 0.20 rad | 이 범위에서 도착 판정 |
| progress 조건 | 8초 내 0.08 m 이동 | 미진행 시 실패/복구 경로 |

Mecanum 차량이고 AMCL은 omni motion model이지만, 현재 Pure Pursuit 파라미터에는
횡이동(`linear.y`)을 활용하는 정책이 없다. 따라서 일반 경로추종은 전진·회전
중심으로 검증하고, 정밀 측면 정렬은 별도 docking controller 및 명령 중재로
분리하는 것이 안전하다.

## 5. 복구와 최종 속도 전달

`behavior_server`에는 `spin`, `backup`, `drive_on_heading`, `wait`가 등록돼
있다. 실제 어떤 순서로 재계획/복구를 실행할지는 BT XML이 아니라
`nav2_bringup`의 기본 NavigateToPose 동작에 의존한다. 현재 구성은
`default_nav_to_pose_bt_xml`을 고정하지 않았으므로, Nav2 패키지 버전이
달라지면 정확한 복구 트리도 달라질 수 있다.

속도 전달은 설계상 다음과 같다.

```text
/cmd_vel_nav -> cmd_vel_relay -> /robot_1/controller/cmd_vel
```

relay는 20 Hz timer로 마지막 명령을 감시하고, 0.35초 동안 새 명령이 없으면
`Twist()`를 발행해 차량을 정지시킨다. 이는 Nav2 또는 통신 노드 종료 시
무한히 마지막 속도가 유지되는 위험을 줄인다.

## 검증 필요 및 개선 우선순위

### P0 — 실제 Nav2 토픽 연결 확인

현재 `navigation.launch.py`는 `nav2_bringup/navigation_launch.py`를 include하고,
relay는 `/cmd_vel_nav`를 구독한다. 그러나 이 파일 자체에는 Nav2 controller의
`cmd_vel`을 `/cmd_vel_nav`로 remap하는 선언이 없다. 과거 reference 구현의
`shared_map_navigation.launch.py`에는 해당 remap이 명시돼 있다.

따라서 실제 컨테이너에서 다음을 확인해야 한다.

```bash
./run.sh --env dev nav-up auto
./run.sh --env dev nav-status
ros2 topic info /cmd_vel_nav -v
ros2 topic info /robot_1/controller/cmd_vel -v
```

Nav2 controller publisher가 `/cmd_vel_nav`에 나타나지 않으면, relay는 정상이어도
차량에는 주행 명령이 전달되지 않는다. 이 항목은 정적 contract test가
relay의 구독 이름만 확인하므로 아직 검출하지 못한다.

### P0 — standard Nav2 launch와 parameter/lifecycle 정합성 확인

현재 config의 `lifecycle_manager_navigation.node_names`는 controller, planner,
behavior, BT navigator만 나열한다. 반면 표준 `nav2_bringup/navigation_launch.py`
가 시작하는 node 집합은 Nav2 배포판에 따라 smoother, waypoint follower,
velocity smoother 등을 포함할 수 있다. 현재 config에는 이 노드들의 plugin
파라미터가 없다.

컨테이너 로그에서 lifecycle activation 실패가 없는지 확인하고, 실제 필요한
node만 직접 launch하거나 Nav2 버전에 맞는 전체 params/lifecycle 구성을
명시해야 한다.

### P1 — BT와 recovery 정책 고정

현재 기본 BT 의존성을 제거하려면 프로젝트의 `NavigateToPose` BT XML을 패키지에
두고 `default_nav_to_pose_bt_xml`을 지정한다. 포크 장착 상태에서는 좁은
통로의 `spin` recovery가 위험할 수 있으므로, 회전 허용 조건 또는 backup
우선 정책을 BT 수준에서 정한다.

### P1 — 실물/적재 상태 footprint 및 safety arbitration

포크·적재물이 있다면 global/local costmap의 동일 footprint를 상태별로 전환하고,
LiDAR 사각지대는 별도 collision monitor 또는 근거리 센서로 보완한다. 또한
현재 goal bridge의 정지 publisher와 relay가 동일 최종 command topic에 닿으므로,
수동 정지·safety stop·docking·Nav2를 포함할 때는 하나의 command arbiter만
`/robot_1/controller/cmd_vel`에 발행하도록 바꾼다.

### P2 — 지도 호환성 검증 확장

지도 세션 manifest에 기록되는 robot/world/model/TF calibration 정보를
`map_session.py`의 선택 조건에도 반영한다. checksum은 파일 변조만 방지하며,
다른 월드 또는 보정값에서 생성한 유효한 지도의 오사용은 막지 못한다.

## 현재 검증 결과

다음 정적 테스트를 2026-08-05에 실행했고 모두 통과했다.

```text
python3 -m unittest discover -s vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/test -p 'test_*.py' -v
# 9 tests passed

python3 -m unittest vehicle_simulator_model/ubuntu/test/test_navigation_bundle.py -v
# 4 tests passed
```

테스트는 세션 checksum 선택, navigation 구성 문자열 계약, goal 선점 보호,
컨테이너 bundle 선언을 검증한다. Gazebo에서 실제로 목표를 전송해 planner,
controller, `/cmd_vel_nav`, costmap, TF를 종단 간 검증하는 integration test는
아직 확인되지 않았다.
