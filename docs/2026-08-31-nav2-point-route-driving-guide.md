# MentorPi Nav2 point1~point4 주행 가이드

## 목적과 범위

이 문서는 MentorPi M1에서 RViz의 Goal Pose를 반복해서 지정하지 않고,
`point1 → point2 → point3 → point4` 순서의 주행 명령을 실행하는 방법을
정리한다. 설명과 실제 실행 절차를 분리하며, 예시는 프로젝트의
`mentorpi_navigation` 2대 로봇 shared-map 구성에서 `robot_1`을 대상으로 한다.

point는 `map` 프레임의 절대 좌표다. 따라서 사용 전 저장 지도, AMCL 위치추정,
`map → odom → robot_1/base_footprint` TF가 정상이어야 한다.

> 주의: 아래 코드 근거는 `artifacts/vehicle/raw/`의 보존된 참조 소스다. 이
> 파일을 직접 수정하지 않는다. 실제 차량에서는 실행 중인 패키지 버전과 action,
> topic, 파라미터를 먼저 확인한다.

## 1. 설명

### 1.1 방식 선택

| 방식 | Goal Pose UI 필요 | point1~4의 의미 | 추천 상황 |
| --- | --- | --- | --- |
| `NavigateThroughPoses` | 아니오 | 순서대로 경유하는 pose | 일반적인 물류 경유 주행; 기본 추천 |
| `FollowWaypoints` | 아니오 | 각 point 도착 후 다음 point 실행 | 적재, 촬영, 대기 같은 point별 작업 필요 |
| `FollowPath` | 아니오 | 사전에 만든 경로의 pose 샘플 | 정해진 통로를 촘촘하고 일관되게 추종 |
| 카메라/LiDAR 추종 | 아니오 | 물리 라인·마커·대상으로 point를 식별 | 좌표보다 라인/표식이 기준인 환경 |
| `DriveOnHeading` | 아니오 | 현재 방향 기준의 짧은 변위 | 최종 point의 도킹 전진·후진 보정 |

### 1.2 현재 프로젝트에서의 실행 경로

`shared_map_navigation.launch.py`는 `robot_1`, `robot_2` 각각에 AMCL과 Nav2
서버를 올린다. `robot_1`의 주요 경로는 다음과 같다.

```text
/robot_1/navigate_through_poses
  → /robot_1/bt_navigator
  → ComputePathThroughPoses
  → /robot_1/controller_server (FollowPath/DWB)
  → /robot_1/cmd_vel_nav
  → /robot_1/velocity_smoother
  → /robot_1/cmd_vel_smoothed
  → /robot_1/collision_monitor
  → /robot_1/controller/cmd_vel
  → chassis controller → motors
```

현재 `navigate_through_poses.xml`은 1 Hz로 경로를 다시 계산한 뒤 DWB
`FollowPath` controller로 추종한다. 이 XML에는 recovery node가 없으므로,
장애물이 계속 막고 있는 경우 복구 동작을 반복하기보다 action 실패가 날 수
있다는 점을 운용 절차에 반영해야 한다.

현재 프로젝트의 DWB 제한은 최대 전진 속도 `0.22 m/s`, 최대 각속도
`0.75 rad/s`이며, `linear.y`는 `0.0`으로 제한된다. Mecanum 하드웨어가
횡이동을 지원하더라도 이 Nav2 구성은 횡이동하지 않는다.

### 1.3 방식별 동작 차이

#### NavigateThroughPoses — 기본 선택

`PoseStamped[]` 배열에 point1~4를 넣어 한 번 요청한다. Nav2가 현재
위치에서 point1로 가고, 중간 point를 경유하여 point4를 최종 도착점으로
처리한다. 중간 지점에서 확정적으로 멈추는 용도는 아니다.

이 방식은 Goal Pose 버튼을 누르지 않을 뿐 Nav2 내부에서는 pose goal을
사용한다. 장점은 planner, local costmap, DWB, 속도 제한, Collision Monitor
경로를 그대로 활용한다는 것이다.

#### FollowWaypoints — 지점별 작업이 있을 때

같은 pose 배열을 받아 각 waypoint를 개별적으로 완료한다. 현재 설정의
`WaitAtWaypoint` 대기 시간은 `0초`이므로, 기본값만으로는 실제 대기나 적재
작업을 하지 않는다. point마다 작업이 필요하면 대기 시간을 설정하거나
waypoint task executor를 추가해야 한다.

#### FollowPath — 티칭된 통로를 따라갈 때

외부 노드가 `nav_msgs/Path`를 만들고 `/robot_1/follow_path` action에
전달한다. point1~4만 네 개 넣을 수는 있지만, 코너를 깎거나 경로가 느슨해질
수 있다. 선분을 5~10 cm 간격으로 보간하고, 통로 중심선을 따라 여러 pose를
넣는 것이 적합하다.

`FollowPath`는 전역 planner가 새 path를 만들어 주는 요청이 아니다. 외부에서
제공한 path가 기준이며, local controller와 costmap은 근거리 장애물 회피에
사용된다. 장시간 막힘에 대한 재계획 정책은 path 생성 노드 또는 상위 mission
node가 별도로 결정해야 한다.

#### 카메라/LiDAR 추종 — 물리 표식이 기준일 때

기존 앱에는 line following, LiDAR obstacle avoidance/following, object
tracking 노드가 있다. 이들은 좌표 point를 이해하지 않으므로 다음 중 하나가
추가로 필요하다.

1. 각 point에 색상 마커, AprilTag, 라인 교차점 같은 식별 기준을 둔다.
2. 추종 노드가 point 식별 시 mission 상태를 `1 → 2 → 3 → 4`로 진행한다.
3. point4를 인식하면 정지 또는 정밀 docking 동작으로 전환한다.

이 방식은 기존 앱이 `controller/cmd_vel`에 직접 발행하면 Collision Monitor를
우회할 수 있다. Nav2 명령과 절대로 같은 topic을 경쟁 발행하지 말고, command
mux/arbiter로 한 소스만 선택한 뒤 Collision Monitor를 마지막 단계로 둔다.

```text
Nav2 또는 vision/LiDAR follower
  → /robot_1/<selected_cmd_vel>
  → command mux/arbiter
  → /robot_1/cmd_vel_smoothed
  → Collision Monitor
  → /robot_1/controller/cmd_vel
```

#### DriveOnHeading — 최종 위치 보정용

절대 지도 좌표가 아니라 현재 차량 heading 기준의 짧은 거리 이동이다. 따라서
여러 번 꺾이는 point1~4 임무의 주된 주행 수단으로는 맞지 않고, point4 도착 후
선반 쪽으로 0.3 m 전진하는 용도에 적합하다. 현재 설정에는 `drive_on_heading`은
등록되어 있지만 `spin`은 등록되어 있지 않다.

### 1.4 안전 및 운영 원칙

- Nav2, joystick, 앱 노드가 동시에 chassis 명령을 발행하지 않도록 한다.
- 현재 Collision Monitor의 입력은 `cmd_vel_smoothed`, 출력은
  `controller/cmd_vel`이다. 신규 명령도 이 안전 경로를 거쳐야 한다.
- 실제 차량 크기·포크·적재물을 반영한 footprint와 Collision Monitor polygon을
  저속에서 검증한다.
- 현재 shared-map launch는 `use_sim_time: true`를 고정한다. 실제 차량에
  `/clock`이 없다면, 이 launch를 그대로 운용하지 말고 운용용 설정에서
  simulation time을 끈다.
- 지도 생성 SLAM과 저장 지도 기반 AMCL은 같은 로봇에서 동시에
  `map → odom` TF를 발행하면 안 된다.

## 2. 실제 사용 방법 및 예시

### 2.1 사전 점검

아래 명령은 차량에서 ROS 환경을 source한 뒤 실행한다. workspace 경로는 실제
설치 경로로 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source <vehicle-workspace>/install/setup.bash

ros2 lifecycle get /robot_1/bt_navigator
ros2 lifecycle get /robot_1/controller_server
ros2 topic echo /robot_1/amcl_pose --once
ros2 action list -t
ros2 topic echo /robot_1/controller/cmd_vel
```

`bt_navigator`와 `controller_server`가 `active`여야 하며, `amcl_pose`가
연속적으로 나와야 한다. simulation time 여부도 확인한다.

```bash
ros2 param get /robot_1/bt_navigator use_sim_time
ros2 topic echo /clock --once
```

`use_sim_time`이 `true`인데 `/clock`이 없으면 시간 기반 Nav2 동작이 진행되지
않을 수 있다. 이 경우 차량 운용 전에 launch/configuration을 바로잡는다.

### 2.2 예시 좌표 정의

다음은 설명용 point다. 실제 지도에서 얻은 좌표와 각도로 반드시 교체한다.

| Point | x (m) | y (m) | yaw (rad) | 용도 예시 |
| --- | ---: | ---: | ---: | --- |
| point1 | 0.50 | -0.80 | 0.00 | 통로 진입 |
| point2 | 1.20 | -0.80 | 0.00 | 첫 번째 교차점 |
| point3 | 1.20 | 0.00 | 1.57 | 랙 통로 전환 |
| point4 | 2.00 | 0.00 | 0.00 | 목적지 앞 |

ROS pose orientation은 quaternion이다. 2D yaw `θ`에 대해 `z = sin(θ / 2)`,
`w = cos(θ / 2)`를 사용한다. 예를 들어 yaw=0은 `z=0.0, w=1.0`, yaw=π/2는
`z≈0.7071, w≈0.7071`이다.

재사용할 pose 배열을 shell 변수에 넣는다.

```bash
points='[
  {header: {frame_id: map}, pose: {position: {x: 0.50, y: -0.80, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 1.20, y: -0.80, z: 0.0}, orientation: {z: 0.0, w: 1.0}}},
  {header: {frame_id: map}, pose: {position: {x: 1.20, y:  0.00, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}},
  {header: {frame_id: map}, pose: {position: {x: 2.00, y:  0.00, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}
]'
```

### 2.3 권장 실행: NavigateThroughPoses

```bash
ros2 action send_goal --feedback \
  /robot_1/navigate_through_poses \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: $points, behavior_tree: ''}"
```

`--feedback` 출력의 `number_of_poses_remaining`, `distance_remaining`을 보고
진행 상태를 확인한다. 즉시 취소하려면 다음 명령을 사용한다.

```bash
ros2 action cancel /robot_1/navigate_through_poses
```

### 2.4 point별 작업: FollowWaypoints

```bash
ros2 action send_goal --feedback \
  /robot_1/follow_waypoints \
  nav2_msgs/action/FollowWaypoints \
  "{poses: $points}"
```

이 action은 feedback의 `current_waypoint`으로 현재 처리 중인 지점 번호를
알려 준다. 현 설정은 point 도착 후 대기 시간이 0초다. point2에서 적재물을
확인해야 한다면, 현장 배포 설정의
`wait_at_waypoint.waypoint_pause_duration`을 변경하거나 전용 task executor를
사용한다.

### 2.5 고정 통로 추종: FollowPath

먼저 action 인터페이스가 현재 차량의 ROS 배포판과 일치하는지 확인한다.

```bash
ros2 interface show nav2_msgs/action/FollowPath
```

형식 시험은 아래와 같이 할 수 있다. 실제 운용에서는 `$points` 대신 현재
위치부터 point1~4를 지나는 촘촘한 pose 배열을 넣는다.

```bash
ros2 action send_goal --feedback \
  /robot_1/follow_path \
  nav2_msgs/action/FollowPath \
  "{path: {header: {frame_id: map}, poses: $points},
    controller_id: FollowPath,
    goal_checker_id: goal_checker}"
```

외부 path generator는 다음을 지켜야 한다.

1. 첫 pose를 현재 pose 또는 안전한 path 합류점으로 둔다.
2. 통로 직선부와 코너를 5~10 cm 간격으로 보간한다.
3. 각 pose의 `header.frame_id`를 `map`으로 통일한다.
4. 장애물로 인한 장기 정지·재계획·재시도 정책을 mission node에 둔다.

### 2.6 라인 추종 예시

기존 MentorPi 앱을 단독으로 시험할 때의 명령이다.

```bash
ros2 launch app line_following_node.launch.py debug:=true

ros2 service call /line_following/enter std_srvs/srv/Trigger {}
ros2 service call /line_following/set_running \
  std_srvs/srv/SetBool "{data: true}"
```

이 명령을 Nav2와 동시에 운용하기 전에는 command mux/arbiter와 Collision
Monitor 경로를 구성해야 한다. 원본 앱의 `controller/cmd_vel` 직접 출력만으로
point1~4를 좌표 기준으로 경유시키는 기능은 제공되지 않는다.

### 2.7 point4 도착 후 전진 보정: DriveOnHeading

```bash
ros2 action send_goal --feedback \
  /robot_1/drive_on_heading \
  nav2_msgs/action/DriveOnHeading \
  "{target: {x: 0.30, y: 0.0, z: 0.0},
    speed: 0.10,
    time_allowance: {sec: 5, nanosec: 0}}"
```

`target.x=0.30`은 현재 heading 방향으로 0.3 m 이동한다는 의미다. 장애물
확인을 끄는 옵션을 사용하지 않는다. 이 action은 docking 정밀도를 보장하지
않으므로, 포크 삽입이나 팔레트 정렬은 별도의 거리·마커 기반 정밀 제어로
완료한다.

### 2.8 권장 검증 순서

1. 바퀴가 공중에 뜬 상태에서 action 목록과 topic 흐름을 확인한다.
2. 장애물이 없는 넓은 공간에서 point1 하나만 저속으로 시험한다.
3. point1→point2 두 지점 주행 후, 실제 footprint와 costmap을 RViz에서
   확인한다.
4. point1→point4 전체 임무를 저속으로 수행하고, 각 point의 통과 순서와
   point4 자세를 확인한다.
5. 안전한 위치에 장애물을 놓아 Collision Monitor 정지/감속을 확인한다.
6. joystick, 앱 노드, Nav2를 함께 켠 상태에서는 command mux의 우선순위와
   action 취소를 검증한다.

## 참고 근거

- [프로젝트 Nav2 launch](../artifacts/vehicle/raw/ros2_ws/src/mentorpi_navigation/launch/shared_map_navigation.launch.py)
- [프로젝트 Nav2 파라미터](../artifacts/vehicle/raw/ros2_ws/src/mentorpi_navigation/config/nav2.yaml)
- [NavigateThroughPoses 행동 트리](../artifacts/vehicle/raw/ros2_ws/src/mentorpi_navigation/behavior_trees/navigate_through_poses.xml)
- [Collision Monitor 파라미터](../artifacts/vehicle/raw/ros2_ws/src/mentorpi_safety/config/collision_monitor.yaml)
- [MentorPi M1 navigation 스택 분석](../llm-wiki/concepts/mentorpi-m1-navigation-stack.md)
- [MentorPi 앱 제어 구현 가이드](../artifacts/vehicle/sources/hiwonder-mentorpi-getting-ready-implementation-guide.md)
- [Nav2 Humble NavigateThroughPoses API](https://api.nav2.org/actions/humble/navigatethroughposes.html)
- [Nav2 Humble FollowPath action 정의](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_msgs/action/FollowPath.action)
- [Nav2 Humble DriveOnHeading action 정의](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_msgs/action/DriveOnHeading.action)
