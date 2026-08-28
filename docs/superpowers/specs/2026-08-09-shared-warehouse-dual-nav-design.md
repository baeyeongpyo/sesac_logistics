# 공용 Warehouse 이중 Nav2 설계

## 목표

하나의 Gazebo warehouse와 하나의 검증된 저장 PGM 지도를 `robot_1`, `robot_2`가
공유한다. 각 차량은 자기 센서로 AMCL 위치추정과 Nav2 주행을 독립 실행하고,
Foxglove는 같은 좌표계에서 창고·두 차량·센서·주행 상태를 함께 표시한다.

## 범위

- `sim-up`은 Gazebo, ROS adapter, Foxglove Bridge를 함께 시작하는 현재 동작을 유지한다.
- 저장 지도 주행에서는 map server가 `/map`을 한 번만 제공한다.
- `robot_1`, `robot_2`는 각각 AMCL과 Nav2 namespace, 목표 입력, 속도 출력을 갖는다.
- 각 차량의 LiDAR scan은 자기 local/global costmap의 관측 입력이며, 상대 차량을
  센서 범위에서 관측하면 동적 장애물로 반영한다.
- Foxglove는 `map`을 Fixed frame으로 사용해 `/map`, `/warehouse_scene/static`,
  `/warehouse_scene/dynamic`, 두 차량의 scan·odom·주행 상태를 함께 표시한다.

## 범위 밖

- 차량 간 우선순위, 구역 예약, 양보, 추월 금지, 교착 해소 같은 fleet/traffic 제어
- 상대 차량을 항상 탐지한다고 보장하는 전역 충돌 예측
- 충돌 이벤트 기반의 강제 정지 safety arbiter
- 다중 차량 SLAM과 지도 병합

Gazebo의 물리 충돌 형상은 유지한다. 다만 이번 변경의 상호 인지는 각 차량 LiDAR와
costmap에 한정된다. 센서 사각, 시간 지연, 동시 교차 진입은 별도 safety/fleet 기능이
필요하다.

## 좌표계 및 소유권

저장 지도 주행의 단일 TF tree는 다음과 같다.

```text
map
├── warehouse                       (정적 map-to-warehouse 보정 TF)
├── robot_1/odom                    (robot_1 AMCL만 발행)
│   └── robot_1/base_footprint
└── robot_2/odom                    (robot_2 AMCL만 발행)
    └── robot_2/base_footprint
```

`warehouse`는 절대로 `robot_1/odom` 또는 `robot_2/odom`의 자식으로 두지 않는다.
PGM 원점과 Gazebo world 원점의 보정은 `map -> warehouse` 정적 TF가 담당하며,
기본값은 기존 시뮬레이션 지도와 SDF가 같은 원점을 사용한다는 가정의 identity다.

AMCL은 각 차량의 `map -> robot_i/odom`만 발행한다. map server, warehouse TF,
각 차량의 odom-to-base TF는 이 변환을 발행하지 않는다. 이 규칙으로 TF의 부모
중복을 방지한다.

## 구성 요소

### Gazebo와 Foxglove scene

`sim_adapter`는 warehouse SDF와 scene publisher를 계속 실행한다. SceneUpdate의
frame은 `warehouse`이며 static scene은 고정 구조를, dynamic scene은 `robot_1`,
`robot_2`, pallet 상태를 발행한다. Gazebo-only 모드에서는 Foxglove Fixed frame을
`warehouse`로 사용한다.

### 공용 지도와 차량별 navigation

다중 차량 navigation launch는 유효한 map session에서 map server를 하나만 기동한다.
각 차량은 독립 namespace 안에서 AMCL, planner, controller, behavior, BT navigator,
goal bridge, cmd_vel relay를 실행한다. 차량별 계약은 다음과 같다.

```text
/robot_i/move_base_simple/goal
  -> /robot_i/navigate_to_pose
  -> /robot_i/cmd_vel_nav
  -> /robot_i/controller/cmd_vel

/robot_i/scan_raw
/robot_i/odom
/robot_i/initialpose
/robot_i/navigation/status
```

목표와 initial pose의 frame_id는 `map`이다. mapping fallback은 robot_1 단일 차량
지도 생성 흐름을 유지하며, 동시 이중 차량 Nav2는 검증된 저장 지도에서만 지원한다.

### 상호 인지

두 차량은 같은 Gazebo physics world에 존재한다. 각 차량의 LiDAR가 상대 차량을
가시 범위에서 관측하면 자기 scan과 자기 costmap에 반영한다. 이 기능은 상대의
namespace 토픽을 직접 구독해 위치를 주입하지 않는다. 따라서 실제 센서 기반
장애물 인지와 같은 성질을 유지한다.

## 오류 처리 및 검증

- map session이 유효하지 않으면 이중 차량 localization을 시작하지 않고 단일 차량
  mapping fallback을 명시적으로 보고한다.
- 두 AMCL 중 하나가 활성화되지 않거나 TF 부모 중복이 있으면 navigation health check가
  실패해야 한다.
- 각 차량 goal bridge는 자기 goal/action/cmd_vel 토픽만 사용해야 한다.
- 회귀 테스트는 warehouse가 robot_1 또는 robot_2의 자식이 아님, 두 AMCL contract가
  공용 `/map` 아래 각자 odom을 소유함, robot별 토픽 격리가 유지됨을 검증한다.
- 통합 검증은 `sim-up` 후 이중 navigation을 시작하고, Foxglove와 ROS graph에서
  공용 map과 두 차량 TF·scan·cmd_vel을 확인한다.

## 수용 기준

1. Foxglove의 `map` 프레임에서 warehouse와 두 차량을 동시에 표시할 수 있다.
2. `warehouse`는 Foxglove TF tree에서 robot_1 또는 robot_2 하위가 아니다.
3. `robot_1`, `robot_2`는 같은 `/map`에 대해 서로 다른 `map -> robot_i/odom` TF를 가진다.
4. 두 차량에 동시로 서로 다른 목표를 보낼 수 있고 속도 명령은 해당 차량에만 전달된다.
5. 각 차량의 costmap 관측 입력은 자기 LiDAR scan이며, 공용 Gazebo world의 상대 차량을
   관측 가능하다.
