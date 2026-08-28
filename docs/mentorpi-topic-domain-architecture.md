# MentorPi 토픽 도메인 아키텍처

## 목적과 범위

이 문서는 MentorPi 시뮬레이터의 ROS 2 topic을 운영·관찰 목적에 따라 분류한다.
현재 구현은 `robot_1` 한 대를 기준으로 설명하며, 향후에는 차량별 topic의
`robot_1` 부분을 `robot_id`로 바꿔 같은 계약을 적용한다.

핵심 원칙은 Gazebo의 3D 환경 데이터와 Nav2가 사용하는 2D 주행 지도를 혼동하지
않는 것이다. Foxglove 레이아웃과 topic 관리표도 아래 네 묶음을 기준으로 한다.

## 네 개의 topic 묶음

| 묶음 | 목적 | 대표 topic | 주 생산자 |
| --- | --- | --- | --- |
| 차량 센서·상태 | 차량이 관측한 값과 운동 상태를 전달 | `/robot_1/scan_raw`, `/robot_1/depth/image_raw`, `/robot_1/depth/camera_info`, `/robot_1/imu/data_raw`, `/robot_1/odom`, `/robot_1/ground_truth/pose` | Gazebo 차량 모델, ROS-Gazebo bridge |
| Gazebo 3D 환경 | 창고·팔레트 등 시뮬레이터 환경을 Foxglove 3D에 표시 | `/warehouse_scene/static`, `/warehouse_scene/dynamic` | SDF scene publisher |
| 지도·위치추정 | 로봇이 경로계획 기준으로 쓰는 2D 지도와 현재 위치를 제공 | `/map`, `/initialpose` | SLAM, map server, AMCL |
| Nav2 주행 제어 | 목표 수신부터 속도 명령과 결과 상태까지 제어 | `/move_base_simple/goal`, `/navigate_to_pose`, `/cmd_vel_nav`, `/robot_1/controller/cmd_vel`, `/robot_1/navigation/status` | goal bridge, Nav2, velocity relay |

`/tf`와 `/tf_static`은 별도 업무 묶음이 아니다. 네 묶음의 데이터를 하나의
좌표계에서 해석하게 만드는 공통 기반이다.

```text
map -> robot_1/odom -> robot_1/base_footprint -> lidar / depth_cam / imu
                       \
                        -> warehouse
```

## 1. 차량 센서·상태

이 묶음은 차량이 보는 환경과 실제 운동 결과다.

- `/robot_1/scan_raw`: 2D LiDAR `LaserScan`. 현재 SLAM, AMCL, Nav2의 LiDAR
  기반 관측 입력이다.
- `/robot_1/depth/image_raw`, `/robot_1/depth/camera_info`: depth camera 영상과
  내부 파라미터다. 현재는 관제·검증용이며, depth 기반 장애물 처리나 costmap
  정책은 아직 추가하지 않는다.
- `/robot_1/imu/data_raw`: IMU 측정값이다.
- `/robot_1/odom`: mecanum drive가 발행하는 차량의 누적 운동 추정값이다.
- `/robot_1/ground_truth/pose`: Gazebo가 아는 시뮬레이터 기준 실제 자세다.
  Nav2 입력이 아니라 시뮬레이터 검증·관찰용이다.

## 2. Gazebo 3D 환경

Gazebo의 원본 환경은 `warehouse.sdf`와 포함된 모델이다. 이것은 벽, 랙, 바닥,
팔레트의 물리·충돌·렌더링 정의이며 Nav2 `/map`이 아니다.

```text
warehouse.sdf
  -> /warehouse_scene/static       정적 벽·랙·바닥을 Foxglove SceneUpdate로 발행

Gazebo /warehouse/entity_poses
  -> ROS bridge
  -> /warehouse_scene/dynamic      움직이는 팔레트 등의 자세를 SceneUpdate로 발행
```

`/warehouse/entity_poses`는 Gazebo `Pose_V`를 ROS `TFMessage` 형태로 bridge한
내부 입력이다. Foxglove 운영 화면에서는 일반적으로 정적·동적 scene topic 두
개를 구독하면 된다.

## 3. 지도·위치추정

`/map`은 점유 격자로 표현된 2D 주행 지도다. 다음 두 방식으로 제공된다.

- 지도 생성 모드: `robot_1` LiDAR scan을 `slam_toolbox`가 처리해 임시 지도를
  생성하고, 세션 종료 시 `map.yaml`과 `map.pgm`으로 저장한다.
- 저장 지도 주행 모드: `map_server`가 검증된 저장 지도를 `/map`으로 제공하고,
  AMCL이 LiDAR와 `/robot_1/odom`을 이용해 `map -> robot_1/odom` 변환을 추정한다.

`/initialpose`는 운영자가 Foxglove에서 AMCL에 대략적인 초기 위치와 방향을
지정하는 입력이다. 이 묶음의 목적은 3D 시각화가 아니라 Nav2가 차량의 위치를
알고 경로를 만들 수 있게 하는 것이다.

## 4. Nav2 주행 제어

현재 단일 차량의 목표·속도 데이터 흐름은 다음과 같다.

```text
Foxglove /move_base_simple/goal (frame_id=map)
  -> goal_bridge
  -> /navigate_to_pose Action
  -> Nav2 planner/controller
  -> /cmd_vel_nav
  -> cmd_vel_relay
  -> /robot_1/controller/cmd_vel
  -> Gazebo mecanum drive
```

`/robot_1/navigation/status`에는 목표 접수·성공·취소·실패 상태가 발행된다.
속도 명령이 중단되면 watchdog relay가 정지 `Twist`를 발행한다.

## Foxglove 운영 분류

Foxglove에서는 네 개의 layout 또는 패널 묶음으로 시작한다.

1. **차량 상태**: robot model, LiDAR, depth image, IMU, odometry
2. **Gazebo 3D 환경**: `/warehouse_scene/static`, `/warehouse_scene/dynamic`
3. **지도·위치추정**: `/map`, TF, LiDAR, AMCL 초기 자세
4. **Nav2 주행**: goal, navigation status, planned path, `/cmd_vel_nav`

처음 단계에서는 4번의 주행 경향을 `robot_1` 한 대로 검증한다. 여러 차량의
공용 저장 지도, 차량별 AMCL/Nav2 namespace, 카메라 융합, costmap 정책, 충돌
회피 및 교통 제어는 별도 단계다.
