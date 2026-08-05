# Foxglove SDF Scene Design

## Goal

원격 서버에서 실행되는 Gazebo simulation의 warehouse SDF를 Foxglove Studio에서
시각화한다. warehouse는 정적 배경으로, robot_1·robot_2 및 Gazebo pallet model은
동적 객체로 표시한다. Foxglove는 개발 PC에서 SSH tunnel을 통해 접속하며 Gazebo GUI나
X11 전달은 사용하지 않는다.

## Scope

- `warehouse.sdf`와 `models/warehouse_*/model.sdf`의 `<visual>` geometry를 해석한다.
- 지원 geometry는 `box`, `cylinder`, `sphere`, `plane`이다. plane은 얇은 cube로 바꾼다.
- `model://warehouse_*` include를 재귀적으로 해석하고 pose를 합성한다.
- Gazebo world PosePublisher의 `gz.msgs.Pose_V`를 ROS 2
  `tf2_msgs/msg/TFMessage`로 bridge한다.
- pose stream에서 `robot_1`, `robot_2`, `pallet_*`, `pallet_*_payload`를 골라
  Foxglove dynamic scene으로 갱신한다.
- robot은 포크를 포함한 단순 cube assembly로 표시한다. pallet deck과 payload도 cube
  assembly로 표시한다.

## Out of Scope

- Gazebo physics, sensor, plugin 또는 material shader를 Foxglove에서 실행하지 않는다.
- robot STL mesh의 GLB 변환은 포함하지 않는다.
- real robot의 scene source는 포함하지 않는다.
- 팔레트 상태를 별도 custom ROS message로 publish하지 않는다. payload model의 존재로
  loaded/empty를 표현하며, fresh/normal kind는 공통 payload 색상으로 표시한다.

## Architecture

새 ROS 2 Python package `mentorpi_foxglove_scene`를 `ros2_ws/src`에 둔다.

```text
warehouse.sdf + models/warehouse_*/model.sdf
                 │
                 ▼
          sdf_scene_publisher
                 │ static SceneUpdate (transient local)
                 ▼
       /warehouse_scene/static

Gazebo PosePublisher ── gz.msgs.Pose_V ── ros_gz_bridge ── TFMessage
                                                              │
                                                              ▼
                                                    sdf_scene_publisher
                                                              │ dynamic SceneUpdate
                                                              ▼
                                             /warehouse_scene/dynamic
```

`warehouse.sdf`의 world-level PosePublisher는 `publish_model_pose=true`,
`publish_nested_model_pose=true`, `use_pose_vector_msg=true`로 10 Hz에 model pose를
발행한다. `sim_adapter`의 `ros_gz_bridge`는 이 Gazebo topic을
`/warehouse/entity_poses` (`tf2_msgs/msg/TFMessage`)로 단방향 bridge한다. 이 topic은
`/tf`에 remap하지 않는다.

static 및 dynamic scene entity의 frame은 `robot_1/odom`이다. simulator의
`gz_pose_to_odom.py`가 Gazebo world pose를 같은 좌표로 robot_1 odom에 publish하고,
SLAM은 `map -> robot_1/odom` transform을 제공한다. Foxglove의 fixed frame을 `map`으로
선택하면 map·scene·robot pose가 같은 transform tree 안에서 렌더링된다.

## Static SDF Conversion

`sdf_scene.py`는 XML parser와 순수 transform helper를 제공한다.

1. world의 inline static model과 include를 읽는다.
2. `model://name`은 package의 `models/name/model.sdf`로 해석한다. package root 밖의
   URI와 존재하지 않는 model은 `ValueError`로 거부한다.
3. model, link, visual pose를 부모 pose와 합성한다. SDF pose 값은 `x y z roll pitch yaw`이며
   Foxglove pose에는 quaternion을 저장한다.
4. visual 하나를 ID가 안정적인 scene primitive로 변환한다. ID는
   `<world-model>/<link>/<visual>` 또는 `<include-name>/<link>/<visual>` 형식이다.
5. material diffuse RGBA가 없으면 불투명 회색 `(0.7, 0.7, 0.7, 1.0)`을 사용한다.

`SceneUpdate`는 `/warehouse_scene/static`에 transient-local reliable QoS로 발행한다.
정적 entity는 lifetime 0이며, SDF를 다시 읽은 경우 기존 ID를 replacement한다.

## Dynamic Entities

`sdf_scene_publisher`는 `/warehouse/entity_poses`를 구독하고 최신 entity pose snapshot을
10 Hz로 `/warehouse_scene/dynamic`에 발행한다. 각 snapshot은 현재 보이는 dynamic entity
전체를 포함하고, 직전 snapshot에 있었으나 사라진 ID는 `deletions`에 넣는다.

| Gazebo entity name | Foxglove ID | Representation |
| --- | --- | --- |
| `robot_1` | `robot_1` | chassis cube, mast cube, fork cubes, `robot_1` pose |
| `robot_2` | `robot_2` | chassis cube, mast cube, fork cubes, `robot_2` pose |
| `pallet_<id>` | `pallet_<id>` | brown deck and supports |
| `pallet_<id>_payload` | `pallet_<id>_payload` | 공통 payload cube assembly |

로봇의 chassis는 `0.30 × 0.20 × 0.12 m`, mast는 `0.05 × 0.16 × 0.28 m`, fork는
각각 `0.16 × 0.025 × 0.02 m`로 표시한다. pallet deck·support·payload는 현재
`models/pallet/*.sdf.in`의 box size와 pose를 그대로 사용하고, payload는 pose stream만으로
kind를 구별할 수 없으므로 중립적인 보라색으로 표시한다.

알 수 없는 entity name은 무시한다. robot pose가 없을 때는 해당 robot entity를 발행하지
않고, pallet payload가 없어지면 이전 payload ID를 deletion으로 발행한다.

## Runtime Integration

- `mentorpi_foxglove_scene`는 `sim-adapter` launch에 node로 추가한다.
- `mentorpi_gz_sim`의 world와 bridge config에 PosePublisher 및 entity pose bridge를 추가한다.
- Docker image에는 `foxglove_msgs` ROS package가 포함되어야 한다.
- Foxglove bridge가 running인 경우 local Studio는 `/warehouse_scene/static`,
  `/warehouse_scene/dynamic`, `/map`, `/tf`, `/robot_1/scan_raw`를 3D panel에 추가한다.

## Failure Handling

- world path, model URI, geometry size, pose 숫자 형식이 올바르지 않으면 node는 명확한
  error를 내고 시작하지 않는다.
- 지원하지 않는 geometry는 warning을 내고 그 visual만 건너뛴다.
- entity pose stream이 아직 없으면 static scene만 발행하며 node는 계속 실행한다.
- entity pose의 parent frame이 Gazebo world가 아닌 경우에는 해당 transform을 무시하고
  warning을 rate-limit한다.

## Tests and Acceptance Criteria

1. unit test는 inline box와 include box의 pose 합성·색상·entity ID를 검증한다.
2. unit test는 model root 밖 URI와 누락 model을 거부하는지 검증한다.
3. unit test는 robot, fresh pallet, normal pallet snapshot과 deletion을 검증한다.
4. launch/config contract test는 PosePublisher와 `/warehouse/entity_poses` bridge,
   scene publisher node가 함께 등록됐는지 검증한다.
5. package test와 `./run.sh --env dev test`가 통과한다.
6. 실행 중 Foxglove 3D panel에서 static warehouse와 robot_1·robot_2·pallet scene topics가
   나타나며, `/map` fixed frame에서 TF와 함께 위치가 변한다.
