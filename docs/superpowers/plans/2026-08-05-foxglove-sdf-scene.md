# Foxglove SDF Warehouse Scene 구현 계획

> **실행 방식:** 현재 워크트리에서 순차 구현한다. 각 단계는 먼저 실패하는 테스트를 추가하고, 최소 구현 후 관련 테스트를 실행한다.

**목표:** Gazebo warehouse SDF의 정적 구조물과 Gazebo pose 스트림의 로봇·팔레트 상태를 Foxglove `SceneUpdate`로 발행한다. Foxglove는 개발 PC에서 실행하며, 이 패키지는 서버 ROS 2 환경에서만 동작한다.

**아키텍처:** 새 `mentorpi_foxglove_scene` Python ament 패키지가 SDF 정적 기하를 `/warehouse_scene/static`에 durable QoS로 한 번 발행한다. `/warehouse/entity_poses`의 `TFMessage`를 받아 10 Hz 완전 스냅샷 형태의 `/warehouse_scene/dynamic`을 발행한다. 좌표계는 `robot_1/odom`이다.

## 1. 순수 SDF 파서와 정적 장면 변환

**파일**

- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/mentorpi_foxglove_scene/sdf_scene.py`
- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/test/test_sdf_scene.py`

**테스트 우선**

1. inline `box`, `cylinder`, `sphere`, `plane`의 크기·색상·pose를 Scene primitive로 변환한다.
2. `model://warehouse_*` include의 `model.sdf`를 해석하고 부모와 자식 pose를 합성한다.
3. 허용 디렉터리 밖 URI, 누락 모델, 지원하지 않는 기하는 안전하게 제외하거나 명확한 오류를 낸다.

**구현**

- `Pose`, `Color`, `Cube`, `Cylinder`, `Sphere`, `SceneEntity` 자료형과 pose 합성 함수를 만든다.
- world SDF와 설치된 models 디렉터리를 입력으로 받아 warehouse 정적 엔티티 목록을 반환한다.
- mesh 등 지원 범위 밖 기하는 경고 후 제외한다.

## 2. 동적 로봇·팔레트 장면 생성

**파일**

- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/mentorpi_foxglove_scene/dynamic_scene.py`
- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/test/test_dynamic_scene.py`

**테스트 우선**

1. `robot_1`, `robot_2` pose가 chassis·mast·fork primitive 조립체를 만든다.
2. `pallet_*` pose가 pallet 조립체를 만들고, `pallet_*_payload`가 있으면 중립 보라색 payload를 추가한다.
3. 이전 스냅샷에 있던 payload가 사라지면 해당 entity id가 삭제 목록에 포함된다.
4. 알 수 없는 Gazebo entity는 무시한다.

**구현**

- SDF 대신 현재 운영 목적에 맞는 단순 primitive 조립체를 생성한다.
- 모든 동적 객체는 입력 pose를 기준으로 `robot_1/odom`에 배치한다.

## 3. ROS 2 SceneUpdate 발행 노드

**파일**

- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/mentorpi_foxglove_scene/sdf_scene_publisher.py`
- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/test/test_scene_update_conversion.py`
- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/setup.py`
- 생성: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_foxglove_scene/package.xml`

**테스트 우선**

1. 내부 정적/동적 엔티티가 `foxglove_msgs/msg/SceneUpdate`와 `SceneEntity`로 정확히 직렬화된다.
2. 정적 토픽이 transient-local/reliable QoS를 사용한다.
3. 입력 TF에서 Gazebo model 이름과 pose를 추출하고 동적 갱신이 삭제를 포함한다.

**구현**

- `/warehouse/entity_poses`를 구독하고 `/warehouse_scene/static`, `/warehouse_scene/dynamic`을 발행한다.
- static은 시작 시 한 번, dynamic은 10 Hz 완전 스냅샷과 삭제 목록으로 발행한다.
- launch file에서 world/model 경로와 frame을 parameter로 제공한다.

## 4. Gazebo pose 스트림과 런치 통합

**파일**

- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/config/warehouse_scene_bridge.yaml` (신규)
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/sim_adapter.launch.py`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/CMakeLists.txt`
- 수정: `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/package.xml`

**테스트 우선**

1. world에 model pose를 발행하는 Gazebo PosePublisher 시스템이 선언된다.
2. bridge가 Gazebo `Pose_V`를 ROS `TFMessage`의 `/warehouse/entity_poses`로 연결한다.
3. adapter launch가 bridge 및 scene publisher를 기동한다.

**구현**

- PosePublisher의 실제 Gazebo topic은 실행 환경에서 확인한 뒤 bridge YAML에 고정한다.
- 기존 `/tf`와 분리하여 장면용 pose topic만 연결한다.

## 5. 컨테이너·문서·통합 검증

**파일**

- 수정: `vehicle_simulator_model/ubuntu/Dockerfile`
- 수정: `vehicle_simulator_model/ubuntu/README.md`
- 수정: `vehicle_simulator_model/ubuntu/run.sh`
- 수정: `vehicle_simulator_model/ubuntu/test/test_navigation_artifacts.py` 또는 신규 통합 테스트

**검증**

1. `ros-humble-foxglove-msgs` 의존성을 이미지에 추가한다.
2. README에 Foxglove 3D 패널의 두 scene topic 사용법과 기대 결과를 기록한다.
3. `./run.sh --env dev test` 전체를 통과시킨다.
4. `git diff --check` 및 관련 패키지 테스트를 실행한다.

## 커밋 계획

1. `docs: add Foxglove SDF scene implementation plan`
2. `feat: publish warehouse SDF scene for Foxglove`
