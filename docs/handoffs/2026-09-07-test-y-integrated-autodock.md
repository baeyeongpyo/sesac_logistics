# 1호차 test_y를 auto_dock Y PLACE에 통합

## vehicle=0 시작 거절 수정

실제 launcher는 vehicle=0/domain 215였으나 기존 constructor가 self.vehicle=0을 유지하여 새 vehicle 1 gate에서 거절되는 누락을 수정했다. constructor가 기존 `resolve_vehicle_id`로 DDS domain을 해석하도록 연결했다. 로컬 관련 비실행 검사 38개, 차량 실제 설치본 constructor 회귀 4개 통과. 일반 colcon build 성공, 설치/source SHA256 `31ccc2b1163d5553ba3a43c4ea389a07b9778812bca8434046ba0818e195ad37` 일치, 설치 package symlink 0개. 소스 백업은 `auto_dock_node.py.before-domain-resolution`. 아래 최초 배포 manifest의 node 해시는 이 변경 전 값이다. 노드 재시작/명령 발행은 하지 않았다.

## Control GUI 버튼 연결 보완

사용자의 버튼 동작 확인 중 `publish_arrival_trigger()`가 Y에서 항상 `stage_only=true`와 GUI 거리값을 보내 새 고정 profile에 거절되는 누락을 발견했다. 로컬 및 1호차 `tools/vehicle_camera_teleop_gui.py`의 해당 callback만 수정하여 Y는 insertion 35cm, stage_only false를 보낸다. GUI의 `publish_arrival()`는 false인 stage_only 필드를 생략하므로 기존 기본 전체 실행 계약을 사용한다. DOCK 동작은 유지. GUI/ROS import 없이 AST로 callback만 꺼내 mock 호출하여 Y PLACE/DOCK PICK 입력 검증 및 문법 검사 통과. 차량 백업 `/home/ubuntu/ros2_ws/tools/vehicle_camera_teleop_gui.py.before-full-y-place-20260907T053550`, 수정 파일/백업 symlink 0개. GUI 직접 실행 파일이므로 추가 colcon 빌드는 하지 않았다. 실행 중 GUI와 auto_dock은 사용자 재시작 후 적용되며 에이전트는 실행하지 않았다.

사용자의 “config이고 뭐고 지금 test_y가 주행하는 그대로” 지시에 따라, 기존 Y PLACE 기본 알고리즘을 현재 차량 test_y 실행 흐름으로 대체했다. 이전 사전 검토 문서의 옵션 호환/알고리즘 재설계 계획보다 이번 고정 실행 지시를 우선했다.

## 기준과 동작

- 차량 `/home/ubuntu/ros2_ws/tools/y_place_square_topline_trial.py` SHA256 `7f2d9dc7c3101a398065256328f0bd981f0569bb89ecdad6f448c9e10a178144`가 기준이다. 이 버전은 staging/중간 재검출 실패 시 **10cm 직선 후진 후 재검출**하며, 예전 회전 복구 버전이 아니다.
- `.zshrc`의 test_y 실효값: staging 40cm, insertion 35cm, speed 0.10m/s, angular 0.35rad/s, 좌측 offset 2cm, 시작 검출 1프레임. 차량 설정 JSON SHA256 `42d6479d5400e70c4c79e044f48c8cf0be446578aaca809dabe146e09b8756f2`를 패키지에 복사했다. `/shared` 설정이나 기존 Y 보조 설정으로 이 값을 덮어쓰지 않는다.
- 새 파일 `auto_dock/y_place.py`가 auto_dock 안의 worker를 관리하고, `y_place_sequence.py`가 기존 live 반복문/계산 순서를 유지한다. 검출·계산 helper와 observation callback은 기준본에서 옮겼으며 bare import만 패키지 import로 바꿨다. `loaded_response_planner.py`는 기존 실험이 사용하던 동일 수치 모듈이며 변경하지 않았다.
- 별도 ROS Node/프로세스나 rclpy context를 만들지 않는다. 기존 auto_dock의 이미지/명령/포크 callback과 새 `/imu` 구독으로 worker의 bounded inbox에 전달한다. worker가 기존 spin/drain/sleep 순서로 처리하고 기존 publisher로 명령을 보낸다. 무거운 계산 중에도 메인 노드가 정지를 수신한다.
- 취소와 nonzero/포크 발행은 같은 lock으로 직렬화하며 marker 정지도 유지한다. 취소 후 늦게 반환한 계산은 움직임을 재개하지 못한다. worker 정리 전 새 작업도 거절한다. 예외는 ERROR로 종료하며 기존 Y FSM 재시도로 복귀하지 않는다.
- Y 실행에서는 기존 검출/주행/odom 후진/LiDAR backoff 분기에 진입하지 않는다. 이는 현재 standalone 실행 및 현재 lidar=false 설정을 그대로 사용하라는 요청에 따른 것이다. 비Y 작업의 기존 흐름은 유지한다.

## 토픽과 호환 범위

토픽 이름/타입/QoS는 유지한다: `/nav2/arrival` String JSON, `/auto_dock/stop` Empty, `/auto_dock/status` String JSON, `/auto_dock/drive_ready` Empty, `/controller/cmd_vel` Twist, `/fork/command` 및 `/fork/state` String. Y1~Y4→Y 정규화, NEAREST→NONE 호환도 유지한다. 상태는 ALIGNING→INSERTING→WAIT_DOWN_COMPLETE→REVERSING→READY; 실패/취소는 ERROR다. stage/action 세부 reason은 `test_y_*`로 보고한다.

DOWN_COMPLETE 뒤 UNLOADED를 보고하고 현재 trial의 nominal 시간 기준 35cm 후진을 실행한다. 전체 sequence 및 정지 정리가 끝난 뒤에만 READY/drive_ready를 한 번 보고한다. `actual_position_verified=false`와 nominal distance model을 포함하며 실측 후진 성공으로 보증하지 않는다.

**고정 실행 범위:** arrival에 `stage_only=true` 또는 35cm가 아닌 명시적 `insertion_distance_cm`가 있으면 `test_y_requires_full_35cm_place`로 거절한다. 기존 Y 수동 진입/기본거리/response/pose-source 보조 토픽은 이름을 유지하되 `test_y_profile_is_fixed`로 거절하여 기존 알고리즘에 우회 진입하지 못하게 한다. 1호차 계수의 타 차량 사용을 막기 위해 vehicle 1에서만 시작한다. 일반적인 Y PLACE 도착 명령은 추가 설정 없이 고정 profile을 실행한다.

현재 패키지의 profile은 복사한 기준본이다. 이후 standalone `tools/test_y` 관련 파일이나 외부 config를 바꿔도 통합본이 자동 변경되지는 않는다. 다시 반영할 때 새 기준본 검증·복사·일반 빌드가 필요하다.

## 검증/배포

- 로컬 ROS Jazzy 환경에서 ROS init/Node 생성 없는 패키지 테스트 **365개 통과**. 구 Y 설정/깊이 callback 테스트는 새 고정 profile/깊이 미사용 기대값으로 갱신했다.
- 차량 ROS Humble staging에서 364개 통과 후, 빠진 로컬 config test fixture를 staging에만 추가해 나머지 1개도 통과했다. 새 통합 테스트 23개도 차량 staging에서 별도 통과했다.
- 순수 helper 전체 함수와 이미지/IMU callback의 AST 동일성, 전체 정상 live sequence의 executor/상태 adapter 외 AST 동일성을 확인했다. 취소 뒤 계산 반환, marker 정지, inbox 처리, 공개 상태/READY 한 번, 구 Y 제어 차단을 모의 검증했다. 이는 실차 경로/정지 성능 검증이 아니다.
- 차량 기존 소스 hash를 확인한 뒤 수정/추가 파일만 일반 복사. 소스 및 설치본 전체 백업: `/home/ubuntu/ros2_ws/backups/auto_dock_before_test_y_20260907T053014Z`. 백업이 중복 package로 검색되지 않도록 그 디렉터리에 `COLCON_IGNORE`를 두었다.
- `/home/ubuntu/ros2_ws`에서 **`colcon build --packages-select auto_dock` 성공**. symlink-install은 사용하지 않았다. setup.py의 package_data로 고정 JSON을 설치하며 package.xml에 python3-scipy 의존성을 추가했다.
- 설치된 새 모듈/설정과 배포 소스 hash 일치, `install/auto_dock` 및 백업 symlink **0개** 확인. 상세 파일 hash는 `analysis/test_y_integrated_baseline_20260907/deployment.json`, 기준 원본은 패키지 `test/fixtures/test_y_vehicle`에 있다.
- 빌드 후 차량의 **실제 설치 모듈을 import하여 통합 테스트 23개 통과**. ROS 환경만 불러왔으며 테스트에서 Node 생성/ROS init/모터 명령은 하지 않았다. 로컬과 배포한 29개 파일 hash도 일치했다.
- 노드/launcher/GUI/stopper/주행을 실행하거나 재시작하지 않았다. 실행 중 노드에는 아직 새 코드가 적용되지 않으며, 사용자가 재시작한 후 기존 Y PLACE 도착 명령으로 실행한다. 실제 이동/포크/정지 검증은 사용자 실행이 필요하다.
