# Task 5 완료 보고서: 참고 이미지 기반 정적 창고 월드와 기본 배치

## 상태

- 상태: DONE
- 기준 브랜치/시작 커밋: `gazebo` / `a57cb7f`
- 구현 커밋: 이 보고서와 함께
  `feat: 참고 이미지 기반 물류 창고 월드 구성`으로 생성한다.
- 기존에 stage되어 있던
  `llm-wiki-core/llm_wiki_core/__pycache__/__init__.cpython-314.pyc`,
  `llm-wiki-core/llm_wiki_core/__pycache__/core.cpython-314.pyc`는
  수정하거나 Task 5 커밋에 포함하지 않았다.

## 구현

- `warehouse_conveyor`를 2.2 × 0.45 × 0.45 m 규격의 프레임,
  벨트, 양쪽 가드와 단순화된 collision으로 구성했다.
- `warehouse_robot_arm`을 원통 베이스와 3개 링크로 구성하고 모든 joint를
  `fixed`로 제한했다. 라이다 경계 인식을 위해 collision은 4개만 사용했다.
- `warehouse_charger`를 0.55 × 0.22 × 0.65 m 본체와 양쪽 보호 볼라드로
  구성했다.
- `warehouse_rack`을 2.0 × 0.75 × 1.2 m 프레임, 선반, 배경 상자로
  구성하고 전체 설비 경계를 하나의 box collision으로 단순화했다.
- 모든 정적 설비 모델은 `<static>true</static>`를 사용한다.
- 결정적 생성기 `generate_floor_markings.py`에 브리프의 5×7 비트맵
  `GLYPHS`를 그대로 선언했다. `FRESH`, `NORMAL`, `PICO`, `ROAD 2`,
  작업장 번호 `1`~`4`, 충전 구역과 번개, 빨간색 파렛트 적재 존 3개를
  visual-only SDF로 생성한다.
- 생성된 `warehouse_markings/model.sdf`는 414개의 얇은 box visual을
  포함하며 collision은 없다. 테스트가 임시 경로에서 다시 생성한 결과와
  커밋 결과의 byte-for-byte 동일성을 검증한다.
- `warehouse.sdf`에는 이름이 지정된 8개 include
  (`left_conveyor`, `pico_conveyor`, `robot_arm_upper`,
  `robot_arm_lower`, `charging_station`, `fresh_rack`, `normal_rack`,
  `floor_markings`)를 표준 SDF `<include><name>…</name>` 형식으로 배치했다.
- `mentorpi_gz_sim::WarehousePalletManager` 플러그인에 브리프의 service,
  template, spawn bounds, 기본 pose, 정지 임계값과
  fresh 3개 + normal 3개의 loaded 기본 파렛트를 정확히 설정했다.
- `_robot_nodes`가 yaw를 받도록 변경하고
  `robot_1=(1.8,-2.8,0.05,1.5708)`,
  `robot_2=(3.2,-2.8,0.05,1.5708)`에 생성되도록 했다.
- 설치 이미지 smoke test에서 environment hook이 빌드 시점의
  `COLCON_CURRENT_PREFIX`보다 남아 있던 `AMENT_CURRENT_PREFIX`를 우선해
  Gazebo 경로를 `/opt/ros/humble`로 잘못 설정하는 기존 문제를 발견했다.
  올바른 colcon install prefix를 우선하도록 hook을 수정하고 실제 shell
  실행 계약 테스트를 추가했다.

## RED 증거

1. 정적 월드/설비 테스트를 먼저 확장하고 구현 전 실행했다.

   ```bash
   python3 vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_warehouse_assets.py -v
   ```

   결과: 신규 모델, 표식, include, 플러그인 설정이 없어
   7 tests 중 2 failures + 6 errors로 예상대로 실패했다.

2. launch 계약을 yaw와 설계 pose까지 확장하고 구현 전 실행했다.

   ```bash
   python3 vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py -v
   ```

   결과: `_robot_nodes`에 yaw 인자가 없어 6 tests 중 1 error로
   예상대로 실패했다.

3. 첫 월드 구현은 `<include name="…">` 속성을 사용해 SDF parser 계약
   테스트가 실패했다. 표준 `<name>` 자식 요소로 수정해 해결했다.

4. Docker 설치 환경의 hook을 실제로 실행하는 테스트를 추가했을 때
   기대한 `/opt/mentorpi_ws/install/mentorpi_gz_sim/...` 대신
   `/opt/ros/humble/...`가 출력되어 예상대로 실패했다.

## GREEN 및 회귀 증거

1. 최종 정적 월드/설비 테스트:

   ```bash
   python3 vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_warehouse_assets.py -v
   ```

   결과: 8 tests, 0 failures.

2. 최종 launch 및 environment hook 계약 테스트:

   ```bash
   python3 vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py -v
   ```

   결과: 6 tests, 0 failures.

3. 신규 모델 SDF 검증:

   ```bash
   for model in warehouse_conveyor warehouse_robot_arm warehouse_charger \
     warehouse_rack warehouse_markings; do
     gz sdf -k ".../models/${model}/model.sdf"
   done
   ```

   결과: 5개 모두 `Valid.`.

4. 전체 차량/SLAM 번들 회귀:

   ```bash
   ./vehicle_simulator_model/ubuntu/run.sh test
   ```

   결과: host static 28, description 8, launch 6 및 runtime package
   CTest가 모두 통과했다. 최종 합계는
   59 tests, 0 errors, 0 failures, 환경 조건부 1 skipped다.

5. environment hook 수정 후 최종 이미지 재빌드:

   ```bash
   ./vehicle_simulator_model/ubuntu/run.sh build
   ```

   결과: 45.1초에 `mentorpi-sim:harmonic` 이미지와 manifest 빌드 성공.

6. 최종 설치 환경과 production 월드 smoke:

   ```text
   GZ_SIM_RESOURCE_PATH=/opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim/models
   GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/mentorpi_ws/install/mentorpi_gz_sim/lib
   ```

   설치된 `warehouse.sdf`를 headless로 실행한 뒤 list service가 다음을
   반환했고 process는 exit 0이었다.

   ```text
   ok|list|pallet_01:fresh:loaded,pallet_02:fresh:loaded,pallet_03:fresh:loaded,pallet_04:normal:loaded,pallet_05:normal:loaded,pallet_06:normal:loaded
   ```

7. 정적 diff 검사:

   ```bash
   git diff --check
   ```

   결과: exit 0, 출력 없음.

## macOS GUI 확인

- 로컬 Gazebo 8.14.0에서 server와 GUI를 별도 process로 실행했다.
- Orca computer-use로 실제 GUI를 확인했으며 벽과 바닥, 왼쪽 conveyor,
  robot arm 2개, 작업장 1~4, FRESH, PICO와 빨간 적재 존, NORMAL과 rack,
  charger와 번개, ROAD 2가 모두 식별됐다.
- Entity Tree에서 8개 include가 모두 로드된 것을 확인했다.
- macOS에는 Linux용
  `mentorpi_warehouse_pallet_manager` shared library가 없으므로 plugin
  load 오류가 발생했지만 정적 월드 렌더링에는 영향이 없었다. 실제
  플러그인과 기본 파렛트 6개는 위의 Linux Docker production smoke로
  확인했다.

## 변경 파일

- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_conveyor/model.config`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_conveyor/model.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_robot_arm/model.config`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_robot_arm/model.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_charger/model.config`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_charger/model.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_rack/model.config`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_rack/model.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_markings/model.config`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/models/warehouse_markings/model.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/tools/generate_floor_markings.py`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_warehouse_assets.py`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/test/test_harmonic_launch_contract.py`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/worlds/warehouse.sdf`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/launch/sim_adapter.launch.py`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_gz_sim/env-hooks/mentorpi_gz_sim.sh.in`
- `.superpowers/sdd/2026-07-28-mentorpi-warehouse-pallet/task-5-report.md`

## 자체검토 및 우려

- 바닥 표식은 collision이 없어 차량 물리와 라이다에 불필요한 장애물을
  추가하지 않는다. 설비에는 단순화된 경계 collision만 있다.
- Mac GUI에서는 Linux 전용 plugin 부재로 기본 파렛트를 직접 볼 수 없지만
  Linux production smoke가 6개 loaded 기본 파렛트의 실제 생성을 확인했다.
- Docker Compose가 기존 orphan container
  `ubuntu-gazebo-viewer-1`, `ubuntu-web-gateway-1` 경고를 출력했다.
  Task 5와 무관하며 사용자 환경을 보존하기 위해 삭제하지 않았다.
- 전체 `run.sh test` 이후 environment hook만 추가 수정했다. 그 뒤 해당
  hook의 행동 테스트, 정적 테스트 전체, 최종 이미지 rebuild, 설치 환경
  경로 확인, production world/service smoke를 다시 수행해 변경 범위를
  회귀 검증했다.
