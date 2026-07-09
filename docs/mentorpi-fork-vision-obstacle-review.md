# MentorPi M1 포크 구조물 부착 및 정밀 적재 기능 검토

작성일: 2026-07-09

## 목적

MentorPi M1 차량 전면에 물류 창고의 지게차 또는 핸드 쟈키와 유사한 포크 구조물을 부착하고, 포크를 이용해 물건을 지정 위치에 정확히 적재하는 기능을 구현하기 위한 검토 문서다.

이 문서는 구현 전에 확인해야 할 차량 제원, 센서 구성, vision 기반 정렬 방식, 포크 부착에 따른 장애물 회피 거동, 제어 구조, 구현 우선순위를 정리한다.

## 근거 자료

이 문서는 최신 llm-wiki-core context bundle에서 선택된 다음 자료를 기준으로 작성했다.

- `artifacts/vehicle/sources/hiwonder-mentorpi-m1.md`
- `artifacts/vehicle/sources/hiwonder-mentorpi-getting-ready-implementation-guide.md`
- `artifacts/vehicle/sources/hiwonder-mentorpi-group-control-code-map.md`
- `llm-wiki/concepts/mentorpi-nav2-slam-implementation-notes.md`
- `llm-wiki/concepts/mentorpi-vehicle-error-control-and-steering-signs.md`
- `llm-wiki/concepts/mentorpi-multi-vehicle-autonomous-control.md`

주의: 이 문서는 검토용 작업 산출물이며 Project/Team wiki canonical truth가 아니다. Project/Team 지식으로 승격하려면 별도 promotion review가 필요하다.

## 현재 차량 기준 제원

현재 기준 차량은 Hiwonder MentorPi M1이다.

| 항목 | 내용 |
|---|---|
| 차량 모델 | Hiwonder MentorPi M1 |
| 구동 방식 | Mecanum wheel chassis |
| 크기 | 212 x 171 x 147 mm |
| 중량 | 1.2 kg |
| 휠 | 지름 65 mm, 폭 30 mm |
| 모터 | 310 metal gear geared motor |
| 엔코더 | AB-phase quadrature encoder |
| 제어기 | Raspberry Pi 5 + RRC Lite controller |
| 배터리 | 7.4 V 2200 mAh 10C LiPo |
| LiDAR | Oradar MS200 |
| Depth camera | Nuwa-HP60C, depth kit 기준 |
| ROS 환경 | Raspberry Pi OS / Ubuntu 22.04 / ROS2 Humble in Docker |

RRC Lite controller는 4채널 encoder motor port, IMU, PWM servo port, serial servo port, 5 V 5 A external power supply port를 제공한다. 다만 기본 서보인 LFD-01은 약 1.4 kg.cm급이므로 실제 지게차처럼 하중을 들어 올리는 리프트 액추에이터로 쓰기에는 부족할 가능성이 높다.

## 차량 플랫폼 해석

현재 차량 타입은 `MentorPi_Mecanum`이다. 따라서 Ackermann 조향 서보가 있는 차량이 아니라, `Twist` 명령의 `linear.x`, `linear.y`, `angular.z`가 Mecanum 구동으로 직접 반영된다.

제어 부호 기준은 다음과 같이 잡는다.

| 명령 | 의미 |
|---|---|
| `linear.x > 0` | 전진 |
| `linear.x < 0` | 후진 |
| `linear.y > 0` / `< 0` | 좌우 평행 이동, 실제 방향은 하드웨어 테스트로 확정 필요 |
| `angular.z > 0` | 좌측 yaw / 좌회전 / 좌측 보정 |
| `angular.z < 0` | 우측 yaw / 우회전 / 우측 보정 |

MVP에서는 일반 주행 단계에서 `linear.y=0`으로 제한하고, 포크 정밀 정렬 단계에서만 Mecanum의 횡이동을 사용하는 편이 안전하다. 기존 Nav2 설정도 `max_vel_y=0.0`으로 differential-like 운용에 가깝다.

## 포크 구조물 부착 시 영향

포크를 전면에 부착하면 로봇의 물리적 외곽, 센서 사각지대, 회전 반경, 전방 충돌 위험이 모두 달라진다.

### 주요 변화

- 기존 차체 길이 212 mm보다 전방 길이가 늘어난다.
- 기존 `robot_radius=0.05` 수준의 Nav2 costmap 설정은 사용할 수 없다.
- 포크 끝단이 장애물과 먼저 접촉하므로 collision 기준점이 차체 중심이 아니라 포크 끝으로 이동한다.
- 제자리 회전 시 포크 끝단이 큰 원호를 그리므로 선반 근처 회전이 위험해진다.
- 물건을 적재한 상태에서는 footprint가 포크만 있을 때보다 더 커진다.
- 포크가 LiDAR 또는 depth camera의 시야를 가릴 수 있다.
- 전면 하중이 증가하면 Mecanum wheel의 접지, odometry, 제동 거리, 전복 안정성이 악화될 수 있다.

### 필수 모델 변경

포크 부착 후에는 원형 `robot_radius` 대신 polygon footprint를 사용해야 한다.

예상 footprint는 다음 3개 상태로 나누는 것이 좋다.

| 상태 | 설명 |
|---|---|
| `base_footprint_empty` | 차체만 있는 상태 |
| `base_footprint_fork` | 포크가 부착된 빈 상태 |
| `base_footprint_loaded` | 포크 위 또는 포크 사이에 물건이 있는 상태 |

TF도 다음 기준 프레임을 추가하는 편이 좋다.

| 프레임 | 용도 |
|---|---|
| `base_footprint` | 기존 차량 기준 |
| `fork_root` | 포크 장착 기준점 |
| `fork_tip` | 포크 전방 끝단 |
| `fork_left_tip` | 좌측 포크 끝 |
| `fork_right_tip` | 우측 포크 끝 |
| `load_center` | 적재물 중심, 물체 감지 후 추정 |

## Vision 및 센서 구성

정확 적재는 Nav2만으로 해결하기 어렵다. Nav2는 지도 기반 이동과 큰 장애물 회피에는 적합하지만, 포크와 팔레트 슬롯 또는 선반 기준점을 몇 cm 단위로 맞추는 데에는 별도 vision/docking controller가 필요하다.

### 센서 역할 분리

| 센서 | 주 역할 | 한계 |
|---|---|---|
| Oradar MS200 LiDAR | SLAM, 전역/지역 costmap, 큰 장애물 회피 | 포크 끝/팔레트 슬롯 정밀 정렬에는 부족 |
| Depth camera | 선반, 팔레트, 포크 전방 공간의 3D 인식 | 조명, 반사, 근거리 사각지대 영향 |
| RGB camera | marker, QR, 색상, 객체 인식 | 거리 추정은 단독으로 불안정 |
| AprilTag / ArUco marker | docking pose 기준점 제공 | 환경에 marker 부착 필요 |
| 근거리 ToF / limit switch | 포크 삽입 깊이, 접촉, 적재물 유무 확인 | 별도 하드웨어 추가 필요 |

### 추천 vision MVP

가장 안정적인 MVP는 선반 또는 팔레트 위치에 AprilTag 또는 ArUco marker를 부착하는 방식이다.

1. RGB 또는 depth camera에서 marker를 검출한다.
2. `camera_frame -> marker_frame` pose를 구한다.
3. marker 기준으로 `shelf_frame` 또는 `pallet_frame`을 정의한다.
4. 그 앞에 `pre_dock_pose`, `align_pose`, `insert_pose`, `place_pose`를 만든다.
5. Nav2는 `pre_dock_pose`까지 이동한다.
6. 이후에는 visual servo controller가 저속으로 포크를 정렬한다.

이 방식은 물체 자체를 YOLO로 바로 인식하는 것보다 재현성이 높다. YOLO 또는 segmentation은 marker 없는 환경으로 확장할 때 적용하는 것이 좋다.

## 기능 모드 분리

기능은 하나의 주행 알고리즘으로 묶지 말고 상태기계로 분리해야 한다.

### 1. Navigate mode

Nav2가 지도 기준으로 선반 근처의 `pre_dock_pose`까지 이동한다.

- 입력: map, AMCL pose, LiDAR costmap
- 출력: Nav2 `cmd_vel`
- 정책: 일반 주행 중 `linear.y=0` 유지
- 성공 조건: 선반 또는 팔레트 기준점이 camera FOV에 들어오는 위치 도달

### 2. Detect mode

전방 camera에서 marker, 팔레트 슬롯, 선반 기준선을 찾는다.

- 입력: RGB image, depth image, camera info
- 출력: `target_frame` pose
- 실패 처리: target 미검출 시 정지, 소각도 scan, 후퇴 후 재접근

### 3. Align mode

포크와 목표 위치의 전후, 좌우, yaw 오차를 저속으로 줄인다.

권장 제어량:

```text
depth_error = target_depth - observed_depth
lateral_error = target_x - observed_x
yaw_error = target_yaw - observed_yaw

cmd.linear.x = clamp(kx * depth_error, -0.05, 0.05)
cmd.linear.y = clamp(ky * lateral_error, -0.04, 0.04)
cmd.angular.z = clamp(kw * yaw_error, -0.20, 0.20)
```

이 단계에서는 Nav2 local planner보다 별도 visual servo controller가 적합하다.

### 4. Insert / Place mode

포크를 목표 슬롯 또는 적재 위치로 천천히 삽입한다.

- 전진 속도는 매우 낮게 제한한다.
- 포크 끝단 또는 전방 depth ROI에 충돌 위험이 있으면 즉시 정지한다.
- 포크 높이 제어가 있다면 lift/lower 명령을 별도 actuator controller로 분리한다.
- 적재물 유무는 depth ROI, limit switch, load switch 중 하나로 확인한다.

### 5. Retreat mode

적재 또는 하역 후 후진하여 선반에서 이탈한다.

- 후진 중 회전 부호는 전진과 다르게 체감될 수 있으므로 별도 테스트 케이스가 필요하다.
- 선반 근처에서는 제자리 회전보다 직선 후퇴를 우선한다.
- 충분히 이탈한 뒤 Nav2 command ownership으로 복귀한다.

## 장애물 회피 거동

포크 장착 후 회피 정책은 단순한 차체 회피가 아니라 포크 끝단과 적재물까지 포함한 회피가 되어야 한다.

### Nav2 costmap 변경

현재 문서화된 Nav2 costmap 설정은 `scan_raw` LaserScan 중심이며 `robot_radius=0.05`, `inflation_radius=0.05`로 작게 잡혀 있다. MentorPi M1 실제 폭 171 mm와 포크 전방 돌출을 고려하면 이 값은 사용할 수 없다.

필수 변경:

- `robot_radius` 대신 polygon `footprint` 사용
- local/global costmap 모두 포크 포함 footprint 적용
- `inflation_radius`를 실제 localization 오차와 포크 여유폭을 반영해 증가
- 포크 장착 상태와 적재 상태에서 footprint를 다르게 적용할지 검토
- RViz에서 map, TF, LaserScan, local/global costmap, footprint 정렬 확인

### 회피 정책

| 상황 | 권장 동작 |
|---|---|
| 일반 통로 주행 | Nav2 local costmap 기반 회피 |
| 전방 포크 영역 장애물 | 즉시 정지 후 후퇴 또는 재계획 |
| 선반 근처 장애물 | lateral 회피보다 정지/후퇴 우선 |
| 포크 삽입 중 접촉 위험 | 삽입 중단, 후퇴, 재정렬 |
| 사람 또는 동적 장애물 | safety stop이 Nav2보다 우선 |
| 좁은 통로 제자리 회전 | 제한 또는 금지 |

Nav2 recovery behavior 중 `spin`은 포크가 달린 상태에서 위험할 수 있다. 선반 근처 또는 좁은 통로에서는 spin recovery를 끄거나, 회전 가능 공간을 확인한 경우에만 허용해야 한다.

## Command ownership

MentorPi 프로젝트에는 joystick, app behavior, autonomous app, Nav2 등 여러 command source가 존재할 수 있다. 포크 기능을 추가하면 safety stop과 docking controller도 command source가 된다.

따라서 `controller/cmd_vel`에 여러 publisher가 동시에 쓰는 구조는 피해야 한다.

권장 우선순위:

```text
manual emergency stop
  > safety stop
  > fork/docking controller
  > fleet coordinator / Nav2
  > app behavior
  > joystick normal control
```

운영 규칙:

- 하나의 arbiter만 최종 `controller/cmd_vel`에 publish한다.
- Nav2 출력은 `cmd_vel_nav`처럼 중간 topic으로 받고 arbiter를 통과시킨다.
- docking controller는 Align/Insert/Retreat mode에서만 ownership을 가진다.
- emergency stop과 safety stop은 모든 모드보다 우선한다.

## 구현 우선순위

### Phase 0. 기구 검토

- 포크 길이, 폭, 간격, 높이, 장착 위치 확정
- 예상 적재물 크기와 중량 확정
- 무게중심 이동과 전복 위험 확인
- 포크가 LiDAR/camera 시야를 가리는지 확인
- 기본 서보/모터로 가능한 하중인지 확인

### Phase 1. 기존 주행 안정화

- `scan_raw`, `/odom`, TF, AMCL, Nav2 목표 주행 검증
- `base_footprint`, `base_laser`, camera frame 정렬 확인
- 기존 `robot_radius=0.05` 설정을 실차 기준 footprint로 조정
- `linear.y=0` 일반 주행 유지

### Phase 2. 포크 모델 반영

- URDF 또는 static TF에 포크 프레임 추가
- local/global costmap footprint를 포크 포함 polygon으로 변경
- RViz에서 footprint와 센서 위치 검증
- 포크 장착 후 odometry drift와 회전 반경 변화 확인

### Phase 3. Vision docking MVP

- 선반 또는 팔레트에 AprilTag/ArUco marker 부착
- camera pose 기반 `target_frame` 계산
- `pre_dock_pose`, `align_pose`, `insert_pose` 정의
- Align mode visual servo controller 구현
- 저속 제한과 timeout, target lost 처리 추가

### Phase 4. Safety 및 삽입 검증

- 포크 끝단 근거리 센서 또는 depth ROI 기반 충돌 감지
- `safety stop` command source 추가
- Insert / Place / Retreat mode 상태기계 구현
- 실제 적재물 유무 확인 로직 추가

### Phase 5. 확장

- marker 없는 객체 인식 또는 YOLO/segmentation 적용
- loaded footprint 동적 전환
- 다중 로봇이면 fleet coordinator가 `cmd_vel`이 아니라 Nav2 action goal을 보내도록 구성
- 선반/통로 graph 기반 traffic reservation 추가

## 주요 리스크

| 리스크 | 영향 | 대응 |
|---|---|---|
| 포크 하중 과다 | 모터 과전류, 전복, 제동 실패 | 가벼운 mock payload로 시작 |
| footprint 미반영 | 포크 끝단 충돌 | polygon footprint 필수 |
| 센서 사각지대 | 삽입 중 충돌 | depth ROI 또는 근거리 센서 추가 |
| command source 충돌 | 갑작스러운 주행, 불안정 제어 | command arbiter 도입 |
| marker 미검출 | docking 실패 | 탐색/후퇴/재접근 상태 추가 |
| 후진 부호 혼동 | 선반 근처 충돌 | 후진 전용 테스트 케이스 작성 |
| Mecanum 횡이동 과신 | localization 오차 증가 | 일반 주행은 `linear.y=0`, 정렬 단계만 사용 |

## 검토 체크리스트

- [ ] 실제 MentorPi kit가 depth camera 버전인지 확인
- [ ] 포크 CAD 또는 치수 도면 확보
- [ ] 적재 대상 물체의 최대 크기와 중량 확정
- [ ] 포크 부착 후 LiDAR/camera 시야 가림 여부 확인
- [ ] `base_footprint -> fork_tip` 실제 치수 측정
- [ ] Nav2 footprint를 포크 포함 polygon으로 수정
- [ ] marker 기반 target pose 검출 실험
- [ ] Align mode에서 `linear.x`, `linear.y`, `angular.z` 부호 실차 검증
- [ ] Insert mode 전방 충돌 감지 구현
- [ ] command arbiter 설계 및 emergency stop 우선순위 확정

## 결론

이 기능은 Nav2 하나로 구현하는 것이 아니라 다음 3계층으로 나누는 것이 안전하다.

1. Nav2: 선반 근처까지 이동하고 큰 장애물을 회피한다.
2. Vision docking: 선반/팔레트 기준점을 인식하고 포크를 정밀 정렬한다.
3. Fork safety controller: 포크 삽입, 접촉 감지, 적재물 확인, 후퇴를 저속으로 관리한다.

가장 먼저 해야 할 일은 vision 모델 선택이 아니라 포크를 포함한 footprint, TF, command ownership, emergency stop을 정리하는 것이다. 이 기반이 없으면 포크 끝단 충돌을 Nav2가 인지하지 못하고, 정밀 적재 단계에서 여러 command source가 충돌할 수 있다.
