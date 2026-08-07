# MentorPi 지도·위치추정·기록 용어

## 목적

이 문서는 MentorPi 물류 차량의 지도 생성, 저장 지도 주행, 주행 데이터 기록에서
사용하는 명칭을 구분한다. 현재 구현된 단일 차량(`robot_1`) 기준과, 향후 장기
적재물 배치 변경에 대응하기 위한 지도 갱신 방향을 함께 기록한다.

## 전체 관계

```text
지도 생성
LiDAR + odom + TF
  -> slam_toolbox
  -> 실시간 /map, map -> robot_1/odom
  -> map.yaml + map.pgm + posegraph + rosbag2 저장

저장 지도 주행
map.yaml + map.pgm
  -> map_server -> /map
LiDAR + odom + /initialpose
  -> AMCL -> map -> robot_1/odom
  -> Nav2

주행 기록 및 향후 지도 갱신
LiDAR + odom + TF 등의 원본 토픽
  -> rosbag2
  -> 격리된 환경의 slam_toolbox 재처리
  -> 후보 지도 검증
  -> 승인된 지도만 map_server 운영 지도에 반영
```

## 용어

| 명칭 | 의미와 역할 | 현재 구성에서의 주의점 |
| --- | --- | --- |
| `slam_toolbox` | LiDAR scan, odometry, TF를 정합하여 미지 환경의 2D 점유 지도를 만드는 SLAM 노드다. 지도 생성 중 실시간 `/map`과 `map -> robot_1/odom` 관계를 제공하고, 지도 및 posegraph를 저장할 수 있다. | 지도 생성 모드에서만 사용한다. 저장 지도 주행의 AMCL과 같은 ROS 환경에서 동시에 실행하지 않는다. |
| `map_server` | 이미 완성·승인된 `map.yaml`과 `map.pgm`을 읽어 고정 `/map`으로 제공하는 노드다. | 지도 생성·갱신 또는 차량 위치추정을 하지 않는다. 저장 지도 주행에서는 AMCL과 함께 사용한다. |
| AMCL | 저장 지도와 현재 LiDAR scan, odom을 비교해 차량의 위치를 확률적으로 추정하는 위치추정 노드다. | `/initialpose`로 시작 위치의 대략적인 위치·방향을 지정한다. 현재 구성에서는 저장 지도 모드에서만 실행한다. |
| `/map` | 벽, 랙, 통로 등 주행 가능한지 여부를 격자로 표현한 ROS `OccupancyGrid` 토픽이다. Nav2의 전역 경로계획 기준이다. | SLAM 모드에서는 실시간으로 바뀌고, `map_server` 모드에서는 저장 지도 내용으로 고정된다. Gazebo 3D scene topic과는 별개다. |
| `map -> robot_1/odom` | 지도 좌표계와 차량의 누적 이동 좌표계 사이를 연결하는 TF 변환이다. | SLAM 모드에서는 `slam_toolbox`, 저장 지도 모드에서는 AMCL이 소유한다. 두 소유자가 동시에 발행하면 안 된다. |
| `/initialpose` | 운영자가 `map` 프레임에서 차량의 대략적인 시작 위치와 방향을 AMCL에 전달하는 입력 토픽이다. | 토픽 자체는 언제든 발행할 수 있지만, 현재 SLAM 모드에는 AMCL이 없으므로 효과가 없다. |
| `map.pgm` | 픽셀 값으로 점유 상태를 나타내는 지도 이미지 파일이다. 일반적으로 검정은 장애물, 흰색은 자유 공간, 회색은 미확인 영역을 표현한다. | 단독으로는 운용하지 않는다. 해상도·원점·임계값이 적힌 `map.yaml`과 한 쌍이다. |
| `map.yaml` | 지도 이미지 파일 경로, 해상도, 지도 원점, 점유·자유 공간 임계값을 담은 지도 메타데이터 파일이다. | `map_server`는 이 파일을 진입점으로 사용해 연결된 PGM 지도를 읽는다. |
| posegraph | SLAM이 저장한 scan 위치 노드와 노드 간 정합·loop closure 관계 데이터다. | 지도 품질 분석·재처리에 유용하지만 `map_server`가 직접 읽는 파일은 아니다. 운영 주행에는 `map.yaml`과 `map.pgm`이 사용된다. |
| rosbag2 | ROS 2 토픽 메시지를 시간 순서대로 기록한 디렉터리 형식의 데이터 묶음이다. `metadata.yaml`과 `.db3` 또는 `.mcap` 저장 파일로 구성된다. | 지도 자체가 아니라, LiDAR·odom·TF처럼 지도를 다시 만들고 장애를 분석할 수 있는 원본 주행 기록이다. |
| manifest | 지도 세션의 ID, 생성 시간, 이미지·world·모델·TF 보정 버전 등 출처 메타데이터다. | 현재 세션 검증은 기본 필드와 checksum을 확인한다. 지도 호환성 정책은 향후 강화 대상이다. |
| checksum | 지도와 세션 산출물이 저장 후 훼손·변조되지 않았는지 확인하는 SHA-256 무결성 정보다. | 파일 무결성만 확인하며, 실제 창고 환경과의 일치 또는 지도 품질을 보증하지는 않는다. |

## 현재 지도 세션 산출물

`mapping-up <session-id>`로 시작한 지도 생성은 안전하게 종료될 때 다음 형태로
저장된다.

```text
/slam-data/<session-id>/
├── map.yaml
├── map.pgm
├── posegraph/mentorpi.posegraph
├── rosbag2/mapping/
├── manifest.json
└── checksums.sha256
```

현재 지도 생성 세션의 rosbag2는 다음 원본 토픽을 기록한다.

```text
/clock
/tf
/tf_static
/robot_1/scan_raw
/robot_1/imu/data_raw
/robot_1/odom
```

## 운영 상태별 역할

| 상태 | 실행 주체 | 목적 | `/initialpose` |
| --- | --- | --- | --- |
| 지도 없음 또는 신규 지도 생성 | `slam_toolbox` + Nav2 | LiDAR 기반 임시 지도 생성 및 탐색 | 현재 구성에서는 사용하지 않음 |
| 승인된 저장 지도 주행 | `map_server` + AMCL + Nav2 | 반복 주행과 위치추정 | AMCL 수렴을 위해 사용 |
| 장기 배치 변경 검토 | 운영 Nav2 + recorder | 원본 관측을 축적해 후보 지도 재생성 근거 확보 | 기존 운영 지도 기준으로 AMCL 사용 |
| 후보 지도 재생성·검증 | 격리된 `slam_toolbox` 환경 | rosbag2 또는 별도 매핑 주행으로 후보 지도 생성 | 운영 차량·운영 TF와 분리 |

## 물류 적재물과 지도 갱신 원칙

- 벽, 기둥, 고정 랙처럼 장기적으로 변하지 않는 구조는 저장 지도에 반영한다.
- 일시적인 팔레트, 박스, 사람, 다른 차량은 저장 지도 대신 costmap의 현재 장애물로
  처리한다.
- 적재물이 장기간 고정 배치로 확정되면 원본 관측을 근거로 새 지도 버전을 만든다.
- 새 지도는 기존 지도를 덮어쓰지 않고 새 세션 ID로 저장·검증한다.
- AMCL 초기화와 대표 경로 주행 검증을 통과한 후보만 운영 `map_server` 지도에
  선택한다. 이전 세션은 롤백을 위해 보존한다.
- recorder는 단순 구독·기록만 하므로 운영 중인 `map_server + AMCL`과 충돌하지
  않는다. 반면 지도 재생성용 `slam_toolbox`는 운영 `/map`·TF와 분리된 환경에서
  실행해야 한다.

## 구현 범위와 후속 작업

현재 구현에는 명시적 지도 생성 세션의 rosbag2 기록, 지도·posegraph 저장,
checksum 검증, 검증된 저장 지도 선택이 포함된다. 일반 주행 중 상시 또는 이벤트
기반으로 원본 토픽을 서버에 기록하는 운영용 recorder, rosbag2의 격리 재처리,
지도 후보 승인·활성 버전 전환은 후속 구현 항목이다.

## 근거

- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/config/slam.yaml`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_slam/scripts/run_mapping_session.sh`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/launch/navigation.launch.py`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/config/nav2.yaml`
- `vehicle_simulator_model/ubuntu/ros2_ws/src/mentorpi_nav/scripts/map_session.py`
- `vehicle_simulator_model/ubuntu/README.md`
