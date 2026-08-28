# 차량 관제 토픽과 rosbag 기록 설계

**작성일:** 2026-08-21
**상태:** 사용자 승인 설계, 구현 계획 검토 대기

## 목표

서버 전용 `fleet_bridge` 번들을 확장해 각 차량의 주행, 상태, RGB, raw depth
telemetry를 중앙 ROS Domain 225에서 수신하고 rosbag2에 기록한다. 중앙 Foxglove
endpoint는 운영 telemetry와 RGB 이미지를 표시하되 depth 이미지는 절대 노출하지
않는다. 공통 지도는 차량 telemetry가 아니며, 기존 중앙 map service가
`/controller_server/map`의 유일한 발행자로 남는다.

## 경계와 소유권

```text
차량 ROS + 차량 Foxglove Bridge
  /odom, /tf, /scan_raw, /depth/image_raw, ...
      |  WebSocket CDR
      v
worker-robot-N (Domain 225) --> /robot_N/*
      |                                  |
      |                                  +--> rosbag-recorder
      v
server-foxglove :8765 --> 중앙 Foxglove (RGB, depth 제외)

중앙 map-server (Domain 225) --> /controller_server/map
                                      |             |
                                      +--> Foxglove +--> rosbag-recorder

worker-robot-N --> /robot_N/fleet_bridge/status
```

토픽 발행자는 세 종류이며, 설정을 서로 섞지 않는다.

| 발행자 | 토픽 형태 | 설정 소유자 |
| --- | --- | --- |
| 차량 중계 | `/{robot}/...` | `config/telemetry.yaml` |
| worker가 생성하는 연결 상태 | `/{robot}/fleet_bridge/status` | worker runtime; 활성 차량이면 항상 기록 |
| 중앙 서버 발행자 | `/controller_server/map` | `config/central_topics.yaml` |

`/map`과 `/{robot}/map`은 일반 차량 telemetry로 중계하지 않는다. 공통 지도 하나는
중앙 map service가 제공한다. `fleet_bridge` Compose 파일은 이 서비스를 새로 만들지
않고, Domain 225에서 이미 발행 중인 지도를 사용한다.

## 차량 Telemetry 계약

`config/telemetry.yaml`의 각 항목은 기존 계약을 유지한다. 필드는 `enabled`,
`source`, `target`, `type`, `worker_rate`, `qos`다. `enabled: true`이면 worker가
차량 Bridge channel을 구독하여 namespaced target으로 재발행하고, recorder도 해당
target을 포함한다. 값을 변경하면 대상 worker와 rosbag-recorder를 재생성해야 하며,
hot reload는 범위에 포함하지 않는다.

아래가 초기 활성 항목이다. source 이름은 현재 실제 차량 계약을 기준으로 한다.
차량 Bridge가 특정 항목을 advertise하지 않으면 worker는 나머지 stream을 실패시키지
않고 그 항목만 구독하지 않는다.

| ID | 차량 source | 중앙 target | Type | 중앙 Foxglove | rosbag |
| --- | --- | --- | --- | --- | --- |
| `odom` | `/odom` | `/{robot}/odom` | `nav_msgs/msg/Odometry` | 표시 | 기록 |
| `tf` | `/tf` | `/{robot}/tf` | `tf2_msgs/msg/TFMessage` | 표시 | 기록 |
| `tf_static` | `/tf_static` | `/{robot}/tf_static` | `tf2_msgs/msg/TFMessage` | 표시 | 기록 |
| `amcl_pose` | `/amcl_pose` | `/{robot}/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 표시 | 기록 |
| `scan_raw` | `/scan_raw` | `/{robot}/scan_raw` | `sensor_msgs/msg/LaserScan` | 표시 | 기록 |
| `scan_filtered` | `/scan_filtered` | `/{robot}/scan_filtered` | `sensor_msgs/msg/LaserScan` | 표시 | 기록 |
| `imu_data_raw` | `/imu/data_raw` | `/{robot}/imu/data_raw` | `sensor_msgs/msg/Imu` | 표시 | 기록 |
| `battery` | `/ros_robot_controller/battery` | `/{robot}/battery` | `sensor_msgs/msg/BatteryState` | 표시 | 기록 |
| `diagnostics` | `/diagnostics` | `/{robot}/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 표시 | 기록 |
| `rgb_image_raw` | `/ascamera/camera_publisher/rgb0/image` | `/{robot}/rgb/image_raw` | `sensor_msgs/msg/Image` | 표시 | 기록 |
| `depth_image_raw` | `/depth/image_raw` | `/{robot}/depth/image_raw` | `sensor_msgs/msg/Image` | 미표시 | 기록 |
| `depth_camera_info` | `/depth/camera_info` | `/{robot}/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 미표시 | 기록 |
| `navigation_goal` | `/move_base_simple/goal` | `/{robot}/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | 표시 | 기록 |
| `navigation_status` | `/navigation/status` | `/{robot}/navigation/status` | `std_msgs/msg/String` | 표시 | 기록 |
| `navigation_cmd_vel` | `/navigation/cmd_vel` | `/{robot}/navigation/cmd_vel` | `geometry_msgs/msg/Twist` | 표시 | 기록 |
| `controller_cmd_vel` | `/controller/cmd_vel` | `/{robot}/controller/cmd_vel` | `geometry_msgs/msg/Twist` | 표시 | 기록 |

주행 디버깅용으로 `/plan`, `/local_plan`, `/global_costmap/costmap`,
`/local_costmap/costmap`, `/navigate_to_pose/_action/status`도 선언하되 초기값은
`enabled: false`로 둔다. message type은 각각 `nav_msgs/msg/Path`,
`nav_msgs/msg/Path`, `nav_msgs/msg/OccupancyGrid`, `nav_msgs/msg/OccupancyGrid`,
`action_msgs/msg/GoalStatusArray`다. 코드 변경 없이 설정만으로 활성화할 수 있다.

센서 QoS는 짧은 queue의 best-effort/volatile을 사용하고, `tf_static`은
reliable/transient-local을 사용한다. 주행, 상태, status message는
reliable/volatile을 사용한다. raw image와 IMU는 받은 stream을 bag에 보존하도록
server-side rate cap을 두지 않는다. `scan_filtered`에는 운영 화면을 위한 2 Hz cap을
유지하고, `scan_raw`에는 bag replay를 위해 cap을 두지 않는다.

이 정책은 raw image 전송량을 줄이지 않는다. depth를 기록하는 동안에는 차량 Bridge를
통과해야 한다. 차량 측 image 압축, 해상도 축소, FPS 제한은 별도 후속 작업으로 한다.

## 중앙 표시 정책

`config/server_foxglove.yaml`은 현재의 넓은 `^/robot_N/.*$` pattern 대신 중앙 표시용
정확한 allowlist를 사용한다. 위 표에서 표시하는 target, `/robot_N/fleet_bridge/status`,
`/controller_server/map`만 허용한다. 두 depth target과 비활성 Nav2 debugging target은
포함하지 않는다.

따라서 차량 Foxglove Bridge에 직접 연결하면 raw depth를 볼 수 있지만,
`ws://<central-server>:8765`에 연결한 클라이언트는 RGB만 보고 depth는 발견하거나
구독할 수 없다. 중앙 Bridge는 publishing, service, parameter를 모두 막은 관측 전용
상태를 유지한다.

## 기록 서비스

`docker-compose.server.yaml`에 `rosbag-recorder` 서비스를 추가한다. 기존 server
image를 사용하고 Domain 225, host network, host IPC를 사용한다. 다음을 mount한다.

- `config/fleet.yaml`, `config/telemetry.yaml` (read-only)
- 새 `config/central_topics.yaml` (read-only)
- 필수 `ROSBAG_HOST_DIRECTORY`를 `/rosbag`으로 mount

새 `central_topics.yaml`은 차량과 독립된 중앙 지도 항목을 가진다.

```yaml
version: 1
topics:
  - id: controller_map
    enabled: true
    topic: /controller_server/map
```

`fleet_rosbag_recorder` console entry point는 `ROBOT_IDS=robot_1,robot_2`에 대해
활성 telemetry를 읽어 모든 target을 확장하고, 차량별 worker status와 활성 중앙
topic을 추가한다. 그 명시적인 목록으로 `ros2 bag record`를 실행한다.
`ROSBAG_SESSION_ID`가 비어 있으면 UTC 기반 session directory를 만들고, 지정된
session ID는 유효해야 하며 기존 directory와 겹칠 수 없다. 기존 bag은 덮어쓰지
않는다.

server runtime에는 `ros-humble-rosbag2`를 설치한다. 추후 optional Nav2
action-status를 활성화할 수 있도록 `ros-humble-action-msgs`도 설치한다.

## Runtime 검증

새 실제 차량 source를 활성화하기 전에 해당 차량에서 아래를 확인한다.

```bash
ros2 topic list -t | sort
ros2 topic info -v /ascamera/camera_publisher/rgb0/image
ros2 topic info -v /depth/image_raw
ros2 topic info -v /navigation/cmd_vel
ros2 topic hz /ascamera/camera_publisher/rgb0/image
```

이 번들의 Nav2는 camera node를 시작하거나 중지하지 않는다. 주행 중 RGB publisher가
사라지면 중앙 whitelist 전환이 아니라 차량 camera process 또는 차량 Bridge 문제다.
중앙 worker는 차량 Bridge가 advertise한 channel만 중계할 수 있다.

## 테스트와 완료 기준

1. configuration loader가 central topic document를 검증하고, unknown key, 중복 활성
   topic path, 잘못된 ID, 잘못된 topic name을 거부한다.
2. repository telemetry test는 활성 운영 set, 비활성 Nav2 debug set, 차량 map relay
   항목 부재를 검증한다.
3. recorder unit test는 두 차량 target의 결정적 확장, worker status와 중앙 map 포함,
   비활성 항목 제외, 안전한 session name/overwrite 처리를 검증한다.
4. Compose contract test는 recorder의 Domain 225, host network/IPC, read-only config
   mount, 필수 bag directory를 검증한다.
5. Foxglove contract test는 `/controller_server/map`과 RGB는 허용하고 depth path는
   허용하지 않으며 관측 전용 permission을 유지하는지 검증한다.
6. `docker compose --env-file .env.example -f docker-compose.server.yaml config
   --quiet`, configuration test, worker test, bundle/compose contract test가 통과한다.
   기존의 무관한 `.env.example` 변경은 보존하고, baseline contract mismatch는
   덮어쓰지 않고 보고한다.

## 범위 제외

- 차량 측 Foxglove Bridge 배포, 그 whitelist, camera launch process, Nav2 launch
  process 변경. 이들은 `fleet_bridge`의 범위 밖이다.
- image compression, image republisher, 중앙 map-server service, map 생성, TF frame
  rewriting 추가.
- recording 또는 telemetry 설정의 container 재생성 없는 hot reload.
