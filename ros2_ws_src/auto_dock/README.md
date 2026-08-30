# Auto Dock ROS 2 연동 규격

이 패키지는 Nav2 도착 이벤트를 받아 팔레트를 탐색·정렬·삽입하고, 포크 작업이 끝나면 삽입한 거리만큼 후진한 뒤 `drive_ready`를 발행한다. 차량 1과 차량 2는 같은 토픽 이름을 일부 공유하지만 DDS 도메인이 분리되어 있다.

| 차량 | `ROS_DOMAIN_ID` | 기본 로봇 prefix |
| --- | ---: | --- |
| 1호차 | `215` | `/robot_1` |
| 2호차 | `216` | `/robot_2` |

아래 예시는 2호차 기준이다. 1호차에서는 `ROS_DOMAIN_ID=215`와 `/robot_1`을 사용한다.

## 빌드와 실행

소스를 차량의 `/home/ubuntu/ros2_ws/src/auto_dock`에 둔 뒤 일반 빌드를 사용한다. 이 프로젝트에서는 `--symlink-install`을 사용하지 않는다.

```bash
cd /home/ubuntu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select auto_dock
source install/setup.bash
```

2호차 실행 환경:

```bash
export ROS_DOMAIN_ID=216
ros2 launch auto_dock auto_dock.launch.py vehicle:=2
```

launch는 `/yolo_tag`가 이미 있으면 기존 YOLO 노드를 사용하고, 없으면 `/shared/yolo_symbol_seg_node.py`를 실행한다. `fork_controller`와 `auto_dock`도 함께 실행한다.

## 전체 토픽 계약

### 다른 노드가 Auto Dock으로 발행해야 하는 토픽

| 발행 주체 | 토픽 | 타입 | 용도 |
| --- | --- | --- | --- |
| Nav2/Fleet Adapter | `/robot_2/nav2/arrival` | `std_msgs/msg/String` | 도착 완료와 PICK/PLACE 목표 전달 |
| 안전 UI/관제 | `/robot_2/auto_dock/stop` | `std_msgs/msg/Empty` | 즉시 주행 정지, 포크 `STOP`, 작업 취소 |
| Fork Controller | `/robot_2/fork/state` | `std_msgs/msg/String` | 포크 상승/하강 완료 또는 실패 통보 |
| YOLO 노드 | `/robot_2/symbol_seg/detections` | `std_msgs/msg/String` | 팔레트·태그·거리·자세 JSON |
| UI | `/robot_2/dock/inventory/reset` | `std_msgs/msg/Empty` | 테스트 위치 변경 시 DOCK 슬롯 기록 초기화 |
| 테스트 UI 전용 | `/robot_2/auto_dock/test/load_state` | `std_msgs/msg/String` | 정지 상태에서 `LOADED`/`UNLOADED` 강제 지정 |
| 센서 드라이버 | `/scan_raw` | `sensor_msgs/msg/LaserScan` | 충돌 방지와 후방/측면 거리 |
| Odom | `/odom_raw` | `nav_msgs/msg/Odometry` | 목표 pose 유지와 이동 거리 |
| IMU | `/imu/rpy/filtered` | `geometry_msgs/msg/Vector3Stamped` | `vector.z` yaw(rad) |
| RGB 카메라 | `/ascamera/camera_publisher/rgb0/image` | `sensor_msgs/msg/Image` | 주의선·도크 끝선·슬롯 검출 |
| 카메라 정보 | `/ascamera/camera_publisher/rgb0/camera_info` | `sensor_msgs/msg/CameraInfo` | PnP 카메라 보정값 |

### Auto Dock이 발행하는 토픽

| 토픽 | 타입 | 수신 주체/의미 |
| --- | --- | --- |
| `/controller/cmd_vel` | `geometry_msgs/msg/Twist` | 차량 베이스 주행 명령 |
| `/fork/command` | `std_msgs/msg/String` | 포크에 `UP`, `DOWN`, `STOP` 전달 |
| `/robot_2/auto_dock/status` | `std_msgs/msg/String` | 상태 JSON; Reliable + Transient Local |
| `/robot_2/dock/inventory` | `std_msgs/msg/String` | DOCK 3×8 중 현재 확인된 슬롯 JSON |
| `/robot_2/auto_dock/drive_ready` | `std_msgs/msg/Empty` | 후진/회전까지 끝나 Nav2가 다시 주행 가능 |

`/fork/command`, `/controller/cmd_vel`, `/scan_raw`, `/odom_raw`는 절대 토픽이다. 차량 prefix가 없어도 1호차와 2호차의 DDS 도메인이 달라 서로 섞이지 않는다.

## Nav2/Fleet Adapter가 발행할 Arrival JSON

토픽은 `/robot_2/nav2/arrival`, 타입은 `std_msgs/msg/String`이다. `data` 안에는 반드시 JSON object 한 개가 들어가야 한다.

공통 필드:

| 필드 | 허용값 | 설명 |
| --- | --- | --- |
| `status` | `SUCCEEDED` | Nav2가 목적지에 정상 도착했을 때만 작업 시작 |
| `location` | 예: `DOCK_1`, `NORMAL`, `FRESH` | 현재 작업 위치 |
| `operation` | `PICK`, `PLACE` | 포크 작업 방향 |
| `product_type` | `NORMAL`, `FRESH` | 별 태그가 하나라도 있으면 `FRESH`, 아니면 `NORMAL` |
| `target.type` | `SYMBOLS`, `NEAREST`, `SLOT`, `AUTO_SLOT`, `NONE` | 목표 선택 방식 |

### 태그 윗줄을 지정해 PICK

```json
{
  "status": "SUCCEEDED",
  "location": "DOCK_1",
  "operation": "PICK",
  "product_type": "NORMAL",
  "target": {
    "type": "SYMBOLS",
    "left": "diamond",
    "right": "spade"
  }
}
```

심볼은 소문자 `star`, `diamond`, `spade`, `clover`, `heart` 중 하나다. 좌우 심볼이 같아도 허용한다.

2호차 발행 명령:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/nav2/arrival std_msgs/msg/String "data: '{\"status\":\"SUCCEEDED\",\"location\":\"DOCK_1\",\"operation\":\"PICK\",\"product_type\":\"NORMAL\",\"target\":{\"type\":\"SYMBOLS\",\"left\":\"diamond\",\"right\":\"spade\"}}'"
```

### 태그를 지정하지 않고 최근접 상품 PICK

`NEAREST`는 `PICK`에서만 허용한다. 지정한 상품 종류와 일치하는 완전한 2×2 태그 엔티티 중 현재 접근 가능한 C1 후보를 선택한다. DOCK 슬롯 정보가 아직 없으면 화면에서 측정 가능한 최근접 후보를 사용한다.

NORMAL:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/nav2/arrival std_msgs/msg/String "data: '{\"status\":\"SUCCEEDED\",\"location\":\"DOCK_1\",\"operation\":\"PICK\",\"product_type\":\"NORMAL\",\"target\":{\"type\":\"NEAREST\"}}'"
```

FRESH:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/nav2/arrival std_msgs/msg/String "data: '{\"status\":\"SUCCEEDED\",\"location\":\"DOCK_1\",\"operation\":\"PICK\",\"product_type\":\"FRESH\",\"target\":{\"type\":\"NEAREST\"}}'"
```

### 슬롯 목표

명시 슬롯 형식은 `R1C1`, `R1_C1`, `NORMAL_R1_C1`, `FRESH_R1_C1`을 허용한다. 현재 구현에서 `SLOT`은 목표를 검증하고 저장하지만 실제 슬롯 주행은 아직 구현되지 않았으므로 운영 명령으로 사용하지 않는다. `AUTO_SLOT`도 슬롯 스캔 단계까지만 개발 상태다.

```json
{
  "status": "SUCCEEDED",
  "location": "NORMAL",
  "operation": "PLACE",
  "product_type": "NORMAL",
  "target": {"type": "SLOT", "slot_id": "R3C1"}
}
```

## Fork Controller가 지켜야 하는 포맷

Auto Dock은 삽입을 마치면 `/fork/command`에 평문 `UP` 또는 `DOWN`을 발행한다. Fork Controller는 실제 동작이 완료된 뒤 `/robot_2/fork/state`에 JSON을 발행해야 한다. 명령을 받자마자 완료를 발행하면 Auto Dock이 포크가 움직이기 전에 후진하므로 반드시 실제 완료 시점에 발행한다.

PICK 상승 완료:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/fork/state std_msgs/msg/String "data: '{\"state\":\"UP_COMPLETE\",\"error\":\"\"}'"
```

PLACE 하강 완료:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/fork/state std_msgs/msg/String "data: '{\"state\":\"DOWN_COMPLETE\",\"error\":\"\"}'"
```

실패:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/fork/state std_msgs/msg/String "data: '{\"state\":\"FAILED\",\"error\":\"limit switch timeout\"}'"
```

Auto Dock은 PICK 중에는 `UP_COMPLETE`, PLACE 중에는 `DOWN_COMPLETE`만 완료로 인정하며 다른 상태는 무시한다. `FAILED`를 받으면 주행과 포크를 정지하고 `ERROR` 상태를 발행한다.

## YOLO 노드가 지켜야 하는 detection JSON

기본 구현은 `tools/yolo_symbol_seg_node.py`가 이 메시지를 발행한다. 다른 인식 노드로 교체할 경우 최소한 아래 구조를 유지해야 한다.

```json
{
  "source_stamp_ns": 1234567890000000000,
  "target_top": ["diamond", "spade"],
  "detections": [
    {
      "class": "diamond",
      "confidence": 0.82,
      "box": [120, 180, 180, 240],
      "depth": {
        "forward_distance_cm": 24.1,
        "camera_depth_m": 0.241,
        "bearing_deg": -3.2
      }
    }
  ],
  "entities": [
    {
      "entity_id": 7,
      "seen_count": 3,
      "matrix": ["diamond", "spade", "heart", "clover"],
      "image_pallet_box": [90, 140, 310, 410],
      "frontal_error": 0.08,
      "top_row_error": 0.04,
      "bottom_row_error": 0.06,
      "pnp": {
        "forward_distance_cm": 24.5,
        "lateral_ratio": -0.03,
        "yaw_deg": 1.8,
        "reprojection_error_px": 1.2
      },
      "depth_yaw": {
        "forward_distance_cm": 24.1,
        "yaw_deg": 1.5
      }
    }
  ],
  "candidate": {
    "entity_id": 7,
    "streak": 3,
    "matrix": ["diamond", "spade", "heart", "clover"],
    "center_error": -0.03,
    "frontal_error": 0.08,
    "top_row_error": 0.04,
    "bottom_row_error": 0.06,
    "pallet_box": [90, 140, 310, 410],
    "pnp": {
      "forward_distance_cm": 24.5,
      "lateral_ratio": -0.03,
      "yaw_deg": 1.8,
      "reprojection_error_px": 1.2
    },
    "depth_yaw": {
      "forward_distance_cm": 24.1,
      "yaw_deg": 1.5
    }
  }
}
```

`SYMBOLS` 작업에는 `target_top`과 `candidate`가 필요하다. `NEAREST` 작업에는 완전한 네 태그를 가진 `entities[]`가 필요하며, `matrix` 순서는 `[왼쪽 위, 오른쪽 위, 왼쪽 아래, 오른쪽 아래]`다. 거리 선택은 `depth_yaw.forward_distance_cm`을 우선하고 없으면 `pnp.forward_distance_cm`을 사용한다. `entity_id`는 프레임 사이에서 같은 팔레트에 대해 안정적으로 유지해야 중복 인식과 목표 변경을 막을 수 있다.

## Auto Dock 상태를 수신하는 방법

```bash
ROS_DOMAIN_ID=216 ros2 topic echo /robot_2/auto_dock/status std_msgs/msg/String
```

상태 JSON 공통 예시:

```json
{
  "state": "ALIGNING",
  "reason": "translation_first_alignment",
  "stamp_monotonic": 12345.67,
  "operation": "PICK",
  "product_type": "NORMAL",
  "location": "DOCK_1",
  "load_state": "UNLOADED",
  "slot_id": null
}
```

외부 FSM은 `reason` 문자열로 완료 여부를 판정하지 말고 `state`를 사용한다.

| `state` | 의미 |
| --- | --- |
| `IDLE` | 새 Arrival 수신 가능 |
| `SEARCHING` | 횡탐색/후보 확인 중 |
| `ALIGNING` | 목표 정렬 중 |
| `INSERTING` | 팔레트 방향 전진 중 |
| `WAIT_UP_COMPLETE` | PICK 포크 상승 완료 대기 |
| `WAIT_DOWN_COMPLETE` | PLACE 포크 하강 완료 대기 |
| `REVERSING` | 삽입 거리만큼 후진 중 |
| `TURNING` | Ready 자세 회전 중 |
| `READY` | Auto Dock 작업 완료 |
| `ERROR` | 취소·센서·안전 오류 |

`READY`와 별도로 `/robot_2/auto_dock/drive_ready`의 `Empty`를 받으면 다음 Nav2 주행을 시작할 수 있다.

## DOCK 슬롯 정보와 초기화

```bash
ROS_DOMAIN_ID=216 ros2 topic echo /robot_2/dock/inventory std_msgs/msg/String
```

`visible_nearest`에는 현재 확인된 슬롯만 들어간다. 보이지 않은 슬롯은 빈 슬롯이 아니라 `UNKNOWN`이다. `accessible: true`는 현재 가장 가까운 C1이라 바로 접근 가능하다는 뜻이고, C2/C3은 앞 열에 가려진 것으로 취급한다.

테스트 시작 위치를 바꿨거나 다른 도크로 이동했으면 누적된 슬롯을 먼저 초기화한다.

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/dock/inventory/reset std_msgs/msg/Empty '{}'
```

## 긴급정지와 테스트용 상태 변경

Auto Dock 정지:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/auto_dock/stop std_msgs/msg/Empty '{}'
```

테스트용 UNLOADED 지정:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/auto_dock/test/load_state std_msgs/msg/String "data: 'UNLOADED'"
```

테스트용 LOADED 지정:

```bash
ROS_DOMAIN_ID=216 ros2 topic pub --once /robot_2/auto_dock/test/load_state std_msgs/msg/String "data: 'LOADED'"
```

이 테스트 토픽은 Auto Dock이 `IDLE` 또는 `READY`일 때만 적용된다. 실제 운용에서는 포크 완료 이벤트로 `load_state`가 바뀌어야 한다.

## 연동 확인 순서

```bash
ROS_DOMAIN_ID=216 ros2 topic info /robot_2/nav2/arrival -v
ROS_DOMAIN_ID=216 ros2 topic info /robot_2/symbol_seg/detections -v
ROS_DOMAIN_ID=216 ros2 topic echo /robot_2/auto_dock/status
ROS_DOMAIN_ID=216 ros2 topic echo /robot_2/auto_dock/drive_ready std_msgs/msg/Empty
```

Arrival publisher는 발행 전에 `/robot_2/nav2/arrival`에 matching subscription이 1개 이상인지 확인한다. Nav2와 Auto Dock이 동시에 `/controller/cmd_vel`을 발행하지 않도록 작업 소유권 전환 또는 velocity mux가 필요하다.
