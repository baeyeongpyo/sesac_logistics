# 중앙 PC 다중 차량 Domain Bridge 설계

**작성일:** 2026-08-10  
**상태:** 사용자 설계 승인 반영, 구현 계획 검토 대기

## 목표

실제 차량 `robot_1`과 `robot_2`는 서로 다른 ROS 2 Domain에서 독립적으로
동작한다. 중앙 PC는 Domain 215에서 두 차량의 상태를 Foxglove로 관측하고,
차량별 수동 조작과 Nav2 목표 주행을 보낸다.

| 실행 위치 | ROS_DOMAIN_ID | 역할 |
| --- | ---: | --- |
| robot_1 | 1 | 차량 1 센서, 위치추정, Nav2, 최종 구동 제어 |
| robot_2 | 2 | 차량 2 센서, 위치추정, Nav2, 최종 구동 제어 |
| sim_robot_1 | 100 | 실제 차량 Domain 분리 조건을 재현하는 시뮬레이션 차량 |
| sim_robot_2 | 101 | 실제 차량 Domain 분리 조건을 재현하는 시뮬레이션 차량 |
| 중앙 PC | 215 | Domain Bridge, Foxglove Bridge, 관제 명령 발행 |

기존 `sim-up`은 Gazebo와 현재 단일 DDS 구성으로 계속 실행한다. 실제 차량 관제는
Gazebo를 시작하지 않는 독립된 `central-up` 경로로 제공한다.

## 아키텍처

중앙 PC의 `domain_bridge` 한 프로세스가 Domain 1, 2, 100, 101, 215의 참가자를 생성한다.
토픽별 `from_domain`과 `to_domain`을 명시해 자동 브로드캐스트나 와일드카드를
사용하지 않는다.

```text
robot_1 Domain 1       -- 차량 상태 -->\
robot_2 Domain 2       -- 차량 상태 --->\
sim_robot_1 Domain 100 -- 차량 상태 ----> domain_bridge --> 중앙 Domain 215 --> Foxglove
sim_robot_2 Domain 101 -- 차량 상태 --->/

중앙 Domain 215 -- {차량 namespace} 명령 --> 해당 차량 Domain
```

각 브리지 항목은 단방향이다. 같은 토픽을 양방향으로 브리지하지 않아 메시지
루프를 만들지 않는다. ROS 2 `domain_bridge`는 YAML에서 토픽 이름, 메시지 타입,
출발 Domain, 목적 Domain과 QoS를 명시할 수 있다.

## 토픽 계약

### 차량에서 중앙으로: 관측 전용

각 차량에 다음 이름공간 토픽을 Domain 215로 복제한다.

| 패턴 | 타입 | 용도 | QoS |
| --- | --- | --- | --- |
| `/{robot}/scan_raw` | `sensor_msgs/msg/LaserScan` | LiDAR 표시·Nav2 관측 | Best effort, keep_last 5 |
| `/{robot}/imu/data_raw` | `sensor_msgs/msg/Imu` | IMU 표시 | Best effort, keep_last 10 |
| `/{robot}/odom` | `nav_msgs/msg/Odometry` | 차량 자세·속도 | Best effort, keep_last 10 |
| `/{robot}/navigation/status` | `std_msgs/msg/String` | Nav2 상태 | Reliable, keep_last 10 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 차량 좌표 변환 | Best effort, keep_last 100 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 정적 좌표 변환 | Reliable, transient local, keep_last 1 |

`robot_1`과 `robot_2`의 frame ID는 이미 차량 접두어를 가져야 한다. 예를 들어
`robot_1/base_footprint`와 `robot_2/base_footprint`는 서로 충돌하지 않는다.

카메라·배터리·fork 등 실제 차량에서 추가로 사용하는 토픽은 메시지 타입과 QoS를
확인한 뒤 이 YAML의 차량별 관측 목록에 명시적으로 추가한다. 임의의 모든 토픽을
넘기지는 않는다.

### 중앙에서 차량으로: 제어

| 중앙 Domain 215 토픽 | 목적 차량 Domain 토픽 | 타입 | 경로 |
| --- | --- | --- | --- |
| `/{robot}/manual/cmd_vel` | `/{robot}/manual/cmd_vel` | `geometry_msgs/msg/Twist` | 수동 조작 요청 |
| `/{robot}/move_base_simple/goal` | `/{robot}/move_base_simple/goal` | `geometry_msgs/msg/PoseStamped` | Nav2 목표 입력 |
| `/{robot}/navigation/cancel` | `/{robot}/navigation/cancel` | `std_msgs/msg/Empty` | 활성 Nav2 목표 취소 |
| `/{robot}/safety/stop` | `/{robot}/safety/stop` | `std_msgs/msg/Empty` | 차량별 즉시 정지 요청 |

중앙에서 최종 모터 토픽 `/{robot}/controller/cmd_vel`로 직접 보내지 않는다.
각 차량의 로컬 제어 노드가 `manual/cmd_vel`, Nav2의 `cmd_vel_nav`, 정지 요청을
중재하여 최종 `controller/cmd_vel`을 하나만 발행한다.

우선순위는 `safety/stop` > `manual/cmd_vel` > `cmd_vel_nav`로 한다. 수동 명령은
짧은 watchdog 시간 안에 새 명령이 없으면 영(0) 속도로 바꾸고, 수동 조작 중에는
Nav2 속도를 최종 제어기에 전달하지 않는다. 수동 명령 종료 후에만 Nav2 주행을
재개한다. 이는 두 제어 원천이 동시에 차량을 구동하지 않게 하는 필수 안전 경계다.

`robot_1`의 네 개 제어 브리지 항목은 Domain 1만 목적지로 갖고, `robot_2`의
항목은 Domain 2만 목적지로 갖는다. `sim_robot_1`, `sim_robot_2`도 각각 Domain
100, 101만 목적지로 갖는다. 공용 `/cmd_vel` 또는 이름공간 없는 goal 토픽은
만들지 않는다.

## 중앙 PC 런타임

새 중앙 관제 compose overlay는 다음 서비스를 제공한다.

1. `domain-bridge`: `ros-humble-domain-bridge`와 중앙 브리지 YAML을 실행한다.
   실제 차량을 발견해야 하므로 host network와 UDPv4 DDS transport를 사용한다.
2. `foxglove-bridge`: Domain 215에서만 실행하며 Domain Bridge가 제공한 토픽을
   WebSocket으로 노출한다.

`central-up`, `central-down`, `central-logs`, `central-topics` 명령을 추가한다.
기존 `sim-up`은 Domain 0과 `robot_1`, `robot_2`의 공유 Gazebo world를 유지한다.
실제 차량과 같은 다중 Domain 검증이 필요할 때만 `sim_robot_1`과 `sim_robot_2`를
각각 Domain 100, 101로 시작한다.

각 실제 차량과 중앙 PC는 같은 LAN에서 해당 DDS Domain의 discovery가 가능해야
한다. 멀티캐스트가 차단된 네트워크라면 차량별 Discovery Server 또는 동등한 DDS
discovery 구성을 별도로 맞춰야 하며, 이 구성은 중앙 Bridge가 차량 토픽을
발견하기 위한 운영 전제다.

## 구현 범위와 제외 범위

포함:

- 이미지에 `ros-humble-domain-bridge` 설치
- 중앙 Domain 215 용 Docker Compose, 환경 예시, 브리지 YAML
- 차량별 관측·수동·Nav2 목표·취소·정지 토픽 매핑
- 차량 내부 수동/Nav2/정지 속도 중재 노드와 단위 테스트
- Foxglove 연결 및 중앙 Domain에서의 토픽 격리 검증

제외:

- 다중 차량 충돌 회피, 우선순위, 구역 예약, 교통 제어
- 원격 차량의 네트워크 방화벽·VLAN·Discovery Server 배포 자동화
- 실제 차량에 존재하지 않는 센서 토픽을 추측해 브리지하는 작업

## 검증 기준

1. Compose 정적 검사에서 중앙 서비스의 `ROS_DOMAIN_ID=215`와 host network를
   확인한다.
2. Bridge YAML 검사에서 모든 `robot_1` 제어 항목의 목적지가 1이고, 모든
   `robot_2` 제어 항목의 목적지가 2인지 확인한다.
3. 단위 테스트에서 robot_1 수동·Nav2·정지 요청이 robot_2 최종 제어 출력에
   영향을 주지 않음을 확인한다. 반대 방향도 같은 방식으로 확인한다.
4. 시뮬레이션 통합 테스트에서 Domain 100과 101의 대체 publisher를 사용해 중앙
   Domain 215에서 두 시뮬레이션 차량 관측 토픽을 받고, 각 중앙 명령이 정확히
   하나의 차량 Domain으로만 전달됨을 확인한다.
5. 실제 차량 연결 검증에서는 Foxglove가 두 namespace를 동시에 표시하고,
   수동 조작 watchdog과 safety stop이 각 차량에서 독립적으로 동작하는지
   확인한다.

## 근거

- 현재 프로젝트의 다중 차량 Nav2 계약은 `/{robot_id}/move_base_simple/goal`,
  `/{robot_id}/cmd_vel_nav`, `/{robot_id}/controller/cmd_vel`를 차량별 namespace로
  분리한다.
- ROS 2 Domain Bridge는 YAML로 개별 토픽의 출발·목적 Domain과 QoS를 설정한다.
  https://docs.ros.org/en/humble/p/domain_bridge/
- ROS 2 공식 Domain ID 문서는 Linux 기본 환경에서 `0–101` 및 `215–232`를
  임시 UDP 포트와 충돌하지 않는 안전 범위로 제시한다.
  https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Domain-ID.html
