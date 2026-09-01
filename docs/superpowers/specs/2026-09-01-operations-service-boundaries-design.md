# 운영 서비스 경계 분리 설계

## 목표

차량과 통신하는 중개 서버인 `fleet_bridge`에서 중앙 지도, 창고 zone
오버레이, 관제용 Foxglove Bridge 및 3D 자산 제공 책임을 제거한다. 각 책임은
독립적인 Docker bundle로 배포하되, 관제 PC의 ROS 2 Domain 225에서 기존 topic과
외부 URL을 그대로 제공한다.

## 서비스 경계

### `operations/fleet_bridge`

차량 통신 중개만 담당한다.

- 차량별 Foxglove WebSocket telemetry를 `/{robot_id}/*`로 재발행한다.
- 차량 Command API에 HTTP 명령을 중계한다.
- 재발행된 차량 TF와 battery telemetry의 최신 상태를 SQLite에 기록한다.
- `worker-robot-*`, `command-api`, `telemetry-writer`만 Compose 서비스로 둔다.
- 중앙 Foxglove Bridge, 지도, 창고 zone, Foxglove 3D 자산은 포함하지 않는다.

내부 ROS Python package 이름은 `fleet_bridge_worker`로 변경한다. 이는 차량
Foxglove 프로토콜을 사용하는 구현 세부 사항을 보존하면서, 중앙 Foxglove 서버와
다른 서비스라는 경계를 코드에서도 드러낸다.

### `operations/map_server`

중앙 정적 지도만 담당한다.

- `maps/map_0825.yaml`과 PGM을 소유하고 `/map` 및 `map -> map_visualization`
  변환을 `/tf`로 발행한다.
- `map-publisher` 한 서비스만 실행한다.
- map 데이터는 read-only mount로 컨테이너에 전달한다.
- Nav2 map-server나 차량 telemetry WebSocket에는 연결하지 않는다.

### `operations/warehouse_server`

창고 물류 지점 overlay만 담당한다.

- `config/warehouse_zones.yaml`을 소유하고 `/warehouse/zones` MarkerArray를
  reliable/transient-local QoS로 발행한다.
- `warehouse-zone-publisher` 한 서비스만 실행한다.
- 차량 API, 재고 DB, 지도 파일을 소유하거나 호출하지 않는다.

### `operations/monitoring`

관제 화면에 필요한 ROS WebSocket과 3D 자산 HTTP endpoint를 함께 담당한다.

- `foxglove-bridge`는 observation-only 설정으로 `:8765`에서 `/robot_N/*`,
  `/map`, `/tf`, `/warehouse/zones`를 노출한다.
- `asset-server`는 `:8088`에서 URDF와 mesh를 CORS 및 HTTP byte range 지원과
  함께 제공한다.
- 두 서비스는 하나의 monitoring bundle에 속하지만 별도의 최소 이미지로
  빌드한다. Foxglove Bridge 이미지에는 자산을 넣지 않고, asset-server에는 ROS/DDS
  의존성을 넣지 않는다.

## 배포 및 인터페이스 계약

모든 ROS 서비스는 같은 관제 호스트의 `ROS_DOMAIN_ID=225`, host network 및 host
IPC를 사용한다. 따라서 Docker 네트워크 의존성 대신 ROS topic이 유일한 서비스 간
계약이다.

| 제공자 | 제공 인터페이스 | 소비자 |
| --- | --- | --- |
| `fleet_bridge` | `/{robot_id}/*`, `/{robot_id}/fleet_bridge/status` | `monitoring`, `fleet_manager` telemetry DB 소비자 |
| `map_server` | `/map`, `/tf`의 `map -> map_visualization` | `monitoring` |
| `warehouse_server` | `/warehouse/zones` | `monitoring` |
| `monitoring` | `ws://<server-ip>:8765`, `http://<server-ip>:8088/...` | Foxglove 클라이언트 |

`fleet_bridge` Command API의 `:8080`, monitoring Foxglove Bridge의 `:8765`,
asset-server의 `:8088` 외부 endpoint는 주소와 의미를 바꾸지 않는다. 통합 실행
파일인 `operations/compose.local.yaml`은 네 신규 bundle을 include한다. 각 bundle도
자체 Compose 파일만으로 독립 실행할 수 있어야 한다.

## 이미지와 소스 구성

- `fleet_bridge`는 custom `fleet_bridge_config`와 `fleet_bridge_worker`, MentorPi
  message package 및 HTTP/WebSocket Python 의존성만 빌드한다. 중앙
  `foxglove_bridge` C++ 패키지는 빌드하지 않는다.
- `map_server`와 `warehouse_server`는 각자 하나의 ament Python package와 그
  서비스에 필요한 ROS message 패키지만 포함한다.
- `monitoring`의 Foxglove Bridge 이미지만 pin된 Foxglove Bridge source를
  빌드한다. 기존 asset-server의 Python slim 이미지는 monitoring 하위에서 유지한다.
- source/config/map/assets/tests는 서비스 소유 디렉터리로 함께 옮긴다. 이전
  `operations/foxglove_assert_server` 및 `fleet_bridge` 내부 map/warehouse/
  central Foxglove 파일은 남기지 않는다.

## 오류 처리와 운영 제약

- 각 Compose 서비스는 `restart: unless-stopped`, `SIGINT`, 10초 grace period를
  유지한다.
- 지도와 창고 설정 mount는 read-only이며 host 경로를 자동 생성하지 않는다.
- monitoring의 Foxglove Bridge는 `capabilities: [none]` 및 빈 client/service/
  parameter whitelist를 유지한다. 관제 endpoint에서 차량 명령을 발행할 수 없다.
- map 및 warehouse publisher가 중지되면 monitoring은 살아 있어도 해당 topic이
  사라진다. 이는 Compose `depends_on`이나 HTTP 호출로 숨기지 않고 ROS topic
  health로 운영자가 관찰한다.

## 검증 전략

1. 이동 전과 동일하게 map loader의 P2/P5/Nav2 geometry, warehouse marker
   layout, asset CORS/range 동작을 단위 테스트로 검증한다.
2. 새 Compose contract test로 각 bundle의 서비스 목록, image build context,
   host ROS runtime 환경, read-only mount, 외부 포트를 검증한다.
3. `fleet_bridge` contract test는 map/warehouse/central Foxglove 서비스와
   파일이 더 이상 없고 차량 중계 서비스만 존재함을 검증한다.
4. 최상위 Compose contract는 네 bundle을 모두 include하는지 검증한다.
5. Docker Compose config와 해당 Python unittest suites를 실행한다. ROS 2
   integration test는 서버 이미지가 있는 환경에서만 실행하며, 로컬의 FastAPI 및
   Chrome 부재로 인한 기존 baseline 실패는 별도로 보고한다.

## 비목표

- ROS topic 이름, QoS, 지도 형식, 창고 zone 형식, 차량 Command API 계약을
  변경하지 않는다.
- 차량 컨테이너나 `vehicle_communication` bundle은 변경하지 않는다.
- inventory, fleet manager, logistics orchestrator의 업무 책임을 이동하지 않는다.
