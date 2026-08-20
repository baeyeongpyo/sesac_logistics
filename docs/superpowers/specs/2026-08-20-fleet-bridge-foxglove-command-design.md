# Fleet Bridge 직접 Foxglove 중계와 테스트 명령 API 설계

## 목적

차량 간 DDS 중계를 제거하고, 차량의 Foxglove Bridge를 통해 서버로 telemetry를
전달한다. 서버는 차량별 고정 WebSocket URI를 신뢰 경계로 사용해 원본 telemetry를
서버 Domain 225의 `/{robot_id}/*` topic으로 재발행한다. 서버에는 테스트 목적의
`cmd_vel` 및 `stop` REST API와 Swagger UI를 제공한다.

## 배경과 제약

- `robot_1`은 Domain 215, `robot_2`는 Domain 216, 서버는 Domain 225를 사용한다.
- 차량 주행 ROS graph와 동일한 컨테이너/IPC 환경에서 차량 Foxglove Bridge가
  실행되어야 한다. 별도 sidecar가 원본 `/tf`를 수신하지 못하는 현재 문제를 이
  구조의 전제 조건으로 삼지 않는다.
- 차량과 서버 사이에는 ROS 2 DDS discovery나 domain bridge를 두지 않는다.
- 차량 ID는 WebSocket payload나 client가 주장하는 값이 아니라 서버의
  `fleet.yaml`의 `id`와 `foxglove_uri` 고정 매핑으로 결정한다.
- 기존 pin인 Foxglove Bridge 0.8.5와 `foxglove.websocket.v1` protocol을 유지한다.
- telemetry의 topic prefix만 바꾼다. `TFMessage` 내부의 `frame_id`와
  `child_frame_id`는 이번 범위에서 재작성하지 않는다.

## 목표 구조

```text
vehicle ROS graph (Domain 215 / 216)
  /tf, /odom, /amcl_pose, ...       /cmd_vel
             |                         ^
             v                         |
       Foxglove Bridge :8766 -----------+ clientPublish
             | WebSocket
             v
server telemetry worker (Domain 225)    server command API
  /tf -> /robot_1/tf                    REST -> command client -> /cmd_vel
  /odom -> /robot_1/odom
             |
             v
       server Foxglove Bridge :8765
```

## 구성 방식

### Telemetry

`telemetry.yaml`에서 `source`는 차량 Bridge가 노출하는 원본 차량 topic,
`target`은 서버 Domain 225에서 발행하는 차량 prefix topic으로 사용한다. 과거
sidecar filter용 `uplink`은 하위 호환 설정으로 유지하되, 직접 Foxglove 모드의
서버 worker는 이를 선택 기준으로 사용하지 않는다.

차량 Bridge의 `topic_whitelist`는 활성 telemetry의 원본 `source`만 허용한다.
필요할 때 scan/camera/battery를 기존 `enabled`, QoS, rate/filter 정책으로
추가한다. 원본을 직접 송신하는 모드에서는 차량 `fleet_telemetry_filter`를 시작하지
않는다.

### 명령

각 enabled 차량은 설정으로 다음 명령 계약을 가진다.

- topic: 기본 `/cmd_vel`
- type: `geometry_msgs/msg/Twist`
- 최대 선속도(`max_linear_x`), 최대 각속도(`max_angular_z`)
- 최대 유지 시간(`max_hold_ms`)

REST API는 Foxglove protocol의 `clientPublish` capability를 확인한 뒤 CDR로
serialized `Twist`를 고정 차량 Bridge에 발행한다. WebSocket client는 연결마다
`/cmd_vel` CDR channel을 advertise하고, 해당 channel에 message data를 전송한다.
Bridge의 `client_topic_whitelist`가 `/cmd_vel`만 수락하므로 다른 ROS topic을 원격
발행할 수 없다.

`POST /api/v1/robots/{robot_id}/cmd_vel` request body는 다음과 같다.

```json
{
  "linear_x": 0.1,
  "angular_z": 0.0,
  "hold_ms": 300
}
```

`hold_ms`는 `1..max_hold_ms` 범위이고, command worker는 유지 시간 동안 제한된
주기로 Twist를 발행한 뒤 반드시 zero Twist를 전송한다. `POST
/api/v1/robots/{robot_id}/stop`은 즉시 zero Twist를 전송한다. 알 수 없는 robot,
disabled robot, capability 부재, 연결/발행 실패는 성공으로 위장하지 않고 명시적
HTTP 오류로 반환한다.

이 API는 테스트 전용이며 인증을 포함하지 않는다. 기본 bind address는
`127.0.0.1`로 제한하고, Fleet Manager가 다른 서버/컨테이너이면 설정으로 명시적으로
노출한다. 차량의 8766은 방화벽으로 서버 IP만 접근하도록 제한해야 한다.

### Swagger / OpenAPI

서버 command API는 FastAPI로 제공한다. `/docs`는 Swagger UI, `/openapi.json`은
OpenAPI schema, `/healthz`는 API 상태 확인 endpoint이다. Pydantic request model의
범위·설명·예시를 Swagger에서 바로 볼 수 있게 한다. API 컨테이너는 telemetry worker와
독립 서비스로 실행하며, 자체 ROS publisher가 아니라 WebSocket command client만
사용한다.

## 설정 및 배포

`fleet.yaml`의 서버 항목에 command API bind host/port를 추가하고, vehicle 항목에는
명령 topic과 안전 한계를 둔다. `.env.example`에는 API host/port와 차량 URI 예시를
추가한다. `docker-compose.server.yaml`에는 `command-api` 서비스를 추가한다.

vehicle launch는 raw 직접 모드에서 Bridge만 실행하고 다음을 적용한다.

- `topic_whitelist`: 활성 `source` telemetry topic
- `capabilities`: `clientPublish`
- `client_topic_whitelist`: 구성된 `/cmd_vel` 하나

이 변경은 Bridge 프로세스를 주행 컨테이너 안에서 시작했을 때 적용된다. 기존
sidecar compose 방식은 raw telemetry를 보장하지 않으므로 문서에서는 레거시로
분리한다.

## 검증 기준

1. config loader는 command API 설정, vehicle command safety policy, URI/토픽/범위를
   검증한다.
2. raw `/tf` channel은 `/{robot}/tf` publisher와 연결되고, `uplink` 이름만 제공된
   channel은 직접 모드에서 선택하지 않는다.
3. command protocol은 `clientPublish` capability가 없는 Bridge를 거부하고,
   advertise와 binary message frame을 올바르게 생성한다.
4. API는 request validation, unknown/disabled robot, WebSocket failure를 HTTP 오류로
   변환하고, 유효 요청은 제한된 속도와 자동 zero Twist를 command client에 전달한다.
5. Swagger/OpenAPI에 두 command endpoint와 schema 예시가 노출된다.
6. compose와 문서는 API port, `/docs`, 차량 Bridge whitelist 및 실행 절차를 정확히
   반영한다.

## 범위 밖

- Open-RMF task, navigation goal, docking, lifecycle/service 명령
- 인증/권한/감사 로그와 TLS termination
- TF frame ID 재작성과 다중 차량 통합 TF tree
- vehicle image 또는 이미 실행 중인 주행 컨테이너를 자동 변경하는 작업
