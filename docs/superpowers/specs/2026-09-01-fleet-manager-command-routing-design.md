# Fleet Manager 명령 중계 설계

## 목적

물류 오케스트레이터가 차량 모델별 통신 계층을 직접 알지 않도록 명령 경로를
다음과 같이 통일한다.

```text
logistics_orchestrator → fleet_manager → fleet_bridge → vehicle
```

차량이 보낸 상태와 명령 결과는 기존처럼 Bridge가 Fleet Manager에 전달하고,
Fleet Manager의 이벤트 전달을 통해 Orchestrator가 수신한다.

```text
vehicle → fleet_bridge → fleet_manager → logistics_orchestrator
```

## 현재 상태

- Orchestrator는 `HttpFleetBridgeClient`로 navigation과 auto-dock 명령을
  `FLEET_BRIDGE_URL`에 직접 전송한다.
- Fleet Manager는 차량 상태의 저장·조회와 Orchestrator 이벤트 전달만 제공한다.
- Fleet Bridge는 차량 명령 API로의 relay와 차량 상태의 Fleet Manager 전달을
  이미 구현했다.

따라서 차량·Bridge 기능을 재구현하지 않고, 명령 중계 책임만 Fleet Manager로
옮긴다.

## 책임 경계

| 구성요소 | 책임 | 하지 않는 일 |
| --- | --- | --- |
| Orchestrator | 재고·작업 단계에 따라 `navigate`, `auto_dock` 역할을 요청 | 차종·Bridge 주소·차량-native 프로토콜 판단 |
| Fleet Manager | 차량 ID의 모델/Bridge 조회, 지원 역할 확인, 표준 명령 relay | ROS/Foxglove/제조사 API 해석, 물류 작업 판단 |
| 모델별 Fleet Bridge | 표준 명령을 해당 차량의 API·ROS·Foxglove 명령으로 변환 | 재고·작업 배정 판단 |
| 차량 | 변환된 명령 수행 및 상태 보고 | 중앙 서비스 정책 판단 |

`map`, `warehouse zones`, 서버 Foxglove 관측 서비스는 차량 명령 변환 책임이
아니므로 이번 변경 범위에서 이동하거나 수정하지 않는다.

## 명령 계약

Fleet Manager는 Orchestrator에 다음 공통 명령 API를 제공한다. Bridge가 현재
사용하는 경로와 payload는 내부 구현으로만 유지한다.

```text
POST /api/v1/vehicles/{robot_id}/commands/navigation/goals
POST /api/v1/vehicles/{robot_id}/commands/auto-dock
```

- 요청 body는 현재 Orchestrator가 생성하는 navigation/auto-dock payload를
  그대로 사용한다.
- Fleet Manager는 `robot_id`의 등록 정보로 모델과 Bridge endpoint를 찾아
  해당 Bridge의 기존 `/api/v1/vehicle-command/{robot_id}/...` 경로로 전달한다.
- Bridge가 반환한 성공 또는 차량 거부 응답의 HTTP 상태와 JSON body는 변경하지
  않고 반환한다.
- 등록되지 않은 차량·비활성 차량·해당 역할 미지원 차량은 Fleet Manager가
  전달 전에 404 또는 409로 거부한다.
- Bridge 연결 실패는 503으로 반환한다.

초기 공통 역할은 `navigate`, `auto_dock`, `stop`, `report_status`로 정의한다.
이번 구현은 Orchestrator가 이미 사용하는 `navigate`와 `auto_dock`만 새 Manager
API로 중계한다. 나머지 역할은 같은 패턴으로 확장한다.

## 차량 모델과 Bridge 등록

Fleet Manager는 모델별 Bridge endpoint와 차량별 모델 할당을 분리해 배포 설정에
둔다.

```yaml
models:
  - id: mentorpi
    bridge_url: http://command-api:8080
    capabilities: [navigate, auto_dock, stop, report_status]

vehicles:
  - id: robot_1
    model: mentorpi
  - id: robot_2
    model: mentorpi
```

`bridge_url`은 차량 API가 아니라 Fleet Bridge의 command-api 주소다. 통합
`compose.local.yaml` 실행에서는 Fleet Manager와 현재 `command-api`가 동일 Compose
기본 네트워크에 있으므로 `http://command-api:8080` 서비스 DNS를 사용한다.
`host.docker.internal`은 컨테이너에서 호스트에 공개된 포트로 접근할 때만 쓰는
Docker Desktop 전용 별칭이며, 이 통합 배포의 서비스 간 주소로 사용하지 않는다.

각 차량의 실제 command API 주소(예: `http://192.168.100.38:8082`)는 해당 모델의
Fleet Bridge 설정에 둔다. MentorPi Bridge는 `robot_id`로 이 주소를 찾아 차량에
전달한다. 동일 모델의 여러 차량은 하나의 Bridge 배포본과 모델 capability를
공유한다. 새 차종은 별도의 Fleet Bridge 배포와 model 등록 항목을 더하는 방식으로
확장한다. Fleet Manager에는 차종별 ROS 또는 제조사 API 코드가 추가되지 않는다.

## 신뢰성과 오류 처리

- Orchestrator의 기존 command outbox는 Fleet Manager API 호출의 멱등성과
  delivery-unknown 처리를 계속 담당한다.
- 이번 범위에서는 Fleet Manager에 별도 명령 outbox나 자동 재시도를 추가하지
  않는다. Bridge 전달 여부가 불명확한 명령을 Manager가 중복 실행하는 위험을
  피하기 위함이다.
- Fleet Manager command API의 503은 Bridge 전달 완료 여부를 확인할 수 없다는
  뜻으로 취급한다. Orchestrator의 command client는 이 응답을 timeout과 동일하게
  `DELIVERY_UNKNOWN`으로 기록하며 즉시 재전송하지 않는다.
- Fleet Bridge의 상태 보고 경로는 유지한다. 결과 상태는 Fleet Manager에
  기록되고 기존 이벤트 outbox를 통해 Orchestrator에 전달된다.
- Manager가 역할을 지원하지 않는 차량을 거부하면 Orchestrator는 기존 오류
  기록 경로에 원인을 남긴다.

## 구현 범위

1. `fleet_manager`에 차량 등록 설정 loader, Bridge HTTP client, 명령 relay API를
   추가한다.
2. `logistics_orchestrator`의 Bridge client 의존성을 Manager command client로
   교체하고 `FLEET_BRIDGE_URL` 환경 변수를 제거한다.
3. Compose, `.env.example`, README, 계약·단위 테스트를 새 명령 경계에 맞춘다.
4. 기존 `fleet_bridge`의 차량 명령 변환·상태 relay·Foxglove·지도·zone 기능은
   변경하지 않는다.

## 검증

- Orchestrator가 Fleet Manager 명령 API만 호출하고 Bridge URL을 참조하지 않는지
  단위 테스트한다.
- Fleet Manager가 등록 차량의 capability를 확인해 올바른 Bridge endpoint로
  요청·응답을 relay하는지 테스트한다.
- 미등록/비활성/미지원 차량과 Bridge 연결 오류의 HTTP 응답을 테스트한다.
- 서비스별 전체 Python 테스트와 `docker compose -f operations/compose.local.yaml
  config --quiet`를 실행한다.

## 제외 범위

- 차량-native 명령 payload와 Fleet Bridge 내부 구현 변경
- Map, warehouse zone, Foxglove 서비스 분리
- Fleet Manager의 명령 영속화·재시도 구현
- 새 차량 모델 또는 새 Fleet Bridge 추가
