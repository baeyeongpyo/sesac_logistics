# Fleet Manager

Fleet Manager는 차량의 최신 상태와 **상태 변경 로그**를 SQLite에 저장하고,
Orchestrator의 표준 명령을 차량 모델별 Fleet Bridge에 중계합니다. 물류 작업과
재고의 원장은 `inventory_db`에 있으며, 다음 작업 판단은
`logistics_orchestrator`의 책임입니다.

## 실행

```bash
cd operations/fleet_manager
cp .env.example .env
docker compose --env-file .env up --build
```

기본 API 주소는 `.env`의 `FLEET_MANAGER_API_PORT`를 사용합니다.

### Pallet 3 직접 실행 runner

Logistics Orchestrator를 거치지 않고 `NORMAL` 파렛트 하나를 `docker`에서
`p3`로 옮길 때는 Fleet Manager 호스트에서 다음과 같이 실행합니다.

```bash
cd /opt/logistics/operations/fleet_manager
python3 pallet3_direct_runner.py --robot-id robot_1
```

runner는 기본적으로 Inventory `http://192.168.100.27:8081`과 Fleet Manager
`http://192.168.100.27:8090`을 사용합니다. 배포 환경이 다르면
`--inventory-url`, `--fleet-url`, `--request-timeout-sec`로 덮어쓸 수 있습니다.
차량의 `:8083` Logistics Orchestrator나 ROS action/topic을 직접 호출하지 않으며,
모든 차량 명령은 Fleet Manager와 Fleet Bridge를 통과합니다. 따라서 기존 Fleet
상태 보고와 대시보드에서 주행 및 장치 보고를 계속 확인할 수 있습니다.

P3의 `place-completions`가 성공해 물류 `PLACE_COMPLETE`에 도달하면 Inventory 작업은
종료됩니다. 이후 Dock 1 복귀 명령에는 원래 `operation_id`가 없고, runner는 복귀
완료 보고를 기다리지 않습니다. `COMMUNICATION_LOST`(`CommunicationLost`),
`OPERATION_STATE_MISMATCH`(`OperationStateMismatch`), 동일 작업의 Fleet `FAIL`,
명령 전달 실패, `SIGINT` 또는 `SIGTERM`이면 runner는 실패 종료합니다. 이때
Inventory 작업을 자동 완료하거나 취소하지 않으므로 운영자가 활성 작업을 확인해
복구해야 합니다. signal 종료는 Fleet Manager stop 명령을 한 번 best-effort로
요청합니다.

`VEHICLE_REGISTRY_PATH`의 기본값은 `/app/config/vehicles.yaml`입니다. 통합
`operations/compose.local.yaml`에서는 이 파일이 모델별 Bridge endpoint와 차량별
모델 할당을 정의합니다.

```yaml
models:
  - id: mentorpi
    bridge_url: http://command-api:8080
    capabilities: [navigate, auto_dock, stop, report_status, fork, manual_drive]
vehicles:
  - id: robot_1
    model: mentorpi
  - id: robot_2
    model: mentorpi
```

`http://command-api:8080`은 통합 Compose 내부의 MentorPi Fleet Bridge 서비스
주소입니다. 차량의 실제 command API 주소는 Fleet Bridge 설정에만 두며, Fleet
Manager는 차량 IP나 ROS/Foxglove 프로토콜을 알지 않습니다.

## 차량 상태 보고

Fleet Bridge가 아래 API를 호출합니다.

```text
POST /api/v1/vehicles/{robot_id}/state
```

```json
{
  "state": "DRIVE",
  "previous_state": "WAIT",
  "operation_id": "73d5b9af-5a12-4f34-a96c-5de116df1e8e",
  "attempt_id": "2d6d014b-4041-4a22-ac02-917a77e6c709",
  "source": "NAV2",
  "detail": "NAVIGATION_STARTED",
  "observed_at": "2026-08-31T12:00:00Z"
}
```

`state`는 `INIT`, `WAIT`, `DRIVE`, `PICK`, `PLACE`, `FAIL` 중 하나입니다.
같은 상태의 상세 이벤트는 최신 상태만 갱신하고 로그를 추가하지 않습니다. 다만
`NAVIGATION_SUCCEEDED`, `AUTO_DOCK_PICK_COMPLETED`, `AUTO_DOCK_PLACE_COMPLETED`처럼
같은 `WAIT` 상태라도 Orchestrator가 구분해야 하는 실제 차량 보고는 모두 event outbox에
보존됩니다.

## 조회

```text
GET /healthz
GET /api/v1/vehicles
GET /api/v1/vehicles/{robot_id}
GET /api/v1/vehicles/{robot_id}/logs?limit=100&before_id={id}
```

## 차량 명령 중계

Orchestrator는 Fleet Bridge가 아닌 Fleet Manager에 아래 표준 역할을 요청합니다.

```text
POST /api/v1/vehicles/{robot_id}/commands/navigation/goals
POST /api/v1/vehicles/{robot_id}/commands/navigation/waypoints
POST /api/v1/vehicles/{robot_id}/commands/fork/down
POST /api/v1/vehicles/{robot_id}/commands/cmd-vel
POST /api/v1/vehicles/{robot_id}/commands/stop
POST /api/v1/vehicles/{robot_id}/commands/auto-dock
```

Fleet Manager는 `robot_id`의 model과 capability를 registry에서 확인한 뒤 해당
Bridge의 기존 차량 command API로 payload와 응답을 변경하지 않고 전달합니다.
미등록 차량은 404, 지원하지 않는 역할은 409, Bridge 전달 완료 여부를 확인할 수
없으면 503을 반환합니다. Orchestrator는 마지막 경우를 `DELIVERY_UNKNOWN`으로
기록하고 즉시 재시도하지 않습니다.

## Orchestrator 이벤트 전달

`ORCHESTRATOR_EVENT_URL`이 설정되면 Fleet Manager는 각 상태 보고를 상태 저장과 같은
SQLite transaction에 outbox로 기록하고, 같은 `event_id`로
`POST /api/v1/events/fleet`를 성공 2xx까지 지수 backoff 재전송합니다. 상태 변경 로그의
기록 여부와 무관하게 모든 report가 전달 대상입니다.

```dotenv
ORCHESTRATOR_EVENT_URL=http://host.docker.internal:8083/api/v1/events/fleet
EVENT_RETRY_INTERVAL_SEC=2
```
