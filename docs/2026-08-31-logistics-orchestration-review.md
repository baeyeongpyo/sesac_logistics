# 물류 예약·주행 연계 로직 검토

작성일: 2026-08-31
범위: `inventory_db`, `fleet_bridge`, `vehicle_communication`, MentorPi/Nav2 실행 경로

## 결론

## 구현 반영 (2026-08-31)

차량 상태 저장 경계를 다음처럼 확정·구현했다.

```text
vehicle_communication (실제 Nav2·Auto Dock 이벤트)
  -> fleet_bridge (등록 차량 검증 + HTTP 전달만)
  -> fleet_manager (SQLite 최신 상태 + 상태 변경 로그)
```

- `fleet_manager`는 `INIT`, `WAIT`, `DRIVE`, `PICK`, `PLACE`, `FAIL`의 최신
  snapshot과 상태가 바뀔 때만 추가되는 로그를 저장한다.
- `inventory_db`는 계속 `operation_id`의 물류·재고 원장이다. Fleet Manager는 그
  값을 참조로만 저장하며 Inventory를 동기 조회하지 않는다.
- `logistics_orchestrator`는 아직 구현 범위가 아니다. 두 원장을 읽어 다음 작업을
  만들고 명령하는 책임은 이 서비스에 남겨 둔다.
- 상태 보고는 명령 수신이 아니라 Nav2 goal 수락·결과, Auto Dock status·drive_ready,
  stop/cancel, 차량 기동으로만 발생한다.

현재 세 서비스만으로는 "주행 완료 뒤 다음 물류 작업을 자동 생성"하는
운영을 안전하게 수행할 수 없다. `fleet_bridge`는 차량 HTTP 중계기로
유지하고, 예약·배차·시도·복구를 소유하는 별도 **물류 오케스트레이터**를
추가해야 한다.

Nav2의 성공은 목표 허용오차 안에 차량이 도착했다는 신호일 뿐이다. 그것은
PICK/PLACE, 포크 적재, 적치, 재고 정산의 성공 증명이 아니므로 다음 작업을
만들 수 있는 조건이 아니다.

## 현재 서비스 경계

```text
inventory_db ─── 예약·재고 원장 ───┐
                                  │
                         logistics_orchestrator
                           ├─ 작업 중복 제거 / 차량 lease
                           ├─ dispatch attempt / timeout / recovery
                           └─ 완료 이벤트 inbox / outbox
                                  │
fleet_bridge ─── 무상태 HTTP relay ─┘
                                  │
vehicle_communication ─ Nav2·Auto Dock 실행 및 물리 관측
```

| 구성요소 | 소유해야 하는 것 | 소유하지 말아야 하는 것 |
| --- | --- | --- |
| `inventory_db` | 재고, source/destination 예약, 검증된 차량 적재 장부, location revision, 재고 이벤트 | 명령 전송, 차량 선택 정책, HTTP 재시도 |
| `fleet_bridge` | 차량 설정, relay 상태, 연결 상태, 제한된 delivery ledger | 재고 예약, 전역 작업 큐, 차량 lease의 권위, 다음 작업 정책 |
| `vehicle_communication` | Nav2·Auto Dock 실행 attempt, 단일 차량 상태, 물리 관측 증거 | 재고 정산의 권위, 다차량 배차 |
| `logistics_orchestrator` | 요청 중복 제거, 차량 lease, 작업 상태기계, 시도/재시도/보상, 완료 이벤트 처리 | 모터 직접 제어, 재고 수량의 원장 |

## 5회 독립 검토 결과

세 에이전트가 각 관점에서 총 15회의 읽기 전용 검토를 수행했다.

| 검토 축 | 확인된 문제 | 필요한 보완 |
| --- | --- | --- |
| 1. 데이터·위치 모델 | 활성 작업이 참조하는 zone 좌표를 수정하면 다음 지시가 예약 당시가 아닌 최신 좌표를 사용한다. | 작업에 `location_revision`, `map_version`, pose snapshot을 저장한다. |
| 2. 예약·동시성 | SQLite 내부 source/destination 예약은 원자적이지만 작업 생성은 멱등하지 않고, 읽기 시점도 분리돼 있다. | `client_request_id` UNIQUE, 작업/차량/zone을 한 read snapshot으로 조회한다. |
| 3. Bridge 계약·배차 | Bridge는 요청/응답 중계만 하며 `operation_id`의 attempt, lease, delivery 상태를 저장하지 않는다. | 오케스트레이터가 `assignment_id`, `attempt_id`, `command_version`을 소유한다. |
| 4. 완료·보상 | Inventory에는 성공 PICK/PLACE만 반영되고 cancel/fail/retry 보상 경로가 없다. | PICK 전 실패는 예약 해제, PICK 후 실패는 `RECOVERY_REQUIRED`로 분리한다. |
| 5. 물리·복구 | Nav2 성공과 Auto Dock `drive_ready`만으로는 동일 작업의 물리 완료를 보장하지 못하며, 재시작 시 차량 상태가 사라진다. | 작업·dock attempt 상관관계, 포크/비전 증거, reconciliation 절차를 강제한다. |

## 현재 구현에서 우선 해결할 위험

1. **예약 고착**: `FAILED`, `CANCELLED`, `RECOVERY_REQUIRED` enum은 있지만,
   실제 쓰기 전이는 성공 PICK/PLACE 중심이다. 차량 취소·실패 후 source 예약,
   destination slot, 차량 잠금이 남을 수 있다.
2. **전송 결과 불명**: Bridge의 HTTP timeout 뒤 차량이 명령을 이미 수락했는지
   알 수 없다. 이 상태를 실패로 재시도하면 중복 goal 또는 중복 도킹이 생긴다.
3. **완료 상관관계 부족**: Auto Dock 도착 payload와 `drive_ready`가 현재 작업의
   `operation_id`/dock attempt와 강하게 결합돼 있지 않다. 지연된 이전 이벤트가
   다음 작업을 완료 처리할 수 있다.
4. **물리 장부 불일치**: PICK 직후 재고 완료 API 호출 전에 통신이 끊기면 실제
   차량은 적재됐지만 Inventory는 `TO_PICK`으로 남는다. 이때 자동 재배정은
   이중 PICK 위험을 만든다.
5. **명령 소유권 경쟁**: joystick, 앱, Nav2, 수동 HTTP 명령은 모두 구동 명령을
   만들 수 있다. stop/cancel만으로는 다른 publisher를 차단하지 못한다.

## 권장 작업 상태기계

```text
REQUEST_DEDUPED
  -> RESERVED
  -> VEHICLE_LEASED
  -> NAV_DISPATCHING
  -> NAV_ACCEPTED(operation_id, attempt_id)
  -> NAV_ARRIVED_VERIFIED
  -> PICK_REQUESTED | PLACE_REQUESTED
  -> PHYSICAL_PICK_VERIFIED | PHYSICAL_PLACE_VERIFIED
  -> INVENTORY_COMMITTED
  -> NEXT_JOB_RESERVED
```

각 상태 전이는 `operation_id`, `assignment_id`, `attempt_id`, `command_id`,
`state_version`, `observed_at`을 가진다. 상태 변경은 compare-and-swap(CAS)로
수행하고, 외부 호출 전 outbox를 기록하며, 수신 이벤트는 다음 키로 중복 제거한다.

```text
(operation_id, attempt_id, action, vehicle_event_id)
```

### 다음 작업을 생성해도 되는 유일한 조건

1. PLACE에 대해 같은 `operation_id`와 dock attempt를 가진 물리 완료 증거가 있다.
2. Inventory의 PLACE 완료 트랜잭션이 성공해 차량 적재 장부가 비적재이고,
   운송 작업이 `COMPLETED`다.
3. 차량 lease가 여전히 유효하고, command health·필수 telemetry가 fresh다.
4. 이전 작업의 outbox를 전달하거나 중복 안전하게 재전달할 수 있다.

위 조건이 모두 참일 때만 오케스트레이터가 다음 후보 작업을 조회·예약하고
새 lease를 발급한다. `Nav2 SUCCEEDED`, HTTP 202, Auto Dock `READY`는 이
조건을 충족하지 않는다.

### 자동 진행을 반드시 차단할 조건

- HTTP timeout 또는 재시작으로 명령 수락 여부가 불명인 경우
- PICK 뒤 실제 적재 여부가 Inventory와 일치하지 않는 경우
- 취소/stop 뒤 Nav2 cancel 또는 0속도 정지가 확정되지 않은 경우
- operation, assignment, attempt, dock event의 상관관계가 일치하지 않는 경우
- 차량 telemetry가 stale이거나 재연결 뒤 새 telemetry를 아직 받지 못한 경우

이 경우 상태는 `RECOVERY_REQUIRED`로 전이한다. 자동 재전송·자동 재배정은
금지하며, 차량 위치와 포크 적재 상태를 reconciliation한 뒤에만 PLACE 재개,
반송 또는 장부 보정을 선택한다.

## 구현 우선순위

1. **오케스트레이터 원장**: request dedupe, vehicle lease, dispatch attempt,
   inbox/outbox, `DELIVERY_UNKNOWN` 상태를 추가한다.
2. **Inventory 보상 API**: pre-pick cancel/fail 예약 해제와 post-pick
   `RECOVERY_REQUIRED`·reconciliation 전이를 구현한다.
3. **차량 Reporter와 상관관계**: Nav2/Auto Dock 이벤트에 operation, attempt,
   dock attempt, monotonic event ID, 물리 증거를 포함한다.
4. **명령 arbiter**: emergency stop > safety > assisted docking > manual > Nav2
   우선순위를 강제하고 lease epoch 이외의 구동 명령을 차단한다.

## 검토 근거와 검증 범위

- `inventory_db` 도메인 테스트 17건 통과
- `fleet_bridge` 구성/배포 계약 테스트 20건 통과
- `vehicle_communication` 테스트 34건 및 Python compile 통과
- 로컬 환경에 FastAPI/pytest가 없어 Inventory HTTP API와 Fleet Bridge API의
  일부 모듈 테스트는 실행하지 못했다. 이는 코드 실패와 구분해 컨테이너 또는
  의존성 설치 환경에서 재검증해야 한다.

주요 구현 근거:

- `operations/inventory_db/app/inventory.py`: 예약·PICK·PLACE의 SQLite transaction과
  활성 작업 상태
- `operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/fleet_bridge_worker/api.py`:
  vehicle-native relay 경계
- `operations/fleet_bridge/server/ros2_ws/src/fleet_bridge_worker/fleet_bridge_worker/command.py`:
  HTTP timeout·전송 동작
- `vehicle_communication/vehicle_command_api.py`: Nav2/Auto Dock 상태 전이
- `llm-wiki/concepts/mentorpi-m1-navigation-stack.md`: MentorPi 명령 소유권과
  Nav2 안전 제약
