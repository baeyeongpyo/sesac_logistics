# Pallet 3 운영 작업 통합 설계

작성일: 2026-09-04

## 목적

Pallet 3 시연 경로를 별도 POC 미션이 아니라 Inventory의 일반 운송 작업으로
실행한다. 모든 새 작업은 하나의 `operation_id`로 재고 예약, 차량 명령, 포크
완료, 복구, 운영 화면 표시를 연결한다.

`bypass_pick`은 Auto Dock을 단순히 끄는 기능이 아니다. 작업자가 물류가 이미
차량 포크에 수동 적재되었음을 확인한 경우에만 PICK 완료를 Inventory에
기록하고 Pallet 3 경로부터 시작하는 명시적 운영 확인이다.

## 소유권과 데이터 모델

| 데이터 | 소유 서비스 | 역할 |
| --- | --- | --- |
| `transport_operations` | Inventory | `docker -> p3` 재고 예약, PICK/PLACE 완료, 작업 상태 |
| `robot_pallet_states` | Inventory | 차량의 적재 여부와 payload type |
| `pallet3_operation_workflows` | Orchestrator | `operation_id`에 연결된 Pallet 3 레시피 단계와 수동 PICK 확인 |
| `orchestrator_steps`, `command_outbox` | Orchestrator | 화면용 단계와 차량 명령 멱등성 |

`pallet3_operation_workflows`의 기본 키는 `operation_id`다. 별도 `mission_id`를
새로 만들지 않으므로, 화면과 재고 이력에서 같은 작업을 직접 추적할 수 있다.
새 Pallet 3 작업은 기존 `pallet3_poc_missions` 테이블에 행을 만들지 않는다.

레시피 적용 대상은 source zone이 `docker`이고 destination zone이 `p3`인
활성 작업이다. 이 규칙은 기존 Inventory zone ID를 변경하지 않는다. PICK Auto
Dock 명령에서만 `docker`를 물리 marker ID `DOCK_1`로 매핑한다.

## 시작과 bypass 계약

일반 작업은 기존 Inventory `POST /api/v1/operations`로 만든다. 생성 즉시
`TO_PICK` 상태가 되고 docker 재고와 p3 적치 슬롯을 예약하며, 운영 화면의
일반 작업 목록에 표시된다. 작업 생성 시 새 UUID `operation_id`가 발급된다.

새 Orchestrator API는 수동 PICK 확인을 해당 작업에 연결한다.

```text
POST /api/v1/operations/{operation_id}/pallet-3/bypass-pick
{
  "operator_confirmed": true
}
```

이 API는 다음 조건을 모두 확인한다.

- 작업의 source/destination이 각각 `docker`와 `p3`다.
- 작업 소유 차량이 존재하고 작업 상태가 `TO_PICK`이다.
- Inventory가 해당 차량을 비적재 상태로 보고한다.
- 이 작업에 `NAV_TO_PICK` 또는 `AUTO_DOCK_PICK` 명령이 이미 전송되지 않았다.
- 요청 본문의 `operator_confirmed`가 `true`다.

조건을 충족하면 Orchestrator는 workflow에 수동 PICK 확인을 영속화하고,
`{operation_id}:manual-pick` 멱등 키로 Inventory PICK 완료를 호출한다. 그
결과 docker 재고는 1 감소하고, 차량은 적재 상태가 되며, 작업은 `TO_PLACE`로
전이한다. 차량 주행 명령은 이 API에서 보내지 않는다.

같은 요청의 재전송은 같은 PICK 완료 이벤트를 사용하므로 재고를 다시 변경하지
않는다. 이미 `TO_PLACE`이고 workflow가 수동 PICK으로 기록된 경우에는 성공
응답을 재반환한다. 다른 상태, 다른 경로, 이미 일반 PICK 명령이 전송된 작업은
409로 거부한다.

`WAIT` Fleet 이벤트는 수신 즉시 reconciliation을 일으킨다. 따라서 bypass는
수동 적재 후 `operation/idle`을 호출하기 전에 설정해야 한다. `WAIT` 이후에는
DOCK_1 접근 주행이 먼저 전송될 수 있으므로 bypass API가 이를 안전한 경쟁
상태로 허용하지 않는다.

## 정상 및 수동 적재 실행 흐름

### 정상 Auto Dock PICK

```text
Inventory 작업 생성 (docker -> p3, TO_PICK, 재고 예약)
  -> 차량 operation/idle
  -> Fleet WAIT / OPERATOR_READY
  -> docker zone 접근 좌표로 Nav2 주행
  -> NAVIGATION_SUCCEEDED
  -> Auto Dock PICK(location=DOCK_1)
  -> AUTO_DOCK_PICK_COMPLETED
  -> Inventory PICK 완료 (docker -1, TO_PLACE, 차량 적재)
  -> Pallet 3 출발 waypoint
```

`DOCK_1`은 Nav2 목적지가 아니라 Auto Dock marker location이다. Nav2는
Inventory `docker` zone의 접근 pose를 사용한다.

### 수동 적재 bypass

```text
Inventory 작업 생성 (docker -> p3, TO_PICK, 재고 예약)
  -> 작업자가 실제로 차량 포크에 적재
  -> bypass-pick API의 operator_confirmed=true
  -> Inventory PICK 완료 (docker -1, TO_PLACE, 차량 적재)
  -> 차량 operation/idle
  -> Fleet WAIT / OPERATOR_READY
  -> Pallet 3 출발 waypoint
```

수동 적재를 한 뒤 bypass를 쓰지 않아도 Inventory PICK 완료가 이미 정상적으로
기록되어 `TO_PLACE`와 차량 적재 상태가 만들어졌다면 Pallet 3 레시피는 바로
재개할 수 있다. bypass API는 이 완료 기록을 안전하고 멱등적으로 만드는
운영용 단축 경로다.

### Pallet 3 적치와 복귀

```text
Pallet 3 출발 waypoint 성공
  -> Fork DOWN 명령
  -> FORK_DOWN_COMPLETE
  -> Inventory PLACE 완료 (p3 +1, 작업 COMPLETED, 차량 비적재)
  -> linear_x=-0.18, hold_ms=1000 후진
  -> 수동 주행 만료
  -> dock_1 복귀 waypoint
  -> 복귀 Nav2 성공
  -> workflow COMPLETED
```

Pallet 3 출발 waypoint의 Nav2 성공과 operation ID가 일치하면 Orchestrator는
Fork DOWN을 한 번 자동 전송한다. 작업자 하역 확인 API는 제공하지 않는다.
`FORK_DOWN_COMPLETE`와 operation ID가 모두 일치할 때만 Inventory PLACE 완료와
후진을 진행한다. 따라서 p3 재고 증가는 waypoint 도착이나 Fork DOWN 명령이
아니라 실제 Fork DOWN 완료가 유일한 근거다.

## 레시피 단계와 복구

workflow phase는 `PICK_PENDING`, `OUTBOUND_SENT`, `FORK_DOWN_SENT`,
`REVERSE_SENT`, `RETURN_SENT`, `COMPLETED`, `FAILED`를 사용한다.
`PICK_PENDING`은 일반 Auto Dock PICK을 기다리거나 수동 PICK 확인 뒤 차량의
`WAIT`을 기다리는 단계다.

Fleet가 `FAIL`이면 기존과 같이 operation recovery marker를 저장하고 새로운
차량 명령을 보내지 않는다. 운영자가 차량을 `idle`로 복구해 `WAIT`이 되면
Inventory 상태와 workflow phase를 다시 대조한다.

- `TO_PICK`이면 docker 접근 및 Auto Dock PICK 단계부터 재개한다.
- `TO_PLACE`와 `PICK_PENDING` 또는 `OUTBOUND_SENT`이면 Pallet 3 출발
  waypoint부터 재개한다.
- `FORK_DOWN_SENT`, `REVERSE_SENT`, `RETURN_SENT`에서 실패한 경우에는
  이전 명령의 실제 수행 여부가 불명확하므로 자동 재발행하지 않는다. 해당
  workflow를 `FAILED`로 남기고 stop을 요청한다.

동일 작업을 초기 상태로 되돌리거나 재사용하지 않는다. PICK 전 실패는 같은
`TO_PICK` 작업을 복구해 재시도한다. PICK 후에는 실제 물류를 p3에 적치해
`COMPLETED`로 끝낸 뒤 새 `operation_id`로 다음 반복 작업을 만든다. 이 규칙은
이미 차감된 docker 재고와 차량 적재 상태를 임의로 되돌리는 것을 막는다.

## 호환성과 표시

기존 `pallet3_poc_missions`는 이전 시연 이력과 이미 실행 중인 legacy 미션을
마무리하는 용도로만 유지한다. 새 요청은 legacy 미션을 만들지 않으며, legacy
활성 미션은 종료 전까지만 기존 처리 경로를 계속 사용한다.

Operations Control Center는 이미 Inventory의 `transport_operations`를 읽어
작업과 차량의 활성 작업을 표시한다. 새 Pallet 3 작업은 별도 화면이나 별도
식별자 없이 `docker -> p3` 일반 작업으로 보인다. Orchestrator step의 Pallet
3 phase는 같은 `operation_id`로 제공한다.

## 실패 처리

- Inventory PICK 또는 PLACE 완료 실패는 workflow를 진행시키지 않고 오류를
  저장한다.
- waypoint, Fork DOWN, 후진, 복귀 waypoint의 명령 전달 실패 또는 차량 FAIL은
  stop을 요청하고 phase를 `FAILED`로 저장한다.
- 다른 operation ID의 Nav2/Fork/수동 주행 완료 이벤트는 무시한다.
- source 재고 부족, p3 용량 부족, 차량의 다른 활성 작업 또는 적재 상태는
  Inventory가 409로 거부하며 명령을 보내지 않는다.

## 검증 기준

1. 새 `docker -> p3` 작업은 다른 일반 작업과 동일하게 대시보드와 Inventory
   active operation 목록에 나타난다.
2. 정상 PICK은 docker 접근 Nav2 성공 후에만 `DOCK_1` Auto Dock PICK을
   전송하고, PICK 완료는 docker를 정확히 한 번 차감한다.
3. 수동 적재 bypass는 `TO_PICK` 작업을 정확히 한 번 `TO_PLACE` 및 차량
   적재 상태로 바꾸고, `WAIT` 전에는 주행 명령을 보내지 않는다.
4. bypass가 설정된 작업의 `WAIT`은 DOCK_1 주행과 Auto Dock PICK 없이
   Pallet 3 출발 waypoint를 전송한다.
5. `FORK_DOWN_COMPLETE`는 p3을 정확히 한 번 증가시키고, 그 뒤에만
   `-0.18` / `1000 ms` 후진을 전송한다.
6. Fleet FAIL 뒤 `idle -> WAIT` 복구는 `TO_PICK` 또는 `TO_PLACE`의 실제
   Inventory 상태에 맞는 안전한 단계만 재개한다.
7. Pallet 3 도착 Nav2 성공은 Fork DOWN을 정확히 한 번 자동 전송하며,
   중복 Nav2/Fleet 이벤트는 재고나 차량 명령을 중복 처리하지 않는다.
8. 기존 legacy POC 미션과 일반 docker/p3 작업은 같은 차량에 동시에
   명령을 보내지 않는다.
