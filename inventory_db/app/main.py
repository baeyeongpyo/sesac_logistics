from __future__ import annotations

from contextlib import asynccontextmanager
from dataclasses import asdict
import os
from typing import Annotated

from fastapi import FastAPI, Path, Request, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from .inventory import (
    ConflictError,
    InventoryEvent,
    InventoryStore,
    NotFoundError,
    Operation,
    PayloadType,
    RobotPalletState,
    Stock,
    Zone,
)


API_DESCRIPTION = """
파렛트 ID 없이 `(zone_id, payload_type)`별 집계 재고를 관리하는 중앙 서비스입니다.

## 기본 흐름

1. **zones**에서 Nav2 주행 목적지와 적치 용량을 설정합니다.
2. **stocks**에서 초기 재고 또는 실사 재고를 절대 수량으로 설정합니다.
3. **operations**에서 차량을 지정해 source 파렛트와 destination 적치 슬롯을 함께 예약하고 즉시 작업에 배정합니다.
4. 외부 비전·포크 노드가 PICK/PLACE 성공을 확인하면 완료 API를 호출해 실제 재고와 차량 적재 상태를 변경합니다.

## 작업 상태

- `TO_PICK`: 차량이 source zone으로 주행해 PICK해야 하는 상태입니다.
- `TO_PLACE`: PICK 완료로 차량에 파렛트가 적재되어 destination zone으로 주행해야 하는 상태입니다.
- `COMPLETED`: PLACE가 반영되어 작업이 끝난 상태입니다.
"""


OPENAPI_TAGS = [
    {"name": "health", "description": "서비스 상태를 확인합니다."},
    {
        "name": "zones",
        "description": "주행 목적지, 지도 좌표, 적치 용량을 관리합니다. 재고 수량은 변경하지 않습니다.",
    },
    {
        "name": "stocks",
        "description": "zone별 현재 재고의 절대 수량을 초기화하거나 실사값으로 보정합니다.",
    },
    {
        "name": "operations",
        "description": "차량에 즉시 배정하는 운송 예약과 실제 PICK/PLACE 완료를 기록합니다.",
    },
    {
        "name": "robots",
        "description": "차량의 포크 적재 상태와 다음 주행 목적지를 조회합니다.",
    },
]


class RequestModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class ZoneRequest(RequestModel):
    name: str = Field(min_length=1, description="운영 화면에 표시할 zone 이름")
    map_name: str = Field(min_length=1, description="Nav2 지도 이름")
    nav_x: float = Field(description="zone 접근 목표의 지도 X 좌표(m)")
    nav_y: float = Field(description="zone 접근 목표의 지도 Y 좌표(m)")
    nav_yaw: float = Field(description="zone 접근 목표의 yaw(rad)")
    capacity: int | None = Field(
        default=None,
        ge=0,
        description="모든 payload_type을 합친 최대 파렛트 수량. null은 제한 없음",
    )
    enabled: bool = Field(default=True, description="운영 가능 여부")


class StockRequest(RequestModel):
    quantity: int = Field(
        ge=0,
        description="증감값이 아닌 이 zone/payload_type의 현재 총 재고 수량",
    )


class OperationRequest(RequestModel):
    robot_id: str = Field(
        min_length=1,
        description="작업 생성과 동시에 배정할 차량 식별자입니다.",
    )
    payload_type: PayloadType = Field(
        description="운송할 파렛트 적재물 유형입니다."
    )
    source_zone_id: str = Field(
        min_length=1, description="파렛트를 픽업할 출발 zone ID입니다."
    )
    destination_zone_id: str = Field(
        min_length=1, description="파렛트를 place할 목적지 zone ID입니다."
    )
    priority: int = Field(default=0, description="값이 클수록 먼저 처리하는 우선순위입니다.")


class CompletionRequest(RequestModel):
    robot_id: str = Field(min_length=1, description="PICK 또는 PLACE를 완료한 차량 식별자")
    idempotency_key: str = Field(
        min_length=1, description="재전송해도 같은 완료를 한 번만 반영하는 고유 키"
    )
    occurred_at: str | None = Field(
        default=None, description="실제 완료 시각(UTC ISO-8601). 생략하면 서버 수신 시각 사용"
    )


def _response(value: Zone | Stock | Operation | InventoryEvent | RobotPalletState) -> dict:
    response = asdict(value)
    if isinstance(value, Stock):
        response["available_quantity"] = value.available_quantity
    return response


def _store(request: Request) -> InventoryStore:
    return request.app.state.store


def _endpoint_description(process: str, result: str, note: str | None = None) -> str:
    description = f"**처리:** {process}\n\n**결과:** {result}"
    if note:
        description += f"\n\n**주의:** {note}"
    return description


def create_app(database_path: str) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        store = InventoryStore(database_path)
        store.initialize()
        app.state.store = store
        yield

    app = FastAPI(
        title="Pallet Inventory API",
        version="1.0.0",
        description=API_DESCRIPTION,
        openapi_tags=OPENAPI_TAGS,
        lifespan=lifespan,
    )

    @app.exception_handler(ConflictError)
    def conflict_error_handler(_: Request, error: ConflictError):
        return JSONResponse(
            status_code=status.HTTP_409_CONFLICT, content={"detail": str(error)}
        )

    @app.exception_handler(NotFoundError)
    def not_found_error_handler(_: Request, error: NotFoundError):
        return JSONResponse(
            status_code=status.HTTP_404_NOT_FOUND, content={"detail": str(error)}
        )

    @app.get(
        "/healthz",
        tags=["health"],
        summary="서비스 상태 확인",
        description=_endpoint_description(
            "HTTP API가 요청을 처리할 수 있는지 확인합니다.",
            '`{"status":"ok"}`를 반환합니다.',
            "SQLite 초기화 실패 시 서비스가 시작되지 않으므로 이 엔드포인트도 제공되지 않습니다.",
        ),
    )
    def healthz() -> dict[str, str]:
        return {"status": "ok"}

    @app.put(
        "/api/v1/zones/{zone_id}",
        tags=["zones"],
        summary="Zone 설정 또는 수정",
        description=_endpoint_description(
            "주행 목적지의 지도 좌표, 적치 용량, 운영 가능 여부를 생성하거나 같은 zone_id의 설정을 수정합니다.",
            "저장된 zone 설정을 반환하며 재고 수량은 변경하지 않습니다.",
            "이미 적치된 총 재고와 inbound 예약보다 capacity를 작게 설정하면 409 오류가 반환됩니다.",
        ),
    )
    def upsert_zone(
        zone_id: Annotated[
            str, Path(description="생성하거나 수정할 주행 목적지 식별자")
        ],
        body: ZoneRequest,
        request: Request,
    ) -> dict:
        zone = _store(request).upsert_zone(
            Zone(
                zone_id,
                body.name,
                body.map_name,
                body.nav_x,
                body.nav_y,
                body.nav_yaw,
                body.capacity,
                body.enabled,
            )
        )
        return _response(zone)

    @app.get(
        "/api/v1/zones",
        tags=["zones"],
        summary="Zone 목록 조회",
        description=_endpoint_description(
            "등록된 모든 zone의 지도 좌표, 적치 용량, 운영 가능 여부를 조회합니다.",
            "zone_id 오름차순의 zone 목록을 반환합니다.",
        ),
    )
    def list_zones(request: Request) -> list[dict]:
        return [_response(zone) for zone in _store(request).list_zones()]

    @app.put(
        "/api/v1/stocks/{zone_id}/{payload_type}",
        tags=["stocks"],
        summary="현재 재고 수량 설정",
        description=_endpoint_description(
            "한 zone과 payload_type 조합의 현재 재고를 초기 입력 또는 실사값으로 설정합니다. quantity는 증감값이 아닌 절대 수량입니다.",
            "저장된 수량, 예약 수량, 즉시 운송 가능한 수량을 반환합니다.",
            "출고 전용 API가 아닙니다. source 예약 또는 destination inbound 예약과 충돌하거나 zone 용량을 넘으면 409 오류입니다.",
        ),
    )
    def set_stock(
        zone_id: Annotated[
            str, Path(description="재고를 설정할 zone 식별자")
        ],
        payload_type: Annotated[
            PayloadType, Path(description="설정할 파렛트 적재물 종류")
        ],
        body: StockRequest,
        request: Request,
    ) -> dict:
        return _response(_store(request).set_stock(zone_id, payload_type, body.quantity))

    @app.get(
        "/api/v1/stocks",
        tags=["stocks"],
        summary="재고 목록 조회",
        description=_endpoint_description(
            "모든 zone과 payload_type 조합의 현재 재고를 조회합니다.",
            "각 항목의 quantity, reserved_quantity, available_quantity을 반환합니다.",
        ),
    )
    def list_stocks(request: Request) -> list[dict]:
        return [_response(stock) for stock in _store(request).list_stocks()]

    @app.post(
        "/api/v1/operations",
        status_code=status.HTTP_201_CREATED,
        tags=["operations"],
        summary="파렛트 운송 작업 생성 및 양쪽 zone 예약",
        description=_endpoint_description(
            "robot_id가 지정된 비적재·비작업 차량, source 파렛트 1개, destination 적치 슬롯 1개를 하나의 transaction에서 함께 확인·예약하고 UUID operation_id를 생성합니다.",
            "robot_id가 저장된 TO_PICK 작업을 201으로 반환합니다.",
            "차량에 활성 작업 또는 적재 파렛트가 있거나 source 재고 부족, destination의 현재 적치·inbound 예약을 합친 용량 초과이면 409 오류입니다.",
        ),
    )
    def create_operation(body: OperationRequest, request: Request) -> dict:
        operation = _store(request).create_operation(
            body.robot_id,
            body.payload_type,
            body.source_zone_id,
            body.destination_zone_id,
            body.priority,
        )
        return _response(operation)

    @app.get(
        "/api/v1/operations/active",
        tags=["operations"],
        summary="진행 중인 운송 작업 조회",
        description=_endpoint_description(
            "재고를 예약했거나 차량 PICK/PLACE가 아직 완료되지 않은 운송 작업을 조회합니다.",
            "TO_PICK, PICKING, TO_PLACE, PLACING, RECOVERY_REQUIRED 상태의 작업을 우선순위 내림차순으로 반환합니다.",
            "COMPLETED, FAILED, CANCELLED 상태의 종료 작업은 포함하지 않습니다.",
        ),
    )
    def list_active_operations(request: Request) -> list[dict]:
        return [
            _response(operation)
            for operation in _store(request).list_active_operations()
        ]

    @app.post(
        "/api/v1/operations/{operation_id}/pick-completions",
        tags=["operations"],
        summary="PICK 완료 반영",
        description=_endpoint_description(
            "외부 비전·포크 노드가 PICK 성공을 확인한 뒤 완료 이벤트를 기록합니다.",
            "source zone 재고와 예약 수량을 각각 1 감소시키고, 차량을 적재 상태 및 작업을 TO_PLACE로 전이합니다.",
            "같은 idempotency_key 재전송은 같은 이벤트를 반환합니다. 작업 소유 차량이 아니거나 TO_PICK 상태가 아니면 409 오류입니다.",
        ),
    )
    def complete_pick(
        operation_id: Annotated[
            str, Path(description="PICK 완료를 기록할 운송 작업의 UUID")
        ],
        body: CompletionRequest,
        request: Request,
    ) -> dict:
        event = _store(request).complete_pick(
            operation_id, body.robot_id, body.idempotency_key, body.occurred_at
        )
        return _response(event)

    @app.post(
        "/api/v1/operations/{operation_id}/place-completions",
        tags=["operations"],
        summary="PLACE 완료 반영",
        description=_endpoint_description(
            "외부 비전·포크 노드가 PLACE 성공을 확인한 뒤 완료 이벤트를 기록합니다.",
            "destination zone 재고를 1 증가시키고 차량을 비적재 상태 및 작업을 COMPLETED로 전이합니다.",
            "같은 idempotency_key 재전송은 같은 이벤트를 반환합니다. 차량 적재 상태 불일치 또는 TO_PLACE 이외 상태는 409 오류입니다.",
        ),
    )
    def complete_place(
        operation_id: Annotated[
            str, Path(description="PLACE 완료를 기록할 운송 작업의 UUID")
        ],
        body: CompletionRequest,
        request: Request,
    ) -> dict:
        event = _store(request).complete_place(
            operation_id, body.robot_id, body.idempotency_key, body.occurred_at
        )
        return _response(event)

    @app.get(
        "/api/v1/robots/{robot_id}/next-instruction",
        tags=["robots"],
        summary="차량의 다음 주행 지시 조회",
        description=_endpoint_description(
            "차량의 포크 적재 상태와 활성 운송 작업을 기준으로 다음 주행 목적지를 계산합니다.",
            "비적재 차량에는 PICK source zone, 적재 차량에는 PLACE destination zone의 지도 좌표와 payload_type을 반환합니다.",
            "진행 중인 작업이 없으면 null을 반환합니다. 이 API는 Nav2 주행을 직접 시작하지 않습니다.",
        ),
    )
    def next_instruction(
        robot_id: Annotated[
            str, Path(description="다음 주행 지시를 조회할 차량 식별자")
        ],
        request: Request,
    ) -> dict | None:
        instruction = _store(request).next_instruction(robot_id)
        return asdict(instruction) if instruction is not None else None

    @app.get(
        "/api/v1/robots/{robot_id}/pallet-state",
        tags=["robots"],
        summary="차량 파렛트 적재 상태 조회",
        description=_endpoint_description(
            "차량 포크에 파렛트가 적재되어 있는지와 적재물 종류를 조회합니다.",
            "has_pallet, payload_type, 마지막 보고 시각을 반환합니다. 상태가 없던 차량은 비적재 상태로 반환합니다.",
            "포크의 상승·하강 높이 상태는 이 서비스에서 관리하지 않습니다.",
        ),
    )
    def robot_pallet_state(
        robot_id: Annotated[
            str, Path(description="파렛트 적재 상태를 조회할 차량 식별자")
        ],
        request: Request,
    ) -> dict:
        return _response(_store(request).get_robot_pallet_state(robot_id))

    return app


app = create_app(os.environ.get("INVENTORY_DB_PATH", "/data/inventory.db"))
