# 파렛트 재고 서비스 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `inventory_db/`에서 독립 실행되는 SQLite 기반 파렛트 재고 API를 만들어 zone 재고, 차량 적재 상태, 운송 작업과 입출고 이력을 안전하게 저장·조회한다.

**Architecture:** 표준 라이브러리 `sqlite3`의 connection-per-operation 저장소가 모든 상태 전이를 짧은 `BEGIN IMMEDIATE` transaction으로 처리한다. FastAPI는 application lifespan에서 `InventoryStore` 하나만 만들고 connection을 주입하지 않으며, 별도 Docker Compose가 DB 파일과 API 프로세스를 관리한다.

**Tech Stack:** Python 3.12, FastAPI 0.115.12, Uvicorn 0.34.0, Pydantic 2, SQLite WAL, `unittest`, Docker Compose.

**Spec:** `docs/superpowers/specs/2026-08-25-pallet-inventory-service-design.md`

## Global Constraints

- 모든 서비스 파일은 `inventory_db/` 아래에 둔다. 기존 `fleet_bridge` API 파일을 수정하지 않는다.
- DB 파일은 `inventory_db/data/` bind mount에만 생성하며 Git에서 제외한다.
- 파렛트 재고는 개별 pallet ID 없이 `(zone_id, payload_type)` 집계로 관리한다.
- `EMPTY`, `FRESH`, `NORMAL`은 적재물 종류이며 파렛트 부재는 `payload_type=NULL`이다.
- HTTP 요청 간 `sqlite3.Connection`을 공유하지 않는다.
- 모든 재고 변경 SQL은 parameter binding을 사용하고 PICK/PLACE 전이를 하나의 transaction으로 처리한다.
- 테스트는 실제 임시 SQLite 파일을 사용하며 mock DB를 사용하지 않는다.

---

### Task 1: 독립 서비스 골격과 SQLite 스키마

**Files:**
- Create: `inventory_db/app/__init__.py`
- Create: `inventory_db/app/inventory.py`
- Create: `inventory_db/tests/test_inventory.py`
- Create: `inventory_db/.gitignore`

**Interfaces:**
- Produces: `PayloadType`, `OperationStatus`, `Zone`, `Stock`, `RobotPalletState`, `InventoryStore`.
- `InventoryStore(database_path: str | Path)`, `initialize() -> None`, `upsert_zone(zone: Zone) -> Zone`, `set_stock(zone_id: str, payload_type: PayloadType, quantity: int) -> Stock`, `list_stocks() -> list[Stock]`.

- [x] **Step 1: Write the failing schema and stock test**

```python
def test_initializes_seeded_payload_types_and_zone_stock():
    store = InventoryStore(self.database_path)
    store.initialize()
    store.upsert_zone(Zone('fresh_zone', 'Fresh', 'map', 1.0, 2.0, 0.0, 8, True))

    stock = store.set_stock('fresh_zone', PayloadType.FRESH, 3)

    self.assertEqual(stock.quantity, 3)
    self.assertEqual(stock.reserved_quantity, 0)
    self.assertEqual(stock.available_quantity, 3)
```

- [x] **Step 2: Run the test to verify RED**

Run: `python3 -m unittest inventory_db.tests.test_inventory.InventoryStoreTest.test_initializes_seeded_payload_types_and_zone_stock -v`

Expected: FAIL because `inventory_db.app.inventory` does not exist.

- [x] **Step 3: Implement minimal SQLite schema and read/write models**

```python
class InventoryStore:
    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self._database_path, timeout=5, isolation_level=None)
        connection.row_factory = sqlite3.Row
        connection.execute('PRAGMA foreign_keys = ON')
        connection.execute('PRAGMA busy_timeout = 5000')
        connection.execute('PRAGMA synchronous = FULL')
        return connection
```

Create all six specified tables, seed the three payload types with `INSERT OR IGNORE`, and use a composite primary key for `pallet_stocks`.

- [x] **Step 4: Run the focused test to verify GREEN**

Run: `python3 -m unittest inventory_db.tests.test_inventory.InventoryStoreTest.test_initializes_seeded_payload_types_and_zone_stock -v`

Expected: PASS.

### Task 2: 작업 예약과 차량 상태·다음 지시

**Files:**
- Modify: `inventory_db/app/inventory.py`
- Modify: `inventory_db/tests/test_inventory.py`

**Interfaces:**
- Consumes: `InventoryStore`, `PayloadType`, `Zone`, `Stock` from Task 1.
- Produces: `Operation`, `NextInstruction`, `InsufficientStockError`, `ConflictError`.
- `create_operation(operation_id, payload_type, source_zone_id, destination_zone_id, priority=0) -> Operation`.
- `assign_operation(operation_id, robot_id) -> Operation`.
- `get_robot_pallet_state(robot_id) -> RobotPalletState`.
- `next_instruction(robot_id) -> NextInstruction | None`.

- [x] **Step 1: Write the failing reservation and PICK instruction test**

```python
def test_assigned_operation_reserves_one_stock_and_directs_empty_robot_to_pick(self):
    operation = self.store.create_operation('op-1', PayloadType.FRESH, 'source', 'destination')
    assigned = self.store.assign_operation(operation.operation_id, 'robot_1')

    self.assertEqual(assigned.status, OperationStatus.TO_PICK)
    self.assertEqual(self.store.list_stocks()[0].reserved_quantity, 1)
    self.assertEqual(self.store.next_instruction('robot_1').action, 'PICK')
    self.assertEqual(self.store.next_instruction('robot_1').zone_id, 'source')
```

- [x] **Step 2: Run the test to verify RED**

Run: `python3 -m unittest inventory_db.tests.test_inventory.InventoryStoreTest.test_assigned_operation_reserves_one_stock_and_directs_empty_robot_to_pick -v`

Expected: FAIL because `create_operation` is not implemented.

- [x] **Step 3: Implement reservation, assignment, and next-instruction reads**

```python
connection.execute('BEGIN IMMEDIATE')
updated = connection.execute(
    'UPDATE pallet_stocks SET reserved_quantity = reserved_quantity + 1, version = version + 1 '
    'WHERE zone_id = ? AND payload_type = ? AND quantity > reserved_quantity',
    (source_zone_id, payload_type.value),
)
if updated.rowcount != 1:
    raise InsufficientStockError(source_zone_id, payload_type)
```

Reject an assignment if the robot has an active nonterminal operation or already carries a pallet.

- [x] **Step 4: Run the focused test to verify GREEN**

Run: `python3 -m unittest inventory_db.tests.test_inventory.InventoryStoreTest.test_assigned_operation_reserves_one_stock_and_directs_empty_robot_to_pick -v`

Expected: PASS.

### Task 3: 멱등 PICK/PLACE 전이와 이력

**Files:**
- Modify: `inventory_db/app/inventory.py`
- Modify: `inventory_db/tests/test_inventory.py`

**Interfaces:**
- Consumes: operation assignment and stock reservation from Task 2.
- Produces: `InventoryEvent`, `complete_pick(operation_id, robot_id, idempotency_key, occurred_at=None) -> InventoryEvent`, `complete_place(...) -> InventoryEvent`.

- [x] **Step 1: Write the failing atomic PICK and duplicate notification test**

```python
def test_pick_moves_reserved_stock_to_robot_once_when_notification_is_retried(self):
    self._assign_fresh_operation()
    event = self.store.complete_pick('op-1', 'robot_1', 'pick-op-1')
    duplicate = self.store.complete_pick('op-1', 'robot_1', 'pick-op-1')

    self.assertEqual(event.event_id, duplicate.event_id)
    self.assertEqual(self.store.stock('source', PayloadType.FRESH).quantity, 2)
    self.assertEqual(self.store.stock('source', PayloadType.FRESH).reserved_quantity, 0)
    self.assertTrue(self.store.get_robot_pallet_state('robot_1').has_pallet)
    self.assertEqual(self.store.next_instruction('robot_1').action, 'PLACE')
```

- [x] **Step 2: Run the test to verify RED**

Run: `python3 -m unittest inventory_db.tests.test_inventory.InventoryStoreTest.test_pick_moves_reserved_stock_to_robot_once_when_notification_is_retried -v`

Expected: FAIL because `complete_pick` is not implemented.

- [x] **Step 3: Implement one transaction per completion event**

```python
connection.execute('BEGIN IMMEDIATE')
existing = connection.execute(
    'SELECT * FROM inventory_events WHERE idempotency_key = ?',
    (idempotency_key,),
).fetchone()
if existing is not None:
    connection.execute('COMMIT')
    return self._event_from_row(existing)
```

For a new PICK event, update source stock, upsert robot pallet state, advance the operation, then insert the event before commit. For a new PLACE event, check destination capacity, add destination stock, clear robot state, complete the operation, insert the event, then commit. Roll back every exception.

- [x] **Step 4: Write and run the PLACE completion test**

```python
def test_place_returns_robot_to_empty_state_and_increases_destination_stock(self):
    self._picked_fresh_operation()
    self.store.complete_place('op-1', 'robot_1', 'place-op-1')

    self.assertEqual(self.store.stock('destination', PayloadType.FRESH).quantity, 1)
    self.assertFalse(self.store.get_robot_pallet_state('robot_1').has_pallet)
    self.assertEqual(self.store.operation('op-1').status, OperationStatus.COMPLETED)
```

Run: `python3 -m unittest inventory_db.tests.test_inventory -v`

Expected: PASS.

### Task 4: FastAPI 경계와 Docker Compose 배포

**Files:**
- Create: `inventory_db/app/main.py`
- Create: `inventory_db/requirements.txt`
- Create: `inventory_db/Dockerfile`
- Create: `inventory_db/docker-compose.yaml`
- Create: `inventory_db/.env.example`
- Create: `inventory_db/README.md`
- Create: `inventory_db/tests/test_api.py`
- Create: `inventory_db/tests/test_compose_contract.py`

**Interfaces:**
- Consumes: `InventoryStore` public API from Tasks 1-3.
- Produces: `app.main:create_app(database_path: str) -> FastAPI` and the documented HTTP paths.

- [x] **Step 1: Write the failing API tests for health, stock, and duplicate PICK completion**

```python
def test_pick_completion_endpoint_returns_same_event_for_same_idempotency_key(self):
    response = self.client.post('/api/v1/operations/op-1/pick-completions', json={
        'robot_id': 'robot_1', 'idempotency_key': 'pick-op-1',
    })
    repeated = self.client.post('/api/v1/operations/op-1/pick-completions', json={
        'robot_id': 'robot_1', 'idempotency_key': 'pick-op-1',
    })

    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.json()['event_id'], repeated.json()['event_id'])
```

- [x] **Step 2: Run the API test to verify RED**

Run: `inventory_db/.venv/bin/python -m unittest inventory_db.tests.test_api.InventoryApiTest.test_pick_completion_endpoint_returns_same_event_for_same_idempotency_key -v`

Expected: FAIL because `app.main` does not exist.

- [x] **Step 3: Implement FastAPI app and HTTP error translation**

```python
@asynccontextmanager
async def lifespan(app: FastAPI):
    store = InventoryStore(app.state.database_path)
    store.initialize()
    app.state.store = store
    yield
```

Create synchronous route functions so FastAPI runs blocking SQLite calls outside the event loop. Convert not-found errors to 404, domain conflicts to 409, and schema validation errors to 422.

- [x] **Step 4: Add container files and Compose contract test**

```yaml
services:
  inventory-api:
    build: .
    environment:
      INVENTORY_DB_PATH: ${INVENTORY_DB_PATH:-/data/inventory.db}
    ports:
      - "${INVENTORY_API_HOST:-127.0.0.1}:${INVENTORY_API_PORT:-8081}:8080"
    volumes:
      - ./data:/data
```

The Dockerfile must install only `fastapi==0.115.12` and `uvicorn==0.34.0`, copy `app/`, and execute `uvicorn app.main:app --host 0.0.0.0 --port 8080`.

- [x] **Step 5: Run API and Compose tests to verify GREEN**

Run: `inventory_db/.venv/bin/python -m unittest inventory_db.tests.test_api inventory_db.tests.test_compose_contract -v`

Expected: PASS.

### Task 5: 전체 검증

**Files:**
- Modify: `inventory_db/README.md`

**Interfaces:**
- Consumes: all Tasks 1-4.
- Produces: documented local startup, health check, and test commands.

- [x] **Step 1: Add exact operator commands**

```bash
cd inventory_db
cp .env.example .env
docker compose up --build -d
curl http://127.0.0.1:8081/healthz
```

- [x] **Step 2: Run the complete service test suite**

Run: `inventory_db/.venv/bin/python -m unittest discover -s inventory_db/tests -p 'test_*.py' -v`

Expected: PASS.

- [x] **Step 3: Build and start the production Compose service**

Run: `docker compose --env-file inventory_db/.env.example -f inventory_db/docker-compose.yaml up --build -d`

Run: `curl --fail http://127.0.0.1:8081/healthz`

Expected: JSON response with `{"status":"ok"}`.

- [x] **Step 4: Stop the verification container without deleting persisted data**

Run: `docker compose --env-file inventory_db/.env.example -f inventory_db/docker-compose.yaml down`

Expected: service stops and `inventory_db/data/` remains intact.
