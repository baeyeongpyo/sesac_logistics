from pathlib import Path
import sqlite3
import tempfile
import unittest

from inventory_db.app.inventory import (
    ConflictError,
    InventoryStore,
    InsufficientStockError,
    OperationStatus,
    PayloadType,
    Zone,
)


class InventoryStoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.database_path = Path(self.temporary_directory.name) / "inventory.db"

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def test_initializes_seeded_payload_types_and_zone_stock(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(
            Zone(
                "fresh_zone",
                "Fresh",
                "warehouse_map",
                1.0,
                2.0,
                0.0,
                8,
                True,
            )
        )

        stock = store.set_stock("fresh_zone", PayloadType.FRESH, 3)

        self.assertEqual(stock.quantity, 3)
        self.assertEqual(stock.reserved_quantity, 0)
        self.assertEqual(stock.available_quantity, 3)

    def test_assigned_operation_reserves_one_stock_and_directs_empty_robot_to_pick(
        self,
    ) -> None:
        store, assigned = self._assigned_fresh_operation()

        self.assertEqual(assigned.status, OperationStatus.TO_PICK)
        self.assertEqual(store.list_stocks()[0].reserved_quantity, 1)
        instruction = store.next_instruction("robot_1")
        self.assertIsNotNone(instruction)
        self.assertEqual(instruction.action, "PICK")
        self.assertEqual(instruction.zone_id, "source")

    def test_pick_moves_reserved_stock_to_robot_once_when_notification_is_retried(
        self,
    ) -> None:
        store, assigned = self._assigned_fresh_operation()

        event = store.complete_pick(assigned.operation_id, "robot_1", "pick-op-1")
        duplicate = store.complete_pick(
            assigned.operation_id, "robot_1", "pick-op-1"
        )

        self.assertEqual(event.event_id, duplicate.event_id)
        self.assertEqual(store.stock("source", PayloadType.FRESH).quantity, 2)
        self.assertEqual(store.stock("source", PayloadType.FRESH).reserved_quantity, 0)
        self.assertTrue(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(store.next_instruction("robot_1").action, "PLACE")

    def test_place_returns_robot_to_empty_state_and_increases_destination_stock(
        self,
    ) -> None:
        store, operation_id = self._picked_fresh_operation()

        store.complete_place(operation_id, "robot_1", "place-op-1")

        self.assertEqual(store.stock("destination", PayloadType.FRESH).quantity, 1)
        self.assertFalse(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(store.operation(operation_id).status, OperationStatus.COMPLETED)

    def test_second_operation_cannot_reserve_the_same_last_pallet(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(Zone("dest-a", "A", "map", 1.0, 0.0, 0.0, 8, True))
        store.upsert_zone(Zone("dest-b", "B", "map", 2.0, 0.0, 0.0, 8, True))
        store.set_stock("source", PayloadType.EMPTY, 1)

        store.create_operation("robot_a", PayloadType.EMPTY, "source", "dest-a")
        with self.assertRaises(InsufficientStockError):
            store.create_operation("robot_b", PayloadType.EMPTY, "source", "dest-b")

        stock = store.stock("source", PayloadType.EMPTY)
        self.assertEqual(stock.quantity, 1)
        self.assertEqual(stock.reserved_quantity, 1)

    def test_destination_capacity_reserves_a_slot_when_an_operation_is_created(
        self,
    ) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 1, True)
        )
        store.set_stock("source", PayloadType.FRESH, 2)

        store.create_operation("robot_a", PayloadType.FRESH, "source", "destination")

        with self.assertRaisesRegex(ConflictError, "destination zone capacity is full"):
            store.create_operation(
                "robot_b", PayloadType.FRESH, "source", "destination"
            )

        self.assertEqual(store.stock("source", PayloadType.FRESH).reserved_quantity, 1)

    def test_stock_update_cannot_consume_a_destination_slot_reserved_by_an_operation(
        self,
    ) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 1, True)
        )
        store.set_stock("source", PayloadType.FRESH, 1)
        store.create_operation("robot_a", PayloadType.FRESH, "source", "destination")

        with self.assertRaisesRegex(ConflictError, "zone capacity would be exceeded"):
            store.set_stock("destination", PayloadType.FRESH, 1)

    def test_zone_capacity_cannot_be_reduced_below_inbound_reservations(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, None, True)
        )
        store.set_stock("source", PayloadType.FRESH, 1)
        store.create_operation("robot_a", PayloadType.FRESH, "source", "destination")

        with self.assertRaisesRegex(
            ConflictError, "current stock and inbound reservations"
        ):
            store.upsert_zone(
                Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 0, True)
            )

    def test_destination_slot_remains_reserved_after_pick(
        self,
    ) -> None:
        store, assigned = self._assigned_fresh_operation()
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 1, True)
        )

        with self.assertRaises(ConflictError):
            store.set_stock("destination", PayloadType.NORMAL, 1)

        store.complete_pick(assigned.operation_id, "robot_1", "pick-op-1")

        with self.assertRaises(ConflictError):
            store.set_stock("destination", PayloadType.NORMAL, 1)

        self.assertTrue(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(
            store.operation(assigned.operation_id).status, OperationStatus.TO_PLACE
        )

    def test_stock_cannot_be_lowered_below_reserved_quantity(self) -> None:
        store, _ = self._assigned_fresh_operation()

        with self.assertRaises(ConflictError):
            store.set_stock("source", PayloadType.FRESH, 0)

        self.assertEqual(store.stock("source", PayloadType.FRESH).reserved_quantity, 1)

    def test_stock_write_respects_zone_capacity_across_payload_types(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("small", "Small", "map", 0.0, 0.0, 0.0, 1, True))
        store.set_stock("small", PayloadType.FRESH, 1)

        with self.assertRaises(ConflictError):
            store.set_stock("small", PayloadType.EMPTY, 1)

        self.assertEqual(store.stock("small", PayloadType.FRESH).quantity, 1)

    def test_zone_id_is_bound_as_data_not_executed_as_sql(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        zone_id = "source'; DROP TABLE pallet_stocks; --"
        store.upsert_zone(Zone(zone_id, "Source", "map", 0.0, 0.0, 0.0, 8, True))

        stock = store.set_stock(zone_id, PayloadType.EMPTY, 1)

        self.assertEqual(stock.zone_id, zone_id)
        self.assertEqual(store.stock(zone_id, PayloadType.EMPTY).quantity, 1)

    def test_zone_capacity_cannot_be_reduced_below_current_stock(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 2, True))
        store.set_stock("source", PayloadType.FRESH, 2)

        with self.assertRaises(ConflictError):
            store.upsert_zone(
                Zone("source", "Source", "map", 0.0, 0.0, 0.0, 1, True)
            )

        self.assertEqual(store.list_zones()[0].capacity, 2)

    def test_initialize_removes_legacy_queued_operations_and_releases_stock(self) -> None:
        timestamp = "2026-08-27T00:00:00Z"
        connection = sqlite3.connect(self.database_path)
        try:
            connection.executescript(
                """
                CREATE TABLE zones (
                    zone_id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    map_name TEXT NOT NULL,
                    nav_x REAL NOT NULL,
                    nav_y REAL NOT NULL,
                    nav_yaw REAL NOT NULL,
                    capacity INTEGER,
                    enabled INTEGER NOT NULL,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                );
                CREATE TABLE payload_types (
                    payload_type TEXT PRIMARY KEY,
                    enabled INTEGER NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE TABLE pallet_stocks (
                    zone_id TEXT NOT NULL,
                    payload_type TEXT NOT NULL,
                    quantity INTEGER NOT NULL,
                    reserved_quantity INTEGER NOT NULL,
                    version INTEGER NOT NULL,
                    updated_at TEXT NOT NULL,
                    PRIMARY KEY (zone_id, payload_type)
                );
                CREATE TABLE transport_operations (
                    operation_id TEXT PRIMARY KEY,
                    robot_id TEXT,
                    payload_type TEXT NOT NULL,
                    source_zone_id TEXT NOT NULL,
                    destination_zone_id TEXT NOT NULL,
                    status TEXT NOT NULL CHECK (status IN (
                        'QUEUED', 'TO_PICK', 'PICKING', 'TO_PLACE', 'PLACING',
                        'COMPLETED', 'FAILED', 'RECOVERY_REQUIRED', 'CANCELLED'
                    )),
                    priority INTEGER NOT NULL DEFAULT 0,
                    failure_code TEXT,
                    version INTEGER NOT NULL DEFAULT 1,
                    created_at TEXT NOT NULL,
                    updated_at TEXT NOT NULL,
                    completed_at TEXT
                );
                """
            )
            connection.executemany(
                "INSERT INTO zones VALUES (?, ?, 'map', 0, 0, 0, 8, 1, ?, ?)",
                [
                    ("source", "Source", timestamp, timestamp),
                    ("destination", "Destination", timestamp, timestamp),
                ],
            )
            connection.execute(
                "INSERT INTO payload_types VALUES ('FRESH', 1, ?)", (timestamp,)
            )
            connection.execute(
                "INSERT INTO pallet_stocks VALUES ('source', 'FRESH', 3, 2, 1, ?)",
                (timestamp,),
            )
            connection.executemany(
                """
                INSERT INTO transport_operations (
                    operation_id, robot_id, payload_type, source_zone_id,
                    destination_zone_id, status, priority, failure_code, version,
                    created_at, updated_at, completed_at
                ) VALUES (?, ?, 'FRESH', 'source', 'destination', ?, 0, NULL, 1, ?, ?, NULL)
                """,
                [
                    ("queued-operation", None, "QUEUED", timestamp, timestamp),
                    ("active-operation", "robot_1", "TO_PICK", timestamp, timestamp),
                ],
            )
            connection.commit()
        finally:
            connection.close()

        store = InventoryStore(self.database_path)
        store.initialize()

        self.assertEqual(
            [operation.operation_id for operation in store.list_active_operations()],
            ["active-operation"],
        )
        self.assertEqual(store.stock("source", PayloadType.FRESH).reserved_quantity, 1)
        connection = sqlite3.connect(self.database_path)
        try:
            table_sql = connection.execute(
                "SELECT sql FROM sqlite_master WHERE type = 'table' AND name = 'transport_operations'"
            ).fetchone()[0]
        finally:
            connection.close()
        self.assertNotIn("'QUEUED'", table_sql)

    def _assigned_fresh_operation(self):
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 8, True)
        )
        store.set_stock("source", PayloadType.FRESH, 3)
        operation = store.create_operation(
            "robot_1", PayloadType.FRESH, "source", "destination"
        )
        return store, operation

    def _picked_fresh_operation(self) -> tuple[InventoryStore, str]:
        store, assigned = self._assigned_fresh_operation()
        store.complete_pick(assigned.operation_id, "robot_1", "pick-op-1")
        return store, assigned.operation_id


if __name__ == "__main__":
    unittest.main()
