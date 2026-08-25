from pathlib import Path
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
        store, _ = self._assigned_fresh_operation()

        event = store.complete_pick("op-1", "robot_1", "pick-op-1")
        duplicate = store.complete_pick("op-1", "robot_1", "pick-op-1")

        self.assertEqual(event.event_id, duplicate.event_id)
        self.assertEqual(store.stock("source", PayloadType.FRESH).quantity, 2)
        self.assertEqual(store.stock("source", PayloadType.FRESH).reserved_quantity, 0)
        self.assertTrue(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(store.next_instruction("robot_1").action, "PLACE")

    def test_place_returns_robot_to_empty_state_and_increases_destination_stock(
        self,
    ) -> None:
        store = self._picked_fresh_operation()

        store.complete_place("op-1", "robot_1", "place-op-1")

        self.assertEqual(store.stock("destination", PayloadType.FRESH).quantity, 1)
        self.assertFalse(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(store.operation("op-1").status, OperationStatus.COMPLETED)

    def test_second_operation_cannot_reserve_the_same_last_pallet(self) -> None:
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(Zone("dest-a", "A", "map", 1.0, 0.0, 0.0, 8, True))
        store.upsert_zone(Zone("dest-b", "B", "map", 2.0, 0.0, 0.0, 8, True))
        store.set_stock("source", PayloadType.EMPTY, 1)

        store.create_operation("op-a", PayloadType.EMPTY, "source", "dest-a")
        with self.assertRaises(InsufficientStockError):
            store.create_operation("op-b", PayloadType.EMPTY, "source", "dest-b")

        stock = store.stock("source", PayloadType.EMPTY)
        self.assertEqual(stock.quantity, 1)
        self.assertEqual(stock.reserved_quantity, 1)

    def test_full_destination_keeps_robot_loaded_and_operation_ready_to_place(
        self,
    ) -> None:
        store, _ = self._assigned_fresh_operation()
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 1, True)
        )
        store.set_stock("destination", PayloadType.NORMAL, 1)
        store.complete_pick("op-1", "robot_1", "pick-op-1")

        with self.assertRaises(ConflictError):
            store.complete_place("op-1", "robot_1", "place-op-1")

        self.assertTrue(store.get_robot_pallet_state("robot_1").has_pallet)
        self.assertEqual(store.operation("op-1").status, OperationStatus.TO_PLACE)

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

    def _assigned_fresh_operation(self):
        store = InventoryStore(self.database_path)
        store.initialize()
        store.upsert_zone(Zone("source", "Source", "map", 0.0, 0.0, 0.0, 8, True))
        store.upsert_zone(
            Zone("destination", "Destination", "map", 2.0, 0.0, 0.0, 8, True)
        )
        store.set_stock("source", PayloadType.FRESH, 3)
        operation = store.create_operation(
            "op-1", PayloadType.FRESH, "source", "destination"
        )
        assigned = store.assign_operation(operation.operation_id, "robot_1")
        return store, assigned

    def _picked_fresh_operation(self) -> InventoryStore:
        store, _ = self._assigned_fresh_operation()
        store.complete_pick("op-1", "robot_1", "pick-op-1")
        return store


if __name__ == "__main__":
    unittest.main()
