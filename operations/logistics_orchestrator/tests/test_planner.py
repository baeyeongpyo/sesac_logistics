import unittest

from logistics_orchestrator.app.planner import (
    InventorySnapshot,
    TransferCandidate,
    select_next_transfer,
    validate_required_zones,
)


def _zones(*, disabled: set[str] | None = None) -> list[dict]:
    disabled = disabled or set()
    zone_ids = ["docker", "p1", "p2", "p3"]
    zone_ids += [f"f{index}" for index in range(1, 10)]
    zone_ids += [f"n{index}" for index in range(1, 10)]
    return [
        {
            "zone_id": zone_id,
            "name": zone_id.upper(),
            "map_name": "map_0825",
            "nav_x": 0.0,
            "nav_y": 0.0,
            "nav_yaw": 0.0,
            "capacity": 24 if zone_id == "docker" else 1,
            "enabled": zone_id not in disabled,
        }
        for zone_id in zone_ids
    ]


def _stock(zone_id: str, payload_type: str, quantity: int, reserved: int = 0) -> dict:
    return {
        "zone_id": zone_id,
        "payload_type": payload_type,
        "quantity": quantity,
        "reserved_quantity": reserved,
        "available_quantity": quantity - reserved,
    }


def _snapshot(
    stocks: list[dict], active_operations: list[dict] | None = None
) -> InventorySnapshot:
    return InventorySnapshot(
        zones=_zones(),
        stocks=stocks,
        active_operations=active_operations or [],
    )


class TransferSelectionTest(unittest.TestCase):
    def test_docker_fresh_moves_to_lowest_empty_pallet_slot_before_normal(self) -> None:
        # This catches dispatching NORMAL while physical FRESH cargo remains in docker.
        candidate = select_next_transfer(
            _snapshot([_stock("docker", "FRESH", 1), _stock("docker", "NORMAL", 4)])
        )

        self.assertEqual(candidate, TransferCandidate("docker", "p1", "FRESH"))

    def test_reserved_docker_fresh_still_blocks_normal_dispatch(self) -> None:
        # This catches selecting NORMAL just because FRESH is currently reserved.
        candidate = select_next_transfer(
            _snapshot(
                [
                    _stock("docker", "FRESH", 1, reserved=1),
                    _stock("docker", "NORMAL", 4),
                ]
            )
        )

        self.assertIsNone(candidate)

    def test_full_pallet_slots_send_docker_fresh_to_lowest_empty_fresh_slot(self) -> None:
        # This catches stopping at full P slots instead of clearing docker into F storage.
        candidate = select_next_transfer(
            _snapshot(
                [
                    _stock("docker", "FRESH", 1),
                    _stock("p1", "FRESH", 1),
                    _stock("p2", "NORMAL", 1),
                    _stock("p3", "FRESH", 1),
                ]
            )
        )

        self.assertEqual(candidate, TransferCandidate("docker", "f1", "FRESH"))

    def test_empty_docker_refills_lowest_pallet_slot_from_fresh_before_normal(self) -> None:
        # This catches choosing internal NORMAL while any FRESH source is available.
        candidate = select_next_transfer(
            _snapshot([_stock("f2", "FRESH", 1), _stock("n1", "NORMAL", 1)])
        )

        self.assertEqual(candidate, TransferCandidate("f2", "p1", "FRESH"))

    def test_destination_with_inbound_reservation_is_not_reused(self) -> None:
        # This catches assigning two robots to one capacity-1 P slot.
        candidate = select_next_transfer(
            _snapshot(
                [_stock("docker", "NORMAL", 1)],
                active_operations=[
                    {
                        "operation_id": "existing-operation",
                        "destination_zone_id": "p1",
                        "status": "TO_PICK",
                    }
                ],
            )
        )

        self.assertEqual(candidate, TransferCandidate("docker", "p2", "NORMAL"))

    def test_missing_or_disabled_required_zone_reports_configuration_error(self) -> None:
        # This catches issuing automatic work when Inventory's physical map is incomplete.
        zones = [zone for zone in _zones(disabled={"f3"}) if zone["zone_id"] != "n9"]

        self.assertEqual(validate_required_zones(zones), ["f3 disabled", "n9 missing"])


if __name__ == "__main__":
    unittest.main()
