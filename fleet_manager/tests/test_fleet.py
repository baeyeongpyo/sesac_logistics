import tempfile
import unittest
from pathlib import Path

from fleet_manager.app.fleet import (
    StateSource,
    VehicleState,
    VehicleStateReport,
    VehicleStateStore,
)


class VehicleStateStoreTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory()
        self.store = VehicleStateStore(Path(self.tempdir.name) / "fleet_manager.db")

    def tearDown(self) -> None:
        self.store.close()
        self.tempdir.cleanup()

    def test_create_state_adds_snapshot_and_log(self) -> None:
        report = VehicleStateReport(
            state=VehicleState.INIT,
            previous_state=None,
            operation_id=None,
            attempt_id=None,
            source=StateSource.VEHICLE,
            detail="VEHICLE_BOOTED",
            observed_at="2026-08-31T12:00:00Z",
        )

        snapshot = self.store.record_state("robot-1", report)

        self.assertEqual(snapshot.robot_id, "robot-1")
        self.assertEqual(snapshot.state, VehicleState.INIT)
        self.assertIsNone(snapshot.previous_state)
        self.assertEqual(snapshot.detail, "VEHICLE_BOOTED")
        logs = self.store.list_logs("robot-1")
        self.assertEqual(len(logs), 1)
        self.assertEqual(logs[0].state, VehicleState.INIT)
        self.assertEqual(logs[0].previous_state, None)

    def test_same_state_updates_snapshot_without_adding_log(self) -> None:
        self.store.record_state(
            "robot-1",
            VehicleStateReport(
                state=VehicleState.PICK,
                previous_state=VehicleState.DRIVE,
                operation_id="operation-1",
                attempt_id=None,
                source=StateSource.AUTO_DOCK,
                detail="AUTO_DOCK_SEARCHING",
                observed_at="2026-08-31T12:00:00Z",
            ),
        )

        snapshot = self.store.record_state(
            "robot-1",
            VehicleStateReport(
                state=VehicleState.PICK,
                previous_state=VehicleState.DRIVE,
                operation_id="operation-1",
                attempt_id=None,
                source=StateSource.AUTO_DOCK,
                detail="AUTO_DOCK_ALIGNING",
                observed_at="2026-08-31T12:00:01Z",
            ),
        )

        self.assertEqual(snapshot.detail, "AUTO_DOCK_ALIGNING")
        self.assertEqual(len(self.store.list_logs("robot-1")), 1)

    def test_state_change_adds_a_new_log(self) -> None:
        self.store.record_state(
            "robot-1",
            VehicleStateReport(
                state=VehicleState.DRIVE,
                previous_state=VehicleState.WAIT,
                operation_id="operation-1",
                attempt_id="attempt-1",
                source=StateSource.NAV2,
                detail="NAVIGATION_STARTED",
                observed_at="2026-08-31T12:00:00Z",
            ),
        )

        self.store.record_state(
            "robot-1",
            VehicleStateReport(
                state=VehicleState.PICK,
                previous_state=VehicleState.DRIVE,
                operation_id="operation-1",
                attempt_id=None,
                source=StateSource.AUTO_DOCK,
                detail="AUTO_DOCK_SEARCHING",
                observed_at="2026-08-31T12:00:01Z",
            ),
        )

        logs = self.store.list_logs("robot-1")
        self.assertEqual(len(logs), 2)
        self.assertEqual(logs[0].state, VehicleState.PICK)
        self.assertEqual(logs[0].previous_state, VehicleState.DRIVE)
        self.assertEqual(logs[1].state, VehicleState.DRIVE)

    def test_log_listing_pages_from_newest_to_oldest(self) -> None:
        for index, state in enumerate(
            (VehicleState.INIT, VehicleState.WAIT, VehicleState.DRIVE)
        ):
            self.store.record_state(
                "robot-1",
                VehicleStateReport(
                    state=state,
                    previous_state=None if index == 0 else VehicleState.INIT,
                    operation_id=None,
                    attempt_id=None,
                    source=StateSource.VEHICLE,
                    detail=f"EVENT_{index}",
                    observed_at=f"2026-08-31T12:00:0{index}Z",
                ),
            )

        newest_page = self.store.list_logs("robot-1", limit=2)
        older_page = self.store.list_logs(
            "robot-1", before_id=newest_page[-1].id, limit=2
        )

        self.assertEqual([log.state for log in newest_page], [VehicleState.DRIVE, VehicleState.WAIT])
        self.assertEqual([log.state for log in older_page], [VehicleState.INIT])


if __name__ == "__main__":
    unittest.main()
