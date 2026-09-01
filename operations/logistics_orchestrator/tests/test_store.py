from datetime import datetime, timezone
from pathlib import Path
import tempfile
import unittest

from logistics_orchestrator.app.models import EventEnvelope
from logistics_orchestrator.app.store import OrchestratorStore


class OrchestratorStoreTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.database_path = Path(self.temporary_directory.name) / "orchestrator.db"
        self.store = OrchestratorStore(self.database_path)

    def tearDown(self) -> None:
        self.store.close()
        self.temporary_directory.cleanup()

    def test_records_each_source_event_once(self) -> None:
        # This catches a replayed source event triggering the same work twice.
        envelope = EventEnvelope(
            event_id="fleet-event-1",
            event_type="fleet.vehicle_reported",
            occurred_at=datetime(2026, 8, 31, 12, 0, tzinfo=timezone.utc),
            payload={"robot_id": "robot_1", "state": "WAIT"},
        )

        self.assertTrue(self.store.record_event("fleet", envelope))
        self.assertFalse(self.store.record_event("fleet", envelope))
        self.assertEqual(self.store.status()["inbox_event_count"], 1)

    def test_command_is_stored_before_delivery_and_keeps_delivery_unknown(self) -> None:
        # This catches dropping an ambiguous timeout command and sending a duplicate later.
        command = self.store.enqueue_command(
            operation_id="operation-1",
            robot_id="robot_1",
            command_type="NAV_TO_PICK",
            payload={"purpose": "PICK"},
        )
        self.store.mark_command_delivery_unknown(command.command_id, "request timeout")

        saved = self.store.get_command(command.command_id)
        self.assertIsNotNone(saved)
        self.assertEqual(saved.status, "DELIVERY_UNKNOWN")
        self.assertEqual(saved.last_error, "request timeout")


if __name__ == "__main__":
    unittest.main()
