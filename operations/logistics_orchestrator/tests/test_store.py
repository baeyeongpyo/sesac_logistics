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

    def test_pallet3_mission_is_idempotent_per_active_robot_and_persists_confirmation(self) -> None:
        mission, created = self.store.create_or_get_pallet3_mission("robot_1")
        repeated, repeated_created = self.store.create_or_get_pallet3_mission("robot_1")

        self.assertTrue(created)
        self.assertFalse(repeated_created)
        self.assertEqual(repeated.mission_id, mission.mission_id)
        arrived, transitioned = self.store.transition_pallet3_mission(
            mission.mission_id,
            "OUTBOUND_SENT",
            "AWAIT_UNLOAD_CONFIRMATION",
        )
        confirmed, confirmation_transitioned = self.store.confirm_pallet3_unload(
            mission.mission_id
        )
        confirmation_replay, replay_transitioned = self.store.confirm_pallet3_unload(
            mission.mission_id
        )

        self.assertTrue(transitioned)
        self.assertEqual(arrived.phase, "AWAIT_UNLOAD_CONFIRMATION")
        self.assertTrue(confirmation_transitioned)
        self.assertEqual(confirmed.phase, "FORK_DOWN_SENT")
        self.assertIsNotNone(confirmed.unload_confirmed_at)
        self.assertFalse(replay_transitioned)
        self.assertEqual(confirmation_replay.phase, "FORK_DOWN_SENT")

        self.store.close()
        self.store = OrchestratorStore(self.database_path)

        restored = self.store.get_pallet3_mission(mission.mission_id)
        self.assertEqual(restored.phase, "FORK_DOWN_SENT")
        self.assertIsNotNone(restored.unload_confirmed_at)

    def test_pallet3_mission_replaces_an_active_poc_with_a_new_mission(self) -> None:
        # This catches a repeat POC request being trapped by a stale active mission.
        original, created = self.store.create_or_get_pallet3_mission("robot_1")

        replacement, replacement_created = self.store.create_or_get_pallet3_mission(
            "robot_1", replace_active=True
        )

        replaced = self.store.get_pallet3_mission(original.mission_id)
        self.assertTrue(created)
        self.assertTrue(replacement_created)
        self.assertNotEqual(replacement.mission_id, original.mission_id)
        self.assertEqual(replacement.phase, "OUTBOUND_SENT")
        self.assertEqual(replaced.phase, "FAILED")
        self.assertEqual(replaced.failure_detail, "SUPERSEDED_BY_NEW_POC_REQUEST")

    def test_pallet3_operation_workflow_persists_manual_pick_and_terminal_phase(self) -> None:
        # This catches a Pallet 3 recipe being detached from its Inventory operation.
        workflow, created = self.store.create_or_get_pallet3_workflow(
            "operation-1", "robot_1"
        )
        repeated, repeated_created = self.store.create_or_get_pallet3_workflow(
            "operation-1", "robot_1"
        )
        confirmed, confirmation_changed = self.store.confirm_pallet3_manual_pick(
            "operation-1"
        )
        replayed, replay_changed = self.store.confirm_pallet3_manual_pick("operation-1")
        outbound, outbound_changed = self.store.transition_pallet3_workflow(
            "operation-1", "PICK_PENDING", "OUTBOUND_SENT"
        )
        completed, completed_changed = self.store.transition_pallet3_workflow(
            "operation-1", "OUTBOUND_SENT", "COMPLETED"
        )

        self.assertTrue(created)
        self.assertFalse(repeated_created)
        self.assertEqual(workflow.operation_id, "operation-1")
        self.assertEqual(repeated.operation_id, "operation-1")
        self.assertEqual(workflow.phase, "PICK_PENDING")
        self.assertTrue(confirmation_changed)
        self.assertIsNotNone(confirmed.manual_pick_confirmed_at)
        self.assertFalse(replay_changed)
        self.assertEqual(replayed.manual_pick_confirmed_at, confirmed.manual_pick_confirmed_at)
        self.assertTrue(outbound_changed)
        self.assertEqual(outbound.phase, "OUTBOUND_SENT")
        self.assertTrue(completed_changed)
        self.assertEqual(completed.phase, "COMPLETED")
        self.assertIsNotNone(completed.completed_at)

        self.store.close()
        self.store = OrchestratorStore(self.database_path)

        restored = self.store.get_pallet3_workflow("operation-1")
        self.assertIsNotNone(restored)
        self.assertEqual(restored.phase, "COMPLETED")
        self.assertIsNotNone(restored.manual_pick_confirmed_at)


if __name__ == "__main__":
    unittest.main()
