from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_fleet.registry import VehicleSpec
from mentorpi_fleet.worker_manager import BridgeWorkerManager


class FakeProcess:
    def __init__(self):
        self.terminated = False
        self.waited = False

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout):
        self.waited = True

    def kill(self):
        raise AssertionError('healthy worker must not be killed')


def vehicle(vehicle_id, domain_id):
    return VehicleSpec(vehicle_id, 'physical', domain_id, f'/{vehicle_id}', 'mentorpi', True)


class BridgeWorkerManagerTest(unittest.TestCase):
    def test_adding_vehicle_keeps_existing_worker_running(self):
        first = FakeProcess()
        second = FakeProcess()
        with TemporaryDirectory() as directory, patch(
            'mentorpi_fleet.worker_manager.subprocess.Popen', side_effect=[first, second]
        ):
            manager = BridgeWorkerManager(215, Path(directory))
            manager.reconcile([vehicle('robot_1', 1)])
            manager.reconcile([vehicle('robot_1', 1), vehicle('robot_3', 3)])

        self.assertFalse(first.terminated)
        self.assertIn('robot_1', manager.worker_ids)
        self.assertIn('robot_3', manager.worker_ids)

    def test_replacing_one_vehicle_does_not_stop_another_vehicle_worker(self):
        first = FakeProcess()
        original_second = FakeProcess()
        replacement_second = FakeProcess()
        with TemporaryDirectory() as directory, patch(
            'mentorpi_fleet.worker_manager.subprocess.Popen',
            side_effect=[first, original_second, replacement_second],
        ):
            manager = BridgeWorkerManager(215, Path(directory))
            manager.reconcile([vehicle('robot_1', 1), vehicle('robot_2', 2)])
            manager.reconcile([vehicle('robot_1', 1), vehicle('robot_2', 4)])

        self.assertFalse(first.terminated)
        self.assertTrue(original_second.terminated)
        self.assertIn('robot_2', manager.worker_ids)

    def test_removing_vehicle_stops_only_that_vehicle_worker(self):
        first = FakeProcess()
        second = FakeProcess()
        with TemporaryDirectory() as directory, patch(
            'mentorpi_fleet.worker_manager.subprocess.Popen', side_effect=[first, second]
        ):
            manager = BridgeWorkerManager(215, Path(directory))
            manager.reconcile([vehicle('robot_1', 1), vehicle('robot_2', 2)])
            manager.reconcile([vehicle('robot_2', 2)])

        self.assertTrue(first.terminated)
        self.assertFalse(second.terminated)
        self.assertEqual(manager.worker_ids, ('robot_2',))


if __name__ == '__main__':
    unittest.main()
