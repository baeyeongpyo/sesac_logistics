from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_fleet.fleet_state import FleetPresence
from mentorpi_fleet.registry import VehicleSpec


def vehicle(vehicle_id, kind='physical', domain_id=1):
    return VehicleSpec(vehicle_id, kind, domain_id, f'/{vehicle_id}', 'mentorpi', True)


class FleetPresenceTest(unittest.TestCase):
    def test_odom_marks_only_matching_vehicle_online(self):
        presence = FleetPresence(timeout_seconds=3.0)
        presence.reconcile([vehicle('robot_1'), vehicle('robot_2', domain_id=2)])

        presence.record_odom('robot_1', now=10.0)

        self.assertEqual(presence.snapshot(now=11.0), [
            {'id': 'robot_1', 'kind': 'physical', 'domain_id': 1, 'online': True, 'state': 'online'},
            {'id': 'robot_2', 'kind': 'physical', 'domain_id': 2, 'online': False, 'state': 'offline'},
        ])

    def test_timeout_removes_only_stale_vehicle_from_online_state(self):
        presence = FleetPresence(timeout_seconds=3.0)
        presence.reconcile([vehicle('robot_1'), vehicle('robot_2', domain_id=2)])
        presence.record_odom('robot_1', now=1.0)
        presence.record_odom('robot_2', now=4.0)

        statuses = presence.snapshot(now=5.0)

        self.assertFalse(statuses[0]['online'])
        self.assertTrue(statuses[1]['online'])

    def test_reconcile_only_keeps_requested_vehicle_kind(self):
        presence = FleetPresence(timeout_seconds=3.0)
        presence.reconcile([
            vehicle('robot_1'),
            vehicle('sim_robot_1', kind='simulation', domain_id=100),
        ], kind='physical')

        self.assertEqual([status['id'] for status in presence.snapshot(now=0.0)], ['robot_1'])


if __name__ == '__main__':
    unittest.main()
