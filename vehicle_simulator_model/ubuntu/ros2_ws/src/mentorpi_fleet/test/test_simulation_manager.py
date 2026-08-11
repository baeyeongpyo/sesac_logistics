from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_fleet.registry import SpawnPose, VehicleSpec
from mentorpi_fleet.simulation_manager import SimulationManager


class Process:
    def __init__(self, events, label):
        self.events = events
        self.label = label

    def poll(self):
        return None

    def terminate(self):
        self.events.append(f'terminate:{self.label}')

    def wait(self, timeout):
        self.events.append(f'wait:{self.label}')

    def kill(self):
        raise AssertionError('simulation process should terminate gracefully')


class WorkerManager:
    def __init__(self, events):
        self.events = events

    def reconcile(self, vehicles):
        self.events.append('workers:' + ','.join(vehicle.vehicle_id for vehicle in vehicles))


def simulation(vehicle_id, domain_id, enabled=True, nav_enabled=True):
    return VehicleSpec(
        vehicle_id, 'simulation', domain_id, f'/{vehicle_id}', 'mentorpi', enabled,
        SpawnPose(1.0, 2.0, 0.05, 0.0), nav_enabled,
    )


class SimulationManagerTest(unittest.TestCase):
    def test_starts_only_enabled_simulation_vehicles(self):
        events = []
        with TemporaryDirectory() as directory:
            root = Path(directory)
            template = root / 'vehicle_bridge.yaml.in'
            template.write_text('/__ROBOT_ID__/controller/cmd_vel\n')
            manager = SimulationManager(
                215, root, template,
                worker_manager=WorkerManager(events),
                run_command=lambda command, env: events.append(
                    'run:' + command[command.index('-name') + 1]
                ),
                start_process=lambda command, env, label: (
                    events.append('start:' + label) or Process(events, label)
                ),
            )
            manager.reconcile([
                simulation('sim_robot_1', 100),
                simulation('sim_robot_2', 101, enabled=False),
                VehicleSpec('robot_1', 'physical', 1, '/robot_1', 'mentorpi', True),
            ])

        self.assertEqual(manager.vehicle_ids, ('sim_robot_1',))
        self.assertIn('start:adapter:sim_robot_1', events)
        self.assertNotIn('start:adapter:sim_robot_2', events)
        self.assertEqual(events[-1], 'workers:sim_robot_1')

    def test_removal_runs_safe_cleanup_for_only_removed_vehicle(self):
        events = []
        with TemporaryDirectory() as directory:
            root = Path(directory)
            template = root / 'vehicle_bridge.yaml.in'
            template.write_text('/__ROBOT_ID__/controller/cmd_vel\n')
            manager = SimulationManager(
                215, root, template,
                worker_manager=WorkerManager(events),
                run_command=lambda command, env: events.append(
                    'run:' + command[command.index('-name') + 1]
                ),
                start_process=lambda command, env, label: (
                    events.append('start:' + label) or Process(events, label)
                ),
                delete_scene=lambda vehicle_id: events.append('scene:' + vehicle_id),
            )
            manager.reconcile([simulation('sim_robot_1', 100), simulation('sim_robot_2', 101)])
            events.clear()
            manager.reconcile([simulation('sim_robot_2', 101)])

        self.assertEqual(manager.vehicle_ids, ('sim_robot_2',))
        self.assertEqual(events, [
            'workers:sim_robot_2',
            'terminate:nav:sim_robot_1', 'wait:nav:sim_robot_1',
            'terminate:adapter:sim_robot_1', 'wait:adapter:sim_robot_1',
            'run:sim_robot_1', 'scene:sim_robot_1', 'workers:sim_robot_2',
        ])


if __name__ == '__main__':
    unittest.main()
