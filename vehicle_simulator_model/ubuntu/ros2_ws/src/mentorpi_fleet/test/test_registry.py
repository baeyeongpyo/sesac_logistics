from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from mentorpi_fleet.registry import RegistryValidationError, enabled_vehicles, load_registry


def write_registry(root: Path, body: str) -> Path:
    path = root / 'fleet_registry.yaml'
    path.write_text(body)
    return path


VALID_REGISTRY = '''\
control_domain: 215
profiles: [mentorpi]
vehicles:
  - id: robot_1
    kind: physical
    domain_id: 1
    namespace: /robot_1
    profile: mentorpi
    enabled: true
  - id: robot_2
    kind: physical
    domain_id: 2
    namespace: /robot_2
    profile: mentorpi
    enabled: false
  - id: sim_robot_1
    kind: simulation
    domain_id: 100
    namespace: /sim_robot_1
    profile: mentorpi
    enabled: true
    spawn: {x: 1.0, y: 2.0, z: 0.0, yaw: 0.0}
    nav_enabled: true
'''


class FleetRegistryTest(unittest.TestCase):
    def test_loads_declared_physical_and_simulation_vehicles(self):
        with TemporaryDirectory() as directory:
            registry = load_registry(write_registry(Path(directory), VALID_REGISTRY))

        self.assertEqual(registry.control_domain, 215)
        self.assertEqual(
            [(vehicle.vehicle_id, vehicle.kind, vehicle.domain_id, vehicle.namespace)
             for vehicle in registry.vehicles],
            [
                ('robot_1', 'physical', 1, '/robot_1'),
                ('robot_2', 'physical', 2, '/robot_2'),
                ('sim_robot_1', 'simulation', 100, '/sim_robot_1'),
            ],
        )
        self.assertEqual(
            [vehicle.vehicle_id for vehicle in enabled_vehicles(registry)],
            ['robot_1', 'sim_robot_1'],
        )
        self.assertEqual(
            [vehicle.vehicle_id for vehicle in enabled_vehicles(registry, kind='simulation')],
            ['sim_robot_1'],
        )

    def test_rejects_duplicate_vehicle_identity_and_transport_identity(self):
        cases = {
            'vehicle_id': VALID_REGISTRY + '''\
  - id: robot_1
    kind: physical
    domain_id: 3
    namespace: /robot_3
    profile: mentorpi
    enabled: true
''',
            'namespace': VALID_REGISTRY + '''\
  - id: robot_3
    kind: physical
    domain_id: 3
    namespace: /robot_1
    profile: mentorpi
    enabled: true
''',
            'domain': VALID_REGISTRY + '''\
  - id: robot_3
    kind: physical
    domain_id: 1
    namespace: /robot_3
    profile: mentorpi
    enabled: true
''',
        }
        with TemporaryDirectory() as directory:
            root = Path(directory)
            for name, document in cases.items():
                with self.subTest(name=name):
                    with self.assertRaises(RegistryValidationError):
                        load_registry(write_registry(root, document))

    def test_rejects_invalid_profile_domain_and_simulation_pose(self):
        cases = {
            'profile': VALID_REGISTRY.replace('profile: mentorpi\n    enabled: false', 'profile: unknown\n    enabled: false'),
            'domain': VALID_REGISTRY.replace('domain_id: 100', 'domain_id: 102'),
            'missing_spawn': VALID_REGISTRY.replace('    spawn: {x: 1.0, y: 2.0, z: 0.0, yaw: 0.0}\n', ''),
        }
        with TemporaryDirectory() as directory:
            root = Path(directory)
            for name, document in cases.items():
                with self.subTest(name=name):
                    with self.assertRaises(RegistryValidationError):
                        load_registry(write_registry(root, document))


if __name__ == '__main__':
    unittest.main()
