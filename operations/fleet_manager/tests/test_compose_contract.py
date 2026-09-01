from pathlib import Path
import unittest

import yaml


class ComposeContractTests(unittest.TestCase):
    def test_compose_mounts_fleet_manager_database_directory(self) -> None:
        compose_path = Path(__file__).parents[1] / "docker-compose.yaml"

        compose = yaml.safe_load(compose_path.read_text())

        service = compose["services"]["fleet-manager-api"]
        self.assertIn("FLEET_MANAGER_DB_PATH", service["environment"])
        self.assertIn("VEHICLE_REGISTRY_PATH", service["environment"])
        self.assertIn("../data:/data", service["volumes"])
        registry_mount = next(
            volume
            for volume in service["volumes"]
            if isinstance(volume, dict)
            and volume["target"] == "/app/config/vehicles.yaml"
        )
        self.assertEqual(registry_mount["source"], "./config/vehicles.yaml")
        self.assertTrue(registry_mount["read_only"])


if __name__ == "__main__":
    unittest.main()
