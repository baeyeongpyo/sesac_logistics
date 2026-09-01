from pathlib import Path
import unittest

import yaml


class ComposeContractTests(unittest.TestCase):
    def test_compose_mounts_fleet_manager_database_directory(self) -> None:
        compose_path = Path(__file__).parents[1] / "docker-compose.yaml"

        compose = yaml.safe_load(compose_path.read_text())

        service = compose["services"]["fleet-manager-api"]
        self.assertIn("FLEET_MANAGER_DB_PATH", service["environment"])
        self.assertIn("../data:/data", service["volumes"])


if __name__ == "__main__":
    unittest.main()
