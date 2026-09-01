from pathlib import Path
import subprocess
import unittest

import yaml


class OrchestratorComposeContractTest(unittest.TestCase):
    def test_compose_persists_database_and_configures_current_service_urls(self) -> None:
        # This catches a restart losing step state or a container lacking its source URLs.
        compose_path = Path(__file__).parents[1] / "docker-compose.yaml"
        result = subprocess.run(
            ["docker", "compose", "-f", str(compose_path), "config"],
            check=True,
            capture_output=True,
            text=True,
        )
        compose = yaml.safe_load(result.stdout)

        service = compose["services"]["logistics-orchestrator-api"]
        self.assertIn("ORCHESTRATOR_DB_PATH", service["environment"])
        self.assertIn("INVENTORY_URL", service["environment"])
        self.assertIn("FLEET_MANAGER_URL", service["environment"])
        self.assertIn("FLEET_BRIDGE_URL", service["environment"])
        self.assertEqual(service["volumes"][0]["type"], "bind")
        self.assertEqual(service["volumes"][0]["target"], "/data")
        self.assertEqual(
            service["ports"],
            [
                {
                    "mode": "ingress",
                    "host_ip": "127.0.0.1",
                    "target": 8080,
                    "published": "8083",
                    "protocol": "tcp",
                }
            ],
        )


if __name__ == "__main__":
    unittest.main()
