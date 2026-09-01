from pathlib import Path
import unittest


class ComposeContractTest(unittest.TestCase):
    def test_compose_keeps_database_file_in_inventory_db_data_directory(self) -> None:
        compose_path = Path(__file__).parents[1] / "docker-compose.yaml"

        compose = compose_path.read_text()

        self.assertIn("inventory-api:", compose)
        self.assertIn("./data:/data", compose)
        self.assertIn("INVENTORY_DB_PATH", compose)
        self.assertIn("127.0.0.1", compose)


if __name__ == "__main__":
    unittest.main()
