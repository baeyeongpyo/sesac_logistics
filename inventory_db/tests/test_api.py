from pathlib import Path
import tempfile
import unittest
from uuid import UUID
import warnings

from fastapi.testclient import TestClient

from inventory_db.app.main import create_app


warnings.filterwarnings(
    "ignore",
    message="'asyncio.iscoroutinefunction' is deprecated",
    category=DeprecationWarning,
    module="fastapi.routing",
)


class InventoryApiTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        database_path = Path(self.temporary_directory.name) / "inventory.db"
        self.client = TestClient(create_app(str(database_path)))
        self.client.__enter__()

        for zone_id, name, nav_x in (
            ("source", "Source", 0.0),
            ("destination", "Destination", 2.0),
        ):
            response = self.client.put(
                f"/api/v1/zones/{zone_id}",
                json={
                    "name": name,
                    "map_name": "warehouse_map",
                    "nav_x": nav_x,
                    "nav_y": 0.0,
                    "nav_yaw": 0.0,
                    "capacity": 8,
                    "enabled": True,
                },
            )
            self.assertEqual(response.status_code, 200)

        self.assertEqual(
            self.client.put(
                "/api/v1/stocks/source/FRESH", json={"quantity": 3}
            ).status_code,
            200,
        )
        operation_response = self.client.post(
            "/api/v1/operations",
            json={
                "robot_id": "robot_1",
                "payload_type": "FRESH",
                "source_zone_id": "source",
                "destination_zone_id": "destination",
                "priority": 0,
            },
        )
        self.assertEqual(operation_response.status_code, 201)
        self.operation_id = operation_response.json()["operation_id"]

    def tearDown(self) -> None:
        self.client.__exit__(None, None, None)
        self.temporary_directory.cleanup()

    def test_pick_completion_endpoint_returns_same_event_for_same_idempotency_key(
        self,
    ) -> None:
        response = self.client.post(
            f"/api/v1/operations/{self.operation_id}/pick-completions",
            json={"robot_id": "robot_1", "idempotency_key": "pick-op-1"},
        )
        repeated = self.client.post(
            f"/api/v1/operations/{self.operation_id}/pick-completions",
            json={"robot_id": "robot_1", "idempotency_key": "pick-op-1"},
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(repeated.status_code, 200)
        self.assertEqual(response.json()["event_id"], repeated.json()["event_id"])

    def test_next_instruction_endpoint_returns_pick_zone_for_empty_robot(self) -> None:
        response = self.client.get("/api/v1/robots/robot_1/next-instruction")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["action"], "PICK")
        self.assertEqual(response.json()["zone_id"], "source")

    def test_operation_creation_generates_an_inventory_owned_uuid(self) -> None:
        response = self.client.post(
            "/api/v1/operations",
            json={
                "robot_id": "robot_2",
                "payload_type": "FRESH",
                "source_zone_id": "source",
                "destination_zone_id": "destination",
                "priority": 5,
            },
        )

        self.assertEqual(response.status_code, 201)
        UUID(response.json()["operation_id"])
        self.assertEqual(response.json()["robot_id"], "robot_2")
        self.assertEqual(response.json()["status"], "TO_PICK")

    def test_operation_creation_requires_the_assigned_robot(self) -> None:
        response = self.client.post(
            "/api/v1/operations",
            json={
                "payload_type": "FRESH",
                "source_zone_id": "source",
                "destination_zone_id": "destination",
            },
        )

        self.assertEqual(response.status_code, 422)
        self.assertEqual(self.client.get("/api/v1/stocks").json()[0]["reserved_quantity"], 1)

    def test_place_completion_endpoint_clears_robot_load(self) -> None:
        self.assertEqual(
            self.client.post(
                f"/api/v1/operations/{self.operation_id}/pick-completions",
                json={"robot_id": "robot_1", "idempotency_key": "pick-op-1"},
            ).status_code,
            200,
        )

        response = self.client.post(
            f"/api/v1/operations/{self.operation_id}/place-completions",
            json={"robot_id": "robot_1", "idempotency_key": "place-op-1"},
        )

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["event_type"], "PLACE_COMPLETED")

    def test_stock_list_endpoint_returns_available_quantity(self) -> None:
        response = self.client.get("/api/v1/stocks")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()[0]["zone_id"], "source")
        self.assertEqual(response.json()[0]["available_quantity"], 2)

    def test_robot_pallet_state_endpoint_reports_empty_robot(self) -> None:
        response = self.client.get("/api/v1/robots/robot_1/pallet-state")

        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()["has_pallet"])
        self.assertIsNone(response.json()["payload_type"])

    def test_zone_list_endpoint_returns_navigation_target(self) -> None:
        response = self.client.get("/api/v1/zones")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()[0]["zone_id"], "destination")
        self.assertEqual(response.json()[0]["map_name"], "warehouse_map")

    def test_busy_robot_cannot_receive_a_second_operation(self) -> None:
        response = self.client.post(
            "/api/v1/operations",
            json={
                "robot_id": "robot_1",
                "payload_type": "FRESH",
                "source_zone_id": "source",
                "destination_zone_id": "destination",
            },
        )

        self.assertEqual(response.status_code, 409)
        self.assertIn("detail", response.json())
        stocks = self.client.get("/api/v1/stocks").json()
        self.assertEqual(stocks[0]["reserved_quantity"], 1)

    def test_unknown_zone_stock_write_returns_http_404(self) -> None:
        response = self.client.put(
            "/api/v1/stocks/unknown/FRESH", json={"quantity": 1}
        )

        self.assertEqual(response.status_code, 404)
        self.assertIn("detail", response.json())

    def test_openapi_documents_every_operation_and_its_parameters(self) -> None:
        schema = create_app(":memory:").openapi()

        for path, path_item in schema["paths"].items():
            for method, operation in path_item.items():
                if method not in {"get", "post", "put"}:
                    continue
                with self.subTest(path=path, method=method):
                    self.assertTrue(operation["summary"].strip())
                    self.assertIn("**처리:**", operation["description"])
                    self.assertIn("**결과:**", operation["description"])
                    for parameter in operation.get("parameters", []):
                        self.assertTrue(parameter["description"].strip())

        request_schema = schema["components"]["schemas"]["OperationRequest"]
        self.assertIn("robot_id", request_schema["required"])
        self.assertNotIn("/api/v1/operations/{operation_id}/assignments", schema["paths"])

    def test_active_operations_lists_pending_work_by_priority(self) -> None:
        higher_priority = self.client.post(
            "/api/v1/operations",
            json={
                "robot_id": "robot_2",
                "payload_type": "FRESH",
                "source_zone_id": "source",
                "destination_zone_id": "destination",
                "priority": 10,
            },
        )
        self.assertEqual(higher_priority.status_code, 201)

        response = self.client.get("/api/v1/operations/active")

        self.assertEqual(response.status_code, 200)
        operations = response.json()
        self.assertEqual(
            [operation["operation_id"] for operation in operations],
            [higher_priority.json()["operation_id"], self.operation_id],
        )
        self.assertEqual(
            [operation["status"] for operation in operations],
            ["TO_PICK", "TO_PICK"],
        )

    def test_active_operations_excludes_completed_work(self) -> None:
        self.assertEqual(
            self.client.post(
                f"/api/v1/operations/{self.operation_id}/pick-completions",
                json={"robot_id": "robot_1", "idempotency_key": "pick-active-1"},
            ).status_code,
            200,
        )
        self.assertEqual(
            self.client.post(
                f"/api/v1/operations/{self.operation_id}/place-completions",
                json={"robot_id": "robot_1", "idempotency_key": "place-active-1"},
            ).status_code,
            200,
        )

        response = self.client.get("/api/v1/operations/active")

        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), [])


if __name__ == "__main__":
    unittest.main()
