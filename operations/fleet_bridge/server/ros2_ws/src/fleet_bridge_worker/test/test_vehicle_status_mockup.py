import importlib.util
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import sys
import threading
from types import SimpleNamespace
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / "common" / "fleet_bridge_config"
REPOSITORY_ROOT = PACKAGE.parents[5]
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fastapi.testclient import TestClient

from fleet_bridge_config.models import (
    CommandApiConfig,
    FleetConfig,
    ServerConfig,
    VehicleConfig,
)
from fleet_bridge_worker.api import create_app


def load_vehicle_module():
    path = REPOSITORY_ROOT / "vehicle_communication" / "vehicle_command_api.py"
    spec = importlib.util.spec_from_file_location("vehicle_command_api_mockup", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RecordingFleetManager:
    def __init__(self):
        self.calls: list[tuple[str, dict]] = []

    async def record_vehicle_state(self, robot_id: str, payload: dict):
        self.calls.append((robot_id, payload))
        return SimpleNamespace(status_code=200, body={"status": "stored"})


def fleet() -> FleetConfig:
    return FleetConfig(
        server=ServerConfig(
            domain_id=225,
            foxglove_port=8765,
            command_api=CommandApiConfig(host="127.0.0.1", port=8080),
        ),
        vehicles=(
            VehicleConfig(
                id="robot_2",
                foxglove_uri="ws://vehicle.example:8765",
                command_api_url="http://vehicle.example:8082",
                enabled=True,
            ),
        ),
    )


class VehicleStatusMockupTest(unittest.TestCase):
    def test_vehicle_status_reporter_posts_init_and_ready_to_bridge(self):
        vehicle = load_vehicle_module()
        fleet_manager = RecordingFleetManager()
        bridge = create_app(fleet(), object(), fleet_manager)

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers["Content-Length"])
                payload = json.loads(self.rfile.read(length))
                with TestClient(bridge) as client:
                    response = client.post(self.path, json=payload)
                self.server.paths.append(self.path)
                self.server.payloads.append(payload)
                body = json.dumps(response.json()).encode("utf-8")
                self.send_response(response.status_code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, _format, *_args):
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.paths = []
        server.payloads = []
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        reporter = vehicle.FleetStatusReporter(
            "robot_2", f"http://127.0.0.1:{server.server_port}"
        )
        service = vehicle.VehicleCommandService(
            velocity=SimpleNamespace(),
            navigation=SimpleNamespace(),
            max_linear_x=0.1,
            max_angular_z=0.5,
            max_hold_ms=1000,
            status_reporter=reporter,
        )
        try:
            service.report_current_status()
            service.mark_idle({"reason": "MOCKUP_OPERATOR_READY"})
            reporter.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

        self.assertEqual(
            server.paths,
            [
                "/api/v1/vehicle-status/robot_2",
                "/api/v1/vehicle-status/robot_2",
            ],
        )
        self.assertEqual([robot_id for robot_id, _payload in fleet_manager.calls], ["robot_2", "robot_2"])

        init_payload = fleet_manager.calls[0][1]
        ready_payload = fleet_manager.calls[1][1]
        self.assertEqual(
            {key: value for key, value in init_payload.items() if key != "observed_at"},
            {
                "state": "INIT",
                "previous_state": None,
                "operation_id": None,
                "attempt_id": None,
                "source": "VEHICLE",
                "detail": "VEHICLE_BOOTED",
            },
        )
        self.assertEqual(
            {key: value for key, value in ready_payload.items() if key != "observed_at"},
            {
                "state": "WAIT",
                "previous_state": "INIT",
                "operation_id": None,
                "attempt_id": None,
                "source": "API",
                "detail": "OPERATOR_READY",
            },
        )
        for sent_payload, forwarded_payload in zip(
            server.payloads, (init_payload, ready_payload), strict=True
        ):
            self.assertRegex(
                sent_payload["observed_at"],
                r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$",
            )
            self.assertEqual(
                datetime.fromisoformat(sent_payload["observed_at"].replace("Z", "+00:00")),
                datetime.fromisoformat(
                    forwarded_payload["observed_at"].replace("Z", "+00:00")
                ),
            )


if __name__ == "__main__":
    unittest.main()
