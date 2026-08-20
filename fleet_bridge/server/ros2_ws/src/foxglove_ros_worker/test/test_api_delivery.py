import asyncio
from pathlib import Path
import sys
import unittest

from fastapi import HTTPException
from websockets.exceptions import WebSocketException


PACKAGE = Path(__file__).resolve().parents[1]
COMMON = PACKAGE.parents[3] / 'common' / 'fleet_bridge_config'
sys.path[:0] = [str(COMMON), str(PACKAGE)]

from fleet_bridge_config.models import (
    CommandApiConfig,
    CommandConfig,
    FleetConfig,
    ServerConfig,
    VehicleConfig,
)
from foxglove_ros_worker.api import create_app


def fleet() -> FleetConfig:
    return FleetConfig(
        server=ServerConfig(
            domain_id=225,
            foxglove_port=8765,
            command_api=CommandApiConfig(host='127.0.0.1', port=8080),
        ),
        vehicles=(VehicleConfig(
            id='robot_1',
            foxglove_uri='ws://10.0.0.11:8766',
            enabled=True,
            command=CommandConfig(
                topic='/cmd_vel',
                message_type='geometry_msgs/msg/Twist',
                max_linear_x=0.3,
                max_angular_z=1.0,
                max_hold_ms=1000,
                publish_rate_hz=10.0,
            ),
        ),),
    )


class HandshakeFailingCommandClient:
    async def stop(self, _vehicle) -> None:
        raise WebSocketException('server rejected WebSocket connection: HTTP 400')


class CommandApiDeliveryErrorTest(unittest.TestCase):
    def test_websocket_handshake_error_returns_service_unavailable(self):
        app = create_app(fleet(), HandshakeFailingCommandClient())
        stop_endpoint = next(
            route.endpoint
            for route in app.routes
            if route.path == '/api/v1/robots/{robot_id}/stop'
        )

        with self.assertRaises(HTTPException) as raised:
            asyncio.run(stop_endpoint('robot_1'))

        self.assertEqual(raised.exception.status_code, 503)
        self.assertIn('command delivery failed', raised.exception.detail)


if __name__ == '__main__':
    unittest.main()
