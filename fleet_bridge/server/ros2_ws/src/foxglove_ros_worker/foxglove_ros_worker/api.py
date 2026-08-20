"""Test-only REST command API for vehicle Foxglove Bridges."""

import argparse
import os
from typing import Any

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, ConfigDict, Field

from fleet_bridge_config.loader import load_fleet
from fleet_bridge_config.models import FleetConfig, VehicleConfig

from .command import (
    CommandValidationError,
    FoxgloveCommandClient,
    validate_command,
)
from .protocol import ProtocolError


class CmdVelRequest(BaseModel):
    """Planar velocity command accepted by the test command endpoint."""

    model_config = ConfigDict(extra='forbid')

    linear_x: float = Field(
        ...,
        description='Forward/backward velocity in metres per second.',
        examples=[0.1],
    )
    angular_z: float = Field(
        ...,
        description='Yaw angular velocity in radians per second.',
        examples=[0.0],
    )
    hold_ms: int = Field(
        ...,
        ge=1,
        le=60000,
        description='Command hold duration; a zero Twist follows automatically.',
        examples=[300],
    )


class CommandAccepted(BaseModel):
    robot_id: str
    command: str
    hold_ms: int | None = None


def _active_vehicle(fleet: FleetConfig, robot_id: str) -> VehicleConfig:
    try:
        vehicle = fleet.vehicle(robot_id)
    except KeyError as error:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f'unknown robot: {robot_id}',
        ) from error
    if not vehicle.enabled:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f'robot is disabled: {robot_id}',
        )
    return vehicle


def _delivery_error(error: Exception) -> HTTPException:
    return HTTPException(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        detail=f'command delivery failed: {error}',
    )


def create_app(fleet: FleetConfig, command_client: Any) -> FastAPI:
    """Create the documented test command API with injected command transport."""

    app = FastAPI(
        title='Fleet Bridge Test Command API',
        version='1.0.0',
        description=(
            '테스트 목적의 cmd_vel 및 stop API입니다. '
            '운영 환경에서는 인증과 네트워크 접근 제어를 추가해야 합니다.'
        ),
    )

    @app.get('/healthz', tags=['health'])
    async def healthz() -> dict[str, str]:
        return {'status': 'ok'}

    @app.post(
        '/api/v1/robots/{robot_id}/cmd_vel',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Send a bounded cmd_vel test command',
    )
    async def cmd_vel(robot_id: str, request: CmdVelRequest) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            validate_command(
                vehicle,
                request.linear_x,
                request.angular_z,
                request.hold_ms,
            )
            await command_client.send_twist(
                vehicle,
                request.linear_x,
                request.angular_z,
                request.hold_ms,
            )
        except CommandValidationError as error:
            raise HTTPException(
                status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
                detail=str(error),
            ) from error
        except (ConnectionError, OSError, ProtocolError) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(
            robot_id=robot_id,
            command='cmd_vel',
            hold_ms=request.hold_ms,
        )

    @app.post(
        '/api/v1/robots/{robot_id}/stop',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Send an immediate zero cmd_vel command',
    )
    async def stop(robot_id: str) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            await command_client.stop(vehicle)
        except (ConnectionError, OSError, ProtocolError) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(robot_id=robot_id, command='stop')

    return app


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Serve the Fleet Bridge test command REST API.',
    )
    parser.add_argument(
        '--fleet-config',
        default=os.environ.get('FLEET_CONFIG', '/config/fleet.yaml'),
    )
    parser.add_argument('--host', default=os.environ.get('COMMAND_API_HOST'))
    parser.add_argument(
        '--port',
        type=int,
        default=(
            int(os.environ['COMMAND_API_PORT'])
            if os.environ.get('COMMAND_API_PORT')
            else None
        ),
    )
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = _arguments(argv)
    fleet = load_fleet(args.fleet_config, os.environ)
    import uvicorn

    uvicorn.run(
        create_app(fleet, FoxgloveCommandClient()),
        host=args.host or fleet.server.command_api.host,
        port=args.port or fleet.server.command_api.port,
    )


if __name__ == '__main__':
    main()
