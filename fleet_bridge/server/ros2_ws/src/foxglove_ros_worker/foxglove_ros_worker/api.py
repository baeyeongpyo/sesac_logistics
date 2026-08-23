"""Test-only REST command API for vehicle Foxglove Bridges."""

import argparse
import os
from typing import Any

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, ConfigDict, Field
from websockets.exceptions import WebSocketException

from fleet_bridge_config.loader import load_fleet
from fleet_bridge_config.models import FleetConfig, VehicleConfig

from .command import (
    CommandValidationError,
    FoxgloveCommandClient,
    StopDeliveryError,
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


class RosTimeRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')

    sec: int = Field(..., ge=0, le=2_147_483_647)
    nanosec: int = Field(..., ge=0, le=999999999)


class HeaderRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')

    stamp: RosTimeRequest
    frame_id: str = Field(..., min_length=1)


class PositionRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)

    x: float
    y: float
    z: float


class OrientationRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)

    x: float
    y: float
    z: float
    w: float


class PoseRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')

    position: PositionRequest
    orientation: OrientationRequest


class GoalPoseRequest(BaseModel):
    """ROS ``geometry_msgs/msg/PoseStamped`` JSON shape."""

    model_config = ConfigDict(extra='forbid')

    header: HeaderRequest
    pose: PoseRequest


class CommandAccepted(BaseModel):
    robot_id: str
    command: str
    hold_ms: int | None = None
    nav2_return_code: int | None = None
    nav2_goals_canceling: int | None = None


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
            '테스트 목적의 cmd_vel, Nav2 goal/cancel 및 통합 stop API입니다. '
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
        except (ConnectionError, OSError, ProtocolError, WebSocketException) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(
            robot_id=robot_id,
            command='cmd_vel',
            hold_ms=request.hold_ms,
        )

    @app.post(
        '/api/v1/robots/{robot_id}/nav2/goal_pose',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Send a Nav2 PoseStamped goal',
    )
    async def nav2_goal_pose(
        robot_id: str,
        request: GoalPoseRequest,
    ) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            await command_client.send_goal_pose(vehicle, request.model_dump())
        except (
            ConnectionError,
            OSError,
            ProtocolError,
            WebSocketException,
        ) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(robot_id=robot_id, command='nav2_goal_pose')

    @app.post(
        '/api/v1/robots/{robot_id}/nav2/cancel',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Cancel all active Nav2 NavigateToPose goals',
    )
    async def nav2_cancel(robot_id: str) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            result = await command_client.cancel_navigation(vehicle)
        except (
            ConnectionError,
            OSError,
            ProtocolError,
            WebSocketException,
        ) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(
            robot_id=robot_id,
            command='nav2_cancel',
            nav2_return_code=result.return_code,
            nav2_goals_canceling=result.goals_canceling,
        )

    @app.post(
        '/api/v1/robots/{robot_id}/stop',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Cancel Nav2 and send an immediate zero cmd_vel command',
    )
    async def stop(robot_id: str) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            result = await command_client.stop(vehicle)
        except (
            ConnectionError,
            OSError,
            ProtocolError,
            StopDeliveryError,
            WebSocketException,
        ) as error:
            raise _delivery_error(error) from error
        return CommandAccepted(
            robot_id=robot_id,
            command='stop',
            nav2_return_code=result.return_code,
            nav2_goals_canceling=result.goals_canceling,
        )

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
