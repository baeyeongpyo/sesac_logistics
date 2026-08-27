"""REST command API that delegates commands to vehicle HTTP command APIs."""

import argparse
import os
from typing import Any

from fastapi import Body, FastAPI, HTTPException, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ConfigDict, Field

from fleet_bridge_config.loader import load_fleet
from fleet_bridge_config.models import FleetConfig, VehicleConfig

from .command import (
    CommandValidationError,
    VehicleCommandApiError,
    VehicleCommandClient,
    VehicleCommandTransportError,
    validate_command,
)


class CmdVelRequest(BaseModel):
    """Planar velocity command accepted by the Fleet Manager endpoint."""

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


class NavigationCancelRequest(BaseModel):
    """Optional vehicle-generated operation ID to cancel a specific goal."""

    model_config = ConfigDict(extra='forbid')

    operation_id: str | None = Field(default=None, min_length=1)


class CommandAccepted(BaseModel):
    robot_id: str
    command: str
    hold_ms: int | None = None
    operation_id: str | None = None
    state: str | None = None
    cancel_requested: bool | None = None


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
    if isinstance(error, VehicleCommandApiError):
        status_code = error.status_code
        if status_code < 400 or status_code > 599:
            status_code = status.HTTP_502_BAD_GATEWAY
        return HTTPException(
            status_code=status_code,
            detail=f'vehicle command API rejected command: {error.detail}',
        )
    return HTTPException(
        status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
        detail=f'command delivery failed: {error}',
    )


def _command_accepted(
    robot_id: str,
    command: str,
    result: dict[str, Any],
    *,
    hold_ms: int | None = None,
) -> CommandAccepted:
    operation_id = result.get('operation_id')
    state = result.get('state')
    cancel_requested = result.get('cancel_requested')
    return CommandAccepted(
        robot_id=robot_id,
        command=command,
        hold_ms=hold_ms,
        operation_id=operation_id if isinstance(operation_id, str) else None,
        state=state if isinstance(state, str) else None,
        cancel_requested=(
            cancel_requested if isinstance(cancel_requested, bool) else None
        ),
    )


def create_app(fleet: FleetConfig, command_client: Any) -> FastAPI:
    """Create the documented Fleet Manager API with injected vehicle transport."""

    app = FastAPI(
        title='Fleet Manager Vehicle Command API',
        version='1.0.0',
        description=(
            '차량별 vehicle_command_api에 cmd_vel, Nav2 goal/cancel, stop을 전달합니다. '
            '운영 환경에서는 인증과 네트워크 접근 제어를 추가해야 합니다.'
        ),
    )

    @app.get('/healthz', tags=['health'])
    async def healthz() -> dict[str, str]:
        return {'status': 'ok'}

    async def relay_vehicle_command(
        robot_id: str,
        method: str,
        vehicle_path: str,
        payload: Any | None = None,
    ) -> JSONResponse:
        """Forward one allowlisted vehicle API request without changing its result."""

        vehicle = _active_vehicle(fleet, robot_id)
        try:
            response = await command_client.relay(
                vehicle,
                method,
                vehicle_path,
                payload,
            )
        except VehicleCommandApiError as error:
            return JSONResponse(
                status_code=error.status_code,
                content=error.body,
            )
        except (
            ConnectionError,
            OSError,
            VehicleCommandTransportError,
        ) as error:
            raise _delivery_error(error) from error
        return JSONResponse(status_code=response.status_code, content=response.body)

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/healthz',
        tags=['vehicle-command relay'],
        summary='Relay vehicle command API health status',
    )
    async def vehicle_command_healthz(robot_id: str) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/healthz')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/openapi.json',
        tags=['vehicle-command relay'],
        summary='Relay the vehicle command API OpenAPI document',
    )
    async def vehicle_command_openapi(robot_id: str) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/openapi.json')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/operation-status',
        tags=['vehicle-command relay'],
        summary='Relay current vehicle operation status',
    )
    async def vehicle_operation_status(robot_id: str) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/v1/operation-status')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/vehicle-status',
        tags=['vehicle-command relay'],
        summary='Relay vehicle status including battery and operation state',
    )
    async def vehicle_status(robot_id: str) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/v1/vehicle-status')

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/cmd-vel',
        tags=['vehicle-command relay'],
        summary='Relay a vehicle-native cmd_vel request',
    )
    async def vehicle_cmd_vel(
        robot_id: str,
        payload: Any = Body(default=None),
    ) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'POST', '/v1/cmd-vel', payload)

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/navigation/goals',
        tags=['vehicle-command relay'],
        summary='Relay a vehicle-native Nav2 goal request',
    )
    async def vehicle_navigation_goal(
        robot_id: str,
        payload: Any = Body(default=None),
    ) -> JSONResponse:
        return await relay_vehicle_command(
            robot_id,
            'POST',
            '/v1/navigation/goals',
            payload,
        )

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/navigation/cancel',
        tags=['vehicle-command relay'],
        summary='Relay a vehicle-native Nav2 cancel request',
    )
    async def vehicle_navigation_cancel(
        robot_id: str,
        payload: Any = Body(default=None),
    ) -> JSONResponse:
        return await relay_vehicle_command(
            robot_id,
            'POST',
            '/v1/navigation/cancel',
            payload,
        )

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/localization/initial-pose',
        tags=['vehicle-command relay'],
        summary='Relay a vehicle-native AMCL initial-pose request',
    )
    async def vehicle_initial_pose(
        robot_id: str,
        payload: Any = Body(default=None),
    ) -> JSONResponse:
        return await relay_vehicle_command(
            robot_id,
            'POST',
            '/v1/localization/initial-pose',
            payload,
        )

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/stop',
        tags=['vehicle-command relay'],
        summary='Relay a vehicle-native stop request',
    )
    async def vehicle_stop(
        robot_id: str,
        payload: Any = Body(default=None),
    ) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'POST', '/v1/stop', payload)

    @app.post(
        '/api/v1/robots/{robot_id}/cmd_vel',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Send a bounded cmd_vel command to the vehicle API',
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
            result = await command_client.send_twist(
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
        except (
            ConnectionError,
            OSError,
            VehicleCommandApiError,
            VehicleCommandTransportError,
        ) as error:
            raise _delivery_error(error) from error
        return _command_accepted(
            robot_id=robot_id,
            command='cmd_vel',
            hold_ms=request.hold_ms,
            result=result,
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
            result = await command_client.send_goal_pose(
                vehicle,
                request.model_dump(),
            )
        except (
            ConnectionError,
            OSError,
            VehicleCommandApiError,
            VehicleCommandTransportError,
        ) as error:
            raise _delivery_error(error) from error
        return _command_accepted(robot_id, 'nav2_goal_pose', result)

    @app.post(
        '/api/v1/robots/{robot_id}/nav2/cancel',
        status_code=status.HTTP_202_ACCEPTED,
        response_model=CommandAccepted,
        tags=['commands'],
        summary='Cancel all active Nav2 NavigateToPose goals',
    )
    async def nav2_cancel(
        robot_id: str,
        request: NavigationCancelRequest | None = None,
    ) -> CommandAccepted:
        vehicle = _active_vehicle(fleet, robot_id)
        try:
            result = await command_client.cancel_navigation(
                vehicle,
                request.operation_id if request is not None else None,
            )
        except (
            ConnectionError,
            OSError,
            VehicleCommandApiError,
            VehicleCommandTransportError,
        ) as error:
            raise _delivery_error(error) from error
        return _command_accepted(robot_id, 'nav2_cancel', result)

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
            VehicleCommandApiError,
            VehicleCommandTransportError,
        ) as error:
            raise _delivery_error(error) from error
        return _command_accepted(robot_id, 'stop', result)

    return app


def _arguments(argv=None):
    parser = argparse.ArgumentParser(
        description='Serve the Fleet Manager vehicle command REST API.',
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
        create_app(fleet, VehicleCommandClient()),
        host=args.host or fleet.server.command_api.host,
        port=args.port or fleet.server.command_api.port,
    )


if __name__ == '__main__':
    main()
