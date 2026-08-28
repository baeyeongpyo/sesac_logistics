"""REST command API that delegates commands to vehicle HTTP command APIs."""

import argparse
import os
from typing import Annotated, Any

from fastapi import Body, FastAPI, HTTPException, Path, status
from fastapi.responses import JSONResponse

from fleet_bridge_config.loader import load_fleet
from fleet_bridge_config.models import FleetConfig, VehicleConfig

from .command import (
    VehicleCommandApiError,
    VehicleCommandClient,
    VehicleCommandTransportError,
)


RobotId = Annotated[
    str,
    Path(
        title='차량 ID',
        description='명령을 전달하거나 상태를 조회할 차량 ID입니다.',
        examples=['robot_1'],
    ),
]

VEHICLE_COMMAND_TAG = {
    'name': 'vehicle-command relay',
    'description': (
        'Fleet Manager는 등록된 차량의 vehicle_command_api로 요청과 응답을 '
        '변경하지 않고 중계합니다.'
    ),
}


def _relay_error_responses() -> dict[int, dict[str, Any]]:
    return {
        404: {
            'description': '등록되지 않았거나 비활성화된 차량입니다.',
            'content': {
                'application/json': {
                    'example': {'detail': 'unknown robot: robot_1'},
                },
            },
        },
        503: {
            'description': '차량 command API에 연결할 수 없습니다.',
            'content': {
                'application/json': {
                    'example': {
                        'detail': 'command delivery failed: vehicle command API unavailable',
                    },
                },
            },
        },
    }


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


def create_app(fleet: FleetConfig, command_client: Any) -> FastAPI:
    """Create the documented Fleet Manager API with injected vehicle transport."""

    app = FastAPI(
        title='Fleet Manager Vehicle Command API',
        version='1.0.0',
        description=(
            'Fleet Manager가 등록된 차량의 vehicle_command_api를 중계하는 API입니다. '
            '각 경로는 차량 API의 요청 본문, HTTP 상태 코드, JSON 응답 본문을 변경하지 않습니다. '
            '아래 요청 예시는 차량-native 형식이며, 실제 허용 범위와 최신 응답 형식은 '
            '차량별 `/api/v1/vehicle-command/{robot_id}/openapi.json`에서 확인합니다.'
        ),
        openapi_tags=[
            VEHICLE_COMMAND_TAG,
            {
                'name': 'health',
                'description': 'Fleet Manager Command API 프로세스 상태를 확인합니다.',
            },
        ],
    )

    @app.get(
        '/healthz',
        tags=['health'],
        summary='Fleet Manager Command API 상태 조회',
        description='Fleet Manager의 중계 API 프로세스가 요청을 받을 준비가 되었는지 확인합니다.',
        responses={
            200: {
                'description': '중계 API 프로세스가 정상입니다.',
                'content': {'application/json': {'example': {'status': 'ok'}}},
            },
        },
    )
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
        summary='차량 Command API 상태 조회',
        description='선택한 차량의 Command API 생존 상태를 그대로 중계합니다.',
        responses={
            200: {
                'description': '차량 Command API가 정상입니다.',
                'content': {'application/json': {'example': {'status': 'ok'}}},
            },
            **_relay_error_responses(),
        },
    )
    async def vehicle_command_healthz(robot_id: RobotId) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/healthz')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/openapi.json',
        tags=['vehicle-command relay'],
        summary='차량 Command API OpenAPI 문서 조회',
        description=(
            '선택한 차량이 제공하는 원본 OpenAPI 문서를 중계합니다. 차량별 제한값과 '
            '최신 요청·응답 형식은 이 문서를 기준으로 확인합니다.'
        ),
        responses={
            200: {
                'description': '차량이 제공하는 원본 OpenAPI 문서입니다.',
            },
            **_relay_error_responses(),
        },
    )
    async def vehicle_command_openapi(robot_id: RobotId) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/openapi.json')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/operation-status',
        tags=['vehicle-command relay'],
        summary='차량 작업 상태 조회',
        description='선택한 차량의 현재 작업 상태를 그대로 중계합니다.',
        responses={
            200: {
                'description': '차량의 현재 작업 상태입니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                            'state': 'NAVIGATING',
                            'detail': 'NAVIGATION_GOAL_ACCEPTED',
                        },
                    },
                },
            },
            **_relay_error_responses(),
        },
    )
    async def vehicle_operation_status(robot_id: RobotId) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/v1/operation-status')

    @app.get(
        '/api/v1/vehicle-command/{robot_id}/vehicle-status',
        tags=['vehicle-command relay'],
        summary='차량 배터리·작업 상태 조회',
        description='선택한 차량의 식별자, 최근 배터리 측정값, 현재 작업 상태를 그대로 중계합니다.',
        responses={
            200: {
                'description': '차량 상태입니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'robot_id': 'robot_1',
                            'battery': {
                                'raw_value': 8354,
                                'received_at': '2026-08-27T02:30:00.000Z',
                                'stale': False,
                            },
                            'operation': {
                                'operation_id': None,
                                'state': 'IDLE',
                                'detail': 'READY',
                            },
                        },
                    },
                },
            },
            **_relay_error_responses(),
        },
    )
    async def vehicle_status(robot_id: RobotId) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'GET', '/v1/vehicle-status')

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/cmd-vel',
        tags=['vehicle-command relay'],
        summary='수동 속도 명령 전달',
        description=(
            '선택한 차량에 제한 시간 수동 속도 명령을 전달합니다. 차량은 Nav2 주행 중 '
            '수동 명령을 받으면 기존 주행을 취소하며, 허용 속도 범위는 차량 OpenAPI를 따릅니다.'
        ),
        responses={
            202: {
                'description': '차량이 수동 속도 명령을 수락했습니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'state': 'MANUAL',
                            'linear_x': 0.2,
                            'angular_z': 0.0,
                            'hold_ms': 500,
                        },
                    },
                },
            },
            422: {'description': '차량이 속도 명령 형식 또는 범위를 거부했습니다.'},
            **_relay_error_responses(),
        },
    )
    async def vehicle_cmd_vel(
        robot_id: RobotId,
        payload: Any = Body(
            default=None,
            description='차량-native 수동 속도 명령입니다.',
            openapi_examples={
                'sample': {
                    'summary': '직진 수동 주행',
                    'description': '0.2 m/s로 500 ms 동안 직진합니다.',
                    'value': {
                        'linear_x': 0.2,
                        'angular_z': 0.0,
                        'hold_ms': 500,
                    },
                },
            },
        ),
    ) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'POST', '/v1/cmd-vel', payload)

    @app.post(
        '/api/v1/vehicle-command/{robot_id}/navigation/goals',
        tags=['vehicle-command relay'],
        summary='Nav2 목표 주행 요청 전달',
        description=(
            '선택한 차량의 Nav2에 map 좌표계 목표를 전달합니다. 차량이 생성한 '
            '`operation_id`로 이후 취소 또는 상태 조회 대상을 지정할 수 있습니다.'
        ),
        responses={
            202: {
                'description': '차량이 Nav2 목표를 수락했습니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                            'state': 'NAVIGATING',
                        },
                    },
                },
            },
            422: {'description': '차량이 목표 좌표 형식을 거부했습니다.'},
            503: {'description': '차량 Nav2 action server 또는 차량 API를 사용할 수 없습니다.'},
            **_relay_error_responses(),
        },
    )
    async def vehicle_navigation_goal(
        robot_id: RobotId,
        payload: Any = Body(
            default=None,
            description='차량-native Nav2 목표 좌표입니다.',
            openapi_examples={
                'sample': {
                    'summary': 'map 좌표계 목표 주행',
                    'value': {
                        'frame_id': 'map',
                        'x': 1.5,
                        'y': 0.0,
                        'yaw': 0.0,
                    },
                },
            },
        ),
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
        summary='Nav2 주행 취소 요청 전달',
        description=(
            '선택한 차량의 활성 Nav2 작업을 취소합니다. `operation_id`를 생략하면 '
            '현재 활성 작업을 취소하고, 지정하면 해당 활성 작업만 취소합니다.'
        ),
        responses={
            202: {
                'description': '차량이 Nav2 취소 요청을 수락했습니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                            'state': 'CANCELLING',
                        },
                    },
                },
            },
            409: {'description': '지정한 작업이 현재 활성 Nav2 작업이 아닙니다.'},
            **_relay_error_responses(),
        },
    )
    async def vehicle_navigation_cancel(
        robot_id: RobotId,
        payload: Any = Body(
            default=None,
            description='취소할 차량-native Nav2 작업 ID입니다. 생략하면 현재 활성 작업을 취소합니다.',
            openapi_examples={
                'sample': {
                    'summary': '특정 Nav2 작업 취소',
                    'value': {'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58'},
                },
                'active-operation': {
                    'summary': '현재 활성 Nav2 작업 취소',
                    'value': {},
                },
            },
        ),
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
        summary='AMCL 초기 위치 설정 전달',
        description=(
            '선택한 차량의 AMCL 초기 위치를 설정합니다. 차량이 Nav2 또는 수동 주행 중이면 '
            '차량이 409로 거부하므로, 호출자가 먼저 별도 stop을 성공시켜야 합니다.'
        ),
        responses={
            202: {
                'description': '차량이 AMCL 초기 위치를 발행했습니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                            'state': 'INITIAL_POSE_PUBLISHED',
                            'frame_id': 'map',
                            'x': 0.0,
                            'y': 0.0,
                            'yaw': 0.0,
                        },
                    },
                },
            },
            409: {'description': '차량이 주행 중이어서 초기 위치 설정을 거부했습니다.'},
            422: {'description': '차량이 초기 위치 형식을 거부했습니다.'},
            **_relay_error_responses(),
        },
    )
    async def vehicle_initial_pose(
        robot_id: RobotId,
        payload: Any = Body(
            default=None,
            description='차량-native AMCL 초기 위치입니다.',
            openapi_examples={
                'sample': {
                    'summary': 'map 원점으로 초기 위치 설정',
                    'value': {
                        'frame_id': 'map',
                        'x': 0.0,
                        'y': 0.0,
                        'yaw': 0.0,
                    },
                },
            },
        ),
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
        summary='즉시 정지 요청 전달',
        description=(
            '선택한 차량에 요청 본문 없이 즉시 정지를 전달합니다. 차량은 속도 0을 우선 적용하고 '
            '활성 Nav2 작업이 있으면 취소를 함께 요청하며 자동 재개하지 않습니다.'
        ),
        responses={
            200: {
                'description': '차량이 즉시 정지 요청을 처리했습니다.',
                'content': {
                    'application/json': {
                        'example': {
                            'operation_id': '1c3e8b56-7c4d-4d9e-98ac-ced38f8c8a58',
                            'state': 'STOPPED',
                            'cancel_requested': True,
                        },
                    },
                },
            },
            **_relay_error_responses(),
        },
    )
    async def vehicle_stop(robot_id: RobotId) -> JSONResponse:
        return await relay_vehicle_command(robot_id, 'POST', '/v1/stop')

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
