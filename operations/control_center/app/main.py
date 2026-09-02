"""FastAPI entrypoint for the local operations dashboard."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field
import yaml

from .command_relay import CommandRelay, CommandRelayError, RelayResponse
from .reader import SnapshotReader
from .settings import Settings, runtime_settings


MAP_CONFIG_FILENAME = 'map_0825.yaml'


class ManualCommand(BaseModel):
    """A short-lived vehicle-native manual command."""

    model_config = ConfigDict(extra='forbid')

    linear_x: float = Field(ge=-1.0, le=1.0)
    linear_y: float = Field(default=0.0, ge=-1.0, le=1.0)
    angular_z: float = Field(ge=-1.0, le=1.0)
    hold_ms: int = Field(default=300, ge=100, le=1000)


class InitialPoseCommand(BaseModel):
    """Initial AMCL pose selected by a click-and-drag on the map."""

    model_config = ConfigDict(extra='forbid')

    x: float = Field(ge=-1000.0, le=1000.0)
    y: float = Field(ge=-1000.0, le=1000.0)
    yaw: float = Field(ge=-math.pi, le=math.pi)


def create_app(
    settings: Settings | None = None,
    *,
    reader: SnapshotReader | None = None,
    relay: CommandRelay | None = None,
) -> FastAPI:
    """Create the same-origin dashboard, read-only reader, and API relay."""

    active_settings = settings or runtime_settings()
    app = FastAPI(title='Operations Control Center', version='0.1.0')
    app.state.settings = active_settings
    app.state.reader = reader or SnapshotReader(active_settings.data_directory)
    app.state.relay = relay or CommandRelay(
        active_settings.fleet_bridge_command_url,
        request_timeout_sec=active_settings.request_timeout_sec,
    )

    @app.get('/healthz')
    def healthz() -> dict[str, str]:
        return {'status': 'ok'}

    @app.get('/api/snapshot')
    def snapshot() -> dict[str, Any]:
        return app.state.reader.snapshot()

    @app.get('/api/map/config')
    def map_config() -> dict[str, Any]:
        config_path = active_settings.map_directory / MAP_CONFIG_FILENAME
        try:
            config = yaml.safe_load(config_path.read_text(encoding='utf-8'))
        except (OSError, yaml.YAMLError) as error:
            raise HTTPException(
                status_code=503,
                detail='운영 지도 설정을 읽을 수 없습니다.',
            ) from error
        if not isinstance(config, dict) or not _valid_map_config(config):
            raise HTTPException(status_code=503, detail='운영 지도 설정 형식이 올바르지 않습니다.')
        return {
            'name': MAP_CONFIG_FILENAME.removesuffix('.yaml'),
            'resolution': config['resolution'],
            'origin': config['origin'],
            'negate': config.get('negate', 0),
            'occupied_thresh': config.get('occupied_thresh'),
            'free_thresh': config.get('free_thresh'),
        }

    @app.get('/api/map/pgm')
    def map_pgm() -> FileResponse:
        image_path = active_settings.map_directory / 'map_0825.pgm'
        if not image_path.is_file():
            raise HTTPException(status_code=503, detail='운영 PGM 지도를 읽을 수 없습니다.')
        return FileResponse(
            image_path,
            media_type='application/octet-stream',
            filename='map_0825.pgm',
        )

    @app.post('/api/vehicles/{robot_id}/manual')
    def manual(robot_id: str, command: ManualCommand) -> JSONResponse:
        return _relay_response(
            app.state.relay,
            'send_manual',
            robot_id,
            linear_x=command.linear_x,
            linear_y=command.linear_y,
            angular_z=command.angular_z,
            hold_ms=command.hold_ms,
        )

    @app.post('/api/vehicles/{robot_id}/stop')
    def stop(robot_id: str) -> JSONResponse:
        return _relay_response(app.state.relay, 'send_stop', robot_id)

    @app.post('/api/vehicles/{robot_id}/navigation/cancel')
    def navigation_cancel(robot_id: str) -> JSONResponse:
        return _relay_response(app.state.relay, 'send_navigation_cancel', robot_id)

    @app.post('/api/vehicles/{robot_id}/operation/idle')
    def operation_idle(robot_id: str) -> JSONResponse:
        return _relay_response(app.state.relay, 'send_operation_idle', robot_id)

    @app.post('/api/vehicles/{robot_id}/initial-pose')
    def initial_pose(robot_id: str, command: InitialPoseCommand) -> JSONResponse:
        return _relay_response(
            app.state.relay,
            'send_initial_pose',
            robot_id,
            x=command.x,
            y=command.y,
            yaw=command.yaw,
        )

    static_directory = Path(__file__).parent.parent / 'static'
    app.mount('/assets', StaticFiles(directory=static_directory), name='assets')

    @app.get('/', include_in_schema=False)
    def dashboard() -> FileResponse:
        return FileResponse(static_directory / 'index.html')

    return app


def _valid_map_config(config: dict[str, Any]) -> bool:
    origin = config.get('origin')
    return (
        isinstance(config.get('resolution'), (int, float))
        and config['resolution'] > 0
        and isinstance(origin, list)
        and len(origin) >= 3
        and all(isinstance(value, (int, float)) for value in origin[:3])
    )


def _relay_response(
    relay: CommandRelay,
    method_name: str,
    robot_id: str,
    **kwargs: Any,
) -> JSONResponse:
    try:
        response: RelayResponse = getattr(relay, method_name)(robot_id, **kwargs)
    except ValueError as error:
        raise HTTPException(status_code=422, detail=str(error)) from error
    except CommandRelayError as error:
        raise HTTPException(status_code=503, detail=str(error)) from error
    return JSONResponse(status_code=response.status_code, content=response.body)


app = create_app()
