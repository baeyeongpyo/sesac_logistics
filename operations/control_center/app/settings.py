"""Runtime configuration for the read-only dashboard service."""

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Mapping


@dataclass(frozen=True)
class Settings:
    """Immutable, environment-derived Control Center settings."""

    data_directory: Path
    map_directory: Path
    fleet_bridge_command_url: str
    request_timeout_sec: float

    @classmethod
    def from_environment(cls, environ: Mapping[str, str]) -> 'Settings':
        timeout = float(environ.get('CONTROL_CENTER_REQUEST_TIMEOUT_SEC', '3'))
        if timeout <= 0:
            raise ValueError('CONTROL_CENTER_REQUEST_TIMEOUT_SEC must be greater than zero')
        return cls(
            data_directory=Path(environ.get('CONTROL_CENTER_DATA_DIRECTORY', '/data')),
            map_directory=Path(environ.get('CONTROL_CENTER_MAP_DIRECTORY', '/maps')),
            fleet_bridge_command_url=environ.get(
                'FLEET_BRIDGE_COMMAND_URL',
                'http://host.docker.internal:8080',
            ).rstrip('/'),
            request_timeout_sec=timeout,
        )


def runtime_settings() -> Settings:
    """Construct settings from the process environment."""

    return Settings.from_environment(os.environ)
