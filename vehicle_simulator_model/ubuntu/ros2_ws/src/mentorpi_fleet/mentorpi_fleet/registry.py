"""Declarative fleet registry parsing and validation."""

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import yaml


VehicleKind = Literal['physical', 'simulation']
SAFE_DOMAIN_IDS = frozenset(range(0, 102)) | frozenset(range(215, 233))


class RegistryValidationError(ValueError):
    """Raised when a fleet registry cannot be used safely."""


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class VehicleSpec:
    vehicle_id: str
    kind: VehicleKind
    domain_id: int
    namespace: str
    profile: str
    enabled: bool
    spawn: SpawnPose | None = None
    nav_enabled: bool = False


@dataclass(frozen=True)
class FleetRegistry:
    control_domain: int
    profiles: tuple[str, ...]
    vehicles: tuple[VehicleSpec, ...]


def _require_mapping(value: object, path: str) -> dict:
    if not isinstance(value, dict):
        raise RegistryValidationError(f'{path} must be a mapping')
    return value


def _require_string(value: object, path: str) -> str:
    if not isinstance(value, str) or not value:
        raise RegistryValidationError(f'{path} must be a non-empty string')
    return value


def _require_int(value: object, path: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise RegistryValidationError(f'{path} must be an integer')
    return value


def _parse_spawn(value: object, path: str) -> SpawnPose:
    mapping = _require_mapping(value, path)
    coordinates: list[float] = []
    for key in ('x', 'y', 'z', 'yaw'):
        coordinate = mapping.get(key)
        if not isinstance(coordinate, (int, float)) or isinstance(coordinate, bool):
            raise RegistryValidationError(f'{path}.{key} must be a number')
        coordinates.append(float(coordinate))
    return SpawnPose(*coordinates)


def _parse_vehicle(value: object, index: int, profiles: set[str], control_domain: int) -> VehicleSpec:
    path = f'vehicles[{index}]'
    mapping = _require_mapping(value, path)
    vehicle_id = _require_string(mapping.get('id'), f'{path}.id')
    kind = _require_string(mapping.get('kind'), f'{path}.kind')
    if kind not in {'physical', 'simulation'}:
        raise RegistryValidationError(f'{path}.kind must be physical or simulation')
    domain_id = _require_int(mapping.get('domain_id'), f'{path}.domain_id')
    if domain_id not in SAFE_DOMAIN_IDS or domain_id == control_domain:
        raise RegistryValidationError(f'{path}.domain_id is not an allowed vehicle Domain ID')
    namespace = _require_string(mapping.get('namespace'), f'{path}.namespace')
    if not namespace.startswith('/') or namespace == '/':
        raise RegistryValidationError(f'{path}.namespace must be an absolute non-root namespace')
    profile = _require_string(mapping.get('profile'), f'{path}.profile')
    if profile not in profiles:
        raise RegistryValidationError(f'{path}.profile is not declared in profiles')
    enabled = mapping.get('enabled')
    if not isinstance(enabled, bool):
        raise RegistryValidationError(f'{path}.enabled must be a boolean')

    if kind == 'simulation':
        spawn = _parse_spawn(mapping.get('spawn'), f'{path}.spawn')
        nav_enabled = mapping.get('nav_enabled', False)
        if not isinstance(nav_enabled, bool):
            raise RegistryValidationError(f'{path}.nav_enabled must be a boolean')
    else:
        if 'spawn' in mapping or 'nav_enabled' in mapping:
            raise RegistryValidationError(f'{path} physical vehicles cannot declare simulation settings')
        spawn = None
        nav_enabled = False
    return VehicleSpec(vehicle_id, kind, domain_id, namespace, profile, enabled, spawn, nav_enabled)


def load_registry(path: Path) -> FleetRegistry:
    """Parse a complete registry, rejecting any unsafe partial configuration."""
    try:
        document = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError) as error:
        raise RegistryValidationError(f'cannot read registry {path}: {error}') from error
    root = _require_mapping(document, 'registry')
    control_domain = _require_int(root.get('control_domain'), 'control_domain')
    if control_domain not in SAFE_DOMAIN_IDS:
        raise RegistryValidationError('control_domain is not a safe ROS Domain ID')
    raw_profiles = root.get('profiles')
    if not isinstance(raw_profiles, list) or not raw_profiles:
        raise RegistryValidationError('profiles must be a non-empty list')
    profiles = tuple(_require_string(profile, 'profiles[]') for profile in raw_profiles)
    if len(set(profiles)) != len(profiles):
        raise RegistryValidationError('profiles must not contain duplicates')
    raw_vehicles = root.get('vehicles')
    if not isinstance(raw_vehicles, list):
        raise RegistryValidationError('vehicles must be a list')
    vehicles = tuple(
        _parse_vehicle(vehicle, index, set(profiles), control_domain)
        for index, vehicle in enumerate(raw_vehicles)
    )
    for attribute in ('vehicle_id', 'namespace', 'domain_id'):
        values = [getattr(vehicle, attribute) for vehicle in vehicles]
        if len(set(values)) != len(values):
            raise RegistryValidationError(f'vehicles must not share {attribute}')
    return FleetRegistry(control_domain, profiles, vehicles)


def enabled_vehicles(registry: FleetRegistry, kind: VehicleKind | None = None) -> list[VehicleSpec]:
    """Return enabled vehicles, optionally restricted to one vehicle kind."""
    return [
        vehicle for vehicle in registry.vehicles
        if vehicle.enabled and (kind is None or vehicle.kind == kind)
    ]
