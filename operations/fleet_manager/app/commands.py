from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import httpx
import yaml


SUPPORTED_CAPABILITIES = frozenset(
    {"navigate", "auto_dock", "stop", "report_status"}
)


class UnknownVehicleError(KeyError):
    pass


class UnsupportedCapabilityError(ValueError):
    pass


class BridgeUnavailableError(ConnectionError):
    pass


@dataclass(frozen=True)
class RelayResponse:
    status_code: int
    body: Any | None


@dataclass(frozen=True)
class ModelBridge:
    id: str
    bridge_url: str
    capabilities: frozenset[str]


@dataclass(frozen=True)
class RegisteredVehicle:
    id: str
    model: ModelBridge


class BridgeCommandGateway(Protocol):
    def relay(
        self, vehicle: RegisteredVehicle, path: str, payload: dict[str, Any]
    ) -> RelayResponse: ...


class VehicleRegistry:
    def __init__(self, vehicles: dict[str, RegisteredVehicle]) -> None:
        self._vehicles = vehicles

    def require(self, robot_id: str, capability: str) -> RegisteredVehicle:
        vehicle = self._vehicles.get(robot_id)
        if vehicle is None:
            raise UnknownVehicleError(robot_id)
        if capability not in vehicle.model.capabilities:
            raise UnsupportedCapabilityError(
                f"vehicle {robot_id} does not support {capability}"
            )
        return vehicle


class BridgeCommandClient:
    def __init__(self, *, client: httpx.Client | None = None) -> None:
        self._client = client or httpx.Client(timeout=5)
        self._owns_client = client is None

    def close(self) -> None:
        if self._owns_client:
            self._client.close()

    def relay(
        self, vehicle: RegisteredVehicle, path: str, payload: dict[str, Any]
    ) -> RelayResponse:
        try:
            response = self._client.post(
                f"{vehicle.model.bridge_url.rstrip('/')}{path}", json=payload
            )
        except httpx.RequestError as error:
            raise BridgeUnavailableError(str(error)) from error
        return RelayResponse(response.status_code, _response_body(response))


def load_vehicle_registry(path: str | Path) -> VehicleRegistry:
    try:
        document = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"unable to load vehicle registry: {error}") from error
    if not isinstance(document, dict):
        raise ValueError("vehicle registry must be a mapping")
    _require_keys(document, {"models", "vehicles"}, "vehicle registry")
    raw_models = _list(document["models"], "models")
    models: dict[str, ModelBridge] = {}
    for index, raw_model in enumerate(raw_models):
        location = f"models[{index}]"
        model = _mapping(raw_model, location)
        _require_keys(model, {"id", "bridge_url", "capabilities"}, location)
        model_id = _string(model["id"], f"{location}.id")
        if model_id in models:
            raise ValueError(f"duplicate model id: {model_id}")
        bridge_url = _string(model["bridge_url"], f"{location}.bridge_url")
        if not bridge_url.startswith(("http://", "https://")):
            raise ValueError(f"{location}.bridge_url must be an HTTP URL")
        capabilities = frozenset(
            _string(value, f"{location}.capabilities[{capability_index}]")
            for capability_index, value in enumerate(
                _list(model["capabilities"], f"{location}.capabilities")
            )
        )
        if not capabilities:
            raise ValueError(f"{location}.capabilities must not be empty")
        unsupported = capabilities - SUPPORTED_CAPABILITIES
        if unsupported:
            raise ValueError(
                f"{location}.capabilities contains unsupported values: "
                f"{', '.join(sorted(unsupported))}"
            )
        models[model_id] = ModelBridge(model_id, bridge_url, capabilities)

    vehicles: dict[str, RegisteredVehicle] = {}
    for index, raw_vehicle in enumerate(_list(document["vehicles"], "vehicles")):
        location = f"vehicles[{index}]"
        vehicle = _mapping(raw_vehicle, location)
        _require_keys(vehicle, {"id", "model"}, location)
        vehicle_id = _string(vehicle["id"], f"{location}.id")
        if vehicle_id in vehicles:
            raise ValueError(f"duplicate vehicle id: {vehicle_id}")
        model_id = _string(vehicle["model"], f"{location}.model")
        model = models.get(model_id)
        if model is None:
            raise ValueError(f"{location}.model is not registered: {model_id}")
        vehicles[vehicle_id] = RegisteredVehicle(vehicle_id, model)
    return VehicleRegistry(vehicles)


def _response_body(response: httpx.Response) -> Any | None:
    if not response.content:
        return None
    try:
        return response.json()
    except ValueError:
        return {"detail": response.text}


def _mapping(value: Any, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{location} must be a mapping")
    return value


def _list(value: Any, location: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{location} must be a list")
    return value


def _string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{location} must be a non-empty string")
    return value


def _require_keys(document: dict[str, Any], expected: set[str], location: str) -> None:
    actual = set(document)
    if actual != expected:
        raise ValueError(
            f"{location} keys must be {sorted(expected)}, got {sorted(actual)}"
        )
