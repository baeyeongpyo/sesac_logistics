from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Mapping


@dataclass(frozen=True)
class ServerConfig:
    domain_id: int
    foxglove_port: int


@dataclass(frozen=True)
class VehicleConfig:
    id: str
    domain_id: int
    foxglove_uri: str
    namespace: str
    enabled: bool


@dataclass(frozen=True)
class FleetConfig:
    server: ServerConfig
    vehicles: tuple[VehicleConfig, ...]

    def vehicle(self, robot_id: str) -> VehicleConfig:
        for vehicle in self.vehicles:
            if vehicle.id == robot_id:
                return vehicle
        raise KeyError(robot_id)


@dataclass(frozen=True)
class QosConfig:
    reliability: str
    durability: str
    history: str
    depth: int


@dataclass(frozen=True)
class CriticalConfig:
    field: str | None = None
    below: float | None = None
    bypass_rate_limit: bool = False


@dataclass(frozen=True)
class FilterConfig:
    mode: str
    max_rate_hz: float | None = None
    heartbeat_sec: float | None = None
    thresholds: Mapping[str, float] = field(
        default_factory=lambda: MappingProxyType({}),
    )
    critical: CriticalConfig = field(default_factory=CriticalConfig)


@dataclass(frozen=True)
class RateConfig:
    max_rate_hz: float | None = None


@dataclass(frozen=True)
class TopicConfig:
    id: str
    enabled: bool
    source: str
    uplink: str
    target: str
    message_type: str
    filter: FilterConfig
    worker_rate: RateConfig
    qos: QosConfig
    debug: bool

