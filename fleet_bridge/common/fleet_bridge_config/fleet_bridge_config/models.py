from dataclasses import dataclass


@dataclass(frozen=True)
class CommandApiConfig:
    host: str
    port: int


@dataclass(frozen=True)
class ServerConfig:
    domain_id: int
    foxglove_port: int
    command_api: CommandApiConfig


@dataclass(frozen=True)
class VehicleConfig:
    id: str
    foxglove_uri: str
    command_api_url: str
    enabled: bool

    @property
    def namespace(self) -> str:
        """Return the canonical telemetry namespace for this vehicle."""

        return f'/{self.id}'


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
class RateConfig:
    max_rate_hz: float | None = None


@dataclass(frozen=True)
class TopicConfig:
    id: str
    enabled: bool
    source: str
    target: str
    message_type: str
    worker_rate: RateConfig
    qos: QosConfig
    paired_with: str | None = None
    replay_rate_hz: float | None = None


@dataclass(frozen=True)
class CentralTopicConfig:
    id: str
    enabled: bool
    source: str
    target: str
    message_type: str
    replay_rate_hz: float
    qos: QosConfig
